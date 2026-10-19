from __future__ import annotations
import argparse
import contextlib
import hashlib
import json
import sys
import time
from pathlib import Path


def infer(payload: dict) -> dict:
    import torch
    torch.set_num_threads(2)
    path = Path(payload['model_path']).resolve(strict=True)
    positions = torch.tensor(payload['positions'], dtype=torch.float32)
    frames = torch.tensor(payload['frames'], dtype=torch.int64)
    tpose = torch.tensor(payload['tpose'], dtype=torch.float32)
    if positions.ndim != 3 or tuple(positions.shape[1:]) != (43, 3):
        raise ValueError('positions must have shape [K,43,3] in canonical point order')
    k = positions.shape[0]
    if not 4 <= k <= 128 or tuple(frames.shape) != (k,):
        raise ValueError('Four to 128 matching key poses and frame indices are required')
    if not bool(torch.all(frames[1:] > frames[:-1])):
        raise ValueError('Frame indices must be strictly increasing')
    if int(frames[-1] - frames[0]) > 1200:
        raise ValueError('Requested span exceeds the 1200-frame inference limit')
    if tuple(tpose.shape) != (43, 3):
        raise ValueError('tpose must have shape [43,3]')
    if not bool(torch.isfinite(positions).all() and torch.isfinite(tpose).all()):
        raise ValueError('Positions must be finite')
    style = torch.tensor(payload['style'], dtype=torch.int64)
    if style.ndim == 0:
        style = style.repeat(k)
    if tuple(style.shape) != (k,) or bool(torch.any(style < 0)):
        raise ValueError('style must be a nonnegative integer or [K] label array')
    frame_mask = torch.tensor(payload.get('frames_mask', [1] * k), dtype=torch.int32)
    point_mask = torch.tensor(payload.get('points_mask', [[[1.0]] * 43 for _ in range(k)]), dtype=torch.float32)
    if tuple(point_mask.shape) == (k, 43):
        point_mask = point_mask.unsqueeze(-1)
    if tuple(frame_mask.shape) != (k,) or tuple(point_mask.shape) != (k, 43, 1):
        raise ValueError('frames_mask must be [K]; points_mask must be [K,43,1]')
    if not bool(torch.isfinite(point_mask).all()):
        raise ValueError('Point mask must be finite')
    started = time.perf_counter()
    model = torch.jit.load(str(path), map_location='cpu').eval()
    schema = str(model.forward.schema)
    if 'MibVelocityLoop' not in schema or 'Tensor points_mask' not in schema:
        raise ValueError('This adapter requires the recovered MibVelocityLoop model')
    with contextlib.redirect_stdout(sys.stderr), torch.inference_mode():
        result = model(positions, torch.tensor(0.0), frames, style, tpose,
                       torch.tensor(float(payload.get('height', 1.0))), frame_mask,
                       point_mask.clone())
    expected = int(frames[-2] - frames[1]) + 1
    if tuple(result.shape) != (expected, 43, 3) or not bool(torch.isfinite(result).all()):
        raise ValueError('Model produced invalid or nonfinite canonical motion')
    return {
        'positions': result.tolist(),
        'first_frame': int(frames[1]), 'last_frame': int(frames[-2]),
        'contract': {'model': 'MibVelocityLoop', 'canonical_points': 43,
                     'axis': 'y-up', 'units': 'caller supplied; same scale as tpose',
                     'outer_keys_are_context': True,
                     'style_label_semantics': 'not recovered',
                     'model_sha256': hashlib.sha256(path.read_bytes()).hexdigest()},
        'seconds': time.perf_counter() - started,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path)
    args = parser.parse_args()
    try:
        payload = json.loads(args.input.read_text(encoding='utf-8') if args.input else sys.stdin.read())
        print(json.dumps(infer(payload), allow_nan=False))
        return 0
    except Exception as exc:
        print(json.dumps({'error': str(exc)}))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
