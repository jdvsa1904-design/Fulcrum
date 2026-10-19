import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys

CANONICAL_ORDER = ['foot_self_l', 'hand_r', 'clavicle_add_r', 'foot_r', 'toe_dir_l', 'calf_l', 'head', 'forearm_add_l', 'toe_dir_r', 'neck', 'foot_l', 'forearm_l', 'calf_r', 'hand_dir_l', 'calf_add_l', 'hand_dir_r', 'clavicle_l', 'arm_r', 'toe_l', 'clavicle_r', 'stomach', 'toe_add_l', 'thigh_r', 'foot_self_r', 'hand_add_r', 'forearm_r', 'stomach_add', 'forearm_add_r', 'arm_l', 'pelvis', 'chest_add', 'hand_add_l', 'head_add', 'pelvis_add', 'calf_add_r', 'thigh_l', 'head_dir', 'chest', 'clavicle_add_l', 'hand_l', 'neck_add', 'toe_add_r', 'toe_r']
MODEL = Path(os.environ.get('FULCRUM_NEURAL_POSE_MODEL', Path(__file__).resolve().parent.parent / 'models' / 'neural' / 'model_0.pt'))
INITIALIZER = Path(os.environ.get('FULCRUM_NEURAL_DC_MODEL', Path(__file__).resolve().parent.parent / 'models' / 'neural' / 'model_3.pt'))


def metadata():
    return {
        'backend': 'neural_torchscript',
        'runtime_available': importlib.util.find_spec('torch') is not None,
        'model_available': MODEL.is_file(),
        'initializer_available': INITIALIZER.is_file(),
        'model': str(MODEL),
        'initializer': str(INITIALIZER),
        'canonical_order': CANONICAL_ORDER,
        'point_order_evidence': 'tools.dll FUN_180009300, corroborated by shipped model skeleton edges',
        'native_settings_defaults_recovered': False,
        'production_guided_adapter_validated': False,
        'modes': ['metadata', 'init_dc', 'pose', 'auto_unlock'],
        'required_pose_settings': ['snap_p[6]', 'spine_p[1]', 'plane_normals[F,4,3]', 'plane_positions[F,4,3]'],
        'required_pose_inputs': ['tposes[43,3]', 'nodes[F,43,3]', 'nodes_s[F,43,1]', 'dc_s[F,3,1]'],
        'required_auto_unlock_settings': ['snap_p[6]', 'spine_p[1]', 'loss_p[3]', 'fulcrum_mask[F,43,1]', 'plane_normals[F,4,3]', 'plane_positions[F,4,3]'],
        'limit': 'Explicit tensor interface; native UI parameter defaults and skeletal adapter remain under validation.',
    }


def tensor(torch, value, shape, name, integer=False):
    if value is None:
        raise ValueError('Missing explicit input: '+name)
    try:
        probe = torch.tensor(value, dtype=torch.float64)
    except (TypeError, ValueError, RuntimeError) as error:
        raise ValueError('Invalid numeric tensor '+name) from error
    if list(probe.shape) != list(shape):
        raise ValueError(f'{name} must have shape {shape}, received {list(probe.shape)}')
    if not bool(torch.isfinite(probe).all()):
        raise ValueError(name+' must contain finite numbers')
    if integer and not bool((probe == torch.round(probe)).all()):
        raise ValueError(name+' must contain integer mask values')
    if integer and not bool(((probe >= 0) & (probe <= 7)).all()):
        raise ValueError(name+' masks must be between 0 and 7')
    return probe.to(torch.int64 if integer else torch.float32)


def predict(request):
    mode = request.get('mode', 'metadata')
    if mode == 'metadata':
        return metadata()
    if mode not in ['init_dc', 'pose', 'auto_unlock']:
        raise ValueError('Unknown mode '+str(mode))
    import torch
    torch.set_num_threads(max(1, min(8, int(request.get('threads', 2)))))
    torch.manual_seed(0)
    nodes_raw = request.get('nodes')
    if not isinstance(nodes_raw, list) or not 1 <= len(nodes_raw) <= 4096:
        raise ValueError('nodes must contain between 1 and 4096 frames')
    frames = len(nodes_raw)
    nodes = tensor(torch, nodes_raw, [frames, 43, 3], 'nodes')
    init = torch.jit.load(str(INITIALIZER), map_location='cpu').eval()
    with torch.inference_mode():
        dc = torch.stack([init(frame) for frame in nodes])
    if not bool(torch.isfinite(dc).all()):
        raise ValueError('Direction initialization produced invalid numbers; verify point geometry')
    if mode == 'init_dc':
        return {'backend':'neural_torchscript','mode':mode,'dc':dc.tolist(),'shape':list(dc.shape)}
    tpose = tensor(torch, request.get('tposes'), [43,3], 'tposes')
    snap = tensor(torch, request.get('snap_p'), [6], 'snap_p')
    spine = tensor(torch, request.get('spine_p'), [1], 'spine_p')
    normals = tensor(torch, request.get('plane_normals'), [frames,4,3], 'plane_normals')
    planes = tensor(torch, request.get('plane_positions'), [frames,4,3], 'plane_positions')
    model = torch.jit.load(str(MODEL), map_location='cpu').eval()
    with torch.inference_mode():
        if mode == 'pose':
            nodes_s = tensor(torch, request.get('nodes_s'), [frames,43,1], 'nodes_s', integer=True)
            dc_s = tensor(torch, request.get('dc_s'), [frames,3,1], 'dc_s', integer=True)
            result = model.posing(tpose.unsqueeze(0).repeat(frames,1,1), nodes, nodes_s, dc, dc_s, spine.reshape(1,1,1).repeat(frames,1,1), snap, normals, planes)
            labels = ['nodes','dc']
        else:
            if frames < 3:
                raise ValueError('auto_unlock requires at least three temporal frames')
            loss = tensor(torch, request.get('loss_p'), [3], 'loss_p')
            fulcrum = tensor(torch, request.get('fulcrum_mask'), [frames,43,1], 'fulcrum_mask', integer=True)
            result = model(tpose, nodes, dc, snap, spine, loss, fulcrum, normals, planes)
            labels = ['nodes','nodes_s','dc','dc_s']
    for name, value in zip(labels,result):
        if not bool(torch.isfinite(value).all()):
            raise ValueError('Model returned invalid '+name+'; request was not applied')
    return {'backend':'neural_torchscript','mode':mode,'frames':frames,'outputs':{key:value.tolist() for key,value in zip(labels,result)},'shapes':{key:list(value.shape) for key,value in zip(labels,result)},'canonical_order':CANONICAL_ORDER}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--metadata',action='store_true')
    parser.add_argument('--input',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    try:
        request = {'mode':'metadata'} if args.metadata else json.loads(args.input.read_text() if args.input else sys.stdin.read())
        result = {'ok':True,**predict(request)}
    except Exception as error:
        result = {'ok':False,'error':str(error),'backend':'neural_torchscript'}
    payload=json.dumps(result,allow_nan=False)
    if args.output:
        args.output.write_text(payload)
    else:
        print(payload)
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
