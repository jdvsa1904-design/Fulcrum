# Cascadeur inbetweening: shipped model evidence

Investigated 2026-10-16. Static extraction and TorchScript schema loading, followed by a separately recorded synthetic inference probe. This report does not establish full equivalence with Cascadeur's native caller or UI.

## Artifact identity and reproducibility

- Source: `C:\Program Files\Cascadeur\tool_inbetweening.dll`.
- Size: 178,077,184 bytes.
- SHA-256: `e11bdf5ef65eea97e44708af8aec0796881eab3f553b4dfb67eb7739b58f5f12`.
- Structural inventory/extractor: `C:\re_work\fulcrum_cascadeur\inspect_inbetween.py`.
- Inventory with every ZIP member and archive hashes: `C:\re_work\fulcrum_cascadeur\inbetween\inventory.json`.
- Tensor/schema inspection: `C:\re_work\fulcrum_cascadeur\inbetween_contract.py`.
- Controlled inference: `C:\re_work\fulcrum_cascadeur\inbetween_probe.py`, result `inbetween\probe.json`.
- Python runtime: `C:\Projects\Fulcrum\.venv-ml\Scripts\python.exe`.

The extractor validates ZIP central directory boundaries and reads source members, metadata and pickle opcodes; it never executes pickle payloads. The subsequent schema inspection explicitly loads identified TorchScript models in the local CPU runtime. No native Cascadeur DLL was loaded or executed by these scripts.

## Observations

Three embedded TorchScript ZIPs are present:

| Index | File offset | Root | Bytes | Role established by forward code |
| --- | --- | --- | ---: | --- |
| 0 | `0x6254` | `casc_generator1234` | 46,843,706 | Velocity-conditioned pose inbetweening and retargeting |
| 1 | `0x2cb2992` | `casc_generator_root_motion_ref_132` | 73,073,258 | Root-motion generation with reference animation inputs |
| 2 | `0x7262c00` | `casc_generator_root_motion_194` | 58,099,428 | Root-motion generation with pose/style weights |

Each is materialized at `C:\re_work\fulcrum_cascadeur\inbetween\archive_N\model.pt`; exact forward schemas and tensor constant shapes/short values are in `archive_N\contract.json`. All three loaded successfully with Torch CPU.

### Exact tensor argument order

Archive 0, `MibVelocityLoop.forward`:

`pos_k, _0, frames_k, style, src_tpose, src_height, frames_mask, points_mask -> Tensor`

Archive 1, `MibRootMotionRefLoop.forward`:

`pos_k, _0, frames_k, style, src_tpose, src_height, frames_mask, pelvis_trajectory_lambda, ref_pos_k, ref_tpose, ref_height, head_lambda, label_lambdas, ref_lambdas -> Tensor`

Archive 2, `MibRootMotionLoop.forward`:

`pos_k, _0, frames_k, style, src_tpose, src_height, frames_mask, pelvis_trajectory_lambda, head_lambda, label_lambdas, poses_lambdas -> Tensor`

All arguments are Torch tensors. Generated code casts position inputs to float32 and frame/style indices to int64 internally. `src_tpose` is reshaped to `[1,43,3]`. The unused argument `_0` is passed through but has no contributing computation in the exposed velocity graph. The scalar height argument appears in the signature but is not referenced by that graph's computation; this does not prove other models ignore it.

### Frames and context

All root forwards assert at least four entries in `frames_k`. Frame indices are shifted relative to the first input frame. Position storage is `[last-first+1,43,3]`. The returned motion is explicitly sliced from `frames_k[1]` through `frames_k[-2]` inclusive. Thus the first/last poses provide velocity/context beyond the requested output interval, rather than being output endpoints themselves.

The velocity model's `split_intervals` detects adjacent-frame pairs and partitions with outer context; its explicit assertion says `need at least 4 frames for velocity computation`. The root-motion model source retained in debug metadata instead currently uses a single interval containing all keys. These are different contracts and must not be collapsed into a generic two-pose interpolation call.

### Canonical point ordering

Independent native initializer analysis by the parallel checklist investigation recovered 43 consecutive names; this order is corroborated by embedded model c14/c15 kinematic edge constants and T-pose geometry. Source artifact: `C:\re_work\fulcrum_cascadeur\posing\canonical_order_candidate.json`.

```
0 foot_self_l       1 hand_r             2 clavicle_add_r
3 foot_r            4 toe_dir_l          5 calf_l
6 head              7 forearm_add_l      8 toe_dir_r
9 neck             10 foot_l            11 forearm_l
12 calf_r          13 hand_dir_l        14 calf_add_l
15 hand_dir_r      16 clavicle_l        17 arm_r
18 toe_l           19 clavicle_r        20 stomach
21 toe_add_l       22 thigh_r           23 foot_self_r
24 hand_add_r      25 forearm_r         26 stomach_add
27 forearm_add_r   28 arm_l             29 pelvis
30 chest_add       31 hand_add_l        32 head_add
33 pelvis_add      34 calf_add_r        35 thigh_l
36 head_dir        37 chest             38 clavicle_add_l
39 hand_l          40 neck_add          41 toe_add_r
42 toe_r
```

These are rig points, including orientation-helper points, rather than 43 arbitrary skeleton joints. Index 29 is the pelvis. The principal chain observed in c14 is pelvis29 -> stomach20 -> chest37 -> neck9 -> head6 -> head_dir36. c180 in the velocity retarget module supplies the canonical world-space T-pose `[1,43,3]`; c250 supplies a local rest structure. Canonical T-pose height is approximately 182.49 units with y vertical. Centimeter interpretation is an inference from scale, not a native caller measurement.

### Constraint/control semantics from code

- Per-point mask is indexed by key frame and forced to ones at the first two and last two context/key entries, then used by velocity conditioning. Treating it as an arbitrary joint-lock bitset without verifying caller polarity is unsafe.
- Frame mask is indexed along with key poses, cast to integer, and fed into the subsequent graph. The precise native UI meaning remains unverified.
- `style` is cast to int64; the velocity model indexes it per key. Label ID names and valid ranges still require native caller evidence.
- Root models expose explicit pelvis-trajectory, head, label and pose/reference weighting tensors. Generated code contains repeated weighted refinement computations, rather than only a feed-forward pose interpolation.
- Retained scripted transforms perform rest-pose retargeting, coordinate alignment, blending and non-fulcrum root-height correction. `fix_non_fulcrum_intervals` adjusts vertical root paths while preserving source endpoint behavior. Presence of this graph establishes computation, not its actual native invocation settings.
- Public shipped stub `resources\scripts\stubs\csc\tools\__init__.pyi` exposes only `InbetweeningTool.get_update_parameter()` and `set_update_parameter(bool)`. The shipped `ml\editable_animation.py` temporarily disables and restores this flag around its changes. It is not a public model-tensor API.

## Integration implications and remaining unknowns

Controlled execution succeeded with synthetic T-pose translations at frame keys `[0,1,9,10]`, style label 0, frame mask ones `[4]`, and point mask ones `[4,43,1]`. Output was finite `[9,43,3]` (frames 1..9), maximum endpoint drift 0.022644 canonical units, and elapsed time 19.70 seconds including CPU model loading. The first trial with a rank-two point mask failed a dimension check; the valid trailing singleton is now enforced by the adapter. This is a synthetic smoke test, not a character animation quality test or proof of label-0 meaning.

Fulcrum adapter `tools\cascadeur_inbetween_backend.py` accepts JSON on stdin or `--input` with an explicit model path, positions `[K,43,3]`, frame indices `[K]`, T-pose `[43,3]` and explicit `style`. All-one masks are the tested baseline, not a recovered UI default. The adapter rejects nonfinite inputs/results, wrong model schemas, invalid dimensions, non-increasing keys, and excessive frame spans. It reports original output range so callers can preserve untouched context and restore exact authored endpoint keys.

Observed data is enough to build a local inference adapter using the original installed model files, canonical rig-point mapping, explicit four-frame context and protected output range. It is not enough to substitute raw Fulcrum joint coordinates without helper-point construction and coordinate conversion.

Still unresolved: exact native tensor construction/default mask polarity, label enumeration, native UI weight ranges, collision/contact postprocessing in the DLL caller, GPU provider behavior, end-to-end parity on Cascadeur-authored reference scenes, and robust retargeting for nonstandard skeletons. No binary recompilation has been performed by this investigation; runnable TorchScript recovery is distinct from native source reconstruction.
