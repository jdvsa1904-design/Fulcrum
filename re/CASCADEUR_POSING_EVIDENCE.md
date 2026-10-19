# Cascadeur posing evidence

Date: 2026-10-16. Installed target: `C:\Program Files\Cascadeur\tool_auto_posing.dll` (37,790,208 bytes). SHA-256: `C8B95DD03677888824D65103B40C44A5D8B8FE228ED952686D82D3A42602ED1A`.

## Method and result

Ghidra 12.1.2 headless successfully imported and analyzed this PE DLL directly, using the installed JDK21, isolated project `C:\re_work\fulcrum_cascadeur\posing\CascadeurPosing`, `analysisTimeoutPerFile 180`, and shipped `StringXrefDump.java` with `maxFns 12`. The `model` search found serialized TorchScript resource strings but no native containing-function xrefs. This does not recover the native posing caller. The REA MCP provider's DLL limitation does not apply to direct Ghidra CLI.

Independent ZIP64 archive inspection recovered six genuine embedded TorchScript archives. Generated TorchScript code and restricted metadata decoding exposed the real contracts. CPU loading and one synthetic pose inference subsequently succeeded; this establishes execution, not production quality.

| Archive | DLL file offset | Bytes | Local artifact |
|---|---|---|---|
| auto_unlock_2026_09_10_banzai3 | 0x6254 | 6,982,754 | posing/model_0.pt |
| fingers | 0xefdb6c | 1,921,680 | posing/model_1.pt |
| fingers_auto_unlock | 0x10d2e00 | 2,956,136 | posing/model_2.pt |
| dc_init, 43-point humanoid | 0x13a496c | 16,151 | posing/model_3.pt |
| auto_unlock_2026_08_14, quadruped | 0x1d10299 | 7,245,768 | posing/model_4.pt |
| dc_init, quadruped | 0x23f9265 | 12,945 | posing/model_5.pt |

Artifacts above are under `C:\re_work\fulcrum_cascadeur`; archive scripts and constants metadata are retained under `posing/archive_*` and `posing/model_*_constants.pkl.json`. `posing/contract.json` gives a machine-readable partial contract.

## Actual recovered functions and signatures

The humanoid archive contains `AutoLockModel.forward`, `MaskUpdater.forward`, `MaskUpdater.update_nodes_masks`, `MaskUpdater.update_dc_masks`, `LossModel.forward`, and `PosingModel.forward`. The initializer contains `DCInitModel.forward(poses)`. These are actual serialized model functions, not inferred C++ names.

`AutoLockModel.forward(tposes, nodes, dc_group, snap_p, spine_p, loss_p, fulcrum_mask, plane_normals, plane_positions)` returns `(pred_nodes, nodes_s, pred_dc, dc_s)` with float32/bool/float32/bool output conversions. It computes direction vectors from nodes, infers locking masks, checks temporal displacement, and invokes `PosingModel.forward`. The declared `dc_group` argument is not used by this humanoid forward body.

Recovered trace shapes and storage types:

| Input | Type | Trace shape |
|---|---|---|
| tposes | float32 | 43 × 3 |
| nodes | float32 | 9 × 43 × 3 |
| dc_group | float32 | 9 × 3 × 3 |
| snap_p | float32 | 6 |
| spine_p | float32 | 1 |
| loss_p | float32 | 3 |
| fulcrum_mask | int64 | 9 × 43 × 1 |
| plane_normals | float32 | 9 × 4 × 3 |
| plane_positions | float32 | 9 × 4 × 3 |

The 9 trace frames do not establish a required fixed frame count; forward reshapes use a variable frame dimension. The serialized trace tensor storage entries are zero bytes, so example numeric settings and rest-point coordinates cannot be recovered from traced inputs. Constants storage contains actual values and is decoded locally without executing model pickle globals.

## Rig evidence and remaining integration blockers

Shipped `resources/scripts/python/rigging/rig_builders/units/unit/component_builders/autoposing/_autoposing.py` creates `AutoPosingLink` behaviors with `name_property` for box, main, direction, additional, and first manual/self points. It can also supply `end_finger_position`. `prototypes/rigs/qrt/biped/constants.py` supplies 43 unique non-prop point names, with main/additional/directional/self variants. These are richer than Fulcrum's ordinary joint positions.

The actual 43-point ordering was recovered from `tools.dll` function `0x180009300`, whose stack initializer constructs 43 contiguous strings. Instruction emulation reconstructed the short-string stores and constructor calls. Its ordering is corroborated by the inbetween model's directed edges: pelvis29 → stomach20 → chest37 → neck9 → head6 → head_dir36, and the corresponding arm/leg chains. Saved ordering: `C:\re_work\fulcrum_cascadeur\posing\canonical_order_candidate.json`; embedded in `tools/cascadeur_pose_backend.py`. Native caller DLL SHA-256: `71ED5E8D016DA5CD5A87312F593F5DC9E842625768195E58B665B4BEF60E9203`.

Exact snap parameter defaults, spine parameter units/defaults, loss thresholds, plane conventions, model output reconstruction, and current-character mapping remain unverified. Native `tools.dll` addresses `0x1802fde77` through `0x1802fdf7b` associate six contiguous float fields with SnappingFactor, SnappingAngle, SnappingLateralAddAngle, SnappingInvertedAngle, SnappingDisablingAngle, and SnappingDisablingAddAngle. `0x1802fdb7b` names HumanoidSpineForwardBackwardSwitchingAngle. These names alone do not recover defaults or units.

Qt resource-tree parsing also confirmed `:/AutoPosingTool/config/autoposing_2026_09_10.onnx` (8,711,342 bytes) and quadruped ONNX (9,861,646 bytes), alongside the six TorchScript archives.

CPU PyTorch 2.14.1+cpu was installed in isolated `C:\Projects\Fulcrum\.venv-ml` using existing Python3.12 and the official PyTorch CPU package index. Both humanoid and initializer loaded successfully. DCInit accepts only one pose per invocation because its output reshape is hardcoded `[3,3]`; the backend invokes it per frame and stacks results.

`tools/cascadeur_pose_backend.py` supplies an executable explicit-tensor CLI with metadata, init_dc, pose, and auto_unlock modes. It requires explicit settings and validates input dimensions, finite values, integer masks, and finite outputs. `pose` invokes the actual `PosingModel.forward` and returns positions and direction controls. A synthetic three-frame request using the inbetween model's reference rest points returned finite nodes `[3,43,3]` and DC `[3,3,3]`. Its deliberately explicit test settings were zero snap/spine, masks1, and distant ground planes. Maximum coordinate displacement was 0.1814601822 in template units: this is not a fidelity pass and confirms exact preservation must be checked independently. No production defaults are inferred from this test. The synthetic result is retained in `posing/synthetic_pose_result.json`.
