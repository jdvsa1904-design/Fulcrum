# Fulcrum feature checklist audit

Date: 2026-10-16

Read-only source audit of the attached 145-item dependency checklist. This records the baseline observed before the current integration work. Implemented means the behavior has a source implementation, not Cascadeur equivalence or validated production completeness. Partial means a narrower implementation or missing validation. Missing means no implementation found in the inspected paths. Items 53–133 were outside this audit's assigned scope and require integration validation. Concurrent edits may supersede findings; update after verification.

Primary present gaps: multi-character scenes, independent prop attachment, clip selection, edit-preserving reimport, semantic rig controls, IK/FK switching, mirroring, retiming, pose library, global undo, preview/accept/reject, recovery, and exhaustive export validation. Rule-based correction is not evidence of a trained AI posing model.

| Item | Baseline status | Evidence / limitation |
|---|---|---|
| 1 | Partial | FBX character loader; non-skinned prop meshes hidden (`app.js:114-129`). |
| 2 | Implemented | Bone traversal and parent indices (`app.js:112,147`). |
| 3 | Implemented | Canonical name/alias and geometric mapping (`wire/rigmap.js`). |
| 4 | Partial | Similarity/unit fitting; no explicit import axes/unit controls (`app.js:264-282`). |
| 5 | Partial | Root/pelvis movers and procedural travel (`app.js:273,846`). |
| 6 | Partial | Layer rotateOrder supported; imported rotation-order metadata not explicitly retained (`wire/animlayers.js:4`). |
| 7 | Implemented | First skinned mesh and bind skeleton associated (`app.js:120-136`). |
| 8 | Missing | Single S.body/S.take scene model (`app.js:35,147`). |
| 9 | Missing | No generic prop socket/attachment editor found. |
| 10 | Partial | Only first animation used (`app.js:239`). |
| 11 | Partial | FPS/ranges supported; appended FPS mismatch not resampled (`app.js:176,319`). |
| 12 | Partial | Missing canonical slots/confidence and HIK error; malformed hierarchy validation limited (`app.js:137,208,347`). |
| 13 | Missing | setBody clears edits (`app.js:172`). |
| 14 | Partial | Sampled export exists; complete skeleton/timing fidelity not established (`app.js:1793`; `tools/frames_to_fbx.py`). |
| 15 | Partial | Deterministic conversion math exists; source-format invariants not verified. |
| 16 | Implemented | Canonical humanoid CORE (`wire/rigmap.js:3`). |
| 17 | Partial | Name/geometric mapping; no manual mapping editor (`wire/rigmap.js`). |
| 18 | Partial | Missing slots/confidence/HIK validity checks; incomplete topology diagnostics. |
| 19 | Missing | Fixed sphere controls only (`app.js:155`). |
| 20 | Partial | Direct bone markers/gizmo; no semantic body-control rig. |
| 21 | Partial | Foot/hand solver goals; no draggable IK goal controls (`app.js:480,539`). |
| 22 | Implemented | FK rotation gizmo keys selected joint (`app.js:1385`). |
| 23 | Missing | No IK/FK switching or snap preservation found. |
| 24 | Partial | Computed stable poles; no authored pole-vector controls (`app.js:480-526`). |
| 25 | Partial | Direct bone rotation/procedural root; no dedicated root/pelvis translation control. |
| 26 | Partial | Direct spine/torso bone rotation only. |
| 27 | Partial | Finger alias map; finger markers hidden by default (`app.js:155`). |
| 28 | Partial | Constraints and reference-angle limits; anatomy degrees-of-freedom not established (`advanced.js`). |
| 29 | Missing | No control/pose mirroring found. |
| 30 | Partial | Canonical rig names/confidence only (`wire/rigmap.js`; `app.js:144`). |
| 31 | Missing | Humanoid assumptions in solvers; no non-humanoid characterization. |
| 32 | Partial | Sampled skeleton export; no control rig bake into editable keys (`app.js:1793`). |
| 33 | Missing | Project version exists but no rig compatibility/version enforcement. |
| 34 | Implemented | Timeline seek and frame-range selection (`app.js:1660-1718`). |
| 35 | Implemented | Six joint curve channels (`wire/animlayers.js:13-36`). |
| 36 | Partial | Key selection/movement; no key copy/paste/scaling found (`ui/grapheditor.js:108,138`). |
| 37 | Implemented | Editable tangent types/handles (`ui/grapheditor.js:156-176`). |
| 38 | Implemented | Step/linear/spline and additional tangent modes (`wire/animlayers.js`). |
| 39 | Implemented | Pose-layer quaternion slerp (`app.js:352-373`); Euler layers intentionally use curves. |
| 40 | Partial | Gizmo automatically keys; explicit graph key action; no auto-key switch (`app.js:1385,1470`). |
| 41 | Implemented | Sparse pose and animation-layer keys (`app.js:352`; `wire/animlayers.js`). |
| 42 | Implemented | Selection-scoped fixes and operations (`app.js:1519`). |
| 43 | Missing | No clip retiming/time-warp system found. |
| 44 | Partial | Looped playback; no cycle editing tools (`app.js:1720`). |
| 45 | Partial | Procedural travel/root offsets and translation curves; no authored root path. |
| 46 | Implemented | Pose/animation layers and additive offsets (`wire/animlayers.js`). |
| 47 | Partial | Per-bone key membership; no explicit mask/body-region controls. |
| 48 | Implemented | Weight/mute/override controls (`wire/animlayers.js`; `app.js:1414`). |
| 49 | Partial | Source and fix stack preserved; undo limited to graph editor (`ui/grapheditor.js:178`). |
| 50 | Partial | Per-frame export bake; no bake-to-editable-key operation (`app.js:1793`). |
| 51 | Partial | Numerical fidelity gate; no visual before/after toggle (`app.js:1213`). |
| 52 | Missing | No reusable pose library/bookmarks found. |
| 53 | Requires integration validation | Not inspected in this audit. |
| 54 | Requires integration validation | Not inspected in this audit. |
| 55 | Requires integration validation | Not inspected in this audit. |
| 56 | Requires integration validation | Not inspected in this audit. |
| 57 | Requires integration validation | Not inspected in this audit. |
| 58 | Requires integration validation | Not inspected in this audit. |
| 59 | Requires integration validation | Not inspected in this audit. |
| 60 | Requires integration validation | Not inspected in this audit. |
| 61 | Requires integration validation | Not inspected in this audit. |
| 62 | Requires integration validation | Not inspected in this audit. |
| 63 | Requires integration validation | Not inspected in this audit. |
| 64 | Requires integration validation | Not inspected in this audit. |
| 65 | Requires integration validation | Not inspected in this audit. |
| 66 | Requires integration validation | Not inspected in this audit. |
| 67 | Requires integration validation | Not inspected in this audit. |
| 68 | Requires integration validation | Not inspected in this audit. |
| 69 | Requires integration validation | Not inspected in this audit. |
| 70 | Requires integration validation | Not inspected in this audit. |
| 71 | Requires integration validation | Not inspected in this audit. |
| 72 | Requires integration validation | Not inspected in this audit. |
| 73 | Requires integration validation | Not inspected in this audit. |
| 74 | Requires integration validation | Not inspected in this audit. |
| 75 | Requires integration validation | Not inspected in this audit. |
| 76 | Requires integration validation | Not inspected in this audit. |
| 77 | Requires integration validation | Not inspected in this audit. |
| 78 | Requires integration validation | Not inspected in this audit. |
| 79 | Requires integration validation | Not inspected in this audit. |
| 80 | Requires integration validation | Not inspected in this audit. |
| 81 | Requires integration validation | Not inspected in this audit. |
| 82 | Requires integration validation | Not inspected in this audit. |
| 83 | Requires integration validation | Not inspected in this audit. |
| 84 | Requires integration validation | Not inspected in this audit. |
| 85 | Requires integration validation | Not inspected in this audit. |
| 86 | Requires integration validation | Not inspected in this audit. |
| 87 | Requires integration validation | Not inspected in this audit. |
| 88 | Requires integration validation | Not inspected in this audit. |
| 89 | Requires integration validation | Not inspected in this audit. |
| 90 | Requires integration validation | Not inspected in this audit. |
| 91 | Requires integration validation | Not inspected in this audit. |
| 92 | Requires integration validation | Not inspected in this audit. |
| 93 | Requires integration validation | Not inspected in this audit. |
| 94 | Requires integration validation | Not inspected in this audit. |
| 95 | Requires integration validation | Not inspected in this audit. |
| 96 | Requires integration validation | Not inspected in this audit. |
| 97 | Requires integration validation | Not inspected in this audit. |
| 98 | Requires integration validation | Not inspected in this audit. |
| 99 | Requires integration validation | Not inspected in this audit. |
| 100 | Requires integration validation | Not inspected in this audit. |
| 101 | Requires integration validation | Not inspected in this audit. |
| 102 | Requires integration validation | Not inspected in this audit. |
| 103 | Requires integration validation | Not inspected in this audit. |
| 104 | Requires integration validation | Not inspected in this audit. |
| 105 | Requires integration validation | Not inspected in this audit. |
| 106 | Requires integration validation | Not inspected in this audit. |
| 107 | Requires integration validation | Not inspected in this audit. |
| 108 | Requires integration validation | Not inspected in this audit. |
| 109 | Requires integration validation | Not inspected in this audit. |
| 110 | Requires integration validation | Not inspected in this audit. |
| 111 | Requires integration validation | Not inspected in this audit. |
| 112 | Requires integration validation | Not inspected in this audit. |
| 113 | Requires integration validation | Not inspected in this audit. |
| 114 | Requires integration validation | Not inspected in this audit. |
| 115 | Requires integration validation | Not inspected in this audit. |
| 116 | Requires integration validation | Not inspected in this audit. |
| 117 | Requires integration validation | Not inspected in this audit. |
| 118 | Requires integration validation | Not inspected in this audit. |
| 119 | Requires integration validation | Not inspected in this audit. |
| 120 | Requires integration validation | Not inspected in this audit. |
| 121 | Requires integration validation | Not inspected in this audit. |
| 122 | Requires integration validation | Not inspected in this audit. |
| 123 | Requires integration validation | Not inspected in this audit. |
| 124 | Requires integration validation | Not inspected in this audit. |
| 125 | Requires integration validation | Not inspected in this audit. |
| 126 | Requires integration validation | Not inspected in this audit. |
| 127 | Requires integration validation | Not inspected in this audit. |
| 128 | Requires integration validation | Not inspected in this audit. |
| 129 | Requires integration validation | Not inspected in this audit. |
| 130 | Requires integration validation | Not inspected in this audit. |
| 131 | Requires integration validation | Not inspected in this audit. |
| 132 | Requires integration validation | Not inspected in this audit. |
| 133 | Requires integration validation | Not inspected in this audit. |
| 134 | Partial | Source/base/output separation; generated fixes immediately applied (`app.js:379,1046`). |
| 135 | Partial | Precomputed playback; no performance benchmark (`app.js:1320,1720`). |
| 136 | Partial | Raycast selection and graph hit testing; interactive predictability not verified. |
| 137 | Implemented | Visible foot locks, hand pins and sole markers (`app.js:1333-1354`). |
| 138 | Partial | Rig confidence/missing slots and import errors; malformed-input coverage incomplete. |
| 139 | Partial | Status strings/export logs; synchronous generation lacks progress. |
| 140 | Partial | Deterministic formulas; full repeatability validation absent. |
| 141 | Missing | No local preview/accept/reject workflow found. |
| 142 | Partial | Full-clip arrays and repeated rebuilds; no long-clip benchmark/worker. |
| 143 | Implemented | Flags/taste reports with frame ranges and body-part information (`app.js:1159,1286,1356`). |
| 144 | Partial | Manual project save/open; no interruption recovery/autosave (`app.js:1751-1790`). |
| 145 | Partial | Gate/taste/export roundtrip; only 4 frames sampled, fingers omitted, fitted error may mask global drift (`app.js:1884-1942`). |

Evidence paths are relative to `C:\Projects\Fulcrum`. Line numbers describe the inspected baseline and may move during integration. Source inspection alone does not establish visual quality, solver convergence, FRT contact preservation, or FBX correctness across all frames.

