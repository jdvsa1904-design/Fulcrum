# MotionBuilder 2027: Story, layers, aux effectors, remaining filters

## Evidence
- **MEAS**: measured in a live MotionBuilder 2027 through the Python console (telnet 127.0.0.1:4242).
  Every probe ran in a fresh, unsaved scene and used only `mbprobe:` objects. No file was saved, and the
  scene was left empty. The probe scripts and their raw JSON output are in `probes/`, and the tests read
  that JSON directly.
- **OBS**: read from code earlier (`re/MOTIONBUILDER.md`).
- **SDK**: stated in an OpenRealitySDK header.
- **INF**: inferred and not measured.

REA could not be used. It has no provider for Windows DLLs here, and Ghidra was out of scope.

Units in the probes:
- Story ran at 24 fps.
- Values are the evaluated global translation of a null, or the HIK-solved left-hand joint for the aux
  probes.

## 1. Story (`story.js`)

| Item | Result | Max error | Tag |
|---|---|---|---|
| Transition between overlapping clips | `w = curve((t - in.start)/(out.stop - in.start))`; the outgoing clip gets `1 - w`. Only the incoming clip's `InInterpolator` matters; the outgoing clip's `OutInterpolator` had no effect. | 8.7e-6 % | MEAS |
| Linear / Smooth / Slow / Fast | `u`, smoothstep `u²(3-2u)`, `u³`, `1-(1-u)³` | in row above | MEAS |
| Time warp Normal / Smoothed Ends / Going Faster / Slowing Down | `u`, smoothstep, `u³`, `1-(1-u)³`; "Reversed" variants are `1 - f(u)` | 0.074 % of clip length, at the flat tail of the curve (u ≥ 0.8) | MEAS |
| Source time | `local = (t - Start)·Speed`. First loop starts at `SourceMarkIn + Offset` (`Offset = FirstLoopMarkIn - SourceMarkIn`) and runs to `SourceMarkOut`. Later loops repeat `[SourceMarkIn, SourceMarkOut)`. The loop index is 0 for the first loop. Non-looping clips hold the last source frame. | exact (frame) | MEAS |
| Loop translation | value = source value + loopIndex · `LoopTranslation` | 8.9e-16 | MEAS |
| AutoLoop translation | `travel(last) - travel(first)` with Y zeroed | exact | MEAS |
| Clip frame (`ClipTranslation/ClipRotation`) | T = travel node at first frame with Y = 0; yaw = `atan2(-dz, dx)` (clip +X points along travel) | 1e-4° | MEAS |
| Match algorithm (Gravity T `(A.x,B.y,A.z)`, Gravity R = shortest arc between XZ-projected X axes) | unchanged | n/a | OBS |
| Match on feet (midpoint of feet XZ, lowest Y) | Fulcrum choice; MoBu matches on whatever pivot model is listed | n/a | INF |

Clip behaviour:
- Writable clips (Insert Current Take) do not loop. Setting Stop on one extends the source, and setting
  `Offset` is ignored.
- Looping needs a read-only clip loaded from a file. The probe exported the clip to a file and reloaded
  it (the FBX was deleted afterwards).
- Setting `SourceMarkIn` turns `AutoLoop` off and keeps the old `LoopTranslation`.

## 2. Animation layers (`layers.js`)
Every guess from the first pass matched. The test grid was base T=1, R=(30,0,0) and a layer T=10,
R=(0,60,20), in 3 modes × 2 rotation modes × weights 100/50/25/0, plus a parent/child weight case.

| Item | Result | Max error | Tag |
|---|---|---|---|
| Additive | `below + w·v` | 0 | MEAS |
| Override | `w·v`; layers below are ignored. **Base 1, Override 10 at weight 50 gives 5, not 5.5.** | 0 | MEAS |
| Override Passthrough | `lerp(below, v, w)`, which gives 5.5 in the same test | 0 | MEAS |
| Parent weight | multiplies into its children: 1 + 10·0.5 + 4·0.25 = 7 | 0 | MEAS |
| Euler rotation mode | per component, same formulas | 3.4e-6° | MEAS |
| Quaternion rotation mode | Euler XYZ (`R = Rz·Ry·Rx`). Additive `q_below · slerp(I, q, w)` (layer on the right). Override `slerp(I, q, w)`. Passthrough `slerp(q_below, q, w)`. | 3.4e-6° | MEAS |

## 3. Auxiliary effectors and pivots (`auxfx.js`)
Measured on a 15-joint test character with a control rig and one aux on the left wrist (`CreateAuxiliary`
lives on `FBCharacter`). The effector properties are `IK Reach Translation`, `IK Reach Rotation` (0–100),
`IK Pull` (0–100), `IK Pull Hips` and `IKPivot`.

| Item | Result | Max error | Tag |
|---|---|---|---|
| Combined translation target | reach-weighted mean of the effector sets | 0.029 cm | MEAS |
| Combined rotation target | normalised reach-weighted quaternion sum, i.e. weighted nlerp (100/50 gives 29.28°, not slerp's 30°) | < 0.0001° | MEAS |
| Combined reach | max of the set reaches. Both at 50 gives the same pose as one effector at the mean target with reach 50; 25/75 matches one effector with reach 75. | identical poses | MEAS |
| Pivot | wrist = effector position + R_effector · `IKPivot` | < 0.0001 cm | MEAS |

Not ported, because HIK solver behaviour belongs to the other agent:
- **Partial reach** (< 100) is not a linear lerp from FK to the target. One effector at reach 50 lands
  12.9 cm off the straight lerp and swings the elbow out of the plane. The first-pass `applyReach` was
  removed for this reason.
- **Pull.** `IK Pull` alone bends the spine and leaves the hips in place (spine Z −81° at pull 100).
  Hips move only with `IK Pull Hips`: at pull 100 / hips 100 they move from (0,100,0) to (52.3,130.9,0),
  and at pull 50 to (9.9,103.9,0). The first-pass `pullDisplacement` was wrong and was removed.

The raw poses are in `probes/aux_out.json` and `probes/aux2_out.json` for the HIK agent.

## 4. Remaining filters (`layers.js`)
The filters in `FBFilterManager` and their defaults are listed in `probes/filters.txt`.

**Constant Key Reducer** (MEAS, exact key sets on 30+ cases):
- Thresholds by node are T **9e-5**, R **0.009** and S **0.004**. The first-pass table from code had the
  T and S values swapped or misread.
- Key i is removed when the value range of each window `[i-2..i]`, `[i-1..i+1]` and `[i..i+2]` (clipped
  to the curve) is ≤ the threshold.
- This keeps two keys on each side of every change and drops the ends of flat runs at the curve ends.
- A slow ramp whose step is under the threshold survives.
- `Keep At Least One Keyframe` defaults to true; a fully flat curve keeps its first key.

**Resample** (MEAS, max error 7e-7):
- Evaluates the source curve at `t0 + n/fps` for every point before `t1`, then adds `t1`. This also works
  when `t0` is not on a frame.
- `With interpolation` made no difference on the test curve.

**Keys on Frame** (MEAS, exact): rounds each key time to the nearest frame at the filter's frame rate.

**Time Shift And Scale** (MEAS, exact): `t' = (t + shift) · scale`. Scaling is about time 0, after the
shift.

## Tests
`node --test tests/mobu_story.test.mjs tests/mobu_layers.test.mjs tests/mobu_auxfx.test.mjs` runs 15
tests, all passing. The tests read the probe JSON and print the max errors as diagnostics.
