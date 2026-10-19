# MotionBuilder 2027 mocap cleanup: what the binaries show

Scope: HumanIK characterization and retarget solver settings, Story clip blending and match-object
alignment, the curve filters (Butterworth, Peak Removal, Unroll, Key Reducing, Smooth), and FBX export.
The install at `C:\Program Files\Autodesk\MotionBuilder 2027` was not modified. All binaries were copied
to `C:\re_work\mobu` and analysed there.

Sources:
- SDK headers `OpenRealitySDK/include/fbsdk/{fbfilter,fbcharacter,fbstory,fbfbxoptions,fbapplication}.h`
  and `bin/config/Python/pyfbsdk_gen_doc.py`.
- Shipped scripts under `bin/config/PythonStartup` and `bin/config/Scripts` (`FBX Export.py`,
  `BatchExportCharacterAnimationTool.py`, `PlotSelectedCharStoryTracks.py`, Story samples).
- Ghidra 12.1.2 headless dumps, all in `C:\re_work\mobu`: `kfcurve.c` and `kf2.c` (`kfcurve.dll`, the
  filter math), `kdata.c` (`kdata.dll`, the UI filter wrappers), `ktimelinex.c` and `ktl2.c`
  (`ktimelinex.dll`, Story), `hikhost2016.c` (`extensions/hikhost2016.dll`, the HIK host), and `kc2.c`
  (`kcharacter.dll`).
- Raw table reads from the same DLLs, done with small PE readers. The parsed HIK property table is in
  `C:\re_work\mobu\kct.txt`.

Addresses are VAs in the 0x180000000 image base. `kf2.c` and `ktl2.c` were dumped by exported name, so
their addresses are the export thunks. The bodies follow each thunk in the same dump.

Labels: **OBSERVED** means it is in the code, the strings or the bytes. **INFERRED** means it is reasoned
from observed code or names. **UNKNOWN** means the evidence does not settle it.

Limits of the evidence:
- The HumanIK solver core is statically linked into `hikhost2016.dll` without symbols. The same library
  ships *with* symbols as `OpenRealitySDK\lib\x64\HIK2016\humanik.lib`: 1253 of 1342 functions are
  byte-identical to the copy in hikhost2016.dll. It was decompiled and driven in-process in
  **`HUMANIK_SOLVER.md`**, which is now the authority for the solver math. The §2 rows below were
  corrected against it on 2026-10-15; each corrected row says so. Statements in §2 that remain INFERRED
  were neither confirmed nor contradicted.
- The Story evaluation of the blend (how two clips' poses are mixed per bone) was not decompiled. Only
  the matching code and the blend-weight function were.

---

## 1. Filters (`kfcurve.dll`, UI in `kdata.dll`)

### 1.0 How they are organized (OBSERVED)
`kdata.dll` registers the UI filters by name and wraps a `KFCurveFilter*` from `kfcurve.dll`. The pairs
are: "Butterworth" → `KFCurveFilterButterworth`, "Peak Removal" → `KFCurveFilterPeakRemover`, "Key
Reducing" → `KFCurveFilterKeyReducer` ("Bezier Fit/Key Reducer"), "Smooth" → `KFCurveFilterSmooth`,
"Smooth Translation" → `KFCurveFilterSmooth2`, "Unroll Rotations" → `KFCurveFilterUnroll`, plus "Constant
Key Reducer" and "Resample". The strings are at `kdata 180023d98…1800289ec`.

Every filter works on one FCurve (a scalar key list) over `[Start, Stop]` (props at +0x158/+0x160). Each
returns an error string when it has nothing to do, for example `"PeakRemover, No key to be filtered."`.

### 1.1 Butterworth (OBSERVED unless marked)
Code: `Apply` `FUN_180038ce0`, constructor `FUN_180037820`, `ComputeFilterCoefficients @180002824`,
`ButterworthFilter @180001988`.

Defaults:

| Property | Default | Source |
|---|---|---|
| `CutOffFreq` (UI "Cut-off Frequency (Hz)") | 7.0 | `0x401c…` at ctor +0x2c8, and `kdata FUN_180003910` +0x290 |
| `SamplingRate` (UI "Sampling Rate (fps)") | 1/frame (scene fps) | ctor: `1.0 / KTime::GetOneFrameValue().seconds` |
| `OrderOfFilter` | 4 | `0x4010…` at +0x400 |
| `InternalRate` | `KFCURVE_GetButterworthFilterInternalRate()` | global; its value is UNKNOWN |
| `KeysOnFrame` | bool | aligns the start to the output period |

Algorithm:
1. If `SamplingRate ≤ 0`, it fails with "Bad Sampling Rate". It needs at least 2 keys.
2. It sets `dt_in = 1/InternalRate` (the internal resample period) and `dt_out = 1/SamplingRate`.
3. The working interval is `[start + dt_out, stop − dt_out]`. With KeysOnFrame, the start is first
   snapped up to a multiple of `dt_out`. The first and last frames are left untouched.
4. It builds the buffer `x[N+500]`, where `N = (stop − start)/dt_in`:
   - **Pad 250 samples on each side by linear extrapolation**:
     `x[i] = v0 + (i − 250)·s0` for `i < 250`, using the slope `s0` from a one-sample forward/back
     difference at the start. The end is padded the same way (constant 250.0 at `DAT_1800a5810`).
   - The middle holds `curve.Evaluate(start + k·dt_in)`.
5. It computes the coefficients for `(Fc, dt_in, order)` and runs the filter **forward** over the whole
   buffer, then recomputes the coefficients (which resets the state) and runs it **backward**. This is
   zero-phase filtfilt. The effective order is 2×`order`, with no lag.
6. It deletes every key inside `[start, stop]` and writes one key per `dt_in` sample. The key attribute
   is `0x108`, which is INFERRED to mean cubic auto tangent. It then calls
   `KFCurveUtils::Resample(curve, dt_out, …)` to bring the keys back to the output rate.

Coefficients (`ComputeFilterCoefficients`). This is a cascade of bilinear-transform biquads:
```
wc  = tan(π · Fc · dt)                          wrap π·Fc·dt into (−2π,2π] first
w2  = wc²
sections = order/2 + order%2
for k = 1 .. order/2:
    φ   = π · (k − (order odd ? 0 : 0.5)) / order
    c   = cos(φ) · wc
    den = 1 + 2c + w2
    a1  = −2(w2 − 1)/den          stored at [k-10]
    a2  = −(1 − 2c + w2)/den      stored at [k]
    b0  = w2/den                  stored at [k+30]
    b1 = 2, b2 = 1                numerator (1 + 2z⁻¹ + z⁻²)
if order odd: first-order section  a1 = (1 − wc)/(1 + wc), b0 = wc/(1 + wc), b1 = 1
```
Per sample, per section (`ButterworthFilter`, Direct Form II):
`w = x + a1·w1 + a2·w2;  y = b0·(w + 2·w1 + w2);  w2 = w1; w1 = w`.
The section state is primed with `x0/(1 − a1 − a2)·…` in `Apply` (the loop at `+0x520`). This sets
DC-steady initial conditions, so the edges do not ring.

### 1.2 Peak Removal (OBSERVED)
Code: ctor `FUN_18004e370`, `Apply` `FUN_18004e6a0`, `PeakRemovalStep @1800026b7`.

Defaults: `TimeTolerance` (+0x198) = 3.0 and `ValueTolerance` (+0x1a0) = 3.0.

`Apply` runs `PeakRemovalStep` up to **20 times**, stopping at the first pass that removes nothing. One
step over the keys `i ∈ [a, b)`:
1. Set `meanDt² = Σ(tᵢ − tᵢ₋₁)² / n`.
2. For each key i, find every neighbour j (left and right) with `(tⱼ − tᵢ)² < meanDt²·TimeTol`. Over
   those neighbours, take `sMin = min(slopeᵢⱼ)` and `sMax = max(slopeᵢⱼ)`, where
   `slopeᵢⱼ = (vⱼ − vᵢ)/(tⱼ − tᵢ)`. If there are none, both are 0.
3. Set `meanSpread² = Σ(sMax − sMin)² / n`.
4. Key i is a **spike** when `sMin·sMax < 0` (the slopes point in opposite directions, so it is a local
   extremum) and `(sMax − sMin)² > meanSpread²·ValueTol`.
5. For each spike, grow left index L and right index R while the neighbours are also over threshold.
   Then replace the spike with the linear interpolation
   `v = v_R − ((t_R − t)/(t_R − t_L))·(v_R − v_L)`.

### 1.3 Smooth (`KFCurveFilterSmooth`, OBSERVED with one caveat)
Code: ctor `FUN_180054160`, `Apply @180002707`.

Defaults: `SampleCount` = 8 (UI, `kdata FUN_180017840`, `0x4020…`). The ctor stores `Width = 8 ×
plotPeriod`.

For each interior key, it evaluates the source curve at `SampleCount+1` evenly spaced times across a
window of `Width` centred on the key. The window is clipped at the curve ends. The weights are:
```
u   = 1 − |i − N/2| / (N+1)             N = SampleCount
w_i = 1.078 · exp(−u² / 0.274)          constants at 1800aacd0 / 1800aace0
v'  = Σ w_i·f(t_i) / Σ w_i
```
**Caveat (UNKNOWN):** as decompiled, `u` is largest at the window centre, which would down-weight the
centre sample. That is unusual for a smoothing kernel. The disassembly near `@180054300` was not
checked. For Fulcrum, use `x = |i − N/2|/(N/2)` and `w = exp(−x²/0.274)`, which is the INFERRED intent.

Edge ramp (OBSERVED): within one half-width of the curve's first or last key, the result is blended
back to the original, `v'' = (1 − d/hw)·v + (d/hw)·v'`, where d is the distance to the edge. Smoothing
therefore fades in, and the endpoints are never moved.

"Smooth Translation" (`KFCurveFilterSmooth2`) needs exactly 3 curves ("This filter only works on 3
curves") and has `Factor` = 1.5 by default (`kdata FUN_180010e30`). It also has `DampGT`. Its body was
not analysed (UNKNOWN).

### 1.4 Key Reducing (`KFCurveFilterKeyReducer`, OBSERVED)
Code: ctor `@180002423`, `KeyReducer @18000234c`, `Subdivise @180002ca2`, `FindMaxError @1800012ad`.

Defaults: `Precision` = 1.0 (+0x198) and `KeySync` = false.

The algorithm is a recursive Bezier fit, Douglas–Peucker style, on a curve that starts with two keys:
1. Keep the first and last keys with their original tangents: right derivative at the start, left at the
   end.
2. `Subdivise(a, b)`: find the source key k in `(a, b)` with the largest `(fit(t_k) − v_k)²`.
3. If that error is at most `Precision²`, stop. Otherwise insert key k with its original left and right
   derivatives. If they are equal, the key is marked smooth (0x400); otherwise broken (0xc00). Recurse
   on `(a, k)` and `(k, b)`.
4. Recursion depth is capped at 10, through the counter at +0x1a0.

### 1.5 Constant Key Reducer (OBSERVED)
Code: ctor `@1800013ca`, `Apply @1800012f8`.

The thresholds depend on which curve node is being reduced, selected by the node's name:

| Node | Threshold |
|---|---|
| Translation (T / "Lcl Translation") | 0.009 |
| Rotation (R / "Lcl Rotation") | 0.009 |
| Scaling (S / "Lcl Scaling") | 9e-5 |
| Default | 0.009 |
| `DerivativeTolerance` | 0.004 |

Quaternion nodes are skipped. `KeepOneKey` and `KeepFirstAndLastKeys` are flags. The removal loop body
was not analysed. It is INFERRED to delete key i when
`|vᵢ − vᵢ₋₁| < thr` and `|vᵢ₊₁ − vᵢ| < thr`, with matching derivatives within `DerivativeTolerance`.

### 1.6 Unroll (`KFCurveFilterUnroll`, OBSERVED)
Code: ctor `@180001c03`, `DoUnroll @180002c43`, `InterpolationQualityFactor @1800014e2`.

- It needs a 3-curve rotation node and fails with "Works only on rotation CurveNodes" otherwise. It first
  key-syncs X/Y/Z (`KFCurveFilterKeySync`) so all three channels have keys at the same times.
- Walking the keys in time order, it calls
  `KMBRotationOrder::V2VRef(order, out, cur, prevOut)`. This is INFERRED to choose, among the Euler
  triples that give the same rotation (`(x,y,z)+2πk` and the flip
  `(x+π, π−y, z+π)`), the one nearest the previous unrolled key. The output is written back as float keys.
- Quality metric: with `e1, e2` two Euler triples and
  `q = quat(M(e1)·M(e2)⁻¹)`, it computes `θ = 2·acos(q.w)·(180/π)` and
  `QF = θ² / |e1 − e2|²`. When `|e1 − e2|²` is at most ε, QF is 1. A low QF means the Euler path takes a
  much longer arc than the true rotation does, so an unroll is needed. `QualityTolerance` comes from
  `KFCURVE_GetRotationFilterUnrollQuality()`; its value is UNKNOWN. `TestForPath` comes from
  `KFCURVE_GetRotationFilterUnrollTestForPath()`.
- `KFCurveFilterUnrollDof` adds per-axis `Min/MaxFreedom` limits and `EnableMin/MaxX/Y/Z` flags. With
  these it unrolls toward a DOF window, not toward the previous key (INFERRED).

---

## 2. HumanIK character solver (`hikhost2016.dll` + `kcharacter.dll`)

### 2.1 Property table (OBSERVED)
`kcharacter.dll` holds a static table at `1802f45a0` with a stride of 0x30. Each row is
`{-1, 0, type, 0, min, max, flags, ?, default}` followed by a name pointer. It is copied into the
runtime table by `FUN_1802031d0`. For `type==1` (percent), max and default are stored as 0..1 and
multiplied by 100 (`DAT_1802f8514` = 100.0) for display. The complete dump of 283 rows is in
`C:\re_work\mobu\kct.txt`.

`hikhost2016.dll FUN_1800c8450` maps the properties to HIK parameters through a 249-entry table at
`1800e6b30`, laid out as `{hikName, fbPropName, fbModeName, ?}`. When it pushes a value it divides by
100 (`DAT_1800ec108` = 100.0) unless the parameter is in id range 0xc1–0xc8 or is not a percent type;
those use a divisor of 1.0 (`DAT_1800ec0f8`). **So every "%" in the UI is a 0..1 weight inside HIK.**

Defaults relevant to mocap cleanup (OBSERVED, `kct.txt`):

| Group | Property | Default | Range |
|---|---|---|---|
| Solve | PullIterationCount | 10 | 0–30 |
| Solve | Posture (Biped/Quadruped) | 0 | enum 0–2 |
| Retarget | ScaleCompensation (Mode) | 100 | 1–1000 |
| Retarget | HipsHeightCompensation | 0 | ±1000 (cm) |
| Retarget | AnkleHeightCompensation | 0 | ±1000 |
| Retarget | AnkleProximityCompensation | 0 | ±1000 |
| Retarget | MassCenterCompensation | 80 | 0–120 |
| Retarget | HipsTOffsetX/Y/Z, ChestTOffsetX/Y/Z | 0 | ±1000 |
| Reach (actor/source) | ReachActorLeft/RightAnkle (T) | **100%** | 0–100 |
| Reach (actor/source) | ReachActorLeft/RightAnkleRotation | **100%** | |
| Reach (actor/source) | ReachActorKnee/Chest/Head/Wrist/Elbow/Toes/Fingers | 0% | |
| Floor | FloorContact (feet), HandFloorContact | off | bool |
| Floor | FloorPivot / HandFloorPivot | 0 = Auto | enum 0–3 |
| Floor | FootBottomToAnkle | 7.5 | 0–10000 |
| Floor | FootBackToAnkle | 4.5 | |
| Floor | FootMiddleToAnkle | 13 | |
| Floor | FootFrontToMiddle | 7 | |
| Floor | FootInToAnkle / FootOutToAnkle | 5 / 5 | |
| Floor | Hand equivalents (…ToWrist) | 7.5 / 4.5 / 13 / 7 / 5 / 5 | |
| Floor | ContactSize, HandContactSize | 100% | 10–1000 |
| Floor | AutomaticToes, AutomaticFingers | off | |
| Floor | FootContactType / HandContactType | 0 = Normal | enum 0–4 |
| Floor | FootFingerContactMode / Hand… | 1 | enum 0–3 |
| Stiffness | ParamFootContactStiffness / Hand | 0% | |
| Stiffness | ParamFoot/HandFingerContactRollStiffness | 0% | |
| Pull (control rig) | CtrlPullLeft/RightHand | **100%** | |
| Pull (control rig) | all other CtrlPull (foot, knee, toe, elbow, finger, head, chest-hand) | 0% | |
| Resist | CtrlEnforceGravity | **60%** | |
| Resist | CtrlResistLeft/RightCollar | 50% | |
| Resist | CtrlResist Knee/Elbow, MaximumExtension…, CompressionFactor… | 50% each | |
| Resist | CtrlResistHips/Chest Position/Orientation | 0% | |
| Stiffness | ParamCtrlSpineStiffness, ParamCtrlNeckStiffness | 0% | |
| Stiffness | Chest/Collar/Neck/HeadReduction | 0% | |
| Roll | Left/Right UpLeg/Leg/Arm/ForeArm Roll | 100% | |
| Roll | Leaf roll 1 (`ParamLeaf…Roll1`) | 50% | |
| Roll | RollExtractionMode | 0 | enum 0–2 |
| Roll | RollSolver | 0 | enum 0–2 |
| Stabilization | Damping, Displacement, Hips/Ankle/WristDisplacementDamping | 0 | 0–100 |
| Stabilization | Orientation, DisplacementMemory | 0 | 0–0.25 s |
| Stabilization | Stabilization | 0 | 0–100 |
| Stabilization | AnkleStabilizationTime | 0 | 0–1 s |
| Stabilization | AnkleStabilizationPerimeter | 0 | 0–15 cm |
| Stabilization | AnkleStabilizationAngularPerimeter | 0 | 0–60° |
| Stabilization | AnkleStabilizationFloorProximity | 0 | 0–10 cm |
| Stabilization | AnkleStabilizationDamping | 0 | 0–10 |
| Stabilization | AnkleStabilizationRecoveryTime | 0 | 0–1 s |
| Misc | HipsTranslationMode | 0 = World Rigid | enum 0–2 |
| Misc | ContactBehaviour | 2 | enum 0–3 |
| Misc | ShoulderCorrection, Mirror | 0 | |

These defaults are MoBu's own table (`kct.txt`). The bare SDK property set
(`HIKPropertySetStateCreate`) starts differently:
- FootBottomToAnkle = 0
- CtrlPullLeft/RightFoot = 1.0
- Up/forearm Roll = 0.6
- ContactBehaviour = 0 (it is a hidden slot at `+0x634`)

The host overwrites all of these.
*Source: `C:\re_work\hik\results\properties.csv`, from HUMANIK_SOLVER.md §0.2.*

Enum labels in `kcharacter.dll`, in this order: `Auto, User | Relative, Absolute | Ankle, Toes, Wrist,
Fingers | Normal, FingerBase, Hoof, ToeBase | Sticky, Spread | World Rigid, Body Rigid | Never Sync`.
The mapping below is OBSERVED in the SDK header `hikproperty.h`. *Corrected 2026-10-15; the old INFERRED
FootContactType order was wrong.*
- `FloorPivot` = {Auto, Ankle, Toes} for feet and {Auto, Wrist, Fingers} for hands.
- `FootContactType` = {Normal (6 markers), Ankle, ToeBase, Hoof}. `HandContactType` =
  {Normal, Wrist, FingerBase, Hoof}.
- `FootFingerContactMode` = {Sticky, Spread, Sticky and Spread (the average)}.
- `HipsTranslationMode` = {World Rigid, Body Rigid, Trajectory}.

Debug dump `FUN_1800c51f0` (OBSERVED): every one of the 0x2c (44) effectors carries `mT` (reach T), `mR`
(reach R), `mPull` and `mResist`. That is the full per-effector state the solver reads.

### 2.2 What the settings do
*Corrected 2026-10-15 against the decompiled and executed HIK2016 library. Section numbers in this list
refer to HUMANIK_SOLVER.md. The rows that were wrong are marked "was:".*

- **Reach T** (0..1): OBSERVED + MEASURED, §2.
  - *Was: "the weight with which the IK target overrides the FK result".*
  - Reach T is not a final-pose blend. Each aim step rotates the hip so the ankle moves a fraction T of
    the remaining error, and the knee's law-of-cosines distance is interpolated by T.
  - Low T values therefore move the ankle along an arc, which at T = 0.1 swings it backwards.
  - The defaults reach only the ankles, at 100% T and R.
- **Reach R** (0..1): MEASURED, §2.3. The ankle rotation is a slerp by R from the rotation the leg aim
  carried the ankle to, toward the goal rotation.
- **Pull** (0..1): OBSERVED, §4.
  - *Was: "solved over PullIterationCount (10) passes".*
  - There are **PullIterationCount + 2** passes, so 12 by default.
  - Each limb's pull is raised to the power `max(1, 0.2·passes)` before use, which is 2.4 at the default.
  - Each limb is a spring: gain `pull` from its FK length, then a soft Bezier zone, then 1:1 overflow
    beyond the limb's maximum length.
  - Within one limb, pull is a priority between the knee, ankle and toe effectors.
- **Resist, knee and elbow**: OBSERVED + MEASURED, §3.
  - *Was: "keep limbs from locking straight", with no formula.*
  - MaximumExtension damps the goal distance near full extension with
    `r = sqrt(Resist·MaxExtension)` and exponent `30(1 − r) + 15`. At the 0.5/0.5 defaults a reachable
    target is missed by about 0.016 cm.
  - CompressionFactor is the spring gain of a limb shorter than its rest length. That is how a planted
    leg pushes the hips back up: 0 / 0.5 / 1 keeps 100 / 40 / 15 % of a forced hips drop.
- **EnforceGravity** (60%): OBSERVED + MEASURED, §4.4.
  - *Was: "keeps the hips from being pulled upward by the hands". That was wrong.*
  - It removes the **horizontal** part of the hands' pull on the hips, `F' = lerp(F, u(u·F), g)` with u
    the body up axis. The vertical part is kept.
  - At 100% a sideways hand goal lifts the hips (39 cm in the test) instead of sliding them.
- **Floor contact**: OBSERVED + MEASURED, §5.
  - *Was: "ankle raised and pitched about the pivot; Auto picks the lower line; stiffness 1 holds the
    foot flat".* All three parts were wrong.
  - The markers are 6 points (heel, ball and toe tip, inner and outer) at `BottomToAnkle` below the ankle.
  - **Stiffness 0, the smooth solve.** It first clamps the ankle to ≥ `BottomToAnkle` above the floor.
    It then runs 4 rigid refits of two heel/ball marker triangles, each clamped to the floor. The foot
    ends with only its lowest edge touching, and is not forced flat. The toe joint then bends the
    toe-tip markers onto the floor.
  - **Stiffness 1, the stiff solve.** It lifts the foot straight up by the deepest of the 6 markers and
    keeps the input rotation.
  - **In between**, translation is lerped and rotation nlerped by `1 − stiffness`.
  - **Pivot.** Auto does no correction. Ankle restores the ankle's x and z. Toes restores the toe base's
    x and z.
  - **Roll stiffness** acts on the toe/finger tips. It removes the sideways component of each tip's
    floor correction (§5.6).
  - ContactSize has no reader in the solver library. It is presumably applied by the MoBu host; still
    UNKNOWN.
- **AnkleStabilization\***, **DisplacementDamping**: OBSERVED absent from the HIK2016 solver library and
  its property set. The strings exist only in `hikhost2016.dll` and `kcharacter.dll`, so they are MoBu
  host-side filters. The description is still INFERRED: a planted-foot lock within `Perimeter` cm,
  `AngularPerimeter`° and `FloorProximity` cm for at least `Time` s, released over `RecoveryTime` s.
  Hips/Ankle/Wrist DisplacementDamping is a per-frame low-pass.

---

## 3. Story: clip matching and blending (`ktimelinex.dll`)

### 3.1 Settings (OBSERVED)
`FUN_18003ea90` (load) and `FUN_18003fd20` (save) read and write `@Application.txt [Story]`:

| Key | Default | Values |
|---|---|---|
| `MatchTranslation` | Gravity | None / Direct / Gravity → 0 / 1 / 2 |
| `MatchRotation` | Gravity | None / Direct / Gravity → 0 / 1 / 2 |
| `MatchWhen` | StartOfClip | Now / Stop / Start / StartStop / StopOfClip / StartOfNextClip / MiddleOfOutBlend → 0..6 |

The SDK enums (`fbstory.h`) define TranslationGravityXZ as "along global X and Z" and RotationGravityXZ
as "around global Y".

### 3.2 Match algorithm (`KTimelineXTrackAnimation::MatchClip @180003cce`, OBSERVED)
`KClipAnimation::Match @18000172b` calls it with the global modes and the current time. `KMatchClips`
(`operator() @180002a27`) applies it to every clip marked for move, walking forward and then backward
from the reference clip.

1. **Choose the reference clip and the two sample times** from `MatchWhen`:
   - Start: the previous clip, at the selected clip's start.
   - Stop / StopOfClip: the previous clip's end.
   - StartOfNextClip: the next clip.
   - StartStop / MiddleOfOutBlend: `(start + stop)/2` of the overlap.
   - Now: the current time; whichever neighbour is closer to the clip's midpoint is the reference.

   Both times are clamped into each clip's `[Start, Stop − 1 tick]`.
2. Clear the selected clip's stored offset matrix (at `+0x19d8…+0x1a48`) and keep the old one, `O_old`.
3. Evaluate the **match object** (pivot) global transform:
   - `A` = the reference clip's match object at `t_ref`.
   - `B` = the selected clip's match object at `t_sel`, recomposed with `O_old`.

   The match object is the pivot property, or by default the clip's "special global transform" of its
   root. A `KTransformNode` passed in uses `GetTransfromNodeXForm` instead.
4. Translation: None keeps `B.t`. Direct uses `A.t`. **Gravity uses `(A.t.x, B.t.y, A.t.z)`**, keeping the
   selected clip's own height.
5. Rotation:
   - None keeps `B.R` and Direct uses `A.R`.
   - **Gravity** takes the X axis of each rotation, `fA = R_A·(1,0,0)` and `fB = R_B·(1,0,0)`
     (`DAT_1801141a0` = 1), zeroes the Y component, and builds the shortest-arc quaternion
     `q = GetQuaternionFromPositionToPosition(fB_xz, fA_xz)`. This is a rotation about world Y only.
     The result is `R = q·R_B`.
6. Scale always comes from `B`.
7. New offset: `O_new = TRS(R, T, S_B) · B_raw⁻¹`, stored as the clip's Offset T/R (exposed as "Clip
   Offset T & R").

### 3.3 Blend weight (`KClipAnimation::GetPreBlendFactor @180002fc2`, OBSERVED)
`u = (t − preBlend.start)/preBlend.duration`, or 1 when the duration is 0. Then `w = curve(u)`, where
the curve is chosen by the interpolator at +0xf90:

| Interpolator | Curve |
|---|---|
| 0 | the clip's custom FCurve |
| 1 | linear, `w = u` |
| 2 | `sDefaultInterpolator_EasePreBlend` |
| 3 | `SlowInBlend` |
| 4 | `FastInBlend` |

The result is clamped to [0,1]. Post-blend is the mirror. The default curve objects come from
`KClip::CreateDefaultInterpolatorCurve("UserEaseIn",2 / "UserEaseOut",3 / "UserTimeWarp",0)`. Their key
values are UNKNOWN; INFERRED as smoothstep-like Bezier eases.

FBX persistence (OBSERVED, `FUN_1800976e0`): a clip stores `Matching{Translation, Rotation}`,
`PivotAlgorythm`, `In/Out/WarpInterpolatorStr`, `PreBlendTS/PostBlendTS`, `Scale`, `Ghost{…}`,
`AutoLoop{LoopTranslation}` and `GhostManipulatorOffset`.

---

## 4. FBX export (SDK and scripts, OBSERVED)
- `FBX Export.py` offers FBX 2019 / 2018 / 2016 / 2014 / 2013 (`FBFileFormatAndVersion.kFBFBX2019` first)
  and calls `FileSave` with `FBFbxOptions(False)`.
- `FBFbxOptions` (`fbfbxoptions.h` lines 172–334) has per-element `Bones/Characters/Story/…` actions and
  per-element `*Animation` bools. Other fields: `EmbedMedia`, which is true by default and the only
  option that is; `UseASCIIFormat`; `SaveCharacter / SaveControlSet / SaveCharacterExtensions` for
  `SaveCharacterRigAndAnimation`; `TransferMethod` (`kFBCharacterLoadRetarget / Plot / PlotIfSampled`);
  `RetargetOnBaseLayer`; and `SetPropertyStaticIfPossible`.
- `FBPlotOptions` (`fbapplication.h:185`) has `mPlotOnFrame`, `mPlotPeriod` (1/fps), `mPlotTangentMode`,
  `mUseConstantKeyReducer`, `mConstantKeyReducerKeepOneKey`, `mPlotTranslationOnRootOnly`,
  `mPlotLockedProperties`, `mPlotAuxEffectors`, `mPreciseTimeDiscontinuities` and
  `RotationFilterToApply` (None / GimbleKiller / Unroll).
- The game-export path (`BatchExportCharacterAnimationTool.py`) is: plot the character onto the
  **skeleton**, then save with `SaveCharacter=False, SaveControlSet=True`. Plot means bake every frame at
  `PlotPeriod`, then optionally run the constant key reducer with the thresholds in §1.5 and the rotation
  filter.

---

## Port to Fulcrum (ranked)

Fulcrum already has foot locks, slerp-based smoothing (`smoothstep`, `app.js:246`), shots and a Blender
FBX export through `server.py`. The ranking puts the largest cleanup gain for the
sprint → plant → slide → kneel → rise take first.

1. **Zero-phase Butterworth on positions and on quaternion logs.**
   - Coefficients: `wc = tan(π·Fc/fps)`. For each pair k = 1..order/2, use `φ = π(k − 0.5)/order`,
     `c = wc·cos φ`, `den = 1 + 2c + wc²`, `a1 = 2(1 − wc²)/den`, `a2 = −(1 − 2c + wc²)/den`,
     `b0 = wc²/den`, numerator (1, 2, 1).
   - Run Direct Form II forward, reset, then backward.
   - **Pad 250 samples on each side by linear extrapolation** (the endpoint value plus the endpoint
     slope), and prime the state to steady DC. Leave the first and last frames unchanged.
   - Defaults: `Fc = 7 Hz` and `order = 4`. For rotations, filter `log(q_ref⁻¹·q)` per bone after
     hemisphere-aligning (`dot < 0 → −q`), then take `exp` back. Use a lower Fc (4–5 Hz) on the hips
     during the slide and keep 7 Hz on the limbs.
2. **Peak Removal before any smoothing.**
   - `meanDt² = mean((Δt)²)`. Per frame, take the min and max slope to the neighbours within
     `√(3·meanDt²)`.
   - A frame is a spike when `sMin·sMax < 0` and `(sMax − sMin)² > 3·mean((sMax − sMin)²)`.
   - Grow the run while the neighbours are also spikes, and replace it by linear interpolation between
     the bracketing good frames. Repeat up to 20 passes.
   - Run it per channel on root translation and on each joint's rotation log (3 channels). This removes
     the marker-swap pops that Butterworth would only smear.
3. **Unroll and hemisphere continuity on import and before export.**
   - For quaternions: `if dot(q_i, q_{i−1}) < 0: q_i = −q_i`.
   - For any Euler bake to FBX: choose, among `e + 2πk` and `(x + π, π − y, z + π) + 2πk`, the triple
     nearest the previous frame.
   - Flag a frame when `QF = (2·acos|q_rel.w|·180/π)² / |Δe|² < QualityTolerance`; start with 0.5 and
     tune it.
   - UE and FBX readers interpolate Euler curves, so unroll is the cheapest fix for the "spin" at the
     kneel.
4. **HIK-style floor contact for the plant, slide and kneel.**
   *Corrected 2026-10-15. The old pivot-line pitch and the "stiffness holds flat" reading were wrong. The
   exact algorithm is in HUMANIK_SOLVER.md §5 and §9.*
   - Markers in the ankle frame, scaled to the character's foot:
     `{back 4.5, middle 13, front 7, in 5, out 5, bottom 7.5}`.
   - **Smooth result.** Clamp the ankle to at least `bottom` above the floor. Then run 4 iterations of a
     rigid refit on the triangles (heelIn, heelOut, ballIn) and (heelOut, ballIn, ballOut). In each
     triangle, clamp the marker y values to the floor and apply `frame(clamped)·frame(current)⁻¹` to the
     ankle. Then bend the toe joint until the toe-tip markers touch the floor.
   - **Stiff result.** Lift the foot straight up by the deepest of the 6 markers.
   - **Combine.** `T = lerp(stiff, smooth, 1 − stiffness)` and `Q = nlerp(stiff, smooth, 1 − stiffness)`.
   - **Pivot.** Ankle restores the ankle's x and z. Toes restores the toe base's x and z. Auto does
     nothing.
   - Feed the result into the two-bone IK with reach T = R = 1 on the ankles.
   - For the kneel, add a knee contact point. HIK has none: use the stiff test with a single marker at the
     patella.
5. **AnkleStabilization lock as automatic foot-lock detection**, to replace manual lock marks.
   - Lock frame i when `|p_i − p_anchor| < Perimeter` (start at 2–4 cm), the angle between them is less
     than `AngularPerimeter` (10°), and `height < FloorProximity` (3 cm), sustained for at least
     `Time` (0.1 s).
   - On release, blend out over `RecoveryTime` (0.15 s) with a critically damped spring of
     `Damping` (5).
   - During the slide, allow a moving anchor: replace the hard hold with
     `anchor += clamp(v_foot, maxSlide)·dt`, so the slide keeps its ground contact without foot skate.
6. **Story Gravity match for stitching the shots** (`shot1/2/3`).
   - `T = (A.x, B.y, A.z)`. Take `R = qY·R_B`, where `qY` is the shortest arc from `xz(R_B·X̂)` to
     `xz(R_A·X̂)`. Use the hips (or root) as the match object, sampled at shot1's last frame and shot2's
     first frame.
   - `Offset = TRS(R, T, S_B)·B⁻¹`, applied to the whole later shot.
   - Then crossfade over the overlap with `w = smoothstep((t − t0)/(t1 − t0))` (Ease). Use slerp per
     bone and lerp for root translation after the offset.
7. **Key reduction for the FBX** after the bake.
   - Use Bezier-fit Douglas–Peucker. Keep the endpoints with their source tangents. Insert the key with
     the largest `(fit − src)²` while it exceeds `Precision²` (default 1.0 in curve units; use 0.05 cm
     and 0.1° for UE). Keep the source tangents at inserted keys, and cap recursion at 10.
   - Then run the constant-key reducer: thresholds T and R 0.009, S 9e-5, derivative 0.004; keep one
     key.
8. **Retarget compensations when the take and the UE mannequin differ.** Apply them to the effector targets
   before IK.
   *Corrected 2026-10-15 from HUMANIK_SOLVER.md §7. The old ScaleCompensation and MassCenter formulas
   were wrong.*
   - `HipsHeightCompensation` (cm): move the hips and both hip targets along the
     ankles→hips up axis. In HIK the result is then damped by the leg CompressionFactor.
   - `AnkleHeightCompensation` (cm): move each ankle target along its own up axis, `R(q_ankle)·(0, h, 0)`.
   - `ScaleCompensation%`: scale the whole effector set about the reference by `value/100`. In Auto
     mode the factor is the destination/source leg length ratio.
   - `MassCenterCompensation` (80%): for each leg, clamp `hipTarget − ankleTarget` to the leg length.
     Then move the hips 80% of the way toward the average clamped hip position. This is a no-op whenever
     the legs can reach; it is not a centre-of-mass XZ shift.
9. **Pull and resist for the rise.**
   *Corrected 2026-10-15 from HUMANIK_SOLVER.md §3 and §4. The old "clamp at 0.5 of the remaining angle"
   and "project the hips back down" readings were wrong.*
   - With a hand pinned (hand on knee during the rise), run `PullIterationCount + 2 = 12` passes.
   - In each pass, every limb applies the HIK spring force: `GetPullForce`, gain `pull^2.4`, along its
     reach-weighted pull direction, accumulated on the hips and chest.
   - Before the limb IK, damp the knee and elbow goal with
     `w = ((a_t + a_c)/180)^(30(1 − r) + 15)` and `r = sqrt(resist·maxExt)`.
   - Apply EnforceGravity to the hands' pull on the hips as `lerp(F, u(u·F), 0.6)`. This removes the
     sideways drag, not upward motion.
10. **Smooth (Gaussian) as a light final pass on selected ranges.**
    - Use a `SampleCount + 1 = 9` tap kernel over 8 frames with `w = exp(−x²/0.274)`, where
      `x = |i − 4|/4`.
    - Fade it in and out linearly within the half-width of the range ends, so the range edges are left
      untouched. This replaces the current single smoothstep slerp in `smoothRange`.
