# Maya 2027: what the scripts and binaries show

Sources:
- the shipped MEL in `C:\Program Files\Autodesk\Maya2027\scripts` (`others\hik*.mel`,
  `AETemplates\AEHIK*`, `others\performAnimLayerMerge.mel`, `others\assumeRetargetRestPose.mel`)
- the FBX export preset `plug-ins\fbx\plug-ins\FBX\Presets\export\Autodesk Media and Entertainment.fbxexportpreset`
- Ghidra headless runs on copies of `KinSlice.dll` (ik solvers) and `AnimSlice.dll` (filterCurve, Euler
  filter, bakeResults), made in `C:\re_work\maya\bin`

Dumps are in `C:\re_work\maya\out`:
- `KinSlice.txt` and `AnimSlice.txt` (string xrefs)
- `euler.c` (every `TfilterEuler::*` method decompiled)
- `rpsolve.c` (the `TikRPsolver` vtable slots and their callees, two levels deep)
- `butter.c` (every `TfilterButterworth::*` body, plus `TfilterCurveCmd::updateButterworth`)

They were made with `C:\Projects\Fulcrum\tools\ghidra\DecompAt.java`, which decompiles given addresses and
follows their callees. The vtable was found from the MSVC RTTI type descriptor `.?AVTikRPsolver@@`, then the
Complete Object Locator, then the vtable.

Black-box tests ran in `mayapy.exe` (Maya standalone). The scripts and their JSON results are in
`C:\re_work\maya\tests`:
- `bw_test.py` and `bw_model.py` test the Butterworth filter (`bw_result.json`)
- `ik_test.py`, `ik_test2.py` and `ik_test3.py` test the RP, SC and spring solvers (`ik_result*.json`)
- `sc_test2.py` tests the SC twist (`sc_result2.json`)
- `ik_fit2.py` and `sc_fit4.py` are the fits

Addresses are VAs with image base 0x180000000. Nothing in the install was changed.

Labels: **OBSERVED** means it is in the code, strings or decompilation. **MEASURED** means it was fitted
against mayapy output; the residual is given each time. **INFERRED** means it is reasoned from observed code
or from documented Maya behaviour that matches the names. **UNKNOWN** means the evidence does not settle it.

---

## 1. ikRPsolver and ikSCsolver (KinSlice.dll)

### 1.1 Attributes (OBSERVED)
- `TdnVSTikHandle @ 1800eb530` registers these handle attributes:
  - `poleVector` (X, Y, Z) and `twist`
  - `twistType`, `rootTwistMode`
  - `dTwistValueType`, `dTwistStart`, `dTwistEnd`, `dTwistRamp` (R, G, B), `dTwistRampMult`,
    `dTwistControlEnable`
- `TikHandle::isRotatePlane`, `TikSolverKernel::rotatePlane`, `TikHandle::poleVector(bool)` and
  `setPoleVector` are exported.
- The solver names `ikRPsolver` and `ikSCsolver` are referenced from `isDefaultSolver @ 1801286a0`. There is
  also a profiler label, "Measure Footprint of TikRPsolver".

### 1.2 Where the solve lives (OBSERVED)
- RTTI: the type descriptor `.?AVTikRPsolver@@` is at RVA 0x23c628, and the primary vtable is
  `1801ba788`. The `TikSolverKernel` subobject is at offset 0x2b8, and its vtable is `1801bad78`. Its slots
  line up with the exported `TikSolverKernel` vtable `1801bc9b8`:
  - slot 4 `preSolve`: thunk `180118a94` → `180111c60`
  - slot 5 `doSolve`: thunk `180118960` → `1801106e0`
  - slot 6 `postSolve`: thunk `180118a78` → `180110b80`
  - slot 8 `setHandleGroup`: thunk `180118c18` → `180112260`
- Slot 7 (`180118c88`) returns the string `"ikSCsolver"`. RP and SC are one single-chain implementation.
  Bit 0x40 of the kernel flags at `+8` selects the rotate-plane branch.
- There is no `TikRPsolver` class in `KinSlice.dll` that is separate from this. `ik2Bsolver.mll` and
  `ikSpringSolver.mll` are separate plugins.

### 1.3 Algorithm: a precalculated reach table, not a closed-form solve (OBSERVED, MEASURED)
**Precalculation** (`180110bb0`, called from `preSolve`, cached in `TsingleChainData`):
1. The rest pose is the preferred-angle pose. `TikHandle::jointChainGeometry` gives the chain length and
   the rest plane normal `N0`, which is stored at `+0xe8`. The rest radius² is `|e_rest|²`.
2. **Near table.** Place the goal at the root (the origin) and run the limit-aware optimiser
   `FUN_180122c80`. A `TsingleChainRecorder` stores every pose it visits as `(radius², joint angles)`.
3. **Far table.** Place the goal at `1.5 · sqrt(len²/rest²) · e_rest` (`DAT_1801b9e30` = 1.5) and record
   again. If the far solve does not converge, the length estimate is doubled and the solve is retried.
4. Joint limits are passed into the optimiser as `TvarLimit` (`getJointLimits`), so both tables respect
   them.

**Solve** (`doSolve` → `1801106e0`):
1. `TikSolverKernel::doSolve` runs, then `180112740` does the reach step:
   - `D² = max(|goal − root|², k²·len²)`, where `k` is the kernel field `+0x48`.
   - Binary-search the near or far table for the two entries that bracket `D²`.
   - Blend the angles as `θ = s·A + (1−s)·B`. Solve for `s` with up to 100 safeguarded regula-falsi or
     Newton steps, evaluating FK through `plainBFGSfunc`. Stop when `|r²(s) − D²| ≤ tol·maxRadius²`.
   - Beyond the last entry, the chain is clamped to that entry. That is the fully extended pose or the
     limit pose.
2. If stiffness or damping is set, `stiffDampSolve` runs.
3. `180112ee0` then orients the root:
   - `q_arc = Tquaternion(e, goal)`, the shortest arc from the current effector vector to the goal
     vector. Then `d̂ = normalize(q_arc·e)`.
   - **RP branch** (flag 0x40):
     - Set `r = q_arc·N0`, then `r ← normalize(r − (r·d̂)d̂)`.
     - Skip this step if `| |r·d̂| − 1 | ≤ 1e-10`.
     - Set `n = normalize(d̂ × P)`, where `P` is the pole at handle-data `+0x00` and twist is at `+0x18`.
     - `φ = acos(r·n)`, clamped at −1 to π. Use `2π − φ` when `(r × n)·d̂ < 0`.
     - The root gets `q = axisAngle(d̂, φ + twist) · q_arc`.
     - If `d̂ × P` cannot be normalised, `φ = 0` and only `twist` is applied.
   - **SC branch:**
     - Set `q_a = quat(effectorFrame)·q_arc` and `q_g = quat(goalTransform)`.
     - `τ = π − 2·atan2(q_a·q_g, v·d̂)`, which is the swing-twist twist of the goal relative to the
       effector about `d̂`.
     - The root gets `axisAngle(d̂, τ)·q_arc`.
   - The result is decomposed with `TrotateXYZ::decompose1` in the root's rotate order.

**MEASURED, 2-bone RP** (`ik_test.py`, 192 cases: 8 goals × 6 poles × 4 twists, including 2 out-of-reach
goals):
- The knee is at `root + aĤ + h·rot(b, Ĥ, +twist)`, where:
  - `a = (L1² − L2² + d²)/(2d)` and `h = sqrt(L1² − a²)`, which is the law of cosines
  - `b = normalize(P − (P·Ĥ)Ĥ)`
  - twist is right-handed about `Ĥ = goal − root`
- Max knee error 5.0e-6 and max effector error 9.0e-6, from the solver tolerance.
- Out of reach: the chain is straight along `Ĥ` and the effector stops short. There is **no stretch and no
  soft IK**.
- Pole parallel to `H`: the bend stays in the arc-rotated rest plane. `P = (1, 1e-7, 0)` already flips the
  knee to +Y, so the guard is only `|d̂ × P| > ~0` and the flip is abrupt.
- The default pole on creation is `2·knee − root − effector`: `(0,0,2)` for the knee `(4,0,1)` and the
  effector `(8,0,0)`.

**MEASURED, N-bone shape** (`ik_test3.py` + `ik_fit2.py`, a planar 3-bone chain with rest bends (8°, 35°),
9 reaches):
- Compressing (`d < d_rest`): `θ = θ_rest − λ·∇r²(θ_rest)/|∇r²|`. That is a straight line in joint space
  along the steepest descent of radius² at the rest pose. For this chain the increment ratio was a constant
  0.6540, which equals the gradient ratio.
- Extending: `θ = (1 − s)·θ_rest`, a linear blend toward straight.
- Max error 0.0014°. So for 2 bones this reduces exactly to the law of cosines in the preferred-angle
  plane. For 3 or more bones, the distribution is set by the rest-pose gradient and not by CCD or FABRIK.

**MEASURED, straight chain:**
- With a preferred angle, the bend follows that angle's axis and sign: `pa = (0,−30,0)` gives
  `ry = −18.1…−120°`, and `(0,0,30)` gives `rz = +18.1…+120°`.
- With zero preferred angle, it still bends: positive `ry` on the mid joint in this joint orientation.
  Treat that direction as arbitrary and always set preferred angles.

**MEASURED, limits:** with knee `ry ∈ [−20°, 20°]` and the goal at 5, the knee stops at 20° and the
effector at 7.53. Nothing else compensates for the limit.

**MEASURED, ikSCsolver** (`sc_test2.py` + `sc_fit4.py`, 28 cases, `snapEnable = 0`):
- The chain is the RP shape rotated about `Ĥ` by `τ = twist_Ĥ(G_handle · K⁻¹)`. `K` is the world
  orientation of the effector's parent joint for the same goal with zero extra twist. The fit residual is
  0.0°, and the swing residual is 1.7e-6°.
- So SC has no pole. It rolls the chain so that the effector frame follows the handle's rotation about the
  reach axis.
- On creation the handle's rotation is set to the effector frame (`r = (0, 14.04, 0)` here). With
  `snapEnable = 1` the handle rotation snaps back to the effector after every solve.

### 1.4 ikSpringSolver (`plug-ins\ikSpringSolver.mll`, OBSERVED, MEASURED)
The plugin was copied to `C:\re_work\maya\bin` and dumped to `out\spring.c` and `out\spring2.c`.

Classes and entry points:
- RTTI: `.?AVikSpringSolver@@` is an `MPxIkSolverNode` with vtable `18000ed20`. The math is in
  `ikSpringSolverKernel`. The plugin also registers the `ikSpringSolverRestPose` and
  `ikSpringSolverCallbacks` commands.
- `doSolve` is `180001610`. It reads the handle's dynamic attributes: `springRestPose` (rest joint
  positions), `springRestPoleVector`, `springAngleBias` (a ramp), `poleVector` and `twist`. These
  attributes are created on the first solve. It then builds the kernel (`180005420`), solves
  (`1800072f0`), and applies two `rotateBy` passes per joint (`1800057c0` → `1800056c0`): the bend
  angles, then the root rotation.

Algorithm (`180007010` → `180007570` → `180007830`):
1. Put the chain in the rest pose. For each inner joint, compute the interior angle `a_j`, the angle
   between `p_{j−1} − p_j` and `p_{j+1} − p_j`, in degrees (180 is straight). Set `d_rest = |e − root|`
   and `D = |goal − root|`.
2. The bias for inner joint j is `b_j = 1 + clamp(v_j, 0, 1)`, where `v_j` is the j-th **entry value** of
   the `springAngleBias` ramp (`180002340`, `MRampAttribute::getEntries`). Ramp positions are ignored.
   Then `w_j = b_j / min(b)`.
3. One scalar `t ∈ [0,1]` sets every joint: `f_j = clamp(t·w_j, 0, 1)`.
   - compressing (`D < d_rest`): `a'_j = a_j(1 − f_j)`
   - extending: `a'_j = a_j + f_j(180 − a_j)`
   - Each joint rotates by its angle change about `n_j = (p_j − p_{j−1}) × (p_{j+1} − p_j)`. When the
     two bones are parallel it uses the x axis and an angle of 0.
4. Bisection on `t`: start at 0.5, up to 100 steps, and stop when the reach error is under 1e-5. If it
   does not converge, the best `t` is kept.
5. Root (`180006ef0`, which calls `180008030` and `180008ae0`):
   - Aim the effector at the goal.
   - Roll about the reach axis by the angle that takes `springRestPoleVector` to `poleVector` (π if they
     are antiparallel).
   - Apply `axisAngle(twist)`.
   The rest pose of a fresh handle is the joints at creation, and the default rest pole is
   `2·knee − root − end`, which is `(0,0,2)` for the test chain.

**MEASURED** (`spring_test.py`, `spring_pole.py`, `spring_fit.py`):
- **Bends:** the model above, in pure Python (`spring_fit.py`), reproduces 27 interior-angle rows to
  max 2.0e-6°. The rows are a 4-bone chain (rest interior 172°/145°/160°) at 6 reaches, using the
  default ramp (0.5, 0.5, 0.5), the entries (1, 0.625, 0.25) and the entries (0.2, 1, 0.2), plus the
  earlier 3-bone table. With the default ramp all weights are equal, so every joint keeps the same
  fraction of its rest bend. Saturation also matches: at reach 2.5 with bias 1.0, joint 1 is folded
  shut (`a' = 0`).
- **Pole and twist:** 48 3D cases (4 goals × 4 poles × 3 twists). Every inner joint lies in the plane
  through `Ĥ` and `rot(normalize(P⊥), Ĥ, +twist)`, on the pole side. Off-plane error is max 7.7e-15
  and effector error is max 9.3e-6. So the plane rule is the same as RP.
- Differences from RP:
  - RP moves along the gradient of the rest pose. Spring scales each joint's bend by its bias weight,
    so with equal bias every joint keeps the same fraction of its rest bend.
  - There are no joint limits in the spring kernel, and no stretch.

## 2. HumanIK retarget and full-body IK (mayaHIK.mll; the scripts are OBSERVED)

### 2.1 Solver properties (OBSERVED, `AEHIKProperty2StateTemplate.mel`)
- **Floor and contact:**
  - `FloorContact`, `FootContactType`, `FootFingerContact`, `ParamFootContactStiffness`,
    `FootFingerContactRollStiffness`
  - the floor-contact marker geometry `footBack`, `footFront`, `footInSide`, `footOutSide`, `footMiddle`,
    `footHeight`, from `AEhikFloorContactMarkerTemplate.mel`
- **Height compensation:**
  - `AnkleHeightCompensation`, `AnkleHeightCompensationMode`, `AnkleProximityCompensationMode`
  - `HipsHeightCompensation`, `HipsHeightCompensationMode`
  - `MassCenterCompensationMode`, `ScaleCompensationMode`
  - `HipsTranslationMode`, `HipsTOffsetX/Y/Z`, `ChestTOffsetX/Y/Z`
- **Reach and pull:**
  - `SnSReachLeftAnkle` and the other `SnSReach*` properties (squash and stretch)
  - `SnSSmoothReach`
  - `CtrlPullLeftFoot`, `CtrlPullHead` and the other `CtrlPull*` properties, plus
    `CtrlChestPullLeftHand`
- **Stiffness:**
  - `ParamCtrlSpineStiffness`, `ParamCtrlNeckStiffness`
  - `CollarStiffnessX/Y/Z`, `ExtraCollarRatio`
- **Roll extraction:**
  - `LeftUpLegRoll`, `LeftLegRoll`, `LeftArmRoll`, `LeftForeArmRoll`, each with `...Mode` and `...Ex`
  - `LeftLegFullRollExtraction`, `RollExtractionMode`
  - `ParamLeafLeftLegRoll` and the other `ParamLeaf*Roll` properties
  - So HIK moves a set fraction of a limb's twist about the bone axis onto the roll bones.

### 2.2 Rest pose (OBSERVED)
- `assumeRetargetRestPose.mel` restores the `dagPose` that has `currentRestPose`, through
  `dagPose -restore -global`.
- `gotoHikStancePose.mel` puts the character in the T-stance that the HIK definition was built in.
- INFERRED: the retarget offsets are computed between the two skeletons' stance poses, the same idea as
  the AdvancedSkeleton zero-out.

### 2.3 Bake (OBSERVED, `hikBakeOperation.mel`)
- The bake goes through a temporary animation layer, `gHikBakeToControlRigLayer`. Deleting that layer
  removes the skeleton-to-rig network.
- The comment says "baking always produce euler independant curve". Afterwards the script calls
  `rotationInterpolation -c` on the FK and IK rig nodes to restore their default mode. The options are
  `none/euler/quaternion/quaternionSlerp/quaternionSquad`, from the optionVars
  `rotationInterpolationDefault` and `hikRotationInterpolationDefault`.

### 2.4 Solver internals
*Superseded 2026-10-15 by `HUMANIK_SOLVER.md`. Three kernels in mayaHIK.mll were matched to the
symbolled HIK2016 library by their constants and struct offsets: AdjustLowEffector `0x1800486f0`,
GetPullForce `0x180036db0` and BodyPull `0x18003a400`. The algorithm is the same; the Maya build is
vectorised.*
- **OBSERVED order:**
  1. Begin.
  2. Floor-contact approximation.
  3. Body pull: PullIterationCount + 2 passes, where the hips and chest move.
  4. Limbs: fractional-aim and distance-lerp 2-bone reach, then SnS.
  5. Fingers and toes, then head.
  6. Floor contact.
  7. Hips translation, roll extraction, modifiers.
- **Floor contact (corrected; the old reading "clamp the sole, stiffness resists the clamp" was
  wrong).** Stiffness 0 rotates the foot by a rigid refit of the heel and ball markers onto the floor.
  Stiffness 1 lifts the foot by the deepest marker without rotating it. Values in between interpolate.
  See HUMANIK_SOLVER.md §5.

## 3. Animation layers (OBSERVED script, INFERRED math)
- `performAnimLayerMerge.mel` reads `override`, `rotationAccumulationMode` and `scaleAccumulationMode`
  for each layer, then merges with `animLayer -extractAnimation`. A layer cannot merge into a destination
  whose override or rotation mode differs from its own without baking.
- **INFERRED** from Maya documentation, which matches these attribute names. Per channel, with layer
  weight `w`:
  - additive translate: `v = base + w * layer`
  - override: `v = lerp(base, layer, w)`
  - rotation accumulation "component": add the Euler channels, `r = base + w * layer`
  - rotation accumulation "quaternion": `q = q_base * slerp(I, q_layer, w)`
  - scale "multiply": `s = s_base * s_layer^w`
  - scale "additive": `s = s_base + w * (s_layer - 1)`

## 4. Euler filter and filterCurve (AnimSlice.dll, OBSERVED)

### 4.1 Dispatch
`TfilterCurveCmd::processParamCurvesCommand @ 18015e9e0` switches on the filter type:
- `euler`: `TfilterEuler::eulerFilter`
- `simplify`: tolerance, time tolerance, min step and max step
- `resample`: period plus a kernel. The kernels are box, triangle, gaussian, gaussian2 and gaussian4, with
  indices 0 to 5. Only `gaussian2` and `gaussian4` were observed by string, as indices 4 and 5.
- `butterworth`: `TfilterButterworth::apply`, with `cutoffFrequency` and `samplingRate`, plus a
  `keepKeysOnFrame` flag at `this+0x318`

### 4.2 `TfilterEuler::apply @ 18008c200`
1. It takes the three rotation channels (X, Y, Z) and the rotate order.
2. For each key time in `[start, end]` it evaluates all three curves. When the key times differ, it
   evaluates the missing channel.
3. It fixes the triple against the previous filtered triple. Option `this+4` picks the method:
   - `1`: `TrotateXYZ::bound`, which wraps each angle into ±180
   - `2`: `TrotateXYZ::closestCut(prev)`, a per-channel ±360·k shift toward the previous value
   - `3`: `TrotateXYZ::closestSolution(prev)`, which also tries the equivalent Euler triple and keeps the
     one nearest to the previous triple
4. An optional `initialRotation` (at `this+0x28..0x38`) seeds `prev` when `initialRotationEnabled` is set.
5. It writes the result back with `Tkeyframe::setValue`.

For an XYZ rotate order, INFERRED: the equivalent triple is `(x+180, 180-y, z+180)`.

### 4.3 Butterworth (OBSERVED, MEASURED)
Code:
- `apply(TclipChannel*, TclipChannel*) @ 18008e260`
- `ComputeFilterCoefficients @ 18008da30`, `ButterworthFilter @ 18008d7f0`, `InitFilterMemory @ 18008dd10`
- `Resample @ 18008df20`, ctor `@ 18008d140`
- the command setter `TfilterCurveCmd::updateButterworth @ 180161220`

Fields:

| Field | Offset | Default |
|---|---|---|
| `cutoffFrequency` (Hz) | `+0x18` | 7.0 |
| `samplingRate` (output keys/s) | `+0x20` | `1/TanimTime(1, unit 0x34)` = scene fps |
| `keepKeysOnFrame` | `+0x28` | 1 |
| `filterOrder` | `+0x30` | 4.0 |
| `internalRate` | `+0x38` | 240.0 |
| initial filter-memory value | `+0x40` | 0 |

`kPreheatFactor` is 3. `filterCurve` has no order or internal-rate flag (`-cof`, `-sr`, `-kof` only), so
from the command it is always 4th order at 240 Hz. Times are in ticks, at 141 120 000 per second.

Algorithm:
1. It does nothing if `samplingRate ≤ 0`. `[s, e]` is clamped to the key range.
2. With `kof`, `s` is snapped up to a multiple of `dt_out = 1/sr`. Then `s += dt_out` and `e -= dt_out`,
   so the first and last keys are left alone.
3. With `dt_in = 1/240`, `N = (e − s)/dt_in + 1`. The buffer holds `7N` samples:
   - **3N of linear extrapolation** on each side. The slope is a one-sample difference: forward at the
     start if `s − dt_in` is before the first key (backward otherwise), and backward at the end if
     `e + dt_in` is past the last key (forward otherwise).
   - Between them, `N` samples of `curve.evaluate(s + k·dt_in)`.
4. Compute the coefficients, set the memory to the steady state for input 0 (in effect zeros), and filter
   forward over all `7N` samples. Then recompute, reset and filter **backward**. This is zero-phase
   filtfilt with effective order 8.
5. The temporary curve gets keys at the original `s0`, `e0` (original values) plus the `N` filtered
   samples. The key type is 0x12.
6. `Resample` writes keys at `s0`, at every `dt_out` from `s0` (frame-snapped with `kof`) up to `e0`, and
   at `e0`.

Coefficients:
```
wc = π_t·Fc·dt_in        π_t = 3.1415926535 (truncated constant at 180240348); wrap if |wc| > 2π
t  = tan(wc), t2 = t²
k = 1..order/2:  c = t·cos(π_t(k − (order even ? 0.5 : 0))/order),  den = 1 + 2c + t2
                 a1 = 2(1 − t2)/den,  a2 = −(1 − 2c + t2)/den,  g = t2/den
odd order:       a1 = (1 − t)/(1 + t), g = t/(1 + t), first-order section
section:         w = x + a1·w1 + a2·w2;  y = g·(w + 2w1 + w2);  w2 = w1; w1 = w
```
`ComputeFilterCoefficients` also adds up `Σ c·dt/t2` into `+0x320`. `apply` never reads that value.

**MEASURED** (`bw_test.py`):
- Test signals: impulse, step, ramp plus noise, an 8-key curve, and sines at 1, 2, 4, 6, 7, 8 and 10 Hz.
- Settings: `(cof, sr, kof)` = (7, 24, on), (3, 24, off), (7, 12, on) and (2, 30, off). That makes 44 runs
  on linear-tangent keys at 24 fps.
- A pure-Python port of the above (`bw_model.py`) matches every output key to a **max abs error of 1.8e-13**.
  It found no unmatched key, except one duplicate end key on the 8-key curve at sr 12 and 30.
- The response is `|H(f)|² = 1/(1 + (tan(πf/240)/tan(πFc/240))⁸)`, which is −6.02 dB at `Fc`. At Fc 7:
  4 Hz −0.10 dB, 6 Hz −2.21 dB and 10 Hz −25.5 dB.

**Compared with MotionBuilder** (MOTIONBUILDER.md §1.1, `kfcurve FUN_180038ce0`): it is the **same
algorithm**. The coefficient formula, section form, forward-backward pass, `±dt_out` end protection,
`KeysOnFrame` snap, internal resample and output `Resample` are all the same, and so are the method names.
There are three differences:
1. The padding is 3N on each side in Maya and 250 samples in MoBu.
2. Maya starts the filter memory at zero, and MoBu primes it to `x0` steady state. Maya relies on the
   long linear pad for the start-up transient. For any range of 1 frame or more, the pad is at least 30 samples.
3. Maya's internal rate is a fixed 240 Hz. MoBu's comes from `KFCURVE_GetButterworthFilterInternalRate()`.

With equal internal rates and pads long enough for the transients to die out, the outputs agree to
within the start-up transient.

### 4.4 Quaternion conversion on bake
`TbakeResultsCmd::convertQuatToEuler` and `TparamCurve::convertCopyResultToIndependentEuler` exist, and so
does the `-forceIndependentEulerAngles` flag. So baked rotations are always written as independent Euler
curves, which is why the HIK bake (section 2.3) restores the interpolation mode afterwards.

## 5. FBX axis, units and animation export (OBSERVED, preset XML and FBX MEL)
- `UpAxis` is an enum, Y=0 and Z=1. The default is Y.
- `UnitsSelector` is "Scene units converted to", with Centimeters selected. `DynamicScaleConversion=1`
  means the unit conversion is automatic.
- `Quaternion` is "Quaternion Interpolation Mode", default 2 = "Resample As Euler Interpolation".
- `BakeComplexAnimation` has start, end and step (step 1). There is also `ResampleAnimationCurves`.
- `CurveFilterApplyCstKeyRed` is the constant key reducer. Its precision defaults are T 0.0001,
  R 0.009 and S 0.004, with "auto tangents only".
- `Constraints=0` and `Deformation=1`.
- MEL commands: `FBXExportUpAxis`, `FBXExportBakeComplexAnimation/Start/End/Step`,
  `FBXExportBakeResampleAnimation`, `FBXExportQuaternion`, `FBXExportApplyConstantKeyReducer`,
  `FBXExportSkeletonDefinitions`, `FBXExportAnimationOnly`.
- INFERRED: Unreal imports cm-based Y-up or Z-up FBX and converts it on import. The safe Maya setup is to
  stay in cm, use Y-up, bake complex animation at step 1, resample quaternions as Euler, and turn the
  constant key reducer off. That way contact frames keep their exact keys. AdvancedSkeleton instead puts
  the Z-up change into the root joint (ADVANCEDSKELETON.md section 4.2).

---

## Port to Fulcrum (ranked)

1. **Euler continuity before FBX write.** In `bakedClip`, for each bone and each frame `f`, convert the
   quaternion to Euler in the FBX rotate order (XYZ), with `e = toEuler(q_f)`.
   - Use `e2 = (e.x+π, π-e.y, e.z+π)` as the second candidate.
   - For each candidate, shift every channel by `2π·round((prev_c - c)/2π)`.
   - Keep the candidate with the smallest `Σ|c - prev_c|`, then set `prev = chosen`.

   This is Maya's `closestSolution` (section 4.2). Also enforce `dot(q_f, q_{f-1}) ≥ 0` on the quaternion
   tracks. Without both, Unreal's Euler resample can spin a joint 360 degrees across the slide-to-kneel
   transition.
2. **RP pole plane with an explicit twist** (section 1.3, MEASURED to 5e-6). In `twoBoneIK`:
   - Set `Ĥ = normalize(goal − root)` and `d = min(|goal − root|, L1 + L2)`. Maya does not stretch.
   - Set `a = (L1² − L2² + d²)/(2d)` and `h = sqrt(max(0, L1² − a²))`.
   - Set `b = normalize(P − (P·Ĥ)Ĥ)`, where `P` is a world pole target, not the current knee.
   - The knee is `root + aĤ + h·rot(b, Ĥ, twist)`, with twist right-handed about `Ĥ`. The effector is
     `root + dĤ`.
   - Build the bone rotations from those two points with the shortest arc on each bone.

   Maya's own guard is only `|Ĥ × P| > 0`, and it then falls back to the arc-rotated rest plane. Fulcrum
   should be stricter: when `|P×Ĥ| < 1e-3·|P|`, use the previous frame's `b`. That removes the knee flip
   at full extension in the sprint and the rise. Seed the default pole as `2·knee − root − effector` from
   the rest pose.
   - For chains of 3 or more bones (spine, tail), Maya's shape law is a line in joint space from the rest
     pose. Compressing uses `θ = θ_rest − λ·∇r²/|∇r²|` and extending uses `θ = (1−s)θ_rest`. Solve the
     scalar for `|e| = d` by bisection, then apply the same root arc and pole roll.
   - The spring alternative (section 1.4, exact to 2e-6°) is simpler and usually better for legs with
     3 or more bones and for tails:
     1. Take the rest interior angles `a_j` and the weights `w_j = (1+bias_j)/min`.
     2. Bisect `t` so that the FK reach equals `d`, using `f_j = clamp(t·w_j, 0, 1)`. Compressing uses
        `a' = a(1−f)`; extending uses `a' = a + f(180−a)`.
     3. Aim, roll to the pole and add twist, exactly as in RP.
   - For the foot-roll segments (ikSC), set `τ = twist_Ĥ(q_goal · q_effector⁻¹)` and roll the chain by `τ`
     about `Ĥ`.
3. **Floor clamp in the HIK style for plant and kneel.** For each foot marker point `m` (heel, ball, toe,
   inner, outer), compute `pen = floorY - m.y`. If `max(pen) > 0`, raise the ankle IK goal by `max(pen)`.
   That lift is HIK's *stiffness = 1* branch. HIK's default, stiffness 0, rotates the foot by a rigid
   heel/ball refit instead. Blend the two as `lerp(lift, refit, 1 − k)`; the exact algorithm is in
   HUMANIK_SOLVER.md §9 (*corrected 2026-10-15*). During the kneel, add a knee marker the same way.
4. **Hips height compensation.** When a leg goal is out of reach (`d > lab + lbc`), lower the pelvis by
   `Δ = d - (lab+lbc)*0.999` along the up axis, blended by `HipsHeightCompensation` (0 to 1). Clamping the
   reach alone makes the foot float during the deep plant.
5. **Layered fixes, not destructive edits.** Keep each fix as a layer:
   - rotation: `q = q_base * slerp(I, q_layer, w)`
   - translation: `t = t_base + w*t_layer`

   This is Maya's quaternion rotation accumulation mode. `layerDelta` already exists; give each layer its
   own weight curve and merge only on export.
6. **Gaussian resample for the noisy pelvis.** Apply a gaussian kernel resample with σ ≈ 1 to 2 frames to
   the root and pelvis only, outside the locked contact ranges, and never to frames inside a lock.
   - **Butterworth alternative** (section 4.3, MEASURED to 1.8e-13), applied per scalar channel between
     two lock ranges `[s0, e0]`:
     1. Keep the `s0` and `e0` keys.
     2. Sample `[s0 + dt, e0 − dt]` at 240 Hz (`N` samples).
     3. Pad `3N` samples of linear extrapolation on each side.
     4. Run the 4th-order biquad cascade forward, reset it to zero, and run it backward.
     5. Resample at the clip fps.

     Use `Fc ≈ 4 to 7 Hz`, which is −6 dB at `Fc` with no lag. This is the same algorithm as MoBu's
     Butterworth, so one implementation covers both. `bw_model.py` is a reference that reproduces Maya
     bit for bit.
7. **FBX export settings to match.** Use cm units (`upm` → ×100), Y-up with a −90 X root rest for Unreal,
   and a per-frame bake. Do not reduce keys: Maya's default rotation precision of 0.009 would drop small
   contact corrections.
