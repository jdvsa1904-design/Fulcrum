# HumanIK full-body solver: recovered math

This document covers how Reach, Pull, Resist, Stiffness, Floor Contact, the retarget compensations and
Enforce Gravity change a HumanIK solve. It closes the "solver core UNKNOWN" gaps in `MOTIONBUILDER.md` §2 and
`MAYA.md` §2.4. Nothing in either install was modified. The mayapy black-box measurements belong to the
companion document `HUMANIK_MEASURED.md`, which another agent writes. Everything marked MEASURED here
was produced differently: the real solver library was loaded in-process and driven directly (see §0.3).

Labels:
- **OBSERVED**: read from decompiled code with symbols. Addresses are given.
- **MEASURED**: a black-box fit against the real solver library. Residuals are given.
- **INFERRED**: reasoned from observed code or names, and not checked.
- **UNKNOWN**: the evidence does not settle it.

---

## 0. Sources and the key finding

### 0.1 The solver ships with symbols, as a static library
`C:\Program Files\Autodesk\MotionBuilder 2027\OpenRealitySDK\lib\x64\HIK2016\humanik.lib` is the HumanIK
2016 SDK. The headers ship beside it (`include\HIK2016\humanik\humanik.h`, `hikproperty.h`, `hikutils.h`),
along with the sample host `Samples\constraints\CharacterSolver\HIK2016Solver`.

The `.lib` holds 26 COFF x64 objects with full C++ symbol names. Among them are
`kcharacterpartsolver.obj`, `kcharactermodularsolver.obj`, `kcharacterbasemembersolver.obj`,
`kcharactersolver.obj` and `humanik.obj`. Working copies are in `C:\re_work\hik\lib` and
`C:\re_work\hik\sdk`.

- **The objects were decompiled with symbols.** I used Ghidra 12.1.2 headless with the new script
  `C:\Projects\Fulcrum\tools\ghidra\ObjDecomp.java`. The output is in `C:\re_work\hik\dec\*.obj.c`.
  Addresses written `lib:<obj>+0xNNNN` are offsets inside that object's Ghidra image.
- **MoBu is byte-identical to this library.** `C:\re_work\hik\fpmatch.py` matches each library function's
  bytes, with relocation sites masked, against `hikhost2016.dll`. 1253 of 1342 functions match exactly.
  `hikhost2016.dll` therefore statically links exactly this library. The table is in
  `C:\re_work\hik\results\fingerprints.csv`. Below, `mobu:0x18…` means a VA in `hikhost2016.dll`.
- **Maya is a different build of the same algorithm.** `mayaHIK.mll` matches 0 of 1342 functions by bytes
  and does not import an HIK DLL. It is a newer compile with SSE-vectorised code and polynomial
  asin/acos. The kernels were matched by float-constant sets and struct offsets instead, using
  `C:\re_work\hik\constmatch.py` and `C:\Projects\Fulcrum\tools\ghidra\DecompAt.java`, with output in
  `C:\re_work\hik\dec\maya_pick.c`:

  | Library function | MoBu VA | Maya VA | Evidence the algorithm is the same |
  |---|---|---|---|
  | `KPartSetSkeleton::AdjustLowEffector` | `mobu:0x180094a6c` | `maya:0x1800486f0` | Same offsets (`+0x60`, `+0x5a0`, `+0xdc`, `+0x100`) and constants (15, 30, 57.2958, 180), with a polynomial asin |
  | `GetPullForce` (vector) | `mobu:0x18008b158` | `maya:0x180036db0` | Same expressions, SIMD over 4 limbs, 1.5 at `maya:0x18013ac40` |
  | `GetPullForce` (scalar) | `mobu:0x18008bdb4` | `maya:0x180036bd0` | 1.5 and 1/3 |
  | `EffectorSetToCharacterBodyPull` | `mobu:0x180084dcc` | `maya:0x18003a400` | The iteration-gain constants 0.2, 0.3, 0.4, 0.6 and 0.7 |

### 0.2 Property storage (OBSERVED)
`HIKPropertySetState` is the solver's `KFilterSetBase`, 0x8f0 bytes. Property *i* is stored as
`{int mode @ i*8, float value @ i*8+4}`. `KFilterSetBase::IsParamActive(i)` (`lib:kcharactersolver+0xb69c`)
reads the mode and the per-index definition row. The definition rows are `KCHARACTERDEF_11::GetFilterSetDefinition`:
0x30 bytes each, `{parentIdx, modeType, …}`. This is the same row format as the 283-row kcharacter.dll
table at `1802f45a0`. The first eight rows match MoBu's `kct.txt` order exactly; after that the two lists
diverge. The value offsets for every SDK property are in `C:\re_work\hik\results\properties.csv`, produced
by `x_props.py`.

Values used below (offset → property):

| Offset | Property |
|---|---|
| 0x04 | PullIterationCount |
| 0x0c | Posture |
| 0x1c | ScaleCompensation |
| 0x24 | HipsHeightCompensation |
| 0x2c | AnkleHeightCompensation |
| 0x34 | AnkleProximityCompensation |
| 0x3c | MassCenterCompensation |
| 0x224 | FloorContact |
| 0x22c / 234 / 23c / 244 / 24c / 254 | FootBottomToAnkle / Back / Middle / FrontToMiddle / In / Out |
| 0x25c | foot ContactSize (hidden, 100) |
| 0x26c | FloorPivot |
| 0x274 | FootContactStiffness |
| 0x614 | FootContactType |
| 0x61c | FootFingerContact |
| 0x62c | FootFingerContactRollStiffness |
| 0x634 | ContactBehaviour (hidden; SDK default 0, MoBu sets 2) |
| 0x40c | CtrlEnforceGravity |
| 0x43c / 0x444 / 0x44c | ResistLeftKnee / MaxExtension / CompressionFactor (left knee) |
| 0x49c | ParamCtrlSpineStiffness |
| 0x4a4 | ParamCtrlNeckStiffness |

Properties with mode type `OffAutoUser` (the compensations) use **mode 0 = Off, 1 = Auto (computed),
2 = User (value)**.

Effector state (OBSERVED, `KEffectorSetState`):
- Effector *e* transform: T at `e*0x30`, Q at `e*0x30+0x10`.
- Reach state at `0x840 + e*0x10`, laid out as `{reachT, reachR, pull, resist}`.
- Solving-step mask at `0xf44`.

`KEffectorSetState::ResistFromFilterSet` (`lib:kcontrolset+0x4a24`) writes these resists:

| Effector resist | Property |
|---|---|
| Hips | ResistHipsOrientation |
| L/R Knee | CtrlResistLeft/RightKnee |
| L/R Elbow | CtrlResistLeft/RightElbow |
| ChestOrigin | **SpineStiffness** |
| ChestEnd | ResistChestOrientation |
| L/R Shoulder | ResistCollar |
| Head | **NeckStiffness** |

`HIKSetEffectorPullResistFromPropertyState` gives these defaults: pull 1 on ankles, wrists and ChestOrigin;
resist 0.5 on knees, elbows and shoulders.

### 0.3 Black box: the real library driven from Python
`C:\re_work\hik\hikload.py` is a small COFF linker. It maps all 26 objects into RWX memory and applies the
AMD64 relocations. It resolves the 19 externals (`sqrtf`, `acosf`, `memcpy`, `_controlfp_s`, …) to
ucrtbase/ntdll, and stubs `__security_check_cookie`.

`C:\re_work\hik\hik.py` wraps the C API with ctypes. It builds a 23-joint cm, Y-up, +Z-facing humanoid
(hip 95, knee 52, ankle 8, toe base (±10, 2, 13), wrist ±68 at 145) and calls `HIKSolveForEffectorSet` and
`HIKSolveForCharacter`. It uses the customer string that ships in the SDK, `include\HIK2016\autodeskmwkey.h`.

Because this is the exact code inside `hikhost2016.dll`, these measurements are MoBu ground truth. The
experiment scripts are `C:\re_work\hik\x_*.py`, and the CSV results are in `C:\re_work\hik\results\`.

---

## 1. Overall solve order (OBSERVED)

### 1.1 `HIKSolveForEffectorSet` (`lib:humanik+0x1e740`, `mobu:0x18002393c`)
1. Set FP control to `_controlfp_s(0x8001f, 0x308031f)`.
2. `Begin`: InitialRoll if bit 0x10000 is set, then compute local and apply limits.
3. `FloorContactApprox`: only if bit 4 is set **and** `floor(ContactBehaviour + 0.5) == 2` (`lib:modularsolver+0x9c7c`).
4. `BodyPull`: if bit 1 is set, the full-body pull. Otherwise `BodyReach` (rigid).
5. `ArmsAndLegs` (`mobu:0x180023abc`): shoulders, then the SnS limbs, then
   `EffectorSetToCharacterPartReach` for each limb, then shoulder correction.
6. `FingersAndToes`.
7. `HeadAndNeck`.
8. `FloorContact`: bit 2. It runs `EffectorSetToCharacterFloorContact` for all 4 members (`mobu:0x180089c44`).
9. `HipsTranslation`.
10. `RollExtraction`.
11. `Modifiers`.

### 1.2 `HIKSolveForCharacter` (retarget, `mobu:0x180022d80`)
`Retarget` (FK mapping), then `PrepareEffectorSet`, which applies
`KCharacterBase::EffectorSetSpaceCorrection` (`mobu:0x180077e20`) to the compensations. After that comes
the same IK chain as in §1.1.

---

## 2. Reach Translation (per effector, 0..1)

### 2.1 Limb solve: `KPartSetSkeleton::Solve` (`lib:partsolver+0xc57c`, `mobu:0x18009fcc4`) (OBSERVED)
A limb has four effector slots: top (hip/shoulder), mid (knee/elbow), low (ankle/wrist) and end (toe/finger
base). Up to three sub-solves run on copies of the limb. Their weights are a priority cascade over
reach T and pull P:
```
e      = endEnabled ? T_end : 0
w_mid  = T_mid·P_mid
w_low  = (1−w_mid)·P_low·T_low
w_end  = (1−w_low)(1−w_mid)·P_end·e
rem    = (1−w_mid) − w_low − w_end
w_end += rem·e
w_low += rem'·T_low                     rem' recomputed with the updated w_end
w_mid += rem''·T_mid
```
- Each sub-solve (`SolveForMid`, `SolveForLow`, `SolveForEnd`) returns local rotation deltas.
- `AddWeightedQ` (`mobu:0x180094970`) sums `sign·w·Δq`, starting from zero. `QToLQ` then normalises the sum
  and post-multiplies it onto the limb.
- The weights are **relative only**: a single effector with T = 0.3 gets weight 1 here. The partial reach
  is applied *inside* the sub-solve.

### 2.2 `SolveForLow` (`mobu:0x1800a05b8`) (OBSERVED)
The sequence is:
```
L = LowTopMaxLength
AdjustLowEffector(L)
QuatReach(top, low)
MidReachLow
QuatReach(top, low)
TopLowReachMid
[roll on forearm ×2 for arms]
ReachRotation(low) if R_low > 0
QuatReach(end node 15, end)
```
- **`QuatReach(node i, effector j)`** (`mobu:0x18009dc50`): with `P` the current effector node and `G` its
  goal, compute `v = P − X_i` and `u = v + T_j·(G − P)`. Pre-multiply node *i*'s global rotation by the
  shortest arc `q = normalize(v×u, |u||v| + v·u)`, then apply SysLimits. **Reach T moves the aim point a
  fraction T of the remaining error.** It is not a final-pose blend.
- **`MidReachLow`** (`mobu:0x18009c304`) is the two-bone law of cosines about the knee hinge axis
  `n = q_parent·localAxis` (member constants at `+0xbb0`). Here `a` and `b` are the thigh and shin
  projected perpendicular to `n`:
  ```
  c  = |a + b|
  d  = |G − top|⊥n
  d' = c + T_low·(d − c)                       distance lerped by reach T
  θc = ±acos((|a|² + |b|² − c²)/(2|a||b|))     sign from (a×b)·n
  θd =  acos((|a|² + |b|² − d'²)/(2|a||b|))
  knee ← axisAngle(n, θc − θd)·knee
  ```
  c and d' are clamped to |a| + |b| before the acos.
- **MEASURED** (`x_reach4.py`, `results/reach_t.csv`): simulating QuatReach → MidReachLow → QuatReach in
  Python reproduces HIK's ankle for in-plane targets to **≤ 0.02 cm** across T = 0.1…1. The data was
  three targets × T ∈ {0.1, 0.3, 0.5, 0.8, 1}. Off-plane targets leave 0.2–0.7 cm, which is
  `TopLowReachMid`, the knee swivel; its body was not reconstructed. At T = 0.1 the ankle swings
  *backwards* (z = −12.7 for a forward target at z = +20). That is a real consequence of applying the
  distance-lerp and the fractional aim in sequence, and it is not a bug.

### 2.3 Reach Rotation (MEASURED, `x_reachR3.py`, `results/reach_r.csv`)
`q_ankle = slerp(q_carried, q_goal, R)`, where `q_carried` is the swing-only rotation that the leg aim
gave the ankle. Concretely, take the arc from the rest shin direction to the solved shin, composed with
the thigh arc. The residual is **≤ 0.63°** over 5 axes, 5 R values and T ∈ {0, 1}. It is zero at R = 0.5
and symmetric about it, so HIK's interpolant is slightly non-slerp (UNKNOWN which). An ankle yaw goal also
twists the knee through `KneeReachAnkleRotation` (`mobu:0x18009a034`); that twist formula was not
reconstructed (UNKNOWN).

---

## 3. Resist (knee/elbow), MaxExtension, CompressionFactor

### 3.1 Max-extension damping: `AdjustLowEffector` (`mobu:0x180094a6c`, `maya:0x1800486f0`) (OBSERVED)
Here `r = sqrt(ResistKnee · ResistMaximumExtensionKnee)`, `L = LowTopMaxLength`, `d_t = |G − top|` and
`d_c = |P_low − top|`, where the current pose is the input FK pose.
```
a_t = asin(clamp(d_t/L, 0, 1))·57.2958
a_c = asin(clamp(d_c/L, 0, 1))·57.2958
w   = r ≥ 1e-20 ? clamp((a_t + a_c)/180, 0, 1)^((1−r)·30 + c) : 0     c = 15, or 0 if the part mode is 1
d'  = L·sin((a_t + (a_c − a_t)·w)·π/180)
G   ← top + (G − top)·d'/d_t
```
Near full extension (both angles near 90°) the goal distance is held back toward the current distance, so
the leg does not snap straight.

**MEASURED** (`x_resist.py`, `results/resist_maxext.csv`): 5 resist × 3 max-ext × 8 distances, with the
leg starting straight (`d_c = 87`). The formula predicts the final hip–ankle distance with **max 0.015 cm,
mean 0.0013 cm**. Defaults 0.5/0.5 give `r = 0.5` and exponent 30. That explains the 0.016 cm shortfall at
a reachable target.

### 3.2 Compression factor (OBSERVED in `GetPullForce`, §4.2)
When a limb is shorter than its rest length, its spring gain is multiplied by `CtrlResistCompressionFactor`.
- **MEASURED** (`x_retarget.py`): with HipsHeightCompensation = −10, CompressionFactor 0 / 0.5 / 1 left the
  hips at 90.0 / 95.98 / 98.51. The compressed legs push the hips back up by 0 / 60 / 85 % of the shift.

### 3.3 Joint resist on the pull side
Knee and elbow resist also enter `GetPartPullLength` (`mobu:0x180098b6c`) as
`k = (1 − w_reach)·resist + w_reach`. That is the compression-side gain passed to GetPullForce (OBSERVED).

---

## 4. Pull and the body-pull engine

### 4.1 Iteration structure: `EffectorSetToCharacterBodyPull` (`mobu:0x180084dcc`, 15 KB) (OBSERVED)
- **Iterations:** `N = floor(PullIterationCount + 0.5) + 2`, so the default 10 runs **12 passes**
  (`lib:humanik+0x1ec1c` passes `param_4 + 2`).
- **Pull sharpening:** each limb's pull weight is raised to `max(1, 0.2·N)`, which is 2.4 at the default.
- **Per limb (4 limbs plus the neck):**
  - `GetPartPullLength` gives the rest length `L0` (weighted current root→mid/low/end distances), the max
    length `Lmax` (`LowTopMaxLength`, times the SnS stretch factor `StretchStop` when SnS is on), the pull
    sum and the compression gain.
  - `GetPartPullDir` (`mobu:0x180098718`) gives `dir`, the reach-weighted sum over that limb's effectors of
    `(goal − limb root)`, and its length `L`. It uses the same weight cascade as §2.1.
- **Per iteration:** forces `F_limb = GetPullForce(L, L0, Lmax, P^e, compression) · dir` are
  accumulated onto the hips and chest. The gains ramp with `t = i/N`: `0.5t + 0.5`, `0.7t + 0.3`,
  `0.4t + 0.6`, `0.3t + 0.7`, using constants `3f000000`, `3f333333/3e99999a`, `3ecccccd/3f19999a`.
  The sum is divided by `max(1, Σweights)`. `BodyReach` (`mobu:0x180081fc0`) then moves and rotates hips
  and chest, and `SpineComputeGlobal` follows.
- **Before the loop:** the body effector set is `KGenericEffectorSetState<3>::Blend(FK, IK)` with
  reach weights. A `BodyReachRigid` call runs before and after the loop.

### 4.2 Spring law: `GetPullForce(L, L0, Lmax, p, r, s)` (scalar `mobu:0x18008bdb4`, vector `mobu:0x18008b158`; both also in Maya) (OBSERVED, exact)
```
k  = (L < L0 ? r : 1)·p                    r = compression factor or resist gain
L5 = Lmax − (Lmax − L0)·k                  start of the soft zone
f  = (L − L0)·k                            linear spring
L3 = ((Lmax − L5)·1.5 + L5)·(1−k) + k·Lmax end of the soft zone
if L > L5:
   if L < L3:
      f = cubic Bezier with control points at 1/3 and 2/3 of [L5, L3]
          (0x3eaaaaab), blending the linear branch into the overflow branch
   else:
      f = (L − Lmax)·(1 − k) + (L − L0)·k  any excess beyond Lmax is transmitted 1:1
return f·s
```

### 4.3 MEASURED behaviour (`x_pull1.py`, `x_pull2.py`, `results/pull_sweep.csv`, `pull_iters.csv`)
The setup is a left-wrist goal 52 cm out of reach at (120, 145, 0), with free feet.
- The wrist error decays **geometrically per pass**: `e_k = e_0·ρ(p)^k`.
- With PullIterationCount ≤ 3 (exponent 1): ρ = 0.975 / 0.881 / 0.700 / 0.40 for p = 0.25 / 0.5 / 0.75 / 1.
- At the default 10 (12 passes, exponent 2.4), the residual is 73 % / 21 % / 1.2 % / 0 % for the same p.
- With the ankles pinned (T = R = 1, pull 1) the body moves about half as far, and the wrist misses by
  2.9 cm at p = 1.

The *exact* force accumulation and BodyReach rotation split were not reconstructed line by line.
The bound: the structure and the spring law are OBSERVED, and the convergence rates are MEASURED.

### 4.4 Enforce Gravity (OBSERVED plumbing, MEASURED effect)
Code: `EffectorSetToCharacterBodyPull`, which reads `+0x40c` at `dec\0009_kcharactermodularsolver.obj.c`
line 1908. The lerp is about 450 lines further into the same function.
- **Gate:** it applies when `Posture == 0` (biped) and `EnforceGravity > 0`.
- **Formula:** the hands' pull force on the hips becomes `F' = lerp(F, u·(u·F), g)`. `u` is a normalised
  body up vector, built from one body node minus the midpoint of two others. **Gravity removes the
  horizontal (⊥u) part of the hand pull on the hips.** The vertical part is kept.
- **MEASURED** (`x_grav.py`, `results/gravity.csv`), hand goal (68, 145, 60) with free feet: the hips'
  horizontal shift is 19.9 / 18.2 / 16.1 / 12.3 / 1.8 cm at g = 0 / 0.25 / 0.5 / 0.75 / 1. This is
  monotone but not linear, because it compounds over 12 passes. With g = 1 and a sideways goal the hips
  rise 39 cm instead of sliding. The earlier MOTIONBUILDER.md reading ("keeps hips from being pulled
  upward") was wrong: gravity keeps them from being pulled **sideways**.
- **INFERRED:** that `u` is pelvis-to-chest rather than world Y. A T-pose cannot distinguish the two.

---

## 5. Floor contact

Entry: `EffectorSetToCharacterFloorContact` (`mobu:0x180089c44`), run for each of the 4 members.
1. It builds the limb's hand/foot effector state from the **solved pose**.
2. It transforms that state into floor space by pre-multiplying by the inverse floor matrix
   (`HIKSetEffectorFloorState`, default the origin). In that space the floor is `y = 0`.
3. It calls `MemberFloorContact`, transforms back, and re-solves the limb with `MemberPartReach`. The
   ankle gets T = R = 1; the toe gets T = 1, plus R when the pivot is Toes.

### 5.1 Contact markers (OBSERVED via `HIKGetFloorContactPointPositionfv`)
In the ankle frame, with `b = BottomToAnkle`:

| Marker | Position |
|---|---|
| back-in | `(−In, −b, −Back)` |
| back-out | `(+Out, −b, −Back)` |
| mid-in | `(−In, −b, +Middle)` |
| mid-out | `(+Out, −b, +Middle)` |
| front-in | `(−In, −b, Middle + Front)` |
| front-out | `(+Out, −b, Middle + Front)` |

The lateral signs are mirrored per side. FootContactType 0 (Normal) uses 6 markers; types 1–3 use 4
(`MemberFloorContactStiff` loop count). The hidden foot ContactSize (`0x25c`) has **no reader in the
library** (UNKNOWN). The MoBu host presumably scales the six distances before passing them.

### 5.2 Stiffness split: `MemberFloorContact` (`mobu:0x18008cb68`) (OBSERVED)
With `s = FootContactStiffness`:
- If `s > 0`: run `Stiff`.
- If `s < 1`: run `Smooth`.
- Combine them with `result = KGenericEffectorSetState<2>::Blend(stiff, smooth, 1 − s)`, which lerps T and
  nlerps Q (`mobu:0x180081a20`).

**MEASURED:** the ankle T is linear in s, and the rotation change scales by (1 − s). At pitch 20 the ankle
y runs 9.478 → 14.358 in steps of 1.22, and the rotation change is −13.248·(1 − s) degrees.

### 5.3 Stiff: `MemberFloorContactStiff` (`mobu:0x1800abf88`) (OBSERVED)
Take the minimum y over 6 (or 4) markers. If it is below the floor, translate the whole foot up by the
penetration. Rotation is untouched.
- **MEASURED** (40 random poses): **max 0.0065 cm**, rotation unchanged to 0.027°.

### 5.4 Smooth: `MemberFloorContactSmooth` (`mobu:0x1800ab304`) plus `MemberFloorWrist` (`mobu:0x1800afd70`, 19 KB) (OBSERVED structure)
1. Pre-clamp the ankle: `ankle.y ≥ floor + BottomToAnkle`. This is MEASURED; the reader was not located.
2. Repeat 4 times:
   - Take `P1..P4 = backIn, backOut, mid'In, mid'Out`. `mid' = Middle`, plus `Front` when the contact
     type is 0 and no toe node exists.
   - Triangle A = `(P1, P2, P3)`. If any of its points is below the floor: `F0 = three_point_transform(A)`.
     Clamp each point's y to `max(y, floor)` and compute `F1` the same way. Apply `D = F1·F0⁻¹` (rigid,
     converted to a quaternion plus translation) to the ankle transform.
   - Recompute the points, then do the same with triangle B = `(P2, P3, P4)`.
3. `three_point_transform(a, b, c)` (`mobu:0x1800b938c`) builds a frame:
   ```
   n = norm((b−a)×(c−a))
   s = norm(((b−a)+(c−a))×n)
   t = n×s
   origin = (a+b+c)/3
   ```
4. **Toes**: when the contact type is 0 (Normal) and the toe node exists, `MemberFloorFingerBase` runs 4
   times. It rotates the toe joint about its lateral axis until the front markers sit on the floor.
5. **FloorPivot** (`+0x148` member parameter):
   - **1 = Ankle**: after the solve, restore the ankle's pre-contact x, z.
   - **2 = Toes**: shift x, z so that the toe-base node returns to its pre-contact x, z.
   - **0 = Auto** and **3**: no restore.
- **MEASURED** (`floorfit3.py`, 120 random poses, `results/floor_random.csv`):
  - Smooth model: position error **mean 0.005, max 0.034 cm**; rotation **mean 0.004°, max 0.025°**.
  - Pivot modes (`x_floor6.py`): **≤ 0.005 cm**.
  - Toe bend (`x_toe.py`): within **0.06°** of the angle that puts the tip on the floor.

### 5.5 Approx pass (OBSERVED, `MemberFloorWristAprox`, `mobu:0x1800b5d98`)
This runs only with ContactBehaviour == 2.
```
a' = max(ankle.y, f)
t' = max(toe.y, f)
d  = max(a' − ankle.y, t' − toe.y)
ankle.y = lerp(a', ankle.y + d, s)
toe.y   = lerp(t', toe.y + d, s)
```
It is written back to the effector set before the pull engine runs, so the pull cannot drag a foot below
the floor.

### 5.6 Hands, fingers and toe tips
Hands use the same code with the Hand* parameters (OBSERVED; it is the same member loop).

**Finger/toe-tip contact** (OBSERVED; MEASURED with the harness, see below). The functions are
`MemberFloorFingerContact` (`lib:basemembersolver+0x6544`, `mobu:0x1800af10c`), `…OneIteration` (`+0x6a40`),
`…Spread` (`+0x6c2c`) and `…Sticky` (`+0x6ea0`). The pass runs when `FootFingerContact` /
`HandFingerContact` is on, after the foot/hand floor pass and its limb re-solve. It works in floor space,
with floor height `f` (member effector state `+0x204`).

For each characterised finger `i` (up to 6), where the tip effector `e` starts at the solved tip position:
```
rho    = mean(tipNode.scale.xyz)·TipSize_i               e.g. LeftFootIndexTip, default 0.5
h      = f + rho                                         the tip's resting height
B      = position of the finger's last-segment root (the node before the tip)
Lmax   = MemberFingerMaxLength(i)
n      = q_fingerNode · bendAxis_i                       characterised lateral axis; local X in a T-pose rig
repeat 4:
   Sticky(e) = (e.x, max(e.y, h), e.z)                   mode 0
   Spread(e) = if e.y < h and |e − B|_xz > 0:            mode 1
                  u  = norm((e − B)_xz)
                  l² = min(|e − B|², Lmax²)
                  r  = sqrt(max(0, l² − (B.y − h)²))
                  (B.x + u.x·r, h, B.z + u.z·r)          slide outward to keep the finger's length
               else e
   s = mode 0 ? Sticky(e) : mode 1 ? Spread(e) : (Sticky(e) + Spread(e))/2     mode 2 = average
   e = s − n·(n·(s − e))·RollStiffness                   remove the sideways part of the correction
```
The finger chain is then re-solved to `e` with `MemberFingerReach` (`lib:fingersolver+0x4220`): Prepare,
then Rotation, then Translation.

**MEASURED** (`x_fingers3.py`, `results/finger_contact3.csv`):
- The setup is a 3-node left index toe chain with 3 cm segments. It covers 4 curl poses × 2 yaws ×
  2 rolls of the toe (γ = 0, 30°) × 3 modes × RollStiffness {0, 0.5, 1}: 144 cases.
- The predicted tip is the final distal segment aimed at `e`: `B_out + |BC|·norm(e − B_out)`.
- The best fit uses B as the base and local X as the axis (ranked against A/B × X/Y/Z × node choice).
- Results by mode:
  - **Sticky**, including roll stiffness on tilted toes: **exact, error 0.0000 cm over 48 cases**.
  - **Spread**: mean 0.051, max 0.12 cm.
  - **Sticky and Spread**: mean 0.033, max 0.17 cm.
- The Spread and mixed residuals appear only where the target is near the finger's full extension.
  They are INFERRED to come from the finger max-extension damping (`MemberFingerAdjustLowEffector`,
  `lib:fingersolver+0x2dd0`), which was not modelled.
- A 2-node toe (no distal joint) is not moved by this pass at all (`x_fingers2.py`).

The harness `C:\re_work\hik\hik.py` now also knows the foot-finger node ids (97–144) and the effector ids
(32–43). This is backward-compatible; the previous version is `hik_backup_before_fingers.py`.

---

## 6. Stiffness (spine and neck)
- **Plumbing (OBSERVED):** `SpineStiffness` → ChestOrigin effector resist (`0x8dc`) → `BodyReach(…, 0x8dc)`.
  `NeckStiffness` → Head resist.
- **Distribution (OBSERVED, `SpineReachDeltaQ`, `mobu:0x18007e608`):** for a chest delta rotation of
  angle θ about axis n:
  ```
  N = spineCount + 1
  b = 2N·s + 0.5
  ```
  Each spine bone *k* (base = −1) gets its global rotation pre-multiplied by `axisAngle(n, θ·(b+1+k)/(b+N))`,
  and the base gets `θ·b/(b+N)`. The top bone ends at the full θ.
  - With s = 0 the base takes 0.5/(N+0.5) and every other bone takes 1/(N+0.5), which is near-uniform.
  - With s = 1 the base takes about 2/3.
- **INFERRED:** that the `s` arriving here is exactly the SpineStiffness value; the BodyReach pass-through
  was not traced.
- Foot and hand ContactStiffness is covered in §5.2.

---

## 7. Retarget compensations (`EffectorSetSpaceCorrection`, `mobu:0x180077e20`) (OBSERVED)
All of these act on the effector set before the IK pass. Filter indices are offset/8.

| Property | Effect |
|---|---|
| **ForceActorSpace** (idx 2) | The reference transform becomes identity. |
| **ScaleCompensation** (idx 3) | Scale `= value·0.01`. In Auto: `((dstLegL + dstLegR)/2 + dstHipsSpan)/(srcLen0 + srcLen1)`. The effector set is scaled about the reference (PreMultGT with scale). |
| **HipsHeightCompensation** (idx 4) | `h` = value (User). In Auto: dst leg length (+ ankle term) − src hips height. Moves the Hips, LeftHip and RightHip effectors by `h` along `norm((LHip + RHip) − (LAnkle + RAnkle))`. |
| **AnkleHeightCompensation** (idx 5, only if FootContactType == 0) | Moves each ankle effector by `R(q_ankle)·(0, h, 0)`. In Auto: `h = (dst ankle heights)·FootBottomToAnkle·0.5 − src`. |
| **AnkleProximityCompensation** (idx 6) | Moves the ankle effectors along their local X by the value. In Auto: `(|LHip − RHip|_dst − src)·0.5`. |
| **MassCenterCompensation** (`+0x3c`, %) | See below. |

MassCenterCompensation, per leg:
```
v  = hipEff − ankleEff
L  = dst leg length
v' = |v| > L ? v·L/|v| : v
target = ((ankleL + v'L) + (ankleR + v'R))/2
hips  += m·0.01·(target − hips)
```
It pulls the hips toward a pose the legs can actually reach. It is a no-op when the legs can reach.

**MEASURED** (`x_retarget.py`, `results/retarget_comp.csv`):
- HipsHeight User −10 with compression 0 → hips exactly −10.
- With the defaults it is damped by the compression spring (§3.2).
- Positive values are cancelled by the leg pull once the legs are straight.
- AnkleHeight User +4 / +8 → ankle y 11.0 / 14.6 and hips +2.4 / +4.8 (legs re-solved).
- The 1.5 cm ankle drop seen with HipsHeight −10 is UNKNOWN.

---

## 8. Status summary

| Property | Before | Now |
|---|---|---|
| Reach T | INFERRED | **OBSERVED + MEASURED** (≤ 0.02 cm in-plane) |
| Reach R | INFERRED | **MEASURED** (≤ 0.63°); knee-twist UNKNOWN |
| Pull | INFERRED | **OBSERVED** (cascade, spring law, N+2 passes, p^max(1, 0.2N)); convergence **MEASURED** |
| Resist knee / MaxExtension | INFERRED | **OBSERVED + MEASURED** (≤ 0.015 cm) |
| CompressionFactor | INFERRED | **OBSERVED + MEASURED** |
| Enforce Gravity | INFERRED (wrong direction) | **OBSERVED + MEASURED** |
| Floor contact (feet/hands), stiffness blend, pivot, markers | INFERRED | **OBSERVED + MEASURED** (≤ 0.034 cm, 0.025°) |
| Toe (finger-base) floor bend | UNKNOWN | **MEASURED** (≤ 0.06°) |
| Finger/toe tips Sticky/Spread/both, tip size, roll stiffness | UNKNOWN | **OBSERVED + MEASURED** (Sticky exact; Spread ≤ 0.17 cm) |
| ContactSize % | INFERRED | UNKNOWN (no library reader); handed to the mayapy agent |
| Knee swivel (TopLowReachMid) | UNKNOWN | UNKNOWN; handed to the mayapy agent |
| Spine/Neck stiffness | INFERRED | **OBSERVED** distribution; binding INFERRED |
| Hips/Ankle height, AnkleProximity, Scale, MassCenter compensation | INFERRED | **OBSERVED**; Hips/Ankle **MEASURED** |
| Maya ≡ MoBu algorithm | UNKNOWN | **OBSERVED** for AdjustLowEffector, GetPullForce, BodyPull (constant/offset match); MoBu byte-identical to SDK lib |

---

## 9. Port-ready algorithm for Fulcrum (reach + pull + floor contact)

All units are cm. Y is up, and the floor is `y = f` in floor space; transform into floor space first if the
floor is tilted. Quaternions are `(x, y, z, w)`; `arc(u, v)` is the shortest-arc quaternion.

```
params: reachT, reachR per effector; pull per effector; resistKnee=0.5, maxExt=0.5, compression=0.5,
        pullIters=10, gravity=0.6, floor: bottom=7.5 (MoBu), back=4.5, middle=13, front=7, in=5, out=5,
        contactStiffness=0, pivot=Auto

solve(pose, goals):
  approxFloor(goals)                                   §5.5, only if ContactBehaviour==2 (MoBu default)
  bodyPull(pose, goals)                                §4
  for limb in legs, arms: limbReach(limb)              §2
  for foot in feet: floorContact(foot); limbReach(foot.limb, T=R=1)

limbReach(limb):                                       top=hip, mid=knee, low=ankle, end=toe
  L  = |thigh| + |shin|                                LowTopMaxLength (180° max-extension angle)
  G  = goal.low.T
  r  = sqrt(resistKnee*maxExt)
  dt = |G - top|; dc = |ankle_FK - top|
  at = asin(min(dt/L,1)); ac = asin(min(dc/L,1))       degrees
  w  = r>0 ? clamp((at+ac)/180,0,1)^((1-r)*30+15) : 0
  G  = top + (G - top) * L*sin(rad(at + (ac-at)*w)) / dt
  T  = reachT.low
  quatReach(top, ankle, G, T)                          q = arc(v, v + T*(G - P)) on the hip, P = ankle
  n  = q_hip * kneeHingeLocal
  a  = (knee-top)⊥n; b = (ankle-knee)⊥n; c = |a+b|; d = |G-top|⊥n
  d' = c + T*(d - c); clamp c, d' ≤ |a|+|b|
  rotateKnee(n, acos((a²+b²-c²)/2ab) - acos((a²+b²-d'²)/2ab))   bend toward the existing knee side
  quatReach(top, ankle, G, T)
  swivelKnee(goal.knee)                                TopLowReachMid; use the pole plane
  q_carried = arc(rest shin, shin) * arc(rest thigh, thigh)
  q_ankle   = slerp(q_carried, goal.low.Q, reachR.low)
  quatReach(ankle, toe, goal.end.T, reachT.end)
  if several effectors in one limb: run per-effector sub-solves and blend local deltas with
    w_mid=Tm*Pm, w_low=(1-w_mid)*Pl*Tl, w_end=..., remainder cascade (§2.1); sum sign-aligned, normalize

pullForce(L, L0, Lmax, p, rc):
  k = (L < L0 ? rc : 1)*p
  L5 = Lmax - (Lmax-L0)*k
  L3 = ((Lmax-L5)*1.5 + L5)*(1-k) + k*Lmax
  lin = (L-L0)*k
  if L <= L5: return lin
  over = (L-Lmax)*(1-k) + lin
  if L >= L3: return over
  return cubicBezier(lin@L5 → over@L3, controls at 1/3, 2/3)    smooth bridge

bodyPull(pose, goals):
  N = round(pullIters) + 2;  e = max(1, 0.2*N)
  for i in 0..N-1:
     t = i/N
     for limb in 4 limbs (+neck):
        dir, L = reach-weighted sum(goal_j - root) over the limb's effectors (cascade weights), and its length
        L0 = current root→effector distance; Lmax = limb max length
        F_limb = pullForce(L, L0, Lmax, pull_limb^e, compression_limb) * normalize(dir)
     F_hands = F_leftHand + F_rightHand
     if biped and gravity>0: F_hands = lerp(F_hands, up*(up·F_hands), gravity)
     hips  += (0.5t+0.5)*(F_legs + F_hands + F_head) / max(1, Σpull)
     chest += (0.3t+0.7 / 0.6+0.4t gains)*(F_arms + F_head) / max(1, Σpull)
     distribute the chest rotation over the spine:
        b = 2N_s*spineStiffness + 0.5; bone k gets θ*(b+1+k)/(b+N_s)
  (Fulcrum target: e_k ≈ e_0·ρ(p)^k, ρ(0.5) ≈ 0.88 per pass at exponent 1; check against
   results/pull_iters.csv)

floorContact(foot):                                    in floor space, f = 0
  m6 = markers(ankle)                                  §5.1 table, scaled by ContactSize%
  stiff = foot translated by (0, max(0, f - min_y(m6)), 0)
  smooth = foot; smooth.ankle.y = max(smooth.ankle.y, f + bottom)
  repeat 4:
     for tri in [(backIn,backOut,midIn), (backOut,midIn,midOut)]:
        P = markers(smooth)[tri]
        if min y < f:
           F0 = frame3(P); F1 = frame3(P with y=max(y,f))
           smooth = (F1 * F0^-1) * smooth              rigid: rotate and translate the ankle
  if contactType == Normal and toe exists:
     repeat 4: rotate the toe about its lateral axis until min(frontIn.y, frontOut.y) = f
  if pivot == Ankle: smooth.ankle.xz = input.ankle.xz
  if pivot == Toes:  smooth.ankle.xz += input.toe.xz - smooth.toe.xz
  foot.T = lerp(stiff.T, smooth.T, 1 - contactStiffness)
  foot.Q = nlerp(stiff.Q, smooth.Q, 1 - contactStiffness)
  if fingerContact: for each toe chain: tip goal = 4× (Sticky | Spread | average) minus
     the lateral component scaled by rollStiffness (§5.6); aim the distal segment at the goal

frame3(a,b,c):
  n = norm((b-a)×(c-a)); s = norm(((b-a)+(c-a))×n); t = n×s
  return [s t n | (a+b+c)/3]
```

### Fulcrum-specific notes
1. **Sprint → plant → slide → kneel → rise.**
   - Use the Smooth floor contact with stiffness 0 on the plant. The heel lifts naturally and the ball
     stays on the floor.
   - Use stiffness 0.5–1 for the slide, so the foot keeps its sole angle and is only lifted.
   - Pivot Ankle prevents the plant foot from drifting horizontally. This is the HIK answer to foot skate
     under contact.
2. **Kneel.** Add a knee contact the same way, as a single marker below the knee. The library has no knee
   contact; this extends the Stiff pass.
3. **Rise with a hand on the knee.**
   - Pull 1 on the hand, with gravity 0.6–1, so the hips go up rather than sideways.
   - Keep compression at 0.5, so the planted leg pushes the hips back up.
4. **Max-extension damping.** `AdjustLowEffector` is cheap and removes knee snap. Apply it before
   Fulcrum's existing `twoBoneIK`.
