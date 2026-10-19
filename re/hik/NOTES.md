# HumanIK: knee twist, ContactSize, spine/neck/head pull, retargeter

This continues `re/HUMANIK_SOLVER.md` and uses the same sources, harness and labels.

- **OBSERVED**: read from decompiled code. Addresses are given.
- **MEASURED**: compared against the real `humanik.lib` (HIK 2016, byte-identical to MoBu's `hikhost2016.dll`) loaded by `C:\re_work\hik\hikload.py` and driven through `C:\re_work\hik\hik.py`. Residuals are given.
- **FITTED**: the structure is observed, but the constants were fitted to measured data.
- **INFERRED**: reasoned, not checked.

Addresses written `lib:<obj>+0xNNNN` are offsets in that object's Ghidra image (the same as `C:\re_work\hik\dec\*.obj.c`). This session re-imported `kcharacterpartsolver.obj` and MoBu's `kcharacter.dll` into the Ghidra project (folder `/hik`) through GhidraMCP. Nothing installed was modified; copies live in `C:\re_work\hik\gh`.

New experiment scripts are in `C:\re_work\hik\x2\`. New fixtures are in `C:\re_work\hik\results\`:
`leg_twist_random.json`, `realistic_knee.json`, `contact_size.json`, `head_pull.json`, `body_eff.json`, `prepare_default.json`, `prepare_high.json`, `retarget_rot.json`, `retarget_hinge.json`, `retarget_scaled.json`.

| Finding | Status | Fulcrum module |
|---|---|---|
| Reach R interpolant is nlerp | MEASURED, max 0.042 deg over 240 cases | `re/hik/knee.js` `nlerp` |
| Default knee twist when the foot rotates | MEASURED, exact at T = 1 | `transferAnkleTwist` |
| RealisticKneeSolving (knee follows foot yaw) | OBSERVED + MEASURED, exact at T = 1 | `realisticKneeSwivel` |
| LegRoll share | MEASURED, linear in angle | `transferAnkleTwist(…, legRoll)` |
| Partial reach T < 1 | OBSERVED + MEASURED: ankle 0.0001 cm, knee Q 0.0002 deg over 107 cases | `solveLeg` |
| ContactSize | OBSERVED + MEASURED: no solver effect, display scale only | `re/hik/contact.js` |
| Neck/head pull force | OBSERVED | `re/hik/pull.js` `neckPullForce` |
| Head pull on hips and spine | OBSERVED (decoded, no fitted constants) + MEASURED: hips max 3.0 cm (mean 0.86), neck 1.8, head 1.8 over 96 cases | `headPull`, `bodyPullPassWeights` |
| Hand pull moves hips only with HandPullHips | OBSERVED | `wire/bodypull.js` `pullHips` |
| Hips / ChestEnd / ChestOrigin effector reach | MEASURED table | `bodyEffectorReach` |
| Spine distribution | OBSERVED (from HUMANIK_SOLVER §6), now ported | `spineShares`, `distributeSpine` |
| Characterization name mapping | OBSERVED (MoBu templates) plus Fulcrum alias rules | `re/hik/retarget.js` `characterize` |
| FK retarget rotation map | MEASURED, 0.06 deg (float noise) | `mapRotations` |
| Effector preparation (scale, hips height, ankle proximity, mass center) | MEASURED, exact (0.00004 cm over 3120 effectors) | `prepareEffectors` |
| Full retarget | MEASURED, every joint within 1 cm and 0.9 deg on plausible poses | `retargetPose` |

---

## 1. Knee twist when the foot is asked to rotate

### 1.1 Where it happens (OBSERVED)
`KPartSetSkeleton::Solve` (`lib:partsolver+0xc57c`) runs, before the limb sub-solves:
```
if R_top > 0: ReachRotation(0)
if R_mid > 0: ReachRotation(1)
if R_low > 0 and part[+0x110] > 0: KneeReachAnkleRotation(part[+0x110])
```
`part[+0x110]` is written by `KPartSetSkeleton::PartFromFilterSet` (`lib:partsolver+0xa65c`) from filter index `this+0xc54`. `KCharacterSkeleton` setup (`lib:kcharacterbase`, the block ending at `0xc54 = -(isLeg) & (0xd5 - side)`) sets that index to 0xd5 − side for legs and 0 for arms. Filter 0xd5 → value offset 0x6ac = **RealisticRightKneeSolving**, 0xd4 → 0x6a4 = **RealisticLeftKneeSolving**. Side 0 is right. Arms never run it. Default 0, so this pass is off by default.

### 1.2 KneeReachAnkleRotation (`lib:partsolver+0x7b08`, OBSERVED, decompiled twice: GhidraMCP and the earlier dump)
Node 1 is the limb top (hip), node `this+0xae8` is the low node (ankle). `qA` = ankle global, `qG` = goal (effector state `+0x70`), `R` = reach R (`+0xe4`), `k` = RealisticKneeSolving.
```
s  = dot(qA, qG) >= 0 ? 1 : -1
qI = normalize(qA + (s*qG - qA)*R)               nlerp
d  = qI * conj(qA)
dk = (k*d.x, k*d.y, k*d.z, d.w)                   un-normalised: angle 2*atan(k*tan(θ/2))
hip ← dk * hip; recompute children
r  = arc(A' - hip, A - hip)                       A' = moved ankle, A = ankle before
hip ← r * hip; ApplySysLimits
```
The name says "knee", but the node rotated is the **hip** (node 1). The swing correction brings the ankle back, so the net effect is a **swivel of the whole leg about the hip–ankle line** by the twist of `dk`, so the knee follows where the foot points. This was MEASURED in two forms:
- straight T-pose legs, k ∈ {0, .25, .5, 1} × R × T × 5 goals (`realistic_knee.json`): knee ≤ 0.017 cm, knee Q ≤ 0.037 deg;
- random bent legs (`leg_twist_random.json`, T = 1): knee ≤ 0.0001 cm.
A first model that twisted the knee about the shin instead was 12–28 cm off on bent knees.

### 1.3 Default knee twist (always on, MEASURED)
With RealisticKneeSolving 0, the leg still twists when the foot is asked to rotate. Rule, measured exact (2.5e-5 deg on 240 straight-leg cases; 1e-4 deg on 53 random bent legs at T = 1):

**After the leg aim, the ankle gets `nlerp(carried, qGoal, R)`. Then the knee is turned about the shin axis so that the ankle's twist relative to the knee (swing–twist about the knee-local shin axis) is the same as before the reach.** With LegRoll active (mode ≠ 0, value v), the knee takes only `(1 − v)` of that angle, linear in angle (v = 0.3 → 0.700 of the twist; v = 0.6 → 0.400; v = 1 → 0). `ParamLeafLeftLegRoll1` in mode 1 also forces the share to `1 − LegRoll value`.

- Turning off `HIKSolvingStepRollExtraction` does not change it; turning off `HIKSolvingStepLeftLeg` removes it. So it belongs to the leg solve, but the exact function was not isolated. Candidates are the roll helpers in `lib:partsolver` (`MidRollCorrection +0x9f7c`, `TopRollReachLow +0xe7a0`) and `KCharacterBase::ExtractRoll` (`lib:kcharactersolver+0x9d58`), which reads LegRoll per node as `(1 − v)` on a DoF roll channel. The law is MEASURED; its code location is INFERRED.
- The earlier "Reach R residual ≤ 0.63 deg, non-slerp" (HUMANIK_SOLVER §2.3) is explained: the interpolant is **nlerp**, not slerp (max 0.042 deg).
- The earlier Maya measurement (`C:\re_work\hik\measure\maya_knee_twist.json`, knee twist 0) is consistent with Maya's HIK running with LegRoll active at 1, or with Maya's newer build. Not settled (UNKNOWN).

### 1.4 Partial reach T < 1 (OBSERVED + MEASURED, closed)
Previous model: up to 0.55 cm ankle, 0.11 cm knee, 0.83 deg knee rotation. Now: ankle 0.00012 cm, knee 0.00013 cm, knee Q 0.0002 deg, ankle Q 0.0001 deg (107 cases, `leg_twist_random.json`).

- `KPartSetSkeleton::Solve` (`lib:partsolver+0xc57c`) does not blend by T. It runs `SolveForLow` (`+0xcca8`) on a copy, takes the local deltas (`PreMultInvLQ +0xa8d8`, `LQToQ +0x82b0`), accumulates them with `AddWeightedQ` (`+0x33f0`: sign-aligned `acc += w·dq`, acc starts at zero) and normalises in `QToLQ` (`+0xaad8`). With one active solve the weight cancels, so T acts only inside `SolveForLow`. (OBSERVED)
- `SolveForLow`: `LowTopMaxLength` → `AdjustLowEffector` (`+0x34bc`, max extension) → `QuatReach(1,2)` → `MidReachLow` → `QuatReach(1,2)` → `TopLowReachMid` → optional `ReachRotation(2)`. (OBSERVED)
- `QuatReach` (`+0xab34`): `u = v + T·(G − P)`, arc(v, u) on the top node. Same as the model. (OBSERVED)
- **`MidReachLow` (`+0x96e8`) is where the model was wrong.** With `n` the knee hinge axis (knee global Q applied to the local axis at `this+0xbb0`), `a = knee − top`, `b = ankle − knee`, `ax = dot(a,n) + dot(b,n)`:
  `c = |a⊥ + b⊥|`, `dd = sqrt(|G − top|² − ax²)` (0 if negative), `dp = c + T·(dd − c)`, both clamped to `|a⊥| + |b⊥|`; the knee turns by the half-angle `(±acos(c) − acos(dp))/2` about `n`.
  The goal distance subtracts the **current ankle's** axial offset, not the goal's own (`|perp(G − top, n)|`, the old model). At T = 1 the following `QuatReach` hides the difference, so only T < 1 showed it. (OBSERVED, MEASURED)
- Consequence (MEASURED): `|ankle − top|` lands exactly on `|A − top| + T·(|G − top| − |A − top|)`; the direction is not a simple interpolation.

`TopLowReachMid` (OBSERVED): `a = knee − hip`, `h = ankle − hip`, `kt = a + T_mid·(kneeGoal − knee)`, `n1 = a × h`, `n2 = kt × h`. Fade `f = clamp((dot(â, ĥ) − 0.94)/0.0597, 0, 1)`, then `f = f²(3 − 2f)`. Rotate the hip by `arc(n1, n2)` with its xyz scaled by `(1 − f)`. It swivels the knee toward the knee goal and fades out as the leg straightens.

---

## 2. Foot (and hand) ContactSize

- **The library ignores it (MEASURED).** Filter slot 75 (value `0x25c`, mode `0x258`) and the hand slot 86 (`0x2b4`) were poked directly in the property state. ContactSize 0 / 50 / 100 / 200 in modes 0 / 1 / 2 gives bit-identical ankle, wrist and contact points (`contact_size.json`, `x2\cs2.py`, each value in a fresh process).
- **The HIK 2016 host never sends it (OBSERVED).** `HIK2016Solver\filterset2fbcharacter.h` maps `"ContactSize" ↔ FootContactSize`, but `hikhostmb.cxx:155` resolves the name with `HIKGetPropertyId`, which returns −1 for it, so the row is dropped. `hikhost2016.dll` carries the same table (`0xe9578`).
- **In MoBu itself it is a display scale (OBSERVED, kcharacter.dll).**
  - `KCharacterWithProperties::FootContactSizeSet` (export thunk `0x180001fd7` → `0x180138690`) clamps the value to [0.01, 1000] (constants at `0x1802477b8`, `0x180288138`) and stores it as a float at native filter `+0x19c` (hand `+0x1f4`).
  - The only other readers are `0x18013699c` (filter → property sync), `0x180156be1` (multiplied by a scene-scale factor) and `0x1801b1aaa`, which returns `value / 100` as a uniform 3-vector (the contact-marker gizmo scale).
  - No solver path reads it.
- **Fulcrum:** `contacts.js footMarkers` scales the markers by contactSize, which HumanIK 2016 does not do. `re/hik/contact.js` gives `hikFootMarkers` / `hikFloorContact` (contactSize pinned to 100) and `gizmoFootMarkers` / `contactGizmoScale` for drawing.

---

## 3. Spine, neck and head pull

### 3.1 Neck chain (OBSERVED)
- `KCharacterBase::GetNeckPullDir` (`lib:modularsolver+0xac64`): `dir = HeadGoal (effector 15, +0x2d0) − neck base node`, `L = |dir|`.
- `KCharacterBase::GetNeckPullLength` (`+0xae18`):
  - `p = pull_head · T_head` (effector reach state `+0x938 · +0x930`);
  - `resist = +0x93c` (NeckStiffness via ResistFromFilterSet);
  - `L0 = |head − neck base|`;
  - `Lmax = Σ neck segments + head segment`.
- In `EffectorSetToCharacterBodyPull` (`+0x617c`) the neck force `F_neck = GetPullForce(L, L0, Lmax, p, resist)·dir` joins the four limb forces. Gains read from the loop (`x2\bodypull.c` lines 735–821), with `t = (i + 1)/N` (§3.2):

| Body | Force | Gain |
|---|---|---|
| hips | neck | 0.7t + 0.3 (`3f333333`, `3e99999a`) |
| hips | legs | 0.5t + 0.5 |
| chest | neck | 0.5t + 0.5 |
| chest | arms | 0.7t + 0.3 |
| chest | rotation arcs | 0.6 + 0.4t and 0.4 + 0.6t |
| hips | rotation | 0.3 + 0.7t |

  The sums are divided by `max(1, Σw)` and `max(2 − x, Σw)`.

### 3.2 Body pull pass, decoded (`EffectorSetToCharacterBodyPull`, `lib:modularsolver+0x617c`; `x2\bodypull.c`)
Effector reach states are 16 bytes from `+0x840` (effector i at `+0x840 + 16i`: T, R, pull, resist). Hips `0x840`, ChestOrigin `0x8d0` (resist `0x8dc` = spine stiffness), ChestEnd `0x8e0`, shoulders `0x910/0x920`, Head `0x930`, hips `0x940/0x950`. HandPullHips: `+0xf48`/`+0xf4c`. (OBSERVED)
- **Passes**: N = round(PullIterationCount) + 2, and `t = (i + 1)/N` (the counter starts at 1). (OBSERVED)
- **Force**: `GetPullForce(L, L0, Lmax, r, s, P)` (`+0xbb08`) equals `hik.js pullForce(L, L0, Lmax, k = (L < L0 ? s : 1)·r)·P`. The neck call passes `r = resist^e` (e = max(1, 0.2N)), `s = 1`, `P = pull·T`. The exponent applies to the **resist**; the pull scales the force linearly. The old fit's `p^2.4` was standing in for `p·p` (force × weight). (OBSERVED)
- **Weights** (`bodyPullPassWeights`): `ce = ChestEnd T·pull`, `neckChest = p(1 − ce)`, `armChest_k = a_k(1 − ce)`, `chestW = Π(1 − armChest)(1 − neckChest)(ChestEndT − ce) + ce`, `free = (1 − hipsT·hipsPull)(1 − chestW)`, `neckHips = p·free`, **`armHips_k = a_k·free·HandPullHips_k`**, `legHips = l(1 − hipsT·hipsPull)`. Denominators: `max(1, Σ hips weights)` and `max(Σ chest weights, 2 − chestW)`. Shoulder-effector pull factors are left out. (OBSERVED)
- **Per pass**: `Δhips = [(0.3 + 0.7t)·F_neck·neckHips + (0.5 + 0.5t)·Σ F_leg·legHips + (0.3 + 0.7t)·Σ F_arm·armHips + (0.5 + 0.5t)·(hips goal − hips)·hipsW] / hipsDen`. Chest target = shoulder mass center + Δhips + `[(0.5 + 0.5t)·F_neck·neckChest + (0.3 + 0.7t)·Σ F_arm·armChest + (0.5 + 0.5t)·(chest goal − chest)·chestW] / chestDen`. (OBSERVED)
- **Rotation accumulation** (`bodypull.c` 700–990): the hips Q gets a sign-aligned sum of `identity + chestOrigin-follow·local_540·(0.7 + 0.3t) + leg arcs·(0.4 + 0.6t)` (`local_540 = 0.5` when hips R = 0), normalised and divided by `max(1, Σ)`. The chest Q gets the shoulder arcs (arm forces) times `(0.4 + 0.6t)` plus the ChestEnd R share. For a head-only pull every chest arc weight is zero. (OBSERVED; the hips-follow term is not ported, and is INFERRED to be most of the remaining ~3 cm.)
- **Placement**: `BodyReach` (`+0x3c88`) with body reach T = R = 1. `HipsReach` (`+0xbd10`) puts the hips on the target. `ChestReachPosition` (`+0x5398`) turns the spine about the hips by `arc(shoulders − hips, target − hips)`, spread by `SpineReachDeltaQ` with the ChestOrigin resist as stiffness (0 by default). (OBSERVED)
- **Hand pull**: an arm force reaches the hips only through `armHips`, i.e. only with HandPullHips > 0. Without it, IK Pull bends the spine (chest share) and the hips stay. This matches the live MotionBuilder finding. `wire/bodypull.js pullOffsets` now takes `pullHips` and scales the pelvis offset by it. The default is 1, which keeps the current app behaviour; HumanIK's own default is 0. Linear scaling is exact when no legs are planted (denominator 1) and INFERRED otherwise.

### 3.2b Head pull model (MEASURED against `head_pull.json`: 96 cases, 6 goals × 4 pulls × free/planted × NeckStiffness 0/0.5)
`headPull` runs the decoded pass with no fitted constants: neck force from the Neck node; legs as `pullForce(L, L0 = Lmax = leg length, resist^e)`; spine `[hips, waist, spine1, neck]` turned by `distributeSpine`; shoulder center at 0.8 of spine1→neck.
- Before (fit): hips max 6.3 cm (mean 1.9), neck 3.4, head 1.4 (48 cases, stiffness 0 only).
- Now: **hips max 2.98 cm (mean 0.86), neck 1.84, head 1.77** over all 96.
- The largest residuals are at pull = 1 (hips 2–3 cm too far forward). Likely causes are the unported hips-rotation follow term and the final `BodyReachRigid`. UNKNOWN until `HipsReach`'s offset and that term are ported.

### 3.3 Hips / ChestEnd / ChestOrigin effectors (MEASURED, `body_eff.json`)
- **Free feet:**
  - Hips T moves the body rigidly by `g(T)·offset`, with `g` = 0.239 / 0.5 / 1 at T = .25 / .5 / 1.
  - ChestEnd T moves the hips by about T² and the neck by about 2T² (0.066 / 0.133 at T = .25), which leans the spine; it is rigid at T = 1.
  - With pull 1 both are close to linear.
- **Planted feet:** the gains drop to about T² (pull 0). A long horizontal reach also lowers the hips about 2.2 cm, from the leg-length limit.
- **ChestOrigin translation does nothing** at any T or pull. It only carries SpineStiffness as resist (HUMANIK_SOLVER §0.2).
- Ported as a measured table, `bodyEffectorReach`: free body ≤ 0.03 of the offset; planted, along the offset, ≤ 0.01.

### 3.4 Spine distribution (OBSERVED in HUMANIK_SOLVER §6, now ported)
`spineShares(count, stiffness)` / `distributeSpine` follow `SpineReachDeltaQ` (`mobu:0x18007e608`): `N = count + 1`, `b = 2N·s + 0.5`, bone k (base = −1) gets `θ(b+1+k)/(b+N)`, and the top bone gets the full θ.

---

## 4. Retargeter

### 4.1 Characterization: any skeleton to HIK slots
- **The library does not map names (OBSERVED).** `HIKCharacterizeGeometry` takes per-node T-pose TQS for nodes the host already identified.
- **MoBu's characterization tool (OBSERVED, `bin\system\CharacterizationTool`):**
  - Matching is per template (`HIK.xml`, `Biped.xml`, `CAT - *.xml`), key → name.
  - `match_type Suffix`, with delimiters `_ : .`.
  - Mirror pairs: `Left/Right`, `L /R `, `L_/R_`, `_L/_R`.
  - Biped's Hips uses `use_root`.
- **Fulcrum's `characterize` does the same and adds aliases.**
  - Names are matched on the namespace-stripped name, then on every suffix after `_ . space`, so `Character1_LeftHand` maps.
  - Known conventions: HIK/Mixamo exact names; Unreal (`pelvis`, `thigh_l`, `calf_l`, `foot_l`, `ball_l`, `spine_0N`, `neck_0N`, `clavicle`, `upperarm`, `lowerarm`, `hand`); Biped (`Bip01 L Thigh`); CMU (`LowerBack`); Rigify (`DEF-thigh.L`, `shin`, `upper_arm`, `forearm`, `shoulder`).
  - Spine and neck bones are ordered by depth into Spine, Spine1…9 and Neck, Neck1…9.
  - Legs without usable names are found by topology: the two descending chains from Hips, with the side taken from the left axis `up × forward` (+X when facing +Z, as in HIK).
  - Required slots are HumanIK's minimum (15). Anything missing is reported.
  - Metrics come from the T-pose: leg lengths and hip width.

### 4.2 FK retarget (OBSERVED + MEASURED)
- **Call chain:** `HIKSolveForCharacterRetarget` (`lib:humanik+0x1e66c`) → `KCharacterBase::CharacterToCharacter` (`lib:kcharactersolver+0x40fc`) → `ApplySysLimitsUpKeepGQ`.
- **Rotations:** `q_dst = q_src · inv(q_srcRest) · q_dstRest`, for every mapped node. MEASURED 0.06 deg (float noise) on a destination with random rest rotations up to 120 deg.
- **Translation:** the hips are not placed by this step.

### 4.3 Effector preparation (OBSERVED + MEASURED exact)
`HIKSolveForCharacterPrepareEffectorSet` (`lib:humanik+0x1e4e0`) runs:
`CtrlSetToEffectorSet(src)` → `EffectorSetSpaceCorrection(dst)` (`lib:kcharacterbase+0x80b0`) → `RetargetTrajectory` (only when the filter at `+0x4e4` equals 2) → reach/pull/resist from the destination properties → `InitialRoll`.

All compensations default to **Auto**. Measured against 240 poses × 3 destination proportions (`prepare_default.json`, `prepare_high.json`): every effector within **0.00004 cm**, and every rotation unchanged.
1. **Effectors.** Each one is the source node's position plus its logical rotation. Hips is the midpoint of the two UpLegs, not the Hips node.
2. **Scale.** Auto: `s = (mean dst leg + dst hip width)/(mean src leg + src hip width)`. User: `s = value/100`. Every effector is scaled about the reference.
3. **HipsHeight.** Auto: `h = mean dst leg − s·mean src leg`. Hips, LeftHip and RightHip move by `h·unit((LHip+RHip) − (LAnkle+RAnkle))`.
4. **AnkleProximity.** Auto: `a = (dst hip width − s·src hip width)/2`. Each ankle **and its foot (toe) effector** moves by `±a·(ankle local X)` **with the up component removed (not renormalised)**. HUMANIK_SOLVER §7 said "along local X", which is off by that vertical part.
5. **MassCenter 80 %.** Only the **Hips effector** moves, toward the midpoint of the reachable hip positions. LeftHip and RightHip do not move. The §7 pseudocode moved all three; that is corrected here.
6. **AnkleHeight.** With FootContactType 0 and the default FootBottomToAnkle it had no measurable effect in these sets.

### 4.4 Full solve (MEASURED)
`HIKSolveForCharacter` = Retarget + Prepare + the IK chain. The defaults are ReachActor ankles T = R = 1, everything else 0, and knee resist/max-extension 0.5/0.5.

`retargetPose` does the following:
1. FK map.
2. Place the hips so the UpLeg midpoint sits on the Hips effector.
3. Leg reach with `solveLeg`: nlerp foot rotation, default knee twist law, max-extension damping.

Against the library on plausible poses (knees bent on their hinge, elbows forward):
- **Non-uniform destination** (`retarget_hinge.json`, 30 poses): every joint ≤ 0.98 cm, every rotation ≤ 0.9 deg, ankles ≤ 0.07 cm.
- **Destination with random rest frames** (`retarget_rot.json`, 25 poses): every joint ≤ 0.73 cm, rotations ≤ 0.6 deg.

The remaining ~0.9 cm / 0.9 deg is a small hips translation and rotation from HIK's body pull and rigid pass, which `retargetPose` does not run.

**Not modelled (MEASURED, out of scope):**
- **Implausible joints.** On poses with random twisted knees and backward-bent elbows, HumanIK re-projects the knee onto its hinge and flips hyperextended elbows (changes of 22–87 deg). `retargetPose` copies such FK as is.
- **Knee hinge in the retarget.** The two candidate axes, the characterized node X and logical X, were indistinguishable (0.59 vs 0.73 cm). The characterized node X is used (INFERRED).

---

## 5. Port notes for Fulcrum
1. Use `re/hik/retarget.js` instead of the best-fit world-space retarget when a source take has a different skeleton. `characterize` both rigs once, then call `retargetPose` per frame. Rest rotations can be anything.
2. For foot plants that turn the foot, pass `realistic` 0.3–0.6 to `solveLeg`, so the knee swings with the foot instead of twisting the shin. The default law twists the shin fully, which reads as a candy-wrapped calf on a mesh without twist bones; LegRoll 0.4–0.6 moves part of that twist back to the foot.
3. ContactSize: drawing only. Do not scale solver markers with it.
4. Head pull: the decoded model is within 3 cm (mean 0.9). Hand pull: pass `pullHips: 0` to `pullOffsets` for HumanIK's default (the spine bends, the hips stay).
