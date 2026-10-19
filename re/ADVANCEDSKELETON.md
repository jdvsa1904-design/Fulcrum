# AdvancedSkeleton: what the scripts show

Source: `C:\Users\Gaban\Downloads\AdvancedSkeleton-win64.msi`, unpacked with `msiexec /a` into
`C:\re_work\maya\as_msi` (nothing was installed). The whole plugin is MEL and Python. The main file is
`Contents\scripts\AdvancedSkeleton.mel` (3.1 MB, 965 global procs). There is no AdvancedSkeleton `.mll`. The
only native binary is `Shapers.mll`, a separate corrective-shape tool that has nothing to do with Fulcrum,
so Ghidra was not used on it. Line numbers below refer to `AdvancedSkeleton.mel`.

Labels: **OBSERVED** means it is in the script. **INFERRED** means it is reasoned from the script.
**UNKNOWN** means the evidence does not settle it.

---

## 1. FK/IK match: `asAlignFKIK` (l.10689) and `asSwitchFKIK` (l.11180)

### 1.1 FK to IK (OBSERVED)
1. Before matching, the script puts the custom attributes back to default: `roll`, `rollStartAngle`,
   `RollHeel`, `RollToes`, `RollToesEnd`, `legAim`. Every child control is set to `t=0` and `r=0`. So the
   foot roll is neutral while the match runs.
2. The IK control is moved to the world position of the FK end joint (the ankle). Its world rotation is
   taken from the helper `AlignIKTo<End>`, which is parented under the FK end joint. That helper stores the
   offset between the IK control's frame and the FK joint's frame at rig-build time. Before the copy, its
   rotate order is set to match the IK control's.
3. Pole vector:
   - `p0 = (|L_end| * S + |L_mid| * E) / (|L_end| + |L_mid|)`, where S is the start joint, E is the end
     joint, `L_mid` is the length of the upper bone and `L_end` is the length of the lower bone. This is a
     pointConstraint with weights `abs(endLen)` on start and `abs(midLen)` on end.
   - Aim from `p0` toward the middle joint M.
   - `pole = p0 + normalize(M - p0) * (height/3.333 + |M - p0|)`. Here `height` is `Main.height`, the
     character height.
4. Mocap-baking variant (`asAnimBakingMoCapCheckBox` on): the script finds the bend axis by rotating the
   middle joint 15 degrees along its joint orient and seeing which local axis the child moves along. It then
   pushes `p0` by `height/10` along that axis, in the middle joint's frame, before the aim. **Purpose:** the
   pole stays stable when the knee is straight or hyperextends past zero bend. A straight leg makes
   `M - p0` degenerate, and the push prevents that flip.
5. The toe IK copies the world rotation of `AlignIKToToes`, and `roll` is set to 0.
6. Stretch:
   - `stretchy=10` and `volume=10`, so the attributes run 0 to 10 and map to weights 0 to 1.
   - `Lenght1 = |S - M| / (restLen1 * sideSign) / measureDiv`, and `Lenght2` is the same for the lower bone.
   - Both are divided by `MainScaleMultiplyDivide` when the character is scaled.
   - So the IK chain reproduces any length change the FK pose has.
7. Reverse foot with `toesAim`: the `Roll<qToes>` control is aimed back at the FK ankle. The aim vector is
   `(-b,0,0)` and the up vector is `(0,b,0)`, taken from the ankle's object rotation. `b` is -1 for `_L`
   and +1 for `_R`.

### 1.2 IK to FK (OBSERVED)
Each FK control copies the world rotation of its `IKX` joint, from start to end. Splines work the same way.
For FK2IK on a spline, the script builds a degree-3 curve through the chain, rebuilds it with
`rebuildCurve -kep 1`, and places IK control *i* at curve percentage `(i-1)/(n-1)`.

### 1.3 Baking (OBSERVED, `asAnimBakeFKIK` l.14162)
The bake goes frame by frame: `currentTime f`, then `asAlignFKIK`, with autoKey forced on. There is no
batch solve. `FKIKBlend` is set first, to 0 or 10.

## 2. Foot roll network (OBSERVED, l.79135 to 79420)
The IK foot control gets these attributes:
- `roll` (default 0)
- `rollStartAngle` (default 30)
- `rollEndAngle` (default 60)
- `rock`
- the controls `RollHeel`, `RollToes`, `RollToesEnd`

The pivots are nested heel, then toesEnd, then toes, then ankle. Each roll segment is driven by an
`ikSCsolver` handle: `IK<joint>Handle` runs from one roll joint to the next and is parented under the next
roller.

The node network computes the following, with `s = rollStartAngle`, `e = rollEndAngle` and `r = roll`:
```
heel.rx    = min(r, 0)                         clamp(input=r, min=r, max=0)
a          = sat(r / s)                        setRange1: oldMin 0, oldMax s, out 0..1
b          = sat((r - s) / (e - s))            setRange2 / ToesEndSetRange
toes.rx    = (1 - b) * a * r                   PMA(1 - b) * a * r
toesEnd.rx = b * r
rockInner.rz = clamp(rock, -200, 0)
rockOuter.rz = clamp(rock, 0, 200)
```
So the ball takes the roll up to `s`, hands it to the toe tip between `s` and `e`, and gives it up fully
past `e`. The heel takes only negative roll. Rock splits into an inner-edge pivot and an outer-edge pivot
on the two sides of zero.

INFERRED: positive `r` lifts the heel. Each `RollRoller` sits on its pivot's `RollOffset`, so each rotation
happens about that pivot's position, which was placed from the fit skeleton's heel, toes and toesEnd joints.

## 3. Mocap matcher (`asMappingUIFunction`, l.20303; bake l.20842)

### 3.1 Connect (OBSERVED)
- Every mapped mocap joint drives its AS control through `pointConstraint -mo` and `orientConstraint -mo`.
  The offsets are kept: they are captured at the frame where both skeletons stand in the matched pose.
- Every constraint's `nodeState` is wired to `MoCapConstraints.disableConstraints`, which is one global
  bypass.
- For the IK limbs, a synthetic pole locator is built:
  - `OffsetA = pointConstraint(hip weight distB, ankle weight distA)`, where `distA` is the length of the
    hip-knee bone and `distB` is the length of the knee-ankle bone. This is the same length-weighted point
    as in section 1.1.
  - OffsetA is oriented by `RootX_M`.
  - `OffsetB.tz = ±(distA + distB)/10`. The sign comes from the side or limb, so the point is pushed
    forward for the knee.
  - OffsetB aims at the knee with aim `(1,0,0)`, up `(0,1,0)`, and world up vector `(0,1,0)`.
  - The `Pole` control is point-constrained, with offsets kept, to a child locator.
- `MoCapZeroOut` sets the mocap rest pose. A per-vendor `.mel` can override it: `moCapMatchers/Plask.mel`
  hard-sets hips `r = (-90, 0, 90)`, upLeg `rz = 180`, foot `rz = -90`, and shoulder `r = (±90, ±90, 0)`.
  `Mixamo.mel` has its own fixes. So vendor bind-pose rotations are zeroed **before** the `-mo` offsets are
  captured.

### 3.2 Bake (OBSERVED)
`bakeResults -simulation true -sampleBy 1 -disableImplicitControl true -preserveOutsideKeys false
-sparseAnimCurveBake false`, then `delete -staticChannels`. The constrained controls are found through
`constraintParentInverseMatrix` connections. The script does not run an Euler filter after the bake.
INFERRED: it relies on Maya's constraint output, which is already continuous because it decomposes per
frame from the previous value.

### 3.3 Video mocap smoothing (OBSERVED, l.16627)
The smoothing pass runs 4 times. In Maya 2022 and later it uses `filterCurve -f gaussian -w 3 -sc 8`
(width 3, scale 8). In earlier versions it uses `-f butterworth -cutoffFrequency 7 -samplingRate 30
-keepKeysOnFrame`. The leg is scaled by `asLegLength/trackerLegLength`, and the root drops by the foot
tracker's Y, which plants the foot on y=0.

## 4. Unreal export conventions

### 4.1 Rename (OBSERVED, `asExportRenameToUnreal` l.22934)
The AS-to-Mannequin map is:

| AS | Mannequin |
|---|---|
| Root_M | pelvis |
| Hip | thigh |
| Knee | calf |
| Ankle | foot |
| Toes | ball |
| Scapula | clavicle |
| Shoulder | upperarm |
| Elbow | lowerarm |
| Wrist | hand |
| Neck_M | neck_01 |
| Head_M | head |

- Spine joints become `spine_01..NN`, counted after Root_M.
- Side suffixes change: `_R` and `_L` become `_r` and `_l`.
- Twist joints:
  - `thigh_twist_NN` and `upperarm_twist_NN` count from the root.
  - `calf_twist_NN` and `lowerarm_twist_NN` count **in reverse**, as `numPart-i+1`, because the Mannequin
    numbers them that way.
- Rigs that use offsetParentMatrix are refused.

### 4.2 Root motion joint (OBSERVED, `asCreateGameEngineRootMotion` l.60262)
- A `root` joint is inserted above the deform hierarchy.
- An `FKroot_M` control drives it, and `followMain` (0 to 10) blends it between `Main` and `MainSystem`.
- On the Unreal rename, `root.jointOrientX = -90` and the constraint offset is `targetOffsetRotateX = -90`.
- `root.s` is connected to the main scale.

INFERRED: the -90 X on `root` is the Y-up to Z-up conversion. It is baked into the root joint, so the
scene can stay Y-up in Maya while the exported skeleton lines up with Unreal's Z-up Mannequin without
relying on FBX axis conversion.

### 4.3 FBX (OBSERVED, `asExportFbxBrowser` l.22309)
- `FBXProperty Export|IncludeGrp|Animation|BakeComplexAnimation -v 1` is set for animation export, and the
  export runs through `FBXExport -f -s`.
- Pre- and post-export hooks come from `fbxExportOptions.exportFbxPre` and `exportFbxPost`.
- UNKNOWN: the up-axis and unit options. AS does not set them in this proc, so they inherit the user's FBX
  preset. Maya's preset defaults are covered in MAYA.md section 5.

### 4.4 HumanIK bridge (OBSERVED name, l.57439 `asBakeHumanIK`)
This proc bakes from an HIK character onto AS controls with the same `bakeResults` flags as section 3.2.

---

## Port to Fulcrum (ranked)

1. **Length-weighted pole point with a push-off for a straight knee** (section 1.1 and section 3.1). For
   the kneel and rise in `twoBoneIK`, compute
   `p0 = (lbc*A + lab*C)/(lab+lbc)` and `n = normalize(B - p0)`. If `|B - p0| < 0.02*height`, use the
   last frame's `n` instead. Then `pole = p0 + n*(height/3.333 + |B - p0|)` and feed `pole - A` as
   `poleHint`. This replaces `Bv - A` projected on `dir`, which flips when the leg straightens at plant and
   at the end of the rise. Keep the previous frame's `n` as a fallback, which works the same way as AS's
   `height/10` push along the bend axis.
2. **Foot roll mapping for the slide-to-kneel toe break.** Add a scalar `roll` per foot with `s = 30` and
   `e = 60` degrees. Apply `heel = min(r,0)`, `ball = (1-b)*a*r` and `tip = b*r` (formulas in section 2)
   about the heel, ball and toe-tip pivots in that order. During the kneel, drive `r` from the
   ankle-height-over-ball angle so the ball stays planted while the heel lifts.
3. **Zero-out vendor bind pose before capturing offsets.** For the retarget, store
   `offset_i = inverse(srcRest_i) * dstRest_i` after forcing the source rest pose. Then
   `dst_i(t) = src_i(t) * offset_i` for orientation, and translation is applied only on the root, scaled by
   `dstLegLen/srcLegLen`.
4. **Unreal names and the twist numbering rule.** The Mannequin bone check in `bakedClip` should accept the
   reverse `calf_twist` and `lowerarm_twist` numbering, so twist bones get mapped instead of dropped.
5. **Root joint carries the axis change.** When exporting, put the Y-up to Z-up rotation `Rx(-90 degrees)`
   into the `root` bone's rest orientation, not into the scene. Root motion is the pelvis ground
   projection with the follow weight blended, `root(t) = lerp(Main, pelvisXZ(t), w)`.
6. **Gaussian smoothing for noisy channels.** The pass is width 3 and scale 8, run 4 times. That is about a
   Gaussian of sigma ≈ 2 frames applied 4 times (INFERRED), which is a total sigma ≈ 4 frames. Use it only
   on the root and pelvis translation outside the locked plant ranges, never on contact frames.
