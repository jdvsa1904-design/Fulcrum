# Cascadeur physics tools: what the binaries show

Sources: `tools.dll.physics.c` and `domain_scene.dll.physics.c` (Ghidra string-xref dumps), the shipped
Python (`resources/scripts/python`), the API stubs (`resources/scripts/stubs/csc`), and read-only byte
reads and disassembly of `tools.dll`, `domain_scene.dll`, `presenter_lib.dll` and `data_source_lib.dll`
done with a small capstone script. Addresses are VAs in the 0x180000000 image base.

Labels: **OBSERVED** means it is in the code, strings or bytes. **INFERRED** means it is reasoned from
observed code. **UNKNOWN** means the evidence does not settle it.

Limits of the evidence: the dump caps how many functions it decompiles, so the bodies of the
`ballistic_trajectory::Core` lambdas (`applyNewGravity`, `initProc` lambdas), the AutoPhysics solver,
fulcrum motion cleaning (`FUN_18042cb60` and nearby) and the timeline interpolation code are **not** in
the dumps. Wherever a claim depends on those bodies, it is marked UNKNOWN.

---

## 1. Ballistic trajectory

### 1.1 Data model (OBSERVED)
The ballistic DTO serializer `FUN_1800832d0 @ 1800832d0` writes these keys in this order:
`name, procId, velocity (vec3), startFrame (int), cutoffHeight (float), startPosition (vec3),
gravity (float), isLandingPointStable (bool), afterInflectionPoint (bool), centerOfMassObjectId (Guid),
visible (int), isActive (bool), curveId (Guid)`.

The property table in the UI, `FUN_1800a7400 @ 1800a7400`, lists `gravity`, `is landing point stable`,
`after inflection point`, `velocity`, `cutoff height`, `start position`, `center of mass`, `start frame`,
`end frame` and `frames count`. It also lists four draggable points: `velocity`, `landing`, `start` and
`height`.

The getter `FUN_1800a8c60` gives `frames count = end - start + 1`, read as
`(int)end + (1 - (int)start)` (OBSERVED). `end frame` comes from `math::SizesInterval::end`.

### 1.2 How a trajectory is created ("Add ballistic trajectory") (OBSERVED)
`FUN_180072570 @ 180072570` does the following:
1. If no center of mass is selected, it shows `"no created or selected center of mass "`.
2. It takes the selected timeline interval, `framesInterval(...) -> [a, b]`.
3. `startPosition = CoM(a)` comes from the CoM object's position data on frame `a`.
4. `velocity = CoM(a) - CoM(a-1)` is a backward difference. It is 0 when `a == 0`:
   `fVar13 = (float)local_158 - SUB84(local_148,0); ...`
5. `gravity = scene->vfunc[0x108]()`, a scene-level gravity getter. Its value and units are UNKNOWN from
   this dump.
6. Flags: `local_ac = 0x101` sets both `isLandingPointStable = true` and `afterInflectionPoint = true`
   at creation. `cutoffHeight = 0`, `visible = 2`, `isActive = 1`.
7. It calls `FUN_180078940` ("create ballistic").

**INFERRED:** velocity is in units per frame, because it is a one-frame difference. Only one frame of
history is used; there is no least-squares fit over the interval at creation.

### 1.3 The parabola (OBSERVED)
`FUN_1800ad980 @ 1800ad980` handles a frame-count change (error string `"wrong framesCount"`):

```
t = framesCount - 1
landing.x = start.x + v.x * t - (g * 0.5 * t * t) * 0.0
landing.y = start.y + v.y * t - (g * 0.5 * t * t) * 1.0
landing.z = start.z + v.z * t - (g * 0.5 * t * t) * 0.0
```
The constants `DAT_1807217d0 = 0.5f` and `DAT_1807217d8 = 1.0f` were read from `tools.dll`. Gravity
acts only on Y, so **Y is up** (OBSERVED). `t` is counted in frames, so `g` here is in length/frame²
and `v` is in length/frame (INFERRED). The Core holds two sets of values: user-facing `velocity` at
`+0x240` and `gravity` at `+0x268`, and working per-frame values `v` at `+0x1f8..0x200` and `g` at
`+0x204` (INFERRED from the different offsets the getter and the solver use).

The four handle points are `start` (`+0x174`), `landing` (`+0x1ac`, written by the formula above),
`height` (`+0x190`) and `velocity` (`+0x158`). **INFERRED:** `height` is the apex,
`y0 + vy²/(2g)` at `t* = vy/g`. The formula for it was not seen.

### 1.4 "After Inflection Point" and "Not enough frames" (OBSERVED)
In `FUN_1800ad980`:
```
if (afterInflectionPoint && (float)framesCount <= ceilf(v.y / g))
    throw "Not enough frames to reach max height. To set up this value, turn off 'After Inflection Point' parameter"
```
- `v.y / g` is the number of frames until the apex. With *After Inflection Point* on, the interval must
  extend **past the apex**, so the landing is on the descending branch.
- `FUN_18007d070 @ 18007d070` shows the companion message
  `"Not enough frames to create a typical ballistic. To change ballistic to typical, turn on 'After Inflection Point' parameter"`
  only when `afterInflectionPoint` (DTO byte `+0x5d`) is **off**. A "typical" ballistic is therefore
  a rise, an apex, then a fall (INFERRED).
- The meaning of `cutoff height` is UNKNOWN. The name suggests a landing plane height.
- `is landing point stable` is UNKNOWN. INFERRED: when gravity or frame count changes, the solver
  keeps the landing fixed and recomputes the velocity.

### 1.5 Gravity value and units
- **OBSERVED:** the AutoPhysics core settings constructor (the function ending at `1801ec5bb`, which
  writes `[rcx]=200.0` first) contains `mov dword [rcx+0x4c0], 0x44750000` (**980.0f**) followed
  directly by `mov qword [rcx+0x4c8], 0x1e` (**30**).
- **INFERRED:** gravity is 980 length/s² in **centimetres**, and the default solver FPS is 30, so the
  per-frame value is `980/30² = 1.089 cm/frame²`. That per-frame constant does not appear anywhere in
  the binary, so it is computed at runtime (OBSERVED by absence).
- The double `-9.81` sits in `.data @ 180be3a50`, but no reference to it was traced, so its user is
  UNKNOWN.

### 1.6 Snap CoM to trajectory, ghosts, free rotation
- `FUN_180072c70` runs `"Snap center of mass to ballistic trajectory"` as an undoable session command
  (`Tool::actionSnapCM`). The body that moves the CoM is UNKNOWN. INFERRED: it sets
  `CenterOfMass.pos_global_target` on every frame of the interval to the parabola point.
- Ballistic tool settings (`FUN_18008c710`, OBSERVED names): `Ballistic Ghosts on Keys Min/Max Distance`,
  `... on Current Frame Min/Max Distance`, `Max Clamp Physics Transform Rotations`,
  `Valuable Frames Weight`, `Weight Angular Velocity`, `Weight Angle Rotations`,
  `Weight Last Frame Rotation`, `Bounds Expansion`, `Take Into Account All Frames`,
  `Free Fly Fit Iterations Number`, `n Simplex in One Dimension`, and `Snap Last Frame To Landing Point`.
- QML actions (OBSERVED): `Set fixation frame for free rotation`,
  `Snap orientation to the ghosts of free rotation`, `Set physic priority frame`,
  `Snap to ballistic ghosts`.
- **INFERRED:** rotation during flight is fitted by a derivative-free simplex search (Nelder-Mead style:
  "n Simplex", "Bounds Expansion", "Iterations"). Its objective is a weighted sum of angular velocity
  consistency, angle error on valuable frames, and the last frame's rotation. The "ghosts" are
  physically correct poses that the animator can snap to. The objective code is UNKNOWN.

---

## 2. Fulcrum points and fulcrum keys

### 2.1 Enums (OBSERVED)
- The layer key/interval fixation enum (`stubs/csc/layers/layer.pyi`) is `Fixation: Free = 0, Fulcrum = 2`.
  The doc comment there reads "Layer Frame Fixation enumerable". The key/interval record in
  `data_source_lib.dll` has the fields `globalLocalType`, `fulcrumedState` (`FULCRUM`) and
  `interpolationType`.
- The per-point `FulcrumState` enum (`domain_scene.dll @ 1806603b0`) is
  `["Fulcrum", "Enforce", "NotFulcrum"]`, so `NotFulcrum = 2`. Its description reads "Fulcrum ...
  considered a fulcrum point when the conditions are met. Enforce ... always ... NotFulcrum ... never".
- The quick-rig template (`prototypes/rigs/qrt/biped/constants.py`) sets `"Fulcrum State": 2`
  (NotFulcrum) on the pelvis, spine, neck, head, clavicles, upper and lower arms, thighs and calves.
  Only **hands, feet and toes** stay `Fulcrum`. The fulcrum groups are `[HAND_L]`, `[HAND_R]`,
  `[FOOT_L, TOE_L]`, `[FOOT_R, TOE_R]`, `[TOE_L]` and `[TOE_R]`.
- The `FulcrumPoint` behaviour fields are `fulcrum_state`, `collision_radius` and `max_speed`. The
  `FulcrumGroup` fields are `is_active`, `group_name` and `ids` (controllers in the group).

### 2.2 "Change to fulcrum key" (OBSERVED strings, behaviour INFERRED)
`presenter_lib.dll` contains these neighbouring strings: `Change fulcrum`, `Change to fulcrum key`,
`Change to fulcrum key on interval`, `<br>Local`, `<br>Fulcrum` and `fulcrumColor`, next to
`Change IK|FK key [on interval]`. Fulcrum is therefore a **per-key / per-interval property on a
layer**, in the same family as IK/FK and Local/Global (OBSERVED). The interpolation code that acts on
it is not in the dumps (UNKNOWN).

The closest direct evidence of what it means is in `python/ml/editable_animation.py::best_interpolation`
(OBSERVED). It asks `FulcrumCollisionInfo(left, right, scene, StaticPointsTypes)` for the fulcrum points
in contact on each key. If a limb's points are in contact on **both** bounding keys, the interval is
forced to `CLAMPED_BEZIER + IK`:
```
if obj_ids & left_points & right_points:
    modify_section(key_left, layer, CLAMPED_BEZIER, IK)
```
**INFERRED meaning of a fulcrum key/interval:** between two keys, the limb's contact point is held at
its world position. Clamped interpolation stops overshoot and IK keeps the end effector planted while
the body moves, so the body pivots around the contact point. `Fixation.Fulcrum` is the explicit flag
for this, and the ML script reproduces the effect with IK plus clamped bezier. Whether Cascadeur also
rotates the foot about the contact edge (heel-to-toe roll) is UNKNOWN.

### 2.3 Fulcrum detection (OBSERVED names, logic INFERRED)
`FUN_18017dc30` registers `CustomLogicFulcrumPoint` with the fields `IsCustomLogicFulcrumPoint`,
`UseOnlyKey`, `VelocityWeight`, `HeightWeight`, `ActivationThreshold`, `GlobalActivationThreshold`,
`NMaxPoints`, `WindowSize`, `Percentile`, `MaxSeparationDistance`, `CharacterThresholdHeightPercent`,
`PostFiltration`, `NElementsAnomalities`, `FiltrateByPoints` and `FiltrateByGroups`.
**INFERRED:** each candidate point gets a per-frame score of `VelocityWeight·speed + HeightWeight·height`
(height relative to the character's height percentage), smoothed over `WindowSize` frames with a
percentile. The point is a fulcrum when the score is below `ActivationThreshold`. At most `NMaxPoints`
are kept, and short anomalous runs (`NElementsAnomalities`) are filtered. The exact formula is
UNKNOWN. The UI option "Using only key frames" maps to `isOnlyKeyFulcrumPoints`. A `FulcrumCollisionInfo`
call returns `{frame: set(point ids)}` for the fulcrum, floor and touch points.

### 2.4 Fulcrum motion cleaning (OBSERVED strings)
The settings are `Fulcrum Point Indent`, `Inside Fulcrum Interval Smoothness Frame Count`,
`Outside Fulcrum Interval Smoothness Frame Count` and `Use Fulcrum Motion Clearing`. The option
`Call Fulcrum Motion Cleaning After Interpolation` belongs to the inbetweening tool. The log/error
strings are `"No touch points found for fulcrum group. Can't perform cleaning"`,
`"No local coordinate system found for fulcrum group in frame {}"` and
`"Fulcrum motion cleaning completed from {} to {} frames"`.
**INFERRED:** for each contact interval, the cleaner builds a local frame from the group's touch points
(for example heel and toe). It holds that frame fixed in world space across the interval, then blends
over N frames inside and outside the interval edges. This is anti-sliding with smooth entry and exit.

---

## 3. AutoPhysics and AutoPhysicsApply

### 3.1 Scene behaviours (OBSERVED, `_generated.py` and `domain_scene FUN_1802460e0`, `FUN_180251130`)
- The `AutoPhysics` behaviour sits on the CoM object. Its fields are `frame_weight`,
  `rotation_blending` (reset to **100.0** by `commands/restore_values.py`), `vertical_jerk`,
  `horizontal_jerk` and `priority_frame`.
- `AutoPhysicsApply` has the fields `keep_global_rotation` and `keep_global_translation`.
- The `ApplyType` enum is `["Mutable", "Local fixed", "Fixed"]` (`domain_scene @ 180660430`). Which
  behaviour uses it is UNKNOWN.

### 3.2 Pipeline stages and blend amounts (OBSERVED, setting descriptors `FUN_18017e…`–`FUN_180184…`)
Each stage has an on/off toggle (default on) and an alpha from 0 to 100 (step 1):

| Stage | Toggle fn | Alpha (default) |
|---|---|---|
| Physics corrector | `FUN_180182100` | Physics_Alpha 100 (`FUN_18017e970`) |
| Separation of motion | `FUN_1801829e0` | Separation_alpha 50 |
| Smooth trajectory | `FUN_180182b40` | Trajectory_alpha 50 |
| Smooth rotation | `FUN_180182a90` | Rotation_alpha 50 |
| Secondary motion | `FUN_180182940` | Secondary_alpha 50 |
| Compensation motion | `FUN_18017fe00` | Compensation_alpha 50 |
| Penetration cleaning | `FUN_18017fd50` | — |

More defaults (OBSERVED):
- **Separation of motion:** `Linear momentum weight` = **1000** (`FUN_180181300`) and
  `Angular momentum weight` = **0.001** (`FUN_18017f730`). The UI range goes up to `1000000000`.
- **Physics corrector:** `Normal ballistic weight` = **1000**, `Tangent ballistic weight` = **1**,
  `Strict ballistics` = off, `Prioritize ballistics` = on, `Prioritize first ballistic frame` = on,
  `Prioritize last ballistic frame` = on.
- **Trajectory smoothness:** `Normal ballistic weight` = 1000 and `Tangent ballistic weight` = 1, with a
  finer step of 0.01.
- **Ragdoll:** `With gravity` = on. **Secondary motion:** `With gravity` = off.

The internal settings registry (`FUN_18017c700`) lists these groups and names (OBSERVED):
- **Corrector weights:** `Frame Weight`, `Key Frame Weight`, `Ballistic Trajectory ConstraintWeight`,
  `Ballistic Trajectory AM Constraint Weight`, `Moving Point Weight`, `Force Constraint Weight`,
  `Torque Constraint Weight`, `Friction Constraint Weight`, `Geometrical Constraint Weight`,
  `Jerk Constraint Weight`.
- **Ballistics:** `Enable Nonlinear Optimization In Ballistics`, `Frame Weight Rotation`,
  `Angular Velocity Direction Preserve Weight`, `Ballisitc Rotation Constraint Weight`,
  `Weight of First and Last Ballisic Frames`, `Allow Ballistic Overlapping Intervals`.
- **Common:** `Weight of Priority Frames`, `Fulcrum Smoothness Frames`, `Start End Interpolation Frames`,
  `Priority Frame Interpolation Frames`, `Move Fulcrum Points Without Fulcrum Group`,
  `Restore Directions in Apply`, `Straightforward Apply`.
- **Nonlinear solver:** `Enable Ghost Forces`, `Size of Ghost Points Square`, `Nonlinear Max Iterations`,
  `Max Solver Time Seconds`, `Physics Engine`, `Position/Velocity/Collision Steps`, `Angular Damping`,
  `Linear Damping`, `FPS`, `Child Mass Multiplier`, `Inertia Multiplier`.
- **Floor:** `Floor Friction`, `Floor Bounciness`.
- **Cleaning:** `Max IK Iterations`, `IK Step Size`, `Enable Hinge Constraints`.

The setting-to-default mapping inside the 0x4d0-byte constructor could not be aligned with these
offsets, so most of these defaults are UNKNOWN.

### 3.3 What the solver optimizes (INFERRED from names; solver body UNKNOWN)
The corrector solves a weighted least-squares problem over the CoM trajectory, one row per frame, with
these terms:
- **Frame fidelity:** stay near the animated CoM. Keys and priority frames get higher weights
  (`Frame Weight`, `Key Frame Weight`, `Weight of Priority Frames`).
- **Ballistic:** on intervals with no fulcrum contact, the CoM must follow a parabola under gravity.
  The normal-to-path error weighs **1000×** more than the tangential error by default, so the arc shape
  is enforced while timing along the arc stays free. The flight's first and last frames are prioritized.
- **Angular momentum (AM):** with no contacts, angular momentum about the CoM is constant
  (`Ballistic Trajectory AM Constraint Weight`, `Angular Velocity Direction Preserve Weight`).
- **Contact dynamics:** the force, torque and friction terms keep the ground reaction force
  achievable, with a friction cone at the fulcrum points.
- **Smoothness:** jerk penalties, split into vertical and horizontal per behaviour.
- **Separation of motion:** split the result into a linear and a rotational part. The defaults strongly
  favour preserving linear momentum (1000 vs 0.001).
- **Physics Engine** (Jolt is shipped, and `domain_scene` imports `JPH::Shape::GetCenterOfMass`):
  secondary motion and ragdoll run as rigid-body simulations (`Simulation Steps`, `Spring
  Frequency/Damping`).
- **Ghost forces:** virtual support forces at "ghost points" for limbs that have no fulcrum group.

### 3.4 Apply (OBSERVED)
- `FUN_1801a6e00` shows the dialog `"Apply AutoPhysics"` /
  `"Which frames should AutoPhysics results be applied to?"` with the choices **All Frames** or
  **Only keys**. It then calls `Tool::snapWithAdditionalLogic(bool)` and the action `snap_autophysics`.
- `FUN_18018d0b0`: if a layer lacks a key on a frame that apply needs, the user sees
  `"The layer is missing a keyframe for AutoPhysics on frame {}..."` and the code calls
  `snapWithBorderKeys`.
- AutoPhysics is skipped when Ragdoll is on (`FUN_1801f2570`). It also conflicts with the Fulcrum
  Visualizer (`FUN_1801a69f0`).
- Without a Pro licence, interaction between several CoMs is limited, and only floor collision is used
  (`FUN_1801f0b30`).
- The stub for `StaticPointsTypes.get_interpolation_group` reads "points that should be interpolated
  and use instead direction controller in apply".
- **INFERRED apply:** the solved CoM and rotation are written back into animation keys, either on
  every frame or on existing keys only. `keep_global_translation/rotation` decide whether a controller
  keeps its world transform or follows the body. Limbs in the interpolation group are re-interpolated
  instead of baked, and direction controllers get restored (`Restore Directions in Apply`).

---

## 4. Center of mass

**OBSERVED** in `rigging/rig_builders/cms/objects/center_of_mass.py` and
`rig_gen/connection_objects/center_of_mass.py`:
- The CoM node `"Sum objs"` is a `WeightedAverageVector3f` over every rigid body in the CoM, with
  `operands[i].element = rigid body Position` and `operands[i].weight = rigid body Mass`:
  ```
  CoM = Σ m_i · p_i / Σ m_i
  ```
  `p_i` is the rigid body object's `Position`. INFERRED: that is the body's own centre, not the joint.
- `CoM Mass = Σ m_i` (`total_mass += source_node.node('Mass').value()`).
- The CoM rotation is copied from the first source rigid body (`CopyRotation`).
- `Fixation = Fixed AND NOT is_interpolation`. The weighted average is active only when the CoM is
  neither fixed nor driven by its own animation (`NOR` of "CoM Actual" and fixation). When the CoM is
  fixed or keyed, the binding pushes it back onto the bodies instead.
- The `CenterOfMass` behaviour has the fields `rigid_bodies`, `pos_global_target`, `is_fixed` and
  `mesh_objects`. Composite CoMs nest other CoMs (`ProtoCenterOfMass.cms`).

Default biped masses in kg (OBSERVED, `qrt/biped/constants.py`, second argument of `RigValues`):

| Part | Mass | Part | Mass |
|---|---|---|---|
| pelvis | 12.75 | thigh (each) | 5.75 |
| stomach | 7.6 | calf (each) | 4.37 |
| chest | 9.475 | foot (each) | 1.7 |
| neck | 2.885 | toe (each) | 0.9 |
| head | 3.8 | clavicle (each) | 2.12 |
| upper arm (each) | 2.52 | forearm (each) | 1.52 |
| hand (each) | 1.0 | fingers | 0 |

The total is **76.27 kg** (INFERRED, summed). The trunk (pelvis to head) is 36.5 kg, about 48 %.
Rigid bodies also carry `principal_moments`, `ellipsoid` and `width_rigid_body`, which the
angular-momentum terms use (INFERRED). A unit with mass below 0.001 and physics enabled raises
`"has zero mass"` (OBSERVED, `create_points.py`).

---

## What Fulcrum should implement

These are concrete additions to `app.js`, which already has segment CoM, friction-limited momentum,
two-bone IK foot locks, hand pin and ballistic-free.

1. **Cascadeur mass table.** Use the masses in §4 as the default segment masses, map them to the UE
   names already in `major`, and give fingers zero mass. Weight each segment at its midpoint, not at its
   joint.
2. **Ballistic interval from the CoM.**
   - When the user marks a flight interval `[a, b]`, set `p0 = CoM(a)` and
     `v = (CoM(a) - CoM(a-1))` per frame.
   - Use `g = 9.81·upm / fps²` per frame² on world Y only.
   - Draw `p(t) = p0 + v·t - ½·g·t²·ŷ` with start, apex (`t* = v.y/g`), landing and velocity handles.
3. **"After inflection point" check.** If enabled and `frames ≤ ceil(v.y/g)`, refuse with "not enough
   frames to reach max height". If disabled and the arc cannot fall back down, warn that this is not a
   typical ballistic.
4. **Landing-stable solve.** When the user drags the landing point or changes the frame count with
   *landing stable* on, re-solve `v` so that the arc hits the landing point:
   `v = (pL - p0 + ½·g·t²·ŷ) / t`.
5. **Snap CoM to the arc.** On every frame in the interval, offset the root by `arc(t) - CoM(t)` and
   keep the limb poses. This is one command and it can be undone.
6. **Two-term fit instead of a hard snap.** Minimize `wN·|normal error|² + wT·|tangent error|²` with the
   defaults `wN = 1000` and `wT = 1`. The arc shape becomes exact while the spacing along it can drift.
   Weight the first and last flight frames higher.
7. **Fulcrum detection per frame.** For hands, feet and toes only, score `speed·wV + height·wH`, smooth
   the score over a window, and threshold it. Add per-point overrides `Fulcrum / Enforce / NotFulcrum`.
8. **Fulcrum key/interval flag.** Add a per-interval flag. When a limb's contact is present on both
   keys, hold the contact in world space with IK and use clamped interpolation. This generalizes
   today's foot locks to any fulcrum group.
9. **Fulcrum motion cleaning.** Build a heel-toe local frame for each fulcrum group, hold it fixed
   across the contact, and blend N frames inside and outside the interval edges.
10. **Flight angular momentum check.** In ballistic intervals, compute `L = Σ(Iᵢωᵢ + mᵢ rᵢ×vᵢ)` about
    the CoM and flag frames whose `|ΔL|` exceeds a tolerance. A later fit can keep `L` constant.
11. **Apply dialog.** Bake the results to **All frames** or **Only keys**. Offer
    `keep global translation/rotation` per controller and a blend alpha from 0 to 100 (default 50
    for smoothing stages, 100 for the corrector).
