# Maya 2027 animation math: curves, layers, skinCluster

Labels:
- **MEASURED**: matched numerically against Maya. The checks ran on headless mayapy fixtures and on a live Maya 2027 session through commandPort 7002.
- **CODE**: read from x86-64 disassembly. The tool was capstone over PE exports from `pefile`.
- **UNKNOWN**: not resolved.

REA was tried first. Every deep REA tool failed with `provider_unavailable`: Ghidra is not configured for REA, REA's Ghidra path refuses DLLs on Windows, and Hopper and IDA are absent. Ghidra was not used directly because another agent owns it. The disassembly helper and the probe scripts are in `probes/`. The fixtures are in `tests/fixtures/maya_{curves,layers,skin}.json`.

## Results

| Area | Fixture max error | Live Maya max error |
|---|---|---|
| Curve eval, unweighted (Hermite) | 2e-13 | 4e-14 |
| Curve eval, weighted (Bezier) | 1.6e-9 | 4e-14 |
| Tangents, all automatic types, 24 and 30 fps | 8e-14 deg/weight | 2e-14 |
| Infinity, all 5 modes | 5e-12 | n/a |
| Blend nodes (DL/scale/bool/rotation, 3 modes x 6 orders) | 1e-13 (rotation 5e-6 deg) | 7e-14 deg |
| Full layer stacks on a joint (24 random) | 1e-13 | n/a |
| skinCluster linear / DQ / blended | 2-5e-7 (the mesh stores float32 points) | 2.4e-7 / 2.2e-7 |
| skinPercent normalization (abs/rel, locks) | 1e-16 | n/a |

The tests are `node tests/maya_curves.test.mjs`, `maya_layers.test.mjs` and `maya_skin.test.mjs`: 24 tests, all passing.

## 1. Animation curves (AnimEngine.dll)

### Classes and dispatch (CODE)
- `T2dHermite` is the unweighted interpolator and `T2dBezier` the weighted one. Both derive from `T2dAnimCurve`.
- `T2dAnimCurve::calculateTangents` switches on the internal `TtangentType`:
  - 1 fixed (no-op), 2 linear, 3 flat, 4 oldSmooth, 5 step (no-op), 6 slow, 7 fast, 8 oldClamped
  - 9 spline (`calculateSmoothTangents`), 10 clamped, 16 plateau, 17 stepnext (as step)
  - 18 auto, 27 automix, 28 autoease, 29 autocustom (all through `calculateAutoCubicTangents`)
- Time is stored in ticks: 141,120,000 per second. All math runs in seconds.

### Units (MEASURED)
- Key tangent angle = `atan(dv/dframe)` in the current time unit.
- Weight = |(dx_frames, dy)| / 3, which is the Bezier handle length.
- `getTangentXY` returns 3x the handle, in seconds. It is a unit vector on unweighted curves.
- `curves.js` works in frames with Maya's angle and weight. This is exact because time is affine.

### Evaluation (CODE, MEASURED)
**Hermite** (`T2dHermite::create` / `evaluate`):
- Inputs: `m1 = tan(outAngle k0)`, `m2 = tan(inAngle k1)`.
- If both tangents are collinear with the chord within `|m*dx*2e-7|`, the segment is linear.
- Otherwise:
  - `a = (m1 dx + m2 dx - 2dy)/dx^3`
  - `b = (3dy - 2 m1 dx - m2 dx)/dx^2`
  - `c = m1`, `d = y0`
- The value is evaluated by Horner in `t - t0`.

**Bezier** (`T2dBezier::create`):
- Normalized handle times: `u1 = (x1-x0)/dx`, `u2 = (x2-x0)/dx`.
- If `u1 ≈ 1/3` and `u2 ≈ 2/3` (within 1e-6), time is linear and no root solve is needed.
- Clamping:
  - `u1 < 0` becomes 0, and `u2 > 1` becomes 1.
  - If `u1 > 1` or `u2 < 0`, `checkMonotonic`/`constrainInsideBounds` pulls the handles into the monotonic ellipse. This uses the 4/3 bound with eps 1e-10 and flag = true. Flag = false was measured as wrong.
- Handle y values are rescaled by the same ratio as their u.
- Evaluation solves `Bx(u) = s` for u, then evaluates `By(u)`.
- The 1.6e-9 residual is Maya's own root solver at `0x1800470b0`.

**Step**:
- `outType step` holds v0.
- `stepnext` holds v1.

**Infinity** (MEASURED):
- The enum is constant 0, linear 1, cycle 3, cycleRelative 4, oscillate 5. There is no 2.
- Linear extrapolation uses `tan(inAngle)` of the first key and `tan(outAngle)` of the last key.
- cycleRelative adds `(v_last - v_first)` per cycle.
- Oscillate mirrors odd cycles.

### Tangent rules (CODE, MEASURED to 1e-14)
Notation: h1 = t - tp, h2 = tn - t. Weighted tangents use x = h (the full segment) and weight = |(h, h*s)|/3. Unweighted tangents keep only the slope s.

- **linear**: each side points at its neighbour. A missing neighbour gives (1 s, 0), so its weight is fps/3.
- **flat**: (h, 0). A missing side copies the other side's h.
- **spline**:
  - Interior: s = (vn - vp)/(tn - tp).
  - End keys: the chord to the single neighbour.
- **clamped**:
  - Tolerance is 0.05 / normScale, and normScale defaults to 1.
  - If |vp - v| ≤ tol or |vn - v| ≤ tol, both sides are flat. Otherwise each side is spline.
  - A missing neighbour is replaced by the other neighbour's value.
  - The end key's missing side copies the other side.
- **plateau**:
  - End keys are always flat.
  - Interior keys are flat in any of these cases:
    - The key is a local extremum (by 1e-10).
    - The key's value equals a neighbour's (within 1e-10).
    - The control-point hull of either adjacent segment, built from the trial spline tangent and the neighbour's current tangent, overshoots both of its endpoints. The hull is `getBoundingValue`: min/max of `v0, v0 + m0 dt/3, v1 - m1 dt/3, v1`.
  - Otherwise the key is spline.
- **auto family** (helper at `0x180048390`):
  - End keys are flat.
  - Interior keys use `lim1 = 3 d1/h1` and `lim2 = 3 d2/h2`.
  - The candidate slope s depends on the type:
    - auto: (vn - vp)/H
    - automix: `(1-a) m1 + a m2`, with `a = h1/H`
    - autoease and autocustom: weight `w = b(4c b^2 + 1 - c) + 0.5`, with `b = a - 0.5`, then `s = (1-w) m1 + w m2`
    - autoease uses `c = -0.5`; autocustom uses `c = currentSceneCubicCoefficient`, which is -0.5 by default.
  - If the signs of lim1, lim2 and s differ, s = 0. Otherwise s is clamped to min/max(s, lim1, lim2).

## 2. Animation layers (DataModel.dll, MEASURED)
- **Stack order:** each layer's blend node takes the layer below it as inputA, starting from BaseAnimation, and the layer curve as inputB. `weightA = layer.backgroundWeight` and `weightB = layer.foregroundWeight`.
- **Layer weights:**
  - fg = 0 if muted, otherwise `weight * parentWeight`.
  - bg = 1 for an additive layer or a muted layer.
  - bg for an override layer is `1 - weight` with passthrough on, or 0 with it off.
  - The override bg uses the layer's own weight, not the parent's.
- **A non-passthrough override at weight < 1 fades toward zero, not toward the base.**
- **Scalar (animBlendNodeAdditiveDL):**
  - additive: `wa A + wb B`
  - multiply: `sp(A,wa) * sp(B,wb)`, where `sp(x,w) = sign(x)|x|^w` and w = 0 gives 1
- **Scale:**
  - additive is plain `wa A + wb B` (with no "-1").
  - multiply is the same signed-power product.
- **Boolean:** the result is B if wb > 0, otherwise A.
- **Rotation, by the node's accumulationMode** (the layer's `outRotationAccumulationMode`):
  - **component** (rotationAccumulationMode 0): per-channel `wa A + wb B` in Euler.
  - **layerCumulative** (by-layer on an additive layer): `q = pow(qA, wa) * pow(qB, wb)`.
    - The product is Hamilton, with B local to A.
    - `pow` scales the angle `2 atan2(|v|, w)` without forcing the shortest arc. This is `Tquaternion::weight`.
  - **layerBlended** (by-layer on an override layer): with `S = |wa| + |wb|`, `q = slerp_shortest(pow(qA, S sA), pow(qB, S sB), |wb|/S)`.
    - This was read from the inverse solve and confirmed by measurement.
  - The quaternion comes from Euler using the joint's rotateOrder.
  - **Euler back:** Maya takes the principal solution (middle axis within ±90°) and the alternate `(i+180, 180-j, k+180)` without wrapping. It keeps whichever has the smaller |x|+|y|+|z|.
  - `rotationInterpolation` (Euler / Quaternion Slerp) had no effect in any mode.
- **Keying on a layer** (`setKeyframe -animLayer`) stores the value from `TDNanimBlendNodeAdditiveRotation::Resolve`, which is the inverse solve. That makes the composite equal the typed value. It is not ported yet.

## 3. skinCluster (DeformSlice.dll)
- **Space:**
  - Input points go to world by `geomMatrix`.
  - Influence matrix `M_i = bindPreMatrix_i * matrix_i`, using row vectors.
  - The result goes back by `inverse(geomMatrix)` (MEASURED).
- **Linear:** `Σ w_i p M_i`.
- **Normalize weights:**
  - none: raw weights. Sums ≠ 1 scale the point.
  - post: divide by the sum at deform time.
  - interactive: weights are kept normalized on edit.
- **Zero-weight vertex:**
  - linear returns the original object point.
  - DQ returns the world point uncorrected. This is a Maya quirk and is ported.
- **DQ, rigid path** (`computeDQSMatrices`, CODE, MEASURED):
  - Decompose `M_i` into scale, shear, rotation and translation, then form `dq(conj rotation, t)`.
  - Blend the DQs with weights after a hemisphere flip against the first influence, normalize by |real|, and apply.
  - Per-influence scale (no shear) is blended linearly as a matrix and applied before the DQ. It is not renormalized, so raw weights scale the point.
- **DQ, dqsSupportNonRigid** (CODE, MEASURED):
  - `pre_i = bindPre_i * SRT(S_world, H_world, R_bind, T_bind)`.
  - The rigid DQ comes from `bindPre_i * SRT(S_bind, H_bind, R_world, T_world)`.
  - `dqsScale` wraps both: `inv(dqsScale) * world` and `dqsScale * bindWorld`. Only the default (1, 1, 1) was measured.
  - **Gate** (MEASURED, thresholds bracketed by sweeps): the non-rigid matrix blend runs only if at least one influence's world `matrix_i` (not `bindPre_i * matrix_i`) is non-rigid: some |scale - 1| > 1e-10 (1+8e-11 off, 1+1e-10 on) or some |shear| > 1e-4 (0.9e-4 off, 1.1e-4 on). The gate is global to the skinCluster. A scaled influence with zero weight on every vertex still turns it on. Bind-time scale does not count.
  - **Gate on:** `p' = R(s * Σ w_i pre_i * p) + T`, with `s = Σ w_i` (raw weights after the normalize mode). The DQ is still normalized by |real|. The extra factor `s` is why raw weights went quadratic: one joint with weight w gives `T + w² R(pre p)` (MEASURED in a weight sweep of 0.5/1/2/3, pure quadratic, no linear term).
  - **Gate off:** the DQ still comes from the non-rigid split (bind scale/shear kept, world R/T), but `pre_i = I`, so `p' = R(s p) + T`. Bind scale is ignored here. The rigid path would apply it.
  - With normalized weights, `s = 1`, which is why this was invisible before.
  - Confidence: MEASURED. 80 fixture cases in `tests/fixtures/maya_skin_nonrigid.json` (from `probes/probe_skin_nonrigid.py`) cover DQ and blended, raw and post, non-rigid on and off, rigid, uniform, non-uniform, one-joint, parent scale+rotate, shear, below-gate scale and shear, and bind-scaled kept and reset. Max error is 1.6e-6, which is float32 storage on points up to ~30 units. Ghidra was not checked.
- **Weight blended:** per-vertex `(1-b) linear + b DQ`, where b comes from `blendWeights` (MEASURED).
- **skinPercent normalization:**
  - Set the target. Absolute values clamp to [0,1]. Relative values may go negative, but never above `1 - Σlocked`.
  - Rescale the other unlocked influences proportionally to fill the rest. If they sum to 0, the rest is split equally.
- **Closed 2026-10-15:** raw weights + DQ + dqsSupportNonRigid + scaled joints used to be off by up to 31 units. They now match to 1.6e-6 (see Gate above).
- **UNKNOWN:** non-default `dqsScale` combined with the gate; the exact scale threshold between 8e-11 and 1e-10; whether Maya's shear tolerance is tested on the same shear terms as Fulcrum's `decompose` (single-axis sweeps agree, combined shears were not swept).
- Run probes from a copy outside `re/maya/probes`, because `probes/dis.py` shadows the stdlib `dis` and crashes mayapy.

## Port to Fulcrum
1. Use `curves.js` `computeTangents` + `evaluate` to read and write Maya-exact curves in FBX/`.anim` round trips, with auto/plateau/clamped exactly as Maya does them.
2. Use `layers.js` `evaluateLayerStack` for the edit layer. Override with passthrough is lerp. Additive by-layer rotation is `qA * qB^w`, which is the right choice for rotation offsets.
3. Use `skin.js` `skinPoints` (linear/DQ) for preview deformation, with `setInfluenceWeight` for paint normalization.
4. Not done:
   - the layer-key inverse (`Resolve`)
   - the legacy fast/slow/old* tangent types
