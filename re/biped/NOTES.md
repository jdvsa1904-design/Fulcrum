# 3ds Max 2027 Biped footstep engine: decoded rules

Target: `C:\Program Files\Autodesk\3ds Max 2027\stdplugs\biped.dlc` (3ds Max 2027.2, 29.2.0.20588). MD5 `1547fc15466d317de0ca936a592282a4`. Analysis ran on the byte-identical copy `C:\re_work\max\biped.dlc`. The install was not modified.

Port: `re/biped/footsteps.js`. Tests: `tests/biped_footsteps.test.mjs` and `tests/biped_balance_adapt.test.mjs` (12 tests, all pass). Fixtures: `tests/fixtures/biped_max_*.json`.

## How it was done

- **REA could not open the DLL.** REA's only Windows deep provider is Ghidra. It reports `not_configured`, and `target_role_unsupported: Windows Ghidra P0 accepts PE applications, not DLL`. Ghidra was off limits for this task.
- **Static analysis.** Done with capstone 5.0.7 and pefile instead. Function bounds come from `.pdata` (11,553 functions). The full listing is `C:\re_work\max\rea_bip\biped.asm`. Helper scripts are in `C:\re_work\max\rea_bip` (`build.py`, `dx.py`, `fn.sh`, `summ.py`, `u.py`).
- **Batch measurement.** 3dsmaxbatch scripts are in `C:\re_work\max\tests\fsgen` (`lib.ms`, `s1`-`s6.ms` with their `.out` files). The parsers and fits are in `C:\re_work\max\rea_bip\an`.
- **Live check.** Six new cases were run in the live 3ds Max 2027.2 through the Python bridge on 7003 (`C:\re_work\max\rea_bip\js\live.py`). The script made its own Biped, deleted it afterwards and restored the multiple-footstep dialog values. The scene was empty before and after, the animation range is back to 0-100, and nothing was saved.
- **Labels:** OBSERVED (in the code or bytes), MEASURED (read back from Max and fitted, with residuals), INFERRED, UNKNOWN.

Units: S is Biped's leg-length unit. Every stride and width parameter is a multiple of S. On a default Biped, `S = 0.481928 * height` (MEASURED: heights 100, 170 and 250 give identical results in S units). Time is in frames at 30 fps (160 ticks per frame).

## 1. Footstep generator (Create Multiple Footsteps)

### 1.1 Entry points and parameter blocks (OBSERVED)

| What | Address |
|---|---|
| MAXScript `biped.getMultipleFSParams <#walk/#run/#jump>` (one argument, the gait name, not the controller) | `1802a0400` |
| Gait to parameter block | `1802b9670`: walk `180540660`, run `1805406a0`, jump `1805406e0` |
| MAXScript `addMultipleFootprints` | `180285750`. It saves the gait mode, calls IBipMaster `+0x240` SetGait, then `+0x278` AddMultipleFootprints. |
| MAXScript `newFootprintKeys` | `1802aafc0` → IBipMaster `+0x280` |
| IBipMaster vtable | `1804532c0`. It sits at BipMaster `+0x88` (RTTI COL `1804dca00`); GetInterface id `0x9172` returns `[ctrl+0x48]+0x88` (`180034df3`). |
| AddMultipleFootprints | `1800840c0` |
| NewFootprintKeys | `18008c8b0` |
| Footstep timing | `180085330` (decoded in full below) |
| Zero-speed test | `180348f40` (`|v| < 1e-4`, constant `1804ca5e0`) |

The parameter block is 0x40 bytes:

| Offset | Field | Defaults (walk / run / jump) |
|---|---|---|
| +00 | numFootsteps | 4 |
| +08 | width | 1.0 |
| +10 | strideLength1 | 0.75 / 1.5 / 2.4 |
| +14 | height1 | 0 |
| +18 | cycle1 | 0 in the file; MEASURED at runtime as 15 / 15 / 20 frames |
| +20 | strideLength2 | same as strideLength1 |
| +24 | height2 | 0 |
| +28 | cycle2 | as cycle1 |
| +2c | autoTiming | 1 |
| +30 | interpTiming | 0 |
| +34 | alternate | 1 |
| +38 | multiInsertInTime | 0 |

The fields at +04 (7.229) and +3c are UNKNOWN.

### 1.2 Timing law, `180085330` (OBSERVED, with every measured case reproduced exactly)

Arguments: gait (1 walk, 2 run, 3 jump), S, step length, and cycle `c` in ticks. The arithmetic is float32.

```
v = |step| / c                                  (units per tick)
walk:
  gap = max(0.93*S - |step|, 0.05*S)            (18045f360 = 0.93, 18045f340 = 0.05)
  ds  = trunc(gap / v - tpf);  ds = min(ds, 5*tpf)
run, jump:
  dur = (|v| < 1e-4) ? trunc(0.7*c) : min(trunc(0.6*S / v), trunc(0.7*c))   (18045f350 = 0.6, 18045f358 = 0.7)
round to the nearest frame (remainder >= tpf/2 rounds up)
walk:  ds = min(ds, tpf*trunc(c/(5*tpf)));  ds = max(ds, 2*tpf)
       footstep duration = c + ds;  if 2*ds >= duration then duration = 2*ds + tpf
run/jump: dur = max(dur, 2*tpf);  airborne = c - dur
```

What the law means:

- **Run and jump contact time:** `0.6 S / speed`. The body travels 0.6 leg lengths over the stance foot, capped at 70% of the cycle.
- **Walk double support:** the time to cover `(0.93 S - step)`, minus one frame. It is capped at 5 frames and at `c/5`, and is at least 2 frames.

Port: `gaitTiming`. A 1e-3 tick tie-break reproduces one exact float tie (run with stride 2.0, cycle 15).

### 1.3 Placement and schedule (MEASURED: 55 batch cases plus 6 live cases, 0 timing mismatches, worst position error 0.004 units)

**Placement:**

- Step length is `stride * S`. It is the distance between consecutive opposite footprints.
- Half the stance width is `0.1 * S * width`. Left is +y and goes first.
- Footprint heights accumulate `actualStrideHeight` once per step.
- **Interpolate:** with `m` moving steps, step `i` uses `L = L1 + (L2 - L1) * i/(m-1)`. Height does the same, and cycle is assumed to (INFERRED, not measured).

**Schedule**, where `c` is the cycle in frames, `ds` the double support and `k` the footstep index (1-based):

| Gait | Footstep 1 (L) | Footstep 2 (R) | Footstep k ≥ 3 | Moving steps |
|---|---|---|---|---|
| walk | `[0, c+ds]` | `[0, 2c+ds+1]` (one frame longer; a Max quirk) | starts `(k-1)c`, lasts `c+ds`, at `x = sum of steps` | n-2 |
| run | `[0, dur]` | `[0, dur]` | starts `(k-2)c`, lasts `dur` | n-2 |
| jump | both feet in pairs | | pair j starts `j*c`, both feet at the same x, lasts `dur` | n/2 - 1 |

- **autoTiming = false:** Max ignores `cycle1` and reuses the last timing it computed. The rule is UNKNOWN, so the port does not support this mode.

## 2. Body (COM) motion that `newFootprintKeys` creates (MEASURED)

### 2.1 Key layout

- **Run and jump:** horizontal and vertical keys sit at every touchdown and liftoff.
- **Walk:**
  - Horizontal keys sit at the middle of single support: `touch + ds + (c-ds)/2`.
  - Vertical keys sit at touch, liftoff and mid single support.
- **Lateral position:** the COM root never moves laterally (`y = 0` on every key). The lateral sway comes from balance (§3).

### 2.2 Laws, with residuals

| Law | Statement | Residual |
|---|---|---|
| Leg-reach circle (run/jump) | At touchdown and liftoff, the distance from COM to footprint equals the standing COM height `R0 = 1.10375 S` | 36 cases: within 0.6% |
| Mid-stance offset | The COM is ahead of the stance footprint by `m = 0.9595*v + 0.0107*S` at mid-stance (v in units/frame at 30 fps) | run max 0.0015 S; jump max 0.02 S. Walk uses the same law: predicted 4.81 units, measured 4.86. |
| Steady speed | In steady state the horizontal speed is uniform, `v = step/c`, through stance and flight | |
| Start-up | The first liftoff key from the standing pair is at `x = 0.5 S` | Later early keys are irregular; that transient is not modelled. |
| Walk heights | Touchdown: the COM is on a circle of radius `1.1185 S` about the front (heel) footprint. Liftoff: on `1.155 S` about the rear (toe) footprint. Mid single support: on `R0` about the stance footprint. | spread ±0.6% |
| Flight | Ballistic, `v_up = gT/2 + dz/T`, `v_dn = gT/2 - dz/T` (decoded before, `FUN_1802f6090`). Default `g = 5.63857*H/s² = 11.70 S/s²`. | |
| Stance (run/jump) | Cubic Hermite from the touchdown to the liftoff key. End tangents are the flight velocities × `1.2*(1 - ballisticTension)`. The 1.2 is fitted. Max itself uses a bisected Bezier (`FUN_1802f7260`). | |

**Model error against Max per frame, steady state:**

| Cases | Horizontal | Vertical | Note |
|---|---|---|---|
| Batch walk/run | ≤ 0.54% S | ≤ 1.2% S | half-stride walk vertical 2.3% S |
| Live held-out walk/run | ≤ 0.53% S (0.46 cm at H = 180) | ≤ 1.39% S (1.17 cm) | |
| Live jump | 2.9% S | 1.6% S | |

The start transient (first two steps) is not modelled. The error there reaches about 35 units.

## 3. Balance factor (MEASURED; the law is exact to print precision)

- **Measurement:** `balanceFactor` was set on every horizontal key to 0, 1 and 2 on a walk and a run (`s4.ms`).
- **What moves:** the COM root does not move. The pelvis and every node above it move together by a horizontal vector (z = 0). The feet stay planted.
- **Law:** with bf = 0 the pelvis sits exactly on the COM root. The shift is linear in bf, and bf = 2 doubles bf = 1 (to 0.002).

```
pelvisShift = -bf * 0.794 * horizontal(Spine3 - Spine1)
```

Residual 0.003 units on peak shifts of 1.65 units (rms 0.0003 over 600 frames).

**Reading:** Biped leans the trunk itself, sideways toward the stance foot. Balance then slides the hips the opposite way, so the trunk's mass point stays over the COM.

- The gain 0.794 over the Spine1→Spine3 segment (21.3 units) is equivalent to holding a point 26 units above the pelvis, about 0.15 H, over the COM. That is roughly the trunk centroid.
- Spine nodes and head are collinear, so any equivalent pair fits (for example `0.7676*Spine1 - 0.7856*Spine3`).
- The ±0.70-unit lateral pelvis sway seen at bf = 1 is this term.

It is not a ZMP or acceleration law; the old fit `s ≈ 1.84*a_com` was a correlation. The steady run has zero COM acceleration and the same offset. The static code path was not traced, so the gain is a fit, not a constant read from the binary.

Port: `balanceOffset(lower, upper, bf, upAxis)`. For a UE-style spine, use spine_02 and spine_04.

## 4. Footstep editor adaptation (MEASURED; mechanism partly UNKNOWN)

- **Adapt locks (OBSERVED):** globals `180613034..18061304c`, read through IBipMaster `+0xf0` (`1802b7ff0`). In jump-table order: Horz, Vert, Turn, LLeg, RLeg, Freeform, Time.
- **Readers:**
  - Horizontal: `1802bf7a0`, `1802d1650`, `1802f4291`.
  - Legs: `1802bef50`.
  - Freeform: `1802becc5`, `1802bfe8b`.
- **The math was not reached statically.**

Experiment: footstep 6 of a 10-step walk (c = 15, step 0.75) and run (c = 15, step 1.5) was moved by Δ (`s5.ms`, `s6.ms`). The table gives the change in each COM key divided by Δ. Keys are relative to the edited footstep: the walk keys are at mid single support; the run keys are touch and lift.

| Edit | Previous step | Edited step | Next step |
|---|---|---|---|
| walk, along path (x) | 0.042 | **0.988** | -0.029 (-2: -0.002) |
| walk, lateral (y) | 0.232 | **0.575** | 0.191 |
| run, along path | touch 0.033, lift 0.231 | **touch 0.799, lift 0.785** | touch 0.175, lift -0.023 |
| run, lateral | touch 0.110, lift 0.265 | **touch 0.654, lift 0.644** | touch 0.221, lift 0.065 |

What holds and what does not:

- **Linear in |Δ| along the path:** x10, x30 and x60 give identical weights.
- **Cycle-independent, roughly:** cycles 10, 15 and 20 give 0.97 / 0.99 / 0.99.
- **Height edits:** a z edit (step height) changes no COM key (vertical stays within 0.01).
- **Pair edits** are roughly additive.
- **Not separable by axis:**
  - A diagonal edit gives one scalar weight parallel to Δ. Walk (30, 10) moves the mid key 0.855Δ on both axes, (10, 10) 0.685, and (60, 10) 0.908.
  - A lateral run edit also moves the run keys along the path, in proportion to |Δy| and with alternating sign (0.456, -0.51).
  - So Max solves something geometric. It is not a fixed kernel.

### 4a. Adaptation law, decoded (2026-10-15, Ghidra project "test", /biped/biped.dlc)

- **Adaptation is regeneration (MEASURED, `s8.ms`):** with Horz unlocked, the adapted horizontal keys equal keys generated from scratch on the edited footsteps, to 0.0000 units in all 24 walk/run, width 1/2, axis/diagonal cases.
- **Call chain (OBSERVED):**
  - `1802bf7a0` is the adapt entry. It calls `1802c0ef0`, which copies the biped into the scratch biped at `+0x12b98`, copies the footsteps (`18033b900`), clears key tracks (`1802c59c0`) and regenerates them (`18033b7b0`, `18033d020`).
  - If Horz (`180613034`) is 0, `1802bec10` walks the horizontal keys (`+0x34728`). It takes the scratch key at the same time, or a key interpolated from the scratch track (`1802fac00`/`1802f52f0`). `1802bee00` then adds a turn-track term: the turn track at `+0x347b0` (`180339380`) rotated by the key heading at `+0x10`, normalised, and scaled by `+0xf068 * key[+0x14]`. That term is 0 on straight paths (MEASURED, the 0.0000 match).
- **Generator post-pass: `1802f3be0` (OBSERVED), called once from `18033d1a2` inside `18033d020` with cap `P = 0.25 * [+0xf068]`, over keys 1..n-3:**
  - For each interior key k_i (horizontal x,y; Max internal x,z): `u = unit((k_{i-1} + k_{i+1})/2 - k_i)`.
  - `1802f5060` returns `a = |k_i - k_{i-1}|`, `b = |k_{i+1} - k_i|`, and `bend = min(acos(dot(unit(k_i - k_{i-1}), unit(k_{i+1} - k_i))), pi/2) / (pi/2)`.
  - `k_i' = k_i + u * bend * min(0.48 a, 0.48 b, P)`. Jacobi: all new values go to a buffer (`180b15b38`) first, then are written back.
  - This explains the non-separable diagonal behaviour. A straight key line has bend = 0 and is untouched, so axis edits look linear; any lateral bump is pulled toward the neighbour midpoint by an amount that depends on the bend angle, not on |Δ|.
- **`+0xf068` ≈ S (INFERRED):** the walk fit is best at P ≈ 20 = 0.25 S for h = 170. `18033d020` also stores `0.5 * f068` at `+0x2e888` and `0.25 * f068` at `+0x2e88c`.
- **Raw keys before the pass (MEASURED by inverting the pass):**
  - Walk: the raw response to a footstep move is isotropic, the along-path kernel (0.988 own, 0.042 previous, -0.029 next) applied to both axes. Raw lateral key = foot y minus side × 0.1 S, which is why width 2 gives a ±1.35 zigzag after smoothing.
  - Run: the raw lateral response is already spread over neighbours: touch/lift 0.664/0.654 own, 0.109/0.262 previous, 0.218/0.064 next. There is also an alternating x coupling of about ±0.12 per unit |Δy|. At |Δy| = 30 the spread is about 3% wider, so it is nearly linear.
  - Run raw keys are written by `18033d690`/`18033b2f0` into `+0x2e878` and pushed by `1802d1650`. **Not decoded (UNKNOWN).**
- **Port:** `adaptCOM(plan, keys, editIndex | [[i, Δ], ...], Δ)` builds raw keys (base + kernels + width excess) and runs `smoothKeys(hk, 0.25 S)`, the exact `1802f3be0` law. `MAX.adapt.run.rawLateral`/`rawCouple` hold the inverted run kernels.

Error on `s7` (36 edits per gait/width, |Δ| = 10 and 30, 0-180°, footsteps 5 and 6), worst key error / |Δ| (MEASURED, test `diagonal footstep edits`):

| Set | Before (axis kernels) | After |
|---|---|---|
| walk w1, diagonal | 23.6% | 0.4% |
| walk w1, axis | 2.5% | 0.4% |
| walk w2 (unsupported before) | — | 0.2% |
| run w1, diagonal | 13.2% | 10.3% |
| run w1, axis | 9.2% | 7.9% |
| run w2 (unsupported before) | — | 14.3% |

The remaining run error comes from the raw run generator. `comKeys` places early run keys up to 13 units away from Max, for example touch 2 at 107 against 120. Because the bend and the 0.48 a/b limits depend on key spacing, that offset feeds into the smoothing. The run x coupling also flips sign between width 1 and width 2, so it is geometric rather than a fixed kernel.

## 5. Footstep detection (Rule A, mocap): constants confirmed

Independent capstone pass over `18010bea0`, OBSERVED:

| Step | Rule | Where |
|---|---|---|
| Contact | `|p_i - p_{i-1}| <= fsExtractionTol * S` (3-D). If it passes, `p[i-1] = p[i] = 1`, else `p[i] = 0`. | `18010c597`; S at `leg+0x644` |
| Vertical | Only when `fsUseVerticalTol`. **Frame 0 is not tested.** Clear when `h > zLevel + vt*S` or `zLevel - vt*S > h`. | `18010c6fc` |
| Fill | `1,0,1` is filled. Then `p[i] && p[i+3]` sets `p[i+1]` and `p[i+2]`. | |
| Drop | `!p[i] && !p[i+4]` clears `p[i+1..i+3]`. Runs that touch the clip ends are kept. | |
| Frame 0 | `p[0] = 1` if `p[1]` | `18010c920` |
| Footprint | Height is the maximum over the run (start -1e6, `180465a14`). The middle frame is `(s+e)>>1`. | |
| Slide | `d > footLength * fsSlidingDist / 50` (constant 50.0 at `1804659a8`), or heading change > `fsSlidingAngle` (π at `1804659b0`, 180 at `180456f90`). | |

- **Defaults (MEASURED before):** tol 0.07, vertical tol 0.5, sliding distance 25, sliding angle 0.
- **IK-blend threshold for Rule B:** 0.99 at `1804658f4` (OBSERVED).
- **Rule B round trip (MEASURED, `s4.ms`):** `convertToFreeForm` then `convertToFootSteps` worked in 3dsmaxbatch this time. It returned all 10 footsteps with identical start frames and durations. Positions came back within 0.8 units, the footprint ankle offset; z resets to 0.

**Differences from the existing `contacts.detectPlants`:**

1. It also tests frame 0 against the vertical tolerance.
2. It drops short runs at the clip ends; Max keeps them.
3. It has no `p[0]` back-fill.

`detectFootstepsMax` in `footsteps.js` is the exact version. The test `detectFootstepsMax implements Biped Rule A` shows the two disagree only at the clip edges.

Rule A has still not been read back from a mocap load, because `loadMocapFile` creates no footsteps in batch. The constants are from code.

## 6. Running in place: the sprint fix

- **Why the drill sprint runs in place:**
  - Biped's run has the body cross 0.6 S over each stance foot, while successive feet land a full step (≈1.5 S) apart.
  - The pelvis moves at `step/c`, the same speed as the footfall pattern.
  - The FRT capture moves its pelvis at 0.37 leg lengths per second, against 2.14 for the feet. That is the opposite.
- **The fix:** `sprintPlan({ speed: 3.5, S: 0.87, fps: 30 })` builds a run.
  - Step: 1.5 S = 1.3 m.
  - Cycle: 11 frames.
  - Contact: 0.6 S / v ≈ 0.15 s, rounded to 4 frames.
  - Airborne: 7 frames.
  - The COM moves at the foot pattern speed (test: within 3%).
- **For 4 m/s:** raise the speed. The contact shrinks toward Max's 2-frame floor, and the 70% cap never binds at sprint speeds.
- **Using it:** feed the footprints as planted locks and the COM as the pelvis path. Add `balanceOffset` with spine_02 and spine_04.

## 7. Open items

- **Walk start-up (UNKNOWN):** the first two steps' COM keys follow an irregular transient.
- **Footstep adaptation:** walk solved (regeneration + `1802f3be0` smoothing, ≤0.4% of Δ). Still UNKNOWN: the run raw-key generator (`18033d690`/`18033b2f0`), the exact value of `+0xf068` (≈ S, inferred), and the turn-track term in `1802bee00` on curved paths.
- **Stance vertical shape (fitted):** Max's handle bisection was decoded before; the port uses a fitted Hermite (1.2 × (1 - tension)).
- **Fields +04 (7.229) and +3c of the parameter block, and autoTiming = false (UNKNOWN).**
- **Balance gain (fitted):** 0.794 is a fit; the constant was not located in code.
