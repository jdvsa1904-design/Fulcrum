# 3ds Max 2027 Biped, CAT and IK solvers: what the binaries show

Sources: the shipped plugins `stdplugs/biped.dlc`, `CAT.dlc`, `HISolver.dlc`, `IKLimb.dlc` and their
`en-US/*.mui` string tables. They were copied to `C:\re_work\max` and analysed with Ghidra 12.1.2 headless.
The install was not modified. Shipped MAXScript (`stdplugs/stdscripts/CATScripts`, `MacroScripts`) has no
solver math; it only wires UI. Addresses are VAs. The image base is `0x180000000` for every plugin.

Dumps (all under `C:\re_work\max\out` and `C:\re_work\max\full`):
`HISolver.dlc.txt` and `IKLimb.dlc.txt` are full decompiles. `CAT.vt.txt` holds the CAT controller
vtables. `CAT.a.txt` holds CAT GetValue and its callees. `biped.dlc.txt` is the string-xref dump.
`grav.txt` lists everything that touches the gravity field. `conv.txt` is the footstep-conversion pipeline.
`herm.txt` holds the ballistic transition curves. `bip.vt.txt` holds the mocap filters.
The helper scripts (`VtableDump`, `AddrDump`, `OffsetXref`, `RefXref`, and a `StringXrefDump` variant
that reads its regex from `@file`) are in `C:\Projects\Fulcrum\tools\ghidra\max`.

Second-pass dumps: `catg.txt` (CAT graph vtables incl. slot 251 key builders), `ee.txt` (CAT key
evaluator), `kd.txt` + `knee.txt` (knee/spike detectors and fix), `fsrange.txt` + `dlg.txt` (mocap
parameter-block xrefs). Black-box MAXScript sweeps run with `3dsmaxbatch.exe` live in
`C:\re_work\max\tests` (`probe.ms`, `ballistic.ms`, `balance.ms`, `balance2.ms`, `contact.ms`,
`cdebug*.ms`) with their `.out` files; fitting code is `C:\re_work\max\tools\fit_ballistic.py`.

Labels: **OBSERVED** means it is in the code, strings or bytes. **INFERRED** means it is reasoned from
observed code. **MEASURED** means it was read back from a running 3ds Max 2027 (3dsmaxbatch) and fitted;
residuals are given. **UNKNOWN** means the evidence does not settle it.

Max time units: 4800 ticks per second, which is 160 ticks per frame at 30 fps (OBSERVED: constant 160.0 at
`1804ca6b8`/`1804ca6c0` and `25600.0 = 160^2` at `1804ca7b0`).

---

## 1. Biped footsteps and key types

### 1.1 Planted / sliding / free keys (OBSERVED strings, INFERRED semantics)
- The UI commands are `Set Planted Key`, `Set Sliding Key` and `Set Free Key`. Their MAXScript entry
  points are `setPlantedKey` (`FUN_18000b080`) and `setSlidingKey` (`FUN_18000b240`). Both are thin
  wrappers.
- Legality rules come from the `.mui` strings:
  - There must be at least one *move-state* leg key between a *liftoff* key and a *touchdown* key.
  - There must be at least one frame between two footsteps on the same side.
  - A footstep must be at least two frames long. No footsteps may sit in negative frames.
  - The double-support period must be less than half the footstep duration.
  - Hopping footsteps need at least one frame between them.
- Key state names: `Touch`, `Lift`, `Planted`, `Sliding`, `Free`, `Airborne`, `Walk/Run/Jump Footstep`.
- **INFERRED:** a footstep is an interval `[touch, lift]`. Inside it, a planted key pins the foot pivot in
  world space with IK Blend 1 and an object-space join. A sliding key keeps IK Blend 1 but lets the pivot
  translate. A free key releases the IK constraint (IK Blend 0).
- Per-type TCB defaults are read from the biped INI (`FUN_1803178d0`). `PlantedTension/Continuity/Bias`,
  `SlidingTension/Continuity/Bias` and `FreeTension` are stored at `180540c64..180540c7c`. All seven
  default to **25.0** (OBSERVED bytes).
  **INFERRED:** this is Max's 0..50 TCB UI scale, so 25 is the neutral Kochanek-Bartels value
  (`t = c = b = 0`). The conversion is `kb = (ui - 25) / 25`.

### 1.2 Footstep extraction from mocap (OBSERVED)
- The MocapManager globals are `fsExtractionTol @ 180615e28`, `fsVerticalTol @ 180615e24`,
  `fsSlidingAngle @ 180616254` and `fsSlidingDist @ 180616258`. Their exported getters are at
  `1802b8430..1802b8490`.
- The MAXScript setter for `fsSlidingAngle` (`FUN_1802a7c30`) range-checks the value against
  `DAT_1804659e0`. The `fsSlidingDist` setter requires the value to be at least 0.
- `FUN_180093ce0` is "Convert to footsteps". It sets the sliding thresholds from the INI ints
  `SlidingAngle` and `SlidingDist`, read as int **360** and int **100** (OBSERVED bytes `180541634/638`).
  It then runs this pipeline:
  `FUN_1802f4a70` → `FUN_1802c93e0` → **`FUN_1802f6090` (ballistic launch velocities)** →
  `FUN_18033dd10` → `FUN_180341780` → `FUN_1802c05d0` → `FUN_1802c07a0` →
  **`FUN_1802d3fa0` (per-footstep COM height normalisation)**.
- **Contact test: decoded (OBSERVED, third pass), `FUN_18010bea0 @ 18010bea0`.**
  - **How it was found (writers, not readers).** The "globals" are fields of one object: the
    extraction engine `DAT_1806130e0` that the convert routine passes as `param_1`
    (`FUN_18010bea0(&DAT_1806130e0, biped, biped, mode)` from `FUN_180093ce0`). Its fields alias the
    MocapManager accessors exactly: `+0x2d48 = 180615e28 fsExtractionTol`, `+0x2d44 = 180615e24
    fsVerticalTol`, `+0x2d40 = 180615e20 fsZLevel`, `+0x2d10 = 180615df0 fsUseVerticalTol`,
    `+0x2cec = 180615dcc` (conversion flag, see below), `+0x3174 = 180616254 fsSlidingAngle`,
    `+0x3178 = 180616258 fsSlidingDist`, `+0x2818 → frame block (+0x50 frame count)`. The engine only
    reads them as `[param_1 + off]`, so no direct xref to the global exists; a raw RIP-relative byte scan
    of `.text` (`C:\re_work\max\tools\ripscan.py`) confirms the only direct users are the accessors.
  - `SetFSConversionMode` (`1802bab70`): mode 0 → `dc4 = dcc = 0`; mode 1 → `dcc = 1`; mode 2 →
    `dc4 = 1`. `GetFSConversionMode` returns 1 if `dcc`, else 2 if `dc4`, else 0.
  - **Defaults (MEASURED, `mocapdef.ms`, 3dsmaxbatch):** `fsExtractionTol 0.07`, `fsVerticalTol 0.5`,
    `fsZLevel 0`, `fsUseVerticalTol false`, `fsSlidingDist 25`, `fsSlidingAngle 0`,
    `fsConversionMode 0`, `fsExtractionMode 0`, `upVector 2`.
  - Biped's internal frame is **y-up** here: horizontal = (x, z), height = y (OBSERVED from the
    component offsets the engine uses).
  - `S = leg[+0x644]`, with `leg = FUN_1802e8d00(biped, 4) = biped+0xff28`. **INFERRED:** a leg-length
    scale, so the tolerances are body-relative. Foot lengths are `leg[+0x1da4]` (left) and
    `leg[+0x1da0]` (right) (INFERRED from use).

  **Rule A: mocap extraction (`mode` argument 0).** Per foot, per frame `i ≥ 1`, with foot position `p`:
  ```
  planted[i] = planted[i-1] = 1      if |p_i - p_{i-1}| <= fsExtractionTol * S      (3-D, per frame)
  if fsUseVerticalTol: planted[i] = 0  if |p_i.y - fsZLevel| > fsVerticalTol * S
  1,0,1     -> 1,1,1                  (fill 1-frame gaps)
  1,0,0,1   -> 1,1,1,1                (fill 2-frame gaps)
  0,x,x,x,0 -> 0,0,0,0,0              (drop contacts of 3 frames or fewer)
  planted[0] = 1 if planted[1] == 1
  ```
  Each maximal run `[s, e]` becomes one footstep (`FUN_180346060`, side, start `s`, duration
  `e - s + 1` frames). The footprint TM is the foot matrix at the **middle frame** `(s+e)/2`. Its height
  is the **maximum** foot height over the run; the initial value is `-1e6` (`180465a14`).

  **Slide flag per run (both rules).** Take the foot heading `ψ_i = ±acos(row.y)` (sign from `row.x`)
  and the horizontal distance `d_i` from frame `s`. The run is flagged (bit in `engine+0x68`) if any frame
  meets either test:
  ```
  |wrap(ψ_i - ψ_s)| > fsSlidingAngle * π / 180
  d_i               > footLength * fsSlidingDist / 50
  ```
  In mocap mode with `dcc == 0` (conversion mode ≠ 1), the flag is set for every run. **INFERRED:** mode 1
  is the only mode that tests sliding, and the flag marks a run that keeps its sliding motion. Because
  `fsSlidingAngle` defaults to 0, any heading change counts. With `fsSlidingDist = 25` the distance limit is
  half a foot length.

  **Rule B: Convert-to-footsteps from freeform (`mode` argument 1, `FUN_180093ce0`).** No geometry is
  involved. Per foot:
  - A frame is planted when the leg's IK blend is above **0.99** (`1804658f4`) and the IK space state
    equals 2 (left `+0xf4cc/+0xf4d4`, right `+0x1213c/+0x12144`).
  - The last planted frame before a lift is dropped (`planted[i-1] = 0` when `planted[i] == 0`).
  - Trailing unplanted frames at the clip end are back-filled on the other foot when needed.
  - The footprint takes the first frame's height.
  - The INI sliding thresholds 360 and 100 apply (`180541634/638`), so the angle test never fires and the
    distance limit is 2 foot lengths.
  This is why perturbing foot positions in `contact.ms` cannot change Convert-to-footsteps: it reads IK
  keys only.

- **Black-box status.**
  - 3dsmaxbatch loads mocap (`biped.loadMocapFile` on the synthetic CSM walk
    `C:\re_work\max\tests\csm\z0.csm` returns true and drives the biped). It does **not** create
    footsteps: mode 0 gives 200 freeform keys, mode 1 gives 22 vertical keys and 0 footsteps, mode 2
    loads nothing.
  - `convertToFootSteps` fails with "Cannot switch this mode" (MEASURED, reproducible). Footstep
    creation needs the Motion panel.
  - GUI 3ds Max does not start unattended. With `-silent`, Max.log says
    `License Error: lgs UI requires GUI mode` and the process exits (code -8). Without `-silent`, the
    window sits at licensing, not responding (0.2–0.3 s CPU, 140 MB, over 4–10 min on two attempts) until
    killed.
  - So Rules A and B are not yet MEASURED. A ready sweep is in place: `C:\re_work\max\tests\csmsweep.ms`
    covers fsConversionMode 1/2 × fsExtractionTol 0.02/0.07/0.2 × 13 CSMs. The CSMs are `z*` (left-foot
    lift bumps 0–80 mm during the third stance) and `y*` (left-foot slide 10–400 mm). The generator is
    `C:\re_work\max\tools\gencsm.py`. The sweep writes `csmsweep.csv` and calls `quitMax #noPrompt`.
    Run it in a signed-in GUI session:
    `"C:\Program Files\Autodesk\3ds Max 2027\3dsmax.exe" -U MAXScript C:\re_work\max\tests\csmsweep.ms`.
    It should reproduce Rule A: a z-bump above `0.07·S` per frame splits the third left stance, and a
    slide above half a foot length sets the slide flag.

### 1.3 Mocap cleanup filters in biped (OBSERVED class names, partial code)
The RTTI names are `KneeExtensionDetector`, `KneeExtensionFilter`, `KneeWobbleDetector`,
`KneeWobbleFilter`, `SpikeDetector`, `AdaptiveGaussianQuatFilter`, `NonLinearQuatFilter`,
`BasicPosFilter`, `BasicQuatFilter` and `KeyReducerFilter`. The vtables start at `1804cc030..1804cf7a0`.
The MAXScript names are `DoKneeExtensionAnalysis/Fix/Filter` and `DoKneeWobbleAnalysis/Fix/Filter`, and
`mixdown` takes `FilterHyperExtLegs` and `MaxKneeAngle`.

- **AdaptiveGaussianQuatFilter::Apply `FUN_18038a850` (OBSERVED):**
  1. Take a window of size `N` (odd) and half-width `h = N/2`. Indices past either end are mirrored:
     `i<0 → -i` and `i≥n → 2n-i-2`.
  2. Sort the window by `|v_i|^2`, where `v` is the 3-vector part of the per-frame sample. Take the
     **median** sample `m`.
  3. Weight each sample as `w_i = g_i / (|v_i - m| + 1)`, where `g_i` is the stored Gaussian kernel
     (`param_1+8 → +0x10`). Normalise with `w_i /= Σw`.
  4. Output `Σ w_i v_i`.
  This is a median-anchored, outlier-damped Gaussian. Spikes far from the median get weight ≈ g/dist.
- `KneeWobbleFilter` slots 6 and 7 (`FUN_1803868a0`, `FUN_180386c20`) take a per-instance angle in
  degrees, `param_1+8`, multiply it by `0.0174533` (OBSERVED `18044ef70`) and pass it to `FUN_18002e9d0`
  over the detected frame ranges. **INFERRED:** this clamps or smooths the knee swivel angle across each
  wobble range.
- **Knee criteria (OBSERVED, second pass).** Per-frame leg records are 0x478 bytes at
  `legTrack+0x2ac8`; field `+0x00` is the knee angle (radians) and `+0x80` is IK blend (copied as
  `param_1[0x20]` in the record copier `FUN_180321e90`). Detectors run only on leg tracks
  (`+0x68 == 4/5`, `+0x6c == 1`, record bases `+0xd2b8` / `+0xff28`).
  - **KneeExtensionDetector** (`FUN_18034b3f0/b500` → `FUN_18002e690`): flag frame `i` when
    `knee_i > θmax` and `ikBlend_i == 1.0`, with `θmax = param_deg * 0.0174533`.
  - **KneeWobbleDetector** (`FUN_18034b5e0/b720` → `FUN_18002e7a0` → `FUN_18002ece0`): with
    `Δ_i = knee_{i+1} - knee_i`, a sign reversal `Δ_i·Δ_{i-1} < 0` with `Δ_i > θ` is a wobble event; two
    events closer than `W = frames * ticksPerFrame` (`param_1[5]` frames) mark a wobble range ending at the
    second event (frame index - 1).
  - **KneeExtensionFix** (`FUN_18002e9d0`): for each flagged frame with `ikBlend > 0.99`
    (`180454420`), re-run leg IK with the knee limited to `θmax` (`FUN_180325610`), insert a key, then
    smooth the window `[i-2, i+2]` (`FUN_180328410`, `FUN_1803009b0`) and finally the whole track
    (`FUN_180328530`, `FUN_180300db0`).
  - **SpikeDetector** (`FUN_18034cf10`): for each key `i` with neighbours on the sample grid, compute
    `w0 = log(q_{i-1}⁻¹ q_i)` and `w1 = log(q_i⁻¹ q_{i+1})`; it is a spike when `w0·w1 ≤ 0` (angular
    velocity reverses across one sample).

---

## 2. Biped dynamics: gravAccel, ballistic tension, dynamics blend, balance

### 2.1 gravAccel storage and units (OBSERVED)
- The MAXScript property `gravAccel` has getter `FUN_180288bb0` and setter `FUN_18028fd50`. The setter
  rejects values below `0.01` (string `"0.01f"`). It calls IBipMaster slot `0xd8`, which is
  `FUN_1802bae60`.
- `FUN_1802bae60` stores `g_int = gravAccel / 900.0` (OBSERVED constant `180454370 = 900.0`) at
  `master+0x2e844` and mirrors it to `+0x34a24`. The getter `FUN_180020890` returns `g_int * 900`.
  **INFERRED:** `g_int` is in units/frame² at 30 fps (900 = 30²), so `gravAccel` is in units/s².
- **Default gravity, `FUN_1802e30c0` (OBSERVED):**
  `g_int = H / (n*15 + 200 + 200) * 2.6`. Here `H = FUN_1802c4970()`, which is **INFERRED** to be the
  biped height. `n = FUN_1802e9f10(this, 0x10)` is UNKNOWN; it is probably a step or frame count.
- **MEASURED (`probe.ms`):** biped heights 99.2771 / 168.771 / 248.193 give gravAccel 559.779 /
  951.625 / 1399.45, i.e. `gravAccel = 5.63857·H` for all three (max relative residual 2e-5). With the
  formula above this fixes `n = 1`, so the default is `g = H·2.6·900/415` (units/s²).
- **Height rescale, `FUN_1802da420` (OBSERVED):** when the figure is resized,
  `g_int = (g_prev / H_prev) * H_new`. Gravity scales linearly with body height. Keep this when
  retargeting.

### 2.2 Airborne launch and landing speeds (OBSERVED `FUN_1802f6090`)
The routine walks the footstep table (`master+0x347e0`, stride 0x54). An airborne period is a run where
both feet have fewer than 2 contacts (`+0x28 < 2 || +0x2c < 2`). For each airborne period `k`, with
duration `T` in ticks and COM rise `Δz = z_land - z_lift` taken from footstep field `+0x40`, it computes:

```
g_t   = g_int / 25600            (per tick^2)
v_up  = max( (T - (0.5*T - Δz/(T*g_t))) * g_t , 0.000625 )   = g_t*T/2 + Δz/T
v_dn  = max( (0.5*T - Δz/(T*g_t)) * g_t , 0.000625 )         = g_t*T/2 - Δz/T
```

`v_up` and `v_dn` are stored in the airborne record at `+0x18` and `+0x1c`. The floor is
`0x3a23d70a = 0.000625`. **INFERRED:** this is the closed form of `z(t) = z0 + v_up t - ½ g t²` with
`z(T) = z0 + Δz`.

### 2.3 Vertical COM evaluation and the ballistic transition curves (OBSERVED `FUN_1802f6420`, `FUN_1802f7260`, `FUN_1802f7040`, `FUN_1802f6e20`)
- `FUN_1802f6420(t)` classifies `t` against each airborne record (`master+0x34b30`, stride 0x20, fields
  `+4 liftoff, +0x10 touchdown, +8/+0x14 bounding footstep indices`). It writes a phase code to
  `master+0x349d0`:
  - `0` means exactly at liftoff.
  - `1` means airborne.
  - `3` means exactly at touchdown.
  - `2` means a **transition window** before liftoff (loading) or after touchdown (absorption).
- The window length is the adjacent support duration. When two airborne periods share a footstep, it is
  `floor(support * 0.5)` (OBSERVED `18045e3f0 = 0.5`). A window is used only if it is longer than
  2 frames (`2*ticksPerFrame < len`).
- In a transition window the vertical COM is a **cubic Bézier** (`FUN_1802f7260`, de Casteljau):
  ```
  C1 = P0 - v0*h0 ;  C2 = P3 - v1*h1
  B(s) = (1-s)^3 P0 + 3(1-s)^2 s C1 + 3(1-s) s^2 C2 + s^3 P3
  ```
  - `P0` and `P3` are the support height and the liftoff/landing height.
  - `v0` and `v1` are `v_up*160` or `v_dn*160`, converted to per-frame.
  - `h0` and `h1` are handle lengths.
- The handle lengths are found by **bisection** (`FUN_1802f7040` / `FUN_1802f6e20`). The search range
  starts at `[0, 3]` or `[3, 20]` (OBSERVED `3.0` and `1804659dc = 20.0`). It stops when the curve's
  extreme dip or rise matches the target displacement within **0.2** units (OBSERVED `18045ef18`).
- **Ballistic tension (OBSERVED use):** after the solve, each handle is scaled by `(1 - tension)`, where
  `tension` is the bounding footstep record's `+0x4c` field:
  `h_final = (1 - ballisticTension_at_that_footstep) * h_solved`. Tension 1 makes the transition a
  straight hold (no squash). Tension 0 gives the full momentum-matching squash.
  **INFERRED:** the key accessor `FUN_180265f80` shows `ballisticTension` lives on vertical-track keys
  (type `0xc`) at key offset `+0x28`. The key is copied into the footstep record `+0x4c`.
- **MEASURED (`ballistic.ms`, `fit_ballistic.py`):** an 8-step run with 10-frame contacts every
  15 frames (5-frame flights). A quadratic fitted to each interior flight gives `z'' = -gravAccel/900`
  per frame² exactly: -1.0574 (g 951.625), -0.5555 (g 500), -2.2222 (g 2000); std 1e-5, max fit residual
  1e-4 units. Support dip below liftoff height vs ballisticTension: τ=0 → -6.761, τ=0.5 → -3.251,
  τ=1 → +0.059 (flat). The dip is close to linear in `(1-τ)` (0.5 predicts -3.38, measured -3.25, 4 %).
  Dip scales with g at fixed τ=0.5: -1.673 (500), -3.251 (951.6), -7.200 (2000), i.e. ∝ g^1.05.
- The weight returned in `*param_3`, used in the blend below, is 1 when the curve applies and 0 outside
  every airborne or transition window.

### 2.4 Dynamics blend (OBSERVED `FUN_1802c6490`)
```
zb   = ballistic_or_bezier(t)                      FUN_1802f6420, weight w
zmix = (zb - comOffset)*w + (1 - w)*zSpline        comOffset = master+0x349d8
if dynamicsBlend < 1:  z = (1 - dynamicsBlend)*zKey + dynamicsBlend*zmix
```
- `dynamicsBlend` is read from `master+0x34840` here. Its per-key copy sits at key `+0x24`
  (OBSERVED in `FUN_180265f80`).
- `zKey` is the keyed or spline COM height (`+0x34828`).
- `dynamicsType` switches between Spline Dynamics and Biped Dynamics.

### 2.5 Balance factor (OBSERVED storage, INFERRED behaviour)
- `balanceFactor` is a key property. The accessor (`FUN_180265f80`) checks the key type before reading
  `+0x28`. **INFERRED:** it lives on the horizontal COM track.
- `balanceLateralRatio`, `balancePropagation` and `balanceMute` are mixer or layer properties
  (`FUN_1802802e0`).
- **MEASURED (`balance.ms`, `balance2.ms`):** keys expose `balanceFactor` on `#horizontal` keys
  (default 1.0) next to `x`, `y`; vertical keys carry `z`, `dynamicsBlend` (default 1.0) and
  `ballisticTension` (default 0.5). Sweeping balanceFactor `bf ∈ {0, 0.5, 1, 1.5, 2}` on a 6-step walk:
  - the COM (biped root) trajectory is **unchanged** (Δ = 0 at every frame);
  - pelvis and head move **together** (same offset to 1e-4), so bf translates the whole body relative to
    the COM along the travel axis;
  - per frame the offset is exactly linear: `body(t,bf) = body(t,0) + bf·s(t)` (max residual 4e-5);
  - `s(t)` oscillates with the stride (period 30 frames, amplitude ≈ 0.74 units for a 170-unit biped)
    and correlates 0.94 with the COM horizontal acceleration one frame later
    (`s ≈ 1.84·a_com(t+1)`, a in units/frame², rms residual 0.28).
  **INFERRED:** bf scales a per-frame body counter-shift that leans the body against COM acceleration
  (an inverted-pendulum style balance term); bf = 0 removes the shift and leaves the pelvis riding the
  COM trajectory. The exact `s(t)` law (beyond the 0.94 acceleration fit) is UNKNOWN.
- `FUN_1802d3fa0` does a per-footstep normalisation (OBSERVED):
  - footstep `+0x40` = `comOffset + keyZ`
  - `+0x44` = `(zKey - zLegMin) / (zKeyNow - zLegMin)`
  - `+0x48` = `(zKey - zLegMax) / (legLen + pelvisH)`
  These are the COM height ratios relative to the leg-reach envelope that feed `Δz` above.

---

## 3. CAT (CATMotion, limbs, foot platforms)

### 3.1 CATMotion cycle clock (OBSERVED)
- One CATMotion cycle is **19200 units** (`0x4b00`, `1801a7d74 = 19200.0`). Quarter marks fall at
  `4800` and `14400` (`1801a7d54`, `1801a7d70`). Every graph wraps its time with `t % 19200`. The
  `* 0x1b4e81b5 >> 0xb` idiom is the compiler's division by 19200.
- **Step-phase lookup, `FUN_180102850` (OBSERVED):**
  `stepIdx = floor(u / 19200)`, where `u` is the CATMotion hub's warped time (`+0x7a8` evaluate). Each
  limb has a phase offset param 5 that is clamped to `[-0.5, 0.5]` and scaled by 19200. The step window
  is `[idx*19200 + off_i, (idx+1)*19200 + off_{i+1}]`. When `u` falls outside the window, the index
  steps forward or back by one. The routine caches the previous, current and next step boundaries in
  `+0x68..+0x7c`.
- **Step progress and plant weight, `FUN_180102cb0` (OBSERVED):**
  - `p = (t - start) / (end - start + 1)` is stored at `+0x84`.
  - The output time interpolates the two boundary ranges by `p`.
  - The plant weight `+0x5c` is `1 - p`, `p`, 0 or 1. Which one depends on whether the previous,
    current and next stride lengths (`hub+0x7a8` evaluated at the boundaries) exceed
    `maxStepLength` (`+0xf0`).
  - **INFERRED:** when a stride is too long for the leg, the foot fades out of the plant over that step.

### 3.2 LiftPlantMod (OBSERVED `FUN_1801216f0`)
`LiftPlantRatio` `r` is clamped to `[0.02, 0.98]`. With `a = ((1 - r)*50 - 25)*192` (ticks, range ±4800)
and `u = t mod 19200`:
```
if u < a + 4800             : v = (u / (a + 4800)) * (-a)
elif u < 14400 - a          : v = ((u - (a + 4800)) / (4800 - a) - 1) * a
else                         : v = a - ((u - (14400 - a)) / (a + 4800)) * a
```
This is a piecewise-linear time-warp offset. It moves the lift/plant split inside the cycle without
changing the cycle length.

### 3.3 Graph controllers (OBSERVED `FUN_18011e600` FootLift, `FUN_18011fd00` KneeAngle, `FUN_180123f10` PivotPos, `FUN_1800eea60` StepShape / generic)
- They share one pattern. Get the step-local time and plant weight `w` from `FUN_180102cb0`. Evaluate
  the user graph `G` (`vfunc +0x7d8`) at `t mod 19200`. Then combine:
  - FootLift: `out = G * w * scale`.
  - KneeAngle: `out = (G*scale - rest)*w + rest`, with `rest = param 0xb` in degrees × `0.0174533`.
  - PivotPos: `w' = 1 - (1 - w)^2` (ease-out), then `out = (G*scale - rest)*w' + rest`.

### 3.4 CATMotionPlatform (foot platform) GetValue (OBSERVED `FUN_180104bb0`)
1. Get the hub transform `M`. Scale its translation by `CATUnits` (`+0xf0`). Zero the axis-up
   component (`iVar3`, the up-axis index).
2. **Tilt removal:** `θ = acos(min(dot(M.row_up, worldUp), 1))`. If `θ > 1e-4`, rotate `M` about
   `cross(row_up, worldUp)` by `θ`. This keeps the platform plane level with the ground.
3. Get the step window and the time warp:
   `t' = t + (stepTime - (t mod 19200)) * (|Δstep| / 19200)`.
4. Then branch on mode (`param 0x18`):
   - **0, procedural:** translate by `(sin φ, cos φ)·L·(t - t')/stepLen`, with `φ` the heading in
     radians (degrees × `0.0174533`).
   - **1, footprint:** take the footprint TM at `t'`.
   - **2, blended:** blend the footprint matrices of step `i+1` and step `i+2` (`FUN_180106160`), using
     the blend weight `param 5` through the matrix-blend `FUN_1800355d0`.
5. Add the lift offset (`param 3`), plus `w * param 2` lift when `w > 0`, all scaled by CATUnits and
   by `±limbSide` on the lateral axis.
6. Pre-rotate by the Euler roll and pitch (`param 4`) times `w`.

### 3.5 What was not reached
- **CATGraph key evaluation (OBSERVED, second pass, `FUN_1800ee180`).** Graph controllers build key
  pairs in slot 251 (e.g. FootLift `FUN_18011e250`, MonoGraph `FUN_1801225c0`). A key is
  `{t, v, slope, outLen, inLen}` with `t` in cycle units (keys at 4800 / 14400 = `0x45960000` /
  `0x46610000`), tangent lengths clamped to `[0, 1]`, and slope converted to an angle
  `a = atan(slope / frameRate)`. The segment `A → B` is a **2-D cubic Bézier in (time, value)**:
  ```
  Δt = B.t - A.t
  P0 = (A.t, A.v)
  P1 = P0 + (Δt·A.outLen/cos aA)·(cos aA, sin aA)
  P2 = P3 - (Δt·B.inLen /cos aB)·(cos aB, sin aB)     (if inLen < 0: length = Δt/3/cos aB)
  P3 = (B.t, B.v)
  ```
  For query time `u` it solves `x(s) = u` with a closed-form Cardano cubic (`FUN_1800edc30`: one real
  root via `cbrt(√D ± q)`, three via `2√(-p)·cos(acos(...)/3 + k·2π/3)`), keeps the largest root in
  `[0, 1]` (`1801a7bf0 = 1.0`), and returns `y(s)` (Bernstein form with factor 3, `1801a7c20`).
  Endpoint shortcuts: `u == A.t → s = 0`, `u == B.t → s = 1`; a degenerate cubic falls back to
  `s = (u - A.t)/Δt`. FootLift only evaluates inside `u ∈ [4800, 14400]` (`u - 4800 < 9601`).
- **UNKNOWN:** the CAT limb IK body in `LimbData2` (slots 161/167, `FUN_1800c26e0`, `FUN_1800be2e0`,
  decompiled in `CAT.a.txt`). They mix FK and IK by `IKFKRatio` and use `SetupSwivel`/`SwivelValues`.
  A full two-bone read was not done.

---

## 4. IK solvers

### 4.1 IK Limb solver (analytic two-bone), `IKLimb.dlc FUN_180002150` (OBSERVED)
Inputs: `L1 = this+0x1c` and `L2 = this+0x20` are the bone lengths. `Rmax = this+0x24` and
`Rmin = |this+0x28|` are the reach limits. The goal is `G` in root space.

```
d    = |G| ; dir = G/|G|
d    = clamp(d, Rmin, Rmax)           (reach clamp, else uses dir*limit)
cosK = (L1² + L2² - d²) / (2 L1 L2)   knee interior angle  (acos clamped to [-1,1])
cosH = (L1² + d²  - L2²) / (2 d L1)   hip offset from goal line
```
Swivel plane normal `n`:
- If a pole target is enabled (`vfunc +0x50`), project `pole` onto the plane ⟂ `dir` and normalise:
  `n = pole - dir·(pole·dir)`.
- Otherwise, if the swivel angle type is 1, rotate the solver plane about `dir` by `swivel`
  (a quaternion with half-angle `0.5*swivel`, sign-flipped).
- Otherwise, start from the parent-space start plane and rotate it about `dir` by `swivel`.
- If the projection degenerates (length 0), fall back to the default plane vector.

The signed swivel is then
`φ = acos(n·n0)`, flipped to `2π - φ` when `cross(n0, n)·dir < 0`.

The hip rotation applies `RotAngleAxis(dir, φ)` and then the hip angle about the plane normal. The knee
gets one rotation about its local axis. Both are written back as Euler XYZ (`MatrixToEuler`).

### 4.2 HI solver (history-independent), `HISolver.dlc` (OBSERVED)
- **The iterative core is `FUN_180004ac0`.** It is projected gradient descent with an active set:
  - `FUN_180004980` computes the gradient `g = 2 Jᵀ e`, written as `(J+J)*e` per column.
  - `FUN_1800057a0` partitions the joints into free, at-lower-limit and at-upper-limit
    sets with epsilon `1.19e-6` (`18000aac4`).
  - The arguments are the threshold (`param_4`) and the max iterations (`param_5`).
  - The convergence test is `|err_new - err_old| <= threshold`. On failure the step sign is flipped.
  - The step scale starts at `1.0` (`18000aae8`).
  - The start-position error is weighted by `100 * pos²` (`18000ab3c = 100.0`).
- **Root alignment after solve, `FUN_180003aa0`:**
  1. Build the start quaternion from Euler XYZ.
  2. **Swing:** rotate the chain axis `a` (the end effector relative to the root) onto the goal
     direction `G`. Use axis `cross(a, G)` and angle `acos(a·G / |G|)`, with the half-angle quaternion
     `(axis * -sin(θ/2), cos(θ/2))`. A parallel or antiparallel pair falls back to the solver plane
     normal (θ = 0 or π).
  3. **Swivel:** project the plane normal onto the plane ⟂ the goal axis: `n' = n - ê(n·ê)`. Then get
     the reference from one of three sources: the pole (`vfunc +0x60`), a swivel-angle rotation of the
     start plane (`vfunc +0x58`), or the parent-space plane. The rest matches IK Limb:
     `φ = acos(n'·r)`, flipped to `2π - φ` by the sign of `cross(n', r)·ê`. Rotate about `ê` by `φ`.
  4. Convert with `ContinuousQuatToEuler` against the previous Euler (`param_2+0x48`) so the curves do
     not flip.

---

## Port to Fulcrum (ranked)

These are ordered for the sprint → plant → slide → kneel → rise capture.

1. **Two-bone leg IK with clamped reach and signed pole swivel (§4.1).** This is the base of plant,
   slide and kneel.
   - `d = clamp(|G|, |L1-L2|+ε, (L1+L2)·0.999)`.
   - `knee = π - acos((L1²+L2²-d²)/(2L1L2))`.
   - `hipOff = acos((L1²+d²-L2²)/(2dL1))`.
   - `n = normalize(pole - ê(pole·ê))`.
   - `φ = atan2(dot(cross(n0,n), ê), dot(n0,n))`. This is `acos` plus the sign test in one call.
   - Recompose the hip rotation as `R(ê, φ) · R(n×ê, hipOff) · aim(ê)`.
   - Use `ContinuousQuatToEuler`-style continuity (§4.2.4): choose the quaternion sign
     `q·q_prev ≥ 0` before export to Unreal.
2. **Ballistic COM through the sprint and plant (§2.2–2.3).**
   - For each flight phase of `T` frames: `v_up = gT/2 + Δz/T` and `v_dn = gT/2 - Δz/T`, both floored
     at a small ε.
   - `g = 9.81 m/s²` by default. When retargeting, scale g by body height, as in §2.1:
     `g' = g·H'/H`.
   - Outside flight, blend with the cubic Bézier `P0, P0 - v0·h0, P3 - v1·h1, P3` over the adjacent
     support window. Use `min(support, 0.5·support when shared)`, and only when the window is longer than
     2 frames.
   - Bisect `h` in `[0, 3]` until the curve's dip matches the captured squash within 0.2 cm.
   - Expose a per-contact **ballisticTension** `τ ∈ [0, 1]` (Biped default 0.5) and scale `h *= (1 - τ)`.
   - Regression target (MEASURED): flight `z'' = -g` exactly, and the support dip at τ = 0 / 0.5 / 1 is
     -6.76 / -3.25 / +0.06 units for a 170-unit body with g = 951.6 units/s², 5-frame flights.
3. **Dynamics blend for COM height (§2.4).**
   `z = lerp(zCapture, w·(zBallistic - offset) + (1 - w)·zSpline, dynamicsBlend)`, with
   `w ∈ {0, 1}` from the phase classifier. This gives one slider from raw mocap to physical.
4. **Contact classification with planted / sliding / free states (§1.1).**
   - Per foot: `planted` pins the world pivot (IK weight 1), `sliding` keeps IK 1 but integrates pivot
     velocity, and `free` sets IK 0.
   - Enforce the Biped legality rules: at least 2 frames per contact, at least 1 frame between contacts
     on the same side, and double support shorter than ½ the step.
   - Map Biped's TCB defaults: `25 → KB (0,0,0)`.
   - The slide phase of the capture should be tagged `sliding` so the foot follows the floor and is not
     pinned.
   - Detect contacts with Biped's Rule A (§1.2):
     - per foot, `planted[i] = planted[i-1] = 1` when the per-frame displacement is at most
       `0.07·legLength`;
     - optionally gate on `|y - floor| ≤ 0.5·legLength`;
     - close 1- and 2-frame gaps, then delete runs of 3 frames or fewer;
     - place the footprint at the run's middle frame, at the run's maximum height;
     - tag a run `sliding` when its heading drifts beyond `fsSlidingAngle` or its horizontal travel
       from the first frame exceeds `footLength·fsSlidingDist/50`. Biped's default is half a foot length.
       Use a small non-zero angle, about 10°, instead of Biped's 0.
   - For the kneel, the knee contact needs a separate channel. Rule A sees only the feet, so run it on
     the knee joint with the same thresholds.
5. **Median-anchored adaptive Gaussian smoothing for rotations (§1.3).**
   - Use the quaternion log `v = log(q)` per frame and mirror-pad the ends.
   - `m = median-by-|v|²` over the window.
   - `w_i = gauss_i / (|v_i - m| + 1)`, normalised. Output `exp(Σ w_i v_i)`.
   - It removes mocap spikes without the lag of a plain Gaussian. Apply it to the knee before the kneel
     and to the pelvis on rise.
6. **CAT-style plant weight easing (§3.1, §3.3).**
   - The step progress is `p = (t - start)/(end - start + 1)`.
   - Fade the plant out over the step when the stride exceeds max reach. The weight is `1 - p` on the
     outgoing step and `p` on the incoming one.
   - For pivot or offset channels use the ease-out `w' = 1 - (1 - w)²`.
   - Final channel: `out = rest + (target - rest)·w'`.
7. **Foot platform leveling (§3.4).**
   - `θ = acos(min(up_foot·up_world, 1))`. If `θ > 1e-4`, rotate about `up_foot × up_world` by `θ`.
   - Then apply the authored roll and pitch × plant weight.
   - Use this for the kneel: level the planted shin and toe platform to the floor before the IK solve.
8. **LiftPlantRatio time warp (§3.2).** This is optional, for re-timing the sprint.
   - Clamp `r` to `[0.02, 0.98]` and set `a = (25 - 50r)·(cycle/100)`.
   - The piecewise-linear offset shifts the lift/plant split inside a stride while keeping the stride
     duration.
9. **HI-style gradient polish (§4.2) for the spine and pelvis on rise.**
   - Iterate `θ -= α·2Jᵀe` with a joint-limit active set (ε 1.2e-6).
   - Stop at `|Δerr| ≤ threshold` or at the iteration limit, and flip the step on overshoot.
   - Finish with a swing-then-swivel root alignment.

5b. **Knee extension and wobble cleanup (§1.3, OBSERVED criteria).** Before the kneel and on rise:
   - extension: flag frames with `knee > θmax` while the foot IK weight is 1; re-solve that frame with
     the knee clamped to `θmax`, then smooth `[i-2, i+2]`;
   - wobble: `Δ_i = knee_{i+1} - knee_i`; an event is `Δ_i > θ` with `Δ_i·Δ_{i-1} < 0`; two events within
     `W` frames mark a wobble range to smooth;
   - spikes: `log(q_{i-1}⁻¹q_i)·log(q_i⁻¹q_{i+1}) ≤ 0` marks a one-sample reversal; feed those frames to
     the median-anchored filter of rank 5.
6b. **CAT curve keys for authored lift / knee / pivot graphs (§3.5).** Store keys as
   `{t, v, slope, outLen, inLen}` and evaluate a 2-D Bézier in (time, value) with
   `P1 = P0 + Δt·outLen·(1, slope/fps)`, `P2 = P3 - Δt·inLen·(1, slope/fps)`; invert `x(s) = u` with
   Cardano (largest root in [0, 1]). Same curve type works for Fulcrum's per-contact ease channels.
10. **Balance factor as a body counter-shift (§2.5, MEASURED).** Keep the COM path fixed and shift
   pelvis-and-above by `bf·s(t)` along travel, with `s(t) ≈ 1.84·a_com(t+1)` (frames², fitted rms 0.28
   units on a 170-unit biped). Use bf = 0 for the slide (body rides the COM) and bf ≈ 1 for the sprint.
   Treat the gain as a tunable until the exact law is decompiled.

Still open:
- MEASURED confirmation of the contact rule (§1.2). The rule is decoded statically. The 3dsmaxbatch sweep
  cannot create footsteps, and unattended GUI Max is stuck at licensing. Run `csmsweep.ms` in a signed-in
  GUI session to get the residuals.
- the exact balance law `s(t)` (§2.5).
