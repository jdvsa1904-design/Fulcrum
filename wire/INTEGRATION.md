# One-click FIX: wiring it into app.js

Four pure modules: no DOM, no fetch, browser and Node. Tests: `tests/fix_engine.test.mjs` (with `tests/fixrig.mjs`), `tests/ballistic.test.mjs`, `tests/fulcrum.test.mjs`.

| Module | Job |
|---|---|
| `wire/facts.js` | `computeFacts()`: one ground, one contact track (feet, hands, knees), phases, COM, balance, flights, seams, knees, trust. |
| `wire/judge.js` | `judgeMotion()`: one list of findings, one scalar `cost`, a fidelity penalty and `score = cost + fidelity.penalty`. |
| `wire/fixplan.js` | `planCandidates()`: rule to fix, phase-aware, with parameter variants. Also `applyChange`, `withoutFixIds` and `chainRootOffsets`. |
| `wire/fixengine.js` | `runFix()`: a stage-by-stage keep-or-revert loop. It is judge-agnostic, has no undo side effects and stops before export. |
| `wire/phaseadapter.js` | Loads `segmentPhases` from `phases.js`, or from `tastebridge.js` if `phases.js` is missing. |
| `wire/prior.js` | Library-learned motion prior: `loadPrior(json)`, `priorFindings()`. The model is `wire/motion_prior.json`, built by `tools/motionlib_prior.mjs` from the motion library. `judgeMotion({ ..., prior })` adds its `unnatural_*` findings; app.js fetches the JSON at startup and passes it. Tests: `tests/prior.test.mjs`. |

## 1. Inputs app.js already has

```js
import { computeFacts, rigIndices } from './wire/facts.js';
import { judgeMotion } from './wire/judge.js';
import { chainRootOffsets, withoutFixIds } from './wire/fixplan.js';
import { runFix } from './wire/fixengine.js';

S.rig = rigIndices(k => S.body.byKey.get(k));          // once per body (setBody)
const factsOf = W => computeFacts({ W, nb, n, fps: S.take.fps, upm: S.upm, rig: S.rig, restW: S.body.restW,
  shots: S.take.shots, trust: S.take.trust ?? null, ground: S.factsGround });
```

`restW` accepts THREE.Matrix4[] (it reads `.elements`), 16-arrays or a flat array. `trust` is optional: a per-frame 0..1 tracking confidence, for example from the mocap app. `fidelityGate` scales each frame's change by it, so a low-trust source frame is not treated as truth.

**`S.factsGround`**: compute it once, from the first `rebuild()` after a take loads, cut or insert: `factsOf(p1.W).ground` with `S.factsGround` unset. Store it in `projectJSON()`. During FIX the floor must not move with the fixes. The old loop judged against a floor that floor and ground fixes themselves moved, which is one reason AutoFix reverted good ground fixes. `facts.groundLegacy` keeps the old 3rd-percentile number for comparison. The new estimate takes the densest 2 cm band of slow-foot heights, so a long sink or float does not drag it.

## 2. Where it goes in `rebuild()` (app.js ~1214-1341)

1. **After `p1`** (`runPass(S.base, seamOff, {ik:false})`): `const F1 = factsOf(p1.W)`. It replaces:
   - `contactsFromWorld(p1.W)`: use `{ ground: F1.ground, per: F1.contacts.per, any: F1.contacts.feetDown, speed: F1.contacts.speed }`. These are the same 10 cm and 0.3 m/s rules on the shared ground.
   - `S.comRaw = comSeries(p1.W)`: use `F1.com`, built from the same mass table.
   - The plant detections in `stancePlants`, `detectPlantsNow`, `autoLocks`, `travelOffsets` (bounce) and `stridePlan`: use `F1.contacts.feet[s].plants`. Callers that used `minFrames: 2` pass `opts.plants`.
   - `detectFulcrums` in the Fulcrums button: use `F1.contacts.hands` / `F1.contacts.knees`.
   - `detectFlights(ct.any)` in the Ballistic button: use `F1.flights` (`plausible` holds the old airtime of at most 0.8 s and apex of at most 0.9 m test).

   The full old-to-new map is in `LEGACY_MAP` in facts.js.
2. **Root offsets: chain them, do not sum them.** Today `extra = combineOffsets(momentum, ballistic, kneeFloor, travel, join, ...fulcrum)` is all computed from `p1` and added up, so two solvers can both correct the same error. Replace it with:
   ```js
   const pass = acc => runPass(S.base, withYaw(acc), { layer: true, ik: false }).W;
   const producers = [
     seamFx && { name: 'seams', fn: W => /* seamPlan + seamRootOffsets on W */ },
     travels.length && { name: 'travel', fn: W => travelOffsets(W) }, joins.length && { name: 'join', fn: joinOffsets },
     flights.length && { name: 'ballistic', fn: W => ballisticOffsets(comSeries(W), flights, fps, g, 'endpoints') },
     moms.length && { name: 'momentum', fn: W => momentumOffsets(comSeries(W), factsOf(W).contacts.feetDown) },
     ...fulcrums.map(x => ({ name: 'fulcrum', fn: W => fulcrumOffsets(W, nb, x.idx, x.a, x.b, 'auto', 3) })),
     { name: 'kneeFloor', fn: W => floorOffsets(W, S.factsGround) },
   ].filter(Boolean);
   const { offsets, W: Wc } = chainRootOffsets({ n, producers, pass });   // order: fixplan.ROOT_ORDER
   ```
   Each producer sees the motion with every earlier producer already applied. The order is seams, travel, join, ballistic, momentum, fulcrum, knee floor; the ground drop and pull stay after IK planning as today. Each active producer costs one `runPass` without IK, so skip producers that have no fixes. `p2` becomes the last pass (`Wc`). Keep `offsets` as `S.rootOff`.
3. **After `p2`**: `const F2 = factsOf(p2.W)` replaces `ct2` for `groundPlan`, `levelPlan` and `pullPlan` and the floor fix (`ground = F2.ground`).
4. **New fix types** go in here (section 4).
5. **End of rebuild**: replace `S.contact / S.com / S.ground / inspectMotion / analyze() / gate()` with:
   ```js
   S.facts = factsOf(S.out.W);
   S.judgement = judgeMotion({ motion: S.out, source: { q: S.take.q, p: S.take.p }, nb, n, fps: S.take.fps, upm: S.upm, rig: S.rig,
     names: S.body.bones.map(b => b.name), parents: S.body.parentIdx, restW: S.body.restW, shots: S.take.shots,
     trust: S.take.trust ?? null, state: { fixes: S.fixes, locks: S.locks, pins: S.pins }, edited: editedSet(), facts: S.facts,
     ground: S.factsGround, opts: { mu: +$('mu').value || 0.8, snapLim: +$('snapLim').value || 900, gateLim: +$('gateLim').value || 8 } });
   S.flags = S.judgement.flags;  S.gate = S.judgement.fidelity;  S.motionQuality = S.judgement.quality;
   S.contact = { ground: S.facts.ground, per: S.facts.contacts.per, any: S.facts.contacts.feetDown, speed: S.facts.contacts.speed };
   S.com = S.facts.com;  S.ground = S.facts.ground;
   ```
   - `editedSet()` is the bones with `S.layer` keys or `S.animCompiled` entries; gate() left those out too.
   - `judgement.fidelity` has gate()'s shape (`rows`, `free`, `pass`, `worst`, `worstBone`, `pelvisCm`, `lim`) plus `rotPenalty` and `pelvisPenalty`.
   - `judgement.flags` is analyze()'s output plus `peak`, `side` and `bone`, with two deliberate differences. A foot slides only inside a Rule A plant: analyze() flags every low swing foot. A float flag next to a contact frame is ignored (`edgeGuard: 3`): the 5-tap smoothing flags the landing of a perfect arc. `opts: { slideContact: 'height', edgeGuard: 0 }` reproduces analyze() exactly.
   - The export gate checks `S.judgement.counts.blocker` instead of taste blockers.

## 3. The FIX button: `apply()`, `evaluate()`, one undo

```js
async function fixAll() {
  const before = editSnapshot();
  S.fixing = true;                       // rebuild() skips renderLists/showFrame/buildComPath/scheduleTaste/recordEdit
  const names = S.body.bones.map(b => b.name), src = { q: S.take.q, p: S.take.p };
  try {
    const r = await runFix({
      state: { fixes: S.fixes, locks: S.locks.map(({ side, a, b, required, fixId }) => ({ side, a, b, required, fixId })), pins: S.pins },
      apply: st => { S.fixes = structuredClone(st.fixes); S.locks = structuredClone(st.locks); S.pins = structuredClone(st.pins); rebuild(); return S.out; },
      evaluate: (st, out) => S.judgement,           // rebuild() already judged S.out
      log: line => { $('fixLog').textContent += line + '\n'; },
    });
    S.fixes = r.state.fixes; S.locks = r.state.locks; S.pins = r.state.pins; S.fixRun = r;
  } finally { S.fixing = false; }
  rebuild();                              // exactly one recordEdit(): FIX is one undo step back to `before`
}
```

- The rebuilds inside the run must not call `recordEdit()`. The old AutoFix wiped the 20-row undo history this way. The single `rebuild()` at the end records one row, so Undo returns to the state before FIX.
- `apply()` gets fresh copies on every trial. The app planners still write `g.side`, `g.anchor`, `fx.hits`, `fx.ref` and `lk.target` into fix objects; those writes land on copies and never reach the engine's state. In time they should move to side tables.
- `evaluate()` may be any judge that returns `{ findings, cost, fidelity? }`. Taste-style findings (`frame_start`, `body_part`, `warn`) are normalised by `normalizeFinding`, so an external judge can be plugged in.
- Caps (`runFix({ caps })`): `fidelityRise` (rotation penalty per kept change, default 15), `fidelityWorstDeg` (8, the gate), `pelvisRiseCm` (40), `maxTrials` (80). A trial is kept only when its score drops, its blocker count does not rise and it breaks no lock, pin or bone length. Otherwise it is reverted.

## 4. New fix types app.js must implement

1. **`{type:'knee', side, a, b, mode:'spike'|'wobble'|'extension', strength?, maxDeg?}`**. Put it in `buildBase()`, after `applyCleanup` and before `applyAdvancedMotion`.
   1. Take the hinge axis in thigh space from the take: the average of `cross(K-H, A-K)` in thigh-local space over frames with flex above 10 degrees, or from `restW` if the rest pose is bent.
   2. Take the signed flex angle about that axis per frame.
   3. Change the angle: `contacts.kneeWobbleFix(angle, [{s:a, e:b}], {strength})` for spike and wobble; `-kneeExtensionFix(-angle, null, maxDeg).knee` for extension.
   4. Write back `q_calf' = q_calf * axisAngle(hingeLocal, newAngle - angle)`.

   `tests/fixrig.mjs` `pipeline()` is a working reference with a local-x hinge. For gate(), the knee type drives that leg (`judge.drivenBones` does this). Label: "Knee pop removed, left f50".
2. **`{type:'ankleLock', side, a, b, perimeter (cm), maxSlide (m/s), floorProximity (cm), recovery (s)}`**. Put it beside `groundPlan`, on `p2.W`.
   1. Run `contacts.ankleLock(footTrack[a..b], { perimeter: cm*upm/100, floorProximity: cm*upm/100, maxSlide: ms*upm, fps, floor: F2.ground + level.foot (in units), rot: foot world quats })`.
   2. For each locked frame, push `{ side, target: anchor, q: foot world quat, w }` into `S.groundTargets`. Ramp 2 frames in at the run start.
   3. After the release frame, ease out with `recoveryBlend(n, release, fps, { recoveryTime: recovery })`.

   `maxSlide` above 0 gives the moving anchor for the drill slide; 0 gives a hard lock with a perimeter.
3. **`pins[]` with `hand:'r'`** (optional). `runPass` only pins the left arm. The planner emits right-hand fixes only as `fulcrum` today, so this is needed only to pin the right hand.
4. `despike` / `filter` with `bones: [indices]` already work through `wire/cleanup.js`. `fixLabel()` should say "on N bones". Contact-aware filtering arrives as several `filter` fixes: the upper body over the whole range, and each leg only between its plants and locks.
5. **`projectJSON()`**: keep `fixId` on locks and pins (it strips unknown fields today). Fixes keep it already.
6. **`fixLabel()`** today labels level, carry, lean, join and travel as "Smoothing". Give every type its own label; `fixengine.describeChange` has short texts for all of them.

## 5. Layers, toggles, report (UI)

- Every fix, lock and pin a kept change added carries `fixId: 'fix-N'`. `r.kept[i]` has `{ id, stage, rule, label, detail, finding: { text, a, b, side, phase }, before, after, rejected }`.
- **FIX layers list**: one row per kept item, with a checkbox, the plain label and the frame range (`data-sel`). To mute a row, move its items out with `withoutFixIds(state, [id])` into `S.parkedFixes`; to unmute, move them back; then `rebuild()`. Save parked items in the project with `mute: true`. Each toggle is one undo row.
- **Report**: `r.report` holds plain-English lines in order. The first line is "Start: ...". Then one line per kept or reverted change, then "Stopped before export ...", then "Still open (...): ..." for each remaining finding, with the reason when no auto fix exists (`fixplan.NO_AUTO_FIX`), then "Done: ...". Draw `r.flagged` (merged frame ranges with their severity and rules) on the timeline.
- **Layout**:
  - A big **FIX** button in the top tool shelf, a status line, and the report list under it.
  - The existing buttons move into a collapsed `<details>` **Advanced** panel: lock L/R, auto locks, ground, floor, level, roll, despike, filter, median, smooth, momentum, ballistic, fulcrums, seams, pull, pins, footsteps, cut/insert/story, and the old taste AutoFix.
  - FIX never exports. Export keeps its own button and gates on `S.judgement.counts.blocker`.

## 6. Stages (fixplan.STAGES)

| Stage | Rules handled |
|---|---|
| preflight | Notes only: missing bones, phase code missing, low-trust frames, floor far from the legacy floor. |
| noise | `pop` → despike, pad 2 or 4, or median. `knee_pop` → knee spike, or despike on the leg bones. `knee_wobble` → knee wobble. `jitter` → contact-aware filter at 8, 6 or 4 Hz. |
| structure | `seam_jump` → seams (blend 6 or 10, carry or local mode; retune `force` if a seams fix already exists). |
| body (root, chained) | `ground_float` in a plausible flight → ballistic. `overload` → momentum. `hand_slip` in a vault, plant or kneel → fulcrum on the hand. `knee_slip` in a kneel → fulcrum on the knee. External `stride_ratio`, `bounce` and stance width in gait phases → travel and footsteps parameter search (width, cycle, speed, bounce). |
| contacts | `foot_slide` / `foot_skate`: lock the plant, with variants a trimmed lock or an ankleLock with a 3 or 6 cm perimeter. In a slide phase, an ankleLock with a moving anchor instead. Nothing in flight, vault or prone. `ground_float` outside flight → ground per side. `ground_sink` → floor (ankle or toes pivot, stiffness 0 or 0.5, pad 3 or 6). `lock_residual` → ground instead of the lock, or a trimmed lock. `pin_residual` → pull. `hand_slip` (left, stand or rise) → world pin, plus pull in a rise. `knee_hyperextension` → knee extension. `foot_tilt` → level. External pitch and heel-raise rules → roll on locks. |
| polish | `pop` that is still there → smooth. `jitter` → filter at 6 or 4 Hz. External `crossing` and `freeze` → smooth or median. |
| stop | Nothing is exported. |

## 7. Judge rules (superset of `autofix.fulcrumFindings`)

| Rule | Severity | Source |
|---|---|---|
| `foot_slide` | blocker / warning | analyze slide inside a plant / Rule A sliding plant |
| `ground_float` | blocker | analyze float |
| `pop` | blocker | analyze snap |
| `overload` | warning | analyze overload |
| `ground_sink` | warning | sole under the floor by more than 2.5 cm, merged with the motionQuality ankle check |
| `lock_residual` / `pin_residual` | blocker | motionQuality constraint residual |
| `bone_stretch` | blocker | motionQuality |
| `nonfinite` | blocker | motionQuality |
| `prop_penetration` | warning | motionQuality |
| `seam_jump` | blocker | `seamPlan` jumpBefore greater than max(allowed, 1 cm) |
| `knee_pop` | blocker above 20 deg, else warning | `spikeFrames` plus a 12 deg/frame gate |
| `knee_wobble` | warning | `kneeWobbleRanges` |
| `knee_hyperextension` | warning | `kneeExtensionFlags` on planted legs |
| `foot_skate` | warning | plant drift over 4 cm; "forefoot" when the ball pivots |
| `jitter` | warning | window median of the third difference |
| `hand_slip` / `knee_slip` | warning | |
| `foot_tilt` | warning | needs `restW` |
| `balance` | info | |
| `unnatural_<joint>_speed` / `_snap` | warning; blocker when far past the band for 3+ frames | prior: joint angular speed, or a one-frame excursion from the neighbouring frames, outside the library's per-phase 99.5% band. Fix: despike/median on that joint (knee: knee spike first), smooth in polish |
| `unnatural_knee_hyperextension` | warning (blocker past 30 deg) | prior: knee bends backwards past 10 deg (fixed limit; the library knees never straighten, so there is no "must bend" rule). Fix: knee extension |
| `unnatural_frozen_limb` | warning | prior: a leg moves less than a third of the library minimum in a gait phase. Report only |
| `unnatural_plant_drift` / `unnatural_foot_slide` | warning | prior: plant drift or flat-sole speed past the library band. Fix: foot lock (as foot_skate) |
| `unnatural_<joint>_flexion` / `_extension`, `unnatural_pelvis_*`, `unnatural_stance_duty`, `unnatural_step_period` | warning | prior: pose, pelvis and gait timing outside the library. Report only |

The rules are scoped per phase: `PHASE_SCOPE` drops airborne foot rules in flight, vault and prone, and softens slides in a slide phase to info. `RULE_ALIASES` maps other judges' names to these.
