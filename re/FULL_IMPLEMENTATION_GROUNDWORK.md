# Fulcrum Cascadeur-style implementation groundwork

Updated 2026-10-16. Scope: the user's 145-item behavior checklist in .codex/attachments/986c954f-e643-4cd1-beaf-86a789d4cad6/Pasted text.txt.

## Current verified foundation

- Source take, editable layers, derived output, constraints and previews are separate data paths; project save/reopen and browser-local crash recovery exist. Project manifests now preserve the original source file, SHA-256 and clip index independently of edited shot ranges, including after cuts and retiming. Locally uploaded FBX sources are copied into Fulcrum's exports directory before import so projects can reopen them. Reopening rejects a same-name source whose bytes changed.
- Recovered MibVelocityLoop is actually run for pose-guided inbetweening. The recovered auto-posing TorchScript models now have a guarded preview path: a selected, actually moved rig joint supplies guidance; range-limited output is staged as an editable layer; explicit six-value snap and spine parameters are required. The default values shown are explicit neutral test values, not Cascadeur defaults.
- Take import now discovers every FBX animation clip, stores clips as separate shots, and resamples mixed clip rates to the active take rate. Range retiming is isolated in wire/timeline.js, maps source indices and dependent frame ranges, and replays from saved projects after cuts.
- Animation layers now accept explicit joint and named body-region masks. Pose-preview accept records a before/after state for undo. Direct gizmo edits, range retiming and pose bookmarks record history.
- The Checkpoint rig now exposes named body/finger controls, FK keying, and hand/foot two-bone IK with numeric targets, pole offsets, blend and undo. Switching FK/IK does not itself change a pose. Pose inference now builds frame-local guidance from the selected control and active locks/pins, restores guided points after inference, and rejects previews that worsen required contact failures. These are Fulcrum editing semantics, not a claim of native Cascadeur parity.
- Existing contacts, hand pins, weapon grip offsets, root motion, quality gates, spring secondary motion, guided previews and FBX checks remain wired as before.

These are Fulcrum implementations and adapters; the presence of a Cascadeur model or similarly named feature does not establish behavioral equivalence to Cascadeur.

## Remaining work (not represented as complete)

1. **Multi-character scene ownership.** S.body, S.take, rig map, locks, layers, pins and quality state are still single-character globals. A scene/entity identity layer and per-character evaluation state are needed before multi-character projects or independent prop rigs are safe.
2. **Pose model semantic validation.** The serialized model tensor API and inference are verified. Cascadeur's caller defaults, nodes_s mask meaning, plane conventions in editor use, prediction-to-rig reconstruction, and quality across representative Checkpoint poses are not verified. The current preview is an explicitly parameterized experiment, not a validated production posing tool.
3. **Inbetween semantics.** Model shape, canonical 43-point order, local range and CPU execution are verified. Numeric style IDs have no recovered user-facing names; no claim is made that the model reproduces Cascadeur's native constraints, collision handling or trajectory controls.
4. **Physics/contact equivalence.** Fulcrum has contact detection, pins, floor correction, momentum/ballistic tools and measurable quality checks. A unified collision scene, authored timed contact-state graph, contact conflict priorities and a reconstruction of Cascadeur AutoPhysics are absent.
5. **Rig controls.** Named Checkpoint body/finger controls, FK keys, two-bone hand/foot IK, numeric pole offsets, blend, and non-destructive FK/IK mode switching are implemented. Viewport control shapes, animation-preserving IK/FK match/bake across ranges, mirroring, joint-limit authoring, and non-humanoid control characterization remain incomplete.
6. **Secondary motion.** Spring simulation is deterministic, bounded and tunable by joint/range. UI authoring of per-controller presets and secondary collision/baking workflows are incomplete.
7. **Reliability and proof.** No multi-character/long-clip performance benchmark, end-to-end UE import automation against the production Checkpoint project, or exhaustive malformed-FBX and all-frame visual acceptance set exists. Current FRT exports/tests validate only their recorded inputs.
8. **History coverage.** Preview accept/reject, retiming, bookmarks, direct gizmo edits and most rebuild-backed edits record snapshots. Scene/body/import changes and all settings are not yet routed through a single transactional history service; undo/redo coverage still needs a matrix test.

## Evidence ledger

| Claim | Evidence | Confidence / boundary |
|---|---|---|
| The 145-item checklist is a requested behavior inventory, not Cascadeur implementation evidence. | Pasted checklist at the scope path above. | Direct source text. |
| A real Cascadeur inbetween model and canonical representation are used by Fulcrum. | CASCADEUR_INBETWEEN_EVIDENCE.md; CASCADEUR_INTEGRATION_STATUS.md; tools/cascadeur_inbetween_backend.py; wire/cascadeur_rig.js. | Model identity, tensor call and exercised inference observed. Style semantics and native constraints remain unavailable. |
| A real posing model runs, but exact editor semantics are unresolved. | CASCADEUR_POSING_EVIDENCE.md; tools/cascadeur_pose_backend.py; synthetic CPU inference. | Serialized contract and finite output observed; native settings and Checkpoint reconstruction are not validated. |
| Retiming and layer masks have standalone regression coverage. | wire/timeline.js, tests/timeline.test.mjs, wire/animlayers.js, tests/integration_wire.test.mjs. | Source-level and unit-test evidence; broader browser edit/persistence matrix still required. |
| FRT source manifest survives a save/reopen cycle, and rejects changed bytes. | Browser tests on 2026-10-16: saved a fresh FRT project, reset the character, reopened it via the project picker; three shots, 356 frames and 24 fps returned. The saved manifest contained SHA-256 values for body and all three shots. A test manifest with a forged body hash was rejected visibly. Temporary test manifests were removed. | In-app verification for the single-clip-per-FBX FRT source only. Multi-clip, uploaded-file, cut and retimed project round trips still need explicit in-app tests. |
| Export round-trip and FRT quality results apply to the named reviewed export only. | CASCADEUR_INTEGRATION_STATUS.md, exports/FRT_fulcrum_cascadeur_review.validation.json. | Measured on that file/project, not a general exporter guarantee. |

## Completion gate

Do not label the checklist fully implemented until every numbered item has an implementation, an automated test, an in-app workflow test on representative rigs, and a stated acceptance threshold. For any Cascadeur-equivalence claim, also require recovered runtime evidence or clearly scope it as a Fulcrum-designed alternative. Unknown proprietary settings must remain disclosed unless new evidence resolves them.
