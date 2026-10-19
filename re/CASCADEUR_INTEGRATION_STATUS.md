# Cascadeur-style features in Fulcrum

Updated 2026-10-16.

## Implemented and exercised

- **Neural pose-guided inbetween preview:** Fulcrum now samples the selected take interval and its two neighboring context poses, builds the recovered Cascadeur 43-point representation, runs the genuine `MibVelocityLoop` TorchScript model on CPU, retargets the controls onto the Checkpoint skeleton, and stages the result as an editable animation layer. The control rig reconstructs helper points from the target skeleton's rest pose. This is a Fulcrum retarget adapter, not Cascadeur's native rig generator.
- **Preview and edit safety:** accept/reject, undo/redo, project recovery, range-scoped rebuild, root path preservation, foot locks, and existing hand/weapon pins are wired into the guided workflow. A live model preview changed no quaternion values outside frames 30–45; the quality analyzer found 0 required constraint failures and a 1.98 cm maximum residual on the exercised FRT project.
- **Pose assistance:** Fulcrum's take-local statistical pose suggestions remain available. The recovered Cascadeur posing model is also exposed through the MCP tensor endpoint, but is not automatically applied in the editor because Cascadeur's native snapping, spine, plane, mask defaults and target-rig reconstruction are not yet recovered.
- **Physics-aware cleanup:** Fulcrum's momentum, floor, contact, smoothing, and ballistic tools compose in a reversible preview; the quality gate measures transforms, balance support, ground penetration, and required contacts. This is not a reconstruction of Cascadeur's full physics solver.
- **Secondary motion:** an editable, bounded damped-spring pass is available with strength, damping, stiffness, frame rate, joint selection, pin preservation, and exact range boundaries. It is Fulcrum code, not a recovered Cascadeur binary solver.
- **Weapon-aware motion:** a weapon FBX can be attached to a hand driver; two-hand grip offsets persist in the project and are exported as animated FBX anchors. A proxy grip remains available without a weapon asset.

## Reverse-engineering evidence

- `C:\Program Files\Cascadeur\tool_inbetweening.dll` (SHA-256 `e11bdf5ef65eea97e44708af8aec0796881eab3f553b4dfb67eb7739b58f5f12`) contains three TorchScript archives. The velocity model's exact argument order, 43-point order, frame context and output span were recovered from the serialized model and corroborated by native initializer and skeleton-edge evidence. The actual velocity model was executed in Fulcrum; style label names remain unknown.
- `tool_auto_posing.dll` (SHA-256 `C8B95DD03677888824D65103B40C44A5D8B8FE228ED952686D82D3A42602ED1A`) contains real posing and auto-unlock models. CPU synthetic inference succeeded. Native caller settings/defaults and a reliable mapping from Cascadeur masks/outputs to this character remain unverified.
- `tool_auto_interpolation.dll` was imported/decompiled with Ghidra. That artifact primarily contained UI/resource registration; its interpolation solver was not recovered from that DLL. Shipped Python scripts did establish contact-aware interpolation selection behavior.

Detailed notes: `CASCADEUR_INBETWEEN_EVIDENCE.md`, `CASCADEUR_POSING_EVIDENCE.md`, and `FEATURE_CHECKLIST_AUDIT.md`.

## FRT export verification

Exported `exports/FRT_fulcrum_cascadeur_review.fbx` from the saved 324-frame `FRT_fulcrum_v11` project, leaving the existing named exports untouched. The locomotion judge returned WARN with zero blockers; the fidelity gate passed at a 3.94° worst joint against an 8° limit. Blender matched 161/161 bones and keyed all 324 frames at 24 fps. The FBX re-import comparison sampled every frame and joint: maximum position error 0.0018 cm, mean 0.00029 cm. See `exports/FRT_fulcrum_cascadeur_review.validation.json` and `exports/guided_integration_validation.json`.

The saved baseline project was left intact; the tested neural result was rejected after verification rather than silently replacing the production motion.

## Verification performed

- MCP stdio → Fulcrum browser bridge: 13 guided workflow checks passed (range isolation, accept/reject, undo/redo, weapon grips, project round trip, quality constraints).
- Actual recovered model: 16-frame neural output, 23.05 seconds CPU inference, explicit numeric style 0 (label semantics unknown), finite and correctly shaped output.
- Full existing Node suite: 385 passed, 0 failed.
- Python bytecode compilation and JavaScript syntax checks passed.

## Remaining gaps

This does **not** reproduce all 145 checklist items or all of Cascadeur. In particular, the native posing UI defaults and anatomical output adapter, other two inbetween networks' native settings, Cascadeur's full nonlinear physics/contact and collision solve, and native interpolation solver are not yet established. See the checklist audit for evidence level per item.
