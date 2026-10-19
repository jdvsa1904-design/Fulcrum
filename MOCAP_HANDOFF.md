# Handoff: Fulcrum video → rigged 3D → WASD

## Goal (user)
Make Fulcrum a finished product: any reference MP4 → full 3D animation on a rig, playable with WASD.

## Decision
Fulcrum's strength is mocap cleanup: Butterworth/despike (filters.js), foot plants (contacts.js, wire/plants.js), two-bone IK (ik.js), HIK retarget (wire/hikretarget.js). Build a video-mocap front end on top of it:
MediaPipe Pose (browser) → clean → solve rig → ground + foot lock → loop/in-place → WASD player → GLB/BVH export.

## Environment facts
- Branch: `claude/optimistic-goodall-z280ij`. Nothing committed yet.
- Node tests: `node tests/<x>.test.mjs`; 38 files passing at start.
- Network: cdn.jsdelivr.net and unpkg are BLOCKED here; registry.npmjs.org, pypi and storage.googleapis.com work. Existing studio index.html uses jsdelivr (fine on user's Windows machine).
- Test video: `pip download sports2d --no-deps`, unzip `Sports2D/Demo/demo.mp4` (VP9, 1768x994, 30fps, 230 frames, person walks sideways; a statue and second person also in frame). Copy is at `/tmp/claude-0/-home-user-Fulcrum/f702f651-84c7-5b3d-a211-475c082d9bca/scratchpad/media/demo.mp4`.
- Playwright Chromium likely can't decode H.264 → server transcode fallback added.

## Done (uncommitted)
- `tools/fetch_vendor.py`: downloads three@0.169.0 (build + examples/jsm controls/exporters/loaders/utils/libs/curves), @mediapipe/tasks-vision@0.10.21 (vision_bundle.mjs + wasm/), pose_landmarker_{lite,full,heavy}.task into `vendor/` (gitignored, already downloaded here).
- `server.py`: `fetch_vendor.ensure()` on start; mime map (.mjs/.wasm/.task/.glb); `GET /api/mocap/status`; `POST /api/transcode?name=` (ffmpeg → VP9 webm).
- `.gitignore`: added `vendor/`.
- `mocap/math.js`: vec/quat helpers, basisQuat(primary, secondary), frameRotation, slerp, log/exp, ZXY euler (BVH), median/percentile.
- `mocap/skeleton.js`: Mixamo-named 22-bone skeleton (Hips…ToeBase), T-pose facing +Z, Y-up, metres, character left = +X, rest world rotations identity. MediaPipe LM indices, measureFromLandmarks, buildSkeleton, fk, landmarksFromPose (synthetic landmarks for tests/rest frames), footPoints, standLocal (arms-down idle).
  - PENDING FIX: in landmarksFromPose replace the nose line with `const head = B.Head, eh = 0.38 * m.neck; L[LM.nose] = at(head, [0, eh - 0.012, 0.105 * h]);` and drop unused `scale` import.

## Remaining plan
1. `mocap/landmarks.js`: MediaPipe world (x right, y down, z away) → Y-up via (x,-y,-z). Track selection (numPoses 2, nearest to previous / largest bbox / user click), gap fill. Root translation by linear least squares per frame: `t.x - u t.z = u P.z - P.x`, `t.y - v t.z = v P.z - P.y`, u,v from normalized image coords with assumed hfov 60° (user adjustable). Filter depth at ~1.5 Hz, x/y ~6 Hz.
2. `mocap/clean.js`: peakRemovalVec + butterworthVec from filters.js on hip-relative landmarks (fc ~6 Hz).
3. `mocap/solve.js`: world rot = basisQuat(target) * conj(basisQuat(rest)). Hips: up=shMid-hipMid, sec=LHip-RHip. Spine2: same up, sec=LSh-RSh. Spine/Spine1 slerp 1/3,2/3; Neck slerp chest/head 0.5. Head: nose-earMid, earL-earR. Arms/legs: primary bone dir, secondary = cross(upper, lower) hinge axis, blended with minimal-twist fallback (rest axis swung by arc from parent frame); flip if opposite fallback. Rest hinge axes: left arm (0,-1,0), right arm (0,1,0), knees (1,0,0). Hands: wrist→mid(index,pinky), sec index-pinky; clamp to 80° from forearm. Feet: toe-heel, sec ankle-heel. Clavicles/toes identity. Rest frames for non-hinge bones from landmarksFromPose(rest).
4. `mocap/ground.js`: up from plane fit of per-frame lowest foot points (fallback camera up); floor = median lowest point; root Y grounding on contact frames; plants via contacts.js detectPlants; pelvis drop if unreachable; ik.js twoBoneIK lock with smoothstep blends; flatten foot roll in contact.
5. `mocap/cycle.js`: best loop (pose distance, 0.7–2.6 s), ramp error correction, remove horizontal travel, rotate travel to +Z, speed, phase start at left contact; classify idle/walk/run/action.
6. `mocap/gait.js`: procedural walk/run (foot trajectory + IK, cadence T=clamp(1.25-0.13v,0.62,1.2)) and idle breathing as fallbacks.
7. `mocap/bvh.js` export; GLB via three GLTFExporter.
8. Node tests: synthetic gait → landmarksFromPose (+noise, projected) → pipeline → compare joints/speed.
9. UI: new product at `index.html` (move current studio to `studio.html`, update Fulcrum.pyw/HANDOFF/MCP notes), importmap to `vendor/three`. Steps: drop video (transcode fallback) → capture with MediaPipe (VIDEO mode, seek per frame, GPU→CPU fallback, skeleton overlay, trim) → solve/clean controls → procedural skinned mannequin (or user GLB/FBX retargeted via wire/hikretarget.js) → Play mode: WASD camera-relative, Shift run, Space jump, E action, orbit camera, phase-synced idle/walk/run blending, playback rate = speed/clipSpeed → export GLB/BVH/project JSON.
10. Playwright E2E on demo video, README, commit, push.
