# Fulcrum handoff

## Fulcrum Mocap: video to rigged 3D to WASD (2026-10-19)
- **Run:** `python server.py 8790`, then http://localhost:8790 is the mocap app. The clean-up studio moved unchanged to http://localhost:8790/studio.html; the MCP bridge needs that page open.
- **Pipeline (`mocap/`):**
  - MediaPipe Pose Landmarker in the browser (tasks-vision 0.10.21, VIDEO mode, 2 poses). Frames are seeked one by one; the next frame decodes while the current one is analysed, and the faster of GPU and CPU is kept after 3 frames.
  - Person pick: duplicate detections dropped (box IoU > 0.5), score = size x visibility x (1 + travel), click a person to override. Ends where the box touches the frame border are trimmed.
  - Root: least-squares camera-space translation from 2D + 3D landmarks with an assumed lens (default 60 degrees, adjustable); landmarks blended halfway toward the 2D rays.
  - Clean: despike (`peakRemovalVec`) on limbs and root, then Butterworth low-pass with linear padding (6 Hz limbs, 4 Hz root, 1.5 Hz depth).
  - Floor: plane fit to slow, low foot points with a ridge prior toward the mean torso direction.
  - Skeleton: 22 Mixamo-named bones, T-pose facing +Z, Y up, metres, rest rotations identity, proportions measured from the landmarks.
  - Solve: basis-frame rotations; elbows and knees use the hinge axis with a minimal-twist fallback near straight; knees never bend backwards more than 5 degrees; half of the wrist twist goes to the forearm.
  - Ground and feet: grounding from contact frames (lower-foot percentile when contacts are rare), `wire/plants.js` plants trimmed to where the foot stays within 7 cm. The body is then shifted up or down (smoothly) so planted feet touch the floor, and `ik.js` two-bone locks pin each planted foot at its median contact point, with a soft reach only in the last 0.7 percent of leg length. The pelvis drop is off by default (`maxDrop`): any drop bends near-straight legs a lot.
  - Moves: still/move segments split into walk and run bands; loops found by pose-feature autocorrelation, ramp-closed, made in place facing +Z, started at a left foot plant; jumps need both feet up and a hip rise of 8 cm; other segments become actions; the whole take is a replay clip. Missing idle, walk, run or jump are generated (`gait.js`) and flagged as made up.
- **Controller (`mocap/controller.js`):** camera-relative WASD, Shift run, Alt slow walk, Space jump, E and 1-9 actions, R replays the whole take with its root motion, F full screen, Esc. Idle, walk and run blend by speed with a shared phase; playback rate follows stride length so feet do not slide.
- **Exports:** GLB (mannequin, or the loaded GLB/glTF/FBX character through `wire/hikretarget.js`, every move as a clip), FBX through Blender (`tools/glb_to_fbx.py`, `POST /api/glb-to-fbx`; same export settings as `frames_to_fbx.py`, all actions as separate takes), BVH (Mixamo names, cm, ZXY), project JSON (tracking results, no video).
- **Server:** `tools/fetch_vendor.py` puts three.js 0.169.0, tasks-vision and the three pose models in `vendor/` (gitignored, background download on start); missing vendor files redirect to the CDN. `POST /api/transcode` converts videos the browser cannot play (needs ffmpeg). `GET /api/mocap/status`.
- **Measured:**
  - Node tests: `tests/mocap_core`, `mocap_motion`, `mocap_pipeline` (21 tests, all passing). The pipeline tests film a synthetic person with a virtual camera plus tracker-like noise: mean joint error about 3.5 cm, walk speed within 0.03 m/s, loop length within one frame, camera height within 2 cm, skate after locking under 1 cm.
  - Sports2D `Demo/demo.mp4` (VP9, 1768x994, 230 frames; walking woman, a man on a box, a passer-by): picks the woman, frames 59-195 after border trim, height 1.60 m, walk loop 1.16 m/s with a 1.23 s cycle, no false jump; idle, run and jump are made up. The blue reprojection over the video lines up with MediaPipe's 2D points.
  - Foot locking on that clip: stance knee bend 19 degrees median (16 before locking; 27 to 32 with the old pelvis drop), planted feet within 3 mm of the floor for 90 percent of frames, at most 0.6 cm slide.
  - In this container (no GPU) tracking took 59 s for the 230 frames on CPU.
  - Exports: the GLB reloads in three.js (1 skinned mesh, 22 bones, 5 clips); the BVH parses in three's BVHLoader; the FBX made through Blender 5.2.2 re-imports with 22 bones and all 5 actions at the right lengths.
- **Gaps:**
  - Depth comes from one camera and the lens guess; a wrong lens stretches walking toward or away from the camera.
  - The spine only twists (pelvis and chest share the torso axis); fingers and toes are not tracked.
  - The made-up jump is a standing jump; from a run it is blended in.
  - Loading third-party characters goes through the HumanIK retargeter; only Fulcrum's own GLB was tested here.
  - Phones are out of scope (user decision): it is a keyboard and mouse app.

## Studio goal (2026-10-15)
Fulcrum (C:\Projects\Fulcrum) replaces Cascadeur. It fixes the FRT drill capture (sprint → plant → slide → kneel → rise → stop/shoulder) on the Checkpoint soldier and exports FBX to Unreal. The current task was to reverse-engineer Maya with AdvancedSkeleton, 3ds Max and MotionBuilder using Ghidra and REA, then port the useful math into Fulcrum.

## App state
- Run it with `python server.py 8790` (launch config "fulcrum" in C:\.claude\launch.json), then open http://localhost:8790/studio.html.
- Code:
  - app.js and studio.html are the UI and the pipeline.
  - ballistic.js and fulcrum.js hold the Cascadeur-derived physics.
  - filters.js, ik.js, contacts.js, stitch.js and hik.js are the ported solvers.
  - phases.js splits a take into movement phases (stand, walk or run, running on the spot, flight, kneel and so on). It used to be tastebridge.js.
  - tools/frames_to_fbx.py is the Blender 5.2 FBX writer.
- Current export: exports/FRT_fulcrum_v6.fbx (round-trip 0.00 cm, 356 frames, 123 joints). Project file: exports/FRT_fulcrum_v6.fulcrum.json.
  - Exported with "export anyway" unticked: taste PASS with no findings, gate PASS (worst untouched joint 6.0 deg, pelvis moved 0.2 cm).
  - Fixes in v6:
    - smoothing over both shot seams, f93-108 and f256-271
    - "ground feet" on the right foot over f66-80, f130-144 and f282-296
  - How the ground-feet fix works:
    - It picks the lower foot in the window and the frame where its ball moves slowest.
    - It holds the ball's x/z there and lowers the foot until its heel or ball reaches its own planted height from the standing frames.
    - Foot IK with a 6-frame smoothstep blend; the pelvis drops only if the leg cannot reach.
  - Measured result:
    - The ball holds at 0 cm/s during each plant.
    - Joint steps at the plant edges stay at or under 21.7 deg, against 23 deg in the raw capture.
    - Release speeds are 3.5-5.9 m/s, which is sprint swing speed.
  - Unreal 5.8 (throwaway ue_check project, headless): v6 imports with 0 artifacts, and all 161 bones are within 0.07 deg of the source on every frame. The only steps over 25 deg are 4 real arm moves that are in the capture (f220, f307, f344); the seam jumps are gone.
  - Superseded: v5 (seams only) and v4 (correct import, no fixes).
- v2 and v3 have two import bugs (fixed 2026-10-15 in sampleTake). Do not use them.
  - **Last-frame wrap:** three.js LoopRepeat wrapped t = duration to 0, so each shot's last frame was a copy of its first (v3 f100 = f0, f263 = f101).
  - **Bad retarget:** local rotations were copied, but the shot FBXs load Z-up while CheckpointBody loads Y-up (exactly 90 deg about X, zero residual). pelvis->thigh was off 87 deg, foot->ball 80 deg, spine_05->clavicle 27 deg.
  - **The fix:**
    - The clip plays once (LoopOnce, clamped).
    - Rotations are retargeted in world space, `G * srcWorld * inv(srcRest) * inv(G) * dstRest`, where G is the best-fit rest alignment.
    - Root and pelvis are placed in world space through the same fit.
  - **Verified:** all joints relative to the pelvis match the source shots to 0.000 cm, and bone directions to within 0.001 deg, in all three shots.
- Edit layer checked by script: a 17.2 deg key blends with the 12-frame falloff, edited joints leave the gate, and clearing restores the take.

## LocomotionTaste removed (2026-10-19)
- **What happened:** LocomotionTaste (the separate program at C:\Projects\LocomotionTaste that scored locomotion against CMU data) was deleted, so Fulcrum no longer uses it.
- **Removed with it:**
  - The "Locomotion taste" panel, its "judge after every edit" run, and the toolbar button.
  - The server routes /api/taste, /api/taste-status and /api/taste-last, the `--taste-phases` command, and the files exports/taste_last.json and exports/taste_ledger.jsonl. The "taste" setting in fulcrum.json is ignored.
  - The locomotion check before FBX export, and the MCP `judge` tool.
- **Kept:**
  - **Phases (phases.js):** segmentPhases splits the take into stand, locomotion, in_place, flight, vault, plant, slide, kneel, crouch, prone, rise, upper_body and freeform. It is Fulcrum's own code and is kept for the coming one-click fix; nothing in the studio calls it yet. phaseInput turns the studio's joint matrices into the 22-joint, Z-up, centimetre layout it reads.
    - **flight** only counts if its airtime fits the pelvis rise, so floating feet cannot hide as a jump.
    - **in_place** is locomotion where the pelvis moves at under 0.35x the swing-foot speed: running on the spot. The FRT sprint (f51-145) moves the pelvis at 0.37 leg lengths/s while the feet move at 2.14.
  - **Auto Fix** (Review workspace, MCP `auto_fix`): the same loop, now judged by Fulcrum's own checks instead of LocomotionTaste.
    - The physics flags are blockers: planted foot sliding becomes foot_slide, airborne but not falling becomes ground_float, pose snaps become pop.
    - Motion quality: a foot below the floor becomes ground_sink (a warning). Failed foot locks or hand pins, stretched bones and invalid numbers are blockers.
    - Sliding foot plants (from wire/plants.js) become foot_slide warnings.
    - The whole run is one undo step; the trial rebuilds no longer fill the undo history.
  - **Export gate:** the motion-quality check still stops the FBX export when foot locks or hand pins fail or transforms are broken. "export anyway" now sits in the Export section.
- **Older notes below** quote "taste" results (PASS, WARN, blockers). Those came from LocomotionTaste when it was still installed.

## Reverse-engineering reports (C:\Projects\Fulcrum\re)
Every claim is labelled OBSERVED, MEASURED, INFERRED or UNKNOWN, and every report ends with a "port to Fulcrum" list.

| File | Status |
|---|---|
| CASCADEUR_PHYSICS.md | Done. |
| MAYA.md | Done: ikRP/SC solver, ikSpringSolver, Euler filter, Butterworth (Python model matches to 1e-13), FBX defaults. The only UNKNOWN left is the HumanIK knee swivel. |
| ADVANCEDSKELETON.md | Done: FK/IK pole math, foot roll, mocap matcher, Unreal export conventions. |
| MOTIONBUILDER.md | Done: Butterworth, peak removal, key reducers, smooth, unroll, Story gravity match. HumanIK rows were corrected from the real library. |
| HUMANIK_SOLVER.md | Nearly done, decompiled from the real humanik.lib (MoBu SDK, with symbols) plus a COFF-loader harness. Done: reach T/R, pull, knee resist/MaxExtension, enforce gravity, floor contact, finger/toe sticky/spread, stiffness. §9 has port pseudocode. **Open: the knee swivel term under Reach R, and ContactSize %.** |
| MAX_BIPED_CAT.md | Nearly done: IKLimb, HI solver, gravAccel, ballistic tension, dynamics blend, mocap smoothing, CAT keys, knee cleanup. Contact rule decoded from code (§1.2). **Open: the contact rule needs a measured check, and balanceFactor needs its exact law (measured as a linear pelvis/head counter-shift).** |

Reports not written (agents stopped): HUMANIK_MEASURED.md, MAX_BALANCE.md, PORT_PLAN.md.

## Porting (done 2026-10-15)
- **Solver modules:** filters.js, ik.js, contacts.js, stitch.js, hik.js.
- **App adapters (wire/):** floor.js, roll.js, plants.js, seams.js, bodypull.js, cleanup.js.
- **Node tests:** 185 across 13 files (`node tests/<name>.test.mjs`), all passing.
- **In the UI:**
  - **Contacts:** "Floor contact" (whole take, off by default), "Floor contact in selection", "Foot roll" (over locked ranges in the selection), "Auto-detect plants".
    - Floor contact is HumanIK's six-marker sole fit. A bent toe that would turn the ball more than 45 deg is rejected (wire/floor.js MAX_TOE_TURN_DEG).
    - Auto-detect plants uses Biped Rule A and counts a foot as planted when the ball OR the ankle is down. Sliding contacts are listed, not locked, and manual locks outside the selection are kept.
  - **Take:** "Match shot seams" (off by default). It uses hip-axis heading, because this rig's pelvis X axis points up.
  - **Support hand:** "Pull body to hand", with pull and gravity inputs (HumanIK body pull).
  - **Physics:** "Remove pops", "Filter jitter" (Fc), "Median smooth". The FBX export log shows a key-reduction report.
- **Defaults leave v6 unchanged:** 0.0000 cm difference against the v6 frames.
- **Browser-checked on the FRT take** (no NaN in any case):
  - **Floor contact on a range:** f295-344, 54 frames corrected; gate and taste pass.
  - **Whole-take floor contact:** now "Floor contact on plants". It acts only on stance plants from detectFootPlants (non-sliding), and holds the ball still on slow runs.
    - On the FRT take: 203 frames corrected, taste PASS with no findings, gate PASS, worst joint step 25.5 deg (the capture's own is 22.7).
    - On every frame it failed. The capture's run-phase feet skim the floor at speed, and lifting or planting them only moves the skate around.
  - **"Level feet":** removes the sole's sideways roll near the floor, with a time-smoothed weight that fades when the foot points steeply. The right foot was rolled about 42 deg on average onto its inner edge in the capture.
    - Alone: roll drops to 0.1 deg mean, taste PASS, gate PASS, worst step 29.6 deg on ball_r at f63.
    - Combined with "Floor contact on plants": it currently fails taste (right-foot slide at f267-271), so they do not stack yet.
  - **Auto-detect plants:** stand f0-58, kneel f173-205, final stand f296-355; 13 sprint contacts listed as sliding.
  - **Foot roll:** on the kneel plants; gate and taste pass.
  - **Seams, despike, filter and median:** gate and taste pass.
  - **Body pull:** "Pin left hand in world" (Support hand) pins the hand to a fixed world point, which is the goal "Pull body to hand" needs. Grip pins are ignored by pull.
    - Test: world pin at f220, held over f220-250. The hand reaches the goal (0.0 cm; 5-11 cm without pull) and the pelvis is pulled up to 18.7 cm. No NaN, gate pass.
    - Taste fails that test, correctly: pulling the body mid-run makes the feet slide. Use it on stands or kneels against a wall or prop.
- **Gaps:**
  - hik.js knee swivel under Reach R and ContactSize are UNKNOWN, and the body-pull constants are fitted.
  - Foot rock and ball-pivot locks are not wired.
  - The Blender-side key reduction is a report only.

## Next steps
1. Import v6 into the Checkpoint project and look at it in game (the throwaway ue_check project is not Checkpoint).
2. Optional: HumanIK knee swivel and ContactSize (C:\re_work\hik\hik.py), Biped balanceFactor (C:\re_work\max\biped.dlc), Biped contact sweep in a signed-in Max (C:\re_work\max\tests\csmsweep.ms).
e_work\hik\hik.py), Biped balanceFactor (C:
e_work\maxiped.dlc), Biped contact sweep in a signed-in Max (C:
e_work\max	ests\csmsweep.ms).

## v7: patched toward the Delta FRT drill video (2026-10-16)
- **Reference video:** C:\Users\Gaban\Downloads\the-delta-team-tactical-3position-frt-being-used-in-a-drill-ytmp4.savetube.vip.mp4. Contact sheets are in exports/review/ref (sheet.jpg is the video, v6_sheet.png is v6).
- **Export:** exports/FRT_fulcrum_v7.fbx and FRT_fulcrum_v7.fulcrum.json; 282 frames.
  - Taste WARN (pelvis bounce, stance width); gate PASS (5.7 deg); round-trip 0.00 cm; exported without "export anyway".
- **Changes from v6:**
  - **Kneel cut:** v6 f146-219 removed with the new cut tool. A join fix carries the pelvis across the cut, with smoothing at +-6 frames.
  - **Sprint travel:** forward travel at 2 m/s over f54-188, about 10.8 m.
    - The stance feet are held at their footfalls while the body passes over them.
    - Speeds of 3-4 m/s made the feet float, because the capture's short stance times cannot reach that far.
  - **Lean:** 18 deg forward on spine_01 over the sprint.
  - **Braking stop:** comes from the 10-frame ease-out of the travel.
- **New fix types:** travel{a,b,speed,ease,plant}, lean{a,b,deg,ease}, join{at}. Cuts are stored in the project as `cuts` and re-applied on open.
- **Fixed:** verifyFbx had the same LoopRepeat last-frame wrap as import, and its best-fit alignment added error over long paths. It now plays once and reports the smaller of the direct and fitted errors.
- **Still unlike the video:**
  - The rifle carry is high during the run; the video swings it low across the body.
  - There is one long sprint, not sprint, stop and fire, then sprint again.
  - The sprint is 2 m/s, against an estimated 4-5 m/s in the video.

## v8: sprint, stop and fire, sprint, stop and fire (2026-10-16)
- **Export:** exports/FRT_fulcrum_v8.fbx and FRT_fulcrum_v8.fulcrum.json; 324 frames.
  - Taste WARN with no blockers; gate PASS (3.9 deg); round-trip 0.00 cm; exported without "export anyway".
- **Structure, built from v7:**
  - stand f0-51
  - sprint 1, f54-137, 2 m/s with an 18 deg lean, braking over the last 14 frames
  - stop and fire f139-180: a copy of the firing stand f226-267, inserted with the new insert tool, both feet locked
  - sprint 2, f181-230
  - stop f231-ish, then the final fire stand
- **Join settings:**
  - The cut point (sprint f139 into fire f226-267) has the smallest leg and hand pose difference. That difference is still about 47 cm per joint, so the transitions are 12-frame blends.
  - The gate now skips join-blend frames, which are deliberate edits.
- **travel.flatten:** 0.12 partly flattens toe-down footfalls toward the rest pitch.
  - At 0 the pitch fails at -30.0 deg (limit -29.8); from 0.2 up the ankle angle fails.
  - The speed is jogging but the pose is sprinting, so the gait bands are tight. 2.4-2.7 m/s still fails on the ankle.
- **Remaining warnings:**
  - left ankle pop at f189 (2.1x, limit 1.9)
  - ankle slightly dorsiflexed, toes out about 30 deg, stance width 1.0
- **Not done:** the low rifle carry during the run.

## Fulcrum MCP (2026-10-16)
- **Server:** mcp/fulcrum_mcp.py, registered at user scope as `fulcrum`. It goes through server.py /api/bridge to the open page; app.js polls /api/bridge/next.
- **Needs:** `python server.py 8790` running and http://localhost:8790/studio.html open in a browser.
- **Tools:** status, load_frt, open_project, save_project, add_fix, set_fixes, remove_fix, cut, insert, set_locks, set_pins, set_layer, select, auto_fix, pose, screenshot (side, front, back, top, three_quarter, relative to the character's facing), export_fbx, eval_js.

## Tools and work dirs
- Ghidra 12.1.2:
  - Install: C:\Users\Gaban\Downloads\ghidra_12.1.2_PUBLIC_20260605\ghidra_12.1.2_PUBLIC
  - JAVA_HOME: C:\Users\Gaban\Downloads\OpenJDK21U-jdk_x64_windows_hotspot_21.0.12.1_1\jdk-21.0.12.1+1
  - GhidraMCP: port 8089
  - Copy binaries to a path without spaces or parentheses first.
  - Use a separate Ghidra project directory for each parallel run.
- Scripts in tools/ghidra: StringXrefDump, ClassDump, DumpTool, DecompAt, ObjDecomp. Max-specific ones are in tools/ghidra/max.
- Work dirs:
  - C:\re_work\maya (bin, out, tests)
  - C:\re_work\mobu
  - C:\re_work\max (out, full, tests, proj_contact)
  - C:\re_work\hik (hikload.py COFF loader, hik.py wrapper, x_*.py, results, dec)
- Rules: no code comments, don't modify installs, never write into C:\Projects\Checkpoint.

## v10 sprint strides and app parity modules (2026-10-15)
- `FRT_fulcrum_v10` (324 frames): both sprints at 4 m/s with real strides.
  - **Travel options:** `reach` (each foot plants at mid-stance and swings forward to the next footfall), `toe:1` (planted feet turned toward the run direction), `pitch:-30`.
  - **Old ground fixes:** inside a sprint they become short stances; the stance length comes from Biped contact timing (0.3 × leg / speed).
  - **Checks:** taste warn with no blockers; gate 3.9; round-trip 0.00 cm.
- **App parity modules** (in `re/`, tested, not yet wired into app.js):
  - `re/hik`: retargeter, knee twist, contact size, neck and head pull.
  - `re/biped/footsteps.js`: Max footstep generator, balance, footstep detection.
  - `re/maya`: curves, layers, skinCluster.
  - `re/mobu`: Story, layers, aux effectors, filters.
  - Each folder's `NOTES.md` gives the confidence of every finding.
- **Live app bridges:**
  - Maya commandPort 127.0.0.1:7002 (opened by the user).
  - Max `tools/max_bridge.py` on port 7003.
  - MotionBuilder telnet on port 4242.
  - The client is `tools/app_bridge.py`.

## v11: re/ modules wired into the app (2026-10-15)
- **Export:** `exports/FRT_fulcrum_v11.fbx` and `FRT_fulcrum_v11.fulcrum.json`, made by re-opening v10 in the integrated app.
  - The output is bit-identical to v10: every S.out quaternion and position differs by 0, and v11.frames.json equals v10.frames.json.
  - Gate PASS (3.9 deg, ball_r). Taste WARN with no blockers (left foot pop f188, shin crossing f79, pelvis bounce). Round-trip 0.00 cm. Exported through the bridge with override false.
- **New adapters (wire/), all pure and node-tested:**
  - `hikretarget.js`: HIK characterization of any humanoid. It drops ik/twist/corrective helpers, reads three.js-sanitised namespaces (`mixamorigLeftUpLeg`) and aligns up/left/forward between rigs, then runs `retargetPose` per frame and fills unmapped target bones from their rest locals.
  - `footsteps.js`: Biped footstep plan for travel fixes. Contacts come from Rule A plus ball-height minima, contact length from Max's run timing (0.6 leg / speed), footprints at COM - (0.9595 v + 0.0107 leg) forward, with half width 0.1 leg x width. Balance is the measured balanceFactor shift.
  - `animlayers.js`: Maya animation layers on the pose layer: per-channel Euler curves with Maya tangents (`re/maya/curves.js`), stacked through `re/maya/layers.js` (additive/override, by layer/by component, weight, mute), plus an inverse key solve.
  - `story.js`: Story transitions for cuts and inserts. The outgoing clip plays on into the removed source frames and crossfades (ease/linear/slow_in/fast_in) into the incoming one, with a gravity hips match carried to the end of the take.
  - `floor.js` now uses HumanIK's solver markers (`re/hik/contact.js`, ContactSize pinned to 100). Contact size only scales the drawn sole markers.
- **App, UI and bridge:**
  - **Retarget:** selector auto | by bone name | HumanIK. Auto uses HIK when under 80% of the core UE names match. The bind pose comes from skin bone inverses when present. Projects store `retarget: {shot: 'hik'}`.
  - **Take section:** Cut selection / Insert selection at playhead, with Story blend (default on in the UI) plus curve and blend frames. Fix type `story{at,pre,post,curve,match}`. The source take and frame map are kept as `S.srcTake` / `S.srcIdx`, and Story windows are excluded from the gate like join blends.
  - **Pose layer section:** Maya layers (active, add/delete, weight, mute, override, rotation accumulation, tangent). With a layer active, the ring keys into it. Saved in projects as `animLayers`.
  - **Contacts section:** "Biped footsteps in selection" (speed, balance) adds `travel{footsteps:true}`. Contact size and "Show sole markers" are display only.
  - **server.py:** serves `C:\Projects\Fulcrum\imports\*.fbx` through /samples and /api/samples.
  - **MCP:**
    - New tools: `import_fbx`, `set_anim_layers`, `key_anim_layer`, `sole_markers`.
    - `cut`/`insert` take `story`, `curve` and `match`.
    - `status` reports retarget, animLayers, footsteps and story.
    - The MCP server must be restarted to show the new tools; the bridge commands already work.
- **Tests:** `tests/integration_retarget.test.mjs` (5) and `tests/integration_wire.test.mjs` (10). Fixture: `tests/fixtures/checkpoint_v10_sample.json`, with 12 v10 frames, all 161 bones and their hierarchy. 282 node tests, all passing.
- **Measured:**
  - **HIK retarget:**
    - In node: a Mixamo-renamed, Z-up, metre-scaled copy of v10 with random per-bone rest rotations reproduces v10 within 0.05 units and 0.05 deg.
    - In the page: a Mixamo-named clone of the Checkpoint body animated with shot1 retargets to 0.000 cm against the name import, gate pass.
  - **Story cut** of f262-277 on v10: gate pass (4.0 deg), match offset 0.13 cm.
  - **Maya layer** (upperarm_r, 3 auto keys, weight 0.8): gate pass, no NaN.
  - **Footsteps instead of reach on v10's sprints:** 12 and 8 footprints, 3-frame contacts, gate pass.
    - Taste fails at width 1.0 (shin crossing f122, knee pop f126, floats).
    - At width 1.5 one blocker remains: left knee pop f126. Reach stays the default for the drill.
- **Bridge note:** the Opera tab running the bridge goes idle in the background, and calls then fail with "page is not open" until it polls again. Don't open a second bridge page while it is alive, because the two race for jobs.
