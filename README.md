# Fulcrum

Fulcrum turns a video of a person into a 3D animated character you can drive with your keyboard.

1. Drop in a video (MP4 or WebM).
2. Fulcrum tracks the person, rebuilds the motion in 3D on a skeleton, and cleans it up (no jitter, no foot sliding).
3. It cuts the motion into moves: standing, walking, running, jumping and anything else the person did.
4. You play the character with W A S D, and export it for Unreal, Blender or any 3D app.

The older animation clean-up tool is still here, at `studio.html`.

## Start it

On Windows, double-click `Fulcrum.pyw`. Or run:

```
python server.py 8790
```

and open http://localhost:8790 in Edge or Chrome.

The first start downloads the 3D engine and the pose tracker (about 80 MB) into `vendor/`. After that it works offline. If the download is blocked, the page loads the same files from the internet instead.

## The four steps

1. **Video.** Drag a video onto the left panel. Phone clips are fine. Keep the camera still if you can, and keep the whole body in view. You can trim the start and end.
2. **Track.** Press *Track motion*. Fulcrum finds the body in every frame (Fast, Balanced or Best quality). The blue lines drawn over the video show the 3D result, so you can check it lines up. If the video has several people and it picked the wrong one, click the right person in the video. *Fine-tune* has the camera lens, smoothing, the person's height and foot locking.
3. **Moves.** The moves found in the video are listed. Click one to preview it. Walking and running always come from the video if the person moves around in it at all. If the video has walking but no running, the run is the video's walk played faster (and a run with no walk is slowed down into a walk); these say *from the video (sped-up walk)* or *(slowed run)*. Only if the person never moves around does Fulcrum make up a walk and run so the controls still work; those are marked *made up*. Every other move found, including extra walks, runs and short moves, can be played from the keyboard or mouse.
4. **Play.** Press *Start playing*.

## Keys in play mode

| Key | What it does |
| --- | --- |
| W A S D or arrow keys | Move, relative to the camera |
| Shift (hold) | Run |
| Alt (hold) | Slow walk |
| Space | Jump |
| Q | Pick the next move (Shift+Q picks the previous one). The picked move is shown at the top of the screen and in the key list |
| E or left click | Play the picked move (a click only counts if you don't drag) |
| 1 to 9, 0 | Play move 1 to 9, and 0 plays move 10 |
| R | Replay the whole video's motion from where the character stands |
| F | Full screen |
| Mouse drag / wheel | Look around / zoom |
| Esc | Stop a move, or leave play mode |

You can play the built-in mannequin or load your own character (GLB, glTF or FBX with a humanoid skeleton, for example from Mixamo). Fulcrum maps the motion onto it.

## Export

- **Character + moves (.glb):** the character with every move as a separate animation. Opens in Blender, Unreal 5 and three.js.
- **For Unreal (.fbx):** the same, converted to FBX through Blender. Needs Blender installed (set its path in `%APPDATA%\Fulcrum\fulcrum.json` as `"blender"` if it is not found automatically). A copy is kept in `exports/`.
- **Full take (.bvh):** the whole tracked motion as BVH, with Mixamo bone names, in centimetres.
- **Save project:** keeps the tracking result, so you can reopen it later without tracking again.

## Getting good results

- The whole body should be visible, head to feet, for most of the video.
- A still camera works best. Side or three-quarter views of walking and running work very well.
- If the motion looks too deep or too shallow, change *Camera lens* (wider for phone wide-angle lenses).
- If you know the person's height, type it in. It makes speeds and jump heights exact.
- Videos the browser cannot play (some HEVC or AVI files) are converted automatically when ffmpeg is installed.

### Checking the depth from other angles

A single camera sees exactly where each joint is across the picture, but not how far away it is. Fulcrum works out the distance from the fact that bones keep their length. That leaves one question per bone: does it point toward the camera or away? Fulcrum answers it from many clues: knees and elbows only bend one way, feet point where you walk, feet rest on the floor, and the tracker's own depth guess. It then checks every answer against thousands of practice videos where the true answer is known.

After tracking, the report says how much of the motion is verified, for example "94% of joint positions verified from any angle to within 5 cm". Verified means that on videos like the practice set, at most 1 position in 100 is off by more than that, and this holds for any viewing angle at once. Spans the camera cannot settle, such as an arm pointing straight at the lens, are marked for review instead of guessed.

The check assumes a still camera and roughly the right lens setting. If the video is unlike the practice videos (very shaky tracking, limbs hidden most of the time, a jump that does not match gravity), the report says "not verified" and why. For the best result, film from about 45 degrees and 1.5 to 2 m up, at 60 fps if you can, and turn around once. Enter the person's height so sizes are exact too.

This depth check is still being tuned, so it is switched off for now. Until it is switched on and calibrated, the report shows no depth line.

## For developers

- `mocap/`: the video-to-3D pipeline (`landmarks.js`, `clean.js`, `solve.js`, `ground.js`, `cycle.js`, `gait.js`, `pipeline.js`), the WASD controller (`controller.js`), BVH export (`bvh.js`) and the browser app (`app.js`, `capture.js`, `viewer.js`, `character.js`, `exporters.js`).
- It reuses Fulcrum's solvers: Butterworth and despike filters (`filters.js`), foot-plant detection (`wire/plants.js`), two-bone IK (`ik.js`) and the HumanIK retargeter (`wire/hikretarget.js`).
- Tests: `node tests/mocap_core.test.mjs`, `node tests/mocap_motion.test.mjs`, `node tests/mocap_pipeline.test.mjs`. The pipeline tests film a synthetic person with a virtual camera, add tracker-like noise, and check the rebuilt motion against the truth.
- Depth check: `mocap/depth.js` (bone-length geometry and the depth solver), `mocap/depthsign.js` (toward-or-away decisions), `mocap/depthstats.js`, `mocap/depthcert.js` (the error bounds and the report line), `mocap/depth_calibration.js` (generated), `mocap/mpfail.js` (simulated tracker mistakes for the practice videos) and `mocap/depthbench.js` (practice scenes and scores). It runs when the pipeline setting `depth` is true (off by default).
- `node tools/depth_calibrate.mjs` rewrites `mocap/depth_calibration.js`; rerun it after any change to the files it hashes. `node tools/depth_benchmark.mjs` compares the depth check against the plain pipeline. Both take `--quick` for a short run.
- `node --test tests/mocap_depth.test.mjs` runs the depth check tests.
- `python tools/fetch_vendor.py` downloads the browser libraries by hand; `--check` reports what is there.
- `tools/glb_to_fbx.py` is the Blender script behind the FBX export.
- The studio no longer uses the separate LocomotionTaste program; it was removed. Its Auto Fix now uses Fulcrum's own checks.
