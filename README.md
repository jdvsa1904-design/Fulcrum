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
3. **Moves.** The moves found in the video are listed. Click one to preview it. If the video has no running, for example, Fulcrum makes up a run so the controls still work; those moves are marked *made up*.
4. **Play.** Press *Start playing*.

## Keys in play mode

| Key | What it does |
| --- | --- |
| W A S D or arrow keys | Move, relative to the camera |
| Shift (hold) | Run |
| Alt (hold) | Slow walk |
| Space | Jump |
| E | Play the first extra move |
| 1 to 9 | Play extra move 1 to 9 |
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

## For developers

- `mocap/`: the video-to-3D pipeline (`landmarks.js`, `clean.js`, `solve.js`, `ground.js`, `cycle.js`, `gait.js`, `pipeline.js`), the WASD controller (`controller.js`), BVH export (`bvh.js`) and the browser app (`app.js`, `capture.js`, `viewer.js`, `character.js`, `exporters.js`).
- It reuses Fulcrum's solvers: Butterworth and despike filters (`filters.js`), foot-plant detection (`wire/plants.js`), two-bone IK (`ik.js`) and the HumanIK retargeter (`wire/hikretarget.js`).
- Tests: `node tests/mocap_core.test.mjs`, `node tests/mocap_motion.test.mjs`, `node tests/mocap_pipeline.test.mjs`. The pipeline tests film a synthetic person with a virtual camera, add tracker-like noise, and check the rebuilt motion against the truth.
- `python tools/fetch_vendor.py` downloads the browser libraries by hand; `--check` reports what is there.
- `tools/glb_to_fbx.py` is the Blender script behind the FBX export.
