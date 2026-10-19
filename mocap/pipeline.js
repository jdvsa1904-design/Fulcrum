import { NLM, buildSkeleton, measureFromLandmarks, standLocal, NB, getV, getQ, setQ } from './skeleton.js';
import { buildTracks, pickTrack, trackSeries, cameraPoints } from './landmarks.js';
import { cleanPoints } from './clean.js';
import { floorFromPoints, floorTransform, applyFloor, groundTake, findPlants, lockFeet, footSkate } from './ground.js';
import { solveTake } from './solve.js';
import { buildClips, pickLibrary, standingFrame } from './cycle.js';
import { gaitClip, idleClip, jumpClip, defaultSpeed } from './gait.js';
import { qmul, qrot, qyaw } from './math.js';
import { depthPoints, gravityScale } from './depth.js';
import { certifyTake } from './depthcert.js';

export const DEFAULTS = Object.freeze({ hfov: 60, imageFit: 0.5, fc: 6, fcRoot: 4, fcDepth: 1.5, footLock: true, height: null, trim: null, person: null, depth: false, certify: false, gravity: true, omegaMax: 15 });

export function processCapture(capture, settings = {}) {
  const s = { ...DEFAULTS, ...settings }, t0 = Date.now();
  if (!capture?.frames?.length) throw new Error('The capture has no frames.');
  const tracks = buildTracks(capture);
  const track = pickTrack(tracks, capture, s.person ?? {});
  if (!track) throw new Error('No person was found in the video. Try a clip where the whole body is visible.');
  const series = trackSeries(capture, track, { trim: s.trim });
  if (series.n < 8) throw new Error('The person is visible for only ' + series.n + ' frames. At least 8 are needed.');
  const fps = series.fps, n = series.n;
  const cam = cameraPoints(series, { hfov: s.hfov, imageFit: s.imageFit });
  if (!cam) throw new Error('Could not place the person in 3D. Check that the whole body is in view.');
  const depth = s.depth ? depthPoints(series, cam, { fps, omegaMax: s.omegaMax }) : null;
  const cleaned = cleanPoints(depth ? depth.P : cam.P, n, fps, depth ? { ...s, fcDepth: Math.max(s.fcDepth, s.fcRoot) } : s);
  const floor = floorFromPoints(cleaned.P, n, fps);
  const tf = floorTransform(floor);
  let P = applyFloor(cleaned.P, n, tf);
  const frames = Array.from({ length: n }, (_, i) => P.subarray(i * NLM * 3, (i + 1) * NLM * 3));
  let dims = measureFromLandmarks(frames);
  let skel = buildSkeleton(dims), scale = 1;
  const applyScale = k => { scale = k; P = P.map(x => x * scale); dims = Object.fromEntries(Object.entries(dims).map(([j, v]) => [j, v * scale])); skel = buildSkeleton(dims); };
  let scaleSource = 'mediapipe', grav = null;
  if (s.height > 0.5) { applyScale(s.height / skel.dims.height); scaleSource = 'height'; }
  else if (depth && s.gravity && (grav = gravityScale(P, n, fps))?.ok) { applyScale(grav.scale); scaleSource = 'gravity'; }
  const take = solveTake(skel, P, n);
  take.fps = fps;
  const ground = groundTake(skel, take, fps);
  const found = findPlants(skel, take, fps);
  const lock = s.footLock ? lockFeet(skel, take, fps, found) : null;
  const plants = lock?.plants ?? found, skate = footSkate(skel, take, fps, plants);
  const built = buildClips(skel, take, fps, plants, s.clips ?? {});
  const library = completeLibrary(skel, take, fps, built.clips, s);
  const camera = { focal: cam.focal, width: cam.width, height: cam.height, q: tf.q, offset: tf.offset, height_m: -tf.offset * scale, hfov: s.hfov, scale };
  const cert = depth && s.certify ? certifyTake({ series, cam, depth, skel, take, camera, fps, n, scaleSource, grav, height: s.height }) : null;
  return {
    depth, cert, settings: s, skel, take, fps, n, first: series.first, last: series.last, tracks, track, series, camera, floor, landmarks: P, plants,
    clips: built.clips, segments: built.segments, library,
    report: { frames: n, seconds: n / fps, tracks: tracks.length, coverage: series.valid.reduce((a, v) => a + v, 0) / n, spikes: cleaned.spikes, floor: floor.method, floorPoints: floor.points, ground, lock, plants: plants.length, sliding: plants.filter(p => p.sliding).length, skate: skate.length ? Math.max(...skate.map(x => x.drift)) : 0, ms: Date.now() - t0, depth: cert?.summary ?? (depth ? { on: true, scaleSource, kappa: depth.kappa, ms: depth.ms, line: '' } : undefined) },
  };
}

export function completeLibrary(skel, take, fps, clips, o = {}) {
  const lib = pickLibrary(clips);
  const stand = standingFrame(skel, take, fps);
  const base = stand?.still ? take.rot.slice(stand.frame * NB * 4, (stand.frame + 1) * NB * 4) : standLocal();
  if (stand?.still) {
    const f = qrot(getQ(base, 0), [0, 0, 1]);
    setQ(base, 0, qmul(qyaw(-Math.atan2(f[0], f[2])), getQ(base, 0)));
  }
  const rootY = stand?.still ? getV(take.root, stand.frame)[1] : skel.rest[0][1] - 0.005;
  if (!lib.idle) lib.idle = { ...idleClip(skel, { fps, local: base, root: [0, rootY, 0] }), name: 'Idle' };
  const retime = (c, kind, speed, derived) => { const f = speed / c.speed; return { ...c, kind, name: kind === 'run' ? 'Run' : 'Walk', fps: c.fps * f, cycle: c.cycle / f, speed, derived, source: c }; };
  if (!lib.walk && lib.run) lib.walk = retime(lib.run, 'walk', Math.min(1.4, lib.run.speed * 0.45), 'slowed run');
  if (!lib.run && lib.walk) lib.run = retime(lib.walk, 'run', Math.max(defaultSpeed(skel, 'run'), lib.walk.speed * 2.2), 'sped-up walk');
  if (!lib.walk) lib.walk = { ...gaitClip(skel, { kind: 'walk', fps, speed: lib.run ? Math.min(1.4, lib.run.speed * 0.45) : defaultSpeed(skel, 'walk') }), name: 'Walk' };
  if (!lib.run) lib.run = { ...gaitClip(skel, { kind: 'run', fps, speed: Math.max(defaultSpeed(skel, 'run'), lib.walk.speed * 2.2) }), name: 'Run' };
  if (!lib.jump) lib.jump = { ...jumpClip(skel, { fps, local: base, root: [0, rootY, 0] }), name: 'Jump' };
  return lib;
}
