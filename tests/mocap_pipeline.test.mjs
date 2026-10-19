import test from 'node:test';
import assert from 'node:assert/strict';
import { near, rng } from './helpers.mjs';
import * as M from '../mocap/math.js';
import * as S from '../mocap/skeleton.js';
import { gaitClip, idleClip, jumpClip } from '../mocap/gait.js';
import { createController, recordTake, stepController } from '../mocap/controller.js';
import { lookAtCamera, synthCapture, staticPerson, toReconstructionFrame } from '../mocap/synth.js';
import { processCapture } from '../mocap/pipeline.js';
import { takeWorld, worldAt } from '../mocap/ground.js';
import { toBVH, parseBVH } from '../mocap/bvh.js';

const truthSkel = S.buildSkeleton({ ...S.DEFAULT_DIMS, thigh: 0.44, shin: 0.43, torso: 0.52 });
const truthLib = () => ({ idle: idleClip(truthSkel), walk: gaitClip(truthSkel, { kind: 'walk', speed: 1.25 }), run: gaitClip(truthSkel, { kind: 'run', speed: 3.45 }), jump: jumpClip(truthSkel), actions: [], take: null });
const cam = lookAtCamera([0.3, 1.45, 4.5], [0, 0.85, -0.5], { width: 1280, height: 720, hfov: 60 });

function scenario(steps, seed, o = {}) {
  const lib = truthLib(), ctl = createController(lib, { yaw: Math.PI / 2 });
  ctl.pos = [-3, 0, -0.5];
  const take = recordTake(ctl, steps.map(s => ({ ...s, input: { ...s.input, camYaw: Math.PI } })), 30);
  const { capture } = synthCapture(truthSkel, take, cam, { rand: rng(seed), noiseImg: 0.003, noiseWorld: 0.015, occlusion: true, ...o });
  return { lib, take, capture };
}

function mpjpe(res, take) {
  const TW = takeWorld(res.skel, res.take), tmp = { pos: new Float64Array(S.NB * 3), q: new Float64Array(S.NB * 4) };
  const joints = [S.B.Hips, S.B.LeftLeg, S.B.RightLeg, S.B.LeftFoot, S.B.RightFoot, S.B.LeftForeArm, S.B.RightForeArm, S.B.LeftHand, S.B.RightHand, S.B.Head];
  let sum = 0, cnt = 0;
  for (let i = 0; i < res.n; i++) {
    const f = res.first + i;
    S.fk(truthSkel, S.getV(take.root, f), take.rot.subarray(f * S.NB * 4, (f + 1) * S.NB * 4), tmp);
    for (const j of joints) { sum += M.dist(toReconstructionFrame(cam, S.getV(tmp.pos, j)), S.getV(worldAt(TW, i).pos, j)); cnt++; }
  }
  return sum / cnt;
}

test('pipeline: a noisy video of idle, walk, idle becomes a measured walk loop, idle and the full take', () => {
  const { lib, take, capture } = scenario([{ seconds: 1.5, input: {} }, { seconds: 4.5, input: { right: true } }, { seconds: 1.5, input: {} }], 3);
  const res = processCapture(capture, { hfov: 60 });
  assert.ok(mpjpe(res, take) < 0.06, `joint error ${mpjpe(res, take)}`);
  near(res.camera.height_m, cam.pos[1], 0.1, 'camera height');
  near(res.skel.dims.height, truthSkel.dims.height, 0.05, 'body height');
  const walk = res.library.walk;
  assert.equal(walk.kind, 'walk');
  assert.ok(!walk.procedural, 'the walk comes from the video');
  near(walk.speed, lib.walk.speed, 0.08, 'walk speed');
  near(walk.cycle, lib.walk.cycle, 2 / 30, 'walk cycle');
  assert.ok(res.library.idle && !res.library.idle.procedural, 'idle comes from the video');
  assert.ok(res.library.run.procedural && res.library.jump.procedural, 'missing moves are filled in');
  assert.ok(res.library.take.n === res.n);
  for (const c of res.clips) assert.ok(c.rot.every(Number.isFinite) && c.root.every(Number.isFinite), c.name);
  assert.ok(res.report.skate < 0.01, `foot skate ${res.report.skate}`);
});

test('pipeline: walking then running yields both loops at the right speeds', () => {
  const { lib, capture } = scenario([{ seconds: 1.5, input: {} }, { seconds: 2, input: { right: true } }, { seconds: 2.5, input: { right: true, run: true } }, { seconds: 1.5, input: {} }], 5);
  const res = processCapture(capture, { hfov: 60 });
  assert.ok(!res.library.walk.procedural && !res.library.run.procedural);
  near(res.library.walk.speed, lib.walk.speed, 0.12, 'walk');
  near(res.library.run.speed, lib.run.speed, 0.25, 'run');
  near(res.library.run.cycle, lib.run.cycle, 2 / 30, 'run cycle');
});

test('pipeline: a bystander is ignored, and choosing them by click switches the subject', () => {
  const statue = staticPerson(truthSkel, cam, [2.2, 0, -5.5], { local: S.standLocal() });
  const { capture } = scenario([{ seconds: 1, input: {} }, { seconds: 3, input: { right: true } }], 7, { extraPerson: statue, shuffle: true });
  const res = processCapture(capture, { hfov: 60 });
  assert.equal(res.tracks.length, 2);
  assert.ok(!res.library.walk.procedural, 'the walker was picked');
  const p = statue(), pt = [(p.img[S.LM.lHip * 5] + p.img[S.LM.rHip * 5]) / 2, (p.img[S.LM.lHip * 5 + 1] + p.img[S.LM.rHip * 5 + 1]) / 2];
  const res2 = processCapture(capture, { hfov: 60, person: { point: pt, frame: 20 } });
  assert.notEqual(res2.track.id, res.track.id);
  assert.ok(res2.library.walk.procedural, 'the bystander never walks');
});

test('pipeline: the result drives the WASD controller and exports a valid BVH', () => {
  const { capture } = scenario([{ seconds: 1.5, input: {} }, { seconds: 4, input: { right: true } }, { seconds: 1, input: {} }], 11);
  const res = processCapture(capture, { hfov: 60, height: 1.75 });
  near(res.skel.dims.height, 1.75, 1e-6);
  const ctl = createController(res.library);
  let out;
  for (let i = 0; i < 120; i++) out = stepController(ctl, { forward: true, camYaw: 0 }, 1 / 60);
  assert.ok(out.pos[2] > 1, 'W moved the character forward');
  for (let i = 0; i < 60; i++) out = stepController(ctl, { forward: true, run: true, camYaw: 0 }, 1 / 60);
  assert.ok(out.speed > res.library.walk.speed, 'Shift runs');
  out = stepController(ctl, { replay: true }, 1 / 60);
  assert.equal(out.state, 'action');
  const b = parseBVH(toBVH(res.skel, res.library.take));
  assert.equal(b.frames, res.n);
  assert.ok(b.values.every(row => row.every(Number.isFinite)));
});
