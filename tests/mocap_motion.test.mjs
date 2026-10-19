import test from 'node:test';
import assert from 'node:assert/strict';
import { near, rng } from './helpers.mjs';
import * as M from '../mocap/math.js';
import * as S from '../mocap/skeleton.js';
import { gaitClip, idleClip, jumpClip, cycleTime } from '../mocap/gait.js';
import { floorFromPoints, takeWorld, worldAt, soleAt, findPlants, lockFeet, footSkate, groundTake } from '../mocap/ground.js';
import { findLoop, poseFeatures, facing, makeLoopClip, segmentTake, oneShotClip } from '../mocap/cycle.js';
import { createController, stepController, recordTake, moveIntent, sampleClip, inputFromKeys } from '../mocap/controller.js';
import { toBVH, parseBVH } from '../mocap/bvh.js';
import { quatFromEuler } from '../filters.js';

const skel = S.buildSkeleton();
const lib = () => ({ idle: idleClip(skel), walk: gaitClip(skel, { kind: 'walk', speed: 1.3 }), run: gaitClip(skel, { kind: 'run', speed: 3.4 }), jump: jumpClip(skel), actions: [], take: null });

function lowest(clip) {
  const TW = takeWorld(skel, clip);
  let lo = Infinity;
  for (let i = 0; i < clip.n; i++) for (const side of ['L', 'R']) { const s = soleAt(skel, worldAt(TW, i), side); lo = Math.min(lo, s.heel[1], s.ball[1], s.toe[1]); }
  return lo;
}

test('gait: walk and run loops close seamlessly, keep feet on the floor and match the asked speed', () => {
  for (const [kind, v] of [['walk', 1.3], ['run', 3.4]]) {
    const c = gaitClip(skel, { kind, speed: v, fps: 30 });
    near(c.cycle, Math.round(cycleTime(v) * 30) / 30, 1e-9, kind + ' cycle');
    assert.ok(lowest(c) > -0.012, `${kind} sole below floor ${lowest(c)}`);
    let seam = 0, inner = 0;
    for (let b = 0; b < S.NB; b++) {
      seam = Math.max(seam, M.qangle(S.getQ(c.rot, (c.n - 1) * S.NB + b), S.getQ(c.rot, b)));
      for (let f = 1; f < c.n; f++) inner = Math.max(inner, M.qangle(S.getQ(c.rot, f * S.NB + b), S.getQ(c.rot, (f - 1) * S.NB + b)));
    }
    assert.ok(seam <= inner * 1.05 + 1e-9, `${kind} loop seam ${seam} vs largest step ${inner}`);
    const TW = takeWorld(skel, c), st = c.contacts.find(x => x.side === 'L'), dt = 1 / c.fps;
    let slide = 0;
    for (let f = st.a + 2; f < st.b - 2; f++) {
      const a = S.getV(worldAt(TW, f).pos, S.B.LeftToeBase), b = S.getV(worldAt(TW, f + 1).pos, S.B.LeftToeBase);
      slide = Math.max(slide, Math.abs((b[2] - a[2]) / dt + v));
    }
    assert.ok(slide < 0.25 * v, `${kind} planted ball moves at -speed in place (off by ${slide})`);
  }
});

test('ground: a pitched camera floor is recovered from foot points', () => {
  const pitch = 0.18, q = M.qaxis([1, 0, 0], pitch), P = new Float64Array(200 * 99), r = rng(9);
  for (let i = 0; i < 200; i++) {
    const step = Math.floor(i / 10), x = -2 + 4 * step / 19, z = -3 - 1.5 * Math.sin(step / 2);
    for (let j = 0; j < 33; j++) {
      const y = j >= 29 ? 0.01 * r() : 0.5 + j * 0.03;
      P.set(M.add(M.qrot(q, [x + 0.01 * j, y, z]), [0, -1.4, 0]), (i * 33 + j) * 3);
    }
  }
  const fl = floorFromPoints(P, 200, 30, { prior: [0, 1, 0] });
  assert.ok(M.angle(fl.normal, M.qrot(q, [0, 1, 0])) < 0.02, 'normal');
  near(fl.offset, -1.4 * Math.cos(pitch), 0.03, 'offset');
});

test('ground: foot locking removes stance skate without pulling the pelvis far', () => {
  const c = gaitClip(skel, { kind: 'walk', speed: 1.3, fps: 30 }), n = c.n * 3, r = rng(4);
  const take = { n, fps: 30, root: new Float64Array(n * 3), rot: new Float64Array(n * S.NB * 4) };
  for (let k = 0; k < n; k++) {
    const src = k % c.n;
    take.root.set([c.root[src * 3] + 0.01 * (r() - 0.5), c.root[src * 3 + 1], c.root[src * 3 + 2] + 1.3 * k / 30 + 0.04 * Math.sin(2 * Math.PI * 0.8 * k / 30) + 0.006 * (r() - 0.5)], k * 3);
    take.rot.set(c.rot.subarray(src * S.NB * 4, (src + 1) * S.NB * 4), k * S.NB * 4);
  }
  groundTake(skel, take, 30);
  const plants = findPlants(skel, take, 30);
  const before = Math.max(...footSkate(skel, take, 30, plants).map(x => x.drift));
  const res = lockFeet(skel, take, 30, plants);
  const after = Math.max(...footSkate(skel, take, 30, res.plants).map(x => x.drift));
  assert.ok(before > 0.03, `test data skates (${before})`);
  assert.ok(after < 0.005, `skate after lock ${after}`);
  assert.ok(res.maxDrop < 0.06, `pelvis drop ${res.maxDrop}`);
  assert.ok(take.rot.every(Number.isFinite));
});

test('cycle: the loop finder recovers the gait period and the loop clip runs in place at the true speed', () => {
  const c = gaitClip(skel, { kind: 'walk', speed: 1.25, fps: 30 }), n = c.n * 4;
  const take = { n, fps: 30, root: new Float64Array(n * 3), rot: new Float64Array(n * S.NB * 4) }, yaw0 = 0.7, R = M.qyaw(yaw0);
  for (let k = 0; k < n; k++) {
    const src = k % c.n, p = M.qrot(R, [c.root[src * 3], 0, c.root[src * 3 + 2] + 1.25 * k / 30]);
    take.root.set([p[0] + 2, c.root[src * 3 + 1], p[2] - 1], k * 3);
    take.rot.set(c.rot.subarray(src * S.NB * 4, (src + 1) * S.NB * 4), k * S.NB * 4);
    S.setQ(take.rot, k * S.NB, M.qmul(R, S.getQ(c.rot, src * S.NB)));
  }
  const TW = takeWorld(skel, take), yaw = facing(take, 30), feat = poseFeatures(take, TW, yaw, 30);
  const lp = findLoop(feat, 0, n - 1, 30);
  assert.equal(lp.len, c.n);
  assert.ok(lp.quality < 0.05, `quality ${lp.quality}`);
  const clip = makeLoopClip(skel, take, 30, lp.start, lp.len, [], { yaw });
  near(clip.speed, 1.25, 0.02);
  assert.equal(clip.direction, 'forward');
  let mx = 0;
  for (let k = 0; k < clip.n; k++) mx = Math.max(mx, Math.hypot(clip.root[k * 3], clip.root[k * 3 + 2]));
  assert.ok(mx < 0.1, 'root stays in place');
  const f = M.qrot(S.getQ(clip.rot, 0), [0, 0, 1]);
  assert.ok(Math.abs(Math.atan2(f[0], f[2])) < 0.15, 'clip faces +Z');
  const seg = segmentTake(take, 30);
  assert.equal(seg.segments.filter(s => s.kind === 'move').length, 1);
  const one = oneShotClip(skel, take, 30, 10, 70, { yaw });
  near(one.rootMotion.distance, 1.25 * 2, 0.08);
  assert.ok(Math.abs(one.rootMotion.end[0]) < 0.08, 'one-shot travels along +Z');
});

test('controller: WASD is camera relative, Shift runs, Space jumps and the pose never breaks', () => {
  assert.equal(moveIntent({}), null);
  near(moveIntent({ forward: true, camYaw: 0 }).yaw, 0, 1e-12);
  near(moveIntent({ right: true, camYaw: 0 }).yaw, -Math.PI / 2, 1e-12);
  near(moveIntent({ forward: true, camYaw: Math.PI / 2 }).yaw, Math.PI / 2, 1e-12);
  const L = lib(), ctl = createController(L);
  const rec = recordTake(ctl, [
    { seconds: 0.5, input: {} }, { seconds: 2, input: { forward: true } }, { seconds: 2, input: { forward: true, run: true } },
    { seconds: 1.2, input: { forward: true, run: true, jump: true } }, { seconds: 1.5, input: { left: true } }, { seconds: 1.5, input: {} }], 30);
  const at = s => rec.frames[Math.round(s * 30) - 1];
  near(at(2.5).speed, L.walk.speed, 1e-6, 'walk speed');
  near(at(4.5).speed, L.run.speed, 1e-6, 'run speed');
  assert.equal(at(4.7).state, 'jump');
  assert.ok(Math.abs(M.wrapAngle(at(6.2).yaw - Math.PI / 2)) < 0.05, 'A turns left');
  near(at(7.7).speed, 0, 1e-6, 'stops');
  assert.ok(rec.rot.every(Number.isFinite) && rec.root.every(Number.isFinite));
  let worst = 0;
  for (let i = 1; i < rec.n; i++) for (let b = 1; b < S.NB; b++) worst = Math.max(worst, M.qangle(S.getQ(rec.rot, i * S.NB + b), S.getQ(rec.rot, (i - 1) * S.NB + b)));
  assert.ok(worst < 0.7, `largest joint jump between frames ${worst}`);
});

test('controller: actions play with root motion, R replays the take, keys map to inputs', () => {
  const L = lib(), walk = L.walk, n = 60;
  const take = { n, fps: 30, root: new Float64Array(n * 3), rot: new Float64Array(n * S.NB * 4) };
  for (let k = 0; k < n; k++) { take.root.set([0, walk.root[(k % walk.n) * 3 + 1], 1.3 * k / 30], k * 3); take.rot.set(walk.rot.subarray((k % walk.n) * S.NB * 4, (k % walk.n + 1) * S.NB * 4), k * S.NB * 4); }
  L.take = { ...oneShotClip(skel, take, 30, 0, n - 1), name: 'Full take' };
  L.actions = [L.take];
  const ctl = createController(L, { yaw: Math.PI / 2 });
  ctl.pos = [1, 0, 1];
  stepController(ctl, {}, 1 / 30);
  stepController(ctl, { replay: true }, 1 / 30);
  assert.equal(ctl.state, 'action');
  for (let i = 0; i < 80; i++) stepController(ctl, {}, 1 / 30);
  assert.equal(ctl.state, 'move');
  near(ctl.pos[0], 1 + 1.3 * 59 / 30, 0.05, 'replay moved the character along its facing');
  near(ctl.pos[2], 1, 0.05);
  const keys = inputFromKeys(new Set(['KeyW', 'ShiftLeft']), new Set(['Space', 'Digit3']));
  assert.ok(keys.forward && keys.run && keys.jump && !keys.back);
  assert.equal(keys.slot, 2);
  assert.equal(inputFromKeys(new Set(), new Set(['Digit0'])).slot, 9);
  assert.equal(inputFromKeys(new Set(), new Set(['KeyQ'])).select, 1);
  assert.equal(inputFromKeys(new Set(['ShiftLeft']), new Set(['KeyQ'])).select, -1);
  assert.ok(inputFromKeys(new Set(), new Set(['Click'])).click);
  const c2 = createController({ ...L, actions: [L.take, L.walk] });
  stepController(c2, { select: 1 }, 1 / 30);
  assert.equal(c2.selected, 1);
  stepController(c2, { select: -1 }, 1 / 30); stepController(c2, { select: -1 }, 1 / 30);
  assert.equal(c2.selected, 1, 'selection wraps');
  stepController(c2, { click: true }, 1 / 30);
  assert.equal(c2.state, 'action');
  assert.equal(c2.action.clip, L.walk);
  const s = sampleClip(walk, walk.cycle * 0.25);
  assert.ok(s.rot.every(Number.isFinite));
});

test('bvh: export parses back to the same joint rotations and root path', () => {
  const c = gaitClip(skel, { kind: 'run', fps: 30 }), text = toBVH(skel, c), b = parseBVH(text);
  assert.equal(b.joints.length, S.NB);
  assert.equal(b.frames, c.n);
  near(b.frameTime, 1 / 30, 1e-6);
  assert.deepEqual(b.joints.map(j => j.name).sort(), [...S.BONES].sort());
  let worst = 0, rootErr = 0;
  for (let f = 0; f < c.n; f++) {
    let col = 0;
    for (const j of b.joints) {
      const bi = S.B[j.name], v = b.values[f];
      if (j.channels.length === 6) { rootErr = Math.max(rootErr, Math.abs(v[col] / 100 - c.root[f * 3]), Math.abs(v[col + 1] / 100 - c.root[f * 3 + 1])); col += 3; }
      const [z, x, y] = v.slice(col, col + 3).map(d => d * Math.PI / 180);
      col += 3;
      worst = Math.max(worst, M.qangle(quatFromEuler([x, y, z], 'yxz'), S.getQ(c.rot, f * S.NB + bi)));
    }
  }
  assert.ok(worst < 1e-4, `rotation error ${worst}`);
  assert.ok(rootErr < 1e-5, `root error ${rootErr}`);
  const off = b.joints.find(j => j.name === 'LeftLeg').offset;
  near(off[1], -skel.m.thigh * 100, 1e-3);
});
