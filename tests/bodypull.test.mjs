import test from 'node:test';
import assert from 'node:assert/strict';
import { exists, json, v3 } from './helpers.mjs';
import { pullOffsets } from '../wire/bodypull.js';

const POSE = [
  [10, 95, 0], [10, 52, 6], [10, 9, 0],
  [-10, 95, 0], [-10, 52, 6], [-10, 9, 0],
  [18, 145, 0], [22, 120, 8], [24, 98, 14],
];
const NB = POSE.length;
const HAND = { shoulder: 6, elbow: 7, hand: 8 };
const LEGS = { l: { thigh: 0, calf: 1, foot: 2 }, r: { thigh: 3, calf: 4, foot: 5 } };

function makeW(n, pose = POSE) {
  const W = new Float32Array(n * NB * 16);
  for (let f = 0; f < n; f++) for (let i = 0; i < NB; i++) {
    const o = (f * NB + i) * 16;
    W[o] = W[o + 5] = W[o + 10] = W[o + 15] = 1;
    W[o + 12] = pose[i][0]; W[o + 13] = pose[i][1]; W[o + 14] = pose[i][2];
  }
  return W;
}

const P = (W, nb, f, i) => { const o = (f * nb + i) * 16 + 12; return [W[o], W[o + 1], W[o + 2]]; };
const finiteAll = offs => offs.every(o => o === null || o.every(Number.isFinite));
const horiz = o => Math.hypot(o[0], o[2]);

function run(goal, o = {}) {
  const n = o.n ?? 30;
  return pullOffsets({
    W: makeW(n), nb: NB, n, frames: o.frames ?? [0, n - 1], hand: HAND, handGoal: typeof goal === 'function' ? goal : () => goal,
    legs: LEGS, feetPlanted: () => o.planted ?? false, gravity: o.gravity ?? 0.6, pull: o.pull ?? 1, ...(o.extra ?? {}),
  });
}

function legOk(W, nb, f, ids, off) {
  const top = v3.add(P(W, nb, f, ids.thigh), off), knee = P(W, nb, f, ids.calf), foot = P(W, nb, f, ids.foot);
  const L = v3.dist(P(W, nb, f, ids.thigh), knee) + v3.dist(knee, foot);
  const d0 = v3.dist(P(W, nb, f, ids.thigh), foot);
  const d = v3.dist(top, foot);
  const h = top[1] - foot[1], h0 = P(W, nb, f, ids.thigh)[1] - foot[1];
  return d <= Math.max(0.995 * L, d0) + 1e-4 && h >= Math.min(0.3 * L, h0) - 1e-4;
}

test('hand goal already reached: no offset on any frame', () => {
  const offs = run(POSE[8], { planted: true });
  assert.equal(offs.length, 30);
  assert.ok(offs.every(o => o === null));
});

test('pull 0 or empty range: no offset', () => {
  assert.ok(run([90, 120, 40], { pull: 0 }).every(o => o === null));
  assert.ok(run([90, 120, 40], { frames: [10, 5] }).every(o => o === null));
});

test('out-of-reach goal: pelvis moves toward it, gravity strips the horizontal part proportionally', () => {
  const goal = [110, 98, 14];
  const h = {}, y = {};
  for (const g of [0, 0.25, 0.5, 0.75, 1]) {
    const o = run(goal, { gravity: g })[15];
    assert.ok(o && o.every(Number.isFinite), `g=${g}`);
    h[g] = horiz(o); y[g] = o[1];
  }
  assert.ok(h[0] > 5, `h0 ${h[0]}`);
  const o0 = run(goal, { gravity: 0 })[15];
  assert.ok(o0[0] > 0 && Math.abs(o0[2]) < o0[0], String(o0));
  assert.ok(h[0] > h[0.25] && h[0.25] > h[0.5] && h[0.5] > h[0.75] && h[0.75] > h[1], JSON.stringify(h));
  assert.ok(h[1] < 1e-9, `g=1 leaves ${h[1]}`);
  const one = run(goal, { gravity: 0, extra: { iterations: -1 } })[15];
  const half = run(goal, { gravity: 0.5, extra: { iterations: -1 } })[15];
  assert.ok(Math.abs(horiz(half) / horiz(one) - 0.5) < 0.1, `${horiz(half)} vs ${horiz(one)}`);
});

test('gravity keeps the vertical part: a high sideways goal lifts the hips at g = 1', () => {
  const o = run([80, 200, 14], { gravity: 1 })[15];
  assert.ok(o[1] > 5, String(o));
  assert.ok(horiz(o) < 1e-9, String(o));
});

test('planted legs never get overstretched and resist the pull', () => {
  const W = makeW(30);
  for (const goal of [[24, 260, 14], [140, 98, 14], [24, 98, 140], [24, -40, 14], [120, 220, -90]]) {
    for (const g of [0, 0.6, 1]) {
      const free = run(goal, { gravity: g })[15];
      const offs = run(goal, { gravity: g, planted: true });
      assert.ok(finiteAll(offs));
      offs.forEach((o, f) => { if (o) for (const s of ['l', 'r']) assert.ok(legOk(W, NB, f, LEGS[s], o), `${goal} g=${g} f=${f} ${s}`); });
      const pl = offs[15];
      if (free && pl) assert.ok(v3.len(pl) <= v3.len(free) + 1e-6, `${goal}: planted ${v3.len(pl)} > free ${v3.len(free)}`);
    }
  }
});

test('range easing: zero outside, smaller at the ends than in the middle', () => {
  const offs = run([110, 98, 14], { n: 40, frames: [10, 30], gravity: 0 });
  for (let f = 0; f < 40; f++) if (f < 10 || f > 30) assert.equal(offs[f], null);
  assert.ok(horiz(offs[10]) < horiz(offs[20]) && horiz(offs[30]) < horiz(offs[20]));
  assert.ok(Math.abs(horiz(offs[14]) - horiz(offs[20])) < 1e-9);
});

test('non-finite goals are skipped without NaN', () => {
  const offs = run(f => f % 2 ? [NaN, 0, 0] : [110, 98, 14]);
  assert.ok(finiteAll(offs));
  assert.equal(offs[1], null);
  assert.ok(offs[2]);
});

const FRT = 'C:/Projects/Fulcrum/exports/FRT_fulcrum_v6.frames.json';
test('FRT v6 take: no NaN, no-op at the current hand, planted legs stay reachable', { skip: exists(FRT) ? false : 'missing FRT frames' }, () => {
  const j = json(FRT), nb = j.bones.length, n = j.frames.length;
  const W = new Float32Array(n * nb * 16);
  j.frames.forEach((fr, f) => W.set(fr, f * nb * 16));
  const id = k => { const i = j.bones.indexOf(k); assert.ok(i >= 0, k); return i; };
  const hand = { shoulder: id('upperarm_l'), elbow: id('lowerarm_l'), hand: id('hand_l') };
  const legs = { l: { thigh: id('thigh_l'), calf: id('calf_l'), foot: id('foot_l') }, r: { thigh: id('thigh_r'), calf: id('calf_r'), foot: id('foot_r') } };
  let ground = Infinity;
  for (let f = 0; f < n; f++) for (const s of ['l', 'r']) ground = Math.min(ground, P(W, nb, f, legs[s].foot)[1]);
  const upm = 91.44;
  const planted = (f, s) => P(W, nb, f, legs[s].foot)[1] - ground < 0.1 * upm;
  const base = { W, nb, n, frames: [0, n - 1], hand, legs, feetPlanted: planted };
  const none = pullOffsets({ ...base, handGoal: f => P(W, nb, f, hand.hand) });
  assert.ok(none.every(o => o === null));
  const reachGap = 0.35 * upm;
  const offs = pullOffsets({ ...base, handGoal: f => v3.add(P(W, nb, f, hand.hand), [0, -reachGap, reachGap]) });
  assert.ok(finiteAll(offs));
  const moved = offs.filter(Boolean);
  assert.ok(moved.length > n * 0.9, `${moved.length}/${n}`);
  offs.forEach((o, f) => {
    if (!o) return;
    for (const s of ['l', 'r']) if (planted(f, s)) assert.ok(legOk(W, nb, f, legs[s], o), `f${f} ${s}`);
    assert.ok(v3.len(o) < reachGap * 1.5, `f${f} ${v3.len(o)}`);
  });
});

test('pullHips: hand pull alone leaves the pelvis, pullHips scales the hips share (HumanIK EffectorSetToCharacterBodyPull)', () => {
  const goal = [70, 150, 60];
  assert.ok(run(goal, { extra: { pullHips: 0 } }).every(o => o === null));
  const full = run(goal), half = run(goal, { extra: { pullHips: 0.5 } });
  const f = 15;
  assert.ok(full[f] && half[f]);
  assert.ok(Math.abs(v3.len(half[f]) - 0.5 * v3.len(full[f])) < 1e-6);
});
