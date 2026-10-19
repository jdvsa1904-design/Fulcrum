import test from 'node:test';
import assert from 'node:assert/strict';
import { json, exists, near, nearVec, v3, q4, deg } from './helpers.mjs';
import { pullPasses, neckPullLength, neckPullForce, headPull, bodyPullPassWeights, spineShares, spineReachDeltaQ, distributeSpine, bodyEffectorReach, PULL_GAINS } from '../re/hik/pull.js';

const DIR = 'C:/re_work/hik/results/';
const skip = f => exists(DIR + f) ? false : `missing ${f}`;
const FEET = [{ hipOffset: [10, -5, 0], ankle: [10, 8, 0] }, { hipOffset: [-10, -5, 0], ankle: [-10, 8, 0] }];

test('pullPasses: N = round(count) + 2 and exponent max(1, 0.2 N)', () => {
  assert.equal(pullPasses(10).passes, 12);
  near(pullPasses(10).exponent, 2.4, 1e-12);
  assert.equal(pullPasses(2).passes, 4);
  assert.equal(pullPasses(2).exponent, 1);
  assert.equal(pullPasses(9.6).passes, 12);
  assert.equal(PULL_GAINS.neckHips[0] + PULL_GAINS.neckHips[1], 1);
});

test('neck pull: length is the chain sum, force is zero at rest and grows past full extension', () => {
  const { L0, Lmax } = neckPullLength([[0, 150, 0], [0, 155, 1], [0, 160, 0]]);
  near(Lmax, 2 * Math.hypot(5, 1), 1e-12);
  near(L0, 10, 1e-12);
  nearVec(neckPullForce([0, 150, 0], [0, 160, 0], 10, 10, 1), [0, 0, 0], 1e-12);
  const f1 = neckPullForce([0, 150, 0], [0, 170, 0], 10, 10, 1), f2 = neckPullForce([0, 150, 0], [0, 180, 0], 10, 10, 1);
  assert.ok(f1[1] > 0 && f2[1] > f1[1]);
  nearVec(neckPullForce([0, 150, 0], [0, 180, 0], 10, 10, 0), [0, 0, 0], 0);
});

test('head_pull.json: decoded body pull (no fitted constants) matches HumanIK (hips <= 3.1 cm, mean <= 0.9; neck <= 1.9; head <= 1.8)', { skip: skip('head_pull.json') }, () => {
  const rows = json(DIR + 'head_pull.json').filter(r => r.pull > 0);
  assert.equal(rows.length, 96);
  let mh = 0, mn = 0, mhd = 0, sh = 0;
  for (const r of rows) {
    const o = headPull({ hips: [0, 100, 0], neck: [0, 150, 0], head: [0, 160, 0], spine: [[0, 100, 0], [0, 110, 0], [0, 125, 0], [0, 150, 0]], goal: r.goal, pull: r.pull, resist: r.neckStiffness, feet: r.feet ? FEET : [] });
    const eh = v3.dist(o.hips, r.out.Hips[0]);
    mh = Math.max(mh, eh); sh += eh;
    mn = Math.max(mn, v3.dist(o.neck, r.out.Neck[0]));
    mhd = Math.max(mhd, v3.dist(o.head, r.out.Head[0]));
  }
  console.log(`head pull: hips max ${mh.toFixed(3)} mean ${(sh / rows.length).toFixed(3)}, neck max ${mn.toFixed(3)}, head max ${mhd.toFixed(3)} cm`);
  assert.ok(mh <= 3.1 && sh / rows.length <= 0.9, `hips ${mh}`);
  assert.ok(mn <= 1.9, `neck ${mn}`);
  assert.ok(mhd <= 1.8, `head ${mhd}`);
});

test('bodyPullPassWeights: hand pull bends the spine only; hips move with pullHips', () => {
  const off = bodyPullPassWeights({ armPulls: [1, 0] });
  near(off.armHips[0], 0, 0);
  near(off.armChest[0], 1, 0);
  near(off.chestDen, 2, 0);
  const on = bodyPullPassWeights({ armPulls: [1, 0], handPullHips: [0.5, 0] });
  near(on.armHips[0], 0.5, 1e-12);
  const neck = bodyPullPassWeights({ neckPull: 0.5, legPulls: [1, 1] });
  near(neck.neckHips, 0.5, 1e-12);
  near(neck.hipsDen, 2.5, 1e-12);
  const pinned = bodyPullPassWeights({ neckPull: 1, hipsT: 1, hipsPull: 1 });
  near(pinned.neckHips, 0, 0);
});

test('head_pull.json: zero pull leaves the body and only aims the neck', { skip: skip('head_pull.json') }, () => {
  for (const r of json(DIR + 'head_pull.json').filter(x => x.pull === 0)) {
    const o = headPull({ hips: [0, 100, 0], neck: [0, 150, 0], head: [0, 160, 0], goal: r.goal, pull: 0 });
    nearVec(o.hips, r.out.Hips[0], 1e-3);
    nearVec(o.head, r.out.Head[0], 2e-3);
  }
});

test('spineShares: base gets b/(b+N), top bone reaches the full angle, stiffness loads the base', () => {
  const s0 = spineShares(3, 0);
  assert.equal(s0.length, 5);
  near(s0[0], 0.5 / 4.5, 1e-12);
  near(s0[1], 1.5 / 4.5, 1e-12);
  near(s0[s0.length - 1], 1, 1e-12);
  for (let i = 1; i < s0.length; i++) assert.ok(s0[i] > s0[i - 1]);
  const s1 = spineShares(3, 1);
  assert.ok(s1[0] > 0.6 && s1[0] < 0.7);
  const q = spineReachDeltaQ(0.6, [1, 0, 0], 3, 0.5);
  near(deg(q4.angle(q[q.length - 1], [0, 0, 0, 1])), deg(0.6), 1e-9);
});

test('distributeSpine keeps segment lengths and turns the chest by the full delta', () => {
  const pts = [[0, 100, 0], [0, 110, 0], [0, 125, 0], [0, 150, 0]], qs = pts.map(() => [0, 0, 0, 1]);
  const d = q4.axis([1, 0, 0], 0.4);
  const o = distributeSpine(pts, qs, d, 0.3);
  for (let i = 1; i < pts.length; i++) near(v3.dist(o.points[i], o.points[i - 1]), v3.dist(pts[i], pts[i - 1]), 1e-9);
  near(deg(q4.angle(o.quats[o.quats.length - 1], d)), 0, 1e-9);
  nearVec(o.points[0], pts[0], 0);
});

test('body_eff.json: Hips/ChestEnd reach gains reproduce HumanIK (free body <= 0.07, planted horizontal along the offset <= 0.01)', { skip: skip('body_eff.json') }, () => {
  const d = json(DIR + 'body_eff.json');
  const key = { Hips: 'hips', ChestEnd: 'chestEnd', ChestOrigin: 'chestOrigin' };
  let worst = 0;
  for (const r of d.rows.filter(x => !x.feet)) {
    const off = r.d;
    const o = bodyEffectorReach({ effector: key[r.eff], offset: off, T: r.T, pull: r.pull, planted: r.feet });
    const L = v3.len(off);
    const dh = v3.sub(r.out.Hips[0], [0, 100, 0]);
    worst = Math.max(worst, v3.dist(o.hips, dh) / L);
  }
  console.log(`body effector reach: worst hips error ${worst.toFixed(3)} x offset`);
  assert.ok(worst <= 0.07, `${worst}`);
  const horiz = d.rows.filter(r => r.d[0] === 0 && r.d[1] === 0);
  assert.equal(horiz.length, 36);
  let wh = 0;
  for (const r of horiz) {
    const o = bodyEffectorReach({ effector: key[r.eff], offset: r.d, T: r.T, pull: r.pull, planted: r.feet });
    const u = v3.unit(r.d);
    wh = Math.max(wh, Math.abs(v3.dot(v3.sub(o.hips, v3.sub(r.out.Hips[0], [0, 100, 0])), u)) / v3.len(r.d));
  }
  assert.ok(wh <= 0.01, `horizontal ${wh}`);
});

test('ChestOrigin translation has no reach effect', () => {
  const o = bodyEffectorReach({ effector: 'chestOrigin', offset: [0, 0, 20], T: 1, pull: 1 });
  nearVec(o.hips, [0, 0, 0], 0);
  nearVec(o.chest, [0, 0, 0], 0);
});
