import test from 'node:test';
import assert from 'node:assert/strict';
import { csv, exists, near, nearVec, v3, q4, deg } from './helpers.mjs';
import {
  DEFAULTS, RETARGET_PULL_NORM, slerp, maxExtensionGoal, quatReach, midReachLow, swivelKnee, reachRotation, limbReach,
  limbWeights, addWeightedQ, pullForce, bodyPull, bodyReachRigid, hipsHeightCompensation, ankleHeightCompensation,
  scaleCompensation, massCenterCompensation, autoScale, autoHipsHeight,
} from '../hik.js';

const DIR = 'C:/re_work/hik/results/';
const have = f => exists(DIR + f);
const data = f => csv(DIR + f);
const skip = f => have(f) ? false : `missing ${f}`;

const TOP = [10, 95, 0], KNEE = [10, 52, 0], ANKLE = [10, 8, 0];
const LEG = { top: TOP, knee: KNEE, ankle: ANKLE, hinge: [1, 0, 0] };
const parseTarget = s => JSON.parse(s.replace('(', '[').replace(')', ']'));

const arm = (goal, pull) => ({ kind: 'arm', body: 'chest', offset: [15, 20, 0], goal, pull, L0: 53, Lmax: 53 });
const legs = (ankleY = [8, 8]) => [1, -1].map((s, i) => ({ kind: 'leg', body: 'hips', offset: [10 * s, -5, 0], goal: [10 * s, ankleY[i], 0], pull: 1, T: 1, L0: 87, Lmax: 87, rc: 0.5 }));
const wristErr = (o, goal) => Math.max(0, v3.dist(goal, v3.add(o.chest, [15, 20, 0])) - 53);
const pullRun = (goal, pull, o = {}) => {
  const r = bodyPull({ hips: [0, 100, 0], chest: [0, 125, 0], limbs: [arm(goal, pull), ...(o.feet ? legs() : [])], pullIters: o.iters ?? 10, gravity: o.gravity ?? 0.6, pullNorm: o.feet ? RETARGET_PULL_NORM : undefined });
  return { err: wristErr(r, goal), hips: r.hips };
};

test('reach_t.csv: in-plane leg reach matches HumanIK (ankle <= 0.06 cm, knee <= 0.05 cm)', { skip: skip('reach_t.csv') }, () => {
  let ma = 0, mk = 0, n = 0;
  for (const r of data('reach_t.csv')) {
    const g = parseTarget(r.target);
    if (g[0] !== 10) continue;
    const o = limbReach({ ...LEG, goal: g, T: r.reachT });
    ma = Math.max(ma, v3.dist(o.ankle, [r.ax, r.ay, r.az]));
    mk = Math.max(mk, v3.dist(o.knee, [r.kx, r.ky, r.kz]));
    n++;
  }
  assert.equal(n, 22);
  assert.ok(ma <= 0.06, `ankle ${ma}`);
  assert.ok(mk <= 0.05, `knee ${mk}`);
});

test('reach_t.csv: off-plane target keeps the swing-only residual (ankle <= 0.7 cm, knee <= 0.16 cm)', { skip: skip('reach_t.csv') }, () => {
  let ma = 0, mk = 0;
  for (const r of data('reach_t.csv')) {
    const g = parseTarget(r.target);
    if (g[0] === 10) continue;
    const o = limbReach({ ...LEG, goal: g, T: r.reachT });
    ma = Math.max(ma, v3.dist(o.ankle, [r.ax, r.ay, r.az]));
    mk = Math.max(mk, v3.dist(o.knee, [r.kx, r.ky, r.kz]));
  }
  assert.ok(ma <= 0.7, `ankle ${ma}`);
  assert.ok(mk <= 0.16, `knee ${mk}`);
});

test('reach T: 0 leaves the pose, 1 lands on a reachable goal, bone lengths hold', () => {
  const rest = limbReach({ ...LEG, goal: [10, 30, 20], T: 0 });
  nearVec(rest.ankle, ANKLE, 1e-9);
  nearVec(rest.knee, KNEE, 1e-9);
  for (const g of [[10, 30, 20], [25, 20, 10], [-5, 40, 25], [10, 20, 25]]) {
    const o = limbReach({ ...LEG, goal: g, T: 1, resistKnee: 0 });
    nearVec(o.ankle, g, 1e-6, String(g));
    near(v3.dist(TOP, o.knee), 43, 1e-9);
    near(v3.dist(o.knee, o.ankle), 44, 1e-9);
  }
});

test('reach T: the ankle swings backwards at low T for a forward target (measured -12.7 cm)', () => {
  const o = limbReach({ ...LEG, goal: [10, 30, 20], T: 0.1 });
  assert.ok(o.ankle[2] < -12 && o.ankle[2] > -13.5, String(o.ankle));
});

test('resist_maxext.csv: AdjustLowEffector reproduces the prediction column exactly', { skip: skip('resist_maxext.csv') }, () => {
  const dir = [0, -Math.cos(0.3), Math.sin(0.3)];
  for (const r of data('resist_maxext.csv')) {
    const g = v3.add(TOP, v3.mul(dir, r.target_dist));
    const G = maxExtensionGoal(TOP, g, ANKLE, 87, r.resist_knee, r.resist_maxext);
    near(v3.dist(G, TOP), r.pred_dist, 1e-9);
    nearVec(v3.unit(v3.sub(G, TOP)), dir, 1e-12);
  }
});

test('resist_maxext.csv: full limb solve lands within 0.02 cm of the measured hip-ankle distance', { skip: skip('resist_maxext.csv') }, () => {
  const dir = [0, -Math.cos(0.3), Math.sin(0.3)];
  let m = 0, n = 0;
  for (const r of data('resist_maxext.csv')) {
    const g = v3.add(TOP, v3.mul(dir, r.target_dist));
    const o = limbReach({ ...LEG, goal: g, T: 1, resistKnee: r.resist_knee, maxExt: r.resist_maxext });
    m = Math.max(m, Math.abs(v3.dist(o.ankle, TOP) - r.hik_dist));
    n++;
  }
  assert.equal(n, 120);
  assert.ok(m <= 0.02, `max ${m}`);
});

test('maxExtensionGoal: zero resist is a no-op, a full-resist straight leg holds the goal back', () => {
  const g = [10, 15, 10];
  nearVec(maxExtensionGoal(TOP, g, ANKLE, 87, 0, 0.5), g, 1e-12);
  const held = maxExtensionGoal(TOP, [10, 8.5, 0], [10, 25, 0], 87, 1, 1, 1);
  near(v3.dist(held, TOP), 70, 1e-9);
  const far = maxExtensionGoal(TOP, [10, -50, 0], ANKLE, 87, 0.5, 0.5);
  assert.ok(v3.dist(far, TOP) <= 87 + 1e-9);
});

test('reach_r.csv: ankle rotation = slerp(carried swing, goal, R), max 0.35 deg over 40 cases', { skip: skip('reach_r.csv') }, () => {
  let m = 0, n = 0;
  for (const r of data('reach_r.csv')) {
    const qt = q4.axis([r.ax, r.ay, r.az], r.deg * Math.PI / 180);
    const o = limbReach({ ...LEG, goal: [10, 20, 15], T: r.reachT, qGoal: qt, R: r.reachR, restThigh: [0, -1, 0], restShin: [0, -1, 0] });
    m = Math.max(m, deg(q4.angle(o.qAnkle, [r.qx, r.qy, r.qz, r.qw])));
    n++;
  }
  assert.equal(n, 40);
  assert.ok(m <= 0.35, `max ${m} deg`);
});

test('reach R: endpoints and swing carry', () => {
  const qg = q4.axis([0, 1, 0], 1);
  const base = { top: TOP, knee: KNEE, ankle: ANKLE, restThigh: [0, -1, 0], restShin: [0, -1, 0], qGoal: qg };
  nearVec(reachRotation({ ...base, R: 1 }), qg, 1e-12);
  nearVec(reachRotation({ ...base, R: 0 }), [0, 0, 0, 1], 1e-12);
  const bent = { ...base, knee: [10, 52, 8], ankle: [10, 9, 6] };
  const c = reachRotation({ ...bent, R: 0 });
  nearVec(q4.rot(c, [0, -1, 0]), v3.unit(v3.sub(bent.ankle, bent.knee)), 1e-9);
  near(deg(q4.angle(slerp([0, 0, 0, 1], qg, 0.5), qg)), 0.5 * deg(1), 1e-9);
});

test('quatReach / midReachLow primitives', () => {
  const q = quatReach(TOP, ANKLE, [10, 30, 20], 1);
  const a = v3.add(TOP, q4.rot(q, v3.sub(ANKLE, TOP)));
  nearVec(v3.unit(v3.sub(a, TOP)), v3.unit(v3.sub([10, 30, 20], TOP)), 1e-12);
  nearVec(quatReach(TOP, ANKLE, [10, 30, 20], 0), [0, 0, 0, 1], 1e-12);
  const m = midReachLow(TOP, KNEE, ANKLE, [10, 30, 0], 1, [1, 0, 0]);
  near(v3.dist(m.ankle, TOP), 65, 1e-9);
  const neg = midReachLow(TOP, KNEE, [10, 30, -20], [10, 30, 0], 1, [1, 0, 0]);
  assert.ok(Math.sign(neg.angle) !== 0);
});

test('swivelKnee moves the knee into the pole plane without moving the ankle', () => {
  const k = swivelKnee(TOP, [14, 52, 10], ANKLE, [60, 60, 0]);
  const H = v3.unit(v3.sub(ANKLE, TOP));
  const kp = v3.sub(v3.sub(k, TOP), v3.mul(H, v3.dot(v3.sub(k, TOP), H)));
  const pp = v3.sub(v3.sub([60, 60, 0], TOP), v3.mul(H, v3.dot(v3.sub([60, 60, 0], TOP), H)));
  nearVec(v3.unit(kp), v3.unit(pp), 1e-9);
  near(v3.dist(k, TOP), v3.dist([14, 52, 10], TOP), 1e-9);
});

test('limbWeights cascade (section 2.1)', () => {
  const w = limbWeights({ Tmid: 0, Pmid: 0, Tlow: 0.3, Plow: 0, endEnabled: false });
  near(w.low, 0.3, 1e-12);
  near(w.mid, 0, 1e-12);
  near(w.end, 0, 1e-12);
  const full = limbWeights({ Tmid: 1, Pmid: 1, Tlow: 1, Plow: 1 });
  near(full.mid, 1, 1e-12);
  near(full.low, 0, 1e-12);
  const h = limbWeights({ Tmid: 0.5, Pmid: 1, Tlow: 1, Plow: 1, Tend: 1, Pend: 1, endEnabled: true });
  near(h.mid + h.low + h.end, 1, 1e-12);
  near(h.mid, 0.5, 1e-12);
});

test('addWeightedQ is sign-aligned and normalised', () => {
  const a = q4.axis([0, 0, 1], 0.4), b = a.map(x => -x);
  nearVec(addWeightedQ([[a, 0.5], [b, 0.5]]), a, 1e-12);
  const q = addWeightedQ([[[0, 0, 0, 1], 1], [q4.axis([0, 1, 0], 1), 1]]);
  near(Math.hypot(...q), 1, 1e-12);
});

test('pullForce: linear spring, compression gain, continuous cubic bridge, 1:1 overflow', () => {
  near(pullForce(50, 50, 87, 1, 0.5), 0, 1e-12);
  near(pullForce(60, 50, 87, 0.5, 0.5), 5, 1e-12);
  near(pullForce(40, 50, 87, 0.5, 0.5), -2.5, 1e-12);
  near(pullForce(87, 87, 87, 1, 0.5), 0, 1e-12);
  near(pullForce(100, 87, 87, 0.2, 0.5), 13, 1e-12);
  const L0 = 50, Lmax = 87, p = 0.5, k = p;
  const L5 = Lmax - (Lmax - L0) * k, L3 = ((Lmax - L5) * 1.5 + L5) * (1 - k) + k * Lmax;
  near(pullForce(L5, L0, Lmax, p, 0.5), (L5 - L0) * k, 1e-9);
  near(pullForce(L3, L0, Lmax, p, 0.5), (L3 - Lmax) * (1 - k) + (L3 - L0) * k, 1e-9);
  let prev = -Infinity;
  for (let L = L0; L <= 130; L += 0.25) {
    const f = pullForce(L, L0, Lmax, p, 0.5);
    assert.ok(f >= prev - 1e-9, `monotone at ${L}`);
    assert.ok(Math.abs(f - pullForce(L + 1e-6, L0, Lmax, p, 0.5)) < 1e-4, `continuous at ${L}`);
    prev = f;
  }
  near(pullForce(120, L0, Lmax, p, 0.5), (120 - Lmax) * 0.5 + (120 - L0) * 0.5, 1e-9);
});

test('pull_iters.csv: free-hand pull converges like HumanIK (wrist <= 4.5 cm, hips x <= 4.0 cm of measured)', { skip: skip('pull_iters.csv') }, () => {
  let me = 0, mh = 0;
  const g = [120, 145, 0];
  for (const r of data('pull_iters.csv')) {
    const m = pullRun(g, r.pull, { iters: r.iters });
    me = Math.max(me, Math.abs(m.err - r.wrist_err));
    mh = Math.max(mh, Math.abs(m.hips[0] - r.hx));
  }
  assert.ok(me <= 4.5, `wrist ${me}`);
  assert.ok(mh <= 4.0, `hips ${mh}`);
});

test('pull_iters.csv: error never grows with more passes and shrinks with more pull (model and measured)', { skip: skip('pull_iters.csv') }, () => {
  const rows = data('pull_iters.csv');
  for (const p of [0.25, 0.5, 0.75, 1]) {
    const rs = rows.filter(r => r.pull === p);
    for (let i = 1; i < rs.length; i++) {
      assert.ok(rs[i].wrist_err <= rs[i - 1].wrist_err + 1e-3);
      assert.ok(pullRun([120, 145, 0], p, { iters: rs[i].iters }).err <= pullRun([120, 145, 0], p, { iters: rs[i - 1].iters }).err + 1e-9);
    }
  }
  for (const n of [0, 5, 10, 30]) {
    let prev = Infinity;
    for (const p of [0, 0.25, 0.5, 0.75, 1]) {
      const e = pullRun([120, 145, 0], p, { iters: n }).err;
      assert.ok(e <= prev + 1e-9);
      prev = e;
    }
  }
});

test('pull_iters.csv: pass count is round(iters) + 2', () => {
  const r = bodyPull({ hips: [0, 100, 0], chest: [0, 125, 0], limbs: [arm([120, 145, 0], 1)], pullIters: 10, trace: true });
  assert.equal(r.passes, 12);
  assert.equal(r.trace.length, 12);
  assert.equal(bodyPull({ hips: [0, 100, 0], chest: [0, 125, 0], limbs: [], pullIters: 3.4 }).passes, 5);
});

test('pull_sweep.csv: free-feet rows (wrist err <= 5.5 cm, hips x/z <= 3.5 cm, hips y <= 5.5 cm of measured)', { skip: skip('pull_sweep.csv') }, () => {
  let me = 0, mx = 0, my = 0, n = 0;
  for (const r of data('pull_sweep.csv')) {
    if (r.feet_pinned) continue;
    const m = pullRun([r.tx, r.ty, r.tz], r.pull);
    me = Math.max(me, Math.abs(m.err - r.wrist_err));
    mx = Math.max(mx, Math.abs(m.hips[0] - r.hx), Math.abs(m.hips[2] - r.hz));
    my = Math.max(my, Math.abs(m.hips[1] - r.hy));
    n++;
  }
  assert.equal(n, 20);
  assert.ok(me <= 5.5, `wrist ${me}`);
  assert.ok(mx <= 3.5, `hips xz ${mx}`);
  assert.ok(my <= 5.5, `hips y ${my}`);
});

test('pull_sweep.csv: pinned ankles hold the body back (qualitative only, spine lean is not modelled)', { skip: skip('pull_sweep.csv') }, () => {
  for (const r of data('pull_sweep.csv')) {
    if (!r.feet_pinned || r.pull === 0) continue;
    const free = pullRun([r.tx, r.ty, r.tz], r.pull);
    const pin = pullRun([r.tx, r.ty, r.tz], r.pull, { feet: true });
    assert.ok(pin.err >= free.err - 1e-9);
    assert.ok(Math.hypot(pin.hips[0], pin.hips[2]) <= Math.hypot(free.hips[0], free.hips[2]) + 1e-9);
  }
});

test('gravity.csv: free-feet hips follow measured for g <= 0.75 (xz <= 3.9 cm for g <= 0.75, y <= 6 cm for g <= 0.6)', { skip: skip('gravity.csv') }, () => {
  let mx = 0, my = 0, n = 0;
  for (const r of data('gravity.csv')) {
    if (r.feet || r.gravity > 0.75) continue;
    const m = pullRun([r.tx, r.ty, r.tz], 1, { gravity: r.gravity });
    mx = Math.max(mx, Math.abs(m.hips[0] - r.hx), Math.abs(m.hips[2] - r.hz));
    if (r.gravity <= 0.6) my = Math.max(my, Math.abs(m.hips[1] - r.hy));
    n++;
  }
  assert.equal(n, 20);
  assert.ok(mx <= 3.9, `xz ${mx}`);
  assert.ok(my <= 6, `y ${my}`);
});

test('gravity.csv: g = 1 removes the horizontal hand pull on the hips (measured <= 3.6 cm left over)', { skip: skip('gravity.csv') }, () => {
  for (const r of data('gravity.csv')) {
    if (r.feet || r.gravity !== 1) continue;
    const m = pullRun([r.tx, r.ty, r.tz], 1, { gravity: 1 });
    assert.ok(Math.hypot(m.hips[0], m.hips[2]) < 1e-9);
    assert.ok(Math.hypot(r.hx, r.hz) <= 3.6);
  }
});

test('gravity.csv: horizontal hips shift is non-increasing in g, both pinned and free (model and measured)', { skip: skip('gravity.csv') }, () => {
  const rows = data('gravity.csv');
  const keys = [...new Set(rows.map(r => `${r.tx},${r.ty},${r.tz},${r.feet}`))];
  assert.equal(keys.length, 8);
  for (const k of keys) {
    const rs = rows.filter(r => `${r.tx},${r.ty},${r.tz},${r.feet}` === k);
    let pm = Infinity, pd = Infinity;
    for (const r of rs) {
      const h = Math.hypot(pullRun([r.tx, r.ty, r.tz], 1, { gravity: r.gravity, feet: r.feet }).hips[0], pullRun([r.tx, r.ty, r.tz], 1, { gravity: r.gravity, feet: r.feet }).hips[2]);
      const d = Math.hypot(r.hx, r.hz);
      assert.ok(h <= pm + 1e-9, `model ${k} g=${r.gravity}`);
      assert.ok(d <= pd + 1e-9, `measured ${k} g=${r.gravity}`);
      pm = h; pd = d;
    }
  }
});

test('gravity: vertical hand pull is kept (hips drop for a low goal even at g = 1)', () => {
  const m = pullRun([68, 90, 0], 1, { gravity: 1 });
  assert.ok(m.hips[1] < 100 - 10);
  near(m.hips[0], 0, 1e-9);
});

test('retarget_comp.csv: HipsHeightCompensation through leg pull and compression (hips y <= 0.05 cm)', { skip: skip('retarget_comp.csv') }, () => {
  let m = 0, n = 0;
  for (const r of data('retarget_comp.csv')) {
    if (r.prop !== 'HipsHeightCompensation') continue;
    const e = hipsHeightCompensation({ hips: [0, 100, 0], lHip: [10, 95, 0], rHip: [-10, 95, 0], lAnkle: [10, 8, 0], rAnkle: [-10, 8, 0] }, r.value);
    const o = bodyPull({ hips: e.hips, chest: v3.add(e.hips, [0, 25, 0]), limbs: legs(), pullNorm: RETARGET_PULL_NORM, gravity: 0 });
    m = Math.max(m, Math.abs(o.hips[1] - r.hy));
    n++;
  }
  assert.equal(n, 5);
  assert.ok(m <= 0.05, `max ${m}`);
});

test('retarget_comp.csv: AnkleHeightCompensation hips response (hips y <= 0.05 cm); ankle y residual documented', { skip: skip('retarget_comp.csv') }, () => {
  let m = 0, n = 0, ay = 0;
  for (const r of data('retarget_comp.csv')) {
    if (r.prop !== 'AnkleHeightCompensation') continue;
    const e = ankleHeightCompensation({ hips: [0, 100, 0], lAnkle: [10, 8, 0], rAnkle: [-10, 8, 0] }, r.value);
    const o = bodyPull({ hips: [0, 100, 0], chest: [0, 125, 0], limbs: legs([e.lAnkle[1], e.rAnkle[1]]), pullNorm: RETARGET_PULL_NORM, gravity: 0 });
    m = Math.max(m, Math.abs(o.hips[1] - r.hy));
    ay = Math.max(ay, Math.abs(e.lAnkle[1] - r.lay));
    n++;
  }
  assert.equal(n, 4);
  assert.ok(m <= 0.05, `max ${m}`);
  assert.ok(ay <= 1.5, `ankle goal vs solved ankle ${ay}`);
});

test('retarget_comp.csv: compression factor 0 gives no push back, 1 gives most of it', () => {
  const run = rc => {
    const ls = legs().map(l => ({ ...l, rc }));
    return bodyPull({ hips: [0, 90, 0], chest: [0, 115, 0], limbs: ls, pullNorm: RETARGET_PULL_NORM, gravity: 0 }).hips[1];
  };
  near(run(0), 90, 1e-9);
  assert.ok(run(0.5) > 94 && run(0.5) < 97);
  assert.ok(run(1) > run(0.5));
  assert.ok(run(1) > 97.5 && run(1) < 99.5);
});

test('bodyReachRigid pulls hips down onto reachable legs and ignores arms', () => {
  const r = bodyReachRigid({ hips: [0, 105, 0], chest: [0, 130, 0] }, [...legs(), arm([200, 145, 0], 1)]);
  near(r.hips[1], 100, 1e-9);
  near(r.chest[1], 125, 1e-9);
});

test('compensations: hips height, ankle height, scale, mass center, auto formulas', () => {
  const e = { hips: [0, 100, 0], lHip: [10, 95, 0], rHip: [-10, 95, 0], lAnkle: [10, 8, 0], rAnkle: [-10, 8, 0] };
  const h = hipsHeightCompensation(e, -10);
  nearVec(h.hips, [0, 90, 0], 1e-12);
  nearVec(h.lHip, [10, 85, 0], 1e-12);
  nearVec(h.lAnkle, e.lAnkle, 1e-12);
  const tilt = hipsHeightCompensation({ ...e, lHip: [10, 95, 0], lAnkle: [10, 8, 0], rAnkle: [-10, 8, 0] }, 3);
  nearVec(v3.sub(tilt.hips, e.hips), [0, 3, 0], 1e-12);
  const a = ankleHeightCompensation({ ...e, lAnkleQ: q4.axis([0, 0, 1], Math.PI / 2) }, 4);
  nearVec(a.lAnkle, [6, 8, 0], 1e-9);
  nearVec(a.rAnkle, [-10, 12, 0], 1e-12);
  const s = scaleCompensation(e, 50);
  nearVec(s.lAnkle, [5, 4, 0], 1e-12);
  nearVec(scaleCompensation(e, 200, [0, 0, 0]).hips, [0, 200, 0], 1e-12);
  nearVec(massCenterCompensation({ ...e, hips: [0, 95, 0] }, 100, 87).hips, [0, 95, 0], 1e-9);
  const high = massCenterCompensation({ ...e, hips: [0, 110, 0], lHip: [10, 105, 0], rHip: [-10, 105, 0] }, 100, 87);
  near(high.hips[1], 95, 1e-9);
  const half = massCenterCompensation({ ...e, hips: [0, 110, 0], lHip: [10, 105, 0], rHip: [-10, 105, 0] }, 50, 87);
  near(half.hips[1], 110 + 0.5 * (high.hips[1] - 110), 1e-9);
  near(autoScale(90, 90, 20, 100, 100), 110 / 200, 1e-12);
  near(autoHipsHeight(95, 100), -5, 1e-12);
});

test('DEFAULTS match the recovered property defaults', () => {
  assert.equal(DEFAULTS.resistKnee, 0.5);
  assert.equal(DEFAULTS.maxExt, 0.5);
  assert.equal(DEFAULTS.compression, 0.5);
  assert.equal(DEFAULTS.pullIters, 10);
  assert.equal(DEFAULTS.gravity, 0.6);
});
