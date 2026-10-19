import test from 'node:test';
import assert from 'node:assert/strict';
import { json, exists, near, nearVec, v3, q4, deg, rad, rng } from './helpers.mjs';
import { nlerp, twistAngle, scaledDelta, realisticKneeSwivel, transferAnkleTwist, solveLeg, qaxis, qmul, qconj } from '../re/hik/knee.js';

const DIR = 'C:/re_work/hik/results/';
const skip = f => exists(DIR + f) ? false : `missing ${f}`;
const qErr = (a, b) => deg(q4.angle(a, b));

test('leg_twist_random.json: full-reach legs match HumanIK (knee <= 0.001 cm, knee Q <= 0.05 deg, ankle Q <= 0.05 deg)', { skip: skip('leg_twist_random.json') }, () => {
  const rows = json(DIR + 'leg_twist_random.json').filter(r => r.T === 1);
  assert.ok(rows.length >= 40);
  let mk = 0, mqk = 0, mqa = 0;
  for (const r of rows) {
    const o = solveLeg({ top: r.top, knee: r.knee, ankle: r.ankle, qTop: r.qh, qKnee: r.qk, qAnkle: r.qa, goal: r.goal, qGoal: r.qGoal, T: r.T, R: r.R, realistic: r.k, legRoll: r.roll });
    mk = Math.max(mk, v3.dist(o.knee, r.out.knee));
    mqk = Math.max(mqk, qErr(o.qKnee, r.out.qk));
    mqa = Math.max(mqa, qErr(o.qAnkle, r.out.qa));
  }
  console.log(`T=1: knee ${mk.toFixed(5)} cm, knee Q ${mqk.toFixed(4)} deg, ankle Q ${mqa.toFixed(4)} deg over ${rows.length}`);
  assert.ok(mk <= 0.001, `knee ${mk}`);
  assert.ok(mqk <= 0.05, `knee Q ${mqk}`);
  assert.ok(mqa <= 0.05, `ankle Q ${mqa}`);
});

test('leg_twist_random.json: partial reach T < 1 matches HumanIK (ankle <= 0.002 cm, knee <= 0.002 cm, knee Q <= 0.05 deg)', { skip: skip('leg_twist_random.json') }, () => {
  const rows = json(DIR + 'leg_twist_random.json').filter(r => r.T < 1);
  assert.ok(rows.length >= 60);
  let ma = 0, mk = 0, mqk = 0, mqa = 0;
  for (const r of rows) {
    const o = solveLeg({ top: r.top, knee: r.knee, ankle: r.ankle, qTop: r.qh, qKnee: r.qk, qAnkle: r.qa, goal: r.goal, qGoal: r.qGoal, T: r.T, R: r.R, realistic: r.k, legRoll: r.roll });
    ma = Math.max(ma, v3.dist(o.ankle, r.out.ankle));
    mk = Math.max(mk, v3.dist(o.knee, r.out.knee));
    mqk = Math.max(mqk, qErr(o.qKnee, r.out.qk));
    mqa = Math.max(mqa, qErr(o.qAnkle, r.out.qa));
  }
  console.log(`T<1: ankle ${ma.toFixed(5)} cm, knee ${mk.toFixed(5)} cm, knee Q ${mqk.toFixed(4)} deg, ankle Q ${mqa.toFixed(4)} deg over ${rows.length}`);
  assert.ok(ma <= 0.002, `ankle ${ma}`);
  assert.ok(mk <= 0.002, `knee ${mk}`);
  assert.ok(mqk <= 0.05, `knee Q ${mqk}`);
  assert.ok(mqa <= 0.05, `ankle Q ${mqa}`);
});

test('partial reach keeps the hip-ankle distance on the lerp of start and goal distances', () => {
  const r = rng(9);
  for (let i = 0; i < 30; i++) {
    const top = [10, 95, 0], knee = [10, 52, 4 + 8 * r()], ankle = [10, 9, 0];
    const goal = [10 + 30 * (r() - 0.5), 25 + 30 * r(), 30 * (r() - 0.5)], T = r();
    const o = solveLeg({ top, knee, ankle, goal, T });
    const d0 = v3.dist(top, ankle), dg = Math.min(v3.dist(top, goal), v3.dist(top, knee) + v3.dist(knee, ankle));
    near(v3.dist(top, o.ankle), d0 + T * (dg - d0), 1e-6);
  }
});

test('realistic_knee.json: straight T-pose legs with RealisticKneeSolving 0..1 (knee <= 0.02 cm)', { skip: skip('realistic_knee.json') }, () => {
  const rows = json(DIR + 'realistic_knee.json');
  let mk = 0, mq = 0;
  for (const r of rows) {
    const o = solveLeg({ top: [10, 95, 0], knee: [10, 52, 0], ankle: [10, 8, 0], goal: r.tgt, qGoal: r.qg, T: r.T, R: r.R, realistic: r.k });
    mk = Math.max(mk, v3.dist(o.knee, r.o.LeftKnee[0]));
    mq = Math.max(mq, qErr(o.qKnee, r.o.LeftKnee[1]));
  }
  console.log(`realistic: knee ${mk.toFixed(4)} cm, knee Q ${mq.toFixed(4)} deg over ${rows.length}`);
  assert.equal(rows.length, 120);
  assert.ok(mk <= 0.02, `knee ${mk}`);
  assert.ok(mq <= 0.05, `knee Q ${mq}`);
});

test('nlerp is the Reach R interpolant: endpoints, sign alignment, unit length', () => {
  const a = qaxis([0, 1, 0], 0.3), b = qaxis([1, 0.2, 0], 1.4);
  nearVec(nlerp(a, b, 0), a, 1e-12);
  nearVec(nlerp(a, b, 1), b, 1e-12);
  nearVec(nlerp(a, b.map(x => -x), 1), b, 1e-12);
  const m = nlerp(a, b, 0.37);
  near(Math.hypot(...m), 1, 1e-12);
});

test('twistAngle and scaledDelta: k scales tan of the half angle', () => {
  const q = qaxis([0, 1, 0], 1.0);
  near(twistAngle(q, [0, 1, 0]), 1.0, 1e-12);
  near(twistAngle(q, [0, -1, 0]), -1.0, 1e-12);
  const h = scaledDelta(q, 0.5);
  near(twistAngle(h, [0, 1, 0]), 2 * Math.atan(0.5 * Math.tan(0.5)), 1e-12);
  nearVec(scaledDelta(q, 1), q, 1e-12);
});

test('realisticKneeSwivel keeps hip and ankle fixed and swings the knee about the hip-ankle line', () => {
  const r = rng(4);
  for (let i = 0; i < 20; i++) {
    const leg = { top: [10, 95, 0], knee: [10, 52, 8 + 10 * r()], ankle: [10, 9, 0], qTop: [0, 0, 0, 1], qKnee: [0, 0, 0, 1], qAnkle: [0, 0, 0, 1] };
    const out = realisticKneeSwivel(leg, qaxis([r() - 0.5, 1, r() - 0.5], rad(80 * (r() - 0.5))), r(), 0.2 + 0.8 * r());
    nearVec(out.top, leg.top, 1e-12);
    near(v3.dist(out.knee, leg.top), v3.dist(leg.knee, leg.top), 1e-9);
    near(v3.dist(out.knee, leg.ankle), v3.dist(leg.knee, leg.ankle), 1e-9);
  }
  const same = realisticKneeSwivel({ top: [0, 1, 0], knee: [0, 0.5, 0.1], ankle: [0, 0, 0], qTop: [0, 0, 0, 1], qKnee: [0, 0, 0, 1], qAnkle: [0, 0, 0, 1] }, qaxis([0, 1, 0], 1), 1, 0);
  nearVec(same.knee, [0, 0.5, 0.1], 0);
});

test('transferAnkleTwist preserves the ankle-local twist and LegRoll keeps a share on the foot', () => {
  const leg = { top: [0, 95, 0], knee: [0, 52, 5], ankle: [0, 8, 0], qTop: [0, 0, 0, 1], qKnee: qaxis([1, 0, 0], 0.2), qAnkle: qaxis([0.2, 1, 0.1], 0.9) };
  const carried = qaxis([1, 0, 0], 0.2);
  const s = v3.unit(v3.sub(leg.ankle, leg.knee));
  const full = transferAnkleTwist(leg, carried, 0);
  const local = q => qmul(qconj(full.qKnee), q);
  const axis = q4.rot(qconj(full.qKnee), s);
  near(twistAngle(local(leg.qAnkle), axis), twistAngle(qmul(qconj(leg.qKnee), carried), q4.rot(qconj(leg.qKnee), s)), 1e-9);
  const part = transferAnkleTwist(leg, carried, 0.6);
  near(part.kneeTwist, full.kneeTwist * 0.4, 1e-12);
  near(transferAnkleTwist(leg, carried, 1).kneeTwist, 0, 1e-12);
});
