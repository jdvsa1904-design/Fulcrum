import { fileURLToPath } from 'node:url';
import test from 'node:test';
import assert from 'node:assert/strict';
import { json, exists, near, nearVec, v3, q4, rad } from './helpers.mjs';
import { detectPlants } from '../contacts.js';
import { rollCurveFromPlant, rollPose } from '../wire/roll.js';

const mat = (q, p) => {
  const [x, y, z, w] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0,
    2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0,
    2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0,
    p[0], p[1], p[2], 1];
};
const yaw = rad(37), Qy = q4.axis([0, 1, 0], yaw), Qf = q4.mul(Qy, q4.axis([1, 0, 0], rad(-20)));
const Qb = q4.mul(Qy, q4.axis([0, 0, 1], rad(8)));
const ANK = [12, 7.5, -4], BALL = v3.add(ANK, q4.rot(Qy, [0, -6.5, 15.2])), KNEE = v3.add(ANK, q4.rot(Qy, [0, 42, -3]));
const rig = () => Float64Array.from([...mat(Qy, KNEE), ...mat(Qf, ANK), ...mat(Qb, BALL)]);
const LEG = { calf: 0, foot: 1, ball: 2 };
const pose = (roll, o = {}) => rollPose({ W: rig(), nb: 3, f: 0, side: 'l', legIdx: LEG, roll, ...o });
const delta = (q, q0) => q4.mul(q, q4.conj(q0));
const finite = o => Object.values(o).every(v => Array.isArray(v) ? v.every(Number.isFinite) : true);

test('roll 0 is the identity', () => {
  const p = pose(0);
  nearVec(p.footP, ANK, 1e-9);
  nearVec(p.ballP, BALL, 1e-9);
  assert.ok(q4.angle(p.footQ, Qf) < 1e-9);
  assert.ok(q4.angle(p.ballQ, Qb) < 1e-9);
  assert.ok(finite(p));
});

test('heel, ball and toe pivots stay fixed in their phases', () => {
  const p0 = pose(0), heel0 = p0.pivots.heel, toe0 = p0.pivots.toe;
  nearVec(heel0, [ANK[0], BALL[1], ANK[2]], 1e-9, 'heel sits below the ankle at sole height');
  near(v3.dist(toe0, BALL), 0.45 * v3.dist(ANK, BALL), 1e-9);
  for (const r of [-30, -12, -3]) {
    const p = pose(r), h = v3.add(p.footP, q4.rot(delta(p.footQ, Qf), v3.sub(heel0, ANK)));
    nearVec(h, heel0, 1e-9, `heel r ${r}`);
    nearVec(p.heelP, heel0, 1e-9);
    assert.ok(q4.angle(p.ballQ, Qb) > rad(Math.abs(r)) - 1e-9, 'whole foot tips back on heel strike');
  }
  for (const r of [5, 15, 29]) {
    const p = pose(r);
    nearVec(p.ballP, BALL, 1e-9, `ball r ${r}`);
    assert.ok(q4.angle(p.ballQ, Qb) < 1e-9, 'toes stay flat while the heel lifts');
    near(q4.angle(p.footQ, Qf), rad(r * Math.min(1, r / 30)), 1e-9);
    assert.ok(p.footP[1] > ANK[1], 'ankle rises');
  }
  for (const r of [60, 75, 90]) {
    const p = pose(r), t = v3.add(p.ballP, q4.rot(delta(p.ballQ, Qb), v3.sub(toe0, BALL)));
    nearVec(t, toe0, 1e-9, `toe r ${r}`);
    nearVec(p.toeP, toe0, 1e-9);
  }
});

test('bone lengths and rigid foot shape are preserved', () => {
  const L = v3.dist(ANK, BALL);
  for (let r = -40; r <= 100; r += 2.5) {
    const p = pose(r);
    assert.ok(finite(p));
    near(v3.dist(p.footP, p.ballP), L, 1e-9, `r ${r}`);
    nearVec(v3.sub(p.ballP, p.footP), q4.rot(delta(p.footQ, Qf), v3.sub(BALL, ANK)), 1e-9, `rotation matches position r ${r}`);
    near(v3.dist(p.heelP, p.toeP), v3.dist(p.pivots.heel, p.pivots.toe), 1e-9);
  }
});

test('positive roll lifts the heel about the lateral axis', () => {
  const p = pose(20);
  assert.ok(p.heelP[1] > p.pivots.heel[1] + 1);
  near(v3.dot(p.axis, [0, 1, 0]), 0, 1e-12);
  near(v3.dot(p.axis, v3.unit(v3.sub(BALL, ANK).map((x, i) => i === 1 ? 0 : x))), 0, 1e-12);
});

test('degenerate inputs do not produce NaN', () => {
  const W = new Float64Array(48);
  for (const o of [0, 16, 32]) W[o] = W[o + 5] = W[o + 10] = W[o + 15] = 1;
  const p = rollPose({ W, nb: 3, f: 0, side: 'r', legIdx: LEG, roll: 25 });
  assert.ok(finite(p) && p.footQ.every(Number.isFinite));
  assert.ok(finite(pose(NaN)));
  const c = rollCurveFromPlant({ heights: { ankle: [0, 0, 0], ball: [0, 0, 0] }, a: 0, b: 2 });
  assert.ok(c.every(Number.isFinite));
  assert.equal(rollCurveFromPlant({ heights: { ankle: [1, 2], ball: [0, 0] }, a: 1, b: 0 }).every(x => x === 0), true);
});

const synth = rolls => {
  const ankle = [], ball = [];
  for (const r of rolls) { const p = pose(r); ankle.push(p.footP[1]); ball.push(p.ballP[1]); }
  return { ankle, ball };
};

test('rollCurve recovers a synthetic push-off and rises monotonically as the heel lifts', () => {
  const rolls = [0, 0, 0, 0, 3, 7, 12, 18, 25, 33, 41, 50, 58];
  const L = v3.dist(ANK, BALL);
  const c = rollCurveFromPlant({ heights: synth(rolls), a: 0, b: rolls.length - 1, length: L });
  for (let i = 0; i < rolls.length; i++) near(c[i], rolls[i], 1e-6, `frame ${i}`);
  for (let i = 4; i < rolls.length; i++) assert.ok(c[i] > c[i - 1], `monotone at ${i}`);
  const est = rollCurveFromPlant({ heights: synth(rolls), a: 0, b: rolls.length - 1 });
  for (let i = 4; i < rolls.length; i++) assert.ok(est[i] >= est[i - 1], 'monotone without a length');
  assert.ok(est[4] > 0 && est[rolls.length - 1] > 30);
  near(est[0], 0, 1e-12);
});

test('rollCurve goes negative on heel strike and is zero when flat', () => {
  const rolls = [-20, -14, -8, -3, 0, 0, 0, 0];
  const c = rollCurveFromPlant({ heights: synth(rolls), a: 0, b: rolls.length - 1, length: v3.dist(ANK, BALL) });
  for (let i = 0; i < 4; i++) { assert.ok(c[i] < 0); near(c[i], rolls[i], 1e-6); }
  for (let i = 4; i < rolls.length; i++) assert.equal(c[i], 0);
  const flat = rollCurveFromPlant({ heights: synth([0, 0, 0, 0, 0]), a: 0, b: 4, length: v3.dist(ANK, BALL) });
  assert.ok(flat.every(x => x === 0), 'no-op when nothing rolls');
  const ranged = rollCurveFromPlant({ heights: synth([30, 30, 0, 10, 30]), a: 2, b: 3, length: v3.dist(ANK, BALL) });
  assert.equal(ranged.length, 5);
  assert.equal(ranged[0], 0);
  assert.equal(ranged[4], 0);
  near(ranged[3], 10, 1e-6);
});

const FRAMES = fileURLToPath(new URL('../exports/FRT_fulcrum_v6.frames.json', import.meta.url));
test('real FRT take: plant rolls are finite, bounded and rigid', { skip: !exists(FRAMES) }, () => {
  const J = json(FRAMES), nb = J.bones.length, n = J.frames.length, W = new Float64Array(n * nb * 16);
  J.frames.forEach((fr, f) => W.set(fr, f * nb * 16));
  for (const s of ['l', 'r']) {
    const legIdx = { calf: J.bones.indexOf('calf_' + s), foot: J.bones.indexOf('foot_' + s), ball: J.bones.indexOf('ball_' + s) };
    const at = (f, k) => [W[(f * nb + k) * 16 + 12], W[(f * nb + k) * 16 + 13], W[(f * nb + k) * 16 + 14]];
    const ballPos = new Float64Array(n * 3), ankle = [], ball = [];
    for (let f = 0; f < n; f++) { ballPos.set(at(f, legIdx.ball), f * 3); ankle.push(at(f, legIdx.foot)[1]); ball.push(at(f, legIdx.ball)[1]); }
    const L = v3.dist(at(0, legIdx.foot), at(0, legIdx.ball));
    const { runs } = detectPlants(ballPos, { legLength: 91.33, tol: 0.01 });
    assert.ok(runs.length > 0, `plants found for ${s}`);
    for (const run of runs) {
      const c = rollCurveFromPlant({ heights: { ankle, ball }, a: run.s, b: run.e, length: L });
      for (let f = run.s; f <= run.e; f++) {
        assert.ok(Number.isFinite(c[f]) && Math.abs(c[f]) <= 90);
        const p = rollPose({ W, nb, f, side: s, legIdx, roll: c[f] });
        assert.ok(finite(p) && p.footQ.every(Number.isFinite) && p.ballQ.every(Number.isFinite));
        near(v3.dist(p.footP, p.ballP), L, 1e-3);
        near(Math.hypot(...p.footQ), 1, 1e-9);
      }
    }
  }
});
