import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as A from '../re/mobu/auxfx.js';
import { eulerToQuat } from '../re/mobu/layers.js';

const P = (f) => JSON.parse(readFileSync(new URL(`../re/mobu/probes/${f}`, import.meta.url)));
const a1 = P('aux_out.json');
const a2 = P('aux2_out.json');
const PA = [55, 130, 20];
const PB = [50, 150, -20];
const QA = eulerToQuat([0, 0, 0]);
const QB = eulerToQuat([0, 90, 0]);
const dist = (p, q) => Math.hypot(...p.map((x, i) => x - q[i]));
const ang = (p, q) => (2 * Math.acos(Math.min(1, Math.abs(p.reduce((s, x, i) => s + x * q[i], 0)))) * 180) / Math.PI;
const sets = (rm, ra) => [{ t: PA, q: QA, reachT: rm, reachR: rm }, { t: PB, q: QB, reachT: ra, reachR: ra }];

test('full-reach combination lands on the reach-weighted target', (t) => {
  let wt = 0;
  let wr = 0;
  for (const [rm, ra] of [[100, 0], [0, 100], [100, 100], [100, 50]]) {
    const c = A.combineEffectors(sets(rm, ra));
    assert.equal(Math.max(c.reachT, c.reachR), 1);
    const m = a1[`T_${rm}_${ra}`];
    wt = Math.max(wt, dist(c.t, m.hand));
    if (rm === 100 && ra > 0) wr = Math.max(wr, ang(c.q, eulerToQuat(m.handR)));
  }
  t.diagnostic(`translation max error ${wt.toFixed(4)} cm, rotation max error ${wr.toFixed(4)} deg`);
  assert.ok(wt < 0.03);
  assert.ok(wr < 0.01);
});

test('combined reach is the max of the set reaches', () => {
  const c50 = A.combineEffectors(sets(50, 50));
  assert.deepEqual(c50.t, [52.5, 140, 0]);
  assert.equal(c50.reachT, 0.5);
  assert.deepEqual(a2.both50.hand, a2.single_avg_50.hand);
  const c = A.combineEffectors(sets(25, 75));
  assert.ok(dist(c.t, [51.25, 145, -10]) < 1e-9);
  assert.equal(c.reachT, 0.75);
  assert.deepEqual(a2.m25_a75.hand, a2.single_w2575_r75.hand);
});

test('pivot offset is carried by the effector rotation', (t) => {
  const w = A.wristFromEffector(a1.eff_after, eulerToQuat([0, 90, 0]), [10, 0, 0]);
  const e = dist(w, a1.piv10_rot90.hand);
  t.diagnostic(`pivot max error ${e.toFixed(4)} cm`);
  assert.ok(e < 0.01);
});

test('aux relative to main', () => {
  const y = [0, Math.sin(Math.PI / 4), 0, Math.cos(Math.PI / 4)];
  const r = A.auxRelative([1, 0, 0], y, [1, 0, -1], y);
  assert.ok(Math.abs(r.t[0] - 1) < 1e-9);
  assert.ok(Math.abs(r.q[3] - 1) < 1e-9);
});
