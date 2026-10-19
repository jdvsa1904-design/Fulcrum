import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as L from '../re/mobu/layers.js';

const P = (f) => JSON.parse(readFileSync(new URL(`../re/mobu/probes/${f}`, import.meta.url)));
const lay = P('layers_out.json');
const filt = P('filt_out.json');
const ckr = P('ckr_out.json');
const ckr2 = P('ckr2_out.json');

const angle = (a, b) => {
  const qa = L.eulerToQuat(a);
  const qb = L.eulerToQuat(b);
  const d = Math.abs(qa.reduce((s, x, i) => s + x * qb[i], 0));
  return (2 * Math.acos(Math.min(1, d)) * 180) / Math.PI;
};

test('layer modes, weights and rotation modes match MotionBuilder', (t) => {
  const M = { add: L.MODE.ADDITIVE, ovr: L.MODE.OVERRIDE, ovp: L.MODE.OVERRIDE_PASSTHROUGH };
  const RM = { eul: L.ROT.EULER, quat: L.ROT.QUAT };
  let wt = 0;
  let wr = 0;
  for (const [key, [T, R]] of Object.entries(lay)) {
    if (key === 'parentchild') continue;
    const [m, r, w] = key.split('_');
    const v = L.evalScalar([{ id: 0, value: 1 }, { id: 1, value: 10, weight: +w, mode: M[m] }]);
    wt = Math.max(wt, Math.abs(v - T[0]));
    const e = L.blendRotation([30, 0, 0], [0, 60, 20], M[m], +w / 100, RM[r]);
    wr = Math.max(wr, angle(e, R));
  }
  const pc = L.evalScalar([{ id: 0, value: 1 }, { id: 1, value: 10, weight: 50 }, { id: 2, parent: 1, value: 4, weight: 50 }]);
  wt = Math.max(wt, Math.abs(pc - lay.parentchild[0][0]));
  t.diagnostic(`translation max error ${wt.toExponential(2)}, rotation max error ${wr.toExponential(2)} deg`);
  assert.equal(lay.ovr_eul_50[0][0], 5);
  assert.ok(wt < 1e-6);
  assert.ok(wr < 1e-3);
});

const toKeys = (rows) => rows.map(([t, v]) => ({ t, v }));
const seq = (vals) => vals.map((v, t) => ({ t, v }));
const thrFor = (name) => (name.endsWith('_S') ? L.CKR_THRESHOLD.S : name.endsWith('_R') ? L.CKR_THRESHOLD.R : L.CKR_THRESHOLD.T);

test('constant key reducer matches MotionBuilder', () => {
  for (const [name, c] of Object.entries(filt).filter(([n]) => n.startsWith('ckr_'))) {
    const got = L.constantKeyReducer(toKeys(c.before), { threshold: thrFor(name), keepOneKey: name !== 'ckr_const_nokeep' });
    assert.deepEqual(got.map((k) => k.t), c.after.map((k) => k[0]), name);
  }
  for (const [prop, thr, base] of [['Translation', L.CKR_THRESHOLD.T, 0], ['Rotation', L.CKR_THRESHOLD.R, 0], ['Scaling', L.CKR_THRESHOLD.S, 1]]) {
    for (const [e, len] of Object.entries(ckr[`bump_${prop}`])) {
      const vals = [base, base, base, base + Number(e), base, base, base];
      assert.equal(L.constantKeyReducer(seq(vals), { threshold: thr }).length, len, `${prop} ${e}`);
    }
  }
  const runs = {
    interior: [0, 1, 1, 1, 1, 1, 0, 2],
    two_flat: [0, 1, 1, 0],
    three_flat: [0, 1, 1, 1, 0],
    start_flat2: [0, 0, 5],
    start_flat3: [0, 0, 0, 5],
  };
  for (const [name, vals] of Object.entries(runs)) {
    assert.deepEqual(L.constantKeyReducer(seq(vals), { threshold: L.CKR_THRESHOLD.T }).map((k) => k.t), ckr[name].map((k) => k[0]), name);
  }
  for (const name of ['interior_noise_R', 'interior_drift_R']) {
    const vals = name === 'interior_noise_R' ? [0, 1, 1.004, 1, 1.004, 1, 0, 2] : [0, 1, 1.004, 1.008, 1.012, 1.016, 0, 2];
    assert.deepEqual(L.constantKeyReducer(seq(vals)).map((k) => k.t), ckr[name].map((k) => k[0]), name);
  }
  for (const name of ['R_ramp008', 'R_ramp008_x10']) {
    const ks = toKeys(ckr2[name]);
    assert.equal(L.constantKeyReducer(ks).length, ks.length, name);
  }
  for (const [e, len] of Object.entries(ckr2.S_fine0)) {
    assert.equal(L.constantKeyReducer(seq([0, 0, 0, +e, 0, 0, 0]), { threshold: L.CKR_THRESHOLD.S }).length, len, `S ${e}`);
  }
  for (const [e, len] of Object.entries(ckr2.T_fine)) {
    assert.equal(L.constantKeyReducer(seq([0, 0, 0, +e, 0, 0, 0]), { threshold: L.CKR_THRESHOLD.T }).length, len, `T ${e}`);
  }
});

test('resample, keys on frame and time shift match MotionBuilder', (t) => {
  let worst = 0;
  for (const [name, fps] of [['resample24', 24], ['resample12', 12], ['resample_noint', 12]]) {
    const c = filt[name];
    const ev = (f) => c.eval_before_halfframes[Math.round(f * 2)];
    const got = L.resample(ev, 0, 20, fps / 24);
    assert.deepEqual(got.map((k) => k.t), c.after.map((k) => k[0]), name);
    worst = Math.max(worst, ...got.map((k, i) => Math.abs(k.v - c.after[i][1])));
  }
  const fr = filt.resample_frac;
  const got = L.resample((f) => (10 * (f - 0.3)) / 10.3, 0.3, 10.6, 1);
  assert.equal(got.length, fr.after.length);
  got.forEach((k, i) => { worst = Math.max(worst, Math.abs(k.t - fr.after[i][0]), Math.abs(k.v - fr.after[i][1])); });
  t.diagnostic(`resample max error ${worst.toExponential(2)}`);
  assert.ok(worst < 1e-5);
  for (const [name, fps] of [['kof', 24], ['kof12', 12]]) {
    const out = L.keysOnFrame(toKeys(filt[name].before).map((k) => ({ ...k, t: k.t / 24 })), fps).map((k) => Math.round(k.t * 24 * 1e6) / 1e6);
    assert.deepEqual(out, filt[name].after.map((k) => k[0]), name);
  }
  for (const [name, sh, sc] of [['tss', 5, 2], ['tss_late', 0, 2], ['tss_late_half', 3, 0.5]]) {
    assert.deepEqual(L.timeShiftAndScale(toKeys(filt[name].before), sh, sc).map((k) => k.t), filt[name].after.map((k) => k[0]), name);
  }
});

test('euler round trip', () => {
  const e = [10, 20, 30];
  L.quatToEuler(L.eulerToQuat(e)).forEach((x, i) => assert.ok(Math.abs(x - e[i]) < 1e-9));
});
