import test from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { json, exists, near, nearVec, q4, rng, deg, rad } from './helpers.mjs';
import {
  TPS, butterworthCoefficients, butterworthCurve, butterworth, butterworthVec, butterworthQuat, sampleCurve,
  peakRemoval, peakRemovalQuat, quatContinuity, eulerFromQuat, quatFromEuler, eulerEquivalent, eulerFilter, unrollQuats,
  unrollQuality, unrollFlags, constantKeyReducer, evalLinearKeys, keyReducer, evalKeys, KEY_THRESHOLDS,
  adaptiveGaussian, adaptiveGaussianQuat, gaussianKernel, readTrack, writeTrack,
} from '../filters.js';

const here = dirname(fileURLToPath(import.meta.url));
const REF = json(join(here, 'fixtures', 'bw_ref.json'));
const MAYA = 'C:/re_work/maya/tests/bw_result.json';

function lin(keys, fps) {
  const ts = keys.slice().sort((a, b) => a[0] - b[0]);
  return sec => {
    const fr = sec * fps;
    if (fr <= ts[0][0]) return ts[0][1];
    if (fr >= ts[ts.length - 1][0]) return ts[ts.length - 1][1];
    for (let i = 0; i < ts.length - 1; i++) {
      const [a, va] = ts[i], [b, vb] = ts[i + 1];
      if (a <= fr && fr <= b) return va + (vb - va) * (fr - a) / (b - a);
    }
  };
}

test('butterworth matches bw_model.py on all 44 Maya cases to 1e-9', () => {
  const fps = REF.fps;
  let worst = 0, n = 0;
  for (const [name, keys] of Object.entries(REF.signals)) {
    for (const [cname, fc, sr, kof] of REF.cases) {
      const ref = REF.model[name + ':' + cname];
      const pts = butterworthCurve(lin(keys, fps), keys[0][0] / fps, keys[keys.length - 1][0] / fps, { fc, sr, kof });
      if (ref === null) { assert.equal(pts, null); continue; }
      assert.equal(pts.length, ref.length, name + ':' + cname);
      for (let i = 0; i < ref.length; i++) {
        assert.equal(pts.ticks[i], ref[i][0]);
        const e = Math.abs(pts[i][1] - ref[i][1]);
        worst = Math.max(worst, e); n++;
      }
    }
  }
  assert.ok(worst <= 1e-9, 'max abs error ' + worst);
  console.log(`  bw_model: ${n} samples, max abs error ${worst.toExponential(2)}`);
});

test('butterworth reproduces the mayapy filterCurve keys in bw_result.json', { skip: !exists(MAYA) }, () => {
  const maya = json(MAYA), fps = REF.fps;
  let worst = 0, n = 0, skipped = 0;
  for (const [key, r] of Object.entries(maya)) {
    const [name, cname] = key.split(':'), c = REF.cases.find(x => x[0] === cname), keys = REF.signals[name];
    const pts = butterworthCurve(lin(keys, fps), keys[0][0] / fps, keys[keys.length - 1][0] / fps, { fc: c[1], sr: c[2], kof: c[3] });
    if (!pts) continue;
    const onKey = new Set(pts.ticks);
    for (const [fr, v] of r.sample) {
      if (!onKey.has(Math.round(fr / fps * TPS))) { skipped++; continue; }
      worst = Math.max(worst, Math.abs(sampleCurve(pts, fr / fps) - v)); n++;
    }
    if (c[2] === fps) {
      const vals = Float64Array.from(keys.map(k => k[1])), out = butterworth(vals, fps, { fc: c[1], kof: c[3] });
      for (const [fr, v] of r.sample) worst = Math.max(worst, Math.abs(out[Math.round(fr)] - v));
    }
  }
  assert.ok(worst <= 1e-9, 'max abs error vs Maya ' + worst);
  assert.ok(skipped <= 1);
  console.log(`  mayapy keys: ${n} samples, max abs error ${worst.toExponential(2)}, ${skipped} off-grid end key skipped`);
});

test('butterworth response is -6.02 dB at Fc and matches MAYA.md 4.3', () => {
  const amp = (hz, fc) => {
    const pts = butterworthCurve(s => Math.sin(2 * Math.PI * hz * s), 0, 10, { fc, sr: 24, kof: true });
    let m = 0;
    for (const [t, v] of pts) if (t > 3 && t < 7) m = Math.max(m, Math.abs(v));
    return m;
  };
  const H2 = (hz, fc) => 1 / (1 + (Math.tan(Math.PI * hz / 240) / Math.tan(Math.PI * fc / 240)) ** 8);
  for (const [hz, db] of [[4, -0.10], [6, -2.21], [7, -6.02], [10, -25.5]]) {
    const a = amp(hz, 7);
    near(a, H2(hz, 7), 2e-3, `amplitude at ${hz} Hz`);
    near(20 * Math.log10(a), db, 0.06, `dB at ${hz} Hz`);
  }
});

test('MotionBuilder variant: 250-sample pad, primed state, keeps DC exactly and agrees with Maya inside', () => {
  const dc = butterworthCurve(() => 3.25, 0, 2, { fc: 7, sr: 30, variant: 'mobu' });
  for (const [, v] of dc) near(v, 3.25, 1e-12, 'dc');
  const f = s => Math.sin(2 * Math.PI * 1.3 * s) + 0.3 * Math.sin(2 * Math.PI * 9 * s) + 0.2 * s;
  const a = butterworthCurve(f, 0, 6, { fc: 5, sr: 30 }), b = butterworthCurve(f, 0, 6, { fc: 5, sr: 30, variant: 'mobu' });
  assert.equal(a.length, b.length);
  let m = 0;
  for (let i = 0; i < a.length; i++) if (a[i][0] > 1 && a[i][0] < 5) m = Math.max(m, Math.abs(a[i][1] - b[i][1]));
  assert.ok(m < 1e-6, 'interior difference ' + m);
  near(b[0][1], f(0), 0, 'first key kept');
  near(b[b.length - 1][1], f(6), 1e-15, 'last key kept');
  const z = butterworthCurve(f, 0, 6, { fc: 5, sr: 30, variant: 'mobu', prime: false });
  assert.ok(Math.abs(z[1][1] - b[1][1]) > 1e-6, 'priming changes the start transient');
});

test('per-frame butterworth keeps the range ends and leaves frames outside untouched', () => {
  const r = rng(11), n = 120, v = Float64Array.from({ length: n }, (_, i) => Math.sin(i / 9) + (r() - 0.5) * 0.4);
  const out = butterworth(v, 30, { fc: 4, a: 20, b: 90 });
  for (let i = 0; i <= 20; i++) assert.equal(out[i], v[i]);
  for (let i = 90; i < n; i++) assert.equal(out[i], v[i]);
  let rough0 = 0, rough1 = 0;
  for (let i = 22; i < 88; i++) { rough0 += (v[i + 1] - 2 * v[i] + v[i - 1]) ** 2; rough1 += (out[i + 1] - 2 * out[i] + out[i - 1]) ** 2; }
  assert.ok(rough1 < rough0 * 0.05, 'smoother');
  const vec = Float64Array.from({ length: n * 3 }, (_, i) => v[i / 3 | 0] * (1 + i % 3));
  const fv = butterworthVec(vec, 3, 30, { fc: 4, a: 20, b: 90 });
  for (let f = 0; f < n; f++) near(fv[f * 3 + 2], out[f] * 3, 1e-12);
  const v25 = butterworth(v, 25, { fc: 4 });
  assert.equal(v25[0], v[0]); assert.equal(v25[n - 1], v[n - 1]);
});

test('butterworthQuat equals the scalar filter on a fixed-axis rotation', () => {
  const r = rng(5), n = 200, fps = 30, axis = [0.3, -0.8, 0.52], th = Float64Array.from({ length: n }, (_, i) => 1.4 * Math.sin(i / 25) + (r() - 0.5) * 0.2 + 2.5);
  const tr = new Float64Array(n * 4);
  for (let f = 0; f < n; f++) { const q = q4.axis(axis, th[f]); tr.set(f % 7 === 3 ? q.map(x => -x) : q, f * 4); }
  const fq = butterworthQuat(tr, fps, { fc: 3 }), ft = butterworth(th, fps, { fc: 3 });
  let m = 0;
  for (let f = 0; f < n; f++) { const q = Array.from(fq.subarray(f * 4, f * 4 + 4)); m = Math.max(m, q4.angle(q, q4.axis(axis, ft[f]))); }
  assert.ok(m < 1e-9, 'max angle error ' + m);
});

test('peak removal: MoBu 3x spread rule removes marker pops and leaves clean motion alone', () => {
  const n = 150, clean = Float64Array.from({ length: n }, (_, i) => 10 * Math.sin(i / 12) + 0.02 * i * i / n);
  const a = peakRemoval(clean);
  assert.equal(a.removed.length, 0);
  nearVec(a.values, clean, 0);
  const pop = Float64Array.from(clean);
  pop[40] += 6; pop[90] -= 9; pop[91] -= 9;
  const b = peakRemoval(pop);
  assert.ok(b.removed.some(i => Math.abs(i - 40) <= 1));
  assert.ok(b.removed.some(i => i >= 89 && i <= 92));
  assert.ok(b.passes <= 20);
  for (let i = 0; i < n; i++) near(b.values[i], clean[i], 0.25, 'frame ' + i);
  const ramp = Float64Array.from({ length: 50 }, (_, i) => 0.3 * i);
  assert.equal(peakRemoval(ramp).removed.length, 0);
});

test('peak removal on quaternion logs', () => {
  const n = 80, tr = new Float64Array(n * 4);
  for (let f = 0; f < n; f++) tr.set(q4.axis([0, 1, 0], 0.8 * Math.sin(f / 10)), f * 4);
  const bad = Float64Array.from(tr);
  bad.set(q4.axis([1, 0.2, 0], 0.6), 30 * 4);
  const out = peakRemovalQuat(bad);
  assert.ok(out.removed.includes(30));
  const want = q4.axis([0, 1, 0], 0.8 * Math.sin(3));
  assert.ok(q4.angle(Array.from(out.subarray(120, 124)), want) < rad(1.5));
  assert.ok(q4.angle(Array.from(out.subarray(200, 204)), Array.from(tr.subarray(200, 204))) < 1e-9);
});

test('Euler conversion round-trips in all six rotate orders and the equivalent triple is the same rotation', () => {
  const r = rng(9);
  for (const order of ['xyz', 'yzx', 'zxy', 'xzy', 'yxz', 'zyx']) {
    for (let k = 0; k < 200; k++) {
      const q = q4.axis([r() - 0.5, r() - 0.5, r() - 0.5], (r() - 0.5) * 6);
      const e = eulerFromQuat(q, order);
      assert.ok(q4.angle(quatFromEuler(e, order), q) < 1e-9, order);
      assert.ok(q4.angle(quatFromEuler(eulerEquivalent(e, order), order), q) < 1e-9, order + ' equivalent');
    }
  }
  const e = [0.1, 0.2, 0.3], qx = quatFromEuler(e, 'xyz');
  const want = q4.mul(q4.mul(q4.axis([0, 0, 1], 0.3), q4.axis([0, 1, 0], 0.2)), q4.axis([1, 0, 0], 0.1));
  assert.ok(q4.angle(qx, want) < 1e-12, 'xyz applies X first');
  nearVec(eulerEquivalent([0.1, 0.2, 0.3], 'xyz'), [0.1 + Math.PI, Math.PI - 0.2, 0.3 + Math.PI], 1e-15);
});

test('Euler filter (Maya closestSolution) removes the 360 spin and the flipped triple', () => {
  const n = 120, tr = new Float64Array(n * 4);
  for (let f = 0; f < n; f++) {
    const a = rad(-170 + f * 3), q = q4.mul(q4.axis([0, 0, 1], 0.3 * Math.sin(f / 8)), q4.mul(q4.axis([0, 1, 0], rad(70 + 25 * Math.sin(f / 15))), q4.axis([1, 0, 0], a)));
    tr.set(f % 5 === 2 ? q.map(x => -x) : q, f * 4);
  }
  const raw = new Float64Array(n * 3);
  for (let f = 0; f < n; f++) raw.set(eulerFromQuat(Array.from(tr.subarray(f * 4, f * 4 + 4))), f * 3);
  let rawJump = 0, jump = 0;
  for (let f = 1; f < n; f++) for (let k = 0; k < 3; k++) rawJump = Math.max(rawJump, Math.abs(raw[f * 3 + k] - raw[f * 3 - 3 + k]));
  const sol = unrollQuats(tr);
  for (let f = 1; f < n; f++) for (let k = 0; k < 3; k++) jump = Math.max(jump, Math.abs(sol[f * 3 + k] - sol[f * 3 - 3 + k]));
  assert.ok(rawJump > rad(170));
  assert.ok(jump < rad(12), 'max channel step ' + deg(jump));
  for (let f = 0; f < n; f++) assert.ok(q4.angle(quatFromEuler(Array.from(sol.subarray(f * 3, f * 3 + 3))), Array.from(tr.subarray(f * 4, f * 4 + 4))) < 1e-9);
  const cutOnly = eulerFilter(raw, { mode: 'cut' }), b = eulerFilter(Float64Array.of(4, -4, 7), { mode: 'bound' });
  for (const x of b) assert.ok(Math.abs(x) <= Math.PI + 1e-12);
  for (let f = 1; f < n; f++) for (let k = 0; k < 3; k++) assert.ok(Math.abs(cutOnly[f * 3 + k] - cutOnly[f * 3 - 3 + k]) <= Math.PI + 1e-9);
  const qc = quatContinuity(tr);
  assert.equal(qc.flips > 0, true);
  for (let f = 1; f < n; f++) assert.ok(qc[f * 4] * qc[f * 4 - 4] + qc[f * 4 + 1] * qc[f * 4 - 3] + qc[f * 4 + 2] * qc[f * 4 - 2] + qc[f * 4 + 3] * qc[f * 4 - 1] >= 0);
});

test('MoBu unroll quality factor flags Euler paths much longer than the rotation', () => {
  near(unrollQuality([0.1, 0.2, 0.3], [0.1, 0.2, 0.3]), 1, 0);
  near(unrollQuality([0.11, 0.2, 0.3], [0.1, 0.2, 0.3]), 1, 1e-6);
  assert.ok(unrollQuality([0.1 + 2 * Math.PI, 0.2, 0.3], [0.1, 0.2, 0.3]) < 1e-6);
  const e = Float64Array.of(0, 0, 0, 0.01, 0, 0, 0.01 + 2 * Math.PI, 0, 0);
  assert.deepEqual(unrollFlags(e).map(x => x.f), [2]);
});

test('constant key reducer: MoBu thresholds, bounded error, endpoints kept', () => {
  assert.deepEqual(KEY_THRESHOLDS.mobu, { translation: 0.009, rotation: 0.009, scaling: 9e-5, default: 0.009, derivative: 0.004 });
  const v = Float64Array.from({ length: 100 }, (_, i) => i < 30 ? 2 : i < 60 ? 2 + (i - 30) * 0.5 : i < 80 ? 17 + 0.001 * Math.sin(i) : 17 + (i - 80) * 0.004);
  const kept = constantKeyReducer(v);
  assert.equal(kept[0], 0); assert.equal(kept[kept.length - 1], 99);
  assert.ok(kept.length < 50, 'kept ' + kept.length);
  for (let i = 0; i < 100; i++) near(evalLinearKeys(v, kept, i), v[i], 0.018, 'frame ' + i);
  for (let i = 31; i < 59; i++) assert.ok(kept.includes(i), 'ramp key ' + i);
  assert.deepEqual(constantKeyReducer(new Float64Array(10).fill(1)), [0, 9]);
  assert.deepEqual(constantKeyReducer(new Float64Array(10).fill(1), { keepFirstAndLast: false }), [0]);
  assert.deepEqual(constantKeyReducer(new Float64Array(10).fill(1), { keepFirstAndLast: false, keepOneKey: false }), []);
});

test('Bezier key reducer (Douglas-Peucker on Hermite fit) stays within precision', () => {
  const n = 241, v = Float64Array.from({ length: n }, (_, i) => 30 * Math.sin(i / 20) + 8 * Math.sin(i / 5.3) + (i > 150 ? 0.2 * (i - 150) : 0));
  for (const prec of [1, 0.1, 0.05]) {
    const keys = keyReducer(v, { precision: prec });
    assert.equal(keys[0].i, 0); assert.equal(keys[keys.length - 1].i, n - 1);
    for (let i = 0; i < n; i++) assert.ok(Math.abs(evalKeys(keys, i) - v[i]) <= prec + 1e-12, `prec ${prec} frame ${i}`);
    assert.ok(keys.length < n / 2);
  }
  const lk = keyReducer(v, { precision: 0.5, tangents: 'linear' });
  assert.ok(lk.some(k => !k.smooth));
  const capped = keyReducer(Float64Array.from({ length: 4000 }, (_, i) => Math.sin(i)), { precision: 1e-9 });
  assert.ok(capped.length <= 2 ** 11 + 1);
});

test('Max adaptive Gaussian: median-anchored weights damp a spike to the hand-computed value', () => {
  const s = Float64Array.of(0, 0, 0, 0, 10, 0, 0, 0, 0), out = adaptiveGaussian(s, 1, { window: 5, sigma: 1 });
  const g = gaussianKernel(5, 1), w = [g[0], g[1], g[2] / 11, g[3], g[4]], want = 10 * w[2] / w.reduce((a, b) => a + b, 0);
  near(out[4], want, 1e-12);
  near(want, 0.5773322, 1e-6);
  const ramp = Float64Array.from({ length: 30 }, (_, i) => 5 + 0.7 * i), r1 = adaptiveGaussian(ramp, 1, { a: 3, b: 26 });
  for (let i = 3; i <= 26; i++) near(r1[i], ramp[i], 1e-12);
  const r2 = adaptiveGaussian(Float64Array.from({ length: 30 }, (_, i) => -10 + 0.7 * i), 1, { median: 'medoid', a: 3, b: 26 });
  for (let i = 3; i <= 26; i++) near(r2[i], -10 + 0.7 * i, 1e-12);
  const v3 = Float64Array.from({ length: 60 }, (_, i) => [1, 2, 3][i % 3] * 2);
  nearVec(adaptiveGaussian(v3, 3), v3, 1e-12);
  const n = 60, tr = new Float64Array(n * 4);
  for (let f = 0; f < n; f++) tr.set(q4.axis([0, 0, 1], 0.02 * f), f * 4);
  tr.set(q4.axis([1, 0, 0], 0.9), 25 * 4);
  const fq = adaptiveGaussianQuat(tr, { window: 5 });
  assert.ok(q4.angle(Array.from(fq.subarray(100, 104)), q4.axis([0, 0, 1], 0.5)) < q4.angle(Array.from(tr.subarray(100, 104)), q4.axis([0, 0, 1], 0.5)) * 0.25);
  assert.ok(q4.angle(Array.from(fq.subarray(200, 204)), q4.axis([0, 0, 1], 1.0)) < 1e-3);
});

test('track helpers read and write the app layout (f*nb+i)*comps', () => {
  const nb = 3, n = 4, arr = Float32Array.from({ length: n * nb * 4 }, (_, i) => i);
  const t = readTrack(arr, nb, 1, 4);
  assert.equal(t.length, n * 4);
  assert.equal(t[4], arr[(1 * nb + 1) * 4]);
  const w = writeTrack(new Float32Array(arr.length), nb, 1, 4, t);
  assert.equal(w[(2 * nb + 1) * 4 + 3], arr[(2 * nb + 1) * 4 + 3]);
  near(TPS / 240, 588000, 0);
  assert.equal(butterworthCoefficients(7, 1 / 240, 4).length, 2);
  assert.equal(butterworthCoefficients(7, 1 / 240, 3).length, 2);
});
