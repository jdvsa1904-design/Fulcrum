import test from 'node:test';
import assert from 'node:assert/strict';
import { near, nearVec, rng } from './helpers.mjs';
import { detectFlights, fitBallistic, fitBallisticLeastSquares, ballisticOffsets, flightReport } from '../ballistic.js';

const G = 981, FPS = 30;
function arc(n, a, p0, v0, g = G) {
  const c = new Float64Array(n * 3);
  for (let f = 0; f < n; f++) { const t = Math.max(0, f - a) / FPS; c.set([p0[0] + v0[0] * t, p0[1] + v0[1] * t - (f > a ? g * t * t / 2 : 0), p0[2] + v0[2] * t], f * 3); }
  return c;
}

test('detectFlights: a flight runs from the last contact frame to the first landing frame', () => {
  const any = Uint8Array.from([1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1]);
  assert.deepEqual(detectFlights(any, 3), [{ a: 1, b: 6 }]);
  assert.deepEqual(detectFlights(any, 2), [{ a: 1, b: 6 }, { a: 7, b: 10 }]);
  assert.deepEqual(detectFlights(Uint8Array.from([0, 0, 0, 1, 1, 0, 0, 0, 0]), 3), [], 'no take-off before the take starts, no landing at the end');
  assert.deepEqual(detectFlights(new Uint8Array(10).fill(1), 1), []);
});

test('fitBallistic: an exact arc gives back the take-off point, velocity, apex and both endpoints', () => {
  const p0 = [10, 100, -5], v0 = [120, 300, 40], a = 5, b = 23, c = arc(30, a, p0, v0);
  const fit = fitBallistic(c, a, b, FPS, G);
  nearVec(fit.p0, p0, 1e-9); nearVec(fit.v0, v0, 1e-9);
  near(fit.T, (b - a) / FPS, 1e-12);
  near(fit.apexT, v0[1] / G, 1e-12);
  near(fit.apexY, p0[1] + v0[1] * v0[1] / (2 * G), 1e-9);
  nearVec(fit.at(b), Array.from(c.subarray(b * 3, b * 3 + 3)), 1e-9);
  nearVec(fit.at(14), Array.from(c.subarray(42, 45)), 1e-9);
  const short = fitBallistic(arc(30, 0, [0, 0, 0], [0, 50, 0]), 0, 2, FPS, G);
  near(short.apexT, Math.min(short.T, 50 / G), 1e-12, 'apex time is capped at the flight end');
});

test('fitBallisticLeastSquares: exact on a clean arc, and closer than the endpoint fit when the ends are noisy', () => {
  const p0 = [0, 100, 0], v0 = [200, 250, 0], a = 0, b = 20, c = arc(21, a, p0, v0);
  const ls = fitBallisticLeastSquares(c, a, b, FPS, G);
  nearVec(ls.p0, p0, 1e-6); nearVec(ls.v0, v0, 1e-6);
  const noisy = Float64Array.from(c), r = rng(3);
  for (let f = 0; f <= b; f++) for (let k = 0; k < 3; k++) noisy[f * 3 + k] += (r() - 0.5) * 2;
  noisy[b * 3 + 1] += 6;
  const e = v => Math.hypot(v[0] - v0[0], v[1] - v0[1], v[2] - v0[2]);
  assert.ok(e(fitBallisticLeastSquares(noisy, a, b, FPS, G).v0) < e(fitBallistic(noisy, a, b, FPS, G).v0) / 2);
});

test('ballisticOffsets: a floaty hold is moved onto the arc, ends untouched, two-frame blends', () => {
  const n = 40, a = 10, b = 30, c = new Float64Array(n * 3);
  for (let f = 0; f < n; f++) c.set([f * 2, f > a && f < b ? 140 : 100, 0], f * 3);
  const off = ballisticOffsets(c, [{ a, b }], FPS, G);
  const fit = fitBallistic(c, a, b, FPS, G);
  for (let f = 0; f <= a; f++) assert.equal(off[f], null);
  for (let f = b; f < n; f++) assert.equal(off[f], null);
  for (let f = a + 2; f <= b - 2; f++) nearVec(off[f].map((v, k) => c[f * 3 + k] + v), fit.at(f), 1e-9, `f${f} on the arc`);
  for (const f of [a + 1, b - 1]) nearVec(off[f], fit.at(f).map((v, k) => (v - c[f * 3 + k]) * 0.5), 1e-9, `f${f} half blend`);
  const ls = ballisticOffsets(c, [{ a, b }], FPS, G, 'fit'), lf = fitBallisticLeastSquares(c, a, b, FPS, G);
  nearVec(ls[20].map((v, k) => c[60 + k] + v), lf.at(20), 1e-9);
  assert.ok(ballisticOffsets(c, [{ a: 5, b: 6 }, { a: -1, b: 9 }, { a: 30, b: 45 }], FPS, G).every(o => o === null), 'too short or out of range');
});

test('flightReport: airtime, apex, take-off speed and angle, distance in metres', () => {
  const p0 = [0, 100, 0], v0 = [300, 400, 0], T = 0.9, b = Math.round(T * FPS), c = arc(b + 1, 0, p0, v0);
  const r = flightReport(fitBallistic(c, 0, b, FPS, G), 100);
  near(r.airtime_s, T, 1e-12);
  near(r.apex_m_above_takeoff, 400 * 400 / (2 * G) / 100, 1e-9);
  near(r.takeoff_speed_ms, 5, 1e-9);
  near(r.takeoff_angle_deg, Math.atan2(4, 3) * 180 / Math.PI, 1e-9);
  near(r.distance_m, 3 * T, 1e-9);
  const m = flightReport(fitBallistic(c.map(v => v / 100), 0, b, FPS, G / 100), 1);
  near(m.takeoff_speed_ms, 5, 1e-9, 'units per metre scale out');
});
