import test from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { json, exists, q4, rng, deg, rad } from './helpers.mjs';
import { applyCleanup, reduceKeysReport } from '../wire/cleanup.js';

const here = dirname(fileURLToPath(import.meta.url));
const FRAMES = join(here, '..', 'exports', 'FRT_fulcrum_v6.frames.json');

const NB = 4, N = 120, FPS = 30;

function cleanTake() {
  const q = new Float32Array(N * NB * 4), p = new Float32Array(N * NB * 3);
  for (let f = 0; f < N; f++) {
    const t = f / FPS;
    for (let i = 0; i < NB; i++) {
      const ax = [Math.sin(i + 1), Math.cos(2 * i + 0.5), 0.4 + 0.1 * i];
      const qq = q4.axis(ax, rad(20 + 10 * i) * Math.sin(2 * Math.PI * (0.6 + 0.2 * i) * t) + rad(15 * i));
      q.set(qq, (f * NB + i) * 4);
      if (i === 0) p.set([30 * t, 90 + 2 * Math.sin(2 * Math.PI * 1.5 * t), 5 * Math.cos(2 * Math.PI * 0.5 * t)], f * NB * 3);
      else p.set([0, 20 * i, 0], (f * NB + i) * 3);
    }
  }
  return { q, p };
}

const getQ = (q, f, i) => Array.from(q.subarray((f * NB + i) * 4, (f * NB + i) * 4 + 4));
const getP = (p, f, i) => Array.from(p.subarray((f * NB + i) * 3, (f * NB + i) * 3 + 3));
const finite = a => a.every(Number.isFinite);

function maxAngleDeg(qa, qb, nb, from = 0, to = Infinity) {
  let m = 0;
  const n = qa.length / (nb * 4);
  for (let f = from; f <= Math.min(to, n - 1); f++) for (let i = 0; i < nb; i++) {
    const o = (f * nb + i) * 4;
    m = Math.max(m, deg(q4.angle(Array.from(qa.subarray(o, o + 4)), Array.from(qb.subarray(o, o + 4)))));
  }
  return m;
}

function meanAngleDeg(qa, qb, nb, from, to) {
  let s = 0, c = 0;
  for (let f = from; f <= to; f++) for (let i = 0; i < nb; i++) {
    const o = (f * nb + i) * 4;
    s += deg(q4.angle(Array.from(qa.subarray(o, o + 4)), Array.from(qb.subarray(o, o + 4)))); c++;
  }
  return s / c;
}

function noisy(seed, rotDeg, posAmp) {
  const c = cleanTake(), r = rng(seed), q = new Float32Array(c.q), p = new Float32Array(c.p);
  for (let f = 0; f < N; f++) for (let i = 0; i < NB; i++) {
    const o = (f * NB + i) * 4, nq = q4.mul(q4.axis([r() - 0.5, r() - 0.5, r() - 0.5], rad(rotDeg) * (r() * 2 - 1)), Array.from(q.subarray(o, o + 4)));
    q.set(nq, o);
  }
  for (let f = 0; f < N; f++) for (let k = 0; k < 3; k++) p[f * NB * 3 + k] += posAmp * (r() * 2 - 1);
  return { clean: c, q, p };
}

test('single-frame quaternion pop is removed by despike', () => {
  const c = cleanTake(), q = new Float32Array(c.q);
  const o = (60 * NB + 2) * 4;
  q.set(q4.mul(q4.axis([0.3, 1, 0.2], rad(25)), getQ(q, 60, 2)), o);
  assert.ok(deg(q4.angle(getQ(q, 60, 2), getQ(c.q, 60, 2))) > 24);
  const r = applyCleanup({ q, p: c.p, nb: NB, n: N, fps: FPS, fixes: [{ type: 'despike', a: 0, b: N - 1 }] });
  const err = deg(q4.angle(getQ(r.q, 60, 2), getQ(c.q, 60, 2)));
  assert.ok(err < 0.5, `pop residual ${err} deg`);
  assert.ok(maxAngleDeg(r.q, c.q, NB) < 0.5);
  const rep = r.report[0];
  assert.equal(rep.type, 'despike');
  assert.equal(rep.bones, 1);
  assert.equal(rep.rotCells, rep.frames);
  assert.ok(rep.frames >= 1 && rep.frames <= 3, `frames ${rep.frames}`);
  assert.ok(rep.maxDeg > 20 && rep.maxDeg < 30);
  assert.equal(rep.posCells, 0);
  assert.ok(finite(r.q) && finite(r.p));
});

test('single-frame root position pop is removed by despike', () => {
  const c = cleanTake(), p = new Float32Array(c.p);
  p[(40 * NB) * 3 + 1] += 25;
  const r = applyCleanup({ q: c.q, p, nb: NB, n: N, fps: FPS, fixes: [{ type: 'despike', a: 0, b: N - 1 }] });
  const d = Math.hypot(...getP(r.p, 40, 0).map((v, k) => v - getP(c.p, 40, 0)[k]));
  assert.ok(d < 0.2, `root residual ${d}`);
  assert.equal(r.report[0].bones, 1);
  assert.ok(r.report[0].posCells >= 1 && r.report[0].posCells <= 3);
});

test('clean motion is unchanged by despike within 0.01 deg and reports nothing', () => {
  const c = cleanTake();
  const r = applyCleanup({ q: c.q, p: c.p, nb: NB, n: N, fps: FPS, fixes: [{ type: 'despike', a: 0, b: N - 1 }] });
  assert.ok(maxAngleDeg(r.q, c.q, NB) < 0.01);
  assert.deepEqual(Array.from(r.p), Array.from(c.p));
  assert.equal(r.report[0].frames, 0);
  assert.equal(r.report[0].bones, 0);
});

test('despike leaves sharp but genuine motion alone (fast swing peaks, impact kinks)', () => {
  const n = 96, nb = 2, q = new Float32Array(n * nb * 4), p = new Float32Array(n * nb * 3);
  for (let f = 0; f < n; f++) {
    const swing = rad(70) * Math.sin(2 * Math.PI * 2.2 * f / 24) * Math.exp(-((f - 48) ** 2) / 400);
    const kink = rad(2) * Math.max(0, f - 40) * (f < 60 ? 1 : 0) + (f >= 60 ? rad(40) : 0);
    q.set(q4.axis([1, 0.2, 0], swing), f * nb * 4);
    q.set(q4.axis([0, 1, 0.3], kink), (f * nb + 1) * 4);
    p.set([f < 50 ? f * 4 : 200, 90, 0], f * nb * 3);
    p.set([0, 10, 0], (f * nb + 1) * 3);
  }
  const r = applyCleanup({ q, p, nb, n, fps: 24, fixes: [{ type: 'despike', a: 0, b: n - 1 }] });
  assert.ok(maxAngleDeg(r.q, q, nb) < 0.01, `changed ${maxAngleDeg(r.q, q, nb)}`);
  assert.deepEqual(Array.from(r.p), Array.from(p));
  assert.equal(r.report[0].frames, 0);
});

test('no fixes, unknown fixes and too-short ranges are exact no-ops', () => {
  const c = cleanTake();
  for (const fixes of [[], undefined, [{ type: 'momentum', a: 0, b: 50 }], [{ type: 'filter', a: 10, b: 11 }]]) {
    const r = applyCleanup({ q: c.q, p: c.p, nb: NB, n: N, fps: FPS, fixes });
    assert.deepEqual(Array.from(r.q), Array.from(c.q));
    assert.deepEqual(Array.from(r.p), Array.from(c.p));
    assert.ok(r.report.every(x => x.frames === 0));
  }
});

test('butterworth filter keeps range ends and outside frames, eases in, reduces jitter', () => {
  const { clean, q, p } = noisy(7, 1.5, 0.5), a = 20, b = 90;
  const r = applyCleanup({ q, p, nb: NB, n: N, fps: FPS, fixes: [{ type: 'filter', a, b, fc: 5 }] });
  for (const f of [...Array.from({ length: a + 1 }, (_, k) => k), ...Array.from({ length: N - b }, (_, k) => b + k)]) {
    for (let i = 0; i < NB; i++) {
      assert.deepEqual(getQ(r.q, f, i), getQ(q, f, i), `q f${f} b${i}`);
      assert.deepEqual(getP(r.p, f, i), getP(p, f, i), `p f${f} b${i}`);
    }
  }
  const before = maxAngleDeg(q, clean.q, NB, a + 6, b - 6), after = maxAngleDeg(r.q, clean.q, NB, a + 6, b - 6);
  assert.ok(after < before * 0.6, `jitter ${before} -> ${after}`);
  const step1 = deg(q4.angle(getQ(r.q, a + 1, 1), getQ(q, a + 1, 1))), stepMid = maxAngleDeg(r.q, q, NB, 50, 50);
  assert.ok(step1 < stepMid + 1e-9, 'ease limits first-frame change');
  const rep = r.report[0];
  assert.equal(rep.type, 'filter');
  assert.equal(rep.bones, NB);
  assert.ok(rep.frames > 0 && rep.frames <= b - a - 1);
  assert.ok(rep.posCells > 0 && rep.posCells <= b - a - 1);
  assert.ok(finite(r.q) && finite(r.p));
});

test('median (adaptive gaussian) reduces jitter and stays in range', () => {
  const { clean, q, p } = noisy(11, 2, 0), a = 10, b = 100;
  const r = applyCleanup({ q, p, nb: NB, n: N, fps: FPS, fixes: [{ type: 'median', a, b, window: 5 }] });
  const before = meanAngleDeg(q, clean.q, NB, a + 3, b - 3), after = meanAngleDeg(r.q, clean.q, NB, a + 3, b - 3);
  assert.ok(after < before, `${before} -> ${after}`);
  assert.ok(maxAngleDeg(r.q, q, NB, 0, a - 1) < 1e-9);
  assert.ok(maxAngleDeg(r.q, q, NB, b + 1, N - 1) < 1e-9);
  assert.ok(finite(r.q));
});

test('bones option limits which tracks change', () => {
  const { q, p } = noisy(3, 1.5, 0.5);
  const names = ['pelvis', 'thigh_twist_01_l', 'calf_l', 'foot_l'];
  const rMajor = applyCleanup({ q, p, nb: NB, n: N, fps: FPS, names, fixes: [{ type: 'filter', a: 0, b: N - 1, bones: 'major' }] });
  assert.equal(maxAngleDeg(rMajor.q.subarray(0), q, NB, 0, N - 1) > 0, true);
  for (let f = 0; f < N; f++) assert.deepEqual(getQ(rMajor.q, f, 1), getQ(q, f, 1));
  assert.equal(rMajor.report[0].bones, 3);
  const rIdx = applyCleanup({ q, p, nb: NB, n: N, fps: FPS, fixes: [{ type: 'filter', a: 0, b: N - 1, bones: [2] }] });
  assert.equal(rIdx.report[0].bones, 1);
  for (let f = 0; f < N; f++) assert.deepEqual(getQ(rIdx.q, f, 0), getQ(q, f, 0));
});

test('inputs are never mutated and outputs are copies', () => {
  const { q, p } = noisy(5, 3, 1);
  q.set(q4.mul(q4.axis([1, 0, 0], rad(30)), getQ(q, 50, 3)), (50 * NB + 3) * 4);
  const q0 = Array.from(q), p0 = Array.from(p);
  const fixes = [{ type: 'despike', a: 0, b: N - 1 }, { type: 'filter', a: 5, b: 110 }, { type: 'median', a: 30, b: 80 }];
  const f0 = JSON.stringify(fixes);
  const r = applyCleanup({ q, p, nb: NB, n: N, fps: FPS, fixes });
  assert.deepEqual(Array.from(q), q0);
  assert.deepEqual(Array.from(p), p0);
  assert.equal(JSON.stringify(fixes), f0);
  assert.notEqual(r.q, q);
  assert.notEqual(r.p, p);
  assert.ok(r.q instanceof Float32Array && r.p instanceof Float32Array);
  assert.equal(r.report.length, 3);
  assert.deepEqual(r.report.map(x => x.fix), [0, 1, 2]);
});

test('quaternion sign continuity is kept even when input signs flip', () => {
  const { q, p } = noisy(9, 1, 0);
  for (let f = 0; f < N; f += 3) for (let i = 0; i < NB; i++) { const o = (f * NB + i) * 4; for (let k = 0; k < 4; k++) q[o + k] = -q[o + k]; }
  const r = applyCleanup({ q, p, nb: NB, n: N, fps: FPS, fixes: [{ type: 'filter', a: 10, b: 100 }] });
  for (let i = 0; i < NB; i++) for (let f = 12; f < 99; f++) {
    const d = getQ(r.q, f, i).reduce((s, x, k) => s + x * getQ(r.q, f + 1, i)[k], 0);
    assert.ok(d > 0, `flip at f${f} bone ${i}`);
  }
});

test('fixes survive JSON save/open round trip with identical result', () => {
  const { q, p } = noisy(13, 1.5, 0.5);
  const fixes = [{ type: 'despike', a: 0, b: N - 1, bones: 'all' }, { type: 'filter', a: 10, b: 100, fc: 6 }, { type: 'median', a: 40, b: 60, window: 7 }];
  const r1 = applyCleanup({ q, p, nb: NB, n: N, fps: FPS, fixes });
  const r2 = applyCleanup({ q, p, nb: NB, n: N, fps: FPS, fixes: JSON.parse(JSON.stringify(fixes)) });
  assert.deepEqual(Array.from(r1.q), Array.from(r2.q));
  assert.deepEqual(r1.report, r2.report);
});

test('reduceKeysReport: constant tracks keep 2 keys, motion keeps more, W and q agree', () => {
  const nb = 2, n = 60, q = new Float32Array(n * nb * 4), p = new Float32Array(n * nb * 3), W = new Float32Array(n * nb * 16);
  for (let f = 0; f < n; f++) {
    const qa = [0, 0, 0, 1], qb = q4.axis([0, 1, 0], rad(40) * Math.sin(f / 6));
    q.set(qa, f * nb * 4); q.set(qb, (f * nb + 1) * 4);
    p.set([0, 0, 0], f * nb * 3); p.set([0, 0, 0], (f * nb + 1) * 3);
    for (const [i, qq] of [[0, qa], [1, qb]]) {
      const [x, y, z, w] = qq, o = (f * nb + i) * 16;
      W.set([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0, 2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0, 2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0, 0, 0, 0, 1], o);
    }
  }
  const rq = reduceKeysReport({ q, p, nb, n, fps: 30 }), rw = reduceKeysReport({ W, nb, n, fps: 30 });
  assert.equal(rq.keptPerBone[0], 2);
  assert.ok(rq.keptPerBone[1] > 2 && rq.keptPerBone[1] < n);
  assert.equal(rq.totalFrames, n * nb);
  assert.equal(rq.totalKept, rq.keptPerBone[0] + rq.keptPerBone[1]);
  assert.deepEqual(rw.keptPerBone, rq.keptPerBone);
  const loose = reduceKeysReport({ q, p, nb, n, fps: 30, precisionDeg: 2 });
  assert.ok(loose.keptPerBone[1] <= rq.keptPerBone[1]);
});

test('real FRT v6 export: cleanup on world tracks has no NaN, reports, key report', { skip: !exists(FRAMES) }, () => {
  const J = json(FRAMES), nb = J.bones.length, n = J.frames.length;
  const W = new Float32Array(n * nb * 16);
  J.frames.forEach((fr, f) => W.set(fr, f * nb * 16));
  const q = new Float32Array(n * nb * 4), p = new Float32Array(n * nb * 3);
  for (let f = 0; f < n; f++) for (let i = 0; i < nb; i++) {
    const o = (f * nb + i) * 16, c = k => Math.hypot(W[o + k * 4], W[o + k * 4 + 1], W[o + k * 4 + 2]);
    const s = [c(0), c(1), c(2)], m = (r, cc) => W[o + cc * 4 + r] / s[cc];
    const tr = m(0, 0) + m(1, 1) + m(2, 2);
    let qq;
    if (tr > 0) { const t = 0.5 / Math.sqrt(tr + 1); qq = [(m(2, 1) - m(1, 2)) * t, (m(0, 2) - m(2, 0)) * t, (m(1, 0) - m(0, 1)) * t, 0.25 / t]; }
    else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) { const t = 2 * Math.sqrt(1 + m(0, 0) - m(1, 1) - m(2, 2)); qq = [0.25 * t, (m(0, 1) + m(1, 0)) / t, (m(0, 2) + m(2, 0)) / t, (m(2, 1) - m(1, 2)) / t]; }
    else if (m(1, 1) > m(2, 2)) { const t = 2 * Math.sqrt(1 + m(1, 1) - m(0, 0) - m(2, 2)); qq = [(m(0, 1) + m(1, 0)) / t, 0.25 * t, (m(1, 2) + m(2, 1)) / t, (m(0, 2) - m(2, 0)) / t]; }
    else { const t = 2 * Math.sqrt(1 + m(2, 2) - m(0, 0) - m(1, 1)); qq = [(m(0, 2) + m(2, 0)) / t, (m(1, 2) + m(2, 1)) / t, 0.25 * t, (m(1, 0) - m(0, 1)) / t]; }
    const l = Math.hypot(...qq);
    q.set(qq.map(x => x / l), (f * nb + i) * 4);
    p.set([W[o + 12], W[o + 13], W[o + 14]], (f * nb + i) * 3);
  }
  const t0 = Date.now();
  const r = applyCleanup({ q, p, nb, n, fps: J.fps, names: J.bones, fixes: [{ type: 'despike', a: 0, b: n - 1, bones: 'major' }, { type: 'filter', a: 0, b: n - 1, fc: 7 }] });
  const ms = Date.now() - t0;
  assert.ok(finite(r.q) && finite(r.p));
  const [ds, bw] = r.report;
  assert.ok(ds.maxDeg < 3, `despike on the clean take moved ${ds.maxDeg} deg`);
  assert.ok(bw.maxDeg < 10, `filter max ${bw.maxDeg}`);
  const hand = J.bones.findIndex(b => b.toLowerCase() === 'hand_r'), qp = new Float32Array(q), o = (200 * nb + hand) * 4;
  qp.set(q4.mul(q4.axis([0, 0, 1], rad(30)), Array.from(qp.subarray(o, o + 4))), o);
  const rp = applyCleanup({ q: qp, p, nb, n, fps: J.fps, fixes: [{ type: 'despike', a: 150, b: 250, bones: [hand] }] });
  const res = deg(q4.angle(Array.from(rp.q.subarray(o, o + 4)), Array.from(q.subarray(o, o + 4))));
  assert.ok(res < 3, `injected 30 deg pop on hand_r left ${res} deg`);
  assert.ok(rp.report[0].spikes >= 1);
  const k = reduceKeysReport({ W, nb, n, fps: J.fps, upm: 91.33 });
  assert.equal(k.totalFrames, n * nb);
  assert.ok(k.totalKept > 0 && k.totalKept < k.totalFrames);
  assert.ok(k.keptPerBone.every(x => x >= 1 && x <= n));
  console.log(JSON.stringify({ ms, despike: ds, filter: bw, keys: { totalKept: k.totalKept, totalFrames: k.totalFrames, ratio: +k.ratio.toFixed(3) } }));
});
