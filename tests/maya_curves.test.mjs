import test from 'node:test';
import assert from 'node:assert/strict';
import { json, near } from './helpers.mjs';
import { evaluate, computeTangents, makeCurve, hermiteCoeffs, hermiteEval, bezierSegment, bezierEval, checkMonotonic, autoSlope } from '../re/maya/curves.js';

const fx = json(new URL('./fixtures/maya_curves.json', import.meta.url));
const TICKS = 141120000;

const toCurve = c => ({
  weighted: c.weighted, preInfinity: c.preName ?? 'constant', postInfinity: c.postName ?? 'constant',
  keys: c.keys.map(k => ({ t: k.t, v: k.v, inType: k.itt, outType: k.ott, inAngle: k.ia, outAngle: k.oa, inWeight: k.iw, outWeight: k.ow })),
});
const tick = (t, fps) => Math.round(t * TICKS / fps) / (TICKS / fps);

function maxEvalError(c) {
  const cur = toCurve(c);
  let m = 0;
  for (const [t, v] of c.samples) m = Math.max(m, Math.abs(evaluate(cur, tick(t, c.fps)) - v));
  return m;
}

test('fixture covers every tangent type, both curve kinds and all infinity modes', () => {
  const kinds = new Set(fx.cases.map(c => c.kind));
  for (const k of ['fixed', 'spline', 'linear', 'flat', 'clamped', 'plateau', 'auto', 'autoease', 'automix', 'autocustom', 'step', 'stepnext']) assert.ok(kinds.has(k), k);
  const inf = new Set(fx.cases.flatMap(c => [c.preName, c.postName]).filter(Boolean));
  for (const m of ['constant', 'linear', 'cycle', 'cycleRelative', 'oscillate']) assert.ok(inf.has(m), m);
  assert.ok(fx.cases.some(c => c.weighted) && fx.cases.some(c => !c.weighted));
});

test('unweighted (Hermite) evaluation matches Maya on every fixture curve', () => {
  for (const c of fx.cases.filter(c => !c.weighted)) near(maxEvalError(c), 0, 1e-10, `${c.kind} ${c.unit}`);
});

test('weighted (Bezier) evaluation matches Maya, including out-of-range handles', () => {
  for (const c of fx.cases.filter(c => c.weighted)) near(maxEvalError(c), 0, 1e-8, `${c.kind} ${c.unit}`);
});

test('infinity modes match Maya outside the key range', () => {
  const inf = fx.cases.filter(c => c.preName);
  assert.ok(inf.length >= 20);
  for (const c of inf) near(maxEvalError(c), 0, 1e-8, `${c.preName}/${c.postName}`);
});

test('computed tangents match Maya for every automatic tangent type', () => {
  for (const c of fx.cases.filter(c => c.kind !== 'fixed')) {
    const cur = toCurve(c);
    const blank = { ...cur, keys: cur.keys.map(k => ({ ...k, inAngle: 0, outAngle: 0, inWeight: 1, outWeight: 1 })) };
    const got = computeTangents(blank, { fps: c.fps });
    got.keys.forEach((k, i) => {
      const m = cur.keys[i];
      near(k.inAngle, m.inAngle, 1e-9, `${c.kind} k${i} in`);
      near(k.outAngle, m.outAngle, 1e-9, `${c.kind} k${i} out`);
      if (c.weighted) {
        near(k.inWeight, m.inWeight, 1e-9, `${c.kind} k${i} inW`);
        near(k.outWeight, m.outWeight, 1e-9, `${c.kind} k${i} outW`);
      }
    });
  }
});

test('Hermite segment hits both keys and both slopes', () => {
  const h = hermiteCoeffs(2, 1, 0.5, -0.25, 12, 4);
  near(hermiteEval(h, 2), 1, 1e-12);
  near(hermiteEval(h, 12), 4, 1e-12);
  const e = 1e-6;
  near((hermiteEval(h, 2 + e) - hermiteEval(h, 2)) / e, 0.5, 1e-5);
  near((hermiteEval(h, 12) - hermiteEval(h, 12 - e)) / e, -0.25, 1e-5);
});

test('Bezier segment with thirds handles is linear in time', () => {
  const b = bezierSegment(0, 0, 10 / 3, 5, 20 / 3, 5, 10, 0);
  assert.equal(b.linearTime, true);
  near(bezierEval(b, 5), 3.75, 1e-12);
});

test('checkMonotonic pulls overlong handles back inside the monotonic region', () => {
  const [u1, u2] = checkMonotonic(1.8, 0.9, true);
  const a = -3 * u2 + 3 * u1 + 1, b = 3 * u2 - 6 * u1, c = 3 * u1;
  const disc = b * b - 3 * a * c;
  assert.ok(disc <= 1e-9 || [(-b + Math.sqrt(disc)) / (3 * a), (-b - Math.sqrt(disc)) / (3 * a)].every(r => r <= 0 || r >= 1));
});

test('auto tangents never overshoot: the slope is clamped to 3x the neighbouring secants', () => {
  const p = { t: 0, v: 0 }, k = { t: 1, v: 10 }, n = { t: 10, v: 10.5 };
  for (const ty of ['auto', 'autoease', 'automix']) {
    const s = autoSlope(p, k, n, ty);
    assert.ok(s <= 3 * 0.5 / 9 + 1e-12, ty);
  }
  near(autoSlope({ t: 0, v: 0 }, { t: 1, v: 5 }, { t: 2, v: 0 }, 'auto'), 0, 0);
});

test('makeCurve builds a usable curve from keys alone', () => {
  const c = makeCurve([{ t: 0, v: 0 }, { t: 10, v: 5 }, { t: 14, v: 6 }, { t: 30, v: -2 }], { fps: 24 });
  near(evaluate(c, 10), 5, 1e-12);
  near(c.keys[1].inAngle, 23.198590513648185, 1e-9);
  near(c.keys[0].outAngle, 0, 0);
});
