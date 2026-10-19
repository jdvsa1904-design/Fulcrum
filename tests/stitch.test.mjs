import test from 'node:test';
import assert from 'node:assert/strict';
import { near, nearVec, v3, q4, rng } from './helpers.mjs';
import { gravityMatch, applyOffset, crossfade, stitchShots, transformAt, yawArc, smoothstep } from '../stitch.js';

const NB = 3, SPEED = 0.05, TURN = 0.02;
const Y = [0, 1, 0], X = [1, 0, 0], Z = [0, 0, 1];

const path = (() => {
  const out = [[0, 0]];
  for (let t = 1; t < 600; t++) {
    const h = TURN * (t - 1);
    out.push([out[t - 1][0] + SPEED * Math.cos(h), out[t - 1][1] - SPEED * Math.sin(h)]);
  }
  return out;
})();

function walkFrame(t) {
  const h = TURN * t;
  const rq = q4.mul(q4.axis(Y, h), q4.axis(Z, 0.2));
  return {
    p: [[path[t][0], 1 + 0.02 * Math.sin(0.5 * t), path[t][1]], [0, 0.5, 0], [0.2, -0.5, 0]],
    q: [rq, q4.axis(X, 0.3 * Math.sin(0.3 * t)), q4.axis(Z, 0.4 * Math.cos(0.2 * t))],
  };
}

function shotFrom(a, len, yaw = 0, move = [0, 0, 0], F32 = false) {
  const n = len, p = new (F32 ? Float32Array : Float64Array)(n * NB * 3), q = new (F32 ? Float32Array : Float64Array)(n * NB * 4);
  const qy = q4.axis(Y, yaw);
  for (let f = 0; f < n; f++) {
    const w = walkFrame(a + f);
    for (let i = 0; i < NB; i++) {
      const pp = i === 0 ? v3.add(q4.rot(qy, w.p[i]), move) : w.p[i];
      const qq = i === 0 ? q4.mul(qy, w.q[i]) : w.q[i];
      p.set(pp, (f * NB + i) * 3);
      q.set(qq, (f * NB + i) * 4);
    }
  }
  return { n, nb: NB, p, q };
}

const heading = q => { const v = q4.rot(q, X); return Math.atan2(-v[2], v[0]); };
const wrap = a => a - 2 * Math.PI * Math.round(a / (2 * Math.PI));

test('gravity match puts B first frame on A last frame in x, z and heading, and keeps B height', () => {
  const r = rng(5);
  for (let k = 0; k < 40; k++) {
    const A = shotFrom(r() * 100 | 0, 30, (r() - 0.5) * 6, [r() * 5, 0.3, r() * 5]);
    const B = shotFrom(r() * 100 | 0, 30, (r() - 0.5) * 6, [r() * 5, 0.7, r() * 5]);
    const a = transformAt(A, 29, 0), b = transformAt(B, 0, 0);
    const { offset, matched } = gravityMatch(a, b);
    const M = applyOffset(B, offset);
    const m = transformAt(M, 0, 0);
    near(m.p[0], a.p[0], 1e-9);
    near(m.p[2], a.p[2], 1e-9);
    near(m.p[1], b.p[1], 1e-12);
    near(wrap(heading(m.q) - heading(a.q)), 0, 1e-9);
    nearVec(m.p, matched.p, 1e-9);
    assert.ok(q4.angle(m.q, matched.q) < 1e-9);
  }
});

test('offset rotation is a pure turn about world Y along the shortest arc', () => {
  const r = rng(9);
  for (let k = 0; k < 40; k++) {
    const a = { p: [1, 2, 3], q: q4.mul(q4.axis(Y, (r() - 0.5) * 6.2), q4.axis(Z, r() - 0.5)) };
    const b = { p: [4, 1, -2], q: q4.mul(q4.axis(Y, (r() - 0.5) * 6.2), q4.axis(X, r() - 0.5)) };
    const R = gravityMatch(a, b).offset.R;
    near(R[0], 0, 1e-12); near(R[2], 0, 1e-12);
    const th = 2 * Math.atan2(Math.abs(R[1]), Math.abs(R[3]));
    assert.ok(th <= Math.PI + 1e-12);
    near(wrap(2 * Math.atan2(R[1], R[3])), wrap(heading(a.q) - heading(b.q)), 1e-9);
  }
  const R = yawArc(q4.axis(Y, 3), q4.axis(Y, -3));
  near(Math.abs(2 * Math.atan2(R[1], R[3])), 2 * Math.PI - 6, 1e-12);
});

test('a vertical match axis degrades to identity yaw', () => {
  const up = q4.axis(Z, Math.PI / 2);
  nearVec(yawArc(up, q4.axis(Y, 1)), [0, 0, 0, 1], 1e-12);
  nearVec(yawArc(q4.axis(Y, 1), up), [0, 0, 0, 1], 1e-12);
});

test('applying the offset to all bones is rigid: pairwise distances and relative rotations survive', () => {
  const B = shotFrom(7, 40, 1.1, [3, 0.4, -2]);
  const a = { p: [-4, 1.5, 9], q: q4.mul(q4.axis(Y, -2.2), q4.axis(Z, 0.2)) };
  const { offset } = gravityMatch(a, transformAt(B, 0, 0));
  const M = applyOffset(B, offset, { all: true });
  for (let f = 0; f < B.n; f++) {
    for (let i = 0; i < NB; i++) for (let j = i + 1; j < NB; j++) {
      const d0 = v3.dist(transformAt(B, f, i).p, transformAt(B, f, j).p), d1 = v3.dist(transformAt(M, f, i).p, transformAt(M, f, j).p);
      near(d0, d1, 1e-9);
      const r0 = q4.angle(transformAt(B, f, i).q, transformAt(B, f, j).q), r1 = q4.angle(transformAt(M, f, i).q, transformAt(M, f, j).q);
      near(r0, r1, 1e-9);
    }
    near(transformAt(M, f, 0).p[1], transformAt(B, f, 0).p[1], 1e-12);
  }
});

test('root-only offset leaves the other bones and the input untouched', () => {
  const B = shotFrom(3, 20, 0.5, [1, 0, 1]);
  const snap = Float64Array.from(B.p), snapQ = Float64Array.from(B.q);
  const { offset } = gravityMatch(transformAt(shotFrom(0, 5, 2, [9, 0, 9]), 4, 0), transformAt(B, 0, 0));
  const M = applyOffset(B, offset);
  assert.deepEqual(B.p, snap);
  assert.deepEqual(B.q, snapQ);
  for (let f = 0; f < B.n; f++) for (const i of [1, 2]) {
    nearVec(transformAt(M, f, i).p, transformAt(B, f, i).p, 0);
    nearVec(transformAt(M, f, i).q, transformAt(B, f, i).q, 0);
  }
});

test('crossfade endpoints equal each source and the middle follows smoothstep', () => {
  const A = shotFrom(0, 30), B = shotFrom(200, 30, 1.3, [5, 2, 5]);
  const ov = 9, C = crossfade(A, B, ov);
  assert.equal(C.n, A.n + B.n - ov);
  for (let f = 0; f < A.n - ov; f++) for (let i = 0; i < NB; i++) {
    nearVec(transformAt(C, f, i).p, transformAt(A, f, i).p, 1e-12);
    assert.ok(q4.angle(transformAt(C, f, i).q, transformAt(A, f, i).q) < 1e-12);
  }
  for (let i = 0; i < NB; i++) {
    nearVec(transformAt(C, A.n - ov, i).p, transformAt(A, A.n - ov, i).p, 1e-12);
    assert.ok(q4.angle(transformAt(C, A.n - ov, i).q, transformAt(A, A.n - ov, i).q) < 1e-12);
    nearVec(transformAt(C, A.n - 1, i).p, transformAt(B, ov - 1, i).p, 1e-12);
    assert.ok(q4.angle(transformAt(C, A.n - 1, i).q, transformAt(B, ov - 1, i).q) < 1e-12);
  }
  for (let f = ov; f < B.n; f++) for (let i = 0; i < NB; i++) {
    nearVec(transformAt(C, A.n - ov + f, i).p, transformAt(B, f, i).p, 1e-12);
    assert.ok(q4.angle(transformAt(C, A.n - ov + f, i).q, transformAt(B, f, i).q) < 1e-12);
  }
  for (let k = 0; k < ov; k++) {
    const w = smoothstep(k / (ov - 1)), pa = transformAt(A, A.n - ov + k, 0).p, pb = transformAt(B, k, 0).p;
    nearVec(transformAt(C, A.n - ov + k, 0).p, v3.add(v3.mul(pa, 1 - w), v3.mul(pb, w)), 1e-12);
  }
  near(smoothstep(0.5), 0.5, 1e-15);
});

test('crossfade slerps along the short arc and returns unit quaternions in Float64Array', () => {
  const A = shotFrom(0, 12), B = shotFrom(0, 12, 1.0);
  const C = crossfade(A, B, 12);
  assert.ok(C.q instanceof Float64Array && C.p instanceof Float64Array);
  for (let f = 0; f < C.n; f++) {
    const q = transformAt(C, f, 0).q, a = transformAt(A, f, 0).q, b = transformAt(B, f, 0).q;
    near(Math.hypot(...q), 1, 1e-12);
    const w = smoothstep(f / 11);
    near(q4.angle(a, q), w * 1.0, 1e-9);
    near(q4.angle(q, b), (1 - w) * 1.0, 1e-9);
  }
});

test('no quaternion sign flip at the seam, even when the later shot has the opposite sign', () => {
  for (const ov of [0, 1, 2, 7]) {
    const A = shotFrom(0, 25), B = shotFrom(25 - ov, 25, 0.8, [1, 0, 1]);
    const Bn = { ...B, q: B.q.map(x => -x) };
    for (const later of [B, Bn]) {
      const C = crossfade(A, later, ov);
      for (let i = 0; i < NB; i++) for (let f = 1; f < C.n; f++) {
        const a = transformAt(C, f - 1, i).q, b = transformAt(C, f, i).q;
        assert.ok(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] > 0, `ov ${ov} bone ${i} frame ${f}`);
      }
    }
    const P = crossfade(A, B, ov), N = crossfade(A, Bn, ov);
    for (let f = 0; f < P.n; f++) for (let i = 0; i < NB; i++) assert.ok(q4.angle(transformAt(P, f, i).q, transformAt(N, f, i).q) < 1e-9);
  }
});

test('overlap of zero is a plain concatenation and overlap clamps to the shorter shot', () => {
  const A = shotFrom(0, 10), B = shotFrom(10, 6);
  const C = crossfade(A, B, 0);
  assert.equal(C.n, 16);
  nearVec(transformAt(C, 10, 0).p, transformAt(B, 0, 0).p, 1e-12);
  assert.equal(crossfade(A, B, 99).n, 10);
});

test('stitched 3-shot walk has no root jump at the seams', () => {
  const r = rng(21);
  for (const ov of [0, 4, 8]) {
    const len = 40;
    const shots = [0, 1, 2].map(k => shotFrom(k * (len - ov) + (ov === 0 ? k * ov : 0), len, (r() - 0.5) * 6, [(r() - 0.5) * 40, 0, (r() - 0.5) * 40]));
    const S = stitchShots(shots, { overlap: ov });
    assert.equal(S.n, 3 * len - 2 * ov);
    assert.deepEqual(S.starts, [0, len - ov, 2 * (len - ov)]);
    let normal = 0, seam = 0;
    const seams = new Set(S.starts.slice(1).flatMap(s => [s - 1, s, s + 1, s + ov, s + ov + 1].filter(x => x >= 0)));
    for (let f = 1; f < S.n; f++) {
      const d = v3.dist(transformAt(S, f, 0).p, transformAt(S, f - 1, 0).p);
      if (seams.has(f)) seam = Math.max(seam, d); else normal = Math.max(normal, d);
    }
    assert.ok(normal < 0.07, `normal ${normal}`);
    assert.ok(seam < normal * 1.2 + 0.02, `ov ${ov} seam ${seam} normal ${normal}`);
    for (let f = 1; f < S.n; f++) {
      const a = transformAt(S, f - 1, 0).q, b = transformAt(S, f, 0).q;
      assert.ok(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] > 0);
      assert.ok(q4.angle(a, b) < 0.05, `turn jump ${q4.angle(a, b)} at ${f}`);
    }
  }
});

test('stitched shots follow the first shot frame for frame with every later shot matched to the end of the previous', () => {
  const ov = 6, len = 30;
  const shots = [0, 1, 2].map(k => shotFrom(k * (len - ov), len, k * 1.7, [k * 11, 0, -k * 7]));
  const S = stitchShots(shots, { overlap: ov });
  for (let f = 0; f < len; f++) nearVec(transformAt(S, f, 0).p, transformAt(shots[0], f, 0).p, 1e-12);
  for (let f = 0; f < S.n; f++) {
    const w = walkFrame(f);
    nearVec(transformAt(S, f, 0).p, w.p[0], 1e-9, `frame ${f}`);
    assert.ok(q4.angle(transformAt(S, f, 0).q, w.q[0]) < 1e-9);
  }
});

test('Float32 inputs stitch and keep the first-frame match to float precision', () => {
  const a = shotFrom(0, 20, 0, [0, 0, 0], true), b = shotFrom(20, 20, 2.5, [30, 0, 4], true);
  const S = stitchShots([a, b]);
  assert.ok(S.p instanceof Float64Array);
  nearVec(transformAt(S, 20, 0).p.map((x, k) => k === 1 ? 0 : x), transformAt(S, 19, 0).p.map((x, k) => k === 1 ? 0 : x), 1e-5);
  near(wrap(heading(transformAt(S, 20, 0).q) - heading(transformAt(S, 19, 0).q)), 0, 1e-5);
});
