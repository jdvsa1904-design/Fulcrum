import test from 'node:test';
import assert from 'node:assert/strict';
import { json, exists, q4, v3 } from './helpers.mjs';
import { seamPlan, seamRootOffsets } from '../wire/seams.js';
import { yawArc } from '../stitch.js';

const NB = 4, PEL = 1, TL = 2, TR = 3, UPM = 91.33, FPS = 24;
const SPEED = 1.4 * UPM / FPS, TURN = 0.015, HIP = 0.1 * UPM;
const Y = [0, 1, 0], Z = [0, 0, 1], YAW_TOL = 2.5 * Math.PI / 180;

const truth = (() => {
  const out = [{ p: [0, 0], h: 0 }];
  for (let t = 1; t < 400; t++) {
    const h = TURN * t, o = out[t - 1];
    out.push({ p: [o.p[0] + SPEED * Math.cos(h), o.p[1] - SPEED * Math.sin(h)], h });
  }
  return out;
})();

function mat(q, p) {
  const [x, y, z, w] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0,
    2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0,
    2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0,
    p[0], p[1], p[2], 1];
}

function pose(t, yaw = 0, move = [0, 0, 0]) {
  const s = truth[t], h = s.h + 0.05 * Math.sin(0.4 * t);
  const pel = [s.p[0], 0.95 * UPM + 2 * Math.sin(0.8 * t), s.p[1]];
  const lat = [-Math.sin(h), 0, -Math.cos(h)];
  const qp = q4.mul(q4.axis(Y, h), q4.axis(Z, Math.PI / 2));
  const pts = [[0, 0, 0], pel, v3.add(pel, v3.mul(lat, -HIP)), v3.add(pel, v3.mul(lat, HIP))];
  const qy = q4.axis(Y, yaw);
  return pts.map((p, i) => ({ p: i === 0 ? p : v3.add(q4.rot(qy, p), move), q: i === 0 ? [0, 0, 0, 1] : q4.mul(qy, qp) }));
}

function build(segs) {
  const n = segs.reduce((a, s) => a + s.len, 0), W = new Float64Array(n * NB * 16), shots = [];
  let f = 0;
  for (const s of segs) {
    shots.push({ name: s.name, start: f, n: s.len });
    for (let k = 0; k < s.len; k++, f++) pose(s.t0 + k, s.yaw, s.move).forEach((b, i) => W.set(mat(b.q, b.p), (f * NB + i) * 16));
  }
  return { W, n, shots };
}

const P = (W, f, i) => { const o = (f * NB + i) * 16 + 12; return [W[o], W[o + 1], W[o + 2]]; };

function apply(W, n, off, up = Y) {
  const out = Float64Array.from(W);
  for (let f = 0; f < n; f++) {
    const o = off[f];
    if (!o) continue;
    const p = P(W, f, PEL), qy = q4.axis(up, o[3]);
    for (let i = 1; i < NB; i++) {
      const r = v3.add(v3.add(q4.rot(qy, v3.sub(P(W, f, i), p)), p), [o[0], o[1], o[2]]);
      out.set(r, (f * NB + i) * 16 + 12);
    }
  }
  return out;
}

const hstep = (W, a, b) => { const x = P(W, a, PEL), y = P(W, b, PEL); return Math.hypot(x[0] - y[0], x[2] - y[2]); };
const head = (W, f) => { const h = v3.sub(P(W, f, TR), P(W, f, TL)); return Math.atan2(-h[2], h[0]); };
const wrap = a => a - 2 * Math.PI * Math.round(a / (2 * Math.PI));
const finite = a => a.every(o => o === null || o.every(Number.isFinite));
const ctx = { pelvis: PEL, thighL: TL, thighR: TR, nb: NB };

test('pelvis local X points up, so stitch.yawArc alone is blind to heading', () => {
  const a = pose(10)[PEL].q, b = pose(10, 0.8)[PEL].q;
  assert.deepEqual(yawArc(a, b).map(x => +x.toFixed(9)), [0, 0, 0, 1]);
});

test('two-shot walk with a deliberate offset and yaw is made continuous', () => {
  const { W, n, shots } = build([{ name: 'a', t0: 0, len: 60 }, { name: 'b', t0: 60, len: 60, yaw: 0.7, move: [120, 0, -80] }]);
  assert.ok(hstep(W, 59, 60) > 10 * SPEED);
  const plan = seamPlan({ W, shots, ...ctx });
  assert.equal(plan.length, 1);
  assert.ok(plan[0].jumpBefore > 10 * SPEED);
  const off = seamRootOffsets(plan, n, { W, ...ctx });
  assert.ok(finite(off));
  assert.equal(off.slice(0, 60).every(o => o === null), true);
  const C = apply(W, n, off);
  const seam = hstep(C, 59, 60);
  let maxStep = 0;
  for (let f = 1; f < n; f++) if (f !== 60) maxStep = Math.max(maxStep, hstep(C, f - 1, f));
  assert.ok(seam <= maxStep * 1.05, `seam ${seam} vs step ${maxStep}`);
  assert.ok(Math.abs(seam - SPEED) < 0.05 * SPEED);
  assert.ok(Math.abs(wrap(head(C, 60) - head(C, 59))) < 0.1);
  assert.ok(Math.abs(P(C, 60, PEL)[1] - P(W, 60, PEL)[1]) < 1e-9);
  assert.ok(Math.abs(plan[0].offset.yawRad + 0.7) < YAW_TOL, `yaw ${plan[0].offset.yawRad}`);
  assert.ok(plan[0].needed);
});

test('missing capture frames read from shot names are bridged by velocity', () => {
  const { W, n, shots } = build([
    { name: 'shot2_f102-161_ue5.fbx', t0: 0, len: 60 },
    { name: 'shot3_f167-226_ue5.fbx', t0: 65, len: 60, yaw: -0.4, move: [-50, 0, 30] },
  ]);
  const plan = seamPlan({ W, shots, ...ctx });
  assert.equal(plan[0].gapFrames, 5);
  const C = apply(W, n, seamRootOffsets(plan, n, { W, ...ctx }));
  assert.ok(Math.abs(hstep(C, 59, 60) - 6 * SPEED) < 0.05 * 6 * SPEED);
  assert.ok(Math.abs(plan[0].offset.yawRad - 0.4) < YAW_TOL, `yaw ${plan[0].offset.yawRad}`);
  const g = seamPlan({ W, shots: shots.map((s, i) => i ? { ...s, gapFrames: 0 } : s), ...ctx });
  assert.equal(g[0].gapFrames, 0);
});

test('identity when the take is already continuous', () => {
  const { W, n, shots } = build([{ name: 'a', t0: 0, len: 50 }, { name: 'b', t0: 50, len: 40 }, { name: 'c', t0: 90, len: 40 }]);
  const plan = seamPlan({ W, shots, ...ctx });
  assert.equal(plan.length, 2);
  for (const e of plan) {
    assert.equal(e.needed, false);
    assert.deepEqual([...e.offset.T, e.offset.yawRad], [0, 0, 0, 0]);
  }
  const forced = seamPlan({ W, shots, ...ctx, force: true });
  assert.ok(Math.hypot(...forced[0].offset.T) < 0.05 * SPEED, `T ${forced[0].offset.T}`);
  for (const e of forced) assert.ok(Math.abs(e.offset.yawRad) < YAW_TOL);
  const C = apply(W, n, seamRootOffsets(plan, n, { W, ...ctx }));
  let d = 0;
  for (let f = 0; f < n; f++) d = Math.max(d, v3.dist(P(C, f, PEL), P(W, f, PEL)));
  assert.equal(d, 0);
  assert.deepEqual(seamRootOffsets([], 10), new Array(10).fill(null));
  assert.deepEqual(seamPlan({ W, shots: shots.slice(0, 1), ...ctx }), []);
});

test('three shots: later corrections chain on the corrected previous shot', () => {
  const { W, n, shots } = build([{ name: 'a', t0: 0, len: 50 }, { name: 'b', t0: 50, len: 40, yaw: 0.5, move: [40, 0, 40] }, { name: 'c', t0: 90, len: 40, yaw: -0.9, move: [-60, 0, 10] }]);
  const plan = seamPlan({ W, shots, ...ctx });
  const C = apply(W, n, seamRootOffsets(plan, n, { W, ...ctx }));
  assert.ok(Math.abs(plan[0].offset.yawRad + 0.5) < YAW_TOL && Math.abs(plan[1].offset.yawRad - 0.9) < YAW_TOL, plan.map(e => e.offset.yawRad).join());
  for (const s of [50, 90]) assert.ok(Math.abs(hstep(C, s - 1, s) - SPEED) < 0.1 * SPEED, `seam ${s} ${hstep(C, s - 1, s)}`);
  for (let f = 0; f < 50; f++) assert.ok(v3.dist(P(C, f, PEL), P(W, f, PEL)) === 0);
});

test('local mode fades the correction out over blend frames', () => {
  const { W, n, shots } = build([{ name: 'a', t0: 0, len: 60 }, { name: 'b', t0: 60, len: 60, move: [30, 0, 0] }]);
  const plan = seamPlan({ W, shots, ...ctx, blend: 6 });
  const off = seamRootOffsets(plan, n, { W, ...ctx, mode: 'local' });
  assert.ok(Math.hypot(off[60][0], off[60][2]) > 25);
  assert.ok(off.slice(66).every(o => o === null));
  for (let f = 61; f < 66; f++) assert.ok(Math.hypot(off[f][0], off[f][2]) <= Math.hypot(off[f - 1][0], off[f - 1][2]) + 1e-9);
});

test('seamRootOffsets refuses yaw without world matrices', () => {
  assert.throws(() => seamRootOffsets([{ start: 0, n: 2, offset: { T: [0, 0, 0], yawRad: 0.1, pivot: [0, 0, 0] }, blendFrames: 6 }], 2));
  const t = seamRootOffsets([{ start: 1, n: 2, offset: { T: [1, 0, 2], yawRad: 0, pivot: [0, 0, 0] }, blendFrames: 6 }], 3);
  assert.deepEqual(t, [null, [1, 0, 2, 0], [1, 0, 2, 0]]);
});

const SRC = name => { const m = /f0*(\d+)-0*(\d+)/.exec(name); return +m[2] - +m[1] + 1; };
const deg = r => r * 180 / Math.PI;

test('FRT v6 frames: seam corrections measured', { skip: !exists('exports/FRT_fulcrum_v6.frames.json') }, () => {
  const fr = json('exports/FRT_fulcrum_v6.frames.json'), pj = json('exports/FRT_fulcrum_v6.fulcrum.json');
  const nb = fr.bones.length, n = fr.frames.length, W = new Float64Array(n * nb * 16);
  fr.frames.forEach((m, f) => W.set(m, f * nb * 16));
  let start = 0;
  const shots = pj.shots.map(name => { const s = { name, start, n: SRC(name) }; start += s.n; return s; });
  assert.equal(start, n);
  const b = k => fr.bones.indexOf(k);
  const plan = seamPlan({ W, nb, shots, pelvis: b('pelvis'), thighL: b('thigh_l'), thighR: b('thigh_r') });
  const off = seamRootOffsets(plan, n, { W, nb, pelvis: b('pelvis') });
  assert.ok(finite(off));
  assert.ok(plan.every(e => !e.needed), 'FRT shots are cut from one capture and already line up');
  assert.ok(off.every(o => o === null));
  const cm = UPM / 100;
  for (const e of plan) {
    const T = Math.hypot(e.offset.T[0], e.offset.T[2]) / cm;
    console.log(`v6 seam -> shot ${e.shot + 1} at f${e.start} (gap ${e.gapFrames}): move ${T.toFixed(2)} cm, yaw ${deg(e.offset.yawRad).toFixed(2)} deg, jump ${(e.jumpBefore / cm).toFixed(2)} cm, allowed ${(e.allowedJump / cm).toFixed(2)} cm, needed ${e.needed}, hip-heading jump ${deg(e.yawJumpRad).toFixed(2)} deg`);
    const f = seamPlan({ W, nb, shots, pelvis: b('pelvis'), thighL: b('thigh_l'), thighR: b('thigh_r'), force: true }).find(x => x.shot === e.shot);
    console.log(`  forced: move ${(Math.hypot(f.offset.T[0], f.offset.T[2]) / cm).toFixed(2)} cm, yaw ${deg(f.offset.yawRad).toFixed(2)} deg`);
    assert.ok(Number.isFinite(T) && Number.isFinite(e.offset.yawRad));
  }
  assert.equal(plan[1].gapFrames, 5);
});

test('taste_in (Z-up cm, 22 bones): seam corrections measured with up = Z', { skip: !exists('exports/taste_in.json') }, () => {
  const ti = json('exports/taste_in.json'), n = ti.B.length, nb = 22, W = new Float64Array(n * nb * 16);
  ti.B.forEach((fr, f) => fr.forEach((p, i) => W.set(p, (f * nb + i) * 16 + 12)));
  const pj = json('exports/FRT_fulcrum_v6.fulcrum.json');
  let start = 0;
  const shots = pj.shots.map(name => { const s = { name, start, n: SRC(name) }; start += s.n; return s; });
  const plan = seamPlan({ W, nb, shots, pelvis: 0, thighL: 14, thighR: 18, up: [0, 0, 1] });
  const off = seamRootOffsets(plan, n, { W, nb, pelvis: 0, up: [0, 0, 1] });
  assert.ok(finite(off));
  for (const e of plan) {
    console.log(`taste_in seam -> shot ${e.shot + 1} at f${e.start}: move ${Math.hypot(e.offset.T[0], e.offset.T[1]).toFixed(2)} cm, vertical ${e.offset.T[2].toFixed(3)} cm, yaw ${deg(e.offset.yawRad).toFixed(2)} deg, jump ${e.jumpBefore.toFixed(2)} cm, allowed ${e.allowedJump.toFixed(2)} cm, needed ${e.needed}, hip-heading jump ${deg(e.yawJumpRad).toFixed(2)} deg`);
    const f = seamPlan({ W, nb, shots, pelvis: 0, thighL: 14, thighR: 18, up: [0, 0, 1], force: true }).find(x => x.shot === e.shot);
    console.log(`  forced: move ${Math.hypot(f.offset.T[0], f.offset.T[1]).toFixed(2)} cm, vertical ${f.offset.T[2].toFixed(3)} cm, yaw ${deg(f.offset.yawRad).toFixed(2)} deg`);
    assert.ok(Math.abs(f.offset.T[2]) < 1e-6);
    assert.ok(Math.abs(e.offset.T[2]) < 1e-6);
  }
});
