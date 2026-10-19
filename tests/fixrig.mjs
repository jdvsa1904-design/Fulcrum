// Synthetic mannequin, take authoring and a small pure rebuild for the FIX engine tests. The pipeline
// mirrors app.js rebuild order (cleanup -> knee -> FK -> chained root offsets -> contact IK) using the
// real wire/ and root solvers where they exist.
import { applyCleanup } from '../wire/cleanup.js';
import { kneeWobbleFix, kneeExtensionFix, ankleLock } from '../contacts.js';
import { twoBoneIK, defaultPole } from '../ik.js';
import { seamPlan, seamRootOffsets } from '../wire/seams.js';
import { ballisticOffsets } from '../ballistic.js';
import { fulcrumOffsets } from '../fulcrum.js';
import { chainRootOffsets } from '../wire/fixplan.js';
import { rigFromNames, comSegments, comSeries, computeFacts } from '../wire/facts.js';
import { judgeMotion } from '../wire/judge.js';

export const UPM = 100, FPS = 30;
const DEF = [
  ['pelvis', -1, [0, 0, 0]], ['spine_01', 0, [0, 10, 0]], ['spine_03', 1, [0, 15, 0]], ['spine_05', 2, [0, 15, 0]], ['neck_01', 3, [0, 10, 0]], ['head', 4, [0, 10, 0]],
  ['clavicle_l', 3, [5, 5, 0]], ['upperarm_l', 6, [12, 0, 0]], ['lowerarm_l', 7, [0, -28, 0]], ['hand_l', 8, [0, -25, 0]],
  ['clavicle_r', 3, [-5, 5, 0]], ['upperarm_r', 10, [-12, 0, 0]], ['lowerarm_r', 11, [0, -28, 0]], ['hand_r', 12, [0, -25, 0]],
  ['thigh_l', 0, [10, -5, 0]], ['calf_l', 14, [0, -45, 0]], ['foot_l', 15, [0, -45, 0]], ['ball_l', 16, [0, -7, 13]],
  ['thigh_r', 0, [-10, -5, 0]], ['calf_r', 18, [0, -45, 0]], ['foot_r', 19, [0, -45, 0]], ['ball_r', 20, [0, -7, 13]],
];
export const NAMES = DEF.map(d => d[0]), PARENTS = DEF.map(d => d[1]), OFFSETS = DEF.map(d => d[2]), NB = DEF.length;
export const RIG = rigFromNames(NAMES);
export const REST_PELVIS = [0, 103, 0];

export const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
export const qinv = q => [-q[0], -q[1], -q[2], q[3]];
export const qaxis = (k, ang) => { const l = Math.hypot(...k), s = Math.sin(ang / 2) / l; return [k[0] * s, k[1] * s, k[2] * s, Math.cos(ang / 2)]; };
const norm = v => { const l = Math.hypot(...v) || 1; return v.map(x => x / l); };
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const smoothstep = t => { const x = Math.max(0, Math.min(1, t)); return x * x * (3 - 2 * x); };

function quatFromCols(c0, c1, c2) {
  const m00 = c0[0], m10 = c0[1], m20 = c0[2], m01 = c1[0], m11 = c1[1], m21 = c1[2], m02 = c2[0], m12 = c2[1], m22 = c2[2], t = m00 + m11 + m22;
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4]; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; q = [s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s]; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; q = [(m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s]; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; q = [(m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s]; }
  return norm(q);
}
const worldQuat = (W, o) => { const c = k => norm([W[o + k * 4], W[o + k * 4 + 1], W[o + k * 4 + 2]]); return quatFromCols(c(0), c(1), c(2)); };
// Bone pointing down its -Y, with +Z toward the pole hint.
const basisQuat = (down, hint) => { const Y = norm(down.map(x => -x)), d = dot(hint, Y), Z = norm(hint.map((h, i) => h - d * Y[i])), X = cross(Y, Z); return quatFromCols(X, Y, Z); };

function compose(q, p, out, o) {
  const [x, y, z, w] = q, x2 = x + x, y2 = y + y, z2 = z + z, xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  out[o] = 1 - (yy + zz); out[o + 1] = xy + wz; out[o + 2] = xz - wy; out[o + 3] = 0;
  out[o + 4] = xy - wz; out[o + 5] = 1 - (xx + zz); out[o + 6] = yz + wx; out[o + 7] = 0;
  out[o + 8] = xz + wy; out[o + 9] = yz - wx; out[o + 10] = 1 - (xx + yy); out[o + 11] = 0;
  out[o + 12] = p[0]; out[o + 13] = p[1]; out[o + 14] = p[2]; out[o + 15] = 1;
}
function mul4(A, ao, B, bo, out, o) {
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += A[ao + k * 4 + r] * B[bo + c * 4 + k];
    out[o + c * 4 + r] = s;
  }
}
const L = new Float64Array(16);
export function fkFrame(q, p, f, W) {
  for (let i = 0; i < NB; i++) {
    compose(q.subarray((f * NB + i) * 4, (f * NB + i) * 4 + 4), p.subarray((f * NB + i) * 3, (f * NB + i) * 3 + 3), L, 0);
    const o = (f * NB + i) * 16;
    if (PARENTS[i] < 0) W.set(L, o); else mul4(W, (f * NB + PARENTS[i]) * 16, L, 0, W, o);
  }
}
export function fk(q, p, n) { const W = new Float32Array(n * NB * 16); for (let f = 0; f < n; f++) fkFrame(q, p, f, W); return W; }
export function restW() {
  const q = new Float64Array(NB * 4), p = new Float64Array(NB * 3);
  for (let i = 0; i < NB; i++) { q[i * 4 + 3] = 1; p.set(i ? OFFSETS[i] : REST_PELVIS, i * 3); }
  return fk(q, p, 1);
}

const LEG = s => ({ thigh: RIG['thigh_' + s], calf: RIG['calf_' + s], foot: RIG['foot_' + s], ball: RIG['ball_' + s] });

// Author a take in world terms: pelvis path, ankle paths (feet flat, facing +Z), small upper-body sway.
export function makeTake({ n = 90, fps = FPS, pelvis = () => [0, 100, 0], ankle = { l: () => [10, 8, 0], r: () => [-10, 8, 0] }, sway = 2, shots = null }) {
  const q = new Float64Array(n * NB * 4), p = new Float64Array(n * NB * 3);
  for (let f = 0; f < n; f++) {
    for (let i = 0; i < NB; i++) { q[(f * NB + i) * 4 + 3] = 1; p.set(i ? OFFSETS[i] : pelvis(f), (f * NB + i) * 3); }
    q.set(qaxis([0, 0, 1], sway * Math.PI / 180 * Math.sin(2 * Math.PI * f / 50)), (f * NB + RIG.spine_01) * 4);
    for (const [k, ph] of [['upperarm_l', 0], ['upperarm_r', Math.PI]]) q.set(qaxis([1, 0, 0], 4 * Math.PI / 180 * Math.sin(2 * Math.PI * f / 40 + ph)), (f * NB + RIG[k]) * 4);
    const P = pelvis(f);
    for (const s of ['l', 'r']) {
      const g = LEG(s), H = [P[0] + OFFSETS[g.thigh][0], P[1] + OFFSETS[g.thigh][1], P[2]], A = ankle[s](f);
      const sol = twoBoneIK(H, [H[0], H[1] - 45, H[2] + 1e-3], [H[0], H[1] - 90, H[2]], A, { pole: [0, 0, 1] });
      const qt = basisQuat([sol.mid[0] - H[0], sol.mid[1] - H[1], sol.mid[2] - H[2]], [0, 0, 1]);
      const qc = basisQuat([sol.end[0] - sol.mid[0], sol.end[1] - sol.mid[1], sol.end[2] - sol.mid[2]], [0, 0, 1]);
      q.set(qt, (f * NB + g.thigh) * 4); q.set(qmul(qinv(qt), qc), (f * NB + g.calf) * 4); q.set(qinv(qc), (f * NB + g.foot) * 4);
    }
  }
  return { q, p, n, fps, nb: NB, W: fk(q, p, n), shots: shots ?? [{ name: 'take', start: 0, n }] };
}
export const refreshW = t => ({ ...t, W: fk(t.q, t.p, t.n) });
export function rotateLocal(t, bone, f, axis, deg) {
  const o = (f * NB + RIG[bone]) * 4, cur = Array.from(t.q.subarray(o, o + 4));
  t.q.set(qmul(cur, qaxis(axis, deg * Math.PI / 180)), o);
  return refreshW(t);
}

const ramp = (f, a, b, e, n) => Math.min(1, a <= 0 ? 1 : (f - a + 1) / e, b >= n - 1 ? 1 : (b - f + 1) / e);
const wp = (W, f, i) => { const o = (f * NB + i) * 16 + 12; return [W[o], W[o + 1], W[o + 2]]; };

function legIK(q, p, W, f, side, target, w) {
  if (w <= 0) return;
  const g = LEG(side), H = wp(W, f, g.thigh), K = wp(W, f, g.calf), A = wp(W, f, g.foot);
  const T = A.map((v, k) => v + (target[k] - v) * w);
  const sol = twoBoneIK(H, K, A, T, { pole: defaultPole(H, K, A), guard: 'strict' });
  const qp = worldQuat(W, (f * NB + RIG.pelvis) * 16), qt = worldQuat(W, (f * NB + g.thigh) * 16), qc = worldQuat(W, (f * NB + g.calf) * 16), qf = worldQuat(W, (f * NB + g.foot) * 16);
  const qt2 = qmul(sol.rotRoot, qt), qc2 = qmul(sol.rotMid, qmul(sol.rotRoot, qc));
  q.set(norm(qmul(qinv(qp), qt2)), (f * NB + g.thigh) * 4); q.set(norm(qmul(qinv(qt2), qc2)), (f * NB + g.calf) * 4); q.set(norm(qmul(qinv(qc2), qf)), (f * NB + g.foot) * 4);
  fkFrame(q, p, f, W);
}

function levels(W, n, ground) {
  const out = {};
  for (const s of ['l', 'r']) {
    const g = LEG(s), fy = [], by = [];
    for (let f = 1; f < n - 1; f++) {
      const b0 = wp(W, f - 1, g.ball), b1 = wp(W, f + 1, g.ball), h = Math.min(wp(W, f, g.foot)[1], wp(W, f, g.ball)[1]) - ground;
      if (h < 10 && Math.hypot(b1[0] - b0[0], b1[2] - b0[2]) * FPS / 2 / UPM < 0.3) { fy.push(wp(W, f, g.foot)[1] - ground); by.push(wp(W, f, g.ball)[1] - ground); }
    }
    const med = a => { const x = [...a].sort((u, v) => u - v); return x.length ? x[x.length >> 1] : 0; };
    out[s] = { foot: med(fy), ball: med(by) };
  }
  return out;
}

// The pure rebuild: state = { fixes, locks, pins } in app.js formats.
export function pipeline(take, state, ground) {
  const { n, fps } = take, fixes = (state.fixes || []).filter(x => !x.mute);
  const cl = applyCleanup({ q: take.q, p: take.p, nb: NB, n, fps, fixes, names: NAMES });
  const q = cl.q, p0 = cl.p;
  for (const fx of fixes.filter(x => x.type === 'knee')) {
    const ci = RIG['calf_' + fx.side], th = new Float64Array(n);
    for (let f = 0; f < n; f++) { const o = (f * NB + ci) * 4; th[f] = 2 * Math.atan2(q[o], q[o + 3]); }
    let nt;
    if (fx.mode === 'extension') nt = Float64Array.from(kneeExtensionFix(th.map(v => -v * 180 / Math.PI), null, fx.maxDeg ?? 0).knee, v => -v * Math.PI / 180);
    else nt = kneeWobbleFix(th, [{ s: fx.a, e: fx.b }], { strength: fx.strength ?? 1 });
    for (let f = 0; f < n; f++) { if (Math.abs(nt[f] - th[f]) < 1e-12) continue; const o = (f * NB + ci) * 4; q.set(norm(qmul(Array.from(q.subarray(o, o + 4)), qaxis([1, 0, 0], nt[f] - th[f]))), o); }
  }
  const pel = RIG.pelvis, g = 9.81 * UPM;
  const withRoot = acc => {
    const p = Float64Array.from(p0), qq = Float64Array.from(q);
    for (let f = 0; f < n; f++) {
      const o = acc[f];
      if (!o) continue;
      p[(f * NB + pel) * 3] += o[0]; p[(f * NB + pel) * 3 + 1] += o[1]; p[(f * NB + pel) * 3 + 2] += o[2];
      if (o[3]) { const k = (f * NB + pel) * 4; qq.set(qmul(qaxis([0, 1, 0], o[3]), Array.from(qq.subarray(k, k + 4))), k); }
    }
    return { p, q: qq };
  };
  const pass = acc => { const r = withRoot(acc); return fk(r.q, r.p, n); };
  const producers = [];
  const seam = fixes.find(x => x.type === 'seams');
  if (seam && take.shots.length > 1) producers.push({ name: 'seams', fn: W => { const plan = seamPlan({ W, nb: NB, shots: take.shots, pelvis: pel, thighL: RIG.thigh_l, thighR: RIG.thigh_r, blend: seam.blend ?? 6, force: !!seam.force }); return plan.some(e => e.needed) ? seamRootOffsets(plan, n, { W, nb: NB, pelvis: pel, mode: seam.mode || 'carry' }) : null; } });
  const flights = fixes.filter(x => x.type === 'ballistic').map(x => ({ a: x.a, b: x.b }));
  if (flights.length) producers.push({ name: 'ballistic', fn: W => ballisticOffsets(comSeries(W, NB, n, comSegments(RIG)), flights, fps, g, 'endpoints') });
  for (const x of fixes.filter(y => y.type === 'fulcrum')) producers.push({ name: 'fulcrum', fn: W => fulcrumOffsets(W, NB, x.idx, x.a, x.b, 'auto', 3) });
  const chain = chainRootOffsets({ n, producers, pass });
  const root = withRoot(chain.offsets), qo = root.q, po = root.p, W = fk(qo, po, n);
  const lv = levels(W, n, ground), targets = Array.from({ length: n }, () => []);
  for (const lk of state.locks || []) {
    const fi = LEG(lk.side).foot;
    let best = lk.a, bs = Infinity;
    for (let f = lk.a; f <= lk.b; f++) { const a = wp(W, Math.max(lk.a, f - 1), fi), b = wp(W, Math.min(lk.b, f + 1), fi), s = Math.hypot(b[0] - a[0], b[2] - a[2]); if (s < bs) { bs = s; best = f; } }
    const T = wp(W, best, fi);
    for (let f = lk.a; f <= lk.b; f++) targets[f].push({ side: lk.side, T, w: ramp(f, lk.a, lk.b, 4, n) });
  }
  for (const fx of fixes.filter(x => x.type === 'ankleLock')) {
    const fi = LEG(fx.side).foot, m = fx.b - fx.a + 1, pos = new Float64Array(m * 3);
    for (let f = fx.a; f <= fx.b; f++) pos.set(wp(W, f, fi), (f - fx.a) * 3);
    const r = ankleLock(pos, { perimeter: (fx.perimeter ?? 3) * UPM / 100, floorProximity: (fx.floorProximity ?? 10) * UPM / 100, maxSlide: (fx.maxSlide ?? 0) * UPM, fps, floor: ground + lv[fx.side].foot, time: 0.1 });
    for (let k = 0; k < m; k++) if (r.locked[k]) { const e = Math.min(1, (k + 1) / 2, (m - k) / 2); targets[fx.a + k].push({ side: fx.side, T: r.anchors[k], w: e }); }
  }
  for (const fx of fixes.filter(x => x.type === 'ground')) {
    const k = LEG(fx.side ?? 'l');
    let anchor = fx.a, slow = Infinity;
    for (let f = fx.a; f <= fx.b; f++) { const a0 = wp(W, Math.max(fx.a, f - 1), k.ball), a1 = wp(W, Math.min(fx.b, f + 1), k.ball), v = Math.hypot(a1[0] - a0[0], a1[2] - a0[2]); if (v < slow) { slow = v; anchor = f; } }
    const hold = wp(W, anchor, k.ball);
    for (let f = fx.a; f <= fx.b; f++) {
      const ball = wp(W, f, k.ball), foot = wp(W, f, k.foot), d = Math.min(ball[1] - (ground + lv[fx.side ?? 'l'].ball), foot[1] - (ground + lv[fx.side ?? 'l'].foot));
      targets[f].push({ side: fx.side ?? 'l', T: [foot[0] + hold[0] - ball[0], foot[1] - Math.max(0, d), foot[2] + hold[2] - ball[2]], w: smoothstep(Math.max(0, ramp(f, fx.a, fx.b, 6, n))) });
    }
  }
  for (let f = 0; f < n; f++) for (const t of targets[f]) legIK(qo, po, W, f, t.side, t.T, t.w);
  for (const fx of fixes.filter(x => x.type === 'floor')) for (const s of fx.side ? [fx.side] : ['l', 'r']) {
    const k = LEG(s);
    for (let f = Math.max(0, fx.a); f <= Math.min(n - 1, fx.b); f++) {
      const ball = wp(W, f, k.ball), foot = wp(W, f, k.foot), lift = Math.max(0, ground + lv[s].ball - ball[1], ground + lv[s].foot - foot[1]);
      if (lift > 1e-6) legIK(qo, po, W, f, s, [foot[0], foot[1] + lift, foot[2]], 1);
    }
  }
  return { W, q: Float32Array.from(qo), p: Float32Array.from(po) };
}

export function harness(take, { trust = null } = {}) {
  const rw = restW(), src = { q: Float32Array.from(take.q), p: Float32Array.from(take.p) };
  const ground = computeFacts({ W: take.W, nb: NB, n: take.n, fps: take.fps, upm: UPM, rig: RIG, restW: rw, shots: take.shots, trust }).ground;
  const apply = state => pipeline(take, state, ground);
  const evaluate = (state, motion) => judgeMotion({ motion, source: src, nb: NB, n: take.n, fps: take.fps, upm: UPM, rig: RIG, names: NAMES, parents: PARENTS, restW: rw, shots: take.shots, trust, state, ground });
  return { apply, evaluate, ground, restW: rw, judge: state => evaluate(state, apply(state)) };
}
