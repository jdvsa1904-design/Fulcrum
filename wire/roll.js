import { rollSegments, rollFoot } from '../contacts.js';

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = a => Math.hypot(a[0], a[1], a[2]);
const norm = a => { const l = len(a); return l > 1e-12 ? scale(a, 1 / l) : null; };
const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
const qaxis = (k, deg) => { const h = deg * Math.PI / 360, s = Math.sin(h); return [k[0] * s, k[1] * s, k[2] * s, Math.cos(h)]; };

const FLAT_ELEVATION = Math.asin(7.49 / 16.78);
const TOE_RATIO = 0.45;

function matQuat(W, o) {
  const c = [0, 1, 2].map(j => { const v = [W[o + j * 4], W[o + j * 4 + 1], W[o + j * 4 + 2]]; return scale(v, 1 / (len(v) || 1)); });
  const m = (i, j) => c[j][i], t = m(0, 0) + m(1, 1) + m(2, 2);
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(m(2, 1) - m(1, 2)) / s, (m(0, 2) - m(2, 0)) / s, (m(1, 0) - m(0, 1)) / s, s / 4]; }
  else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) { const s = Math.sqrt(1 + m(0, 0) - m(1, 1) - m(2, 2)) * 2; q = [s / 4, (m(0, 1) + m(1, 0)) / s, (m(0, 2) + m(2, 0)) / s, (m(2, 1) - m(1, 2)) / s]; }
  else if (m(1, 1) > m(2, 2)) { const s = Math.sqrt(1 + m(1, 1) - m(0, 0) - m(2, 2)) * 2; q = [(m(0, 1) + m(1, 0)) / s, s / 4, (m(1, 2) + m(2, 1)) / s, (m(0, 2) - m(2, 0)) / s]; }
  else { const s = Math.sqrt(1 + m(2, 2) - m(0, 0) - m(1, 1)) * 2; q = [(m(0, 2) + m(2, 0)) / s, (m(1, 2) + m(2, 1)) / s, s / 4, (m(1, 0) - m(0, 1)) / s]; }
  return qnorm(q);
}
const matPos = (W, o) => [W[o + 12], W[o + 13], W[o + 14]];

const pitchToRoll = (p, s) => p <= 0 || s <= 0 || p >= s ? p : Math.sqrt(p * s);

export function rollCurveFromPlant({ heights, a, b, start = 30, end = 60, length, flat, dead = 0.05, maxRoll }) {
  const A = heights.ankle, B = heights.ball, n = Math.min(A.length, B.length), out = new Float64Array(n);
  const lo = clamp(Math.round(a), 0, n - 1), hi = clamp(Math.round(b), 0, n - 1);
  if (hi < lo) return out;
  let minA = Infinity, minB = Infinity;
  for (let f = lo; f <= hi; f++) { minA = Math.min(minA, A[f]); minB = Math.min(minB, B[f]); }
  let h0 = flat;
  if (h0 == null) {
    let best = Infinity;
    for (let f = lo; f <= hi; f++) { const e = (A[f] - minA) + (B[f] - minB); if (e < best) { best = e; h0 = A[f] - B[f]; } }
  }
  const L = length > 0 ? length : Math.max(Math.abs(h0) / Math.sin(FLAT_ELEVATION), 1e-9);
  const phi0 = Math.asin(clamp(h0 / L, -1, 1)), cap = maxRoll ?? Math.max(end + 30, 90);
  for (let f = lo; f <= hi; f++) {
    const pitch = (Math.asin(clamp((A[f] - B[f]) / L, -1, 1)) - phi0) * 180 / Math.PI;
    out[f] = Math.abs(pitch) < dead ? 0 : clamp(pitchToRoll(pitch, start), -cap, cap);
  }
  return out;
}

export function rollPose({ W, nb, f, side, legIdx, roll, start = 30, end = 60, toeLen, up = [0, 1, 0] }) {
  const of = (f * nb + legIdx.foot) * 16, ob = (f * nb + legIdx.ball) * 16;
  const footQ0 = matQuat(W, of), ballQ0 = matQuat(W, ob), ankle = matPos(W, of), ball = matPos(W, ob);
  const U = norm(up) ?? [0, 1, 0];
  const ab = sub(ball, ankle), flatAb = sub(ab, scale(U, dot(ab, U)));
  let fwd = norm(flatAb);
  if (!fwd && legIdx.calf != null) {
    const knee = matPos(W, (f * nb + legIdx.calf) * 16), shin = sub(ankle, knee);
    fwd = norm(cross(cross(U, shin), U));
  }
  if (!fwd) fwd = norm(cross(U, Math.abs(U[0]) < 0.9 ? [1, 0, 0] : [0, 0, 1]));
  const axis = norm(cross(U, fwd));
  const r = Number.isFinite(roll) ? roll : 0;
  const seg = rollSegments(r, start, end);
  const lift = dot(ab, U), heel = add(ankle, scale(U, lift));
  const tl = toeLen > 0 ? toeLen : TOE_RATIO * len(ab);
  const toe = add(ball, scale(fwd, tl));
  const pivots = { heel, ball, toe };
  const rolled = rollFoot({ ankle, ball, heel, toe }, r, pivots, { start, end, axis }).points;
  const footQ = qnorm(qmul(qaxis(axis, seg.heel + seg.toes + seg.toesEnd), footQ0));
  const ballQ = qnorm(qmul(qaxis(axis, seg.heel + seg.toesEnd), ballQ0));
  return {
    footQ, ballQ, footP: rolled.ankle, ballP: rolled.ball, heelP: rolled.heel, toeP: rolled.toe,
    pivots, axis, side, segments: seg,
  };
}
