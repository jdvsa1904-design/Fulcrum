export const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
export const lerp = (a, b, t) => a + (b - a) * t;
export const smoothstep = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const len = a => Math.hypot(a[0], a[1], a[2]);
export const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
export const norm = a => { const l = len(a); return l > 1e-12 ? [a[0] / l, a[1] / l, a[2] / l] : [0, 0, 0]; };
export const mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
export const vlerp = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
export const perp = (v, n) => sub(v, scale(n, dot(v, n)));
export const angle = (a, b) => Math.atan2(len(cross(a, b)), dot(a, b));

export const QI = Object.freeze([0, 0, 0, 1]);
export const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
export const qconj = q => [-q[0], -q[1], -q[2], q[3]];
export const qdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
export const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
export const qneg = q => [-q[0], -q[1], -q[2], -q[3]];
export const qalign = (q, ref) => qdot(q, ref) < 0 ? qneg(q) : q;
export const qaxis = (k, a) => { const n = norm(k), s = Math.sin(a / 2); return [n[0] * s, n[1] * s, n[2] * s, Math.cos(a / 2)]; };
export const qyaw = a => [0, Math.sin(a / 2), 0, Math.cos(a / 2)];
export function qrot(q, v) {
  const [x, y, z, w] = q, ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w - iw * x - iy * z + iz * y, iy * w - iw * y - iz * x + ix * z, iz * w - iw * z - ix * y + iy * x];
}
export function qarc(u, v) {
  const a = norm(u), b = norm(v), d = dot(a, b);
  if (d < -1 + 1e-10) {
    const k = norm(cross(a, Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0]));
    return [k[0], k[1], k[2], 0];
  }
  const c = cross(a, b);
  return qnorm([c[0], c[1], c[2], 1 + d]);
}
export function qangle(a, b) {
  const d = qmul(qconj(qnorm(a)), qnorm(b));
  return 2 * Math.atan2(Math.hypot(d[0], d[1], d[2]), Math.abs(d[3]));
}
export function qslerp(a, b, t) {
  let c = qdot(a, b), bb = b;
  if (c < 0) { c = -c; bb = qneg(b); }
  if (c > 0.9995) return qnorm([a[0] + (bb[0] - a[0]) * t, a[1] + (bb[1] - a[1]) * t, a[2] + (bb[2] - a[2]) * t, a[3] + (bb[3] - a[3]) * t]);
  const th = Math.acos(c), s = Math.sin(th), wa = Math.sin((1 - t) * th) / s, wb = Math.sin(t * th) / s;
  return [a[0] * wa + bb[0] * wb, a[1] * wa + bb[1] * wb, a[2] * wa + bb[2] * wb, a[3] * wa + bb[3] * wb];
}
export function qlog(q) {
  const s = Math.hypot(q[0], q[1], q[2]);
  if (s < 1e-12) return [q[0], q[1], q[2]];
  const k = Math.atan2(s, q[3]) / s;
  return [q[0] * k, q[1] * k, q[2] * k];
}
export function qexp(v) {
  const t = Math.hypot(v[0], v[1], v[2]);
  if (t < 1e-12) return qnorm([v[0], v[1], v[2], 1]);
  const k = Math.sin(t) / t;
  return [v[0] * k, v[1] * k, v[2] * k, Math.cos(t)];
}
export const qpow = (q, t) => qexp(scale(qlog(qalign(q, QI)), t));
export function qfromMat(m) {
  const t = m[0][0] + m[1][1] + m[2][2];
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4]; }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) { const s = Math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2; q = [s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]; }
  else if (m[1][1] > m[2][2]) { const s = Math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2; q = [(m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]; }
  else { const s = Math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2; q = [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s]; }
  return qnorm(q);
}
export function basisQuat(primary, secondary) {
  const e1 = norm(primary);
  let e2 = norm(perp(secondary, e1));
  if (len(e2) < 0.5) e2 = norm(perp(Math.abs(e1[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0], e1));
  const e3 = cross(e1, e2);
  return qfromMat([[e1[0], e2[0], e3[0]], [e1[1], e2[1], e3[1]], [e1[2], e2[2], e3[2]]]);
}
export const frameRotation = (tp, ts, rp, rs) => qmul(basisQuat(tp, ts), qconj(basisQuat(rp, rs)));
export function swingTwist(q, axis) {
  const a = norm(axis), p = dot([q[0], q[1], q[2]], a);
  let tw = [a[0] * p, a[1] * p, a[2] * p, q[3]];
  const l = Math.hypot(...tw);
  tw = l < 1e-12 ? [0, 0, 0, 1] : tw.map(x => x / l);
  return { swing: qmul(q, qconj(tw)), twist: tw };
}
export function twistAngle(tw, axis) {
  return 2 * Math.atan2(dot([tw[0], tw[1], tw[2]], norm(axis)), tw[3]);
}
export function clampRotation(q, maxAngle) {
  const a = qalign(q, QI), ang = 2 * Math.acos(Math.min(1, a[3]));
  return ang <= maxAngle ? a : qslerp(QI, a, maxAngle / ang);
}
export const yawOf = q => { const f = qrot(q, [0, 0, 1]); return Math.atan2(f[0], f[2]); };
export const wrapAngle = a => Math.atan2(Math.sin(a), Math.cos(a));

export function median(xs) {
  const v = Array.from(xs).filter(Number.isFinite).sort((a, b) => a - b);
  if (!v.length) return NaN;
  const m = v.length >> 1;
  return v.length % 2 ? v[m] : (v[m - 1] + v[m]) / 2;
}
export function percentile(xs, p) {
  const v = Array.from(xs).filter(Number.isFinite).sort((a, b) => a - b);
  if (!v.length) return NaN;
  const x = clamp(p, 0, 1) * (v.length - 1), i = Math.floor(x), u = x - i;
  return i + 1 < v.length ? v[i] + (v[i + 1] - v[i]) * u : v[i];
}
export const mean = xs => { let s = 0, c = 0; for (const x of xs) if (Number.isFinite(x)) { s += x; c++; } return c ? s / c : NaN; };

export function solve3(A, b) {
  const [a, c, d] = A, det = a[0] * (c[1] * d[2] - c[2] * d[1]) - a[1] * (c[0] * d[2] - c[2] * d[0]) + a[2] * (c[0] * d[1] - c[1] * d[0]);
  if (Math.abs(det) < 1e-14) return null;
  const col = (i, v) => A.map((r, k) => r.map((x, j) => j === i ? v[k] : x));
  const D = M => M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  return [D(col(0, b)) / det, D(col(1, b)) / det, D(col(2, b)) / det];
}

export function eigSym3(M) {
  const A = M.map(r => r.slice()), V = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let sweep = 0; sweep < 50; sweep++) {
    const off = A[0][1] ** 2 + A[0][2] ** 2 + A[1][2] ** 2;
    if (off < 1e-22) break;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      if (Math.abs(A[p][q]) < 1e-30) continue;
      const th = (A[q][q] - A[p][p]) / (2 * A[p][q]), t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1)), c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < 3; k++) { const akp = A[k][p], akq = A[k][q]; A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq; }
      for (let k = 0; k < 3; k++) { const apk = A[p][k], aqk = A[q][k]; A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk; }
      for (let k = 0; k < 3; k++) { const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq; }
    }
  }
  return [0, 1, 2].map(i => ({ value: A[i][i], vector: [V[0][i], V[1][i], V[2][i]] })).sort((a, b) => a.value - b.value);
}
