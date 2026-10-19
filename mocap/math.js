export const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
export const lerp = (a, b, t) => a + (b - a) * t;
export const smoothstep = t => { const u = clamp(t, 0, 1); return u * u * (3 - 2 * u); };

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

export const QI = () => [0, 0, 0, 1];
export const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
export const qconj = q => [-q[0], -q[1], -q[2], q[3]];
export const qdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
export const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
export const qaxis = (axis, ang) => { const n = norm(axis), s = Math.sin(ang / 2); return [n[0] * s, n[1] * s, n[2] * s, Math.cos(ang / 2)]; };
export const qangle = (a, b) => 2 * Math.acos(clamp(Math.abs(qdot(a, b)), 0, 1));

export function qrot(q, v) {
  const [x, y, z, w] = q, ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w - iw * x - iy * z + iz * y, iy * w - iw * y - iz * x + ix * z, iz * w - iw * z - ix * y + iy * x];
}

export function qslerp(a, b, t) {
  let d = qdot(a, b), bb = b;
  if (d < 0) { d = -d; bb = [-b[0], -b[1], -b[2], -b[3]]; }
  if (d > 0.9995) return qnorm([a[0] + (bb[0] - a[0]) * t, a[1] + (bb[1] - a[1]) * t, a[2] + (bb[2] - a[2]) * t, a[3] + (bb[3] - a[3]) * t]);
  const th = Math.acos(d), s = Math.sin(th), wa = Math.sin((1 - t) * th) / s, wb = Math.sin(t * th) / s;
  return [a[0] * wa + bb[0] * wb, a[1] * wa + bb[1] * wb, a[2] * wa + bb[2] * wb, a[3] * wa + bb[3] * wb];
}

export function qarc(u, v) {
  const a = norm(u), b = norm(v), d = dot(a, b);
  if (d < -1 + 1e-9) { const k = norm(cross(a, Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0])); return [k[0], k[1], k[2], 0]; }
  const c = cross(a, b);
  return qnorm([c[0], c[1], c[2], 1 + d]);
}

export function qlog(q) {
  const w = clamp(q[3], -1, 1), s = Math.hypot(q[0], q[1], q[2]);
  if (s < 1e-12) return [0, 0, 0];
  const k = Math.atan2(s, w) / s;
  return [q[0] * k, q[1] * k, q[2] * k];
}

export function qexp(v) {
  const t = Math.hypot(v[0], v[1], v[2]);
  if (t < 1e-12) return [0, 0, 0, 1];
  const k = Math.sin(t) / t;
  return [v[0] * k, v[1] * k, v[2] * k, Math.cos(t)];
}

export function quatFromMat(m) {
  const t = m[0] + m[4] + m[8];
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s, s / 4]; }
  else if (m[0] > m[4] && m[0] > m[8]) { const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2; q = [s / 4, (m[1] + m[3]) / s, (m[2] + m[6]) / s, (m[7] - m[5]) / s]; }
  else if (m[4] > m[8]) { const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2; q = [(m[1] + m[3]) / s, s / 4, (m[5] + m[7]) / s, (m[2] - m[6]) / s]; }
  else { const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2; q = [(m[2] + m[6]) / s, (m[5] + m[7]) / s, s / 4, (m[3] - m[1]) / s]; }
  return qnorm(q);
}

export function basisQuat(primary, secondary) {
  const a = norm(primary);
  let b = perp(secondary, a);
  if (len(b) < 1e-9) b = perp(Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0], a);
  b = norm(b);
  const c = cross(a, b);
  return quatFromMat([a[0], b[0], c[0], a[1], b[1], c[1], a[2], b[2], c[2]]);
}

export const frameRotation = (target, rest) => qmul(target, qconj(rest));

export const qyaw = q => { const f = qrot(q, [0, 0, 1]); return Math.atan2(f[0], f[2]); };

export function qToEulerZXY(q) {
  const [x, y, z, w] = q;
  const m11 = 1 - 2 * (y * y + z * z), m12 = 2 * (x * y - z * w);
  const m21 = 2 * (x * y + z * w), m22 = 1 - 2 * (x * x + z * z);
  const m31 = 2 * (x * z - y * w), m32 = 2 * (y * z + x * w), m33 = 1 - 2 * (x * x + y * y);
  const ex = Math.asin(clamp(m32, -1, 1));
  let ey, ez;
  if (Math.abs(m32) < 0.9999999) { ey = Math.atan2(-m31, m33); ez = Math.atan2(-m12, m22); }
  else { ey = 0; ez = Math.atan2(m21, m11); }
  return [ex, ey, ez];
}

export const qFromEulerZXY = ([ex, ey, ez]) => qmul(qmul(qaxis([0, 0, 1], ez), qaxis([1, 0, 0], ex)), qaxis([0, 1, 0], ey));

export function median(xs) {
  const a = Array.from(xs).filter(Number.isFinite).sort((p, q) => p - q);
  if (!a.length) return NaN;
  const m = a.length >> 1;
  return a.length % 2 ? a[m] : (a[m - 1] + a[m]) / 2;
}

export function percentile(xs, p) {
  const a = Array.from(xs).filter(Number.isFinite).sort((u, v) => u - v);
  if (!a.length) return NaN;
  const k = clamp((a.length - 1) * p, 0, a.length - 1), i = Math.floor(k), f = k - i;
  return i + 1 < a.length ? a[i] * (1 - f) + a[i + 1] * f : a[i];
}
