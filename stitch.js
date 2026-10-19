import { quatContinuity, readTrack, writeTrack } from './filters.js';

const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
const qrot = (q, v) => {
  const [x, y, z, w] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
};
const clamp = (x, a, b) => x < a ? a : x > b ? b : x;

export const smoothstep = u => { const t = clamp(u, 0, 1); return t * t * (3 - 2 * t); };

export function yawArc(RA, RB) {
  const fa = qrot(RA, [1, 0, 0]), fb = qrot(RB, [1, 0, 0]);
  if (Math.hypot(fa[0], fa[2]) < 1e-9 || Math.hypot(fb[0], fb[2]) < 1e-9) return [0, 0, 0, 1];
  let th = Math.atan2(-fa[2], fa[0]) - Math.atan2(-fb[2], fb[0]);
  th -= 2 * Math.PI * Math.round(th / (2 * Math.PI));
  return [0, Math.sin(th / 2), 0, Math.cos(th / 2)];
}

export function gravityMatch(A, B) {
  const qY = yawArc(A.q, B.q);
  const T = [A.p[0], B.p[1], A.p[2]];
  const R = qnorm(qmul(qY, B.q));
  const rb = qrot(qY, B.p);
  const offset = { R: qnorm(qmul(R, [-B.q[0], -B.q[1], -B.q[2], B.q[3]])), T: [T[0] - rb[0], T[1] - rb[1], T[2] - rb[2]] };
  return { offset, matched: { p: T, q: R } };
}

export function transformAt(shot, f, bone) {
  const nb = shot.nb, po = (f * nb + bone) * 3, qo = (f * nb + bone) * 4;
  return { p: [shot.p[po], shot.p[po + 1], shot.p[po + 2]], q: [shot.q[qo], shot.q[qo + 1], shot.q[qo + 2], shot.q[qo + 3]] };
}

export function applyOffset(shot, offset, o = {}) {
  const nb = shot.nb, n = shot.n;
  const bones = o.all ? Array.from({ length: nb }, (_, i) => i) : (o.bones ?? [o.root ?? 0]);
  const p = Float64Array.from(shot.p), q = Float64Array.from(shot.q);
  const R = offset.R, T = offset.T;
  for (let f = 0; f < n; f++) {
    for (const i of bones) {
      const po = (f * nb + i) * 3, qo = (f * nb + i) * 4;
      const r = qrot(R, [p[po], p[po + 1], p[po + 2]]);
      p[po] = r[0] + T[0]; p[po + 1] = r[1] + T[1]; p[po + 2] = r[2] + T[2];
      const rq = qnorm(qmul(R, [q[qo], q[qo + 1], q[qo + 2], q[qo + 3]]));
      q[qo] = rq[0]; q[qo + 1] = rq[1]; q[qo + 2] = rq[2]; q[qo + 3] = rq[3];
    }
  }
  return { n, nb, p, q };
}

function slerp(a, b, w) {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  const s = d < 0 ? -1 : 1;
  d *= s;
  let ka, kb;
  if (d > 0.9995) { ka = 1 - w; kb = w; }
  else {
    const th = Math.acos(d), sn = Math.sin(th);
    ka = Math.sin((1 - w) * th) / sn; kb = Math.sin(w * th) / sn;
  }
  return qnorm([a[0] * ka + s * b[0] * kb, a[1] * ka + s * b[1] * kb, a[2] * ka + s * b[2] * kb, a[3] * ka + s * b[3] * kb]);
}

export function overlapWeight(k, overlap) {
  if (overlap <= 1) return 0.5;
  return smoothstep(k / (overlap - 1));
}

export function crossfade(shotA, shotB, overlap = 0) {
  const nb = shotA.nb, nA = shotA.n, nB = shotB.n;
  const ov = clamp(Math.floor(overlap), 0, Math.min(nA, nB));
  const n = nA + nB - ov, bA = nA - ov;
  const p = new Float64Array(n * nb * 3), q = new Float64Array(n * nb * 4);
  const per3 = nb * 3, per4 = nb * 4;
  for (let f = 0; f < bA; f++) {
    for (let k = 0; k < per3; k++) p[f * per3 + k] = shotA.p[f * per3 + k];
    for (let k = 0; k < per4; k++) q[f * per4 + k] = shotA.q[f * per4 + k];
  }
  for (let k = 0; k < ov; k++) {
    const w = overlapWeight(k, ov), fo = bA + k;
    for (let j = 0; j < per3; j++) p[fo * per3 + j] = shotA.p[fo * per3 + j] * (1 - w) + shotB.p[k * per3 + j] * w;
    for (let i = 0; i < nb; i++) {
      const a = (fo * nb + i) * 4, b = (k * nb + i) * 4;
      const r = slerp([shotA.q[a], shotA.q[a + 1], shotA.q[a + 2], shotA.q[a + 3]], [shotB.q[b], shotB.q[b + 1], shotB.q[b + 2], shotB.q[b + 3]], w);
      q.set(r, a);
    }
  }
  for (let f = ov; f < nB; f++) {
    const fo = bA + f;
    for (let k = 0; k < per3; k++) p[fo * per3 + k] = shotB.p[f * per3 + k];
    for (let k = 0; k < per4; k++) q[fo * per4 + k] = shotB.q[f * per4 + k];
  }
  for (let i = 0; i < nb; i++) writeTrack(q, nb, i, 4, quatContinuity(readTrack(q, nb, i, 4)));
  return { n, nb, p, q };
}

export function stitchShots(shots, o = {}) {
  const root = o.root ?? 0, overlap = Math.max(0, Math.floor(o.overlap ?? 0));
  const nb = shots[0].nb;
  let acc = { n: shots[0].n, nb, p: Float64Array.from(shots[0].p), q: Float64Array.from(shots[0].q) };
  const starts = [0], offsets = [{ R: [0, 0, 0, 1], T: [0, 0, 0] }];
  for (let k = 1; k < shots.length; k++) {
    const ov = clamp(overlap, 0, Math.min(acc.n, shots[k].n));
    const A = transformAt(acc, acc.n - Math.max(ov, 1), root);
    const B = transformAt(shots[k], 0, root);
    const { offset } = gravityMatch(A, B);
    const moved = applyOffset(shots[k], offset, o);
    starts.push(acc.n - ov);
    offsets.push(offset);
    acc = crossfade(acc, moved, ov);
  }
  return { ...acc, starts, offsets };
}
