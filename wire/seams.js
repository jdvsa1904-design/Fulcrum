import { gravityMatch, smoothstep } from '../stitch.js';

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const wrap = a => a - 2 * Math.PI * Math.round(a / (2 * Math.PI));
const qconj = q => [-q[0], -q[1], -q[2], q[3]];
const qrot = (q, v) => {
  const [x, y, z, w] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
};
const yawQ = th => [0, Math.sin(th / 2), 0, Math.cos(th / 2)];

function arcToY(up) {
  const u = unit(up), d = u[1];
  if (d > 1 - 1e-12) return [0, 0, 0, 1];
  if (d < -1 + 1e-12) return [1, 0, 0, 0];
  const c = cross(u, [0, 1, 0]), s = Math.sqrt((1 + d) * 2);
  return [c[0] / s, c[1] / s, c[2] / s, s / 2];
}

function axisRot(axis, ang, v) {
  const k = unit(axis), c = Math.cos(ang), s = Math.sin(ang), kv = cross(k, v), kd = dot(k, v) * (1 - c);
  return [v[0] * c + kv[0] * s + k[0] * kd, v[1] * c + kv[1] * s + k[1] * kd, v[2] * c + kv[2] * s + k[2] * kd];
}

const pos = (W, nb, f, i) => { const o = (f * nb + i) * 16 + 12; return [W[o], W[o + 1], W[o + 2]]; };

function sourceRange(name) {
  const m = /f0*(\d+)\s*[-_]\s*0*(\d+)/i.exec(String(name ?? ''));
  return m ? [+m[1], +m[2]] : null;
}

function gapBetween(prev, next) {
  if (Number.isFinite(next.gapFrames)) return Math.max(0, Math.round(next.gapFrames));
  const a = sourceRange(prev.name), b = sourceRange(next.name);
  return a && b ? Math.max(0, b[0] - a[1] - 1) : 0;
}

export function seamPlan({ W, nb, shots, pelvis, thighL, thighR, up = [0, 1, 0], blend = 6, velFrames = 2, headFrames = 24, slack = 1.5, force = false, tol = 1e-6 }) {
  if (!W || !shots || shots.length < 2) return [];
  const qu = arcToY(up);
  const toY = v => qrot(qu, v), fromY = v => qrot(qconj(qu), v);
  const live = [];
  const corrected = (k, f) => {
    const c = live[k], P = toY(pos(W, nb, f, pelvis)), L = toY(pos(W, nb, f, thighL)), R = toY(pos(W, nb, f, thighR));
    if (!c) return { p: P, L, R };
    const q = yawQ(c.yaw), m = v => add(add(qrot(q, sub(v, c.pivot)), c.pivot), c.T);
    return { p: m(P), L: m(L), R: m(R) };
  };
  const heading = s => { const h = sub(s.R, s.L); return Math.atan2(-h[2], h[0]); };
  const line = (k, a, len) => {
    let prev = null, acc = 0, st = 0, sh = 0, stt = 0, sth = 0;
    for (let j = 0; j < len; j++) {
      const h = heading(corrected(k, a + j));
      acc = prev === null ? h : acc + wrap(h - prev);
      prev = h;
      st += j; sh += acc; stt += j * j; sth += j * acc;
    }
    const mt = st / len, mh = sh / len, den = stt / len - mt * mt, slope = den > 1e-12 ? (sth / len - mt * mh) / den : 0;
    return { at: j => mh + slope * (j - mt), slope };
  };
  const flat = v => [v[0], 0, v[2]];
  const hlen = v => Math.hypot(v[0], v[2]);
  const vel = (k, a, b) => b > a ? mul(flat(sub(corrected(k, b).p, corrected(k, a).p)), 1 / (b - a)) : [0, 0, 0];
  const peak = (k, a, b) => { let m = 0; for (let f = a + 1; f <= b; f++) m = Math.max(m, hlen(sub(corrected(k, f).p, corrected(k, f - 1).p))); return m; };
  const out = [];
  for (let k = 1; k < shots.length; k++) {
    const A = shots[k - 1], B = shots[k];
    if (!A.n || !B.n) continue;
    const gap = gapBetween(A, B), steps = 1 + gap, lastA = A.start + A.n - 1, lastB = B.start + B.n - 1;
    const kA = Math.min(Math.max(1, headFrames), A.n), kB = Math.min(Math.max(1, headFrames), B.n);
    const a1 = corrected(k - 1, lastA), b0 = corrected(-1, B.start);
    const jumpBefore = hlen(sub(b0.p, a1.p));
    const allowed = slack * Math.max(peak(k - 1, lastA - kA + 1, lastA), peak(-1, B.start, B.start + kB - 1)) * steps;
    const hA = line(k - 1, lastA - kA + 1, kA), hB = line(-1, B.start, kB);
    const slope = kA > 1 && kB > 1 ? (hA.slope + hB.slope) / 2 : kA > 1 ? hA.slope : hB.slope;
    const angA = hA.at(kA - 1) + slope * steps, angB = hB.at(0);
    const needed = force || jumpBefore > allowed;
    let yaw = 0, T = [0, 0, 0];
    if (needed) {
      const vA = vel(k - 1, Math.max(A.start, lastA - velFrames), lastA);
      const r = gravityMatch({ p: a1.p, q: yawQ(angA) }, { p: b0.p, q: yawQ(angB) });
      yaw = wrap(2 * Math.atan2(r.offset.R[1], r.offset.R[3]));
      const vB = qrot(yawQ(yaw), vel(-1, B.start, Math.min(lastB, B.start + velFrames)));
      const target = add(a1.p, mul(add(vA, vB), steps / 2));
      const g = gravityMatch({ p: target, q: yawQ(angA) }, { p: b0.p, q: yawQ(angB) });
      T = sub(g.matched.p, b0.p);
      if (hlen(T) < tol && Math.abs(yaw) < tol) { T = [0, 0, 0]; yaw = 0; }
    }
    live[k] = { yaw, pivot: b0.p, T };
    out.push({
      shot: k, start: B.start, n: B.n, gapFrames: gap, needed,
      offset: { T: fromY(T), yawRad: yaw, pivot: pos(W, nb, B.start, pelvis) },
      blendFrames: Math.max(0, Math.round(blend)),
      jumpBefore, jumpAfter: hlen(sub(add(b0.p, T), a1.p)), allowedJump: allowed,
      yawJumpRad: wrap(angA - angB),
    });
  }
  return out;
}

export function seamRootOffsets(plan, n, { W, nb, pelvis, up = [0, 1, 0], mode = 'carry' } = {}) {
  const res = new Array(n).fill(null);
  if (!plan?.length) return res;
  const u = unit(up);
  for (const e of plan) {
    const { T, yawRad, pivot } = e.offset;
    if (!yawRad && !T[0] && !T[1] && !T[2]) continue;
    if (yawRad && !W) throw new Error('seamRootOffsets: W, nb and pelvis are needed when a seam has yaw');
    const end = Math.min(n, e.start + (e.n ?? n - e.start)), bl = e.blendFrames || 0;
    for (let f = Math.max(0, e.start); f < end; f++) {
      const k = f - e.start;
      const w = mode === 'local' ? (bl > 0 ? 1 - smoothstep(k / bl) : (k === 0 ? 1 : 0)) : 1;
      if (w <= 0) continue;
      const yaw = yawRad * w;
      let d = mul(T, w);
      if (yaw) {
        const p = pos(W, nb, f, pelvis), r = add(axisRot(u, yaw, sub(p, pivot)), pivot);
        d = add(d, sub(r, p));
      }
      res[f] = [d[0], d[1], d[2], yaw];
    }
  }
  return res;
}
