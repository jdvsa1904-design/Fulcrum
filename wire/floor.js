import { FOOT_MARKERS, kneeFloor } from '../contacts.js';
import { hikFootMarkers as footMarkers, hikFloorContact as floorContact, gizmoFootMarkers } from '../solvers/hik/contact.js';

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => { const l = Math.hypot(a[0], a[1], a[2]); return l > 0 ? [a[0] / l, a[1] / l, a[2] / l] : [0, 0, 0]; };
const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qconj = q => [-q[0], -q[1], -q[2], q[3]];
const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
const median = a => { const s = [...a].sort((x, y) => x - y); return s.length ? s[s.length >> 1] : 0; };

function quatFromCols(c0, c1, c2) {
  const m00 = c0[0], m10 = c0[1], m20 = c0[2], m01 = c1[0], m11 = c1[1], m21 = c1[2], m02 = c2[0], m12 = c2[1], m22 = c2[2];
  const t = m00 + m11 + m22;
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4]; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; q = [s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s]; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; q = [(m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s]; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; q = [(m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s]; }
  return qnorm(q);
}

function poseOf(M, o) {
  const c0 = norm([M[o], M[o + 1], M[o + 2]]), c1 = norm([M[o + 4], M[o + 5], M[o + 6]]), c2 = norm([M[o + 8], M[o + 9], M[o + 10]]);
  return { p: [M[o + 12], M[o + 13], M[o + 14]], q: quatFromCols(c0, c1, c2) };
}

const SIDE = { l: 1, r: -1 };

function flatFrame(ankle, ball) {
  const d = sub(ball.p, ankle.p), fwd = norm([d[0], 0, d[2]]), up = [0, 1, 0], x = cross(up, fwd);
  return { H: quatFromCols(x, up, fwd), fwd, up, d };
}

function averageQ(qs) {
  const r = qs[0], acc = [0, 0, 0, 0];
  for (const q of qs) { const s = dot(q, r) + q[3] * r[3] < 0 ? -1 : 1; for (let i = 0; i < 4; i++) acc[i] += q[i] * s; }
  return qnorm(acc);
}

function restPose(rest, i) {
  if (!rest) return null;
  if (Array.isArray(rest[i]) || ArrayBuffer.isView(rest[i])) return poseOf(rest[i], 0);
  return poseOf(rest, i * 16);
}

function calibrate(key, leg, { W, nb, n, upm, ground, rest, restFloor, markers }) {
  const u = upm / 100, refs = [];
  if (rest) {
    const a = restPose(rest, leg.foot), b = restPose(rest, leg.ball);
    if (a && b) refs.push({ a, b, floor: restFloor });
  }
  if (!refs.length) {
    const rows = [];
    for (let f = 0; f < n; f++) rows.push({ f, ay: W[(f * nb + leg.foot) * 16 + 13] - ground, by: W[(f * nb + leg.ball) * 16 + 13] - ground });
    const byLo = [...rows].sort((x, y) => Math.abs(x.by) - Math.abs(y.by)), low = byLo.slice(0, Math.max(1, Math.ceil(n * 0.4)));
    const pick = low.sort((x, y) => x.ay - y.ay).slice(0, Math.max(1, Math.ceil(low.length * 0.5)));
    for (const { f } of pick) refs.push({ a: poseOf(W, (f * nb + leg.foot) * 16), b: poseOf(W, (f * nb + leg.ball) * 16), floor: ground, f });
  }
  const offs = [], toes = [], bottoms = [], mids = [], ballH = [];
  for (const r of refs) {
    const fr = flatFrame(r.a, r.b);
    offs.push(qmul(qconj(r.a.q), fr.H));
    toes.push(qmul(qconj(fr.H), r.b.q));
    bottoms.push(r.a.p[1] - r.floor);
    mids.push(dot(fr.d, fr.fwd));
    ballH.push(r.b.p[1] - r.floor);
  }
  const middle = median(mids), k = middle / (FOOT_MARKERS.middle * u);
  const m = {
    bottom: median(bottoms), middle,
    back: FOOT_MARKERS.back * u * k, front: FOOT_MARKERS.front * u * k,
    inn: FOOT_MARKERS.inn * u, out: FOOT_MARKERS.out * u,
  };
  for (const [kk, v] of Object.entries(markers || {})) if (v != null) m[kk] = v * u;
  return { side: SIDE[key], source: rest && refs[0]?.f === undefined ? 'rest' : 'take', offsetQ: averageQ(offs), toeRestQ: averageQ(toes), markers: m, ballHeight: median(ballH), refFrames: refs.map(r => r.f).filter(f => f !== undefined) };
}

export function soleMarkerRig({ W, nb, n, upm, ground, legs, rest = null }) {
  const rig = {};
  for (const key of ['l', 'r']) {
    const leg = legs?.[key];
    if (!leg || leg.foot === undefined || leg.ball === undefined) continue;
    rig[key] = { leg, cal: calibrate(key, leg, { W, nb, n, upm, ground, rest, restFloor: 0, markers: null }) };
  }
  return rig;
}

export function soleMarkers(rig, W, nb, f, contactSize = 100) {
  const out = {};
  for (const [key, { leg, cal }] of Object.entries(rig)) {
    const foot = poseOf(W, (f * nb + leg.foot) * 16);
    out[key] = gizmoFootMarkers({ p: foot.p, q: qnorm(qmul(foot.q, cal.offsetQ)) }, { ...cal.markers, side: cal.side, contactSize });
  }
  return out;
}

export const MAX_TOE_TURN_DEG = 45;

export function floorTargets({ W, nb, n, upm, ground, legs, frames = null, stiffness = 0, pivot = 'ankle', rest = null, restFloor = 0, markers = null, contactType = 0, toeBend = true, tol_cm = 0.01 }) {
  const out = new Map(), u = upm / 100, tol = tol_cm * u, rig = {};
  const a0 = frames ? Math.max(0, frames[0]) : 0, b0 = frames ? Math.min(n - 1, frames[1]) : n - 1;
  const below = (mk, keys) => Math.min(...keys.map(k => mk[k][1]));
  const FRONT = ['frontIn', 'frontOut'], ALL = ['backIn', 'backOut', 'midIn', 'midOut', 'frontIn', 'frontOut'];
  for (const key of ['l', 'r']) {
    const leg = legs?.[key];
    if (!leg || leg.foot === undefined || leg.ball === undefined) continue;
    const cal = calibrate(key, leg, { W, nb, n, upm, ground, rest, restFloor, markers });
    rig[key] = cal;
    const o = { ...cal.markers, floor: ground, side: cal.side, stiffness, pivot, contactType, toeBend };
    for (let f = a0; f <= b0; f++) {
      const foot = poseOf(W, (f * nb + leg.foot) * 16), ball = poseOf(W, (f * nb + leg.ball) * 16);
      const ankle = { p: foot.p, q: qnorm(qmul(foot.q, cal.offsetQ)) };
      const flat0 = qnorm(qmul(ankle.q, cal.toeRestQ));
      const lo = below(footMarkers(ankle, o, { p: ball.p, q: ball.q, q0: flat0 }), ALL);
      if (!(lo < ground - tol)) continue;
      const res = floorContact({ ankle, toe: { p: ball.p, q: flat0 } }, o);
      const ankleQ = qnorm(qmul(res.ankle.q, qconj(cal.offsetQ)));
      const flat1 = qnorm(qmul(res.ankle.q, cal.toeRestQ)), carried = qnorm(qmul(res.ankle.q, qmul(qconj(ankle.q), ball.q)));
      let toeQ = carried, mk = footMarkers(res.ankle, o, { p: res.toe.p, q: carried, q0: flat1 });
      const bend = toeBend && contactType === 0 && res.toeAngle !== undefined && below(mk, FRONT) < ground - tol;
      const bentQ = qnorm(res.toe.q), turn = 2 * Math.acos(Math.min(1, Math.abs(bentQ[0] * carried[0] + bentQ[1] * carried[1] + bentQ[2] * carried[2] + bentQ[3] * carried[3]))) * 180 / Math.PI;
      const useBend = bend && turn <= MAX_TOE_TURN_DEG;
      if (useBend) { toeQ = bentQ; mk = footMarkers(res.ankle, o, { p: res.toe.p, q: toeQ, q0: flat1 }); }
      const t = { side: key, ankleP: res.ankle.p, ankleQ, lift_cm: (res.ankle.p[1] - foot.p[1]) / u, pen_cm: (ground - lo) / u, markers: mk };
      if (useBend) { t.ballQ = toeQ; t.toeAngle = res.toeAngle; }
      else if (bend) t.toeRejected = +turn.toFixed(1);
      if ([...t.ankleP, ...t.ankleQ, ...(t.ballQ || [])].some(v => !Number.isFinite(v))) continue;
      if (!out.has(f)) out.set(f, []);
      out.get(f).push(t);
    }
  }
  out.rig = rig;
  return out;
}

export function kneeFloorTargets({ W, nb, n, upm, ground, knees, frames = null, radius_cm = 6 }) {
  const out = new Map(), u = upm / 100, r = radius_cm * u;
  const a0 = frames ? Math.max(0, frames[0]) : 0, b0 = frames ? Math.min(n - 1, frames[1]) : n - 1;
  for (const key of ['l', 'r']) {
    const k = knees?.[key];
    if (k === undefined) continue;
    for (let f = a0; f <= b0; f++) {
      const o = (f * nb + k) * 16, p = [W[o + 12], W[o + 13], W[o + 14]], res = kneeFloor(p, { floor: ground, radius: r });
      if (res.penetration <= 1e-9) continue;
      if (!out.has(f)) out.set(f, []);
      out.get(f).push({ side: key, kneeP: res.p, lift_cm: res.lift / u });
    }
  }
  return out;
}
