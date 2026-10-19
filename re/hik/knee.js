import { arc, qrot } from '../../ik.js';
import { maxExtensionGoal } from '../../hik.js';

const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => { const l = len(a); return l > 1e-300 ? mul(a, 1 / l) : [0, 0, 0]; };
const perp = (v, n) => sub(v, mul(n, dot(v, n)));
const qdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]); return l > 0 ? q.map(x => x / l) : [0, 0, 0, 1]; };
export const qconj = q => [-q[0], -q[1], -q[2], q[3]];
export const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
export const qaxis = (k, ang) => { const n = unit(k), s = Math.sin(ang / 2); return [n[0] * s, n[1] * s, n[2] * s, Math.cos(ang / 2)]; };

export function nlerp(a, b, t) {
  const s = qdot(a, b) >= 0 ? 1 : -1;
  return qnorm(a.map((x, i) => x + (s * b[i] - x) * t));
}

export function twistAngle(q, axis) {
  const n = unit(axis), p = dot([q[0], q[1], q[2]], n);
  return 2 * Math.atan2(p, q[3]);
}

export function scaledDelta(d, k) {
  return qnorm([d[0] * k, d[1] * k, d[2] * k, d[3]]);
}

const wrap = a => { let x = (a + Math.PI) % (2 * Math.PI); if (x < 0) x += 2 * Math.PI; return x - Math.PI; };
const turn = (c, q, p) => add(c, qrot(q, sub(p, c)));

export function realisticKneeSwivel(leg, qGoal, R, k) {
  if (!(R > 0) || !(k > 0)) return { ...leg };
  const d = qmul(nlerp(leg.qAnkle, qGoal, R), qconj(leg.qAnkle));
  const dk = scaledDelta(d, k);
  const a1 = turn(leg.top, dk, leg.ankle);
  const w = qmul(arc(sub(a1, leg.top), sub(leg.ankle, leg.top)), dk);
  return {
    ...leg,
    knee: turn(leg.top, w, leg.knee),
    qTop: qmul(w, leg.qTop), qKnee: qmul(w, leg.qKnee), qAnkle: qmul(w, leg.qAnkle),
  };
}

export function transferAnkleTwist(leg, qCarried, legRoll = 0) {
  const s = unit(sub(leg.ankle, leg.knee));
  const yl = qrot(qconj(leg.qKnee), s);
  const L0 = qmul(qconj(leg.qKnee), qCarried), Lc = qmul(qconj(leg.qKnee), leg.qAnkle);
  const dl = wrap(twistAngle(Lc, yl) - twistAngle(L0, yl));
  return { ...leg, qKnee: qmul(qaxis(s, dl * (1 - legRoll)), leg.qKnee), kneeTwist: dl * (1 - legRoll) };
}

function quatReach(top, P, G, T) {
  const v = sub(P, top), u = add(v, mul(sub(G, P), T));
  if (len(v) < 1e-9 || len(u) < 1e-9) return [0, 0, 0, 1];
  return arc(v, u);
}

export function solveLeg(o) {
  const T = o.T ?? 1, R = o.R ?? 0, hinge = o.hinge ?? [1, 0, 0];
  let leg = { top: o.top.slice(), knee: o.knee.slice(), ankle: o.ankle.slice(), qTop: o.qTop ?? [0, 0, 0, 1], qKnee: o.qKnee ?? [0, 0, 0, 1], qAnkle: o.qAnkle ?? [0, 0, 0, 1] };
  if (o.qGoal) leg = realisticKneeSwivel(leg, o.qGoal, R, o.realistic ?? 0);
  const top = leg.top;
  const L = o.maxLength ?? len(sub(leg.knee, top)) + len(sub(leg.ankle, leg.knee));
  const G = (o.resistKnee ?? 0) > 0 ? maxExtensionGoal(top, o.goal, leg.ankle, L, o.resistKnee, o.maxExt ?? 0.5, o.partMode ?? 0) : o.goal.slice();
  const rotAll = (c, q) => { leg.knee = turn(c, q, leg.knee); leg.ankle = turn(c, q, leg.ankle); };
  const q1 = quatReach(top, leg.ankle, G, T);
  rotAll(top, q1); leg.qTop = qmul(q1, leg.qTop); leg.qKnee = qmul(q1, leg.qKnee); leg.qAnkle = qmul(q1, leg.qAnkle);
  const n = unit(qrot(leg.qKnee, hinge));
  const a = perp(sub(leg.knee, top), n), b = perp(sub(leg.ankle, leg.knee), n);
  const la = len(a), lb = len(b);
  if (la > 1e-9 && lb > 1e-9) {
    const ax = dot(sub(leg.ankle, top), n), gl = len(sub(G, top));
    const c = Math.min(len(add(a, b)), la + lb), dd = gl * gl > ax * ax ? Math.sqrt(gl * gl - ax * ax) : 0;
    const dp = Math.min(c + T * (dd - c), la + lb);
    const th = x => Math.acos(clamp((la * la + lb * lb - x * x) / (2 * la * lb), -1, 1));
    const phi = Math.atan2(dot(cross(a, b), n), dot(a, b));
    const sg = Math.abs(phi) > 1e-9 ? Math.sign(phi) : (o.bendSign ?? 1);
    const qb = qaxis(n, sg * (th(c) - th(dp)));
    leg.ankle = turn(leg.knee, qb, leg.ankle); leg.qKnee = qmul(qb, leg.qKnee); leg.qAnkle = qmul(qb, leg.qAnkle);
  }
  const q2 = quatReach(top, leg.ankle, G, T);
  rotAll(top, q2); leg.qTop = qmul(q2, leg.qTop); leg.qKnee = qmul(q2, leg.qKnee); leg.qAnkle = qmul(q2, leg.qAnkle);
  const carried = leg.qAnkle;
  if (o.qGoal && R > 0) leg.qAnkle = nlerp(carried, o.qGoal, R);
  leg = transferAnkleTwist(leg, carried, o.legRoll ?? 0);
  return leg;
}
