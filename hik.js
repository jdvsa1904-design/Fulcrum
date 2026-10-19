import { arc, qrot, rotAxis } from './ik.js';

const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => { const l = len(a); return l > 1e-300 ? mul(a, 1 / l) : [0, 0, 0]; };
const perp = (v, n) => sub(v, mul(n, dot(v, n)));
const lerp3 = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]); return l > 0 ? q.map(x => x / l) : [0, 0, 0, 1]; };
const qdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];

export function slerp(a, b, t) {
  let d = qdot(a, b);
  const bb = d < 0 ? b.map(x => -x) : b;
  d = Math.abs(d);
  if (d > 0.9999995) return qnorm(a.map((x, i) => x + t * (bb[i] - x)));
  const th = Math.acos(d), s = Math.sin(th), wa = Math.sin((1 - t) * th) / s, wb = Math.sin(t * th) / s;
  return qnorm(a.map((x, i) => x * wa + bb[i] * wb));
}

export const RETARGET_PULL_NORM = 14.7;

export const DEFAULTS = Object.freeze({
  resistKnee: 0.5, maxExt: 0.5, compression: 0.5, pullIters: 10, gravity: 0.6,
  pullExponent: 2.325, gains: { hips: [0.75, 0.725], chest: [0.1, 0.325] },
});

export function maxExtensionGoal(top, goal, current, L, resist = DEFAULTS.resistKnee, maxExt = DEFAULTS.maxExt, partMode = 0) {
  const r = Math.sqrt(Math.max(0, resist * maxExt));
  const v = sub(goal, top), dt = len(v);
  if (dt < 1e-12) return goal.slice();
  const dc = len(sub(current, top)), k = 180 / Math.PI;
  const at = Math.asin(clamp(dt / L, 0, 1)) * k, ac = Math.asin(clamp(dc / L, 0, 1)) * k;
  const w = r >= 1e-20 ? Math.pow(clamp((at + ac) / 180, 0, 1), (1 - r) * 30 + (partMode === 1 ? 0 : 15)) : 0;
  const d = L * Math.sin((at + (ac - at) * w) * Math.PI / 180);
  return add(top, mul(v, d / dt));
}

export function quatReach(top, P, G, T) {
  const v = sub(P, top), u = add(v, mul(sub(G, P), T));
  if (len(v) < 1e-12 || len(u) < 1e-12) return [0, 0, 0, 1];
  return arc(v, u);
}

const turnAbout = (top, q, p) => add(top, qrot(q, sub(p, top)));

export function midReachLow(top, knee, ankle, G, T, hinge, bendSign = 1) {
  const n = unit(hinge), a = perp(sub(knee, top), n), b = perp(sub(ankle, knee), n);
  const la = len(a), lb = len(b);
  if (la < 1e-9 || lb < 1e-9) return { knee: knee.slice(), ankle: ankle.slice(), angle: 0 };
  const lim = la + lb, c = Math.min(len(add(a, b)), lim);
  const dRaw = len(perp(sub(G, top), n));
  const d = Math.min(c + T * (dRaw - c), lim);
  const th = x => Math.acos(clamp((la * la + lb * lb - x * x) / (2 * la * lb), -1, 1));
  const phi = Math.atan2(dot(cross(a, b), n), dot(a, b));
  const s = Math.abs(phi) > 1e-9 ? Math.sign(phi) : bendSign;
  const angle = s * (th(c) - th(d));
  return { knee: knee.slice(), ankle: add(knee, rotAxis(sub(ankle, knee), n, angle)), angle };
}

export function swivelKnee(top, knee, ankle, pole) {
  const H = unit(sub(ankle, top)), kp = perp(sub(knee, top), H), pp = perp(sub(pole, top), H);
  if (len(kp) < 1e-9 || len(pp) < 1e-9) return knee.slice();
  const a = Math.atan2(dot(cross(kp, pp), H), dot(kp, pp));
  return add(top, rotAxis(sub(knee, top), H, a));
}

export function reachRotation(o) {
  const t1 = unit(sub(o.knee, o.top)), s1 = unit(sub(o.ankle, o.knee));
  const q1 = arc(o.restThigh, t1), q2 = qmul(arc(qrot(q1, o.restShin), s1), q1);
  const carried = qmul(q2, o.qIn ?? [0, 0, 0, 1]);
  return (o.R ?? 0) > 0 ? slerp(carried, o.qGoal, o.R) : carried;
}

export function limbReach(o) {
  const top = o.top, T = o.T ?? 1, hinge = o.hinge ?? [1, 0, 0];
  const L = o.maxLength ?? len(sub(o.knee, top)) + len(sub(o.ankle, o.knee));
  const G = maxExtensionGoal(top, o.goal, o.ankle, L, o.resistKnee, o.maxExt, o.partMode);
  const q1 = quatReach(top, o.ankle, G, T);
  const k1 = turnAbout(top, q1, o.knee), a1 = turnAbout(top, q1, o.ankle);
  const m = midReachLow(top, k1, a1, G, T, qrot(q1, hinge), o.bendSign ?? 1);
  const q2 = quatReach(top, m.ankle, G, T);
  let knee = turnAbout(top, q2, m.knee);
  const ankle = turnAbout(top, q2, m.ankle);
  if (o.pole) knee = swivelKnee(top, knee, ankle, o.pole);
  const out = { knee, ankle, goal: G, flexion: m.angle };
  if (o.qGoal) {
    out.qAnkle = reachRotation({
      top, knee, ankle, restThigh: o.restThigh ?? unit(sub(o.knee, top)), restShin: o.restShin ?? unit(sub(o.ankle, o.knee)),
      qIn: o.qIn, qGoal: o.qGoal, R: o.R ?? 0,
    });
  }
  if (o.end && o.goalEnd) {
    const rel = sub(o.end, o.ankle), end0 = add(ankle, rel);
    const qe = quatReach(ankle, end0, o.goalEnd, o.Tend ?? 1);
    out.end = turnAbout(ankle, qe, end0);
  } else if (o.end) out.end = add(ankle, sub(o.end, o.ankle));
  return out;
}

export function limbWeights(r) {
  const Tm = r.Tmid ?? 0, Pm = r.Pmid ?? 0, Tl = r.Tlow ?? 0, Pl = r.Plow ?? 0, Pe = r.Pend ?? 0;
  const e = r.endEnabled ? r.Tend ?? 0 : 0;
  let wm = Tm * Pm;
  let wl = (1 - wm) * Pl * Tl;
  let we = (1 - wl) * (1 - wm) * Pe * e;
  let rem = (1 - wm) - wl - we;
  we += rem * e;
  rem = (1 - wm) - wl - we;
  wl += rem * Tl;
  rem = (1 - wm) - wl - we;
  wm += rem * Tm;
  return { mid: wm, low: wl, end: we };
}

export function addWeightedQ(items) {
  let acc = [0, 0, 0, 0], ref = null;
  for (const [q, w] of items) {
    if (!ref) ref = q;
    const s = qdot(ref, q) < 0 ? -1 : 1;
    acc = acc.map((x, i) => x + s * w * q[i]);
  }
  return qnorm(acc);
}

export function pullForce(L, L0, Lmax, p, rc) {
  const k = (L < L0 ? rc : 1) * p;
  const L5 = Lmax - (Lmax - L0) * k, lin = (L - L0) * k;
  if (L <= L5) return lin;
  const L3 = ((Lmax - L5) * 1.5 + L5) * (1 - k) + k * Lmax;
  const over = (L - Lmax) * (1 - k) + lin;
  if (L >= L3) return over;
  const span = L3 - L5;
  if (span < 1e-12) return over;
  const s = (L - L5) / span, u = 1 - s;
  const P0 = (L5 - L0) * k, P3 = (L3 - Lmax) * (1 - k) + (L3 - L0) * k;
  const P1 = P0 + span / 3 * k, P2 = P3 - span / 3;
  return u * u * u * P0 + 3 * u * u * s * P1 + 3 * u * s * s * P2 + s * s * s * P3;
}

const ramp = (c, t) => c[0] + c[1] * t;

export function bodyReachRigid(st, limbs) {
  const legs = limbs.filter(l => l.kind === 'leg' && (l.T ?? 1) > 0);
  if (!legs.length) return st;
  let sum = [0, 0, 0], wsum = 0;
  for (const l of legs) {
    const root = add(l.body === 'chest' ? st.chest : st.hips, l.offset);
    const v = sub(l.goal, root), L = len(v), ex = Math.max(0, L - l.Lmax);
    sum = add(sum, mul(unit(v), ex * (l.T ?? 1)));
    wsum += l.T ?? 1;
  }
  const d = mul(sum, 1 / wsum);
  return { hips: add(st.hips, d), chest: add(st.chest, d) };
}

export function bodyPull(o) {
  const iters = o.pullIters ?? DEFAULTS.pullIters, N = Math.round(iters) + 2;
  const e = o.pullExponent ?? DEFAULTS.pullExponent;
  const gains = o.gains ?? DEFAULTS.gains;
  const g = o.gravity ?? DEFAULTS.gravity, up = unit(o.up ?? [0, 1, 0]), biped = o.biped ?? true;
  const limbs = o.limbs.map(l => ({ ...l, w: Math.pow(l.pull ?? 0, e) }));
  const sumW = limbs.reduce((s, l) => s + l.w * (l.T ?? 1), 0);
  const D = Math.max(1, o.pullNorm ?? sumW);
  let hips = o.hips.slice(), chest = o.chest.slice();
  const trace = [];
  for (let i = 0; i < N; i++) {
    const t = i / N;
    let fLegs = [0, 0, 0], fArms = [0, 0, 0], fHead = [0, 0, 0];
    for (const l of limbs) {
      if (l.w <= 0 || (l.T ?? 1) <= 0) continue;
      const root = add(l.body === 'chest' ? chest : hips, l.offset);
      const v = sub(l.goal, root), L = len(v);
      if (L < 1e-12) continue;
      const f = pullForce(L, l.L0, l.Lmax, l.pull, l.rc ?? DEFAULTS.compression);
      const F = mul(unit(v), f * l.w * (l.T ?? 1));
      if (l.kind === 'arm') fArms = add(fArms, F);
      else if (l.kind === 'head') fHead = add(fHead, F);
      else fLegs = add(fLegs, F);
    }
    let fHands = fArms;
    if (biped && g > 0) fHands = lerp3(fArms, mul(up, dot(up, fArms)), g);
    const dH = mul(add(add(fLegs, fHands), fHead), ramp(gains.hips, t) / D);
    const dC = add(dH, mul(add(fArms, fHead), ramp(gains.chest, t) / D));
    hips = add(hips, dH);
    chest = add(chest, dC);
    if (o.trace) trace.push({ pass: i, hips: hips.slice(), chest: chest.slice() });
  }
  let st = { hips, chest };
  if (o.rigid ?? true) st = bodyReachRigid(st, o.limbs);
  return { hips: st.hips, chest: st.chest, passes: N, trace };
}

export function hipsHeightCompensation(eff, h) {
  const d = unit(sub(add(eff.lHip, eff.rHip), add(eff.lAnkle, eff.rAnkle))), s = mul(d, h);
  return { ...eff, hips: add(eff.hips, s), lHip: add(eff.lHip, s), rHip: add(eff.rHip, s) };
}

export function ankleHeightCompensation(eff, h) {
  const lift = q => qrot(q ?? [0, 0, 0, 1], [0, h, 0]);
  return { ...eff, lAnkle: add(eff.lAnkle, lift(eff.lAnkleQ)), rAnkle: add(eff.rAnkle, lift(eff.rAnkleQ)) };
}

export function scaleCompensation(eff, percent, reference = [0, 0, 0]) {
  const s = percent * 0.01, out = { ...eff };
  for (const k of ['hips', 'lHip', 'rHip', 'lAnkle', 'rAnkle']) if (eff[k]) out[k] = add(reference, mul(sub(eff[k], reference), s));
  return out;
}

export function autoScale(dstLegL, dstLegR, dstHipsSpan, srcLen0, srcLen1) {
  return ((dstLegL + dstLegR) / 2 + dstHipsSpan) / (srcLen0 + srcLen1);
}

export function autoHipsHeight(dstLegLength, srcHipsHeight, ankleTerm = 0) {
  return dstLegLength + ankleTerm - srcHipsHeight;
}

export function massCenterCompensation(eff, percent, legL, legR = legL) {
  const clampLeg = (hip, ankle, L) => {
    const v = sub(hip, ankle), l = len(v);
    return l > L ? mul(v, L / l) : v;
  };
  const tl = add(eff.lAnkle, clampLeg(eff.lHip, eff.lAnkle, legL));
  const tr = add(eff.rAnkle, clampLeg(eff.rHip, eff.rAnkle, legR));
  const target = mul(add(tl, tr), 0.5);
  return { ...eff, hips: add(eff.hips, mul(sub(target, eff.hips), percent * 0.01)) };
}
