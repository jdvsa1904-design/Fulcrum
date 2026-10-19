export const BLEND = { CUSTOM: 0, LINEAR: 1, EASE: 2, SLOW_IN: 3, FAST_IN: 4 };

export const WARP = {
  CUSTOM: 0, LINEAR: 1, SMOOTHED_ENDS: 2, GOING_FASTER: 3, SLOWING_DOWN: 4,
  LINEAR_REVERSED: 5, SMOOTHED_ENDS_REVERSED: 6, GOING_FASTER_REVERSED: 7, SLOWING_DOWN_REVERSED: 8,
};

export const MATCH_T = { NONE: 0, DIRECT: 1, GRAVITY: 2 };
export const MATCH_R = { NONE: 0, DIRECT: 1, GRAVITY: 2 };

const clamp01 = (x) => (x < 0 ? 0 : x > 1 ? 1 : x);
const smooth = (u) => u * u * (3 - 2 * u);

export function blendCurve(type, u, custom) {
  u = clamp01(u);
  let w;
  switch (type) {
    case BLEND.CUSTOM: w = custom ? custom(u) : u; break;
    case BLEND.LINEAR: w = u; break;
    case BLEND.EASE: w = smooth(u); break;
    case BLEND.SLOW_IN: w = u * u * u; break;
    case BLEND.FAST_IN: w = 1 - (1 - u) ** 3; break;
    default: w = u;
  }
  return clamp01(w);
}

export function preBlendFactor(t, start, duration, type = BLEND.EASE, custom) {
  const u = duration > 0 ? (t - start) / duration : 1;
  return blendCurve(type, u, custom);
}

export function postBlendFactor(t, start, duration, type = BLEND.EASE, custom) {
  const u = duration > 0 ? (start + duration - t) / duration : 1;
  return blendCurve(type, u, custom);
}

export function transition(t, outgoing, incoming) {
  const start = incoming.start;
  const dur = outgoing.stop - incoming.start;
  if (t < start) return { wOut: 1, wIn: 0 };
  if (t >= start + dur) return { wOut: 0, wIn: 1 };
  const w = preBlendFactor(t, start, dur, incoming.inType ?? BLEND.LINEAR, incoming.custom);
  return { wOut: 1 - w, wIn: w };
}

export function timeWarp(type, u, custom) {
  u = clamp01(u);
  const rev = type >= WARP.LINEAR_REVERSED;
  const base = rev ? type - 4 : type;
  let v;
  switch (base) {
    case WARP.CUSTOM: v = custom ? custom(u) : u; break;
    case WARP.SMOOTHED_ENDS: v = smooth(u); break;
    case WARP.GOING_FASTER: v = u * u * u; break;
    case WARP.SLOWING_DOWN: v = 1 - (1 - u) ** 3; break;
    default: v = u;
  }
  return rev ? 1 - v : v;
}

export function sourceTime(t, clip) {
  const { start, speed = 1, offset = 0, sourceMarkIn, sourceMarkOut, loop = false } = clip;
  const first = sourceMarkIn + offset;
  const local = (t - start) * speed;
  const firstLen = sourceMarkOut - first;
  if (!loop || local < firstLen) return { time: Math.min(first + local, sourceMarkOut), loop: 0 };
  const len = sourceMarkOut - sourceMarkIn;
  if (len <= 0) return { time: sourceMarkOut, loop: 0 };
  const rest = local - firstLen;
  const n = Math.floor(rest / len);
  return { time: sourceMarkIn + (rest - n * len), loop: n + 1 };
}

export function loopedValue(t, clip, evalSource) {
  const { time, loop } = sourceTime(t, clip);
  const v = evalSource(time);
  const lt = loopTranslation(loop, clip.loopTranslation ?? [0, 0, 0]);
  return [v[0] + lt[0], v[1] + lt[1], v[2] + lt[2]];
}

export function loopTranslation(loopIndex, loopT) {
  return [loopT[0] * loopIndex, loopT[1] * loopIndex, loopT[2] * loopIndex];
}

export function autoLoopTranslation(travelFirst, travelLast, gravity = true) {
  const d = [travelLast[0] - travelFirst[0], travelLast[1] - travelFirst[1], travelLast[2] - travelFirst[2]];
  if (gravity) d[1] = 0;
  return d;
}

export function travelNode(points, fn = 'average') {
  const n = points.length;
  const p = [0, 0, 0];
  for (const q of points) { p[0] += q[0] / n; p[1] += q[1] / n; p[2] += q[2] / n; }
  if (fn === 'floor') p[1] = 0;
  return p;
}

export function clipFrame(travelFirst, travelLast) {
  const dx = travelLast[0] - travelFirst[0];
  const dz = travelLast[2] - travelFirst[2];
  const yaw = Math.hypot(dx, dz) > 1e-9 ? Math.atan2(-dz, dx) : 0;
  return { t: [travelFirst[0], 0, travelFirst[2]], yawDeg: yaw * 180 / Math.PI, q: [0, Math.sin(yaw / 2), 0, Math.cos(yaw / 2)] };
}

export function qmul(a, b) {
  return [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
}
export const qconj = (q) => [-q[0], -q[1], -q[2], q[3]];
export function qrot(q, v) {
  const r = qmul(qmul(q, [v[0], v[1], v[2], 0]), qconj(q));
  return [r[0], r[1], r[2]];
}
function qnorm(q) { const l = Math.hypot(...q) || 1; return q.map((x) => x / l); }

export function arcQuat(from, to) {
  const a = from.map((x) => x / (Math.hypot(...from) || 1));
  const b = to.map((x) => x / (Math.hypot(...to) || 1));
  const d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  if (d < -1 + 1e-9) return [0, 1, 0, 0];
  const c = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  return qnorm([c[0], c[1], c[2], 1 + d]);
}

export function matchPose(A, B, tMode = MATCH_T.GRAVITY, rMode = MATCH_R.GRAVITY) {
  let t;
  if (tMode === MATCH_T.DIRECT) t = [...A.t];
  else if (tMode === MATCH_T.GRAVITY) t = [A.t[0], B.t[1], A.t[2]];
  else t = [...B.t];
  let q;
  if (rMode === MATCH_R.DIRECT) q = [...A.q];
  else if (rMode === MATCH_R.GRAVITY) {
    const fa = qrot(A.q, [1, 0, 0]); fa[1] = 0;
    const fb = qrot(B.q, [1, 0, 0]); fb[1] = 0;
    q = qnorm(qmul(arcQuat(fb, fa), B.q));
  } else q = [...B.q];
  return { t, q };
}

export function matchOffset(A, B, tMode, rMode) {
  const M = matchPose(A, B, tMode, rMode);
  const dq = qnorm(qmul(M.q, qconj(B.q)));
  const rb = qrot(dq, B.t);
  return { q: dq, t: [M.t[0] - rb[0], M.t[1] - rb[1], M.t[2] - rb[2]] };
}

export function applyOffset(off, pose) {
  const r = qrot(off.q, pose.t);
  return { t: [r[0] + off.t[0], r[1] + off.t[1], r[2] + off.t[2]], q: qnorm(qmul(off.q, pose.q)) };
}

export function matchObject(pose, kind, opts = {}) {
  if (kind === 'feet') {
    const { left, right } = opts;
    const t = [(left.t[0] + right.t[0]) / 2, Math.min(left.t[1], right.t[1]), (left.t[2] + right.t[2]) / 2];
    return { t, q: pose.q };
  }
  return pose;
}

export function matchTimes(mode, prev, sel, next, now) {
  const cl = (c, t) => Math.max(c.start, Math.min(c.stop - 1e-6, t));
  switch (mode) {
    case 'start': return { ref: prev, tRef: cl(prev, sel.start), tSel: cl(sel, sel.start) };
    case 'stop': return { ref: prev, tRef: cl(prev, prev.stop), tSel: cl(sel, prev.stop) };
    case 'nextStart': return { ref: next, tRef: cl(next, next.start), tSel: cl(sel, next.start) };
    case 'middle': {
      const m = (sel.start + prev.stop) / 2;
      return { ref: prev, tRef: cl(prev, m), tSel: cl(sel, m) };
    }
    default: {
      const mid = (sel.start + sel.stop) / 2;
      const ref = !next || (prev && Math.abs(prev.stop - mid) <= Math.abs(next.start - mid)) ? prev : next;
      return { ref, tRef: cl(ref, now), tSel: cl(sel, now) };
    }
  }
}
