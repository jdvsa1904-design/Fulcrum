import { qlog } from './filters.js';

const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = a => Math.hypot(a[0], a[1], a[2]);
const norm = a => { const l = len(a); return l > 0 ? scale(a, 1 / l) : [0, 0, 0]; };
const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
const qdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
const qinv = q => [-q[0], -q[1], -q[2], q[3]];
const qaxis = (k, ang) => { const n = norm(k), s = Math.sin(ang / 2); return [n[0] * s, n[1] * s, n[2] * s, Math.cos(ang / 2)]; };
const qrot = (q, v) => {
  const [x, y, z, w] = q, ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w - iw * x - iy * z + iz * y, iy * w - iw * y - iz * x + ix * z, iz * w - iw * z - ix * y + iy * x];
};
const nlerp = (a, b, t) => {
  const bb = qdot(a, b) < 0 ? b.map(x => -x) : b;
  return qnorm(a.map((x, i) => x + (bb[i] - x) * t));
};
const lerp3 = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];

const matVec = (m, v) => [dot(m[0], v), dot(m[1], v), dot(m[2], v)];
const matMul = (a, b) => [0, 1, 2].map(i => [0, 1, 2].map(j => a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]));
const matT = m => [0, 1, 2].map(i => [0, 1, 2].map(j => m[j][i]));
const colsToMat = (c0, c1, c2) => [0, 1, 2].map(i => [c0[i], c1[i], c2[i]]);
const matToQuat = m => {
  const t = m[0][0] + m[1][1] + m[2][2];
  let q;
  if (t > 0) {
    const s = Math.sqrt(t + 1) * 2;
    q = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4];
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const s = Math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2;
    q = [s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s];
  } else if (m[1][1] > m[2][2]) {
    const s = Math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2;
    q = [(m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s];
  } else {
    const s = Math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2;
    q = [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s];
  }
  return qnorm(q);
};

export const FOOT_MARKERS = { back: 4.5, middle: 13, front: 7, inn: 5, out: 5, bottom: 7.5 };

export function footMarkers(ankle, o = {}, toe = null) {
  const m = { ...FOOT_MARKERS, ...o }, k = (o.contactSize ?? 100) / 100, side = o.side ?? 1;
  const b = m.bottom * k, back = m.back * k, mid = m.middle * k, front = m.front * k, inn = m.inn * k * side, out = m.out * k * side;
  const loc = {
    backIn: [-inn, -b, -back], backOut: [out, -b, -back],
    midIn: [-inn, -b, mid], midOut: [out, -b, mid],
    frontIn: [-inn, -b, mid + front], frontOut: [out, -b, mid + front],
  };
  const res = {};
  for (const key of Object.keys(loc)) res[key] = add(ankle.p, qrot(ankle.q, loc[key]));
  if (toe) {
    for (const key of ['frontIn', 'frontOut']) res[key] = add(toe.p, qrot(toe.q, qrot(qinv(toe.q0 ?? toe.q), sub(res[key], toe.p))));
  }
  return res;
}

export function frame3(a, b, c) {
  const n = norm(cross(sub(b, a), sub(c, a))), s = norm(cross(add(sub(b, a), sub(c, a)), n)), t = cross(n, s);
  return { R: colsToMat(s, t, n), o: scale(add(add(a, b), c), 1 / 3) };
}

function rigidFit(f0, f1) {
  const R = matMul(f1.R, matT(f0.R));
  return { R, t: sub(f1.o, matVec(R, f0.o)) };
}

function applyRigid(d, ankle) {
  const q = qnorm(qmul(matToQuat(d.R), ankle.q));
  return { p: add(matVec(d.R, ankle.p), d.t), q };
}

export function floorContactStiff(ankle, o = {}, toe = null) {
  const f = o.floor ?? 0, mk = footMarkers(ankle, o, toe), pts = o.contactType && o.contactType > 0 ? [mk.backIn, mk.backOut, mk.midIn, mk.midOut] : Object.values(mk);
  const pen = Math.max(0, f - Math.min(...pts.map(p => p[1])));
  return { p: [ankle.p[0], ankle.p[1] + pen, ankle.p[2]], q: ankle.q.slice(), pen };
}

export function floorContactSmooth(ankle, o = {}, hasToe = false) {
  const f = o.floor ?? 0, iters = o.iterations ?? 4, m = { ...FOOT_MARKERS, ...o };
  const midExtra = o.contactType && o.contactType > 0 ? 0 : hasToe ? 0 : m.front;
  const k = (o.contactSize ?? 100) / 100;
  let a = { p: ankle.p.slice(), q: ankle.q.slice() };
  a.p[1] = Math.max(a.p[1], f + m.bottom * k);
  const quad = cur => {
    const mk = footMarkers(cur, { ...o, front: 0, middle: m.middle + midExtra });
    return [mk.backIn, mk.backOut, mk.midIn, mk.midOut];
  };
  for (let it = 0; it < iters; it++) {
    for (const tri of [[0, 1, 2], [1, 3, 2]]) {
      const P = quad(a), pts = tri.map(i => P[i]);
      if (Math.min(...pts.map(p => p[1])) >= f) continue;
      const F0 = frame3(pts[0], pts[1], pts[2]), F1 = frame3(...pts.map(p => [p[0], Math.max(p[1], f), p[2]]));
      a = applyRigid(rigidFit(F0, F1), a);
    }
  }
  return a;
}

export function toeBend(toe, ankle, o = {}, iterations = 4) {
  const f = o.floor ?? 0, axis = norm(qrot(ankle.q, [1, 0, 0])), side = o.side ?? 1;
  const m = { ...FOOT_MARKERS, ...o }, k = (o.contactSize ?? 100) / 100;
  const base = footMarkers(ankle, { ...o, front: m.front }), local = [base.frontIn, base.frontOut].map(p => sub(p, toe.p));
  let q = toe.q.slice(), total = 0;
  for (let it = 0; it < iterations; it++) {
    const rot = qmul(qaxis(axis, total), [0, 0, 0, 1]);
    const pts = local.map(l => add(toe.p, qrot(rot, l))), ys = pts.map(p => p[1]), lo = Math.min(...ys);
    if (lo >= f - 1e-9) break;
    const rel = local[ys.indexOf(lo)];
    const A = rel[1] - axis[1] * dot(axis, rel), B = cross(axis, rel)[1], C = f - toe.p[1] - axis[1] * dot(axis, rel);
    const r = Math.hypot(A, B);
    if (r < 1e-12 || Math.abs(C) > r) break;
    const phi = Math.atan2(B, A), d = Math.acos(clamp(C / r, -1, 1)), c1 = phi + d, c2 = phi - d;
    const wrap = x => Math.atan2(Math.sin(x), Math.cos(x));
    const cand = [wrap(c1), wrap(c2)].sort((x, y) => Math.abs(x - total) - Math.abs(y - total));
    total = cand[0];
  }
  q = qmul(qaxis(axis, total), toe.q);
  return { q, angle: total, side, k };
}

export function floorContact(foot, o = {}) {
  const f = o.floor ?? 0, s = clamp(o.stiffness ?? 0, 0, 1), pivot = o.pivot ?? 'auto';
  const ankle = foot.ankle, toe = foot.toe ?? null;
  const toeLocal = toe ? { p: qrot(qinv(ankle.q), sub(toe.p, ankle.p)), q: qmul(qinv(ankle.q), toe.q) } : null;
  const follow = a => toeLocal ? { p: add(a.p, qrot(a.q, toeLocal.p)), q: qnorm(qmul(a.q, toeLocal.q)) } : null;
  const stiffA = floorContactStiff(ankle, o, toe && { ...toe, q0: toe.q });
  const smoothA = floorContactSmooth(ankle, o, !!toe);
  let smooth = { p: smoothA.p, q: smoothA.q };
  if (pivot === 'ankle') smooth.p = [ankle.p[0], smooth.p[1], ankle.p[2]];
  else if (pivot === 'toes' && toe) {
    const tn = follow(smooth);
    smooth.p = [smooth.p[0] + toe.p[0] - tn.p[0], smooth.p[1], smooth.p[2] + toe.p[2] - tn.p[2]];
  }
  const T = s <= 0 ? smooth.p : s >= 1 ? stiffA.p : lerp3(smooth.p, stiffA.p, s);
  const Q = s <= 0 ? smooth.q : s >= 1 ? stiffA.q : nlerp(smooth.q, stiffA.q, s);
  const out = { ankle: { p: T, q: Q }, toe: null, stiff: stiffA, smooth, penetration: stiffA.pen, touched: stiffA.pen > 0 || smooth.p[1] !== ankle.p[1] };
  if (toe) {
    let tn = follow(out.ankle);
    if ((o.contactType ?? 0) === 0 && (o.toeBend ?? true)) {
      const mk = footMarkers(out.ankle, o, { p: tn.p, q: tn.q, q0: tn.q });
      if (Math.min(mk.frontIn[1], mk.frontOut[1]) < f) {
        const tb = toeBend(tn, out.ankle, o);
        tn = { p: tn.p, q: tb.q };
        out.toeAngle = tb.angle;
      }
    }
    out.toe = tn;
  }
  return out;
}

export function kneeFloor(knee, o = {}) {
  const f = o.floor ?? 0, r = o.radius ?? 0, pen = Math.max(0, f + r - knee[1]), w = o.weight ?? 1;
  return { lift: pen * w, penetration: pen, p: [knee[0], knee[1] + pen * w, knee[2]] };
}

export function liftOnPenetration(goal, lift, up = [0, 1, 0]) {
  return add(goal, scale(up, lift));
}

export function rollSegments(r, s = 30, e = 60) {
  const sat = x => clamp(x, 0, 1), a = sat(r / s), b = e > s ? sat((r - s) / (e - s)) : r >= e ? 1 : 0;
  return { heel: Math.min(r, 0), toes: (1 - b) * a * r, toesEnd: b * r, a, b };
}

export function rockSegments(rock) {
  return { inner: clamp(rock, -200, 0), outer: clamp(rock, 0, 200) };
}

export function rollFoot(pts, r, pivots, o = {}) {
  const seg = rollSegments(r, o.start ?? 30, o.end ?? 60), axis = o.axis ?? [1, 0, 0];
  const out = {};
  const turn = (p, piv, deg) => add(piv, qrot(qaxis(axis, deg * Math.PI / 180), sub(p, piv)));
  for (const [name, p] of Object.entries(pts)) {
    let v = p;
    v = turn(v, pivots.ball, seg.toes);
    v = turn(v, pivots.toe, seg.toesEnd);
    v = turn(v, pivots.heel, seg.heel);
    out[name] = v;
  }
  return { points: out, ...seg };
}

export function detectPlants(pos, o = {}) {
  const n = pos.length / 3, S = o.legLength ?? 1, tol = (o.tol ?? 0.07) * S;
  const pl = new Uint8Array(n);
  for (let i = 1; i < n; i++) {
    const d = Math.hypot(pos[i * 3] - pos[(i - 1) * 3], pos[i * 3 + 1] - pos[(i - 1) * 3 + 1], pos[i * 3 + 2] - pos[(i - 1) * 3 + 2]);
    if (d <= tol) { pl[i] = 1; pl[i - 1] = 1; }
  }
  if (o.verticalTol != null) {
    const fl = o.floor ?? 0;
    for (let i = 0; i < n; i++) if (Math.abs(pos[i * 3 + 1] - fl) > o.verticalTol * S) pl[i] = 0;
  }
  for (let i = 1; i < n - 1; i++) if (pl[i - 1] && !pl[i] && pl[i + 1]) pl[i] = 1;
  for (let i = 1; i < n - 2; i++) if (pl[i - 1] && !pl[i] && !pl[i + 1] && pl[i + 2]) { pl[i] = 1; pl[i + 1] = 1; }
  const minRun = (o.minRun ?? 3) + 1, runs = [];
  let i = 0;
  while (i < n) {
    if (!pl[i]) { i++; continue; }
    let j = i;
    while (j + 1 < n && pl[j + 1]) j++;
    if (j - i + 1 < minRun) for (let k = i; k <= j; k++) pl[k] = 0;
    else runs.push({ s: i, e: j });
    i = j + 1;
  }
  const foot = o.footLength ?? 1, slideDist = foot * (o.slidingDist ?? 25) / 50, slideAng = (o.slidingAngle ?? 0) * Math.PI / 180;
  const wrap = x => Math.atan2(Math.sin(x), Math.cos(x));
  for (const r of runs) {
    r.mid = (r.s + r.e) >> 1;
    r.height = -1e6;
    r.sliding = false;
    for (let k = r.s; k <= r.e; k++) {
      r.height = Math.max(r.height, pos[k * 3 + 1]);
      if (k === r.s) continue;
      const dx = pos[k * 3] - pos[r.s * 3], dz = pos[k * 3 + 2] - pos[r.s * 3 + 2];
      if (Math.hypot(dx, dz) > slideDist) r.sliding = true;
      if (o.heading && Math.abs(wrap(o.heading[k] - o.heading[r.s])) > slideAng) r.sliding = true;
    }
  }
  return { planted: pl, runs };
}

export function ankleLock(pos, o = {}) {
  const n = pos.length / 3, per = o.perimeter ?? 3, prox = o.floorProximity ?? 3, minT = Math.max(1, Math.round((o.time ?? 0.1) * (o.fps ?? 30)));
  const fl = o.floor ?? 0, locked = new Uint8Array(n), anchors = new Array(n).fill(null), maxSlide = o.maxSlide ?? 0, dt = 1 / (o.fps ?? 30);
  const rot = o.rot ?? null, angPer = (o.angularPerimeter ?? 10) * Math.PI / 180;
  const quatAt = k => [rot[k * 4], rot[k * 4 + 1], rot[k * 4 + 2], rot[k * 4 + 3]];
  const turned = (i, k) => rot !== null && 2 * Math.acos(Math.min(1, Math.abs(qdot(quatAt(i), quatAt(k))))) > angPer;
  let i = 0;
  while (i < n) {
    if (pos[i * 3 + 1] - fl > prox) { i++; continue; }
    let anchor = [pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]], j = i;
    while (j + 1 < n) {
      const p = [pos[(j + 1) * 3], pos[(j + 1) * 3 + 1], pos[(j + 1) * 3 + 2]];
      if (p[1] - fl > prox) break;
      if (turned(i, j + 1)) break;
      const d = Math.hypot(p[0] - anchor[0], p[2] - anchor[2]);
      if (d > per) {
        if (maxSlide <= 0) break;
        const step = clamp(d - per, 0, maxSlide * dt), dir = [(p[0] - anchor[0]) / d, 0, (p[2] - anchor[2]) / d];
        if (d - per > step * 4) break;
        anchor = [anchor[0] + dir[0] * step, anchor[1], anchor[2] + dir[2] * step];
      }
      j++;
    }
    if (j - i + 1 >= minT) {
      let a = [pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]];
      for (let k = i; k <= j; k++) {
        locked[k] = 1;
        if (maxSlide > 0) {
          const p = [pos[k * 3], pos[k * 3 + 1], pos[k * 3 + 2]], d = Math.hypot(p[0] - a[0], p[2] - a[2]);
          if (d > per) { const s = clamp(d - per, 0, maxSlide * dt); a = [a[0] + (p[0] - a[0]) / d * s, a[1], a[2] + (p[2] - a[2]) / d * s]; }
        }
        anchors[k] = a.slice();
      }
    }
    i = j + 1;
  }
  return { locked, anchors };
}

export function recoveryBlend(n, releaseFrame, fps, o = {}) {
  const t = (o.recoveryTime ?? 0.15) * fps, w = new Float64Array(n);
  const zeta = o.damping ?? 5;
  for (let f = 0; f < n; f++) {
    const k = f - releaseFrame;
    if (k < 0) { w[f] = 1; continue; }
    const x = t > 0 ? zeta * k / t : k > 0 ? 1e9 : 0;
    w[f] = (1 + x) * Math.exp(-x);
  }
  return w;
}

export function kneeExtensionFlags(knee, ikBlend, maxAngle) {
  const flags = [];
  for (let i = 0; i < knee.length; i++) if (knee[i] > maxAngle && (ikBlend ? ikBlend[i] === 1 : true)) flags.push(i);
  return flags;
}

export function kneeExtensionFix(knee, ikBlend, maxAngle, o = {}) {
  const out = Float64Array.from(knee), flags = kneeExtensionFlags(knee, ikBlend, maxAngle), n = knee.length, h = o.half ?? 2;
  for (const i of flags) if (!ikBlend || ikBlend[i] > 0.99) out[i] = maxAngle;
  const mark = new Uint8Array(n);
  for (const i of flags) for (let k = Math.max(0, i - h); k <= Math.min(n - 1, i + h); k++) mark[k] = 1;
  const src = Float64Array.from(out);
  for (let i = 0; i < n; i++) {
    if (!mark[i]) continue;
    let s = 0, c = 0;
    for (let k = Math.max(0, i - h); k <= Math.min(n - 1, i + h); k++) { s += src[k]; c++; }
    out[i] = Math.min(maxAngle, s / c);
  }
  return { knee: out, flags };
}

export function kneeWobbleRanges(knee, o = {}) {
  const th = o.threshold ?? 0.02, W = o.window ?? 6, events = [], ranges = [];
  for (let i = 1; i < knee.length - 1; i++) {
    const d = knee[i + 1] - knee[i], p = knee[i] - knee[i - 1];
    if (d * p < 0 && Math.abs(d) > th) events.push(i);
  }
  for (let k = 1; k < events.length; k++) if (events[k] - events[k - 1] < W) ranges.push({ s: events[k - 1], e: events[k] - 1 });
  return { events, ranges };
}

export function kneeWobbleFix(knee, ranges, o = {}) {
  const out = Float64Array.from(knee);
  for (const r of ranges) {
    const a = knee[Math.max(0, r.s - 1)], b = knee[Math.min(knee.length - 1, r.e + 1)], span = r.e + 1 - (r.s - 1);
    for (let i = r.s; i <= r.e; i++) {
      const t = span > 0 ? (i - (r.s - 1)) / span : 0, lin = a + (b - a) * t;
      out[i] = knee[i] + (lin - knee[i]) * (o.strength ?? 1);
    }
  }
  return out;
}

export function spikeFrames(track) {
  const n = track.length / 4, flags = [];
  const q = i => [track[i * 4], track[i * 4 + 1], track[i * 4 + 2], track[i * 4 + 3]];
  for (let i = 1; i < n - 1; i++) {
    let a = q(i - 1), b = q(i), c = q(i + 1);
    if (qdot(a, b) < 0) b = b.map(x => -x);
    if (qdot(b, c) < 0) c = c.map(x => -x);
    const w0 = qlog(qmul(qinv(a), b)), w1 = qlog(qmul(qinv(b), c));
    if (dot(w0, w1) <= 0 && (len(w0) > 1e-9 || len(w1) > 1e-9)) flags.push(i);
  }
  return flags;
}

export function lowestPoint(points) {
  let best = null;
  for (const [name, p] of Object.entries(points)) if (!best || p[1] < best.y) best = { name, y: p[1], p };
  return best;
}
