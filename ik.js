const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = a => Math.hypot(a[0], a[1], a[2]);
const norm = a => { const l = len(a); return l > 0 ? scale(a, 1 / l) : [0, 0, 0]; };
const perpTo = (v, n) => sub(v, scale(n, dot(v, n)));
const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
const qaxis = (k, ang) => { const n = norm(k), s = Math.sin(ang / 2); return [n[0] * s, n[1] * s, n[2] * s, Math.cos(ang / 2)]; };
export function qrot(q, v) {
  const [x, y, z, w] = q, ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w - iw * x - iy * z + iz * y, iy * w - iw * y - iz * x + ix * z, iz * w - iw * z - ix * y + iy * x];
}
export function arc(u, v) {
  const a = norm(u), b = norm(v), d = dot(a, b);
  if (d < -1 + 1e-12) {
    const p = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0], k = norm(cross(a, p));
    return [k[0], k[1], k[2], 0];
  }
  const c = cross(a, b), q = [c[0], c[1], c[2], 1 + d], l = Math.hypot(...q);
  return q.map(x => x / l);
}
export function rotAxis(v, k, ang) {
  const c = Math.cos(ang), s = Math.sin(ang), kv = cross(k, v), d = dot(k, v) * (1 - c);
  return [v[0] * c + kv[0] * s + k[0] * d, v[1] * c + kv[1] * s + k[1] * d, v[2] * c + kv[2] * s + k[2] * d];
}
const anyPerp = n => norm(cross(n, Math.abs(n[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0]));

export function defaultPole(root, mid, end) {
  return [2 * mid[0] - root[0] - end[0], 2 * mid[1] - root[1] - end[1], 2 * mid[2] - root[2] - end[2]];
}

export function twoBoneIK(root, mid, end, goal, o = {}) {
  const L1 = len(sub(mid, root)), L2 = len(sub(end, mid)), H = sub(goal, root), D = len(H);
  const Hh = D > 1e-12 ? scale(H, 1 / D) : norm(sub(end, root));
  const lo = Math.abs(L1 - L2), hi = L1 + L2, d = clamp(D, Math.max(lo, 1e-9 * hi), hi);
  const a = (L1 * L1 - L2 * L2 + d * d) / (2 * d), h = Math.sqrt(Math.max(0, L1 * L1 - a * a));
  const P = o.poleTarget ? sub(o.poleTarget, root) : o.pole ?? defaultPole(root, mid, end);
  const guard = o.guard ?? 'strict', eps = o.eps ?? 1e-3, Pp = perpTo(P, Hh), lp = len(Pp), lP = len(P);
  const restPlane = () => perpTo(qrot(arc(sub(end, root), H), sub(mid, root)), Hh);
  let b = null, source = 'pole';
  if (lP > 0 && (guard === 'maya' ? lp > 1e-12 * lP : lp > eps * lP)) b = scale(Pp, 1 / lp);
  else {
    const cands = guard === 'maya' ? [['rest', restPlane]] : [['prev', () => o.prevBend && perpTo(o.prevBend, Hh)], ['preferred', () => o.preferredBend && perpTo(o.preferredBend, Hh)], ['rest', restPlane]];
    for (const [name, fn] of cands) {
      const c = fn();
      if (c && len(c) > 1e-9) { b = norm(c); source = name; break; }
    }
    if (!b) { b = anyPerp(Hh); source = 'arbitrary'; }
  }
  const bt = o.twist ? rotAxis(b, Hh, o.twist) : b;
  const mid2 = add(add(root, scale(Hh, a)), scale(bt, h)), end2 = add(root, scale(Hh, d));
  const rotRoot = arc(sub(mid, root), sub(mid2, root));
  const rotMid = arc(qrot(rotRoot, sub(end, mid)), sub(end2, mid2));
  return { mid: mid2, end: end2, bend: b, twistedBend: bt, rotRoot, rotMid, d, reach: D, clamped: D > hi || D < lo, source };
}

export function bendDirFromHinge(root, mid, end, hinge, deg = 15) {
  const c = sub(end, mid), moved = sub(rotAxis(c, norm(hinge), deg * Math.PI / 180), c), line = norm(sub(end, root));
  return norm(scale(perpTo(moved, line), -1));
}

export function stablePole(A, B, C, o = {}, state = {}) {
  const height = o.height ?? 180, tol = (o.straightTol ?? 0.02) * height, lab = len(sub(B, A)), lbc = len(sub(C, B));
  let p0 = scale(add(scale(A, lbc), scale(C, lab)), 1 / (lab + lbc));
  const v0 = sub(B, p0);
  if (o.bendDir) {
    const s = clamp(len(v0) / ((o.straightBand ?? 0.05) * height), 0, 1);
    p0 = sub(p0, scale(norm(o.bendDir), height / 10 * (1 - s)));
  }
  const v = sub(B, p0), lv = len(v);
  let n, source = 'knee';
  if (lv < tol && state.prevN) { n = state.prevN; source = 'prev'; }
  else if (lv < 1e-9) { n = o.bendDir ? norm(o.bendDir) : anyPerp(norm(sub(C, A))); source = o.bendDir ? 'preferred' : 'arbitrary'; }
  else n = scale(v, 1 / lv);
  if (state.prevN && o.keepSide !== false && dot(n, state.prevN) < 0 && lv < 2 * tol) { n = state.prevN; source = 'prev'; }
  state.prevN = n;
  const pole = add(p0, scale(n, height / 3.333 + lv));
  return { pole, n, p0, source, poleVector: sub(pole, A) };
}

export function poleGuardTrack(frames, o = {}) {
  const state = {}, out = [];
  let prevBend = null;
  for (const fr of frames) {
    const sp = stablePole(fr.root, fr.mid, fr.end, o, state);
    const r = twoBoneIK(fr.root, fr.mid, fr.end, fr.goal, { ...o, poleTarget: sp.pole, prevBend, preferredBend: o.bendDir, twist: fr.twist ?? o.twist });
    prevBend = r.bend;
    out.push({ ...r, pole: sp.pole, poleSource: sp.source });
  }
  return out;
}

export function ikLimbSolve(L1, L2, G, o = {}) {
  const Rmax = o.rmax ?? (L1 + L2) * 0.999, Rmin = Math.abs(o.rmin ?? Math.abs(L1 - L2) + 1e-6);
  const dg = len(G), dir = dg > 0 ? scale(G, 1 / dg) : [0, 0, 1], d = clamp(dg, Rmin, Rmax);
  const cosK = clamp((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2), -1, 1), cosH = clamp((L1 * L1 + d * d - L2 * L2) / (2 * d * L1), -1, 1);
  return { d, dir, cosK, cosH, kneeInterior: Math.acos(cosK), knee: Math.PI - Math.acos(cosK), hipOff: Math.acos(cosH), clamped: d !== dg };
}

export function swivelAngle(n0, n, dir) {
  const phi = Math.acos(clamp(dot(norm(n0), norm(n)), -1, 1));
  return dot(cross(n0, n), dir) < 0 ? 2 * Math.PI - phi : phi;
}

export function signedSwivel(n0, n, dir) {
  return Math.atan2(dot(cross(norm(n0), norm(n)), norm(dir)), dot(norm(n0), norm(n)));
}

export function swivelPlane(pole, dir, fallback) {
  const n = perpTo(pole, norm(dir));
  return len(n) > 1e-12 ? norm(n) : fallback ?? anyPerp(norm(dir));
}

const interior = (p, q, r) => {
  const u = sub(p, q), v = sub(r, q);
  return Math.acos(clamp(dot(u, v) / (len(u) * len(v)), -1, 1)) * 180 / Math.PI;
};

export function springIK(rest, goal, o = {}) {
  const m = rest.length - 1, root = o.root ?? rest[0], base = rest[0];
  const a = [], axes = [];
  for (let j = 1; j < m; j++) {
    a.push(interior(rest[j - 1], rest[j], rest[j + 1]));
    const n = cross(sub(rest[j], rest[j - 1]), sub(rest[j + 1], rest[j]));
    axes.push(len(n) > 1e-12 ? norm(n) : o.axis ? norm(o.axis) : null);
  }
  const vals = o.bias ?? a.map(() => 0.5), b = vals.map(v => 1 + clamp(v, 0, 1)), bm = Math.min(...b), w = b.map(x => x / bm);
  const dr = len(sub(rest[m], base)), D = len(sub(goal, root)), comp = D - dr < 0;
  const pose = t => {
    const ang = a.map((x, k) => { const f = clamp(t * w[k], 0, 1); return comp ? x * (1 - f) : x + f * (180 - x); });
    const pts = rest.map(p => p.slice()), rots = new Array(m - 1);
    for (let j = m - 1; j >= 1; j--) {
      const k = j - 1, ax = axes[k], delta = ax ? (a[k] - ang[k]) * Math.PI / 180 : 0;
      rots[k] = ax ? qaxis(ax, delta) : [0, 0, 0, 1];
      if (!delta) continue;
      for (let i = j + 1; i <= m; i++) pts[i] = add(rest[j], qrot(rots[k], sub(pts[i], rest[j])));
    }
    return { ang, pts, rots };
  };
  let t = 0, it = 0, err = Math.abs(D - dr);
  if (err >= (o.tol ?? 1e-5)) {
    let lo = 0, hi = 1, best = null;
    t = 0.5;
    for (it = 0; it < (o.maxIter ?? 100); it++) {
      const r = len(sub(pose(t).pts[m], base)), e = D - r;
      if (!best || Math.abs(e) < best[0]) best = [Math.abs(e), t];
      if (Math.abs(e) < (o.tol ?? 1e-5)) break;
      if ((e > 0) === comp) hi = t; else lo = t;
      t = (lo + hi) / 2;
    }
    t = best[1]; err = best[0];
  }
  const P = pose(t), Hh = norm(sub(goal, root));
  const qAim = arc(sub(P.pts[m], base), sub(goal, root));
  const restPole = o.restPole ?? springRestPole(rest), pole = o.pole ?? restPole;
  const ref = perpTo(qrot(qAim, restPole), Hh), tgt = perpTo(pole, Hh);
  let phi = 0;
  if (len(ref) > 1e-12 && len(tgt) > 1e-12) phi = Math.atan2(dot(cross(norm(ref), norm(tgt)), Hh), dot(norm(ref), norm(tgt)));
  const qRoot = qmul(qaxis(Hh, phi + (o.twist ?? 0)), qAim);
  const positions = P.pts.map(p => add(root, qrot(qRoot, sub(p, base))));
  const rotations = [qRoot];
  let acc = [0, 0, 0, 1];
  for (let k = 0; k < m - 1; k++) { acc = qmul(acc, P.rots[k]); rotations.push(qmul(qRoot, acc)); }
  return { positions, rotations, interior: P.ang, t, iterations: it, reachError: err, weights: w, compressing: comp };
}

export function springRestPole(rest) {
  const m = rest.length - 1, axis = norm(sub(rest[m], rest[0]));
  let s = [0, 0, 0];
  for (let j = 1; j < m; j++) s = add(s, perpTo(sub(rest[j], rest[0]), axis));
  return len(s) > 1e-12 ? s : m === 2 ? defaultPole(rest[0], rest[1], rest[2]) : anyPerp(axis);
}
