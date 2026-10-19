import { NLM, buildSkeleton, landmarksFromPose, restPose } from './skeleton.js';
import { lowpass } from './clean.js';
import { floorFromPoints, torsoUp } from './ground.js';
import { decideSigns } from './depthsign.js';

const ST = 5;
export const RAY_JOINTS = [23, 24, 11, 12, 13, 14, 15, 16, 25, 26, 27, 28, 29, 30, 31, 32, 7, 8, 0];
export const K_RAY = 19;
export const TREE = [[23, 24, 'hipW'], [23, 11, 'flank'], [24, 12, 'flank'], [11, 13, 'uArm'], [13, 15, 'fArm'], [12, 14, 'uArm'], [14, 16, 'fArm'],
  [23, 25, 'thigh'], [25, 27, 'shin'], [27, 29, 'heel'], [27, 31, 'toe'], [24, 26, 'thigh'], [26, 28, 'shin'], [28, 30, 'heel'], [28, 32, 'toe'], [11, 7, 'neckS'], [12, 8, 'neckS'], [7, 0, 'noseE']];
export const CLOSE = [[11, 12, 'shW'], [23, 12, 'diag'], [24, 11, 'diag'], [29, 31, 'sole'], [30, 32, 'sole'], [7, 8, 'earW'], [8, 0, 'noseE']];
export const NE = 18;
export const EDGES = Object.freeze([...TREE, ...CLOSE]);
export const SIG_L = { hipW: .01, flank: .025, uArm: .01, fArm: .01, thigh: .01, shin: .01, heel: .008, toe: .015, neckS: .03, noseE: .01, shW: .02, diag: .04, sole: .012, earW: .01 };
export const LENGTH_CLASS = { long: ['thigh', 'shin', 'uArm', 'fArm', 'flank'], width: ['hipW', 'shW', 'diag'], short: ['heel', 'toe', 'sole', 'neckS', 'earW', 'noseE'] };
export const ACC = { 23: 4, 24: 4, 11: 6, 12: 6, 7: 6, 8: 6, 0: 6, 13: 20, 14: 20, 25: 20, 26: 20, 15: 40, 16: 40, 27: 40, 28: 40, 29: 40, 30: 40, 31: 50, 32: 50 };
export const SLOT = Int8Array.from({ length: NLM }, (_, j) => RAY_JOINTS.indexOf(j));
export const ANCHOR = Int8Array.from({ length: NLM }, (_, j) => SLOT[j] >= 0 ? j : j <= 10 ? 0 : j % 2 ? 15 : 16);
export const PATH = Object.freeze(Array.from({ length: NLM }, (_, j) => {
  const out = [];
  for (let c = ANCHOR[j], e; (e = TREE.findIndex(x => x[1] === c)) >= 0; c = TREE[e][0]) out.unshift(e);
  if (out[0] !== 0) out.unshift(0);
  return Object.freeze(out);
}));
export const COM_W = [[[0, 7, 8], .081], [[11, 12, 23, 24], .497], [[11, 13], .028], [[12, 14], .028], [[13, 15], .016], [[14, 16], .016], [[15], .006], [[16], .006],
  [[23, 25], .1], [[24, 26], .1], [[25, 27], .0465], [[26, 28], .0465], [[29, 31], .0145], [[30, 32], .0145]];
export const SOLE = [29, 31, 30, 32];
export const DEPTH_DEFAULTS = Object.freeze({ fc2d: 0, sigTRel: 0.005, sigTMin: 0.015, sigBar: 0.01, cauchy: 2, priorRel: 0.07, minObs: 60, omegaMax: 15,
  iters: 10, outer: 2, lambdaSolver: 6.9, rMoot: 0.02, kGate: 3, window: 900, overlap: 60, mpBlend: 0, mpJoint: 2, accScale: 0.5 });

const KEYS = Object.keys(SIG_L), KEY_EDGES = Object.fromEntries(KEYS.map(k => [k, EDGES.map((x, e) => x[2] === k ? e : -1).filter(e => e >= 0)]));
const SA = Int8Array.from(EDGES, x => SLOT[x[0]]), SB = Int8Array.from(EDGES, x => SLOT[x[1]]);
const pick = (s, j) => typeof s === 'number' ? s : s ? s[j] : 2;

export function branch(d, Xp, L) {
  const t0 = d[0] * Xp[0] + d[1] * Xp[1] + d[2] * Xp[2], rho = Math.hypot(d[1] * Xp[2] - d[2] * Xp[1], d[2] * Xp[0] - d[0] * Xp[2], d[0] * Xp[1] - d[1] * Xp[0]), h2 = (L - rho) * (L + rho);
  return { t0, rho, h2, h: h2 > 0 ? Math.sqrt(h2) : 0 };
}
export function deltaSigma(delta, rho, L, sigRho, sigL) {
  const sy = 2 * Math.sqrt(rho * rho * sigRho * sigRho + L * L * sigL * sigL);
  return sy / (2 * Math.sqrt(delta * delta + sy / 2));
}
export const hBounds = (h2, sH2, k) => ({ lo: Math.sqrt(Math.max(0, h2 - k * sH2)), hi: Math.sqrt(Math.max(0, h2 + k * sH2)) });

function despike(x, n, thr) {
  const c = Float64Array.from(x);
  for (let i = 1; i < n - 1; i++) {
    const a = c[i - 1], b = c[i], d = c[i + 1], m = Math.max(Math.min(a, b), Math.min(Math.max(a, b), d));
    if (Math.abs(b - m) > thr) x[i] = m;
  }
}
export function imageRays(img, n, width, height, focal, o = {}) {
  const R = new Float64Array(n * NLM * 3), u = new Float64Array(n), v = new Float64Array(n), fc = o.fc2d ?? 0;
  for (let j = 0; j < NLM; j++) {
    for (let i = 0; i < n; i++) { const b = (i * NLM + j) * ST; u[i] = img[b] * width; v[i] = img[b + 1] * height; }
    if (o.sigPx && o.despike !== false) { const thr = 6 * pick(o.sigPx, j); despike(u, n, thr); despike(v, n, thr); }
    const U = fc > 0 ? lowpass(u, o.fps ?? 30, fc) : u, V = fc > 0 ? lowpass(v, o.fps ?? 30, fc) : v;
    for (let i = 0; i < n; i++) {
      const a = (U[i] - width / 2) / focal, b = (V[i] - height / 2) / focal, l = Math.sqrt(a * a + b * b + 1), r = (i * NLM + j) * 3;
      R[r] = a / l; R[r + 1] = b / l; R[r + 2] = 1 / l;
    }
  }
  return R;
}
function medianOf(a, m) {
  if (!m) return NaN;
  const s = a.subarray(0, m).sort(), h = m >> 1;
  return m % 2 ? s[h] : (s[h - 1] + s[h]) / 2;
}
function madOf(a, m) {
  const med = medianOf(a, m);
  for (let k = 0; k < m; k++) a[k] = Math.abs(a[k] - med);
  return medianOf(a, m);
}
function jitter3(get, n, comps, valid, buf) {
  let m = 0;
  for (let i = 0; i + 3 < n; i++) {
    if (valid && !(valid[i] && valid[i + 1] && valid[i + 2] && valid[i + 3])) continue;
    for (let c = 0; c < comps; c++) buf[m++] = get(i + 3, c) - 3 * get(i + 2, c) + 3 * get(i + 1, c) - get(i, c);
  }
  return m >= 8 ? 1.4826 * madOf(buf, m) / Math.sqrt(20) : NaN;
}
export function estimateJitter(img, n, width, height, valid) {
  const out = new Float64Array(NLM), buf = new Float64Array(Math.max(1, 2 * n));
  for (let j = 0; j < NLM; j++) {
    const s = jitter3((i, c) => img[(i * NLM + j) * ST + c] * (c ? height : width), n, 2, valid, buf);
    out[j] = Math.max(0.5, Number.isFinite(s) ? s : 2);
  }
  return out;
}

const mate = j => j % 2 ? j + 1 : j - 1;
export function fixLegSwaps(img, n, width, height, sigPx, o = {}) {
  const out = Float64Array.from(img), fixed = [], win = new Uint8Array(n), tmp = new Float64Array(ST), dt = 1 / (o.fps ?? 30), tau = o.torso ?? 1;
  const px = (A, i, j, c) => A[(i * NLM + j) * ST + c] * (c ? height : width), lab = (j, s) => s ? mate(j) : j;
  const cc = new Float64Array(NLM), U = new Float64Array(2 * n), cost = new Float64Array(4 * n).fill(Infinity), back = new Int8Array(4 * n).fill(-1), bh = new Float64Array(Math.max(1, n));
  for (let i = 0; i < n; i++) {
    let y0 = Infinity, y1 = -Infinity;
    for (let j = 0; j < NLM; j++) { const y = px(img, i, j, 1); if (y < y0) y0 = y; if (y > y1) y1 = y; }
    const a2 = (j, k) => (px(img, i, j, 0) - px(img, i, k, 0)) ** 2 + (px(img, i, j, 1) - px(img, i, k, 1)) ** 2, cI = a2(23, 11) + a2(24, 12), cS = a2(24, 11) + a2(23, 12), e = (cS - cI) / (cS + cI + 1e-9);
    U[2 * i] = -tau * e; U[2 * i + 1] = tau * e; bh[i] = y1 - y0;
  }
  const ap = 40 * dt * dt * Math.max(50, medianOf(bh, n) || 0) / 1.7;
  for (let j = 23; j <= 32; j++) cc[j] = 54 * pick(sigPx, j) ** 2 + ap * ap;
  const c2 = (i, a, b, c) => {
    let r = 0;
    for (let j = 23; j <= 32; j++) for (let k = 0; k < 2; k++) { const v = px(img, i, lab(j, c), k) - 2 * px(img, i - 1, lab(j, b), k) + px(img, i - 2, lab(j, a), k); r += Math.log1p(v * v / cc[j]); }
    return r;
  };
  if (n < 3) return { img: out, fixed, window: win };
  for (let p = 0; p < 4; p++) cost[4 + p] = U[p >> 1] + U[2 + (p & 1)];
  for (let i = 2; i < n; i++) for (let p = 0; p < 4; p++) {
    const a = p >> 1, b = p & 1;
    if (!Number.isFinite(cost[4 * (i - 1) + p])) continue;
    for (let c = 0; c < 2; c++) {
      const q = b * 2 + c, v = cost[4 * (i - 1) + p] + c2(i, a, b, c) + U[2 * i + c];
      if (v < cost[4 * i + q]) { cost[4 * i + q] = v; back[4 * i + q] = p; }
    }
  }
  const st = new Uint8Array(n);
  let q = 0;
  for (let p = 1; p < 4; p++) if (cost[4 * (n - 1) + p] < cost[4 * (n - 1) + q]) q = p;
  for (let i = n - 1; i >= 1; i--) { st[i] = q & 1; if (i === 1) st[0] = q >> 1; else q = back[4 * i + q]; }
  for (let i = 0; i < n; i++) if (st[i]) {
    for (let j = 23; j <= 32; j += 2) { const a = (i * NLM + j) * ST, b = (i * NLM + j + 1) * ST; tmp.set(out.subarray(a, a + ST)); out.copyWithin(a, b, b + ST); out.set(tmp, b); }
    fixed.push(i);
  }
  const thr = 4 * Math.max(pick(sigPx, 25), pick(sigPx, 26)) * Math.SQRT2;
  for (let i = 0; i < n; i++) if (Math.hypot(px(out, i, 25, 0) - px(out, i, 26, 0), px(out, i, 25, 1) - px(out, i, 26, 1)) < thr) for (let k = Math.max(0, i - 3); k <= Math.min(n - 1, i + 3); k++) win[k] = 1;
  return { img: out, fixed, window: win };
}
export function swapLegRows(arr, i, stride) {
  for (let j = 23; j <= 32; j += 2) for (let c = 0; c < stride; c++) { const a = (i * NLM + j) * stride + c, b = a + stride, x = arr[a]; arr[a] = arr[b]; arr[b] = x; }
}

export function boneSamples(R, n, Zp, world, sigLat) {
  const E = EDGES.length, N = E * n, rho = new Float64Array(N), dxy = new Float64Array(N), dz = new Float64Array(N), s2 = new Float64Array(N), wxy2 = new Float64Array(N), wz2 = new Float64Array(N), ok = new Uint8Array(N);
  const vxyJ = new Float64Array(NLM), vzJ = new Float64Array(NLM), buf = new Float64Array(Math.max(1, 2 * n));
  for (let j = 0; j < NLM; j++) {
    const sx = jitter3((i, c) => world[(i * NLM + j) * 3 + c], n, 2, null, buf), sz = jitter3(i => world[(i * NLM + j) * 3 + 2], n, 1, null, buf);
    vxyJ[j] = Number.isFinite(sx) ? 2 * sx * sx : 0; vzJ[j] = Number.isFinite(sz) ? sz * sz : 0;
  }
  for (let e = 0; e < E; e++) {
    const [a, b] = EDGES[e], sa = pick(sigLat, a), sb = pick(sigLat, b);
    for (let i = 0; i < n; i++) {
      const ra = (i * NLM + a) * 3, rb = (i * NLM + b) * 3, za = Zp[i * NLM + a], zb = Zp[i * NLM + b], k = e * n + i;
      if (!(za > 0.1 && zb > 0.1)) continue;
      const tp = za / R[ra + 2], X = R[ra] * tp, Y = R[ra + 1] * tp, Z = R[ra + 2] * tp, bx = R[rb], by = R[rb + 1], bz = R[rb + 2], tc = zb / bz;
      rho[k] = Math.hypot(by * Z - bz * Y, bz * X - bx * Z, bx * Y - by * X);
      const wx = world[rb] - world[ra], wy = world[rb + 1] - world[ra + 1], wz = world[rb + 2] - world[ra + 2];
      dxy[k] = wx * bx + wy * by; dz[k] = wz * bz; wxy2[k] = wx * wx + wy * wy; wz2[k] = wz * wz;
      s2[k] = (sa * tp) ** 2 + (sb * tc) ** 2; ok[k] = 1;
    }
  }
  return { n, rho, dxy, dz, s2, wxy2, wz2, ok, vxy: Float64Array.from(EDGES, ([a, b]) => vxyJ[a] + vxyJ[b]), vz: Float64Array.from(EDGES, ([a, b]) => vzJ[a] + vzJ[b]) };
}

function median7(x, n, out, w) {
  for (let i = 0; i < n; i++) {
    let m = 0;
    for (let k = Math.max(0, i - 3); k <= Math.min(n - 1, i + 3); k++) {
      const v = x[k];
      if (v !== v) continue;
      let p = m++;
      while (p > 0 && w[p - 1] > v) { w[p] = w[p - 1]; p--; }
      w[p] = v;
    }
    out[i] = m ? (m % 2 ? w[m >> 1] : (w[(m >> 1) - 1] + w[m >> 1]) / 2) : NaN;
  }
  return out;
}
function wstats(v, w, m, idx) {
  for (let k = 0; k < m; k++) idx[k] = k;
  const ix = idx.subarray(0, m).sort((p, q) => v[p] - v[q]);
  let tot = 0;
  for (let k = 0; k < m; k++) tot += w[ix[k]];
  if (!(tot > 0)) return { med: NaN, mad: NaN, obs: 0 };
  let c = 0, med = NaN;
  for (let k = 0; k < m; k++) { c += w[ix[k]]; if (c >= tot / 2) { med = v[ix[k]]; break; } }
  for (let k = 0; k < m; k++) v[k] = Math.abs(v[k] - med);
  ix.sort((p, q) => v[p] - v[q]);
  let mad = NaN;
  c = 0;
  for (let k = 0; k < m; k++) { c += w[ix[k]]; if (c >= tot / 2) { mad = v[ix[k]]; break; } }
  return { med, mad, obs: tot };
}
let RATIOS = null;
export function depthRatios(skel) {
  const L = landmarksFromPose(skel, restPose(skel)), D = (a, b) => Math.hypot(L[a * 3] - L[b * 3], L[a * 3 + 1] - L[b * 3 + 1], L[a * 3 + 2] - L[b * 3 + 2]), ref = D(23, 25) + D(25, 27), out = {};
  for (const k of KEYS) out[k] = KEY_EDGES[k].reduce((s, e) => s + D(EDGES[e][0], EDGES[e][1]), 0) / KEY_EDGES[k].length / ref;
  return out;
}
export function boneLengths(S, o = {}) {
  const opt = { ...DEPTH_DEFAULTS, ...o }, n = S.n, E = EDGES.length, v = new Float64Array(Math.max(1, E * n)), w = new Float64Array(v.length), idx = new Uint32Array(v.length);
  const LONG = new Set(LENGTH_CLASS.long), dl = (k, kap) => S.dxy[k] + S.dz[k] / kap;
  const estimate = kap => {
    const r = {};
    for (const key of KEYS) {
      let m = 0, ws = 0, w2 = 0;
      for (const e of KEY_EDGES[key]) for (let i = 0; i < n; i++) {
        const k = e * n + i;
        if (!S.ok[k]) continue;
        const d = dl(k, kap), rh = S.rho[k];
        v[m] = Math.sqrt(Math.max(0, rh * rh - 2 * S.s2[k]) + d * d); w[m] = Math.exp(-((d / (0.35 * rh + 1e-9)) ** 2)); ws += w[m]; w2 += w[m] * S.s2[k]; m++;
      }
      const st = wstats(v, w, m, idx);
      r[key] = { L: st.med, obs: st.obs, sd: Math.max(0.01 * st.med, 1.4826 * st.mad * 1.2533 / Math.sqrt(Math.max(st.obs, 1e-9) / 5)), spread: 1.4826 * st.mad, pred: Math.sqrt(w2 / Math.max(ws, 1e-12)) };
    }
    return r;
  };
  const yb = new Float64Array(n), db = new Float64Array(n), ym = new Float64Array(n), dm = new Float64Array(n), w7 = new Float64Array(7);
  const longSamples = (est, kap, fn) => {
    for (const key of LENGTH_CLASS.long) {
      const Lk = est[key].L;
      if (!(Lk > 0)) continue;
      for (const e of KEY_EDGES[key]) {
        for (let i = 0; i < n; i++) { const k = e * n + i; yb[i] = S.ok[k] ? Lk * Lk - Math.max(0, S.rho[k] ** 2 - 2 * S.s2[k]) : NaN; db[i] = S.ok[k] ? Math.abs(dl(k, kap)) : NaN; }
        median7(yb, n, ym, w7); median7(db, n, dm, w7);
        for (let i = 0; i < n; i++) { const h = Math.sqrt(Math.max(0, ym[i])); if (S.ok[e * n + i] && h + dm[i] / kap > Lk && dm[i] > 0.03) fn(e * n + i, h, dm[i], Lk, Math.sqrt(Math.max(0, yb[i])), db[i]); }
      }
    }
  };
  const fitKappa = (est, kap) => {
    for (let it = 0; it < 4; it++) {
      let m = 0;
      longSamples(est, kap, (k, h, d) => { v[m++] = d / h; });
      if (m < 20) return null;
      kap = Math.min(1.2, Math.max(0.3, kap * medianOf(v, m)));
    }
    return kap;
  };
  let kappa = 1, kappaOk = true, est = estimate(1);
  for (let pass = 0; pass < 2; pass++) {
    const k2 = fitKappa(est, kappa);
    if (k2 === null) { kappa = 1; kappaOk = false; est = estimate(1); break; }
    kappa = k2; est = estimate(kappa);
  }
  const ratios = RATIOS ??= depthRatios(buildSkeleton()), ref = (est.thigh.L + est.shin.L) || ratios.thigh + ratios.shin;
  const L = new Float64Array(E), sd = new Float64Array(E), obs = new Float64Array(E), src = Array(E).fill('image');
  for (const key of KEYS) {
    const pr = ratios[key] * ref, ps = opt.priorRel * pr, x = est[key], cls = LONG.has(key) ? 'long' : LENGTH_CLASS.width.includes(key) ? 'width' : 'short';
    let Lk = x.L, sk = x.sd, sr = 'image';
    if (cls === 'long' && !(x.obs > 1 && Lk > 0)) { Lk = pr; sk = ps; sr = 'prior'; }
    else if (cls === 'width') {
      if (x.obs >= opt.minObs && Lk > 0) { const a = 1 / sk ** 2, b = 1 / ps ** 2; Lk = (x.L * a + pr * b) / (a + b); sk = 1 / Math.sqrt(a + b); sr = 'image+prior'; }
      else { Lk = pr; sk = ps; sr = 'prior'; }
    } else if (cls === 'short') {
      let m = 0;
      for (const e of KEY_EDGES[key]) for (let i = 0; i < n; i++) {
        const k = e * n + i;
        if (S.ok[k]) v[m++] = Math.sqrt(Math.max(0, S.wxy2[k] + S.wz2[k] / kappa ** 2 - S.vxy[e] - S.vz[e] / kappa ** 2));
      }
      const lm = medianOf(v, m);
      if (lm > 0) { const smp = Math.max(0.05 * lm, 1.4826 * madOf(v, m) / Math.sqrt(m / 5)), a = 1 / smp ** 2, b = 1 / ps ** 2; Lk = (lm * a + pr * b) / (a + b); sk = 1 / Math.sqrt(a + b); sr = 'prior+mp'; }
      else { Lk = pr; sk = ps; sr = 'prior'; }
    }
    for (const e of KEY_EDGES[key]) { L[e] = Lk; sd[e] = sk; obs[e] = x.obs; src[e] = sr; }
  }
  let m = 0, g = 0;
  const gv = new Float64Array(v.length);
  longSamples(est, kappa, (k, hf, df, Lk, h, d) => { v[m++] = h - d; const sg = deltaSigma(h, S.rho[k], Lk, Math.sqrt(S.s2[k]), 0); gv[g++] = sg * sg; });
  const mad = m >= 30 ? 1.4826 * madOf(v, m) : NaN, sigMP = Number.isFinite(mad) ? Math.max(0.02, Math.sqrt(Math.max(0, mad * mad - medianOf(gv, g)))) : 0.06;
  let q = 0;
  for (const key of LENGTH_CLASS.long) { const x = est[key]; if (x.obs > 5 && x.pred > 0 && x.spread > 0) v[q++] = x.spread / x.pred; }
  return { L, sd, src, obs, kappa, kappaOk, sigMP, latRatio: q ? medianOf(v, q) : 1 };
}

function placeChild(R, i, e, ta, L, lo, signs, sp, f, kg, rho, h, delta, miss) {
  const ra = (i * NLM + EDGES[e][0]) * 3, rb = (i * NLM + EDGES[e][1]) * 3, X = R[ra] * ta, Y = R[ra + 1] * ta, Z = R[ra + 2] * ta, bx = R[rb], by = R[rb + 1], bz = R[rb + 2];
  const t0 = bx * X + by * Y + bz * Z, r = Math.hypot(by * Z - bz * Y, bz * X - bx * Z, bx * Y - by * X), Le = L[lo + e], h2 = (Le - r) * (Le + r), s = signs[i * NE + e], k = i * NE + e;
  let hh = 0;
  miss[k] = 0;
  if (h2 > 0) hh = Math.sqrt(h2);
  else {
    const sa = pick(sp, EDGES[e][0]) * ta / f, sb = pick(sp, EDGES[e][1]) * t0 / f;
    if ((r - Le) / Math.sqrt(sa * sa + sb * sb + SIG_L[EDGES[e][2]] ** 2) > kg) miss[k] = 1;
  }
  rho[k] = r; h[k] = hh; delta[k] = s * hh;
  return Math.max(0.05, t0 + s * hh);
}
function torsoClose(R, i, tl, L, lo, signs, sp, f, kg, rho, h, delta, miss) {
  const tr = placeChild(R, i, 0, tl, L, lo, signs, sp, f, kg, rho, h, delta, miss), t11 = placeChild(R, i, 1, tl, L, lo, signs, sp, f, kg, rho, h, delta, miss), t12 = placeChild(R, i, 2, tr, L, lo, signs, sp, f, kg, rho, h, delta, miss);
  const P = (j, t) => { const r = (i * NLM + j) * 3; return [R[r] * t, R[r + 1] * t, R[r + 2] * t]; }, a = P(23, tl), b = P(24, tr), c = P(11, t11), d = P(12, t12);
  const D = (x, y) => Math.hypot(x[0] - y[0], x[1] - y[1], x[2] - y[2]);
  return ((D(c, d) - L[lo + 18]) / SIG_L.shW) ** 2 + ((D(a, d) - L[lo + 19]) / SIG_L.diag) ** 2 + ((D(b, c) - L[lo + 20]) / SIG_L.diag) ** 2;
}
export function chainDepths(R, n, rootZ, L, signs, o = {}) {
  const t = new Float64Array(n * K_RAY), delta = new Float64Array(n * NE), h = new Float64Array(n * NE), rho = new Float64Array(n * NE), miss = new Uint8Array(n * NE);
  const f = o.focal ?? 1000, sp = o.sigPx ?? 2, kg = o.kGate ?? 3, perFrame = n > 1 && L.length >= n * EDGES.length;
  for (let i = 0; i < n; i++) {
    const rl = (i * NLM + 23) * 3, rr = (i * NLM + 24) * 3, lz = R[rl + 2], rz = R[rr + 2], Zm = rootZ[i], T = i * K_RAY, lo = perFrame ? i * EDGES.length : 0, s0 = signs[i * NE], cs = R[rl] * R[rr] + R[rl + 1] * R[rr + 1] + lz * rz;
    const al = 2 * Zm / rz, be = lz / rz, A = be * be + 1 + 2 * cs * be, Bq = -2 * al * (be + cs), Le = L[lo], disc = Bq * Bq - 4 * A * (al * al - Le * Le);
    let tl = -Bq / (2 * A);
    if (o.rootT) tl = o.rootT[i];
    else if (disc > 0 && s0) {
      const q = Math.sqrt(disc) / (2 * A), u = tl + q, v = tl - q, du = s0 * (al - be * u - u * cs), dv = s0 * (al - be * v - v * cs), t0 = Zm / lz;
      if (du >= 0 && dv >= 0) {
        const cu = torsoClose(R, i, u, L, lo, signs, sp, f, kg, rho, h, delta, miss), cv = torsoClose(R, i, v, L, lo, signs, sp, f, kg, rho, h, delta, miss);
        tl = Math.abs(cu - cv) > 1e-12 ? (cu < cv ? u : v) : Math.abs(u - t0) <= Math.abs(v - t0) ? u : v;
      } else tl = du >= dv ? u : v;
    }
    tl = Math.max(0.05, tl);
    const tr = placeChild(R, i, 0, tl, L, lo, signs, sp, f, kg, rho, h, delta, miss);
    t[T] = tl; t[T + 1] = tr;
    for (let e = 1; e < NE; e++) t[T + SB[e]] = placeChild(R, i, e, t[T + SA[e]], L, lo, signs, sp, f, kg, rho, h, delta, miss);
  }
  return { t, delta, h, rho, miss };
}

export function branchTable(R, n, t, L, Lsd, sigPx, focal, fps, o = {}) {
  const N = n * NE, h = new Float64Array(N), sH2 = new Float64Array(N), hLo = new Float64Array(N), hHi = new Float64Array(N), phiHat = new Float64Array(N), phiLo = new Float64Array(N);
  const wRay = new Float64Array(N), miss = new Uint8Array(N), delta = new Float64Array(N), rho = new Float64Array(N), sRho = new Float64Array(N), sDelta = new Float64Array(N);
  const kg = o.kGate ?? 3, sTp = o.sigTp ?? 0;
  for (let i = 0; i < n; i++) for (let e = 0; e < NE; e++) {
    const [a, b, key] = EDGES[e], ta = t[i * K_RAY + SA[e]], tb = t[i * K_RAY + SB[e]], ra = (i * NLM + a) * 3, rb = (i * NLM + b) * 3, k = i * NE + e;
    const X = R[ra] * ta, Y = R[ra + 1] * ta, Z = R[ra + 2] * ta, bx = R[rb], by = R[rb + 1], bz = R[rb + 2];
    const t0 = bx * X + by * Y + bz * Z, r = Math.hypot(by * Z - bz * Y, bz * X - bx * Z, bx * Y - by * X), Le = L[e], h2 = (Le - r) * (Le + r);
    const sa = pick(sigPx, a) * ta / focal, sb = pick(sigPx, b) * tb / focal, sr2 = sa * sa + sb * sb + (r / ta) ** 2 * sTp * sTp, sL2 = SIG_L[key] ** 2 + (Lsd ? Lsd[e] ** 2 : 0);
    const sy = 2 * Math.sqrt(r * r * sr2 + Le * Le * sL2), hh = h2 > 0 ? Math.sqrt(h2) : 0, lo = Math.sqrt(Math.max(0, h2 - kg * sy)), hi = Math.sqrt(Math.max(0, h2 + kg * sy));
    h[k] = hh; sH2[k] = sy; hLo[k] = lo; hHi[k] = hi; rho[k] = r; sRho[k] = Math.sqrt(sr2);
    phiHat[k] = Math.asin(Math.min(1, hh / Le)); phiLo[k] = Math.asin(Math.min(1, lo / Le));
    miss[k] = h2 < 0 && (r - Le) / Math.sqrt(sr2 + sL2) > kg ? 1 : 0;
    delta[k] = tb - t0; sDelta[k] = deltaSigma(tb - t0, r, Le, sRho[k], Math.sqrt(sL2));
    if (i + 1 < n) {
      const rc = rb + NLM * 3, cx = by * R[rc + 2] - bz * R[rc + 1], cy = bz * R[rc] - bx * R[rc + 2], cz = bx * R[rc + 1] - by * R[rc];
      wRay[k] = fps * Math.atan2(Math.hypot(cx, cy, cz), bx * R[rc] + by * R[rc + 1] + bz * R[rc + 2]);
    } else if (i > 0) wRay[k] = wRay[k - NE];
  }
  return { h, sH2, hLo, hHi, phiHat, phiLo, wRay, miss, delta, rho, sRho, sDelta };
}

export function bandChol(A, N, bw) {
  const w = bw + 1;
  for (let i = 0; i < N; i++) {
    const i0 = i > bw ? i - bw : 0, ri = i * w + i;
    for (let j = i0; j <= i; j++) {
      const rj = j * w + j;
      let s = A[ri - j], s1 = 0, m = j > bw && j - bw > i0 ? j - bw : i0;
      for (; m + 1 < j; m += 2) { s -= A[ri - m] * A[rj - m]; s1 += A[ri - m - 1] * A[rj - m - 1]; }
      if (m < j) s -= A[ri - m] * A[rj - m];
      s -= s1;
      if (j === i) { if (!(s > 0)) return false; A[i * w] = Math.sqrt(s); }
      else A[ri - j] = s / A[j * w];
    }
  }
  return true;
}
export function bandSolve(Lc, N, bw, b, out) {
  const w = bw + 1, y = out ?? new Float64Array(N);
  if (y !== b) y.set(b);
  for (let i = 0; i < N; i++) { let s = y[i]; for (let m = Math.max(0, i - bw); m < i; m++) s -= Lc[i * w + i - m] * y[m]; y[i] = s / Lc[i * w]; }
  for (let i = N - 1; i >= 0; i--) { let s = y[i]; const e = Math.min(N - 1, i + bw); for (let m = i + 1; m <= e; m++) s -= Lc[m * w + m - i] * y[m]; y[i] = s / Lc[i * w]; }
  return y;
}

function acc2(H, g, w, i1, v1, i2, v2, r) {
  g[i1] += v1 * r; g[i2] += v2 * r; H[i1 * w] += v1 * v1; H[i2 * w] += v2 * v2;
  if (i1 > i2) H[i1 * w + i1 - i2] += v1 * v2; else H[i2 * w + i2 - i1] += v1 * v2;
}
function acc1(H, g, w, i1, v1, r) { g[i1] += v1 * r; H[i1 * w] += v1 * v1; }

function solveWindow(p, o, B, a, m, t, chiOnly) {
  const K = K_RAY, N = m * K, bw = 2 * K, w = bw + 1, { R, L, signs, state, pWrong, world, floor } = p, kap = p.kappa || 1, f = p.focal, dt = 1 / p.fps;
  const sMP = p.sigMP || 0.06, cMP = o.cauchy, sBar = o.sigBar, sFl = 0.025, NL = EDGES.length;
  const { dX, dY, dZ, sLen, mpT, mpOn, cab, sT, accI, jT, jOn, mpS } = B, wJ = o.mpJoint || 0;
  for (let li = 0; li < m; li++) for (let k = 0; k < K; k++) { const r = ((a + li) * NLM + RAY_JOINTS[k]) * 3, q = li * K + k; dX[q] = R[r]; dY[q] = R[r + 1]; dZ[q] = R[r + 2]; }
  for (let li = 0; li < m; li++) {
    const i = a + li, T = li * K, tz = p.Tz[i];
    sT[li] = p.sigT == null ? Math.max(o.sigTMin, o.sigTRel * tz) : typeof p.sigT === 'number' ? p.sigT : p.sigT[i];
    for (let e = 0; e < NL; e++) {
      const ka = T + SA[e], kb = T + SB[e], ta = t[ka], tb = t[kb], vx = tb * dX[kb] - ta * dX[ka], vy = tb * dY[kb] - ta * dY[ka], vz = tb * dZ[kb] - ta * dZ[ka], l = Math.hypot(vx, vy, vz) || 1e-9;
      const vd = vx * dX[kb] + vy * dY[kb] + vz * dZ[kb], phi2 = Math.max(0, l * l - vd * vd) / (l * l), sa = pick(p.sigPx, EDGES[e][0]) * ta / f, sb = pick(p.sigPx, EDGES[e][1]) * tb / f;
      sLen[li * NL + e] = e < NE && state && state[i * NE + e] === 1 ? 0.1 : Math.sqrt(SIG_L[EDGES[e][2]] ** 2 + (p.Lsd ? p.Lsd[e] ** 2 : 0) + phi2 * (sa * sa + sb * sb));
      if (e >= NE) continue;
      const q = li * NE + e;
      cab[q] = dX[ka] * dX[kb] + dY[ka] * dY[kb] + dZ[ka] * dZ[kb];
      mpOn[q] = 0;
      if (!world) continue;
      const wa = (i * NLM + EDGES[e][0]) * 3, wb = (i * NLM + EDGES[e][1]) * 3, dm = (world[wb] - world[wa]) * dX[kb] + (world[wb + 1] - world[wa + 1]) * dY[kb] + (world[wb + 2] - world[wa + 2]) / kap * dZ[kb];
      if (!Number.isFinite(dm)) continue;
      const s = signs ? signs[i * NE + e] : 0, st = state ? state[i * NE + e] : 0;
      mpT[q] = st === 2 && s ? s * Math.abs(dm) : st === 1 && s ? (1 - 2 * (pWrong ? pWrong[i * NE + e] : 0)) * s * Math.abs(dm) : dm;
      mpOn[q] = 1; mpS[q] = dm < 0 ? -1 : 1;
    }
    if (wJ > 0) for (let k = 1; k < K; k++) {
      const j = RAY_JOINTS[k], qk = T + k, wj = (i * NLM + j) * 3, w0 = (i * NLM + 23) * 3;
      let on = !!world;
      for (const e of PATH[j]) if (!mpOn[li * NE + e] || (state && state[i * NE + e] > 0 && signs && mpS[li * NE + e] !== signs[i * NE + e])) { on = false; break; }
      const dm = on ? (world[wj] - world[w0]) * dX[qk] + (world[wj + 1] - world[w0 + 1]) * dY[qk] + (world[wj + 2] - world[w0 + 2]) / kap * dZ[qk] : NaN;
      jOn[qk] = Number.isFinite(dm) ? 1 : 0; jT[qk] = dm;
    }
  }
  for (let k = 0; k < K; k++) accI[k] = 1 / ((ACC[RAY_JOINTS[k]] ?? 20) * (o.accScale || 1) * dt * dt);
  const fl = floor && floor.normal ? floor : null, fn = fl ? fl.normal : null;
  const evaluate = (t, H, g, chi) => {
    if (H) { H.fill(0, 0, N * w); g.fill(0, 0, N); }
    let cost = 0;
    for (let li = 0; li < m; li++) {
      const i = a + li, T = li * K;
      let c2 = 0;
      for (let e = 0; e < NL; e++) {
        const ka = T + SA[e], kb = T + SB[e], ta = t[ka], tb = t[kb], vx = tb * dX[kb] - ta * dX[ka], vy = tb * dY[kb] - ta * dY[ka], vz = tb * dZ[kb] - ta * dZ[ka], l = Math.hypot(vx, vy, vz) || 1e-9, s = sLen[li * NL + e];
        const r = (l - L[e]) / s;
        cost += r * r; c2 += r * r;
        if (H) acc2(H, g, w, kb, (vx * dX[kb] + vy * dY[kb] + vz * dZ[kb]) / l / s, ka, -(vx * dX[ka] + vy * dY[ka] + vz * dZ[ka]) / l / s, r);
        if (e >= NE) continue;
        const q = li * NE + e, del = tb - ta * cab[q];
        if (mpOn[q]) {
          const rn = (del - mpT[q]) / sMP, u = 1 + rn * rn / (cMP * cMP), wt = 1 / Math.sqrt(u);
          cost += cMP * cMP * Math.log(u);
          if (H) acc2(H, g, w, kb, wt / sMP, ka, -cab[q] * wt / sMP, rn * wt);
        }
        if (state && state[i * NE + e] === 2) {
          const sg = signs[i * NE + e];
          if (sg * del < 0) { const r2 = sg * del / sBar; cost += r2 * r2; if (H) acc2(H, g, w, kb, sg / sBar, ka, -sg * cab[q] / sBar, r2); }
        }
      }
      if (wJ > 0) for (let k = 1; k < K; k++) {
        const qk = T + k;
        if (!jOn[qk]) continue;
        const c0 = dX[T] * dX[qk] + dY[T] * dY[qk] + dZ[T] * dZ[qk], sj = sMP / wJ, rn = (t[qk] - t[T] * c0 - jT[qk]) / sj, u = 1 + rn * rn / (cMP * cMP), wt = 1 / Math.sqrt(u);
        cost += cMP * cMP * Math.log(u);
        if (H) acc2(H, g, w, qk, wt / sj, T, -c0 * wt / sj, rn * wt);
      }
      if (chi) chi[i] = c2;
      const r0 = ((t[T] * dZ[T] + t[T + 1] * dZ[T + 1]) / 2 - p.Tz[i]) / sT[li];
      cost += r0 * r0;
      if (H) acc2(H, g, w, T, dZ[T] / 2 / sT[li], T + 1, dZ[T + 1] / 2 / sT[li], r0);
      if (li > 0 && li < m - 1) for (let k = 0; k < K; k++) {
        const i0 = T - K + k, i1 = T + k, i2 = T + K + k, c = accI[k], r = (t[i0] * dZ[i0] - 2 * t[i1] * dZ[i1] + t[i2] * dZ[i2]) * c;
        cost += r * r;
        if (H) {
          const v0 = dZ[i0] * c, v1 = -2 * dZ[i1] * c, v2 = dZ[i2] * c;
          g[i0] += v0 * r; g[i1] += v1 * r; g[i2] += v2 * r;
          H[i0 * w] += v0 * v0; H[i1 * w] += v1 * v1; H[i2 * w] += v2 * v2;
          H[i1 * w + K] += v1 * v0; H[i2 * w + K] += v2 * v1; H[i2 * w + 2 * K] += v2 * v0;
        }
      }
      if (fl) for (let q = 0; q < 4; q++) {
        const kq = T + SLOT[SOLE[q]], gr = fn[0] * dX[kq] - fn[1] * dY[kq] - fn[2] * dZ[kq], ht = gr * t[kq] - fl.offset;
        if (fl.contact && fl.contact[i * 4 + q] > 0.5) { const r = ht / sFl; cost += r * r; if (H) acc1(H, g, w, kq, gr / sFl, r); }
        if (ht < -0.01) { const r = (ht + 0.01) / sBar; cost += r * r; if (H) acc1(H, g, w, kq, gr / sBar, r); }
      }
    }
    return cost;
  };
  if (chiOnly) return { cost: evaluate(t, null, null, p.chi2out), iters: 0, ok: true };
  let { H, g, H2, g2, tt, st } = B, cost = evaluate(t, H, g, null), lam = 1e-3, it = 0, ok = Number.isFinite(cost), solved = false, rej = 0, rel = 1;
  for (; it < o.iters && ok && !(rej >= 2 && rel < 1e-2); it++) {
    H2.set(H.subarray(0, N * w));
    for (let q = 0; q < N; q++) H2[q * w] += lam * H[q * w] + 1e-9;
    if (!bandChol(H2, N, bw)) { lam *= 10; continue; }
    bandSolve(H2, N, bw, g.subarray(0, N), st);
    for (let q = 0; q < N; q++) tt[q] = Math.max(0.05, t[q] - st[q]);
    const c1 = evaluate(tt, H2, g2, null);
    if (Number.isFinite(c1) && c1 < cost) {
      t.set(tt.subarray(0, N)); [H, H2] = [H2, H]; [g, g2] = [g2, g]; B.H = H; B.H2 = H2; B.g = g; B.g2 = g2;
      rel = (cost - c1) / cost; cost = c1; lam /= 3; solved = true; rej = 0;
      if (rel < 1e-4) { it++; break; }
    } else { lam *= 10; rej++; }
  }
  return { cost, iters: it, ok: ok && Number.isFinite(cost), solved };
}

function blendMP(p, o, t) {
  const n = p.n, kap = p.kappa || 1, sMP = p.sigMP || 0.06;
  for (let i = 0; i < n; i++) for (let e = 0; e < NE; e++) {
    const k = i * NE + e;
    if (p.state[k] !== 2) continue;
    const [a, b, key] = EDGES[e], ra = (i * NLM + a) * 3, rb = (i * NLM + b) * 3, T = i * K_RAY, ta = t[T + SA[e]], tb = t[T + SB[e]];
    const dm = (p.world[rb] - p.world[ra]) * p.R[rb] + (p.world[rb + 1] - p.world[ra + 1]) * p.R[rb + 1] + (p.world[rb + 2] - p.world[ra + 2]) / kap * p.R[rb + 2];
    if (Math.sign(dm) !== p.signs[k]) continue;
    const cab = p.R[ra] * p.R[rb] + p.R[ra + 1] * p.R[rb + 1] + p.R[ra + 2] * p.R[rb + 2], del = tb - ta * cab, X = p.R[ra] * ta, Y = p.R[ra + 1] * ta, Z = p.R[ra + 2] * ta;
    const r = Math.hypot(p.R[rb + 1] * Z - p.R[rb + 2] * Y, p.R[rb + 2] * X - p.R[rb] * Z, p.R[rb] * Y - p.R[rb + 1] * X);
    const sd = deltaSigma(del, r, p.L[e], pick(p.sigPx, b) * tb / p.focal, Math.sqrt(SIG_L[key] ** 2 + (p.Lsd ? p.Lsd[e] ** 2 : 0)));
    const wv = 1 - o.mpBlend * (1 - sMP * sMP / (sMP * sMP + sd * sd)), nd = wv * del + (1 - wv) * dm, sh = nd - del;
    for (let k2 = 0; k2 < K_RAY; k2++) if (PATH[RAY_JOINTS[k2]].includes(e)) { const rk = (i * NLM + RAY_JOINTS[k2]) * 3; t[T + k2] += sh * (p.R[rk] * p.R[rb] + p.R[rk + 1] * p.R[rb + 1] + p.R[rk + 2] * p.R[rb + 2]); }
  }
}

export function solveDepth(p) {
  const o = { ...DEPTH_DEFAULTS, ...p.o }, n = p.n, K = K_RAY, W = Math.max(16, o.window), ov = Math.min(o.overlap, W >> 1);
  const init = p.init ?? chainDepths(p.R, n, p.Tz, p.L, p.signs, { sigPx: p.sigPx, focal: p.focal }).t, chi2 = new Float64Array(n);
  const wins = [];
  if (n <= W) wins.push([0, n]);
  else { for (let a = 0; a + W < n; a += W - ov) wins.push([a, a + W]); wins.push([n - W, n]); }
  const mx = Math.min(n, W), N = mx * K, w = 2 * K + 1, NL = EDGES.length;
  const B = { dX: new Float64Array(N), dY: new Float64Array(N), dZ: new Float64Array(N), sLen: new Float64Array(mx * NL), mpT: new Float64Array(mx * NE), mpOn: new Uint8Array(mx * NE),
    cab: new Float64Array(mx * NE), sT: new Float64Array(mx), accI: new Float64Array(K), H: new Float64Array(N * w), H2: new Float64Array(N * w), g: new Float64Array(N), g2: new Float64Array(N),
    tt: new Float64Array(N), st: new Float64Array(N), jT: new Float64Array(N), jOn: new Uint8Array(N), mpS: new Int8Array(mx * NE) };
  const t = new Float64Array(n * K), tw = new Float64Array(N), q = { ...p, chi2out: chi2 };
  let cost = 0, iters = 0, ok = true, prevEnd = 0;
  for (const [a, b] of wins) {
    const m = b - a;
    tw.set(init.subarray(a * K, b * K));
    const r = solveWindow(q, o, B, a, m, tw);
    cost += r.cost; iters = Math.max(iters, r.iters); ok = ok && r.ok;
    for (let li = 0; li < m; li++) {
      const i = a + li, u = i < prevEnd ? (i - a + 1) / (prevEnd - a + 1) : 1;
      for (let k = 0; k < K; k++) t[i * K + k] = u < 1 ? (1 - u) * t[i * K + k] + u * tw[li * K + k] : tw[li * K + k];
    }
    prevEnd = b;
  }
  for (let a = 0; a < n; a += W) { const m = Math.min(W, n - a); tw.set(t.subarray(a * K, (a + m) * K)); solveWindow(q, o, B, a, m, tw, true); }
  if (o.mpBlend > 0 && p.world && p.state && p.signs) blendMP(p, o, t);
  return { t, cost, iters, chi2, ok: ok && t.every(Number.isFinite) };
}

export function fillLandmarks(R, n, t, world, kappa) {
  const P = new Float64Array(n * NLM * 3), kap = kappa || 1;
  for (let i = 0; i < n; i++) {
    for (let k = 0; k < K_RAY; k++) { const r = (i * NLM + RAY_JOINTS[k]) * 3, tk = t[i * K_RAY + k]; P[r] = R[r] * tk; P[r + 1] = -R[r + 1] * tk; P[r + 2] = -R[r + 2] * tk; }
    for (let j = 0; j < NLM; j++) {
      if (SLOT[j] >= 0) continue;
      const r = (i * NLM + j) * 3, ra = (i * NLM + ANCHOR[j]) * 3, ta = t[i * K_RAY + SLOT[ANCHOR[j]]], X = R[ra] * ta, Y = R[ra + 1] * ta, Z = R[ra + 2] * ta, dx = R[r], dy = R[r + 1], dz = R[r + 2];
      const dm = world ? (world[r] - world[ra]) * dx + (world[r + 1] - world[ra + 1]) * dy + (world[r + 2] - world[ra + 2]) / kap * dz : 0, tj = Math.max(0.05, dx * X + dy * Y + dz * Z + (Number.isFinite(dm) ? dm : 0));
      P[r] = dx * tj; P[r + 1] = -dy * tj; P[r + 2] = -dz * tj;
    }
  }
  return P;
}

export function comTrack(P, n) {
  const C = new Float64Array(n * 3);
  for (let i = 0; i < n; i++) for (const [js, wt] of COM_W) for (let c = 0; c < 3; c++) {
    let s = 0;
    for (const j of js) s += P[(i * NLM + j) * 3 + c];
    C[i * 3 + c] += wt * s / js.length;
  }
  return C;
}
export function gravityScale(P, n, fps, o = {}) {
  const C = comTrack(P, n), minRun = Math.max(2, Math.ceil(0.25 * fps)), fh = o.floorTol ?? 0.03;
  let best = null;
  for (let i = 0, a = -1; i <= n; i++) {
    const up = i < n && Math.min(P[(i * NLM + 29) * 3 + 1], P[(i * NLM + 31) * 3 + 1]) > fh && Math.min(P[(i * NLM + 30) * 3 + 1], P[(i * NLM + 32) * 3 + 1]) > fh;
    if (up && a < 0) a = i;
    if (!up && a >= 0) { if (i - a >= minRun && (!best || i - a > best[1] - best[0])) best = [a, i]; a = -1; }
  }
  if (!best) return null;
  const a = best[0] + 2, b = best[1] - 2, m = b - a;
  if (m < 3) return null;
  const c0 = (a + b - 1) / 2;
  let S = [0, 0, 0, 0, 0];
  for (let i = a; i < b; i++) { const x = (i - c0) / fps; for (let p = 0; p < 5; p++) S[p] += x ** p; }
  const A = [[S[0], S[1], S[2]], [S[1], S[2], S[3]], [S[2], S[3], S[4]]], inv = inv3(A);
  if (!inv) return null;
  const acc = [0, 0, 0];
  let ss = 0;
  for (let c = 0; c < 3; c++) {
    const r = [0, 0, 0];
    for (let i = a; i < b; i++) { const x = (i - c0) / fps, y = C[i * 3 + c]; r[0] += y; r[1] += x * y; r[2] += x * x * y; }
    const co = inv.map(row => row[0] * r[0] + row[1] * r[1] + row[2] * r[2]);
    acc[c] = 2 * co[2];
    for (let i = a; i < b; i++) { const x = (i - c0) / fps; ss += (C[i * 3 + c] - co[0] - co[1] * x - co[2] * x * x) ** 2; }
  }
  const g = Math.hypot(acc[0], acc[1], acc[2]), rms = Math.sqrt(ss / (3 * m)), tilt = Math.acos(Math.max(-1, Math.min(1, -acc[1] / (g || 1)))) * 180 / Math.PI;
  const buf = new Float64Array(3 * n), sc = o.sigCom ?? jitter3((i, c) => C[i * 3 + c], n, 3, null, buf), pred = Math.max(0.002, (Number.isFinite(sc) ? sc : 0) * Math.sqrt(Math.max(0, m - 3) / m));
  const scale = 9.81 / g, sAcc = 2 * Math.sqrt(Math.max(0, inv[2][2]) * ss / Math.max(1, 3 * (m - 3)));
  return { scale, accel: g, sd: scale * sAcc / g, frames: [best[0], best[1] - 1], tilt, rms, ok: tilt < 5 && rms < 3 * pred && best[1] - best[0] >= 8 && Number.isFinite(scale) };
}
function inv3(A) {
  const [[a, b, c], [d, e, f], [g, h, k]] = A, det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
  if (!(Math.abs(det) > 1e-300)) return null;
  return [[(e * k - f * h) / det, (c * h - b * k) / det, (b * f - c * e) / det], [(f * g - d * k) / det, (a * k - c * g) / det, (c * d - a * f) / det], [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]];
}

export function mpDeltas(R, n, world, kappa) {
  const D = new Float64Array(n * NE), kap = kappa || 1;
  for (let i = 0; i < n; i++) for (let e = 0; e < NE; e++) {
    const ra = (i * NLM + EDGES[e][0]) * 3, rb = (i * NLM + EDGES[e][1]) * 3;
    D[i * NE + e] = (world[rb] - world[ra]) * R[rb] + (world[rb + 1] - world[ra + 1]) * R[rb + 1] + (world[rb + 2] - world[ra + 2]) / kap * R[rb + 2];
  }
  return D;
}
export function footContact(img, n, width, height, fps, P, floor) {
  const c = new Float32Array(n * 4), px = (i, j, k) => img[(i * NLM + j) * ST + k] * (k ? height : width);
  for (let i = 0; i < n; i++) {
    let y0 = Infinity, y1 = -Infinity;
    for (let j = 0; j < NLM; j++) { const y = px(i, j, 1); if (y < y0) y0 = y; if (y > y1) y1 = y; }
    const bh = Math.max(1, y1 - y0), a = Math.max(0, i - 2), b = Math.min(n - 1, i + 2);
    for (let q = 0; q < 4; q++) {
      const j = SOLE[q], sp = b > a ? Math.hypot(px(b, j, 0) - px(a, j, 0), px(b, j, 1) - px(a, j, 1)) * fps / (b - a) / bh : 0, r = (i * NLM + j) * 3;
      const ht = floor ? floor.normal[0] * P[r] + floor.normal[1] * P[r + 1] + floor.normal[2] * P[r + 2] - floor.offset : 0;
      c[i * 4 + q] = ht < 0.1 ? 1 / (1 + Math.exp((sp - 0.25) / 0.05)) : 0;
    }
  }
  return c;
}

export function depthPoints(series, cam, o = {}) {
  const t0 = Date.now(), opt = { ...DEPTH_DEFAULTS, ...o }, { n, width, height } = series, fps = o.fps ?? series.fps ?? 30, f = cam.focal, decide = o.decide ?? decideSigns;
  const sw = fixLegSwaps(series.img, n, width, height, estimateJitter(series.img, n, width, height, series.valid), { fps }), world = Float64Array.from(series.world);
  for (const i of sw.fixed) swapLegRows(world, i, 3);
  const sig0 = estimateJitter(sw.img, n, width, height, series.valid), R = imageRays(sw.img, n, width, height, f, { sigPx: sig0, fc2d: opt.fc2d, fps });
  const Zp = new Float64Array(n * NLM), Tr = new Float64Array(n);
  for (let i = 0; i < n; i++) Tr[i] = cam.T[i * 3 + 2];
  const Tz = lowpass(Tr, fps, 1.5);
  for (let q = 0; q < n * NLM; q++) Zp[q] = -cam.P[q * 3 + 2] + Tz[(q / NLM) | 0] - Tr[(q / NLM) | 0];
  const first = boneLengths(boneSamples(R, n, Zp, world, Float64Array.from(sig0, s => s / f)), opt), jScale = opt.jitterCal === false ? 1 : Math.min(6, Math.max(1, first.latRatio || 1));
  const sigPx = Float64Array.from(sig0, s => s * jScale), lengths = jScale > 1 ? boneLengths(boneSamples(R, n, Zp, world, Float64Array.from(sigPx, s => s / f)), opt) : first, { L, sd: Lsd, kappa, sigMP } = lengths;
  lengths.jitterScale = jScale;
  const mpD = mpDeltas(R, n, world, kappa), mpSigns = Int8Array.from(mpD, x => x < 0 ? -1 : 1), cg = { sigPx, focal: f, kGate: opt.kGate };
  let table = branchTable(R, n, chainDepths(R, n, Tz, L, mpSigns, cg).t, L, Lsd, sigPx, f, fps, opt);
  const sopt = { wMax: opt.omegaMax, kGate: opt.kGate, lambdaSolver: opt.lambdaSolver, rMoot: opt.rMoot, ...o.sign };
  const ctx = { R, n, fps, L, Lsd, table, world, kappa, sigMP, sigPx, rootZ: Tz, floor: null, contact: null, up: torsoUp(cam.P, n), img: sw.img, width, height, focal: f, valid: series.valid, swapWin: sw.window, mpDelta: mpD, mpSigns, Zp, camP: cam.P };
  let dec = decide(ctx, sopt);
  const sp = { R, n, fps, focal: f, L, Lsd, Tz, sigT: null, world, kappa, sigMP, sigPx, floor: null, o: opt };
  let sol = solveDepth({ ...sp, signs: dec.signs, state: dec.state, pWrong: dec.pWrong, init: chainDepths(R, n, Tz, L, dec.signs, cg).t }), floor = null, tableSign = table;
  for (let pass = 1; pass < opt.outer; pass++) {
    const P1 = fillLandmarks(R, n, sol.t, world, kappa), fl = floorFromPoints(P1, n, fps), contact = footContact(sw.img, n, width, height, fps, P1, fl);
    floor = { normal: fl.normal, offset: fl.offset, contact, method: fl.method };
    tableSign = branchTable(R, n, sol.t, L, Lsd, sigPx, f, fps, opt);
    const prev = dec.signs;
    dec = decide({ ...ctx, table: tableSign, floor, contact, up: fl.normal }, sopt);
    const init = Float64Array.from(sol.t), re = chainDepths(R, n, Tz, L, dec.signs, cg).t;
    for (let i = 0; i < n; i++) for (let e = 0; e < NE; e++) if (prev[i * NE + e] !== dec.signs[i * NE + e]) { init.set(re.subarray(i * K_RAY, (i + 1) * K_RAY), i * K_RAY); break; }
    sol = solveDepth({ ...sp, floor, signs: dec.signs, state: dec.state, pWrong: dec.pWrong, init });
  }
  table = branchTable(R, n, sol.t, L, Lsd, sigPx, f, fps, opt);
  return { P: fillLandmarks(R, n, sol.t, world, kappa), t: sol.t, rays: R, sigPx, lengths, kappa, sigMP, signs: dec.signs, state: dec.state, pWrong: dec.pWrong, ints: dec.ints, gates: dec.gates,
    table, tableSign, chi2: sol.chi2, ok: sol.ok, swapWin: sw.window, swapFixed: sw.fixed, Tz, floor, mpDelta: mpD, world, ms: Date.now() - t0 };
}
