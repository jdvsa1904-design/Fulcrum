import { sub, dot, cross, len, norm, mid, perp, angle, median } from './math.js';
import { lse } from './depthstats.js';

const NJ = 33, NE = 18, D = 180 / Math.PI, L2PI = Math.log(2 * Math.PI);
const TREE_AB = [[23, 24], [23, 11], [24, 12], [11, 13], [13, 15], [12, 14], [14, 16], [23, 25], [25, 27], [27, 29], [27, 31], [24, 26], [26, 28], [28, 30], [28, 32], [11, 7], [12, 8], [7, 0]];
const CLOSE_AB = [[11, 12], [23, 12], [24, 11], [29, 31], [30, 32], [7, 8], [8, 0]], SIG_CLOSE = [0.02, 0.04, 0.04, 0.012, 0.012, 0.01, 0.01];
const CLOSURES = { torso: [0, 1, 2], legL: [3], legR: [4], armL: [], armR: [], head: [5, 6] };
const FOOT = { legL: [29, 31], legR: [30, 32] }, SOLE_Q = { 29: 0, 31: 1, 30: 2, 32: 3 }; // contact columns follow depth.js SOLE = [29, 31, 30, 32]

export const CHAINS = Object.freeze({ torso: [0, 1, 2], legL: [7, 8, 9, 10], legR: [11, 12, 13, 14], armL: [3, 4], armR: [5, 6], head: [15, 16, 17] });
export const SIGN_DEFAULTS = Object.freeze({ wMax: 15, kGate: 3, piCross: 0.3, epsGate: 1e-6, minInt: 3, nu: 4, epsO: 0.03, sO: 0.4, tOn: 5, tFlip: 0.5,
  tauMp: 0.3, tauAn: 0.2, tauGeo: 0.2, tauVis: 0.3, epsLim: 1e-3, slackDeg: 5, sigFloor: 0.025, kappaTr: 3, mpFoot: 0.1, piOpen: 0.2 });
export const LIMITS = Object.freeze({ hipSag: [-30, 125], hipAbd: [-30, 50], kneeFlex: 15, kneeToe: [90, 45], ankle: [55, 155], shoulderExt: 60, elbow: 150,
  hangArm: [0.5, 20, 110], capsule: [0.3, 0.02], twist: 50, lateral: 40, flexion: [-30, 100], headYaw: 85, headPitch: [-60, 70] });
const CUES = ['mp', 'anatomy', 'geom', 'vis', 'travel', 'floor'], ORDER = ['torso', 'legL', 'legR', 'armL', 'armR', 'head'];

function erfc(x) {
  const z = Math.abs(x), t = 1 / (1 + 0.5 * z);
  const r = t * Math.exp(-z * z - 1.26551223 + t * (1.00002368 + t * (0.37409196 + t * (0.09678418 + t * (-0.18628806 + t * (0.27886807 + t * (-1.13520398 + t * (1.48851587 + t * (-0.82215223 + t * 0.17087277)))))))));
  return x >= 0 ? r : 2 - r;
}
export const Phi = x => 0.5 * erfc(-x / Math.SQRT2);
const probitLog = (x, eps) => Math.log(eps + (1 - eps) * Phi(x));
const tlog = (x, s, nu) => -((nu + 1) / 2) * Math.log1p(x * x / (nu * s * s)) - Math.log(s);
const asinD = x => Math.asin(x < -1 ? -1 : x > 1 ? 1 : x) * D;
function besselI0(x) { let s = 1, t = 1; for (let k = 1; k < 40; k++) { t *= (x / 2) * (x / 2) / (k * k); s += t; } return s; }

// --- 4.1 gates and intervals ---
export function crossingGates(tab, n, fps, o = {}) {
  const wMax = o.omegaMax ?? o.wMax ?? SIGN_DEFAULTS.wMax, open = new Uint8Array(n * NE), xpt = new Uint8Array(n * NE), regions = [];
  for (let i = 0; i + 1 < n; i++) for (let e = 0; e < NE; e++) {
    const q = i * NE + e, w = tab.wRay ? tab.wRay[q] : 0;
    open[q] = tab.phiLo[q] + tab.phiLo[q + NE] <= (wMax + (Number.isFinite(w) ? w : 0)) / fps ? 1 : 0;
  }
  for (let e = 0; e < NE; e++) for (let i = 0; i + 1 < n; i++) {
    if (!open[i * NE + e]) continue;
    let j = i, t = i, best = Infinity;
    for (; j + 1 < n && open[j * NE + e]; j++) { const s = tab.phiHat[j * NE + e] + tab.phiHat[(j + 1) * NE + e]; if (s < best) { best = s; t = j; } }
    xpt[t * NE + e] = 1;
    regions.push({ e, a: i, b: j, t });
    i = j;
  }
  regions.sort((x, y) => x.e - y.e || x.a - y.a);
  return { open, xpt, regions };
}

export function signIntervals(gates, n, o = {}) {
  const minInt = o.minInt ?? SIGN_DEFAULTS.minInt, cov = new Uint8Array(n * NE), out = [];
  for (const r of gates.regions) for (let i = r.a; i <= r.b; i++) cov[i * NE + r.e] = 1;
  for (let e = 0; e < NE; e++) for (let i = 0; i < n; i++) {
    if (cov[i * NE + e]) continue;
    let j = i;
    while (j + 1 < n && !cov[(j + 1) * NE + e]) j++;
    if (j - i + 1 >= minInt) out.push({ e, a: i, b: j });
    i = j;
  }
  return out;
}

// --- 4.2 hypotheses ---
export function chainHypotheses(R, n, L, edges, parentX, rootZ, o = {}) {
  const B = edges.length, K = 1 << B, joints = [], child = new Set(edges.map(e => TREE_AB[e][1])), sa = o.sigAng;
  for (const e of edges) for (const j of TREE_AB[e]) if (!joints.includes(j)) joints.push(j);
  const J = joints.length, slot = new Int16Array(NJ).fill(-1), X = new Float64Array(n * K * J * 3), h = new Float64Array(n * K * B), delta = new Float64Array(n * K * B), miss = new Uint8Array(n * K * B);
  const sigT = new Float64Array(n * K * J), sH2 = o.sH2, ST = new Float64Array(NJ);
  joints.forEach((j, q) => { slot[j] = q; });
  const rootMode = !parentX && edges[0] === 0, P = new Float64Array(NJ * 3);
  let rb = 0;
  const place = (c, a, Lb, s) => { // child c on its ray from parent a; returns h, or -1 for a miss
    const r = c * 3, dx = R[rb + c * 3], dy = R[rb + c * 3 + 1], dz = R[rb + c * 3 + 2], px = P[a * 3], py = P[a * 3 + 1], pz = P[a * 3 + 2];
    const t0 = dx * px + dy * py + dz * pz, cx = dy * pz - dz * py, cy = dz * px - dx * pz, cz = dx * py - dy * px, rho = Math.sqrt(cx * cx + cy * cy + cz * cz);
    const h2 = (Lb - rho) * (Lb + rho), hh = h2 > 0 ? Math.sqrt(h2) : 0, t = t0 + s * hh;
    P[r] = t * dx; P[r + 1] = t * dy; P[r + 2] = t * dz;
    if (h2 >= 0) return hh;
    const tp = Math.sqrt(px * px + py * py + pz * pz), sg = sa ? Math.hypot(sa[c] * t0, sa[a] * tp) : 1e-3 * tp;
    return (rho - Lb) > 3 * Math.max(sg, 1e-9) ? -1 : 0;
  };
  for (let i = 0; i < n; i++) {
    rb = i * NJ * 3;
    for (let k = 0; k < K; k++) {
      if (rootMode) {
        const s0 = k & 1 ? -1 : 1, dlz = R[rb + 23 * 3 + 2], Z = rootZ[i];
        let tl = Z / dlz;
        for (let it = 0; it < 3; it++) {
          for (let c = 0; c < 3; c++) P[23 * 3 + c] = tl * R[rb + 23 * 3 + c];
          place(24, 23, L[0], s0);
          tl -= ((P[23 * 3 + 2] + P[24 * 3 + 2]) / 2 - Z) / dlz;
        }
        for (let c = 0; c < 3; c++) P[23 * 3 + c] = tl * R[rb + 23 * 3 + c];
      } else for (const j of joints) if (!child.has(j)) for (let c = 0; c < 3; c++) P[j * 3 + c] = parentX[(i * NJ + j) * 3 + c];
      for (const j of joints) ST[j] = 0;
      for (let b = 0; b < B; b++) {
        const e = edges[b], [a, c] = TREE_AB[e], s = (k >> b) & 1 ? -1 : 1, hh = place(c, a, L[e], s), q = (i * K + k) * B + b, hp = Math.max(0, hh);
        h[q] = hp; delta[q] = s * hp; miss[q] = hh < 0 ? 1 : 0;
        const sy = sH2 ? Math.max(0, sH2[i * NE + e]) : 0, sd = sy > 0 ? sy / (2 * Math.sqrt(hp * hp + sy / 2)) : 0;
        ST[c] = Math.sqrt(ST[a] * ST[a] + sd * sd);
      }
      const xb = (i * K + k) * J * 3;
      for (let q = 0; q < J; q++) { sigT[(i * K + k) * J + q] = ST[joints[q]]; for (let c = 0; c < 3; c++) X[xb + q * 3 + c] = P[joints[q] * 3 + c]; }
    }
  }
  return { K, B, J, X, joints: Int16Array.from(joints), slot, edges: edges.slice(), h, delta, miss, sigT };
}

const getter = (H, i, k, parentX) => {
  const xb = (i * H.K + k) * H.J * 3, pb = i * NJ * 3;
  return j => { const q = H.slot[j]; return q >= 0 ? [H.X[xb + q * 3], H.X[xb + q * 3 + 1], H.X[xb + q * 3 + 2]] : [parentX[pb + j * 3], parentX[pb + j * 3 + 1], parentX[pb + j * 3 + 2]]; };
};

const sigAt = (H, i, k, j) => { const q = H.slot[j]; return q >= 0 ? H.sigT[(i * H.K + k) * H.J + q] : 0; };
const sigMax = (H, i, k) => { let m = 0; for (let q = 0; q < H.J; q++) m = Math.max(m, H.sigT[(i * H.K + k) * H.J + q]); return m; };

// --- anatomy (frames: left = lHip - rHip, up = shMid - hipMid, fwd = left x up) ---
export function torsoFrame(get) {
  const lh = get(23), rh = get(24), ls = get(11), rs = get(12), hipMid = mid(lh, rh), shMid = mid(ls, rs), left = norm(sub(lh, rh)), up = norm(sub(shMid, hipMid));
  return { left, up, fwd: norm(cross(left, up)), hipMid, shMid };
}

function segDist(p, a, b) {
  const ab = sub(b, a), l2 = dot(ab, ab), t = l2 > 0 ? Math.min(1, Math.max(0, dot(sub(p, a), ab) / l2)) : 0;
  return len(sub(p, [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t]));
}

export function chainLimits(name, get, upW = null, shW = 0.34, frame = null) {
  const fr = frame ?? torsoFrame(get), { left, up, fwd } = fr, dn = [-up[0], -up[1], -up[2]];
  if (name === 'legL' || name === 'legR') {
    const [hip, knee, ank, heel, toe] = (name === 'legL' ? [23, 25, 27, 29, 31] : [24, 26, 28, 30, 32]).map(get), out = name === 'legL' ? left : [-left[0], -left[1], -left[2]];
    const th = norm(sub(knee, hip)), sh = norm(sub(ank, knee)), ft = norm(sub(toe, heel));
    const sag = Math.atan2(dot(th, fwd), dot(th, dn)) * D, abd = asinD(dot(th, out)), ka = angle(th, sh) * D, flex = Math.atan2(dot(cross(th, sh), left), dot(th, sh)) * D;
    const kneeToe = ka > LIMITS.kneeFlex ? -LIMITS.kneeToe[0] * dot(norm(perp(sub(knee, mid(hip, ank)), norm(sub(ank, hip)))), ft) - LIMITS.kneeToe[1] : -LIMITS.kneeToe[0] - LIMITS.kneeToe[1];
    const ankA = angle(sub(knee, ank), sub(toe, ank)) * D;
    return [Math.max(LIMITS.hipSag[0] - sag, sag - LIMITS.hipSag[1], LIMITS.hipAbd[0] - abd, abd - LIMITS.hipAbd[1], -flex - LIMITS.kneeFlex, kneeToe, LIMITS.ankle[0] - ankA, ankA - LIMITS.ankle[1]), Infinity];
  }
  if (name === 'armL' || name === 'armR') {
    const [sh, elb, wri] = (name === 'armL' ? [11, 13, 15] : [12, 14, 16]).map(get), ua = norm(sub(elb, sh)), fa = norm(sub(wri, elb)), ea = angle(ua, fa) * D;
    let v = Math.max(asinD(-dot(ua, fwd)) - LIMITS.shoulderExt, ea - LIMITS.elbow);
    if (dot(ua, dn) > LIMITS.hangArm[0] && ea > LIMITS.hangArm[1]) v = Math.max(v, angle(perp(fa, ua), perp(fwd, ua)) * D - LIMITS.hangArm[2]);
    const r = LIMITS.capsule[0] * shW, cap = Math.min(segDist(elb, fr.hipMid, fr.shMid), segDist(wri, fr.hipMid, fr.shMid)) - r;
    return [v, cap];
  }
  if (name === 'head') {
    const ls = get(11), rs = get(12), cl = norm(sub(ls, rs)), cf = norm(cross(cl, up)), nd = norm(sub(get(0), mid(get(7), get(8))));
    const hz = perp(nd, up), yaw = len(hz) > 0.2 ? Math.abs(Math.atan2(dot(nd, cl), dot(nd, cf))) * D : 0, pitch = asinD(dot(nd, up));
    return [Math.max(yaw - LIMITS.headYaw, LIMITS.headPitch[0] - pitch, pitch - LIMITS.headPitch[1]), Infinity];
  }
  const hl = norm(perp(sub(get(23), get(24)), up)), sl = norm(perp(sub(get(11), get(12)), up));
  let v = angle(hl, sl) * D - LIMITS.twist;
  if (upW) {
    const lh = norm(perp(sub(get(23), get(24)), upW)), fh = norm(cross(lh, upW)), lat = asinD(dot(up, lh)), flex = Math.atan2(dot(up, fh), dot(up, upW)) * D;
    v = Math.max(v, Math.abs(lat) - LIMITS.lateral, LIMITS.flexion[0] - flex, flex - LIMITS.flexion[1]);
  }
  return [v, Infinity];
}

export function limitViolations(P, up = null, shW = null) {
  const get = j => [P[j * 3], P[j * 3 + 1], P[j * 3 + 2]], w = shW ?? len(sub(get(11), get(12))), out = {};
  for (const name of ORDER) { const [v, cap] = chainLimits(name, get, up, w); out[name] = v; if (Number.isFinite(cap)) out[name + 'Capsule'] = cap; }
  return out;
}

const camUp = ctx => { const u = ctx.floor?.normal ?? ctx.up; return u ? norm([u[0], -u[1], -u[2]]) : null; };
const visOf = ctx => ctx.vis ? (i, j) => ctx.vis[i * NJ + j] : ctx.img ? (i, j) => ctx.img[(i * NJ + j) * 5 + 3] : null;
const focalOf = ctx => ctx.focal ?? 1000;
const kids = H => H.edges.map(e => TREE_AB[e][1]);
function sigPerp(ctx, H, i) { let s = 0; for (const j of H.joints) s = Math.max(s, ctx.sigPx ? ctx.sigPx[j] : 1); return s * (ctx.rootZ ? ctx.rootZ[i] : 5) / focalOf(ctx); }

// --- 4.3 cues: Float64Array(n*K) (cueMP: n*K*2), per-frame log-likelihoods already divided by tau in frames ---
const FOOT_E = new Set([9, 10, 13, 14]); // heel, toe: tracker foot errors are per joint, not chain flips
export function cueMP(H, ctx, o = {}) {
  const O = { ...SIGN_DEFAULTS, ...o }, { n, R, world, table: tab } = ctx, K = H.K, B = H.B, out = new Float64Array(n * K * 2), tau = Math.max(1, O.tauMp * ctx.fps);
  const kap = ctx.kappa || 1, smp = ctx.sigMP ?? 0.05, l1 = Math.log(1 - O.epsO), l0 = Math.log(O.epsO);
  if (!world) return out;
  for (let i = 0; i < n; i++) for (let b = 0; b < B; b++) {
    const e = H.edges[b], [a, c] = TREE_AB[e], rb = (i * NJ + c) * 3, wa = (i * NJ + a) * 3, wc = (i * NJ + c) * 3;
    const dmp = (world[wc] - world[wa]) * R[rb] + (world[wc + 1] - world[wa + 1]) * R[rb + 1] + (world[wc + 2] - world[wa + 2]) / kap * R[rb + 2];
    if (!Number.isFinite(dmp)) continue;
    const sy = tab && tab.sH2 ? Math.max(0, tab.sH2[i * NE + e]) : 0, lo = l0 + tlog(dmp, O.sO, O.nu), wf = FOOT_E.has(e) ? O.mpFoot : 1;
    for (let k = 0; k < K; k++) {
      const dk = H.delta[(i * K + k) * B + b], sd = sy > 0 ? sy / (2 * Math.sqrt(dk * dk + sy / 2)) : 0, s = Math.sqrt(smp * smp + sd * sd);
      out[(i * K + k) * 2] += wf * lse(l1 + tlog(dmp - dk, s, O.nu), lo) / tau;
      out[(i * K + k) * 2 + 1] += wf * lse(l1 + tlog(dmp + dk, s, O.nu), lo) / tau;
    }
  }
  return out;
}

export function cueAnatomy(H, ctx, name, parentX, o = {}) {
  const O = { ...SIGN_DEFAULTS, ...o }, { n } = ctx, K = H.K, out = new Float64Array(n * K), tau = Math.max(1, O.tauAn * ctx.fps), upW = camUp(ctx);
  const Lmin = Math.max(0.02, Math.min(...H.edges.map(e => ctx.L[e]))), shW = ctx.L[18] || 0.34;
  for (let i = 0; i < n; i++) {
    const sl = D * Math.SQRT2 * sigPerp(ctx, H, i) / Lmin, fr = name === 'torso' ? null : torsoFrame(getter(H, i, 0, parentX));
    for (let k = 0; k < K; k++) {
      const st = sigMax(H, i, k), sd = D * Math.min(1, st / Lmin), sv = Math.sqrt(sl * sl + O.slackDeg * O.slackDeg + sd * sd), [v, cap] = chainLimits(name, getter(H, i, k, parentX), upW, shW, fr);
      let ll = Number.isFinite(v) ? probitLog(-v / sv, O.epsLim) : 0;
      if (Number.isFinite(cap)) ll += probitLog(cap / Math.hypot(LIMITS.capsule[1], st), O.epsLim);
      out[i * K + k] = ll / tau;
    }
  }
  return out;
}

export function cueGeom(H, ctx, name, parentX, o = {}) {
  const O = { ...SIGN_DEFAULTS, ...o }, { n } = ctx, K = H.K, out = new Float64Array(n * K), tau = Math.max(1, O.tauGeo * ctx.fps), cl = CLOSURES[name] ?? [];
  if (!cl.length) return out;
  for (let i = 0; i < n; i++) {
    const sp = sigPerp(ctx, H, i);
    for (let k = 0; k < K; k++) {
      const get = getter(H, i, k, parentX); let ll = 0;
      for (const c of cl) {
        const e = 18 + c, [a, b] = CLOSE_AB[c], sd = ctx.Lsd ? ctx.Lsd[e] : 0, pa = get(a), pb = get(b), v = sub(pb, pa), l = len(v) || 1e-9, ra = (i * NJ + a) * 3, rb = (i * NJ + b) * 3;
        const ua = sigAt(H, i, k, a) * (v[0] * ctx.R[ra] + v[1] * ctx.R[ra + 1] + v[2] * ctx.R[ra + 2]) / l, ub = sigAt(H, i, k, b) * (v[0] * ctx.R[rb] + v[1] * ctx.R[rb + 1] + v[2] * ctx.R[rb + 2]) / l;
        const s2 = SIG_CLOSE[c] ** 2 + sd * sd + 2 * sp * sp + ua * ua + ub * ub, r = l - ctx.L[e];
        ll -= 0.5 * r * r / s2;
      }
      out[i * K + k] = ll / tau;
    }
  }
  return out;
}

const img2 = (R, i, j) => { const b = (i * NJ + j) * 3; return [R[b] / R[b + 2], R[b + 1] / R[b + 2]]; };
function inPoly(p, poly) {
  let c = false;
  for (let a = 0, b = poly.length - 1; a < poly.length; b = a++) {
    const [xa, ya] = poly[a], [xb, yb] = poly[b];
    if ((ya > p[1]) !== (yb > p[1]) && p[0] < (xb - xa) * (p[1] - ya) / (yb - ya) + xa) c = !c;
  }
  return c;
}

export function cueVis(H, ctx, name, parentX, o = {}) {
  const O = { ...SIGN_DEFAULTS, ...o }, { n, R } = ctx, K = H.K, out = new Float64Array(n * K), tau = Math.max(1, O.tauVis * ctx.fps), vis = visOf(ctx);
  if (!vis) return out;
  if (name === 'torso') {
    for (let i = 0; i < n; i++) {
      if (!(vis(i, 0) >= 0.6 && vis(i, 2) >= 0.6 && vis(i, 5) >= 0.6)) continue;
      for (let k = 0; k < K; k++) { const fr = torsoFrame(getter(H, i, k, parentX)), ray = norm(fr.shMid); out[i * K + k] = probitLog(-dot(fr.fwd, ray) / 0.1, O.epsO) / tau; }
    }
    return out;
  }
  const kj = kids(H);
  for (let i = 0; i < n; i++) {
    const poly = [11, 12, 24, 23].map(j => img2(R, i, j)), hid = kj.filter(j => vis(i, j) < 0.5 && inPoly(img2(R, i, j), poly));
    if (!hid.length) continue;
    const g = j => [parentX[(i * NJ + j) * 3], parentX[(i * NJ + j) * 3 + 1], parentX[(i * NJ + j) * 3 + 2]], fr = torsoFrame(g), nrm = fr.fwd;
    const c = [0, 1, 2].map(q => (g(11)[q] + g(12)[q] + g(23)[q] + g(24)[q]) / 4), nc = dot(nrm, c);
    for (const j of hid) {
      const rb = (i * NJ + j) * 3, nd = nrm[0] * R[rb] + nrm[1] * R[rb + 1] + nrm[2] * R[rb + 2];
      if (Math.abs(nd) < 0.1) continue;
      const tp = nc / nd, q = H.slot[j];
      for (let k = 0; k < K; k++) { const xb = ((i * K + k) * H.J + q) * 3, t = Math.hypot(H.X[xb], H.X[xb + 1], H.X[xb + 2]); out[i * K + k] += probitLog((t - tp) / Math.hypot(0.03, H.sigT[(i * K + k) * H.J + q]), O.epsO) / tau; }
    }
  }
  return out;
}

function runTau(on, n) { // tau = max(1, run length / 2) of the constant-label run containing i
  const tau = new Float64Array(n);
  for (let i = 0; i < n;) { let j = i; while (j + 1 < n && on[j + 1] === on[i]) j++; for (let q = i; q <= j; q++) tau[q] = Math.max(1, (j - i + 1) / 2); i = j + 1; }
  return tau;
}

export function stanceProb(ctx, name) {
  const { n, R, fps } = ctx, [heel, toe] = FOOT[name], p = new Float64Array(n), bh = new Float64Array(n), fp = new Float64Array(n * 2);
  for (let i = 0; i < n; i++) {
    const a = img2(R, i, heel), b = img2(R, i, toe), nz = img2(R, i, 0), la = img2(R, i, 27), ra = img2(R, i, 28);
    fp[i * 2] = (a[0] + b[0]) / 2; fp[i * 2 + 1] = (a[1] + b[1]) / 2; bh[i] = Math.hypot(nz[0] - (la[0] + ra[0]) / 2, nz[1] - (la[1] + ra[1]) / 2) * 1.15;
  }
  const w = Math.max(1, Math.round(fps / 2));
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1), H = median(Array.from(bh.subarray(Math.max(0, i - w), Math.min(n, i + w + 1)))) || 1;
    const sp = b > a ? Math.hypot(fp[b * 2] - fp[a * 2], fp[b * 2 + 1] - fp[a * 2 + 1]) * fps / (b - a) : 0;
    p[i] = 1 / (1 + Math.exp((sp / H - 0.25) / 0.05));
  }
  return p;
}

export function cueTravel(H, ctx, name, parentX, o = {}) {
  const O = { ...SIGN_DEFAULTS, ...o }, { n, fps } = ctx, K = H.K, out = new Float64Array(n * K), upW = camUp(ctx);
  if (!FOOT[name] || !upW || !parentX || n < 3) return out;
  const pst = stanceProb(ctx, name), tau = runTau(Uint8Array.from(pst, x => x > 0.5 ? 1 : 0), n), w = Math.max(1, Math.round(fps / 6)), [heel, toe] = FOOT[name];
  const hipAt = i => { const b = i * NJ * 3; return [0, 1, 2].map(c => (parentX[b + 23 * 3 + c] + parentX[b + 24 * 3 + c]) / 2); };
  for (let i = 0; i < n; i++) {
    if (pst[i] < 0.1) continue;
    const a = Math.max(0, i - w), b = Math.min(n - 1, i + w), hv = perp(sub(hipAt(b), hipAt(a)), upW), sp = len(hv) * fps / Math.max(1, b - a);
    if (!(sp > 0.4)) continue;
    const dir = norm(hv);
    for (let k = 0; k < K; k++) {
      const get = getter(H, i, k, parentX), fh = perp(sub(get(toe), get(heel)), upW), fl = len(fh);
      if (fl < 1e-6) continue;
      const sth = Math.hypot(sigAt(H, i, k, toe), sigAt(H, i, k, heel)) / Math.max(fl, 0.05), kt = 1 / (1 / O.kappaTr + sth * sth), lc = -L2PI - Math.log(besselI0(kt));
      out[i * K + k] = pst[i] * Math.log(0.9 * Math.exp(lc + kt * dot(norm(fh), dir)) + 0.1 / (2 * Math.PI)) / tau[i];
    }
  }
  return out;
}

export function cueFloor(H, ctx, name, parentX, o = {}) {
  const O = { ...SIGN_DEFAULTS, ...o }, { n } = ctx, K = H.K, out = new Float64Array(n * K), fl = ctx.floor;
  if (!FOOT[name] || !fl || !fl.normal) return out;
  const [heel, toe] = FOOT[name], nn = fl.normal, off = fl.offset ?? 0, f = focalOf(ctx), U = Math.log(1 / 0.85);
  const pc = new Float64Array(n), st = ctx.contact ? null : stanceProb(ctx, name);
  for (let i = 0; i < n; i++) pc[i] = ctx.contact ? Math.min(1, Math.max(0, Math.max(ctx.contact[i * 4 + SOLE_Q[heel]], ctx.contact[i * 4 + SOLE_Q[toe]]))) : st[i];
  const tau = runTau(Uint8Array.from(pc, x => x > 0.5 ? 1 : 0), n), ht = p => nn[0] * p[0] - nn[1] * p[1] - nn[2] * p[2] - off;
  const sp = Math.max(ctx.sigPx ? ctx.sigPx[heel] : 1, ctx.sigPx ? ctx.sigPx[toe] : 1);
  for (let i = 0; i < n; i++) for (let k = 0; k < K; k++) {
    const get = getter(H, i, k, parentX), a = get(heel), b = get(toe), lo = ht(a) <= ht(b), h = lo ? ht(a) : ht(b), Z = Math.max(a[2], b[2]), st = sigAt(H, i, k, lo ? heel : toe), sf2 = O.sigFloor ** 2 + (Z * sp / f) ** 2 + st * st;
    const g = Math.log(Math.max(pc[i], 1e-12)) - 0.5 * h * h / sf2 - 0.5 * Math.log(2 * Math.PI * sf2), u = h >= -0.05 && h <= 0.8 ? Math.log(Math.max(1 - pc[i], 1e-12)) + U : -Infinity;
    out[i * K + k] = lse(lse(g, u), Math.log(1e-6)) / tau[i];
  }
  return out;
}

// --- 4.4 HMM ---
export function chainTransitions(gates, edges, n, fps, o = {}) {
  const O = { ...SIGN_DEFAULTS, ...o }, B = edges.length, K = 1 << B, K2 = K * 2, qOn = Math.min(0.5, 1 / (O.tOn * fps)), qOff = Math.min(0.5, 1 / (O.tFlip * fps));
  const M = [[Math.log1p(-qOn), Math.log(qOn)], [Math.log(qOff), Math.log1p(-qOff)]];
  const build = pf => {
    const T = new Float64Array(K2 * K2), lf = pf.map(p => Math.log(p)), ls = pf.map(p => Math.log1p(-p));
    for (let a = 0; a < K2; a++) for (let b = 0; b < K2; b++) {
      let l = M[a & 1][b & 1]; const x = (a >> 1) ^ (b >> 1);
      for (let q = 0; q < B; q++) l += (x >> q) & 1 ? lf[q] : ls[q];
      T[a * K2 + b] = l;
    }
    return T;
  };
  const base = build(edges.map(() => O.epsGate)), at = new Map(), fx = new Uint8Array(Math.max(0, n - 1) * B);
  for (let i = 0; i + 1 < n; i++) {
    let any = false; const pf = edges.map((e, q) => { const x = gates.xpt[i * NE + e] ? 1 : O.piOpen > 0 && gates.open[i * NE + e] ? 2 : 0; fx[i * B + q] = x; any ||= !!x; return x === 1 ? O.piCross : x === 2 ? O.piOpen : O.epsGate; });
    if (any) at.set(i, build(pf));
  }
  const lp = [[Math.log1p(-O.epsGate), Math.log(O.epsGate)], [Math.log1p(-O.piCross), Math.log(O.piCross)], [Math.log1p(-O.piOpen), Math.log(O.piOpen || 1e-300)]];
  return { base, at, K: K2, fact: { B, x: fx, lp, M } };
}

const trAt = (tr, i) => (tr.at && tr.at.get(i)) || tr.base;
const initOf = (tr, K) => tr.init ?? new Float64Array(K).fill(-Math.log(K));

function fbFact(em, n, K, tr) { // same result as the dense path; transitions applied one independent bit at a time
  const { B, x, lp, M } = tr.fact, la = new Float64Array(n * K), lb = new Float64Array(n * K), post = new Float64Array(n * K), init = initOf(tr, K), v = new Float64Array(K), w = new Float64Array(K);
  const norm1 = (A, o) => { let z = -Infinity; for (let k = 0; k < K; k++) z = lse(z, A[o + k]); if (z > -Infinity) for (let k = 0; k < K; k++) A[o + k] -= z; return z; };
  const apply = (i, back) => {
    for (let q = 0; q < B; q++) {
      const [ls, lf] = lp[x[i * B + q]], bit = 2 << q;
      for (let s = 0; s < K; s++) w[s] = lse(v[s] + ls, v[s ^ bit] + lf);
      v.set(w);
    }
    for (let s = 0; s < K; s++) { const m = s & 1, z = s & ~1; w[s] = back ? lse(v[z] + M[m][0], v[z | 1] + M[m][1]) : lse(v[z] + M[0][m], v[z | 1] + M[1][m]); }
    v.set(w);
  };
  let logZ = 0;
  for (let k = 0; k < K; k++) la[k] = init[k] + em[k];
  logZ += norm1(la, 0);
  for (let i = 1; i < n; i++) {
    for (let k = 0; k < K; k++) v[k] = la[(i - 1) * K + k];
    apply(i - 1, false);
    for (let k = 0; k < K; k++) la[i * K + k] = v[k] + em[i * K + k];
    logZ += norm1(la, i * K);
  }
  for (let i = n - 2; i >= 0; i--) {
    for (let k = 0; k < K; k++) v[k] = em[(i + 1) * K + k] + lb[(i + 1) * K + k];
    apply(i, true);
    for (let k = 0; k < K; k++) lb[i * K + k] = v[k];
    norm1(lb, i * K);
  }
  for (let i = 0; i < n; i++) {
    let z = -Infinity;
    for (let k = 0; k < K; k++) z = lse(z, la[i * K + k] + lb[i * K + k]);
    for (let k = 0; k < K; k++) post[i * K + k] = z === -Infinity ? 1 / K : Math.exp(la[i * K + k] + lb[i * K + k] - z);
  }
  return { post, logZ };
}

export function forwardBackward(em, n, K, tr) {
  if (tr.fact && n > 0 && K === 2 << tr.fact.B && !tr.init) return fbFact(em, n, K, tr);
  const la = new Float64Array(n * K), lb = new Float64Array(n * K), post = new Float64Array(n * K), init = initOf(tr, K), nb = new Float64Array(K);
  if (!n) return { post, logZ: 0 };
  let logZ = 0;
  const norm1 = (A, o) => { let z = -Infinity; for (let k = 0; k < K; k++) z = lse(z, A[o + k]); if (z > -Infinity) for (let k = 0; k < K; k++) A[o + k] -= z; return z; };
  for (let k = 0; k < K; k++) la[k] = init[k] + em[k];
  logZ += norm1(la, 0);
  for (let i = 1; i < n; i++) {
    const T = trAt(tr, i - 1), p = (i - 1) * K;
    for (let b = 0; b < K; b++) {
      let m = -Infinity;
      for (let a = 0; a < K; a++) { const v = la[p + a] + T[a * K + b]; if (v > m) m = v; }
      if (m === -Infinity) { la[i * K + b] = -Infinity; continue; }
      let s = 0;
      for (let a = 0; a < K; a++) s += Math.exp(la[p + a] + T[a * K + b] - m);
      la[i * K + b] = m + Math.log(s) + em[i * K + b];
    }
    logZ += norm1(la, i * K);
  }
  for (let i = n - 2; i >= 0; i--) {
    const T = trAt(tr, i), q = (i + 1) * K;
    for (let b = 0; b < K; b++) nb[b] = em[q + b] + lb[q + b];
    for (let a = 0; a < K; a++) {
      let m = -Infinity;
      for (let b = 0; b < K; b++) { const v = T[a * K + b] + nb[b]; if (v > m) m = v; }
      if (m === -Infinity) { lb[i * K + a] = -Infinity; continue; }
      let s = 0;
      for (let b = 0; b < K; b++) s += Math.exp(T[a * K + b] + nb[b] - m);
      lb[i * K + a] = m + Math.log(s);
    }
    norm1(lb, i * K);
  }
  for (let i = 0; i < n; i++) {
    let z = -Infinity;
    for (let k = 0; k < K; k++) z = lse(z, la[i * K + k] + lb[i * K + k]);
    for (let k = 0; k < K; k++) post[i * K + k] = z === -Infinity ? 1 / K : Math.exp(la[i * K + k] + lb[i * K + k] - z);
  }
  return { post, logZ };
}

export function viterbi(em, n, K, tr) {
  const out = new Uint8Array(n), init = initOf(tr, K);
  if (!n) return out;
  const bp = new Uint8Array(n * K);
  let d = new Float64Array(K), nd = new Float64Array(K);
  for (let k = 0; k < K; k++) d[k] = init[k] + em[k];
  for (let i = 1; i < n; i++) {
    const T = trAt(tr, i - 1);
    for (let b = 0; b < K; b++) {
      let m = -Infinity, arg = 0;
      for (let a = 0; a < K; a++) { const v = d[a] + T[a * K + b]; if (v > m) { m = v; arg = a; } }
      nd[b] = m + em[i * K + b]; bp[i * K + b] = arg;
    }
    [d, nd] = [nd, d];
  }
  let best = 0;
  for (let k = 1; k < K; k++) if (d[k] > d[best]) best = k;
  out[n - 1] = best;
  for (let i = n - 1; i > 0; i--) out[i - 1] = bp[i * K + out[i]];
  return out;
}

export function chainEmissions(H, ctx, name, parentX, o = {}) {
  const { n } = ctx, K = H.K, K2 = K * 2, use = new Set(o.cues ?? CUES), em = (use.has('mp') && ctx.world) ? cueMP(H, ctx, o) : new Float64Array(n * K2);
  const parts = [];
  if (use.has('anatomy')) parts.push(cueAnatomy(H, ctx, name, parentX, o));
  if (use.has('geom')) parts.push(cueGeom(H, ctx, name, parentX, o));
  if (use.has('vis')) parts.push(cueVis(H, ctx, name, parentX, o));
  if (use.has('travel') && name.startsWith('leg')) parts.push(cueTravel(H, ctx, name, parentX, o));
  if (use.has('floor') && name.startsWith('leg') && ctx.floor) parts.push(cueFloor(H, ctx, name, parentX, o));
  for (const p of parts) for (let i = 0; i < n * K; i++) { em[i * 2] += p[i]; em[i * 2 + 1] += p[i]; }
  for (let i = 0; i < n; i++) {
    let m = -Infinity;
    for (let s = 0; s < K2; s++) { const q = i * K2 + s; if (Number.isNaN(em[q])) em[q] = -Infinity; if (em[q] > m) m = em[q]; }
    if (m === -Infinity) for (let s = 0; s < K2; s++) em[i * K2 + s] = 0;
  }
  return em;
}

export function decideSigns(ctx, o = {}) {
  const O = { ...SIGN_DEFAULTS, lambdaSolver: 6.9, rMoot: 0.02, ...o }, { R, n, fps, L, table: tab } = ctx;
  const signs = new Int8Array(n * NE), state = new Uint8Array(n * NE), pWrong = new Float32Array(n * NE), ints = [], chains = {};
  const gates = crossingGates(tab, n, fps, O), spans = signIntervals(gates, n, O), PX = new Float64Array(n * NJ * 3), f = focalOf(ctx);
  const sigAng = new Float64Array(NJ).fill(1 / f);
  if (ctx.sigPx) for (let j = 0; j < NJ; j++) sigAng[j] = ctx.sigPx[j] / f;
  const rootZ = ctx.rootZ ?? new Float64Array(n).fill(5);
  for (const name of ORDER) {
    const edges = CHAINS[name], B = edges.length, H = chainHypotheses(R, n, L, edges, name === 'torso' ? null : PX, rootZ, { sigAng, sH2: tab.sH2 }), K = H.K, K2 = K * 2;
    const em = chainEmissions(H, { ...ctx, rootZ }, name, PX, O), tr = chainTransitions(gates, edges, n, fps, O), { post, logZ } = forwardBackward(em, n, K2, tr), vit = viterbi(em, n, K2, tr);
    const g1 = new Float64Array(n * B), g0 = new Float64Array(n * B);
    for (let i = 0; i < n; i++) for (let s = 0; s < K2; s++) { const p = post[i * K2 + s], k = s >> 1; for (let b = 0; b < B; b++) if ((k >> b) & 1) g1[i * B + b] += p; else g0[i * B + b] += p; }
    edges.forEach((e, b) => {
      for (let i = 0; i < n; i++) { const bit = (vit[i] >> 1 >> b) & 1; signs[i * NE + e] = bit ? -1 : 1; pWrong[i * NE + e] = bit ? g0[i * B + b] : g1[i * B + b]; }
      for (const I of spans) {
        if (I.e !== e) continue;
        let c1 = 0, s1 = 0, s0 = 0, hm = 0;
        for (let i = I.a; i <= I.b; i++) { c1 += (vit[i] >> 1 >> b) & 1; s1 += g1[i * B + b]; s0 += g0[i * B + b]; if (tab.hHi) hm = Math.max(hm, tab.hHi[i * NE + e]); }
        const m = I.b - I.a + 1, bit = 2 * c1 > m || (2 * c1 === m && s1 > s0) ? 1 : 0, sign = bit ? -1 : 1;
        const S = Math.max(-30, Math.min(30, bit ? Math.log(s1) - Math.log(s0) : Math.log(s0) - Math.log(s1)));
        for (let i = I.a; i <= I.b; i++) { const q = i * NE + e; signs[q] = sign; pWrong[q] = bit ? g0[i * B + b] : g1[i * B + b]; state[q] = S >= O.lambdaSolver ? 2 : 1; }
        ints.push({ e, a: I.a, b: I.b, sign, S, moot: 2 * hm <= O.rMoot });
      }
    });
    for (let i = 0; i < n; i++) {
      let k = 0;
      edges.forEach((e, b) => { if (signs[i * NE + e] < 0) k |= 1 << b; });
      const xb = (i * K + k) * H.J * 3;
      for (let q = 0; q < H.J; q++) for (let c = 0; c < 3; c++) PX[(i * NJ + H.joints[q]) * 3 + c] = H.X[xb + q * 3 + c];
    }
    if (O.debug) chains[name] = { H, em, post, vit, logZ };
  }
  ints.sort((x, y) => x.e - y.e || x.a - y.a);
  const res = { signs, state, pWrong, ints, gates, X: PX };
  if (O.debug) res.chains = chains;
  return res;
}
