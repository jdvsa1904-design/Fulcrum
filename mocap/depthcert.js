import { NLM, landmarksFromPose } from './skeleton.js';
import { takeWorld, worldAt } from './ground.js';
import { qrot, qconj, median, percentile } from './math.js';
import { TREE, EDGES, NE, PATH, LENGTH_CLASS, gravityScale, footContact, swapLegRows } from './depth.js';
import { conformalRank } from './depthstats.js';
import { DEPTH_CAL } from './depth_calibration.js';

const ST = 5;
export const EVAL_JOINTS = [0, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32];
export const CERT_GROUPS = { torso: [11, 12, 23, 24], head: [0], elbows: [13, 14], wrists: [15, 16], knees: [25, 26], feet: [27, 28, 29, 30, 31, 32] };
export const STATE = { GREY: 0, GREEN: 1, AMBER: 2, RED: 3 };
export const REASONS = ['', 'FRONT_BACK', 'EDGE_ON', 'RADIUS', 'HIDDEN', 'GAP', 'SWAP', 'IMAGE', 'LENGTH', 'OUT', 'DOMAIN', 'UNCALIBRATED'];
export const DOMAIN_STATS = [ // [name, side: 1 upper, -1 lower, 2 two-sided, plain-English reason]
  ['lenCV', 2, 'tracker bone lengths behave unusually'], ['chi2', 1, 'the skeleton does not fit the video well'], ['mpRho', 2, 'tracker depth behaves unusually'],
  ['kappa', 2, 'tracker depth is unusually flat or deep'], ['missRate', 1, 'bones too short for the image'], ['vis', -1, 'limbs mostly hidden'],
  ['jitter', 1, 'tracking is very shaky'], ['crossFrac', 1, 'limbs stay edge-on to the camera'], ['mpDisagree', 1, 'tracker depth often contradicts the geometry'],
  ['footDrift', 1, 'planted feet slide in depth'], ['gravity', 1, 'the jump does not match gravity (check fps or height)']]; // 14 tails
export const DEPTH_FAILED = 'the depth solve did not converge', TOO_SHORT = 'the take is too short to check', MIN_FRAMES = 30;

const RS = Object.fromEntries(REASONS.map((r, k) => [r, k])), GROUPS = Object.keys(CERT_GROUPS);
const GROUP_OF = Int8Array.from({ length: NLM }, (_, j) => GROUPS.findIndex(g => CERT_GROUPS[g].includes(j)));
const LONG_EDGES = TREE.map((x, e) => LENGTH_CLASS.long.includes(x[2]) ? e : -1).filter(e => e >= 0);
const VIS_JOINTS = [13, 14, 15, 16, 25, 26, 27, 28, 29, 30, 31, 32];

export function toCameraFrame(camera, w) { const s = camera.scale || 1, p = qrot(qconj(camera.q), [w[0] / s, w[1] / s + camera.offset, w[2] / s]); return [s * p[0], -s * p[1], -s * p[2]]; }

export function finalLandmarks(skel, take, n = take.n) {
  const TW = takeWorld(skel, take), out = new Float64Array(n * NLM * 3);
  for (let i = 0; i < n; i++) out.set(landmarksFromPose(skel, worldAt(TW, i)), i * NLM * 3);
  return out;
}

export function cameraLandmarks(camera, W, n) {
  const out = new Float64Array(n * NLM * 3);
  for (let q = 0; q < n * NLM; q++) out.set(toCameraFrame(camera, [W[q * 3], W[q * 3 + 1], W[q * 3 + 2]]), q * 3);
  return out;
}

export const R_EDGE = 0.06;
export function boneFlags(depth, n, lamS = DEPTH_CAL.sign.lambda, rMoot = DEPTH_CAL.rMoot, rEdge = DEPTH_CAL.rEdge ?? R_EDGE) {
  const N = n * NE, moot = new Uint8Array(N), cert = new Uint8Array(N), region = new Uint8Array(N), inI = new Uint8Array(N), code = new Uint8Array(N), h = depth.table.h;
  for (const r of depth.gates?.regions ?? []) for (let i = r.a; i <= Math.min(r.b, n - 1); i++) region[i * NE + r.e] = 1;
  for (const I of depth.ints ?? []) {
    const c = !I.moot && I.S >= lamS ? 1 : 0;
    for (let i = I.a; i <= Math.min(I.b, n - 1); i++) { const q = i * NE + I.e; inI[q] = 1; moot[q] = I.moot ? 1 : 0; cert[q] = c; }
  }
  for (let q = 0; q < N; q++) {
    if (!inI[q]) moot[q] = 2 * h[q] <= rMoot ? 1 : 0;
    code[q] = region[q] ? (2 * h[q] > rEdge ? 2 : 0) : moot[q] || cert[q] ? 0 : 1;
  }
  return { moot, cert, region, code };
}

export function sigmaHat(depth, n, o = {}) {
  const f = o.focal ?? depth.focal, rs = o.rSign ?? DEPTH_CAL.rSign, moot = o.moot ?? boneFlags(depth, n, Infinity, o.rMoot ?? DEPTH_CAL.rMoot).moot;
  const { h, sDelta } = depth.table, sp = depth.sigPx, P = depth.P, out = new Float64Array(n * NLM), term = new Float64Array(NE);
  for (let i = 0; i < n; i++) {
    for (let e = 0; e < NE; e++) {
      const q = i * NE + e, p = moot[q] ? 0 : depth.state[q] === 2 ? rs : depth.pWrong[q];
      term[e] = sDelta[q] * sDelta[q] + p * 4 * h[q] * h[q];
    }
    for (let j = 0; j < NLM; j++) {
      const a = -P[(i * NLM + j) * 3 + 2] * sp[j] / f, pj = PATH[j];
      let s = 2 * a * a;
      for (let k = 0; k < pj.length; k++) s += term[pj[k]];
      out[i * NLM + j] = Math.sqrt(s);
    }
  }
  return out;
}

export function reprojectionChi2(Lcam, series, focal, sigPx, o = {}) {
  const n = series.n, W = series.width, H = series.height, img = o.img ?? series.img, minVis = o.minVis ?? 0.3, Q = new Float64Array(n), dof = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    let s = 0, c = 0;
    for (const j of EVAL_JOINTS) {
      const r = (i * NLM + j) * 3, m = (i * NLM + j) * ST, Z = Lcam[r + 2];
      if (!(img[m + 3] >= minVis) || !(Z > 1e-6)) continue;
      const du = focal * Lcam[r] / Z + W / 2 - img[m] * W, dv = focal * Lcam[r + 1] / Z + H / 2 - img[m + 1] * H, sj = sigPx[j];
      s += (du * du + dv * dv) / (sj * sj); c++;
    }
    Q[i] = s; dof[i] = 2 * c;
  }
  return { Q, dof };
}

function prepare(ctx) {
  const { series, depth, skel, take, camera } = ctx, n = ctx.n ?? series.n, img = Float64Array.from(series.img);
  for (const i of depth?.swapFixed ?? []) swapLegRows(img, i, ST);
  const Lout = ctx.landmarks ?? finalLandmarks(skel, take, n), Lcam = cameraLandmarks(camera, Lout, n), focal = ctx.cam?.focal ?? camera.focal;
  return { n, img, Lout, Lcam, focal, chi: reprojectionChi2(Lcam, series, focal, depth.sigPx, { img }) };
}

export function domainStats(ctx, pre = prepare(ctx)) {
  const { series, depth } = ctx, { n, img } = pre, fps = ctx.fps ?? series.fps, W = series.width, H = series.height, T = new Float64Array(DOMAIN_STATS.length).fill(NaN), tb = depth.table, N = n * NE;
  const wd = depth.world ?? series.world, valid = series.valid, buf = [];
  T[0] = median(LONG_EDGES.map(e => {
    const [a, b] = TREE[e], ls = [];
    for (let i = 0; i < n; i++) if (!valid || valid[i]) { const ra = (i * NLM + a) * 3, rb = (i * NLM + b) * 3; ls.push(Math.hypot(wd[rb] - wd[ra], wd[rb + 1] - wd[ra + 1], wd[rb + 2] - wd[ra + 2])); }
    const m = median(ls);
    return median(ls.map(x => Math.abs(x - m))) / m;
  }));
  for (let i = 0; i < n; i++) if (pre.chi.dof[i] > 0) buf.push(pre.chi.Q[i] / pre.chi.dof[i]);
  T[1] = median(buf);
  if (depth.mpDelta) {
    let num = 0, den = 0;
    for (let e = 0; e < NE; e++) {
      let mu = 0, c = 0;
      const ok = i => !tb.miss[i * NE + e] && Number.isFinite(depth.mpDelta[i * NE + e] - tb.delta[i * NE + e]), r = i => depth.mpDelta[i * NE + e] - tb.delta[i * NE + e];
      for (let i = 0; i < n; i++) if (ok(i)) { mu += r(i); c++; }
      if (c < 2) continue;
      mu /= c;
      for (let i = 0; i < n; i++) if (ok(i)) { const x = r(i) - mu; den += x * x; if (i + 1 < n && ok(i + 1)) num += x * (r(i + 1) - mu); }
    }
    T[2] = den > 0 ? num / den : NaN;
  }
  T[3] = depth.kappa;
  let miss = 0, reg = 0, s2 = 0, dis = 0;
  for (let q = 0; q < N; q++) miss += tb.miss[q];
  T[4] = miss / N;
  let vs = 0;
  for (let i = 0; i < n; i++) for (const j of VIS_JOINTS) vs += img[(i * NLM + j) * ST + 3];
  T[5] = vs / (n * VIS_JOINTS.length);
  const ht = [], px = (i, j, c) => img[(i * NLM + j) * ST + c] * (c ? H : W);
  for (let i = 0; i < n; i++) ht.push(Math.hypot((px(i, 23, 0) + px(i, 24, 0) - px(i, 27, 0) - px(i, 28, 0)) / 2, (px(i, 23, 1) + px(i, 24, 1) - px(i, 27, 1) - px(i, 28, 1)) / 2));
  T[6] = median(EVAL_JOINTS.map(j => depth.sigPx[j])) / median(ht);
  for (const r of depth.gates?.regions ?? []) reg += Math.min(r.b, n - 1) - r.a + 1;
  T[7] = reg / N;
  if (depth.mpDelta && depth.state && depth.signs) {
    for (let q = 0; q < N; q++) if (depth.state[q] === 2) { s2++; if ((depth.mpDelta[q] < 0 ? -1 : 1) !== depth.signs[q]) dis++; }
    T[8] = s2 ? dis / s2 : NaN;
  }
  const con = depth.floor?.contact ?? footContact(img, n, W, H, fps, depth.P, null), P = depth.P;
  let cc = 0, fast = 0;
  for (let i = 1; i + 1 < n; i++) for (let s = 0; s < 2; s++) {
    if (!(Math.max(con[i * 4 + 2 * s], con[i * 4 + 2 * s + 1]) >= 0.5)) continue;
    const a = ((i - 1) * NLM + 27 + s) * 3, b = ((i + 1) * NLM + 27 + s) * 3;
    cc++;
    if (Math.hypot(P[b] - P[a], P[b + 1] - P[a + 1], P[b + 2] - P[a + 2]) * fps / 2 > 0.15) fast++;
  }
  T[9] = cc ? fast / cc : NaN;
  if (ctx.height > 0.5) { const g = gravityScale(pre.Lout, n, fps); if (g) T[10] = Math.abs(g.accel / 9.81 - 1); }
  return T;
}

export function domainThresholds(rows, a = 0.05 / 14) {
  const lo = [], hi = [], m = [];
  DOMAIN_STATS.forEach(([, side], k) => {
    const v = Float64Array.from(rows.map(r => r[k]).filter(x => !Number.isNaN(x))).sort(), N = v.length, K = conformalRank(N, a);
    m.push(N);
    hi.push(side === -1 ? Infinity : K === Infinity ? Infinity : v[K - 1]);
    lo.push(side === 1 ? -Infinity : K === Infinity ? -Infinity : v[N - K]);
  });
  return { names: DOMAIN_STATS.map(d => d[0]), lo, hi, n: m, a };
}

export function domainCheck(T, cal = DEPTH_CAL) {
  const lo = cal.domain?.lo ?? [], hi = cal.domain?.hi ?? [], bound = (x, d) => typeof x === 'number' && !Number.isNaN(x) ? x : d;
  for (let k = 0; k < DOMAIN_STATS.length; k++) {
    const x = T[k], side = DOMAIN_STATS[k][1];
    if (Number.isNaN(x) || x === undefined) continue;
    if ((side !== -1 && x > bound(hi[k], Infinity)) || (side !== 1 && x < bound(lo[k], -Infinity))) return { pass: false, which: k, plain: DOMAIN_STATS[k][2] };
  }
  return { pass: true, which: -1, plain: '' };
}

export function buildSpans(state, reason, edge, n, joints = EVAL_JOINTS) {
  const groups = new Map();
  for (const j of joints) for (let i = 0; i < n; i++) {
    const q = i * NLM + j, st = state[q];
    if (st === STATE.GREEN) continue;
    let b = i;
    while (b + 1 < n && state[q + NLM * (b + 1 - i)] === st && reason[q + NLM * (b + 1 - i)] === reason[q] && edge[q + NLM * (b + 1 - i)] === edge[q]) b++;
    const key = (st * 16 + reason[q]) * 32 + edge[q] + 1;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push([i, b, j]);
    i = b;
  }
  const out = [];
  for (const runs of groups.values()) {
    runs.sort((x, y) => x[0] - y[0] || x[1] - y[1]);
    let cur = null;
    for (const [a, b, j] of runs) {
      const q = a * NLM + j;
      if (cur && a <= cur.b + 1) { cur.b = Math.max(cur.b, b); cur.js.add(j); continue; }
      cur = { a, b, state: state[q], reason: reason[q], edge: edge[q], js: new Set([j]) };
      out.push(cur);
    }
  }
  return out.map(({ js, ...s }) => ({ ...s, joints: [...js].sort((x, y) => x - y) })).sort((x, y) => x.a - y.a || x.state - y.state || x.reason - y.reason || x.edge - y.edge);
}

export function certifyTake(ctx, cal = DEPTH_CAL, o = {}) {
  const t0 = Date.now(), { series, depth, camera } = ctx, n = ctx.n ?? series.n, N33 = n * NLM, NB = n * NE, sc = camera?.scale || 1, lamS = cal.sign?.lambda ?? Infinity, sig0 = cal.sigma0 ?? 0.01;
  const state = new Uint8Array(N33), reason = new Uint8Array(N33), edge = new Int8Array(N33).fill(-1), radius = new Float32Array(N33).fill(Infinity), sigma = new Float32Array(N33).fill(NaN);
  const bones = { sign: depth?.signs ? Int8Array.from(depth.signs) : new Int8Array(NB), certified: new Uint8Array(NB), moot: new Uint8Array(NB), pWrongModel: depth?.pWrong ? Float32Array.from(depth.pWrong) : new Float32Array(NB) };
  const failed = !depth || depth.ok === false || !depth.P || !depth.table || !depth.P.every(Number.isFinite);
  let values = new Float64Array(DOMAIN_STATS.length).fill(NaN), dc = { pass: false, which: -1, plain: DEPTH_FAILED }, intervals = [], raw = null, bf = null;
  if (!failed) {
    const pre = prepare(ctx), { img, chi } = pre, valid = series.valid, sw = depth.swapWin, miss = depth.table.miss, chi2F = cal.chi2Frame ?? Infinity, sMax = cal.sigmaMax ?? Infinity;
    values = domainStats(ctx, pre); dc = n < MIN_FRAMES ? { pass: false, which: -1, plain: TOO_SHORT } : domainCheck(values, cal); bf = boneFlags(depth, n, lamS, cal.rMoot ?? 0.02, cal.rEdge ?? R_EDGE);
    bones.moot.set(bf.moot); bones.certified.set(bf.cert);
    intervals = (depth.ints ?? []).map(I => ({ e: I.e, a: I.a, b: I.b, sign: I.sign, S: I.S, moot: !!I.moot, certified: !I.moot && I.S >= lamS }));
    const sig = sigmaHat(depth, n, { focal: pre.focal, rSign: cal.rSign, moot: bf.moot }), lam = GROUPS.map(g => cal.radius?.groups?.[g]?.lambda ?? Infinity);
    for (let i = 0; i < n; i++) {
      const bad = chi.dof[i] > 0 && chi.Q[i] / chi.dof[i] > chi2F, gap = valid && !valid[i], swp = sw && sw[i];
      for (let j = 0; j < NLM; j++) {
        const q = i * NLM + j, so = sig[q] * sc, g = GROUP_OF[j], pj = PATH[j], m = q * ST;
        sigma[q] = so;
        if (g < 0) { state[q] = STATE.AMBER; continue; }
        let st = STATE.RED, rs = 0, ed = -1;
        const x = img[m], y = img[m + 1];
        if (bad) rs = RS.IMAGE;
        else if ((ed = pj.find(e => miss[i * NE + e]) ?? -1) >= 0) rs = RS.LENGTH;
        else if (!(x >= 0 && x <= 1 && y >= 0 && y <= 1)) rs = RS.OUT;
        else {
          st = STATE.AMBER;
          if ((ed = pj.find(e => bf.code[i * NE + e] === 1) ?? -1) >= 0) rs = RS.FRONT_BACK;
          else if ((ed = pj.find(e => bf.code[i * NE + e] === 2) ?? -1) >= 0) rs = RS.EDGE_ON;
          else if (gap) rs = RS.GAP;
          else if (swp && j >= 23) rs = RS.SWAP;
          else if (!(img[m + 3] >= 0.3)) rs = RS.HIDDEN;
          else if (!(so <= sMax)) rs = RS.RADIUS;
          else st = STATE.GREEN;
        }
        state[q] = st; reason[q] = rs; edge[q] = ed;
      }
    }
    if (o.raw) raw = { state: Uint8Array.from(state), reason: Uint8Array.from(reason), norm: Float32Array.from(sigma, s => s + sig0), sigmaCam: sig, Q: chi.Q, dof: chi.dof, domain: values,
      intervals: depth.ints ?? [], boneMoot: bf.moot, boneCert: bf.cert, region: bf.region, scale: sc, Lcam: pre.Lcam };
    for (let q = 0; q < N33; q++) if (state[q] === STATE.GREEN) {
      const l = lam[GROUP_OF[q % NLM]];
      if (Number.isFinite(l)) radius[q] = l * (sigma[q] + sig0); else { state[q] = STATE.AMBER; reason[q] = RS.RADIUS; }
    }
  }
  const quick = !!cal.quick, inDomain = !failed && dc.pass, grey = quick || !inDomain;
  if (grey) { state.fill(STATE.GREY); reason.fill(quick ? RS.UNCALIBRATED : RS.DOMAIN); edge.fill(-1); radius.fill(Infinity); }
  const spans = buildSpans(state, reason, edge, n), cnt = [0, 0, 0, 0], rad = [];
  for (let i = 0; i < n; i++) for (const j of EVAL_JOINTS) { const q = i * NLM + j; cnt[state[q]]++; if (state[q] === STATE.GREEN) rad.push(radius[q] * 100); }
  const tot = n * EVAL_JOINTS.length || 1, bc = [0, 0, 0];
  if (bf) for (let q = 0; q < NB; q++) if (!bf.region[q]) bc[bf.cert[q] ? 0 : bf.moot[q] ? 1 : 2]++;
  const bt = bc[0] + bc[1] + bc[2] || 1, grav = ctx.grav, flagged = spans.filter(s => s.state === STATE.AMBER || s.state === STATE.RED).length;
  const summary = { on: true, calibId: cal.calibId, quick, alpha: cal.alpha, delta: cal.delta, rSign: cal.rSign, inDomain, domainReason: inDomain ? '' : dc.plain, scaleSource: ctx.scaleSource ?? 'mediapipe',
    scaleErr: ctx.scaleSource === 'gravity' && grav?.scale ? Math.max(1, Math.round(200 * grav.sd / grav.scale)) : NaN, kappa: depth?.kappa ?? NaN, sigmaMP: depth?.sigMP ?? NaN,
    lengthsFromPrior: [...new Set((depth?.lengths?.src ?? []).map((s, e) => s === 'prior' ? EDGES[e]?.[2] : null).filter(Boolean))],
    green: cnt[STATE.GREEN] / tot, amber: cnt[STATE.AMBER] / tot, red: cnt[STATE.RED] / tot, grey: cnt[STATE.GREY] / tot,
    radiusMedian: rad.length ? median(rad) : NaN, radiusP95: rad.length ? percentile(rad, 0.95) : NaN, radiusMax: rad.length ? rad.reduce((a, b) => a > b ? a : b) : NaN,
    signCertified: bc[0] / bt, signMoot: bc[1] / bt, signAmber: bc[2] / bt, spans: grey ? 0 : flagged, ms: 0, line: '' };
  summary.ms = Date.now() - t0;
  summary.line = depthLine(summary);
  const cert = { version: cal.version, calibId: cal.calibId, quick, inDomain, domain: { values, which: dc.which, plain: dc.plain },
    status: grey || !cnt[STATE.GREEN] ? 'unverified' : cnt[STATE.GREEN] === tot ? 'verified' : 'partial', state, reason, radius, sigma, bones, intervals, spans, summary };
  if (raw) cert.raw = raw;
  return cert;
}

export function depthLine(s) {
  if (!s?.on) return '';
  if (s.quick) return 'Depth check: not calibrated (developer build), so nothing is verified.';
  if (!s.inDomain) return `Depth check: not verified, because ${s.domainReason || 'the video is unlike the calibration set'}. The animation is still usable.`;
  const conf = Math.round(100 * (1 - (s.delta ?? 0.05))), pct = Math.round(100 * s.green), head = s.green > 0 && Number.isFinite(s.radiusP95)
    ? `Depth check: ${pct < 1 ? 'under 1' : pct}% of joint positions verified from any angle to within ${Math.ceil(s.radiusP95)} cm (misses at most 1 in ${Math.round(1 / s.alpha)}, ${conf}% confidence).`
    : 'Depth check: no joint positions could be verified from this camera.';
  const size = s.scaleSource === 'height' ? 'Size from your height.' : s.scaleSource === 'gravity' ? `Size from the jump (±${s.scaleErr}%).` : 'Overall size is not checked: enter the person\'s height.';
  return [head, s.spans ? `${s.spans} ${s.spans === 1 ? 'span' : 'spans'} flagged for review.` : '', size].filter(Boolean).join(' ');
}

export function shapeErrors(truthC, recC, n, o = {}) {
  const js = o.joints ?? EVAL_JOINTS, use = i => !o.valid || o.valid[i], hip = (A, i, c) => (A[(i * NLM + 23) * 3 + c] + A[(i * NLM + 24) * 3 + c]) / 2, err = new Float64Array(n * NLM).fill(NaN);
  let num = 0, den = 0;
  for (let i = 0; i < n; i++) if (use(i)) for (const j of js) for (let c = 0; c < 3; c++) {
    const r = (i * NLM + j) * 3 + c, a = recC[r] - hip(recC, i, c), b = truthC[r] - hip(truthC, i, c);
    num += a * b; den += a * a;
  }
  const s = den > 0 ? num / den : 1;
  for (let i = 0; i < n; i++) for (const j of js) {
    let e2 = 0;
    for (let c = 0; c < 3; c++) { const r = (i * NLM + j) * 3 + c; e2 += (s * (recC[r] - hip(recC, i, c)) - (truthC[r] - hip(truthC, i, c))) ** 2; }
    err[i * NLM + j] = Math.sqrt(e2);
  }
  return { s, err };
}

export function trueSigns(truthC, n, rMoot = DEPTH_CAL.rMoot) {
  const sign = new Int8Array(n * NE), nonMoot = new Uint8Array(n * NE), delta = new Float64Array(n * NE);
  for (let i = 0; i < n; i++) for (let e = 0; e < NE; e++) {
    const a = (i * NLM + TREE[e][0]) * 3, b = (i * NLM + TREE[e][1]) * 3, l = Math.hypot(truthC[b], truthC[b + 1], truthC[b + 2]), q = i * NE + e;
    const d = ((truthC[b] - truthC[a]) * truthC[b] + (truthC[b + 1] - truthC[a + 1]) * truthC[b + 1] + (truthC[b + 2] - truthC[a + 2]) * truthC[b + 2]) / l;
    delta[q] = d; sign[q] = d < 0 ? -1 : 1; nonMoot[q] = 2 * Math.abs(d) >= rMoot ? 1 : 0;
  }
  return { sign, nonMoot, delta };
}

export function intervalRecords(ints, truth, n) {
  return ints.map(I => {
    let nm = 0, wr = 0;
    for (let i = I.a; i <= Math.min(I.b, n - 1); i++) { const q = i * NE + I.e; if (!truth.nonMoot[q]) continue; nm++; if (truth.sign[q] !== I.sign) wr++; }
    return { S: I.S, nonMoot: nm, wrong: wr };
  });
}
