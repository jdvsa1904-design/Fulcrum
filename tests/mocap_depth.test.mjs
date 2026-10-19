import test from 'node:test';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync, existsSync } from 'node:fs';
import { near, rng } from './helpers.mjs';
import * as S from '../mocap/skeleton.js';
import { gaitClip, idleClip, jumpClip } from '../mocap/gait.js';
import { createController, recordTake } from '../mocap/controller.js';
import { lookAtCamera, synthCapture, toCam, toReconstructionFrame } from '../mocap/synth.js';
import { buildTracks, pickTrack, trackSeries, cameraPoints } from '../mocap/landmarks.js';
import { floorFromPoints, floorTransform, applyFloor, takeWorld, worldAt } from '../mocap/ground.js';
import { processCapture, DEFAULTS } from '../mocap/pipeline.js';
import * as D from '../mocap/depth.js';
import * as G from '../mocap/depthsign.js';
import * as T from '../mocap/depthstats.js';
import * as F from '../mocap/mpfail.js';
import * as C from '../mocap/depthcert.js';
import { DEPTH_CAL } from '../mocap/depth_calibration.js';

const url = p => new URL(p, import.meta.url);
const optional = async p => existsSync(url(p)) ? import(url(p).href).catch(e => ({ __error: e })) : null;
const BENCH = await optional('../mocap/depthbench.js'), bench = BENCH && !BENCH.__error ? BENCH : null;
const PIPE_DEPTH = 'depth' in DEFAULTS;
const NLM = 33, NE = 18, RMOOT = 0.02;
const GOLDEN = 'c365c6a157de5e6e3b95e9ebfba3dc80e3ccd6a788c1558f1a2c71f2302bffd6';
const BIT_DEPTH_OFF = '72a6cfa36b57cfe9a67d95288638796c3cee656469551cd81ba4833236d0ef74'; // landmarks + take.root + take.rot, seed-3 scenario, recorded before the pipeline edit
const NEW_FILES = ['depth', 'depthsign', 'depthstats', 'depthcert', 'depth_calibration', 'mpfail', 'depthbench'].map(f => `../mocap/${f}.js`);
const HASHED = ['landmarks', 'clean', 'ground', 'skeleton', 'solve', 'pipeline', 'depth', 'depthsign', 'depthstats', 'depthcert', 'mpfail', 'depthbench', 'synth', 'gait', 'controller'];

// --- fixtures (built once, lazily) ---
const truthSkel = S.buildSkeleton({ ...S.DEFAULT_DIMS, thigh: 0.44, shin: 0.43, torso: 0.52 });
const cam = lookAtCamera([0.3, 1.45, 4.5], [0, 0.85, -0.5], { width: 1280, height: 720, hfov: 60 });
const truthLib = () => ({ idle: idleClip(truthSkel), walk: gaitClip(truthSkel, { kind: 'walk', speed: 1.25 }), run: gaitClip(truthSkel, { kind: 'run', speed: 3.45 }), jump: jumpClip(truthSkel), actions: [], take: null });
function record(steps, start, yaw = Math.PI / 2) {
  const ctl = createController(truthLib(), { yaw });
  ctl.pos = start;
  return recordTake(ctl, steps.map(s => ({ ...s, input: { ...s.input, camYaw: Math.PI } })), 30);
}
const once = f => { let v; return () => v ??= f(); };
const TAKES = {
  pipe3: once(() => record([{ seconds: 1.5, input: {} }, { seconds: 4.5, input: { right: true } }, { seconds: 1.5, input: {} }], [-3, 0, -0.5])),
  toward: once(() => record([{ seconds: 3, input: { back: true } }], [0, 0, -5.5])),
};
// capture + series + first-pass camera points + truth camera-frame landmarks (flat, aligned with the series)
function prepare(take, o = {}) {
  const c = o.cam ?? cam, { capture, truth, faults } = synthCapture(truthSkel, take, c, { rand: rng(o.seed ?? 3), noiseImg: 0, noiseWorld: 0, occlusion: true, ...o.synth });
  const tracks = buildTracks(capture), series = trackSeries(capture, pickTrack(tracks, capture, {}), {}), cp = cameraPoints(series, { hfov: o.hfov ?? c.hfov, imageFit: 0.5 }), n = series.n;
  const TC = new Float64Array(n * NLM * 3);
  for (let i = 0; i < n; i++) for (let j = 0; j < NLM; j++) TC.set(toCam(c, S.lmAt(truth[series.first + i], j)), (i * NLM + j) * 3);
  return { capture, truth, faults, series, cp, TC, n, cam: c };
}
const FX = {
  towardClean: once(() => prepare(TAKES.toward())),
  towardTyp: once(() => prepare(TAKES.toward(), { synth: { mp: 'typical', mpSeed: 12 } })),
};
const pt = (A, i, j) => [A[(i * NLM + j) * 3], A[(i * NLM + j) * 3 + 1], A[(i * NLM + j) * 3 + 2]];
const mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2], sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]], dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2], len = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => { const l = len(a); return [a[0] / l, a[1] / l, a[2] / l]; };
const storedToCam = (P, n) => Float64Array.from(P, (x, q) => q % 3 ? -x : x);
// true per-frame signs and deltas of the 18 tree edges from camera-frame truth
function truthSigns(TC, n) {
  const sign = new Int8Array(n * NE), delta = new Float64Array(n * NE);
  for (let i = 0; i < n; i++) D.TREE.forEach(([a, b], e) => { const cb = pt(TC, i, b), d = dot(sub(cb, pt(TC, i, a)), unit(cb)); delta[i * NE + e] = d; sign[i * NE + e] = d < 0 ? -1 : 1; });
  return { sign, delta };
}
const truthLengths = (TC, n) => { const L = new Float64Array(n * D.EDGES.length); for (let i = 0; i < n; i++) D.EDGES.forEach(([a, b], e) => { L[i * D.EDGES.length + e] = len(sub(pt(TC, i, b), pt(TC, i, a))); }); return L; };
const median = a => { const s = Float64Array.from(a).sort(), h = s.length >> 1; return s.length % 2 ? s[h] : (s[h - 1] + s[h]) / 2; };
const quant = (a, p) => { const s = Float64Array.from(a).sort(); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };
const truthDecide = TC => ctx => {
  const n = ctx.n, { sign, delta } = truthSigns(TC, n), state = Uint8Array.from(delta, d => 2 * Math.abs(d) >= RMOOT ? 2 : 1);
  return { signs: sign, state, pWrong: new Float32Array(n * NE), ints: [], gates: { open: new Uint8Array(n * NE), xpt: new Uint8Array(n * NE), regions: [] } };
};

// ============================== Geometry ==============================

test('depth 1: branch recovers the child exactly, clamps tiny negative h², and stays exact far away', () => {
  const r = T.mulberry32(101), U = (a, b) => a + (b - a) * r();
  for (const far of [1, 10]) for (let k = 0; k < 2000; k++) {
    const Xp = [U(-1, 1) * far, U(-1, 1) * far, U(2, 6) * far], u = unit([U(-1, 1), U(-1, 1), U(-1, 1)]), L = U(0.05, 0.6), Xc = [Xp[0] + u[0] * L, Xp[1] + u[1] * L, Xp[2] + u[2] * L], d = unit(Xc);
    const b = D.branch(d, Xp, L), s = dot(sub(Xc, Xp), d) < 0 ? -1 : 1, t = b.t0 + s * b.h;
    assert.ok(len(sub([d[0] * t, d[1] * t, d[2] * t], Xc)) < 1e-9 * far, `error at |Xp| ${len(Xp)}`);
  }
  const Xp = [0.1, -0.2, 4], d = unit([0.3, 0.1, 4]), b0 = D.branch(d, Xp, 1), rho = b0.rho, L = Math.sqrt(rho * rho - 1e-12), b = D.branch(d, Xp, L);
  assert.ok(b.h2 <= 0 && b.h2 > -1e-11 && b.h === 0 && Object.values(b).every(Number.isFinite), 'h² = −1e-12 clamps to h = 0');
  const Xf = [3, -2, 50], uf = unit([0.2, 0.9, -0.3]), Xc = [Xf[0] + uf[0] * 0.4, Xf[1] + uf[1] * 0.4, Xf[2] + uf[2] * 0.4], df = unit(Xc), bf = D.branch(df, Xf, 0.4), sf = dot(sub(Xc, Xf), df) < 0 ? -1 : 1, tf = bf.t0 + sf * bf.h;
  assert.ok(len(sub([df[0] * tf, df[1] * tf, df[2] * tf], Xc)) < 1e-8, 'stable at 50 m');
});

test('depth 2: chainDepths with true lengths, signs and root reproduces noise-free truth to 1 mm', () => {
  const { series, TC, n, cam: c } = FX.towardClean(), R = D.imageRays(series.img, n, series.width, series.height, c.focal), { sign } = truthSigns(TC, n);
  const rootZ = Float64Array.from({ length: n }, (_, i) => (pt(TC, i, 23)[2] + pt(TC, i, 24)[2]) / 2), res = D.chainDepths(R, n, rootZ, truthLengths(TC, n), sign, { focal: c.focal });
  let worst = 0;
  for (let i = 0; i < n; i++) D.RAY_JOINTS.forEach((j, k) => { const t = res.t[i * D.K_RAY + k], r = (i * NLM + j) * 3; worst = Math.max(worst, len(sub([R[r] * t, R[r + 1] * t, R[r + 2] * t], pt(TC, i, j)))); });
  assert.ok(worst < 1e-3, `max error ${worst} m`);
});

test('depth 3: deltaSigma is within 20% of Monte Carlo', () => {
  const r = T.mulberry32(303), gauss = () => { let u = 0; while (!u) u = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * r()); };
  const L = 0.4, sR = 0.006, sL = 0.004;
  for (const q of [0, 0.1, 0.3, 0.6]) {
    const del = q * L, rho = Math.sqrt(L * L - del * del), M = 40000;
    let s2 = 0;
    for (let k = 0; k < M; k++) { const rr = rho + sR * gauss(), ll = L + sL * gauss(), y = (ll - rr) * (ll + rr), e = Math.sqrt(Math.max(0, y)) - del; s2 += e * e; }
    const mc = Math.sqrt(s2 / M), an = D.deltaSigma(del, rho, L, sR, sL);
    assert.ok(Math.abs(an / mc - 1) <= 0.2, `δ/L ${q}: analytic ${an} vs MC ${mc}`);
  }
});

test('depth 4: bandChol/bandSolve match a dense solve on a random SPD band matrix', () => {
  const N = 60, bw = 7, w = bw + 1, r = T.mulberry32(404), H = Array.from({ length: N }, () => new Float64Array(N));
  for (let i = 0; i < N; i++) for (let k = 1; k <= bw && i - k >= 0; k++) H[i][i - k] = H[i - k][i] = r() * 2 - 1;
  for (let i = 0; i < N; i++) { let s = 0; for (let j = 0; j < N; j++) if (j !== i) s += Math.abs(H[i][j]); H[i][i] = s + 0.5 + r(); }
  const A = new Float64Array(N * w);
  for (let i = 0; i < N; i++) for (let k = 0; k <= bw && i - k >= 0; k++) A[i * w + k] = H[i][i - k];
  const b = Float64Array.from({ length: N }, () => r() * 2 - 1), M = H.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < N; c++) { let p = c; for (let i = c + 1; i < N; i++) if (Math.abs(M[i][c]) > Math.abs(M[p][c])) p = i; [M[c], M[p]] = [M[p], M[c]]; for (let i = c + 1; i < N; i++) { const f = M[i][c] / M[c][c]; for (let j = c; j <= N; j++) M[i][j] -= f * M[c][j]; } }
  const x = new Float64Array(N);
  for (let i = N - 1; i >= 0; i--) { let s = M[i][N]; for (let j = i + 1; j < N; j++) s -= M[i][j] * x[j]; x[i] = s / M[i][i]; }
  assert.ok(D.bandChol(A, N, bw), 'SPD factorises');
  const y = D.bandSolve(A, N, bw, b);
  for (let i = 0; i < N; i++) near(y[i], x[i], 1e-9, `x[${i}]`);
  const bad = new Float64Array(N * w).fill(0);
  for (let i = 0; i < N; i++) bad[i * w] = i === 30 ? -1 : 1;
  assert.equal(D.bandChol(bad, N, bw), false, 'a negative pivot is reported');
});

test('depth 5: boneLengths recovers long bones and κ under compressed, flipping tracker depth; side-on hips fall back to the prior', () => {
  const f = prepare(TAKES.toward(), { synth: { mp: { base: 'none', gain: 0.6, flipRate: 0.25, flipDur: [0.15, 1.5] }, mpSeed: 21 } }), d = D.depthPoints(f.series, f.cp, { fps: 30, decide: truthDecide(f.TC) }), TL = truthLengths(f.TC, f.n);
  for (const key of D.LENGTH_CLASS.long) {
    const es = D.EDGES.map((x, e) => x[2] === key ? e : -1).filter(e => e >= 0), truth = median(es.flatMap(e => Array.from({ length: f.n }, (_, i) => TL[i * D.EDGES.length + e])));
    for (const e of es) assert.ok(Math.abs(d.lengths.L[e] / truth - 1) <= 0.025, `${key}: ${d.lengths.L[e]} vs ${truth}`);
  }
  assert.ok(Math.abs(d.kappa - 0.6) < 0.12, `kappa ${d.kappa}`);
  const a = prepare(TAKES.pipe3(), { synth: { noiseImg: 0.003, noiseWorld: 0.015 } }), da = D.depthPoints(a.series, a.cp, { fps: 30 });
  assert.equal(da.lengths.src[0], 'prior', 'side-on hip width comes from the prior');
});

test('depth 6: a wrong sign is invisible to lengths and reprojection, moves the child 2h, and anatomy favours the true sign', () => {
  const { series, TC, n, cam: c } = FX.towardClean(), R = D.imageRays(series.img, n, series.width, series.height, c.focal), { sign, delta } = truthSigns(TC, n), L = truthLengths(TC, n);
  const rootZ = Float64Array.from({ length: n }, (_, i) => (pt(TC, i, 23)[2] + pt(TC, i, 24)[2]) / 2), base = D.chainDepths(R, n, rootZ, L, sign, { focal: c.focal });
  const e = 4, flipped = Int8Array.from(sign), frames = [];
  for (let i = 0; i < n; i++) if (Math.abs(delta[i * NE + e]) > 0.03) { flipped[i * NE + e] *= -1; frames.push(i); }
  assert.ok(frames.length > 5, 'the forearm leaves the image plane');
  const alt = D.chainDepths(R, n, rootZ, L, flipped, { focal: c.focal }), X = (res, i, j) => { const t = res.t[i * D.K_RAY + D.SLOT[j]], r = (i * NLM + j) * 3; return [R[r] * t, R[r + 1] * t, R[r + 2] * t]; };
  const W = series.width, H = series.height;
  for (const i of frames) {
    D.EDGES.forEach(([a, b], k) => { if (D.SLOT[a] >= 0 && D.SLOT[b] >= 0) near(len(sub(X(alt, i, b), X(alt, i, a))), len(sub(X(base, i, b), X(base, i, a))), 1e-9, `edge ${k} length`); });
    const p = X(alt, i, 15), m = (i * NLM + 15) * 5;
    near(c.focal * p[0] / p[2] + W / 2, series.img[m] * W, 1e-6, 'u'); near(c.focal * p[1] / p[2] + H / 2, series.img[m + 1] * H, 1e-6, 'v');
    near(len(sub(X(alt, i, 15), X(base, i, 15))), 2 * base.h[i * NE + e], 1e-9, 'moves 2h');
  }
  const parentX = TC, ctx = { n, R, fps: 30, L: D.EDGES.map((x, k) => median(Array.from({ length: n }, (_, i) => L[i * D.EDGES.length + k]))), sigPx: new Float64Array(NLM).fill(1), rootZ, focal: c.focal, table: null };
  for (const name of ['armL', 'armR', 'legL', 'legR']) {
    const edges = G.CHAINS[name], Hy = G.chainHypotheses(R, n, ctx.L, edges, parentX, rootZ), an = G.cueAnatomy(Hy, ctx, name, parentX);
    let lo = 0, used = 0;
    for (let i = 0; i < n; i++) {
      let k = 0;
      edges.forEach((ed, b) => { if (sign[i * NE + ed] < 0) k |= 1 << b; });
      edges.forEach((ed, b) => { if (Math.abs(delta[i * NE + ed]) < 0.05) return; lo += an[i * Hy.K + k] - an[i * Hy.K + (k ^ (1 << b))]; used++; });
    }
    assert.ok(used > 0 && lo > 0, `${name}: anatomy log-odds ${lo} over ${used} bone-frames`);
  }
});

// ============================== Decision layer ==============================

test('depth 7: crossing gates never close on a true crossing and close exactly where the bound sum exceeds wMax·dt', () => {
  const r = T.mulberry32(707), gauss = () => { let u = 0; while (!u) u = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * r()); };
  const n = 60, fps = 30, Lb = 0.4, sH2 = 0.004, k = 3, wMax = G.SIGN_DEFAULTS.wMax;
  let crossings = 0;
  for (let run = 0; run < Math.ceil(1000 / NE); run++) {
    const N = n * NE, tab = { phiHat: new Float64Array(N), phiLo: new Float64Array(N), wRay: new Float64Array(N), sH2: new Float64Array(N).fill(sH2) }, phi = new Float64Array(N);
    for (let e = 0; e < NE; e++) {
      const ph0 = r();
      for (let i = 0; i < n; i++) {
        const q = i * NE + e, p = 0.3 * Math.sin(2 * Math.PI * (i / fps + ph0)) + 0.01 * gauss(), h2 = (Lb * Math.sin(p)) ** 2 + sH2 * gauss(), b = D.hBounds(h2, sH2, k);
        phi[q] = p; tab.phiHat[q] = Math.asin(Math.min(1, Math.sqrt(Math.max(0, h2)) / Lb)); tab.phiLo[q] = Math.asin(Math.min(1, b.lo / Lb)); tab.wRay[q] = 3 * r();
      }
    }
    const g = G.crossingGates(tab, n, fps, { wMax, kGate: k });
    for (let i = 0; i + 1 < n; i++) for (let e = 0; e < NE; e++) {
      const q = i * NE + e;
      if (Math.sign(phi[q]) !== Math.sign(phi[q + NE])) { crossings++; assert.equal(g.open[q], 1, `true crossing closed at run ${run}, frame ${i}, edge ${e}`); }
      assert.equal(g.open[q], tab.phiLo[q] + tab.phiLo[q + NE] <= (wMax + tab.wRay[q]) / fps ? 1 : 0);
    }
  }
  assert.ok(crossings > 2000, `crossings ${crossings}`);
});

test('depth 8: forwardBackward and viterbi match brute force (n = 6, K = 4)', () => {
  const n = 6, K = 4, r = T.mulberry32(808), em = Float64Array.from({ length: n * K }, () => 3 * (r() - 0.5));
  const mat = () => { const M = Float64Array.from({ length: K * K }, () => Math.log(0.05 + r())); for (let a = 0; a < K; a++) { let z = 0; for (let b = 0; b < K; b++) z += Math.exp(M[a * K + b]); for (let b = 0; b < K; b++) M[a * K + b] -= Math.log(z); } return M; };
  const gates = { xpt: new Uint8Array(n * NE), open: new Uint8Array(n * NE), regions: [] };
  gates.xpt[2 * NE + 3] = 1;
  const cases = [{ base: mat(), at: new Map([[2, mat()], [4, mat()]]) }, G.chainTransitions(gates, [3], n, 30)];
  for (const tr of cases) {
    const at = i => tr.at.get(i) ?? tr.base, lse = a => { const m = Math.max(...a); return m === -Infinity ? m : m + Math.log(a.reduce((s, x) => s + Math.exp(x - m), 0)); };
    const paths = [], lp = [];
    for (let code = 0; code < K ** n; code++) {
      const p = Array.from({ length: n }, (_, i) => Math.floor(code / K ** i) % K);
      let l = -Math.log(K) + em[p[0]];
      for (let i = 1; i < n; i++) l += at(i - 1)[p[i - 1] * K + p[i]] + em[i * K + p[i]];
      paths.push(p); lp.push(l);
    }
    const logZ = lse(lp), fb = G.forwardBackward(em, n, K, tr);
    near(fb.logZ, logZ, 1e-12, 'logZ');
    for (let i = 0; i < n; i++) for (let k = 0; k < K; k++) near(fb.post[i * K + k], lp.reduce((s, l, c) => s + (paths[c][i] === k ? Math.exp(l - logZ) : 0), 0), 1e-12, `post ${i},${k}`);
    const v = G.viterbi(em, n, K, tr);
    let l = -Math.log(K) + em[v[0]];
    for (let i = 1; i < n; i++) l += at(i - 1)[v[i - 1] * K + v[i]] + em[i * K + v[i]];
    near(l, Math.max(...lp), 1e-12, 'viterbi path is the brute-force max');
  }
});

function clipLandmarks(skel, clip) {
  const out = [], tmp = { pos: new Float64Array(S.NB * 3), q: new Float64Array(S.NB * 4) };
  for (let i = 0; i < clip.n; i++) { S.fk(skel, S.getV(clip.root, i), clip.rot.subarray(i * S.NB * 4, (i + 1) * S.NB * 4), tmp); out.push(S.landmarksFromPose(skel, tmp)); }
  return out;
}
test('depth 9: anatomy limits hold with margin on every truth clip and catch a backward knee and a back-folded hanging forearm', () => {
  const sk = truthSkel, clips = { walk: gaitClip(sk, { kind: 'walk' }), run: gaitClip(sk, { kind: 'run' }), idle: idleClip(sk), jump: jumpClip(sk) };
  if (bench?.reachClip) clips.reach = bench.reachClip(sk);
  for (const [name, clip] of Object.entries(clips)) for (const [i, P] of clipLandmarks(sk, clip).entries()) {
    const v = G.limitViolations(P, [0, 1, 0]);
    for (const [k, x] of Object.entries(v)) assert.ok(k.endsWith('Capsule') ? x > 0 : x <= -5, `${name} frame ${i}: ${k} = ${x}`);
  }
  const P = Float64Array.from(S.landmarksFromPose(sk, S.restPose(sk))), get = j => pt(P, 0, j), fr = G.torsoFrame(get);
  const rotAbout = (p, o, ax, a) => { const v = sub(p, o), c = Math.cos(a), s = Math.sin(a), k = ax, cr = [k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]], kd = dot(k, v); return [0, 1, 2].map(m => o[m] + v[m] * c + cr[m] * s + k[m] * kd * (1 - c)); };
  const knee = get(25), probe = rotAbout(get(27), knee, fr.left, 0.5), sgn = dot(sub(probe, get(27)), fr.fwd) > 0 ? 1 : -1, bk = Float64Array.from(P);
  for (const j of [27, 29, 31]) bk.set(rotAbout(get(j), knee, fr.left, sgn * 45 * Math.PI / 180), j * 3);
  assert.ok(G.limitViolations(bk).legL > 20, `backward knee ${G.limitViolations(bk).legL}`);
  const arm = Float64Array.from(P), sh = get(11), Lu = len(sub(get(13), sh)), Lf = len(sub(get(15), get(13))), elb = sh.map((x, m) => x - fr.up[m] * Lu), wri = elb.map((x, m) => x - fr.fwd[m] * Lf);
  arm.set(elb, 13 * 3); arm.set(wri, 15 * 3);
  for (const [j, a] of [[17, 15], [19, 15], [21, 15]]) arm.set(wri, j * 3);
  assert.ok(G.limitViolations(arm).armL > 20, `back-folded forearm ${G.limitViolations(arm).armL}`);
  const ok = Float64Array.from(arm), wf = elb.map((x, m) => x + fr.fwd[m] * Lf);
  ok.set(wf, 15 * 3);
  assert.ok(G.limitViolations(ok).armL < 0, 'the same arm folded forward is fine');
});

test('depth 10: with exact tracker depth, MP-only log-odds favour the true sign on every non-moot interval', () => {
  const { series, TC, n, cam: c } = FX.towardClean(), R = D.imageRays(series.img, n, series.width, series.height, c.focal), { sign, delta } = truthSigns(TC, n), Lf = truthLengths(TC, n);
  const L = Float64Array.from(D.EDGES, (x, k) => median(Array.from({ length: n }, (_, i) => Lf[i * D.EDGES.length + k]))), rootZ = Float64Array.from({ length: n }, (_, i) => (pt(TC, i, 23)[2] + pt(TC, i, 24)[2]) / 2);
  const t = D.chainDepths(R, n, rootZ, L, sign, { focal: c.focal }).t, sigPx = new Float64Array(NLM).fill(0.5), tab = D.branchTable(R, n, t, L, null, sigPx, c.focal, 30);
  const gates = G.crossingGates(tab, n, 30), ints = G.signIntervals(gates, n), ctx = { n, R, fps: 30, L, world: series.world, table: tab, kappa: 1, sigMP: 0.02, sigPx, rootZ, focal: c.focal };
  let checked = 0;
  for (const [name, edges] of Object.entries(G.CHAINS)) {
    const Hy = G.chainHypotheses(R, n, L, edges, name === 'torso' ? null : TC, rootZ), mp = G.cueMP(Hy, ctx);
    edges.forEach((e, b) => {
      for (const I of ints) {
        if (I.e !== e) continue;
        let hm = 0, lo = 0;
        for (let i = I.a; i <= I.b; i++) {
          hm = Math.max(hm, tab.hHi[i * NE + e]);
          let k = 0;
          edges.forEach((ed, bb) => { if (sign[i * NE + ed] < 0) k |= 1 << bb; });
          lo += mp[(i * Hy.K + k) * 2] - mp[(i * Hy.K + (k ^ (1 << b))) * 2];
        }
        if (2 * hm <= RMOOT || !(Math.abs(delta[I.a * NE + e]) > 0) || new Set(Array.from({ length: I.b - I.a + 1 }, (_, q) => sign[(I.a + q) * NE + e])).size > 1) continue;
        checked++;
        assert.ok(lo > 0, `${name} edge ${e} [${I.a}, ${I.b}]: MP log-odds ${lo}`);
      }
    });
  }
  assert.ok(checked > 10, `intervals checked ${checked}`);
});

// ============================== Statistics ==============================

test('depth 11: conformal ranks, Clopper–Pearson, Bentkus and learn-then-test match their closed forms', () => {
  const sc = n => Float64Array.from({ length: n }, (_, i) => i + 1);
  const q999 = T.conformalQuantile(sc(999), 1e-3);
  assert.equal(q999.k, 999); assert.equal(q999.q, 999);
  assert.equal(T.conformalQuantile(sc(998), 1e-3).q, Infinity);
  assert.ok(T.cpUpper(0, 2995) <= 1e-3 && 1e-3 < T.cpUpper(0, 2994), `cp ${T.cpUpper(0, 2995)} ${T.cpUpper(0, 2994)}`);
  near(T.bentkusP(0, 5000, 1e-3), 0.018, 0.002, 'bentkus');
  near(T.binomCdf(3, 10, 0.3), 0.6496107184, 1e-9, 'binomCdf');
  const lam = [5, 4, 3, 2, 1], toy = T.lttFixedSequence(lam, [0, 0, 0.002, 0.02, 0.05], 3000, 0.01, 0.05);
  assert.equal(toy.index, 2); assert.equal(toy.lambda, 3); assert.ok(toy.p <= 0.05);
  const none = T.lttFixedSequence(lam, [0.05, 0, 0, 0, 0], 3000, 0.01, 0.05);
  assert.equal(none.index, -1); assert.equal(none.lambda, Infinity);
  const r = T.mulberry32(1), h = rng(1);
  for (let i = 0; i < 5; i++) assert.equal(r(), h(), 'mulberry32 is the helpers generator');
});

test('depth 12: split conformal covers i.i.d. exponential scores at the nominal rate', () => {
  const r = T.mulberry32(1212), N = 99, a = 0.1, reps = 2000, ex = () => -Math.log(1 - r());
  let hit = 0;
  for (let k = 0; k < reps; k++) { const { q } = T.conformalQuantile(Float64Array.from({ length: N }, ex), a); if (ex() <= q) hit++; }
  const cov = hit / reps, se = Math.sqrt(0.9 * 0.1 / reps);
  assert.ok(cov >= 0.9 - 3 * se && cov <= 0.91 + 3 * se, `coverage ${cov}`);
});

// ============================== Failure model ==============================

const digest = capture => { const h = createHash('sha256'); for (const f of capture.frames) for (const p of f.poses) { h.update(Buffer.from(p.img.buffer)); h.update(Buffer.from(p.world.buffer)); } return h.digest('hex'); };
test('depth 13: the default synthCapture output is bit-identical and has no faults key', () => {
  const out = synthCapture(truthSkel, TAKES.pipe3(), cam, { rand: rng(3), noiseImg: 0.003, noiseWorld: 0.015, occlusion: true });
  assert.equal(digest(out.capture), GOLDEN);
  const und = synthCapture(truthSkel, TAKES.pipe3(), cam, { rand: rng(3), noiseImg: 0.003, noiseWorld: 0.015, occlusion: true, mp: undefined });
  assert.ok(!('faults' in und), 'mp: undefined adds no faults key');
  assert.equal(digest(und.capture), GOLDEN);
});

test('depth 14: mpfail profiles do exactly what they say', () => {
  const take = TAKES.toward(), syn = mp => synthCapture(truthSkel, take, cam, { rand: rng(5), noiseImg: 0, noiseWorld: 0, occlusion: true, mp, mpSeed: 9 });
  const base = syn(undefined), n = take.n, P = (cap, i) => cap.frames[i].poses[0], Cs = base.truth.map(L => Array.from({ length: NLM }, (_, j) => toCam(cam, S.lmAt(L, j))));
  const none = syn('none');
  assert.equal(digest(none.capture), digest(base.capture), "'none' is the identity");
  const g = syn({ base: 'none', gain: 0.5 });
  for (let i = 0; i < n; i += 7) for (let j = 0; j < NLM; j++) { near(P(g.capture, i).world[j * 3 + 2], 0.5 * P(base.capture, i).world[j * 3 + 2], 1e-6, 'world z'); near(P(g.capture, i).img[j * 5 + 2], 0.5 * P(base.capture, i).img[j * 5 + 2], 1e-6, 'img z'); }
  const fl = syn({ base: 'none', flipRate: 1.5, flipDur: [0.2, 0.5] });
  let flips = 0;
  for (let i = 0; i < n; i++) {
    if (!(fl.faults.flip[i] & 1) || fl.faults.flip[i] & 4) continue;
    flips++;
    const w = P(fl.capture, i).world, w0 = P(base.capture, i).world, im = P(fl.capture, i).img, im0 = P(base.capture, i).img;
    for (const j of F.FAIL_CHAINS[0][2]) { near(w[j * 3 + 2] + w0[j * 3 + 2], 2 * w0[11 * 3 + 2], 1e-6, 'z_m + z_truth = 2 z_root'); assert.equal(im[j * 5], im0[j * 5]); assert.equal(im[j * 5 + 1], im0[j * 5 + 1]); }
  }
  assert.ok(flips > 0, 'some lArm flips happen');
  const mi = syn({ base: 'none', mirrorRate: 1, mirrorDur: [0.2, 0.4] }), sw = syn({ base: 'none', swapRate: 1, swapDur: [0.2, 0.4] });
  let mir = 0, swp = 0;
  for (let i = 0; i < n; i++) {
    const w0 = P(base.capture, i).world;
    if (mi.faults.mirror[i]) { mir++; const w = P(mi.capture, i).world; for (let j = 0; j < NLM; j++) { const m = S.LM_MIRROR[j]; near(w[j * 3], w0[m * 3], 1e-7); near(w[j * 3 + 2], -w0[m * 3 + 2], 1e-7); } }
    if (sw.faults.swap[i] && !sw.faults.mirror[i]) { swp++; const w = P(sw.capture, i).world; for (let j = 0; j < NLM; j++) { const m = j >= 23 ? S.LM_MIRROR[j] : j; near(w[j * 3], w0[m * 3], 1e-7); near(w[j * 3 + 2], w0[m * 3 + 2], 1e-7); } }
  }
  assert.ok(mir > 0 && swp > 0, 'mirror and swap frames occur');
  const dr = syn({ base: 'none', dropRate: 1, dropDur: [2, 6] });
  assert.ok(dr.faults.drop.some(x => x) && dr.capture.frames.some(fr => fr.poses.length === 0), 'poses are dropped');
  const res = processCapture(dr.capture, { hfov: 60 });
  assert.ok(res.n > 30 && res.landmarks.every(Number.isFinite), 'a capture with dropped poses still processes');
  const L0 = base.capture.frames[0].poses[0], M = 3000, subj = Array.from({ length: M }, () => ({ img: Float32Array.from(L0.img), world: Float32Array.from(L0.world) }));
  F.mediapipeFailures(subj, Array.from({ length: M }, () => Cs[0]), cam, { base: 'none', rho: 0.9, world: 0.01 }, 30, 77);
  const x = Float64Array.from(subj, s => s.world[15 * 3] - L0.world[15 * 3]), mu = x.reduce((a, b) => a + b) / M;
  let num = 0, den = 0;
  for (let i = 0; i < M; i++) { den += (x[i] - mu) ** 2; if (i + 1 < M) num += (x[i] - mu) * (x[i + 1] - mu); }
  near(num / den, 0.9, 0.03, 'AR(1) lag-1');
  const iv = syn({ base: 'none', flipRate: 0.6, interval: 1 });
  F.FAIL_CHAINS.forEach(([name, root, mem], b) => {
    const id = F.signRuns(Cs, root, mem[0]), seen = new Map();
    for (let i = 0; i < n; i++) if (id[i] >= 0) { const v = (iv.faults.flip[i] >> b) & 1; if (seen.has(id[i])) assert.equal(v, seen.get(id[i]), `${name} run ${id[i]}`); else seen.set(id[i], v); }
  });
  assert.ok(iv.faults.flip.some(x => x), 'interval flips occur');
});

// ============================== Solver and physics ==============================

function benchCase(family, k) {
  const c = bench.makeCase(family, bench.caseSeed('unit', k)), tracks = buildTracks(c.capture), series = trackSeries(c.capture, pickTrack(tracks, c.capture, {}), {}), n = series.n, TC = new Float64Array(n * NLM * 3);
  for (let i = 0; i < n; i++) for (let j = 0; j < NLM; j++) TC.set(toCam(c.cam, S.lmAt(c.truth[series.first + i], j)), (i * NLM + j) * 3);
  return { c, series, n, TC, cp: cameraPoints(series, { hfov: c.settings.hfov, imageFit: 0.5 }) };
}
const relDepthErr = (f, P) => {
  const out = [];
  for (let i = 0; i < f.n; i++) {
    const z = j => -P[(i * NLM + j) * 3 + 2], hz = (z(23) + z(24)) / 2, th = (pt(f.TC, i, 23)[2] + pt(f.TC, i, 24)[2]) / 2;
    for (const j of D.RAY_JOINTS) out.push(Math.abs(z(j) - hz - (pt(f.TC, i, j)[2] - th)));
  }
  return out;
};
const mean = a => a.reduce((x, y) => x + y, 0) / a.length;
test('depth 15: solveDepth with true signs beats first-pass depth under typical tracker failures', { skip: !bench && 'depthbench.js not available' }, () => {
  const fails = [];
  for (const family of ['across', 'toward', 'pivot']) {
    const f = benchCase(family, 1), d = D.depthPoints(f.series, f.cp, { fps: f.series.fps, decide: truthDecide(f.TC) }), nw = relDepthErr(f, d.P), bs = relDepthErr(f, f.cp.P);
    const mN = mean(nw), mB = mean(bs), pN = quant(nw, 0.99), pB = quant(bs, 0.99);
    if (!d.ok) fails.push(`${family}: solver not ok`);
    if (!(mN <= 0.045 && mN <= 0.6 * mB)) fails.push(`${family}: mean ${(100 * mN).toFixed(1)} cm vs baseline ${(100 * mB).toFixed(1)} cm`);
    if (!(pN <= 0.18 && pN <= 0.6 * pB)) fails.push(`${family}: p99 ${(100 * pN).toFixed(1)} cm vs baseline ${(100 * pB).toFixed(1)} cm`);
  }
  assert.deepEqual(fails, []);
});

test('depth 16: depthPoints with decided signs cuts the sign error rate below 5% and below baseline', { skip: !bench && 'depthbench.js not available' }, () => {
  const fails = [];
  for (const family of ['toward', 'reachFacing']) {
    const f = benchCase(family, 1), d = D.depthPoints(f.series, f.cp, { fps: f.series.fps }), sN = bench.boneSignError(f.TC, storedToCam(d.P, f.n)).rate, sB = bench.boneSignError(f.TC, storedToCam(f.cp.P, f.n)).rate;
    if (!(sN < 0.05 && sN < sB)) fails.push(`${family}: SER0.2 ${(100 * sN).toFixed(1)}% vs baseline ${(100 * sB).toFixed(1)}%`);
  }
  assert.deepEqual(fails, []);
});

test('depth 17: gravityScale recovers the scale of a ballistic flight', { skip: !bench?.ballisticFlight && 'depthbench.js not available' }, () => {
  const jt = () => bench.ballisticFlight(truthSkel, record([{ seconds: 0.8, input: {} }, { seconds: 1.6, input: { jump: true } }], [0, 0, -1])), take = jt(), Ls = clipLandmarks(truthSkel, take), n = take.n;
  for (const s of [0.8, 1.3]) {
    const P = new Float64Array(n * NLM * 3);
    Ls.forEach((L, i) => { for (let q = 0; q < NLM * 3; q++) P[i * NLM * 3 + q] = L[q] * s; });
    const g = D.gravityScale(P, n, 30);
    assert.ok(g?.ok, `flight found and fitted for shrink ${s}`);
    assert.ok(Math.abs(g.scale * s - 1) < 0.01, `truth landmarks shrunk by ${s}: scale ${g.scale}`);
  }
  for (const hfov of [55, 72]) {
    const c = lookAtCamera([0.3, 1.45, 5], [0, 0.9, -1], { width: 1280, height: 720, hfov }), f = prepare(take, { cam: c, hfov: 60, synth: { noiseImg: 0.001, noiseWorld: 0.01 } }), d = D.depthPoints(f.series, f.cp, { fps: 30 });
    const fl = floorFromPoints(d.P, f.n, 30), g = D.gravityScale(applyFloor(d.P, f.n, floorTransform(fl)), f.n, 30), TL = truthLengths(f.TC, f.n);
    assert.ok(g?.ok, `hfov ${hfov}: gravity fit ok`);
    const truthLeg = median(Array.from({ length: f.n }, (_, i) => TL[i * D.EDGES.length + 7] + TL[i * D.EDGES.length + 8])), recLeg = d.lengths.L[7] + d.lengths.L[8], err = g.scale * recLeg / truthLeg - 1;
    assert.ok(Math.abs(err) <= 0.04, `hfov ${hfov}: scale error ${err}`);
  }
});

test('depth 18: metric identities and the camera-frame round trip', () => {
  const f = prepare(TAKES.pipe3()), { capture, TC, n } = f, res = processCapture(capture, { hfov: 60, depth: false }), W = C.finalLandmarks(res.skel, res.take, res.n), RC = C.cameraLandmarks(res.camera, W, res.n);
  assert.equal(res.n, n);
  let s = 0;
  for (let i = 0; i < n; i++) for (const j of C.EVAL_JOINTS) s += len(sub(pt(RC, i, j), pt(TC, i, j)));
  assert.ok(s / (n * C.EVAL_JOINTS.length) < 0.04, `toCameraFrame round trip ${s / (n * C.EVAL_JOINTS.length)} m`);
  const shift = A => A.map((x, q) => x + [0.3, -0.1, 0.7][q % 3]), sh = C.shapeErrors(TC, shift(TC), n);
  assert.ok(Math.abs(sh.s - 1) < 1e-12 && sh.err.every(e => Number.isNaN(e) || e < 1e-9), 'a constant shift has zero shape error');
  if (!bench?.novelViewError) return;
  const TW = new Float64Array(n * NLM * 3), th = [0, 45, 90, 135, 180];
  for (let i = 0; i < n; i++) TW.set(f.truth[f.series.first + i], i * NLM * 3);
  const id = bench.novelViewError(TW, TW, cam, { thetas: th, mode: 'shape' });
  for (const t of th) assert.ok(id[t].px.mean < 1e-9 && id[t].cm.mean < 1e-9, `truth as reconstruction at ${t}°: ${id[t].px.mean} px`);
  for (const mode of ['shape', 'traj']) { const e = bench.novelViewError(TW, shift(TW), cam, { thetas: [0, 90], mode }); assert.ok(e[0].px.mean < 1e-6 && e[90].cm.mean < 1e-6, `constant root shift in ${mode} mode`); }
  const MC = Float64Array.from(TC), valid = new Uint8Array(n), dl = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const c = pt(TC, i, 15), d = unit(c), b = D.branch(d, pt(TC, i, 13), len(sub(c, pt(TC, i, 13)))), t2 = 2 * b.t0 - len(c);
    MC.set([d[0] * t2, d[1] * t2, d[2] * t2], (i * NLM + 15) * 3); dl[i] = Math.abs(len(c) - t2); valid[i] = dl[i] > 0.05 ? 1 : 0;
  }
  assert.ok(valid.some(v => v), 'the wrist leaves the image plane');
  const MW = bench.camToWorld(cam, MC), mv = bench.novelViewError(TW, MW, cam, { thetas: [0, 90], mode: 'raw', joints: [15], valid });
  assert.ok(mv[0].px.mean < 1e-6, `the other root is invisible at 0°: ${mv[0].px.mean} px`);
  let want = 0, m = 0;
  for (let i = 0; i < n; i++) if (valid[i]) { const B = bench.orbitCamera(cam, mid(pt(TW, i, 23), pt(TW, i, 24)), 90); want += B.focal * dl[i] / toCam(B, pt(TW, i, 15))[2]; m++; }
  assert.ok(Math.abs(mv[90].px.mean / (want / m) - 1) <= 0.05, `90°: ${mv[90].px.mean} px vs f·Δ/D ${want / m}`);
});

// ============================== Pipeline ==============================

const pipeScenario = once(() => synthCapture(truthSkel, TAKES.pipe3(), cam, { rand: rng(3), noiseImg: 0.003, noiseWorld: 0.015, occlusion: true }).capture);
const bitHash = res => { const h = createHash('sha256'); for (const a of [res.landmarks, res.take.root, res.take.rot]) h.update(Buffer.from(a.buffer, a.byteOffset, a.byteLength)); return h.digest('hex'); };
test('depth 19: depth off reproduces the pre-change pipeline bit for bit', () => {
  assert.equal(bitHash(processCapture(pipeScenario(), { hfov: 60 })), BIT_DEPTH_OFF);
  assert.equal(bitHash(processCapture(pipeScenario(), { hfov: 60, depth: false })), BIT_DEPTH_OFF);
});

function mpjpe(res, take, c = cam) {
  const TW = takeWorld(res.skel, res.take), tmp = { pos: new Float64Array(S.NB * 3), q: new Float64Array(S.NB * 4) }, B = S.B;
  const joints = [B.Hips, B.LeftLeg, B.RightLeg, B.LeftFoot, B.RightFoot, B.LeftForeArm, B.RightForeArm, B.LeftHand, B.RightHand, B.Head];
  let sum = 0, cnt = 0;
  for (let i = 0; i < res.n; i++) {
    const f = res.first + i;
    S.fk(truthSkel, S.getV(take.root, f), take.rot.subarray(f * S.NB * 4, (f + 1) * S.NB * 4), tmp);
    for (const j of joints) { sum += len(sub(toReconstructionFrame(c, S.getV(tmp.pos, j)), S.getV(worldAt(TW, i).pos, j))); cnt++; }
  }
  return sum / cnt;
}
test('depth 20a: depth on keeps the three pipeline scenarios within their thresholds', { skip: !PIPE_DEPTH && 'pipeline has no depth setting yet' }, () => {
  const r3 = processCapture(pipeScenario(), { hfov: 60, depth: true });
  assert.ok(mpjpe(r3, TAKES.pipe3()) < 0.06, `joint error ${mpjpe(r3, TAKES.pipe3())}`);
  near(r3.skel.dims.height, truthSkel.dims.height, 0.05, 'body height');
  near(r3.library.walk.speed, 1.25, 0.08, 'walk speed');
  const wide = lookAtCamera([0.5, 1.6, 11], [0.5, 0.9, -0.5], { width: 1920, height: 1080, hfov: 72 }), wr = record([{ seconds: 1.5, input: {} }, { seconds: 2, input: { right: true } }, { seconds: 2.5, input: { right: true, run: true } }, { seconds: 1.5, input: {} }], [-5.5, 0, -0.5]);
  const r5 = processCapture(synthCapture(truthSkel, wr, wide, { rand: rng(5), noiseImg: 0.0015, noiseWorld: 0.015, occlusion: true }).capture, { hfov: 72, depth: true });
  near(r5.library.walk.speed, 1.25, 0.12, 'walk'); near(r5.library.run.speed, 3.45, 0.25, 'run');
  const jt = record([{ seconds: 1, input: {} }, { seconds: 1.6, input: { jump: true } }, { seconds: 1, input: {} }], [0, 0, -1]), rj = processCapture(synthCapture(truthSkel, jt, cam, { rand: rng(7), noiseImg: 0.003, noiseWorld: 0.015, occlusion: true }).capture, { hfov: 60, depth: true });
  assert.ok(mpjpe(rj, jt) < 0.06, `jump joint error ${mpjpe(rj, jt)}`);
  for (const r of [r3, r5, rj]) {
    const rep = r.report.depth;
    assert.ok(rep?.on && Number.isFinite(rep.kappa) && typeof rep.line === 'string', 'report.depth');
    assert.ok(r.landmarks.every(Number.isFinite) && r.take.rot.every(Number.isFinite));
    if (r.cert) { assert.equal(r.cert.state.length, r.n * NLM); assert.equal(r.cert.radius.length, r.n * NLM); assert.equal(r.cert.sigma.length, r.n * NLM); assert.equal(r.cert.bones.sign.length, r.n * NE); }
  }
});

// a certificate context: from the patched pipeline when it has a depth stage, otherwise assembled from a depth-off run plus depthPoints
function certCase(seed) {
  const capture = synthCapture(truthSkel, TAKES.pipe3(), cam, { rand: rng(seed), noiseImg: 0.003, noiseWorld: 0.015, occlusion: true }).capture;
  const res = processCapture(capture, { hfov: 60, depth: PIPE_DEPTH }), depth = res.depth ?? D.depthPoints(res.series, cameraPoints(res.series, { hfov: 60, imageFit: 0.5 }), { fps: res.fps });
  const ctx = { series: res.series, depth, skel: res.skel, take: res.take, camera: res.camera, fps: res.fps, n: res.n, scaleSource: 'mediapipe' }, n = res.n, TC = new Float64Array(n * NLM * 3);
  const truth = synthCapture(truthSkel, TAKES.pipe3(), cam, {}).truth;
  for (let i = 0; i < n; i++) for (let j = 0; j < NLM; j++) TC.set(toCam(cam, S.lmAt(truth[res.first + i], j)), (i * NLM + j) * 3);
  const err = C.shapeErrors(TC, C.cameraLandmarks(res.camera, C.finalLandmarks(res.skel, res.take, n), n), n).err;
  return { res, ctx, n, err };
}
test('depth 20b: the certificate is well formed, grey under the placeholder, and a tiny in-test calibration keeps green joints inside 2× their radius', () => {
  const a = certCase(3), cp = a.res.cert ?? C.certifyTake(a.ctx), n = a.n;
  assert.equal(cp.state.length, n * NLM); assert.equal(cp.reason.length, n * NLM); assert.equal(cp.radius.length, n * NLM); assert.equal(cp.sigma.length, n * NLM);
  for (const k of ['sign', 'certified', 'moot', 'pWrongModel']) assert.equal(cp.bones[k].length, n * NE, k);
  assert.equal(cp.domain.values.length, C.DOMAIN_STATS.length);
  assert.equal(typeof cp.summary.line, 'string');
  if (DEPTH_CAL.quick) {
    assert.ok(cp.state.every(s => s === C.STATE.GREY) && cp.status === 'unverified', 'placeholder: all grey');
    assert.equal(cp.summary.line, 'Depth check: not calibrated (developer build), so nothing is verified.');
  }
  const groups = Object.keys(C.CERT_GROUPS), open = { ...DEPTH_CAL, quick: false, calibId: 'test', sign: { ...DEPTH_CAL.sign, lambda: 5 }, radius: { ...DEPTH_CAL.radius, groups: Object.fromEntries(groups.map(g => [g, { lambda: 1, p: 0, risk: 0 }])) } };
  const b = certCase(5), cb = C.certifyTake(b.ctx, open), lam = {};
  for (const g of groups) {
    let m = 0;
    for (let i = 0; i < b.n; i++) for (const j of C.CERT_GROUPS[g]) { const q = i * NLM + j; if (cb.state[q] === C.STATE.GREEN) m = Math.max(m, b.err[q] / (cb.sigma[q] + open.sigma0)); }
    lam[g] = Math.max(m, 0.5);
  }
  const cal = { ...open, radius: { ...open.radius, groups: Object.fromEntries(groups.map(g => [g, { lambda: lam[g], p: 0, risk: 0 }])) } }, ca = C.certifyTake(a.ctx, cal);
  let green = 0, bad = 0;
  for (let i = 0; i < n; i++) for (const j of C.EVAL_JOINTS) { const q = i * NLM + j; if (ca.state[q] !== C.STATE.GREEN) continue; green++; assert.ok(Number.isFinite(ca.radius[q])); if (a.err[q] > 2 * ca.radius[q]) bad++; }
  assert.ok(green > 0, 'the in-test calibration must certify some joint-frames, or the check below is vacuous');
  assert.equal(bad, 0, `${bad} of ${green} green joint-frames exceed 2× their radius`);
  assert.ok(ca.summary.line.startsWith('Depth check:'), ca.summary.line);
});

test('depth 21: the domain check accepts in-range takes and rejects shaky tracking or a wrong gravity with a plain reason', () => {
  const K = C.DOMAIN_STATS.length, mid = [0.05, 1.2, 0.3, 0.75, 0.01, 0.8, 0.004, 0.1, 0.05, 0.1, 0.1];
  const cal = { domain: { lo: mid.map((x, k) => C.DOMAIN_STATS[k][1] === 1 ? -Infinity : x / 2), hi: mid.map((x, k) => C.DOMAIN_STATS[k][1] === -1 ? Infinity : x * 1.5) } };
  assert.equal(mid.length, K);
  assert.deepEqual(C.domainCheck(Float64Array.from(mid), cal), { pass: true, which: -1, plain: '' });
  const jit = Float64Array.from(mid); jit[6] *= 2;
  const rj = C.domainCheck(jit, cal);
  assert.equal(rj.pass, false); assert.equal(rj.which, 6); assert.equal(rj.plain, 'tracking is very shaky');
  const gv = Float64Array.from(mid); gv[10] = 0.44;
  const rg = C.domainCheck(gv, cal);
  assert.equal(rg.pass, false); assert.equal(rg.which, 10); assert.match(rg.plain, /jump does not match gravity/);
  const na = Float64Array.from(mid); na[10] = NaN;
  assert.equal(C.domainCheck(na, cal).pass, true, 'a statistic that does not apply is skipped');
});

test('depth 22: the calibration table is fresh, depth runs are deterministic and the new files never call Math.random', () => {
  const files = HASHED.map(f => url(`../mocap/${f}.js`)).filter(u => existsSync(u)), h = createHash('sha256');
  for (const u of files) h.update(readFileSync(u));
  assert.ok(DEPTH_CAL.quick || DEPTH_CAL.codeHash === h.digest('hex'), 'rerun tools/depth_calibrate.mjs');
  const f = FX.towardTyp(), a = D.depthPoints(f.series, f.cp, { fps: 30 }), b = D.depthPoints(f.series, f.cp, { fps: 30 });
  assert.ok(Buffer.from(a.P.buffer).equals(Buffer.from(b.P.buffer)), 'bitwise-identical depth output');
  for (const p of NEW_FILES) {
    if (!existsSync(url(p))) continue;
    const src = readFileSync(url(p), 'utf8');
    assert.ok(!/Math\.random/.test(src), `${p} calls Math.random`);
    assert.ok(!/from ['"]node:|\bprocess\./.test(src), `${p} uses Node APIs`);
  }
});
