import * as M from './math.js';
import { NB, NLM, SIDES, DEFAULT_DIMS, buildSkeleton, standLocal, fk, getV, getQ, setQ, landmarksFromPose, soleTrack, lmAt } from './skeleton.js';
import { gaitClip, idleClip, jumpClip, defaultSpeed } from './gait.js';
import { createController, recordTake } from './controller.js';
import { lookAtCamera, synthCapture, toCam } from './synth.js';
import { processCapture } from './pipeline.js';
import { TREE, NE, COM_W } from './depth.js';
import { EVAL_JOINTS, CERT_GROUPS, STATE, finalLandmarks, cameraLandmarks, shapeErrors, trueSigns, intervalRecords } from './depthcert.js';
import { mulberry32 } from './depthstats.js';
import { DEPTH_CAL } from './depth_calibration.js';
import { FAIL_CHAINS } from './mpfail.js';

export const SPLITS = Object.freeze({ unit: [1, 999], sign: [1000000, 1499999], radius: [1500000, 1999999], dev: [2000000, 2999999], domain: [3000000, 3999999], test: [9000000, 9999999] });
export const caseSeed = (split, k) => { const [a, b] = SPLITS[split]; if (!(k >= 0) || a + k > b) throw new Error('seed range'); return a + k; };
export const HELD_OUT = Object.freeze({ families: ['circle', 'reach45', 'walkReachWalk'], profiles: ['hard', 'flipInterval', 'neckerProfile', 'legSwapCross', 'farHallucinate', 'drift', 'compress', 'dropout'] });
export const overlaps = (r, s) => r[0] <= s[1] && s[0] <= r[1];
export function assertDisjoint(ranges) {
  const e = Object.entries(ranges);
  for (let i = 0; i < e.length; i++) for (let j = i + 1; j < e.length; j++) if (overlaps(e[i][1], e[j][1])) throw new Error(`seed ranges overlap: ${e[i][0]} ${e[i][1]} vs ${e[j][0]} ${e[j][1]}`);
}
assertDisjoint(SPLITS);
export const CODE_FILES = Object.freeze(['landmarks', 'clean', 'ground', 'skeleton', 'solve', 'pipeline', 'depth', 'depthsign', 'depthstats', 'depthcert', 'mpfail', 'depthbench', 'synth', 'gait', 'controller'].map(f => `mocap/${f}.js`));

const U = (r, a, b) => a + (b - a) * r(), PI = Math.PI, DEG = PI / 180, GROUPS = Object.keys(CERT_GROUPS);
const GROUP_OF = Int8Array.from({ length: NLM }, (_, j) => GROUPS.findIndex(g => CERT_GROUPS[g].includes(j)));
const PARENT_EDGE = Int8Array.from({ length: NLM }, (_, j) => j === 23 ? 0 : TREE.findIndex(t => t[1] === j));

// ---- clips ----
export function reachClip(skel, o = {}) {
  const fps = o.fps ?? 30, base = o.local ?? standLocal(), keys = o.keys ?? [
    { t: 0, L: null, R: null }, { t: 0.8, L: [0, 0, 1], R: null, eL: 0.1 }, { t: 1.8, L: [0, -0.3, 1], R: [0, 0.2, 1], eL: 0.6, eR: 0.1 },
    { t: 2.8, L: [-0.6, 0, 1], R: [0.4, 0, 1], eL: 1.2, eR: 0.3 }, { t: 3.8, L: null, R: [0, 0, 1], eR: 1.4 }, { t: 4.6, L: null, R: null }];
  const total = keys[keys.length - 1].t, n = Math.round(total * fps) + 1, y0 = o.y ?? skel.rest[0][1] - 0.005, root = new Float64Array(n * 3), rot = new Float64Array(n * NB * 4);
  const W0 = fk(skel, [0, 0, 0], base), rd = {};
  for (const side of ['L', 'R']) rd[side] = M.norm(M.sub(getV(W0.pos, SIDES[side].fore), getV(W0.pos, SIDES[side].arm)));
  for (let k = 0; k < n; k++) {
    const t = k / fps;
    let a = 0;
    while (a < keys.length - 2 && keys[a + 1].t <= t) a++;
    const k0 = keys[a], k1 = keys[a + 1], u = M.smoothstep(k0.t, k1.t, t), local = Float64Array.from(base);
    for (const side of ['L', 'R']) {
      const S = SIDES[side], d0 = k0[side] ? M.norm(k0[side]) : rd[side], d1 = k1[side] ? M.norm(k1[side]) : rd[side];
      setQ(local, S.arm, M.qarc([S.s, 0, 0], M.norm(M.vlerp(d0, d1, u))));
      setQ(local, S.fore, M.qaxis([0, -S.s, 0], M.lerp(k0['e' + side] ?? 0.2, k1['e' + side] ?? 0.2, u)));
    }
    root.set([0, y0, 0], k * 3); rot.set(local, k * NB * 4);
  }
  return { name: 'Reach', kind: 'action', loop: false, fps, n, root, rot, rootMotion: { end: [0, 0, 0], turn: 0 } };
}

export function placeClip(clip, o = {}) {
  const n = clip.n, root = new Float64Array(n * 3), rot = Float64Array.from(clip.rot), p = o.pos ?? [0, 0, 0];
  for (let k = 0; k < n; k++) {
    const R = M.qyaw((o.yaw ?? 0) + (o.spin ?? 0) * k / clip.fps), off = M.qrot(R, [clip.root[k * 3], 0, clip.root[k * 3 + 2]]);
    root.set([p[0] + off[0], clip.root[k * 3 + 1], p[2] + off[2]], k * 3);
    let q = M.qmul(R, getQ(clip.rot, k * NB));
    if (k > 0) q = M.qalign(q, getQ(rot, (k - 1) * NB));
    setQ(rot, k * NB, q);
  }
  return { n, fps: clip.fps, root, rot };
}

export function truthLandmarks(skel, take) {
  const out = [], W = { pos: new Float64Array(NB * 3), q: new Float64Array(NB * 4) };
  for (let i = 0; i < take.n; i++) { fk(skel, getV(take.root, i), take.rot.subarray(i * NB * 4, (i + 1) * NB * 4), W); out.push(landmarksFromPose(skel, W)); }
  return out;
}

const comOf = L => { const c = [0, 0, 0]; for (const [js, w] of COM_W) for (let k = 0; k < 3; k++) { let s = 0; for (const j of js) s += L[j * 3 + k]; c[k] += w * s / js.length; } return c; };

export function ballisticFlight(skel, take) {
  const n = take.n, fps = take.fps ?? 30, W = { pos: new Float64Array(NB * 3), q: new Float64Array(NB * 4) }, up = new Uint8Array(n), com = [], root = Float64Array.from(take.root), flights = [];
  for (let i = 0; i < n; i++) {
    fk(skel, getV(take.root, i), take.rot.subarray(i * NB * 4, (i + 1) * NB * 4), W);
    com.push(comOf(landmarksFromPose(skel, W)));
    up[i] = ['L', 'R'].every(s => { const t = soleTrack(skel, W, s); return Math.min(t.heel[1], t.ball[1], t.toe[1]) > 0.02; }) ? 1 : 0;
  }
  for (let i = 0; i < n; i++) {
    if (!up[i]) continue;
    let b = i;
    while (b + 1 < n && up[b + 1]) b++;
    const p = Math.max(0, i - 1), q = Math.min(n - 1, b + 1), T = (q - p) / fps;
    if (T > 0) {
      const c0 = com[p], c1 = com[q], v = (c1[1] - c0[1]) / T + 9.81 * T / 2;
      for (let k = i; k <= b; k++) {
        const tau = (k - p) / fps, s = tau / T, tgt = [M.lerp(c0[0], c1[0], s), c0[1] + v * tau - 9.81 * tau * tau / 2, M.lerp(c0[2], c1[2], s)];
        for (let c = 0; c < 3; c++) root[k * 3 + c] += tgt[c] - com[k][c];
      }
      flights.push([i, b]);
    }
    i = b;
  }
  return { ...take, root, flights };
}

export function framing(truthL, cam, o = {}) {
  const lo = o.lo ?? 0.01, hi = o.hi ?? 0.99, zMin = o.zMin ?? 1;
  let ok = 0;
  for (const L of truthL) {
    let good = true;
    for (let j = 0; j < NLM && good; j++) {
      const c = toCam(cam, lmAt(L, j)), x = (c[0] / c[2] * cam.focal + cam.width / 2) / cam.width, y = (c[1] / c[2] * cam.focal + cam.height / 2) / cam.height;
      good = c[2] >= zMin && x >= lo && x <= hi && y >= lo && y <= hi;
    }
    ok += good;
  }
  return truthL.length ? ok / truthL.length : 0;
}

export function randomSubject(rand) {
  const g = U(rand, 0.88, 1.12), dims = {};
  for (const k of Object.keys(DEFAULT_DIMS)) dims[k] = DEFAULT_DIMS[k] * g * U(rand, 0.95, 1.05);
  return buildSkeleton(dims);
}

export function randomCamera(rand, center, o = {}) {
  const D = o.run ? U(rand, 8, 11) : U(rand, 4.5, 8), az = U(rand, -25, 25) * DEG, h = U(rand, 0.9, 1.9), fov = o.portrait ? U(rand, 36, 46) : U(rand, 55, 72);
  const tgt = [center[0], center[1] ?? 0.9, center[2]];
  return lookAtCamera([tgt[0] + D * Math.sin(az), h, tgt[2] + D * Math.cos(az)], tgt, o.portrait ? { width: 720, height: 1280, hfov: o.hfov ?? fov } : { width: 1280, height: 720, hfov: o.hfov ?? fov });
}

// ---- families ----
function subjectLib(skel, rand, fps) {
  return { idle: idleClip(skel, { fps }), walk: gaitClip(skel, { kind: 'walk', fps, speed: defaultSpeed(skel, 'walk') * U(rand, 0.9, 1.1) }),
    run: gaitClip(skel, { kind: 'run', fps, speed: defaultSpeed(skel, 'run') * U(rand, 0.9, 1.1) }), jump: jumpClip(skel, { fps }), actions: [], take: null };
}
function drive(lib, yaw, steps, fps, actions) {
  const ctl = createController(actions ? { ...lib, actions } : lib, { yaw });
  ctl.pos = [0, 0, 0];
  return recordTake(ctl, steps.map(s => ({ ...s, input: { camYaw: PI, ...s.input } })), fps);
}
function centerOf(take) {
  let x0 = Infinity, x1 = -Infinity, z0 = Infinity, z1 = -Infinity;
  for (let i = 0; i < take.n; i++) { const x = take.root[i * 3], z = take.root[i * 3 + 2]; x0 = Math.min(x0, x); x1 = Math.max(x1, x); z0 = Math.min(z0, z); z1 = Math.max(z1, z); }
  return [(x0 + x1) / 2, 0.9, (z0 + z1) / 2];
}
const sgn = rand => rand() < 0.5 ? -1 : 1;
function reachKeys(rand, o = {}) {
  const keys = [{ t: 0, L: null, R: null }], m = 4 + Math.floor(rand() * 3), dir = () => rand() < 0.25 ? null : [U(rand, -0.8, 0.8), U(rand, -0.6, 0.8), U(rand, 0.3, 1)];
  let t = U(rand, 0.4, 0.9);
  for (let k = 0; k < m; k++) { keys.push({ t, L: dir(), R: dir(), eL: U(rand, 0.05, 1.4), eR: U(rand, 0.05, 1.4) }); t += U(rand, 0.7, 1.2); }
  keys.push({ t: t + 0.3, L: null, R: null });
  return keys;
}
function walkSteps(rand, input, a, b) { return [{ seconds: U(rand, 0.8, 1.4), input: {} }, { seconds: U(rand, a, b), input }, { seconds: U(rand, 0.8, 1.4), input: {} }]; }
const ctlFamily = (f, opt = {}) => ({ ...opt, build(skel, rand, o = {}) {
  const fps = o.fps ?? 30, lib = subjectLib(skel, rand, fps), { yaw, steps, actions } = f(rand, skel, lib, fps, o), take = drive(lib, yaw, steps, fps, actions);
  return { take, center: centerOf(take) };
} });
const clipFamily = f => ({ build(skel, rand, o = {}) { const fps = o.fps ?? 30, take = f(rand, skel, fps, o); return { take, center: centerOf(take) }; } });
const idleFor = (skel, rand, fps, o) => idleClip(skel, { fps, seconds: o.seconds ?? U(rand, 4, 7) });

export const FAMILIES = Object.freeze({
  across: ctlFamily(rand => { const s = sgn(rand); return { yaw: s * PI / 2, steps: walkSteps(rand, s > 0 ? { right: true } : { left: true }, 2.2, 3.2) }; }),
  toward: ctlFamily(rand => ({ yaw: 0, steps: walkSteps(rand, { back: true }, 2, 3) })),
  away: ctlFamily(rand => ({ yaw: PI, steps: walkSteps(rand, { forward: true }, 2, 3) })),
  diagonal: ctlFamily(rand => {
    const x = sgn(rand), z = sgn(rand);
    return { yaw: Math.atan2(x, z), steps: walkSteps(rand, { ...(x > 0 ? { right: true } : { left: true }), ...(z > 0 ? { back: true } : { forward: true }) }, 2.2, 3.2) };
  }),
  run: ctlFamily(rand => {
    const s = sgn(rand), d = s > 0 ? { right: true } : { left: true };
    return { yaw: s * PI / 2, steps: [{ seconds: U(rand, 0.6, 1), input: {} }, { seconds: 0.8, input: d }, { seconds: U(rand, 1.2, 2), input: { ...d, run: true } }, { seconds: 0.8, input: d }, { seconds: U(rand, 0.6, 1), input: {} }] };
  }, { run: true }),
  jump: { build(skel, rand, o = {}) {
    const fps = o.fps ?? 30, lib = subjectLib(skel, rand, fps), yaw = U(rand, -PI, PI), jumps = rand() < 0.4 ? 2 : 1, steps = [{ seconds: U(rand, 1.2, 1.8), input: {} }];
    for (let k = 0; k < jumps; k++) steps.push({ seconds: 1.8, input: { jump: true } }, { seconds: U(rand, 1, 1.5), input: {} });
    const take = ballisticFlight(skel, drive(lib, yaw, steps, fps));
    return { take, center: centerOf(take) };
  } },
  pivot: ctlFamily(rand => {
    const ax = rand() < 0.5, s = sgn(rand), a = ax ? (s > 0 ? { right: true } : { left: true }) : (s > 0 ? { back: true } : { forward: true }), b = ax ? (s > 0 ? { left: true } : { right: true }) : (s > 0 ? { forward: true } : { back: true });
    return { yaw: ax ? s * PI / 2 : s > 0 ? 0 : PI, steps: [{ seconds: U(rand, 0.6, 1), input: {} }, { seconds: U(rand, 1.5, 2.2), input: a }, { seconds: U(rand, 1.5, 2.2), input: b }, { seconds: U(rand, 0.6, 1), input: {} }] };
  }),
  spin: clipFamily((rand, skel, fps, o) => placeClip(idleFor(skel, rand, fps, o), { yaw: U(rand, 0, 2 * PI), spin: sgn(rand) * U(rand, 0.8, 2) })),
  reachFacing: clipFamily((rand, skel, fps) => placeClip(reachClip(skel, { fps, keys: reachKeys(rand) }), { yaw: U(rand, -0.35, 0.35) })),
  reachBack: clipFamily((rand, skel, fps) => placeClip(reachClip(skel, { fps, keys: reachKeys(rand) }), { yaw: PI + U(rand, -0.35, 0.35) })),
  backIdle: clipFamily((rand, skel, fps, o) => placeClip(idleFor(skel, rand, fps, o), { yaw: PI + U(rand, -0.4, 0.4) })),
  circle: ctlFamily((rand, skel, lib, fps, o) => {
    const r = U(rand, 1.2, 1.8), s = sgn(rand), secs = o.seconds ?? U(rand, 6, 9), w = s * lib.walk.speed / r, a0 = U(rand, -PI, PI), dt = 0.2, steps = [{ seconds: 0.6, input: {} }];
    for (let t = 0; t < secs - 0.6; t += dt) steps.push({ seconds: dt, input: { forward: true, camYaw: a0 + w * t } });
    return { yaw: a0, steps };
  }),
  reach45: clipFamily((rand, skel, fps) => placeClip(reachClip(skel, { fps, keys: reachKeys(rand) }), { yaw: sgn(rand) * PI / 4 + U(rand, -0.15, 0.15) })),
  walkReachWalk: ctlFamily((rand, skel, lib, fps) => {
    const s = sgn(rand), d = s > 0 ? { right: true } : { left: true }, reach = reachClip(skel, { fps, keys: reachKeys(rand) });
    return { yaw: s * PI / 2, actions: [reach], steps: [{ seconds: U(rand, 0.5, 0.9), input: {} }, { seconds: U(rand, 1.5, 2), input: d }, { seconds: (reach.n - 1) / fps + 0.1, input: { action: true } }, { seconds: U(rand, 1.5, 2), input: d }, { seconds: U(rand, 0.5, 0.9), input: {} }] };
  }),
  reachHold: clipFamily((rand, skel, fps) => {
    const T = U(rand, 4, 6), e = U(rand, 0.03, 0.12), hold = { L: [0, 0, 1], R: [0, 0, 1], eL: e, eR: e };
    return placeClip(reachClip(skel, { fps, keys: [{ t: 0, L: null, R: null }, { t: 0.8, ...hold }, { t: T, ...hold }] }), { yaw: U(rand, -0.2, 0.2) });
  }),
});
export const IN_DIST = Object.freeze(Object.keys(FAMILIES).filter(f => !HELD_OUT.families.includes(f) && f !== 'reachHold'));
export const ATTACK_FAMILIES = Object.freeze(['reachHold']);

export function noiseFor(profile, seed, intensity) {
  return profile === 'none' || !profile ? { noiseImg: 0.003, noiseWorld: 0.015, occlusion: true }
    : { noiseImg: 0, noiseWorld: 0, occlusion: true, mp: { base: profile, intensity: intensity ?? 1 }, mpSeed: (seed ^ 0xFA11) >>> 0 };
}

export function makeCase(family, seed, o = {}) {
  const F = FAMILIES[family];
  if (!F) throw new Error('unknown family ' + family);
  const profile = o.profile ?? 'typical', fpsTrue = o.fpsTrue ?? 30, sr = mulberry32((seed ^ 0x5CE7E) >>> 0), skel = o.skel ?? randomSubject(sr);
  const { take, center } = F.build(skel, sr, { fps: fpsTrue, seconds: o.seconds });
  take.fps = fpsTrue;
  const TL = truthLandmarks(skel, take);
  let cam = null;
  for (let k = 0; k < 8 && !cam; k++) { const c = randomCamera(sr, center, { run: F.run, portrait: o.portrait, hfov: o.hfov }); if (framing(TL, c) >= 0.95) cam = c; }
  if (!cam) throw new Error('unframeable');
  const { capture, truth, faults } = synthCapture(skel, take, cam, { rand: mulberry32((seed ^ 0x51ED) >>> 0), ...noiseFor(profile, seed, o.intensity) });
  if (fpsTrue !== 30) { capture.video.fps = 30; capture.video.duration = take.n / 30; capture.frames.forEach((f, i) => { f.t = i / 30; }); }
  const settings = { hfov: o.hfovGuess ?? 60, height: o.knownHeight ? skel.dims.height : null };
  return { family, seed, profile, intensity: o.intensity ?? (profile === 'none' ? 0 : 1), skel, take, cam, capture, truth, faults: faults ?? null, settings, center, fpsTrue, portrait: !!o.portrait };
}

export function drawCase(seed) {
  const r = mulberry32((seed ^ 0xD1CE) >>> 0), family = IN_DIST[Math.floor(r() * IN_DIST.length)], typ = r() < 0.85, intensity = r();
  return { family, seed, o: { profile: typ ? 'typical' : 'none', intensity: typ ? intensity : 0, hfov: U(r, 55, 72), hfovGuess: 60, knownHeight: r() < 0.5 } };
}
export const makeDrawnCase = seed => { const d = drawCase(seed); return makeCase(d.family, seed, d.o); };

// ---- metrics ----
export const truthCamera = (c, i) => { const L = c.truth[i], out = new Float64Array(NLM * 3); for (let j = 0; j < NLM; j++) out.set(toCam(c.cam, lmAt(L, j)), j * 3); return out; };
export function truthSeries(c, res, frame = 'camera') {
  const n = res.n, out = new Float64Array(n * NLM * 3);
  for (let i = 0; i < n; i++) out.set(frame === 'camera' ? truthCamera(c, res.first + i) : c.truth[res.first + i], i * NLM * 3);
  return out;
}
export const reconCamera = res => cameraLandmarks(res.camera, finalLandmarks(res.skel, res.take, res.n), res.n);
export const fromCam = (cam, p) => [cam.pos[0] + p[0] * cam.r[0] + p[1] * cam.dn[0] + p[2] * cam.f[0], cam.pos[1] + p[0] * cam.r[1] + p[1] * cam.dn[1] + p[2] * cam.f[1], cam.pos[2] + p[0] * cam.r[2] + p[1] * cam.dn[2] + p[2] * cam.f[2]];
export function camToWorld(cam, C) { const out = new Float64Array(C.length); for (let q = 0; q < C.length / 3; q++) out.set(fromCam(cam, [C[q * 3], C[q * 3 + 1], C[q * 3 + 2]]), q * 3); return out; }

const hipC = (A, i, c) => (A[(i * NLM + 23) * 3 + c] + A[(i * NLM + 24) * 3 + c]) / 2;
export function shapeAlign(truthC, recC, n, joints = EVAL_JOINTS) {
  const s = shapeErrors(truthC, recC, n, { joints }).s, out = new Float64Array(n * NLM * 3);
  for (let i = 0; i < n; i++) for (let j = 0; j < NLM; j++) for (let c = 0; c < 3; c++) { const r = (i * NLM + j) * 3 + c; out[r] = s * (recC[r] - hipC(recC, i, c)) + hipC(truthC, i, c); }
  return { s, out };
}
export function trajAlign(truthC, recC, n) {
  const off = [0, 1, 2].map(c => M.median(Array.from({ length: n }, (_, i) => hipC(truthC, i, c) - hipC(recC, i, c)))), out = new Float64Array(n * NLM * 3);
  for (let q = 0; q < n * NLM; q++) for (let c = 0; c < 3; c++) out[q * 3 + c] = recC[q * 3 + c] + off[c];
  return { off, out };
}
const align = (T, R, n, mode) => mode === 'raw' ? R : mode === 'traj' ? trajAlign(T, R, n).out : shapeAlign(T, R, n).out;

// pooled-quantile accumulator: log bins, 32 per decade over [1e-3, 1e4)
const HB = 224, binOf = x => x > 1e-3 ? Math.min(HB - 1, Math.floor(32 * Math.log10(x / 1e-3))) : 0;
export const accNew = () => ({ n: 0, sum: 0, max: 0, h: new Uint32Array(HB) });
export const accAdd = (a, x) => { if (!Number.isFinite(x)) return; a.n++; a.sum += x; if (x > a.max) a.max = x; a.h[binOf(x)]++; };
export const accMerge = (a, b) => { a.n += b.n; a.sum += b.sum; a.max = Math.max(a.max, b.max); for (let k = 0; k < HB; k++) a.h[k] += b.h[k]; return a; };
export const accMean = a => a.n ? a.sum / a.n : NaN;
export function accQuantile(a, p) {
  if (!a.n) return NaN;
  const tgt = p * a.n;
  let c = 0;
  for (let k = 0; k < HB; k++) { c += a.h[k]; if (c >= tgt) return Math.min(a.max, 1e-3 * Math.pow(10, (k + 0.5) / 32)); }
  return a.max;
}
const accOut = a => ({ mean: accMean(a), p95: accQuantile(a, 0.95), acc: a });
export const stratumOf = r => r < 0.2 ? 0 : r < 0.5 ? 1 : 2;

export function orbitCamera(camA, target, thetaDeg) {
  const d = [camA.pos[0] - target[0], camA.pos[2] - target[2]], a = thetaDeg * DEG, c = Math.cos(a), s = Math.sin(a);
  return lookAtCamera([target[0] + c * d[0] + s * d[1], camA.pos[1], target[2] - s * d[0] + c * d[1]], target, { width: camA.width, height: camA.height, hfov: camA.hfov });
}

export function truthStrata(truth, camA, n) {
  const st = new Uint8Array(n * NLM), P = camA ? p => toCam(camA, p) : p => p;
  for (let i = 0; i < n; i++) for (let j = 0; j < NLM; j++) {
    const e = PARENT_EDGE[j];
    if (e < 0) continue;
    const pa = P(lmAt(truth, i * NLM + TREE[e][0])), pb = P(lmAt(truth, i * NLM + TREE[e][1]));
    st[i * NLM + j] = stratumOf(Math.abs(pb[2] - pa[2]) / Math.max(1e-9, M.dist(pa, pb)));
  }
  return st;
}

export function novelViewError(truthW, recW, camA, o = {}) {
  const thetas = o.thetas ?? [0, 45, 90, 135, 180], js = o.joints ?? EVAL_JOINTS, n = o.n ?? truthW.length / (NLM * 3), A = align(truthW, recW, n, o.mode ?? 'shape');
  const strata = o.strata ?? truthStrata(truthW, camA, n), use = i => !o.valid || o.valid[i], out = {};
  for (const th of thetas) {
    const px = accNew(), cm = accNew(), pg = Object.fromEntries(GROUPS.map(g => [g, { px: accNew(), cm: accNew() }])), bs = [accNew(), accNew(), accNew()];
    for (let i = 0; i < n; i++) {
      if (!use(i)) continue;
      const Bc = orbitCamera(camA, M.mid(lmAt(truthW, i * NLM + 23), lmAt(truthW, i * NLM + 24)), th);
      for (const j of js) {
        const q = i * NLM + j, t = toCam(Bc, lmAt(truthW, q)), r = toCam(Bc, lmAt(A, q)), dv = M.sub(r, t);
        const e = Math.hypot(Bc.focal * (r[0] / r[2] - t[0] / t[2]), Bc.focal * (r[1] / r[2] - t[1] / t[2])), l = Math.hypot(dv[0], dv[1]) * 100;
        accAdd(px, e); accAdd(cm, l);
        const g = GROUPS[GROUP_OF[j]];
        if (g) { accAdd(pg[g].px, e); accAdd(pg[g].cm, l); }
        accAdd(bs[strata[q]], l);
      }
    }
    out[th] = { px: accOut(px), cm: accOut(cm), perGroup: Object.fromEntries(GROUPS.map(g => [g, { px: accMean(pg[g].px), cm: accMean(pg[g].cm), acc: pg[g] }])), byStratum: bs.map(accMean), strataAcc: bs };
  }
  return out;
}

export function poseError(truthC, recC, o = {}) {
  const n = o.n ?? truthC.length / (NLM * 3), js = o.joints ?? EVAL_JOINTS, S = shapeAlign(truthC, recC, n, js).out, T = trajAlign(truthC, recC, n).out, use = i => !o.valid || o.valid[i];
  const strata = o.strata ?? truthStrata(truthC, null, n), sh = accNew(), tr = accNew(), bg = Object.fromEntries(GROUPS.map(g => [g, accNew()])), bs = [accNew(), accNew(), accNew()];
  for (let i = 0; i < n; i++) {
    if (!use(i)) continue;
    for (const j of js) {
      const q = i * NLM + j, es = M.dist(lmAt(S, q), lmAt(truthC, q)) * 100;
      accAdd(sh, es); accAdd(tr, M.dist(lmAt(T, q), lmAt(truthC, q)) * 100);
      if (GROUP_OF[j] >= 0) accAdd(bg[GROUPS[GROUP_OF[j]]], es);
      accAdd(bs[strata[q]], es);
    }
  }
  return { shape: accMean(sh), traj: accMean(tr), byGroup: Object.fromEntries(GROUPS.map(g => [g, accMean(bg[g])])), byStratum: bs.map(accMean), acc: { shape: sh, traj: tr } };
}

export const SIGN_BONES = Object.freeze([['lUpperArm', 11, 13], ['rUpperArm', 12, 14], ['lForearm', 13, 15], ['rForearm', 14, 16], ['lThigh', 23, 25], ['rThigh', 24, 26],
  ['lShin', 25, 27], ['rShin', 26, 28], ['lFoot', 29, 31], ['rFoot', 30, 32], ['shoulders', 12, 11], ['hips', 24, 23]]);
export function boneSignError(truthC, recC, o = {}) {
  const tau = o.tau ?? 0.2, n = o.n ?? truthC.length / (NLM * 3), use = i => !o.valid || o.valid[i], byBone = {};
  let n1 = 0, b1 = 0, n5 = 0, b5 = 0;
  for (const [name, a, b] of SIGN_BONES) {
    let c = 0, w = 0;
    for (let i = 0; i < n; i++) {
      if (!use(i)) continue;
      const ra = (i * NLM + a) * 3, rb = (i * NLM + b) * 3, dz = truthC[rb + 2] - truthC[ra + 2], L = Math.hypot(truthC[rb] - truthC[ra], truthC[rb + 1] - truthC[ra + 1], dz), r = Math.abs(dz) / L;
      if (r < Math.min(tau, 0.5)) continue;
      const bad = Math.sign(recC[rb + 2] - recC[ra + 2]) !== Math.sign(dz);
      if (r >= tau) { c++; w += bad; n1++; b1 += bad; }
      if (r >= 0.5) { n5++; b5 += bad; }
    }
    byBone[name] = c ? w / c : NaN;
  }
  return { rate: n1 ? b1 / n1 : 0, rate05: n5 ? b5 / n5 : 0, byBone, n: n1, bad: b1, n05: n5, bad05: b5 };
}

export function signRecords(c, res, o = {}) {
  const ints = res.depth?.ints ?? [], tr = trueSigns(o.truthC ?? truthSeries(c, res), res.n, o.rMoot ?? DEPTH_CAL.rMoot), rec = intervalRecords(ints, tr, res.n);
  return rec.map((r, k) => ({ ...r, moot: ints[k].moot ? 1 : 0 }));
}

export function certRecords(c, res, o = {}) {
  const cert = o.cert ?? res.cert, raw = cert?.raw, n = res.n, grid = o.grid ?? DEPTH_CAL.radius.grid, thetas = o.thetas ?? [45, 90, 135, 180], lam = o.lambdas ?? Object.fromEntries(GROUPS.map(g => [g, DEPTH_CAL.radius.groups[g]?.lambda]));
  const truthC = o.truthC ?? truthSeries(c, res), recC = raw?.Lcam ?? reconCamera(res), { s, err } = shapeErrors(truthC, recC, n), norm = raw?.norm ?? Float32Array.from(cert.sigma, x => x + (o.sigma0 ?? DEPTH_CAL.sigma0));
  const pre = raw?.state ?? cert.state, scores = {}, green = {}, fGreen = {}, fMiss = {}, shGreen = {}, shMiss = {};
  for (const g of GROUPS) { scores[g] = new Uint32Array(grid.length); green[g] = fGreen[g] = fMiss[g] = shGreen[g] = shMiss[g] = 0; }
  let maxScore = 0, maxFinal = 0, silent = 0, silentFrames = 0, joints = 0, missTotal = 0;
  const truthW = o.angles === false ? null : truthSeries(c, res, 'world'), A = truthW ? camToWorld(c.cam, shapeAlign(truthC, recC, n).out) : null, angleViol = Object.fromEntries(thetas.map(t => [t, 0]));
  const silentMask = o.mask ? new Uint8Array(n * NLM) : null;
  for (let i = 0; i < n; i++) for (const j of EVAL_JOINTS) {
    const q = i * NLM + j, g = GROUPS[GROUP_OF[j]], e = err[q], sc = e / norm[q];
    joints++;
    if (pre[q] === STATE.GREEN) {
      green[g]++;
      if (sc > maxScore) maxScore = sc;
      for (let k = 0; k < grid.length; k++) if (sc > grid[k]) scores[g][k]++;
      const lg = lam?.[g];
      if (Number.isFinite(lg)) { shGreen[g]++; if (e > lg * norm[q]) shMiss[g]++; }
    }
    if (cert.state[q] !== STATE.GREEN) continue;
    const r = cert.radius[q];
    fGreen[g]++;
    if (sc > maxFinal) maxFinal = sc;
    if (e > r) { fMiss[g]++; missTotal++; }
    if (e > Math.max(r, 0.1)) { silent++; if (silentMask) silentMask[q] = 1; }
    if (A) for (const th of thetas) {
      const Bc = orbitCamera(c.cam, M.mid(lmAt(truthW, i * NLM + 23), lmAt(truthW, i * NLM + 24)), th), t = toCam(Bc, lmAt(truthW, q)), x = toCam(Bc, lmAt(A, q));
      const px = Math.hypot(Bc.focal * (x[0] / x[2] - t[0] / t[2]), Bc.focal * (x[1] / x[2] - t[1] / t[2])), Z = t[2], bound = Bc.focal * (M.len(t) / Z) * r / (Z - r);
      if (Z > r && px > bound) angleViol[th]++;
    }
  }
  return { scores, green, maxScore, final: { green: fGreen, miss: fMiss, silent, maxScore: maxFinal }, shadow: { green: shGreen, miss: shMiss }, angleViol, missTotal, joints, s, inDomain: !!cert.inDomain, silentMask };
}

export function hingeViolations(res, o = {}) {
  const lim = (o.deg ?? 10) * DEG, n = res.take.n, rot = res.take.rot, H = [[SIDES.L.knee, [1, 0, 0]], [SIDES.R.knee, [1, 0, 0]], [SIDES.L.fore, [0, -1, 0]], [SIDES.R.fore, [0, 1, 0]]];
  let bad = 0;
  for (let i = 0; i < n; i++) if (H.some(([b, ax]) => M.twistAngle(M.swingTwist(M.qalign(getQ(rot, i * NB + b), M.QI), ax).twist, ax) < -lim)) bad++;
  return n ? bad / n : 0;
}

export const runCase = (c, settings = {}, run = processCapture) => run(c.capture, { ...c.settings, ...settings });

export function affectedMask(c, res, kind) {
  const n = res.n, m = new Uint8Array(n * NLM), f = c.faults, all = i => m.fill(1, i * NLM, (i + 1) * NLM);
  for (let i = 0; i < n; i++) {
    const fi = res.first + i;
    if (kind === 'all' || !f) { all(i); continue; }
    if (kind === 'arms') { for (const j of [13, 14, 15, 16]) m[i * NLM + j] = 1; continue; }
    if (f.drop[fi] || f.mirror[fi]) { all(i); continue; }
    if (f.swap[fi]) for (let j = 23; j < NLM; j++) m[i * NLM + j] = 1;
    if (f.flip[fi]) FAIL_CHAINS.forEach(([, , mem], b) => { if (f.flip[fi] & (1 << b)) for (const j of mem) m[i * NLM + j] = 1; });
    if (kind === 'occluded') {
      const T = truthCamera(c, fi), l = M.norm(M.sub(lmAt(T, 11), lmAt(T, 12)));
      for (let j = 11; j < NLM; j++) { const side = j % 2 ? 1 : -1; if (side * l[2] > 0.3) m[i * NLM + j] = 1; }
    }
  }
  return m;
}

export const REDTEAM = Object.freeze([
  { id: 'R1', name: 'flips on whole intervals', family: 'in', o: { profile: 'flipInterval' }, affected: 'faults' },
  { id: 'R2', name: 'Necker reversal in profile', family: 'in', o: { profile: 'neckerProfile' }, affected: 'faults' },
  { id: 'R3', name: 'leg swaps at knee crossings', family: 'in', o: { profile: 'legSwapCross' }, affected: 'faults' },
  { id: 'R4', name: 'hidden joints hallucinated', family: 'in', o: { profile: 'farHallucinate' }, affected: 'occluded' },
  { id: 'R5', name: 'wrong lens (hfov 40 or 85, guess 60)', family: 'in', o: { profile: 'typical' }, hfovPick: [40, 85], affected: 'all', shift: true },
  { id: 'R6', name: 'slow 3D drift', family: 'in', o: { profile: 'drift' }, affected: 'all' },
  { id: 'R7', name: 'depth compressed (gain 0.4)', family: 'in', o: { profile: 'compress' }, affected: 'all' },
  { id: 'R8', name: 'long dropouts', family: 'in', o: { profile: 'dropout' }, affected: 'faults' },
  { id: 'R9', name: 'arms held at the camera', family: 'reachHold', o: { profile: 'typical' }, affected: 'arms' },
  { id: 'R10', name: 'fps lie (24 or 40 shown as 30), height known', family: 'jump', o: { profile: 'typical', knownHeight: true }, fpsPick: [24, 40], affected: 'all', shift: true },
  { id: 'R11', name: 'portrait phone lens (hfov 36-46, guess 60)', family: 'in', o: { profile: 'typical', portrait: true }, affected: 'all', shift: true },
  { id: 'R12', name: 'hard profile on held-out families', family: 'held', o: { profile: 'hard' }, affected: 'all' },
]);
export function redteamCase(r, seed) {
  const rr = mulberry32((seed ^ 0x7EA4) >>> 0), pick = a => a[Math.floor(rr() * a.length)];
  const family = r.family === 'in' ? pick(IN_DIST) : r.family === 'held' ? pick(HELD_OUT.families) : r.family, o = { ...r.o };
  if (r.hfovPick) o.hfov = pick(r.hfovPick);
  if (r.fpsPick) o.fpsTrue = pick(r.fpsPick);
  return makeCase(family, seed, o);
}
