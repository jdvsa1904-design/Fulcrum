import { mulberry32 } from './depthstats.js';
import { NLM, LM_MIRROR } from './skeleton.js';

const ST = 5, LEGS = new Set([23, 24, 25, 26, 27, 28, 29, 30, 31, 32]), LEG_PERM = LM_MIRROR.map((m, j) => LEGS.has(j) ? m : j);
export const FAIL_CHAINS = Object.freeze([['lArm', 11, [13, 15, 17, 19, 21]], ['rArm', 12, [14, 16, 18, 20, 22]], ['lFore', 13, [15, 17, 19, 21]], ['rFore', 14, [16, 18, 20, 22]],
  ['lLeg', 23, [25, 27, 29, 31]], ['rLeg', 24, [26, 28, 30, 32]], ['lShin', 25, [27, 29, 31]], ['rShin', 26, [28, 30, 32]], ['lFoot', 27, [29, 31]], ['rFoot', 28, [30, 32]]]);
const NONE = { gain: 1, scale: 1, rho: 0, img: 0, world: 0, depth: 0, outlier: 0, outK: 6, flipRate: 0, flipDur: [0.15, 1.5], flipFar: 3, interval: 0, mirrorRate: 0, mirrorDur: [0.3, 2.5], mirrorBack: 5,
  swapRate: 0, swapDur: [0.1, 0.8], swapCross: 10, crossSwap: 0, halluc: 0, occNoise: 0, visLie: 0, dropRate: 0, dropDur: [1, 10], drift: 0, driftRho: 0.97, necker: 0 };
const TYPICAL = { ...NONE, gain: [0.65, 0.9], scale: [0.92, 1.08], rho: [0.6, 0.9], img: 0.0025, world: 0.02, depth: 0.035, outlier: 0.003, flipRate: 0.08, flipDur: [0.15, 1.5],
  mirrorRate: 0.02, mirrorDur: [0.3, 2.5], swapRate: 0.05, swapDur: [0.1, 0.8], halluc: 0.3, occNoise: 1.5, visLie: 0.3, dropRate: 0.02, dropDur: [1, 10], drift: 0.01 };
export const MP_PROFILES = Object.freeze({
  none: NONE, typical: TYPICAL,
  hard: { ...NONE, gain: [0.5, 0.8], scale: [0.85, 1.15], rho: [0.85, 0.95], img: 0.003, world: 0.03, depth: 0.05, outlier: 0.01, flipRate: 0.25, flipDur: [0.2, 2.5], interval: 0.5,
    mirrorRate: 0.06, mirrorDur: [0.5, 3], swapRate: 0.15, swapDur: [0.1, 1], halluc: 0.6, occNoise: 3, visLie: 0.7, dropRate: 0.05, dropDur: [1, 15], drift: 0.02 },
  flipInterval: { ...TYPICAL, interval: 1, flipRate: 0.3 }, neckerProfile: { ...TYPICAL, necker: 0.5 }, legSwapCross: { ...TYPICAL, swapRate: 0, crossSwap: 0.5 },
  farHallucinate: { ...TYPICAL, halluc: 0.8, visLie: 1 }, drift: { ...TYPICAL, drift: 0.025 }, compress: { ...TYPICAL, gain: 0.4 }, dropout: { ...TYPICAL, dropRate: 0.2, dropDur: [5, 30] },
});
const SCALED = ['img', 'world', 'depth', 'outlier', 'flipRate', 'mirrorRate', 'swapRate', 'crossSwap', 'dropRate', 'halluc', 'occNoise', 'visLie', 'drift', 'necker'];

export function resolveProfile(p, rand) {
  const o = typeof p === 'string' ? { base: p } : p ?? { base: 'none' }, base = o.base ? MP_PROFILES[o.base] : NONE;
  if (!base) throw new Error('unknown MediaPipe profile ' + o.base);
  const P = { ...NONE, ...base, ...o }, u = o.intensity ?? 1;
  for (const k of ['gain', 'scale', 'rho']) if (Array.isArray(P[k])) P[k] = P[k][0] + (P[k][1] - P[k][0]) * rand();
  for (const k of SCALED) P[k] *= u;
  P.gain = 1 - u * (1 - P.gain); P.scale = 1 + u * (P.scale - 1); P.name = o.base ?? 'custom'; P.intensity = u;
  return P;
}

const identity = P => P.gain === 1 && P.scale === 1 && SCALED.every(k => !P[k]);
const gauss = r => { let u = 0; while (!u) u = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * r()); };
const dist3 = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

export function signRuns(C, a, b, tau = 0.15) {
  const n = C.length, id = new Int32Array(n).fill(-1);
  let run = -1, s0 = 0;
  for (let i = 0; i < n; i++) {
    const dz = C[i][b][2] - C[i][a][2], L = dist3(C[i][a], C[i][b]);
    if (Math.abs(dz) <= tau * L) continue;
    const s = Math.sign(dz);
    if (s !== s0) { run++; s0 = s; }
    id[i] = run;
  }
  return id;
}

export function spans(n, fps, rand, rate, dur, hazard = () => 1) {
  const on = new Uint8Array(n);
  if (!(rate > 0)) return on;
  for (let i = 0; i < n; i++) if (rand() < rate * hazard(i) / fps) {
    const d = Math.max(1, Math.round(dur[0] * Math.pow(dur[1] / dur[0], rand()) * fps));
    for (let k = i; k < Math.min(n, i + d); k++) on[k] = 1;
    i += d;
  }
  return on;
}

const fwdZ = c => { const l = [c[11][0] - c[12][0], c[11][1] - c[12][1], c[11][2] - c[12][2]], u = [(c[11][0] + c[12][0] - c[23][0] - c[24][0]) / 2, (c[11][1] + c[12][1] - c[23][1] - c[24][1]) / 2, (c[11][2] + c[12][2] - c[23][2] - c[24][2]) / 2]; return l[0] * u[1] - l[1] * u[0]; };
const knee2d = c => Math.hypot(c[25][0] / c[25][2] - c[26][0] / c[26][2], c[25][1] / c[25][2] - c[26][1] / c[26][2]);

export function faultPlan(subj, C, fps, P, rand) {
  const n = C.length, flip = new Uint16Array(n), visLie = new Uint8Array(NLM);
  FAIL_CHAINS.forEach(([, root, mem], b) => {
    const on = spans(n, fps, rand, P.flipRate * (1 - P.interval), P.flipDur, i => subj[i].img[mem[0] * ST + 3] < 0.6 ? P.flipFar : 1);
    if (P.interval > 0 && P.flipRate > 0) {
      const id = signRuns(C, root, mem[0]), runs = new Map();
      for (let i = 0; i < n; i++) if (id[i] >= 0) { const r = runs.get(id[i]); if (r) r[1] = i; else runs.set(id[i], [i, i]); }
      for (const [k, [a, z]] of runs) if (rand() < Math.min(1, P.interval * P.flipRate * (z - a + 1) / fps)) for (let i = a; i <= z; i++) if (id[i] === k) on[i] = 1;
    }
    for (let i = 0; i < n; i++) if (on[i]) flip[i] |= 1 << b;
  });
  const mirror = spans(n, fps, rand, P.mirrorRate, P.mirrorDur, i => fwdZ(C[i]) > 0 ? P.mirrorBack : 1);
  if (P.necker > 0) {
    const id = signRuns(C, 12, 11, 0.7), seen = new Map();
    for (let i = 0; i < n; i++) if (id[i] >= 0) { if (!seen.has(id[i])) seen.set(id[i], rand() < P.necker); if (seen.get(id[i])) mirror[i] = 1; }
  }
  const swap = spans(n, fps, rand, P.swapRate, P.swapDur, i => knee2d(C[i]) < 0.02 ? P.swapCross : 1);
  if (P.crossSwap > 0) {
    let a = -1;
    for (let i = 1; i < n; i++) {
      const cr = knee2d(C[i]) < 0.02 && knee2d(C[i - 1]) >= 0.02;
      if (!cr) continue;
      if (a >= 0 && rand() < P.crossSwap) for (let k = a; k < i; k++) swap[k] = 1;
      a = i;
    }
  }
  const drop = spans(n, fps, rand, P.dropRate, [P.dropDur[0] / fps, P.dropDur[1] / fps]);
  for (let j = 0; j < NLM; j++) visLie[j] = rand() < P.visLie ? 1 : 0;
  let drift = null;
  if (P.drift > 0) {
    drift = new Float64Array(n * NLM * 3);
    const k1 = Math.sqrt(1 - P.driftRho * P.driftRho);
    for (let q = 0; q < NLM * 3; q++) drift[q] = P.drift * gauss(rand);
    for (let i = 1; i < n; i++) for (let q = 0; q < NLM * 3; q++) drift[i * NLM * 3 + q] = P.driftRho * drift[(i - 1) * NLM * 3 + q] + k1 * P.drift * gauss(rand);
  }
  return { flip, mirror, swap, drop, visLie, drift, params: P };
}

export function applyFaults(subj, C, cam, plan, P, rand) {
  const n = subj.length, rho = P.rho, k1 = Math.sqrt(1 - rho * rho), ar = new Float64Array(NLM * 6), tI = new Float64Array(NLM * ST), tW = new Float64Array(NLM * 3), asp = cam.width / cam.height;
  for (let q = 0; q < ar.length; q++) ar[q] = gauss(rand);
  for (let i = 0; i < n; i++) {
    const { img, world } = subj[i];
    if (plan.drift) for (let q = 0; q < NLM * 3; q++) world[q] += plan.drift[i * NLM * 3 + q];
    for (let q = 0; q < NLM * 3; q++) world[q] *= P.scale;
    for (let j = 0; j < NLM; j++) { world[j * 3 + 2] *= P.gain; img[j * ST + 2] *= P.gain; }
    if (plan.flip[i]) FAIL_CHAINS.forEach(([, root, mem], b) => {
      if (!(plan.flip[i] & (1 << b))) return;
      for (const j of mem) { world[j * 3 + 2] = 2 * world[root * 3 + 2] - world[j * 3 + 2]; img[j * ST + 2] = 2 * img[root * ST + 2] - img[j * ST + 2]; }
    });
    if (P.halluc > 0) for (let j = 11; j < NLM; j++) {
      const w = P.halluc * Math.max(0, (0.8 - img[j * ST + 3]) / 0.6), m = LM_MIRROR[j];
      img[j * ST] += (img[m * ST] - img[j * ST]) * w; img[j * ST + 1] += (img[m * ST + 1] - img[j * ST + 1]) * w;
    }
    for (let j = 0; j < NLM; j++) {
      const occ = 1 + P.occNoise * Math.max(0, 0.8 - img[j * ST + 3]) / 0.6, out = P.outlier > 0 && rand() < P.outlier ? P.outK : 1;
      for (let k = 0; k < 6; k++) ar[j * 6 + k] = rho * ar[j * 6 + k] + k1 * gauss(rand);
      img[j * ST] += ar[j * 6] * P.img * occ * out; img[j * ST + 1] += ar[j * 6 + 1] * P.img * occ * out * asp;
      world[j * 3] += ar[j * 6 + 2] * P.world * occ * out; world[j * 3 + 1] += ar[j * 6 + 3] * P.world * occ * out; world[j * 3 + 2] += ar[j * 6 + 4] * P.depth * occ * out;
      img[j * ST + 2] += ar[j * 6 + 5] * P.depth * occ / 2;
      if (plan.visLie[j]) img[j * ST + 3] = Math.max(img[j * ST + 3], 0.9);
    }
    const perm = plan.mirror[i] ? LM_MIRROR : plan.swap[i] ? LEG_PERM : null;
    if (perm) {
      tI.set(img); tW.set(world);
      for (let j = 0; j < NLM; j++) { for (let k = 0; k < ST; k++) img[j * ST + k] = tI[perm[j] * ST + k]; for (let k = 0; k < 3; k++) world[j * 3 + k] = tW[perm[j] * 3 + k]; }
      if (plan.mirror[i]) for (let j = 0; j < NLM; j++) { world[j * 3 + 2] = -world[j * 3 + 2]; img[j * ST + 2] = -img[j * ST + 2]; }
    }
  }
}

export function mediapipeFailures(subj, C, cam, profile, fps, seed) {
  const rand = mulberry32(seed), P = resolveProfile(profile, rand), n = subj.length;
  if (identity(P)) return { flip: new Uint16Array(n), mirror: new Uint8Array(n), swap: new Uint8Array(n), drop: new Uint8Array(n), visLie: new Uint8Array(NLM), drift: null, params: P };
  const plan = faultPlan(subj, C, fps, P, rand);
  applyFaults(subj, C, cam, plan, P, rand);
  return plan;
}
