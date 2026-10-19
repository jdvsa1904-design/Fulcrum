import { pullForce } from '../../hik.js';
import { qrot } from '../../ik.js';

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const len = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => { const l = len(a); return l > 1e-12 ? mul(a, 1 / l) : [0, 0, 0]; };
const qaxis = (k, ang) => { const n = unit(k), s = Math.sin(ang / 2); return [n[0] * s, n[1] * s, n[2] * s, Math.cos(ang / 2)]; };
const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];

export const PULL_GAINS = Object.freeze({
  neckHips: [0.7, 0.3], neckChest: [0.5, 0.5], armsChest: [0.7, 0.3], legsHips: [0.5, 0.5],
  chestRotA: [0.6, 0.4], chestRotB: [0.4, 0.6], hipsRot: [0.3, 0.7],
});


export function pullPasses(pullIterationCount = 10) {
  const N = Math.floor(pullIterationCount + 0.5) + 2;
  return { passes: N, exponent: Math.max(1, 0.2 * N) };
}

export function neckPullLength(chain) {
  let Lmax = 0;
  for (let i = 1; i < chain.length; i++) Lmax += len(sub(chain[i], chain[i - 1]));
  return { L0: len(sub(chain[chain.length - 1], chain[0])), Lmax };
}

export function neckPullForce(neckBase, goal, L0, Lmax, pull, T = 1, resist = 0.5, exponent = 1) {
  const v = sub(goal, neckBase), L = len(v);
  if (L < 1e-12 || pull * T <= 0) return [0, 0, 0];
  return mul(unit(v), pullForce(L, L0, Lmax, Math.pow(resist, exponent), 1) * pull * T);
}

const lerp3 = (a, b, t) => add(a, mul(sub(b, a), t));
const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const vcross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const qarc = (a, b) => {
  const c = vcross(a, b), w = Math.sqrt(vdot(a, a) * vdot(b, b)) + vdot(a, b), l = Math.hypot(c[0], c[1], c[2], w);
  return l > 1e-12 ? [c[0] / l, c[1] / l, c[2] / l, w / l] : [0, 0, 0, 1];
};

export const BODY_PULL_GAINS = Object.freeze({ neckHips: [0.3, 0.7], legsHips: [0.5, 0.5], neckChest: [0.5, 0.5], armsChest: [0.3, 0.7], chestDenominator: 2 });

export function bodyPullPassWeights(o) {
  const p = o.neckPull ?? 0, arms = o.armPulls ?? [], legs = o.legPulls ?? [];
  const hands = o.handPullHips ?? arms.map(() => 0);
  const hipsPull = (o.hipsT ?? 0) * (o.hipsPull ?? 0), ceT = o.chestEndT ?? 0, ce = ceT * (o.chestEndPull ?? 0);
  const neckChest = p * (1 - ce), armChest = arms.map(a => a * (1 - ce));
  const chestW = armChest.reduce((s, a) => s * (1 - a), 1) * (1 - neckChest) * (ceT - ce) + ce;
  const free = (1 - hipsPull) * (1 - chestW);
  const neckHips = p * free, armHips = arms.map((a, i) => a * free * hands[i]);
  const legHips = legs.map(l => l * (1 - hipsPull));
  const sum = xs => xs.reduce((s, x) => s + x, 0);
  return {
    neckHips, armHips, legHips, neckChest, armChest, chestW,
    hipsDen: Math.max(1, neckHips + sum(armHips) + sum(legHips)),
    chestDen: Math.max(chestW + neckChest + sum(armChest), 2 - chestW),
  };
}

export function headPull(o) {
  const { passes, exponent } = pullPasses(o.pullIterationCount ?? 10);
  const p = (o.pull ?? 0) * (o.T ?? 1);
  const hips0 = o.hips, neck0 = o.neck;
  let P = o.spine ? o.spine.map(x => x.slice()) : [hips0.slice(), lerp3(hips0, neck0, 0.2), lerp3(hips0, neck0, 0.5), neck0.slice()];
  let Q = P.map(() => [0, 0, 0, 1]);
  const last = P.length - 1;
  const sf = o.shoulderFraction ?? 0.8;
  const shoulders = () => lerp3(P[last - 1], P[last], sf);
  const headLen = len(sub(o.head, neck0));
  const chain = o.neckChain ?? [headLen];
  const Lmax = chain.reduce((s, x) => s + x, 0);
  const resist = Math.pow(o.resist ?? 0, exponent);
  const feet = o.feet ?? [];
  const G = BODY_PULL_GAINS;
  const w = bodyPullPassWeights({ neckPull: p, legPulls: feet.map(f => f.pull ?? 1) });
  for (let i = 0; i < passes && p > 0; i++) {
    const t = (i + 1) / passes;
    const hips = P[0], neck = P[last];
    const v = sub(o.goal, neck);
    const F = mul(unit(v), pullForce(len(v), headLen, Lmax, resist, 1) * p);
    let num = mul(F, (G.neckHips[0] + G.neckHips[1] * t) * w.neckHips);
    feet.forEach((f, k) => {
      const root = add(hips, f.hipOffset), vv = sub(f.ankle, root);
      const Lf = f.length ?? len(sub(f.ankle, add(hips0, f.hipOffset)));
      const Fl = mul(unit(vv), pullForce(len(vv), Lf, Lf, Math.pow(f.resist ?? 0, exponent), 1) * (f.pull ?? 1));
      num = add(num, mul(Fl, (G.legsHips[0] + G.legsHips[1] * t) * w.legHips[k]));
    });
    const dH = mul(num, 1 / w.hipsDen);
    const target = add(add(shoulders(), dH), mul(F, (G.neckChest[0] + G.neckChest[1] * t) * w.neckChest / w.chestDen));
    P = P.map(x => add(x, dH));
    const r = distributeSpine(P, Q, qarc(sub(shoulders(), P[0]), sub(target, P[0])), o.spineStiffness ?? 0);
    P = r.points; Q = r.quats;
  }
  const neck = P[last];
  const head = add(neck, mul(unit(sub(o.goal, neck)), headLen));
  return { hips: P[0], neck, head, spine: P, passes };
}

export function spineShares(count, stiffness = 0) {
  const N = count + 1, b = 2 * N * stiffness + 0.5;
  const shares = [];
  for (let k = -1; k < N; k++) shares.push((b + 1 + k) / (b + N));
  return shares;
}

export function spineReachDeltaQ(theta, axis, count, stiffness = 0) {
  return spineShares(count, stiffness).map(s => qaxis(axis, theta * s));
}

export function distributeSpine(points, quats, deltaQ, stiffness = 0) {
  const count = points.length - 1;
  const base = points[0];
  const ang = 2 * Math.atan2(Math.hypot(deltaQ[0], deltaQ[1], deltaQ[2]), deltaQ[3]);
  const axis = Math.hypot(deltaQ[0], deltaQ[1], deltaQ[2]) > 1e-12 ? [deltaQ[0], deltaQ[1], deltaQ[2]] : [0, 1, 0];
  const qs = spineReachDeltaQ(ang, axis, count - 1, stiffness);
  const outP = [base.slice()], outQ = [];
  for (let k = 0; k < count; k++) {
    outQ.push(qmul(qs[k], quats[k]));
    outP.push(add(outP[k], qrot(qs[k], sub(points[k + 1], points[k]))));
  }
  outQ.push(qmul(qs[count], quats[count]));
  return { points: outP, quats: outQ };
}

export const BODY_REACH_TABLE = Object.freeze({
  hips: {
    free: { 0: [[0, 0, 0], [0.25, 0.239, 0.239], [0.5, 0.5, 0.5], [1, 1, 1]], 1: [[0, 0, 0], [0.25, 0.244, 0.244], [0.5, 0.5, 0.5], [1, 1, 1]] },
    planted: { 0: [[0, 0, 0], [0.25, 0.062, 0.062], [0.5, 0.25, 0.25], [1, 0.975, 0.975]], 1: [[0, 0, 0], [0.25, 0.224, 0.224], [0.5, 0.497, 0.497], [1, 1, 1]] },
  },
  chestEnd: {
    free: { 0: [[0, 0, 0], [0.25, 0.066, 0.133], [0.5, 0.264, 0.491], [1, 1, 1]], 1: [[0, 0, 0], [0.25, 0.228, 0.247], [0.5, 0.469, 0.505], [1, 1, 1]] },
    planted: { 0: [[0, 0, 0], [0.25, 0.064, 0.129], [0.5, 0.255, 0.488], [1, 0.975, 1.003]], 1: [[0, 0, 0], [0.25, 0.2, 0.228], [0.5, 0.443, 0.507], [1, 0.963, 1.005]] },
  },
  chestOrigin: {
    free: { 0: [[0, 0, 0], [1, 0, 0]], 1: [[0, 0, 0], [1, 0, 0]] },
    planted: { 0: [[0, 0, 0], [1, 0, 0]], 1: [[0, 0, 0], [1, 0, 0]] },
  },
});

function lerpTable(rows, T) {
  const t = Math.max(0, Math.min(1, T));
  for (let i = 1; i < rows.length; i++) {
    if (t <= rows[i][0]) {
      const [t0, h0, c0] = rows[i - 1], [t1, h1, c1] = rows[i], u = (t - t0) / (t1 - t0);
      return [h0 + (h1 - h0) * u, c0 + (c1 - c0) * u];
    }
  }
  const r = rows[rows.length - 1];
  return [r[1], r[2]];
}

export function bodyEffectorReach({ effector, offset, T, pull = 0, planted = false }) {
  const tab = BODY_REACH_TABLE[effector];
  if (!tab) throw new Error(`unknown body effector ${effector}`);
  const side = tab[planted ? 'planted' : 'free'];
  const lo = lerpTable(side[0], T), hi = lerpTable(side[1], T), p = Math.max(0, Math.min(1, pull));
  const gh = lo[0] + (hi[0] - lo[0]) * p, gc = lo[1] + (hi[1] - lo[1]) * p;
  return { hips: mul(offset, gh), chest: mul(offset, gc), gains: [gh, gc] };
}
