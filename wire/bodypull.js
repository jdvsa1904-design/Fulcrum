import { bodyPull, DEFAULTS } from '../hik.js';

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const len = a => Math.hypot(a[0], a[1], a[2]);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const dist = (a, b) => len(sub(a, b));
const finite = v => v && Number.isFinite(v[0]) && Number.isFinite(v[1]) && Number.isFinite(v[2]);
const smooth = t => t * t * (3 - 2 * t);

const LEG_REACH = 0.995;
const LEG_FOLD = 0.3;

function pos(W, nb, f, i) {
  const o = (f * nb + i) * 16 + 12;
  return [W[o], W[o + 1], W[o + 2]];
}

function easeWeight(f, a, b, ease, n) {
  if (ease <= 0) return 1;
  const head = a <= 0 ? 1 : (f - a + 1) / (ease + 1);
  const tail = b >= n - 1 ? 1 : (b - f + 1) / (ease + 1);
  return smooth(Math.max(0, Math.min(1, head, tail)));
}

function legState(W, nb, f, ids) {
  const top = pos(W, nb, f, ids.thigh), knee = pos(W, nb, f, ids.calf), foot = pos(W, nb, f, ids.foot);
  const L = dist(top, knee) + dist(knee, foot);
  return { top, foot, L, d0: dist(top, foot) };
}

function within(off, legs) {
  for (const g of legs) {
    const v = sub(add(g.top, off), g.foot);
    if (len(v) > g.limit + 1e-9 || dot(v, g.up) < g.fold - 1e-9) return false;
  }
  return true;
}

function clampToLegs(off, legs, iters = 24) {
  if (!legs.length || within(off, legs)) return off;
  let o = off.slice();
  for (let k = 0; k < iters; k++) {
    for (const g of legs) {
      const v = sub(add(g.top, o), g.foot), d = len(v), h = dot(v, g.up);
      if (h < g.fold) o = add(o, mul(g.up, g.fold - h));
      else if (d > g.limit) o = add(o, mul(v, g.limit / d - 1));
    }
    if (within(o, legs)) break;
  }
  if (within(o, legs)) return o;
  let lo = 0, hi = 1;
  for (let k = 0; k < 40; k++) {
    const m = (lo + hi) / 2;
    if (within(mul(o, m), legs)) lo = m; else hi = m;
  }
  return mul(o, lo);
}

export function pullOffsets({
  W, nb, n, frames, hand, handGoal, legs = {}, feetPlanted = () => false,
  pull = 1, gravity = DEFAULTS.gravity, iterations = DEFAULTS.pullIters, up = [0, 1, 0],
  compression = DEFAULTS.compression, ease = 4, minOffset = 1e-6, pullHips = 1,
}) {
  const out = new Array(n).fill(null);
  if (!W || !hand || !frames || pull <= 0 || !(pullHips > 0)) return out;
  const a = Math.max(0, Math.floor(frames[0])), b = Math.min(n - 1, Math.floor(frames[1]));
  const upN = mul(up, 1 / (len(up) || 1));
  const sides = ['l', 'r'].filter(s => legs[s] && [legs[s].thigh, legs[s].calf, legs[s].foot].every(i => i !== undefined && i !== null));
  for (let f = a; f <= b; f++) {
    const goal = handGoal(f);
    if (!finite(goal)) continue;
    const sh = pos(W, nb, f, hand.shoulder), el = pos(W, nb, f, hand.elbow), hd = pos(W, nb, f, hand.hand);
    const legSt = Object.fromEntries(sides.map(s => [s, legState(W, nb, f, legs[s])]));
    const tops = sides.map(s => legSt[s].top);
    const hips = tops.length ? mul(tops.reduce(add, [0, 0, 0]), 1 / tops.length) : sh.slice();
    const limbs = [{
      kind: 'arm', body: 'hips', offset: sub(sh, hips), goal, pull,
      L0: dist(sh, hd), Lmax: dist(sh, el) + dist(el, hd), rc: compression,
    }];
    const planted = [];
    for (const s of sides) {
      if (!feetPlanted(f, s)) continue;
      const g = legSt[s];
      limbs.push({ kind: 'leg', body: 'hips', offset: sub(g.top, hips), goal: g.foot, pull: 1, T: 1, L0: g.d0, Lmax: g.L, rc: compression });
      planted.push({ top: g.top, foot: g.foot, limit: Math.max(LEG_REACH * g.L, g.d0), up: upN, fold: Math.min(LEG_FOLD * g.L, dot(sub(g.top, g.foot), upN)) });
    }
    const r = bodyPull({ hips, chest: hips, limbs, pullIters: iterations, gravity, up, rigid: false });
    let off = sub(r.hips, hips);
    if (!finite(off)) continue;
    off = clampToLegs(mul(off, Math.min(1, pullHips) * easeWeight(f, a, b, ease, n)), planted);
    if (len(off) > minOffset) out[f] = off;
  }
  return out;
}
