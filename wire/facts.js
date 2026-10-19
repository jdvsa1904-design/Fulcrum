// One shared "motion facts" model: ground, contacts (feet, hands, knees), phases, COM, balance,
// flights, seams, knees and trust, all computed once from world matrices W (n*nb*16, column-major
// like three.js Matrix4.elements, Y up). Every planner and judge reads this object instead of
// re-detecting contacts on its own. Pure: no DOM, no fetch.
import { detectFootPlants } from './plants.js';
import { detectFulcrums, pivotSlip } from '../fulcrum.js';
import { detectFlights, fitBallistic, flightReport } from '../ballistic.js';
import { supportHull, supportMargin } from '../motionquality.js';
import { segmentPhases, PHASES_SOURCE } from './phaseadapter.js';

const SIDES = ['l', 'r'];

export const RIG_NAMES = ['pelvis', 'spine_01', 'spine_02', 'spine_03', 'spine_04', 'spine_05', 'neck_01', 'head',
  'clavicle_l', 'upperarm_l', 'lowerarm_l', 'hand_l', 'clavicle_r', 'upperarm_r', 'lowerarm_r', 'hand_r',
  'thigh_l', 'calf_l', 'foot_l', 'ball_l', 'thigh_r', 'calf_r', 'foot_r', 'ball_r'];
export const REQUIRED_BONES = ['pelvis', 'thigh_l', 'calf_l', 'foot_l', 'thigh_r', 'calf_r', 'foot_r'];

// Mass fractions per segment, the same table app.js uses for its COM (segment midpoints weighted by mass).
export const MASS_SEGMENTS = [
  ['pelvis', 'spine_02', 0.142], ['spine_02', 'spine_04|spine_03', 0.163], ['spine_04|spine_03', 'neck_01', 0.16], ['neck_01|head', 'head', 0.069],
  ['upperarm_l', 'lowerarm_l', 0.0271], ['lowerarm_l', 'hand_l', 0.0162], ['hand_l', 'hand_l', 0.0061],
  ['upperarm_r', 'lowerarm_r', 0.0271], ['lowerarm_r', 'hand_r', 0.0162], ['hand_r', 'hand_r', 0.0061],
  ['thigh_l', 'calf_l', 0.1416], ['calf_l', 'foot_l', 0.0433], ['foot_l', 'ball_l|foot_l', 0.0137],
  ['thigh_r', 'calf_r', 0.1416], ['calf_r', 'foot_r', 0.0433], ['foot_r', 'ball_r|foot_r', 0.0137],
];

// Bone order Fulcrum's phase segmentation expects (index constants inside segmentPhases).
export const PHASE_BONES = ['pelvis', 'spine_01', 'spine_03', 'spine_05', 'neck_01', 'head', 'clavicle_l', 'upperarm_l', 'lowerarm_l', 'hand_l',
  'clavicle_r', 'upperarm_r', 'lowerarm_r', 'hand_r', 'thigh_l', 'calf_l', 'foot_l', 'ball_l', 'thigh_r', 'calf_r', 'foot_r', 'ball_r'];
const PHASE_FALLBACK = { spine_01: ['spine_02', 'pelvis'], spine_03: ['spine_02', 'spine_04', 'spine_01'], spine_05: ['spine_04', 'spine_03', 'neck_01'],
  neck_01: ['head'], head: ['neck_01'], clavicle_l: ['upperarm_l'], clavicle_r: ['upperarm_r'], ball_l: ['foot_l'], ball_r: ['foot_r'] };

// Which old definition each facts field replaces. Every one of these re-detected contacts or ground on its own.
export const LEGACY_MAP = Object.freeze({
  'app.js contactsFromWorld().ground (3rd percentile of foot/ball height, all frames)': 'facts.ground (densest 2 cm band of slow-foot heights; facts.groundLegacy keeps the old number)',
  'app.js contactsFromWorld().per / .any / .speed': 'facts.contacts.per / facts.contacts.feetDown / facts.contacts.speed (same 10 cm and 0.3 m/s rules, shared ground)',
  'app.js stancePlants(), detectPlantsNow(), autoLocks, travelOffsets bounce plants, stridePlan plants (wire/plants.js detectFootPlants)': 'facts.contacts.feet.{l,r}.plants (Rule A once: sliding, drift_cm, anchor, point, plus lockable, skate, phase)',
  'app.js groundPlan() level (median planted foot/ball height)': 'facts.contacts.feet.{l,r}.level',
  'fulcrum.js detectFulcrums() for hands and feet (Fulcrums button)': 'facts.contacts.hands / facts.contacts.knees (feet pivots are the plants)',
  'motionquality.js inspectMotion() supports (foot < 10 cm and < 0.25 m/s)': 'facts.balance (supports from planted feet, hands and knees)',
  'ballistic.js detectFlights(ct.any) in the Ballistic button': 'facts.flights (no support at all, with the fit report and a plausibility flag)',
  'app.js comSeries()/comFromWorld()': 'facts.com (same mass table), facts.comAcc (analyze() smoothing)',
  'tastebridge/phases segmentPhases() (Fulcrum phase code; it keeps its own internal ground)': 'facts.phases / facts.phaseAt',
  'seams.js seamPlan() shot boundaries': 'facts.seams (boundaries); the jump test itself is in judge.js',
  'wire/floor.js calibrate(ground) and kneeFloorTargets(ground)': 'pass facts.ground',
});

export const keyOf = n => String(n).split(':').pop().split('|').pop().toLowerCase();

export function rigIndices(find) {
  const rig = {};
  for (const k of RIG_NAMES) { const i = find(k); if (Number.isInteger(i) && i >= 0) rig[k] = i; }
  return rig;
}

export function rigFromNames(names) {
  const by = new Map();
  names.forEach((nm, i) => { const k = keyOf(nm); if (!by.has(k)) by.set(k, i); });
  return rigIndices(k => by.get(k));
}

export const missingBones = rig => REQUIRED_BONES.filter(k => rig[k] === undefined);
const firstOf = (rig, spec) => { for (const k of spec.split('|')) if (rig[k] !== undefined) return rig[k]; return undefined; };

export const wpos = (W, nb, f, i) => { const o = (f * nb + i) * 16 + 12; return [W[o], W[o + 1], W[o + 2]]; };
const wy = (W, nb, f, i) => W[(f * nb + i) * 16 + 13];
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const pctSorted = (s, p) => s.length ? s[Math.min(s.length - 1, Math.max(0, Math.floor(s.length * p)))] : NaN;
const median = a => { const s = Array.from(a).sort((x, y) => x - y); return s.length ? s[s.length >> 1] : NaN; };

export function runsOf(mask, minLen = 1) {
  const out = [];
  let s = -1;
  for (let f = 0; f <= mask.length; f++) {
    const on = f < mask.length && mask[f];
    if (on && s < 0) s = f;
    if (!on && s >= 0) { if (f - s >= minLen) out.push([s, f - 1]); s = -1; }
  }
  return out;
}

function restPos(restW, i) {
  if (!restW || i === undefined) return null;
  if (ArrayBuffer.isView(restW)) { const o = i * 16 + 12; return [restW[o], restW[o + 1], restW[o + 2]]; }
  const m = restW[i];
  if (!m) return null;
  const e = m.elements ?? m;
  return [e[12], e[13], e[14]];
}

function restRot(restW, i) {
  if (!restW || i === undefined) return null;
  const e = ArrayBuffer.isView(restW) ? restW.subarray(i * 16, i * 16 + 16) : (restW[i]?.elements ?? restW[i]);
  if (!e) return null;
  const c = k => { const l = Math.hypot(e[k * 4], e[k * 4 + 1], e[k * 4 + 2]) || 1; return [e[k * 4] / l, e[k * 4 + 1] / l, e[k * 4 + 2] / l]; };
  return [c(0), c(1), c(2)];
}

function boneLength(W, nb, n, restW, i, j) {
  const a = restPos(restW, i), b = restPos(restW, j);
  if (a && b) return dist(a, b);
  const d = [], step = Math.max(1, Math.floor(n / 30));
  for (let f = 0; f < n; f += step) d.push(dist(wpos(W, nb, f, i), wpos(W, nb, f, j)));
  return median(d);
}

export function legsOf(rig) {
  const out = {};
  for (const s of SIDES) out[s] = { thigh: rig['thigh_' + s], calf: rig['calf_' + s], foot: rig['foot_' + s], ball: firstOf(rig, `ball_${s}|foot_${s}`) };
  return out;
}

// Horizontal speed of a bone in m/s, central difference over f-1..f+1 (always /2, like contactsFromWorld).
function hspeed(W, nb, n, fps, upm, i, f) {
  const o1 = (Math.min(n - 1, f + 1) * nb + i) * 16, o0 = (Math.max(0, f - 1) * nb + i) * 16;
  return Math.hypot(W[o1 + 12] - W[o0 + 12], W[o1 + 14] - W[o0 + 14]) * fps / 2 / upm;
}

// One ground for everything. The legacy estimate (3rd percentile of all foot/ball minima) sinks with any
// sustained penetration. This one keeps only slow-foot samples, drops the top 30 percent (feet held still
// in the air), finds the densest 2 cm band (where the feet spend most of their stance) and takes that
// band's 10th percentile, so a sink or float shorter than the time spent on the real floor cannot move it.
export function estimateGround({ W, nb, n, fps, upm, legs, slow = 0.3, band_cm = 2, keep = 0.7 }) {
  const all = [], stance = [];
  for (const s of SIDES) {
    const L = legs[s];
    if (L?.foot === undefined) continue;
    const ball = L.ball ?? L.foot;
    for (let f = 0; f < n; f++) {
      const h = Math.min(wy(W, nb, f, L.foot), wy(W, nb, f, ball));
      all.push(h);
      if (hspeed(W, nb, n, fps, upm, ball, f) < slow) stance.push(h);
    }
  }
  all.sort((x, y) => x - y); stance.sort((x, y) => x - y);
  const legacy = Number.isFinite(pctSorted(all, 0.03)) ? pctSorted(all, 0.03) : 0;
  if (stance.length < 8) return { ground: legacy, legacy, method: 'all-p3', stanceSamples: stance.length };
  const lo = stance.slice(0, Math.max(8, Math.ceil(stance.length * keep))), w = band_cm * upm / 100;
  let bi = 0, bj = 0;
  for (let i = 0, j = 0; i < lo.length; i++) { while (lo[j] < lo[i] - w) j++; if (i - j > bi - bj) { bi = i; bj = j; } }
  return { ground: pctSorted(lo.slice(bj, bi + 1), 0.1), legacy, method: 'stance-band', stanceSamples: stance.length };
}

export function comSegments(rig) {
  return MASS_SEGMENTS.map(([a, b, m]) => [firstOf(rig, a), firstOf(rig, b), m]).filter(s => s[0] !== undefined && s[1] !== undefined);
}

export function comSeries(W, nb, n, segs) {
  const c = new Float64Array(n * 3);
  for (let f = 0; f < n; f++) {
    let m = 0, x = 0, y = 0, z = 0;
    for (const [a, b, w] of segs) {
      const ia = (f * nb + a) * 16 + 12, ib = (f * nb + b) * 16 + 12;
      x += (W[ia] + W[ib]) / 2 * w; y += (W[ia + 1] + W[ib + 1]) / 2 * w; z += (W[ia + 2] + W[ib + 2]) / 2 * w; m += w;
    }
    if (m > 0) { c[f * 3] = x / m; c[f * 3 + 1] = y / m; c[f * 3 + 2] = z / m; }
  }
  return c;
}

// COM acceleration exactly as analyze(): 5-tap moving average, then a clamped second difference.
export function comAccel(com, n, fps) {
  const sm = new Float64Array(n * 3), acc = new Float64Array(n * 3);
  for (let f = 0; f < n; f++) for (let k = 0; k < 3; k++) {
    let s = 0, w = 0;
    for (let d = -2; d <= 2; d++) { const g = Math.min(n - 1, Math.max(0, f + d)); s += com[g * 3 + k]; w++; }
    sm[f * 3 + k] = s / w;
  }
  for (let f = 0; f < n; f++) {
    const a = Math.max(0, f - 1), b = Math.min(n - 1, f + 1);
    for (let k = 0; k < 3; k++) acc[f * 3 + k] = (sm[b * 3 + k] - 2 * sm[f * 3 + k] + sm[a * 3 + k]) * fps * fps;
  }
  return acc;
}

export function facingSeries(W, nb, n, rig) {
  const fwd = new Float32Array(n * 2), tl = rig.thigh_l, tr = rig.thigh_r, fl = rig.foot_l, bl = firstOf(rig, 'ball_l|foot_l');
  for (let f = 0; f < n; f++) {
    let fx = 0, fz = 1;
    if (tl !== undefined && tr !== undefined) {
      const a = wpos(W, nb, f, tl), b = wpos(W, nb, f, tr), rx = b[0] - a[0], rz = b[2] - a[2], rl = Math.hypot(rx, rz) || 1;
      fx = rz / rl; fz = -rx / rl;
      if (fl !== undefined && bl !== undefined && bl !== fl) {
        const p = wpos(W, nb, f, fl), q = wpos(W, nb, f, bl);
        if (fx * (q[0] - p[0]) + fz * (q[2] - p[2]) < 0) { fx = -fx; fz = -fz; }
      }
    }
    fwd[f * 2] = fx; fwd[f * 2 + 1] = fz;
  }
  return fwd;
}

function rotOf(W, o) {
  const c = k => { const l = Math.hypot(W[o + k * 4], W[o + k * 4 + 1], W[o + k * 4 + 2]) || 1; return [W[o + k * 4] / l, W[o + k * 4 + 1] / l, W[o + k * 4 + 2] / l]; };
  return [c(0), c(1), c(2)];
}
function quatFromCols(c0, c1, c2) {
  const m00 = c0[0], m10 = c0[1], m20 = c0[2], m01 = c1[0], m11 = c1[1], m21 = c1[2], m02 = c2[0], m12 = c2[1], m22 = c2[2], t = m00 + m11 + m22;
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4]; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; q = [s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s]; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; q = [(m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s]; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; q = [(m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s]; }
  const l = Math.hypot(...q) || 1;
  return q.map(v => v / l);
}
// Relative rotation inv(Rparent) * Rchild from world matrices, as a quaternion track (n*4).
export function relativeQuatTrack(W, nb, n, parent, child) {
  const out = new Float64Array(n * 4);
  let prev = null;
  for (let f = 0; f < n; f++) {
    const A = rotOf(W, (f * nb + parent) * 16), B = rotOf(W, (f * nb + child) * 16);
    const d = (u, v) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    const cols = [0, 1, 2].map(j => [d(A[0], B[j]), d(A[1], B[j]), d(A[2], B[j])]);
    let q = quatFromCols(cols[0], cols[1], cols[2]);
    if (prev && q[0] * prev[0] + q[1] * prev[1] + q[2] * prev[2] + q[3] * prev[3] < 0) q = q.map(v => -v);
    out.set(q, f * 4); prev = q;
  }
  return out;
}

function kneeTracks(W, nb, n, legs, fwd) {
  const out = {};
  for (const s of SIDES) {
    const L = legs[s];
    if ([L.thigh, L.calf, L.foot].some(v => v === undefined)) continue;
    const flex = new Float32Array(n);
    for (let f = 0; f < n; f++) {
      const H = wpos(W, nb, f, L.thigh), K = wpos(W, nb, f, L.calf), A = wpos(W, nb, f, L.foot);
      const u = [K[0] - H[0], K[1] - H[1], K[2] - H[2]], v = [A[0] - K[0], A[1] - K[1], A[2] - K[2]];
      const lu = Math.hypot(...u) || 1, lv = Math.hypot(...v) || 1;
      const ang = Math.acos(Math.max(-1, Math.min(1, (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]) / (lu * lv)))) * 180 / Math.PI;
      const ha = [A[0] - H[0], A[1] - H[1], A[2] - H[2]], la = Math.hypot(...ha) || 1, t = (u[0] * ha[0] + u[1] * ha[1] + u[2] * ha[2]) / (la * la);
      const off = [u[0] - ha[0] * t, u[2] - ha[2] * t], side = off[0] * fwd[f * 2] + off[1] * fwd[f * 2 + 1];
      flex[f] = side < -1e-3 * lu ? -ang : ang;
    }
    out[s] = { flex, rel: relativeQuatTrack(W, nb, n, L.thigh, L.calf) };
  }
  return out;
}

export function phaseInput({ W, nb, n, fps, upm, rig, restW = null }) {
  const map = [];
  for (const k of PHASE_BONES) {
    let i = rig[k];
    for (const alt of PHASE_FALLBACK[k] || []) if (i === undefined) i = rig[alt];
    if (i === undefined) return null;
    map.push(i);
  }
  const k = 100 / upm, conv = p => [p[0] * k, -p[2] * k, p[1] * k], B = [];
  for (let f = 0; f < n; f++) B.push(map.map(i => conv(wpos(W, nb, f, i))));
  const rest = map.map(i => conv(restPos(restW, i) ?? wpos(W, nb, 0, i)));
  return { fps, B, rest, frame_offset: 0 };
}

function phasesOf(input, n) {
  if (input && segmentPhases) {
    try { const p = segmentPhases(input); if (p?.length) return p.map(x => ({ phase: x.phase, start: x.start, end: x.end })); } catch {}
  }
  return [{ phase: 'freeform', start: 0, end: n - 1 }];
}

export function phaseOf(facts, a, b = a) {
  const cnt = new Map();
  for (let f = Math.max(0, a); f <= Math.min(facts.n - 1, b); f++) { const p = facts.phaseAt[f]; cnt.set(p, (cnt.get(p) || 0) + 1); }
  let best = null, m = -1;
  for (const [p, c] of cnt) if (c > m) { m = c; best = p; }
  return best ?? 'freeform';
}

function contactRuns(W, nb, n, fps, upm, ground, rig, kind, maxHeight, o) {
  const cands = SIDES.map(s => ({ name: `${kind}_${s}`, side: s, idx: rig[(kind === 'knee' ? 'calf_' : 'hand_') + s] })).filter(c => c.idx !== undefined);
  const runs = detectFulcrums(W, nb, n, fps, upm, cands, ground, o.maxSpeed ?? 0.25, maxHeight, o.minFrames ?? 4);
  const out = { l: { runs: [], mask: new Uint8Array(n) }, r: { runs: [], mask: new Uint8Array(n) } };
  for (const r of runs) {
    const side = r.pivot.endsWith('_l') ? 'l' : 'r';
    let anchor = r.a, best = Infinity;
    for (let f = r.a; f <= r.b; f++) { const v = hspeed(W, nb, n, fps, upm, r.idx, f); if (v < best) { best = v; anchor = f; } }
    out[side].runs.push({ side, a: r.a, b: r.b, idx: r.idx, pivot: r.pivot, anchor, slip_cm: pivotSlip(W, nb, r.idx, r.a, r.b, upm) });
    out[side].mask.fill(1, r.a, r.b + 1);
  }
  return out;
}

function footCorners(W, nb, f, L, upm) {
  const A = wpos(W, nb, f, L.foot), B = wpos(W, nb, f, L.ball ?? L.foot);
  let fx = B[0] - A[0], fz = B[2] - A[2];
  const l = Math.hypot(fx, fz);
  if (l < 1e-6 * upm) { fx = 0; fz = 1; } else { fx /= l; fz /= l; }
  const lx = fz, lz = -fx, w = 0.05 * upm, back = 0.06 * upm, front = 0.05 * upm, out = [];
  for (const [P, d] of [[A, -back], [B, front]]) for (const s of [-1, 1]) out.push([P[0] + fx * d + lx * w * s, 0, P[2] + fz * d + lz * w * s]);
  return out;
}

export function computeFacts({ W, nb, n, fps = 30, upm = 100, rig, restW = null, shots = [], trust = null, ground = null, opts = {} }) {
  n = n ?? W.length / (nb * 16);
  const legs = legsOf(rig), g = 9.81 * upm, u = upm / 100;
  const L = legs.l.thigh !== undefined && legs.l.calf !== undefined && legs.l.foot !== undefined ? legs.l : legs.r;
  const legLength_cm = L.thigh !== undefined && L.calf !== undefined && L.foot !== undefined ? (boneLength(W, nb, n, restW, L.thigh, L.calf) + boneLength(W, nb, n, restW, L.calf, L.foot)) / u : 90;
  const footLength_cm = L.ball !== undefined && L.ball !== L.foot ? Math.max(20, 1.6 * boneLength(W, nb, n, restW, L.foot, L.ball) / u) : 26;
  const gr = estimateGround({ W, nb, n, fps, upm, legs });
  const G = Number.isFinite(ground) ? ground : gr.ground;

  const per = { l: new Uint8Array(n), r: new Uint8Array(n) }, speed = { l: new Float32Array(n), r: new Float32Array(n) }, height = { l: new Float32Array(n), r: new Float32Array(n) };
  const feetDown = new Uint8Array(n);
  for (const s of SIDES) {
    const Lg = legs[s];
    if (Lg.foot === undefined) continue;
    for (let f = 0; f < n; f++) {
      const h = Math.min(wy(W, nb, f, Lg.foot), wy(W, nb, f, Lg.ball)) - G;
      height[s][f] = h / u;
      speed[s][f] = hspeed(W, nb, n, fps, upm, Lg.ball, f);
      per[s][f] = h < 0.1 * upm ? 1 : 0;
      if (per[s][f]) feetDown[f] = 1;
    }
  }

  const plants = legs.l.foot === undefined && legs.r.foot === undefined ? [] : detectFootPlants({ W, nb, n, fps, upm, ground: G,
    legs: { l: { foot: legs.l.foot, ball: legs.l.ball }, r: { foot: legs.r.foot, ball: legs.r.ball } }, legLength_cm, footLength_cm, ...(opts.plants || {}) });
  const slideSpeed = opts.slideSpeed ?? 0.3;
  const feet = {};
  for (const s of SIDES) {
    const mask = new Uint8Array(n), mine = plants.filter(p => p.side === s);
    for (const p of mine) {
      p.lockable = !p.sliding && !p.touch;
      const sk = new Uint8Array(p.b - p.a + 1);
      for (let f = p.a; f <= p.b; f++) sk[f - p.a] = speed[s][f] > slideSpeed ? 1 : 0;
      p.skate = runsOf(sk, 2).map(([a, b]) => { let pk = 0; for (let f = a + p.a; f <= b + p.a; f++) pk = Math.max(pk, speed[s][f]); return { a: a + p.a, b: b + p.a, peak_ms: pk }; });
      mask.fill(p.touch ? 3 : p.sliding ? 2 : 1, p.a, p.b + 1);
    }
    const fy = [], by = [];
    const Lg = legs[s];
    if (Lg.foot !== undefined) for (let f = 0; f < n; f++) if (per[s][f] && speed[s][f] < 0.3) { fy.push((wy(W, nb, f, Lg.foot) - G) / u); by.push((wy(W, nb, f, Lg.ball) - G) / u); }
    feet[s] = { plants: mine, mask, level: { foot_cm: fy.length ? median(fy) : 0, ball_cm: by.length ? median(by) : 0 } };
  }

  const hands = contactRuns(W, nb, n, fps, upm, G, rig, 'hand', opts.handHeight ?? 0.08, opts);
  const knees = contactRuns(W, nb, n, fps, upm, G, rig, 'knee', opts.kneeHeight ?? 0.10, opts);
  const support = new Uint8Array(n);
  for (let f = 0; f < n; f++) {
    let m = 0;
    if (per.l[f] || feet.l.mask[f] === 1) m |= 1;
    if (per.r[f] || feet.r.mask[f] === 1) m |= 2;
    if (hands.l.mask[f]) m |= 4;
    if (hands.r.mask[f]) m |= 8;
    if (knees.l.mask[f]) m |= 16;
    if (knees.r.mask[f]) m |= 32;
    support[f] = m;
  }

  const segs = comSegments(rig), com = comSeries(W, nb, n, segs), comAcc = comAccel(com, n, fps);
  const margin = new Float32Array(n).fill(NaN);
  for (let f = 0; f < n; f++) {
    const pts = [];
    for (const s of SIDES) if (feet[s].mask[f] === 1 || (per[s][f] && speed[s][f] < 0.25)) pts.push(...footCorners(W, nb, f, legs[s], upm));
    for (const C of [hands, knees]) for (const s of SIDES) if (C[s].mask[f]) { const r = C[s].runs.find(x => f >= x.a && f <= x.b); if (r) pts.push(wpos(W, nb, f, r.idx)); }
    const m = supportMargin([com[f * 3], com[f * 3 + 1], com[f * 3 + 2]], supportHull(pts));
    if (m != null) margin[f] = m / u;
  }

  const input = phaseInput({ W, nb, n, fps, upm, rig, restW });
  const phases = phasesOf(input, n), phaseAt = new Array(n).fill('freeform');
  for (const p of phases) for (let f = Math.max(0, p.start); f <= Math.min(n - 1, p.end); f++) phaseAt[f] = p.phase;

  const anySupport = Uint8Array.from(support, m => m ? 1 : 0);
  const flights = detectFlights(anySupport, opts.minFlight ?? 3).filter(x => x.b < n).map(x => {
    const report = flightReport(fitBallistic(com, x.a, x.b, fps, g), upm);
    let fl = 0;
    for (let f = x.a + 1; f < x.b; f++) if (phaseAt[f] === 'flight') fl++;
    return { a: x.a, b: x.b, report, plausible: report.airtime_s <= 0.8 && report.apex_m_above_takeoff <= 0.9 && report.apex_m_above_takeoff >= -0.05, flightPhase: fl / Math.max(1, x.b - x.a - 1) };
  });

  const sh = [...(shots || [])].filter(s => s && s.n > 0).sort((a, b) => a.start - b.start);
  const seams = sh.slice(1).map((s, k) => ({ shot: k + 1, at: s.start, name: s.name, prev: sh[k].name }));
  const nearSeam = new Uint8Array(n), seamR = opts.seamRadius ?? 6;
  for (const s of seams) for (let f = Math.max(0, s.at - seamR); f <= Math.min(n - 1, s.at + seamR); f++) nearSeam[f] = 1;

  const fwd = facingSeries(W, nb, n, rig);
  const pelvisSpeed = new Float32Array(n);
  if (rig.pelvis !== undefined) for (let f = 0; f < n; f++) pelvisSpeed[f] = hspeed(W, nb, n, fps, upm, rig.pelvis, f);

  const tr = new Float32Array(n).fill(1);
  if (trust) for (let f = 0; f < n; f++) { const v = +trust[f]; tr[f] = Number.isFinite(v) ? Math.max(0, Math.min(1, v)) : 1; }

  const footRest = {};
  for (const s of SIDES) { const R = restRot(restW, legs[s].foot); if (R) footRest[s] = R; }

  return {
    n, nb, fps, upm, rig, legs, missing: missingBones(rig), legLength_cm, footLength_cm,
    ground: G, groundLegacy: gr.legacy, groundMethod: Number.isFinite(ground) ? 'given' : gr.method,
    contacts: { per, speed, height, feetDown, feet, hands, knees, support, any: anySupport },
    phases, phaseAt, phasesSource: input ? PHASES_SOURCE ?? 'none' : 'missing-bones',
    com, comAcc, balance: { marginCm: margin },
    flights, seams, nearSeam, knees: kneeTracks(W, nb, n, legs, fwd), facing: fwd, pelvisSpeed, trust: tr, footRest,
  };
}

// Plain-language frame range helper shared by the judge, planner and report.
export const frameText = (a, b) => a == null ? '' : a === b ? `f${a}` : `f${a}-${b}`;
export const sideText = s => s === 'l' ? 'left' : s === 'r' ? 'right' : 'both';
