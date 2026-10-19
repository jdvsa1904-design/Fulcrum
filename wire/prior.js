// Library-learned motion prior: per-phase robust envelopes (frame quantiles) of joint angles, joint angular
// speeds, pelvis height and speed, plant drift and gait timing, learned from the motion library by
// tools/motionlib_prior.mjs into wire/motion_prior.json. priorFindings() scores a take against it and returns
// findings in judge.js format (rule 'unnatural_<what>'). Pure: no DOM, no fetch; the caller supplies the JSON.
//
// Knees: the library knees never pass straight (its rigs clamp knee flexion at about 0.18 deg) and lean bent
// (idle knees ~30 deg), so the knee minimum is never a "must bend" rule. Knees are checked only for over-bend
// (upper band), speed, and bending backwards past a fixed anatomical limit the library itself never reaches.
import { relativeQuatTrack, runsOf, phaseOf, frameText, sideText, wpos } from './facts.js';

export const PRIOR_VERSION = 1;
export const QUANTILES = [0.005, 0.02, 0.5, 0.98, 0.995];
export const PRIOR_DEFAULTS = Object.freeze({
  kWarn: 1, kBlock: 3, kWarnSpeed: 1, kBlockSpeed: 3, minRunPose: 3, minRunSpeed: 1, blockRun: 3, frozenSecs: 0.35, frozenWin: 0.5, frozenFrac: 0.35, frozenMin: 30, soleCm: 2.5,
  kneeHyperDeg: 10, minClips: 6, minFrames: 150, gaitSlack: 1.35, gaitMinN: 6, driftSlack: 1.5,
});
const SIDES = ['l', 'r'];
const D = 180 / Math.PI;

// Feature table. check: which tail is judged. tol: absolute floor of the band spread (same unit as the feature).
// group: rule name suffix. Speeds are deg/s; heights and pelvis speeds are in leg lengths (per second).
export const FEATURES = Object.freeze({
  knee_flex: { unit: 'deg', sided: true, check: 'hi', tol: 6, what: 'knee', text: 'bends' },
  hip_flex: { unit: 'deg', sided: true, check: 'both', tol: 8, what: 'hip', text: 'swings' },
  elbow_flex: { unit: 'deg', sided: true, check: 'hi', tol: 8, what: 'elbow', text: 'bends' },
  trunk_flex: { unit: 'deg', sided: false, check: 'hi', tol: 8, what: 'trunk', text: 'leans' },
  speed_knee: { unit: 'deg/s', sided: true, check: 'hi', tol: 120, what: 'knee', speed: true },
  speed_hip: { unit: 'deg/s', sided: true, check: 'hi', tol: 120, what: 'hip', speed: true },
  speed_ankle: { unit: 'deg/s', sided: true, check: 'hi', tol: 150, what: 'ankle', speed: true },
  speed_elbow: { unit: 'deg/s', sided: true, check: 'hi', tol: 150, what: 'elbow', speed: true },
  speed_shoulder: { unit: 'deg/s', sided: true, check: 'hi', tol: 150, what: 'shoulder', speed: true },
  speed_wrist: { unit: 'deg/s', sided: true, check: 'hi', tol: 200, what: 'wrist', speed: true },
  speed_spine: { unit: 'deg/s', sided: false, check: 'hi', tol: 80, what: 'spine', speed: true },
  speed_neck: { unit: 'deg/s', sided: false, check: 'hi', tol: 120, what: 'neck', speed: true },
  // Snap: how far a joint's rotation leaves the midpoint of its neighbours (deg). Fast smooth motion stays near the
  // midpoint; a one-frame pop leaves it by the pop angle and comes back.
  snap_knee: { unit: 'deg', sided: true, check: 'hi', tol: 4, what: 'knee', speed: true, snap: true },
  snap_hip: { unit: 'deg', sided: true, check: 'hi', tol: 4, what: 'hip', speed: true, snap: true },
  snap_ankle: { unit: 'deg', sided: true, check: 'hi', tol: 5, what: 'ankle', speed: true, snap: true },
  snap_elbow: { unit: 'deg', sided: true, check: 'hi', tol: 5, what: 'elbow', speed: true, snap: true },
  snap_shoulder: { unit: 'deg', sided: true, check: 'hi', tol: 5, what: 'shoulder', speed: true, snap: true },
  snap_wrist: { unit: 'deg', sided: true, check: 'hi', tol: 6, what: 'wrist', speed: true, snap: true },
  snap_spine: { unit: 'deg', sided: false, check: 'hi', tol: 3, what: 'spine', speed: true, snap: true },
  snap_neck: { unit: 'deg', sided: false, check: 'hi', tol: 4, what: 'neck', speed: true, snap: true },
  pelvis_height: { unit: 'leg', sided: false, check: 'hi', tol: 0.06, what: 'pelvis', text: 'rides' },
  pelvis_speed: { unit: 'leg/s', sided: false, check: 'hi', tol: 0.6, what: 'pelvis', text: 'travels' },
  pelvis_vspeed: { unit: 'leg/s', sided: false, check: 'hi', tol: 0.4, what: 'pelvis', text: 'moves up or down' },
  sole_speed: { unit: 'leg/s', sided: true, check: 'hi', tol: 0.12, what: 'foot', text: 'slides' },
  leg_motion: { unit: 'deg', sided: true, check: 'lo', what: 'leg', frozen: true },
});
// Joint groups for angular speed: [parent spec, child spec]; '|' lists fallbacks.
export const SPEED_JOINTS = Object.freeze({
  speed_knee: ['thigh_S', 'calf_S'], speed_hip: ['pelvis', 'thigh_S'], speed_ankle: ['calf_S', 'foot_S'], speed_elbow: ['upperarm_S', 'lowerarm_S'],
  speed_shoulder: ['clavicle_S|spine_03|spine_02', 'upperarm_S'], speed_wrist: ['lowerarm_S', 'hand_S'], speed_spine: ['pelvis', 'spine_03|spine_02|neck_01'], speed_neck: ['spine_03|spine_02', 'head|neck_01'],
});
// Bones each rule may touch when a fix exists (fixplan maps speed spikes to despike/smooth on these).
export const GROUP_BONES = Object.freeze({
  knee: s => ['calf_' + s], hip: s => ['thigh_' + s], ankle: s => ['foot_' + s, 'ball_' + s], elbow: s => ['lowerarm_' + s], shoulder: s => ['upperarm_' + s, 'clavicle_' + s],
  wrist: s => ['hand_' + s], spine: () => ['spine_01', 'spine_02', 'spine_03'], neck: () => ['neck_01', 'head'],
});
// Phase fallback when the library has too few clips of a phase.
export const PHASE_FAMILY = Object.freeze({ locomotion: 'gait', in_place: 'gait', crouch: 'low', kneel: 'low', prone: 'low', slide: 'low', rise: 'low', stand: 'still', upper_body: 'still', plant: 'still', flight: 'air', vault: 'air', freeform: 'free' });
const PHASE_WORDS = { locomotion: 'walk or run', in_place: 'walk in place', flight: 'jump', vault: 'vault', plant: 'planted dip', slide: 'slide', kneel: 'kneel', crouch: 'crouch', prone: 'prone move', rise: 'rise', stand: 'stand', upper_body: 'stand', freeform: 'free move' };

const first = (rig, spec, s) => { for (const k of spec.split('|')) { const i = rig[k.replace('_S', '_' + s)]; if (i !== undefined) return i; } return undefined; };
const norm = v => { const l = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / l, v[1] / l, v[2] / l]; };
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const angAt = (a, b, c) => { const u = norm(sub(a, b)), v = norm(sub(c, b)); return Math.acos(Math.max(-1, Math.min(1, dot(u, v)))) * D; };
const qang = (q, i, j) => { const d = Math.abs(q[i * 4] * q[j * 4] + q[i * 4 + 1] * q[j * 4 + 1] + q[i * 4 + 2] * q[j * 4 + 2] + q[i * 4 + 3] * q[j * 4 + 3]); return 2 * Math.acos(Math.min(1, d)) * D; };
const r3 = v => Number.isFinite(v) ? +v.toFixed(3) : null;

// Signed knee flexion (deg). The sign comes from the leg itself, not the body facing: a knee bends toward the toes,
// so a knee point behind the hip-ankle line (opposite the ankle->ball direction) is bending backwards. When the foot
// points along the leg (no usable toe direction), or the knee is bent past 40 deg (a folded leg: kneeling on the
// instep turns the toes backwards), the angle stays positive. facts.knees uses the body facing, which
// flips in prone, sitting and deep-crouch poses.
export function kneeFlex(W, nb, n, th, ca, fo, ba) {
  const out = new Float32Array(n);
  for (let f = 0; f < n; f++) {
    const H = wpos(W, nb, f, th), K = wpos(W, nb, f, ca), A = wpos(W, nb, f, fo), ang = 180 - angAt(H, K, A);
    let sign = 1;
    if (ba !== undefined && ba !== fo) {
      const L = norm(sub(A, H)), B = wpos(W, nb, f, ba), toe = sub(B, A), kt = dot(toe, L), tp = [toe[0] - L[0] * kt, toe[1] - L[1] * kt, toe[2] - L[2] * kt];
      const ko = sub(K, H), kk = dot(ko, L), kp = [ko[0] - L[0] * kk, ko[1] - L[1] * kk, ko[2] - L[2] * kk];
      if (ang < 40 && Math.hypot(...tp) > 0.3 * Math.hypot(...toe) && Math.hypot(...kp) > 1e-4 * Math.hypot(...ko) && dot(tp, kp) < 0) sign = -1;
    }
    out[f] = sign * ang;
  }
  return out;
}

// Per-frame feature tracks of one take. facts = computeFacts() of the same W.
export function priorFeatures({ W, nb, n, fps, rig, facts }) {
  n = n ?? facts.n; fps = fps ?? facts.fps;
  const upm = facts.upm, leg = facts.legLength_cm / 100, g = facts.ground, out = {};
  const P = (f, k) => wpos(W, nb, f, k);
  const top = rig.neck_01 ?? rig.head ?? rig.spine_03 ?? rig.spine_02;
  const trunk = new Float32Array(n), up = [];
  for (let f = 0; f < n; f++) {
    const u = top !== undefined ? norm(sub(P(f, top), P(f, rig.pelvis))) : [0, 1, 0];
    up.push(u); trunk[f] = Math.acos(Math.max(-1, Math.min(1, u[1]))) * D;
  }
  out.trunk_flex = trunk;
  for (const s of SIDES) {
    const th = rig['thigh_' + s], ca = rig['calf_' + s], fo = rig['foot_' + s], ba = rig['ball_' + s];
    if (th !== undefined && ca !== undefined && fo !== undefined) out['knee_flex_' + s] = kneeFlex(W, nb, n, th, ca, fo, ba);
    if (th !== undefined && ca !== undefined) {
      const h = new Float32Array(n);
      for (let f = 0; f < n; f++) {
        const u = up[f], fw = [facts.facing[f * 2], 0, facts.facing[f * 2 + 1]], k = dot(fw, u), fp = norm([fw[0] - u[0] * k, fw[1] - u[1] * k, fw[2] - u[2] * k]), d = norm(sub(P(f, ca), P(f, th)));
        h[f] = Math.atan2(dot(d, fp), -dot(d, u)) * D;
      }
      out['hip_flex_' + s] = h;
    }
    const ua = rig['upperarm_' + s], la = rig['lowerarm_' + s], hd = rig['hand_' + s];
    if (ua !== undefined && la !== undefined && hd !== undefined) { const e = new Float32Array(n); for (let f = 0; f < n; f++) e[f] = 180 - angAt(P(f, ua), P(f, la), P(f, hd)); out['elbow_flex_' + s] = e; }
  }
  for (const [name, [ps, cs]] of Object.entries(SPEED_JOINTS)) for (const s of FEATURES[name].sided ? SIDES : ['']) {
    const pi = first(rig, ps, s), ci = first(rig, cs, s);
    if (pi === undefined || ci === undefined || pi === ci || n < 2) continue;
    const q = relativeQuatTrack(W, nb, n, pi, ci), v = new Float32Array(n), sn = new Float32Array(n);
    for (let f = 1; f < n; f++) v[f] = qang(q, f - 1, f) * fps;
    v[0] = v[1];
    for (let f = 1; f < n - 1; f++) {
      const m = [0, 1, 2, 3].map(k => q[(f - 1) * 4 + k] + q[(f + 1) * 4 + k]), l = Math.hypot(...m) || 1, d = Math.abs(m.reduce((a, x, k) => a + x / l * q[f * 4 + k], 0));
      sn[f] = 2 * Math.acos(Math.min(1, d)) * D;
    }
    out[name + (s ? '_' + s : '')] = v;
    out[name.replace('speed_', 'snap_') + (s ? '_' + s : '')] = sn;
  }
  if (rig.pelvis !== undefined) {
    const h = new Float32Array(n), vs = new Float32Array(n), hs = new Float32Array(n);
    for (let f = 0; f < n; f++) {
      h[f] = (P(f, rig.pelvis)[1] - g) / upm / leg;
      const a = P(Math.max(0, f - 1), rig.pelvis), b = P(Math.min(n - 1, f + 1), rig.pelvis), dt = (Math.min(n - 1, f + 1) - Math.max(0, f - 1)) / fps || 1;
      vs[f] = Math.abs(b[1] - a[1]) / dt / upm / leg; hs[f] = facts.pelvisSpeed[f] / leg;
    }
    out.pelvis_height = h; out.pelvis_speed = hs; out.pelvis_vspeed = vs;
  }
  // Leg motion: rotation path (deg) of hip + knee over a centred window; a frozen leg reads near 0.
  const w = Math.max(2, Math.round(PRIOR_DEFAULTS.frozenWin * fps / 2));
  for (const s of SIDES) {
    const a = out['speed_hip_' + s], b = out['speed_knee_' + s];
    if (!a || !b) continue;
    const c = new Float64Array(n + 1);
    for (let f = 0; f < n; f++) c[f + 1] = c[f] + (a[f] + b[f]) / fps;
    const m = new Float32Array(n);
    for (let f = 0; f < n; f++) { const lo = Math.max(0, f - w), hi = Math.min(n - 1, f + w); m[f] = (c[hi + 1] - c[lo]) * (2 * w + 1) / (hi - lo + 1); }
    out['leg_motion_' + s] = m;
  }
  // Horizontal speed of a foot flat on the floor: ankle and ball both within soleCm of their planted levels
  // (facts.contacts.feet[s].level). A heel strike or toe-off lifts one of them, so swing feet do not count. NaN otherwise.
  for (const s of SIDES) {
    const L = facts.legs[s], lv = facts.contacts.feet[s]?.level;
    if (L?.foot === undefined || !lv) continue;
    const u = upm / 100, tol = PRIOR_DEFAULTS.soleCm, v = new Float32Array(n);
    for (let f = 0; f < n; f++) {
      const hf = (W[(f * nb + L.foot) * 16 + 13] - g) / u, hb = (W[(f * nb + L.ball) * 16 + 13] - g) / u;
      v[f] = Math.abs(hf - lv.foot_cm) < tol && Math.abs(hb - lv.ball_cm) < tol && hb < 6 ? facts.contacts.speed[s][f] / leg : NaN;
    }
    out['sole_speed_' + s] = v;
  }
  const plants = [];
  for (const s of SIDES) for (const p of facts.contacts.feet[s].plants) if (!p.touch) plants.push({ side: s, a: p.a, b: p.b, drift_cm: p.drift_cm, point: p.point, phase: phaseOf(facts, p.a, p.b) });
  const gait = [];
  for (const seg of facts.phases) {
    if (seg.phase !== 'locomotion' || seg.end - seg.start + 1 < Math.round(0.5 * fps)) continue;
    const len = seg.end - seg.start + 1, onsets = [], duty = {};
    let sp = 0; for (let f = seg.start; f <= seg.end; f++) sp += facts.pelvisSpeed[f];
    for (const s of SIDES) {
      let st = 0; const mask = facts.contacts.feet[s].mask;
      for (let f = seg.start; f <= seg.end; f++) if (mask[f] === 1 || mask[f] === 2) st++;
      duty[s] = st / len;
      for (const p of facts.contacts.feet[s].plants) if (!p.touch && p.a > seg.start && p.a <= seg.end) onsets.push([p.a, s]);
    }
    onsets.sort((x, y) => x[0] - y[0]);
    const half = [];
    for (let k = 1; k < onsets.length; k++) if (onsets[k][1] !== onsets[k - 1][1]) half.push((onsets[k][0] - onsets[k - 1][0]) / fps);
    half.sort((x, y) => x - y);
    gait.push({ a: seg.start, b: seg.end, speed: sp / len / leg, duty: (duty.l + duty.r) / 2, period: half.length ? 2 * half[half.length >> 1] : null });
  }
  return { n, fps, leg, phaseAt: facts.phaseAt.slice(), tracks: out, plants, gait };
}

// ---------- model building (used by tools/motionlib_prior.mjs; pure) ----------
const qOf = (sorted, p) => sorted.length ? sorted[Math.min(sorted.length - 1, Math.max(0, Math.round(p * (sorted.length - 1))))] : NaN;
function envelope(vals, clips) {
  const s = Float64Array.from(vals).sort();
  return { q: QUANTILES.map(p => r3(qOf(s, p))), min: r3(s[0]), max: r3(s[s.length - 1]), n: s.length, clips };
}
const featureOf = key => key.replace(/_(l|r)$/, '');
const famOf = p => PHASE_FAMILY[p] ?? 'free';

// clips: [{ features: priorFeatures(...), name? }]. Returns the model JSON (without provenance).
export function buildPrior(clips, opts = {}) {
  const o = { ...PRIOR_DEFAULTS, ...opts };
  const acc = {}, clipSets = {};
  const push = (feat, phase, v, ci) => {
    for (const p of [phase, 'fam:' + famOf(phase), 'all']) {
      ((acc[feat] ??= {})[p] ??= []).push(v);
      ((clipSets[feat] ??= {})[p] ??= new Set()).add(ci);
    }
  };
  clips.forEach((c, ci) => {
    const F = c.features;
    for (const [key, tr] of Object.entries(F.tracks)) {
      const feat = featureOf(key);
      if (!FEATURES[feat]) continue;
      for (let f = 0; f < F.n; f++) if (Number.isFinite(tr[f])) push(feat, F.phaseAt[f], tr[f], ci);
    }
    for (const p of F.plants) push('plant_drift', p.phase, p.drift_cm, ci);
  });
  const env = {};
  for (const [feat, byPhase] of Object.entries(acc)) {
    env[feat] = {};
    for (const [p, vals] of Object.entries(byPhase)) {
      const nc = clipSets[feat][p].size, enough = p === 'all' || (nc >= o.minClips && vals.length >= (feat === 'plant_drift' ? o.minClips : o.minFrames));
      if (enough) env[feat][p] = envelope(vals, nc);
    }
  }
  // Gait timing vs speed (leg lengths per second), forward locomotion segments only.
  const bins = [0, 0.6, 1.2, 1.8, 2.6, 3.6, 5, 99], gait = bins.slice(0, -1).map((lo, k) => ({ lo, hi: bins[k + 1], duty: [], period: [] }));
  for (const c of clips) for (const g of c.features.gait) {
    const b = gait.find(x => g.speed >= x.lo && g.speed < x.hi);
    if (!b) continue;
    b.duty.push(g.duty); if (g.period) b.period.push(g.period);
  }
  const gaitEnv = gait.map(b => ({ lo: b.lo, hi: b.hi, duty: b.duty.length ? envelope(b.duty, b.duty.length) : null, period: b.period.length ? envelope(b.period, b.period.length) : null }));
  return { version: PRIOR_VERSION, quantiles: QUANTILES, params: { kWarn: o.kWarn, kBlock: o.kBlock, kWarnSpeed: o.kWarnSpeed, kBlockSpeed: o.kBlockSpeed, minRunPose: o.minRunPose, minRunSpeed: o.minRunSpeed, blockRun: o.blockRun, frozenSecs: o.frozenSecs, frozenFrac: o.frozenFrac, kneeHyperDeg: o.kneeHyperDeg, gaitSlack: o.gaitSlack, gaitMinN: o.gaitMinN, driftSlack: o.driftSlack },
    features: Object.fromEntries(Object.entries(FEATURES).map(([k, f]) => [k, { unit: f.unit, check: f.check, tol: f.tol ?? 0 }])), env, gait: gaitEnv };
}

// ---------- scoring ----------
// Validates the JSON and returns it (or null when it is not a prior model this code can read).
export function loadPrior(json) {
  const m = typeof json === 'string' ? JSON.parse(json) : json;
  if (!m || m.version !== PRIOR_VERSION || !m.env || !m.params) return null;
  return m;
}

const envFor = (model, feat, phase) => { const e = model.env[feat]; return e ? e[phase] ?? e['fam:' + famOf(phase)] ?? e.all ?? null : null; };
function bounds(model, feat, phase, k) {
  const e = envFor(model, feat, phase);
  if (!e) return null;
  const [q005, , q50, , q995] = e.q, tol = model.features[feat]?.tol ?? 0;
  return { hi: q995 + k * Math.max(q995 - q50, tol), lo: q005 - k * Math.max(q50 - q005, tol), e };
}

function mkFinding(rule, severity, a, b, o) {
  return { rule, severity, a, b, side: o.side ?? null, phase: o.phase ?? null, value: Number.isFinite(o.value) ? +o.value.toFixed(2) : null, limit: Number.isFinite(o.limit) ? +o.limit.toFixed(2) : null, unit: o.unit ?? null, source: 'prior', text: o.text.charAt(0).toUpperCase() + o.text.slice(1), ...(o.bones ? { bones: o.bones } : {}), ...(o.group ? { group: o.group } : {}), kind: o.kind ?? 'other' };
}

const fmt = (v, unit, leg) => unit === 'deg/s' ? `${Math.round(v)} deg/s` : unit === 'deg' ? `${Math.round(v)} deg` : unit === 'leg/s' ? `${(v * leg).toFixed(1)} m/s` : unit === 'leg' ? `${Math.round(v * 100)}% of leg length` : `${v.toFixed(1)} ${unit}`;

// Findings for one take. facts: computeFacts() output; W/nb/rig as the judge has them; features: precomputed (optional).
export function priorFindings({ model, facts, W, nb, rig = facts.rig, features = null, opts = {} }) {
  if (!model) return [];
  const P = { ...PRIOR_DEFAULTS, ...model.params, ...opts }, F = features ?? priorFeatures({ W, nb, n: facts.n, fps: facts.fps, rig, facts });
  const n = F.n, out = [], phaseAt = F.phaseAt, leg = F.leg;
  for (const [key, tr] of Object.entries(F.tracks)) {
    const feat = featureOf(key), spec = FEATURES[feat], side = key.match(/_(l|r)$/)?.[1] ?? null;
    if (!spec) continue;
    const who = `${side ? sideText(side) + ' ' : ''}${spec.what}`;
    if (spec.frozen) {
      const lo = new Float32Array(n), m = new Uint8Array(n);
      for (let f = 0; f < n; f++) { const e = envFor(model, feat, phaseAt[f]); lo[f] = e && e.q[0] > P.frozenMin ? e.q[0] * P.frozenFrac : -1; m[f] = tr[f] < lo[f] ? 1 : 0; }
      for (const [a, b] of runsOf(m, Math.max(3, Math.round(P.frozenSecs * F.fps)))) {
        let v = Infinity; for (let f = a; f <= b; f++) v = Math.min(v, tr[f]);
        const ph = phaseOf({ n, phaseAt }, a, b);
        out.push(mkFinding('unnatural_frozen_limb', 'warning', a, b, { side, phase: ph, value: v, limit: lo[a], unit: 'deg', group: 'leg', text: `${who} holds still for ${((b - a + 1) / F.fps).toFixed(1)} s in a ${PHASE_WORDS[ph] ?? ph}, ${frameText(a, b)} (moves ${Math.round(v)} deg per ${P.frozenWin ?? PRIOR_DEFAULTS.frozenWin} s; library legs move at least ${Math.round(envFor(model, feat, ph)?.q[0] ?? 0)} deg)` }));
      }
      continue;
    }
    const hiW = new Float32Array(n).fill(Infinity), hiB = new Float32Array(n).fill(Infinity), loW = new Float32Array(n).fill(-Infinity), loB = new Float32Array(n).fill(-Infinity);
    for (let f = 0; f < n; f++) {
      const bw = bounds(model, feat, phaseAt[f], spec.speed ? P.kWarnSpeed : P.kWarn), bb = bounds(model, feat, phaseAt[f], spec.speed ? P.kBlockSpeed : P.kBlock);
      if (!bw) continue;
      if (spec.check !== 'lo') { hiW[f] = bw.hi; hiB[f] = bb.hi; }
      if (spec.check === 'both') { loW[f] = bw.lo; loB[f] = bb.lo; }
    }
    // Knees bend backwards past a fixed limit only: the library minimum (~0, clamped) is never a lower band.
    if (feat === 'knee_flex') for (let f = 0; f < n; f++) { loW[f] = -P.kneeHyperDeg; loB[f] = -3 * P.kneeHyperDeg; }
    const minRun = spec.speed ? P.minRunSpeed : P.minRunPose;
    for (const [dir, wl, bl] of [['hi', hiW, hiB], ['lo', loW, loB]]) {
      const m = Uint8Array.from(tr, (v, f) => dir === 'hi' ? (v > wl[f] ? 1 : 0) : (v < wl[f] ? 1 : 0));
      for (const [a, b] of runsOf(m, minRun)) {
        let pk = a, over = 0;
        for (let f = a; f <= b; f++) { if (dir === 'hi' ? tr[f] > tr[pk] : tr[f] < tr[pk]) pk = f; if (dir === 'hi' ? tr[f] > bl[f] : tr[f] < bl[f]) over++; }
        const v = tr[pk], ph = phaseOf({ n, phaseAt }, a, b), e = envFor(model, feat, ph), sev = over >= P.blockRun ? 'blocker' : 'warning', r = frameText(a, b), where = PHASE_WORDS[ph] ?? ph;
        let rule, text;
        if (feat === 'knee_flex' && dir === 'lo') { rule = 'unnatural_knee_hyperextension'; text = `${who} bends backwards ${Math.round(-v)} deg, ${r}; library knees never pass straight`; }
        else if (spec.snap) { rule = `unnatural_${spec.what}_snap`; text = `${who} snaps ${Math.round(v)} deg away from its neighbouring frames and back, ${r}; library ${spec.what}s stay within ${fmt(e.q[4], spec.unit, leg)} in a ${where}`; }
        else if (spec.speed) { rule = `unnatural_${spec.what}_speed`; text = `${who} turns ${fmt(v, spec.unit, leg)}, ${r}, faster than the motion library allows in a ${where} (99.5% of library frames stay under ${fmt(e.q[4], spec.unit, leg)})`; }
        else if (feat === 'sole_speed') { rule = 'unnatural_foot_slide'; text = `${sideText(side)} foot slides ${fmt(v, spec.unit, leg)} while flat on the floor, ${r}; library soles stay under ${fmt(e.q[4], spec.unit, leg)} in a ${where}`; }
        else if (feat.startsWith('pelvis')) { rule = `unnatural_${feat}`; text = `Pelvis ${spec.text} ${fmt(v, spec.unit, leg)}, ${r}, outside the library range for a ${where} (${fmt(dir === 'hi' ? e.q[4] : e.q[0], spec.unit, leg)})`; }
        else { rule = `unnatural_${spec.what}_${dir === 'lo' ? 'extension' : 'flexion'}`; text = `${who} ${spec.text} ${fmt(Math.abs(v), spec.unit, leg)}${dir === 'lo' ? ' back' : ''}, ${r}, past the library range for a ${where} (${fmt(Math.abs(dir === 'hi' ? e.q[4] : e.q[0]), spec.unit, leg)})`; }
        const bones = GROUP_BONES[spec.what] ? GROUP_BONES[spec.what](side ?? 'l').map(k => rig[k]).filter(i => i !== undefined) : null;
        out.push(mkFinding(rule, sev, a, b, { side, phase: ph, value: v, limit: dir === 'hi' ? wl[pk] : wl[pk], unit: spec.unit, group: spec.what, kind: spec.speed ? 'speed' : 'pose', bones, text }));
      }
    }
  }
  // Plants: drift beyond the library's planted-foot drift for that phase.
  for (const p of F.plants) {
    const e = envFor(model, 'plant_drift', p.phase);
    if (!e) continue;
    const lim = Math.max(e.q[4] * P.driftSlack, 3);
    if (p.drift_cm > lim) out.push(mkFinding('unnatural_plant_drift', p.drift_cm > 3 * lim && p.b - p.a + 1 >= P.blockRun ? 'blocker' : 'warning', p.a, p.b, { side: p.side, phase: p.phase, value: p.drift_cm, limit: lim, unit: 'cm',
      text: `${sideText(p.side)} ${p.point === 'ball' ? 'forefoot' : 'heel'} drifts ${p.drift_cm.toFixed(1)} cm through a plant, ${frameText(p.a, p.b)}; library feet drift at most ${e.q[4].toFixed(1)} cm in a ${PHASE_WORDS[p.phase] ?? p.phase}` }));
  }
  // Gait timing: stance duty and step period for the speed, against the library's range at that speed.
  for (const g of F.gait) {
    const b = model.gait?.find(x => g.speed >= x.lo && g.speed < x.hi);
    if (!b) continue;
    const sp = (g.speed * leg).toFixed(1);
    if (b.duty && b.duty.n >= P.gaitMinN && (g.duty > Math.min(1, b.duty.max * P.gaitSlack) || g.duty < b.duty.min / P.gaitSlack))
      out.push(mkFinding('unnatural_stance_duty', 'warning', g.a, g.b, { phase: 'locomotion', value: g.duty, limit: g.duty > b.duty.max ? b.duty.max : b.duty.min, unit: 'fraction', text: `Feet are down ${Math.round(g.duty * 100)}% of the time at ${sp} m/s, ${frameText(g.a, g.b)}; library gaits at this speed: ${Math.round(b.duty.min * 100)}-${Math.round(b.duty.max * 100)}%` }));
    if (g.period && b.period && b.period.n >= P.gaitMinN && (g.period > b.period.max * P.gaitSlack || g.period < b.period.min / P.gaitSlack))
      out.push(mkFinding('unnatural_step_period', 'warning', g.a, g.b, { phase: 'locomotion', value: g.period, limit: g.period > b.period.max ? b.period.max : b.period.min, unit: 's', text: `Stride takes ${g.period.toFixed(2)} s at ${sp} m/s, ${frameText(g.a, g.b)}; library strides at this speed take ${b.period.min.toFixed(2)}-${b.period.max.toFixed(2)} s` }));
  }
  return out;
}
