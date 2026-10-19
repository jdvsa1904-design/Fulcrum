// One local judge over facts + motion. It replaces the four separate judges (physics flags, fidelity
// gate, motionQuality, LocomotionTaste) with one list of findings and one scalar cost. Pure.
//   finding: { rule, severity: 'blocker'|'warning'|'info', a, b, side, phase, value, limit, unit, source, text }
// Rule names are a superset of wire/autofix.js fulcrumFindings (foot_slide, ground_float, pop, ground_sink,
// lock/pin residual, bone stretch, nonfinite) plus overload, seam_jump, knee_*, foot_skate, jitter,
// hand_slip, knee_slip, foot_tilt and balance.
import { inspectMotion } from '../motionquality.js';
import { seamPlan } from './seams.js';
import { spikeFrames, kneeWobbleRanges, kneeExtensionFlags } from '../contacts.js';
import { storyWindow } from './story.js';
import { computeFacts, comAccel, keyOf, runsOf, phaseOf, frameText, sideText, wpos } from './facts.js';

export const JUDGE_DEFAULTS = Object.freeze({
  mu: 0.8, snapLim: 900, gateLim: 8, edgeGuard: 3, slideSpeed: 0.3, sinkTol_cm: 2.5, driftLim_cm: 4,
  kneePopDeg: 12, kneePopBlockerDeg: 20, kneeWobbleRad: 0.06, kneeWobbleWin: 5, kneeHyperDeg: 5,
  jitterCm: 0.6, jitterSecs: 0.5, seamMin_cm: 1, slip_cm: 3, tiltDeg: 15, balanceCm: 10, balanceSecs: 0.33,
  lockTol_cm: 3, fidelity: { perDeg: 20, overGate: 100, pelvisPerCm: 0.5 },
});
export const SEVERITY_WEIGHT = Object.freeze({ blocker: 100, warning: 10, info: 1 });
export const RULE_ALIASES = Object.freeze({ snap: 'pop', slide: 'foot_slide', float: 'ground_float', 'ground-penetration': 'ground_sink', bone_length: 'bone_stretch', 'bone-length': 'bone_stretch', nan: 'nonfinite', lock_fail: 'lock_residual', pin_fail: 'pin_residual', constraint_residual: 'lock_residual', 'constraint-residual': 'lock_residual' });
export const MAJOR_BONES = ['pelvis', 'spine_01', 'spine_02', 'spine_03', 'spine_04', 'spine_05', 'neck_01', 'head', 'clavicle_l', 'clavicle_r', 'upperarm_l', 'upperarm_r', 'lowerarm_l', 'lowerarm_r', 'hand_l', 'hand_r', 'thigh_l', 'thigh_r', 'calf_l', 'calf_r', 'foot_l', 'foot_r'];
const GATE_SKIP = /(thumb|index|middle|ring|pinky|metacarpal|twist|corrective|ik_|weapon|attach)/;
const SIDES = ['l', 'r'];
const LEG = s => ['thigh_' + s, 'calf_' + s, 'foot_' + s];

// Rules each phase does not judge (drop) or only reports (soften to info), after tastebridge PHASE_RULES/SKIP_WHY.
export const PHASE_SCOPE = Object.freeze({
  flight: { drop: ['foot_slide', 'foot_skate', 'knee_hyperextension', 'foot_tilt', 'balance'] },
  vault: { drop: ['foot_slide', 'foot_skate', 'ground_float', 'foot_tilt', 'balance'] },
  prone: { drop: ['foot_slide', 'foot_skate', 'ground_float', 'balance'] },
  slide: { soften: ['foot_slide', 'foot_skate', 'knee_slip', 'hand_slip'], drop: ['balance'] },
  locomotion: { drop: ['balance'] }, in_place: { drop: ['balance'] }, rise: { drop: ['balance'] }, freeform: { drop: ['balance'] },
});

const qa = (q, o) => [q[o], q[o + 1], q[o + 2], q[o + 3]];
// Angle between two rotations, 2*atan2(|v|, |w|) of a^-1 b (acos of the dot loses ~0.05 deg in float32).
const qang = (a, b) => {
  const x = a[3] * b[0] - a[0] * b[3] - a[1] * b[2] + a[2] * b[1], y = a[3] * b[1] + a[0] * b[2] - a[1] * b[3] - a[2] * b[0];
  const z = a[3] * b[2] - a[0] * b[1] + a[1] * b[0] - a[2] * b[3], w = a[3] * b[3] + a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  return 2 * Math.atan2(Math.hypot(x, y, z), Math.abs(w));
};
const peakOf = (arr, a, b) => { let m = -Infinity; for (let f = a; f <= b; f++) if (arr[f] > m) m = arr[f]; return m; };
const fx = v => Number.isFinite(v) ? +v.toFixed(2) : v, cap = t => t.charAt(0).toUpperCase() + t.slice(1);

function finding(rule, severity, a, b, o = {}) {
  return { rule, severity, a, b, side: o.side ?? null, phase: o.phase ?? null, value: fx(o.value ?? null), limit: fx(o.limit ?? null), unit: o.unit ?? null, source: o.source ?? 'judge', text: cap(o.text ?? rule), ...(o.bone ? { bone: o.bone } : {}), ...(o.point ? { point: o.point } : {}) };
}

// Pure port of app.js analyze(). contact = { any, per: {l,r}, speed: {l,r} } in contactsFromWorld form.
// edgeGuard (frames) ignores "airborne but not falling" next to a contact frame, where the 5-tap COM
// smoothing bleeds the take-off push and landing impact into the air frames (0 = exactly analyze()).
export function physicsFlags({ com, contact, q, nb, n, fps, upm, major = [], mu = 0.8, snapLim = 900, edgeGuard = 0, comAcc = null, contactHeight = null }) {
  const g = 9.81 * upm, flags = [];
  const acc = comAcc ?? comAccel(com, n, fps);
  const per = { overload: new Uint8Array(n), slide: new Uint8Array(n), float: new Uint8Array(n), snap: new Uint8Array(n) };
  const mag = { overload: new Float64Array(n), slide: new Float64Array(n), float: new Float64Array(n), snap: new Float64Array(n) }, worst = new Int32Array(n).fill(-1), slideSide = { l: new Uint8Array(n), r: new Uint8Array(n) };
  let air = 0;
  for (let f = 0; f < n; f++) {
    const ah = Math.hypot(acc[f * 3], acc[f * 3 + 2]);
    mag.overload[f] = ah / upm;
    if (contact.any[f] && ah > mu * g) per.overload[f] = 1;
    for (const s of SIDES) if (contact.per[s][f] && contact.speed[s][f] > 0.3) { per.slide[f] = 1; slideSide[s][f] = 1; mag.slide[f] = Math.max(mag.slide[f], contact.speed[s][f]); }
    air = contact.any[f] ? 0 : air + 1;
    if (air >= 4 && acc[f * 3 + 1] > -0.35 * g) per.float[f] = 1;
    mag.float[f] = contactHeight ? Math.min(contactHeight.l[f], contactHeight.r[f]) : 0;
    if (f > 0 && q) {
      let mx = 0, wb = -1;
      for (const i of major) {
        const ang = qang(qa(q, (f * nb + i) * 4), qa(q, ((f - 1) * nb + i) * 4)) * 180 / Math.PI * fps;
        if (ang > mx) { mx = ang; wb = i; }
      }
      mag.snap[f] = mx; worst[f] = wb;
      if (mx > snapLim) per.snap[f] = 1;
    }
  }
  if (edgeGuard > 0) for (let f = 0; f < n; f++) if (per.float[f]) for (let d = -edgeGuard; d <= edgeGuard; d++) { const k = f + d; if (k >= 0 && k < n && contact.any[k]) { per.float[f] = 0; break; } }
  const label = { overload: 'Momentum change too fast for foot friction', slide: 'Planted foot sliding', float: 'Airborne but not falling', snap: 'Pose snaps between frames' };
  for (const k of Object.keys(per)) {
    let s = -1;
    for (let f = 0; f <= n; f++) {
      const on = f < n && per[k][f];
      if (on && s < 0) s = f;
      if (!on && s >= 0) { if (f - s >= (k === 'snap' ? 1 : 2)) flags.push({ kind: k, a: s, b: f - 1, text: label[k] }); s = -1; }
    }
  }
  const merged = [];
  for (const fl of flags.sort((x, y) => x.a - y.a)) {
    const prev = merged.findLast(m => m.kind === fl.kind);
    if (prev && fl.a - prev.b <= 3) prev.b = fl.b; else merged.push(fl);
  }
  for (const fl of merged) {
    fl.peak = peakOf(mag[fl.kind], fl.a, fl.b);
    if (fl.kind === 'snap') { let pf = fl.a; for (let f = fl.a; f <= fl.b; f++) if (mag.snap[f] >= mag.snap[pf]) pf = f; fl.bone = worst[pf]; }
    if (fl.kind === 'slide') { const l = slideSide.l.subarray(fl.a, fl.b + 1).some(Boolean), r = slideSide.r.subarray(fl.a, fl.b + 1).some(Boolean); fl.side = l && r ? null : l ? 'l' : 'r'; }
  }
  return { flags: merged, per, mag };
}

export function flagFindings(flags, { names = [], mu = 0.8, snapLim = 900 } = {}) {
  const out = [];
  for (const fl of flags) {
    const r = frameText(fl.a, fl.b);
    if (fl.kind === 'slide') out.push(finding('foot_slide', 'blocker', fl.a, fl.b, { side: fl.side, value: fl.peak, limit: 0.3, unit: 'm/s', source: 'flags', text: `${fl.side ? sideText(fl.side) + ' foot slides' : 'Feet slide'} ${fl.peak.toFixed(2)} m/s while planted, ${r}` }));
    if (fl.kind === 'float') out.push(finding('ground_float', 'blocker', fl.a, fl.b, { value: fl.peak, limit: 10, unit: 'cm', source: 'flags', text: `Both feet off the floor without falling, ${r} (lowest foot ${fl.peak.toFixed(1)} cm up)` }));
    if (fl.kind === 'snap') out.push(finding('pop', 'blocker', fl.a, fl.b, { value: fl.peak, limit: snapLim, unit: 'deg/s', bone: names[fl.bone], source: 'flags', text: `Pose snaps ${Math.round(fl.peak)} deg/s${names[fl.bone] ? ' at ' + names[fl.bone] : ''}, ${r}` }));
    if (fl.kind === 'overload') out.push(finding('overload', 'warning', fl.a, fl.b, { value: fl.peak, limit: mu * 9.81, unit: 'm/s2', source: 'flags', text: `Body changes speed faster than foot friction allows (${fl.peak.toFixed(1)} m/s2), ${r}` }));
  }
  return out;
}

function groupRuns(items, key, frame = x => x.frame) {
  const by = new Map();
  for (const x of items) { const k = key(x); if (!by.has(k)) by.set(k, []); by.get(k).push(x); }
  const out = [];
  for (const [k, xs] of by) {
    xs.sort((u, v) => frame(u) - frame(v));
    let cur = null;
    for (const x of xs) {
      const f = frame(x);
      if (cur && f <= cur.b + 1) { cur.b = f; cur.items.push(x); }
      else { cur = { key: k, a: f, b: f, items: [x] }; out.push(cur); }
    }
  }
  return out;
}

// Lock targets as app.js lockTargets() picks them (slowest foot frame), when the lock carries none.
function lockTargetsFor(locks, W, nb, rig) {
  return locks.map(lk => {
    if (lk.target) return lk;
    const fi = rig['foot_' + lk.side];
    if (fi === undefined) return lk;
    let best = lk.a, bs = Infinity;
    for (let f = lk.a; f <= lk.b; f++) {
      const p0 = wpos(W, nb, Math.max(lk.a, f - 1), fi), p1 = wpos(W, nb, Math.min(lk.b, f + 1), fi), s = Math.hypot(p1[0] - p0[0], p1[2] - p0[2]);
      if (s < bs) { bs = s; best = f; }
    }
    return { ...lk, target: wpos(W, nb, best, fi) };
  });
}

export function qualityFindings(mq, { names = [], facts, rig = {} }) {
  const feet = new Set(['foot_l', 'foot_r'].map(k => names[rig[k]]).filter(Boolean));
  const out = [];
  if (mq.nonfinite) out.push(finding('nonfinite', 'blocker', 0, facts.n - 1, { value: mq.nonfinite, limit: 0, unit: 'values', source: 'motionQuality', text: `${mq.nonfinite} non-finite values in the pose (NaN or infinity)` }));
  const bl = groupRuns(mq.findings.filter(x => x.kind === 'bone-length'), x => x.bone);
  for (const r of bl) out.push(finding('bone_stretch', 'blocker', r.a, r.b, { bone: r.key, value: Math.max(...r.items.map(x => x.cm)), limit: 0.5, unit: 'cm', source: 'motionQuality', text: `${r.key} stretches ${Math.max(...r.items.map(x => x.cm)).toFixed(1)} cm, ${frameText(r.a, r.b)}` }));
  const pen = groupRuns(mq.findings.filter(x => x.kind === 'ground-penetration'), x => x.bone);
  for (const r of pen) { const s = /_l$/i.test(keyOf(r.key)) ? 'l' : /_r$/i.test(keyOf(r.key)) ? 'r' : null; out.push(finding('ground_sink', 'warning', r.a, r.b, { side: s, bone: r.key, value: Math.max(...r.items.map(x => x.cm)), limit: 3, unit: 'cm', source: 'motionQuality', text: `${r.key} ${Math.max(...r.items.map(x => x.cm)).toFixed(1)} cm under the floor, ${frameText(r.a, r.b)}` })); }
  const res = groupRuns(mq.findings.filter(x => x.kind === 'constraint-residual'), x => (feet.has(x.bone) ? 'foot:' : 'hand:') + x.bone);
  for (const r of res) {
    const foot = r.key.startsWith('foot:'), bone = r.key.slice(r.key.indexOf(':') + 1), s = /_l$/i.test(keyOf(bone)) ? 'l' : /_r$/i.test(keyOf(bone)) ? 'r' : null, v = Math.max(...r.items.map(x => x.errorCm));
    out.push(finding(foot ? 'lock_residual' : 'pin_residual', 'blocker', r.a, r.b, { side: s, bone, value: v, limit: 3, unit: 'cm', source: 'motionQuality', text: `${foot ? 'Locked foot' : 'Pinned hand'} ${bone} misses its target by ${v.toFixed(1)} cm, ${frameText(r.a, r.b)}` }));
  }
  for (const r of groupRuns(mq.findings.filter(x => x.kind === 'prop-penetration'), x => x.bone + '|' + x.collider)) out.push(finding('prop_penetration', 'warning', r.a, r.b, { bone: r.items[0].bone, source: 'motionQuality', text: `${r.items[0].bone} inside ${r.items[0].collider}, ${frameText(r.a, r.b)}` }));
  return out;
}

export function soleFindings(facts, { sinkTol_cm = 2.5 } = {}) {
  const out = [];
  for (const s of SIDES) {
    const h = facts.contacts.height[s], m = Uint8Array.from(h, v => v < -sinkTol_cm ? 1 : 0);
    for (const [a, b] of runsOf(m, 2)) { let d = 0; for (let f = a; f <= b; f++) d = Math.max(d, -h[f]); out.push(finding('ground_sink', 'warning', a, b, { side: s, value: d, limit: sinkTol_cm, unit: 'cm', source: 'sole', text: `${sideText(s)} sole ${d.toFixed(1)} cm under the floor, ${frameText(a, b)}` })); }
  }
  return out;
}

export function plantFindings(facts, { driftLim_cm = 4 } = {}) {
  const out = [], slideLim = facts.footLength_cm / 2;
  for (const s of SIDES) for (const p of facts.contacts.feet[s].plants) {
    const r = frameText(p.a, p.b), pt = p.point === 'ball' ? 'forefoot' : 'heel';
    if (p.sliding && !p.touch) out.push(finding('foot_slide', 'warning', p.a, p.b, { side: s, point: p.point, value: p.drift_cm, limit: slideLim, unit: 'cm', source: 'plants', text: `${sideText(s)} ${pt} slides ${p.drift_cm.toFixed(1)} cm through a plant, ${r}` }));
    else if (p.touch && p.sliding) out.push(finding('foot_slide', 'info', p.a, p.b, { side: s, point: p.point, value: p.drift_cm, limit: slideLim, unit: 'cm', source: 'plants', text: `${sideText(s)} foot brushes the floor while moving, ${r}` }));
    else if (p.drift_cm > driftLim_cm) out.push(finding('foot_skate', 'warning', p.a, p.b, { side: s, point: p.point, value: p.drift_cm, limit: driftLim_cm, unit: 'cm', source: 'plants', text: `${sideText(s)} ${pt} skates ${p.drift_cm.toFixed(1)} cm while planted, ${r}` }));
  }
  return out;
}

export function seamFindings({ W, nb, facts, shots, rig, seamMin_cm = 1 }) {
  if (!shots || shots.length < 2 || [rig.pelvis, rig.thigh_l, rig.thigh_r].some(v => v === undefined)) return { plan: [], findings: [] };
  const plan = seamPlan({ W, nb, shots, pelvis: rig.pelvis, thighL: rig.thigh_l, thighR: rig.thigh_r, blend: 6 });
  const u = facts.upm / 100, out = [];
  for (const e of plan) {
    const lim = Math.max(e.allowedJump, seamMin_cm * u);
    if (!(e.jumpBefore > lim)) continue;
    const a = Math.max(0, e.start - 1), b = e.start;
    out.push(finding('seam_jump', 'blocker', a, b, { value: e.jumpBefore / u, limit: lim / u, unit: 'cm', source: 'seams', text: `Shot seam at f${e.start} jumps ${(e.jumpBefore / u).toFixed(1)} cm${Math.abs(e.yawJumpRad) > 0.05 ? ` and turns ${(e.yawJumpRad * 180 / Math.PI).toFixed(0)} deg` : ''}` }));
  }
  return { plan, findings: out };
}

export function kneeFindings(facts, o = {}) {
  const { kneePopDeg = 12, kneePopBlockerDeg = 20, kneeWobbleRad = 0.06, kneeWobbleWin = 5, kneeHyperDeg = 5 } = o;
  const out = [], n = facts.n, k30 = 30 / facts.fps;
  for (const s of SIDES) {
    const K = facts.knees[s];
    if (!K) continue;
    const spikes = spikeFrames(K.rel), popMask = new Uint8Array(n), popMag = new Float64Array(n);
    for (const i of spikes) {
      const a0 = qang(qa(K.rel, (i - 1) * 4), qa(K.rel, i * 4)) * 180 / Math.PI / k30, a1 = qang(qa(K.rel, i * 4), qa(K.rel, (i + 1) * 4)) * 180 / Math.PI / k30;
      if (Math.min(a0, a1) > kneePopDeg) { popMask[i] = 1; popMag[i] = Math.min(a0, a1); }
    }
    for (const [a, b] of runsOf(popMask)) { const v = peakOf(popMag, a, b); out.push(finding('knee_pop', v > kneePopBlockerDeg ? 'blocker' : 'warning', a, b, { side: s, value: v, limit: kneePopDeg, unit: 'deg/frame', source: 'contacts.spikeFrames', text: `${sideText(s)} knee pops ${v.toFixed(0)} deg in one frame, ${frameText(a, b)}` })); }
    const rad = Float64Array.from(K.flex, v => v * Math.PI / 180);
    const wob = kneeWobbleRanges(rad, { threshold: kneeWobbleRad * k30, window: Math.max(2, Math.round(kneeWobbleWin / k30)) });
    for (const r of wob.ranges) {
      if (popMask.subarray(Math.max(0, r.s - 1), Math.min(n, r.e + 2)).some(Boolean)) continue;
      let v = 0; for (let f = Math.max(1, r.s); f <= Math.min(n - 1, r.e + 1); f++) v = Math.max(v, Math.abs(K.flex[f] - K.flex[f - 1]));
      out.push(finding('knee_wobble', 'warning', r.s, r.e, { side: s, value: v, limit: kneeWobbleRad * 180 / Math.PI, unit: 'deg/frame', source: 'contacts.kneeWobbleRanges', text: `${sideText(s)} knee wobbles, ${frameText(r.s, r.e)}` }));
    }
    const mask = facts.contacts.feet[s].mask, blend = Float64Array.from(mask, m => m === 1 ? 1 : 0), ext = Float64Array.from(K.flex, v => -v);
    const hm = new Uint8Array(n);
    for (const i of kneeExtensionFlags(ext, blend, kneeHyperDeg)) hm[i] = 1;
    for (const [a, b] of runsOf(hm, 2)) { const v = peakOf(ext, a, b); out.push(finding('knee_hyperextension', 'warning', a, b, { side: s, value: v, limit: kneeHyperDeg, unit: 'deg', source: 'contacts.kneeExtensionFlags', text: `${sideText(s)} knee bends backwards ${v.toFixed(0)} deg on a planted leg, ${frameText(a, b)}` })); }
  }
  return out;
}

// Sustained high-frequency energy: window median of the per-frame median |third difference| of major joints.
export function jitterFindings({ W, nb, facts, rig, jitterCm = 0.6, jitterSecs = 0.5 }) {
  const n = facts.n, ids = MAJOR_BONES.map(k => rig[k]).filter(v => v !== undefined), u = facts.upm / 100, k = Math.pow(facts.fps / 30, 3), j = new Float64Array(n);
  if (n < 8 || !ids.length) return [];
  for (let f = 2; f < n - 1; f++) {
    const v = ids.map(i => { const a = wpos(W, nb, f + 1, i), b = wpos(W, nb, f, i), c = wpos(W, nb, f - 1, i), d = wpos(W, nb, f - 2, i); return Math.hypot(...[0, 1, 2].map(c2 => a[c2] - 3 * b[c2] + 3 * c[c2] - d[c2])) / u * k; }).sort((x, y) => x - y);
    j[f] = v[v.length >> 1];
  }
  const R = 7, w = new Float64Array(n), m = new Uint8Array(n);
  for (let f = 0; f < n; f++) { const s = []; for (let d = -R; d <= R; d++) { const g = f + d; if (g >= 2 && g < n - 1) s.push(j[g]); } s.sort((x, y) => x - y); w[f] = s.length ? s[s.length >> 1] : 0; m[f] = w[f] > jitterCm ? 1 : 0; }
  return runsOf(m, Math.max(4, Math.round(jitterSecs * facts.fps))).map(([a, b]) => { const v = peakOf(w, a, b); return finding('jitter', 'warning', a, b, { value: v, limit: jitterCm, unit: 'cm/frame3', source: 'judge', text: `Jitter (high-frequency noise) ${frameText(a, b)}` }); });
}

export function slipFindings(facts, { slip_cm = 3 } = {}) {
  const out = [];
  for (const [kind, C] of [['hand', facts.contacts.hands], ['knee', facts.contacts.knees]]) for (const s of SIDES) for (const r of C[s].runs)
    if (r.slip_cm > slip_cm) out.push(finding(kind + '_slip', 'warning', r.a, r.b, { side: s, value: r.slip_cm, limit: slip_cm, unit: 'cm', source: 'facts', bone: r.pivot, text: `${sideText(s)} ${kind} slides ${r.slip_cm.toFixed(1)} cm while on the floor, ${frameText(r.a, r.b)}` }));
  return out;
}

// Sideways roll of a planted foot (ankle inversion/eversion), measured against the foot's rest "up".
export function tiltFindings({ W, nb, facts, tiltDeg = 15 }) {
  const out = [];
  for (const s of SIDES) {
    const R = facts.footRest[s], L = facts.legs[s];
    if (!R || L.ball === undefined || L.ball === L.foot) continue;
    const upL = [R[0][1], R[1][1], R[2][1]];
    for (const p of facts.contacts.feet[s].plants) {
      if (!p.lockable || p.b - p.a + 1 < 6) continue;
      let sum = 0, c = 0;
      for (let f = p.a; f <= p.b; f++) {
        const o = (f * nb + L.foot) * 16, col = k => { const l = Math.hypot(W[o + k * 4], W[o + k * 4 + 1], W[o + k * 4 + 2]) || 1; return [W[o + k * 4] / l, W[o + k * 4 + 1] / l, W[o + k * 4 + 2] / l]; };
        const C = [col(0), col(1), col(2)], up = [0, 1, 2].map(i => C[0][i] * upL[0] + C[1][i] * upL[1] + C[2][i] * upL[2]);
        const A = wpos(W, nb, f, L.foot), B = wpos(W, nb, f, L.ball), fw = [B[0] - A[0], B[1] - A[1], B[2] - A[2]], lf = Math.hypot(...fw) || 1;
        for (let i = 0; i < 3; i++) fw[i] /= lf;
        if (Math.abs(fw[1]) > 0.7) continue;
        const pu = up.map((v, i) => v - fw[i] * (up[0] * fw[0] + up[1] * fw[1] + up[2] * fw[2])), pt = [0, 1, 0].map((v, i) => v - fw[i] * fw[1]);
        const lu = Math.hypot(...pu), lt = Math.hypot(...pt);
        if (lu < 1e-6 || lt < 1e-6) continue;
        sum += Math.acos(Math.max(-1, Math.min(1, (pu[0] * pt[0] + pu[1] * pt[1] + pu[2] * pt[2]) / (lu * lt)))) * 180 / Math.PI; c++;
      }
      const v = c ? sum / c : 0;
      if (v > tiltDeg) out.push(finding('foot_tilt', 'warning', p.a, p.b, { side: s, value: v, limit: tiltDeg, unit: 'deg', source: 'judge', text: `${sideText(s)} foot rolls ${v.toFixed(0)} deg sideways while planted, ${frameText(p.a, p.b)}` }));
    }
  }
  return out;
}

export function balanceFindings(facts, { balanceCm = 10, balanceSecs = 0.33 } = {}) {
  const m = Uint8Array.from(facts.balance.marginCm, v => Number.isFinite(v) && v < -balanceCm ? 1 : 0);
  return runsOf(m, Math.max(2, Math.round(balanceSecs * facts.fps))).map(([a, b]) => { let v = 0; for (let f = a; f <= b; f++) v = Math.max(v, -facts.balance.marginCm[f]); return finding('balance', 'info', a, b, { value: v, limit: balanceCm, unit: 'cm', source: 'facts', text: `Centre of mass ${v.toFixed(0)} cm outside the support while holding still, ${frameText(a, b)}` }); });
}

// Bones a fix is meant to move: gate() leaves them out of the fidelity check (port of its driven set, from fix objects).
export function drivenBones(state = {}, rig = {}) {
  const set = new Set(), add = (...ks) => { for (const k of ks) if (rig[k] !== undefined) set.add(rig[k]); };
  const legs = (side, ball) => { for (const s of side ? [side] : SIDES) { add(...LEG(s)); if (ball) add('ball_' + s); } };
  for (const l of state.locks || []) legs(l.side);
  for (const p of state.pins || []) add(...(p.hand === 'r' ? ['upperarm_r', 'lowerarm_r', 'hand_r'] : ['upperarm_l', 'lowerarm_l', 'hand_l']));
  for (const x of state.fixes || []) {
    if (x.mute) continue;
    if (x.type === 'ground' || x.type === 'roll' || x.type === 'ankleLock' || x.type === 'knee') legs(x.side, x.type === 'roll');
    else if (x.type === 'floor') legs(x.side, true);
    else if (x.type === 'level') for (const s of x.side ? [x.side] : SIDES) add('foot_' + s, 'ball_' + s);
    else if (x.type === 'lean') { const i = rig.spine_01 ?? rig.spine_02; if (i !== undefined) set.add(i); }
    else if (x.type === 'carry') add('upperarm_l', 'lowerarm_l', 'hand_l', 'upperarm_r', 'lowerarm_r', 'hand_r');
    else if (x.type === 'travel' && (x.footsteps || x.reach || x.plant !== false)) legs(null);
    for (const t of x.contactTargets || []) legs(t.side);
  }
  return set;
}

// Pure port of app.js gate(): p95 rotation change of joints no fix drives, against the source take,
// plus how far the pelvis moved. trust (per frame, 0..1) scales each frame's change: a low-trust source
// frame (video tracking dropout) is not held as truth. penalty is in cost units.
export function fidelityGate({ q, p, srcQ, srcP, nb, n, names, rig, state = {}, edited = null, driven = null, lim = 8, upm = 100, trust = null, weights = JUDGE_DEFAULTS.fidelity }) {
  if (!q || !srcQ || srcQ.length !== q.length) return { rows: [], free: [], lim, pass: true, worst: 0, worstBone: null, pelvisCm: 0, rotPenalty: 0, pelvisPenalty: 0, penalty: 0, skipped: true };
  const drv = driven ?? drivenBones(state, rig), joinMask = new Uint8Array(n), joins = (state.fixes || []).filter(x => x.type === 'join');
  for (const fx of state.fixes || []) if (fx.type === 'smooth' && joins.some(j => j.at >= fx.a && j.at <= fx.b)) for (let f = Math.max(0, fx.a); f <= Math.min(n - 1, fx.b); f++) joinMask[f] = 1;
  for (const fx of state.fixes || []) if (fx.type === 'story') { const w = storyWindow(fx, n); for (let f = w.s; f <= w.e; f++) joinMask[f] = 1; }
  const rows = [];
  for (let i = 0; i < nb; i++) {
    if (GATE_SKIP.test(keyOf(names?.[i] ?? ''))) continue;
    const a = [];
    for (let f = 0; f < n; f++) if (!joinMask[f]) a.push(qang(qa(q, (f * nb + i) * 4), qa(srcQ, (f * nb + i) * 4)) * 180 / Math.PI * (trust ? trust[f] : 1));
    const s = a.sort((x, y) => x - y), m = s.length;
    rows.push({ bone: names?.[i] ?? String(i), index: i, p95: s[Math.floor(m * 0.95)] || 0, max: s[m - 1] || 0, edited: !!edited?.has(i), driven: drv.has(i) });
  }
  rows.sort((x, y) => y.p95 - x.p95);
  let pmax = 0;
  const pel = rig.pelvis;
  if (pel !== undefined && p && srcP) for (let f = 0; f < n; f++) { const o = (f * nb + pel) * 3; pmax = Math.max(pmax, Math.hypot(p[o] - srcP[o], p[o + 1] - srcP[o + 1], p[o + 2] - srcP[o + 2])); }
  const free = rows.filter(r => !r.edited && !r.driven), worst = free[0]?.p95 || 0, pelvisCm = pmax / upm * 100;
  const rotPenalty = weights.perDeg * worst / lim + (worst > lim ? weights.overGate * (worst - lim) / lim : 0), pelvisPenalty = weights.pelvisPerCm * pelvisCm;
  return { rows, free, lim, pass: worst <= lim, worst, worstBone: free[0]?.bone ?? null, pelvisCm, rotPenalty, pelvisPenalty, penalty: rotPenalty + pelvisPenalty };
}

export function scopeByPhase(findings, facts) {
  const out = [];
  for (const f of findings) {
    const phase = f.phase ?? phaseOf(facts, f.a, f.b), sc = PHASE_SCOPE[phase] || {};
    if (sc.drop?.includes(f.rule)) continue;
    out.push(sc.soften?.includes(f.rule) && f.severity !== 'info' ? { ...f, phase, severity: 'info', text: f.text + ` (expected in a ${phase} phase)` } : { ...f, phase });
  }
  return out;
}

// Merge same-rule, same-side findings that overlap (e.g. sole and ankle penetration checks).
function mergeOverlaps(findings, rules = ['ground_sink', 'foot_slide']) {
  const out = [], sev = { blocker: 0, warning: 1, info: 2 };
  for (const f of [...findings].sort((x, y) => x.a - y.a)) {
    const prev = rules.includes(f.rule) && out.findLast(g => g.rule === f.rule && g.side === f.side && g.source !== 'plants' && f.source !== 'plants' && f.a <= g.b + 1);
    if (prev) { prev.b = Math.max(prev.b, f.b); if (f.value > prev.value) { prev.value = f.value; prev.text = f.text; } if (sev[f.severity] < sev[prev.severity]) prev.severity = f.severity; }
    else out.push({ ...f });
  }
  return out;
}

export function findingMagnitude(f) {
  const r = Number.isFinite(f.value) && Number.isFinite(f.limit) && f.limit > 0 ? Math.abs(f.value) / f.limit : 1;
  return Math.min(10, Math.max(1, r));
}

export function costOf(findings, weights = SEVERITY_WEIGHT) {
  let c = 0;
  for (const f of findings) c += (weights[f.severity] ?? 0) * findingMagnitude(f) * Math.sqrt(Math.max(1, (f.b ?? f.a ?? 0) - (f.a ?? 0) + 1));
  return c;
}

export const countOf = findings => ({ blocker: findings.filter(f => f.severity === 'blocker').length, warning: findings.filter(f => f.severity === 'warning').length, info: findings.filter(f => f.severity === 'info').length });

// motion: { W, q, p } (q/p local, optional); source: { q, p } of the untouched take for fidelity.
export function judgeMotion({ motion, source = null, nb, n, fps = 30, upm = 100, rig, names = [], parents = null, restW = null, shots = [], trust = null, state = {}, edited = null, ground = null, facts = null, opts = {} }) {
  const o = { ...JUDGE_DEFAULTS, ...opts }, W = motion.W;
  n = n ?? W.length / (nb * 16);
  facts = facts ?? computeFacts({ W, nb, n, fps, upm, rig, restW, shots, trust, ground, opts: o.facts || {} });
  const major = MAJOR_BONES.map(k => rig[k]).filter(v => v !== undefined);
  // analyze() counts a foot as planted when it is under 10 cm, which makes every low swing foot "slide".
  // By default a foot slides only inside a Rule A plant (facts mask 1 or 2); slideContact:'height' is analyze() exactly.
  const per = o.slideContact === 'height' ? facts.contacts.per : { l: Uint8Array.from(facts.contacts.feet.l.mask, m => m === 1 || m === 2 ? 1 : 0), r: Uint8Array.from(facts.contacts.feet.r.mask, m => m === 1 || m === 2 ? 1 : 0) };
  const contact = { any: facts.contacts.feetDown, per, speed: facts.contacts.speed };
  const phys = physicsFlags({ com: facts.com, comAcc: facts.comAcc, contact, contactHeight: facts.contacts.height, q: motion.q, nb, n, fps, upm, major, mu: o.mu, snapLim: o.snapLim, edgeGuard: o.edgeGuard });
  const locks = lockTargetsFor([...(state.locks || []), ...(state.fixes || []).flatMap(x => x.contactTargets || [])], W, nb, rig);
  const mq = inspectMotion({ W, nb, n, fps, upm, ground: facts.ground, names, parents, com: Float32Array.from(facts.com), feet: { l: rig.foot_l, r: rig.foot_r }, locks, pins: state.pins || [] });
  const seam = seamFindings({ W, nb, facts, shots, rig, seamMin_cm: o.seamMin_cm });
  let findings = [
    ...flagFindings(phys.flags, { names, mu: o.mu, snapLim: o.snapLim }),
    ...qualityFindings(mq, { names, facts, rig }),
    ...soleFindings(facts, o),
    ...plantFindings(facts, o),
    ...seam.findings,
    ...kneeFindings(facts, o),
    ...jitterFindings({ W, nb, facts, rig, jitterCm: o.jitterCm, jitterSecs: o.jitterSecs }),
    ...slipFindings(facts, o),
    ...tiltFindings({ W, nb, facts, tiltDeg: o.tiltDeg }),
    ...balanceFindings(facts, o),
  ];
  findings = scopeByPhase(mergeOverlaps(findings), facts);
  const sevRank = { blocker: 0, warning: 1, info: 2 };
  findings.sort((x, y) => sevRank[x.severity] - sevRank[y.severity] || x.a - y.a);
  const fidelity = fidelityGate({ q: motion.q, p: motion.p, srcQ: source?.q, srcP: source?.p, nb, n, names, rig, state, edited, lim: o.gateLim, upm, trust: trust ? facts.trust : null, weights: o.fidelity });
  const cost = costOf(findings);
  return { findings, cost, score: cost + fidelity.penalty, counts: countOf(findings), fidelity, flags: phys.flags, quality: mq, seamPlan: seam.plan, facts };
}
