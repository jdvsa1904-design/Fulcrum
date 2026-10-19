export const BONES = ['pelvis', 'spine_01', 'spine_03', 'spine_05', 'neck_01', 'head', 'clavicle_l', 'upperarm_l', 'lowerarm_l', 'hand_l',
  'clavicle_r', 'upperarm_r', 'lowerarm_r', 'hand_r', 'thigh_l', 'calf_l', 'foot_l', 'ball_l', 'thigh_r', 'calf_r', 'foot_r', 'ball_r'];

const toTaste = (x, y, z, k) => [x * k, -z * k, y * k];

export function boneIndices(find) {
  const map = [], missing = [];
  for (const name of BONES) {
    const i = find(name);
    if (i === undefined) missing.push(name); else map.push(i);
  }
  return { map, missing };
}

export function tasteInput({ W, nb, n, fps, upm, map, restW, from = 0, to = n - 1 }) {
  const k = 100 / upm, B = [];
  for (let f = from; f <= to; f++) {
    const row = [];
    for (const i of map) { const o = (f * nb + i) * 16 + 12; row.push(toTaste(W[o], W[o + 1], W[o + 2], k).map(v => +v.toFixed(3) + 0)); }
    B.push(row);
  }
  const rest = map.map(i => { const m = restW[i]; return toTaste(m[12], m[13], m[14], k).map(v => +v.toFixed(3) + 0); });
  return { fps, B, rest, frame_offset: from };
}

export function summarize(result) {
  const sev = { blocker: 0, warn: 1, info: 2 };
  const f = [...(result.findings || [])].sort((a, b) => (sev[a.severity] ?? 3) - (sev[b.severity] ?? 3));
  return { verdict: result.verdict, blockers: f.filter(x => x.severity === 'blocker'), findings: f, confidence: result.confidence, unmeasurable: result.unmeasurable || [] };
}

const PELVIS = 0, CHEST = 3, HAND = [9, 13], THIGH = [14, 18], CALF = [15, 19], FOOT = [16, 20], BALL = [17, 21];

export const PHASES = ['locomotion', 'in_place', 'flight', 'vault', 'plant', 'slide', 'kneel', 'crouch', 'prone', 'rise', 'stand', 'upper_body', 'freeform'];

export const GAIT_PHASES = ['locomotion', 'in_place'];

export const SEG = { contact: .1, air: .1, flightRise: 1.06, low: .8, plantDip: .9, riseBelow: .95, kneeLow: .25, rise: .35, drop: .35, slide: .6, footStill: .3, minLen: 6, minFlight: 4, pad: 6, gravity: 981, airSlack: 1.5, airMargin: 0.1, inPlaceRatio: 0.35, inPlaceFoot: 0.5, proneLow: 0.55, proneTilt: 55, handLow: 0.6, armActive: 0.5, straight: 0.5, gaitPeriodic: 0.3, minTravel: 0.3, crouchKnee: 45, standFloor: 0.9, freeformSecs: 1, crouchMaxSpeed: 1.6 };

const pct = (a, p) => { const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.max(0, Math.round(p / 100 * (s.length - 1))))]; };
const median = a => pct(a, 50);

function smooth(a, k) {
  const n = a.length, out = new Array(n);
  for (let i = 0; i < n; i++) {
    let s = 0, c = 0;
    for (let j = Math.max(0, i - k); j <= Math.min(n - 1, i + k); j++) { s += a[j]; c++; }
    out[i] = s / c;
  }
  return out;
}

function deriv(a, fps) {
  const n = a.length;
  return a.map((_, i) => n < 2 ? 0 : (a[Math.min(n - 1, i + 1)] - a[Math.max(0, i - 1)]) * fps / (Math.min(n - 1, i + 1) - Math.max(0, i - 1)));
}

export function runs(labels) {
  const out = [];
  for (let i = 0; i < labels.length; i++) {
    const last = out[out.length - 1];
    if (last && last.phase === labels[i]) last.end = i; else out.push({ phase: labels[i], start: i, end: i });
  }
  return out;
}

export function mergeShort(segs, minLen = SEG.minLen) {
  let s = segs.map(x => ({ ...x }));
  const len = x => x.end - x.start + 1;
  for (;;) {
    let k = -1;
    for (let i = 0; i < s.length; i++) if (len(s[i]) < minLen && (k < 0 || len(s[i]) < len(s[k]))) k = i;
    if (k < 0 || s.length < 2) return s;
    const a = s[k - 1], b = s[k + 1];
    const into = !a ? b : !b ? a : a.phase === b.phase ? a : len(a) >= len(b) ? a : b;
    into.start = Math.min(into.start, s[k].start);
    into.end = Math.max(into.end, s[k].end);
    s.splice(k, 1);
    const out = [];
    for (const x of s) { const last = out[out.length - 1]; if (last && last.phase === x.phase) last.end = x.end; else out.push(x); }
    s = out;
  }
}

export function phaseSignals({ B, rest, fps }) {
  const n = B.length;
  const restFoot = Math.min(...FOOT.map(i => rest[i][2]), ...BALL.map(i => rest[i][2]));
  const leg = rest[PELVIS][2] - restFoot;
  const g = pct(B.map(r => Math.min(r[BALL[0]][2], r[BALL[1]][2])), 2);
  const foot = [0, 1].map(s => B.map(r => Math.min(r[FOOT[s]][2], r[BALL[s]][2]) - g));
  const knee = B.map(r => Math.min(r[CALF[0]][2], r[CALF[1]][2]) - g);
  const ph = B.map(r => r[PELVIS][2] - g);
  const both = [];
  for (let f = 0; f < n; f++) if (foot[0][f] < SEG.contact * leg && foot[1][f] < SEG.contact * leg) both.push(ph[f]);
  const stand = Math.max(both.length >= 5 ? median(both) : .97 * leg, SEG.standFloor * leg);
  const px = smooth(B.map(r => r[PELVIS][0]), 2), py = smooth(B.map(r => r[PELVIS][1]), 2), pz = smooth(ph, 2);
  const vx = deriv(px, fps), vy = deriv(py, fps), vz = deriv(pz, fps);
  const speed = vx.map((v, i) => Math.hypot(v, vy[i]));
  const footSpeed = [0, 1].map(s => {
    const x = smooth(B.map(r => r[BALL[s]][0]), 2), y = smooth(B.map(r => r[BALL[s]][1]), 2), dx = deriv(x, fps), dy = deriv(y, fps);
    return dx.map((v, i) => Math.hypot(v, dy[i]));
  });
  const tilt = B.map(r => { const d = [0, 1, 2].map(k => r[CHEST][k] - r[PELVIS][k]); return Math.acos(Math.max(-1, Math.min(1, d[2] / (Math.hypot(...d) || 1)))) * 180 / Math.PI; });
  const hand = [0, 1].map(s => B.map(r => r[HAND[s]][2] - g));
  const armSpeed = [0, 1].map(s => {
    const d = [0, 1, 2].map(k => deriv(smooth(B.map(r => r[HAND[s]][k] - r[PELVIS][k]), 2), fps));
    return d[0].map((v, i) => Math.hypot(v, d[1][i], d[2][i]));
  });
  const handSpeed = [0, 1].map(s => {
    const d = [0, 1, 2].map(k => deriv(smooth(B.map(r => r[HAND[s]][k]), 2), fps));
    return d[0].map((v, i) => Math.hypot(v, d[1][i], d[2][i]));
  });
  const ang = (a, b, c) => { const u = [0, 1, 2].map(k => a[k] - b[k]), v = [0, 1, 2].map(k => c[k] - b[k]); return 180 - Math.acos(Math.max(-1, Math.min(1, (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]) / ((Math.hypot(...u) * Math.hypot(...v)) || 1)))) * 180 / Math.PI; };
  const kneeFlex = B.map(r => Math.min(...[0, 1].map(s => ang(r[THIGH[s]], r[CALF[s]], r[FOOT[s]]))));
  return { n, fps, leg, ground: g, stand, foot, knee, pelvis: ph, pelvisSmooth: pz, speed, vz, footSpeed, kneeFlex, tilt, hand, armSpeed, handSpeed, px, py };
}

export function labelFrames(sig) {
  const { n, leg, stand, foot, knee, pelvis, pelvisSmooth, speed, vz, footSpeed } = sig;
  const air = Array.from({ length: n }, (_, f) => Math.min(foot[0][f], foot[1][f]) > SEG.air * leg);
  const flight = new Array(n).fill(false);
  for (const r of runs(air)) {
    const len = r.end - r.start + 1;
    if (!r.phase || len < SEG.minFlight) continue;
    let top = -Infinity;
    for (let f = r.start; f <= r.end; f++) top = Math.max(top, pelvis[f]);
    const rise = Math.max(0, top - Math.min(pelvis[r.start], pelvis[r.end])), airtime = len / sig.fps;
    const ballistic = 2 * Math.sqrt(2 * rise / SEG.gravity) * SEG.airSlack + SEG.airMargin;
    if (top > SEG.flightRise * stand && airtime <= ballistic) for (let f = r.start; f <= r.end; f++) flight[f] = true;
  }
  const vault = new Array(n).fill(false);
  if (sig.hand) for (const r of runs(air)) {
    if (!r.phase || r.end - r.start + 1 < SEG.minFlight || flight[r.start]) continue;
    let held = 0;
    for (let f = r.start; f <= r.end; f++) if ([0, 1].some(s => sig.hand[s][f] < pelvis[f] && sig.hand[s][f] < SEG.handLow * stand && sig.handSpeed[s][f] < SEG.footStill * stand)) held++;
    if (held >= SEG.minFlight / 2) for (let f = r.start; f <= r.end; f++) vault[f] = true;
  }
  const out = new Array(n);
  for (let f = 0; f < n; f++) {
    const h = pelvisSmooth[f], rising = vz[f] > SEG.rise * stand, dropping = vz[f] < -SEG.drop * stand;
    const contact = foot[0][f] < SEG.contact * leg && foot[1][f] < SEG.contact * leg;
    const still = footSpeed[0][f] < SEG.footStill * stand && footSpeed[1][f] < SEG.footStill * stand;
    const oneStill = Math.min(footSpeed[0][f], footSpeed[1][f]) < SEG.footStill * stand;
    if (sig.tilt && h < SEG.proneLow * stand && sig.tilt[f] > SEG.proneTilt) out[f] = 'prone';
    else if (h < SEG.low * stand) out[f] = rising ? 'rise' : dropping || (speed[f] > SEG.slide * stand && !oneStill) ? 'slide' : knee[f] < SEG.kneeLow * stand ? 'kneel' : 'crouch';
    else if (flight[f]) out[f] = 'flight';
    else if (vault[f]) out[f] = 'vault';
    else if (rising && h < SEG.riseBelow * stand) out[f] = 'rise';
    else if (h < SEG.plantDip * stand || (dropping && h < SEG.riseBelow * stand)) out[f] = 'plant';
    else if (contact && still) out[f] = 'stand';
    else out[f] = 'locomotion';
  }
  return out;
}

export function segmentPhases(input, opts = {}) {
  if (!input?.B?.length) return [];
  const sig = phaseSignals(input);
  return classifyStill(markInPlace(mergeShort(runs(labelFrames(sig)), opts.minLen ?? SEG.minLen), sig), sig);
}

export function periodicity(a) {
  const n = a.length, m = a.reduce((x, y) => x + y, 0) / (n || 1), d = a.map(v => v - m), v0 = d.reduce((x, y) => x + y * y, 0);
  if (n < 8 || v0 <= 0) return 0;
  let best = -1;
  for (let lag = 4; lag <= n >> 1; lag++) {
    let s = 0;
    for (let i = 0; i + lag < n; i++) s += d[i] * d[i + lag];
    best = Math.max(best, s / v0 * n / (n - lag));
  }
  return best;
}

export function classifyStill(segs, sig) {
  if (!sig.armSpeed) return segs;
  const med = a => { const t = [...a].sort((x, y) => x - y); return t.length ? t[t.length >> 1] : 0; };
  const out = segs.map(seg => {
    const idx = Array.from({ length: seg.end - seg.start + 1 }, (_, k) => seg.start + k);
    const arm = med(idx.map(f => Math.max(sig.armSpeed[0][f], sig.armSpeed[1][f]))), feet = med(idx.map(f => Math.max(sig.footSpeed[0][f], sig.footSpeed[1][f])));
    if (seg.phase === 'stand') return arm > SEG.armActive * sig.stand ? { ...seg, phase: 'upper_body' } : seg;
    if (seg.phase !== 'locomotion' && seg.phase !== 'in_place') return seg;
    const secs = idx.length / sig.fps, a = seg.start, b = seg.end;
    let path = 0;
    for (let f = a + 1; f <= b; f++) path += Math.hypot(sig.px[f] - sig.px[f - 1], sig.py[f] - sig.py[f - 1]);
    const net = Math.hypot(sig.px[b] - sig.px[a], sig.py[b] - sig.py[a]), travel = net / secs / sig.leg;
    if (seg.phase === 'locomotion' && med(idx.map(f => sig.speed[f])) < SEG.crouchMaxSpeed * sig.leg && med(idx.map(f => sig.kneeFlex[f])) > SEG.crouchKnee) return { ...seg, phase: 'crouch' };
    if (seg.phase === 'locomotion' && travel < SEG.minTravel && feet < SEG.inPlaceFoot * sig.leg) return { ...seg, phase: arm > SEG.armActive * sig.stand ? 'upper_body' : 'stand' };
    const gait = periodicity(idx.map(f => sig.foot[0][f] - sig.foot[1][f]));
    if (secs >= SEG.freeformSecs && gait < SEG.gaitPeriodic && (seg.phase === 'in_place' || (path > 0 && net / path < SEG.straight))) return { ...seg, phase: 'freeform' };
    return seg;
  });
  const merged = [];
  for (const x of out) { const last = merged[merged.length - 1]; if (last && last.phase === x.phase) last.end = x.end; else merged.push(x); }
  return merged;
}

export function markInPlace(segs, sig) {
  const med = a => { const t = [...a].sort((x, y) => x - y); return t.length ? t[t.length >> 1] : 0; };
  return segs.map(seg => {
    if (seg.phase !== 'locomotion') return seg;
    const pel = [], foot = [];
    for (let f = seg.start; f <= seg.end; f++) { pel.push(sig.speed[f]); foot.push(Math.max(sig.footSpeed[0][f], sig.footSpeed[1][f])); }
    const fs = med(foot);
    return fs > SEG.inPlaceFoot * sig.leg && med(pel) < SEG.inPlaceRatio * fs ? { ...seg, phase: 'in_place' } : seg;
  });
}

export const INTEGRITY_RULES = ['pop', 'crossing', 'freeze', 'ground_sink'];

export const ENVELOPE_RULES = ['planted_foot_pitch_deg', 'planted_ankle_deg', 'heel_raise_pct', 'planted_toe_out_deg', 'knee_behind_line', 'knee_flex_range_deg', 'knee_flex_max_deg', 'min_stance_width', 'max_stance_width'];

export const GAIT_RULES = ['envelope', 'bounce', 'stride_ratio'];

export const SKIP_WHY = {
  prone: { foot_slide: 'feet drag along the floor while prone or crawling', ground_float: 'feet lift off the floor while prone; the torso, knees and hands carry the contact' },
  flight: { foot_slide: 'both feet airborne', plant_pop: 'both feet airborne', ground_float: 'airborne by design; judged by the ballistic airtime check instead' },
  vault: { foot_slide: 'feet off the floor while the hands carry the body', plant_pop: 'feet off the floor while the hands carry the body', ground_float: 'feet off the floor while the hands carry the body' },
  slide: { foot_slide: 'the whole body slides, so the low feet are not planted', plant_pop: 'the whole body slides, so the low feet are not planted' },
};

export const PHASE_RULES = {
  locomotion: ['foot_slide', 'plant_pop', 'ground_float', 'bounce', 'stride_ratio'],
  in_place: ['foot_slide', 'plant_pop', 'ground_float', 'bounce'],
  flight: [],
  plant: ['foot_slide', 'plant_pop', 'ground_float'],
  vault: [],
  slide: ['ground_float'],
  kneel: ['foot_slide', 'plant_pop', 'ground_float'],
  crouch: ['foot_slide', 'plant_pop', 'ground_float'],
  prone: ['plant_pop'],
  rise: ['foot_slide', 'plant_pop', 'ground_float'],
  stand: ['foot_slide', 'plant_pop', 'ground_float'],
  upper_body: ['foot_slide', 'plant_pop', 'ground_float'],
  freeform: ['foot_slide', 'plant_pop', 'ground_float'],
};

export function phaseScope(phase, frames) {
  const rules = phaseRules(phase, frames);
  const judged = [...rules.filter(r => r !== 'envelope'), ...(rules.includes('envelope') ? ENVELOPE_RULES : []), ...INTEGRITY_RULES];
  if (phase === 'flight') judged.push('ballistic_flight');
  const skipped = [];
  for (const r of ['foot_slide', 'plant_pop', 'ground_float']) if (!rules.includes(r)) skipped.push({ rule: r, why: SKIP_WHY[phase]?.[r] || `not applied in a ${phase} phase` });
  for (const r of GAIT_RULES) if (!rules.includes(r)) {
    const why = !GAIT_PHASES.includes(phase) ? `gait-envelope rule built from CMU walk and run; not meaningful in a ${phase} phase`
      : r === 'envelope' ? `${phase} phase shorter than ${ENVELOPE_MIN_FRAMES} frames or without travel` : `${phase} has no travel to compare against`;
    for (const x of r === 'envelope' ? ENVELOPE_RULES : [r]) skipped.push({ rule: x, why });
  }
  return { judged_with: judged, skipped, label: 'judged with: ' + judged.join(', '), skipped_label: skipped.length ? 'not checked: ' + skipped.map(x => x.rule).join(', ') : 'not checked: nothing' };
}

export const ENVELOPE_MIN_FRAMES = 30, WINDOW_MIN_FRAMES = 15, EDGE_SKIP_FRAMES = 60;

export function phaseRules(phase, frames) {
  const r = [...(PHASE_RULES[phase] || [])];
  if (phase === 'locomotion' && frames >= ENVELOPE_MIN_FRAMES) r.push('envelope');
  return r;
}

export const CONTACT_RULES = ['foot_slide', 'plant_pop'];

export function phasePlan(phases, n, minWindow = WINDOW_MIN_FRAMES, pad = SEG.pad) {
  return phases.map((p, i) => {
    const frames = p.end - p.start + 1;
    let from = p.start, to = p.end;
    if (frames < minWindow) {
      from = Math.max(0, p.start - Math.floor((minWindow - frames) / 2));
      to = Math.min(n - 1, from + minWindow - 1);
      from = Math.max(0, to - minWindow + 1);
    }
    const rules = phaseRules(p.phase, frames);
    const contact = rules.filter(r => CONTACT_RULES.includes(r)), other = rules.filter(r => !CONTACT_RULES.includes(r));
    const passes = [];
    if (contact.length) {
      const prev = phases[i - 1], next = phases[i + 1];
      let cf = prev && prev.phase !== 'slide' ? Math.max(0, Math.min(from, p.start - pad)) : from;
      let ct = next && next.phase !== 'slide' ? Math.min(n - 1, Math.max(to, p.end + pad)) : to;
      if (n > EDGE_SKIP_FRAMES && ct - cf + 1 <= EDGE_SKIP_FRAMES) {
        if (cf === 0) ct = Math.min(n - 1, EDGE_SKIP_FRAMES);
        else if (ct === n - 1) cf = Math.max(0, n - 1 - EDGE_SKIP_FRAMES);
      }
      passes.push({ from: cf, to: ct, rules: contact });
    }
    if (other.length) passes.push({ from, to, rules: other });
    return { phase: p.phase, start: p.start, end: p.end, from, to, rules, passes, ...phaseScope(p.phase, frames) };
  });
}

export function phasedTasteInput(input, opts = {}) {
  const phases = opts.phases || segmentPhases(input, opts);
  return { ...input, phases: phasePlan(phases, input.B.length, opts.minWindow), integrity: { rules: INTEGRITY_RULES } };
}

export function groupByPhase(result) {
  const s = summarize(result);
  const groups = (result.phases || []).map((p, i) => ({ ...(p.judged_with ? {} : phaseScope(p.phase, p.end - p.start + 1)), ...p, index: i, findings: [] }));
  const whole = { phase: 'whole take', index: -1, start: null, end: null, findings: [] };
  for (const f of s.findings) (groups[f.phase_index] || whole).findings.push(f);
  return { summary: s, groups: whole.findings.length ? [whole, ...groups] : groups };
}
