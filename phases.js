export const BONES = ['pelvis', 'spine_01', 'spine_03', 'spine_05', 'neck_01', 'head', 'clavicle_l', 'upperarm_l', 'lowerarm_l', 'hand_l',
  'clavicle_r', 'upperarm_r', 'lowerarm_r', 'hand_r', 'thigh_l', 'calf_l', 'foot_l', 'ball_l', 'thigh_r', 'calf_r', 'foot_r', 'ball_r'];

const zUpCm = (x, y, z, k) => [x * k, -z * k, y * k];

export function boneIndices(find) {
  const map = [], missing = [];
  for (const name of BONES) {
    const i = find(name);
    if (i === undefined) missing.push(name); else map.push(i);
  }
  return { map, missing };
}

export function phaseInput({ W, nb, n, fps, upm, map, restW, from = 0, to = n - 1 }) {
  const k = 100 / upm, B = [];
  for (let f = from; f <= to; f++) {
    const row = [];
    for (const i of map) { const o = (f * nb + i) * 16 + 12; row.push(zUpCm(W[o], W[o + 1], W[o + 2], k).map(v => +v.toFixed(3) + 0)); }
    B.push(row);
  }
  const rest = map.map(i => { const m = restW[i]; return zUpCm(m[12], m[13], m[14], k).map(v => +v.toFixed(3) + 0); });
  return { fps, B, rest, frame_offset: from };
}

const PELVIS = 0, CHEST = 3, HAND = [9, 13], THIGH = [14, 18], CALF = [15, 19], FOOT = [16, 20], BALL = [17, 21];

export const PHASES = ['locomotion', 'in_place', 'flight', 'vault', 'plant', 'slide', 'kneel', 'crouch', 'prone', 'rise', 'stand', 'upper_body', 'freeform'];

export const GAIT_PHASES = ['locomotion', 'in_place'];

export const SEG = { contact: .1, air: .1, flightRise: 1.06, low: .8, plantDip: .9, riseBelow: .95, kneeLow: .12, rise: .35, drop: .35, slide: .6, footStill: .3, minLen: 6, minFlight: 4, gravity: 981, airSlack: 1.5, airMargin: 0.1, inPlaceRatio: 0.35, inPlaceFoot: 0.5, proneLow: 0.55, proneTilt: 55, handLow: 0.6, armActive: 0.5, straight: 0.5, gaitPeriodic: 0.3, minTravel: 0.3, crouchKnee: 45, standFloor: 0.9, freeformSecs: 1, crouchMaxSpeed: 1.6, crouchPelvis: 0.8, kneelTurn: 40, plantMaxSpeed: 1.5, gaitAlt: 0.35 };

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
  const kneeH = [0, 1].map(s => B.map(r => r[CALF[s]][2] - g)), knee = kneeH[0].map((v, f) => Math.min(v, kneeH[1][f]));
  const ph = B.map(r => r[PELVIS][2] - g);
  const both = [];
  for (let f = 0; f < n; f++) if (foot[0][f] < SEG.contact * leg && foot[1][f] < SEG.contact * leg) both.push(ph[f]);
  // Few double-support frames (runs, sprints): the pelvis height while any foot is down.
  const one = both.length >= 5 ? both : ph.filter((_, f) => Math.min(foot[0][f], foot[1][f]) < SEG.contact * leg);
  const stand = Math.max(one.length >= 5 ? median(one) : .97 * leg, SEG.standFloor * leg);
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
  const kneeSpeed = [0, 1].map(s => { const dx = deriv(smooth(B.map(r => r[CALF[s]][0]), 2), fps), dy = deriv(smooth(B.map(r => r[CALF[s]][1]), 2), fps); return dx.map((v, i) => Math.hypot(v, dy[i])); });
  const yaw = B.map(r => Math.atan2(r[THIGH[0]][1] - r[THIGH[1]][1], r[THIGH[0]][0] - r[THIGH[1]][0]) * 180 / Math.PI);
  const kneeFlex = B.map(r => Math.min(...[0, 1].map(s => ang(r[THIGH[s]], r[CALF[s]], r[FOOT[s]]))));
  return { n, fps, leg, ground: g, stand, foot, knee, kneeH, kneeSpeed, yaw, pelvis: ph, pelvisSmooth: pz, speed, vz, footSpeed, kneeFlex, tilt, hand, armSpeed, handSpeed, px, py };
}

// A knee on the floor and not travelling (a crouch walk swings its knees low but moving).
const kneeDown = (sig, f) => sig.kneeH ? [0, 1].some(s => sig.kneeH[s][f] < SEG.kneeLow * sig.stand && sig.kneeSpeed[s][f] < SEG.footStill * sig.stand) : sig.knee[f] < SEG.kneeLow * sig.stand;

export function labelFrames(sig) {
  const { n, leg, stand, foot, knee, pelvis, pelvisSmooth, speed, vz, footSpeed } = sig;
  const air = Array.from({ length: n }, (_, f) => Math.min(foot[0][f], foot[1][f]) > SEG.air * leg);
  const flight = new Array(n).fill(false);
  for (const r of runs(air)) {
    const len = r.end - r.start + 1;
    if (!r.phase || len < SEG.minFlight) continue;
    let top = -Infinity;
    for (let f = r.start; f <= r.end; f++) top = Math.max(top, pelvis[f]);
    let feet = 0;
    for (let f = r.start; f <= r.end; f++) feet = Math.max(feet, Math.min(foot[0][f], foot[1][f]));
    const rise = Math.max(0, top - Math.min(pelvis[r.start], pelvis[r.end]), feet), airtime = len / sig.fps;
    const ballistic = 2 * Math.sqrt(2 * rise / SEG.gravity) * SEG.airSlack + SEG.airMargin;
    // An air run cut by the clip's first or last frame (a jump-start or fall loop) has no measurable airtime.
    const cut = r.start === 0 || r.end === n - 1;
    if (top > SEG.flightRise * stand && (cut || airtime <= ballistic)) for (let f = r.start; f <= r.end; f++) flight[f] = true;
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
    else if (h < SEG.low * stand) out[f] = rising ? 'rise' : dropping || (speed[f] > SEG.slide * stand && !oneStill) ? 'slide' : kneeDown(sig, f) ? 'kneel' : 'crouch';
    else if (flight[f]) out[f] = 'flight';
    else if (vault[f]) out[f] = 'vault';
    else if (rising && h < SEG.riseBelow * stand) out[f] = 'rise';
    else if ((h < SEG.plantDip * stand && speed[f] < SEG.plantMaxSpeed * leg) || (dropping && h < SEG.riseBelow * stand)) out[f] = 'plant';
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
    // A crouch that turns on a knee (over 40 deg/s) pivots it: crouch, not a held kneel.
    if (seg.phase === 'kneel' && sig.yaw) { let t = 0; for (let k = 1; k < idx.length; k++) { let d = sig.yaw[idx[k]] - sig.yaw[idx[k - 1]]; d -= 360 * Math.round(d / 360); t += d; } if (Math.abs(t) > 15 && Math.abs(t) / (idx.length / sig.fps) > SEG.kneelTurn) return { ...seg, phase: 'crouch' }; }
    // Low and fast with the feet taking turns (one swings while the other stands): a crouch run, not a slide.
    const alt = med(idx.map(f => Math.abs(sig.footSpeed[0][f] - sig.footSpeed[1][f]) / (sig.footSpeed[0][f] + sig.footSpeed[1][f] + 1e-6)));
    if (seg.phase === 'slide' && alt > SEG.gaitAlt) return { ...seg, phase: 'crouch' };
    // A held low "plant" that travels on alternating feet is low gait: crouch below crouchPelvis, else a walk.
    if (seg.phase === 'plant' && alt > SEG.gaitAlt && idx.length >= SEG.minLen) {
      const net = Math.hypot(sig.px[seg.end] - sig.px[seg.start], sig.py[seg.end] - sig.py[seg.start]) / (idx.length / sig.fps) / sig.leg;
      if (net >= SEG.minTravel) return { ...seg, phase: med(idx.map(f => sig.pelvis[f])) < SEG.crouchPelvis * sig.leg ? 'crouch' : 'locomotion' };
    }
    if (seg.phase !== 'locomotion' && seg.phase !== 'in_place') return seg;
    const secs = idx.length / sig.fps, a = seg.start, b = seg.end;
    let path = 0;
    for (let f = a + 1; f <= b; f++) path += Math.hypot(sig.px[f] - sig.px[f - 1], sig.py[f] - sig.py[f - 1]);
    const net = Math.hypot(sig.px[b] - sig.px[a], sig.py[b] - sig.py[a]), travel = net / secs / sig.leg;
    if (seg.phase === 'locomotion' && med(idx.map(f => sig.speed[f])) < SEG.crouchMaxSpeed * sig.leg && med(idx.map(f => sig.kneeFlex[f])) > SEG.crouchKnee && med(idx.map(f => sig.pelvis[f])) < SEG.crouchPelvis * sig.leg) return { ...seg, phase: 'crouch' };
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
