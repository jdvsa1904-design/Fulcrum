// Rule -> fix mapping for the one-click FIX. Phase-aware: the phase a finding sits in picks the solver.
// Fix objects use the formats app.js already understands (S.fixes / S.locks / S.pins); new types are
// listed in NEW_FIX_TYPES. Each candidate carries parameter variants for the engine to search. Pure.
import { phaseOf, frameText, sideText } from './facts.js';

export const STAGES = ['preflight', 'noise', 'structure', 'body', 'contacts', 'polish', 'stop'];
export const STAGE_TEXT = { preflight: 'Rig check', noise: 'Noise', structure: 'Shot seams', body: 'Body and root', contacts: 'Contacts', polish: 'Polish', stop: 'Stop' };

// Root solvers run in this order, each on the output of the one before (chainRootOffsets), never summed from one pass.
export const ROOT_ORDER = ['seams', 'travel', 'join', 'ballistic', 'momentum', 'fulcrum', 'kneeFloor', 'groundDrop', 'pull'];

export const NEW_FIX_TYPES = Object.freeze({
  knee: '{type:"knee", side, a, b, mode:"spike"|"wobble"|"extension", strength?, maxDeg?}: re-time the knee flex angle (contacts.js kneeWobbleFix over [a,b] for spike/wobble, kneeExtensionFix clamp at maxDeg for extension) and write it back as a rotation of calf_<side> about its hinge, after cleanup and before the passes.',
  ankleLock: '{type:"ankleLock", side, a, b, perimeter (cm), maxSlide (m/s), floorProximity (cm), recovery (s)}: contacts.js ankleLock on the ball/ankle track gives a per-frame moving anchor; IK the foot to it (like ground targets) with recoveryBlend after release.',
  pinRight: 'pins[] entry with hand:"r" (world pin for the right hand). app.js runPass only pins the left arm today.',
  despikeBones: '{type:"despike"|"filter", bones:[indices]} already works in wire/cleanup.js; app.js passes it through unchanged. Listed so the label code knows a bone subset was used.',
});

export const PHASE_GROUP = Object.freeze({ locomotion: 'gait', in_place: 'gait', flight: 'flight', vault: 'vault', slide: 'slide', rise: 'rise', prone: 'prone', plant: 'planted', stand: 'planted', kneel: 'planted', crouch: 'planted', upper_body: 'planted', freeform: 'planted' });
const groupOf = p => PHASE_GROUP[p] ?? 'planted';

const ALIAS = { snap: 'pop', slide: 'foot_slide', float: 'ground_float', bone_length: 'bone_stretch', nan: 'nonfinite', lock_fail: 'lock_residual', pin_fail: 'pin_residual', constraint_residual: 'lock_residual', freeze: 'freeze', warn: 'warning' };
export function sideOf(part) {
  const p = String(part ?? '').toLowerCase();
  if (/\bleft\b|^l$|_l\b/.test(p)) return 'l';
  if (/\bright\b|^r$|_r\b/.test(p)) return 'r';
  return null;
}
// Accepts this judge's findings and external ones (taste-style frame_start/frame_end/body_part/warn).
export function normalizeFinding(f) {
  const a = f.a ?? f.frame_start ?? 0, b = f.b ?? f.frame_end ?? a, sev = f.severity === 'warn' ? 'warning' : f.severity;
  return { ...f, rule: ALIAS[f.rule] ?? f.rule, severity: sev ?? 'warning', a, b, side: f.side !== undefined ? f.side : sideOf(f.body_part) };
}

const clone = x => JSON.parse(JSON.stringify(x));
const strip = x => { if (!x || typeof x !== 'object') return x; const { hits, fixId, _was, ...y } = x; return y; };
export function changeKey(change) {
  return JSON.stringify({ f: (change.fixes || []).map(strip), l: (change.locks || []).map(strip), p: (change.pins || []).map(x => ({ ...strip(x), world: x.world ? x.world.map(v => +(+v).toFixed(3)) : undefined })), e: change.edits || [], dl: change.dropLocks || [], df: change.dropFixes || [] });
}

// Pure: returns a new state with the change applied. Every added fix, lock and pin carries fixId so the
// UI can show each kept change as its own toggle/layer.
export function applyChange(state, change, tag = null) {
  const st = { fixes: clone(state.fixes || []), locks: clone(state.locks || []), pins: clone(state.pins || []) };
  const id = tag?.id;
  const same = (x, y) => x.side === y.side && x.a === y.a && x.b === y.b;
  if (change.dropLocks?.length) st.locks = st.locks.filter(l => !change.dropLocks.some(d => same(l, d)));
  if (change.dropFixes?.length) st.fixes = st.fixes.filter(x => !change.dropFixes.some(d => x.type === d.type && (d.a === undefined || (x.a === d.a && x.b === d.b)) && (d.side === undefined || x.side === d.side)));
  // An edited item becomes this change's own (fixId = the new id) and keeps the item it replaced in _was, so muting the
  // change puts the original back instead of deleting it.
  for (const e of change.edits || []) if (st.fixes[e.index]) { const was = clone(st.fixes[e.index]); Object.assign(st.fixes[e.index], e.patch, id ? { fixId: id, _was: was } : {}); }
  for (const x of change.fixes || []) st.fixes.push({ ...clone(x), ...(id ? { fixId: id } : {}) });
  for (const x of change.locks || []) st.locks.push({ ...clone(x), ...(id ? { fixId: id } : {}) });
  for (const x of change.pins || []) st.pins.push({ ...clone(x), ...(id ? { fixId: id } : {}) });
  return st;
}

// Remove every fix/lock/pin a kept change added (a muted FIX layer).
export function withoutFixIds(state, ids) {
  const s = new Set(ids), keep = x => !s.has(x.fixId);
  return { fixes: (state.fixes || []).filter(keep), locks: (state.locks || []).filter(keep), pins: (state.pins || []).filter(keep) };
}

const combine4 = (o, v) => v ? (o ? [o[0] + v[0], o[1] + (v[1] || 0), o[2] + v[2], (o[3] || 0) + (v[3] || 0)] : [v[0], v[1] || 0, v[2], v[3] || 0]) : o;
// Chained root offsets: each producer sees the motion with every earlier producer applied.
// producers: [{ name, fn(W, acc) -> offsets[] (n entries of [x,y,z] or [x,y,z,yaw], null = none) }]; pass(acc) -> W.
export function chainRootOffsets({ n, producers, pass }) {
  let acc = new Array(n).fill(null), W = pass(acc);
  const used = [];
  const order = name => { const i = ROOT_ORDER.indexOf(name); return i < 0 ? ROOT_ORDER.length : i; };
  for (const pr of [...producers].sort((x, y) => order(x.name) - order(y.name))) {
    const off = pr.fn(W, acc);
    if (!off || !off.some(Boolean)) continue;
    acc = acc.map((o, f) => combine4(o, off[f]));
    W = pass(acc);
    used.push(pr.name);
  }
  return { offsets: acc, W, used };
}

const clampR = (a, b, pad, n) => [Math.max(0, a - pad), Math.min(n - 1, b + pad)];
const overlaps = (x, a, b) => x.a <= b && x.b >= a;
const sides = f => f.side ? [f.side] : ['l', 'r'];
const legBones = (rig, s) => ['thigh_' + s, 'calf_' + s, 'foot_' + s, 'ball_' + s].map(k => rig[k]).filter(v => v !== undefined);

function plantFor(facts, side, a, b, lockableFirst = true) {
  const ps = facts.contacts.feet[side]?.plants.filter(p => overlaps(p, a, b) && !p.touch) || [];
  const ov = p => Math.min(p.b, b) - Math.max(p.a, a);
  return ps.sort((x, y) => (lockableFirst ? (y.lockable - x.lockable) : 0) || ov(y) - ov(x))[0] ?? null;
}

// Contact-aware range split: [a,b] minus frames where that side is locked or planted (min length len).
function freeRanges(a, b, busy, len = 6) {
  const out = [];
  let s = -1;
  for (let f = a; f <= b + 1; f++) {
    const free = f <= b && !busy(f);
    if (free && s < 0) s = f;
    if (!free && s >= 0) { if (f - s >= len) out.push([s, f - 1]); s = -1; }
  }
  return out;
}

function filterChange(f, ctx, fc) {
  const { rig, facts, state, n } = ctx, [a, b] = clampR(f.a, f.b, 2, n);
  const legs = { l: legBones(rig, 'l'), r: legBones(rig, 'r') }, legSet = new Set([...legs.l, ...legs.r]);
  const all = Array.from({ length: facts.nb }, (_, i) => i), upper = all.filter(i => !legSet.has(i));
  const fixes = [{ type: 'filter', a, b, fc, bones: upper }];
  for (const s of ['l', 'r']) {
    const busy = g => facts.contacts.feet[s].mask[g] === 1 || (state.locks || []).some(l => l.side === s && g >= l.a && g <= l.b);
    for (const [x, y] of freeRanges(a, b, busy)) fixes.push({ type: 'filter', a: x, b: y, fc, bones: legs[s] });
  }
  return { fixes };
}

function cand(stage, rule, f, priority, label, change, variants = []) {
  return { stage, rule, finding: f, priority, label, change, variants: variants.filter(Boolean) };
}

const H = {};
H.pop = H.plant_pop = (f, ctx) => {
  const { n, stage } = ctx;
  if (stage === 'noise') {
    const [a, b] = clampR(f.a, f.b, 2, n), [a4, b4] = clampR(f.a, f.b, 4, n);
    return [cand('noise', f.rule, f, 10, `Remove the pop at ${frameText(f.a, f.b)}`, { fixes: [{ type: 'despike', a, b, bones: 'all' }] },
      [{ fixes: [{ type: 'despike', a: a4, b: b4, bones: 'all' }] }, { fixes: [{ type: 'median', a, b, window: 5 }] }])];
  }
  if (stage === 'polish') {
    const [a, b] = clampR(f.a, f.b, 4, n), [a6, b6] = clampR(f.a, f.b, 6, n);
    return [cand('polish', f.rule, f, 50, `Smooth through the snap at ${frameText(f.a, f.b)}`, { fixes: [{ type: 'smooth', a, b }] }, [{ fixes: [{ type: 'smooth', a: a6, b: b6 }] }])];
  }
  return [];
};
H.crossing = (f, ctx) => {
  if (ctx.stage !== 'polish' || f.severity !== 'blocker') return [];
  const [a, b] = clampR(f.a, f.b, 4, ctx.n);
  return [cand('polish', f.rule, f, 55, `Smooth the limb crossing at ${frameText(f.a, f.b)}`, { fixes: [{ type: 'smooth', a, b }] })];
};
H.freeze = (f, ctx) => {
  if (ctx.stage !== 'polish') return [];
  const [a, b] = clampR(f.a, f.b, 2, ctx.n), [a4, b4] = clampR(f.a, f.b, 4, ctx.n);
  return [cand('polish', f.rule, f, 56, `Blend through the frozen frames ${frameText(f.a, f.b)}`, { fixes: [{ type: 'median', a, b, window: 5 }] }, [{ fixes: [{ type: 'smooth', a: a4, b: b4 }] }])];
};

H.knee_pop = (f, ctx) => {
  if (ctx.stage !== 'noise' || !f.side) return [];
  const { n, rig } = ctx, [a1, b1] = clampR(f.a, f.b, 1, n), [a2, b2] = clampR(f.a, f.b, 2, n), s = f.side;
  return [cand('noise', f.rule, f, 12, `Take the pop out of the ${sideText(s)} knee at ${frameText(f.a, f.b)}`, { fixes: [{ type: 'knee', side: s, a: f.a, b: f.b, mode: 'spike' }] },
    [{ fixes: [{ type: 'knee', side: s, a: a1, b: b1, mode: 'spike' }] }, { fixes: [{ type: 'despike', a: a2, b: b2, bones: legBones(rig, s) }] }])];
};
H.knee_wobble = (f, ctx) => ctx.stage === 'noise' && f.side ? [cand('noise', f.rule, f, 14, `Steady the ${sideText(f.side)} knee wobble at ${frameText(f.a, f.b)}`, { fixes: [{ type: 'knee', side: f.side, a: f.a, b: f.b, mode: 'wobble' }] },
  [{ fixes: [{ type: 'knee', side: f.side, a: f.a, b: f.b, mode: 'wobble', strength: 0.6 }] }])] : [];
H.knee_hyperextension = (f, ctx) => {
  if (ctx.stage !== 'contacts' || !f.side) return [];
  const [a, b] = clampR(f.a, f.b, 2, ctx.n), k = (x, y) => ({ fixes: [{ type: 'knee', side: f.side, a: x, b: y, mode: 'extension', maxDeg: 0 }] });
  return [cand('contacts', f.rule, f, 60, `Stop the ${sideText(f.side)} knee bending backwards at ${frameText(f.a, f.b)}`, k(f.a, f.b), [k(a, b)])];
};

H.jitter = (f, ctx) => {
  if (ctx.stage !== 'noise' && ctx.stage !== 'polish') return [];
  const top = ctx.fps / 2.5, fcs = (ctx.stage === 'noise' ? [8, 6, 4] : [6, 4]).map(x => Math.min(x, top));
  return [cand(ctx.stage, f.rule, f, ctx.stage === 'noise' ? 20 : 60, `Filter jitter at ${frameText(f.a, f.b)} (planted legs left alone)`, filterChange(f, ctx, fcs[0]), fcs.slice(1).map(fc => filterChange(f, ctx, fc)))];
};

H.seam_jump = (f, ctx) => {
  if (ctx.stage !== 'structure') return [];
  const i = ctx.state.fixes.findIndex(x => x.type === 'seams');
  if (i < 0) return [cand('structure', f.rule, f, 10, 'Line up the shot seams', { fixes: [{ type: 'seams', blend: 6, mode: 'carry', force: false }] },
    [{ fixes: [{ type: 'seams', blend: 10, mode: 'carry', force: false }] }, { fixes: [{ type: 'seams', blend: 6, mode: 'local', force: false }] }])];
  return [cand('structure', f.rule, f, 11, 'Retune the shot seam match', { edits: [{ index: i, patch: { force: true } }] }, [{ edits: [{ index: i, patch: { blend: 10 } }] }, { edits: [{ index: i, patch: { mode: 'local' } }] }])];
};

H.ground_float = (f, ctx) => {
  const { facts, state, n, stage } = ctx, g = groupOf(f.phase);
  const fl = facts.flights.find(x => x.a <= f.b + 1 && x.b >= f.a - 1);
  const flightLike = fl && fl.plausible && (g === 'flight' || fl.flightPhase > 0.5);
  if (flightLike) {
    if (stage !== 'body' || state.fixes.some(x => x.type === 'ballistic' && overlaps(x, fl.a, fl.b))) return [];
    return [cand('body', f.rule, f, 20, `Put the jump ${frameText(fl.a, fl.b)} on a ballistic arc (${fl.report.airtime_s.toFixed(2)} s in the air)`, { fixes: [{ type: 'ballistic', a: fl.a, b: fl.b }] },
      [fl.a > 0 && fl.b < n - 1 ? { fixes: [{ type: 'ballistic', a: fl.a - 1, b: fl.b + 1 }] } : null])];
  }
  if (stage !== 'contacts' || g === 'vault' || g === 'prone') return [];
  const [a, b] = clampR(f.a, f.b, 3, n), [a6, b6] = clampR(f.a, f.b, 6, n), ss = sides(f);
  const mk = (x, y, list) => ({ fixes: list.map(s => ({ type: 'ground', a: x, b: y, side: s })) });
  const low = ss.length > 1 ? ss.map(s => ({ s, h: Math.min(...Array.from(facts.contacts.height[s].subarray(f.a, f.b + 1))) })).sort((x, y) => x.h - y.h)[0].s : null;
  return [cand('contacts', f.rule, f, 30, `Ground the floating ${ss.length > 1 ? 'feet' : sideText(ss[0]) + ' foot'} at ${frameText(f.a, f.b)}`, mk(a, b, ss), [mk(a6, b6, ss), low ? mk(a, b, [low]) : null])];
};

H.ground_sink = (f, ctx) => {
  if (ctx.stage !== 'contacts') return [];
  const [a, b] = clampR(f.a, f.b, 3, ctx.n), [a6, b6] = clampR(f.a, f.b, 6, ctx.n), side = f.side ?? null;
  const fl = (x, y, o = {}) => ({ fixes: [{ type: 'floor', a: x, b: y, side, stiffness: 0, pivot: 'ankle', ...o }] });
  return [cand('contacts', f.rule, f, 40, `Lift the ${side ? sideText(side) + ' foot' : 'feet'} out of the floor at ${frameText(f.a, f.b)}`, fl(a, b), [fl(a, b, { pivot: 'toes' }), fl(a, b, { stiffness: 0.5 }), fl(a6, b6)])];
};

function lockCands(f, ctx, s) {
  const { facts, state, n } = ctx;
  if (state.locks.some(l => l.side === s && l.a <= f.a && l.b >= f.b)) return [];
  const p = plantFor(facts, s, f.a, f.b), [a, b] = p ? [p.a, p.b] : clampR(f.a, f.b, 2, n);
  if (b - a < 2) return [];
  const ak = (x, y, o = {}) => ({ fixes: [{ type: 'ankleLock', side: s, a: x, b: y, perimeter: 3, maxSlide: 0, floorProximity: 10, recovery: 0.15, ...o }] });
  const drop = state.locks.filter(l => l.side === s && overlaps(l, a, b)).map(({ side, a, b }) => ({ side, a, b }));
  const lock = (x, y) => ({ locks: [{ side: s, a: x, b: y }], ...(drop.length ? { dropLocks: drop } : {}) });
  return [cand('contacts', f.rule, f, 10, `Lock the ${sideText(s)} foot ${frameText(a, b)}`, lock(a, b), [b - a > 6 ? lock(a + 1, b - 1) : null, ak(a, b), ak(a, b, { perimeter: 6 })])];
}
H.foot_slide = H.foot_skate = (f, ctx) => {
  if (ctx.stage !== 'contacts') return [];
  const g = groupOf(f.phase);
  if (g === 'flight' || g === 'vault' || g === 'prone') return [];
  if (g === 'slide') {
    return sides(f).flatMap(s => {
      const p = plantFor(ctx.facts, s, f.a, f.b, false), a = p ? p.a : f.a, b = p ? p.b : f.b, v = Math.max(0.1, p?.speed ?? 0.5);
      const ak = o => ({ fixes: [{ type: 'ankleLock', side: s, a, b, perimeter: 3, maxSlide: +v.toFixed(2), floorProximity: 10, recovery: 0.15, ...o }] });
      return [cand('contacts', f.rule, f, 11, `Let the ${sideText(s)} foot slide smoothly ${frameText(a, b)} (moving anchor)`, ak({}), [ak({ perimeter: 6 }), ak({ maxSlide: +(v * 0.5).toFixed(2) })])];
    });
  }
  return sides(f).flatMap(s => lockCands(f, ctx, s));
};

H.lock_residual = (f, ctx) => {
  if (ctx.stage !== 'contacts' || !f.side) return [];
  const lk = ctx.state.locks.find(l => l.side === f.side && overlaps(l, f.a, f.b));
  if (!lk) return [];
  const d = { dropLocks: [{ side: lk.side, a: lk.a, b: lk.b }] };
  return [cand('contacts', f.rule, f, 15, `The ${sideText(f.side)} foot cannot reach its lock; ground it instead ${frameText(lk.a, lk.b)}`, { ...d, fixes: [{ type: 'ground', a: lk.a, b: lk.b, side: lk.side }] },
    [f.a > lk.a + 2 ? { ...d, locks: [{ side: lk.side, a: lk.a, b: f.a - 1 }] } : null, f.b < lk.b - 2 ? { ...d, locks: [{ side: lk.side, a: f.b + 1, b: lk.b }] } : null, d])];
};

H.pin_residual = (f, ctx) => {
  if (ctx.stage !== 'contacts' || ctx.state.fixes.some(x => x.type === 'pull') || !ctx.state.pins.some(p => p.world)) return [];
  const pull = (p, gr) => ({ fixes: [{ type: 'pull', pull: p, gravity: gr, iterations: 10 }] });
  return [cand('contacts', f.rule, f, 52, 'Pull the body toward the pinned hand', pull(1, 0.6), [pull(0.5, 0.6), pull(1, 0)])];
};

H.hand_slip = (f, ctx) => {
  const { facts, rig, motion, stage, state } = ctx, g = groupOf(f.phase), s = f.side;
  const r = facts.contacts.hands[s]?.runs.find(x => overlaps(x, f.a, f.b));
  if (!r) return [];
  if (stage === 'body' && (g === 'vault' || f.phase === 'plant' || f.phase === 'kneel' || f.phase === 'crouch'))
    return state.fixes.some(x => x.type === 'fulcrum' && x.idx === r.idx && overlaps(x, r.a, r.b)) ? [] : [cand('body', f.rule, f, 40, `Pivot the body around the planted ${sideText(s)} hand ${frameText(r.a, r.b)}`, { fixes: [{ type: 'fulcrum', a: r.a, b: r.b, idx: r.idx, pivot: r.pivot, slip: +r.slip_cm.toFixed(2) }] })];
  if (stage !== 'contacts' || s !== 'l' || !motion?.W || rig.hand_l === undefined || state.pins.some(p => overlaps(p, r.a, r.b))) return [];
  const o = (r.anchor * facts.nb + rig.hand_l) * 16, world = Array.from(motion.W.subarray(o, o + 16));
  const pin = { a: r.a, b: r.b, ref: r.anchor, world };
  const withPull = g === 'rise' && !state.fixes.some(x => x.type === 'pull') ? { pins: [pin], fixes: [{ type: 'pull', pull: 1, gravity: 0.6, iterations: 10 }] } : null;
  return [cand('contacts', f.rule, f, 50, `Pin the left hand where it is at f${r.anchor} (${frameText(r.a, r.b)})`, { pins: [pin] }, [withPull])];
};

H.knee_slip = (f, ctx) => {
  if (ctx.stage !== 'body' || !['plant', 'kneel', 'crouch'].includes(f.phase)) return [];
  const r = ctx.facts.contacts.knees[f.side]?.runs.find(x => overlaps(x, f.a, f.b));
  if (!r || ctx.state.fixes.some(x => x.type === 'fulcrum' && x.idx === r.idx && overlaps(x, r.a, r.b))) return [];
  return [cand('body', f.rule, f, 41, `Pivot the body around the ${sideText(f.side)} knee on the floor ${frameText(r.a, r.b)}`, { fixes: [{ type: 'fulcrum', a: r.a, b: r.b, idx: r.idx, pivot: r.pivot, slip: +r.slip_cm.toFixed(2) }] })];
};

H.overload = (f, ctx) => {
  if (ctx.stage !== 'body') return [];
  const [a, b] = clampR(f.a, f.b, 3, ctx.n), [a6, b6] = clampR(f.a, f.b, 6, ctx.n);
  if (ctx.state.fixes.some(x => x.type === 'momentum' && x.a <= a && x.b >= b)) return [];
  return [cand('body', f.rule, f, 30, `Limit the momentum change to what foot friction allows at ${frameText(f.a, f.b)}`, { fixes: [{ type: 'momentum', a, b }] }, [{ fixes: [{ type: 'momentum', a: a6, b: b6 }] }])];
};

H.foot_tilt = (f, ctx) => ctx.stage === 'contacts' && !ctx.state.fixes.some(x => x.type === 'level' && (!x.side || x.side === f.side)) ? [cand('contacts', f.rule, f, 45, `Level the ${sideText(f.side)} foot on the floor`, { fixes: [{ type: 'level', side: f.side ?? null }] }, [{ fixes: [{ type: 'level', side: null }] }])] : [];

// Rules from external judges (LocomotionTaste-style); kept so this planner is a superset of wire/autofix.js planFixes.
H.planted_foot_pitch_deg = H.heel_raise_pct = (f, ctx) => {
  if (ctx.stage !== 'contacts') return [];
  return ctx.state.locks.filter(l => (!f.side || l.side === f.side) && overlaps(l, f.a, f.b) && l.b - l.a >= 3 && !ctx.state.fixes.some(x => x.type === 'roll' && x.side === l.side && overlaps(x, l.a, l.b)))
    .map(l => cand('contacts', f.rule, f, 20, `Roll the locked ${sideText(l.side)} foot heel-to-toe ${frameText(l.a, l.b)}`, { fixes: [{ type: 'roll', side: l.side, a: l.a, b: l.b, start: 30, end: 60 }] }, [{ fixes: [{ type: 'roll', side: l.side, a: l.a, b: l.b, start: 20, end: 45 }] }]));
};
H.planted_toe_out_deg = H.planted_ankle_deg = (f, ctx) => {
  if (ctx.stage !== 'contacts') return [];
  const ti = ctx.state.fixes.findIndex(x => x.type === 'travel' && overlaps(x, f.a, f.b));
  if (ti >= 0 && groupOf(f.phase) === 'gait') {
    const v = f.value ?? 0, [lo, hi] = f.expected_range ?? [null, f.limit ?? null], t = hi != null && v > hi ? hi : lo != null && v < lo ? lo : null;
    const patch = f.rule === 'planted_toe_out_deg' ? { toe: Math.round((ctx.state.fixes[ti].toe ?? 0) + (t != null ? t - v : 0)) } : { pitch: t != null ? Math.round(t) : 0 };
    return [cand('contacts', f.rule, f, 46, `Adjust the travel ${Object.keys(patch)[0]}`, { edits: [{ index: ti, patch }] })];
  }
  return H.foot_tilt(f, ctx);
};
function travelCands(f, ctx, kind) {
  if (ctx.stage !== 'body' || groupOf(f.phase) !== 'gait') return [];
  const ti = ctx.state.fixes.findIndex(x => x.type === 'travel' && overlaps(x, f.a, f.b));
  if (ti >= 0) {
    const t = ctx.state.fixes[ti], ed = patch => ({ edits: [{ index: ti, patch }] });
    const vs = kind === 'bounce' ? [0, 2, 4].map(bo => ed({ bounce: bo })) : kind === 'width' ? [0.8, 1.2, 1.5].map(w => ed({ footsteps: true, width: w })) : [-2, 2].map(d => ed({ footsteps: true, cycle: Math.max(6, (t.cycle ?? Math.round(ctx.fps * 0.6)) + d) })).concat([0.9, 1.1].map(k => ed({ speed: +((t.speed ?? 3) * k).toFixed(2) })));
    return [cand('body', f.rule, f, 25, `Retune the footsteps (${kind})`, vs[0], vs.slice(1))];
  }
  let sp = 0; for (let g = f.a; g <= f.b; g++) sp += ctx.facts.pelvisSpeed[g];
  const speed = +Math.max(0.5, sp / Math.max(1, f.b - f.a + 1)).toFixed(2), base = { type: 'travel', a: f.a, b: f.b, speed, ease: 8, footsteps: true, balance: 0, width: 1 };
  return [cand('body', f.rule, f, 25, `Drive ${frameText(f.a, f.b)} with planned footsteps at ${speed} m/s`, { fixes: [base] }, [0.8, 1.2].map(w => ({ fixes: [{ ...base, width: w }] })))];
}
H.stride_ratio = (f, c) => travelCands(f, c, 'stride');
H.bounce = (f, c) => travelCands(f, c, 'bounce');
H.min_stance_width = H.max_stance_width = (f, c) => travelCands(f, c, 'width');
H.flight_airtime = H.ballistic_flight = (f, ctx) => H.ground_float({ ...f, phase: 'flight' }, ctx);

export const NO_AUTO_FIX = Object.freeze({ bone_stretch: 'a bone changes length: fix the rig or retarget, not the motion', nonfinite: 'the pose has broken values: reload the take', prop_penetration: 'a hand goes through a prop: move the prop or the hand by hand', balance: 'FIX only reports this; it does not change it' });
export const PRIORITY_RULES = Object.keys(H);

export function planCandidates({ findings = [], facts, state, stage, motion = null, rig = facts?.rig ?? {}, n = facts?.n, fps = facts?.fps ?? 30, includeInfo = false }) {
  const ctx = { facts, state: { fixes: state.fixes || [], locks: state.locks || [], pins: state.pins || [] }, stage, motion, rig, n, fps };
  const sevRank = { blocker: 0, warning: 1, info: 2 }, out = [], seen = new Set();
  for (const raw of findings) {
    const f = normalizeFinding(raw);
    if (f.severity === 'info' && !includeInfo) continue;
    if (!f.phase && facts) f.phase = phaseOf(facts, f.a, f.b);
    for (const c of (H[f.rule]?.(f, ctx) ?? [])) {
      const k = changeKey(c.change);
      if (seen.has(k)) continue;
      seen.add(k);
      out.push({ ...c, key: k, severity: f.severity });
    }
  }
  const mag = c => Number.isFinite(c.finding.value) && Number.isFinite(c.finding.limit) && c.finding.limit > 0 ? c.finding.value / c.finding.limit : 1;
  return out.sort((x, y) => sevRank[x.severity] - sevRank[y.severity] || x.priority - y.priority || mag(y) - mag(x) || x.finding.a - y.finding.a);
}

// Preflight: what the rig and inputs allow, in plain English. No fixes come out of it.
export function preflight(facts) {
  const notes = [];
  if (facts.missing.length) notes.push(`The rig is missing ${facts.missing.join(', ')}: contact fixes on those limbs are skipped.`);
  if (facts.phasesSource === 'missing-bones') notes.push('Phase detection needs the full mannequin bone set; the whole take is judged as one free-form phase.');
  else if (!facts.phasesSource || facts.phasesSource === 'none') notes.push('Phase code (phases.js / tastebridge.js) not found; the whole take is judged as one free-form phase.');
  const t = facts.trust, low = Array.from(t).filter(v => v < 0.5).length;
  if (low) notes.push(`${low} frames have low tracking trust; changes there are not counted against fidelity as much.`);
  if (Math.abs(facts.ground - facts.groundLegacy) > 2 * facts.upm / 100) notes.push(`Floor estimate ${((facts.ground - facts.groundLegacy) / facts.upm * 100).toFixed(1)} cm away from the old 3rd-percentile floor: part of the take sits below the floor.`);
  return notes;
}

