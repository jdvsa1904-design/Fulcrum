import { qrot } from '../../ik.js';
import { solveLeg, qmul, qconj } from './knee.js';

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const len = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => { const l = len(a); return l > 1e-12 ? mul(a, 1 / l) : [0, 0, 0]; };
const mid = (a, b) => mul(add(a, b), 0.5);
const ID = [0, 0, 0, 1];

const SIDES = ['Left', 'Right'];
const LIMB = { UpLeg: 'Hips', Leg: 'UpLeg', Foot: 'Leg', ToeBase: 'Foot', Shoulder: 'Spine*', Arm: 'Shoulder', ForeArm: 'Arm', Hand: 'ForeArm' };

export const HIK_SLOTS = Object.freeze([
  'Reference', 'Hips',
  ...SIDES.flatMap(s => ['UpLeg', 'Leg', 'Foot', 'ToeBase'].map(p => s + p)),
  'Spine', ...Array.from({ length: 9 }, (_, i) => `Spine${i + 1}`),
  'Neck', ...Array.from({ length: 9 }, (_, i) => `Neck${i + 1}`), 'Head',
  ...SIDES.flatMap(s => ['Shoulder', 'Arm', 'ForeArm', 'Hand'].map(p => s + p)),
]);

export const REQUIRED_SLOTS = Object.freeze([
  'Hips', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot',
  'Spine', 'LeftArm', 'LeftForeArm', 'LeftHand', 'RightArm', 'RightForeArm', 'RightHand', 'Head',
]);

export const NAME_DELIMITERS = Object.freeze([':', '|', '.', '_']);

const PART_WORDS = [
  ['ToeBase', /^(toebase|toe0?|toes|ball|toe_?base)$/],
  ['ForeArm', /^(forearm|lowerarm|elbow|lower_?arm|fore_?arm)$/],
  ['UpLeg', /^(upleg|thigh|upperleg|upper_?leg|hip|femur)$/],
  ['Leg', /^(leg|calf|shin|lowerleg|lower_?leg|knee|tibia)$/],
  ['Foot', /^(foot|ankle)$/],
  ['Shoulder', /^(shoulder|clavicle|collar)$/],
  ['Arm', /^(arm|upperarm|upper_?arm|humerus)$/],
  ['Hand', /^(hand|wrist)$/],
];

function splitName(name) {
  let n = String(name);
  for (const d of [':', '|']) n = n.slice(n.lastIndexOf(d) + 1);
  n = n.replace(/^(def|mch|org|drv|jnt|bn|b|bip\d*|bone)[-_ .]/i, '');
  return n;
}

function suffixes(name) {
  const out = [name];
  for (let i = 0; i < name.length; i++) if ('_. '.includes(name[i]) && i + 1 < name.length) out.push(name.slice(i + 1));
  return out;
}

function sideOf(raw) {
  const n = raw.toLowerCase();
  if (/^left|left$|^l[ _.]|[ _.]l$|[ _.]l[ _.]|^lf|_lf$/.test(n) || /(^|[^a-z])l(?=[A-Z])/.test(raw)) return 'Left';
  if (/^right|right$|^r[ _.]|[ _.]r$|[ _.]r[ _.]|^rt|_rt$/.test(n) || /(^|[^a-z])r(?=[A-Z])/.test(raw)) return 'Right';
  return null;
}

function stripSide(raw) {
  return raw.replace(/^(left|right)/i, '').replace(/(left|right)$/i, '')
    .replace(/^[lr](?=[ _.])|(?<=[ _.])[lr]$/i, '').replace(/[ _.][lr][ _.]/i, '_')
    .replace(/[ _.]+/g, '_').replace(/^_|_$/g, '').toLowerCase();
}

export function classifyName(name) {
  for (const part of suffixes(splitName(name))) {
    const c = classifyPart(part);
    if (c) return c;
  }
  return null;
}

function classifyPart(raw) {
  const low = raw.toLowerCase();
  const exact = HIK_SLOTS.find(s => s.toLowerCase() === low);
  if (exact && !/^(spine|neck)\d*$/.test(low)) return { slot: exact, rank: 0 };
  if (/^(hips|pelvis|root_?hips|hip_?root|cog)$/.test(low) || /^bip\d*$/.test(low)) return { slot: 'Hips', rank: 1 };
  if (/^(head|head_?top)$/.test(low)) return { slot: low === 'head' ? 'Head' : null, rank: 1 };
  const neck = low.match(/^neck[_.]?0*(\d*)$/);
  if (neck) return { slot: 'Neck', index: neck[1] === '' ? 0 : +neck[1], rank: 2 };
  const spine = low.match(/^(spine|chest|upperchest|abdomen|torso|lowerback|back)[_.]?0*(\d*)$/);
  if (spine) {
    const base = { spine: 0, lowerback: -1, abdomen: -1, back: 0, torso: 0, chest: 50, upperchest: 60 }[spine[1]];
    return { slot: 'Spine', index: base + (spine[2] === '' ? 0 : +spine[2]), rank: 2 };
  }
  const side = sideOf(raw);
  if (!side) return null;
  const word = stripSide(raw).replace(/[_.]?0*\d+$/, '');
  for (const [part, re] of PART_WORDS) if (re.test(word)) return { slot: side + part, rank: 3 };
  return null;
}

function ancestors(bones, i) {
  const out = [];
  for (let p = bones[i].parent; p !== undefined && p !== null && p >= 0; p = bones[p].parent) out.push(p);
  return out;
}

function children(bones) {
  const c = bones.map(() => []);
  bones.forEach((b, i) => { if (b.parent !== undefined && b.parent !== null && b.parent >= 0) c[b.parent].push(i); });
  return c;
}

function orderedChain(bones, members) {
  return members.slice().sort((a, b) => ancestors(bones, a).length - ancestors(bones, b).length);
}

function topologyLegs(bones, slots, kids, up, left) {
  const hips = slots.Hips;
  if (hips === undefined) return;
  const down = [];
  const walk = (i, chain) => {
    const nxt = kids[i];
    if (!nxt.length) return chain;
    let best = null, bestDrop = -Infinity;
    for (const k of nxt) {
      const drop = -dot(sub(bones[k].p, bones[i].p), up);
      if (drop > bestDrop) { bestDrop = drop; best = k; }
    }
    return walk(best, chain.concat(best));
  };
  for (const k of kids[hips]) {
    const ch = walk(k, [k]);
    const drop = dot(sub(bones[hips].p, bones[ch[ch.length - 1]].p), up);
    if (ch.length >= 3 && drop > 0.3 * len(sub(bones[hips].p, bones[ch[ch.length - 1]].p))) down.push(ch);
  }
  for (const ch of down) {
    const side = dot(sub(bones[ch[0]].p, bones[hips].p), left) >= 0 ? 'Left' : 'Right';
    const parts = ['UpLeg', 'Leg', 'Foot', 'ToeBase'];
    const start = ch.length >= 5 && len(sub(bones[ch[1]].p, bones[ch[0]].p)) < 0.15 * len(sub(bones[ch[ch.length - 1]].p, bones[ch[0]].p)) ? 1 : 0;
    parts.forEach((p, j) => { const b = ch[start + j]; if (b !== undefined && slots[side + p] === undefined) slots[side + p] = b; });
  }
}

export function characterize(bones, o = {}) {
  const up = unit(o.up ?? [0, 1, 0]), forward = unit(o.forward ?? [0, 0, 1]);
  const left = unit([up[1] * forward[2] - up[2] * forward[1], up[2] * forward[0] - up[0] * forward[2], up[0] * forward[1] - up[1] * forward[0]]);
  const slots = { ...(o.overrides ?? {}) };
  const spines = [], necks = [];
  const used = new Set(Object.values(slots));
  const hits = bones.map((b, i) => ({ i, c: classifyName(b.name) })).filter(h => h.c && h.c.slot);
  hits.sort((a, b) => a.c.rank - b.c.rank || ancestors(bones, a.i).length - ancestors(bones, b.i).length);
  for (const { i, c } of hits) {
    if (used.has(i)) continue;
    if (c.slot === 'Spine' && c.index !== undefined) { spines.push(i); used.add(i); continue; }
    if (c.slot === 'Neck' && c.index !== undefined) { necks.push(i); used.add(i); continue; }
    if (slots[c.slot] === undefined) { slots[c.slot] = i; used.add(i); }
  }
  if (slots.Hips === undefined && bones.length) {
    const kids = children(bones);
    let best = -1, bestScore = -1;
    bones.forEach((b, i) => { if (kids[i].length >= 3 && (best < 0 || ancestors(bones, i).length < bestScore)) { best = i; bestScore = ancestors(bones, i).length; } });
    if (best >= 0) slots.Hips = best;
  }
  const kids = children(bones);
  topologyLegs(bones, slots, kids, up, left);
  const hipsAnc = new Set(slots.Hips !== undefined ? [slots.Hips, ...ancestors(bones, slots.Hips)] : []);
  const spineChain = orderedChain(bones, spines.filter(i => !hipsAnc.has(i)));
  spineChain.slice(0, 10).forEach((b, k) => { slots[k === 0 ? 'Spine' : `Spine${k}`] = b; });
  orderedChain(bones, necks).slice(0, 10).forEach((b, k) => { slots[k === 0 ? 'Neck' : `Neck${k}`] = b; });
  if (slots.Head === undefined && slots.Neck !== undefined) {
    const k = kids[necks.length ? orderedChain(bones, necks).pop() : slots.Neck];
    if (k && k.length) slots.Head = k[0];
  }
  const missing = REQUIRED_SLOTS.filter(s => slots[s] === undefined);
  const rest = {};
  for (const [s, i] of Object.entries(slots)) rest[s] = { p: bones[i].p.slice(), q: (bones[i].q ?? ID).slice() };
  const P = s => rest[s]?.p;
  const legLength = side => (P(side + 'UpLeg') && P(side + 'Leg') && P(side + 'Foot')) ? len(sub(P(side + 'Leg'), P(side + 'UpLeg'))) + len(sub(P(side + 'Foot'), P(side + 'Leg'))) : 0;
  const metrics = {
    legLeft: legLength('Left'), legRight: legLength('Right'),
    hipWidth: P('LeftUpLeg') && P('RightUpLeg') ? len(sub(P('LeftUpLeg'), P('RightUpLeg'))) : 0,
  };
  return { slots, rest, metrics, missing, valid: missing.length === 0, up, forward, left, bones };
}

export const logicalRotation = (q, restQ) => qmul(q, qconj(restQ ?? ID));
export const physicalRotation = (q, restQ) => qmul(q, restQ ?? ID);

export function mapRotations(srcChar, srcPose, dstChar) {
  const out = {};
  for (const s of Object.keys(dstChar.slots)) {
    if (srcChar.slots[s] === undefined) continue;
    const q = srcPose[s]?.q;
    if (!q) continue;
    out[s] = physicalRotation(logicalRotation(q, srcChar.rest[s].q), dstChar.rest[s].q);
  }
  return out;
}

const EFFECTOR_NODES = {
  LeftAnkle: 'LeftFoot', RightAnkle: 'RightFoot', LeftKnee: 'LeftLeg', RightKnee: 'RightLeg',
  LeftWrist: 'LeftHand', RightWrist: 'RightHand', LeftElbow: 'LeftForeArm', RightElbow: 'RightForeArm',
  LeftHip: 'LeftUpLeg', RightHip: 'RightUpLeg', LeftShoulder: 'LeftArm', RightShoulder: 'RightArm',
  LeftFoot: 'LeftToeBase', RightFoot: 'RightToeBase', Head: 'Head',
};

export function effectorsFromPose(ch, pose) {
  const E = {};
  for (const [e, s] of Object.entries(EFFECTOR_NODES)) {
    if (ch.slots[s] === undefined || !pose[s]) continue;
    E[e] = { p: pose[s].p.slice(), q: logicalRotation(pose[s].q, ch.rest[s].q) };
  }
  if (E.LeftHip && E.RightHip) E.Hips = { p: mid(E.LeftHip.p, E.RightHip.p), q: logicalRotation(pose.Hips?.q ?? ID, ch.rest.Hips?.q) };
  return E;
}

export const COMPENSATION_DEFAULTS = Object.freeze({
  scale: { mode: 1, value: 100 }, massCenter: { mode: 1, value: 80 }, hipsHeight: { mode: 1, value: 0 },
  ankleProximity: { mode: 1, value: 0 }, reference: [0, 0, 0],
});

export function autoScale(srcChar, dstChar) {
  const s = srcChar.metrics, d = dstChar.metrics;
  return ((d.legLeft + d.legRight) / 2 + d.hipWidth) / ((s.legLeft + s.legRight) / 2 + s.hipWidth);
}

export function prepareEffectors(srcChar, srcPose, dstChar, comp = {}) {
  const c = { ...COMPENSATION_DEFAULTS, ...comp };
  const up = dstChar.up, ref = c.reference;
  const E = effectorsFromPose(srcChar, srcPose);
  const s = c.scale.mode === 0 ? 1 : c.scale.mode === 2 ? c.scale.value * 0.01 : autoScale(srcChar, dstChar);
  for (const k of Object.keys(E)) E[k] = { p: add(ref, mul(sub(E[k].p, ref), s)), q: E[k].q };
  const sm = srcChar.metrics, dm = dstChar.metrics;
  if (c.hipsHeight.mode !== 0 && E.LeftHip && E.RightHip && E.LeftAnkle && E.RightAnkle) {
    const h = c.hipsHeight.mode === 2 ? c.hipsHeight.value : (dm.legLeft + dm.legRight) / 2 - s * (sm.legLeft + sm.legRight) / 2;
    const u = unit(sub(add(E.LeftHip.p, E.RightHip.p), add(E.LeftAnkle.p, E.RightAnkle.p)));
    for (const k of ['Hips', 'LeftHip', 'RightHip']) if (E[k]) E[k] = { ...E[k], p: add(E[k].p, mul(u, h)) };
  }
  if (c.ankleProximity.mode !== 0) {
    const a = c.ankleProximity.mode === 2 ? c.ankleProximity.value : (dm.hipWidth - s * sm.hipWidth) * 0.5;
    for (const [side, sg] of [['Left', 1], ['Right', -1]]) {
      const ank = E[side + 'Ankle'];
      if (!ank) continue;
      let x = qrot(ank.q, [sg * a, 0, 0]);
      x = sub(x, mul(up, dot(x, up)));
      for (const k of [side + 'Ankle', side + 'Foot']) if (E[k]) E[k] = { ...E[k], p: add(E[k].p, x) };
    }
  }
  if (c.massCenter.mode !== 0 && E.Hips && E.LeftAnkle && E.RightAnkle && E.LeftHip && E.RightHip) {
    const m = c.massCenter.value * 0.01;
    const clampLeg = (hip, ank, L) => { const v = sub(hip, ank), l = len(v); return l > L ? mul(v, L / l) : v; };
    const tgt = mid(add(E.LeftAnkle.p, clampLeg(E.LeftHip.p, E.LeftAnkle.p, dm.legLeft)), add(E.RightAnkle.p, clampLeg(E.RightHip.p, E.RightAnkle.p, dm.legRight)));
    E.Hips = { ...E.Hips, p: add(E.Hips.p, mul(sub(tgt, mid(E.LeftHip.p, E.RightHip.p)), m)) };
  }
  return { effectors: E, scale: s };
}

export function restPose(ch) {
  const pose = {};
  for (const s of Object.keys(ch.slots)) pose[s] = { p: ch.rest[s].p.slice(), q: ch.rest[s].q.slice() };
  return pose;
}

function parentSlot(ch, slot) {
  const bones = ch.bones, inv = new Map(Object.entries(ch.slots).map(([s, i]) => [i, s]));
  for (let p = bones[ch.slots[slot]].parent; p !== undefined && p !== null && p >= 0; p = bones[p].parent) if (inv.has(p)) return inv.get(p);
  return null;
}

export function forwardPose(ch, rotations, rootPos) {
  const order = Object.keys(ch.slots).sort((a, b) => ancestors(ch.bones, ch.slots[a]).length - ancestors(ch.bones, ch.slots[b]).length);
  const pose = {};
  for (const s of order) {
    const par = parentSlot(ch, s);
    const q = rotations[s] ?? (par && pose[par] ? qmul(pose[par].q, qmul(qconj(ch.rest[par].q), ch.rest[s].q)) : ch.rest[s].q);
    if (s === 'Hips' && rootPos) {
      pose[s] = { p: rootPos.slice(), q };
      continue;
    }
    if (!par || !pose[par]) {
      pose[s] = { p: ch.rest[s].p.slice(), q };
      continue;
    }
    const off = qrot(qconj(ch.rest[par].q), sub(ch.rest[s].p, ch.rest[par].p));
    pose[s] = { p: add(pose[par].p, qrot(pose[par].q, off)), q };
  }
  return pose;
}

function moveSubtree(ch, pose, slot, fn) {
  const root = ch.slots[slot];
  for (const [s, i] of Object.entries(ch.slots)) {
    if (i === root || ancestors(ch.bones, i).includes(root)) pose[s] = fn(pose[s], s);
  }
}

export function kneeHinge(ch, slot) {
  return qrot(ch.rest[slot]?.q ?? ID, [1, 0, 0]);
}

export function retargetPose(srcChar, srcPose, dstChar, o = {}) {
  const rot = mapRotations(srcChar, srcPose, dstChar);
  let pose = forwardPose(dstChar, rot);
  const { effectors: E, scale } = prepareEffectors(srcChar, srcPose, dstChar, o.compensation);
  if (E.Hips && pose.LeftUpLeg && pose.RightUpLeg && pose.Hips) {
    const shift = sub(E.Hips.p, mid(pose.LeftUpLeg.p, pose.RightUpLeg.p));
    pose = forwardPose(dstChar, rot, add(pose.Hips.p, shift));
  }
  const reach = { ankleT: 1, ankleR: 1, resistKnee: 0.5, maxExt: 0.5, realistic: 0, legRoll: 0, ...(o.reach ?? {}) };
  for (const side of ['Left', 'Right']) {
    const up = side + 'UpLeg', kn = side + 'Leg', an = side + 'Foot', eff = E[side + 'Ankle'];
    if (!eff || !pose[up] || !pose[kn] || !pose[an] || reach.ankleT <= 0) continue;
    const rq = s => dstChar.rest[s].q;
    const leg = solveLeg({
      top: pose[up].p, knee: pose[kn].p, ankle: pose[an].p,
      qTop: logicalRotation(pose[up].q, rq(up)), qKnee: logicalRotation(pose[kn].q, rq(kn)), qAnkle: logicalRotation(pose[an].q, rq(an)),
      goal: eff.p, qGoal: eff.q, T: reach.ankleT, R: reach.ankleR, resistKnee: reach.resistKnee, maxExt: reach.maxExt,
      realistic: reach.realistic, legRoll: reach.legRoll, hinge: o.kneeHinge ?? kneeHinge(dstChar, kn),
    });
    const before = { up: pose[up], kn: pose[kn], an: pose[an] };
    const qUp = physicalRotation(leg.qTop, rq(up)), qKn = physicalRotation(leg.qKnee, rq(kn)), qAn = physicalRotation(leg.qAnkle, rq(an));
    const dUp = qmul(qUp, qconj(before.up.q));
    moveSubtree(dstChar, pose, up, (n, s) => ({ p: add(before.up.p, qrot(dUp, sub(n.p, before.up.p))), q: qmul(dUp, n.q) }));
    const k0 = pose[kn], dKn = qmul(qKn, qconj(k0.q));
    moveSubtree(dstChar, pose, kn, (n, s) => ({ p: add(leg.knee, qrot(dKn, sub(n.p, k0.p))), q: qmul(dKn, n.q) }));
    const a0 = pose[an], dAn = qmul(qAn, qconj(a0.q));
    moveSubtree(dstChar, pose, an, (n, s) => ({ p: add(leg.ankle, qrot(dAn, sub(n.p, a0.p))), q: qmul(dAn, n.q) }));
  }
  return { pose, effectors: E, scale, rotations: rot };
}

export function poseFromBones(ch, globals) {
  const pose = {};
  for (const [s, i] of Object.entries(ch.slots)) if (globals[i]) pose[s] = { p: globals[i].p.slice(), q: globals[i].q.slice() };
  return pose;
}
