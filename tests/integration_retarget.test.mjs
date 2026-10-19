import test from 'node:test';
import assert from 'node:assert/strict';
import { json, v3, q4, deg, rng } from './helpers.mjs';
import { slotName, characterizeRig, hikRetargetSetup, hikRetargetFrame, dstWorldRotations, bodyFrame, alignmentQuat } from '../wire/hikretarget.js';

const FX = json(new URL('./fixtures/checkpoint_v10_sample.json', import.meta.url));

function decompose(m) {
  const c = k => [m[k], m[k + 1], m[k + 2]], sx = v3.len(c(0)), sy = v3.len(c(4)), sz = v3.len(c(8));
  const r = [[m[0] / sx, m[4] / sy, m[8] / sz], [m[1] / sx, m[5] / sy, m[9] / sz], [m[2] / sx, m[6] / sy, m[10] / sz]];
  const t = r[0][0] + r[1][1] + r[2][2];
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, s / 4]; }
  else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) { const s = Math.sqrt(1 + r[0][0] - r[1][1] - r[2][2]) * 2; q = [s / 4, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s, (r[2][1] - r[1][2]) / s]; }
  else if (r[1][1] > r[2][2]) { const s = Math.sqrt(1 + r[1][1] - r[0][0] - r[2][2]) * 2; q = [(r[0][1] + r[1][0]) / s, s / 4, (r[1][2] + r[2][1]) / s, (r[0][2] - r[2][0]) / s]; }
  else { const s = Math.sqrt(1 + r[2][2] - r[0][0] - r[1][1]) * 2; q = [(r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, s / 4, (r[1][0] - r[0][1]) / s]; }
  const l = Math.hypot(...q);
  return { p: [m[12], m[13], m[14]], q: q.map(x => x / l) };
}

const NB = FX.bones.length;
const restG = FX.rest.map(decompose);
const frameG = f => Array.from({ length: NB }, (_, i) => decompose(FX.frames[f].slice(i * 16, i * 16 + 16)));
const dstBones = FX.bones.map((name, i) => ({ name, parent: FX.parents[i], p: restG[i].p, q: restG[i].q }));

const MIXAMO = {
  pelvis: 'Hips', spine_01: 'Spine', spine_02: 'Spine1', spine_03: 'Spine2', spine_04: 'Spine3', spine_05: 'Spine4', neck_01: 'Neck', neck_02: 'Neck1', head: 'Head',
  thigh_l: 'LeftUpLeg', calf_l: 'LeftLeg', foot_l: 'LeftFoot', ball_l: 'LeftToeBase', thigh_r: 'RightUpLeg', calf_r: 'RightLeg', foot_r: 'RightFoot', ball_r: 'RightToeBase',
  clavicle_l: 'LeftShoulder', upperarm_l: 'LeftArm', lowerarm_l: 'LeftForeArm', hand_l: 'LeftHand', clavicle_r: 'RightShoulder', upperarm_r: 'RightArm', lowerarm_r: 'RightForeArm', hand_r: 'RightHand',
};

function mixamoSource(seed) {
  const r = rng(seed), G = q4.axis([1, 0, 0], -Math.PI / 2), s = 0.01;
  const off = FX.bones.map(() => q4.axis([r() - 0.5, r() - 0.5, r() - 0.5], (r() - 0.5) * 2.4));
  const keep = FX.bones.map(n => n in MIXAMO || n === 'root');
  const map = g => g.map((x, i) => keep[i] ? { p: v3.mul(q4.rot(G, x.p), s), q: q4.mul(q4.mul(G, x.q), off[i]) } : null);
  const bones = FX.bones.map((n, i) => ({ name: n in MIXAMO ? 'mixamorig:' + MIXAMO[n] : n, parent: FX.parents[i] })).map((b, i) => ({ ...b, ...map(restG)[i] }));
  const sel = bones.map((b, i) => i).filter(i => keep[i]);
  const remap = new Map(sel.map((i, k) => [i, k]));
  const parentOf = i => { for (let p = FX.parents[i]; p >= 0; p = FX.parents[p]) if (remap.has(p)) return remap.get(p); return -1; };
  const srcBones = sel.map(i => ({ ...bones[i], parent: parentOf(i) }));
  return { srcBones, frame: f => { const g = map(frameG(f)); return sel.map(i => g[i]); } };
}

const slotsByName = ch => Object.fromEntries(Object.entries(ch.slots).map(([s, i]) => [s, ch.bones[i].name]));

test('characterizeRig maps the Checkpoint body onto HIK slots without its ik_ and twist helper bones', () => {
  const ch = characterizeRig(dstBones);
  const m = slotsByName(ch);
  assert.equal(ch.valid, true, 'missing ' + ch.missing);
  assert.deepEqual([m.Hips, m.LeftUpLeg, m.LeftLeg, m.LeftFoot, m.LeftToeBase, m.RightFoot, m.LeftHand, m.RightHand, m.Head], ['pelvis', 'thigh_l', 'calf_l', 'foot_l', 'ball_l', 'foot_r', 'hand_l', 'hand_r', 'head']);
  assert.equal(m.Spine, 'spine_01');
  assert.equal(m.Spine4, 'spine_05');
});

test('bodyFrame and alignmentQuat undo a Z-up source orientation', () => {
  const { srcBones } = mixamoSource(3);
  const a = bodyFrame(characterizeRig(srcBones)), b = bodyFrame(characterizeRig(dstBones));
  const G = alignmentQuat(a, b);
  assert.ok(deg(Math.acos(Math.min(1, v3.dot(q4.rot(G, a.up), b.up)))) < 1e-4);
  assert.ok(deg(Math.acos(Math.min(1, v3.dot(q4.rot(G, a.left), b.left)))) < 1e-4);
});

test('HIK retarget of a renamed, Z-up, metre-scaled, randomly re-oriented copy of the v10 take reproduces v10 on the Checkpoint body', () => {
  const { srcBones, frame } = mixamoSource(11);
  const setup = hikRetargetSetup(srcBones, dstBones);
  assert.equal(setup.valid, true, setup.missing.join(', '));
  const keys = ['pelvis', 'thigh_l', 'calf_l', 'foot_l', 'ball_l', 'thigh_r', 'calf_r', 'foot_r', 'ball_r', 'spine_03', 'head', 'hand_l', 'hand_r', 'lowerarm_r'];
  let worstP = 0, worstQ = 0;
  for (let f = 0; f < FX.frames.length; f++) {
    const res = hikRetargetFrame(setup, frame(f));
    const W = dstWorldRotations(setup, res, dstBones), truth = frameG(f);
    const pel = FX.bones.indexOf('pelvis');
    for (const k of keys) {
      const i = FX.bones.indexOf(k), slot = Object.entries(setup.dst.slots).find(([, j]) => j === i)[0];
      const rel = v3.sub(res.pose[slot].p, res.pose.Hips.p), relT = v3.sub(truth[i].p, truth[pel].p);
      worstP = Math.max(worstP, v3.dist(rel, relT));
      worstQ = Math.max(worstQ, deg(q4.angle(W.q[i], truth[i].q)));
    }
  }
  assert.ok(worstP < 0.05, `worst joint offset ${worstP.toFixed(4)} units`);
  assert.ok(worstQ < 0.05, `worst rotation ${worstQ.toFixed(4)} deg`);
});

test('dstWorldRotations carries unmapped Checkpoint bones (twist, ik, fingers) rigidly with their parents', () => {
  const { srcBones, frame } = mixamoSource(5);
  const setup = hikRetargetSetup(srcBones, dstBones);
  const res = hikRetargetFrame(setup, frame(4));
  const W = dstWorldRotations(setup, res, dstBones);
  const i = FX.bones.indexOf('thigh_twist_01_l'), p = FX.parents[i];
  const local = q4.mul(q4.conj(W.q[p]), W.q[i]), restLocal = q4.mul(q4.conj(restG[p].q), restG[i].q);
  assert.ok(deg(q4.angle(local, restLocal)) < 1e-6);
  assert.equal(W.mapped[i], 0);
  assert.equal(W.mapped[FX.bones.indexOf('calf_l')], 1);
});

test('slotName reads three.js-sanitised namespaces (mixamorigLeftUpLeg) and drops helper bones', () => {
  assert.equal(slotName('mixamorigLeftUpLeg'), 'LeftUpLeg');
  assert.equal(slotName('mixamorig:Hips'), 'mixamorig:Hips');
  assert.equal(slotName('Character1LeftHand'), 'LeftHand');
  assert.equal(slotName('ik_foot_l'), '');
  assert.equal(slotName('thigh_twist_01_l'), '');
  assert.equal(slotName('thigh_l'), 'thigh_l');
  const { srcBones } = mixamoSource(2);
  const ch = characterizeRig(srcBones.map(b => ({ ...b, name: b.name.replace(':', '') })));
  assert.equal(ch.valid, true, ch.missing.join(','));
});
