import test from 'node:test';
import assert from 'node:assert/strict';
import { json, exists, near, nearVec, v3, q4, deg, rng } from './helpers.mjs';
import {
  classifyName, characterize, mapRotations, prepareEffectors, retargetPose, autoScale, forwardPose, restPose,
  logicalRotation, physicalRotation, REQUIRED_SLOTS,
} from '../re/hik/retarget.js';

const DIR = 'C:/re_work/hik/results/';
const skip = f => exists(DIR + f) ? false : `missing ${f}`;
const qErr = (a, b) => deg(q4.angle(a, b));

const H2M = {
  Reference: 'Reference', Hips: 'Hips', LeftHip: 'LeftUpLeg', LeftKnee: 'LeftLeg', LeftAnkle: 'LeftFoot', LeftFoot: 'LeftToeBase',
  RightHip: 'RightUpLeg', RightKnee: 'RightLeg', RightAnkle: 'RightFoot', RightFoot: 'RightToeBase', Waist: 'Spine', Spine1: 'Spine1',
  Neck: 'Neck', Head: 'Head', LeftCollar: 'LeftShoulder', LeftShoulder: 'LeftArm', LeftElbow: 'LeftForeArm', LeftWrist: 'LeftHand',
  RightCollar: 'RightShoulder', RightShoulder: 'RightArm', RightElbow: 'RightForeArm', RightWrist: 'RightHand',
};
const PAR = {
  Hips: 'Reference', LeftHip: 'Hips', LeftKnee: 'LeftHip', LeftAnkle: 'LeftKnee', LeftFoot: 'LeftAnkle', RightHip: 'Hips', RightKnee: 'RightHip',
  RightAnkle: 'RightKnee', RightFoot: 'RightAnkle', Waist: 'Hips', Spine1: 'Waist', Neck: 'Spine1', Head: 'Neck', LeftCollar: 'Spine1',
  LeftShoulder: 'LeftCollar', LeftElbow: 'LeftShoulder', LeftWrist: 'LeftElbow', RightCollar: 'Spine1', RightShoulder: 'RightCollar',
  RightElbow: 'RightShoulder', RightWrist: 'RightElbow',
};
const NAMES = Object.keys(H2M);
const TPOSE = {
  Reference: [0, 0, 0], Hips: [0, 100, 0], LeftHip: [10, 95, 0], LeftKnee: [10, 52, 0], LeftAnkle: [10, 8, 0], LeftFoot: [10, 2, 13],
  RightHip: [-10, 95, 0], RightKnee: [-10, 52, 0], RightAnkle: [-10, 8, 0], RightFoot: [-10, 2, 13], Waist: [0, 110, 0], Spine1: [0, 125, 0],
  Neck: [0, 150, 0], Head: [0, 160, 0], LeftCollar: [3, 145, 0], LeftShoulder: [15, 145, 0], LeftElbow: [43, 145, 0], LeftWrist: [68, 145, 0],
  RightCollar: [-3, 145, 0], RightShoulder: [-15, 145, 0], RightElbow: [-43, 145, 0], RightWrist: [-68, 145, 0],
};
const bonesFrom = (pos, rot = {}) => NAMES.map(n => ({ name: H2M[n], parent: PAR[n] ? NAMES.indexOf(PAR[n]) : -1, p: pos[n].slice(), q: (rot[n] ?? [0, 0, 0, 1]).slice() }));
const poseFrom = src => Object.fromEntries(Object.entries(src).map(([n, v]) => [H2M[n], { p: v[0], q: v[1] }]));
const scaled = s => Object.fromEntries(NAMES.map(n => [n, TPOSE[n].map((x, i) => x * s[i])]));

test('classifyName: HIK, Mixamo, Unreal, Biped, CMU and Rigify names land on HIK slots', () => {
  const cases = {
    'mixamorig:LeftUpLeg': 'LeftUpLeg', 'thigh_l': 'LeftUpLeg', 'calf_r': 'RightLeg', 'foot_l': 'LeftFoot', 'ball_r': 'RightToeBase',
    'pelvis': 'Hips', 'clavicle_l': 'LeftShoulder', 'upperarm_r': 'RightArm', 'lowerarm_l': 'LeftForeArm', 'hand_r': 'RightHand', 'head': 'Head',
    'Bip01 L Thigh': 'LeftUpLeg', 'Bip01 R Calf': 'RightLeg', 'Bip01 L Clavicle': 'LeftShoulder', 'Bip01 R Forearm': 'RightForeArm',
    'LeftForeArm': 'LeftForeArm', 'RightToeBase': 'RightToeBase', 'DEF-thigh.L': 'LeftUpLeg', 'DEF-shin.R': 'RightLeg', 'upper_arm.L': 'LeftArm',
    'forearm.R': 'RightForeArm', 'shoulder.L': 'LeftShoulder', 'Hips': 'Hips', 'Character1_LeftHand': 'LeftHand',
  };
  for (const [n, s] of Object.entries(cases)) assert.equal(classifyName(n)?.slot, s, n);
  assert.equal(classifyName('spine_03').slot, 'Spine');
  assert.equal(classifyName('spine_03').index, 3);
  assert.equal(classifyName('neck_01').slot, 'Neck');
  assert.equal(classifyName('ik_foot_root'), null);
  assert.equal(classifyName('weapon_r'), null);
});

test('characterize: Unreal mannequin hierarchy maps every required slot and orders the spine', () => {
  const B = (name, parent, p) => ({ name, parent, p, q: [0, 0, 0, 1] });
  const bones = [
    B('root', -1, [0, 0, 0]), B('pelvis', 0, [0, 100, 0]), B('spine_01', 1, [0, 108, 0]), B('spine_02', 2, [0, 118, 0]), B('spine_03', 3, [0, 128, 0]),
    B('spine_04', 4, [0, 136, 0]), B('spine_05', 5, [0, 142, 0]), B('neck_01', 6, [0, 150, 0]), B('neck_02', 7, [0, 154, 0]), B('head', 8, [0, 160, 0]),
    B('clavicle_l', 6, [3, 145, 0]), B('upperarm_l', 10, [15, 145, 0]), B('lowerarm_l', 11, [43, 145, 0]), B('hand_l', 12, [68, 145, 0]),
    B('clavicle_r', 6, [-3, 145, 0]), B('upperarm_r', 14, [-15, 145, 0]), B('lowerarm_r', 15, [-43, 145, 0]), B('hand_r', 16, [-68, 145, 0]),
    B('thigh_l', 1, [10, 95, 0]), B('calf_l', 18, [10, 52, 0]), B('foot_l', 19, [10, 8, 0]), B('ball_l', 20, [10, 2, 13]),
    B('thigh_r', 1, [-10, 95, 0]), B('calf_r', 22, [-10, 52, 0]), B('foot_r', 23, [-10, 8, 0]), B('ball_r', 24, [-10, 2, 13]),
    B('ik_foot_root', 0, [0, 0, 0]), B('ik_foot_l', 26, [10, 8, 0]),
  ];
  const ch = characterize(bones);
  assert.ok(ch.valid, ch.missing.join());
  assert.equal(bones[ch.slots.Spine].name, 'spine_01');
  assert.equal(bones[ch.slots.Spine4].name, 'spine_05');
  assert.equal(bones[ch.slots.Neck].name, 'neck_01');
  assert.equal(bones[ch.slots.Neck1].name, 'neck_02');
  assert.equal(bones[ch.slots.LeftToeBase].name, 'ball_l');
  near(ch.metrics.legLeft, 87, 1e-9);
  near(ch.metrics.hipWidth, 20, 1e-9);
});

test('characterize: unnamed legs are found from topology (two descending chains, side from the left axis)', () => {
  const bones = bonesFrom(TPOSE).map(b => /Leg|Foot|ToeBase/.test(b.name) ? { ...b, name: `joint${Math.random().toString(36).slice(2, 7)}` } : b);
  const ch = characterize(bones);
  assert.ok(ch.valid, ch.missing.join());
  nearVec(bones[ch.slots.LeftUpLeg].p, [10, 95, 0], 0);
  nearVec(bones[ch.slots.RightFoot].p, [-10, 8, 0], 0);
  nearVec(bones[ch.slots.LeftToeBase].p, [10, 2, 13], 0);
});

test('characterize reports missing required slots', () => {
  const ch = characterize([{ name: 'pelvis', parent: -1, p: [0, 1, 0] }]);
  assert.equal(ch.valid, false);
  assert.ok(ch.missing.includes('LeftUpLeg') && ch.missing.includes('Head'));
  assert.equal(REQUIRED_SLOTS.length, 15);
});

for (const f of ['prepare_default.json', 'prepare_high.json']) {
  test(`${f}: effector preparation (scale, hips height, ankle proximity, mass center) matches HumanIK to 0.002 cm`, { skip: skip(f) }, () => {
    const d = json(DIR + f);
    const src = characterize(bonesFrom(TPOSE));
    const dsts = d.dstScales.map(s => characterize(bonesFrom(scaled(s))));
    let worst = 0, n = 0;
    for (const r of d.rows) {
      const { effectors } = prepareEffectors(src, poseFrom(r.src), dsts[r.dst]);
      for (const [e, v] of Object.entries(r.eff)) {
        if (!effectors[e]) continue;
        worst = Math.max(worst, v3.dist(effectors[e].p, v[0]));
        n++;
      }
    }
    console.log(`${f}: ${n} effectors, worst ${worst.toFixed(5)} cm`);
    assert.ok(n >= 1500);
    assert.ok(worst <= 0.002, `${worst}`);
  });
}

test('autoScale: (mean dst leg + dst hip width) / (mean src leg + src hip width)', () => {
  const src = characterize(bonesFrom(TPOSE)), dst = characterize(bonesFrom(scaled([1.1, 1.3, 1.0])));
  near(autoScale(src, dst), (113.1 + 22) / 107, 1e-9);
  near(autoScale(src, characterize(bonesFrom(scaled([1.3, 1.3, 1.3])))), 1.3, 1e-12);
});

test('retarget_rot.json: FK rotation map q_dst = q_src * inv(q_srcRest) * q_dstRest matches HumanIK (<= 0.07 deg)', { skip: skip('retarget_rot.json') }, () => {
  const d = json(DIR + 'retarget_rot.json');
  const src = characterize(bonesFrom(TPOSE));
  const dpos = Object.fromEntries(NAMES.map(n => [n, d.dstT[n][0]])), drot = Object.fromEntries(NAMES.map(n => [n, d.dstT[n][1]]));
  const dst = characterize(bonesFrom(dpos, drot));
  let worst = 0;
  for (const r of d.rows) {
    const rot = mapRotations(src, poseFrom(r.src), dst);
    for (const n of Object.keys(r.fk)) if (n !== 'Reference' && H2M[n]) worst = Math.max(worst, qErr(rot[H2M[n]], r.fk[n][1]));
  }
  console.log(`FK map worst ${worst.toFixed(4)} deg`);
  assert.ok(worst <= 0.07, `${worst}`);
});

function fullRetarget(file, rotated) {
  const d = json(DIR + file);
  const src = characterize(bonesFrom(TPOSE));
  const dst = rotated
    ? characterize(bonesFrom(Object.fromEntries(NAMES.map(n => [n, d.dstT[n][0]])), Object.fromEntries(NAMES.map(n => [n, d.dstT[n][1]]))))
    : characterize(bonesFrom(d.dstT));
  const m = { hips: 0, ankle: 0, knee: 0, all: 0, rot: 0 };
  for (const r of d.rows) {
    const { pose } = retargetPose(src, poseFrom(r.src), dst);
    m.hips = Math.max(m.hips, v3.dist(v3.mul(v3.add(pose.LeftUpLeg.p, pose.RightUpLeg.p), 0.5), v3.mul(v3.add(r.full.LeftHip[0], r.full.RightHip[0]), 0.5)));
    for (const s of ['Left', 'Right']) {
      m.ankle = Math.max(m.ankle, v3.dist(pose[s + 'Foot'].p, r.full[s + 'Ankle'][0]));
      m.knee = Math.max(m.knee, v3.dist(pose[s + 'Leg'].p, r.full[s + 'Knee'][0]));
    }
    for (const n of NAMES) if (n !== 'Reference') {
      m.all = Math.max(m.all, v3.dist(pose[H2M[n]].p, r.full[n][0]));
      m.rot = Math.max(m.rot, qErr(pose[H2M[n]].q, r.full[n][1]));
    }
  }
  console.log(`${file}: hips ${m.hips.toFixed(3)}, ankle ${m.ankle.toFixed(3)}, knee ${m.knee.toFixed(3)}, any joint ${m.all.toFixed(3)} cm, any rotation ${m.rot.toFixed(3)} deg over ${d.rows.length}`);
  return m;
}

test('retarget_rot.json: full retarget onto a character with rotated rest frames matches HumanIK (<= 1 cm, <= 1.5 deg)', { skip: skip('retarget_rot.json') }, () => {
  const m = fullRetarget('retarget_rot.json', true);
  assert.ok(m.hips <= 1 && m.ankle <= 0.5 && m.knee <= 1 && m.all <= 1, JSON.stringify(m));
  assert.ok(m.rot <= 1.5, `${m.rot}`);
});

test('retarget_hinge.json: full retarget onto a non-uniformly scaled character matches HumanIK (<= 1 cm, <= 1.5 deg)', { skip: skip('retarget_hinge.json') }, () => {
  const m = fullRetarget('retarget_hinge.json', false);
  assert.ok(m.hips <= 1 && m.ankle <= 0.5 && m.knee <= 1 && m.all <= 1, JSON.stringify(m));
  assert.ok(m.rot <= 1.5, `${m.rot}`);
});

test('retargetPose to an identical character returns the source pose', () => {
  const ch = characterize(bonesFrom(TPOSE));
  const r = rng(9);
  const rot = {};
  for (const s of Object.keys(ch.slots)) if (s !== 'Reference') rot[s] = q4.axis([r() - 0.5, r() - 0.5, r() - 0.5], 0.4 * r());
  const src = forwardPose(ch, rot, [2, 96, 3]);
  const { pose, scale } = retargetPose(ch, src, ch, { reach: { resistKnee: 0 } });
  near(scale, 1, 1e-12);
  for (const s of Object.keys(src)) {
    nearVec(pose[s].p, src[s].p, 2e-3, s);
    assert.ok(qErr(pose[s].q, src[s].q) < 0.05, s);
  }
});

test('logical and physical rotations are inverse through the rest rotation', () => {
  const rest = q4.axis([0.3, 1, 0.2], 1.1), q = q4.axis([1, 0, 0.4], 0.7);
  nearVec(physicalRotation(logicalRotation(q, rest), rest), q, 1e-12);
  const ch = characterize(bonesFrom(TPOSE));
  const p = restPose(ch);
  nearVec(p.LeftHand.p, [68, 145, 0], 0);
});
