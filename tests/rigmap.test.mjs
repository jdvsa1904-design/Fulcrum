import test from 'node:test';
import assert from 'node:assert/strict';
import { buildRigMap, renameForExport, CORE } from '../wire/rigmap.js';

const FING = ['thumb', 'index', 'middle', 'ring', 'pinky'];
const cap = s => s[0].toUpperCase() + s.slice(1);

function skeleton(nameOf, { spines = 3, pre = [], extras = [], fingers = true } = {}) {
  const bones = [];
  const add = (canon, parent, pos) => {
    const name = nameOf(canon);
    bones.push({ name, parent, pos, canon });
    return bones.length - 1;
  };
  let parent = -1;
  for (const n of pre) { bones.push({ name: n, parent, pos: [0, 100, 0], canon: null }); parent = bones.length - 1; }
  const pelvis = add('pelvis', parent, [0, 100, 0]);
  let top = pelvis;
  for (let k = 1; k <= spines; k++) top = add(`spine_0${k}`, top, [0, 100 + 10 * k, 0]);
  const neck = add('neck_01', top, [0, 150, 0]);
  add('head', neck, [0, 160, 0]);
  for (const [s, x] of [['l', 1], ['r', -1]]) {
    const cl = add(`clavicle_${s}`, top, [5 * x, 145, 0]);
    const ua = add(`upperarm_${s}`, cl, [15 * x, 145, 0]);
    const la = add(`lowerarm_${s}`, ua, [40 * x, 145, 0]);
    const hd = add(`hand_${s}`, la, [65 * x, 145, 0]);
    if (fingers) FING.forEach((f, i) => {
      let p = hd;
      for (let j = 1; j <= 3; j++) p = add(`${f}_0${j}_${s}`, p, [(68 + 3 * j) * x, 145, 4 - 2 * i]);
    });
    const th = add(`thigh_${s}`, pelvis, [10 * x, 95, 0]);
    const ca = add(`calf_${s}`, th, [10 * x, 50, 0]);
    const ft = add(`foot_${s}`, ca, [10 * x, 8, 0]);
    add(`ball_${s}`, ft, [10 * x, 0, 12]);
  }
  for (const [n, of] of extras) {
    const p = bones.findIndex(b => b.canon === of);
    bones.push({ name: n, parent: p, pos: bones[p].pos, canon: null });
  }
  return bones;
}

function split(c) {
  const m = c.match(/^(.*?)(?:_0(\d))?(?:_([lr]))?$/);
  return { base: m[1], n: m[2] ? +m[2] : 0, side: m[3] ?? null };
}

const SIDED = (table, fing, sideWord) => c => {
  const { base, n, side } = split(c);
  if (!side) return table[c];
  if (FING.includes(base)) return fing(base, n, side);
  return sideWord(side, table[base]);
};

const FAMILIES = {
  mixamo: [SIDED(
    { pelvis: 'mixamorig:Hips', spine_01: 'mixamorig:Spine', spine_02: 'mixamorig:Spine1', spine_03: 'mixamorig:Spine2', neck_01: 'mixamorig:Neck', head: 'mixamorig:Head',
      clavicle: 'Shoulder', upperarm: 'Arm', lowerarm: 'ForeArm', hand: 'Hand', thigh: 'UpLeg', calf: 'Leg', foot: 'Foot', ball: 'ToeBase' },
    (f, n, s) => `mixamorig:${s === 'l' ? 'Left' : 'Right'}Hand${cap(f)}${n}`,
    (s, b) => `mixamorig:${s === 'l' ? 'Left' : 'Right'}${b}`)],
  vrm: [SIDED(
    { pelvis: 'J_Bip_C_Hips', spine_01: 'J_Bip_C_Spine', spine_02: 'J_Bip_C_Chest', spine_03: 'J_Bip_C_UpperChest', neck_01: 'J_Bip_C_Neck', head: 'J_Bip_C_Head',
      clavicle: 'Shoulder', upperarm: 'UpperArm', lowerarm: 'LowerArm', hand: 'Hand', thigh: 'UpperLeg', calf: 'LowerLeg', foot: 'Foot', ball: 'ToeBase' },
    (f, n, s) => `J_Bip_${s.toUpperCase()}_${f === 'pinky' ? 'Little' : cap(f)}${n}`,
    (s, b) => `J_Bip_${s.toUpperCase()}_${b}`), { extras: [['J_Sec_Hair1_01', 'head']] }],
  unity: [SIDED(
    { pelvis: 'Hips', spine_01: 'Spine', spine_02: 'Chest', spine_03: 'UpperChest', neck_01: 'Neck', head: 'Head',
      clavicle: 'Shoulder', upperarm: 'UpperArm', lowerarm: 'LowerArm', hand: 'Hand', thigh: 'UpperLeg', calf: 'LowerLeg', foot: 'Foot', ball: 'Toes' },
    (f, n, s) => `${s === 'l' ? 'Left' : 'Right'} ${f === 'pinky' ? 'Little' : cap(f)} ${['Proximal', 'Intermediate', 'Distal'][n - 1]}`,
    (s, b) => `${s === 'l' ? 'Left' : 'Right'}${b}`)],
  rigify: [SIDED(
    { pelvis: 'DEF-spine', spine_01: 'DEF-spine.001', spine_02: 'DEF-spine.002', spine_03: 'DEF-spine.003', neck_01: 'DEF-spine.004', head: 'DEF-spine.006',
      clavicle: 'shoulder', upperarm: 'upper_arm', lowerarm: 'forearm', hand: 'hand', thigh: 'thigh', calf: 'shin', foot: 'foot', ball: 'toe' },
    (f, n, s) => `DEF-${f === 'thumb' ? '' : 'f_'}${f}.0${n}.${s.toUpperCase()}`,
    (s, b) => `DEF-${b}.${s.toUpperCase()}`), { extras: [['ORG-thigh.L', 'thigh_l'], ['MCH-foot_ik.L', 'foot_l'], ['DEF-pelvis.L', 'pelvis']] }],
  blender: [SIDED(
    { pelvis: 'hips', spine_01: 'spine', spine_02: 'chest', spine_03: 'upper_chest', neck_01: 'neck', head: 'head',
      clavicle: 'shoulder', upperarm: 'upper_arm', lowerarm: 'forearm', hand: 'hand', thigh: 'thigh', calf: 'shin', foot: 'foot', ball: 'toe' },
    (f, n, s) => `${f}.0${n}.${s.toUpperCase()}`,
    (s, b) => `${b}.${s.toUpperCase()}`)],
  cc: [SIDED(
    { pelvis: 'CC_Base_Pelvis', spine_01: 'CC_Base_Waist', spine_02: 'CC_Base_Spine01', spine_03: 'CC_Base_Spine02', neck_01: 'CC_Base_NeckTwist01', head: 'CC_Base_Head',
      clavicle: 'Clavicle', upperarm: 'Upperarm', lowerarm: 'Forearm', hand: 'Hand', thigh: 'Thigh', calf: 'Calf', foot: 'Foot', ball: 'ToeBase' },
    (f, n, s) => `CC_Base_${s.toUpperCase()}_${{ middle: 'Mid' }[f] ?? cap(f)}${n}`,
    (s, b) => `CC_Base_${s.toUpperCase()}_${b}`), { pre: ['CC_Base_BoneRoot', 'CC_Base_Hip'], extras: [['CC_Base_L_UpperarmTwist01', 'upperarm_l'], ['CC_Base_L_CalfTwist01', 'calf_l']] }],
  manny: [c => c, { spines: 5, pre: ['root'], extras: [['upperarm_twist_01_l', 'upperarm_l'], ['calf_twist_01_r', 'calf_r'], ['ik_foot_l', 'foot_l'], ['weapon_r', 'hand_r'], ['index_metacarpal_l', 'hand_l']] }],
  biped: [SIDED(
    { pelvis: 'Bip01 Pelvis', spine_01: 'Bip01 Spine', spine_02: 'Bip01 Spine1', spine_03: 'Bip01 Spine2', neck_01: 'Bip01 Neck', head: 'Bip01 Head',
      clavicle: 'Clavicle', upperarm: 'UpperArm', lowerarm: 'Forearm', hand: 'Hand', thigh: 'Thigh', calf: 'Calf', foot: 'Foot', ball: 'Toe0' },
    (f, n, s) => `Bip01 ${s.toUpperCase()} Finger${FING.indexOf(f)}${n === 1 ? '' : n - 1}`,
    (s, b) => `Bip01 ${s.toUpperCase()} ${b}`), { pre: ['Bip01'], extras: [['Bip01 HeadNub', 'head'], ['Bip01 L Toe0Nub', 'ball_l']] }],
  hik: [SIDED(
    { pelvis: 'Character1_Hips', spine_01: 'Character1_Spine', spine_02: 'Character1_Spine1', spine_03: 'Character1_Spine2', neck_01: 'Character1_Neck', head: 'Character1_Head',
      clavicle: 'Shoulder', upperarm: 'Arm', lowerarm: 'ForeArm', hand: 'Hand', thigh: 'UpLeg', calf: 'Leg', foot: 'Foot', ball: 'ToeBase' },
    (f, n, s) => `Character1_${s === 'l' ? 'Left' : 'Right'}Hand${cap(f)}${n}`,
    (s, b) => `Character1_${s === 'l' ? 'Left' : 'Right'}${b}`), { extras: [['Character1_LeftHandIndex4', 'index_03_l']] }],
  rokoko: [SIDED(
    { pelvis: 'Hips', spine_01: 'Spine1', spine_02: 'Spine2', spine_03: 'Spine3', spine_04: 'Spine4', neck_01: 'Neck', head: 'Head',
      clavicle: 'Shoulder', upperarm: 'Arm', lowerarm: 'ForeArm', hand: 'Hand', thigh: 'Thigh', calf: 'Shin', foot: 'Foot', ball: 'Toe' },
    (f, n, s) => `${s === 'l' ? 'Left' : 'Right'}${f === 'pinky' ? 'Little' : cap(f)}${['Proximal', 'Intermediate', 'Distal'][n - 1]}`,
    (s, b) => `${s === 'l' ? 'Left' : 'Right'}${b}`), { spines: 4 }],
};

for (const [fam, [nameOf, opts]] of Object.entries(FAMILIES)) {
  test(`${fam}: every bone maps to its canonical name with full confidence`, () => {
    const bones = skeleton(nameOf, opts);
    const map = buildRigMap(bones);
    for (const b of bones) {
      if (b.canon) assert.equal(map.toCanonical.get(b.name), b.canon, `${fam} ${b.name}`);
      else if (b.name !== 'weapon_r') assert.equal(map.toCanonical.has(b.name), false, `${fam} extra ${b.name} must stay unmapped`);
    }
    assert.deepEqual(map.missing, []);
    assert.equal(map.confidence, 1);
  });

  test(`${fam}: round trip through canonical names restores the source names`, () => {
    const bones = skeleton(nameOf, opts);
    const map = buildRigMap(bones);
    for (const [src, canon] of map.toCanonical) {
      assert.equal(renameForExport(canon, map), src);
      assert.equal(map.toCanonical.get(renameForExport(canon, map)), canon);
    }
    assert.equal(renameForExport('not_a_bone', map), 'not_a_bone');
  });
}

test('manny weapon_r maps to itself', () => {
  const map = buildRigMap(skeleton(...FAMILIES.manny));
  assert.equal(map.toCanonical.get('weapon_r'), 'weapon_r');
});

test('renameForExport accepts namespaced canonical names', () => {
  const map = buildRigMap(skeleton(...FAMILIES.mixamo));
  assert.equal(renameForExport('Body:thigh_l', map), 'mixamorig:LeftUpLeg');
});

test('opaque names fall back to hierarchy and geometry', () => {
  let n = 0;
  const bones = skeleton(() => `b${n++}`, { pre: ['Armature'] });
  const map = buildRigMap(bones);
  for (const b of bones) if (b.canon) assert.equal(map.toCanonical.get(b.name), b.canon, b.name);
  assert.deepEqual(map.missing, []);
  assert.ok(map.confidence > 0.55 && map.confidence < 0.65, String(map.confidence));
  for (const [src, canon] of map.toCanonical) assert.equal(renameForExport(canon, map), src);
});

test('opaque names with mirrored side flip by rest-pose x sign', () => {
  let n = 0;
  const bones = skeleton(() => `j${n++}`, { fingers: false });
  const map = buildRigMap(bones);
  const thighL = bones.find(b => b.canon === 'thigh_l');
  assert.ok(thighL.pos[0] > 0);
  assert.equal(map.toCanonical.get(thighL.name), 'thigh_l');
  assert.equal(map.toCanonical.get(bones.find(b => b.canon === 'hand_r').name), 'hand_r');
});

test('partial names keep named slots and fill gaps from geometry', () => {
  const bones = skeleton(c => (c.startsWith('thigh') || c.startsWith('calf') ? `x_${c.length}${c.at(-1)}q` : c));
  const map = buildRigMap(bones);
  assert.equal(map.toCanonical.get('pelvis'), 'pelvis');
  for (const b of bones) if (b.canon) assert.equal(map.toCanonical.get(b.name), b.canon, b.name);
  assert.ok(map.confidence < 1 && map.confidence > 0.9);
});

test('a bare chain reports missing core slots and low confidence', () => {
  const bones = [{ name: 'a', parent: -1, pos: [0, 0, 0] }, { name: 'b', parent: 0, pos: [0, 1, 0] }];
  const map = buildRigMap(bones);
  assert.equal(map.missing.length, CORE.length + 2);
  assert.equal(map.confidence, 0);
});
