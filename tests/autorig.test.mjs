import test from 'node:test';
import assert from 'node:assert/strict';
import { planHumanoidRig, skinWeightsForPoint } from '../wire/autorig.js';

const bounds = { min: [-0.8, 0, -0.2], max: [0.8, 1.8, 0.3] };
test('starter auto-rig has named symmetric humanoid chains', () => {
  const rig = planHumanoidRig(bounds);
  const byName = new Map(rig.bones.map((b, i) => [b.name, i]));
  for (const name of ['root', 'pelvis', 'spine_01', 'neck_01', 'head', 'upperarm_l', 'lowerarm_l', 'hand_l', 'upperarm_r', 'thigh_l', 'calf_l', 'foot_l', 'ball_r']) assert.ok(byName.has(name));
  assert.equal(rig.bones[byName.get('hand_l')].parent, byName.get('lowerarm_l'));
  assert.equal(rig.bones[byName.get('foot_r')].parent, byName.get('calf_r'));
  assert.equal(rig.bones[byName.get('hand_l')].pos[0], -rig.bones[byName.get('hand_r')].pos[0]);
});
test('skin weights are finite, normalized, and choose the matching limb', () => {
  const rig = planHumanoidRig(bounds);
  const left = skinWeightsForPoint([-.7, 1.26, 0], rig);
  assert.ok(left.indices.some(i => rig.bones[i].name.includes('arm_l')));
  for (const point of [[0, .9, 0], [-.18, .3, 0], [.2, .1, .1], [0, 1.65, 0]]) {
    const result = skinWeightsForPoint(point, rig);
    assert.ok(result.weights.every(Number.isFinite));
    assert.ok(Math.abs(result.weights.reduce((a, b) => a + b, 0) - 1) < 1e-10);
    assert.ok(result.indices.every(i => i >= 0 && i < rig.bones.length));
  }
});
test('non-humanoid bounds are rejected instead of silently skinned', () => {
  assert.throws(() => planHumanoidRig({ min: [0, 0, 0], max: [10, 1, 1] }), /upright/);
  assert.throws(() => planHumanoidRig({ min: [0, 0, 0], max: [0, 1, 0] }), /upright/);
});
