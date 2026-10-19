import test from 'node:test';
import assert from 'node:assert/strict';
import {buildAnimatorControls,ikGoalFromOffset,modelGroundY} from '../wire/animator_rig.js';

test('humanoid hands and feet expose IK only for valid two-bone chains', () => {
  const names=['pelvis','upperarm_l','lowerarm_l','hand_l','thigh_l','calf_l','foot_l','index_01_l'];
  const bones=names.map(name=>({name})), byKey=new Map(names.map((name,i)=>[name,i])), parent=[-1,0,1,2,0,4,5,3];
  const controls=buildAnimatorControls(bones,byKey,parent);
  assert.deepEqual(controls.find(c=>c.id==='hand_l').chain,[1,2,3]);
  assert.deepEqual(controls.find(c=>c.id==='foot_l').modes,['fk','ik']);
  assert.deepEqual(controls.find(c=>c.id==='bone:7').modes,['fk']);
  parent[6]=0;
  assert.deepEqual(buildAnimatorControls(bones,byKey,parent).find(c=>c.id==='foot_l').modes,['fk']);
});

test('IK and model-plane offsets use the same character-to-centimeter scale', () => {
  assert.deepEqual(ikGoalFromOffset([10,20,30],[5,-10,0],200),[20,0,30]);
  assert.equal(modelGroundY(40,200),20);
  assert.throws(()=>ikGoalFromOffset([0,0,0],[NaN,0,0],100));
  assert.throws(()=>modelGroundY(0,0));
});
