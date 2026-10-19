import test from 'node:test';
import assert from 'node:assert/strict';
import {guidanceForFrames,preserveGuidedPoints} from '../wire/pose_guidance.js';

test('guided, planted and pinned controls receive frame-local masks',()=>{
  const controls=Array.from({length:43},(_,i)=>({bone:i}));
  const g=guidanceForFrames({frames:[1,2,3],controls,selectedBone:7,locks:[{side:'l',a:2,b:3}],pins:[{a:3,b:3}],footBones:{l:10},handBones:{l:11,r:12}});
  assert.deepEqual(g.selectedPoints,[7]);
  assert.equal(g.masks[0][10][0],0);
  assert.equal(g.masks[1][10][0],1);
  assert.equal(g.masks[2][11][0],1);
  assert.equal(g.masks[2][12][0],1);
});

test('model output cannot replace a guided point in the Fulcrum adapter',()=>{
  const a=[Array.from({length:43},()=>[1,2,3])],b=[Array.from({length:43},()=>[8,9,10])],mask=[Array.from({length:43},(_,i)=>[i===5?1:0])];
  const c=preserveGuidedPoints(a,b,mask);
  assert.deepEqual(c[0][5],[8,9,10]);
  assert.deepEqual(c[0][6],[1,2,3]);
  assert.deepEqual(a[0][5],[1,2,3]);
});
