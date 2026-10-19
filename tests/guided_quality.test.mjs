import test from 'node:test';
import assert from 'node:assert/strict';
import {suggestPoses} from '../poseassist.js';
import {applyPoseGuidedInbetween} from '../advanced.js';
import {inspectMotion,supportHull,supportMargin} from '../motionquality.js';

const pose=(a)=>[Math.sin(a/2),0,0,Math.cos(a/2)];
test('motion prior infers unedited joints from correlated training poses and preserves guidance',()=>{
  const n=60,nb=2,source=new Float32Array(n*nb*4);
  for(let f=0;f<n;f++){source.set(pose(f/60),f*8);source.set(pose(f/30),f*8+4);}
  const current=source.slice();current.set(pose(.75),0);
  const r=suggestPoses({source,current,n,nb,f:0,guides:[0],context:0,count:3});
  assert.equal(r.suggestions.length,3);assert.ok(r.suggestions[0].sourceFrame>=43&&r.suggestions[0].sourceFrame<=47);
  assert.deepEqual([...r.suggestions[0].pose.slice(0,4)],[...current.slice(0,4)]);
  assert.ok(r.suggestions[0].pose[4]>.6);assert.ok(r.suggestions[0].confidence>.9);
});
test('unguided motion prior explains ambiguity without modifying source',()=>{
  const source=Float32Array.from([0,0,0,1]);const r=suggestPoses({source,current:source,nb:1,n:1,f:0});assert.equal(r.suggestions.length,0);assert.match(r.reason,/control/);
});
test('sparse interpolation preserves anchors, root path and child translations exactly',()=>{
  const n=11,nb=2,q=new Float32Array(n*nb*4),p=new Float32Array(n*nb*3);
  for(let f=0;f<n;f++){q.set(pose(Math.sin(f)),f*8);q.set(pose(Math.cos(f)),f*8+4);p.set([f,f*f,0,1,2,3],f*6);}
  const before=q.slice(),translations=p.slice();applyPoseGuidedInbetween({q,p,n,nb,names:['pelvis','hand'],a:2,b:8,keys:[5]});
  for(const f of [0,1,2,5,8,9,10])assert.deepEqual([...q.slice(f*8,(f+1)*8)],[...before.slice(f*8,(f+1)*8)]);
  assert.deepEqual(p,translations);assert.notDeepEqual(q,before);
});
test('support hull reports signed balance margin',()=>{
  const h=supportHull([[-1,0,-1],[1,0,-1],[1,0,1],[-1,0,1],[0,0,0]]);assert.equal(h.length,4);assert.equal(supportMargin([0,1,0],h),1);assert.equal(supportMargin([2,1,0],h),-1);
});
test('required contact residual blocks quality and invalid transforms are rejected',()=>{
  const n=12,nb=1,W=new Float32Array(n*16);for(let f=0;f<n;f++)W.set([1,0,0,0,0,1,0,0,0,0,1,0,10,0,0,1],f*16);
  const r=inspectMotion({W,nb,n,names:['foot_l'],parents:[-1],feet:{l:0},locks:[{side:'l',a:0,b:11,target:[0,0,0]}]});assert.equal(r.valid,false);assert.equal(r.constraintMaxCm,10);assert.ok(r.failedConstraints>0);
  W[0]=NaN;assert.equal(inspectMotion({W,nb,n,names:['foot_l']}).nonfinite,1);
});
