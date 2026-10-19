import test from 'node:test';
import assert from 'node:assert/strict';
import { applyAdvancedMotion, applyPoseGuidedInbetween, applySecondaryMotion, assistedPose, validatePose } from '../advanced.js';

const fixture=()=>({q:Float32Array.from([
  0,0,0,1, 0,0,0,1, 0,0,0,1, 0,1,0,0,
]),p:Float32Array.from([0,0,0, 9,4,0, 20,0,0, 30,0,0]),nb:1,n:4,names:['pelvis']});

test('pose-guided inbetween replaces interior motion and preserves endpoints',()=>{
  const x=fixture(); const r=applyPoseGuidedInbetween({...x,a:0,b:3});
  assert.equal(r.frames,2); assert.equal(x.p[0],0); assert.equal(x.p[9],30);
  assert.ok(x.p[3] > 6 && x.p[3] < 12); assert.ok(x.p[6] > 18 && x.p[6] < 24);
});

test('secondary motion is deterministic and bounded',()=>{
  const x=fixture(); const before=Array.from(x.q); applySecondaryMotion({...x,a:0,b:3,bones:['pelvis'],strength:.4});
  assert.deepEqual(Array.from(x.q.slice(0,4)),before.slice(0,4));
  for(let f=0;f<4;f++) assert.ok(Math.abs(Math.hypot(...x.q.slice(f*4,f*4+4))-1)<1e-5);
});

test('pose validation reports impossible rotations',()=>{
  const x=fixture(); const r=validatePose({...x,f:3,limits:{pelvis:120}});
  assert.equal(r.valid,false); assert.equal(r.issues[0].bone,'pelvis');
});

test('pose assistance returns a bounded non-destructive delta',()=>{
  const x=fixture(), reference=new Float32Array(x.q); reference.set([0,0,0,1],12);
  const r=assistedPose({...x,reference,f:3,limits:{pelvis:120}});
  assert.equal(r.corrections.length,1); assert.ok(Math.abs(Math.hypot(...r.corrections[0].delta)-1)<1e-8);
});

test('advanced pipeline dispatches fixes',()=>{
  const x=fixture(); const r=applyAdvancedMotion({...x,fixes:[{type:'inbetween',a:0,b:3}]});
  assert.equal(r.report[0].type,'inbetween');
});
