import test from 'node:test';
import assert from 'node:assert/strict';
import {retimeMapFrame,retimeTracks} from '../wire/timeline.js';

test('range retiming changes only the selected duration and keeps outside tracks exact',()=>{
  const n=9,nb=1,q=new Float32Array(n*4),p=new Float32Array(n*3);
  for(let f=0;f<n;f++){q.set([0,Math.sin(f*.05),0,Math.cos(f*.05)],f*4);p.set([f,f*f,-f],f*3);}
  const r=retimeTracks({q,p,nb,n,a:2,b:6,scale:1.5});
  assert.equal(r.n,11);assert.deepEqual(r.range,[2,8]);
  assert.deepEqual([...r.p.slice(0,6)],[...p.slice(0,6)]);
  assert.deepEqual([...r.p.slice(9*3)],[...p.slice(7*3)]);
  assert.equal(retimeMapFrame(0,2,6,1.5),0);
  assert.equal(retimeMapFrame(6,2,6,1.5),8);
  assert.equal(retimeMapFrame(7,2,6,1.5),9);
  for(let f=0;f<r.n;f++)assert.ok(Math.abs(Math.hypot(...r.q.slice(f*4,f*4+4))-1)<1e-6);
});

test('retiming rejects invalid bounds, rates and incomplete tracks',()=>{
  const q=new Float32Array(16),p=new Float32Array(12);
  assert.throws(()=>retimeTracks({q,p,nb:1,n:4,a:1,b:1,scale:2}),/range/);
  assert.throws(()=>retimeTracks({q,p,nb:1,n:4,a:0,b:3,scale:0}),/scale/);
  assert.throws(()=>retimeTracks({q,p,nb:2,n:4,a:0,b:3,scale:1}),/complete/);
});

test('rounded retime duration still maps its last output sample exactly to the source endpoint',()=>{
  const n=8,nb=1,q=new Float32Array(n*4),p=new Float32Array(n*3);
  for(let f=0;f<n;f++){q.set([0,Math.sin(f*.13),0,Math.cos(f*.13)],f*4);p.set([f,2*f,-f],f*3);}
  const r=retimeTracks({q,p,nb,n,a:1,b:6,scale:1.3}),last=r.range[1];
  assert.deepEqual([...r.p.slice(last*3,last*3+3)],[...p.slice(6*3,6*3+3)]);
  assert.ok(Math.abs(Math.abs(r.q.slice(last*4,last*4+4).reduce((s,x,i)=>s+x*q[6*4+i],0))-1)<1e-6);
});
