import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from '../build/cache/three-0.169.0/package/build/three.module.js';
import {buildControlRig,sampleControlRig,controlMotionToLocalQuaternions} from '../wire/cascadeur_rig.js';
const canonical=JSON.parse(fs.readFileSync(new URL('../models/cascadeur_canonical.json',import.meta.url)));
const names={pelvis:'pelvis',stomach:'spine_01',chest:'spine_05',neck:'neck_01',head:'head'};
for(const s of ['l','r'])for(const [a,b]of Object.entries({clavicle:'clavicle',arm:'upperarm',forearm:'lowerarm',hand:'hand',thigh:'thigh',calf:'calf',foot:'foot',toe:'ball'}))names[`${a}_${s}`]=`${b}_${s}`;
function fixture(){
 const entries=Object.entries(names),byKey=new Map(entries.map(([a,b],i)=>[b,i]));
 const parent={stomach:'pelvis',chest:'stomach',neck:'chest',head:'neck'};
 for(const s of ['l','r'])Object.assign(parent,{[`clavicle_${s}`]:'chest',[`arm_${s}`]:`clavicle_${s}`,[`forearm_${s}`]:`arm_${s}`,[`hand_${s}`]:`forearm_${s}`,[`thigh_${s}`]:'pelvis',[`calf_${s}`]:`thigh_${s}`,[`foot_${s}`]:`calf_${s}`,[`toe_${s}`]:`foot_${s}`});
 const restW=entries.map(([a])=>new THREE.Matrix4().makeTranslation(...canonical.positions[canonical.order.indexOf(a)]));
 const parentIdx=entries.map(([a])=>entries.findIndex(([b])=>b===parent[a]));
 const body={bones:entries.map(([,name])=>({name})),restW,parentIdx,byKey};
 const W=Float32Array.from(restW.flatMap(m=>m.toArray())),q=new Float32Array(entries.length*4),p=new Float32Array(entries.length*3);
 for(let i=0;i<entries.length;i++)q[i*4+3]=1;
 return {body,W,q,p,rig:buildControlRig({THREE,body,canonical,upm:100})};
}
test('Canonical helper rig samples coherent finite points and preserves a source pose',()=>{
 const {rig,W,q,p}=fixture(),controls=sampleControlRig(rig,W);
 assert.equal(controls.length,43);
 for(let i=0;i<43;i++)for(let k=0;k<3;k++)assert.ok(Math.abs(controls[i][k]-canonical.positions[i][k])<0.0001,`${i}/${k}`);
 const out=controlMotionToLocalQuaternions({rig,positions:[controls],sourceQ:q,sourceP:p,sourceW:W});
 assert.deepEqual(out.p,p);
 for(let i=0;i<q.length;i++)assert.ok(Math.abs(out.q[i]-q[i])<1e-5);
});
test('Recovered control rotations respect hierarchy while preserving the root path',()=>{
 const {rig,W,q,p}=fixture(),controls=sampleControlRig(rig,W),turn=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),0.5);
 const positions=controls.map(x=>new THREE.Vector3().fromArray(x).applyQuaternion(turn).add(new THREE.Vector3(50,0,0)).toArray());
 const out=controlMotionToLocalQuaternions({rig,positions:[positions],sourceQ:q,sourceP:p,sourceW:W});
 assert.deepEqual(out.p,p);
 const root=new THREE.Quaternion().fromArray(out.q,rig.mapped.pelvis*4);
 assert.ok(root.angleTo(turn)<1e-5);
 assert.ok(out.q.every(Number.isFinite));
});
test('Missing primary mappings and invalid outputs fail explicitly',()=>{
 const f=fixture();f.body.byKey.delete('hand_l');assert.throws(()=>buildControlRig({THREE,body:f.body,canonical}),/Missing required/);
 assert.throws(()=>controlMotionToLocalQuaternions({rig:f.rig,positions:[Array(43).fill([NaN,0,0])],sourceQ:f.q,sourceP:f.p,sourceW:f.W}),/finite/);
});
