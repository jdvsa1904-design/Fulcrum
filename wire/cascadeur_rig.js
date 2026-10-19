const PRIMARY = { pelvis:'pelvis', stomach:'spine_01', chest:'spine_05', neck:'neck_01', head:'head' };
for (const side of ['l','r']) for (const [p,b] of Object.entries({clavicle:'clavicle',arm:'upperarm',forearm:'lowerarm',hand:'hand',thigh:'thigh',calf:'calf',foot:'foot',toe:'ball'})) PRIMARY[`${p}_${side}`]=`${b}_${side}`;
const NEXT = { pelvis:'stomach',stomach:'chest',chest:'neck',neck:'head',head:'head_dir' };
for (const s of ['l','r']) Object.assign(NEXT,{[`clavicle_${s}`]:`arm_${s}`,[`arm_${s}`]:`forearm_${s}`,[`forearm_${s}`]:`hand_${s}`,[`hand_${s}`]:`hand_dir_${s}`,[`thigh_${s}`]:`calf_${s}`,[`calf_${s}`]:`foot_${s}`,[`foot_${s}`]:`toe_${s}`,[`toe_${s}`]:`toe_dir_${s}`});
const finite = xs => Array.from(xs).every(Number.isFinite);
const anchorName = name => name.replace(/_(add|dir|self)(?=_|$)/,'');

export function buildControlRig({THREE,body,canonical,upm=100}) {
  if (!THREE || !body || !(upm>0) || canonical?.order?.length!==43 || canonical?.positions?.length!==43) throw new Error('Canonical43 rig, body and valid units required');
  const index=Object.fromEntries(canonical.order.map((n,i)=>[n,i]));
  const C=canonical.positions.map(p=>new THREE.Vector3().fromArray(p));
  if (!C.every(p=>finite(p.toArray()))) throw new Error('Canonical rest points must be finite');
  const get=k=>body.byKey instanceof Map?body.byKey.get(k):body.byKey[k];
  const mapped={}, missing=[];
  for (const [name,key] of Object.entries(PRIMARY)) {
    let bi=get(key);
    if (name==='chest' && bi===undefined) for (const fallback of ['spine_04','spine_03','spine_02']) { bi=get(fallback); if(bi!==undefined)break; }
    if (bi===undefined || bi<0 || !body.restW[bi]) missing.push(`${name} (${key})`); else mapped[name]=bi;
  }
  if(missing.length) throw new Error(`Missing required rig joints: ${missing.join(', ')}`);
  const rest=body.restW.map(m=>m.clone()), restP=rest.map(m=>new THREE.Vector3().setFromMatrixPosition(m)), restQ=rest.map(m=>new THREE.Quaternion().setFromRotationMatrix(m));
  const canonicalHeight=Math.max(...C.map(p=>p.y))-Math.min(...C.map(p=>p.y));
  const worldHeight=Math.max(...restP.map(p=>p.y))-Math.min(...restP.map(p=>p.y));
  const globalScale=worldHeight/canonicalHeight;
  const controls=canonical.order.map((name,i)=>{
    if(mapped[name]!==undefined)return {name,bone:mapped[name],local:new THREE.Vector3(),primary:true};
    const anchor=anchorName(name), bone=mapped[anchor];
    if(bone===undefined)throw new Error(`No helper anchor for ${name}`);
    const next=NEXT[anchor], nextBone=mapped[next];
    const offset=C[i].clone().sub(C[index[anchor]]);
    let rotation=new THREE.Quaternion(), scale=globalScale;
    if(nextBone!==undefined) {
      const ca=C[index[next]].clone().sub(C[index[anchor]]), actual=restP[nextBone].clone().sub(restP[bone]);
      if(ca.lengthSq()<1e-10||actual.lengthSq()<1e-10)throw new Error(`Degenerate segment ${anchor}`);
      scale=actual.length()/ca.length(); rotation.setFromUnitVectors(ca.normalize(),actual.normalize());
    } else {
      const previous=Object.keys(NEXT).find(k=>NEXT[k]===anchor && mapped[k]!==undefined);
      if(previous) {
        const ca=C[index[anchor]].clone().sub(C[index[previous]]), actual=restP[bone].clone().sub(restP[mapped[previous]]);
        if(ca.lengthSq()>1e-10&&actual.lengthSq()>1e-10) {scale=actual.length()/ca.length();rotation.setFromUnitVectors(ca.normalize(),actual.normalize());}
      }
    }
    const local=offset.multiplyScalar(scale).applyQuaternion(rotation).applyQuaternion(restQ[bone].clone().invert());
    return {name,bone,local,primary:false};
  });
  const helpers={};
  for(const name of Object.keys(mapped)) {
    const side=name.endsWith('_l')?'_l':name.endsWith('_r')?'_r':'';
    const stem=side?name.slice(0,-2):name;
    helpers[name]=[`${stem}_add${side}`,`${stem}_dir${side}`,`${stem}_self${side}`].find(k=>index[k]!==undefined);
    if(!helpers[name] && name.startsWith('arm_'))helpers[name]=`clavicle_add${side}`;
  }
  return {THREE,body,canonical,index,mapped,controls,helpers,upm,nb:body.bones.length,design:'rest-aligned canonical helper points'};
}

export function sampleControlRig(rig,W,frame=0) {
  const {THREE,nb,controls,upm}=rig, scale=100/upm;
  if(!W||W.length<(frame+1)*nb*16)throw new Error('Incomplete world matrices for control sample');
  return controls.map(c=>{
    const m=new THREE.Matrix4().fromArray(W,(frame*nb+c.bone)*16);
    const p=c.local.clone().applyMatrix4(m).multiplyScalar(scale).toArray();
    if(!finite(p))throw new Error(`Nonfinite control ${c.name}`);
    return p;
  });
}

export function controlMotionToLocalQuaternions({rig,positions,sourceQ,sourceP,sourceW,firstFrame=0,preserveRoot=true}) {
  const {THREE,nb,body,index,mapped,helpers,upm}=rig;
  const q=new Float32Array(sourceQ),p=new Float32Array(sourceP);
  const v=a=>new THREE.Vector3().fromArray(a);
  if(!positions.length||firstFrame<0||(firstFrame+positions.length)*nb*4>q.length)throw new Error('Control motion exceeds source range');
  for(let j=0;j<positions.length;j++) {
    const frame=firstFrame+j,target=positions[j];
    if(target.length!==43||!target.every(x=>x.length===3&&finite(x)))throw new Error('Expected finite canonical43 motion');
    const source=sampleControlRig(rig,sourceW,frame), desired=new Map();
    for(const [name,bi] of Object.entries(mapped)) {
      const next=NEXT[name];if(index[next]===undefined)continue;
      const a=v(source[index[next]]).sub(v(source[index[name]])),b=v(target[index[next]]).sub(v(target[index[name]]));
      if(a.lengthSq()<1e-10||b.lengthSq()<1e-10)continue;
      const axis=b.clone().normalize(), swing=new THREE.Quaternion().setFromUnitVectors(a.normalize(),axis);
      const m=new THREE.Matrix4().fromArray(sourceW,(frame*nb+bi)*16),world=new THREE.Quaternion().setFromRotationMatrix(m);
      const helper=helpers[name];
      if(helper) {
        const h0=v(source[index[helper]]).sub(v(source[index[name]])).applyQuaternion(swing),h1=v(target[index[helper]]).sub(v(target[index[name]]));
        h0.addScaledVector(axis,-h0.dot(axis));h1.addScaledVector(axis,-h1.dot(axis));
        if(h0.lengthSq()>1e-10&&h1.lengthSq()>1e-10) {
          h0.normalize();h1.normalize();
          const angle=Math.atan2(axis.dot(h0.clone().cross(h1)),Math.max(-1,Math.min(1,h0.dot(h1))));
          swing.premultiply(new THREE.Quaternion().setFromAxisAngle(axis,angle));
        }
      }
      desired.set(bi,swing.multiply(world).normalize());
    }
    const worldQ=new Map(),active=new Set();
    const resolve=bi=>{
      if(worldQ.has(bi))return worldQ.get(bi);
      if(active.has(bi))throw new Error('Cyclic skeleton hierarchy');active.add(bi);
      const parent=body.parentIdx[bi],local=new THREE.Quaternion().fromArray(q,(frame*nb+bi)*4);
      let parentQ;
      if(parent>=0)parentQ=resolve(parent);
      else {
        const wm=new THREE.Matrix4().fromArray(sourceW,(frame*nb+bi)*16);
        parentQ=new THREE.Quaternion().setFromRotationMatrix(wm).multiply(local.clone().invert());
      }
      if(desired.has(bi))local.copy(parentQ).invert().multiply(desired.get(bi)).normalize();
      if(!finite(local.toArray()))throw new Error('Nonfinite reconstructed rotation');
      local.toArray(q,(frame*nb+bi)*4);const w=parentQ.clone().multiply(local);worldQ.set(bi,w);active.delete(bi);return w;
    };
    for(let bi=0;bi<nb;bi++)resolve(bi);
    if(!preserveRoot) {
      const bi=mapped.pelvis,parent=body.parentIdx[bi],delta=v(target[index.pelvis]).sub(v(source[index.pelvis])).multiplyScalar(upm/100);
      if(parent>=0)delta.applyQuaternion(worldQ.get(parent).clone().invert());
      const off=(frame*nb+bi)*3;p[off]+=delta.x;p[off+1]+=delta.y;p[off+2]+=delta.z;
    }
  }
  return {q,p,firstFrame,lastFrame:firstFrame+positions.length-1,design:rig.design,preserveRoot};
}
