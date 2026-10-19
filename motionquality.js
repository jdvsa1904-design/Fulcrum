const pos=(W,nb,f,i)=>Array.from(W.subarray((f*nb+i)*16+12,(f*nb+i)*16+15));
const distance=(a,b)=>Math.hypot(...a.map((v,k)=>v-b[k]));
const cross=(o,a,b)=>(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);
export function supportHull(points){
  const sorted=points.map(p=>[p[0],p[2]]).sort((a,b)=>a[0]-b[0]||a[1]-b[1]);
  if(sorted.length<3)return sorted;
  const half=arr=>{const h=[];for(const p of arr){while(h.length>1&&cross(h.at(-2),h.at(-1),p)<=0)h.pop();h.push(p);}return h;};
  return [...half(sorted).slice(0,-1),...half([...sorted].reverse()).slice(0,-1)];
}
export function supportMargin(point,hull){
  if(hull.length<3)return null;
  let inside=true,min=Infinity;
  for(let i=0;i<hull.length;i++){
    const a=hull[i],b=hull[(i+1)%hull.length],dx=b[0]-a[0],dy=b[1]-a[1],t=Math.max(0,Math.min(1,((point[0]-a[0])*dx+(point[2]-a[1])*dy)/(dx*dx+dy*dy||1)));
    min=Math.min(min,Math.hypot(point[0]-a[0]-t*dx,point[2]-a[1]-t*dy));
    if(cross(a,b,[point[0],point[2]])<0)inside=false;
  }
  return inside?min:-min;
}
export function inspectMotion({W,nb,n,fps=30,upm=100,ground=0,names,parents,com,feet={},locks=[],pins=[],boxes=[]}){
  const findings=[],constraints=[],balance=[];
  let nonfinite=0,maxStep=0;
  for(const v of W)if(!Number.isFinite(v))nonfinite++;
  if(nonfinite)return {valid:false,nonfinite,findings:[{kind:'nonfinite',severity:'blocker',count:nonfinite}],constraints,balance};
  for(let f=0;f<n;f++){
    const supports=[];
    for(const [side,i] of Object.entries(feet)){
      if(i==null)continue;
      const p=pos(W,nb,f,i),prev=pos(W,nb,Math.max(0,f-1),i),next=pos(W,nb,Math.min(n-1,f+1),i);
      const speed=distance(prev,next)*fps/(f>0&&f<n-1?2:1)/upm;
      if(p[1]-ground<.1*upm&&speed<.25){for(const dx of [-.05,.05])for(const dz of [-.10,.10])supports.push([p[0]+dx*upm,ground,p[2]+dz*upm]);}
      if(p[1]<ground-.03*upm)findings.push({kind:'ground-penetration',severity:'warning',frame:f,bone:names[i],cm:(ground-p[1])/upm*100});
    }
    if(com){const hull=supportHull(supports),margin=supportMargin(Array.from(com.subarray(f*3,f*3+3)),hull);balance.push({frame:f,marginCm:margin==null?null:margin/upm*100,hull});}
    for(let i=0;i<nb;i++){
      const p=pos(W,nb,f,i),parent=parents?.[i];
      if(parent>=0&&f>0&&!/pelvis|root|ik_|twist|corrective|weapon|attach|socket/.test(names[i].toLowerCase())){const rest=distance(pos(W,nb,0,i),pos(W,nb,0,parent)),len=distance(p,pos(W,nb,f,parent));if(Math.abs(rest-len)>.005*upm)findings.push({kind:'bone-length',severity:'blocker',frame:f,bone:names[i],cm:Math.abs(rest-len)/upm*100});}
      if(f>0)maxStep=Math.max(maxStep,distance(p,pos(W,nb,f-1,i))/upm*100);
      for(const box of boxes)if(p.every((v,k)=>v>box.min[k]&&v<box.max[k]))findings.push({kind:'prop-penetration',severity:'warning',frame:f,bone:names[i],collider:box.name});
    }
    for(const lk of locks){if(f<lk.a+4||f>lk.b-4||!lk.target||feet[lk.side]==null)continue;const cm=distance(pos(W,nb,f,feet[lk.side]),lk.target)/upm*100;constraints.push({kind:'foot',frame:f,bone:names[feet[lk.side]],errorCm:cm,required:lk.required!==false});}
    for(const pn of pins){if(f<pn.a+4||f>pn.b-4)continue;const li=names.findIndex(x=>x.toLowerCase()===(pn.driven||'hand_l').toLowerCase()),ri=names.findIndex(x=>x.toLowerCase()===(pn.driver||'hand_r').toLowerCase());if(li<0||(!pn.world&&ri<0))continue;let goal;if(pn.world)goal=pn.world.slice(12,15);else if(pn.off){const o=(f*nb+ri)*16,v=pn.off.slice(12,15);goal=[0,1,2].map(k=>W[o+k]*v[0]+W[o+4+k]*v[1]+W[o+8+k]*v[2]+W[o+12+k]);}if(goal)constraints.push({kind:pn.kind||'hand',frame:f,bone:names[li],errorCm:distance(pos(W,nb,f,li),goal)/upm*100,required:pn.required!==false});}
  }
  const failed=constraints.filter(c=>c.required&&c.errorCm>3);
  for(const c of failed)findings.push({...c,severity:'blocker',kind:'constraint-residual'});
  return {valid:!findings.some(x=>x.severity==='blocker'),nonfinite,maxStepCm:maxStep,constraintMaxCm:Math.max(0,...constraints.map(x=>x.errorCm)),failedConstraints:failed.length,findings,constraints,balance};
}
