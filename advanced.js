import { applySpringSecondary } from './secondary.js';

const clamp = (x, a = 0, b = 1) => Math.max(a, Math.min(b, x));
const ease = t => t * t * (3 - 2 * t);
const qdot = (a, b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3];
const qnorm = q => { const d = Math.hypot(...q) || 1; return q.map(x => x / d); };
export function qslerp(a, b, t) {
  let bb = b, d = qdot(a, b);
  if (d < 0) { bb = b.map(x => -x); d = -d; }
  if (d > 0.9995) return qnorm(a.map((x, i) => x + (bb[i] - x) * t));
  const th = Math.acos(clamp(d, -1, 1)), s = Math.sin(th);
  return a.map((x, i) => (x * Math.sin((1-t)*th) + bb[i] * Math.sin(t*th)) / s);
}
const qread = (q, nb, f, i) => Array.from(q.subarray((f*nb+i)*4, (f*nb+i)*4+4));
const qwrite = (q, nb, f, i, v) => q.set(v, (f*nb+i)*4);
const pread = (p, nb, f, i) => Array.from(p.subarray((f*nb+i)*3, (f*nb+i)*3+3));
const pwrite = (p, nb, f, i, v) => p.set(v, (f*nb+i)*3);
const selected = (name, list) => !list?.length || list.some(x => name.toLowerCase().includes(x.toLowerCase()));

export function applyPoseGuidedInbetween({ q, p, nb, n, names, a, b, strength=1, preserve=[], keys=[], bones=[], preserveRoot=true, roots=[0], spacing='ease', style='soft' }) {
  a = Math.max(0, a|0); b = Math.min(n-1, b|0);
  if (b-a < 2) return { frames:0, joints:0 };
  strength=clamp(Number.isFinite(strength)?strength:1);
  const snapshotQ=new q.constructor(q), snapshotP=new p.constructor(p);
  const anchors=[...new Set([a,...keys.filter(f=>Number.isInteger(f)&&f>a&&f<b),b])].sort((x,y)=>x-y);
  let joints = 0;
  for (let i=0;i<nb;i++) {
    if (preserve?.length && preserve.some(x => (names[i] || '').toLowerCase().includes(x.toLowerCase()))) continue;
    if(!selected(names[i]||'',bones))continue;
    joints++;
    for(let s=0;s<anchors.length-1;s++){
    const left=anchors[s],right=anchors[s+1];
    const qa=qread(snapshotQ,nb,left,i), qb=qread(snapshotQ,nb,right,i), pa=pread(snapshotP,nb,left,i), pb=pread(snapshotP,nb,right,i);
    const va=left>0 ? pa.map((x,k)=>x-pread(snapshotP,nb,left-1,i)[k]) : [0,0,0];
    const vb=right+1<n ? pread(snapshotP,nb,right+1,i).map((x,k)=>x-pb[k]) : [0,0,0];
    for(let f=left+1;f<right;f++) {
      const t=(f-left)/(right-left), u=spacing==='linear'?t:spacing==='heavy'?t*t:ease(t), old=qread(snapshotQ,nb,f,i);
      qwrite(q,nb,f,i,qslerp(old,qslerp(qa,qb,u),strength));
      if(preserveRoot||!roots.includes(i))continue;
      const h00=2*t*t*t-3*t*t+1, h10=t*t*t-2*t*t+t, h01=-2*t*t*t+3*t*t, h11=t*t*t-t*t, span=right-left;
      const hp=pa.map((x,k)=>{const v=h00*x+h10*span*va[k]+h01*pb[k]+h11*span*vb[k];return style==='sharp'?x+(pb[k]-x)*u:clamp(v,Math.min(x,pb[k]),Math.max(x,pb[k]));});
      const op=pread(snapshotP,nb,f,i); pwrite(p,nb,f,i,op.map((x,k)=>x+(hp[k]-x)*strength));
    }
    }
  }
  return { frames:b-a-anchors.length+1, joints, keys:anchors, preserveRoot };
}

export function applySecondaryMotion({ q, nb, n, names, a=0, b=n-1, bones=[], strength=.35, damping=.72, delay=2 }) {
  return applySpringSecondary({q,nb,n,names,a,b,bones,strength,damping,delaySeconds:delay/30,fps:30});
}

const qmul=(a,b)=>[a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];
const qconj=q=>[-q[0],-q[1],-q[2],q[3]];
export function validatePose({ q, reference, nb, f, names, limits={} }) {
  const issues=[];
  for(let i=0;i<nb;i++) {
    const name=(names[i]||'').toLowerCase(), v=qread(q,nb,f,i), rel=reference?qmul(qconj(qread(reference,nb,f,i)),v):v, w=clamp(Math.abs(rel[3]),0,1), deg=2*Math.acos(w)*180/Math.PI;
    const limit=Object.entries(limits).find(([k])=>name.includes(k))?.[1] ?? (/calf|lowerarm/.test(name)?165:175);
    if(deg>limit) issues.push({ bone:names[i], angle:+deg.toFixed(1), limit, kind:'joint-limit' });
  }
  return { valid:!issues.length, issues };
}

export function assistedPose({ q, reference, nb, f, names, limits={} }) {
  const check=validatePose({q,reference,nb,f,names,limits}), corrections=[];
  for(const issue of check.issues) {
    const i=names.indexOf(issue.bone), base=qread(reference,nb,f,i), cur=qread(q,nb,f,i), target=qslerp(base,cur,issue.limit/issue.angle);
    corrections.push({bone:issue.bone,index:i,target,delta:qmul(qconj(base),target),from:issue.angle,to:issue.limit});
  }
  return {valid:!corrections.length,corrections};
}

export function applyAdvancedMotion({ q, p, nb, n, names, fixes, fps=30, roots=[0] }) {
  const report=[];
  for(const fx of fixes||[]) {
    if(fx.type==='inbetween') report.push({type:fx.type,...applyPoseGuidedInbetween({q,p,nb,n,names,roots,...fx})});
    if(fx.type==='secondary') report.push({type:fx.type,...applySpringSecondary({q,nb,n,names,fps,...fx})});
  }
  return { q, p, report };
}
