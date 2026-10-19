const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));

export function retimeMapFrame(f,a,b,scale) {
  const span=Math.max(1,Math.round((b-a)*scale)),delta=span-(b-a);
  return f<a?f:f<=b?Math.round(a+(f-a)*scale):f+delta;
}

export function retimeTracks({q,p,nb,n,a,b,scale}) {
  if(!q||!p||!Number.isInteger(nb)||nb<1||q.length<n*nb*4||p.length<n*nb*3)throw new Error('Retime requires complete position and rotation tracks.');
  if(!Number.isInteger(a)||!Number.isInteger(b)||a<0||b>=n||b<=a)throw new Error('Retime range must be a valid multi-frame interval.');
  if(!Number.isFinite(scale)||scale<=0||scale>10)throw new Error('Retime scale must be between 0 and 10.');
  const span=Math.max(1,Math.round((b-a)*scale)),delta=span-(b-a),nextN=n+delta;
  if(nextN<2||nextN>120000)throw new Error('Retimed take is outside the supported frame range.');
  const qo=new q.constructor(nextN*nb*4),po=new p.constructor(nextN*nb*3);
  for(let f=0;f<nextN;f++){
    let sf=f;
    if(f>=a&&f<=a+span)sf=a+(f-a)*(b-a)/span;
    else if(f>a+span)sf=f-delta;
    sf=clamp(sf,0,n-1);
    const lo=Math.floor(sf),hi=Math.min(n-1,lo+1),w=sf-lo;
    for(let i=0;i<nb;i++){
      const o=(f*nb+i)*4,oa=(lo*nb+i)*4,ob=(hi*nb+i)*4;
      let ax=q[oa],ay=q[oa+1],az=q[oa+2],aw=q[oa+3],bx=q[ob],by=q[ob+1],bz=q[ob+2],bw=q[ob+3];
      let dot=ax*bx+ay*by+az*bz+aw*bw;if(dot<0){dot=-dot;bx=-bx;by=-by;bz=-bz;bw=-bw;}
      if(dot>.9995){ax+=w*(bx-ax);ay+=w*(by-ay);az+=w*(bz-az);aw+=w*(bw-aw);}
      else{const th=Math.acos(clamp(dot,-1,1)),s=Math.sin(th),u=Math.sin((1-w)*th)/s,v=Math.sin(w*th)/s;ax=ax*u+bx*v;ay=ay*u+by*v;az=az*u+bz*v;aw=aw*u+bw*v;}
      const len=Math.hypot(ax,ay,az,aw)||1;qo[o]=ax/len;qo[o+1]=ay/len;qo[o+2]=az/len;qo[o+3]=aw/len;
      for(let k=0;k<3;k++){const x=(lo*nb+i)*3+k,y=(hi*nb+i)*3+k;po[(f*nb+i)*3+k]=p[x]*(1-w)+p[y]*w;}
    }
  }
  return {q:qo,p:po,n:nextN,span,delta,range:[a,a+span]};
}
