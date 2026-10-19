export function guidanceForFrames({frames, controls, selectedBone, locks=[], pins=[], footBones={}, handBones={}}) {
  if (!Array.isArray(frames) || controls?.length !== 43) throw new Error('Canonical43 controls and frames required');
  const selected=new Set(controls.map((c,i)=>c.bone===selectedBone?i:-1).filter(i=>i>=0));
  if(!selected.size)throw new Error('Selected bone has no canonical guide point');
  const preservedBones=frames.map(f=>{
    const bones=new Set([selectedBone]);
    for(const lock of locks)if(f>=lock.a&&f<=lock.b&&footBones[lock.side]!=null)bones.add(footBones[lock.side]);
    for(const pin of pins)if(f>=pin.a&&f<=pin.b){
      for(const key of ['l','r'])if(handBones[key]!=null)bones.add(handBones[key]);
    }
    return bones;
  });
  const masks=preservedBones.map(bones=>controls.map(c=>[bones.has(c.bone)?1:0]));
  return {masks,preservedBones,selectedPoints:[...selected]};
}

export function preserveGuidedPoints(predicted,original,masks) {
  if(predicted.length!==original.length||predicted.length!==masks.length)throw new Error('Pose frame count mismatch');
  return predicted.map((frame,f)=>{
    if(frame.length!==43||original[f].length!==43||masks[f].length!==43)throw new Error('Expected canonical43 frame');
    return frame.map((p,i)=>masks[f][i][0] ? original[f][i].slice() : p.slice());
  });
}
