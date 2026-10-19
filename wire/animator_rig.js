const HUMAN = [
  ['root','Root','pelvis'], ['pelvis','Pelvis','pelvis'],
  ['spine','Spine','spine_01'], ['chest','Chest','spine_05'], ['neck','Neck','neck_01'], ['head','Head','head'],
  ['clavicle_l','Left clavicle','clavicle_l'], ['upperarm_l','Left upper arm','upperarm_l'], ['lowerarm_l','Left forearm','lowerarm_l'], ['hand_l','Left hand','hand_l'],
  ['clavicle_r','Right clavicle','clavicle_r'], ['upperarm_r','Right upper arm','upperarm_r'], ['lowerarm_r','Right forearm','lowerarm_r'], ['hand_r','Right hand','hand_r'],
  ['thigh_l','Left thigh','thigh_l'], ['calf_l','Left calf','calf_l'], ['foot_l','Left foot','foot_l'], ['ball_l','Left toe','ball_l'],
  ['thigh_r','Right thigh','thigh_r'], ['calf_r','Right calf','calf_r'], ['foot_r','Right foot','foot_r'], ['ball_r','Right toe','ball_r'],
];
const IK = {
  hand_l:['upperarm_l','lowerarm_l','hand_l'], hand_r:['upperarm_r','lowerarm_r','hand_r'],
  foot_l:['thigh_l','calf_l','foot_l'], foot_r:['thigh_r','calf_r','foot_r'],
};
const stem = name => name.split(':').pop().split('|').pop().toLowerCase();

export function buildAnimatorControls(bones, byKey, parentIdx) {
  if (!Array.isArray(bones) || !Array.isArray(parentIdx) || bones.length !== parentIdx.length) throw new Error('Valid skeleton hierarchy required');
  const get = key => byKey instanceof Map ? byKey.get(key) : byKey[key];
  const out = [], used = new Set();
  for (const [id,label,key] of HUMAN) {
    let bone = get(key);
    if (bone == null && id === 'chest') for (const fallback of ['spine_04','spine_03','spine_02']) { bone=get(fallback); if (bone != null) break; }
    if (bone == null || bone < 0 || bone >= bones.length) continue;
    const chain = IK[id]?.map(get);
    const validChain = chain?.every(i => Number.isInteger(i) && i >= 0 && i < bones.length) && parentIdx[chain[2]] === chain[1] && parentIdx[chain[1]] === chain[0];
    out.push({ id, label, bone, modes: validChain ? ['fk','ik'] : ['fk'], chain: validChain ? chain : null });
    used.add(bone);
  }
  for (let i=0;i<bones.length;i++) {
    if (used.has(i)) continue;
    const name = bones[i].name;
    if (/(thumb|index|middle|ring|pinky|spine|jaw|eye|weapon|socket)/.test(stem(name))) out.push({id:'bone:'+i,label:name,bone:i,modes:['fk'],chain:null});
  }
  return out;
}

export function ikGoalFromOffset(end, offsetCm, unitsPerMeter) {
  if (end?.length !== 3 || offsetCm?.length !== 3 || ![...end,...offsetCm,unitsPerMeter].every(Number.isFinite) || unitsPerMeter <= 0) throw new Error('Finite IK goal and valid scale required');
  return end.map((v,i) => v + offsetCm[i] * unitsPerMeter / 100);
}

export function modelGroundY(groundWorld, unitsPerMeter) {
  if (!Number.isFinite(groundWorld) || !Number.isFinite(unitsPerMeter) || unitsPerMeter <= 0) throw new Error('Finite ground and valid scale required');
  return groundWorld * 100 / unitsPerMeter;
}
