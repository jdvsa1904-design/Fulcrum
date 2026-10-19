// A deterministic starter rig for an upright, Y-up humanoid mesh. This is a
// geometric fit, not a learned anatomical solver or a replacement for weight painting.
const lerp = (a, b, t) => a + (b - a) * t;

export function planHumanoidRig(bounds) {
  const { min, max } = bounds || {};
  if (!Array.isArray(min) || !Array.isArray(max) || min.length !== 3 || max.length !== 3 || ![...min, ...max].every(Number.isFinite))
    throw new Error('Auto-rig needs finite mesh bounds.');
  const [width, height, depth] = max.map((v, i) => v - min[i]);
  if (height <= 0 || width < height * .12 || width > height * 1.7 || depth > height * 1.15)
    throw new Error('Auto-rig expects an upright Y-up humanoid mesh; its proportions are outside the supported range.');
  const cx = (min[0] + max[0]) / 2, cz = (min[2] + max[2]) / 2;
  const y = f => min[1] + f * height;
  const hipX = Math.min(height * .105, width * .21);
  const shoulderX = Math.min(height * .185, width * .27);
  const handX = Math.min(height * .455, width * .48);
  const elbowX = lerp(shoulderX, handX, .55);
  const toeZ = cz + Math.min(depth * .38, height * .11);
  const bones = [];
  const add = (name, parent, x, yy, z = cz) => bones.push({ name, parent, pos: [x, yy, z] });
  add('root', -1, cx, y(0));
  add('pelvis', 0, cx, y(.53));
  add('spine_01', 1, cx, y(.62));
  add('spine_02', 2, cx, y(.72));
  add('spine_03', 3, cx, y(.82));
  add('neck_01', 4, cx, y(.875));
  add('head', 5, cx, y(.945));
  for (const [side, sign] of [['l', -1], ['r', 1]]) {
    const clavicle = bones.length;
    add(`clavicle_${side}`, 4, cx + sign * shoulderX * .62, y(.825));
    add(`upperarm_${side}`, clavicle, cx + sign * shoulderX, y(.805));
    add(`lowerarm_${side}`, clavicle + 1, cx + sign * elbowX, y(.70));
    add(`hand_${side}`, clavicle + 2, cx + sign * handX, y(.60));
    const thigh = bones.length;
    add(`thigh_${side}`, 1, cx + sign * hipX, y(.50));
    add(`calf_${side}`, thigh, cx + sign * hipX * 1.05, y(.275));
    add(`foot_${side}`, thigh + 1, cx + sign * hipX * 1.08, y(.065));
    add(`ball_${side}`, thigh + 2, cx + sign * hipX * 1.08, y(.035), toeZ);
  }
  return { bones, bounds: { min: min.slice(), max: max.slice() }, height, width, depth, center: [cx, cz], shoulderX };
}

export function skinWeightsForPoint(point, plan) {
  if (!Array.isArray(point) || point.length !== 3 || !point.every(Number.isFinite)) throw new Error('Finite vertex position required.');
  const { bones, bounds, height, center, shoulderX } = plan;
  const names = new Map(bones.map((b, i) => [b.name, i]));
  const side = point[0] < center[0] ? 'l' : 'r';
  const normY = (point[1] - bounds.min[1]) / height;
  const arm = normY > .58 && Math.abs(point[0] - center[0]) > shoulderX * .82;
  const leg = normY < .55 && Math.abs(point[0] - center[0]) > height * .025;
  const allowed = arm
    ? [`spine_03`, `clavicle_${side}`, `upperarm_${side}`, `lowerarm_${side}`, `hand_${side}`]
    : leg
      ? ['pelvis', `thigh_${side}`, `calf_${side}`, `foot_${side}`, `ball_${side}`]
      : ['root', 'pelvis', 'spine_01', 'spine_02', 'spine_03', 'neck_01', 'head'];
  const scored = allowed.map(name => {
    const index = names.get(name), p = bones[index].pos;
    const dz = point[2] - p[2];
    const d2 = (point[0] - p[0]) ** 2 + (point[1] - p[1]) ** 2 + dz * dz * .5;
    return { index, score: 1 / (d2 + (height * .045) ** 2) };
  }).sort((a, b) => b.score - a.score).slice(0, 4);
  const sum = scored.reduce((s, x) => s + x.score, 0);
  const indices = [0, 0, 0, 0], weights = [0, 0, 0, 0];
  scored.forEach((x, i) => { indices[i] = x.index; weights[i] = x.score / sum; });
  return { indices, weights };
}
