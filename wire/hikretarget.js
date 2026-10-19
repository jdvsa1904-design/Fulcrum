import { characterize, retargetPose, poseFromBones, classifyName } from '../solvers/hik/retarget.js';
import { qmul, qconj } from '../solvers/hik/knee.js';
import { qrot } from '../ik.js';

export const HELPER_BONE = /(^|[_.:\s])(ik|twist|twistcor|corrective|correctiveroot|cor|attach|socket|weapon|vb|prop|interaction|center_of_mass|helper|end|nub|null)([_.:\s\d]|$)/i;

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const len = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => { const l = len(a); return l > 1e-12 ? [a[0] / l, a[1] / l, a[2] / l] : [0, 0, 0]; };
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

function isAncestor(bones, a, i) {
  for (let p = bones[i].parent; p != null && p >= 0; p = bones[p].parent) if (p === a) return true;
  return false;
}

const LIMBS = ['LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase', 'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'Spine', 'Neck', 'Head'];

export function slotName(name) {
  const n = String(name ?? '');
  if (!n || HELPER_BONE.test(n)) return '';
  if (classifyName(n)) return n;
  for (let i = 1; i < n.length; i++) if (/[a-z0-9]/.test(n[i - 1]) && /[A-Z]/.test(n[i])) { const t = n.slice(i); if (classifyName(t)) return t; }
  return n;
}

export function characterizeRig(bones, o = {}) {
  const clean = bones.map(b => ({ ...b, name: slotName(b.name) }));
  let ch = characterize(clean, o);
  const hips = ch.slots.Hips;
  if (hips !== undefined) {
    const bad = LIMBS.filter(s => ch.slots[s] !== undefined && !isAncestor(clean, hips, ch.slots[s]));
    if (bad.length) {
      for (const s of bad) clean[ch.slots[s]] = { ...clean[ch.slots[s]], name: '' };
      ch = characterize(clean, o);
    }
  }
  return { ...ch, bones };
}

export function bodyFrame(ch) {
  const P = s => ch.rest[s]?.p;
  const hips = P('Hips'), top = P('Head') ?? P('Neck') ?? P('Spine');
  const lu = P('LeftUpLeg'), ru = P('RightUpLeg');
  if (!hips || !top || !lu || !ru) return null;
  const up = unit(sub(top, hips));
  let left = sub(lu, ru);
  left = unit(sub(left, up.map(x => x * dot(left, up))));
  return { up, left, fwd: cross(left, up) };
}

function basisQuat(up, left, fwd) {
  const m = [[left[0], up[0], fwd[0]], [left[1], up[1], fwd[1]], [left[2], up[2], fwd[2]]];
  const t = m[0][0] + m[1][1] + m[2][2];
  let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; q = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4]; }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) { const s = Math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2; q = [s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]; }
  else if (m[1][1] > m[2][2]) { const s = Math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2; q = [(m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]; }
  else { const s = Math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2; q = [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s]; }
  const l = Math.hypot(...q);
  return q.map(x => x / l);
}

export function alignmentQuat(srcFrame, dstFrame) {
  const a = basisQuat(srcFrame.up, srcFrame.left, srcFrame.fwd), b = basisQuat(dstFrame.up, dstFrame.left, dstFrame.fwd);
  return qmul(b, qconj(a));
}

const rotateBones = (bones, G) => bones.map(b => ({ ...b, p: qrot(G, b.p), q: qmul(G, b.q ?? [0, 0, 0, 1]) }));

export function hikRetargetSetup(srcBones, dstBones, o = {}) {
  const dst = characterizeRig(dstBones, o.dst);
  const src0 = characterizeRig(srcBones, o.src);
  const fs = bodyFrame(src0), fd = bodyFrame(dst);
  const G = fs && fd ? alignmentQuat(fs, fd) : [0, 0, 0, 1];
  const src = characterizeRig(rotateBones(srcBones, G), o.src);
  const missing = [...new Set([...src.missing.map(s => 'source ' + s), ...dst.missing.map(s => 'target ' + s)])];
  return { src, dst, G, missing, valid: src.valid && dst.valid, reach: { realistic: 0, legRoll: 0, ...(o.reach ?? {}) } };
}

export function hikRetargetFrame(setup, srcGlobals) {
  const g = srcGlobals.map(x => x && { p: qrot(setup.G, x.p), q: qmul(setup.G, x.q) });
  const pose = poseFromBones(setup.src, g);
  return retargetPose(setup.src, pose, setup.dst, { reach: setup.reach });
}

export function dstWorldRotations(setup, result, dstBones) {
  const n = dstBones.length, q = new Array(n), slotOf = new Map(Object.entries(setup.dst.slots).map(([s, i]) => [i, s]));
  const order = [], seen = new Uint8Array(n);
  const visit = i => { if (seen[i]) return; const p = dstBones[i].parent; if (p != null && p >= 0) visit(p); seen[i] = 1; order.push(i); };
  for (let i = 0; i < n; i++) visit(i);
  const mapped = new Uint8Array(n);
  for (const i of order) {
    const s = slotOf.get(i), p = dstBones[i].parent;
    if (s && result.pose[s]) { q[i] = result.pose[s].q; mapped[i] = 1; continue; }
    if (p == null || p < 0) { q[i] = dstBones[i].q; continue; }
    q[i] = qmul(q[p], qmul(qconj(dstBones[p].q), dstBones[i].q));
  }
  return { q, mapped, hips: setup.dst.slots.Hips, hipsP: result.pose.Hips?.p };
}
