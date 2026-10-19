// Shared BVH helpers for the motion library tools (motionlib_index.mjs, motionlib_posebank.mjs): quaternion math,
// bone-name cleanup, Auto-Rig Pro alias layer, rig family, FK and canonical pose-bank frames. Quaternions are [x, y, z, w].
import { BANK_BONES, NB, rigCalibration, canonFrames, canonPose, contextFloor, frameContext, qslerp } from '../wire/posebank.js';

export const qmul = (a, b) => [a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
export const qrot = (q, v) => { const [x, y, z, w] = q, ix = w * v[0] + y * v[2] - z * v[1], iy = w * v[1] + z * v[0] - x * v[2], iz = w * v[2] + x * v[1] - y * v[0], iw = -x * v[0] - y * v[1] - z * v[2];
  return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x]; };
export const qaxis = (ax, deg) => { const h = deg / (180 / Math.PI) / 2, s = Math.sin(h); return ax === 'X' ? [s, 0, 0, Math.cos(h)] : ax === 'Y' ? [0, s, 0, Math.cos(h)] : [0, 0, s, Math.cos(h)]; };

export const stripPrefix = n => n.replace(/^\d{3}_/, '');
// Auto-Rig Pro deform names -> names rigmap.js already understands. Only used as a second pass.
export function arpAlias(n) {
  const m = n.match(/^(c_)?(.+?)\.(x|l|r)$/);
  if (!m) return n;
  let b = m[2]; const s = m[3];
  b = b.replace(/_stretch$/, '');
  const T = { root: 'pelvis', 'spine_01': 'spine_01', 'spine_02': 'spine_02', 'spine_03': 'spine_03', neck: 'neck_01', head: 'head', shoulder: 'clavicle', arm: 'upperarm', forearm: 'lowerarm',
    thigh: 'thigh', leg: 'calf', foot: 'foot', toes_01: 'ball', hand: 'hand', arm_twist: 'upperarm_twist', forearm_twist: 'lowerarm_twist', leg_twist: 'calf_twist', thigh_twist: 'thigh_twist' };
  const fm = b.match(/^(thumb|index|middle|ring|pinky)(\d)$/);
  const base = fm ? `${fm[1]}_0${fm[2]}` : T[b] ?? b;
  return s === 'x' ? base : `${base}_${s}`;
}

export function rigFamily(names) {
  if (names.some(n => /mixamorig/i.test(n))) return 'mixamo';
  if (names.some(n => /\.(x|l|r)$/.test(n))) return 'autorigpro';
  if (names.includes('pelvis') && names.includes('thigh_l')) return 'ue_mannequin';
  return 'other';
}

// FK: world positions P, world rotations Q, local rotations L and local translations T (OFFSET + position channels), per frame and joint.
export function forward(bvh, unit) {
  const J = bvh.joints, nj = J.length, n = bvh.values.length, P = new Float64Array(n * nj * 3), Q = new Float64Array(n * nj * 4), L = new Float64Array(n * nj * 4), T = new Float64Array(n * nj * 3);
  const cols = []; let c = 0;
  for (const j of J) { cols.push(c); c += j.channels.length; }
  let finite = true;
  for (let f = 0; f < n; f++) {
    const row = bvh.values[f];
    for (let i = 0; i < nj; i++) {
      const j = J[i], t = [...(j.offset ?? [0, 0, 0])];
      let q = [0, 0, 0, 1];
      j.channels.forEach((ch, k) => {
        const v = row[cols[i] + k];
        if (!Number.isFinite(v)) finite = false;
        if (ch.endsWith('position')) t['XYZ'.indexOf(ch[0])] += v;
        else q = qmul(q, qaxis(ch[0], v));
      });
      for (let k = 0; k < 3; k++) t[k] *= unit;
      L.set(q, (f * nj + i) * 4); T.set(t, (f * nj + i) * 3);
      const p = j.parent;
      if (p < 0) { P.set(t, (f * nj + i) * 3); Q.set(q, (f * nj + i) * 4); continue; }
      const pq = Q.subarray((f * nj + p) * 4, (f * nj + p) * 4 + 4), pp = P.subarray((f * nj + p) * 3, (f * nj + p) * 3 + 3), rt = qrot(pq, t);
      P.set([pp[0] + rt[0], pp[1] + rt[1], pp[2] + rt[2]], (f * nj + i) * 3);
      Q.set(qmul(pq, q), (f * nj + i) * 4);
    }
  }
  return { P, Q, L, T, n, nj, finite };
}

// Pose-bank view of one parsed clip (wire/posebank.js), using the index record's canonical bone map (rigmap.js names;
// Auto-Rig Pro through the arpAlias layer above). null when a bank bone is unmapped.
export function clipRig(bvh, rec) {
  const unit = rec.unit_to_m ?? 1, fk = forward(bvh, unit), names = bvh.joints.map(j => stripPrefix(j.name)), canon = rec.rigmap?.canonical ?? {};
  const lookup = k => canon[k] == null ? -1 : names.indexOf(canon[k]);
  if (BANK_BONES.some(k => lookup(k) < 0)) return null;
  const nj = fk.nj, q = (t, i) => Array.from(fk.Q.subarray((t * nj + i) * 4, (t * nj + i) * 4 + 4)), p = (t, i) => [fk.P[(t * nj + i) * 3], fk.P[(t * nj + i) * 3 + 1], fk.P[(t * nj + i) * 3 + 2]];
  return { fk, n: fk.n, fps: 1 / bvh.frameTime, lookup, q, p, hierarchy: rec.hierarchy ?? names.join(' ') };
}
// feed every `step`th frame of a clip into a rigCalibration shared by clips of the same hierarchy
export function calibrateWith(cal, cr, step = 3) { for (let t = 0; t < cr.n; t += step) cal.add(i => cr.q(t, i), i => cr.p(t, i)); }
export const newCalibration = cr => rigCalibration(cr.lookup);
// canonical frames of every clip frame: heading-free locals L, yaw, pelvis velocity, height and contacts
export function clipCanonical(cr, rig) {
  const { n, fps } = cr, pos = cr.p, floor = contextFloor({ pos, n, rig }), frames = [];
  for (let t = 0; t < n; t++) {
    const { yaw, L } = canonPose(canonFrames(rig, i => cr.q(t, i)), rig.anc);
    frames.push({ L, yaw, ...frameContext({ pos, n, fps, rig, f: t, yaw, floor }) });
  }
  return { fps, n, frames };
}
// resample canonical frames to `fps` (slerp locals, interpolate yaw/velocity/height, nearest contact and phase id)
export function resampleCanonical(can, fps, phase = null) {
  const m = Math.floor((can.n - 1) / can.fps * fps + 1e-6) + 1, out = [];
  for (let k = 0; k < m; k++) {
    const s = Math.min(can.n - 1, k * can.fps / fps), a = Math.floor(s), b = Math.min(can.n - 1, a + 1), w = s - a, A = can.frames[a], B = can.frames[b], L = new Float64Array(NB * 4);
    for (let j = 0; j < NB; j++) L.set(qslerp(Array.from(A.L.subarray(j * 4, j * 4 + 4)), Array.from(B.L.subarray(j * 4, j * 4 + 4)), w), j * 4);
    const dy = B.yaw - A.yaw - 2 * Math.PI * Math.round((B.yaw - A.yaw) / (2 * Math.PI));
    out.push({ L, yaw: A.yaw + dy * w, vel: A.vel.map((v, i) => v + (B.vel[i] - v) * w), height: A.height + (B.height - A.height) * w, contact: (w < 0.5 ? A : B).contact, phase: phase ? phase[w < 0.5 ? a : b] : 255 });
  }
  return out;
}
