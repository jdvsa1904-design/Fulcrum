import { add, sub, scale, dot, cross, len, norm, mid, perp, angle, smoothstep, qmul, qconj, qrot, qarc, qaxis, qslerp, qalign, frameRotation, swingTwist, twistAngle, clampRotation, clamp, QI } from './math.js';
import { B, NB, LM, SIDES, lmAt, restPose, landmarksFromPose, setQ, getQ, worldToLocal } from './skeleton.js';

const LML = { sh: LM.lSh, el: LM.lElb, wr: LM.lWri, ix: LM.lIndex, pk: LM.lPinky, hip: LM.lHip, kn: LM.lKnee, an: LM.lAnk, he: LM.lHeel, to: LM.lToe };
const LMR = { sh: LM.rSh, el: LM.rElb, wr: LM.rWri, ix: LM.rIndex, pk: LM.rPinky, hip: LM.rHip, kn: LM.rKnee, an: LM.rAnk, he: LM.rHeel, to: LM.rToe };
export const LIMB_LM = { L: LML, R: LMR };
export const ARM_REST = side => ({ d0: [SIDES[side].s, 0, 0], h0: [0, -SIDES[side].s, 0] });
export const LEG_REST = { d0: [0, -1, 0], h0: [1, 0, 0] };

export function restFrames(skel) {
  const L = landmarksFromPose(skel, restPose(skel)), P = i => lmAt(L, i), earMid = mid(P(LM.lEar), P(LM.rEar));
  const foot = {};
  for (const [side, m] of [['L', LML], ['R', LMR]]) foot[side] = { fwd: sub(P(m.to), P(m.he)), up: sub(P(m.an), P(m.he)) };
  return { headFwd: sub(P(LM.nose), earMid), headLeft: sub(P(LM.lEar), P(LM.rEar)), foot };
}

export function limbRotations(upper, lower, parentQ, d0, h0, o = {}) {
  const u = norm(upper);
  let l = norm(lower);
  const beta = angle(u, l);
  const hfMin = qrot(qmul(qarc(qrot(parentQ, d0), u), parentQ), h0);
  let hf = hfMin;
  if (o.hint) hf = add(hf, o.hint);
  if (o.prev) hf = add(hf, scale(norm(perp(o.prev, u)), o.prevWeight ?? 2));
  hf = norm(perp(hf, u));
  if (len(hf) < 0.5) hf = norm(perp(hfMin, u));
  let h = cross(u, l);
  const hl = len(h);
  h = hl > 1e-6 ? scale(h, 1 / hl) : hf;
  let flipped = false;
  if (dot(h, hf) < 0 && (o.flipAlways || beta < (o.flipMax ?? 0.6))) { h = scale(h, -1); flipped = true; }
  const w = smoothstep(o.bendLo ?? 0.15, o.bendHi ?? 0.45, beta);
  let hb = norm(perp(add(scale(h, w), scale(hf, 1 - w)), u));
  if (len(hb) < 0.5) hb = hf;
  if (flipped) l = qrot(qaxis(hb, -Math.min(beta, o.maxHyper ?? 0.09)), u);
  else if (o.maxBend && beta > o.maxBend) l = qrot(qaxis(hb, o.maxBend), u);
  return { qUp: frameRotation(u, hb, d0, h0), qLo: frameRotation(l, hb, d0, h0), hinge: hb, beta: flipped ? -Math.min(beta, o.maxHyper ?? 0.09) : beta };
}

export function solveFrame(skel, L, rf, prev = null, o = {}) {
  const P = i => lmAt(L, i), world = new Float64Array(NB * 4), hinge = {};
  const hipMid = mid(P(LM.lHip), P(LM.rHip)), shMid = mid(P(LM.lSh), P(LM.rSh)), up = sub(shMid, hipMid);
  const qH = frameRotation(up, sub(P(LM.lHip), P(LM.rHip)), [0, 1, 0], [1, 0, 0]);
  const qC = qmul(qH, clampRotation(qmul(qconj(qH), frameRotation(up, sub(P(LM.lSh), P(LM.rSh)), [0, 1, 0], [1, 0, 0])), o.maxSpine ?? 1.1));
  setQ(world, B.Hips, qH);
  setQ(world, B.Spine, qslerp(qH, qC, 1 / 3));
  setQ(world, B.Spine1, qslerp(qH, qC, 2 / 3));
  setQ(world, B.Spine2, qC);
  const earMid = mid(P(LM.lEar), P(LM.rEar));
  const qHd = qmul(qC, clampRotation(qmul(qconj(qC), frameRotation(sub(P(LM.nose), earMid), sub(P(LM.lEar), P(LM.rEar)), rf.headFwd, rf.headLeft)), o.maxNeck ?? 1.2));
  setQ(world, B.Neck, qslerp(qC, qHd, 0.5));
  setQ(world, B.Head, qHd);
  for (const side of ['L', 'R']) {
    const S = SIDES[side], m = LIMB_LM[side], { d0, h0 } = ARM_REST(side);
    setQ(world, S.clav, qC);
    const arm = limbRotations(sub(P(m.el), P(m.sh)), sub(P(m.wr), P(m.el)), qC, d0, h0, { prev: prev?.hinge?.[side + 'arm'] });
    hinge[side + 'arm'] = arm.hinge;
    setQ(world, S.arm, arm.qUp);
    const handWorld = frameRotation(sub(mid(P(m.ix), P(m.pk)), P(m.wr)), sub(P(m.ix), P(m.pk)), d0, [0, 0, 1]);
    const { swing, twist } = swingTwist(qalign(qmul(qconj(arm.qLo), handWorld), QI), d0);
    const ta = clamp(twistAngle(twist, d0), -(o.maxWristTwist ?? 2.6), o.maxWristTwist ?? 2.6);
    const half = qaxis(d0, ta / 2), fore = qmul(arm.qLo, half);
    setQ(world, S.fore, fore);
    setQ(world, S.hand, qmul(fore, qmul(qmul(qconj(half), clampRotation(swing, o.maxWrist ?? 1.4)), qaxis(d0, ta))));
    const upper = sub(P(m.kn), P(m.hip)), footFwd = sub(P(m.to), P(m.he));
    const leg = limbRotations(upper, sub(P(m.an), P(m.kn)), qH, LEG_REST.d0, LEG_REST.h0, { hint: cross(norm(footFwd), norm(upper)), flipAlways: true, maxBend: 2.6, prev: prev?.hinge?.[side + 'leg'] });
    hinge[side + 'leg'] = leg.hinge;
    setQ(world, S.up, leg.qUp);
    setQ(world, S.knee, leg.qLo);
    const foot = qmul(leg.qLo, clampRotation(qmul(qconj(leg.qLo), frameRotation(footFwd, sub(P(m.an), P(m.he)), rf.foot[side].fwd, rf.foot[side].up)), o.maxAnkle ?? 1.1));
    setQ(world, S.foot, foot);
    setQ(world, S.toe, foot);
  }
  const root = add(hipMid, qrot(qH, [0, skel.dims.hipsUp, 0]));
  return { root, world, local: worldToLocal(world), hinge };
}

export function solveTake(skel, P, n, o = {}) {
  const rf = restFrames(skel), root = new Float64Array(n * 3), rot = new Float64Array(n * NB * 4);
  let prev = null;
  for (let i = 0; i < n; i++) {
    const r = solveFrame(skel, P.subarray(i * 99, (i + 1) * 99), rf, prev, o);
    root.set(r.root, i * 3);
    for (let b = 0; b < NB; b++) {
      let q = getQ(r.local, b);
      if (i > 0) q = qalign(q, getQ(rot, (i - 1) * NB + b));
      setQ(rot, i * NB + b, q);
    }
    prev = r;
  }
  return { n, root, rot };
}
