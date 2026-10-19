import { add, len, dist, mid, qmul, qrot, qaxis, median, clamp } from './math.js';

export const BONES = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Neck', 'Head',
  'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand',
  'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase'];
export const PARENT = [-1, 0, 1, 2, 3, 4, 3, 6, 7, 8, 3, 10, 11, 12, 0, 14, 15, 16, 0, 18, 19, 20];
export const NB = BONES.length;
export const B = Object.freeze(Object.fromEntries(BONES.map((n, i) => [n, i])));
export const ENDS = Object.freeze({ Head: 'HeadTop_End', LeftHand: 'LeftHand_End', RightHand: 'RightHand_End', LeftToeBase: 'LeftToe_End', RightToeBase: 'RightToe_End' });
export const SIDES = Object.freeze({
  L: { s: 1, up: B.LeftUpLeg, knee: B.LeftLeg, foot: B.LeftFoot, toe: B.LeftToeBase, clav: B.LeftShoulder, arm: B.LeftArm, fore: B.LeftForeArm, hand: B.LeftHand },
  R: { s: -1, up: B.RightUpLeg, knee: B.RightLeg, foot: B.RightFoot, toe: B.RightToeBase, clav: B.RightShoulder, arm: B.RightArm, fore: B.RightForeArm, hand: B.RightHand },
});
export const MIRROR = BONES.map(n => B[n.startsWith('Left') ? 'Right' + n.slice(4) : n.startsWith('Right') ? 'Left' + n.slice(5) : n]);

export const LM = Object.freeze({
  nose: 0, lEyeIn: 1, lEye: 2, lEyeOut: 3, rEyeIn: 4, rEye: 5, rEyeOut: 6, lEar: 7, rEar: 8, mouthL: 9, mouthR: 10,
  lSh: 11, rSh: 12, lElb: 13, rElb: 14, lWri: 15, rWri: 16, lPinky: 17, rPinky: 18, lIndex: 19, rIndex: 20, lThumb: 21, rThumb: 22,
  lHip: 23, rHip: 24, lKnee: 25, rKnee: 26, lAnk: 27, rAnk: 28, lHeel: 29, rHeel: 30, lToe: 31, rToe: 32,
});
export const NLM = 33;
export const LM_MIRROR = [0, 4, 5, 6, 1, 2, 3, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17, 20, 19, 22, 21, 24, 23, 26, 25, 28, 27, 30, 29, 32, 31];
export const LM_EDGES = [[11, 12], [11, 13], [13, 15], [12, 14], [14, 16], [15, 17], [15, 19], [17, 19], [16, 18], [16, 20], [18, 20], [11, 23], [12, 24], [23, 24],
  [23, 25], [25, 27], [27, 29], [29, 31], [27, 31], [24, 26], [26, 28], [28, 30], [30, 32], [28, 32], [0, 7], [0, 8], [9, 10]];

export const DEFAULT_DIMS = Object.freeze({ hipWidth: 0.18, shoulderWidth: 0.34, torso: 0.5, neck: 0.2, earWidth: 0.15, upperArm: 0.28, forearm: 0.25, hand: 0.08, thigh: 0.42, shin: 0.41, foot: 0.24 });
export const lmAt = (L, i) => [L[i * 3], L[i * 3 + 1], L[i * 3 + 2]];

export function measureFromLandmarks(frames, o = {}) {
  const D = (a, b) => frames.map(L => dist(lmAt(L, a), lmAt(L, b)));
  const pair = (a, b, c, d) => frames.map(L => (dist(lmAt(L, a), lmAt(L, b)) + dist(lmAt(L, c), lmAt(L, d))) / 2);
  const midD = (a, b, c, d) => frames.map(L => dist(mid(lmAt(L, a), lmAt(L, b)), mid(lmAt(L, c), lmAt(L, d))));
  const raw = {
    hipWidth: median(D(LM.lHip, LM.rHip)),
    shoulderWidth: median(D(LM.lSh, LM.rSh)),
    torso: median(midD(LM.lSh, LM.rSh, LM.lHip, LM.rHip)),
    neck: median(midD(LM.lEar, LM.rEar, LM.lSh, LM.rSh)),
    earWidth: median(D(LM.lEar, LM.rEar)),
    upperArm: median(pair(LM.lSh, LM.lElb, LM.rSh, LM.rElb)),
    forearm: median(pair(LM.lElb, LM.lWri, LM.rElb, LM.rWri)),
    hand: median(frames.map(L => (dist(lmAt(L, LM.lWri), mid(lmAt(L, LM.lIndex), lmAt(L, LM.lPinky))) + dist(lmAt(L, LM.rWri), mid(lmAt(L, LM.rIndex), lmAt(L, LM.rPinky)))) / 2)),
    thigh: median(pair(LM.lHip, LM.lKnee, LM.rHip, LM.rKnee)),
    shin: median(pair(LM.lKnee, LM.lAnk, LM.rKnee, LM.rAnk)),
    foot: median(pair(LM.lHeel, LM.lToe, LM.rHeel, LM.rToe)),
  };
  const ref = raw.thigh + raw.shin + raw.torso, def = DEFAULT_DIMS.thigh + DEFAULT_DIMS.shin + DEFAULT_DIMS.torso;
  const k = Number.isFinite(ref) && ref > 0.4 * def && ref < 2.5 * def ? ref / def : 1;
  const out = {};
  for (const [name, d] of Object.entries(DEFAULT_DIMS)) {
    const v = raw[name], lo = (o.lo ?? 0.6) * d * k, hi = (o.hi ?? 1.6) * d * k;
    out[name] = Number.isFinite(v) && v > lo && v < hi ? v : d * k;
  }
  if (o.height) {
    const h = buildSkeleton(out).dims.height;
    for (const name of Object.keys(out)) out[name] *= o.height / h;
  }
  return out;
}

export function buildSkeleton(m = DEFAULT_DIMS) {
  const T = m.torso, hw = m.hipWidth, sw = m.shoulderWidth, F = m.foot, k = m.earWidth / 0.15;
  const ankleH = 0.3 * F, ballH = 0.09 * F, heelBack = 0.24 * F, ballFwd = 0.5 * F, toeFwd = 0.76 * F;
  const earUp = 0.035 * k, hipsUp = 0.08 * T, hipY = ankleH + m.shin + m.thigh;
  const neckBone = clamp(m.neck - 0.02 * T - earUp, 0.04, 0.25);
  const off = new Array(NB);
  off[B.Hips] = [0, hipY + hipsUp, 0];
  off[B.Spine] = [0, 0.12 * T, 0];
  off[B.Spine1] = [0, 0.26 * T, 0];
  off[B.Spine2] = [0, 0.26 * T, 0];
  off[B.Neck] = [0, 0.30 * T, 0];
  off[B.Head] = [0, neckBone, 0];
  for (const [side, s] of [['Left', 1], ['Right', -1]]) {
    off[B[side + 'Shoulder']] = [s * 0.16 * sw, 0.21 * T, 0];
    off[B[side + 'Arm']] = [s * 0.34 * sw, 0.07 * T, 0];
    off[B[side + 'ForeArm']] = [s * m.upperArm, 0, 0];
    off[B[side + 'Hand']] = [s * m.forearm, 0, 0];
    off[B[side + 'UpLeg']] = [s * hw / 2, -hipsUp, 0];
    off[B[side + 'Leg']] = [0, -m.thigh, 0];
    off[B[side + 'Foot']] = [0, -m.shin, 0];
    off[B[side + 'ToeBase']] = [0, -(ankleH - ballH), ballFwd];
  }
  const ends = {
    Head: [0, 1.1 * m.earWidth, 0],
    LeftHand: [2.1 * m.hand, 0, 0], RightHand: [-2.1 * m.hand, 0, 0],
    LeftToeBase: [0, -0.4 * ballH, toeFwd - ballFwd], RightToeBase: [0, -0.4 * ballH, toeFwd - ballFwd],
  };
  const rest = new Array(NB);
  for (let i = 0; i < NB; i++) rest[i] = PARENT[i] < 0 ? off[i].slice() : add(rest[PARENT[i]], off[i]);
  const sole = { heel: [0, -ankleH, -heelBack], ball: [0, -ballH, 0], toe: [0, -ballH, toeFwd - ballFwd] };
  const height = rest[B.Head][1] + ends.Head[1];
  const legLength = m.thigh + m.shin + ankleH;
  return {
    m: { ...m }, offsets: off, ends, rest, sole,
    dims: { ankleH, ballH, heelBack, ballFwd, toeFwd, hipsUp, hipY, earUp, neckBone, height, legLength, k, footLength: F },
  };
}

export function scaleSkeleton(skel, s) {
  const m = {};
  for (const [kk, v] of Object.entries(skel.m)) m[kk] = v * s;
  return buildSkeleton(m);
}

export function restLocal() {
  const r = new Float64Array(NB * 4);
  for (let i = 0; i < NB; i++) r[i * 4 + 3] = 1;
  return r;
}

export const getQ = (a, i) => [a[i * 4], a[i * 4 + 1], a[i * 4 + 2], a[i * 4 + 3]];
export const setQ = (a, i, q) => { a[i * 4] = q[0]; a[i * 4 + 1] = q[1]; a[i * 4 + 2] = q[2]; a[i * 4 + 3] = q[3]; };
export const getV = (a, i) => [a[i * 3], a[i * 3 + 1], a[i * 3 + 2]];
export const setV = (a, i, v) => { a[i * 3] = v[0]; a[i * 3 + 1] = v[1]; a[i * 3 + 2] = v[2]; };

export function fk(skel, root, local, out = { pos: new Float64Array(NB * 3), q: new Float64Array(NB * 4) }) {
  const { pos, q } = out;
  for (let i = 0; i < NB; i++) {
    const lq = getQ(local, i), p = PARENT[i];
    if (p < 0) {
      setQ(q, i, lq);
      setV(pos, i, root);
      continue;
    }
    const pq = getQ(q, p);
    setQ(q, i, qmul(pq, lq));
    setV(pos, i, add(getV(pos, p), qrot(pq, skel.offsets[i])));
  }
  return out;
}

export function worldToLocal(worldQ) {
  const local = new Float64Array(NB * 4);
  for (let i = 0; i < NB; i++) {
    const p = PARENT[i], w = getQ(worldQ, i);
    setQ(local, i, p < 0 ? w : qmul([-worldQ[p * 4], -worldQ[p * 4 + 1], -worldQ[p * 4 + 2], worldQ[p * 4 + 3]], w));
  }
  return local;
}

export function endPoint(skel, W, name) {
  return add(getV(W.pos, B[name]), qrot(getQ(W.q, B[name]), skel.ends[name]));
}

export function soleTrack(skel, W, side) {
  const S = SIDES[side], fq = getQ(W.q, S.foot), tq = getQ(W.q, S.toe), fp = getV(W.pos, S.foot), tp = getV(W.pos, S.toe);
  return { heel: add(fp, qrot(fq, skel.sole.heel)), ball: add(tp, qrot(tq, skel.sole.ball)), toe: add(tp, qrot(tq, skel.sole.toe)) };
}

export function landmarksFromPose(skel, W) {
  const L = new Float64Array(NLM * 3), d = skel.dims, m = skel.m, k = d.k, F = d.footLength;
  const at = (bone, o) => add(getV(W.pos, bone), qrot(getQ(W.q, bone), o));
  const put = (i, p) => setV(L, i, p);
  const h = B.Head;
  put(LM.nose, at(h, [0, d.earUp + 0.01 * k, 0.11 * k]));
  for (const [s, eyeIn, eye, eyeOut, ear, mouth] of [[1, LM.lEyeIn, LM.lEye, LM.lEyeOut, LM.lEar, LM.mouthL], [-1, LM.rEyeIn, LM.rEye, LM.rEyeOut, LM.rEar, LM.mouthR]]) {
    put(eyeIn, at(h, [s * 0.016 * k, d.earUp + 0.04 * k, 0.09 * k]));
    put(eye, at(h, [s * 0.032 * k, d.earUp + 0.04 * k, 0.085 * k]));
    put(eyeOut, at(h, [s * 0.046 * k, d.earUp + 0.04 * k, 0.078 * k]));
    put(ear, at(h, [s * 0.5 * m.earWidth, d.earUp, 0]));
    put(mouth, at(h, [s * 0.025 * k, d.earUp - 0.025 * k, 0.095 * k]));
  }
  for (const [side, s, sh, el, wr, pk, ix, th, hp, kn, an, he, to] of [
    ['L', 1, LM.lSh, LM.lElb, LM.lWri, LM.lPinky, LM.lIndex, LM.lThumb, LM.lHip, LM.lKnee, LM.lAnk, LM.lHeel, LM.lToe],
    ['R', -1, LM.rSh, LM.rElb, LM.rWri, LM.rPinky, LM.rIndex, LM.rThumb, LM.rHip, LM.rKnee, LM.rAnk, LM.rHeel, LM.rToe]]) {
    const S = SIDES[side];
    put(sh, getV(W.pos, S.arm));
    put(el, getV(W.pos, S.fore));
    put(wr, getV(W.pos, S.hand));
    put(ix, at(S.hand, [s * m.hand, 0, 0.03 * k]));
    put(pk, at(S.hand, [s * m.hand, 0, -0.03 * k]));
    put(th, at(S.hand, [s * 0.45 * m.hand, -0.015 * k, 0.045 * k]));
    put(hp, getV(W.pos, S.up));
    put(kn, getV(W.pos, S.knee));
    put(an, getV(W.pos, S.foot));
    put(he, at(S.foot, [0, -d.ankleH + 0.06 * F, -d.heelBack]));
    put(to, at(S.toe, [0, -d.ballH + 0.03 * F, d.toeFwd - d.ballFwd]));
  }
  return L;
}

export function standLocal(o = {}) {
  const r = restLocal(), arm = (o.armDown ?? 75) * Math.PI / 180, elbow = (o.elbow ?? 12) * Math.PI / 180;
  setQ(r, B.LeftArm, qaxis([0, 0, 1], -arm));
  setQ(r, B.RightArm, qaxis([0, 0, 1], arm));
  setQ(r, B.LeftForeArm, qaxis([0, -1, 0], elbow));
  setQ(r, B.RightForeArm, qaxis([0, 1, 0], elbow));
  return r;
}

export function boneLengthCheck(skel, W) {
  let worst = 0;
  for (let i = 1; i < NB; i++) worst = Math.max(worst, Math.abs(dist(getV(W.pos, i), getV(W.pos, PARENT[i])) - len(skel.offsets[i])));
  return worst;
}

export const restPose = skel => fk(skel, skel.rest[0], restLocal());
