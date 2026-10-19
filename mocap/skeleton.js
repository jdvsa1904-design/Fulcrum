import { add, sub, scale, qmul, qrot, qaxis, clamp, median, dist, mid, len } from './math.js';

export const BONES = [
  ['Hips', -1], ['Spine', 0], ['Spine1', 1], ['Spine2', 2], ['Neck', 3], ['Head', 4],
  ['LeftShoulder', 3], ['LeftArm', 6], ['LeftForeArm', 7], ['LeftHand', 8],
  ['RightShoulder', 3], ['RightArm', 10], ['RightForeArm', 11], ['RightHand', 12],
  ['LeftUpLeg', 0], ['LeftLeg', 14], ['LeftFoot', 15], ['LeftToeBase', 16],
  ['RightUpLeg', 0], ['RightLeg', 18], ['RightFoot', 19], ['RightToeBase', 20],
].map(([name, parent], index) => ({ name, parent, index }));

export const NB = BONES.length;
export const B = Object.fromEntries(BONES.map(b => [b.name, b.index]));

export const LM = {
  nose: 0, leftEyeInner: 1, leftEye: 2, leftEyeOuter: 3, rightEyeInner: 4, rightEye: 5, rightEyeOuter: 6,
  leftEar: 7, rightEar: 8, mouthLeft: 9, mouthRight: 10, leftShoulder: 11, rightShoulder: 12,
  leftElbow: 13, rightElbow: 14, leftWrist: 15, rightWrist: 16, leftPinky: 17, rightPinky: 18,
  leftIndex: 19, rightIndex: 20, leftThumb: 21, rightThumb: 22, leftHip: 23, rightHip: 24,
  leftKnee: 25, rightKnee: 26, leftAnkle: 27, rightAnkle: 28, leftHeel: 29, rightHeel: 30,
  leftFootIndex: 31, rightFootIndex: 32,
};
export const NLM = 33;

export const DEFAULT_MEASURE = {
  hipWidth: 0.18, thigh: 0.44, shin: 0.43, ankleHeight: 0.075, footLength: 0.25,
  torso: 0.5, shoulderWidth: 0.34, upperArm: 0.29, forearm: 0.26, hand: 0.09, neck: 0.17, headWidth: 0.15,
};

const RANGE = {
  hipWidth: [0.6, 1.6], thigh: [0.75, 1.3], shin: [0.75, 1.3], ankleHeight: [0.6, 1.5], footLength: [0.7, 1.4],
  torso: [0.75, 1.3], shoulderWidth: [0.7, 1.4], upperArm: [0.75, 1.3], forearm: [0.75, 1.3], hand: [0.6, 1.6], neck: [0.6, 1.5], headWidth: [0.7, 1.4],
};

export function measureFromLandmarks(frames) {
  const acc = Object.fromEntries(Object.keys(DEFAULT_MEASURE).map(k => [k, []]));
  const both = (f, a, b, c, d) => (dist(f[a], f[b]) + dist(f[c], f[d])) / 2;
  for (const f of frames) {
    if (!f) continue;
    acc.hipWidth.push(dist(f[LM.leftHip], f[LM.rightHip]));
    acc.thigh.push(both(f, LM.leftHip, LM.leftKnee, LM.rightHip, LM.rightKnee));
    acc.shin.push(both(f, LM.leftKnee, LM.leftAnkle, LM.rightKnee, LM.rightAnkle));
    acc.footLength.push(both(f, LM.leftHeel, LM.leftFootIndex, LM.rightHeel, LM.rightFootIndex));
    const fl = acc.footLength[acc.footLength.length - 1], ah = both(f, LM.leftAnkle, LM.leftHeel, LM.rightAnkle, LM.rightHeel);
    acc.ankleHeight.push(Math.sqrt(Math.max(ah * ah - (0.25 * fl) ** 2, 0.0016)));
    acc.torso.push(dist(mid(f[LM.leftHip], f[LM.rightHip]), mid(f[LM.leftShoulder], f[LM.rightShoulder])));
    acc.shoulderWidth.push(dist(f[LM.leftShoulder], f[LM.rightShoulder]));
    acc.upperArm.push(both(f, LM.leftShoulder, LM.leftElbow, LM.rightShoulder, LM.rightElbow));
    acc.forearm.push(both(f, LM.leftElbow, LM.leftWrist, LM.rightElbow, LM.rightWrist));
    acc.hand.push((dist(f[LM.leftWrist], mid(f[LM.leftIndex], f[LM.leftPinky])) + dist(f[LM.rightWrist], mid(f[LM.rightIndex], f[LM.rightPinky]))) / 2);
    acc.neck.push(dist(mid(f[LM.leftShoulder], f[LM.rightShoulder]), mid(f[LM.leftEar], f[LM.rightEar])));
    acc.headWidth.push(dist(f[LM.leftEar], f[LM.rightEar]));
  }
  const raw = Object.fromEntries(Object.entries(acc).map(([k, v]) => [k, median(v)]));
  const legs = (raw.thigh + raw.shin) / (DEFAULT_MEASURE.thigh + DEFAULT_MEASURE.shin);
  const s = Number.isFinite(legs) && legs > 0.3 ? clamp(legs, 0.5, 1.5) : 1;
  const out = {};
  for (const k of Object.keys(DEFAULT_MEASURE)) {
    const d = DEFAULT_MEASURE[k] * s, v = raw[k];
    out[k] = Number.isFinite(v) ? clamp(v, d * RANGE[k][0], d * RANGE[k][1]) : d;
  }
  return out;
}

export function buildSkeleton(measure = DEFAULT_MEASURE) {
  const m = { ...DEFAULT_MEASURE, ...measure };
  const P = new Array(NB);
  const hipY = m.ankleHeight + m.shin + m.thigh;
  P[B.Hips] = [0, hipY, 0];
  P[B.Spine] = [0, hipY + 0.12 * m.torso, -0.01];
  P[B.Spine1] = [0, hipY + 0.4 * m.torso, -0.015];
  P[B.Spine2] = [0, hipY + 0.68 * m.torso, -0.01];
  P[B.Neck] = [0, hipY + m.torso + 0.03 * m.neck, -0.01];
  P[B.Head] = [0, hipY + m.torso + 0.62 * m.neck, 0];
  for (const [side, sx] of [['Left', 1], ['Right', -1]]) {
    P[B[side + 'Shoulder']] = [sx * 0.025, hipY + m.torso - 0.035, 0];
    P[B[side + 'Arm']] = [sx * m.shoulderWidth / 2, hipY + m.torso, 0];
    P[B[side + 'ForeArm']] = [sx * (m.shoulderWidth / 2 + m.upperArm), hipY + m.torso, 0];
    P[B[side + 'Hand']] = [sx * (m.shoulderWidth / 2 + m.upperArm + m.forearm), hipY + m.torso, 0];
    P[B[side + 'UpLeg']] = [sx * m.hipWidth / 2, hipY, 0];
    P[B[side + 'Leg']] = [sx * m.hipWidth / 2, hipY - m.thigh, 0.005];
    P[B[side + 'Foot']] = [sx * m.hipWidth / 2, m.ankleHeight, 0];
    P[B[side + 'ToeBase']] = [sx * m.hipWidth / 2, 0.022, 0.52 * m.footLength];
  }
  const offsets = BONES.map(b => b.parent < 0 ? P[b.index].slice() : sub(P[b.index], P[b.parent]));
  const legLength = m.thigh + m.shin + m.ankleHeight;
  return { bones: BONES, measure: m, rest: P, offsets, hipHeight: hipY, legLength, height: hipY + m.torso + m.neck + 0.75 * m.headWidth };
}

export function fk(skel, root, local) {
  const n = NB, wq = new Array(n), wp = new Array(n);
  for (let i = 0; i < n; i++) {
    const p = BONES[i].parent, q = local[i];
    if (p < 0) { wq[i] = q; wp[i] = root; continue; }
    wq[i] = qmul(wq[p], q);
    wp[i] = add(wp[p], qrot(wq[p], skel.offsets[i]));
  }
  return { wq, wp };
}

export function restLocal() { return BONES.map(() => [0, 0, 0, 1]); }

export function landmarksFromPose(skel, pose) {
  const { wq, wp } = pose, m = skel.measure, h = m.headWidth / 0.15, L = new Array(NLM);
  const at = (b, off) => add(wp[b], qrot(wq[b], off));
  const head = B.Head, eh = 0.62 * m.neck * 0 + 0.38 * m.neck;
  L[LM.nose] = at(head, scale([0, eh - 0.012, 0.105], 1).map((x, i) => i === 1 ? x : x * h));
  L[LM.leftEyeInner] = at(head, [0.015 * h, eh + 0.022, 0.085 * h]);
  L[LM.leftEye] = at(head, [0.032 * h, eh + 0.024, 0.08 * h]);
  L[LM.leftEyeOuter] = at(head, [0.048 * h, eh + 0.022, 0.072 * h]);
  L[LM.rightEyeInner] = at(head, [-0.015 * h, eh + 0.022, 0.085 * h]);
  L[LM.rightEye] = at(head, [-0.032 * h, eh + 0.024, 0.08 * h]);
  L[LM.rightEyeOuter] = at(head, [-0.048 * h, eh + 0.022, 0.072 * h]);
  L[LM.leftEar] = at(head, [m.headWidth / 2, eh, 0]);
  L[LM.rightEar] = at(head, [-m.headWidth / 2, eh, 0]);
  L[LM.mouthLeft] = at(head, [0.025 * h, eh - 0.045, 0.09 * h]);
  L[LM.mouthRight] = at(head, [-0.025 * h, eh - 0.045, 0.09 * h]);
  for (const [side, sx, s] of [['Left', 1, 'left'], ['Right', -1, 'right']]) {
    const hand = B[side + 'Hand'], hl = m.hand / 0.09;
    L[LM[s + 'Shoulder']] = wp[B[side + 'Arm']];
    L[LM[s + 'Elbow']] = wp[B[side + 'ForeArm']];
    L[LM[s + 'Wrist']] = wp[hand];
    L[LM[s + 'Index']] = at(hand, [sx * 0.095 * hl, 0, 0.022 * hl]);
    L[LM[s + 'Pinky']] = at(hand, [sx * 0.085 * hl, 0, -0.028 * hl]);
    L[LM[s + 'Thumb']] = at(hand, [sx * 0.045 * hl, -0.012 * hl, 0.045 * hl]);
    L[LM[s + 'Hip']] = wp[B[side + 'UpLeg']];
    L[LM[s + 'Knee']] = wp[B[side + 'Leg']];
    L[LM[s + 'Ankle']] = wp[B[side + 'Foot']];
    L[LM[s + 'Heel']] = at(B[side + 'Foot'], [0, -m.ankleHeight + 0.012, -0.25 * m.footLength]);
    L[LM[s + 'FootIndex']] = at(B[side + 'ToeBase'], [0, -0.008, 0.23 * m.footLength]);
  }
  return L;
}

export function footPoints(skel, pose, side) {
  const S = side === 'l' || side === 'Left' ? 'Left' : 'Right', m = skel.measure, f = B[S + 'Foot'], t = B[S + 'ToeBase'];
  const { wq, wp } = pose;
  return {
    ankle: wp[f],
    heel: add(wp[f], qrot(wq[f], [0, -m.ankleHeight, -0.25 * m.footLength])),
    ball: add(wp[t], qrot(wq[t], [0, -0.022, 0])),
    toe: add(wp[t], qrot(wq[t], [0, -0.02, 0.23 * m.footLength])),
  };
}

export function standLocal(skel, o = {}) {
  const q = restLocal(), down = (o.armDown ?? 72) * Math.PI / 180, elbow = (o.elbow ?? 12) * Math.PI / 180;
  q[B.LeftArm] = qaxis([0, 0, 1], -down);
  q[B.RightArm] = qaxis([0, 0, 1], down);
  q[B.LeftForeArm] = qaxis([0, 1, 0], -elbow);
  q[B.RightForeArm] = qaxis([0, 1, 0], elbow);
  return q;
}

export const boneLength = (skel, i) => {
  const kids = BONES.filter(b => b.parent === i);
  return kids.length ? len(skel.offsets[kids[0].index]) : 0.08;
};
