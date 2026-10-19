import { qrot, qconj, qmul } from './story.js';

export const DEFAULT_AUX_REACH = 100;

export function wristFromEffector(effT, effQ, pivotOffset) {
  const r = qrot(effQ, pivotOffset);
  return [effT[0] + r[0], effT[1] + r[1], effT[2] + r[2]];
}

export function combineEffectors(sets) {
  const t = [0, 0, 0];
  const q = [0, 0, 0, 0];
  let sumT = 0;
  let sumR = 0;
  let maxT = 0;
  let maxR = 0;
  let ref = null;
  for (const s of sets) {
    const rt = Math.max(0, Math.min(100, s.reachT ?? 0)) / 100;
    const rr = Math.max(0, Math.min(100, s.reachR ?? 0)) / 100;
    if (rt > 0) {
      for (let i = 0; i < 3; i++) t[i] += rt * s.t[i];
      sumT += rt;
      maxT = Math.max(maxT, rt);
    }
    if (rr > 0) {
      ref = ref ?? s.q;
      const d = ref[0] * s.q[0] + ref[1] * s.q[1] + ref[2] * s.q[2] + ref[3] * s.q[3];
      const sg = d < 0 ? -1 : 1;
      for (let i = 0; i < 4; i++) q[i] += sg * rr * s.q[i];
      sumR += rr;
      maxR = Math.max(maxR, rr);
    }
  }
  const ql = Math.hypot(...q);
  return {
    t: sumT > 0 ? t.map((x) => x / sumT) : null,
    q: sumR > 0 && ql > 0 ? q.map((x) => x / ql) : null,
    reachT: maxT,
    reachR: maxR,
  };
}

export function auxRelative(mainT, mainQ, auxT, auxQ) {
  const iq = qconj(mainQ);
  const d = [auxT[0] - mainT[0], auxT[1] - mainT[1], auxT[2] - mainT[2]];
  return { t: qrot(iq, d), q: qmul(iq, auxQ) };
}
