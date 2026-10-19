import { qslerp } from './advanced.js';
import { canonFrames, canonPose, chainPose, searchBank, bankPose, studioWorld, studioLocal, qangle, qmul, qyaw, NB } from './wire/posebank.js';

const read = (q, nb, f, i) => Array.from(q.subarray((f * nb + i) * 4, (f * nb + i + 1) * 4));
const angle = (a, b) => 2 * Math.acos(Math.min(1, Math.abs(a.reduce((s, x, k) => s + x * b[k], 0))));

// Take-local neighbours, plus (with `library`) nearest poses from the motion-library pose bank (wire/posebank.js):
// library = { bank, rig, parents, rootParentQ, context, k, exclude }. Rows from both are ranked together by
// guided-joint error² + context cost, so take-local rows stay first when they fit better.
export function suggestPoses({ source, current, nb, n, f, guides = [], preserve = [], count = 3, neighbors = 5, context = 0.1, library = null }) {
  if (!source || source.length !== n * nb * 4 || current.length !== source.length) throw new Error('Pose model and target skeleton do not match.');
  const model = library ? 'motion-library-neighbors-v2' : 'motion-library-neighbors-v1';
  if (!guides.length) return { model, suggestions: [], reason: 'Move or select a control to guide the pose.' };
  const locked = new Set([...guides, ...preserve]), ranked = [];
  for (let t = 0; t < n; t++) {
    let error = 0, weight = 0;
    for (const i of guides) {
      if (i < 0 || i >= nb) throw new Error('Invalid guided joint.');
      const d = angle(read(current, nb, f, i), read(source, nb, t, i));
      error += d * d; weight++;
    }
    const contextCost = context * Math.pow((t - f) / Math.max(n, 1), 2);
    ranked.push({ frame: t, error: Math.sqrt(error / weight), score: error / weight + contextCost });
  }
  ranked.sort((a, b) => a.score - b.score || a.frame - b.frame);
  const seeds = [];
  for (const row of ranked) if (seeds.every(x => Math.abs(x.frame - row.frame) >= Math.max(2, n / 30))) { seeds.push(row); if (seeds.length >= count) break; }
  const suggestions = seeds.map(seed => {
    const near = ranked.filter(x => Math.abs(x.frame - seed.frame) <= Math.max(3, n / 20)).slice(0, neighbors);
    const pose = new Float32Array(nb * 4);
    let dispersion = 0;
    for (let i = 0; i < nb; i++) {
      let value = read(source, nb, near[0].frame, i), total = 1 / (0.01 + near[0].score);
      for (const row of near.slice(1)) { const w = 1 / (0.01 + row.score); value = qslerp(value, read(source, nb, row.frame, i), w / (total + w)); total += w; }
      if (locked.has(i)) value = read(current, nb, f, i);
      else dispersion += angle(value, read(source, nb, seed.frame, i));
      pose.set(value, i * 4);
    }
    const uncertainty = dispersion / Math.max(1, nb - locked.size);
    return { pose, source: 'take', sourceFrame: seed.frame, score: seed.score, guidanceErrorDeg: seed.error * 180 / Math.PI, uncertaintyDeg: uncertainty * 180 / Math.PI, confidence: Math.max(0, Math.min(1, Math.exp(-seed.error * 2 - uncertainty))), preserved: [...locked] };
  });
  if (!library?.bank) return { model, trainingFrames: n, suggestions, reason: null };
  const lib = librarySuggestions({ ...library, current, nb, f, guides, locked, k: library.k ?? count });
  const merged = [...suggestions, ...lib.rows].sort((a, b) => a.score - b.score);
  return { model, trainingFrames: n, libraryFrames: library.bank.N, libraryClips: library.bank.clips.length, suggestions: merged, reason: null, libraryReason: lib.reason };
}

function librarySuggestions({ bank, rig, parents, rootParentQ, context = null, exclude = [], current, nb, f, guides, locked, k }) {
  const g = [...new Set(guides.map(i => rig.idx.indexOf(i)).filter(x => x >= 0))];
  if (!g.length) return { rows: [], reason: 'None of the guided joints is a body joint the motion library knows.' };
  const W = studioWorld(current, nb, f, parents, rootParentQ), { yaw, L } = canonPose(canonFrames(rig, i => W[i]), rig.anc);
  const keep = [...locked].map(i => rig.idx.indexOf(i)).filter(x => x >= 0), free = [...Array(NB).keys()].filter(x => rig.idx[x] >= 0 && !keep.includes(x));
  const hits = searchBank(bank, { L, anc: rig.anc, guides: g, context, k, exclude }), poses = hits.map(h => bankPose(bank, h.t, rig.anc, keep, L));
  const rows = hits.map((h, j) => {
    const Fh = chainPose(poses[j], rig.anc), R = qyaw(yaw), F = new Float64Array(NB * 4);
    for (let x = 0; x < NB; x++) F.set(qmul(R, Array.from(Fh.subarray(x * 4, x * 4 + 4))), x * 4);
    const other = poses[j === 0 ? 1 : 0];
    const spread = other && free.length ? free.reduce((s, x) => s + qangle(Array.from(poses[j].subarray(x * 4, x * 4 + 4)), Array.from(other.subarray(x * 4, x * 4 + 4))), 0) / free.length : 0;
    return { pose: studioLocal(rig, F, current, nb, f, parents, rootParentQ), source: 'library', clip: h.clipName, clipFile: h.clipFile, category: h.category, phase: h.phase, clipFrame: h.clipFrame, bankFrame: h.t,
      score: h.score, contextCost: h.context, guidanceErrorDeg: h.error * 180 / Math.PI, uncertaintyDeg: spread * 180 / Math.PI,
      confidence: Math.max(0, Math.min(1, Math.exp(-h.error * 2 - Math.sqrt(h.context) - spread / 2))), preserved: [...locked] };
  });
  return { rows, reason: null };
}
