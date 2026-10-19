import { qslerp } from './advanced.js';

const read = (q, nb, f, i) => Array.from(q.subarray((f * nb + i) * 4, (f * nb + i + 1) * 4));
const angle = (a, b) => 2 * Math.acos(Math.min(1, Math.abs(a.reduce((s, x, k) => s + x * b[k], 0))));

export function suggestPoses({ source, current, nb, n, f, guides = [], preserve = [], count = 3, neighbors = 5, context = 0.1 }) {
  if (!source || source.length !== n * nb * 4 || current.length !== source.length) throw new Error('Pose model and target skeleton do not match.');
  if (!guides.length) return { model: 'motion-library-neighbors-v1', suggestions: [], reason: 'Move or select a control to guide the pose.' };
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
    return { pose, sourceFrame: seed.frame, guidanceErrorDeg: seed.error * 180 / Math.PI, uncertaintyDeg: uncertainty * 180 / Math.PI, confidence: Math.max(0, Math.min(1, Math.exp(-seed.error * 2 - uncertainty))), preserved: [...locked] };
  });
  return { model: 'motion-library-neighbors-v1', trainingFrames: n, suggestions, reason: null };
}
