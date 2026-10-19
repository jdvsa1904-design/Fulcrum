// Phase segmentation (phases.js) on the committed motion library: every usable, unique clip is segmented and its
// majority phase checked against the index's category label (data/MotionLibrary/index/index.json). Prints the
// per-category accuracy. PHASES_MODULE=<path to another phases.js> evaluates that file instead (for before/after).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { phaseInput } from '../wire/facts.js';
import { loadClip } from '../tools/motionlib_clip.mjs';

const LIB = new URL('../data/MotionLibrary/', import.meta.url);
const INDEX = new URL('index/index.json', LIB);
const ANIM = new URL('Animations/', LIB).pathname;
const mod = await import(process.env.PHASES_MODULE ? pathToFileURL(process.env.PHASES_MODULE).href : '../phases.js');

const GAIT = ['locomotion', 'in_place'];
// Accepted majority phases per category; crouch-tagged clips may also be 'crouch', and crouch-tagged turns must not be
// 'kneel'. Jump clips with airborne frames (index feet.longest_air_frames) must hold most of them as flight.
export const EXPECT = {
  idle: ['stand', 'upper_body'], walk: GAIT, run: GAIT, sprint: GAIT, strafe: GAIT,
  turn: ['in_place', 'stand', 'upper_body', 'locomotion', 'freeform'], prone_crawl: ['prone'],
  // CrouchLoop_new and Rifle_CrouchLoop hold one knee on the floor (calf joint ~4 cm up, not moving): kneel is right there.
  crouch: ['crouch', 'kneel'],
};
// Minimum accuracy per category after the fix (the before numbers are in the PR notes).
const MIN_ACC = { idle: 0.9, walk: 0.95, run: 0.9, sprint: 0.9, strafe: 0.95, turn: 0.9, crouch: 1, prone_crawl: 0.95, jump: 1, overall: 0.95 };

export function phasesOfClip(c, segmentPhases) {
  const input = phaseInput({ W: c.W, nb: c.nb, n: c.n, fps: c.fps, upm: c.upm, rig: c.rig, restW: c.restW });
  if (!input) return null;
  const segs = segmentPhases(input), tally = {};
  for (const s of segs) tally[s.phase] = (tally[s.phase] ?? 0) + s.end - s.start + 1;
  return { segs, tally, major: Object.entries(tally).sort((a, b) => b[1] - a[1])[0]?.[0] ?? null };
}

export function judgeClip(rec, r) {
  const cat = rec.category, crouchy = (rec.tags || []).includes('crouch');
  if (cat === 'jump') return rec.feet?.longest_air_frames >= 4 ? (r.tally.flight ?? 0) >= 0.75 * rec.feet.longest_air_frames : null;
  const ok = EXPECT[cat];
  if (!ok) return null;
  if (cat === 'turn' && crouchy && r.major === 'kneel') return false;
  return ok.includes(r.major) || (crouchy && r.major === 'crouch');
}

test('phases.js on the motion library: per-category accuracy', { skip: !existsSync(INDEX) && 'motion library not present' }, () => {
  const idx = JSON.parse(readFileSync(INDEX, 'utf8'));
  const clips = idx.clips.filter(c => c.usable && !c.duplicate_of && (EXPECT[c.category] || c.category === 'jump'));
  const acc = {}, wrong = [];
  for (const rec of clips) {
    const c = loadClip(ANIM, rec.file);
    if (!c.ok || c.n < 6) continue;
    const r = phasesOfClip(c, mod.segmentPhases);
    if (!r) continue;
    const ok = judgeClip(rec, r);
    if (ok === null) continue;
    const a = acc[rec.category] ??= { n: 0, ok: 0 };
    a.n++; if (ok) a.ok++; else wrong.push(`${rec.category} ${rec.name}: ${r.major} (${Object.entries(r.tally).map(([k, v]) => k + ' ' + v).join(', ')})`);
  }
  const tot = Object.values(acc).reduce((s, a) => ({ n: s.n + a.n, ok: s.ok + a.ok }), { n: 0, ok: 0 });
  const rows = Object.entries(acc).map(([k, a]) => `${k.padEnd(12)} ${String(a.ok).padStart(4)}/${String(a.n).padEnd(4)} ${(100 * a.ok / a.n).toFixed(0)}%`);
  console.log(`phases on the motion library (${process.env.PHASES_MODULE || 'phases.js'}):\n  ` + rows.join('\n  ') + `\n  overall      ${tot.ok}/${tot.n} ${(100 * tot.ok / tot.n).toFixed(1)}%`);
  if (process.env.PHASES_VERBOSE) console.log('  misses:\n   ' + wrong.join('\n   '));
  if (process.env.PHASES_MODULE) return;
  for (const [k, a] of Object.entries(acc)) assert.ok(a.ok / a.n >= MIN_ACC[k], `${k}: ${a.ok}/${a.n} below ${MIN_ACC[k]}`);
  assert.ok(tot.ok / tot.n >= MIN_ACC.overall, `overall ${tot.ok}/${tot.n}`);
});
