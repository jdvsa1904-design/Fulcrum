#!/usr/bin/env node
// Builds the studio pose bank from the motion library catalogue: usable, de-duplicated clips, canonical local rotations of
// wire/posebank.js BANK_BONES (int16 quaternions), resampled to a fixed rate, with per-frame root velocity, heading,
// pelvis height, foot contacts and phases.js phase, and per-clip category and provenance.
// node tools/motionlib_posebank.mjs [library dir, default data/MotionLibrary] [out prefix, default wire/pose_bank] [--fps 15] [--limit N]
// Writes <prefix>.bin and <prefix>.json. Treats the library as data only.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { join, dirname, resolve, relative } from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseBVH } from '../mocap/bvh.js';
import { clipRig, clipCanonical, resampleCanonical, newCalibration, calibrateWith } from './motionlib_bvh.mjs';
import { BANK_FORMAT, BANK_BONES, BANK_PARENT, NB, encodeBank, decodeBank, qangle, CONTEXT_WEIGHTS } from '../wire/posebank.js';

const args = process.argv.slice(2);
const flag = (k, d) => { const i = args.indexOf(k); return i >= 0 ? args[i + 1] : d; };
const pos = args.filter((a, i) => !a.startsWith('--') && !(i > 0 && args[i - 1].startsWith('--')));
const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const LIB = resolve(pos[0] ?? join(ROOT, 'data', 'MotionLibrary')), OUT = resolve(pos[1] ?? join(ROOT, 'wire', 'pose_bank'));
const INDEX = join(LIB, 'index', 'index.json');
if (!existsSync(INDEX)) { console.error('usage: node tools/motionlib_posebank.mjs [library dir] [out prefix]; no catalogue at ' + INDEX + ' (run tools/motionlib_index.mjs)'); process.exit(2); }
const FPS = +flag('--fps', 15), LIMIT = +flag('--limit', 0) || Infinity;
const indexText = readFileSync(INDEX, 'utf8'), index = JSON.parse(indexText);
const animDir = existsSync(join(LIB, 'Animations')) ? join(LIB, 'Animations') : LIB;

const phaseNames = [], phaseId = p => { let i = phaseNames.indexOf(p); if (i < 0) { i = phaseNames.length; phaseNames.push(p); } return i; };
const phaseTrack = (rec, n) => { const out = new Uint8Array(n).fill(255); for (const s of rec.fulcrum_phases ?? []) { const m = s.match(/^(.+):(\d+)-(\d+)$/); if (m) out.fill(phaseId(m[1]), +m[2], Math.min(n, +m[3] + 1)); } return out; };

const chosen = index.clips.filter(c => c.usable && !c.duplicate_of).slice(0, LIMIT), skipped = [], clips = [], frames = [];
const t0 = Date.now();
const load = rec => {
  const text = readFileSync(join(animDir, rec.file), 'utf8'), sha = createHash('sha256').update(text).digest('hex');
  if (rec.sha256 && sha !== rec.sha256) return { skip: 'file changed since indexing' };
  const cr = clipRig(parseBVH(text), rec);
  return cr ? { cr, sha } : { skip: 'bank bone unmapped' };
};
// pass 1: one calibration per skeleton hierarchy, pooled over all its clips
const cals = new Map();
for (const rec of chosen) {
  const { cr } = load(rec); if (!cr) continue;
  if (!cals.has(cr.hierarchy)) cals.set(cr.hierarchy, { cal: newCalibration(cr), family: rec.rig_family, clips: 0 });
  const h = cals.get(cr.hierarchy); calibrateWith(h.cal, cr); h.clips++;
}
const rigs = new Map([...cals].map(([k, h]) => [k, { ...h, rig: h.cal.finish() }]));
// pass 2: canonical frames, resampled
for (const rec of chosen) {
  const { cr, sha, skip } = load(rec);
  if (skip) { skipped.push({ file: rec.file, reason: skip }); continue; }
  const can = clipCanonical(cr, rigs.get(cr.hierarchy).rig);
  const res = resampleCanonical(can, FPS, phaseTrack(rec, can.n)), start = frames.length, m = res.length;
  frames.push(...res);
  clips.push({ file: rec.file, name: rec.name, category: rec.category, rig_family: rec.rig_family, hierarchy: cr.hierarchy, rig_assignment: rec.rig_assignment, tags: rec.tags, sha256: sha,
    src_fps: +can.fps.toFixed(3), src_frames: can.n, start, n: m, locomotion_speed_ms: rec.locomotion_speed_ms, in_place: rec.feet?.in_place ?? null });
}

const { bytes, layout } = encodeBank(frames);
const by = key => clips.reduce((o, c) => (o[c[key]] = (o[c[key]] ?? 0) + c.n, o), {});
const meta = {
  format: BANK_FORMAT, fps: FPS, frames: frames.length, bones: BANK_BONES, parents: BANK_PARENT, layout, phases: phaseNames, context_weights: CONTEXT_WEIGHTS,
  canonical: 'Per bone, rigid with the bone: y along the bone to its first canonical descendant; x from the hip line (pelvis), shoulder line (spine_03), knee/elbow/ankle hinge axis averaged over bent frames, index-to-pinky line (hands), else the ancestor x carried over; z = x × y. Calibrated per skeleton hierarchy from all its clips (rigs below, bone-local quaternions). World Y up; pelvis faces +Z with body left +X. Quaternions [x,y,z,w] are local to the canonical parent; the pelvis has its heading removed (root yaw). Velocities are pelvis m/s in the heading frame; contacts: low foot joint within 4 cm of the clip floor (5th percentile) and slower than 0.5 m/s.',
  rigs: Object.fromEntries([...rigs].map(([k, h]) => [k, { family: h.family, clips: h.clips, calibration_frames: h.rig.frames, O: h.rig.O.map(q => q.map(v => +v.toFixed(6))) }])),
  provenance: {
    generated_by: 'tools/motionlib_posebank.mjs', library: relative(ROOT, LIB) || '.', catalogue: relative(ROOT, INDEX), catalogue_sha256: createHash('sha256').update(indexText).digest('hex'),
    selection: 'index clips with usable=true and duplicate_of=null', resample: `slerp to ${FPS} fps from each clip's own rate`, quantization: 'int16 quaternion components / 32767, w >= 0',
    rig_mapping: 'index rigmap.canonical (wire/rigmap.js; Auto-Rig Pro via the indexer alias layer in tools/motionlib_bvh.mjs arpAlias)',
    clips_selected: chosen.length, clips_used: clips.length, skipped, frames_by_rig: by('rig_family'), frames_by_category: by('category'), bin_sha256: createHash('sha256').update(bytes).digest('hex'),
  },
  clips,
};
writeFileSync(OUT + '.bin', bytes);
writeFileSync(OUT + '.json', JSON.stringify(meta));
// round trip check against the unquantized frames
const bank = decodeBank(JSON.parse(readFileSync(OUT + '.json', 'utf8')), readFileSync(OUT + '.bin'));
let worst = 0;
for (let t = 0; t < frames.length; t++) for (let j = 0; j < NB; j++) worst = Math.max(worst, qangle(Array.from(frames[t].L.subarray(j * 4, j * 4 + 4)), Array.from(bank.C.subarray((t * NB + j) * 4, (t * NB + j) * 4 + 4))));
const kb = p => (readFileSync(p).length / 1024).toFixed(0) + ' KB';
console.log(`pose bank: ${clips.length} clips (${skipped.length} skipped), ${frames.length} frames at ${FPS} fps, ${NB} bones; ${OUT}.bin ${kb(OUT + '.bin')}, ${OUT}.json ${kb(OUT + '.json')}; worst quantization ${(worst * 180 / Math.PI).toFixed(4)} deg; ${((Date.now() - t0) / 1000).toFixed(1)} s`);
