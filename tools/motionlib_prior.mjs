#!/usr/bin/env node
// Builds the library-learned motion prior (wire/prior.js model) from the committed motion library and validates it.
//   node tools/motionlib_prior.mjs [--lib data/MotionLibrary] [--out wire/motion_prior.json] [--no-write] [--holdout 0.2] [--verbose]
// Clips: usable, unique (index de-duplication), PAIRED rig assignment only (the retargeted set is mostly frozen and is
// excluded), never static. Each clip is segmented with phases.js (through computeFacts) and contributes per-phase frame
// values. Validation: 20% of clips held out by clip (hash of the file name), the band width kWarn is picked by 5-fold
// cross-validation on the rest, then the held-out clips report false alarms and detection of injected defects
// (hyperextended knee, 2000 deg/s pop, frozen leg, foot slide). The written model is built from all eligible clips.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { join, resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { computeFacts, wpos } from '../wire/facts.js';
import { priorFeatures, buildPrior, priorFindings } from '../wire/prior.js';
import { plantFindings, kneeFindings } from '../wire/judge.js';
import { loadClipText } from './motionlib_clip.mjs';

const args = process.argv.slice(2), flag = (k, d) => { const i = args.indexOf(k); return i >= 0 ? args[i + 1] : d; }, has = k => args.includes(k);
const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const LIB = resolve(flag('--lib', join(ROOT, 'data', 'MotionLibrary'))), OUT = resolve(flag('--out', join(ROOT, 'wire', 'motion_prior.json')));
const HOLD = +flag('--holdout', 0.2), VERBOSE = has('--verbose');
if (!existsSync(join(LIB, 'index', 'index.json'))) { console.error('no index: run tools/motionlib_index.mjs first (' + LIB + ')'); process.exit(2); }
const log = (...a) => console.log(...a);

const indexText = readFileSync(join(LIB, 'index', 'index.json'), 'utf8'), index = JSON.parse(indexText);
const reasons = {};
const eligible = index.clips.filter(c => {
  const why = !c.usable ? 'not usable' : c.duplicate_of ? 'duplicate' : c.rig_assignment !== 'paired' ? 'retargeted' : c.flags.includes('static') ? 'static' : null;
  if (why) reasons[why] = (reasons[why] ?? 0) + 1;
  return !why;
});
log(`eligible clips: ${eligible.length} of ${index.clips.length} (excluded: ${Object.entries(reasons).map(([k, v]) => `${k} ${v}`).join(', ')})`);

const holdOf = c => createHash('sha1').update(c.file).digest()[0] / 256 < HOLD;
const loaded = [];
const t0 = Date.now();
for (const rec of eligible) {
  const c = loadClipText(readFileSync(join(LIB, 'Animations', rec.file), 'utf8'));
  if (!c.ok || c.n < 8) { reasons['unmapped or short'] = (reasons['unmapped or short'] ?? 0) + 1; continue; }
  const facts = computeFacts({ W: c.W, nb: c.nb, n: c.n, fps: c.fps, upm: c.upm, rig: c.rig, restW: c.restW });
  loaded.push({ rec, clip: c, facts, features: priorFeatures({ W: c.W, nb: c.nb, n: c.n, fps: c.fps, rig: c.rig, facts }), hold: holdOf(rec) });
}
log(`segmented and measured ${loaded.length} clips in ${((Date.now() - t0) / 1000).toFixed(1)} s`);

const anyFinding = (model, x) => priorFindings({ model, facts: x.facts, W: x.clip.W, nb: x.clip.nb, features: x.features });
const CLEAN_BAD = ['foot_skate', 'rapid_rotation', 'teleport', 'rotation_flip', 'bone_length_drift', 'knee_hyperextension', 'in_place_treadmill'];
const isClean = x => !x.rec.flags.some(f => CLEAN_BAD.includes(f));

// ---------- kWarn by 5-fold CV on the training clips ----------
const train = loaded.filter(x => !x.hold), held = loaded.filter(x => x.hold);
const K_GRID = [0.25, 0.5, 0.75, 1, 1.25, 1.5, 2, 2.5, 3], TARGET_KIND = 0.015;
const folds = 5, foldOf = x => createHash('sha1').update('fold' + x.rec.file).digest()[0] % folds;
const cvFA = {};
for (const k of K_GRID) cvFA[k] = { clips: 0, flagged: 0 };
for (let fo = 0; fo < folds; fo++) {
  const tr = train.filter(x => foldOf(x) !== fo), te = train.filter(x => foldOf(x) === fo);
  const base = buildPrior(tr);
  for (const k of K_GRID) {
    const m = { ...base, params: { ...base.params, kWarn: k, kBlock: Math.max(3, 2 * k), kWarnSpeed: k, kBlockSpeed: Math.max(3, 2 * k) } };
    for (const x of te.filter(isClean)) {
      const fs = anyFinding(m, x), c = cvFA[k];
      c.clips++;
      if (fs.length) { c.flagged++; for (const r of new Set(fs.map(f => f.rule))) (c.rules ??= {})[r] = (c.rules[r] ?? 0) + 1; }
      for (const kind of ['speed', 'pose']) if (fs.some(f => f.kind === kind)) c[kind] = (c[kind] ?? 0) + 1;
    }
  }
}
// Each band family (joint speeds, poses) gets the narrowest width whose own cross-validated clip false-alarm rate is
// at most TARGET_KIND; the fixed-limit rules (knee hyperextension, frozen leg, sole slide, gait timing) add the rest.
const pick = kind => K_GRID.find(k => (cvFA[k][kind] ?? 0) / cvFA[k].clips <= TARGET_KIND) ?? K_GRID[K_GRID.length - 1];
const kWarnSpeed = pick('speed'), kWarn = pick('pose'), kBlock = Math.max(3, 2 * kWarn), kBlockSpeed = Math.max(3, 2 * kWarnSpeed);
log('cross-validated clip false-alarm rate by band width k (any / speed / pose): ' + K_GRID.map(k => `${k}: ${(100 * cvFA[k].flagged / cvFA[k].clips).toFixed(1)}/${(100 * (cvFA[k].speed ?? 0) / cvFA[k].clips).toFixed(1)}/${(100 * (cvFA[k].pose ?? 0) / cvFA[k].clips).toFixed(1)}%`).join(', ') + ` -> kWarnSpeed ${kWarnSpeed}, kWarn ${kWarn}`);
if (VERBOSE) for (const k of K_GRID) log(`  k ${k}: ` + Object.entries(cvFA[k].rules ?? {}).map(([r, v]) => `${r} ${v}`).join(', '));

// ---------- held-out validation ----------
const valModel = (() => { const m = buildPrior(train, { kWarn, kBlock, kWarnSpeed, kBlockSpeed }); return m; })();
const heldClean = held.filter(isClean);
const fa = { clips: heldClean.length, warned: 0, blocked: 0, rules: {} }, faAll = { clips: held.length, warned: 0, blocked: 0 };
for (const x of held) {
  const fs = anyFinding(valModel, x), clean = isClean(x);
  if (fs.length) faAll.warned++;
  if (fs.some(f => f.severity === 'blocker')) faAll.blocked++;
  if (!clean) continue;
  if (fs.length) { fa.warned++; for (const f of fs) fa.rules[f.rule] = (fa.rules[f.rule] ?? 0) + 1; if (VERBOSE) log(`  false alarm ${x.rec.name}: ${fs.map(f => f.text).join(' | ')}`); }
  if (fs.some(f => f.severity === 'blocker')) fa.blocked++;
}
log(`held-out clips: ${held.length} (${heldClean.length} clean). Clean clips with any prior finding: ${fa.warned} (${(100 * fa.warned / Math.max(1, fa.clips)).toFixed(1)}%), with a blocker: ${fa.blocked}. All held-out: ${faAll.warned} warned, ${faAll.blocked} blocked.` + (fa.warned ? ' Rules: ' + Object.entries(fa.rules).map(([k, v]) => `${k} ${v}`).join(', ') : ''));

// ---------- injected defects ----------
const kids = c => { const k = c.parents.map(() => []); c.parents.forEach((p, i) => { if (p >= 0) k[p].push(i); }); return k; };
const subtree = (c, i) => { const K = kids(c), out = [], st = [i]; while (st.length) { const j = st.pop(); out.push(j); st.push(...K[j]); } return out; };
function rotMat(axis, deg) {
  const [x, y, z] = axis, a = deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), t = 1 - c;
  return [t * x * x + c, t * x * y + s * z, t * x * z - s * y, t * x * y - s * z, t * y * y + c, t * y * z + s * x, t * x * z + s * y, t * y * z - s * x, t * z * z + c];
}
// Rotate a subtree of world matrices about a world pivot (R column-major 3x3).
function rotateAbout(W, nb, f, bones, pivot, R) {
  for (const i of bones) {
    const o = (f * nb + i) * 16;
    for (let col = 0; col < 3; col++) { const v = [W[o + col * 4], W[o + col * 4 + 1], W[o + col * 4 + 2]]; for (let r = 0; r < 3; r++) W[o + col * 4 + r] = R[r] * v[0] + R[3 + r] * v[1] + R[6 + r] * v[2]; }
    const p = [W[o + 12] - pivot[0], W[o + 13] - pivot[1], W[o + 14] - pivot[2]];
    for (let r = 0; r < 3; r++) W[o + 12 + r] = pivot[r] + R[r] * p[0] + R[3 + r] * p[1] + R[6 + r] * p[2];
  }
}
const nrm = v => { const l = Math.hypot(...v) || 1; return v.map(x => x / l); };
const crs = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const ramp = (f, a, b, e = 3) => Math.min(1, (f - a + 1) / e, (b - f + 1) / e);

// judge: the rules the existing judge (facts-only parts) would raise for the same defect, for comparison.
const DEFECTS = {
  hyperextended_knee: { rules: ['unnatural_knee_hyperextension'], judge: f => kneeFindings(f).filter(x => x.rule === 'knee_hyperextension'), make(x) {
    const c = x.clip, s = 'l', th = c.rig.thigh_l, ca = c.rig.calf_l, fo = c.rig.foot_l, flex = x.facts.knees.l.flex, n = c.n;
    let a = -1;
    for (let f = 2; f + 14 < n && a < 0; f++) { let ok = true; for (let g = f; g < f + 12; g++) if (!(flex[g] > 12)) ok = false; if (ok) a = f; }
    if (a < 0) return null;
    const b = a + 11, W = Float32Array.from(c.W), bones = subtree(c, ca);
    for (let f = a; f <= b; f++) {
      const H = wpos(W, c.nb, f, th), K = wpos(W, c.nb, f, ca), A = wpos(W, c.nb, f, fo), ax = nrm(crs(nrm(K.map((v, k) => v - H[k])), nrm(A.map((v, k) => v - K[k]))));
      rotateAbout(W, c.nb, f, bones, K, rotMat(ax, -ramp(f, a, b) * (flex[f] + 25)));
    }
    return { W, a, b, side: s };
  } },
  pop_2000dps: { rules: ['knee', 'hip', 'elbow', 'shoulder', 'spine', 'neck', 'wrist', 'ankle'].flatMap(g => [`unnatural_${g}_speed`, `unnatural_${g}_snap`]), make(x, k) {
    const c = x.clip, names = ['upperarm_l', 'calf_r', 'spine_03', 'lowerarm_r', 'thigh_l', 'head'], bone = c.rig[names[k % names.length]] ?? c.rig.upperarm_l;
    if (bone === undefined || c.n < 10) return null;
    const f = c.n >> 1, W = Float32Array.from(c.W), o = (f * c.nb + bone) * 16, X = nrm([W[o], W[o + 1], W[o + 2]]);
    rotateAbout(W, c.nb, f, subtree(c, bone), wpos(W, c.nb, f, bone), rotMat(X, 2000 / c.fps));
    return { W, a: f, b: f };
  } },
  frozen_leg: { rules: ['unnatural_frozen_limb'], make(x) {
    const c = x.clip, n = c.n, len = Math.min(Math.round(1.5 * c.fps), n - 4);
    const gaitFrames = x.facts.phaseAt.filter(p => p === 'locomotion' || p === 'in_place').length;
    if (gaitFrames < 0.6 * n || len < 0.7 * c.fps) return null;
    const a = Math.max(1, (n - len) >> 1), b = a + len - 1, W = Float32Array.from(c.W), pel = c.rig.pelvis, bones = subtree(c, c.rig.thigh_l);
    const inv = m => { const R = [m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]], t = [m[12], m[13], m[14]]; return { R, t: [-(R[0] * t[0] + R[3] * t[1] + R[6] * t[2]), -(R[1] * t[0] + R[4] * t[1] + R[7] * t[2]), -(R[2] * t[0] + R[5] * t[1] + R[8] * t[2])] }; };
    const mul = (A, B) => { const o = new Float64Array(16); for (let cc = 0; cc < 4; cc++) for (let r = 0; r < 4; r++) { let s = 0; for (let k = 0; k < 4; k++) s += A[k * 4 + r] * B[cc * 4 + k]; o[cc * 4 + r] = s; } return o; };
    const M = (f, i) => Array.from(c.W.subarray((f * c.nb + i) * 16, (f * c.nb + i) * 16 + 16));
    const pa = inv(M(a, pel)), PA = [pa.R[0], pa.R[1], pa.R[2], 0, pa.R[3], pa.R[4], pa.R[5], 0, pa.R[6], pa.R[7], pa.R[8], 0, pa.t[0], pa.t[1], pa.t[2], 1];
    for (let f = a; f <= b; f++) { const rel = mul(M(f, pel), PA); for (const i of bones) W.set(mul(rel, M(a, i)), (f * c.nb + i) * 16); }
    return { W, a, b, side: 'l' };
  } },
  foot_slide: { rules: ['unnatural_plant_drift', 'unnatural_foot_slide'], judge: f => plantFindings(f), make(x) {
    const c = x.clip, p = [...x.facts.contacts.feet.l.plants, ...x.facts.contacts.feet.r.plants].filter(q => q.lockable && q.b - q.a >= 12).sort((u, v) => (v.b - v.a) - (u.b - u.a))[0];
    if (!p) return null;
    // 15 cm over 8 frames inside the plant (the body carries on from there): a planted foot that skids.
    const W = Float32Array.from(c.W), d = 15 * c.upm / 100, a = p.a + 2, b = a + 7;
    for (let f = a; f < c.n; f++) { const t = Math.min(1, (f - a) / (b - a)), off = d * t * t * (3 - 2 * t); for (let i = 0; i < c.nb; i++) W[(f * c.nb + i) * 16 + 12] += off; }
    return { W, a, b, side: p.side };
  } },
};
const det = {};
for (const [name, D] of Object.entries(DEFECTS)) {
  const r = det[name] = { cases: 0, detected: 0, anyPrior: 0, blockers: 0, ...(D.judge ? { judge: 0 } : {}) };
  heldClean.forEach((x, k) => {
    const inj = D.make(x, k);
    if (!inj) return;
    const c = x.clip, facts = computeFacts({ W: inj.W, nb: c.nb, n: c.n, fps: c.fps, upm: c.upm, rig: c.rig, restW: c.restW });
    const fs = priorFindings({ model: valModel, facts, W: inj.W, nb: c.nb }).filter(f => f.a <= inj.b + 3 && f.b >= inj.a - 3);
    r.cases++;
    if (fs.length) r.anyPrior++;
    if (D.judge && D.judge(facts).some(f => f.a <= inj.b + 3 && f.b >= inj.a - 3)) r.judge++;
    if (fs.some(f => D.rules.includes(f.rule))) r.detected++;
    else if (VERBOSE) {
      const F = priorFeatures({ W: inj.W, nb: c.nb, n: c.n, fps: c.fps, rig: c.rig, facts }), ph = facts.phaseAt[inj.a];
      const peaks = Object.entries(F.tracks).map(([key, tr]) => { const base = x.features.tracks[key]; let d = 0, at = inj.a; for (let f = Math.max(0, inj.a - 2); f <= Math.min(c.n - 1, inj.b + 2); f++) if (Math.abs(tr[f] - (base?.[f] ?? 0)) > d) { d = Math.abs(tr[f] - (base?.[f] ?? 0)); at = f; } const e = valModel.env[key.replace(/_(l|r)$/, '')]; const en = e?.[facts.phaseAt[at]] ?? e?.all; return [key, d, tr[at], en?.q[4], en?.q[2]]; }).sort((u, v) => v[1] - u[1]).slice(0, 2);
      log(`  missed ${name} in ${x.rec.name} f${inj.a}-${inj.b} (${ph}): ${fs.map(f => f.rule).join(', ') || 'nothing'}; biggest change ${peaks.map(p => `${p[0]} ${p[2]?.toFixed(1)} (q995 ${p[3]}, q50 ${p[4]})`).join(', ')}` + (name === 'foot_slide' ? ` plants ${F.plants.filter(p => p.a <= inj.b && p.b >= inj.a).map(p => `${p.side}${p.a}-${p.b} ${p.drift_cm.toFixed(1)}cm ${p.phase} lim ${(valModel.env.plant_drift[p.phase] ?? valModel.env.plant_drift.all)?.q[4]}`).join(' ')}` : ''));
    }
    if (fs.some(f => f.severity === 'blocker')) r.blockers++;
  });
  log(`injected ${name}: ${r.detected}/${r.cases} detected by its rule (${(100 * r.detected / Math.max(1, r.cases)).toFixed(0)}%), any prior finding ${r.anyPrior}/${r.cases}, blocker ${r.blockers}/${r.cases}` + (D.judge ? `; existing judge rules catch ${r.judge}/${r.cases}` : ''));
}

// ---------- final model from every eligible clip ----------
const model = buildPrior(loaded, { kWarn, kBlock, kWarnSpeed, kBlockSpeed });
const perPhase = {}, framesPerPhase = {};
for (const x of loaded) { const seen = new Set(x.features.phaseAt); for (const p of seen) perPhase[p] = (perPhase[p] ?? 0) + 1; for (const p of x.features.phaseAt) framesPerPhase[p] = (framesPerPhase[p] ?? 0) + 1; }
const fam = {}; for (const x of loaded) fam[x.rec.rig_family] = (fam[x.rec.rig_family] ?? 0) + 1;
const knee = loaded.flatMap(x => ['l', 'r'].flatMap(s => Array.from(x.features.tracks['knee_flex_' + s] ?? []))).sort((a, b) => a - b);
const idleKnee = loaded.filter(x => x.rec.category === 'idle').flatMap(x => Array.from(x.features.tracks.knee_flex_l ?? [])).sort((a, b) => a - b);
model.knee = { library_min_deg: +knee[0].toFixed(2), library_p2_deg: +knee[Math.round(0.02 * (knee.length - 1))].toFixed(2), idle_median_deg: idleKnee.length ? +idleKnee[idleKnee.length >> 1].toFixed(1) : null,
  index_clamp_deg: 0.18, note: 'library knees never pass straight (rig clamp ~0.18 deg) and lean bent; knees are judged on over-bend, speed and a fixed hyperextension limit only, never on a minimum bend' };
model.provenance = {
  generated_by: 'tools/motionlib_prior.mjs', library: 'data/MotionLibrary', index_sha256: createHash('sha256').update(indexText).digest('hex'),
  source_sha256: createHash('sha256').update(loaded.map(x => x.rec.sha256).sort().join('\n')).digest('hex'),
  clips: loaded.length, frames: loaded.reduce((a, x) => a + x.clip.n, 0), rig_families: fam, excluded: reasons,
  selection: 'usable, unique, rig_assignment = paired, not static', clips_per_phase: perPhase, frames_per_phase: framesPerPhase,
  validation: { holdout: HOLD, train_clips: train.length, held_clips: held.length, held_clean_clips: heldClean.length, cv_false_alarm: Object.fromEntries(K_GRID.map(k => [k, { any: +(cvFA[k].flagged / cvFA[k].clips).toFixed(3), speed: +((cvFA[k].speed ?? 0) / cvFA[k].clips).toFixed(3), pose: +((cvFA[k].pose ?? 0) / cvFA[k].clips).toFixed(3) }])),
    kWarn, kBlock, kWarnSpeed, kBlockSpeed, held_clean_false_alarm: +(fa.warned / Math.max(1, fa.clips)).toFixed(3), held_clean_blockers: fa.blocked, false_alarm_rules: fa.rules, injected: det },
};
const text = JSON.stringify(model);
log(`model: ${(text.length / 1024).toFixed(1)} KB, ${Object.keys(model.env).length} features, phases with own envelopes: ${Object.keys(model.env.knee_flex).filter(k => k !== 'all' && !k.startsWith('fam:')).join(', ')}`);
if (!has('--no-write')) { writeFileSync(OUT, text + '\n'); log('wrote ' + OUT); }
