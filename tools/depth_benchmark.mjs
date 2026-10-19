#!/usr/bin/env node
// Baseline-vs-new depth benchmark and gate verdicts (spec §11, §15).
// node tools/depth_benchmark.mjs [--split dev|test] [--final] [--seeds 8] [--families all] [--profiles none,typical,hard] [--redteam] [--coverage 1000]
//   [--workers N] [--out file.json] [--head <git sha>] [--quick] [--no-runtime] [--tests]
import { isMainThread, parentPort, workerData } from 'node:worker_threads';
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync, appendFileSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, resolve } from 'node:path';
import os from 'node:os';
import * as DB from '../mocap/depthbench.js';
import { certifyTake, trueSigns, CERT_GROUPS, EVAL_JOINTS, STATE } from '../mocap/depthcert.js';
import { clusterBootstrap, cpUpper, mulberry32 } from '../mocap/depthstats.js';
import { DEPTH_CAL } from '../mocap/depth_calibration.js';
import { NE } from '../mocap/depth.js';
import { synthCapture, lookAtCamera } from '../mocap/synth.js';
import { buildSkeleton, DEFAULT_DIMS } from '../mocap/skeleton.js';
import { gaitClip, idleClip, jumpClip } from '../mocap/gait.js';
import { createController, recordTake } from '../mocap/controller.js';
import { runPool, parseArgs, codeHash, calContext } from './depth_calibrate.mjs';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..'), GROUPS = Object.keys(CERT_GROUPS), THETAS = [0, 45, 90, 135, 180];
const PIPE = process.env.DEPTH_PIPELINE ? pathToFileURL(resolve(process.env.DEPTH_PIPELINE)).href : '../mocap/pipeline.js';
const FAMILY_ORDER = Object.keys(DB.FAMILIES), PROFILE_ORDER = ['none', 'typical', ...DB.HELD_OUT.profiles];
const GOLDEN = 'c365c6a157de5e6e3b95e9ebfba3dc80e3ccd6a788c1558f1a2c71f2302bffd6';
export const BENCH_GATES = Object.freeze({
  G1: { pooledRel: 1.10, pooledAbs: 0.3, famRel: 1.20, famAbs: 0.5, px0Abs: 0.5 },
  G2: { px90: 0.40, px90Lo: 0.30, famPx90: 0.20, px45: 0.30, px135: 0.30, traj90: 0.25, px0Abs: 0.5, px180Abs: 0.5 },
  G3: { ser02: 0.03, ser05: 0.01, wrongHi: 0.02 },
  G4: { minTakes: 1000, missHi: 0.015, se: 2.5, minGreen: 100 },
  G5: { typGreen: 0.70, typRadius: 8, noneGreen: 0.90, noneRadius: 5 },
  G6: { heldMult: 3, hardSilent: 0.01, flagShare: 0.80, missMult: 10, falseOOD: 0.05, se: 2.5 },
  G7: { silent: 0.01, silentHi: 0.02, shiftFire: 0.90, shiftMult: 3 },
  G8: { none: 0.05, typical: 0.10, tailMult: 2, tailShare: 0.95, hinge: 0.005 },
  G9: { msPerFrame: 1.2, ratio: 2.5 },
});

// ---- worker side ----
async function workerMain() {
  const { processCapture } = await import(PIPE);
  parentPort.on('message', ({ id, job }) => {
    try { parentPort.postMessage({ id, rec: benchTake(job, processCapture) }); }
    catch (e) { parentPort.postMessage({ id, rec: { ...job, error: String(e?.stack ?? e) } }); }
  });
}

function methodMetrics(c, res, T, TW) {
  const R = DB.reconCamera(res), RW = DB.camToWorld(c.cam, R), strata = DB.truthStrata(TW, c.cam, res.n);
  const nv = DB.novelViewError(TW, RW, c.cam, { thetas: THETAS, strata }), nvt = DB.novelViewError(TW, RW, c.cam, { thetas: [90], mode: 'traj', strata }), pe = DB.poseError(T, R), se = DB.boneSignError(T, R);
  return { px: Object.fromEntries(THETAS.map(t => [t, nv[t].px.acc])), cm90: nv[90].cm.acc, cm90g: Object.fromEntries(GROUPS.map(g => [g, nv[90].perGroup[g].acc.cm])), cm90s: nv[90].strataAcc, traj90: nvt[90].px.acc,
    shape: pe.acc.shape, traj: pe.acc.traj, ser: { n: se.n, bad: se.bad, n05: se.n05, bad05: se.bad05 }, hinge: DB.hingeViolations(res), hingeFrames: res.take.n, ms: res.report.ms, n: res.n, height: res.skel.dims.height };
}

export function benchTake(job, processCapture) {
  const seed = DB.caseSeed(job.split, job.idx);
  let c;
  try { c = job.red ? DB.redteamCase(DB.REDTEAM.find(r => r.id === job.red), seed) : job.drawn ? DB.makeDrawnCase(seed) : DB.makeCase(job.family, seed, { profile: job.profile }); }
  catch (e) { if (e.message === 'unframeable') return { ...job, seed, skip: 'unframeable' }; throw e; }
  const out = { ...job, seed, family: c.family, profile: c.profile, truthHeight: c.skel.dims.height };
  let T = null, TW = null;
  const truth = res => { if (!T) { T = DB.truthSeries(c, res); TW = DB.truthSeries(c, res, 'world'); } };
  if (!job.newOnly) { const r0 = DB.runCase(c, { depth: false }, processCapture); truth(r0); out.base = methodMetrics(c, r0, T, TW); }
  const r1 = DB.runCase(c, { depth: true, certify: true }, processCapture);
  if (!r1.depth) throw new Error('pipeline.js has no depth stage (spec §9); processCapture ignored { depth: true }');
  truth(r1);
  if (!job.newOnly) out.new = methodMetrics(c, r1, T, TW);
  const cert = certifyTake(calContext(r1), DEPTH_CAL, { raw: true }), lam = Object.fromEntries(GROUPS.map(g => [g, DEPTH_CAL.radius.groups[g]?.lambda ?? Infinity]));
  const cr = DB.certRecords(c, r1, { cert, lambdas: lam, truthC: T, mask: !!job.affected, angles: !job.noAngles });
  const ts = trueSigns(T, r1.n, DEPTH_CAL.rMoot), sr = DB.signRecords(c, r1, { truthC: T, rMoot: DEPTH_CAL.rMoot }), lamS = DEPTH_CAL.sign.lambda;
  let cw = 0, cn = 0, nm = 0;
  sr.forEach(r => { if (!r.moot && r.S >= lamS) { cw += r.wrong; cn += r.nonMoot; } });
  for (let q = 0; q < r1.n * NE; q++) nm += ts.nonMoot[q];
  const rad = DB.accNew();
  let green = 0;
  for (let i = 0; i < r1.n; i++) for (const j of EVAL_JOINTS) { const q = i * 33 + j; if (cert.state[q] === STATE.GREEN) { green++; DB.accAdd(rad, cert.radius[q] * 100); } }
  out.cert = { inDomain: cert.inDomain, which: cert.domain.which, green, joints: r1.n * EVAL_JOINTS.length, rad, final: cr.final, shadow: cr.shadow, angleViol: cr.angleViol, missTotal: cr.missTotal,
    sign: { wrong: cw, nonMoot: cn, nonMootAll: nm }, scaleSource: r1.report.depth?.scaleSource ?? 'mediapipe', depthMs: r1.depth.ms, certMs: r1.cert?.summary?.ms ?? NaN, height: r1.skel.dims.height };
  if (job.affected) {
    const m = DB.affectedMask(c, r1, job.affected);
    let a = 0, f = 0, s = 0;
    for (let i = 0; i < r1.n; i++) for (const j of EVAL_JOINTS) { const q = i * 33 + j; if (!m[q]) continue; a++; if (cert.state[q] !== STATE.GREEN) f++; if (cr.silentMask[q]) s++; }
    out.cert.affected = { n: a, flagged: f, silent: s };
  }
  return out;
}

// ---- aggregation ----
const A = { new: DB.accNew, add: DB.accAdd, merge: DB.accMerge, mean: DB.accMean, q: DB.accQuantile };
const mergeAll = list => list.reduce((a, b) => A.merge(a, b), A.new());
const sum = (xs, f) => xs.reduce((a, x) => a + f(x), 0);
const pct = x => Number.isFinite(x) ? (100 * x).toFixed(1) : '-';
const f1 = x => Number.isFinite(x) ? x.toFixed(1) : '-', f2 = x => Number.isFinite(x) ? x.toFixed(2) : '-';
const mp = a => `${f1(A.mean(a))} [${f1(A.q(a, 0.95))}]`;
const ser = (rs, m, k) => { const n = sum(rs, r => r[m].ser[k === '05' ? 'n05' : 'n']), b = sum(rs, r => r[m].ser[k === '05' ? 'bad05' : 'bad']); return n ? b / n : 0; };
const boot = (num, den, seed = 7) => clusterBootstrap(num, den, 1000, mulberry32(seed));
const ones = n => new Float64Array(n).fill(1);

export function methodSummary(rs, m) {
  if (!rs.length || !rs[0][m]) return null;
  const px = Object.fromEntries(THETAS.map(t => [t, mergeAll(rs.map(r => r[m].px[t]))]));
  return { takes: rs.length, frames: sum(rs, r => r[m].n), px, cm90: mergeAll(rs.map(r => r[m].cm90)), traj90: mergeAll(rs.map(r => r[m].traj90)), shape: mergeAll(rs.map(r => r[m].shape)), traj: mergeAll(rs.map(r => r[m].traj)),
    cm90g: Object.fromEntries(GROUPS.map(g => [g, mergeAll(rs.map(r => r[m].cm90g[g]))])), cm90s: [0, 1, 2].map(k => mergeAll(rs.map(r => r[m].cm90s[k]))),
    ser02: ser(rs, m, '02'), ser05: ser(rs, m, '05'), hinge: sum(rs, r => r[m].hinge * r[m].hingeFrames) / Math.max(1, sum(rs, r => r[m].hingeFrames)),
    flagged: m === 'new' ? 1 - sum(rs, r => r.cert.green) / Math.max(1, sum(rs, r => r.cert.joints)) : NaN, msFrame: sum(rs, r => r[m].ms) / Math.max(1, sum(rs, r => r[m].n)) };
}

export function certSummary(rs, alpha = DEPTH_CAL.alpha) {
  const N = rs.length, inD = rs.filter(r => r.cert.inDomain), groups = {};
  for (const g of GROUPS) {
    const share = rs.map(r => r.cert.final.green[g] ? r.cert.final.miss[g] / r.cert.final.green[g] : 0), b = boot(share, ones(N));
    groups[g] = { green: sum(rs, r => r.cert.final.green[g]), miss: sum(rs, r => r.cert.final.miss[g]), takeAvg: N ? sum(share, x => x) / N : NaN, hi: b.hi };
  }
  const joints = sum(rs, r => r.cert.joints), silent = sum(rs, r => r.cert.final.silent), qInf = DEPTH_CAL.radius.qInf;
  const cov = inD.length ? inD.filter(r => r.cert.final.maxScore <= qInf).length / inD.length : NaN, se = inD.length ? Math.sqrt(Math.max(cov * (1 - cov), 1e-12) / inD.length) : NaN;
  const sw = rs.map(r => r.cert.sign.nonMoot ? r.cert.sign.wrong / r.cert.sign.nonMoot : 0), sb = boot(sw, ones(N), 11), rad = mergeAll(rs.map(r => r.cert.rad));
  const shadowRate = r => { const gr = sum(GROUPS, g => r.cert.shadow.green[g]); return gr ? sum(GROUPS, g => r.cert.shadow.miss[g]) / gr : NaN; };
  return { takes: N, alpha, joints, green: sum(rs, r => r.cert.green) / Math.max(1, joints), groups, silent, silentShare: joints ? silent / joints : 0, silentHi: cpUpper(silent, Math.max(1, joints)),
    qInfCov: cov, qInfSE: se, angleViol: Object.fromEntries([45, 90, 135, 180].map(t => [t, sum(rs, r => r.cert.angleViol[t] ?? 0)])), miss: sum(rs, r => r.cert.missTotal),
    radiusMedian: A.q(rad, 0.5), radiusP95: A.q(rad, 0.95), inDomain: N ? inD.length / N : NaN,
    sign: { certified: sum(rs, r => r.cert.sign.nonMoot) / Math.max(1, sum(rs, r => r.cert.sign.nonMootAll)), wrong: N ? sum(sw, x => x) / N : NaN, wrongHi: sb.hi },
    shadowIn: (() => { const v = inD.map(shadowRate).filter(Number.isFinite); return v.length ? 1 - v.reduce((a, b) => a + b, 0) / v.length : NaN; })(),
    shadowOut: (() => { const v = rs.filter(r => !r.cert.inDomain).map(shadowRate).filter(Number.isFinite); return v.length ? 1 - v.reduce((a, b) => a + b, 0) / v.length : NaN; })(),
    shadowRates: rs.map(r => [shadowRate(r), r.cert.inDomain ? 1 : 0]) };
}

function table(head, rows) {
  const w = head.map((h, k) => Math.max(h.length, ...rows.map(r => String(r[k]).length)));
  return [head, w.map(x => '-'.repeat(x)), ...rows].map(r => r.map((x, k) => String(x).padEnd(w[k])).join('  ')).join('\n');
}
const delta = (b, n) => Number.isFinite(b) && b > 0 ? `${(100 * (n - b) / b).toFixed(0)}%` : '-';

async function main() {
  const a = parseArgs(process.argv.slice(2), ['final', 'redteam', 'quick', 'no-runtime', 'tests']), quick = !!a.quick, final = !!a.final;
  if (quick && final) throw new Error('--quick and --final are exclusive');
  const split = final ? 'test' : a.split ?? 'dev', seeds = +(a.seeds ?? (quick ? 2 : 8)), coverage = +(a.coverage ?? 0), workers = +(a.workers ?? (os.availableParallelism?.() ?? os.cpus().length));
  if (!['dev', 'test'].includes(split)) throw new Error('--split must be dev or test');
  const families = !a.families || a.families === 'all' ? FAMILY_ORDER.filter(f => !DB.ATTACK_FAMILIES.includes(f)) : a.families.split(',');
  const profiles = (a.profiles ?? 'none,typical,hard').split(','), hash = codeHash(), uncal = !!DEPTH_CAL.quick, redteam = !!a.redteam || final;
  for (const f of families) if (!DB.FAMILIES[f]) throw new Error('unknown family ' + f);
  for (const p of profiles) if (!PROFILE_ORDER.includes(p)) throw new Error('unknown profile ' + p);
  if (final) {
    if (uncal) throw new Error('--final needs a non-quick DEPTH_CAL: run tools/depth_calibrate.mjs first');
    if (DEPTH_CAL.codeHash !== hash) throw new Error(`--final: DEPTH_CAL.codeHash ${DEPTH_CAL.codeHash.slice(0, 12)} does not match the code ${hash.slice(0, 12)}; rerun tools/depth_calibrate.mjs`);
    if (process.env.DEPTH_PIPELINE) throw new Error('--final cannot use DEPTH_PIPELINE');
  }
  const jobs = [];
  for (const f of families) for (const p of profiles) for (let s = 0; s < seeds; s++) jobs.push({ kind: 'row', split, idx: (FAMILY_ORDER.indexOf(f) * 16 + PROFILE_ORDER.indexOf(p)) * 1000 + s, family: f, profile: p });
  if (redteam) DB.REDTEAM.forEach((r, k) => { for (let s = 0; s < seeds; s++) jobs.push({ kind: 'red', split, idx: 300000 + k * 1000 + s, red: r.id, affected: r.affected }); });
  for (let k = 0; k < coverage; k++) jobs.push({ kind: 'cov', split, idx: 500000 + k, drawn: true, newOnly: true });
  const evalRange = [Math.min(...jobs.map(j => DB.caseSeed(split, j.idx))), Math.max(...jobs.map(j => DB.caseSeed(split, j.idx)))];
  for (const [s, r] of Object.entries(DEPTH_CAL.seeds ?? {})) if (DB.overlaps(r, evalRange)) throw new Error(`evaluated seeds ${evalRange} overlap calibration seeds ${s} ${r}`);
  for (const s of ['domain', 'sign', 'radius', 'unit']) if (DB.overlaps(DB.SPLITS[s], evalRange)) throw new Error(`evaluated seeds overlap split ${s}`);
  const label = quick ? 'QUICK (small N)' : final ? 'FINAL' : 'full', calLabel = uncal ? `UNCALIBRATED table (${DEPTH_CAL.calibId}): certificate gates are N/A` : `calibration ${DEPTH_CAL.calibId}${DEPTH_CAL.codeHash === hash ? '' : ' (STALE: codeHash differs)'}`;
  console.log(`depth_benchmark ${label}: split ${split}, ${families.length} families x ${profiles.length} profiles x ${seeds} seeds${redteam ? ', red team' : ''}${coverage ? `, coverage ${coverage}` : ''}; ${jobs.length} takes on ${workers} workers\n${calLabel}; code ${hash.slice(0, 12)}${process.env.DEPTH_PIPELINE ? '; DEPTH_PIPELINE=' + process.env.DEPTH_PIPELINE : ''}`);
  const t0 = Date.now(), recs = await runPool(jobs, workers, new URL(import.meta.url), 'takes', 'benchmark');
  const errs = recs.filter(r => r.error), skips = recs.filter(r => r.skip), ok = recs.filter(r => !r.error && !r.skip);
  if (errs.length) { console.log(`\n${errs.length} takes FAILED, first: ${errs[0].family ?? errs[0].red} seed ${DB.caseSeed(split, errs[0].idx)}\n${errs[0].error}`); }
  if (skips.length) console.log(`${skips.length} takes unframeable (skipped)`);
  const rows = ok.filter(r => r.kind === 'row'), reds = ok.filter(r => r.kind === 'red'), cov = ok.filter(r => r.kind === 'cov'), report = { label, split, seeds, hash, calibId: DEPTH_CAL.calibId, uncalibrated: uncal, takes: ok.length, failed: errs.length, unframeable: skips.length };
  const by = (f, p) => rows.filter(r => (!f || f.includes(r.family)) && (!p || r.profile === p));

  // Table 1
  const t1 = [];
  for (const f of families) for (const p of profiles) {
    const rs = by([f], p), b = methodSummary(rs, 'base'), n = methodSummary(rs, 'new');
    if (!b) continue;
    for (const [m, s] of [['base', b], ['new', n]]) t1.push([f, p, m, s.takes, s.frames, ...THETAS.map(t => mp(s.px[t]) + (m === 'new' ? ' ' + delta(A.mean(b.px[t]), A.mean(s.px[t])) : '')), mp(s.cm90), f1(A.mean(s.traj90)), f2(A.mean(s.shape)), f2(A.mean(s.traj)), pct(s.ser02), pct(s.ser05), pct(s.hinge), pct(s.flagged), f2(s.msFrame)]);
  }
  console.log('\nTable 1. From other angles (shape mode), mean [p95]; new rows show the change vs base');
  console.log(table(['family', 'profile', 'method', 'takes', 'frames', ...THETAS.map(t => `px${t}`), 'cm90', 'trajpx90', 'mpjpe', 'mpjpeT', 'SER.2%', 'SER.5%', 'hinge%', 'flag%', 'ms/f'], t1));
  const t1b = [];
  for (const p of profiles) { const rs = by(null, p), b = methodSummary(rs, 'base'), n = methodSummary(rs, 'new'); if (!b) continue; for (const [m, s] of [['base', b], ['new', n]]) t1b.push([p, m, ...s.cm90s.map(x => f1(A.mean(x))), ...GROUPS.map(g => f1(A.mean(s.cm90g[g])))]); }
  console.log('\nTable 1b. cm at 90 deg by |dz|/L stratum and by group (all families)');
  console.log(table(['profile', 'method', '[0,.2)', '[.2,.5)', '[.5,1]', ...GROUPS], t1b));

  // Table 2
  const inFam = DB.IN_DIST, held = DB.HELD_OUT.families, t2 = [], cs = {};
  const sets = [['in-distribution' + (cov.length ? ' (drawn)' : ' (typical rows)'), cov.length ? cov : by(inFam, 'typical')], ['held-out families, typical', by(held, 'typical')], ['hard', by(null, 'hard')],
    ...profiles.filter(p => DB.HELD_OUT.profiles.includes(p) && p !== 'hard').map(p => [`attack ${p}`, by(null, p)]), ...DB.REDTEAM.map(r => [`${r.id} ${r.name}`, reds.filter(x => x.red === r.id)])];
  for (const [name, rs] of sets) {
    if (!rs.length) continue;
    const c = cs[name] = certSummary(rs);
    t2.push([name, c.alpha, c.takes, c.joints, pct(c.green), ...GROUPS.map(g => `${pct(c.groups[g].takeAvg)} [${pct(c.groups[g].hi)}]`), pct(c.silentShare) + ` [${pct(c.silentHi)}]`, `${pct(c.qInfCov)} vs 95`,
      [45, 90, 135, 180].map(t => c.angleViol[t]).join('/') + ` <= ${c.miss}`, `${f1(c.radiusMedian)}/${f1(c.radiusP95)}`, pct(c.inDomain)]);
  }
  const sgn = cs[sets[0][0]];
  console.log(`\nTable 2. Certificate${uncal ? ' (UNCALIBRATED: everything is GREY)' : ''}; per-group miss = take-average share of green joint-frames outside the radius [95% cluster-bootstrap upper]`);
  console.log(table(['set', 'alpha', 'takes', 'joint-frames', 'green%', ...GROUPS.map(g => g + '%'), 'silent>10cm%', 'qInf cov', 'angle viol 45/90/135/180', 'radius med/p95 cm', 'in-domain%'], t2));
  if (sgn) console.log(`sign (${sets[0][0]}): certified ${pct(sgn.sign.certified)}% of non-moot bone-frames; wrong share ${pct(sgn.sign.wrong)}% [${pct(sgn.sign.wrongHi)}%] vs rSign ${pct(DEPTH_CAL.rSign)}%`);

  // Table 3
  const t3 = [], red = {};
  for (const r of DB.REDTEAM) {
    const rs = reds.filter(x => x.red === r.id);
    if (!rs.length) continue;
    const b = methodSummary(rs, 'base'), n = methodSummary(rs, 'new'), af = rs.reduce((acc, x) => ({ n: acc.n + x.cert.affected.n, f: acc.f + x.cert.affected.flagged, s: acc.s + x.cert.affected.silent }), { n: 0, f: 0, s: 0 });
    const c = certSummary(rs), silent = af.n ? af.s / af.n : 0, hi = cpUpper(af.s, Math.max(1, af.n)), fired = 1 - c.inDomain, covers = GROUPS.every(g => c.groups[g].hi <= BENCH_GATES.G7.shiftMult * DEPTH_CAL.alpha);
    const pass = silent <= BENCH_GATES.G7.silent && hi <= BENCH_GATES.G7.silentHi && (!r.shift || fired >= BENCH_GATES.G7.shiftFire || covers);
    red[r.id] = { takes: rs.length, silent, hi, fired, covers, pass, affected: af };
    t3.push([r.id, r.name, rs.length, `${pct(b.ser02)} / ${f1(A.mean(b.px[90]))}`, `${pct(n.ser02)} / ${f1(A.mean(n.px[90]))}`, pct(af.n ? af.f / af.n : NaN), `${pct(silent)} [${pct(hi)}]`, r.shift ? `${pct(fired)}% fired${covers ? ', covered' : ''}` : '', uncal ? 'n/a' : pass ? 'PASS' : 'FAIL']);
  }
  if (t3.length) { console.log('\nTable 3. Red team'); console.log(table(['id', 'attack', 'takes', 'base SER%/px90', 'new SER%/px90', 'flagged% affected', 'silent% [CP95]', 'shift', 'pass'], t3)); }

  // Table 4 (main thread, sequential, best of 3)
  let rt = null;
  if (!a['no-runtime'] && !quick) {
    const { processCapture } = await import(PIPE), c = DB.makeCase('circle', DB.caseSeed(split, 900000), { profile: 'typical', seconds: 100 });
    const best = s => { let r = null, m = Infinity; for (let k = 0; k < 3; k++) { const x = DB.runCase(c, s, processCapture); if (x.report.ms < m) { m = x.report.ms; r = x; } } return r; };
    const b = best({ depth: false }), n = best({ depth: true, certify: true }), n2 = DB.runCase(c, { depth: true, certify: true }, processCapture);
    const dig = r => { const h = createHash('sha256'); for (const x of [r.landmarks, r.take.root, r.take.rot, r.depth.P, r.cert?.state ?? new Uint8Array(0)]) h.update(Buffer.from(x.buffer, x.byteOffset, x.byteLength)); return h.digest('hex'); };
    rt = { frames: n.n, baseMs: b.report.ms, newMs: n.report.ms, depthMs: n.depth.ms, certMs: n.cert?.summary?.ms ?? 0, ratio: n.report.ms / b.report.ms, stageMsFrame: (n.depth.ms + (n.cert?.summary?.ms ?? 0)) / n.n, deterministic: dig(n) === dig(n2) };
    console.log('\nTable 4. Runtime on the 3000-frame circle (best of 3)');
    console.log(table(['frames', 'baseline ms', 'new ms', 'depth ms', 'cert ms', 'ratio', 'depth+cert ms/frame', 'deterministic'], [[rt.frames, rt.baseMs, rt.newMs, rt.depthMs, rt.certMs, f2(rt.ratio), f2(rt.stageMsFrame), rt.deterministic]]));
  }

  // Table 5
  const t5 = [];
  for (const p of [...new Set([...profiles, ...(cov.length ? ['drawn'] : [])])]) { const rs = p === 'drawn' ? cov : by(null, p); if (!rs.length) continue; const c = certSummary(rs); t5.push([p, rs.length, pct(c.inDomain), pct(c.shadowIn), pct(c.shadowOut)]); }
  console.log('\nTable 5. Domain check (coverage = 1 - take-average miss of would-be greens at the calibrated radii)');
  console.log(table(['profile', 'takes', 'in-domain%', 'coverage in domain%', 'coverage out of domain%'], t5));

  // Gates
  const G = {}, set = (k, pass, info, na = false) => { G[k] = { pass: na ? null : !!pass, info }; };
  {
    const { capture } = goldenCapture(), h = createHash('sha256');
    for (const f of capture.frames) for (const p of f.poses) { h.update(Buffer.from(p.img.buffer)); h.update(Buffer.from(p.world.buffer)); }
    const dg = h.digest('hex'), tests = a.tests ? spawnSync(process.execPath, ['--test', ...testFiles()], { cwd: ROOT, encoding: 'utf8' }) : null, testsOk = tests ? tests.status === 0 : null;
    set('G0', dg === GOLDEN && testsOk !== false && !errs.length, `golden digest ${dg === GOLDEN ? 'unchanged' : 'CHANGED ' + dg}; mocap tests ${testsOk === null ? 'not run (pass --tests)' : testsOk ? 'pass' : 'FAIL'}; ${errs.length} take failures`, testsOk === null && dg === GOLDEN && !errs.length);
  }
  {
    const g = BENCH_GATES.G1, rs = by(inFam, 'none'), b = methodSummary(rs, 'base'), n = methodSummary(rs, 'new');
    if (b) {
      const fam = inFam.map(f => { const x = by([f], 'none'), bb = methodSummary(x, 'base'), nn = methodSummary(x, 'new'); return bb ? [f, A.mean(bb.shape), A.mean(nn.shape)] : null; }).filter(Boolean), bad = fam.filter(([, bb, nn]) => !(nn <= g.famRel * bb + g.famAbs));
      const pooled = A.mean(n.shape) <= g.pooledRel * A.mean(b.shape) + g.pooledAbs, px0 = A.mean(n.px[0]) <= A.mean(b.px[0]) + g.px0Abs;
      set('G1', pooled && !bad.length && px0, `mpjpe ${f2(A.mean(b.shape))} -> ${f2(A.mean(n.shape))} cm; px0 ${f1(A.mean(b.px[0]))} -> ${f1(A.mean(n.px[0]))}; families over: ${bad.map(x => x[0]).join(',') || 'none'}`);
    } else set('G1', false, 'no profile-none rows', true);
  }
  {
    const g = BENCH_GATES.G2, rs = by(inFam, 'typical'), b = methodSummary(rs, 'base'), n = methodSummary(rs, 'new');
    if (b) {
      const red_ = t => 1 - A.mean(n.px[t]) / A.mean(b.px[t]), bt = boot(rs.map(r => r.new.px[90].sum), rs.map(r => r.base.px[90].sum), 3), lo = 1 - bt.hi;
      const fam = inFam.map(f => { const x = by([f], 'typical'), bb = methodSummary(x, 'base'), nn = methodSummary(x, 'new'); return bb ? [f, 1 - A.mean(nn.px[90]) / A.mean(bb.px[90])] : null; }).filter(Boolean), bad = fam.filter(x => !(x[1] >= g.famPx90));
      const trj = 1 - A.mean(n.traj90) / A.mean(b.traj90), ends = A.mean(n.px[0]) <= A.mean(b.px[0]) + g.px0Abs && A.mean(n.px[180]) <= A.mean(b.px[180]) + g.px180Abs;
      set('G2', red_(90) >= g.px90 && lo >= g.px90Lo && !bad.length && red_(45) >= g.px45 && red_(135) >= g.px135 && trj >= g.traj90 && ends,
        `reductions: px90 ${pct(red_(90))}% (95% lower bound ${pct(lo)}%), px45 ${pct(red_(45))}%, px135 ${pct(red_(135))}%, traj px90 ${pct(trj)}%; px0 ${f1(A.mean(b.px[0]))}->${f1(A.mean(n.px[0]))}, px180 ${f1(A.mean(b.px[180]))}->${f1(A.mean(n.px[180]))}; families under 20%: ${bad.map(x => x[0]).join(',') || 'none'}`);
    } else set('G2', false, 'no typical rows', true);
  }
  {
    const g = BENCH_GATES.G3, rs = by(null, 'typical'), n = methodSummary(rs, 'new'), c = rs.length ? certSummary(rs) : null;
    if (n) set('G3', n.ser02 <= g.ser02 && n.ser05 <= g.ser05 && (uncal || (c.sign.wrong <= DEPTH_CAL.rSign && c.sign.wrongHi <= g.wrongHi)),
      `SER0.2 ${pct(n.ser02)}%, SER0.5 ${pct(n.ser05)}%; typical rows: certified ${pct(c.sign.certified)}% of non-moot bone-frames, wrong ${pct(c.sign.wrong)}% [${pct(c.sign.wrongHi)}%]${uncal ? ' (sign part N/A: uncalibrated)' : ''}`);
    else set('G3', false, 'no typical rows', true);
  }
  {
    const g = BENCH_GATES.G4, c = cov.length ? certSummary(cov) : null;
    if (!c || uncal || cov.length < g.minTakes) set('G4', false, `${cov.length} drawn test takes (needs --coverage ${g.minTakes} and a calibrated table)`, true);
    else {
      const bad = GROUPS.filter(k => !(c.groups[k].takeAvg <= c.alpha && c.groups[k].hi <= g.missHi)), ang = Object.values(c.angleViol).every(v => v <= c.miss), thin = GROUPS.filter(k => c.groups[k].green < g.minGreen);
      set('G4', !bad.length && !thin.length && c.qInfCov >= 0.95 - g.se * c.qInfSE && ang, `groups failing: ${bad.join(',') || 'none'}; groups under ${g.minGreen} green joint-frames (vacuous): ${thin.join(',') || 'none'}; qInf coverage ${pct(c.qInfCov)}% (SE ${pct(c.qInfSE)}); angle violations ${Object.values(c.angleViol).join('/')} <= ${c.miss}`);
    }
  }
  {
    const g = BENCH_GATES.G5, ct = by(inFam, 'typical'), cn = by(inFam, 'none'), a1 = ct.length ? certSummary(ct) : null, a0 = cn.length ? certSummary(cn) : null;
    if (uncal || !a1 || !a0) set('G5', false, uncal ? 'uncalibrated' : 'needs none and typical rows', true);
    else set('G5', a1.green >= g.typGreen && a1.radiusMedian <= g.typRadius && a0.green >= g.noneGreen && a0.radiusMedian <= g.noneRadius, `typical green ${pct(a1.green)}% median ${f1(a1.radiusMedian)} cm; none green ${pct(a0.green)}% median ${f1(a0.radiusMedian)} cm`);
  }
  {
    const g = BENCH_GATES.G6, h = by(held, 'typical'), hd = by(null, 'hard'), all = [...rows, ...reds, ...cov], indist = cov.length ? cov : [...by(inFam, 'typical'), ...by(inFam, 'none')];
    if (uncal || !h.length || !hd.length) set('G6', false, uncal ? 'uncalibrated' : 'needs held-out typical and hard rows', true);
    else {
      const ch = certSummary(h), chd = certSummary(hd), heldOk = GROUPS.every(k => ch.groups[k].hi <= g.heldMult * ch.alpha), bad = certSummary(all).shadowRates.filter(([m]) => m > g.missMult * DEPTH_CAL.alpha);
      const flagged = bad.length ? bad.filter(([, d]) => !d).length / bad.length : 1, ci = certSummary(indist), fo = 1 - ci.inDomain, se = Math.sqrt(Math.max(fo * (1 - fo), 1e-12) / indist.length);
      const gh = sum(GROUPS, k => ch.groups[k].green);
      set('G6', gh > 0 && heldOk && chd.silentShare <= g.hardSilent && flagged >= g.flagShare && fo <= g.falseOOD + g.se * se,
        `held-out green joint-frames ${gh}${gh ? '' : ' (vacuous)'}; held-out miss upper max ${pct(Math.max(...GROUPS.map(k => ch.groups[k].hi)))}%; hard silent ${pct(chd.silentShare)}%; flagged ${pct(flagged)}% of ${bad.length} bad takes; false out-of-domain ${pct(fo)}%`);
    }
  }
  {
    const ids = Object.keys(red);
    if (!ids.length) set('G7', false, 'red team not run (pass --redteam)', true);
    else { const gr = sum(reds, r => r.cert.green); set('G7', !uncal && gr > 0 && ids.length === DB.REDTEAM.length && ids.every(k => red[k].pass), `${ids.filter(k => red[k].pass).length}/${DB.REDTEAM.length} rows pass; ${gr} green joint-frames${uncal ? ' (uncalibrated: vacuous)' : gr ? '' : ' (vacuous)'}`, uncal); }
  }
  {
    const g = BENCH_GATES.G8, res = {};
    let ok8 = true, info = [];
    for (const p of ['none', 'typical']) {
      const rs = by(['jump'], p);
      if (!rs.length) continue;
      const e = rs.map(r => r.cert.scaleSource === 'gravity' ? Math.abs(r.cert.height / r.truthHeight - 1) : Infinity).sort((x, y) => x - y), med = e[(e.length - 1) >> 1], tail = e.filter(x => x <= g.tailMult * g[p]).length / e.length;
      res[p] = { med, tail, noGravity: e.filter(x => x === Infinity).length }; ok8 &&= med <= g[p] && tail >= g.tailShare;
      info.push(`${p}: median scale error ${Number.isFinite(med) ? pct(med) + '%' : 'n/a'}, within ${pct(g.tailMult * g[p])}% in ${pct(tail)}% of takes (${res[p].noGravity}/${e.length} without a gravity scale)`);
    }
    const b = methodSummary(rows, 'base'), n = methodSummary(rows, 'new');
    if (n) { ok8 &&= n.hinge <= g.hinge && n.hinge <= b.hinge; info.push(`hinge ${pct(b.hinge)}% -> ${pct(n.hinge)}%`); }
    set('G8', ok8 && Object.keys(res).length, info.join('; ') || 'no jump rows', !Object.keys(res).length);
  }
  {
    const g = BENCH_GATES.G9, rnd = ['depth', 'depthsign', 'depthstats', 'depthcert', 'mpfail', 'depthbench'].filter(f => /Math\.random/.test(readFileSync(resolve(ROOT, `mocap/${f}.js`), 'utf8')));
    if (!rt) set('G9', false, `runtime not measured; Math.random in: ${rnd.join(',') || 'none'}`, true);
    else set('G9', rt.stageMsFrame <= g.msPerFrame && rt.ratio <= g.ratio && rt.deterministic && !rnd.length, `depth+cert ${f2(rt.stageMsFrame)} ms/frame, total ratio ${f2(rt.ratio)}, deterministic ${rt.deterministic}, Math.random in: ${rnd.join(',') || 'none'}`);
  }
  console.log(`\nGates${uncal ? ' (UNCALIBRATED table: certificate gates are N/A)' : ''}${quick ? ' (QUICK: small N, indicative only)' : ''}`);
  for (const [k, v] of Object.entries(G)) console.log(`${k} ${v.pass === null ? 'N/A ' : v.pass ? 'PASS' : 'FAIL'}  ${v.info}`);
  const depthOk = ['G0', 'G1', 'G2', 'G3', 'G6', 'G7', 'G8', 'G9'].every(k => G[k].pass === true), certOk = depthOk && G.G4.pass === true && G.G5.pass === true;
  console.log(`decision: depth default ${final && depthOk ? 'may flip to true' : 'stays false'}${final ? '' : ' (needs --final)'}; certificate line ${final && certOk ? 'may be shown' : 'stays hidden'}. ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  Object.assign(report, { gates: G, red, runtime: rt, cert: Object.fromEntries(Object.entries(cs).map(([k, v]) => [k, { ...v, shadowRates: undefined }])), table1: t1, table1b: t1b, table2: t2, table3: t3, table5: t5 });
  if (a.out) writeFileSync(resolve(a.out), JSON.stringify(report, (k, v) => ArrayBuffer.isView(v) ? undefined : v, 1));
  if (final) appendFileSync(resolve(ROOT, 'tools/depth_benchmark.log'), `${new Date().toISOString()} head ${a.head ?? 'unknown'} hash ${hash} cal ${DEPTH_CAL.calibId} N ${ok.length} gates ${Object.entries(G).map(([k, v]) => k + ':' + (v.pass === null ? 'NA' : v.pass ? 'P' : 'F')).join(' ')}\n`);
  process.exitCode = errs.length ? 1 : 0;
}

function testFiles() { return ['mocap_core', 'mocap_motion', 'mocap_pipeline', 'mocap_depth'].map(f => `tests/${f}.test.mjs`).filter(f => { try { readFileSync(resolve(ROOT, f)); return true; } catch { return false; } }); }

// the seed-3 pipeline scenario of tests/mocap_pipeline.test.mjs: the golden digest of §6.1
export function goldenCapture() {
  const skel = buildSkeleton({ ...DEFAULT_DIMS, thigh: 0.44, shin: 0.43, torso: 0.52 }), lib = { idle: idleClip(skel), walk: gaitClip(skel, { kind: 'walk', speed: 1.25 }), run: gaitClip(skel, { kind: 'run', speed: 3.45 }), jump: jumpClip(skel), actions: [], take: null };
  const cam = lookAtCamera([0.3, 1.45, 4.5], [0, 0.85, -0.5], { width: 1280, height: 720, hfov: 60 }), ctl = createController(lib, { yaw: Math.PI / 2 });
  ctl.pos = [-3, 0, -0.5];
  const take = recordTake(ctl, [{ seconds: 1.5, input: {} }, { seconds: 4.5, input: { right: true } }, { seconds: 1.5, input: {} }].map(s => ({ ...s, input: { ...s.input, camYaw: Math.PI } })), 30);
  return synthCapture(skel, take, cam, { rand: mulberry32(3), noiseImg: 0.003, noiseWorld: 0.015, occlusion: true });
}

if (!isMainThread && workerData?.tool === 'benchmark') workerMain();
else if (isMainThread && process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) main().catch(e => { console.error(e); process.exit(2); });
