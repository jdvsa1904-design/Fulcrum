#!/usr/bin/env node
// Calibrates the depth certificate (spec §11) and writes mocap/depth_calibration.js.
// node tools/depth_calibrate.mjs [--domain 1000] [--sign 3000] [--radius 3000] [--alpha 0.01] [--rsign 0.01] [--delta 0.05] [--workers N] [--quick] [--strict] [--out file] [--dry]
import { Worker, isMainThread, parentPort, workerData } from 'node:worker_threads';
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, resolve } from 'node:path';
import os from 'node:os';
import { caseSeed, makeDrawnCase, drawCase, runCase, signRecords, certRecords, truthSeries, CODE_FILES, SPLITS, overlaps } from '../mocap/depthbench.js';
import { certifyTake, domainThresholds, domainCheck, CERT_GROUPS, DOMAIN_STATS, R_EDGE } from '../mocap/depthcert.js';
import { lttFixedSequence, conformalQuantile } from '../mocap/depthstats.js';
import { DEPTH_CAL } from '../mocap/depth_calibration.js';
import { DEPTH_DEFAULTS } from '../mocap/depth.js';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..'), GROUPS = Object.keys(CERT_GROUPS);
const PIPE = process.env.DEPTH_PIPELINE ? pathToFileURL(resolve(process.env.DEPTH_PIPELINE)).href : '../mocap/pipeline.js';
export const DOMAIN_TAILS = DOMAIN_STATS.reduce((a, d) => a + (d[1] === 2 ? 2 : 1), 0);
export const SIGMA_MAX = 0.08, SIGMA0 = 0.01, DELTA_DOMAIN = 0.05, ALPHA_INF = 0.05;
export const SIGN_GRID = Object.freeze([30, 25, 20, 16, 13, 10, 8, 6.5, 5, 4, 3, 2.3, 1.6, 1]);
export const RADIUS_GRID = Object.freeze(Array.from({ length: 46 }, (_, k) => +(60 * 0.9 ** k).toPrecision(4)));
export const codeHash = (root = ROOT) => { const h = createHash('sha256'); for (const f of CODE_FILES) h.update(readFileSync(resolve(root, f))); return h.digest('hex'); };

// ---- worker side ----
async function workerMain() {
  const { processCapture } = await import(PIPE);
  parentPort.on('message', ({ id, job }) => {
    try { parentPort.postMessage({ id, rec: calTake(job, processCapture) }); }
    catch (e) { parentPort.postMessage({ id, rec: { k: job.k, seed: job.seed, error: String(e?.message ?? e) } }); }
  });
}

export function calContext(res) {
  return { series: res.series, depth: res.depth, skel: res.skel, take: res.take, camera: res.camera, fps: res.fps, n: res.n, scaleSource: res.report?.depth?.scaleSource ?? 'mediapipe', height: res.settings.height };
}

export function calTake(job, processCapture) {
  const { split, k, phase } = job, seed = caseSeed(split, k), d = drawCase(seed);
  let c;
  try { c = makeDrawnCase(seed); } catch (e) { if (e.message === 'unframeable') return { k, seed, family: d.family, skip: 'unframeable' }; throw e; }
  const res = runCase(c, { depth: true, certify: false }, processCapture);
  if (!res.depth) throw new Error('pipeline.js has no depth stage (spec §9); processCapture ignored { depth: true }');
  const cal = job.cal ?? DEPTH_CAL, cert = certifyTake(calContext(res), cal, { raw: true }), raw = cert.raw, base = { k, seed, family: d.family, profile: d.o.profile, ok: !!raw };
  if (!raw) return base;
  const out = { ...base, domain: Float64Array.from(raw.domain) };
  if (phase === 'domain') {
    const v = [];
    for (let i = 0; i < res.n; i++) if (raw.dof[i] > 0) v.push(raw.Q[i] / raw.dof[i]);
    out.chi = Float32Array.from(v);
  } else if (phase === 'sign') {
    const recs = signRecords(c, res, { rMoot: cal.rMoot }).filter(r => !r.moot), t = new Float64Array(recs.length * 3);
    recs.forEach((r, q) => { t[q * 3] = r.S; t[q * 3 + 1] = r.nonMoot; t[q * 3 + 2] = r.wrong; });
    out.ints = t;
  } else {
    const cr = certRecords(c, res, { cert, grid: job.grid, angles: false, truthC: truthSeries(c, res) });
    out.scores = cr.scores; out.green = cr.green; out.maxScore = cr.maxScore;
  }
  return out;
}

// ---- main side ----
export async function runPool(jobs, nWorkers, url, label = '', tool = 'calibrate') {
  const out = new Array(jobs.length), W = Math.max(1, Math.min(nWorkers, jobs.length));
  let next = 0, done = 0, lastPct = -1;
  const t0 = Date.now();
  await Promise.all(Array.from({ length: W }, () => new Promise((ok, fail) => {
    const w = new Worker(url, { workerData: { tool } });
    const feed = () => { if (next < jobs.length) { const id = next++; w.postMessage({ id, job: jobs[id] }); } else w.terminate().then(ok); };
    w.on('message', ({ id, rec }) => {
      out[id] = rec; done++;
      const pct = Math.floor(100 * done / jobs.length), tty = process.stderr.isTTY;
      if (pct >= lastPct + (tty ? 1 : 10) || done === jobs.length) { lastPct = pct; process.stderr.write(`${tty ? '\r' : ''}${label} ${done}/${jobs.length} (${((Date.now() - t0) / 1000).toFixed(0)} s)${tty && done < jobs.length ? '   ' : '\n'}`); }
      feed();
    });
    w.on('error', fail);
    feed();
  })));
  return out;
}

export function parseArgs(argv, flags) {
  const o = {};
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (!a.startsWith('--')) throw new Error('unexpected argument ' + a);
    const k = a.slice(2);
    if (flags.includes(k)) o[k] = true; else { if (i + 1 >= argv.length) throw new Error('missing value for ' + a); o[k] = argv[++i]; }
  }
  return o;
}

export const lit = v => JSON.stringify(v, (k, x) => typeof x === 'number' && !Number.isFinite(x) ? `@@${x}@@` : ArrayBuffer.isView(x) ? Array.from(x) : x).replace(/"@@(-?Infinity|NaN)@@"/g, '$1');
const litTop = o => '{\n' + Object.entries(o).map(([k, v]) => `  ${k}: ${lit(v)}`).join(',\n') + '\n}';
const r6 = x => Number.isFinite(x) ? +x.toPrecision(6) : x;

export function signRisk(recs, grid) {
  return grid.map(l => {
    let s = 0;
    for (const r of recs) { let w = 0, m = 0; for (let q = 0; q < r.ints.length; q += 3) if (r.ints[q] >= l) { m += r.ints[q + 1]; w += r.ints[q + 2]; } s += m > 0 ? w / m : 0; }
    return recs.length ? s / recs.length : NaN;
  });
}
export function radiusRisk(recs, grid, g) { return grid.map((_, k) => recs.length ? recs.reduce((a, r) => a + (r.green[g] > 0 ? r.scores[g][k] / r.green[g] : 0), 0) / recs.length : NaN); }

async function main() {
  const A = parseArgs(process.argv.slice(2), ['quick', 'strict', 'dry']), quick = !!A.quick, strict = !!A.strict;
  if (quick && strict) throw new Error('--quick and --strict are exclusive');
  const N = { domain: +(A.domain ?? (quick ? 60 : 1000)), sign: +(A.sign ?? (quick ? 150 : strict ? 10000 : 3000)), radius: +(A.radius ?? (quick ? 150 : strict ? 10000 : 3000)) };
  const alpha = +(A.alpha ?? (quick ? 0.1 : strict ? 1e-3 : 0.01)), rSign = +(A.rsign ?? (quick ? 0.1 : strict ? 1e-3 : 0.01)), delta = +(A.delta ?? 0.05);
  const workers = +(A.workers ?? (os.availableParallelism?.() ?? os.cpus().length)), url = new URL(import.meta.url), outPath = resolve(ROOT, A.out ?? 'mocap/depth_calibration.js');
  const seeds = Object.fromEntries(Object.entries(N).map(([s, n]) => [s, [caseSeed(s, 0), caseSeed(s, n - 1)]]));
  for (const [s, r] of Object.entries(seeds)) for (const t of ['dev', 'test', 'unit']) if (overlaps(r, SPLITS[t])) throw new Error(`calibration seeds ${s} overlap ${t}`);
  const forcedQuick = quick || !!process.env.DEPTH_PIPELINE, hash = codeHash(), t0 = Date.now();
  console.log(`depth_calibrate: ${quick ? 'QUICK (uncalibrated) ' : strict ? 'STRICT ' : ''}domain ${N.domain}, sign ${N.sign}, radius ${N.radius} takes; alpha ${alpha}, rSign ${rSign}, delta ${delta}; ${workers} workers; code ${hash.slice(0, 12)}`);
  if (process.env.DEPTH_PIPELINE) console.log(`  DEPTH_PIPELINE=${process.env.DEPTH_PIPELINE}: development pipeline, so the table is written as quick (uncalibrated).`);
  const jobsA = [...Array.from({ length: N.domain }, (_, k) => ({ split: 'domain', k, phase: 'domain' })), ...Array.from({ length: N.sign }, (_, k) => ({ split: 'sign', k, phase: 'sign' }))];
  const recA = await runPool(jobsA, workers, url, 'domain+sign'), dom = recA.slice(0, N.domain), sig = recA.slice(N.domain);
  const tally = rs => ({ used: rs.filter(r => r.ok).length, unframeable: rs.filter(r => r.skip).length, failed: rs.filter(r => r.error || (!r.skip && !r.ok)).length, errors: [...new Set(rs.filter(r => r.error).map(r => r.error))].slice(0, 3) });
  const domOk = dom.filter(r => r.ok), thr = domainThresholds(domOk.map(r => r.domain), DELTA_DOMAIN / DOMAIN_TAILS);
  const pooled = Float64Array.from(domOk.flatMap(r => Array.from(r.chi))).sort(), chi2Frame = pooled.length ? pooled[Math.min(pooled.length - 1, Math.ceil(0.999 * pooled.length) - 1)] : Infinity;
  const calD = { ...DEPTH_CAL, quick: false, domain: { names: thr.names, lo: thr.lo, hi: thr.hi }, chi2Frame, sigmaMax: SIGMA_MAX, rMoot: DEPTH_DEFAULTS.rMoot, rEdge: R_EDGE, sigma0: SIGMA0 };
  const inDom = r => r.ok && domainCheck(r.domain, calD).pass, sigIn = sig.filter(inDom);
  const sRisk = signRisk(sigIn, SIGN_GRID), sL = lttFixedSequence(SIGN_GRID, sRisk, sigIn.length, rSign, delta / 2);
  const calS = { ...calD, sign: { grid: [...SIGN_GRID], lambda: sL.lambda, p: sL.p, risk: sL.index >= 0 ? sRisk[sL.index] : NaN } };
  const recB = await runPool(Array.from({ length: N.radius }, (_, k) => ({ split: 'radius', k, phase: 'radius', cal: calS, grid: [...RADIUS_GRID] })), workers, url, 'radius');
  const radIn = recB.filter(inDom), groups = {};
  for (const g of GROUPS) {
    const risk = radiusRisk(radIn, RADIUS_GRID, g), withG = radIn.filter(r => r.green[g] > 0).length, enough = withG >= Math.ceil(1 / alpha);
    const L = enough ? lttFixedSequence(RADIUS_GRID, risk, radIn.length, alpha, delta / 12) : { lambda: Infinity, p: 1, index: -1 };
    groups[g] = { lambda: L.lambda, p: r6(L.p), risk: L.index >= 0 ? r6(risk[L.index]) : NaN, index: L.index, green: radIn.reduce((a, r) => a + r.green[g], 0), takesWithGreen: withG };
  }
  const qi0 = conformalQuantile(radIn.map(r => r.maxScore), ALPHA_INF), qi = qi0.q > 0 ? qi0 : { ...qi0, q: Infinity }, created = new Date().toISOString();
  const cal = { version: 1, quick: forcedQuick, strict, calibId: `${forcedQuick ? 'quick' : strict ? 'strict' : 'cal'}-${created.slice(0, 10)}-${hash.slice(0, 8)}`, codeHash: hash, created, alpha, delta, rSign,
    seeds, n: { domain: domOk.length, sign: sigIn.length, radius: radIn.length }, takes: { domain: tally(dom), sign: tally(sig), radius: tally(recB) },
    domain: { names: thr.names, lo: thr.lo.map(r6), hi: thr.hi.map(r6), a: thr.a, n: thr.n }, chi2Frame: r6(chi2Frame), sigmaMax: SIGMA_MAX, rMoot: DEPTH_DEFAULTS.rMoot, rEdge: R_EDGE, sigma0: SIGMA0,
    sign: { grid: [...SIGN_GRID], lambda: sL.lambda, p: r6(sL.p), risk: sL.index >= 0 ? r6(sRisk[sL.index]) : NaN, index: sL.index, riskCurve: sRisk.map(r6) },
    radius: { grid: [...RADIUS_GRID], groups, qInf: r6(qi.q), alphaInf: ALPHA_INF, qInfRank: qi.k } };
  const src = `// Generated by tools/depth_calibrate.mjs on ${created}${forcedQuick ? ' (QUICK: uncalibrated, everything stays GREY "not calibrated")' : ''}. Do not edit: rerun the tool after any change to the hashed files.\n`
    + `export const DEPTH_CAL = Object.freeze(${litTop(cal)});\n`;
  if (!A.dry) writeFileSync(outPath, src);
  const fail = [sL.index, ...GROUPS.map(g => groups[g].index)].some(i => i < 0) || !Number.isFinite(qi.q);
  console.log(`\ntakes used (in domain / framed): domain ${domOk.length}/${N.domain}, sign ${sigIn.length}/${sig.filter(r => r.ok).length}, radius ${radIn.length}/${recB.filter(r => r.ok).length}`);
  for (const [s, t] of Object.entries(cal.takes)) if (t.failed || t.unframeable) console.log(`  ${s}: ${t.unframeable} unframeable, ${t.failed} failed ${t.errors.join(' | ')}`);
  console.log(`chi2Frame ${cal.chi2Frame}; domain bounds: ${thr.names.map((nm, k) => `${nm} [${r6(thr.lo[k])}, ${r6(thr.hi[k])}]`).join(', ')}`);
  console.log(`sign: lambda ${sL.lambda} (p ${r6(sL.p)}, risk ${cal.sign.risk}, target ${rSign}, level ${delta / 2})`);
  for (const g of GROUPS) console.log(`radius ${g.padEnd(6)}: lambda ${groups[g].lambda} (p ${groups[g].p}, risk ${groups[g].risk}, green ${groups[g].green} in ${groups[g].takesWithGreen} takes, target ${alpha}, level ${r6(delta / 12)})`);
  console.log(`qInf ${cal.radius.qInf} (rank ${qi.k} of ${qi.n}, alpha ${ALPHA_INF})`);
  console.log(`${A.dry ? 'dry run, nothing written' : 'wrote ' + outPath} in ${((Date.now() - t0) / 1000).toFixed(0)} s${fail ? '\nFAIL: at least one learn-then-test sequence certified nothing (index -1: too few green takes, or risk above target) or qInf is undefined' : ''}`);
  process.exitCode = fail ? 1 : 0;
}

if (!isMainThread && workerData?.tool === 'calibrate') workerMain();
else if (isMainThread && process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) main().catch(e => { console.error(e); process.exit(2); });
