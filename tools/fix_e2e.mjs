#!/usr/bin/env node
// End-to-end check of the studio's one-click FIX, in a real page: starts server.py on a free port, opens
// studio.html in headless Chromium (Playwright), builds a synthetic UE-named mannequin and a take with known
// defects through the page's own MCP bridge (`eval`), clicks the FIX button, waits for it, then checks with
// wire/judge.js that the defect is gone, no blocker is new, the report is plain English, one Undo restores
// the original and nothing was exported. Prints a table; exit code 1 when a check fails.
//
//   node tools/fix_e2e.mjs [--scenarios slide,pop,...] [--require-fix] [--wait-fix 30] [--json out.json] [--verbose] [--headed]
//                          [--rev HEAD]   serve the studio's .js/.html/.css from a git revision instead of the working tree
//                          [--mock-fix]   harness self-test: add a stand-in FIX button wired as wire/INTEGRATION.md section 3
//
// No character or take files are needed. three.js comes from vendor/three (server.py downloads it from
// npm on start) and is served in place of the CDN, so the page needs no other network access.
import { spawn, execFileSync } from 'node:child_process';
import { createServer } from 'node:net';
import { existsSync, mkdtempSync, mkdirSync, readdirSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { makeTake, rotateLocal, smoothstep, NAMES, PARENTS, OFFSETS, REST_PELVIS } from '../tests/fixrig.mjs';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const argv = process.argv.slice(2);
const arg = (k, d) => { const i = argv.indexOf('--' + k); return i < 0 ? d : argv[i + 1] && !argv[i + 1].startsWith('--') ? argv[i + 1] : true; };
const OPT = {
  scenarios: arg('scenarios', ''), requireFix: !!arg('require-fix', false), waitFix: +arg('wait-fix', 0) || 0, json: arg('json', null),
  verbose: !!arg('verbose', false), headed: !!arg('headed', false), timeout: (+arg('timeout', 240) || 240) * 1000, port: +arg('port', 0) || 0,
  selector: arg('selector', process.env.FIX_E2E_SELECTOR || null), rev: arg('rev', null), mockFix: !!arg('mock-fix', false),
};
const log = (...a) => OPT.verbose && console.log('  ·', ...a);

// ---------- scenarios: fixrig takes (cm, 30 fps, Y up) with one known defect each ----------
const hold = (f, a, b, e) => f < a ? smoothstep((f - (a - e)) / e) : f <= b ? 1 : smoothstep((b + e - f) / e);
const sway = f => [0, 100 + 0.5 * Math.sin(2 * Math.PI * f / 45), 0.3 * Math.sin(2 * Math.PI * f / 60)];
const SCENARIOS = {
  stand: { what: 'clean stand (control)', rules: [], take: () => makeTake({ n: 90, pelvis: sway }) },
  slide: { what: 'foot slide in a stand', rules: ['foot_slide'], take: () => makeTake({ n: 90, pelvis: sway, ankle: { l: f => [10, 8, 20 * smoothstep((f - 40) / 10)], r: () => [-10, 8, 0] } }) },
  float: { what: 'floating feet', rules: ['ground_float'], take: () => makeTake({ n: 90, pelvis: sway, ankle: { l: f => [10, 8 + 13 * hold(f, 40, 49, 6), 0], r: f => [-10, 8 + 13 * hold(f, 40, 49, 6), 0] } }) },
  pop: { what: 'single-frame pop', rules: ['pop'], take: () => rotateLocal(makeTake({ n: 90, pelvis: sway }), 'spine_03', 45, [1, 0, 0], 40) },
  sink: { what: 'foot through floor', rules: ['ground_sink'], take: () => makeTake({ n: 90, pelvis: f => [0, 100 - 6 * hold(f, 40, 55, 6), 0], ankle: { l: f => [10, 8 - 6 * hold(f, 40, 55, 6), 0], r: f => [-10, 8 - 6 * hold(f, 40, 55, 6), 0] } }) },
  knee: { what: 'knee pop', rules: ['knee_pop'], take: () => rotateLocal(makeTake({ n: 90, pelvis: sway }), 'calf_l', 50, [1, 0, 0], 25) },
  seam: {
    what: 'seam jump between two shots', rules: ['seam_jump'],
    take: () => makeTake({ n: 120, pelvis: f => [f >= 60 ? 30 : 0, 100 + 0.5 * Math.sin(2 * Math.PI * f / 45), 0], ankle: { l: f => [10 + (f >= 60 ? 30 : 0), 8, 0], r: f => [-10 + (f >= 60 ? 30 : 0), 8, 0] }, shots: [{ name: 'shotA', start: 0, n: 60 }, { name: 'shotB', start: 60, n: 60 }] }),
  },
  combo: {
    what: 'all of the above in one two-shot take', rules: ['pop', 'knee_pop', 'seam_jump', 'ground_float', 'foot_slide'],
    take: () => {
      const sx = f => f >= 75 ? 30 : 0;
      let t = makeTake({ n: 150, pelvis: f => [sx(f), 100 + 0.5 * Math.sin(2 * Math.PI * f / 45), 0.3 * Math.sin(2 * Math.PI * f / 60)],
        ankle: { l: f => [10 + sx(f), 8 + 13 * hold(f, 95, 104, 6), 20 * smoothstep((f - 125) / 10)], r: f => [-10 + sx(f), 8 + 13 * hold(f, 95, 104, 6), 0] },
        shots: [{ name: 'shotA', start: 0, n: 75 }, { name: 'shotB', start: 75, n: 75 }] });
      t = rotateLocal(t, 'spine_03', 30, [1, 0, 0], 40);
      return rotateLocal(t, 'calf_l', 50, [1, 0, 0], 25);
    },
  },
};

// The page body: fixrig's 22 UE-named bones plus a head_end so the rest height is 178 cm (studio upm = 100).
const BODY = {
  name: 'E2E_Mannequin.fbx',
  bones: [...NAMES.map((name, i) => ({ name, parent: PARENTS[i], offset: i ? OFFSETS[i] : REST_PELVIS })), { name: 'head_end', parent: NAMES.indexOf('head'), offset: [0, 16, 0] }],
};
const takeSpec = (key, t) => ({ key, fps: t.fps, n: t.n, names: NAMES, q: Array.from(t.q, v => +v.toFixed(7)), p: Array.from(t.p, v => +v.toFixed(5)), shots: t.shots });

// ---------- page-side code (runs through the bridge `eval`: S, THREE, api) ----------
const LOAD_CODE = (body, take) => `
const body = ${JSON.stringify(body)}, take = ${JSON.stringify(take)};
const { FBXLoader } = await import('three/addons/loaders/FBXLoader.js');
const build = () => { const g = new THREE.Group(); g.name = 'E2E'; const bs = body.bones.map(b => { const o = new THREE.Bone(); o.name = b.name; o.position.fromArray(b.offset); return o; }); body.bones.forEach((b, i) => (b.parent < 0 ? g : bs[b.parent]).add(bs[i])); return g; };
const P = FBXLoader.prototype, orig = P.parse;
P.parse = function () { return build(); };
try { S.take = null; await api.loadBodyBytes(new TextEncoder().encode(JSON.stringify(body)).buffer, body.name, { location: 'samples', name: body.name }); } finally { P.parse = orig; }
const nb = S.body.bones.length, src = S.body.bones.map(b => take.names.indexOf(b.name)), snb = take.names.length;
for (const sh of take.shots) {
  const q = new Float32Array(sh.n * nb * 4), p = new Float32Array(sh.n * nb * 3);
  for (let f = 0; f < sh.n; f++) for (let i = 0; i < nb; i++) {
    const j = src[i], F = sh.start + f;
    if (j < 0) { S.body.restQ[i].toArray(q, (f * nb + i) * 4); S.body.restP[i].toArray(p, (f * nb + i) * 3); continue; }
    for (let k = 0; k < 4; k++) q[(f * nb + i) * 4 + k] = take.q[(F * snb + j) * 4 + k];
    for (let k = 0; k < 3; k++) p[(f * nb + i) * 3 + k] = take.p[(F * snb + j) * 3 + k];
  }
  api.appendTake({ fps: take.fps, n: sh.n, q, p, matched: snb, total: nb }, sh.name);
}
api.renderShots(); api.rebuild();
return { bones: nb, frames: S.take.n, shots: S.take.shots.length, upm: S.upm };`;

// Installed once per page: the reference judge (wire/judge.js on the page's own S, fixed floor) and export spies.
async function installProbe(page) {
  await page.evaluate(async () => {
    const [{ judgeMotion }, { rigIndices, computeFacts }, { GLTFExporter }] = await Promise.all([import('/wire/judge.js'), import('/wire/facts.js'), import('three/addons/exporters/GLTFExporter.js')]);
    const S = window.__fulcrum, E = window.__e2e = { exports: [], ground: null };
    const num = (id, d) => +document.getElementById(id)?.value || d;
    const rigOf = () => rigIndices(k => S.body.byKey.get(k));
    E.floor = () => computeFacts({ W: S.out.W, nb: S.body.bones.length, n: S.take.n, fps: S.take.fps, upm: S.upm, rig: rigOf(), restW: S.body.restW, shots: S.take.shots }).ground;
    E.judgeFull = () => {
      const nb = S.body.bones.length, rig = rigOf();
      return judgeMotion({ motion: S.out, source: { q: S.take.q, p: S.take.p }, nb, n: S.take.n, fps: S.take.fps, upm: S.upm, rig, names: S.body.bones.map(b => b.name), parents: S.body.parentIdx, restW: S.body.restW,
        shots: S.take.shots, trust: S.take.trust ?? null, state: { fixes: S.fixes, locks: S.locks, pins: S.pins }, ground: S.factsGround ?? E.ground, opts: { mu: num('mu', 0.8), snapLim: num('snapLim', 900), gateLim: num('gateLim', 8) } });
    };
    E.judge = () => {
      const j = E.judgeFull();
      return { counts: j.counts, cost: +j.cost.toFixed(1), score: +j.score.toFixed(1), fidelityDeg: +j.fidelity.worst.toFixed(2), worstBone: j.fidelity.worstBone,
        findings: j.findings.map(f => ({ rule: f.rule, severity: f.severity, side: f.side ?? null, a: f.a, b: f.b, text: f.text })) };
    };
    E.state = () => JSON.parse(JSON.stringify({ fixes: S.fixes, locks: S.locks, pins: S.pins }));
    E.keepW = () => { E.W0 = S.out.W.slice(); };
    E.diffW = () => { const W = S.out.W; if (!E.W0 || W.length !== E.W0.length) return Infinity; let m = 0; for (let i = 0; i < W.length; i++) m = Math.max(m, Math.abs(W[i] - E.W0[i])); return m / S.upm * 100; };
    E.app = () => ({ judgement: S.judgement ? { counts: S.judgement.counts } : null, fixRun: S.fixRun ? { report: S.fixRun.report, kept: (S.fixRun.kept || []).map(k => ({ id: k.id, rule: k.rule, label: k.label, detail: k.detail })), reverted: (S.fixRun.reverted || []).length, reason: S.fixRun.reason } : null,
      autoFix: S.autoFixReport ? { kept: S.autoFixReport.kept?.length ?? 0, reverted: S.autoFixReport.reverted?.length ?? 0, reason: S.autoFixReport.reason } : null, busy: !!(S.fixing || S.autoFixing) });
    const spy = (o, k, tag) => { const f = o[k]; o[k] = function (...a) { E.exports.push(tag); return f.apply(this, a); }; };
    spy(GLTFExporter.prototype, 'parse', 'GLTFExporter.parse'); spy(GLTFExporter.prototype, 'parseAsync', 'GLTFExporter.parseAsync');
    for (const id of ['exportFbx', 'exportGlb', 'saveProj']) {
      const el = document.getElementById(id);
      if (!el) continue;
      el.addEventListener('click', () => E.exports.push('click #' + id), true);
      if (el.onclick) { const f = el.onclick; el.onclick = function (...a) { E.exports.push('#' + id + '.onclick'); return f.apply(this, a); }; }
    }
  });
}

// Stand-in for the integrator's button (self-test only): runFix on the page's rebuild(), one undo row, no export.
async function installMockFix(page) {
  await page.evaluate(async () => {
    const { runFix } = await import('/wire/fixengine.js'), S = window.__fulcrum, E = window.__e2e;
    const shelf = document.querySelector('.tool-shelf'), b = document.createElement('button'), out = document.createElement('pre');
    b.id = 'fix'; b.textContent = 'FIX'; out.id = 'fixLog'; shelf.append(b); shelf.after(out);
    b.onclick = async () => {
      out.textContent = ''; S.autoFixing = true; S.fixing = true;
      const copy = x => structuredClone(x);
      try {
        const r = await runFix({ state: { fixes: S.fixes, locks: S.locks, pins: S.pins },
          apply: st => { S.fixes = copy(st.fixes); S.locks = copy(st.locks); S.pins = copy(st.pins); window.__fulcrumRebuild(); return S.out; },
          evaluate: () => E.judgeFull(), log: l => { out.textContent += l + '\n'; } });
        S.fixes = r.state.fixes; S.locks = r.state.locks; S.pins = r.state.pins; S.fixRun = r;
      } finally { S.autoFixing = false; S.fixing = false; }
      window.__fulcrumRebuild();
    };
  });
}

// ---------- FIX button discovery ----------
const FIX_SELECTORS = ['#fix', '#fixAll', '#fixBtn', '#fixButton', '[data-action="fix"]', '[data-action="fixAll"]', 'button:text-is("FIX")', 'button:text-is("Fix")'];
async function findFix(page) {
  const end = Date.now() + OPT.waitFix * 1000;
  for (;;) {
    for (const sel of OPT.selector ? [OPT.selector] : FIX_SELECTORS) if (await page.locator(sel).count()) return { sel, legacy: false };
    if (Date.now() >= end) break;
    await page.waitForTimeout(500);
  }
  if (!OPT.requireFix && await page.locator('#autoFix').count()) return { sel: '#autoFix', legacy: true };
  return null;
}
// Make a control clickable the way a user would: open its workspace and any collapsed <details>.
async function reveal(page, sel) {
  await page.locator(sel).first().evaluate(el => {
    for (let d = el.closest('details'); d; d = d.parentElement?.closest('details')) d.open = true;
    const sec = el.closest('section[data-workspaces]');
    if (sec && sec.hidden) document.querySelector(`.workspace-nav [data-workspace="${sec.dataset.workspaces.split(' ')[0]}"]`)?.click();
  });
}
const REPORT_SELECTORS = ['#fixReport', '#fixLog', '#fixStatus', '#fixList', '#fixLayers', '#autoFixLog'];
// Text the user can read, including a log inside a collapsed <details> ("Full report"), whose innerText is empty until opened.
const READ_TEXT = 'el => el ? (el.innerText.trim() ? el.innerText : el.textContent) : null';
const reportText = page => page.evaluate(([sels, rd]) => { const read = eval(rd); return sels.map(s => [s, read(document.querySelector(s))]).filter(([, t]) => t != null && t.trim()); }, [REPORT_SELECTORS, READ_TEXT]);

// ---------- plain-English check on the report the user sees ----------
function plainEnglish(lines, { legacy, bones }) {
  const text = lines.join('\n'), issues = [];
  if (!text.trim()) return ['no report text'];
  if (/\bundefined\b|\bNaN\b|\[object|\bnull\b|Infinity/.test(text)) issues.push('raw value (undefined/NaN/null/[object])');
  if (/[{}]|":/.test(text)) issues.push('JSON in the text');
  const snake = [...new Set((text.match(/\b[a-z][a-z0-9]*_[a-z0-9_]+\b/g) || []).filter(w => !bones.has(w)))];
  if (snake.length) issues.push('code names: ' + snake.slice(0, 4).join(', '));
  const camel = [...new Set(text.match(/\b[a-z]+[A-Z][A-Za-z]*\b/g) || [])];
  if (camel.length) issues.push('code names: ' + camel.slice(0, 4).join(', '));
  if (/\b(Error|TypeError|ReferenceError|failed:)/.test(text)) issues.push('error text');
  if (!legacy) {
    if (!lines.some(l => /^\s*Done\b/.test(l))) issues.push('no "Done" line');
    if (!lines.some(l => /Stopped before export/.test(l))) issues.push('no "Stopped before export" line');
  }
  return issues;
}

// ---------- infrastructure ----------
const freePort = () => new Promise((ok, no) => { const s = createServer(); s.unref(); s.on('error', no); s.listen(0, '127.0.0.1', () => { const { port } = s.address(); s.close(() => ok(port)); }); });
const sleep = ms => new Promise(r => setTimeout(r, ms));
async function getJSON(url, init) { const r = await fetch(url, init); return r.json(); }

async function startServer(port, home) {
  const py = process.env.PYTHON || 'python3';
  const proc = spawn(py, [join(ROOT, 'server.py'), String(port)], { cwd: ROOT, env: { ...process.env, APPDATA: home, PYTHONDONTWRITEBYTECODE: '1' }, stdio: ['ignore', 'pipe', 'pipe'] });
  let out = '';
  proc.stdout.on('data', d => { out += d; }); proc.stderr.on('data', d => { out += d; });
  const base = `http://127.0.0.1:${port}`, end = Date.now() + 30000;
  for (;;) {
    if (proc.exitCode != null) throw new Error('server.py exited:\n' + out);
    try { await getJSON(base + '/api/mocap/status'); break; } catch { if (Date.now() > end) throw new Error('server.py did not answer:\n' + out); await sleep(200); }
  }
  // server.py fetches three.js into vendor/ in the background; the page needs it before it can boot.
  const vend = Date.now() + 180000;
  while (!existsSync(join(ROOT, 'vendor/three/build/three.module.js'))) {
    if (Date.now() > vend) throw new Error('vendor/three is missing and server.py could not download it (needs registry.npmjs.org).\n' + out);
    await sleep(500);
  }
  return { proc, base, output: () => out };
}

async function loadPlaywright() {
  for (const m of [process.env.PLAYWRIGHT_MODULE, 'playwright', '/opt/node-tools/node_modules/playwright/index.mjs'].filter(Boolean)) {
    try { return await import(m.startsWith('/') ? pathToFileURL(m).href : m); } catch {}
  }
  throw new Error('Playwright not found: set PLAYWRIGHT_MODULE to its index.mjs');
}

async function bridge(base, cmd, args = {}, timeout = 300) {
  const r = await getJSON(base + '/api/bridge/call', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ cmd, args, timeout }) });
  if (!r.ok) throw new Error(`bridge ${cmd}: ${r.error}`);
  return r.result;
}

const REV = new Map();
const atRev = rel => { if (!REV.has(rel)) { try { REV.set(rel, execFileSync('git', ['show', `${OPT.rev}:${rel}`], { cwd: ROOT, encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore'], maxBuffer: 64 << 20 })); } catch { REV.set(rel, null); } } return REV.get(rel); };
async function openStudio(browser, base) {
  const ctx = await browser.newContext({ viewport: { width: 1440, height: 900 }, acceptDownloads: true });
  const three = join(ROOT, 'vendor/three');
  await ctx.route(/^https?:\/\//, route => {
    const u = route.request().url();
    if (u.startsWith(base)) {
      const rel = decodeURIComponent(new URL(u).pathname).replace(/^\//, '') || 'index.html', body = OPT.rev && /\.(m?js|html|css)$/.test(rel) ? atRev(rel) : null;
      return body == null ? route.continue() : route.fulfill({ body, contentType: rel.endsWith('.css') ? 'text/css' : rel.endsWith('.html') ? 'text/html' : 'text/javascript' });
    }
    const m = u.match(/^https:\/\/cdn\.jsdelivr\.net\/npm\/three@[\d.]+\/(.+?)(\?.*)?$/);
    if (m && existsSync(join(three, m[1]))) return route.fulfill({ path: join(three, m[1]), contentType: 'text/javascript' });
    return route.abort();
  });
  const page = await ctx.newPage(), net = { exports: [], downloads: 0, errors: [] };
  page.on('request', r => { const p = new URL(r.url()).pathname; if (/^\/api\/(export-fbx|glb-to-fbx|save\/)/.test(p)) net.exports.push(r.method() + ' ' + p); });
  page.on('download', () => { net.downloads++; });
  page.on('pageerror', e => net.errors.push(String(e.message || e)));
  page.on('console', m => { if (m.type() === 'error' && !/^Failed to load resource/.test(m.text())) net.errors.push(m.text()); });
  page.on('response', r => { const u = new URL(r.url()); if (r.status() >= 400 && u.origin === base && !/^\/(favicon\.ico|exports\/frt_pose_model[^/]*\.json)$/.test(u.pathname)) net.errors.push(`HTTP ${r.status()} ${u.pathname}`); });
  await page.goto(base + '/studio.html', { waitUntil: 'load' });
  await page.waitForFunction(() => window.__fulcrum && document.getElementById('status'), null, { timeout: 30000 });
  const end = Date.now() + 30000;
  while ((await getJSON(base + '/api/bridge/alive')).page_seen_s_ago == null || (await getJSON(base + '/api/bridge/alive')).page_seen_s_ago > 5) {
    if (Date.now() > end) throw new Error('studio page never reached the bridge; page errors: ' + net.errors.join(' | '));
    await sleep(200);
  }
  return { ctx, page, net };
}

// ---------- one scenario ----------
async function runScenario(browser, base, key, home) {
  const sc = SCENARIOS[key], row = { key, what: sc.what, rules: sc.rules, checks: {}, notes: [] };
  const t0 = Date.now();
  let ctx = null, page, net = { errors: [] };
  try {
    ({ ctx, page, net } = await openStudio(browser, base));
    const loaded = await bridge(base, 'eval', { code: LOAD_CODE(BODY, takeSpec(key, sc.take())) });
    log(key, 'loaded', JSON.stringify(loaded));
    await installProbe(page);
    if (OPT.mockFix) await installMockFix(page);
    await page.evaluate(() => { window.__e2e.ground = window.__fulcrum.factsGround ?? window.__e2e.floor(); window.__e2e.keepW(); });
    const before = await page.evaluate(() => window.__e2e.judge()), state0 = await page.evaluate(() => window.__e2e.state());
    const hist0 = (await bridge(base, 'status')).guided.history;
    row.before = before.counts;
    const present = sc.rules.filter(r => before.findings.some(f => f.rule === r && f.severity !== 'info'));
    row.checks.injected = present.length === sc.rules.length && (sc.rules.length > 0 || before.counts.blocker + before.counts.warning === 0);
    if (!row.checks.injected) row.notes.push('defect not detected before FIX: ' + sc.rules.filter(r => !present.includes(r)).join(', ') + ' / found ' + before.findings.map(f => f.rule).join(', '));
    log(key, 'before', before.findings.map(f => `${f.severity}:${f.rule}`).join(' '));

    const btn = await findFix(page);
    if (!btn) throw new Error('no FIX button (' + FIX_SELECTORS.join(' ') + ')');
    row.button = btn.sel + (btn.legacy ? ' (legacy Auto Fix)' : '');
    const report0 = JSON.stringify(await reportText(page)), app0 = await page.evaluate(() => window.__e2e.app()), exportLog0 = await page.locator('#exportLog').innerText().catch(() => '');
    await reveal(page, btn.sel);
    const tf = Date.now();
    await page.locator(btn.sel).first().click();
    await page.waitForFunction(([r0, run0, sels, rd]) => {
      const E = window.__e2e, a = E.app(), read = eval(rd);
      const txt = sels.map(s => read(document.querySelector(s)) ?? '').join('\n');
      const changed = JSON.stringify(sels.map(s => [s, read(document.querySelector(s))]).filter(([, t]) => t != null && t.trim())) !== r0 || JSON.stringify(a.fixRun) !== run0;
      return !a.busy && changed && /(^|\n)\s*Done\b|\b(FIX|Fix|Auto Fix) failed/.test(txt + '\n' + (a.fixRun?.report || []).join('\n'));
    }, [report0, JSON.stringify(app0.fixRun), REPORT_SELECTORS, READ_TEXT], { timeout: OPT.timeout, polling: 250 });
    row.fixMs = Date.now() - tf;
    const after = await page.evaluate(() => window.__e2e.judge()), app = await page.evaluate(() => window.__e2e.app()), state1 = await page.evaluate(() => window.__e2e.state());
    const hist1 = (await bridge(base, 'status')).guided.history;
    row.after = after.counts; row.fidelityDeg = after.fidelityDeg;
    log(key, 'after', after.findings.map(f => `${f.severity}:${f.rule}`).join(' ') || 'clean');

    const left = sc.rules.filter(r => after.findings.some(f => f.rule === r && f.severity !== 'info'));
    row.checks.defectGone = left.length === 0;
    if (left.length) row.notes.push('still there: ' + after.findings.filter(f => left.includes(f.rule) && f.severity !== 'info').map(f => f.text).join('; '));
    const blk0 = new Set(before.findings.filter(f => f.severity === 'blocker').map(f => f.rule));
    const newBlk = after.findings.filter(f => f.severity === 'blocker' && !blk0.has(f.rule));
    row.checks.noNewBlockers = !newBlk.length && after.counts.blocker <= before.counts.blocker;
    if (newBlk.length) row.notes.push('new blockers: ' + newBlk.map(f => f.text).join('; '));
    if (after.counts.blocker) row.notes.push(`${after.counts.blocker} blocker(s) left: ` + after.findings.filter(f => f.severity === 'blocker').map(f => f.text).join('; '));
    if (app.judgement && app.judgement.counts.blocker !== after.counts.blocker) row.notes.push(`app judgement says ${app.judgement.counts.blocker} blocker(s), reference judge ${after.counts.blocker}`);

    const shown = (await reportText(page)).flatMap(([, t]) => t.split('\n')).map(s => s.trim()).filter(Boolean);
    const lines = shown.length ? shown : app.fixRun?.report || [];
    row.report = lines;
    const bones = new Set(BODY.bones.map(b => b.name));
    const pe = plainEnglish(lines, { legacy: btn.legacy, bones });
    row.checks.report = !pe.length;
    if (pe.length) row.notes.push('report: ' + pe.join('; '));
    if (app.fixRun?.report?.length && shown.length && !app.fixRun.report.every(l => shown.join('\n').includes(l.split(':')[0]))) row.notes.push('report on screen differs from S.fixRun.report');

    const changed = JSON.stringify(state1) !== JSON.stringify(state0);
    row.kept = app.fixRun?.kept?.map(k => k.detail || k.label) ?? (app.autoFix ? [`${app.autoFix.kept} kept`] : []);
    row.oneUndoRow = changed ? hist1.undo === hist0.undo + 1 : hist1.undo === hist0.undo;
    if (!row.oneUndoRow) row.notes.push(`undo rows ${hist0.undo} -> ${hist1.undo} (FIX must add exactly one${changed ? '' : ', none when nothing changed'})`);
    await page.locator('.tool-shelf [data-action="editUndo"]').click();
    await page.waitForFunction(() => !window.__e2e.app().busy, null, { timeout: 30000 });
    const undone = await page.evaluate(() => window.__e2e.state()), dW = await page.evaluate(() => window.__e2e.diffW()), back = await page.evaluate(() => window.__e2e.judge());
    row.undoDiffCm = +dW.toFixed(4);
    const sameFindings = JSON.stringify(back.findings.map(f => [f.rule, f.severity, f.a, f.b])) === JSON.stringify(before.findings.map(f => [f.rule, f.severity, f.a, f.b]));
    row.checks.undo = row.oneUndoRow && JSON.stringify(undone) === JSON.stringify(state0) && dW < 0.01 && sameFindings;
    if (JSON.stringify(undone) !== JSON.stringify(state0)) row.notes.push('undo left fixes/locks/pins: ' + JSON.stringify(undone).slice(0, 160));
    if (dW >= 0.01) row.notes.push(`undo left the motion ${dW.toFixed(2)} cm away from the original`);
    if (!sameFindings) row.notes.push('undo findings differ from the original');

    const ex = await page.evaluate(() => window.__e2e.exports), exportLog1 = await page.locator('#exportLog').innerText().catch(() => '');
    const files = existsSync(join(home, 'exports')) ? readdirSync(join(home, 'exports')) : [];
    row.checks.noExport = !ex.length && !net.exports.length && !net.downloads && exportLog1 === exportLog0 && !files.length;
    if (!row.checks.noExport) row.notes.push('export triggered: ' + [...ex, ...net.exports, net.downloads ? net.downloads + ' download(s)' : '', exportLog1 !== exportLog0 ? 'export log changed' : '', files.length ? 'files ' + files.join(',') : ''].filter(Boolean).join(', '));
    if (btn.legacy) row.notes.push('FIX button not found yet: drove the legacy Auto Fix');
    else {
      // Layout decisions (notes only): FIX visible in the top shelf, old buttons in a collapsed Advanced panel.
      const ui = await page.evaluate(sel => {
        const b = document.querySelector(sel), adv = [...document.querySelectorAll('details')].find(d => /advanced/i.test(d.querySelector('summary')?.textContent || ''));
        return { inShelf: !!b?.closest('.tool-shelf'), advanced: !!adv, collapsed: adv ? !adv.hasAttribute('open') : null, oldInside: adv ? ['autoFix', 'despikeRange', 'lockL', 'fixRange'].filter(id => adv.querySelector('#' + id)).length : 0 };
      }, btn.sel);
      if (!ui.inShelf) row.notes.push('FIX button is not in the top tool shelf');
      if (!ui.advanced) row.notes.push('no "Advanced" <details> panel found');
      else if (!ui.collapsed || !ui.oldInside) row.notes.push(`Advanced panel: ${ui.collapsed ? 'collapsed' : 'open by default'}, ${ui.oldInside} of 4 old buttons inside`);
    }
  } catch (e) {
    row.error = String(e?.message || e).split('\n')[0];
  } finally {
    if (net.errors.length) row.notes.push('page errors: ' + [...new Set(net.errors)].slice(0, 3).join(' | '));
    row.ms = Date.now() - t0;
    await ctx?.close();
  }
  row.pass = !row.error && Object.values(row.checks).length === 6 && Object.values(row.checks).every(Boolean);
  return row;
}

// ---------- table ----------
function table(rows) {
  const yn = v => v === undefined ? '-' : v ? 'ok' : 'FAIL';
  const c = x => x ? `${x.blocker}B ${x.warning}W` : '-';
  const head = ['scenario', 'defect', 'before', 'after', 'injected', 'gone', 'no new B', 'report', 'undo', 'no export', 'fid°', 'secs', 'result'];
  const body = rows.map(r => [r.key, r.rules.join('+') || '(none)', c(r.before), c(r.after), yn(r.checks.injected), yn(r.checks.defectGone), yn(r.checks.noNewBlockers), yn(r.checks.report), yn(r.checks.undo), yn(r.checks.noExport), r.fidelityDeg ?? '-', ((r.fixMs ?? 0) / 1000).toFixed(1), r.pass ? 'PASS' : 'FAIL']);
  const w = head.map((h, i) => Math.max(h.length, ...body.map(b => String(b[i]).length)));
  const line = cells => cells.map((x, i) => String(x).padEnd(w[i])).join('  ');
  console.log('\n' + line(head) + '\n' + w.map(n => '-'.repeat(n)).join('  '));
  for (const b of body) console.log(line(b));
  console.log('');
  for (const r of rows) {
    if (r.error) console.log(`${r.key}: ERROR ${r.error}`);
    for (const n of r.notes) console.log(`${r.key}: ${n}`);
    if (OPT.verbose && r.report) { console.log(`${r.key} report (${r.button}):`); for (const l of r.report) console.log('    ' + l); }
  }
}

async function main() {
  const keys = OPT.scenarios ? String(OPT.scenarios).split(',').map(s => s.trim()).filter(Boolean) : Object.keys(SCENARIOS);
  for (const k of keys) if (!SCENARIOS[k]) throw new Error(`unknown scenario ${k}; have ${Object.keys(SCENARIOS).join(', ')}`);
  const home = mkdtempSync(join(process.env.FIX_E2E_TMP || tmpdir(), 'fulcrum-e2e-'));
  mkdirSync(join(home, 'Fulcrum'), { recursive: true });
  writeFileSync(join(home, 'Fulcrum', 'fulcrum.json'), JSON.stringify({ exports: join(home, 'exports'), imports: join(home, 'imports'), samples: join(home, 'samples'), blender: join(home, 'no-blender') }));
  const port = OPT.port || await freePort(), srv = await startServer(port, home);
  console.log(`server.py on ${srv.base}  (config + exports in ${home})`);
  const { chromium } = await loadPlaywright();
  const browser = await chromium.launch({ headless: !OPT.headed, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  const rows = [];
  try {
    for (const k of keys) { process.stdout.write(`${k} … `); const r = await runScenario(browser, srv.base, k, home); rows.push(r); console.log(r.pass ? 'PASS' : 'FAIL', `(${(r.ms / 1000).toFixed(1)} s)`); }
  } finally {
    await browser.close();
    srv.proc.kill('SIGTERM');
  }
  table(rows);
  if (OPT.json) writeFileSync(OPT.json, JSON.stringify(rows, null, 1));
  rmSync(home, { recursive: true, force: true });
  const bad = rows.filter(r => !r.pass);
  console.log(bad.length ? `${bad.length}/${rows.length} scenario(s) failed.` : `All ${rows.length} scenario(s) passed.`);
  process.exitCode = bad.length ? 1 : 0;
}
main().catch(e => { console.error(e.stack || e); process.exit(2); });
