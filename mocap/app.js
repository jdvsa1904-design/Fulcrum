import * as THREE from 'three';
import { createViewer } from './viewer.js';
import { buildMannequin, skeletonLines, loadCustomCharacter } from './character.js';
import { processCapture } from './pipeline.js';
import { captureVideo, captureFromJSON } from './capture.js';
import { probeMp4, estimateFps } from './mp4.js';
import { createController, stepController, inputFromKeys, sampleClip } from './controller.js';
import { exportGLB, download, projectJSON } from './exporters.js';
import { toBVH } from './bvh.js';
import { NB, NLM, PARENT, LM_EDGES, fk, getV, buildSkeleton, DEFAULT_DIMS } from './skeleton.js';
import { IMG_STRIDE, projectPoint } from './landmarks.js';
import { qrot, qconj } from './math.js';
import { gaitClip, idleClip, jumpClip } from './gait.js';
import { lookAtCamera, synthCapture } from './synth.js';

const $ = id => document.getElementById(id);
const app = $('app'), video = $('video'), overlay = $('overlay'), tl = $('tl');
const KIND_COLOR = { walk: '--walk', run: '--run', idle: '--idle', jump: '--jump', action: '--action', take: '--take' };
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const S = {
  step: 'video', mode: 'review', video: null, capture: null, result: null, frame: 0, playing: false, preview: null,
  settings: { hfov: 60, imageFit: 0.5, fc: 6, footLock: true, height: null, person: null, trim: null },
  character: null, charFile: null, lines: null, ctl: null, down: new Set(), hit: new Set(), abort: null, tracking: false, lastDraw: -1,
};
window.fulcrumMocap = S;

let toastTimer = 0;
function toast(msg, ms = 2600) {
  const t = $('toast');
  t.textContent = msg;
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), ms);
}
function status(id, msg, kind = '') { const el = $(id); el.textContent = msg; el.className = 'status' + (kind ? ' ' + kind : ''); }

const THEME_KEY = 'fulcrum.mocap.theme';
let theme = 'system';
try { theme = localStorage.getItem(THEME_KEY) || 'system'; } catch {}
const isDark = () => theme === 'dark' || (theme === 'system' && matchMedia('(prefers-color-scheme: dark)').matches);
function applyTheme() {
  if (theme === 'system') delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = theme;
  $('theme').value = theme;
  viewer.setTheme(isDark());
  drawTimeline();
}
$('theme').addEventListener('change', e => { theme = e.target.value; try { localStorage.setItem(THEME_KEY, theme); } catch {} applyTheme(); });
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => applyTheme());

const viewer = createViewer($('three'), { dark: isDark(), preserve: true });
window.fulcrumViewer = viewer;
new ResizeObserver(() => { viewer.resize(); sizeOverlay(); drawTimeline(); }).observe($('viewPane'));
new ResizeObserver(() => { sizeOverlay(); drawTimeline(); }).observe($('videoPane'));
applyTheme();

function setStep(name) {
  S.step = name;
  for (const b of document.querySelectorAll('.steps button')) b.classList.toggle('on', b.dataset.step === name);
  for (const sec of document.querySelectorAll('.panel section[data-step]')) sec.classList.toggle('on', sec.dataset.step === name);
  if (name !== 'play' && S.mode === 'play') exitPlay();
}
function markDone() {
  const done = { video: !!(S.video || S.capture), track: !!S.result, moves: !!S.result, play: false };
  for (const b of document.querySelectorAll('.steps button')) b.classList.toggle('done', !!done[b.dataset.step] && b.dataset.step !== S.step);
}
document.querySelectorAll('.steps button').forEach(b => b.addEventListener('click', () => { setStep(b.dataset.step); markDone(); }));

function fps() { return S.video?.fps || S.capture?.video?.fps || 30; }
function totalFrames() { return S.video?.frames || S.capture?.frames?.length || 0; }
function frameRange() {
  const n = totalFrames(), f = fps(), a = Math.max(0, Math.round((+$('trimA').value || 0) * f)), bIn = +$('trimB').value;
  const b = bIn > 0 ? Math.min(n - 1, Math.round(bIn * f) - 1) : n - 1;
  return [Math.min(a, Math.max(0, b)), Math.max(0, b)];
}

function waitVideo(url) {
  return new Promise(resolve => {
    let done = false;
    const finish = ok => { if (done) return; done = true; video.removeEventListener('loadeddata', onData); video.removeEventListener('error', onErr); resolve(ok); };
    const onData = () => finish(video.videoWidth > 0);
    const onErr = () => finish(false);
    video.addEventListener('loadeddata', onData);
    video.addEventListener('error', onErr);
    setTimeout(() => finish(video.videoWidth > 0), 20000);
    video.src = url;
    video.load();
  });
}

async function serverConvert(file) {
  status('trackStatus', '');
  toast('This browser cannot play that format. Converting it with ffmpeg…', 8000);
  const r = await fetch('/api/transcode?name=' + encodeURIComponent(file.name), { method: 'POST', body: file });
  const j = await r.json().catch(() => ({ ok: false, error: 'The server did not answer.' }));
  if (!j.ok) throw new Error(j.error || 'Conversion failed.');
  const blob = await (await fetch(j.url)).blob();
  return { url: URL.createObjectURL(blob), info: j };
}

function resetResult() {
  S.capture = null; S.result = null; S.preview = null; S.settings.person = null;
  $('clipList').innerHTML = '';
  for (const id of ['toMoves', 'toPlay', 'playMode', 'glbBtn', 'fbxBtn', 'bvhBtn', 'saveBtn']) $(id).disabled = true;
  $('trackInfo').textContent = '';
  status('trackStatus', '');
}

async function loadVideoFile(file) {
  if (!file) return;
  if (S.tracking) S.abort?.abort();
  resetResult();
  $('videoPane').classList.add('has-video');
  $('videoPane').classList.remove('points-only');
  $('videoInfo').textContent = 'Opening ' + file.name + '…';
  let meta = null;
  if (/\.(mp4|m4v|mov|3gp)$/i.test(file.name)) meta = await probeMp4(file).catch(() => null);
  let url = URL.createObjectURL(file), converted = null;
  let ok = await waitVideo(url);
  if (!ok) {
    try {
      converted = await serverConvert(file);
      url = converted.url;
      ok = await waitVideo(url);
    } catch (e) {
      $('videoPane').classList.remove('has-video');
      $('videoInfo').textContent = '';
      toast(e.message, 9000);
      status('trackStatus', e.message, 'bad');
      return;
    }
  }
  if (!ok) { toast('Could not open this video.', 6000); return; }
  let rate = converted?.info?.fps || meta?.fps || 0;
  if (!rate) rate = await estimateFps(video) || 30;
  const frames = converted?.info?.frames || meta?.frames || Math.max(1, Math.round(video.duration * rate));
  S.video = { name: file.name, fps: rate, width: video.videoWidth, height: video.videoHeight, duration: video.duration, frames };
  $('fpsIn').value = rate;
  $('trimA').value = 0;
  $('trimB').value = (frames / rate).toFixed(2);
  $('trimB').max = $('trimA').max = (frames / rate).toFixed(2);
  showVideoInfo();
  S.frame = 0;
  await seekFrame(0);
  $('toTrack').disabled = false;
  $('trackBtn').disabled = false;
  sizeOverlay();
  drawTimeline();
  markDone();
  toast('Video ready. Next, track the person.');
}

function showVideoInfo() {
  const v = S.video;
  if (!v) return;
  $('videoInfo').textContent = `${v.name}\n${v.width}×${v.height} · ${(+v.fps).toFixed(3).replace(/\.?0+$/, '')} fps · ${v.duration.toFixed(1)} s · ${v.frames} frames`;
}

$('fpsIn').addEventListener('change', () => {
  const f = +$('fpsIn').value;
  if (!(f >= 5 && f <= 240) || !S.video) return;
  S.video.fps = f;
  S.video.frames = Math.max(1, Math.round(S.video.duration * f));
  showVideoInfo();
  drawTimeline();
});

for (const id of ['videoIn', 'videoIn2']) $(id).addEventListener('change', e => { loadVideoFile(e.target.files[0]); e.target.value = ''; });
const drop = $('drop');
for (const ev of ['dragenter', 'dragover']) $('videoPane').addEventListener(ev, e => { e.preventDefault(); drop.classList.add('over'); });
for (const ev of ['dragleave', 'drop']) $('videoPane').addEventListener(ev, e => { e.preventDefault(); drop.classList.remove('over'); });
$('videoPane').addEventListener('drop', e => {
  const f = [...(e.dataTransfer?.files ?? [])][0];
  if (!f) return;
  if (/\.json$/i.test(f.name)) openProject(f); else loadVideoFile(f);
});
$('toTrack').addEventListener('click', () => { setStep('track'); markDone(); });
$('toMoves').addEventListener('click', () => { setStep('moves'); markDone(); });
$('toPlay').addEventListener('click', () => { setStep('play'); markDone(); });

function seekFrame(f) {
  S.frame = Math.max(0, Math.min(totalFrames() - 1, Math.round(f)));
  S.clock = S.frame;
  if (!S.video || !video.src) return Promise.resolve();
  return new Promise(resolve => {
    const t = (S.frame + 0.5) / fps();
    if (Math.abs(video.currentTime - t) < 1e-3) return resolve();
    const done = () => { video.removeEventListener('seeked', done); resolve(); };
    video.addEventListener('seeked', done);
    video.currentTime = t;
    setTimeout(done, 1500);
  });
}

function settings() {
  const h = +$('heightIn').value;
  return {
    ...S.settings, hfov: +$('fov').value, imageFit: +$('imageFit').value, fc: +$('smooth').value, footLock: $('lockOn').checked,
    height: h >= 80 && h <= 230 ? h / 100 : null, trim: S.capture?.range ?? null,
  };
}

let solveTimer = 0;
function scheduleSolve() { clearTimeout(solveTimer); solveTimer = setTimeout(solve, 180); }
for (const [id, out, fmt] of [['fov', 'fovOut', v => v + '°'], ['smooth', 'smoothOut', v => v + ' Hz'], ['imageFit', 'fitOut', v => (+v).toFixed(2)]]) {
  $(id).addEventListener('input', () => { $(out).textContent = fmt($(id).value); scheduleSolve(); });
}
$('lockOn').addEventListener('change', scheduleSolve);
$('heightIn').addEventListener('change', scheduleSolve);
$('fitOn').addEventListener('change', () => { S.lastDraw = -1; });

async function runTracking() {
  if (!S.video || S.tracking) return;
  const [first, last] = frameRange();
  S.tracking = true;
  S.abort = new AbortController();
  resetResult();
  $('trackBtn').disabled = true;
  $('cancelBtn').hidden = false;
  $('progress').hidden = false;
  status('trackStatus', 'Loading the pose tracker…');
  const bar = $('progress').firstElementChild;
  try {
    const cap = await captureVideo(video, {
      fps: fps(), first, last, model: $('modelSel').value, name: S.video.name, signal: S.abort.signal,
      onFrame: (f, fr, info) => {
        S.frame = f;
        bar.style.width = (100 * info.done / info.total).toFixed(1) + '%';
        const rate = info.done / Math.max(0.001, info.ms / 1000);
        status('trackStatus', `Tracking frame ${info.done} of ${info.total} · ${rate.toFixed(1)} frames/s · ${info.delegate === 'GPU' ? 'graphics card' : 'processor'}`);
        drawPoses(fr.poses, null);
        drawTimeline();
      },
    });
    S.capture = cap;
    status('trackStatus', cap.aborted ? 'Stopped early. Using the frames tracked so far.' : `Tracked ${cap.range[1] - cap.range[0] + 1} frames.`, cap.aborted ? '' : 'ok');
    solve(true);
  } catch (e) {
    console.error(e);
    status('trackStatus', 'Tracking failed: ' + (e.message || e), 'bad');
  } finally {
    S.tracking = false;
    $('trackBtn').disabled = false;
    $('cancelBtn').hidden = true;
    $('progress').hidden = true;
  }
}
$('trackBtn').addEventListener('click', runTracking);
$('cancelBtn').addEventListener('click', () => S.abort?.abort());

function solve(first = false) {
  if (!S.capture) return;
  let res;
  try {
    res = processCapture(S.capture, settings());
  } catch (e) {
    console.error(e);
    status('trackStatus', e.message || String(e), 'bad');
    return;
  }
  const prevSkel = S.result?.skel;
  S.result = res;
  const r = res.report;
  $('trackInfo').textContent = [
    `${r.frames} frames (${r.seconds.toFixed(1)} s) of person ${res.track.id + 1}${res.tracks.length > 1 ? ` of ${res.tracks.length} found` : ''}`,
    `Body height ${(res.skel.dims.height * 100).toFixed(0)} cm · camera ${res.camera.height_m.toFixed(2)} m up`,
    `${r.plants} foot plants · solved in ${r.ms} ms`,
    res.tracks.length > 1 ? 'Wrong person? Click them in the video.' : '',
  ].filter(Boolean).join('\n');
  $('videoPane').classList.toggle('pickable', res.tracks.length > 1 && !!S.video);
  for (const id of ['toMoves', 'toPlay', 'playMode', 'glbBtn', 'fbxBtn', 'bvhBtn', 'saveBtn']) $(id).disabled = false;
  if (!prevSkel || prevSkel.dims.height !== res.skel.dims.height || first) useCharacter();
  S.ctl = createController(res.library);
  renderClips();
  renderLegend();
  if (first) {
    const r0 = getV(res.take.root, 0);
    viewer.frame(new THREE.Vector3(r0[0], 0.9, r0[2]), 4.8, [0.55, 0.32, 1]);
    seekFrame(res.first);
  }
  S.lastDraw = -1;
  drawTimeline();
  markDone();
}

async function useCharacter() {
  if (!S.result) return;
  const skel = S.result.skel;
  if (S.charFile && $('charSel').value === 'custom') {
    try {
      const ch = await loadCustomCharacter(S.charFile, skel);
      S.character?.dispose?.();
      S.character = ch;
      viewer.setCharacter(ch);
      status('charStatus', `Using ${S.charFile.name}.`, 'ok');
    } catch (e) {
      status('charStatus', e.message, 'bad');
      $('charSel').value = 'mannequin';
    }
  }
  if (!S.charFile || $('charSel').value === 'mannequin') {
    S.character?.dispose?.();
    S.character = buildMannequin(skel);
    viewer.setCharacter(S.character);
  }
  if (S.lines) viewer.extras.remove(S.lines.object);
  S.lines = skeletonLines(skel, 0x2f80ed);
  S.lines.object.visible = false;
  viewer.extras.add(S.lines.object);
}

$('charIn').addEventListener('change', async e => {
  const f = e.target.files[0];
  e.target.value = '';
  if (!f) return;
  S.charFile = f;
  if (![...$('charSel').options].some(o => o.value === 'custom')) $('charSel').add(new Option('Your character', 'custom'));
  $('charSel').options[1].textContent = f.name;
  $('charSel').value = 'custom';
  status('charStatus', 'Loading ' + f.name + '…');
  await useCharacter();
});
$('charSel').addEventListener('change', useCharacter);

const KEYS_FOR = { walk: ['W', 'A', 'S', 'D'], run: ['Shift'], idle: [], jump: ['Space'], take: ['R'] };
function keysFor(clip) {
  const lib = S.result?.library;
  if (!lib) return [];
  if (clip === lib.walk) return KEYS_FOR.walk;
  if (clip === lib.run) return ['Shift', '+', 'W'];
  if (clip === lib.jump) return KEYS_FOR.jump;
  if (clip === lib.take) return ['R'];
  const i = lib.actions.indexOf(clip);
  if (i >= 0) return [...(i < 10 ? [String((i + 1) % 10)] : []), ...(i === S.ctl?.selected ? ['E'] : ['Q'])];
  return [];
}

function clipMeta(c) {
  const len = (c.n / c.fps).toFixed(1) + ' s';
  if (c.kind === 'walk' || c.kind === 'run') return `${c.speed.toFixed(2)} m/s · step cycle ${c.cycle.toFixed(2)} s` + (c.derived ? ` · from the video (${c.derived})` : c.procedural ? '' : ' · from the video');
  if (c.kind === 'idle') return c.procedural ? 'standing, breathing' : `${len} loop · plays when no key is held`;
  if (c.kind === 'jump') return c.procedural ? 'made-up jump' : `${len} · ${((c.height ?? 0) * 100).toFixed(0)} cm off the floor`;
  if (c.kind === 'take') return `${len} · the whole video, moving through space`;
  return `${len} · moves ${(c.rootMotion?.distance ?? 0).toFixed(1)} m`;
}

function libraryClips() {
  const lib = S.result.library, list = [];
  const push = c => { if (c && !list.includes(c)) list.push(c); };
  [lib.idle, lib.walk, lib.run, lib.jump, ...lib.actions, lib.take].forEach(push);
  S.result.clips.forEach(push);
  return list;
}

function renderClips() {
  const ul = $('clipList'), lib = S.result.library;
  ul.innerHTML = '';
  for (const c of libraryClips()) {
    const li = document.createElement('li'), keys = keysFor(c), used = [lib.idle, lib.walk, lib.run, lib.jump, lib.take, ...lib.actions].includes(c);
    li.innerHTML = `<span class="sw"></span><span class="name"></span><span class="keyset"></span><span class="act"></span><span class="meta"></span>`;
    li.querySelector('.sw').style.background = css(KIND_COLOR[c.kind] ?? '--take');
    li.querySelector('.name').textContent = c.name;
    if (c.procedural) li.querySelector('.name').insertAdjacentHTML('beforeend', '<span class="made">made up</span>');
    li.querySelector('.keyset').innerHTML = keys.map(k => k === '+' ? '+' : `<kbd>${k}</kbd>`).join('') || (c === lib.idle ? '<span class="meta">no key</span>' : '');
    li.querySelector('.meta').textContent = clipMeta(c) + (used ? '' : ' · not in use');
    const act = li.querySelector('.act');
    if (c !== lib[c.kind] && (c.kind === 'walk' || c.kind === 'run' || c.kind === 'idle')) {
      const b = document.createElement('button');
      b.className = 'btn small';
      b.textContent = 'Use this ' + c.kind;
      b.addEventListener('click', e => { e.stopPropagation(); const i = lib.actions.indexOf(c); if (i >= 0) lib.actions[i] = lib[c.kind].derived ? lib[c.kind].source ?? c : lib[c.kind]; if (lib.actions[i] === c || lib[c.kind].procedural) lib.actions.splice(i, 1); lib[c.kind] = c; S.ctl = createController(lib); renderClips(); renderLegend(); });
      act.appendChild(b);
    }
    li.addEventListener('click', () => {
      S.preview = S.preview?.clip === c ? null : { clip: c, t: 0 };
      for (const x of ul.children) x.classList.remove('sel');
      if (S.preview) li.classList.add('sel');

    });
    ul.appendChild(li);
  }
}

function renderLegend() {
  const lib = S.result.library, from = c => c.procedural ? 'made up' : c.derived ? `from the video (${c.derived})` : 'from the video', sel = lib.actions[S.ctl?.selected ?? 0];
  const rows = [
    [['W', 'A', 'S', 'D'], `walk (${from(lib.walk)})`], [['Shift'], `run (${from(lib.run)})`], [['Alt'], 'slow walk'], [['Space'], `jump (${from(lib.jump)})`], [['F'], 'full screen'],
    ...(lib.actions.length ? [[['Q', 'Shift+Q'], 'next / previous move'], [['E', 'Click'], `play selected move: ${sel?.name ?? ''}`]] : []),
    ...lib.actions.map((a, i) => [i < 10 ? [String((i + 1) % 10)] : ['Q'], a.name + (a === sel ? ' (selected)' : '')]), [['R'], 'replay the whole video'], [['Esc'], 'stop / leave play'],
  ];
  $('legend').innerHTML = rows.map(([k, t]) => `<span>${k.map(x => `<kbd>${x}</kbd>`).join(' ')}</span><span>${t.replace(/</g, '&lt;')}</span>`).join('');
}

function enterPlay() {
  if (!S.result) return;
  S.preview = null;
  S.playing = false;
  video.pause();
  S.mode = 'play';
  app.dataset.mode = 'play';
  S.ctl = createController(S.result.library);
  S.down.clear(); S.hit.clear();
  viewer.follow.yaw = Math.PI;
  viewer.follow.pitch = 0.3;
  viewer.follow.dist = 4.2;
  requestAnimationFrame(() => { viewer.resize(); viewer.setFollow(true); });
  $('playMode').textContent = 'Stop playing';
  document.activeElement?.blur?.();
  $('three').focus({ preventScroll: true });
  toast('Use W A S D to move, Shift to run, Space to jump, Q to pick a move, E or click to play it.');
}
function exitPlay() {
  S.mode = 'review';
  app.dataset.mode = 'review';
  viewer.setFollow(false);
  $('playMode').textContent = 'Start playing';
  requestAnimationFrame(() => { viewer.resize(); sizeOverlay(); drawTimeline(); const f = S.character?.focus?.(); if (f) viewer.frame(new THREE.Vector3(f.x, 0.9, f.z), 4.8, [0.55, 0.32, 1]); });
}
$('playMode').addEventListener('click', () => S.mode === 'play' ? exitPlay() : enterPlay());

const typing = e => /^(INPUT|SELECT|TEXTAREA)$/.test(e.target?.tagName) && e.target.type !== 'range' && e.target.type !== 'checkbox';
window.addEventListener('keydown', e => {
  if (typing(e)) return;
  if (S.mode === 'play') {
    if (e.code === 'Escape' && !S.ctl?.action) { if (document.fullscreenElement) document.exitFullscreen(); exitPlay(); return; }
    if (e.code === 'KeyF' && !e.repeat) {
      if (document.fullscreenElement) document.exitFullscreen();
      else $('viewPane').requestFullscreen?.().catch(() => {});
      return;
    }
    if (['Space', 'Enter', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'AltLeft', 'AltRight', 'Tab'].includes(e.code)) e.preventDefault();
    if (!e.repeat) S.hit.add(e.code);
    S.down.add(e.code);
    return;
  }
  if (e.code === 'Space') { e.preventDefault(); togglePlay(); }
  else if (e.code === 'ArrowRight') { e.preventDefault(); pause(); seekFrame(S.frame + 1); }
  else if (e.code === 'ArrowLeft') { e.preventDefault(); pause(); seekFrame(S.frame - 1); }
});
window.addEventListener('keyup', e => {
  S.down.delete(e.code);
  if (S.mode === 'play' && !typing(e) && ['Space', 'Enter'].includes(e.code)) e.preventDefault();
});
window.addEventListener('blur', () => S.down.clear());
let press = null;
$('three').addEventListener('pointerdown', e => { press = S.mode === 'play' && e.button === 0 ? { x: e.clientX, y: e.clientY } : null; });
$('three').addEventListener('pointerup', e => { if (press && S.mode === 'play' && Math.hypot(e.clientX - press.x, e.clientY - press.y) < 5) S.hit.add('Click'); press = null; });

function pause() { S.playing = false; video.pause(); $('playBtn').textContent = '▶'; }
function togglePlay() {
  if (!totalFrames()) return;
  S.preview = null;
  S.playing = !S.playing;
  $('playBtn').textContent = S.playing ? '❚❚' : '▶';
  if (S.video && video.src) {
    if (S.playing) { if (S.frame >= totalFrames() - 1) seekFrame(0); video.play().catch(() => {}); }
    else video.pause();
  }
}
$('playBtn').addEventListener('click', togglePlay);
video.addEventListener('ended', pause);

function sizeOverlay() {
  const r = $('videoPane').getBoundingClientRect(), dpr = Math.min(2, devicePixelRatio || 1);
  overlay.width = Math.max(1, Math.round(r.width * dpr));
  overlay.height = Math.max(1, Math.round(r.height * dpr));
  S.lastDraw = -1;
}

function videoRect() {
  const W = overlay.width, H = overlay.height, vw = S.video?.width || S.capture?.video?.width || 16, vh = S.video?.height || S.capture?.video?.height || 9;
  const s = Math.min(W / vw, H / vh);
  return { x: (W - vw * s) / 2, y: (H - vh * s) / 2, w: vw * s, h: vh * s };
}

function drawPoses(poses, chosen, extra) {
  const g = overlay.getContext('2d'), R = videoRect(), dpr = Math.min(2, devicePixelRatio || 1);
  g.clearRect(0, 0, overlay.width, overlay.height);
  if (!S.video && S.capture) { g.fillStyle = isDark() ? '#0d0f12' : '#1b1e22'; g.fillRect(R.x, R.y, R.w, R.h); }
  const P = (img, i) => [R.x + img[i * IMG_STRIDE] * R.w, R.y + img[i * IMG_STRIDE + 1] * R.h];
  (poses ?? []).forEach((p, k) => {
    const main = chosen === null ? true : k === chosen;
    g.lineWidth = (main ? 3 : 2) * dpr;
    g.strokeStyle = main ? 'rgba(255,196,64,0.95)' : 'rgba(200,200,200,0.55)';
    g.beginPath();
    for (const [a, b] of LM_EDGES) { const pa = P(p.img, a), pb = P(p.img, b); g.moveTo(pa[0], pa[1]); g.lineTo(pb[0], pb[1]); }
    g.stroke();
    if (main) {
      g.fillStyle = 'rgba(255,255,255,0.95)';
      for (let i = 11; i < NLM; i++) { const q = P(p.img, i); g.beginPath(); g.arc(q[0], q[1], 2.6 * dpr, 0, 7); g.fill(); }
    }
  });
  if (extra) {
    g.lineWidth = 2.5 * dpr;
    g.strokeStyle = 'rgba(64,170,255,0.95)';
    g.beginPath();
    for (const [a, b] of extra) { g.moveTo(R.x + a[0] * R.w, R.y + a[1] * R.h); g.lineTo(R.x + b[0] * R.w, R.y + b[1] * R.h); }
    g.stroke();
  }
}

function fitSegments(takeFrame) {
  const res = S.result, cam = res.camera, t = res.take, W = fk(res.skel, getV(t.root, takeFrame), t.rot.subarray(takeFrame * NB * 4, (takeFrame + 1) * NB * 4));
  const qi = qconj(cam.q), toCam = p => qrot(qi, [p[0] / cam.scale, p[1] / cam.scale + cam.offset, p[2] / cam.scale]);
  const pts = [];
  for (let i = 0; i < NB; i++) pts.push(projectPoint(toCam(getV(W.pos, i)), cam));
  const segs = [];
  for (let i = 1; i < NB; i++) if (pts[i] && pts[PARENT[i]]) segs.push([pts[PARENT[i]], pts[i]]);
  return segs;
}

function drawOverlay() {
  if (S.tracking || S.mode === 'play') return;
  const f = S.frame;
  if (f === S.lastDraw) return;
  S.lastDraw = f;
  const fr = S.capture?.frames?.[f];
  const res = S.result, chosen = res?.track?.frames?.get(f) ?? (res ? -1 : null);
  const extra = res && $('fitOn').checked && f >= res.first && f <= res.last ? fitSegments(f - res.first) : null;
  drawPoses(fr?.poses ?? [], chosen, extra);
}

overlay.addEventListener('click', e => {
  if (!S.result || !S.capture || S.result.tracks.length < 2) return;
  const r = overlay.getBoundingClientRect(), dpr = overlay.width / r.width, R = videoRect();
  const x = ((e.clientX - r.left) * dpr - R.x) / R.w, y = ((e.clientY - r.top) * dpr - R.y) / R.h;
  if (x < 0 || x > 1 || y < 0 || y > 1) return;
  S.settings.person = { point: [x, y], frame: S.frame };
  solve(true);
  toast(`Now following person ${S.result.track.id + 1}.`);
});

function drawTimeline() {
  const r = tl.getBoundingClientRect(), dpr = Math.min(2, devicePixelRatio || 1);
  tl.width = Math.max(1, Math.round(r.width * dpr));
  tl.height = Math.max(1, Math.round(r.height * dpr));
  const g = tl.getContext('2d'), W = tl.width, H = tl.height, n = totalFrames();
  g.clearRect(0, 0, W, H);
  g.fillStyle = css('--panel-2');
  g.fillRect(0, H * 0.18, W, H * 0.64);
  if (!n) return;
  const X = f => (f + 0.5) / n * W;
  if (S.capture?.range) {
    const [a, b] = S.capture.range;
    g.fillStyle = css('--line');
    g.fillRect(X(a), H * 0.18, X(b) - X(a), H * 0.64);
  }
  const res = S.result;
  if (res) {
    for (const c of res.clips) {
      const seg = c.kind === 'walk' || c.kind === 'run' || c.kind === 'idle' ? c.src : c.segment;
      if (!seg || c.kind === 'take') continue;
      g.fillStyle = css(KIND_COLOR[c.kind] ?? '--take');
      g.globalAlpha = 0.85;
      g.fillRect(X(res.first + seg.a), H * 0.28, Math.max(2, X(res.first + seg.b) - X(res.first + seg.a)), H * 0.44);
    }
    g.globalAlpha = 1;
    for (const p of res.plants) {
      g.fillStyle = p.side === 'L' ? css('--walk') : css('--run');
      g.fillRect(X(res.first + p.a), p.side === 'L' ? H * 0.04 : H * 0.84, Math.max(1, X(res.first + p.b) - X(res.first + p.a)), H * 0.1);
    }
  }
  g.fillStyle = css('--fg');
  g.fillRect(X(S.frame) - dpr, 0, 2 * dpr, H);
  const t = S.frame / fps(), T = n / fps();
  $('timeText').innerHTML = `${fmtTime(t)} / ${fmtTime(T)} · frame ${S.frame + 1} of ${n}` + (res ? `<span class="hint"> · blue/orange ticks: left/right foot planted</span>` : '');
}
const fmtTime = s => `${Math.floor(s / 60)}:${(s % 60).toFixed(1).padStart(4, '0')}`;

let scrubbing = false;
const scrubTo = e => { const r = tl.getBoundingClientRect(); pause(); S.preview = null; seekFrame((e.clientX - r.left) / r.width * totalFrames()); drawTimeline(); };
tl.addEventListener('pointerdown', e => { scrubbing = true; tl.setPointerCapture(e.pointerId); scrubTo(e); });
tl.addEventListener('pointermove', e => { if (scrubbing) scrubTo(e); });
tl.addEventListener('pointerup', () => { scrubbing = false; });

function applyPose(root, rot) {
  if (!S.character) return;
  S.character.applyPose(root, rot);
  if (S.lines?.object.visible) S.lines.update(root, rot);
}

let last = performance.now(), hudTimer = 0;
function tick(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  const res = S.result;
  if (S.mode === 'play' && res && S.ctl) {
    const input = inputFromKeys(S.down, S.hit);
    S.hit.clear();
    input.camYaw = viewer.cameraYaw();
    const prevSel = S.ctl.selected, out = stepController(S.ctl, input, dt);
    if (S.ctl.selected !== prevSel) { renderLegend(); renderClips(); hudTimer = 1; }
    applyPose(out.root, out.rot);
    if ((hudTimer += dt) > 0.15) {
      hudTimer = 0;
      $('hud').textContent = (out.state === 'action' ? `${out.action}` : out.state === 'jump' ? 'jump' : out.speed < 0.05 ? 'standing' : `${out.speed > S.ctl.walkSpeed * 1.15 ? 'running' : 'walking'} ${out.speed.toFixed(1)} m/s`) + (out.selected ? ` · selected: ${out.selected} (E / click)` : '');
    }
  } else if (res && S.preview) {
    const c = S.preview.clip, dur = c.n / c.fps;
    S.preview.t += dt;
    if (!c.loop && S.preview.t > dur + 0.6) S.preview.t = 0;
    const p = sampleClip(c, c.loop ? S.preview.t : Math.min(S.preview.t, (c.n - 1) / c.fps));
    applyPose(p.root, p.rot);
    $('hud').textContent = `${c.name} preview`;
  } else if (res) {
    if (S.playing) {
      if (S.video && video.src && !video.paused) S.frame = Math.min(totalFrames() - 1, Math.floor(video.currentTime * fps()));
      else if (!S.video) {
        S.clock = (S.clock ?? S.frame) + dt * fps();
        if (S.clock >= res.last) S.clock = res.first;
        S.frame = Math.floor(S.clock);
      }
    }
    const f = S.frame, k = Math.max(0, Math.min(res.n - 1, f - res.first));
    applyPose(getV(res.take.root, k), res.take.rot.subarray(k * NB * 4, (k + 1) * NB * 4));
    if ((hudTimer += dt) > 0.2) { hudTimer = 0; $('hud').textContent = f < res.first || f > res.last ? 'person not tracked here' : ''; }
    if (S.playing) drawTimeline();
  }
  if (S.mode !== 'play') drawOverlay();
  if (!S.tracking) {
    viewer.update(dt);
    viewer.render();
  }
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);

const exportClips = () => { const lib = S.result.library; return [lib.idle, lib.walk, lib.run, lib.jump, ...lib.actions, lib.take].filter((c, i, a) => c && a.indexOf(c) === i); };
$('glbBtn').addEventListener('click', async () => {
  if (!S.result || !S.character) return;
  status('exportStatus', 'Building the GLB…');
  try {
    const clips = exportClips(), blob = await exportGLB(S.character, clips);
    download(blob, baseName() + '.glb');
    status('exportStatus', `Saved ${baseName()}.glb with ${clips.length} animations (${(blob.size / 1e6).toFixed(1)} MB).`, 'ok');
  } catch (e) {
    console.error(e);
    status('exportStatus', 'GLB export failed: ' + e.message, 'bad');
  }
});
$('fbxBtn').addEventListener('click', async () => {
  if (!S.result || !S.character) return;
  status('exportStatus', 'Building the FBX with Blender… this can take a minute.');
  try {
    const clips = exportClips(), blob = await exportGLB(S.character, clips);
    const r = await fetch(`api/glb-to-fbx?name=${encodeURIComponent(baseName())}&fps=${fps()}`, { method: 'POST', body: blob });
    const j = await r.json().catch(() => ({ ok: false, error: 'The Fulcrum server did not answer. Is server.py running?' }));
    if (!j.ok) throw new Error(j.error || 'Conversion failed.');
    const fbx = await (await fetch(j.fbx)).blob();
    download(fbx, baseName() + '.fbx');
    status('exportStatus', `Saved ${baseName()}.fbx with ${clips.length} animations. A copy is in ${j.path}.`, 'ok');
  } catch (e) {
    console.error(e);
    status('exportStatus', 'FBX export failed: ' + e.message, 'bad');
  }
});
$('bvhBtn').addEventListener('click', () => {
  if (!S.result) return;
  download(toBVH(S.result.skel, S.result.library.take), baseName() + '.bvh', 'text/plain');
  status('exportStatus', `Saved ${baseName()}.bvh (Mixamo bone names, centimetres).`, 'ok');
});
$('saveBtn').addEventListener('click', () => {
  if (!S.capture) return;
  download(projectJSON({ video: S.video ?? S.capture.video, settings: settings(), capture: S.capture }), baseName() + '.fulcrum-mocap.json', 'application/json');
  status('exportStatus', 'Project saved. Open it later to skip tracking.', 'ok');
});
const baseName = () => ((S.video?.name || S.capture?.video?.name || 'motion').replace(/\.[^.]+$/, '').replace(/[^\w\-]+/g, '_') || 'motion');

async function openProject(file) {
  try {
    const data = JSON.parse(await file.text());
    if (data.kind !== 'fulcrum-mocap' || !data.capture) throw new Error('This is not a Fulcrum mocap project.');
    loadCaptureOnly(captureFromJSON(data.capture), data.settings);
    toast('Project opened. The original video is not stored in the project, so the overlay shows the tracked points only.', 6000);
  } catch (e) {
    toast(e.message, 6000);
  }
}
$('projectIn').addEventListener('change', e => { const f = e.target.files[0]; e.target.value = ''; if (f) openProject(f); });

function loadCaptureOnly(capture, settingsIn = {}) {
  video.removeAttribute('src');
  video.load();
  $('videoPane').classList.add('points-only');
  S.video = null;
  resetResult();
  S.capture = capture;
  if (settingsIn) {
    if (settingsIn.hfov) { $('fov').value = settingsIn.hfov; $('fovOut').textContent = settingsIn.hfov + '°'; }
    if (settingsIn.fc) { $('smooth').value = settingsIn.fc; $('smoothOut').textContent = settingsIn.fc + ' Hz'; }
    if (settingsIn.imageFit != null) { $('imageFit').value = settingsIn.imageFit; $('fitOut').textContent = (+settingsIn.imageFit).toFixed(2); }
    if (settingsIn.height) $('heightIn').value = Math.round(settingsIn.height * 100);
    if (settingsIn.footLock != null) $('lockOn').checked = settingsIn.footLock;
    S.settings.person = settingsIn.person ?? null;
  }
  $('videoPane').classList.add('has-video');
  $('videoInfo').textContent = `${capture.video.name} (tracked points only)\n${capture.video.frames} frames at ${capture.video.fps} fps`;
  $('fpsIn').value = capture.video.fps;
  sizeOverlay();
  solve(true);
  setStep('moves');
  markDone();
}

$('demoBtn').addEventListener('click', () => {
  const skel = buildSkeleton({ ...DEFAULT_DIMS, thigh: 0.44, shin: 0.43, torso: 0.52 });
  const lib = { idle: idleClip(skel), walk: gaitClip(skel, { kind: 'walk', speed: 1.3 }), run: gaitClip(skel, { kind: 'run', speed: 3.4 }), jump: jumpClip(skel), actions: [], take: null };
  const ctl = createController(lib, { yaw: Math.PI / 2 });
  ctl.pos = [-5, 0, -0.5];
  const steps = [{ seconds: 1.5, input: {} }, { seconds: 2, input: { right: true } }, { seconds: 1.6, input: { right: true, run: true } }, { seconds: 1, input: { right: true, run: true, jump: true } }, { seconds: 1.6, input: {} }];
  const frames = [], fpsDemo = 30;
  for (const s of steps) for (let i = 0; i < Math.round(s.seconds * fpsDemo); i++) frames.push(stepController(ctl, { ...s.input, camYaw: Math.PI, jump: s.input.jump && i === 0 }, 1 / fpsDemo));
  const take = { n: frames.length, fps: fpsDemo, root: new Float64Array(frames.length * 3), rot: new Float64Array(frames.length * NB * 4) };
  frames.forEach((f, i) => { take.root.set(f.root, i * 3); take.rot.set(f.rot, i * NB * 4); });
  const cam = lookAtCamera([0.5, 1.6, 12], [0.5, 0.9, -0.5], { width: 1920, height: 1080, hfov: 72 });
  let seed = 7;
  const rand = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
  const { capture } = synthCapture(skel, take, cam, { rand, noiseImg: 0.0015, noiseWorld: 0.015, occlusion: true, name: 'test walk' });
  capture.range = [0, take.n - 1];
  $('fov').value = 72;
  loadCaptureOnly(capture, { hfov: 72 });
  toast('Loaded a made-up test walk (no real video). Open step 4 to play it.', 5000);
});

window.addEventListener('beforeunload', e => { if (S.tracking) { e.preventDefault(); e.returnValue = ''; } });
fetch('api/mocap/status').then(r => r.json()).then(j => {
  if (!j.vendor?.['tasks-vision'] && !j.vendor?.running) toast('Pose tracker files are downloading in the background…', 4000);
}).catch(() => {});
