import { evaluate, computeTangents } from '../solvers/maya/curves.js';

export const CHANNELS = ['rotateX', 'rotateY', 'rotateZ', 'translateX', 'translateY', 'translateZ'];
export const EDITOR_TANGENTS = ['auto', 'spline', 'clamped', 'plateau', 'flat', 'linear', 'step'];
const D2R = Math.PI / 180;
const CH_COLOR = { X: '#ff6b5b', Y: '#5fd07c', Z: '#6fa3ff' };
const RULER = 18;

export const channelColor = ch => CH_COLOR[ch.slice(-1)] ?? '#e4ebe6';
export const curveId = c => c.bone + '.' + c.channel;

export function sortCurves(curves) {
  return [...curves].sort((a, b) => a.bone === b.bone ? CHANNELS.indexOf(a.channel) - CHANNELS.indexOf(b.channel) : a.bone < b.bone ? -1 : 1);
}

export function normalizeKey(k) {
  return { inType: 'auto', outType: 'auto', inAngle: 0, outAngle: 0, inWeight: 1, outWeight: 1, ...k };
}

export function prepareCurve(curve, fps = 24) {
  return computeTangents({ weighted: !!curve.weighted, preInfinity: curve.preInfinity ?? 'constant', postInfinity: curve.postInfinity ?? 'constant', keys: curve.keys.map(normalizeKey) }, { fps });
}

export function evalCurve(curve, t, fps = 24) {
  return curve.keys.length ? evaluate(prepareCurve(curve, fps), t) : 0;
}

export function makeView(x0, x1, y0, y1, w, h) {
  return { x0, x1, y0, y1, w, h };
}

export function toScreen(v, t, val) {
  return [(t - v.x0) / (v.x1 - v.x0) * v.w, v.h - (val - v.y0) / (v.y1 - v.y0) * v.h];
}

export function toWorld(v, px, py) {
  return [v.x0 + px / v.w * (v.x1 - v.x0), v.y0 + (v.h - py) / v.h * (v.y1 - v.y0)];
}

export function panView(v, dx, dy) {
  const sx = (v.x1 - v.x0) / v.w, sy = (v.y1 - v.y0) / v.h;
  return { ...v, x0: v.x0 - dx * sx, x1: v.x1 - dx * sx, y0: v.y0 + dy * sy, y1: v.y1 + dy * sy };
}

export function zoomView(v, px, py, fx, fy = fx) {
  const [t, val] = toWorld(v, px, py);
  return { ...v, x0: t + (v.x0 - t) * fx, x1: t + (v.x1 - t) * fx, y0: val + (v.y0 - val) * fy, y1: val + (v.y1 - val) * fy };
}

export function resizeView(v, w, h) {
  return { ...v, w, h };
}

export function frameAll(curves, w, h, { frameCount = 0, fps = 24, pad = 0.08, top = 0 } = {}) {
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const c of curves) {
    if (!c.keys.length) continue;
    const p = prepareCurve(c, fps), a = c.keys[0].t, b = c.keys[c.keys.length - 1].t;
    x0 = Math.min(x0, a); x1 = Math.max(x1, b);
    const n = Math.max(2, Math.min(400, Math.ceil(b - a) + 1));
    for (let i = 0; i <= n; i++) { const val = evaluate(p, a + (b - a) * i / n); y0 = Math.min(y0, val); y1 = Math.max(y1, val); }
  }
  if (x0 === Infinity) { x0 = 0; x1 = Math.max(1, frameCount - 1); y0 = -1; y1 = 1; }
  if (x1 - x0 < 1e-6) { x0 -= 5; x1 += 5; }
  if (y1 - y0 < 1) { const m = (y0 + y1) / 2; y0 = m - 1; y1 = m + 1; }
  const px = (x1 - x0) * pad, py = (y1 - y0) * pad;
  y0 -= py; y1 += py;
  if (top > 0 && h > top) y1 += (y1 - y0) * top / (h - top);
  return makeView(x0 - px, x1 + px, y0, y1, w, h);
}

export const selKey = (ci, ki) => ci + ':' + ki;
export const parseSel = s => s.split(':').map(Number);

export function hitKey(curves, view, px, py, r = 6, visible = null) {
  let best = null, bd = r * r;
  curves.forEach((c, ci) => {
    if (visible && !visible.has(curveId(c))) return;
    c.keys.forEach((k, ki) => {
      const [x, y] = toScreen(view, k.t, k.v), d = (x - px) ** 2 + (y - py) ** 2;
      if (d <= bd) { bd = d; best = { ci, ki }; }
    });
  });
  return best;
}

export function handleScreen(prepared, ki, side, view, len = 40) {
  const k = prepared.keys[ki], a = (side === 'in' ? k.inAngle : k.outAngle) * D2R;
  const [kx, ky] = toScreen(view, k.t, k.v), sgn = side === 'in' ? -1 : 1;
  const [ex, ey] = toScreen(view, k.t + sgn * Math.cos(a), k.v + sgn * Math.sin(a));
  const dx = ex - kx, dy = ey - ky, m = Math.hypot(dx, dy) || 1;
  return [kx + dx / m * len, ky + dy / m * len];
}

export function hitHandle(curves, sel, view, px, py, { fps = 24, r = 6, len = 40 } = {}) {
  const prep = new Map();
  for (const s of sel) {
    const [ci, ki] = parseSel(s), c = curves[ci];
    if (!c?.keys[ki]) continue;
    if (!prep.has(ci)) prep.set(ci, prepareCurve(c, fps));
    for (const side of ['in', 'out']) {
      const [x, y] = handleScreen(prep.get(ci), ki, side, view, len);
      if ((x - px) ** 2 + (y - py) ** 2 <= r * r) return { ci, ki, side };
    }
  }
  return null;
}

export function keysInBox(curves, view, ax, ay, bx, by, visible = null) {
  const x0 = Math.min(ax, bx), x1 = Math.max(ax, bx), y0 = Math.min(ay, by), y1 = Math.max(ay, by), out = new Set();
  curves.forEach((c, ci) => {
    if (visible && !visible.has(curveId(c))) return;
    c.keys.forEach((k, ki) => {
      const [x, y] = toScreen(view, k.t, k.v);
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1) out.add(selKey(ci, ki));
    });
  });
  return out;
}

const cloneCurves = curves => curves.map(c => ({ ...c, keys: c.keys.map(k => ({ ...k })) }));

export function insertKey(curves, ci, t, { fps = 24, v, type = 'auto' } = {}) {
  const out = cloneCurves(curves), c = out[ci];
  const val = v ?? evalCurve(curves[ci], t, fps);
  const i = c.keys.findIndex(k => k.t === t);
  if (i >= 0) { c.keys[i].v = val; return { curves: out, ki: i }; }
  c.keys.push(normalizeKey({ t, v: val, inType: type, outType: type }));
  c.keys.sort((a, b) => a.t - b.t);
  return { curves: out, ki: c.keys.findIndex(k => k.t === t) };
}

export function deleteKeys(curves, sel) {
  const drop = new Set(sel);
  return curves.map((c, ci) => ({ ...c, keys: c.keys.filter((_, ki) => !drop.has(selKey(ci, ki))).map(k => ({ ...k })) }));
}

export function moveKeys(curves, sel, dt, dv, { snap = true } = {}) {
  const out = cloneCurves(curves), moved = new Set(), next = new Set();
  const step = snap ? Math.round(dt) : dt;
  for (const s of sel) {
    const [ci, ki] = parseSel(s), k = out[ci]?.keys[ki];
    if (!k) continue;
    k.t += step; k.v += dv; moved.add(k);
  }
  out.forEach((c, ci) => {
    const movedTimes = new Set(c.keys.filter(k => moved.has(k)).map(k => k.t));
    c.keys = c.keys.filter(k => moved.has(k) || !movedTimes.has(k.t)).sort((a, b) => a.t - b.t);
    c.keys.forEach((k, ki) => { if (moved.has(k)) next.add(selKey(ci, ki)); });
  });
  return { curves: out, sel: next };
}

export function setTangentType(curves, sel, type, side = 'both') {
  const out = cloneCurves(curves);
  for (const s of sel) {
    const [ci, ki] = parseSel(s), k = out[ci]?.keys[ki];
    if (!k) continue;
    if (side !== 'out') k.inType = type;
    if (side !== 'in') k.outType = type;
  }
  return out;
}

export function setHandle(curves, ci, ki, side, dt, dv, { fps = 24, unified = true } = {}) {
  const pk = prepareCurve(curves[ci], fps).keys[ki];
  const out = cloneCurves(curves), k = out[ci].keys[ki];
  const sx = side === 'in' ? -1 : 1, x = Math.max(1e-6, sx * dt), y = sx * dv;
  const ang = Math.atan2(y, x) / D2R, w = Math.hypot(x, y) / 3;
  k.inType = k.outType = 'fixed';
  k.inAngle = pk.inAngle; k.outAngle = pk.outAngle; k.inWeight = pk.inWeight; k.outWeight = pk.outWeight;
  if (side === 'in' || unified) k.inAngle = ang;
  if (side === 'out' || unified) k.outAngle = ang;
  if (out[ci].weighted) { if (side === 'in') k.inWeight = w; else k.outWeight = w; }
  return out;
}

export function createHistory(limit = 200) {
  const undo = [], redo = [];
  return {
    push(state) { undo.push(state); if (undo.length > limit) undo.shift(); redo.length = 0; },
    undo(current) { if (!undo.length) return null; redo.push(current); return undo.pop(); },
    redo(current) { if (!redo.length) return null; undo.push(current); return redo.pop(); },
    get canUndo() { return undo.length > 0; },
    get canRedo() { return redo.length > 0; },
  };
}

const TAN_FIELDS = ['inType', 'outType', 'inAngle', 'outAngle', 'inWeight', 'outWeight'];

export function layerToCurves(layer, bones = null) {
  const out = [], ty = k => k.tangent ?? layer.tangent ?? 'auto', wset = new Set(layer.weighted ?? []);
  for (const [bone, keys] of Object.entries(layer.bones ?? {})) {
    if (bones && !bones.includes(bone)) continue;
    const ks = [...keys].sort((a, b) => a.f - b.f);
    const nch = ks.some(k => k.p) ? 6 : 3;
    for (let c = 0; c < nch; c++) {
      const v = k => c < 3 ? k.e[c] : k.p?.[c - 3] ?? 0;
      const curve = { bone, channel: CHANNELS[c], keys: ks.map(k => normalizeKey({ t: k.f, v: v(k), inType: ty(k), outType: ty(k), ...(k.tan?.[c] ?? {}) })) };
      if (wset.has(bone + '.' + CHANNELS[c])) curve.weighted = true;
      out.push(curve);
    }
  }
  return sortCurves(out);
}

export function curvesToLayer(layer, curves, fps = 24) {
  const bones = { ...(layer.bones ?? {}) }, byBone = new Map(), base = layer.tangent ?? 'auto';
  const weighted = new Set((layer.weighted ?? []).filter(id => !curves.some(c => curveId(c) === id)));
  for (const c of curves) {
    const ci = CHANNELS.indexOf(c.channel);
    if (ci < 0) continue;
    if (!byBone.has(c.bone)) byBone.set(c.bone, [null, null, null, null, null, null]);
    byBone.get(c.bone)[ci] = c;
    if (c.weighted) weighted.add(curveId(c));
  }
  for (const [bone, six] of byBone) {
    const times = [...new Set(six.flatMap(c => c ? c.keys.map(k => k.t) : []))].sort((a, b) => a - b);
    if (!times.length) { delete bones[bone]; continue; }
    const prev = new Map((bones[bone] ?? []).map(k => [k.f, k]));
    const hasT = six.slice(3).some(c => c && c.keys.length);
    bones[bone] = times.map(t => {
      const at = six.map(c => c?.keys.find(k => k.t === t));
      const e = [0, 1, 2].map(i => six[i] ? +evalCurve(six[i], t, fps).toFixed(6) : prev.get(t)?.e?.[i] ?? 0);
      const ty = at.slice(0, 3).find(Boolean)?.outType ?? at.find(Boolean)?.outType;
      const keyTy = ty && ty !== 'fixed' ? ty : base;
      const k = { f: t, e };
      if (keyTy !== base) k.tangent = keyTy;
      if (hasT) k.p = [3, 4, 5].map(i => six[i]?.keys.length ? +evalCurve(six[i], t, fps).toFixed(6) : prev.get(t)?.p?.[i - 3] ?? 0);
      const tan = {};
      at.forEach((kk, c) => {
        if (!kk) return;
        if (kk.inType !== 'fixed' && kk.outType !== 'fixed' && kk.inType === keyTy && kk.outType === keyTy) return;
        tan[c] = Object.fromEntries(TAN_FIELDS.map(f => [f, kk[f]]));
      });
      if (Object.keys(tan).length) k.tan = tan;
      return k;
    });
  }
  const out = { ...layer, bones };
  if (weighted.size) out.weighted = [...weighted]; else delete out.weighted;
  return out;
}

function el(tag, cls, parent, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text != null) e.textContent = text;
  if (parent) parent.appendChild(e);
  return e;
}

const val = x => (typeof x === 'function' ? x() : x);

export function createGraphEditor(container, opts) {
  const { getCurves, setCurves, onChange, onScrub } = opts;
  const fps = () => val(opts.fps) ?? 24;
  const frameCount = () => val(opts.frameCount) ?? 0;
  let frame = val(opts.currentFrame) ?? 0;
  let curves = sortCurves(getCurves() ?? []), sel = new Set(), visible = new Set(curves.map(curveId)), active = curves.length ? 0 : -1;
  let drag = null;
  const hist = createHistory();

  const root = el('div', 'ge-root', container);
  const bar = el('div', 'ge-bar', root);
  const body = el('div', 'ge-body', root);
  const list = el('div', 'ge-list', body);
  const wrap = el('div', 'ge-canvas-wrap', body);
  const canvas = el('canvas', 'ge-canvas', wrap);
  canvas.tabIndex = 0;
  const ctx = canvas.getContext('2d');
  const btn = (label, title, fn) => { const b = el('button', 'ge-btn', bar, label); b.title = title; b.onclick = fn; return b; };
  btn('Frame', 'Frame all (F)', () => doFrameAll());
  btn('+Key', 'Key at current frame (S)', () => addKeyAtFrame());
  btn('Del', 'Delete selected keys (Delete)', () => deleteSelected());
  const tsel = el('select', 'ge-select', bar);
  tsel.title = 'Tangent type';
  tsel.innerHTML = '<option value="">tangent</option>' + EDITOR_TANGENTS.map(t => `<option value="${t}">${t}</option>`).join('');
  tsel.onchange = () => { if (tsel.value) setTangents(tsel.value); tsel.value = ''; };
  const bUndo = btn('Undo', 'Undo (Ctrl+Z)', () => undo());
  const bRedo = btn('Redo', 'Redo (Ctrl+Y)', () => redo());
  const info = el('span', 'ge-info', bar);

  let view = makeView(0, Math.max(1, frameCount() - 1), -1, 1, 1, 1);

  function publish() { setCurves?.(curves); onChange?.(curves); renderList(); draw(); }
  function commit(next, nextSel = sel) {
    hist.push({ curves, sel: [...sel] });
    curves = next; sel = new Set(nextSel);
    publish();
  }
  function restore(st) {
    if (!st) return;
    curves = st.curves; sel = new Set(st.sel);
    publish();
  }
  const undo = () => restore(hist.undo({ curves, sel: [...sel] }));
  const redo = () => restore(hist.redo({ curves, sel: [...sel] }));

  function targets() {
    const cs = new Set([...sel].map(s => parseSel(s)[0]));
    if (!cs.size && active >= 0) cs.add(active);
    return [...cs].filter(ci => curves[ci] && visible.has(curveId(curves[ci])));
  }
  function addKeyAtFrame() {
    let next = curves;
    const ns = new Set();
    for (const ci of targets()) { const r = insertKey(next, ci, Math.round(frame), { fps: fps() }); next = r.curves; ns.add(selKey(ci, r.ki)); }
    if (ns.size) commit(next, ns);
  }
  function deleteSelected() { if (sel.size) commit(deleteKeys(curves, sel), []); }
  function setTangents(type) { if (sel.size) commit(setTangentType(curves, sel, type)); }
  function doFrameAll() {
    const vis = curves.filter(c => visible.has(curveId(c)));
    view = frameAll(vis, view.w, view.h, { frameCount: frameCount(), fps: fps(), top: RULER });
    draw();
  }

  function renderList() {
    list.innerHTML = '';
    let lastBone = null;
    curves.forEach((c, ci) => {
      if (c.bone !== lastBone) { el('div', 'ge-bone', list, c.bone); lastBone = c.bone; }
      const row = el('label', 'ge-chan' + (ci === active ? ' on' : ''), list);
      const cb = el('input', null, row);
      cb.type = 'checkbox'; cb.checked = visible.has(curveId(c));
      cb.onchange = () => { cb.checked ? visible.add(curveId(c)) : visible.delete(curveId(c)); draw(); };
      el('span', 'ge-swatch', row).style.background = channelColor(c.channel);
      el('span', 'ge-name', row, c.channel + ' · ' + c.keys.length);
      row.onclick = e => { if (e.target === cb) return; e.preventDefault(); active = ci; renderList(); draw(); };
    });
    bUndo.disabled = !hist.canUndo; bRedo.disabled = !hist.canRedo;
  }

  function niceStep(span, px, minPx) {
    const raw = span * minPx / Math.max(1, px), p = 10 ** Math.floor(Math.log10(raw));
    return [1, 2, 5, 10].map(m => m * p).find(s => s >= raw);
  }
  const css = n => getComputedStyle(root).getPropertyValue(n).trim();

  function draw() {
    const dpr = window.devicePixelRatio || 1, W = wrap.clientWidth, H = wrap.clientHeight;
    if (!W || !H) return;
    if (canvas.width !== W * dpr || canvas.height !== H * dpr) { canvas.width = W * dpr; canvas.height = H * dpr; }
    if (view.w !== W || view.h !== H) view = resizeView(view, W, H);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = css('--ge-bg'); ctx.fillRect(0, 0, W, H);
    const fc = frameCount();
    if (fc > 0) {
      const [a] = toScreen(view, 0, 0), [b] = toScreen(view, fc - 1, 0);
      ctx.fillStyle = css('--ge-range'); ctx.fillRect(a, 0, b - a, H);
    }
    ctx.font = '10px ui-monospace, monospace'; ctx.lineWidth = 1;
    const xs = Math.max(1, niceStep(view.x1 - view.x0, W, 60)), ys = niceStep(view.y1 - view.y0, H, 40);
    ctx.strokeStyle = css('--ge-grid'); ctx.fillStyle = css('--ge-muted');
    ctx.beginPath();
    for (let t = Math.ceil(view.x0 / xs) * xs; t <= view.x1; t += xs) { const [x] = toScreen(view, t, 0); ctx.moveTo(Math.round(x) + 0.5, RULER); ctx.lineTo(Math.round(x) + 0.5, H); }
    for (let v = Math.ceil(view.y0 / ys) * ys; v <= view.y1; v += ys) { const [, y] = toScreen(view, 0, v); ctx.moveTo(0, Math.round(y) + 0.5); ctx.lineTo(W, Math.round(y) + 0.5); }
    ctx.stroke();
    for (let v = Math.ceil(view.y0 / ys) * ys; v <= view.y1; v += ys) { const [, y] = toScreen(view, 0, v); if (y > RULER + 8) ctx.fillText(String(+v.toFixed(4)), 4, y - 2); }
    curves.forEach((c, ci) => {
      if (!visible.has(curveId(c)) || !c.keys.length) return;
      const p = prepareCurve(c, fps());
      ctx.strokeStyle = channelColor(c.channel); ctx.globalAlpha = ci === active ? 1 : 0.7; ctx.lineWidth = ci === active ? 2 : 1.25;
      ctx.beginPath();
      for (let x = 0; x <= W; x += 2) { const [t] = toWorld(view, x, 0), [, y] = toScreen(view, t, evaluate(p, t)); x ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }
      ctx.stroke(); ctx.globalAlpha = 1; ctx.lineWidth = 1;
      c.keys.forEach((k, ki) => {
        const [x, y] = toScreen(view, k.t, k.v), on = sel.has(selKey(ci, ki));
        if (on) {
          ctx.strokeStyle = css('--ge-handle'); ctx.fillStyle = css('--ge-handle');
          for (const side of ['in', 'out']) {
            const [hx, hy] = handleScreen(p, ki, side, view);
            ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(hx, hy); ctx.stroke();
            ctx.beginPath(); ctx.arc(hx, hy, 3, 0, Math.PI * 2); ctx.fill();
          }
        }
        ctx.fillStyle = on ? css('--ge-sel') : css('--ge-key');
        ctx.fillRect(x - 3, y - 3, 6, 6);
      });
    });
    ctx.fillStyle = css('--ge-ruler'); ctx.fillRect(0, 0, W, RULER);
    ctx.fillStyle = css('--ge-muted');
    for (let t = Math.ceil(view.x0 / xs) * xs; t <= view.x1; t += xs) { const [x] = toScreen(view, t, 0); ctx.fillText(String(t), x + 2, 12); }
    const [fx] = toScreen(view, frame, 0);
    ctx.strokeStyle = css('--ge-frame'); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(fx, 0); ctx.lineTo(fx, H); ctx.stroke();
    ctx.fillStyle = css('--ge-frame'); ctx.fillText(String(Math.round(frame)), fx + 3, 12);
    if (drag?.mode === 'box') {
      ctx.strokeStyle = css('--ge-sel'); ctx.setLineDash([4, 3]);
      ctx.strokeRect(drag.x0, drag.y0, drag.x - drag.x0, drag.y - drag.y0); ctx.setLineDash([]);
    }
    if (sel.size === 1) {
      const [ci, ki] = parseSel([...sel][0]), k = curves[ci]?.keys[ki];
      info.textContent = k ? `${curves[ci].bone}.${curves[ci].channel}  f ${+k.t.toFixed(3)}  v ${+k.v.toFixed(4)}  ${k.inType}/${k.outType}` : '';
    } else info.textContent = sel.size ? sel.size + ' keys' : '';
  }

  const pos = e => { const r = canvas.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
  function scrubTo(px) {
    const fc = frameCount();
    frame = Math.round(toWorld(view, px, 0)[0]);
    if (fc > 0) frame = Math.max(0, Math.min(fc - 1, frame));
    onScrub?.(frame); draw();
  }
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  canvas.addEventListener('pointerdown', e => {
    canvas.focus(); canvas.setPointerCapture(e.pointerId);
    const [x, y] = pos(e);
    if (e.button === 1 || e.button === 2 || (e.button === 0 && e.altKey)) { drag = { mode: 'pan', x, y }; return; }
    if (y < RULER) { drag = { mode: 'scrub' }; scrubTo(x); return; }
    const h = hitHandle(curves, sel, view, x, y, { fps: fps() });
    if (h) { drag = { mode: 'handle', ...h, base: curves, baseSel: new Set(sel), moved: false }; return; }
    const k = hitKey(curves, view, x, y, 6, visible);
    if (k) {
      const id = selKey(k.ci, k.ki);
      if (e.shiftKey || e.ctrlKey) { const n = new Set(sel); n.has(id) ? n.delete(id) : n.add(id); sel = n; }
      else if (!sel.has(id)) sel = new Set([id]);
      active = k.ci; renderList();
      drag = { mode: 'move', x, y, base: curves, baseSel: new Set(sel), moved: false };
      draw(); return;
    }
    drag = { mode: 'box', x0: x, y0: y, x, y, add: e.shiftKey || e.ctrlKey };
  });
  canvas.addEventListener('pointermove', e => {
    if (!drag) return;
    const [x, y] = pos(e);
    if (drag.mode === 'pan') { view = panView(view, x - drag.x, y - drag.y); drag.x = x; drag.y = y; draw(); return; }
    if (drag.mode === 'scrub') { scrubTo(x); return; }
    if (drag.mode === 'box') { drag.x = x; drag.y = y; draw(); return; }
    if (drag.mode === 'move') {
      const [t0, v0] = toWorld(view, drag.x, drag.y), [t1, v1] = toWorld(view, x, y);
      const horiz = Math.abs(x - drag.x) > Math.abs(y - drag.y);
      const r = moveKeys(drag.base, drag.baseSel, e.shiftKey && !horiz ? 0 : t1 - t0, e.shiftKey && horiz ? 0 : v1 - v0);
      curves = r.curves; sel = r.sel; drag.moved = true; draw(); return;
    }
    if (drag.mode === 'handle') {
      const k = drag.base[drag.ci].keys[drag.ki], [t, v] = toWorld(view, x, y);
      curves = setHandle(drag.base, drag.ci, drag.ki, drag.side, t - k.t, v - k.v, { fps: fps(), unified: !e.shiftKey });
      drag.moved = true; draw();
    }
  });
  canvas.addEventListener('pointerup', () => {
    if (!drag) return;
    const d = drag;
    drag = null;
    if ((d.mode === 'move' || d.mode === 'handle') && d.moved) {
      const next = curves, ns = sel;
      curves = d.base; sel = d.baseSel;
      commit(next, ns); return;
    }
    if (d.mode === 'box') {
      const moved = Math.abs(d.x - d.x0) > 3 || Math.abs(d.y - d.y0) > 3;
      const hits = moved ? keysInBox(curves, view, d.x0, d.y0, d.x, d.y, visible) : new Set();
      sel = d.add ? new Set([...sel, ...hits]) : hits;
    }
    draw();
  });
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const [x, y] = pos(e), f = Math.exp(e.deltaY * 0.0015);
    view = zoomView(view, x, y, e.shiftKey ? 1 : f, e.ctrlKey ? 1 : f);
    draw();
  }, { passive: false });
  canvas.addEventListener('keydown', e => {
    const k = e.key.toLowerCase(), mod = e.ctrlKey || e.metaKey;
    if (mod && k === 'z' && !e.shiftKey) undo();
    else if (mod && (k === 'y' || k === 'z')) redo();
    else if (k === 'delete' || k === 'backspace') deleteSelected();
    else if (k === 'f' && !mod) doFrameAll();
    else if (k === 's' && !mod) addKeyAtFrame();
    else return;
    e.preventDefault(); e.stopPropagation();
  });

  const ro = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(() => draw()) : null;
  ro?.observe(wrap);
  renderList();
  requestAnimationFrame(() => { view = resizeView(view, wrap.clientWidth || 1, wrap.clientHeight || 1); doFrameAll(); });

  return {
    refresh() {
      const prevVis = visible, prevIds = new Set(curves.map(curveId));
      curves = sortCurves(getCurves() ?? []);
      visible = new Set(curves.map(curveId).filter(id => prevVis.has(id) || !prevIds.has(id)));
      sel = new Set([...sel].filter(s => { const [ci, ki] = parseSel(s); return curves[ci]?.keys[ki]; }));
      if (active >= curves.length) active = curves.length - 1;
      renderList(); draw();
    },
    setFrame(f) { frame = f; draw(); },
    frameAll: doFrameAll,
    addKey: addKeyAtFrame,
    deleteKeys: deleteSelected,
    setTangent: setTangents,
    undo, redo,
    getSelection: () => [...sel].map(parseSel).map(([ci, ki]) => ({ bone: curves[ci].bone, channel: curves[ci].channel, t: curves[ci].keys[ki].t })),
    setVisible(ids) { visible = new Set(ids); renderList(); draw(); },
    getView: () => ({ ...view }),
    destroy() { ro?.disconnect(); root.remove(); },
  };
}
