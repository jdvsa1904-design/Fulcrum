import { makeCurve, evaluate, TANGENT_TYPES } from '../re/maya/curves.js';
import { layerWeights, rotationAccumulation, blendRotation, eulerToQuat, quatToEuler, quatPow, qmul } from '../re/maya/layers.js';

export const LAYER_DEFAULTS = Object.freeze({ weight: 1, mute: false, override: false, passthrough: true, accumulation: 'layer', tangent: 'auto', rotateOrder: 'xyz' });

export function layerIncludesBone(layer, bone, index, parents = []) {
  const mask = layer.mask;
  if (!mask) return true;
  if (Array.isArray(mask)) return mask.includes(bone) || mask.includes(index);
  if (mask.bones && (mask.bones.includes(bone) || mask.bones.includes(index))) return true;
  if (mask.regions?.length) {
    const key = String(bone).toLowerCase();
    return mask.regions.some(region => {
      const r = String(region).toLowerCase();
      if (r === 'upperbody') return /spine|chest|neck|head|clavicle|arm|shoulder|hand/.test(key);
      if (r === 'lowerbody') return /pelvis|hip|thigh|calf|leg|foot|toe|ball/.test(key);
      if (r === 'arms') return /clavicle|arm|shoulder|elbow|forearm|hand|finger/.test(key);
      if (r === 'legs') return /hip|thigh|knee|calf|leg|foot|toe|ball/.test(key);
      return key.includes(r);
    });
  }
  if (mask.hierarchy && Number.isInteger(index)) {
    const root = typeof mask.hierarchy === 'number' ? mask.hierarchy : mask.hierarchy.root;
    const depth = typeof mask.hierarchy === 'number' ? Infinity : (mask.hierarchy.depth ?? Infinity);
    let i = index, d = 0;
    while (i >= 0 && d <= depth) { if (i === root) return true; i = parents[i] ?? -1; d++; }
    return false;
  }
  return false;
}

export function normalizeLayer(L) {
  const x = { ...LAYER_DEFAULTS, ...L, bones: { ...(L.bones ?? {}) } };
  if (!TANGENT_TYPES.includes(x.tangent)) throw new Error('unknown tangent ' + x.tangent);
  return x;
}

const accumulationOf = L => rotationAccumulation({ override: L.override, rotationAccumulationMode: L.accumulation === 'component' ? 'component' : 1 });

export const LAYER_CHANNELS = ['rotateX', 'rotateY', 'rotateZ', 'translateX', 'translateY', 'translateZ'];

export function channelKey(k, c, L) {
  const ty = k.tangent ?? L.tangent, o = k.tan?.[c];
  const v = c < 3 ? k.e[c] : k.p?.[c - 3] ?? 0;
  return o ? { t: k.f, v, inType: ty, outType: ty, ...o } : { t: k.f, v, inType: ty, outType: ty };
}

const isWeighted = (L, bone, c) => (L.weighted ?? []).includes(bone + '.' + LAYER_CHANNELS[c]);

export function compileLayers(layers, boneIndex, fps = 24, parents = []) {
  const out = new Map();
  layers.forEach((raw, li) => {
    const L = normalizeLayer(raw);
    for (const [bone, keys] of Object.entries(L.bones)) {
      const i = boneIndex(bone);
      if (i === undefined || !keys?.length) continue;
      if (!layerIncludesBone(L, bone, i, parents)) continue;
      const ks = [...keys].sort((a, b) => a.f - b.f);
      const curve = c => makeCurve(ks.map(k => channelKey(k, c, L)), { fps, weighted: isWeighted(L, bone, c) });
      const curves = [0, 1, 2].map(curve);
      const tcurves = ks.some(k => k.p) ? [3, 4, 5].map(curve) : null;
      if (!out.has(i)) out.set(i, []);
      out.get(i).push({ li, curves, tcurves });
    }
  });
  return out;
}

export function evalTranslation(layers, entries, baseP, f) {
  if (!entries?.some(e => e.tcurves)) return baseP;
  let p = [...baseP];
  layers.forEach((raw, li) => {
    const e = entries.find(x => x.li === li);
    if (!e?.tcurves) return;
    const L = normalizeLayer(raw), { fg, bg } = layerWeights(L), v = e.tcurves.map(c => evaluate(c, f));
    p = L.override ? p.map((x, k) => x * bg + v[k] * fg) : p.map((x, k) => x + v[k] * fg);
  });
  return p;
}

function stackValue(layers, entries, f, upto = Infinity) {
  const stack = [];
  layers.forEach((raw, li) => {
    if (li >= upto) return;
    const L = normalizeLayer(raw), e = entries.find(x => x.li === li);
    stack.push({ L, value: e ? e.curves.map(c => evaluate(c, f)) : undefined });
  });
  return stack;
}

export function evalRotation(layers, entries, baseQ, f, upto = Infinity) {
  if (!entries?.length) return baseQ;
  const order = normalizeLayer(layers[0] ?? {}).rotateOrder;
  let e = quatToEuler(baseQ, order);
  for (const { L, value } of stackValue(layers, entries, f, upto)) {
    if (value === undefined) continue;
    const { fg, bg } = layerWeights(L);
    e = blendRotation(e, value, bg, fg, { mode: accumulationOf(L), rotateOrder: L.rotateOrder });
  }
  const q = eulerToQuat(e, order);
  return q[3] * baseQ[3] + q[0] * baseQ[0] + q[1] * baseQ[1] + q[2] * baseQ[2] < 0 ? q.map(v => -v) : q;
}

export function unwrapEuler(e, ref) {
  if (!ref) return e;
  return e.map((v, i) => v + 360 * Math.round((ref[i] - v) / 360));
}

const qinv = q => [-q[0], -q[1], -q[2], q[3]];

export function solveLayerKey(layers, entries, active, baseQ, targetQ, f) {
  const L = normalizeLayer(layers[active]);
  const below = evalRotation(layers, entries ?? [], baseQ, f, active);
  const { fg } = layerWeights({ ...L, mute: false });
  const prev = (entries ?? []).find(x => x.li === active);
  const ref = prev ? prev.curves.map(c => evaluate(c, f)) : null;
  let e;
  if (L.override) e = quatToEuler(targetQ, L.rotateOrder);
  else if (accumulationOf(L) === 'component') {
    const a = quatToEuler(below, L.rotateOrder), t = unwrapEuler(quatToEuler(targetQ, L.rotateOrder), a);
    e = t.map((v, i) => (v - a[i]) / (fg || 1));
  } else e = quatToEuler(quatPow(qmul(qinv(below), targetQ), 1 / (fg || 1)), L.rotateOrder);
  return unwrapEuler(e, ref);
}

export function setLayerKey(layer, bone, f, e) {
  const old = (layer.bones[bone] ?? []).find(k => k.f === f);
  const keys = (layer.bones[bone] ?? []).filter(k => k.f !== f);
  keys.push({ ...old, f, e: e.map(v => +v.toFixed(6)) });
  keys.sort((a, b) => a.f - b.f);
  return { ...layer, bones: { ...layer.bones, [bone]: keys } };
}
