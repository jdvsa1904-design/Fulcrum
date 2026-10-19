const FINGERS = ['thumb', 'index', 'middle', 'ring', 'pinky'];
const LIMB = ['clavicle', 'upperarm', 'lowerarm', 'hand', 'thigh', 'calf', 'foot'];
export const CORE = ['pelvis', 'spine_01', 'neck_01', 'head', ...['l', 'r'].flatMap(s => LIMB.map(b => `${b}_${s}`))];
export const WANTED = [...CORE, 'ball_l', 'ball_r'];

const keyOf = n => String(n).split(':').pop().split('|').pop().toLowerCase();
const PREFIX = /^(\d{3}_|mixamorig\d*[_:]?|j_(?:bip|adj|sec)_(?:c_)?|cc_base_|def[-_]|bip0*1\s*|biped\s*|character\d*_|rig_|armature_)/;
const SIDE_TOKEN = /(^|[\s_.\-])([lr])(?=$|[\s_.\-])/;
const EXCLUDE = /(twist|roll|share|corrective|^ik|pole|nub|end$|metacarpal|bend|adjust|helper|jiggle|breast|eye|jaw|tongue|teeth|hair|skirt|tweak|palm|boneroot)/;
const SINGLE = [
  [/^(hips?|pelvis)$/, 'pelvis', false],
  [/^(head)$/, 'head', false],
  [/^(shoulder|clavicle|collar)$/, 'clavicle', true],
  [/^(upperarm|arm|uparm)$/, 'upperarm', true],
  [/^(forearm|lowerarm|elbow)$/, 'lowerarm', true],
  [/^(hand|wrist)$/, 'hand', true],
  [/^(upleg|upperleg|thigh)$/, 'thigh', true],
  [/^(leg|lowerleg|calf|shin|knee)$/, 'calf', true],
  [/^(foot|ankle)$/, 'foot', true],
  [/^(toebase|toes?|toe0|ball)$/, 'ball', true],
];
// Auto-Rig Pro deform names (root.x, spine_01.x, neck.x, shoulder.l, arm_stretch.l, forearm_stretch.l, thigh_stretch.l,
// leg_stretch.l, toes_01.l, c_index1.l ...) -> names the rules below already know. Only names ending .x/.l/.r are touched.
const ARP = { root: 'pelvis', neck: 'neck_01', shoulder: 'clavicle', arm: 'upperarm', forearm: 'lowerarm', leg: 'calf', toes_01: 'ball', toes: 'ball',
  arm_twist: 'upperarm_twist', forearm_twist: 'lowerarm_twist', leg_twist: 'calf_twist', thigh_twist: 'thigh_twist' };
export function arpAlias(s) {
  const m = s.match(/^(c_)?(.+?)\.(x|l|r)$/);
  if (!m) return s;
  const b = m[2].replace(/_stretch$/, ''), fm = b.match(/^(thumb|index|middle|ring|pinky)(\d)$/), base = fm ? `${fm[1]}_0${fm[2]}` : ARP[b] ?? b;
  return m[3] === 'x' ? base : `${base}_${m[3]}`;
}
const FINGER_RE = /^(?:hand|f)?(thumb|index|pointer|middle|mid|ring|pinky|little)(\d*|proximal|intermediate|distal)$/;
const FINGER_ALIAS = { pointer: 'index', mid: 'middle', little: 'pinky' };
const JOINT = { proximal: 1, intermediate: 2, distal: 3 };

function normBones(bones) {
  return bones.map(b => ({
    name: b.name,
    parent: b.parent ?? b.parentIndex ?? -1,
    pos: b.pos ?? b.position ?? b.rest ?? [0, 0, 0],
  }));
}

function sideOf(s) {
  if (/left/.test(s)) return { side: 'l', rest: s.replace('left', ' ') };
  if (/right/.test(s)) return { side: 'r', rest: s.replace('right', ' ') };
  const m = s.match(SIDE_TOKEN);
  if (m) return { side: m[2], rest: s.replace(SIDE_TOKEN, '$1') };
  return { side: null, rest: s };
}

export function classify(name, rigify = false) {
  const low = keyOf(name).trim();
  if (/^(org|mch)[-_]/.test(low)) return null;
  const s = arpAlias(low.replace(PREFIX, ''));
  if (rigify) {
    const m = s.match(/^spine(?:\.(\d{3}))?$/);
    if (m) {
      const n = m[1] ? +m[1] : 0;
      const t = ['pelvis', 'spine_01', 'spine_02', 'spine_03', 'neck_01', 'neck_02', 'head'][n];
      return t ? { fixed: t } : null;
    }
  }
  const { side, rest } = sideOf(s);
  const c = rest.replace(/[^a-z0-9]/g, '');
  if (!c) return null;
  if (!side && /^neck(twist)?\d*$/.test(c)) return { chain: 'neck' };
  if (EXCLUDE.test(c)) return null;
  if (!side && /^(spine\d*|chest|upperchest|waist|torso|abdomen)$/.test(c)) return { chain: 'spine' };
  if (/(weapon|gun|prop)/.test(c) && side !== 'l') return { fixed: 'weapon_r' };
  for (const [re, base, sided] of SINGLE) {
    if (!re.test(c)) continue;
    if (sided !== !!side) return null;
    return { fixed: sided ? `${base}_${side}` : base };
  }
  if (!side) return null;
  const bf = c.match(/^finger(\d)(\d?)$/);
  if (bf) {
    const f = FINGERS[+bf[1]], j = bf[2] === '' ? 1 : +bf[2] + 1;
    return f && j <= 3 ? { fixed: `${f}_0${j}_${side}` } : null;
  }
  const fm = c.match(FINGER_RE);
  if (fm) {
    const f = FINGER_ALIAS[fm[1]] ?? fm[1];
    const j = fm[2] === '' ? 1 : JOINT[fm[2]] ?? +fm[2];
    return j >= 1 && j <= 3 ? { fixed: `${f}_0${j}_${side}` } : null;
  }
  return null;
}

function topology(B) {
  const kids = B.map(() => []);
  B.forEach((b, i) => { if (b.parent >= 0 && b.parent < B.length) kids[b.parent].push(i); });
  const depth = B.map(() => 0);
  const dOf = i => { let d = 0, p = B[i].parent, g = 0; while (p >= 0 && g++ < B.length) { d++; p = B[p].parent; } return d; };
  B.forEach((_, i) => { depth[i] = dOf(i); });
  const size = B.map(() => 1), height = B.map(() => 1);
  [...B.keys()].sort((a, b) => depth[b] - depth[a]).forEach(i => {
    for (const k of kids[i]) { size[i] += size[k]; height[i] = Math.max(height[i], height[k] + 1); }
  });
  return { kids, depth, size, height };
}

const isAncestor = (B, a, i) => { let p = B[i].parent, g = 0; while (p >= 0 && g++ < B.length) { if (p === a) return true; p = B[p].parent; } return false; };

function byNames(B, T) {
  const rigify = B.some(b => /spine\.\d{3}/.test(keyOf(b.name)));
  const out = new Map(), chains = { spine: [], neck: [], pelvis: [] };
  B.forEach((b, i) => {
    const c = classify(b.name, rigify);
    if (!c) return;
    if (c.chain) chains[c.chain].push(i);
    else if (c.fixed === 'pelvis') chains.pelvis.push(i);
    else if (!out.has(c.fixed)) out.set(c.fixed, i);
  });
  if (chains.pelvis.length) {
    const legs = ['thigh_l', 'thigh_r'].map(k => out.get(k)).filter(i => i !== undefined);
    const anc = chains.pelvis.filter(p => legs.length && legs.every(l => isAncestor(B, p, l)));
    const pool = anc.length ? anc : chains.pelvis;
    const sign = anc.length ? -1 : 1;
    out.set('pelvis', pool.sort((a, b) => sign * (T.depth[a] - T.depth[b]))[0]);
  }
  for (const ch of ['spine', 'neck']) {
    chains[ch].sort((a, b) => T.depth[a] - T.depth[b]).forEach((i, k) => {
      const key = `${ch}_${String(k + 1).padStart(2, '0')}`;
      if (!out.has(key)) out.set(key, i);
    });
  }
  return out;
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

function centroid(B, T, i) {
  const acc = [0, 0, 0]; let n = 0;
  const walk = j => { acc[0] += B[j].pos[0]; acc[1] += B[j].pos[1]; acc[2] += B[j].pos[2]; n++; T.kids[j].forEach(walk); };
  walk(i);
  return acc.map(v => v / n);
}

function longestPath(T, i) {
  const path = [i];
  let cur = i;
  while (T.kids[cur].length) {
    cur = T.kids[cur].reduce((a, b) => (T.height[b] > T.height[a] ? b : a));
    path.push(cur);
  }
  return path;
}

function byGeometry(B, T) {
  const out = new Map();
  const big = i => T.kids[i].filter(k => T.size[k] >= 3);
  const order = [...B.keys()].sort((a, b) => T.depth[a] - T.depth[b]);
  const pelvis = order.find(i => big(i).length >= 3);
  if (pelvis === undefined) return out;
  out.set('pelvis', pelvis);
  const branches = big(pelvis).sort((a, b) => T.size[b] - T.size[a]);
  const spineRoot = branches[0], P = B[pelvis].pos;
  const sideOfPos = p => (p[0] - P[0] >= 0 ? 'l' : 'r');
  const up = sub(centroid(B, T, spineRoot), P);
  const fwd = cross([1, 0, 0], up);
  for (const leg of branches.slice(1, 3)) {
    const s = sideOfPos(centroid(B, T, leg));
    longestPath(T, leg).slice(0, 4).forEach((i, k) => out.set(`${['thigh', 'calf', 'foot', 'ball'][k]}_${s}`, i));
  }
  const spine = [];
  let cur = spineRoot, chest;
  for (let g = 0; g < B.length && cur !== undefined; g++) {
    spine.push(cur);
    if (T.kids[cur].filter(k => T.size[k] >= 2).length >= 3) { chest = cur; break; }
    const next = T.kids[cur];
    cur = next.length ? next.reduce((a, b) => (T.size[b] > T.size[a] ? b : a)) : undefined;
  }
  spine.forEach((i, k) => out.set(`spine_${String(k + 1).padStart(2, '0')}`, i));
  if (chest === undefined) return out;
  const tops = T.kids[chest].filter(k => T.size[k] >= 2);
  const C = B[chest].pos;
  const neck = tops.reduce((a, b) => (dot(sub(centroid(B, T, b), C), up) > dot(sub(centroid(B, T, a), C), up) ? b : a));
  const np = longestPath(T, neck);
  if (np.length >= 2) { out.set('neck_01', np[0]); out.set('head', np[1]); } else out.set('head', np[0]);
  const arms = tops.filter(k => k !== neck).sort((a, b) => T.size[b] - T.size[a]).slice(0, 2);
  for (const arm of arms) {
    const s = sideOfPos(centroid(B, T, arm));
    const path = longestPath(T, arm).slice(0, 4);
    path.forEach((i, k) => out.set(`${['clavicle', 'upperarm', 'lowerarm', 'hand'][k]}_${s}`, i));
    const hand = path[3];
    if (hand === undefined || T.kids[hand].length !== 5) continue;
    const H = B[hand].pos;
    T.kids[hand].slice().sort((a, b) => dot(sub(B[b].pos, H), fwd) - dot(sub(B[a].pos, H), fwd))
      .forEach((f, fi) => longestPath(T, f).slice(0, 3).forEach((i, j) => out.set(`${FINGERS[fi]}_0${j + 1}_${s}`, i)));
  }
  return out;
}

export function buildRigMap(bones) {
  const B = normBones(bones), T = topology(B);
  const named = byNames(B, T);
  const used = new Set(named.values());
  const merged = new Map(named), fromGeo = new Set();
  if (CORE.some(k => !named.has(k))) {
    for (const [k, i] of byGeometry(B, T)) {
      if (merged.has(k) || used.has(i)) continue;
      merged.set(k, i); used.add(i); fromGeo.add(k);
    }
  }
  const toCanonical = new Map(), fromCanonical = new Map();
  for (const [k, i] of merged) { toCanonical.set(B[i].name, k); fromCanonical.set(k, B[i].name); }
  const score = CORE.reduce((a, k) => a + (!merged.has(k) ? 0 : fromGeo.has(k) ? 0.6 : 1), 0);
  return {
    toCanonical,
    fromCanonical,
    missing: WANTED.filter(k => !merged.has(k)),
    confidence: Math.round((score / CORE.length) * 100) / 100,
  };
}

export function renameForExport(name, map) {
  return map.fromCanonical.get(keyOf(name)) ?? name;
}

export function renameForImport(name, map) {
  return map.toCanonical.get(name) ?? name;
}
