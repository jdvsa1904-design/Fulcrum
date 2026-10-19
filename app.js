import * as THREE from 'three';
import { FBXLoader } from 'three/addons/loaders/FBXLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { detectFlights, fitBallistic, ballisticOffsets, flightReport } from './ballistic.js';
import { fulcrumOffsets, combineOffsets, detectFulcrums, pivotSlip } from './fulcrum.js';
import { twoBoneIK as solveTwoBone, stablePole } from './ik.js';
import { quatContinuity, readTrack, writeTrack } from './filters.js';
import { kneeFloor } from './contacts.js';
import { floorTargets, soleMarkerRig, soleMarkers } from './wire/floor.js';
import { rollCurveFromPlant, rollPose } from './wire/roll.js';
import { detectFootPlants } from './wire/plants.js';
import { seamPlan, seamRootOffsets } from './wire/seams.js';
import { pullOffsets } from './wire/bodypull.js';
import { applyCleanup, reduceKeysReport } from './wire/cleanup.js';
import { hikRetargetSetup, hikRetargetFrame, dstWorldRotations } from './wire/hikretarget.js';
import { planFootsteps, balanceShift } from './wire/footsteps.js';
import { compileLayers, evalRotation, evalTranslation, solveLayerKey, setLayerKey, normalizeLayer } from './wire/animlayers.js';
import { buildRigMap, CORE as RIG_CORE } from './wire/rigmap.js';
import { autoFixLoop, fulcrumFindings } from './wire/autofix.js';
import { createGraphEditor, layerToCurves, curvesToLayer } from './ui/grapheditor.js';
import { applyStoryBlends, storyWindow, cutIndex, insertIndex } from './wire/story.js';
import { contactGizmoScale, gizmoFootMarkers } from './re/hik/contact.js';
import { applyAdvancedMotion, assistedPose, validatePose } from './advanced.js';
import { suggestPoses } from './poseassist.js';
import { parsePoseModel, predictNextPose } from './wire/posemodel.js';
import { inspectMotion } from './motionquality.js';
import { buildControlRig, sampleControlRig, controlMotionToLocalQuaternions } from './wire/cascadeur_rig.js';
import { retimeMapFrame, retimeTracks } from './wire/timeline.js';
import { buildAnimatorControls, ikGoalFromOffset, modelGroundY } from './wire/animator_rig.js';
import { guidanceForFrames, preserveGuidedPoints } from './wire/pose_guidance.js';
import { planHumanoidRig, skinWeightsForPoint } from './wire/autorig.js';

const $ = id => document.getElementById(id);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const keyOf = n => n.split(':').pop().split('|').pop().toLowerCase();
const LOCAL = location.protocol.startsWith('http') && /^(localhost|127\.0\.0\.1)$/.test(location.hostname);
const THEME_KEY = 'fulcrum.appearance';
let savedTheme = 'system';
try { savedTheme = localStorage.getItem(THEME_KEY) || 'system'; } catch {}
if (!['system', 'light', 'dark'].includes(savedTheme)) savedTheme = 'system';
document.documentElement.dataset.theme = savedTheme;
$('themePicker').value = savedTheme;
$('themePicker').addEventListener('change', e => {
  document.documentElement.dataset.theme = e.target.value;
  try { localStorage.setItem(THEME_KEY, e.target.value); } catch {}
  themeColors();
  drawTL();
});

const WORKSPACE_NAMES = {scene:'Scene',animate:'Animate',rig:'Rig',physics:'Physics',review:'Review',export:'Export'};
function setWorkspace(name) {
  if (!WORKSPACE_NAMES[name]) return;
  for (const button of document.querySelectorAll('[data-workspace]')) {
    const active = button.dataset.workspace === name;
    button.classList.toggle('on', active);
    if (active) button.setAttribute('aria-current', 'page');
    else button.removeAttribute('aria-current');
  }
  for (const section of document.querySelectorAll('#panel section[data-workspaces]'))
    section.hidden = !section.dataset.workspaces.split(' ').includes(name);
  $('workspaceTitle').textContent = WORKSPACE_NAMES[name];
  $('panel').scrollTop = 0;
}
document.querySelectorAll('[data-workspace]').forEach(button =>
  button.addEventListener('click', () => setWorkspace(button.dataset.workspace)));
const ACTION_WORKSPACE = {loadFRT:'scene',rigKeyFk:'rig',assistPose:'animate',lockL:'rig',lockR:'rig',pinHand:'rig'};
document.querySelectorAll('.tool-shelf [data-action]').forEach(button =>
  button.addEventListener('click', () => {
    if (ACTION_WORKSPACE[button.dataset.action]) setWorkspace(ACTION_WORKSPACE[button.dataset.action]);
    document.getElementById(button.dataset.action)?.click();
  }));
setWorkspace('animate');

const S = {
  body: null, take: null, sourceShots: [], base: null, out: null,
  layer: new Map(), animLayers: [], activeLayer: -1, locks: [], pins: [], fixes: [], props: [], bookmarks: [], retimes: [], srcTake: null, srcIdx: null,
  frame: 0, playing: false, sel: null, selBone: -1, animatorControls: [], controlId: null,
  flags: [], gate: null, com: null, comRaw: null, contact: null, upm: 100, ground: 0,
};
const EDITS = { rows:[], cursor:-1, restoring:false, preview:null, identity:null };
let initialRecovery=null;try{initialRecovery=JSON.parse(localStorage.getItem('fulcrum.recovery.v2')||'null');}catch{}
const propGroup=new THREE.Group();

const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(devicePixelRatio);
view.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.add(propGroup);
const camera = new THREE.PerspectiveCamera(40, 1, 1, 100000);
camera.position.set(350, 220, 420);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0, 90, 0);
scene.add(new THREE.HemisphereLight(0xffffff, 0x445048, 2.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(300, 600, 200);
scene.add(sun);
let grid = new THREE.GridHelper(2000, 40);
scene.add(grid);
const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.setMode('rotate');
gizmo.setSpace('local');
gizmo.setSize(0.7);
scene.add(gizmo.getHelper ? gizmo.getHelper() : gizmo);
gizmo.addEventListener('dragging-changed', e => {
  orbit.enabled = !e.value;
  if (!e.value) commitGizmo();
});
gizmo.addEventListener('mouseDown', () => { if(S.take&&!EDITS.preview)EDITS.gizmoBefore=editSnapshot(); });

const overlay = new THREE.Group();
scene.add(overlay);
const jointGeo = new THREE.SphereGeometry(1, 10, 8);
let jointMeshes = [];
const comMarker = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), new THREE.MeshBasicMaterial({ depthTest: false, transparent: true }));
comMarker.renderOrder = 10;
const comDrop = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineBasicMaterial({ depthTest: false, transparent: true }));
let comPath = null;
overlay.add(comMarker, comDrop);
const lockMarks = new THREE.Group(), pinMarks = new THREE.Group();
overlay.add(lockMarks, pinMarks);

function themeColors() {
  comMarker.material.color.set(css('--com'));
  comDrop.material.color.set(css('--com'));
  if (comPath) comPath.material.color.set(css('--com'));
  const bg = css('--panel');
  scene.background = new THREE.Color(bg);
  scene.remove(grid);
  grid = new THREE.GridHelper(4 * S.upm * 5, 40, css('--line'), css('--line'));
  grid.position.y = S.ground;
  scene.add(grid);
  for (const m of jointMeshes) m.material.color.set(css('--accent'));
}
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { themeColors(); drawTL(); });

function resize() {
  const r = view.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  renderer.domElement.style.width = r.width + 'px';
  renderer.domElement.style.height = r.height + 'px';
  camera.aspect = r.width / Math.max(1, r.height);
  camera.updateProjectionMatrix();
  sizeTL();
}
new ResizeObserver(() => requestAnimationFrame(resize)).observe(view);

function status(t) { $('status').textContent = t; }

const loader = new FBXLoader();
async function fetchSample(name) {
  const r = await fetch('/samples/' + encodeURIComponent(name));
  if (!r.ok) throw new Error('Could not read ' + name);
  return r.arrayBuffer();
}
async function sourceWithDigest(bytes, source) {
  const hash = await crypto.subtle.digest('SHA-256', bytes);
  return { ...source, sha256: Array.from(new Uint8Array(hash), b => b.toString(16).padStart(2, '0')).join('') };
}
async function fetchProjectSource(source) {
  if (!source || !['samples', 'exports'].includes(source.location) || typeof source.name !== 'string' || !source.name || source.name !== source.name.split(/[\\/]/).at(-1)) throw new Error('Invalid project source reference.');
  const response = await fetch('/' + source.location + '/' + encodeURIComponent(source.name));
  if (!response.ok) throw new Error('Project source missing: ' + source.location + '/' + source.name);
  const bytes = await response.arrayBuffer();
  if (source.sha256 && (await sourceWithDigest(bytes, source)).sha256 !== source.sha256) throw new Error('Project source changed since import: ' + source.name);
  return bytes;
}
function parseFBX(buf) { return loader.parse(buf, ''); }

function bonesOf(obj) { const b = []; obj.traverse(o => { if (o.isBone) b.push(o); }); return b; }

function autoRigObject(obj) {
  const existing = bonesOf(obj);
  if (existing.length) {
    const skinned = !!obj.getObjectByProperty('isSkinnedMesh', true);
    if (!skinned && obj.getObjectByProperty('isMesh', true)) throw new Error('This FBX has bones and an unskinned mesh. Bind the mesh to its skeleton first, or import a mesh without bones for a starter auto-rig.');
    return { kind: 'recognized', bones: existing.length, skinned };
  }
  obj.updateMatrixWorld(true);
  const meshes = [];
  obj.traverse(o => { if (o.isMesh && !o.isSkinnedMesh && o.geometry?.getAttribute('position')?.count) meshes.push(o); });
  if (!meshes.length) throw new Error('Auto-rig needs a mesh or an existing skeleton.');
  const localFromWorld = obj.matrixWorld.clone().invert(), prepared = [], bounds = new THREE.Box3();
  for (const mesh of meshes) {
    const geometry = mesh.geometry.clone();
    geometry.applyMatrix4(localFromWorld.clone().multiply(mesh.matrixWorld));
    geometry.computeBoundingBox();
    if (geometry.boundingBox.isEmpty()) continue;
    bounds.union(geometry.boundingBox);
    prepared.push({ mesh, geometry });
  }
  if (!prepared.length || bounds.isEmpty()) throw new Error('Auto-rig found no finite mesh geometry.');
  const plan = planHumanoidRig({ min: bounds.min.toArray(), max: bounds.max.toArray() });
  const bones = plan.bones.map(spec => { const bone = new THREE.Bone(); bone.name = spec.name; return bone; });
  plan.bones.forEach((spec, i) => {
    const parent = spec.parent;
    bones[i].position.fromArray(spec.pos);
    if (parent >= 0) { bones[i].position.sub(new THREE.Vector3().fromArray(plan.bones[parent].pos)); bones[parent].add(bones[i]); }
  });
  obj.add(bones[0]);
  obj.updateMatrixWorld(true);
  const skeleton = new THREE.Skeleton(bones);
  for (const { mesh, geometry } of prepared) {
    const positions = geometry.getAttribute('position'), indices = new Uint16Array(positions.count * 4), weights = new Float32Array(positions.count * 4);
    for (let i = 0; i < positions.count; i++) {
      const fitted = skinWeightsForPoint([positions.getX(i), positions.getY(i), positions.getZ(i)], plan);
      indices.set(fitted.indices, i * 4); weights.set(fitted.weights, i * 4);
    }
    geometry.setAttribute('skinIndex', new THREE.Uint16BufferAttribute(indices, 4));
    geometry.setAttribute('skinWeight', new THREE.Float32BufferAttribute(weights, 4));
    const skinned = new THREE.SkinnedMesh(geometry, mesh.material);
    skinned.name = mesh.name || 'AutoRigMesh';
    obj.add(skinned);
    skinned.updateMatrixWorld(true);
    skinned.bind(skeleton);
    mesh.parent.remove(mesh);
  }
  obj.updateMatrixWorld(true);
  return { kind: 'generated', bones: bones.length, meshes: prepared.length, method: 'upright Y-up humanoid template + proximity skinning' };
}

function setBody(obj, name, bytes, source = { location: 'samples', name }, autorig = null) {
  if (S.body) scene.remove(S.body.obj);
  obj.updateMatrixWorld(true);
  const bones = bonesOf(obj);
  const byKey = new Map(bones.map((b, i) => [keyOf(b.name), i]));
  let mesh = null;
  let rigBones = null;
  obj.traverse(o => { if (o.isSkinnedMesh && !mesh) mesh = o; });
  obj.traverse(o => {
    if (o.isMesh && !o.isSkinnedMesh) o.visible = false;
    if (o.isMesh) {
      o.material = new THREE.MeshStandardMaterial({ color: 0x8d968f, roughness: 0.8, transparent: true, opacity: 0.55, depthWrite: false });
      o.frustumCulled = false;
    }
  });
  const restQ = bones.map(b => b.quaternion.clone()), restP = bones.map(b => b.position.clone());
  const restW = bones.map(b => b.matrixWorld.clone());
  if (mesh) {
    mesh.skeleton.bones.forEach((b, i) => {
      const k = bones.indexOf(b);
      if (k >= 0) restW[k] = mesh.skeleton.boneInverses[i].clone().invert();
    });
  }
  const rig = buildRigMap(bones.map((b, i) => ({ name: b.name, parent: bones.indexOf(b.parent), pos: new THREE.Vector3().setFromMatrixPosition(restW[i]).toArray() })));
  const aliased = {};
  if (RIG_CORE.some(k => !byKey.has(k))) for (const [canon, nm] of rig.fromCanonical) {
    if (byKey.has(canon)) continue;
    const i = bones.findIndex(b => b.name === nm);
    if (i >= 0) { byKey.set(canon, i); aliased[canon] = nm; }
  }
  rigBones = { confidence: rig.confidence, missing: rig.missing, aliased, map: Object.fromEntries(rig.fromCanonical) };
  const helper = mesh ? null : new THREE.SkeletonHelper(obj);
  if (helper) obj.add(helper);
  S.body = { obj, bones, byKey, mesh, restQ, restP, restW, name, bytes, source, rig: rigBones, autorig, parentIdx: bones.map(b => bones.indexOf(b.parent)) };
  S.animatorControls = buildAnimatorControls(bones, byKey, S.body.parentIdx); S.controlId = null; S.selBone = -1; gizmo.detach(); renderAnimatorControls();
  scene.add(obj);
  const pts = restW.map(m => new THREE.Vector3().setFromMatrixPosition(m));
  const ys = pts.map(p => p.y);
  const H = Math.max(...ys) - Math.min(...ys);
  S.upm = H / 1.78;
  S.ground = Math.min(...ys);
  jointMeshes.forEach(m => overlay.remove(m));
  jointMeshes = bones.map((b, i) => {
    const m = new THREE.Mesh(jointGeo, new THREE.MeshBasicMaterial({ depthTest: false, transparent: true, opacity: 0.85 }));
    const k = keyOf(b.name);
    const small = /(thumb|index|middle|ring|pinky|metacarpal|twist|corrective|ik_|_ik|weapon|attach|socket)/.test(k);
    m.scale.setScalar(S.upm * (small ? 0.006 : 0.014));
    m.visible = !small;
    m.renderOrder = 11;
    m.userData.bone = i;
    m.position.setFromMatrixPosition(restW[i]);
    overlay.add(m);
    return m;
  });
  comMarker.scale.setScalar(S.upm * 0.03);
  orbit.target.set(pts[0].x, S.ground + S.upm * 0.95, pts[0].z);
  camera.near = S.upm * 0.05; camera.far = S.upm * 400;
  camera.position.set(pts[0].x + S.upm * 3.2, S.ground + S.upm * 1.6, pts[0].z + S.upm * 3.8);
  camera.updateProjectionMatrix();
  themeColors();
  S.take = null; S.sourceShots = []; S.base = null; S.out = null; S.flags = []; S.gate = null; S.motionQuality = null; S.autoFixReport = null; S.com = null; S.contact = null; S.frame = 0; S.sel = null; S.playing = false;
  S.cuts = []; S.layer.clear(); S.animLayers = []; S.activeLayer = -1; S.srcTake = null; S.srcIdx = null; S.locks = []; S.pins = []; S.fixes = [];S.props=[];S.bookmarks=[];S.retimes=[];S.poseSuggestions=null;EDITS.rows=[];EDITS.cursor=-1;EDITS.preview=null;updateProps();
  if (comPath) { overlay.remove(comPath); comPath = null; }
  comMarker.visible = false; comDrop.visible = false;
  $('hud').textContent = autorig?.kind === 'generated' ? 'Starter auto-rig · inspect fit and weights' : 'No take loaded';
  $('frameLbl').textContent = 'No take loaded'; $('selLbl').textContent = ''; $('flagList').replaceChildren(); $('gate').replaceChildren(); drawTL();
  renderShots();
}

function detectFps(clip) {
  let best = Infinity;
  for (const t of clip.tracks.slice(0, 20)) for (let i = 1; i < t.times.length; i++) { const d = t.times[i] - t.times[i - 1]; if (d > 1e-5 && d < best) best = d; }
  const f = 1 / best;
  for (const c of [24, 25, 30, 48, 50, 60, 120]) if (Math.abs(f - c) < 0.6) return c;
  return Math.round(f);
}

const NAME_SLOTS = ['pelvis', 'thigh_l', 'calf_l', 'foot_l', 'thigh_r', 'calf_r', 'foot_r', 'spine_01', 'upperarm_l', 'lowerarm_l', 'hand_l', 'upperarm_r', 'lowerarm_r', 'hand_r', 'head'];
function nameMatchShare(obj) {
  const keys = new Set(bonesOf(obj).map(b => keyOf(b.name)));
  const want = NAME_SLOTS.filter(k => S.body.byKey.has(k));
  return want.length ? want.filter(k => keys.has(k)).length / want.length : 0;
}
function dstRig() {
  return S.body.restW.map((m, i) => { const p = new THREE.Vector3(), q = new THREE.Quaternion(), s = new THREE.Vector3(); m.decompose(p, q, s); return { name: S.body.bones[i].name, parent: S.body.parentIdx[i], p: p.toArray(), q: q.toArray() }; });
}
function sampleTakeHik(obj, name, clip, fps, n, o = {}) {
  const src = bonesOf(obj), nb = S.body.bones.length;
  obj.updateMatrixWorld(true);
  const bind = new Map();
  obj.traverse(m => { if (m.isSkinnedMesh) m.skeleton.bones.forEach((bn, k) => { if (!bind.has(bn)) bind.set(bn, m.skeleton.boneInverses[k].clone().invert()); }); });
  const restOf = b => { const M = bind.get(b) || b.matrixWorld, pp = new THREE.Vector3(), qq = new THREE.Quaternion(), ss = new THREE.Vector3(); M.decompose(pp, qq, ss); return { p: pp.toArray(), q: qq.toArray() }; };
  const srcBones = src.map(b => ({ name: b.name, parent: src.indexOf(b.parent), ...restOf(b) }));
  const dst = dstRig();
  const setup = hikRetargetSetup(srcBones, dst, { reach: o.reach });
  if (!setup.valid) throw new Error(`${name}: HIK characterization is missing ${setup.missing.join(', ')}`);
  const q = new Float32Array(n * nb * 4), p = new Float32Array(n * nb * 3);
  const mixer = new THREE.AnimationMixer(obj), action = mixer.clipAction(clip);
  action.setLoop(THREE.LoopOnce); action.clampWhenFinished = true; action.play();
  const order = [], seen = new Uint8Array(nb);
  const visit = i => { if (seen[i]) return; const pi = S.body.parentIdx[i]; if (pi >= 0) visit(pi); seen[i] = 1; order.push(i); };
  for (let i = 0; i < nb; i++) visit(i);
  const restScale = S.body.restW.map(m => { const a = new THREE.Vector3(), b = new THREE.Quaternion(), c = new THREE.Vector3(); m.decompose(a, b, c); return c; });
  const worldM = Array.from({ length: nb }, () => new THREE.Matrix4()), lp = new THREE.Vector3(), lr = new THREE.Quaternion(), ls = new THREE.Vector3(), wq0 = new THREE.Quaternion(), wp0 = new THREE.Vector3();
  const hips = setup.dst.slots.Hips;
  for (let f = 0; f < n; f++) {
    mixer.setTime(Math.min(f / fps, clip.duration - 1e-6));
    obj.updateMatrixWorld(true);
    const g = src.map(b => ({ p: b.getWorldPosition(wp0).toArray(), q: b.getWorldQuaternion(wq0).toArray() }));
    const res = hikRetargetFrame(setup, g), W = dstWorldRotations(setup, res, dst);
    for (const i of order) {
      const pi = S.body.parentIdx[i];
      const pos = i === hips && W.hipsP ? new THREE.Vector3().fromArray(W.hipsP) : null;
      const parentM = pi >= 0 ? worldM[pi] : null;
      if (pos) worldM[i].compose(pos, new THREE.Quaternion().fromArray(W.q[i]), restScale[i]);
      else {
        const local = new THREE.Matrix4().compose(S.body.restP[i], S.body.restQ[i], S.body.bones[i].scale);
        const base = parentM ? parentM.clone().multiply(local) : S.body.restW[i].clone();
        const bp = new THREE.Vector3().setFromMatrixPosition(base);
        worldM[i].compose(bp, new THREE.Quaternion().fromArray(W.q[i]), restScale[i]);
      }
      const rel = parentM ? parentM.clone().invert().multiply(worldM[i]) : S.body.restW[i].clone().multiply(new THREE.Matrix4().compose(S.body.restP[i], S.body.restQ[i], S.body.bones[i].scale).invert()).invert().multiply(worldM[i]);
      rel.decompose(lp, lr, ls);
      lr.normalize().toArray(q, (f * nb + i) * 4);
      (pos ? lp : S.body.restP[i]).toArray(p, (f * nb + i) * 3);
    }
  }
  const mapped = Object.keys(setup.src.slots).length;
  return { fps, n, q, p, matched: mapped, total: src.length, unit: setup.dst.metrics.legLeft / Math.max(1e-9, setup.src.metrics.legLeft), retarget: 'hik', slots: Object.fromEntries(Object.entries(setup.src.slots).map(([s, i]) => [s, src[i].name])) };
}

function sampleTake(obj, name, mode = 'auto', clip = obj.animations && obj.animations[0]) {
  if (!clip) throw new Error(name + ' has no animation');
  if (mode === 'hik' || (mode === 'auto' && nameMatchShare(obj) < 0.8)) {
    const fps0 = detectFps(clip);
    return sampleTakeHik(obj, name, clip, fps0, Math.round(clip.duration * fps0) + 1);
  }
  const fps = detectFps(clip);
  const n = Math.round(clip.duration * fps) + 1;
  const src = bonesOf(obj);
  const map = src.map(b => S.body.byKey.get(keyOf(b.name)));
  const nb = S.body.bones.length;
  const q = new Float32Array(n * nb * 4), p = new Float32Array(n * nb * 3);
  obj.updateMatrixWorld(true);
  const srcRestW = src.map(b => b.getWorldQuaternion(new THREE.Quaternion()));
  const dec = m => { const pp = new THREE.Vector3(), qq = new THREE.Quaternion(), ss = new THREE.Vector3(); m.decompose(pp, qq, ss); return qq; };
  const dstRestW = S.body.restW.map(dec);
  const dstParentRestW = S.body.bones.map((b, i) => dstRestW[i].clone().multiply(S.body.restQ[i].clone().invert()));
  const fitP = [], fitQ = [];
  src.forEach((b, j) => { const i = map[j]; if (i !== undefined) { fitP.push(new THREE.Vector3().setFromMatrixPosition(b.matrixWorld)); fitQ.push(new THREE.Vector3().setFromMatrixPosition(S.body.restW[i])); } });
  const G = fitP.length >= 3 ? dec(similarity(fitP, fitQ)) : new THREE.Quaternion(), Gi = G.clone().invert();
  const order = [], seen = new Uint8Array(nb);
  const visit = i => { if (seen[i]) return; const pi = S.body.parentIdx[i]; if (pi >= 0) visit(pi); seen[i] = 1; order.push(i); };
  for (let i = 0; i < nb; i++) visit(i);
  const mixer = new THREE.AnimationMixer(obj);
  const action = mixer.clipAction(clip);
  action.setLoop(THREE.LoopOnce);
  action.clampWhenFinished = true;
  action.play();
  const moveKeys = new Set(['root', 'pelvis']);
  const dstW = Array.from({ length: nb }, () => new THREE.Quaternion()), has = new Uint8Array(nb), wq0 = new THREE.Quaternion(), lq = new THREE.Quaternion();
  let matched = 0;
  let num = 0, den = 0;
  src.forEach((b, j) => {
    const i = map[j];
    if (i === undefined || moveKeys.has(keyOf(b.name))) return;
    num += S.body.restP[i].length(); den += b.position.length();
  });
  const unit = den > 0 ? num / den : 1;
  const fitM = fitP.length >= 3 ? similarity(fitP, fitQ) : new THREE.Matrix4();
  const restScale = S.body.restW.map(m => { const a = new THREE.Vector3(), b = new THREE.Quaternion(), c = new THREE.Vector3(); m.decompose(a, b, c); return c; });
  const parentRestM = S.body.bones.map((b, i) => S.body.restW[i].clone().multiply(new THREE.Matrix4().compose(S.body.restP[i], S.body.restQ[i], b.scale.clone()).invert()));
  const movers = new Map(), moverM = new Map(), wp0 = new THREE.Vector3(), lp = new THREE.Vector3(), ls = new THREE.Vector3(), lr = new THREE.Quaternion();
  src.forEach((b, j) => { const i = map[j]; if (i !== undefined && moveKeys.has(keyOf(b.name))) movers.set(i, b); });
  for (let f = 0; f < n; f++) {
    mixer.setTime(Math.min(f / fps, clip.duration - 1e-6));
    obj.updateMatrixWorld(true);
    has.fill(0);
    for (let i = 0; i < nb; i++) S.body.restP[i].toArray(p, (f * nb + i) * 3);
    src.forEach((b, j) => {
      const i = map[j];
      if (i === undefined) return;
      if (f === 0) matched++;
      b.getWorldQuaternion(wq0);
      dstW[i].copy(G).multiply(wq0).multiply(srcRestW[j].clone().invert()).multiply(Gi).multiply(dstRestW[i]);
      has[i] = 1;
    });
    for (const i of order) {
      const pi = S.body.parentIdx[i], pw = pi >= 0 ? dstW[pi] : dstParentRestW[i];
      if (has[i]) lq.copy(pw).invert().multiply(dstW[i]);
      else { lq.copy(S.body.restQ[i]); dstW[i].copy(pw).multiply(lq); }
      lq.normalize().toArray(q, (f * nb + i) * 4);
    }
    moverM.clear();
    for (const i of order) {
      const b = movers.get(i);
      if (!b) continue;
      const pi = S.body.parentIdx[i], parentM = pi >= 0 && moverM.has(pi) ? moverM.get(pi) : parentRestM[i];
      b.getWorldPosition(wp0).applyMatrix4(fitM);
      const worldM = new THREE.Matrix4().compose(wp0, dstW[i], restScale[i]);
      new THREE.Matrix4().copy(parentM).invert().multiply(worldM).decompose(lp, lr, ls);
      lp.toArray(p, (f * nb + i) * 3);
      moverM.set(i, worldM);
    }
  }
  return { fps, n, q, p, matched, total: src.length, unit };
}

function sampleAllClips(obj, name, mode = 'auto') {
  const clips = obj.animations || [];
  if (!clips.length) throw new Error(name + ' has no animation clips');
  const parts = clips.map((clip, i) => sampleTake(obj, name + (clips.length > 1 ? ' · ' + clip.name : ''), mode, clip));
  if (parts.length === 1) return { ...parts[0], shots: [{ name, clipIndex: 0, clipName: clips[0].name }] };
  const fps = Math.max(...parts.map(x => x.fps));
  const normalized = parts.map(x => x.fps === fps ? x : resampleTake(x, fps));
  const nb = S.body.bones.length, n = normalized.reduce((s, x) => s + x.n, 0), q = new Float32Array(n * nb * 4), p = new Float32Array(n * nb * 3), shots = [];
  let offset = 0;
  for (let i = 0; i < normalized.length; i++) {
    const t = normalized[i];
    q.set(t.q, offset * nb * 4); p.set(t.p, offset * nb * 3);
    shots.push({ name: name + ' · ' + clips[i].name, clipIndex: i, clipName: clips[i].name, start: offset, n: t.n, matched: t.matched, total: t.total, ...(t.retarget ? { retarget: t.retarget, slots: t.slots } : {}) });
    offset += t.n;
  }
  return { fps, n, q, p, shots, matched: parts.reduce((m, x) => Math.min(m, x.matched || 0), Infinity), total: parts.reduce((s, x) => s + (x.total || 0), 0) };
}

function appendTake(t, name) {
  const nb = S.body.bones.length;
  if (!S.take) { S.take = { fps: t.fps, n: 0, q: new Float32Array(0), p: new Float32Array(0), shots: [] }; S.sourceShots = []; }
  if (S.take.n && t.fps !== S.take.fps) t = resampleTake(t, S.take.fps);
  const q = new Float32Array((S.take.n + t.n) * nb * 4), p = new Float32Array((S.take.n + t.n) * nb * 3);
  q.set(S.take.q); q.set(t.q, S.take.n * nb * 4);
  p.set(S.take.p); p.set(t.p, S.take.n * nb * 3);
  const shot = t.shots?.[0], retarget = shot?.retarget || t.retarget;
  S.take.shots.push({ name, start: S.take.n, n: t.n, matched: t.matched, total: t.total, source: shot?.source, clipIndex: shot?.clipIndex ?? 0, clipName: shot?.clipName, ...(retarget ? { retarget, slots: shot?.slots || t.slots } : {}) });
  S.sourceShots.push({ name, source: shot?.source || { location: 'samples', name }, clipIndex: shot?.clipIndex ?? 0, clipName: shot?.clipName, mode: retarget || 'names' });
  S.take.q = q; S.take.p = p; S.take.n += t.n;
  S.srcTake = null; S.srcIdx = null;
}

function resampleTake(t, fps) {
  const frames = Math.round((t.n - 1) * fps / t.fps) + 1, nb = S.body.bones.length;
  const q = new Float32Array(frames * nb * 4), p = new Float32Array(frames * nb * 3);
  for (let f = 0; f < frames; f++) {
    const at = f * t.fps / fps, lo = Math.min(t.n - 1, Math.floor(at)), hi = Math.min(t.n - 1, lo + 1), w = at - lo;
    for (let i = 0; i < nb; i++) {
      const o = (f * nb + i) * 4, a = Array.from(t.q.subarray((lo * nb + i) * 4, (lo * nb + i + 1) * 4)), b = Array.from(t.q.subarray((hi * nb + i) * 4, (hi * nb + i + 1) * 4));
      new THREE.Quaternion(...a).slerp(new THREE.Quaternion(...b), w).toArray(q, o);
      for (let k = 0; k < 3; k++) p[(f * nb + i) * 3 + k] = t.p[(lo * nb + i) * 3 + k] * (1 - w) + t.p[(hi * nb + i) * 3 + k] * w;
    }
  }
  return { ...t, q, p, n: frames, fps };
}

async function loadBodyBytes(buf, name, source = { location: 'samples', name }, autoRig = false) {
  const obj = parseFBX(buf);
  const result = autoRig ? autoRigObject(obj) : null;
  setBody(obj, name, buf, await sourceWithDigest(buf, source), result);
  status(`Character ${name}: ${S.body.bones.length} bones${S.body.mesh ? ', skinned mesh' : ', skeleton only'}${result?.kind === 'generated' ? ' · starter auto-rig generated' : result ? ' · existing rig recognized' : ''}.`);
}
async function appendBytes(buf, name, mode = 'auto', source = { location: 'samples', name }, clipIndex = null, targetFps = null) {
  const obj = parseFBX(buf);
  source = await sourceWithDigest(buf, source);
  if (!S.body) setBody(obj, name, buf, source);
  const sampled = clipIndex == null ? sampleAllClips(obj, name, mode) : (() => {
    const clip = obj.animations?.[clipIndex];
    if (!clip) throw new Error(`Clip ${clipIndex} is missing from ${name}`);
    const one = sampleTake(obj, name, mode, clip);
    return { ...one, shots: [{ name, clipIndex, clipName: clip.name }] };
  })();
  const t = targetFps && sampled.fps !== targetFps ? resampleTake(sampled, targetFps) : sampled;
  for (const shot of t.shots) shot.source = source;
  if (t.shots?.length > 1) for (const shot of t.shots) appendTake({ ...t, n: shot.n, q: t.q.slice(shot.start * S.body.bones.length * 4, (shot.start + shot.n) * S.body.bones.length * 4), p: t.p.slice(shot.start * S.body.bones.length * 3, (shot.start + shot.n) * S.body.bones.length * 3), shots: [shot] }, shot.name);
  else appendTake(t, name);
  renderShots();
  rebuild();
  status(`Take: ${S.take.shots.length} shot(s), ${S.take.n} frames at ${S.take.fps} fps (${(S.take.n / S.take.fps).toFixed(1)} s).`);
}

function renderShots() {
  const ul = $('shotList'); ul.innerHTML = '';
  if (S.body) ul.insertAdjacentHTML('beforeend', `<li><span class="grow">character · ${S.body.name}</span><span>${S.body.bones.length} bones</span></li>`);
  const R = S.body?.rig;
  if (R) ul.insertAdjacentHTML('beforeend', `<li><span class="grow">rig map · confidence ${R.confidence.toFixed(2)}${Object.keys(R.aliased).length ? ` · ${Object.keys(R.aliased).length} canonical names aliased` : ' · native names'}</span><span>${R.missing.length ? 'missing ' + R.missing.join(', ') : 'no missing slots'}</span></li>`);
  if (S.body?.autorig) ul.insertAdjacentHTML('beforeend', `<li><span class="grow">auto-rig · ${S.body.autorig.kind === 'generated' ? 'starter skeleton + proximity skin' : 'existing skeleton characterized'}</span><span>${S.body.autorig.bones} bones</span></li>`);
  for (const s of S.take?.shots || []) ul.insertAdjacentHTML('beforeend', `<li><span class="grow">${s.name}</span><span>f${s.start}–${s.start + s.n - 1} · ${s.matched}/${s.total}</span></li>`);
}

const qa = new THREE.Quaternion(), qb = new THREE.Quaternion(), qc = new THREE.Quaternion(), qI = new THREE.Quaternion();
function layerDelta(i, f, out) {
  const keys = S.layer.get(i);
  out.identity();
  if (!keys || !keys.length) return out;
  const fo = Math.max(1, +$('falloff').value || 12);
  if (f <= keys[0].f) {
    const w = 1 - (keys[0].f - f) / fo;
    if (w > 0) out.slerp(qa.fromArray(keys[0].q), smooth(w));
    return out;
  }
  const L = keys[keys.length - 1];
  if (f >= L.f) {
    const w = 1 - (f - L.f) / fo;
    if (w > 0) out.slerp(qa.fromArray(L.q), smooth(w));
    return out;
  }
  let k = 0; while (keys[k + 1].f <= f) k++;
  const A = keys[k], B = keys[k + 1];
  return out.slerpQuaternions(qa.fromArray(A.q), qb.fromArray(B.q), smooth((f - A.f) / (B.f - A.f)));
}
const smooth = t => t * t * (3 - 2 * t);
const ramp = (f, a, b, e = 4) => Math.min(1, a <= 0 ? 1 : (f - a + 1) / e, b >= (S.take?.n ?? Infinity) - 1 ? 1 : (b - f + 1) / e);

function idx(k) { return S.body.byKey.get(k); }
function firstIdx(...ks) { for (const k of ks) { const i = idx(k); if (i !== undefined) return i; } return undefined; }

const CLEAN_TYPES = ['despike', 'filter', 'median'];
function buildBase() {
  const nb = S.body.bones.length, n = S.take.n;
  const ck = JSON.stringify(S.fixes.filter(x => CLEAN_TYPES.includes(x.type))) + '|' + n + '|' + nb;
  if (!S.cleanCache || S.cleanCache.key !== ck || S.cleanCache.take !== S.take) {
    const cl = applyCleanup({ q: S.take.q, p: S.take.p, nb, n, fps: S.take.fps, fixes: S.fixes.map(({ hits, ...x }) => x), names: S.body.bones.map(b => b.name) });
    S.cleanCache = { key: ck, take: S.take, q: cl.q, p: cl.p, report: cl.report };
  }
  S.cleanReport = S.cleanCache.report;
  const q = new Float32Array(S.cleanCache.q), p = new Float32Array(S.cleanCache.p);
  const advanced = applyAdvancedMotion({ q, p, nb, n, fps:S.take.fps, roots:S.body.parentIdx.map((p,i)=>p<0?i:-1).filter(i=>i>=0), names: S.body.bones.map(b => b.name), fixes: S.guidedReferenceBuild?[]:S.fixes });
  S.advancedReport = advanced.report;
  const stories = S.fixes.filter(x => x.type === 'story');
  S.storyReport = stories.length ? applyStoryBlends({ q, p, nb, n, src: S.srcTake, srcIdx: S.srcIdx, fixes: stories, root: firstIdx('pelvis'), up: storyUp() }) : [];
  for (const fx of S.fixes.filter(x => x.type === 'smooth')) {
    const src = new Float32Array(q), srcP = new Float32Array(p);
    const sig = 2.0, R = 5;
    for (let f = fx.a; f <= fx.b; f++) {
      const w = ramp(f, fx.a, fx.b, 3);
      for (let i = 0; i < nb; i++) {
        const o = (f * nb + i) * 4;
        qa.fromArray(src, o);
        let x = 0, y = 0, z = 0, ww = 0;
        for (let d = -R; d <= R; d++) {
          const g = Math.min(n - 1, Math.max(0, f + d));
          const k = Math.exp(-(d * d) / (2 * sig * sig));
          qb.fromArray(src, (g * nb + i) * 4);
          const s = qa.dot(qb) < 0 ? -k : k;
          x += qb.x * s; y += qb.y * s; z += qb.z * s; ww += qb.w * s;
        }
        qb.set(x, y, z, ww).normalize();
        qa.slerp(qb, w).toArray(q, o);
        const po = (f * nb + i) * 3;
        let px = 0, py = 0, pz = 0, pw = 0;
        for (let d = -R; d <= R; d++) {
          const g = Math.min(n - 1, Math.max(0, f + d)), k = Math.exp(-(d * d) / (2 * sig * sig)), o2 = (g * nb + i) * 3;
          px += srcP[o2] * k; py += srcP[o2 + 1] * k; pz += srcP[o2 + 2] * k; pw += k;
        }
        p[po] += (px / pw - p[po]) * w; p[po + 1] += (py / pw - p[po + 1]) * w; p[po + 2] += (pz / pw - p[po + 2]) * w;
      }
    }
  }
  return { q, p };
}

function storyUp() {
  const pel = firstIdx('pelvis'), par = pel === undefined ? -1 : S.body.parentIdx[pel];
  if (par < 0) return [0, 1, 0];
  const q = new THREE.Quaternion(); S.body.restW[par].decompose(new THREE.Vector3(), q, new THREE.Vector3());
  return new THREE.Vector3(0, 1, 0).applyQuaternion(q.invert()).toArray();
}
function ensureSrc() {
  if (S.srcTake) return;
  S.srcTake = { q: S.take.q, p: S.take.p, n: S.take.n };
  S.srcIdx = Int32Array.from({ length: S.take.n }, (_, f) => f);
}
function compileAnimLayers() {
  S.evaluatedLayers=S.guidedReferenceBuild?S.animLayers.map(L=>L.guidedRange?{...L,mute:true}:L):S.animLayers;
  S.animCompiled = S.evaluatedLayers.length ? compileLayers(S.evaluatedLayers, name => S.body.byKey.get(keyOf(name)), S.take?.fps ?? 24, S.body.parentIdx) : null;
}
const tmpM = new THREE.Matrix4(), tmpV = new THREE.Vector3(), tmpV2 = new THREE.Vector3();
function poseFrame(f, base, rootOff, stage) {
  const B = S.body, nb = B.bones.length;
  for (let i = 0; i < nb; i++) {
    const b = B.bones[i];
    b.quaternion.fromArray(base.q, (f * nb + i) * 4);
    if (stage.layer) b.quaternion.multiply(layerDelta(i, f, qc));
    if (stage.layer && S.animCompiled?.has(i)) b.quaternion.fromArray(evalRotation(S.evaluatedLayers, S.animCompiled.get(i), b.quaternion.toArray(), f));
    b.position.fromArray(base.p, (f * nb + i) * 3);
    if (stage.layer && S.animCompiled?.has(i)) { const e = S.animCompiled.get(i); if (e.some(x => x.tcurves)) b.position.fromArray(evalTranslation(S.evaluatedLayers, e, b.position.toArray(), f)); }
  }
  B.obj.updateMatrixWorld(true);
  const pel = firstIdx('pelvis');
  if (stage.layer && S.leanW) applyLean(f);
  if (rootOff && pel !== undefined) {
    const o = rootOff[f];
    if (o && (o[0] || o[1] || o[2] || o[3])) {
      const b = B.bones[pel];
      tmpM.copy(b.parent.matrixWorld).invert();
      const w0 = tmpV.setFromMatrixPosition(b.matrixWorld);
      const l0 = w0.clone().applyMatrix4(tmpM);
      const l1 = tmpV2.set(w0.x + o[0], w0.y + (o[1] || 0), w0.z + o[2]).applyMatrix4(tmpM);
      b.position.add(l1.sub(l0));
      b.updateMatrixWorld(true);
      if (o[3]) {
        setWorldQuat(b, new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), o[3]).multiply(b.getWorldQuaternion(new THREE.Quaternion())));
        b.updateMatrixWorld(true);
      }
    }
  }
}

const LEGS = { l: ['thigh_l', 'calf_l', 'foot_l'], r: ['thigh_r', 'calf_r', 'foot_r'] };
const ARM_L = ['upperarm_l', 'lowerarm_l', 'hand_l'];

const wq = new THREE.Quaternion(), pq = new THREE.Quaternion();
function setWorldQuat(bone, q) {
  bone.parent.getWorldQuaternion(pq);
  bone.quaternion.copy(pq.invert().multiply(q));
  bone.updateMatrixWorld(true);
}
const A = new THREE.Vector3(), Bv = new THREE.Vector3(), Cv = new THREE.Vector3(), T = new THREE.Vector3();
const IKSTATE = new Map();
function twoBoneIK(ia, ib, ic, target, targetQ, w, keepLocal, poleHint) {
  if (w <= 0) return;
  const bones = S.body.bones, a = bones[ia], b = bones[ib], c = bones[ic];
  const endQ = c.getWorldQuaternion(new THREE.Quaternion());
  A.setFromMatrixPosition(a.matrixWorld); Bv.setFromMatrixPosition(b.matrixWorld); Cv.setFromMatrixPosition(c.matrixWorld);
  T.copy(Cv).lerp(target, w);
  const st = IKSTATE.get(ib) || {};
  IKSTATE.set(ib, st);
  const pole = poleHint ? poleHint.clone() : new THREE.Vector3().fromArray(stablePole(A.toArray(), Bv.toArray(), Cv.toArray(), { height: 1.8 * S.upm }, st).poleVector);
  const sol = solveTwoBone(A.toArray(), Bv.toArray(), Cv.toArray(), T.toArray(), { pole: pole.toArray(), prevBend: st.bend, guard: 'strict' });
  st.bend = sol.bend;
  const E = new THREE.Vector3().fromArray(sol.mid);
  const r1 = new THREE.Quaternion().setFromUnitVectors(Bv.clone().sub(A).normalize(), E.clone().sub(A).normalize());
  a.getWorldQuaternion(wq);
  setWorldQuat(a, r1.multiply(wq));
  const Bn = new THREE.Vector3().setFromMatrixPosition(b.matrixWorld);
  Cv.setFromMatrixPosition(c.matrixWorld);
  const r2 = new THREE.Quaternion().setFromUnitVectors(Cv.clone().sub(Bn).normalize(), T.clone().sub(Bn).normalize());
  b.getWorldQuaternion(wq);
  setWorldQuat(b, r2.multiply(wq));
  const goal = targetQ ? c.getWorldQuaternion(new THREE.Quaternion()).copy(endQ).slerp(targetQ, w) : endQ;
  if (targetQ && keepLocal && S.twist !== false) {
    const ax = new THREE.Vector3().setFromMatrixPosition(c.matrixWorld).sub(new THREE.Vector3().setFromMatrixPosition(b.matrixWorld)).normalize();
    const bq = b.getWorldQuaternion(new THREE.Quaternion());
    const dq = goal.clone().multiply(keepLocal.clone().invert()).multiply(bq.clone().invert());
    if (dq.w < 0) dq.set(-dq.x, -dq.y, -dq.z, -dq.w);
    const k = dq.x * ax.x + dq.y * ax.y + dq.z * ax.z;
    const tw = new THREE.Quaternion(ax.x * k, ax.y * k, ax.z * k, dq.w).normalize();
    setWorldQuat(b, new THREE.Quaternion().slerp(tw, 0.5).multiply(bq));
  }
  setWorldQuat(c, goal);
}

function armPole() {
  const w = k => { const i = idx(k); return i === undefined ? null : new THREE.Vector3().setFromMatrixPosition(S.body.bones[i].matrixWorld); };
  const neck = w('neck_01') || w('head'), pel = w('pelvis'), cl = w('clavicle_l'), cr = w('clavicle_r');
  if (!neck || !pel || !cl || !cr) return null;
  return pel.sub(neck).normalize().addScaledVector(cl.sub(cr).normalize(), 0.6);
}

const SEG = [
  ['pelvis', 'spine_02', 0.142], ['spine_02', 'spine_04|spine_03', 0.163], ['spine_04|spine_03', 'neck_01', 0.16], ['neck_01|head', 'head', 0.069],
  ['upperarm_l', 'lowerarm_l', 0.0271], ['lowerarm_l', 'hand_l', 0.0162], ['hand_l', 'hand_l', 0.0061],
  ['upperarm_r', 'lowerarm_r', 0.0271], ['lowerarm_r', 'hand_r', 0.0162], ['hand_r', 'hand_r', 0.0061],
  ['thigh_l', 'calf_l', 0.1416], ['calf_l', 'foot_l', 0.0433], ['foot_l', 'ball_l|foot_l', 0.0137],
  ['thigh_r', 'calf_r', 0.1416], ['calf_r', 'foot_r', 0.0433], ['foot_r', 'ball_r|foot_r', 0.0137],
];
function segIdx() {
  return SEG.map(([a, b, m]) => [firstIdx(...a.split('|')), firstIdx(...b.split('|')), m]).filter(s => s[0] !== undefined && s[1] !== undefined);
}
function comFromWorld(W, f, nb, segs, out, o) {
  let m = 0, x = 0, y = 0, z = 0;
  for (const [a, b, w] of segs) {
    const ia = (f * nb + a) * 16 + 12, ib = (f * nb + b) * 16 + 12;
    x += (W[ia] + W[ib]) / 2 * w; y += (W[ia + 1] + W[ib + 1]) / 2 * w; z += (W[ia + 2] + W[ib + 2]) / 2 * w; m += w;
  }
  out[o] = x / m; out[o + 1] = y / m; out[o + 2] = z / m;
}

function runPass(base, rootOff, stage) {
  const B = S.body, nb = B.bones.length, n = S.take.n;
  IKSTATE.clear();
  const W = new Float32Array(n * nb * 16), q = new Float32Array(n * nb * 4), p = new Float32Array(n * nb * 3);
  const L = { l: LEGS.l.map(idx), r: LEGS.r.map(idx) }, AL = ARM_L.map(idx), hr = idx('hand_r');
  for (let f = 0; f < n; f++) {
    poseFrame(f, base, rootOff, stage);
    if (stage.ik) {
      if (S.levelFeet) levelFeet(f);
      for (const lk of S.locks) {
        if (f < lk.a || f > lk.b || !lk.target) continue;
        const ids = L[lk.side];
        if (ids.some(v => v === undefined)) continue;
        twoBoneIK(ids[0], ids[1], ids[2], new THREE.Vector3().fromArray(lk.target), new THREE.Quaternion().fromArray(lk.targetQ), ramp(f, lk.a, lk.b, 4));
      }
      for (const t of S.groundTargets?.get(f) || []) {
        const ids = L[t.side];
        if (ids.some(v => v === undefined)) continue;
        twoBoneIK(ids[0], ids[1], ids[2], new THREE.Vector3().fromArray(t.target), t.q ? new THREE.Quaternion().fromArray(t.q) : null, t.w);
      }
      for (const t of S.rollTargets?.get(f) || []) {
        const ids = L[t.side];
        if (ids.some(v => v === undefined)) continue;
        twoBoneIK(ids[0], ids[1], ids[2], new THREE.Vector3().fromArray(t.footP), new THREE.Quaternion().fromArray(t.footQ), t.w);
        const cur = B.bones[t.ball].getWorldQuaternion(new THREE.Quaternion());
        setWorldQuat(B.bones[t.ball], cur.slerp(new THREE.Quaternion().fromArray(t.ballQ), t.w));
      }
      for (const t of S.floorTargets?.get(f) || []) {
        const ids = L[t.side];
        if (ids.some(v => v === undefined)) continue;
        twoBoneIK(ids[0], ids[1], ids[2], new THREE.Vector3().fromArray(t.ankleP), new THREE.Quaternion().fromArray(t.ankleQ), t.w ?? 1);
        const bi = firstIdx('ball_' + t.side);
        if (t.ballQ && bi !== undefined) { const cur = B.bones[bi].getWorldQuaternion(new THREE.Quaternion()); setWorldQuat(B.bones[bi], cur.slerp(new THREE.Quaternion().fromArray(t.ballQ), t.w ?? 1)); }
      }
      for (const cf of S.fixes) if (cf.type === 'carry' && f >= cf.a && f <= cf.b) applyCarry(cf, f);
      for(const fx of S.guidedReferenceBuild?[]:S.fixes)for(const t of fx.contactTargets||[]){
        if(f<t.a||f>t.b)continue;
        const ids=L[t.side];if(!ids||ids.some(i=>i===undefined))continue;
        twoBoneIK(ids[0],ids[1],ids[2],new THREE.Vector3().fromArray(t.target),new THREE.Quaternion().fromArray(t.targetQ),ramp(f,t.a,t.b,4));
      }
      for (const pn of S.pins) {
        if (f < pn.a || f > pn.b || AL.some(v => v === undefined) || hr === undefined) continue;
        const m = pn.world ? new THREE.Matrix4().fromArray(pn.world) : B.bones[hr].matrixWorld.clone().multiply(new THREE.Matrix4().fromArray(pn.off));
        const tp = new THREE.Vector3(), tq = new THREE.Quaternion(), ts = new THREE.Vector3();
        m.decompose(tp, tq, ts);
        twoBoneIK(AL[0], AL[1], AL[2], tp, tq, ramp(f, pn.a, pn.b, 4), B.bones[AL[2]].quaternion.clone(), armPole());
      }
    }
    for (let i = 0; i < nb; i++) {
      const b = B.bones[i];
      b.matrixWorld.toArray(W, (f * nb + i) * 16);
      b.quaternion.toArray(q, (f * nb + i) * 4);
      b.position.toArray(p, (f * nb + i) * 3);
    }
  }
  for (let i = 0; i < nb; i++) {
    const c = quatContinuity(readTrack(q, nb, i, 4));
    if (c.flips) writeTrack(q, nb, i, 4, c);
  }
  return { W, q, p };
}

function comSeries(W) {
  const nb = S.body.bones.length, n = S.take.n, segs = segIdx(), c = new Float32Array(n * 3);
  for (let f = 0; f < n; f++) comFromWorld(W, f, nb, segs, c, f * 3);
  return c;
}

function momentumOffsets(c, contact) {
  const n = S.take.n, fps = S.take.fps, g = 9.81 * S.upm, mu = +$('mu').value || 0.8, amax = mu * g;
  const off = new Array(n).fill(null);
  for (const fx of S.fixes.filter(x => x.type === 'momentum')) {
    const a = fx.a, b = Math.min(fx.b, n - 1);
    if (b - a < 3) continue;
    const cx = [c[a * 3]], cz = [c[a * 3 + 2]];
    let vx = (c[(a + 1) * 3] - c[a * 3]) * fps, vz = (c[(a + 1) * 3 + 2] - c[a * 3 + 2]) * fps;
    for (let f = a; f < b; f++) {
      const tvx = (c[(f + 1) * 3] - c[f * 3]) * fps, tvz = (c[(f + 1) * 3 + 2] - c[f * 3 + 2]) * fps;
      let ax = (tvx - vx) * fps, az = (tvz - vz) * fps;
      const lim = contact[f] ? amax : 0;
      const mag = Math.hypot(ax, az);
      if (mag > lim) { ax *= lim / (mag || 1); az *= lim / (mag || 1); }
      vx += ax / fps; vz += az / fps;
      cx.push(cx[cx.length - 1] + vx / fps); cz.push(cz[cz.length - 1] + vz / fps);
    }
    const ex = c[b * 3] - cx[b - a], ez = c[b * 3 + 2] - cz[b - a];
    for (let f = a; f <= b; f++) {
      const s = smooth((f - a) / (b - a));
      off[f] = [cx[f - a] + ex * s - c[f * 3], 0, cz[f - a] + ez * s - c[f * 3 + 2]];
    }
  }
  return off;
}

function contactsFromWorld(W) {
  const nb = S.body.bones.length, n = S.take.n, fps = S.take.fps;
  const feet = ['l', 'r'].map(s => [idx('foot_' + s), firstIdx('ball_' + s, 'foot_' + s)]);
  const ys = [];
  for (let f = 0; f < n; f++) for (const [a, b] of feet) if (a !== undefined) ys.push(Math.min(W[(f * nb + a) * 16 + 13], W[(f * nb + b) * 16 + 13]));
  ys.sort((x, y) => x - y);
  const ground = ys[Math.floor(ys.length * 0.03)] ?? S.ground;
  const per = { l: new Uint8Array(n), r: new Uint8Array(n) }, any = new Uint8Array(n), speed = { l: new Float32Array(n), r: new Float32Array(n) };
  ['l', 'r'].forEach((s, k) => {
    const [a, b] = feet[k];
    if (a === undefined) return;
    for (let f = 0; f < n; f++) {
      const o = (f * nb + b) * 16, o1 = (Math.min(n - 1, f + 1) * nb + b) * 16, o0 = (Math.max(0, f - 1) * nb + b) * 16;
      const h = Math.min(W[(f * nb + a) * 16 + 13], W[o + 13]) - ground;
      const sp = Math.hypot(W[o1 + 12] - W[o0 + 12], W[o1 + 14] - W[o0 + 14]) * fps / 2 / S.upm;
      speed[s][f] = sp;
      per[s][f] = h < 0.1 * S.upm ? 1 : 0;
      if (per[s][f]) any[f] = 1;
    }
  });
  return { ground, per, any, speed };
}

function floorOffsets(W, ground) {
  const nb = S.body.bones.length, n = S.take.n, knees = ['calf_l', 'calf_r'].map(idx).filter(v => v !== undefined);
  const r = 0.06 * S.upm, off = new Array(n).fill(null);
  let hits = 0;
  for (let f = 0; f < n; f++) {
    let lift = 0;
    for (const k of knees) lift = Math.max(lift, kneeFloor([0, W[(f * nb + k) * 16 + 13], 0], { floor: ground, radius: r }).penetration);
    if (lift > 1e-6) { off[f] = [0, lift, 0]; hits++; }
  }
  S.kneeFloorFrames = hits;
  return off;
}

const SEG_GROUND_EASE = 6;
const FLOOR_EASE = 3;
function stancePlants(W, ground) {
  const nb = S.body.bones.length, R = S.body.restW, d = (i, j) => new THREE.Vector3().setFromMatrixPosition(R[i]).distanceTo(new THREE.Vector3().setFromMatrixPosition(R[j])) / S.upm * 100;
  const legLength_cm = d(idx('thigh_l'), idx('calf_l')) + d(idx('calf_l'), idx('foot_l'));
  const footLength_cm = idx('ball_l') !== undefined ? Math.max(20, 1.6 * d(idx('foot_l'), idx('ball_l'))) : 26;
  const legs = { l: { foot: idx('foot_l'), ball: firstIdx('ball_l', 'foot_l') }, r: { foot: idx('foot_r'), ball: firstIdx('ball_r', 'foot_r') } };
  return detectFootPlants({ W, nb, n: S.take.n, fps: S.take.fps, upm: S.upm, ground, legs, legLength_cm, footLength_cm }).filter(p => !p.sliding);
}
function groundPlan(W, ct) {
  const nb = S.body.bones.length, n = S.take.n, fixes = S.fixes.filter(x => x.type === 'ground');
  const targets = new Map(), drop = new Array(n).fill(null);
  if (!fixes.length) return { targets, drop };
  const ids = s => ({ hip: idx('thigh_' + s), knee: idx('calf_' + s), foot: idx('foot_' + s), ball: firstIdx('ball_' + s, 'foot_' + s) });
  const y = (f, i) => W[(f * nb + i) * 16 + 13], pos = (f, i) => [W[(f * nb + i) * 16 + 12], W[(f * nb + i) * 16 + 13], W[(f * nb + i) * 16 + 14]];
  const med = a => { const s = [...a].sort((u, v) => u - v); return s.length ? s[s.length >> 1] : 0; };
  const level = {};
  for (const s of ['l', 'r']) {
    const k = ids(s), fy = [], by = [];
    for (let f = 0; f < n; f++) if (ct.per[s][f] && ct.speed[s][f] < 0.3) { fy.push(y(f, k.foot) - ct.ground); by.push(y(f, k.ball) - ct.ground); }
    level[s] = { foot: med(fy), ball: med(by) };
  }
  const reachAt = f => S.fixes.some(t => t.type === 'travel' && t.reach && f >= t.a && f <= t.b + (t.ease ?? 8));
  for (const g of fixes) {
    const lift = s => { let t = 0; for (let f = g.a; f <= g.b; f++) t += Math.min(y(f, ids(s).foot), y(f, ids(s).ball)); return t; };
    const side = g.side || (lift('l') <= lift('r') ? 'l' : 'r');
    g.side = side;
    const k = ids(side);
    if ([k.hip, k.knee, k.foot, k.ball].some(v => v === undefined)) continue;
    let anchor = g.a, slow = Infinity;
    for (let f = g.a; f <= g.b; f++) {
      const f0 = Math.max(g.a, f - 1), f1 = Math.min(g.b, f + 1), a0 = pos(f0, k.ball), a1 = pos(f1, k.ball);
      const v = Math.hypot(a1[0] - a0[0], a1[2] - a0[2]);
      if (v < slow) { slow = v; anchor = f; }
    }
    g.anchor = anchor;
    const hold = pos(anchor, k.ball);
    for (let f = g.a; f <= g.b; f++) {
      const d = Math.min(y(f, k.ball) - (ct.ground + level[side].ball), y(f, k.foot) - (ct.ground + level[side].foot));
      const w = smooth(Math.max(0, ramp(f, g.a, g.b, SEG_GROUND_EASE))), target = pos(f, k.foot);
      const ball = pos(f, k.ball);
      target[0] += hold[0] - ball[0];
      target[2] += hold[2] - ball[2];
      target[1] -= Math.max(0, d);
      const hip = pos(f, k.hip), L = Math.hypot(...pos(f, k.knee).map((v, j) => v - hip[j])) + Math.hypot(...pos(f, k.foot).map((v, j) => v - pos(f, k.knee)[j]));
      const h = Math.hypot(target[0] - hip[0], target[2] - hip[2]), Lm = 0.995 * L;
      if (h < Lm) {
        const need = (hip[1] - target[1]) - Math.sqrt(Lm * Lm - h * h);
        if (need > 0 && !reachAt(f)) drop[f] = [0, -need * w, 0];
      }
      if (!targets.has(f)) targets.set(f, []);
      targets.get(f).push({ side, target, w });
    }
  }
  return { targets, drop, level };
}

function lockTargets(W) {
  const nb = S.body.bones.length;
  for (const lk of S.locks) {
    const fi = idx('foot_' + lk.side);
    if (fi === undefined) continue;
    let best = lk.a, bs = Infinity;
    for (let f = lk.a; f <= lk.b; f++) {
      const f0 = Math.max(lk.a, f - 1), f1 = Math.min(lk.b, f + 1);
      const s = Math.hypot(W[(f1 * nb + fi) * 16 + 12] - W[(f0 * nb + fi) * 16 + 12], W[(f1 * nb + fi) * 16 + 14] - W[(f0 * nb + fi) * 16 + 14]);
      if (s < bs) { bs = s; best = f; }
    }
    const m = new THREE.Matrix4().fromArray(W, (best * nb + fi) * 16);
    const p = new THREE.Vector3(), q = new THREE.Quaternion(), sc = new THREE.Vector3();
    m.decompose(p, q, sc);
    lk.target = p.toArray(); lk.targetQ = q.toArray(); lk.anchor = best;
  }
}

function rollPlan(W) {
  const out = new Map(), nb = S.body.bones.length, n = S.take.n;
  for (const fx of S.fixes.filter(x => x.type === 'roll')) {
    const s = fx.side, foot = idx('foot_' + s), ball = idx('ball_' + s), calf = idx('calf_' + s);
    if (foot === undefined || ball === undefined || calf === undefined) continue;
    const a = Math.max(0, fx.a), b = Math.min(n - 1, fx.b);
    const ankle = new Array(n).fill(0), bh = new Array(n).fill(0);
    for (let f = 0; f < n; f++) { ankle[f] = W[(f * nb + foot) * 16 + 13]; bh[f] = W[(f * nb + ball) * 16 + 13]; }
    let ref = fx.ref;
    if (ref === undefined || ref < a || ref > b) {
      let mA = Infinity, mB = Infinity, best = Infinity;
      for (let f = a; f <= b; f++) { mA = Math.min(mA, ankle[f]); mB = Math.min(mB, bh[f]); }
      for (let f = a; f <= b; f++) { const v = ankle[f] - mA + bh[f] - mB; if (v < best) { best = v; ref = f; } }
      fx.ref = ref;
    }
    const o1 = (ref * nb + foot) * 16 + 12, o2 = (ref * nb + ball) * 16 + 12;
    const length = Math.hypot(W[o1] - W[o2], W[o1 + 1] - W[o2 + 1], W[o1 + 2] - W[o2 + 2]);
    const start = fx.start ?? 30, end = fx.end ?? 60;
    const curve = rollCurveFromPlant({ heights: { ankle, ball: bh }, a, b, start, end, length });
    for (let f = a; f <= b; f++) {
      const r = rollPose({ W, nb, f: ref, side: s, legIdx: { calf, foot, ball }, roll: curve[f], start, end });
      if (!out.has(f)) out.set(f, []);
      out.get(f).push({ side: s, footP: r.footP, footQ: r.footQ, ballQ: r.ballQ, ball, w: ramp(f, a, b, 3) });
    }
  }
  return out;
}

function pullPlan(W, ct) {
  const fx = S.fixes.find(x => x.type === 'pull');
  if (!fx || !S.pins.length) return [];
  const nb = S.body.bones.length, n = S.take.n, hr = idx('hand_r');
  const hand = { shoulder: idx('upperarm_l'), elbow: idx('lowerarm_l'), hand: idx('hand_l') };
  if (hr === undefined || Object.values(hand).some(v => v === undefined)) return [];
  const legIds = s => ({ thigh: idx('thigh_' + s), calf: idx('calf_' + s), foot: idx('foot_' + s) });
  const res = [];
  for (const pn of S.pins) {
    if (!pn.world) continue;
    const wp = [pn.world[12], pn.world[13], pn.world[14]];
    const goal = () => wp;
    const o = pullOffsets({ W, nb, n, frames: [pn.a, pn.b], hand, handGoal: goal, legs: { l: legIds('l'), r: legIds('r') }, feetPlanted: (f, s) => !!ct.per[s][f] || S.locks.some(l => l.side === s && f >= l.a && f <= l.b), pull: fx.pull ?? 1, gravity: fx.gravity ?? 0.6, iterations: fx.iterations ?? 10 });
    if (o.some(Boolean)) res.push(o);
  }
  fx.hits = res.reduce((t, o) => t + o.filter(Boolean).length, 0);
  return res;
}

const LEVEL_FULL_CM = 15, LEVEL_FADE_CM = 15, LEVEL_SMOOTH = 4, LEVEL_STEEP = 0.7, LEVEL_STEEP_FADE = 0.35;
function levelPlan(ground, W) {
  const lf = S.fixes.find(x => x.type === 'level');
  if (!lf) return null;
  const up = new THREE.Vector3(0, 1, 0), plan = { ground, sides: [] };
  for (const s of lf.side ? [lf.side] : ['l', 'r']) {
    const fi = idx('foot_' + s), bi = firstIdx('ball_' + s);
    if (fi === undefined || bi === undefined) continue;
    const rq = new THREE.Quaternion(), rp = new THREE.Vector3(), rs = new THREE.Vector3(), bp = new THREE.Vector3().setFromMatrixPosition(S.body.restW[bi]);
    S.body.restW[fi].decompose(rp, rq, rs);
    const fwd = bp.clone().sub(rp); fwd.y = 0; fwd.normalize();
    const inv = rq.clone().invert();
    const nb = S.body.bones.length, n = S.take.n, raw = new Float64Array(n), w = new Float64Array(n);
    for (let f = 0; f < n; f++) { const h = (W[(f * nb + bi) * 16 + 13] - ground) / S.upm * 100; raw[f] = Math.min(1, Math.max(0, (LEVEL_FULL_CM + LEVEL_FADE_CM - h) / LEVEL_FADE_CM)); }
    for (let f = 0; f < n; f++) { let t = 0, c = 0; for (let k = -LEVEL_SMOOTH; k <= LEVEL_SMOOTH; k++) { const g = Math.min(n - 1, Math.max(0, f + k)); t += raw[g]; c++; } w[f] = smooth(t / c); }
    plan.sides.push({ fi, bi, w, upL: up.clone().applyQuaternion(inv), fwdL: fwd.applyQuaternion(inv) });
  }
  return plan;
}
function levelFeet(f) {
  const L = S.levelFeet, up = new THREE.Vector3(0, 1, 0), q = new THREE.Quaternion();
  for (const sd of L.sides) {
    const b = S.body.bones[sd.fi], ball = S.body.bones[sd.bi];
    const w = sd.w[f];
    if (w <= 0) continue;
    b.getWorldQuaternion(q);
    const fwd = sd.fwdL.clone().applyQuaternion(q).normalize(), u = sd.upL.clone().applyQuaternion(q);
    const flat = Math.min(1, Math.max(0, (LEVEL_STEEP - Math.abs(fwd.y)) / LEVEL_STEEP_FADE));
    if (flat <= 0) continue;
    const a = u.clone().projectOnPlane(fwd), t = up.clone().projectOnPlane(fwd);
    if (a.lengthSq() < 1e-8 || t.lengthSq() < 1e-8) continue;
    const ang = Math.atan2(a.clone().cross(t).dot(fwd), a.dot(t));
    setWorldQuat(b, new THREE.Quaternion().setFromAxisAngle(fwd, ang * w * smooth(flat)).multiply(q));
  }
}

function applyCarry(cf, f) {
  const e = Math.max(1, cf.ease ?? 10), w = smooth(Math.min(1, (f - cf.a + 1) / e, (cf.b - f + 1) / e));
  if (w <= 0) return;
  const B = S.body.bones, P = k => new THREE.Vector3().setFromMatrixPosition(B[idx(k)].matrixWorld);
  const right = P('thigh_r').sub(P('thigh_l')).setY(0).normalize(), up = new THREE.Vector3(0, 1, 0), fwd = up.clone().cross(right).normalize();
  const fl = P('foot_l'), bl = P(idx('ball_l') !== undefined ? 'ball_l' : 'foot_l');
  if (fwd.dot(bl.sub(fl)) < 0) fwd.negate();
  const drop = (cf.drop ?? 25) * S.upm / 100, pull = (cf.pull ?? 8) * S.upm / 100, tilt = (cf.tilt ?? 35) * Math.PI / 180;
  for (const [side, arm, k] of [['r', ['upperarm_r', 'lowerarm_r', 'hand_r'], 1], ['l', ['upperarm_l', 'lowerarm_l', 'hand_l'], cf.left ?? 0.7]]) {
    const ids = arm.map(idx);
    if (ids.some(v => v === undefined)) continue;
    const h = P(arm[2]), tq = B[ids[2]].getWorldQuaternion(new THREE.Quaternion());
    const target = h.clone().addScaledVector(up, -drop * k).addScaledVector(fwd, -pull * k);
    if (side === 'r') tq.premultiply(new THREE.Quaternion().setFromAxisAngle(right, tilt));
    twoBoneIK(ids[0], ids[1], ids[2], target, side === 'r' ? tq : null, w);
  }
}
function facingAt(W, f) {
  const tl = wpos(W, f, idx('thigh_l')), tr = wpos(W, f, idx('thigh_r')), fl = wpos(W, f, idx('foot_l')), bl = wpos(W, f, firstIdx('ball_l', 'foot_l'));
  const rx = tr[0] - tl[0], rz = tr[2] - tl[2], rl = Math.hypot(rx, rz) || 1;
  let fx = rz / rl, fz = -rx / rl;
  if (fx * (bl[0] - fl[0]) + fz * (bl[2] - fl[2]) < 0) { fx = -fx; fz = -fz; }
  return { fwd: [fx, 0, fz], right: [rx / rl, 0, rz / rl] };
}
function travelOffsets(W) {
  const n = S.take.n, fps = S.take.fps, out = new Array(n).fill(null), fixes = S.fixes.filter(x => x.type === 'travel');
  if (!fixes.length) return out;
  const acc = new Float64Array(n * 3);
  for (const fx of fixes) {
    const { fwd } = facingAt(W, fx.a), e = Math.max(1, fx.ease ?? 8), v = (fx.speed ?? 3) * S.upm / fps;
    let x = 0, z = 0;
    for (let f = fx.a; f < n; f++) {
      if (f <= fx.b) { const w = smooth(Math.min(1, (f - fx.a + 1) / e, (fx.b - f + 1) / e)); x += fwd[0] * v * w; z += fwd[2] * v * w; }
      acc[f * 3] += x; acc[f * 3 + 2] += z;
    }
  }
  for (const fx of fixes.filter(x => x.footsteps && x.balance)) {
    const lo = firstIdx('spine_02', 'spine_01'), hi = firstIdx('spine_04', 'spine_03'), e = Math.max(1, fx.ease ?? 8);
    if (lo === undefined || hi === undefined) continue;
    for (let f = fx.a; f <= fx.b; f++) {
      const sh = balanceShift(wpos(W, f, lo), wpos(W, f, hi), fx.balance * smooth(Math.min(1, (f - fx.a + 1) / e, (fx.b - f + 1) / e)));
      acc[f * 3] += sh[0]; acc[f * 3 + 2] += sh[2];
    }
  }
  const bf = fixes.filter(x => x.bounce);
  if (bf.length) {
    const nb = S.body.bones.length, legs = { l: { foot: idx('foot_l'), ball: firstIdx('ball_l', 'foot_l') }, r: { foot: idx('foot_r'), ball: firstIdx('ball_r', 'foot_r') } };
    const R = S.body.restW, d = (i, j) => new THREE.Vector3().setFromMatrixPosition(R[i]).distanceTo(new THREE.Vector3().setFromMatrixPosition(R[j])) / S.upm * 100;
    const plants = detectFootPlants({ W, nb, n, fps, upm: S.upm, ground: contactsFromWorld(W).ground, legs, legLength_cm: d(idx('thigh_l'), idx('calf_l')) + d(idx('calf_l'), idx('foot_l')), footLength_cm: Math.max(20, 1.6 * d(idx('foot_l'), idx('ball_l'))), minFrames: 2 });
    for (const fx of bf) {
      const e = Math.max(1, fx.ease ?? 8), amp = fx.bounce * S.upm / 100;
      for (const pl of plants) {
        if (pl.b < fx.a || pl.a > fx.b) continue;
        const a = pl.a - 1, L = pl.b - pl.a + 3;
        for (let f = Math.max(0, a); f <= Math.min(n - 1, a + L); f++) {
          const w = smooth(Math.min(1, Math.max(0, (f - fx.a + 1) / e), Math.max(0, (fx.b - f + 1) / e)));
          acc[f * 3 + 1] -= amp * w * Math.sin(Math.PI * (f - a) / L);
        }
      }
    }
  }
  for (let f = 0; f < n; f++) if (acc[f * 3] || acc[f * 3 + 1] || acc[f * 3 + 2]) out[f] = [acc[f * 3], acc[f * 3 + 1], acc[f * 3 + 2]];
  return out;
}
function joinOffsets(W) {
  const n = S.take.n, out = new Array(n).fill(null), joins = S.fixes.filter(x => x.type === 'join').sort((u, v) => u.at - v.at);
  let dx = 0, dz = 0, j = 0;
  const pel = idx('pelvis');
  for (let f = 0; f < n; f++) {
    while (j < joins.length && joins[j].at === f && f >= 2) {
      const p2 = wpos(W, f - 2, pel), p1 = wpos(W, f - 1, pel), p0 = wpos(W, f, pel);
      dx += p1[0] + (p1[0] - p2[0]) - p0[0]; dz += p1[2] + (p1[2] - p2[2]) - p0[2];
      j++;
    }
    if (dx || dz) out[f] = [dx, 0, dz];
  }
  return out;
}
const sub2 = (a, b) => [a[0] - b[0], a[2] - b[2]];
function stridePlan(Wpre, Wpost) {
  S.reachHolds = []; S.footstepPlans = [];
  const fixes = S.fixes.filter(x => x.type === 'travel' && x.plant !== false);
  const targets = new Map();
  if (!fixes.length) return targets;
  const nb = S.body.bones.length, R = S.body.restW, d = (i, j) => new THREE.Vector3().setFromMatrixPosition(R[i]).distanceTo(new THREE.Vector3().setFromMatrixPosition(R[j])) / S.upm * 100;
  const legs = { l: { foot: idx('foot_l'), ball: firstIdx('ball_l', 'foot_l') }, r: { foot: idx('foot_r'), ball: firstIdx('ball_r', 'foot_r') } };
  const ground = contactsFromWorld(Wpre).ground;
  const rest = Float32Array.from(S.body.restW.flatMap(m => m.toArray()));
  const plants = detectFootPlants({ W: Wpre, nb, n: S.take.n, fps: S.take.fps, upm: S.upm, ground, legs, legLength_cm: d(idx('thigh_l'), idx('calf_l')) + d(idx('calf_l'), idx('foot_l')), footLength_cm: Math.max(20, 1.6 * d(idx('foot_l'), idx('ball_l'))), minFrames: 2 });
  const footQ = f => { const m = new THREE.Matrix4().fromArray(Wpost, f), pp = new THREE.Vector3(), qq = new THREE.Quaternion(), ss = new THREE.Vector3(); m.decompose(pp, qq, ss); return qq; };
  const restFoot = new THREE.Vector3().setFromMatrixPosition(R[idx('foot_l')]).distanceTo(new THREE.Vector3().setFromMatrixPosition(R[firstIdx('ball_l', 'foot_l')]));
  const yawTo = (f, fv, frac) => { frac *= Math.min(1, Math.max(0, (Math.hypot(fv.x, fv.z) / restFoot - 0.15) / 0.25)); if (!frac) return new THREE.Quaternion(); const dir = facingAt(Wpost, f).fwd, a = Math.atan2(fv.x, fv.z) - Math.atan2(dir[0], dir[2]), wr = Math.max(-0.8, Math.min(0.8, Math.atan2(Math.sin(a), Math.cos(a)))); return new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -wr * frac); };
  for (const fx of fixes) {
    const own = fx.footsteps ? [] : plants.filter(pl => !(pl.b < fx.a || pl.a > fx.b + (fx.ease ?? 8)));
    const fsPlan = fx.footsteps ? footstepPlan(fx, Wpre, Wpost, legs, ground) : null;
    if (fsPlan) S.footstepPlans.push({ a: fx.a, b: fx.b, cycle: fsPlan.cycle, contactFrames: fsPlan.contactFrames, stepS: +fsPlan.stepS.toFixed(3), footprints: fsPlan.footprints.map(k => ({ side: k.side, m: k.m, a: k.a, b: k.b, from: k.from })) });
    if (fx.reach || fsPlan) {
      const tw = f => smooth(Math.min(1, Math.max(0, (f - fx.a + 1) / Math.max(1, fx.ease ?? 8)), Math.max(0, (fx.b - f + 1) / Math.max(1, fx.ease ?? 8))));
      for (const side of ['l', 'r']) {
        const gs = fsPlan ? [] : S.fixes.filter(g => g.type === 'ground' && (!g.side || g.side === side) && g.b >= fx.a && g.a <= fx.b).map(g => { const m = g.anchor ?? Math.round((g.a + g.b) / 2), r = fx.stance ?? Math.max(1, Math.round(0.3 * 0.87 * S.take.fps / Math.max(0.5, fx.speed ?? 3))); return { side, a: Math.max(g.a, m - r), b: Math.min(g.b, m + r), m, pseudo: true }; });
        const k = legs[side], ps = fsPlan ? fsPlan.footprints.filter(pl => pl.side === side).map(pl => ({ ...pl, pseudo: true })) : own.filter(pl => pl.side === side && !gs.some(g => pl.b >= g.a && pl.a <= g.b)).concat(gs).sort((u, v) => u.a - v.a);
        const offs = ps.map(pl => { const h = pl.pos ? pl.pos.slice() : wpos(Wpost, pl.m ?? Math.round((pl.a + pl.b) / 2), k.ball); if (pl.pseudo && !pl.pos) h[1] = ground + S.upm / 100; return { pl, h, oa: sub2(h, wpos(Wpost, pl.a, k.ball)), ob: sub2(h, wpos(Wpost, pl.b, k.ball)) }; });
        const swing = (f, ox, oz, w) => {
          const ank = wpos(Wpost, f, k.foot);
          const toe = (fx.toe ?? 0) * smooth(Math.min(1, Math.max(0, (f - fx.a + 1) / 4), Math.max(0, (fx.b - f + 1) / 4))), bl = wpos(Wpost, f, k.ball), q = toe ? yawTo(f, new THREE.Vector3(bl[0] - ank[0], 0, bl[2] - ank[2]), toe).multiply(footQ((f * nb + k.foot) * 16)).toArray() : null;
          targets.set(f, [...(targets.get(f) || []), { side, target: [ank[0] + ox * w, ank[1], ank[2] + oz * w], w: 1, q }]);
        };
        for (let i = 0; i + 1 < offs.length; i++) {
          const A = offs[i], Bn = offs[i + 1], L = Bn.pl.a - A.pl.b;
          for (let f = A.pl.b + 1; f < Bn.pl.a; f++) {
            const s = smooth((f - A.pl.b) / L);
            swing(f, A.ob[0] + (Bn.oa[0] - A.ob[0]) * s, A.ob[1] + (Bn.oa[1] - A.ob[1]) * s, fsPlan ? 1 : tw(f));
          }
        }
        if (fsPlan && offs.length) {
          const L = fx.settle ?? Math.max(2, fsPlan.cycle - fsPlan.contactFrames), free = f => f >= 0 && f < S.take.n && !(targets.get(f) || []).some(t => t.side === side);
          const first = offs[0], last = offs[offs.length - 1];
          for (let j = 1; j <= L; j++) {
            const f = first.pl.a - j, s = smooth(1 - j / (L + 1));
            if (free(f)) swing(f, first.oa[0] * s, first.oa[1] * s, 1);
          }
          for (let j = 1; j <= L; j++) {
            const f = last.pl.b + j, s = smooth(1 - j / (L + 1));
            if (free(f)) swing(f, last.ob[0] * s, last.ob[1] * s, 1);
          }
        }
        for (const o of offs) if (o.pl.pseudo) for (let f = Math.max(o.pl.a, fx.a); f <= Math.min(o.pl.b, fx.b + (fx.ease ?? 8)); f++) {
          const ank = wpos(Wpost, f, k.foot), bl = wpos(Wpost, f, k.ball), toe = (fx.toe ?? 0) * smooth(Math.min(1, Math.max(0, (f - fx.a + 1) / 4), Math.max(0, (fx.b - f + 1) / 4)));
          const fv = new THREE.Vector3(bl[0] - ank[0], bl[1] - ank[1], bl[2] - ank[2]), rot = yawTo(f, new THREE.Vector3(fv.x, 0, fv.z), toe), nf = fv.clone().applyQuaternion(rot);
          targets.set(f, [...(targets.get(f) || []), { side, own: true, target: [o.h[0] - nf.x, ank[1] + o.h[1] - (ank[1] + nf.y), o.h[2] - nf.z], w: 1, q: rot.multiply(footQ((f * nb + k.foot) * 16)).toArray() }]);
        }
        S.reachHolds = (S.reachHolds || []).concat(offs.map(o => ({ side, a: o.pl.a, b: o.pl.b, m: Math.round((o.pl.a + o.pl.b) / 2) })));
      }
    }
    for (const pl of own) {
    const k = legs[pl.side], hold = wpos(Wpost, fx.reach ? Math.round((pl.a + pl.b) / 2) : pl.a, k.ball), e = 2, flat = fx.flatten ?? 0;
    const rf = wpos(rest, 0, k.foot), rb = wpos(rest, 0, k.ball), restElev = Math.atan2(rb[1] - rf[1], Math.hypot(rb[0] - rf[0], rb[2] - rf[2]));
    for (let f = pl.a; f <= pl.b; f++) {
      let ball = wpos(Wpost, f, k.ball);
      const ank = wpos(Wpost, f, k.foot), w = smooth(Math.min(1, (f - pl.a + 1) / e, (pl.b - f + 1) / e));
      let q = null;
      if (flat || fx.toe || fx.pitch != null) {
        const fv = new THREE.Vector3(ball[0] - ank[0], ball[1] - ank[1], ball[2] - ank[2]);
        const elev = Math.atan2(fv.y, Math.hypot(fv.x, fv.z)), lat = fv.clone().cross(new THREE.Vector3(0, 1, 0)).normalize();
        const tw2 = smooth(Math.min(1, Math.max(0, (f - fx.a + 1) / 4), Math.max(0, (fx.b - f + 1) / 4))), rot = yawTo(f, new THREE.Vector3(fv.x, 0, fv.z), (fx.toe ?? 0) * tw2).multiply(new THREE.Quaternion().setFromAxisAngle(lat, (fx.pitch != null ? elev - restElev - fx.pitch * Math.PI / 180 : (elev - restElev) * flat)));
        const qq = footQ((f * nb + k.foot) * 16);
        q = rot.clone().multiply(qq).toArray();
        const nf = fv.applyQuaternion(rot);
        ball = [ank[0] + nf.x, ank[1] + nf.y, ank[2] + nf.z];
      }
      const t = { side: pl.side, target: [ank[0] + hold[0] - ball[0], ank[1], ank[2] + hold[2] - ball[2]], w: fx.reach ? 1 : w, q };
      targets.set(f, [...(targets.get(f) || []), t]);
    }
    }
  }
  return targets;
}
function footstepPlan(fx, Wpre, Wpost, legs, ground) {
  const n = S.take.n, nb = S.body.bones.length, R = S.body.restW, fps = S.take.fps;
  const d = (i, j) => new THREE.Vector3().setFromMatrixPosition(R[i]).distanceTo(new THREE.Vector3().setFromMatrixPosition(R[j]));
  const Sl = d(idx('thigh_l'), idx('calf_l')) + d(idx('calf_l'), idx('foot_l'));
  const track = (W, i) => { const t = new Float64Array(n * 3); for (let f = 0; f < n; f++) { const o = (f * nb + i) * 16; t[f * 3] = W[o + 12]; t[f * 3 + 1] = W[o + 13]; t[f * 3 + 2] = W[o + 14]; } return t; };
  const com = comSeries(Wpost);
  return planFootsteps({
    ball: { l: track(Wpre, legs.l.ball), r: track(Wpre, legs.r.ball) }, a: fx.a, b: fx.b, fps, S: Sl, speed: (fx.speed ?? 3) * S.upm, ground, cycle: fx.cycle, contact: fx.contact,
    com: f => [com[f * 3], com[f * 3 + 1], com[f * 3 + 2]], fwd: f => facingAt(Wpost, f).fwd, right: f => facingAt(Wpost, f).right,
    vPerFrame: (fx.speed ?? 3) * S.upm / fps, width: fx.width ?? 1, lift: S.upm / 100,
  });
}
function leanPlan(W) {
  const fixes = S.fixes.filter(x => x.type === 'lean');
  if (!fixes.length) return null;
  const n = S.take.n, w = new Float64Array(n), right = [];
  for (const fx of fixes) { const e = Math.max(1, fx.ease ?? 8); for (let f = fx.a; f <= fx.b; f++) w[f] += (fx.deg ?? 15) * smooth(Math.min(1, (f - fx.a + 1) / e, (fx.b - f + 1) / e)); }
  for (let f = 0; f < n; f++) right.push(facingAt(W, f).right);
  return { w, right };
}
function applyLean(f) {
  const deg = S.leanW.w[f];
  if (!deg) return;
  const bi = firstIdx('spine_01', 'spine_02');
  if (bi === undefined) return;
  const r = S.leanW.right[f], b = S.body.bones[bi];
  setWorldQuat(b, new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(r[0], 0, r[2]), -deg * Math.PI / 180).multiply(b.getWorldQuaternion(new THREE.Quaternion())));
  S.body.obj.updateMatrixWorld(true);
}
function insertTake(at, a, b) {
  const nb = S.body.bones.length, n = S.take.n, k = b - a + 1;
  if (a < 0 || b >= n || k < 1 || at < 1 || at > n - 1) throw new Error('insert range out of the take');
  ensureSrc(); S.srcIdx = insertIndex(S.srcIdx, at, a, b);
  const q = new Float32Array((n + k) * nb * 4), p = new Float32Array((n + k) * nb * 3);
  q.set(S.take.q.subarray(0, at * nb * 4)); q.set(S.take.q.subarray(a * nb * 4, (b + 1) * nb * 4), at * nb * 4); q.set(S.take.q.subarray(at * nb * 4), (at + k) * nb * 4);
  p.set(S.take.p.subarray(0, at * nb * 3)); p.set(S.take.p.subarray(a * nb * 3, (b + 1) * nb * 3), at * nb * 3); p.set(S.take.p.subarray(at * nb * 3), (at + k) * nb * 3);
  S.take.q = q; S.take.p = p; S.take.n = n + k;
  const mapF = f => f < at ? f : f + k;
  S.take.shots = S.take.shots.map(sh => sh.start >= at ? { ...sh, start: sh.start + k } : sh.start + sh.n > at ? { ...sh, n: sh.n + k } : sh);
  const remap = x => x.a == null ? x : { ...x, a: mapF(x.a), b: x.b >= at && x.a < at ? x.b + k : mapF(x.b) };
  S.fixes = S.fixes.map(x => x.type === 'join' || x.type === 'story' ? { ...x, at: mapF(x.at) } : remap(x));
  S.locks = S.locks.map(remap); S.pins = S.pins.map(remap);
  for (const [i, keys] of S.layer) S.layer.set(i, keys.map(kk => ({ ...kk, f: mapF(kk.f) })));
  S.animLayers = S.animLayers.map(L => ({ ...L, bones: Object.fromEntries(Object.entries(L.bones || {}).map(([bn, ks]) => [bn, ks.map(kk => ({ ...kk, f: mapF(kk.f) }))])) }));
  S.sel = null;
  return [at, at + k];
}
function cutTake(a, b) {
  const nb = S.body.bones.length, n = S.take.n, k = b - a + 1;
  if (a < 1 || b >= n - 1 || k < 1) throw new Error('cut must leave frames on both sides');
  ensureSrc(); S.srcIdx = cutIndex(S.srcIdx, a, b);
  const q = new Float32Array((n - k) * nb * 4), p = new Float32Array((n - k) * nb * 3);
  q.set(S.take.q.subarray(0, a * nb * 4)); q.set(S.take.q.subarray((b + 1) * nb * 4), a * nb * 4);
  p.set(S.take.p.subarray(0, a * nb * 3)); p.set(S.take.p.subarray((b + 1) * nb * 3), a * nb * 3);
  S.take.q = q; S.take.p = p; S.take.n = n - k;
  const mapF = f => f < a ? f : f > b ? f - k : null;
  S.take.shots = S.take.shots.map(sh => { const s0 = sh.start, s1 = sh.start + sh.n - 1, ns = mapF(s0) ?? a, ne = mapF(s1) ?? a - 1; return { ...sh, start: ns, n: Math.max(0, ne - ns + 1) }; }).filter(sh => sh.n > 0);
  const remap = x => { if (x.a == null) return x; const na = mapF(x.a) ?? a, nb2 = mapF(x.b) ?? a - 1; return nb2 < na ? null : { ...x, a: na, b: nb2 }; };
  S.fixes = S.fixes.map(x => x.type === 'join' || x.type === 'story' ? { ...x, at: mapF(x.at) ?? a } : remap(x)).filter(Boolean);
  S.locks = S.locks.map(remap).filter(Boolean);
  S.pins = S.pins.map(remap).filter(Boolean);
  for (const [i, keys] of S.layer) S.layer.set(i, keys.map(kk => ({ ...kk, f: mapF(kk.f) })).filter(kk => kk.f != null));
  S.animLayers = S.animLayers.map(L => ({ ...L, bones: Object.fromEntries(Object.entries(L.bones || {}).map(([bn, ks]) => [bn, ks.map(kk => ({ ...kk, f: mapF(kk.f) })).filter(kk => kk.f != null)])) }));
  S.frame = Math.min(S.frame, S.take.n - 1); S.sel = null;
  return a;
}

function rebuild() {
  if (!S.body || !S.take) return;
  let guidedReference=null;
  const localGuided=[...S.fixes.filter(x=>x.type==='inbetween'||x.type==='secondary'),...S.animLayers.filter(L=>L.guidedRange&&!L.mute).map(L=>({a:L.guidedRange[0],b:L.guidedRange[1]}))];
  if(localGuided.length&&!S.guidedReferenceBuild){
    S.guidedReferenceBuild=true;
    try{rebuild();guidedReference=S.out;}finally{S.guidedReferenceBuild=false;}
  }
  const t0 = performance.now();
  if (!S.fixes.some(x => x.type === 'lean') || S.leanW?.w.length !== S.take.n) S.leanW = null;
  compileAnimLayers();
  S.base = buildBase();
  const nb = S.body.bones.length, n = S.take.n;
  const seamFx = S.fixes.find(x => x.type === 'seams');
  let seamOff = null;
  S.seamPlan = null;
  if (seamFx && S.take.shots.length > 1) {
    const p0 = runPass(S.base, null, { layer: true, ik: false });
    const pel = firstIdx('pelvis');
    const plan = seamPlan({ W: p0.W, nb, shots: S.take.shots, pelvis: pel, thighL: idx('thigh_l'), thighR: idx('thigh_r'), blend: seamFx.blend ?? 6, force: !!seamFx.force });
    S.seamPlan = plan;
    if (plan.some(x => x.needed)) seamOff = seamRootOffsets(plan, n, { W: p0.W, nb, pelvis: pel, mode: seamFx.mode || 'carry' });
    if (seamOff && !seamOff.some(Boolean)) seamOff = null;
  }
  const withYaw = arr => seamOff ? arr.map((o, f) => { const y = seamOff[f]?.[3]; return o ? (y ? [o[0], o[1], o[2], y] : o) : (y ? [0, 0, 0, y] : null); }) : arr;
  const p1 = runPass(S.base, seamOff, { layer: true, ik: false });
  S.comRaw = comSeries(p1.W);
  const ct = contactsFromWorld(p1.W);
  const g = 9.8 * S.upm, fps = S.take.fps;
  const flights = S.fixes.filter(x => x.type === 'ballistic').map(x => ({ a: x.a, b: x.b }));
  S.flightFits = flights.map(fl => { const fit = fitBallistic(S.comRaw, fl.a, fl.b, fps, g); const tApex = fit.v0[1] / g; return { ...fl, fit, report: flightReport(fit, S.upm), pastApex: (fl.b - fl.a) / fps > tApex }; });
  const bal = flights.length ? ballisticOffsets(S.comRaw, flights, fps, g, 'endpoints') : null;
  const ful = S.fixes.filter(x => x.type === 'fulcrum').map(x => fulcrumOffsets(p1.W, S.body.bones.length, x.idx, x.a, x.b, 'auto', 3));
  const extra = combineOffsets(momentumOffsets(S.comRaw, ct.any), bal, floorOffsets(p1.W, ct.ground), travelOffsets(p1.W), joinOffsets(p1.W), ...ful);
  S.leanW = leanPlan(p1.W);
  const off = withYaw(combineOffsets(seamOff, extra).map(o => o || null));
  S.rootOff = off;
  const p2 = extra.some(Boolean) ? runPass(S.base, off, { layer: true, ik: false }) : p1;
  lockTargets(p2.W);
  const ct2 = contactsFromWorld(p2.W);
  const gp = groundPlan(p2.W, ct2);
  S.groundTargets = gp.targets;
  for (const [f, ts] of stridePlan(p1.W, p2.W)) { const own = ts.filter(t => t.own), have = (S.groundTargets.get(f) || []).filter(h => !own.some(t => t.side === h.side)), keep = ts.filter(t => t.own || !have.some(h => h.side === t.side)); if (keep.length || have.length) S.groundTargets.set(f, [...have, ...keep]); }
  S.rollTargets = rollPlan(p2.W);
  S.levelFeet = levelPlan(ct2.ground, p2.W);
  const pulls = pullPlan(p2.W, ct2);
  const offIk = (gp.drop.some(Boolean) || pulls.length) ? withYaw(combineOffsets(seamOff, extra, gp.drop, ...pulls).map(o => o || null)) : off;
  S.floorTargets = new Map();
  S.floorFrames = 0;
  const needIk = S.fixes.some(x => x.type === 'carry'||x.contactTargets?.length) || S.groundTargets.size || S.levelFeet || S.locks.length || S.pins.length || gp.targets.size || S.rollTargets.size || pulls.length;
  S.out = needIk ? runPass(S.base, offIk, { layer: true, ik: true }) : p2;
  const ff = S.fixes.filter(x => x.type === 'floor');
  if (ff.length) {
    const legs = { l: { thigh: idx('thigh_l'), calf: idx('calf_l'), foot: idx('foot_l'), ball: idx('ball_l') }, r: { thigh: idx('thigh_r'), calf: idx('calf_r'), foot: idx('foot_r'), ball: idx('ball_r') } };
    const rest = S.body.restW.map(m => m.toArray());
    const ground = ct2.ground, seen = new Set();
    for (const fx of ff) {
      const Lg = fx.side ? { [fx.side]: legs[fx.side] } : legs;
      const a = fx.whole ? 0 : fx.a, b = fx.whole ? n - 1 : fx.b;
      const m = floorTargets({ W: S.out.W, nb, n, upm: S.upm, ground, legs: Lg, frames: [a, b], stiffness: fx.stiffness ?? 0, pivot: fx.pivot ?? 'ankle', rest });
      if (fx.whole) {
        const plants = stancePlants(S.out.W, ground);
        for (const [f, ts] of [...m]) {
          const keep = ts.filter(t => plants.some(p => p.side === t.side && f >= p.a && f <= p.b));
          if (keep.length) m.set(f, keep); else m.delete(f);
        }
      }
      fx.hits = m.size;
      for (const [f, ts] of m) for (const t of ts) {
        const k = f + t.side;
        if (seen.has(k)) continue;
        seen.add(k);
        S.floorTargets.set(f, [...(S.floorTargets.get(f) || []), t]);
      }
    }
    for (const side of ['l', 'r']) {
      const bi = firstIdx('ball_' + side);
      if (bi === undefined) continue;
      const Wd = S.out.W, bp = f => [Wd[(f * nb + bi) * 16 + 12], Wd[(f * nb + bi) * 16 + 14]];
      const planted = f => (S.floorTargets.get(f) || []).find(t => t.side === side && !t.swing);
      for (let f = 0; f < n; f++) {
        if (!planted(f)) continue;
        let e = f;
        while (e + 1 < n && planted(e + 1)) e++;
        if (e - f + 1 >= 3) {
          let anchor = f, slow = Infinity;
          for (let g = f; g <= e; g++) { const a = bp(Math.max(f, g - 1)), c = bp(Math.min(e, g + 1)), v = Math.hypot(c[0] - a[0], c[1] - a[1]); if (v < slow) { slow = v; anchor = g; } }
          const hold = bp(anchor);
          for (let g = f; g <= e; g++) { const t = planted(g), c = bp(g); t.ankleP = [t.ankleP[0] + hold[0] - c[0], t.ankleP[1], t.ankleP[2] + hold[1] - c[1]]; }
        }
        f = e;
      }
    }
    for (const side of ['l', 'r']) {
      const on = f => (S.floorTargets.get(f) || []).some(t => t.side === side);
      const was = Array.from({ length: n }, (_, f) => on(f));
      for (let f = 0; f < n; f++) {
        if (was[f]) continue;
        let near = Infinity, src = null;
        for (let k = 1; k <= FLOOR_EASE; k++) for (const g of [f - k, f + k]) if (near === Infinity && g >= 0 && g < n && was[g]) { near = k; src = (S.floorTargets.get(g) || []).find(x => x.side === side); }
        if (src) S.floorTargets.set(f, [...(S.floorTargets.get(f) || []), { ...src, w: smooth(1 - near / (FLOOR_EASE + 1)) }]);
      }
    }
    S.floorFrames = S.floorTargets.size;
    if (S.floorFrames) S.out = runPass(S.base, offIk, { layer: true, ik: true });
  }
  if(S.guidedReferenceBuild)return;
  if(guidedReference){
    for(let f=0;f<n;f++)if(!localGuided.some(x=>f>x.a&&f<x.b)){
      S.out.q.set(guidedReference.q.subarray(f*nb*4,(f+1)*nb*4),f*nb*4);
      S.out.p.set(guidedReference.p.subarray(f*nb*3,(f+1)*nb*3),f*nb*3);
      S.out.W.set(guidedReference.W.subarray(f*nb*16,(f+1)*nb*16),f*nb*16);
    }
  }
  S.com = comSeries(S.out.W);
  S.contact = contactsFromWorld(S.out.W);
  S.ground = S.contact.ground;
  S.motionQuality = inspectMotion({W:S.out.W,nb,n,fps:S.take.fps,upm:S.upm,ground:S.ground,names:S.body.bones.map(b=>b.name),parents:S.body.parentIdx,com:S.com,feet:{l:idx('foot_l'),r:idx('foot_r')},locks:[...S.locks,...S.fixes.flatMap(x=>x.contactTargets||[])],pins:S.pins});
  analyze();
  gate();
  buildComPath();
  themeColors();
  renderLists();
  showFrame(S.frame);
  $('hud').dataset.ms = Math.round(performance.now() - t0);
  recordEdit();
}

function analyze() {
  const n = S.take.n, fps = S.take.fps, nb = S.body.bones.length, g = 9.81 * S.upm, mu = +$('mu').value || 0.8;
  const c = S.com, ct = S.contact, flags = [];
  const sm = new Float32Array(n * 3);
  for (let f = 0; f < n; f++) for (let k = 0; k < 3; k++) {
    let s = 0, w = 0;
    for (let d = -2; d <= 2; d++) { const g2 = Math.min(n - 1, Math.max(0, f + d)); s += c[g2 * 3 + k]; w++; }
    sm[f * 3 + k] = s / w;
  }
  const acc = f => {
    const a = Math.max(0, f - 1), b = Math.min(n - 1, f + 1);
    return [0, 1, 2].map(k => (sm[b * 3 + k] - 2 * sm[f * 3 + k] + sm[a * 3 + k]) * fps * fps);
  };
  const major = ['pelvis', 'spine_01', 'spine_02', 'spine_03', 'spine_04', 'spine_05', 'neck_01', 'head', 'clavicle_l', 'clavicle_r', 'upperarm_l', 'upperarm_r', 'lowerarm_l', 'lowerarm_r', 'hand_l', 'hand_r', 'thigh_l', 'thigh_r', 'calf_l', 'calf_r', 'foot_l', 'foot_r'].map(idx).filter(v => v !== undefined);
  const snapLim = +$('snapLim').value || 900;
  const per = { overload: new Uint8Array(n), slide: new Uint8Array(n), float: new Uint8Array(n), snap: new Uint8Array(n) };
  let air = 0;
  for (let f = 0; f < n; f++) {
    const a = acc(f);
    const ah = Math.hypot(a[0], a[2]);
    if (ct.any[f] && ah > mu * g) per.overload[f] = 1;
    for (const s of ['l', 'r']) if (ct.per[s][f] && ct.speed[s][f] > 0.3) per.slide[f] = 1;
    air = ct.any[f] ? 0 : air + 1;
    if (air >= 4 && a[1] > -0.35 * g) per.float[f] = 1;
    if (f > 0) {
      let mx = 0;
      for (const i of major) {
        qa.fromArray(S.out.q, (f * nb + i) * 4); qb.fromArray(S.out.q, ((f - 1) * nb + i) * 4);
        const ang = 2 * Math.acos(Math.min(1, Math.abs(qa.dot(qb)))) * 180 / Math.PI * fps;
        if (ang > mx) mx = ang;
      }
      if (mx > snapLim) per.snap[f] = 1;
    }
  }
  const label = { overload: 'Momentum change too fast for foot friction', slide: 'Planted foot sliding', float: 'Airborne but not falling', snap: 'Pose snaps between frames' };
  for (const k of Object.keys(per)) {
    let s = -1;
    for (let f = 0; f <= n; f++) {
      const on = f < n && per[k][f];
      if (on && s < 0) s = f;
      if (!on && s >= 0) {
        if (f - s >= (k === 'snap' ? 1 : 2)) flags.push({ kind: k, a: s, b: f - 1, text: label[k] });
        s = -1;
      }
    }
  }
  const merged = [];
  for (const fl of flags.sort((x, y) => x.a - y.a)) {
    const prev = merged.findLast(m => m.kind === fl.kind);
    if (prev && fl.a - prev.b <= 3) prev.b = fl.b; else merged.push(fl);
  }
  S.flags = merged;
}

function gate() {
  const n = S.take.n, nb = S.body.bones.length, lim = +$('gateLim').value || 8;
  const rows = [];
  const driven = new Set();
  for (const l of S.locks) for (const k of LEGS[l.side]) driven.add(idx(k));
  if (S.pins.length) for (const k of ARM_L) driven.add(idx(k));
  for (const g of S.fixes) if ((g.type === 'ground' || g.type === 'roll') && g.side) for (const k of LEGS[g.side]) driven.add(idx(k));
  for (const ts of (S.floorTargets || new Map()).values()) for (const t of ts) for (const k of LEGS[t.side]) driven.add(idx(k));
  for (const ts of (S.floorTargets || new Map()).values()) for (const t of ts) if (t.ballQ) driven.add(firstIdx('ball_' + t.side));
  if (S.levelFeet) for (const sd of S.levelFeet.sides) { driven.add(sd.fi); driven.add(sd.bi); }
  if (S.leanW) { const i = firstIdx('spine_01', 'spine_02'); if (i !== undefined) driven.add(i); }
  if (S.fixes.some(x => x.type === 'carry')) for (const k of ['upperarm_l', 'lowerarm_l', 'hand_l', 'upperarm_r', 'lowerarm_r', 'hand_r']) { const i = idx(k); if (i !== undefined) driven.add(i); }
  for (const ts of (S.groundTargets || new Map()).values()) for (const t of ts) for (const k of LEGS[t.side]) driven.add(idx(k));
  const joinMask = new Uint8Array(n), joins = S.fixes.filter(x => x.type === 'join');
  for (const fx of S.fixes) if (fx.type === 'smooth' && joins.some(j => j.at >= fx.a && j.at <= fx.b)) for (let f = fx.a; f <= fx.b; f++) joinMask[f] = 1;
  for (const fx of S.fixes) if (fx.type === 'story') { const w = storyWindow(fx, n); for (let f = w.s; f <= w.e; f++) joinMask[f] = 1; }
  for (let i = 0; i < nb; i++) {
    const k = keyOf(S.body.bones[i].name);
    if (/(thumb|index|middle|ring|pinky|metacarpal|twist|corrective|ik_|weapon|attach)/.test(k)) continue;
    const a = [];
    for (let f = 0; f < n; f++) {
      if (joinMask[f]) continue;
      qa.fromArray(S.out.q, (f * nb + i) * 4); qb.fromArray(S.take.q, (f * nb + i) * 4);
      a.push(2 * Math.acos(Math.min(1, Math.abs(qa.dot(qb)))) * 180 / Math.PI);
    }
    const s = a.sort((x, y) => x - y), m = s.length;
    rows.push({ bone: S.body.bones[i].name, p95: s[Math.floor(m * 0.95)] || 0, max: s[m - 1] || 0, edited: S.layer.has(i) || !!S.animCompiled?.has(i), driven: driven.has(i) });
  }
  rows.sort((x, y) => y.p95 - x.p95);
  const pel = idx('pelvis');
  let pmax = 0;
  if (pel !== undefined) for (let f = 0; f < n; f++) {
    const o = (f * nb + pel) * 3;
    pmax = Math.max(pmax, Math.hypot(S.out.p[o] - S.take.p[o], S.out.p[o + 1] - S.take.p[o + 1], S.out.p[o + 2] - S.take.p[o + 2]));
  }
  const free = rows.filter(r => !r.edited && !r.driven);
  const worst = free[0]?.p95 || 0;
  S.gate = { rows, free, lim, pass: worst <= lim, worst, worstBone: free[0]?.bone, pelvisCm: pmax / S.upm * 100 };
}

function flightText(x) {
  const f = (S.flightFits || []).find(y => y.a === x.a && y.b === x.b);
  if (!f) return '';
  const r = f.report;
  return `${r.airtime_s.toFixed(2)} s, apex ${r.apex_m_above_takeoff.toFixed(2)} m, ${r.takeoff_speed_ms.toFixed(1)} m/s${f.pastApex ? '' : ' (ends before apex)'}`;
}

function fixLabel(x, i) {
  const r = (S.cleanReport || []).find(y => y.fix === i);
  const cr = r ? ` (${r.frames} frames / ${r.bones} bones changed, max ${r.maxDeg.toFixed(1)}°${x.type === 'despike' ? `, ${r.spikes} pops removed, ${r.rejected} kept as real motion` : ''})` : '';
  if (x.type === 'despike') return 'Remove pops' + cr;
  if (x.type === 'story') { const r = (S.storyReport || []).find(y => y.at === x.at); return `Story blend at f${x.at} (${x.curve ?? 'ease'}, ${x.pre ?? x.blend ?? 6}+${x.post ?? x.blend ?? 6} f${r ? `, outgoing from source f${r.outgoingFrom}, match ${(Math.hypot(...r.matchOffset) / S.upm * 100).toFixed(1)} cm` : S.srcTake ? '' : ', no source frames: re-open the project'})`; }
  if (x.type === 'travel' && x.footsteps) { const pl = (S.footstepPlans || []).find(y => y.a === x.a && y.b === x.b); return `Biped footsteps ${x.speed ?? 3} m/s${pl ? ` (${pl.footprints.length} footprints, cycle ${pl.cycle} f, contact ${pl.contactFrames} f)` : ''}`; }
  if (x.type === 'filter') return `Jitter filter ${x.fc} Hz` + cr;
  if (x.type === 'median') return 'Median smooth' + cr;
  if (x.type === 'inbetween') return `Pose-guided inbetween (${Math.round((x.strength ?? 1) * 100)}%)`;
  if (x.type === 'secondary') return `Secondary motion (${Math.round((x.strength ?? 0.35) * 100)}%, damping ${x.damping ?? 0.72})`;
  if (x.type === 'roll') return 'Foot roll ' + (x.side === 'l' ? 'left' : 'right');
  if (x.type === 'floor') return `Floor contact${x.whole ? ', whole take' : ''}${x.side ? ' ' + x.side : ''} (${x.hits || 0} frames lifted)`;
  if (x.type === 'seams') return 'Match shot seams: ' + ((S.seamPlan || []).map(p => p.needed ? `${p.shot} move ${(Math.hypot(...p.offset.T) / S.upm * 100).toFixed(1)} cm, yaw ${(p.offset.yawRad * 180 / Math.PI).toFixed(1)}°` : `${p.shot} already lines up`).join('; ') || 'single shot');
  if (x.type === 'pull') return `Pull body to hand (pull ${x.pull}, gravity ${x.gravity}, ${S.pins.length ? (x.hits || 0) + ' frames moved' : 'needs a hand pin'})`;
  return 'Smoothing';
}

function syncToggles() {
  $('floorOn').checked = S.fixes.some(x => x.type === 'floor' && x.whole);
  $('levelOn').checked = S.fixes.some(x => x.type === 'level');
  $('seamOn').checked = S.fixes.some(x => x.type === 'seams');
  const pl = S.fixes.find(x => x.type === 'pull');
  $('pullOn').checked = !!pl;
  if (pl) { $('pullAmt').value = pl.pull; $('pullGravity').value = pl.gravity; }
}

function renderLists() {
  syncToggles();
  const fr = (a, b) => a === b ? `f${a}` : `f${a}–${b}`;
  $('lockList').innerHTML = S.locks.map((l, i) => `<li><span class="sev" style="background:var(--lock)"></span><span class="grow" role="button" tabindex="0" data-jump="${l.a}">${l.side === 'l' ? 'Left' : 'Right'} foot ${fr(l.a, l.b)}</span><button aria-label="Remove lock ${i+1}" data-del-lock="${i}">×</button></li>`).join('');
  $('pinList').innerHTML = S.pins.map((p, i) => `<li><span class="sev" style="background:var(--pin)"></span><span class="grow" role="button" tabindex="0" data-jump="${p.a}">${p.kind === 'weapon' ? 'Two-hand weapon grip' : 'Grip'} from f${p.ref}, held ${fr(p.a, p.b)}</span><button aria-label="Remove grip pin ${i+1}" data-del-pin="${i}">×</button></li>`).join('');
  const sevCol = { overload: 'var(--bad)', slide: 'var(--warn)', float: 'var(--warn)', snap: 'var(--bad)' };
  const fixes = S.fixes.map((x, i) => `<li><span class="sev" style="background:var(--accent)"></span><span class="grow" role="button" tabindex="0" data-jump="${x.a}">${x.type === 'momentum' ? 'Momentum fix' : x.type === 'ballistic' ? 'Ballistic ' + flightText(x) : x.type === 'fulcrum' ? 'Fulcrum ' + x.pivot + ' (slip was ' + x.slip.toFixed(1) + ' cm)' : x.type === 'ground' ? 'Ground ' + (x.side === 'l' ? 'left' : x.side === 'r' ? 'right' : '') + ' foot' : fixLabel(x, i)} ${x.a === undefined ? '' : fr(x.a, x.b)}</span><button aria-label="Remove fix ${i+1}" data-del-fix="${i}">×</button></li>`).join('');
  $('floorInfo').textContent = S.floorFrames ? `Floor contact lifted a foot on ${S.floorFrames} frames.` : (S.fixes.some(x => x.type === 'floor') ? 'Floor contact on: no sole marker below the floor.' : '');
  $('plantInfo').innerHTML = (S.slidingPlants || []).map(p => `<li><span class="sev" style="background:var(--warn)"></span><span class="grow" role="button" tabindex="0" data-sel="${p.a},${p.b}">${p.side === 'l' ? 'Left' : 'Right'} ${p.touch ? 'sliding touch' : 'sliding plant'} (${p.point}), not locked</span><span>${fr(p.a, p.b)}</span></li>`).join('');
  $('flagList').innerHTML = fixes + (S.flags.length ? S.flags.map(fl => `<li><span class="sev" style="background:${sevCol[fl.kind]}"></span><span class="grow" role="button" tabindex="0" data-sel="${fl.a},${fl.b}">${fl.text}</span><span>${fr(fl.a, fl.b)}</span></li>`).join('') : '<li>No physics problems found.</li>');
  const G = S.gate;
  $('gate').innerHTML = `<div class="verdict ${G.pass ? 'pass' : 'fail'}">${G.pass ? 'PASS' : 'FAIL'} · untouched joints p95 ≤ ${G.worst.toFixed(1)}° (limit ${G.lim}°)${G.worstBone ? ' · worst ' + G.worstBone : ''} · pelvis moved ≤ ${G.pelvisCm.toFixed(1)} cm</div><p class="note">Joints you keyed (✎) or that a foot lock or hand pin drives (⚙) are meant to move and are listed but not gated.</p>
  <table><tr><th>joint</th><th>p95 °</th><th>max °</th></tr>${G.rows.slice(0, 8).map(r => `<tr><td>${r.bone}${r.edited ? ' ✎' : ''}${r.driven ? ' ⚙' : ''}</td><td class="${r.p95 > G.lim ? 'fail' : ''}">${r.p95.toFixed(1)}</td><td>${r.max.toFixed(1)}</td></tr>`).join('')}</table>`;
  drawTL();
}

document.addEventListener('click', e => {
  const t = e.target.closest('[data-jump],[data-sel],[data-del-lock],[data-del-pin],[data-del-fix]');
  if (!t) return;
  if (t.dataset.jump) showFrame(+t.dataset.jump);
  if (t.dataset.sel) { const [a, b] = t.dataset.sel.split(',').map(Number); S.sel = [Math.max(0, a - 6), Math.min(S.take.n - 1, b + 6)]; showFrame(a); }
  if (t.dataset.delLock) { S.locks.splice(+t.dataset.delLock, 1); rebuild(); }
  if (t.dataset.delPin) { S.pins.splice(+t.dataset.delPin, 1); rebuild(); }
  if (t.dataset.delFix) { S.fixes.splice(+t.dataset.delFix, 1); rebuild(); }
});

function buildComPath() {
  if (comPath) overlay.remove(comPath);
  const pts = [];
  for (let f = 0; f < S.take.n; f++) pts.push(new THREE.Vector3(S.com[f * 3], S.ground + 0.2, S.com[f * 3 + 2]));
  comPath = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ transparent: true, opacity: 0.6 }));
  overlay.add(comPath);
}

function showFrame(f) {
  if (!S.take || !S.out) return;
  S.frame = Math.max(0, Math.min(S.take.n - 1, f | 0));
  const B = S.body, nb = B.bones.length;
  for (let i = 0; i < nb; i++) {
    const display=S.compareSource?S.take:S.out;
    B.bones[i].quaternion.fromArray(display.q, (S.frame * nb + i) * 4);
    B.bones[i].position.fromArray(display.p, (S.frame * nb + i) * 3);
  }
  B.obj.updateMatrixWorld(true);
  updateProps();
  jointMeshes.forEach((m, i) => m.position.setFromMatrixPosition(B.bones[i].matrixWorld));
  const c = new THREE.Vector3().fromArray(S.com, S.frame * 3);
  comMarker.visible = true; comDrop.visible = true;
  comMarker.position.copy(c);
  comDrop.geometry.setFromPoints([c, new THREE.Vector3(c.x, S.ground, c.z)]);
  lockMarks.clear(); pinMarks.clear();
  if ($('showMarkers').checked) {
    if (!S.markerRig || S.markerRig.out !== S.out) S.markerRig = { out: S.out, rig: soleMarkerRig({ W: S.out.W, nb, n: S.take.n, upm: S.upm, ground: S.ground, legs: { l: { foot: idx('foot_l'), ball: firstIdx('ball_l') }, r: { foot: idx('foot_r'), ball: firstIdx('ball_r') } } }) };
    const cs = +$('contactSize').value || 100, sc = contactGizmoScale(cs)[0];
    for (const pts of Object.values(soleMarkers(S.markerRig.rig, S.out.W, nb, S.frame, cs))) for (const pt of Object.values(pts)) {
      const m = new THREE.Mesh(new THREE.SphereGeometry(S.upm * 0.008 * sc, 8, 6), new THREE.MeshBasicMaterial({ color: css('--lock'), depthTest: false, transparent: true }));
      m.renderOrder = 12; m.position.fromArray(pt); lockMarks.add(m);
    }
  }
  for (const l of S.locks) if (S.frame >= l.a && S.frame <= l.b && l.target) {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(S.upm * 0.06, S.upm * 0.06, S.upm * 0.005, 20), new THREE.MeshBasicMaterial({ color: css('--lock'), transparent: true, opacity: 0.7 }));
    m.position.set(l.target[0], S.ground + 0.5, l.target[2]); lockMarks.add(m);
  }
  const hl = idx('hand_l');
  for (const p of S.pins) if (S.frame >= p.a && S.frame <= p.b && hl !== undefined) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(S.upm * 0.025, 12, 8), new THREE.MeshBasicMaterial({ color: css('--pin'), depthTest: false, transparent: true }));
    m.renderOrder = 12;
    m.position.setFromMatrixPosition(B.bones[hl].matrixWorld); pinMarks.add(m);
  }
  const fps = S.take.fps;
  const shot = S.take.shots.findLast(s => S.frame >= s.start);
  const flags = S.flags.filter(x => S.frame >= x.a && S.frame <= x.b).map(x => x.text);
  const cts = ['l', 'r'].filter(s => S.contact.per[s][S.frame]).map(s => s === 'l' ? 'L' : 'R').join('+') || 'none';
  const fl = (S.floorTargets?.get(S.frame) || []).map(t => t.side.toUpperCase()).join('+');
  $('hud').textContent = `${shot ? shot.name : ''}\ncontact ${cts}${fl ? ' · floor lift ' + fl : ''}${S.floorFrames ? '\nfloor contact ' + S.floorFrames + ' f' : ''}${flags.length ? '\n⚠ ' + flags.join('\n⚠ ') : ''}`;
  $('frameLbl').textContent = `frame ${S.frame} / ${S.take.n - 1} · ${(S.frame / fps).toFixed(2)} s · ${fps} fps`;
  $('selLbl').textContent = S.sel ? `selection f${S.sel[0]}–${S.sel[1]}` : 'shift-drag to select frames';
  if (S.selBone >= 0 && !gizmo.dragging) gizmo.attach(B.bones[S.selBone]);
  drawTL();
  GE.ed?.setFrame(S.frame);
}

const ray = new THREE.Raycaster(), mouse = new THREE.Vector2();
let downAt = null;
renderer.domElement.addEventListener('pointerdown', e => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', e => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4 || gizmo.dragging || !S.body) return;
  const r = renderer.domElement.getBoundingClientRect();
  mouse.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
  ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObjects(jointMeshes.filter(m => m.visible))[0];
  if (hit) selectBone(hit.object.userData.bone);
  else if (!gizmo.axis) selectBone(-1);
});
function selectBone(i) {
  S.selBone = i;
  if (i >= 0) { const control=S.animatorControls.find(c=>c.id===S.controlId&&c.bone===i)||S.animatorControls.find(c=>c.bone===i); if(control){S.controlId=control.id;$('rigControl').value=control.id;renderControlMode();} }
  if (i < 0) { gizmo.detach(); $('selBone').textContent = 'no joint'; drawTL(); geRefresh(); return; }
  gizmo.attach(S.body.bones[i]);
  $('selBone').textContent = S.body.bones[i].name + (S.layer.has(i) ? ` · ${S.layer.get(i).length} key(s)` : '');
  drawTL();
  geRefresh();
}
function renderAnimatorControls() {
  const picker=$('rigControl');
  picker.replaceChildren(...S.animatorControls.map(c=>new Option(c.label,c.id)));
  S.controlId=S.animatorControls[0]?.id||null;
  if(S.controlId)picker.value=S.controlId;
  renderControlMode();
}
function renderControlMode() {
  const control=S.animatorControls.find(c=>c.id===S.controlId), mode=$('rigMode');
  mode.querySelector('option[value="ik"]').disabled=!control?.chain;
  if(!control?.chain)mode.value='fk';
  const ik=mode.value==='ik'&&!!control?.chain;
  for(const id of ['rigX','rigY','rigZ','rigPoleX','rigPoleY','rigPoleZ','rigBlend','rigApplyIk'])$(id).disabled=!ik;
  $('rigInfo').textContent=control ? `${control.label} · ${S.body.bones[control.bone].name}${control.chain?' · FK/IK, stable pole':' · FK'}` : 'Load a character to inspect rig controls.';
}
$('rigControl').onchange=e=>{S.controlId=e.target.value;const c=S.animatorControls.find(x=>x.id===S.controlId);if(c)selectBone(c.bone);renderControlMode();};
$('rigMode').onchange=()=>{for(const id of ['rigX','rigY','rigZ','rigPoleX','rigPoleY','rigPoleZ'])$(id).value=0;renderControlMode();};
$('rigKeyFk').onclick=()=>{try{const c=S.animatorControls.find(x=>x.id===S.controlId);if(!c)throw new Error('Select a rig control first.');selectBone(c.bone);geKeyBone(c.bone,S.frame);$('rigInfo').textContent=`FK key set on ${c.label} at f${S.frame}.`;}catch(e){$('rigInfo').textContent=e.message;}};
$('rigApplyIk').onclick=()=>{
  try{
    const c=S.animatorControls.find(x=>x.id===S.controlId);
    if(!S.out||!c?.chain||$('rigMode').value!=='ik')throw new Error('Select a hand or foot in IK mode.');
    const offset=['rigX','rigY','rigZ'].map(id=>Number($(id).value)),poleOffset=['rigPoleX','rigPoleY','rigPoleZ'].map(id=>Number($(id).value)),blend=Number($('rigBlend').value);
    if(![...offset,...poleOffset,blend].every(Number.isFinite)||blend<0||blend>1)throw new Error('Use finite offsets and blend between 0 and 1.');
    if(!blend||![...offset,...poleOffset].some(v=>Math.abs(v)>1e-9))throw new Error('Move the IK target or pole before keying.');
    const before=editSnapshot(),[ia,ib,ic]=c.chain,nb=S.body.bones.length,f=S.frame,B=S.body.bones;
    showFrame(f);
    const world=i=>new THREE.Vector3().setFromMatrixPosition(B[i].matrixWorld),root=world(ia),mid=world(ib),end=world(ic);
    const goal=new THREE.Vector3().fromArray(ikGoalFromOffset(end.toArray(),offset,S.upm));
    const stable=new THREE.Vector3().fromArray(stablePole(root.toArray(),mid.toArray(),end.toArray(),{height:1.8*S.upm}).poleVector);
    stable.add(new THREE.Vector3().fromArray(poleOffset).multiplyScalar(S.upm/100));
    twoBoneIK(ia,ib,ic,goal,null,blend,null,stable);
    let changed=0;
    for(const i of c.chain){
      const q0=new THREE.Quaternion().fromArray(S.out.q,(f*nb+i)*4),delta=q0.invert().multiply(B[i].quaternion).normalize();
      if(delta.angleTo(new THREE.Quaternion())<1e-5)continue;
      const keys=S.layer.get(i)||[],existing=keys.find(k=>k.f===f);
      if(existing)existing.q=delta.toArray();else keys.push({f,q:delta.toArray()});
      keys.sort((a,b)=>a.f-b.f);S.layer.set(i,keys);changed++;
    }
    if(!changed){restoreEdit(before);throw new Error('IK target produced no joint rotation.');}
    rebuild();selectBone(c.bone);recordEdit(before);
    $('rigInfo').textContent=`${c.label} IK key at f${f}: ${changed} chain joints, blend ${blend.toFixed(2)}. FK/IK switching leaves the pose unchanged.`;
  }catch(e){$('rigInfo').textContent='IK key failed: '+e.message;}
};
function commitGizmo() {
  const i = S.selBone; if (i < 0 || !S.base) return;
  const before=EDITS.gizmoBefore;EDITS.gizmoBefore=null;
  const nb = S.body.bones.length, f = S.frame;
  qa.fromArray(S.base.q, (f * nb + i) * 4);
  if (S.activeLayer >= 0 && S.animLayers[S.activeLayer]) {
    const below = qa.clone().multiply(layerDelta(i, f, new THREE.Quaternion())).toArray();
    const e = solveLayerKey(S.animLayers, S.animCompiled?.get(i), S.activeLayer, below, S.body.bones[i].quaternion.toArray(), f);
    S.animLayers[S.activeLayer] = setLayerKey(normalizeLayer(S.animLayers[S.activeLayer]), S.body.bones[i].name, f, e);
    rebuild(); selectBone(i); renderAnimLayers();recordEdit(before);
    return;
  }
  const d = qa.clone().invert().multiply(S.body.bones[i].quaternion);
  const keys = S.layer.get(i) || [];
  const k = keys.find(k => k.f === f);
  if (k) k.q = d.toArray(); else keys.push({ f, q: d.toArray() });
  keys.sort((x, y) => x.f - y.f);
  S.layer.set(i, keys);
  rebuild();
  selectBone(i);
  recordEdit(before);
}
$('delKey').onclick = () => {
  const before=editSnapshot();
  const keys = S.layer.get(S.selBone); if (!keys) return;
  const r = keys.filter(k => k.f !== S.frame);
  r.length ? S.layer.set(S.selBone, r) : S.layer.delete(S.selBone);
  rebuild(); selectBone(S.selBone);recordEdit(before);
};
$('clearBone').onclick = () => { const before=editSnapshot();S.layer.delete(S.selBone); rebuild(); selectBone(S.selBone);recordEdit(before); };
$('falloff').onchange = rebuild;

function renderAnimLayers() {
  const sel = $('animActive');
  sel.innerHTML = '<option value="-1">pose layer</option>' + S.animLayers.map((L, i) => `<option value="${i}">${L.name || 'Layer ' + (i + 1)} · ${Object.values(L.bones || {}).reduce((m, k) => m + k.length, 0)} key(s)</option>`).join('');
  if (S.activeLayer >= S.animLayers.length) S.activeLayer = -1;
  sel.value = String(S.activeLayer);
  const L = S.animLayers[S.activeLayer] && normalizeLayer(S.animLayers[S.activeLayer]);
  for (const id of ['animWeight', 'animMute', 'animOverride', 'animAccum', 'animTangent', 'animDel']) $(id).disabled = !L;
  geRefresh();
  if (!L) return;
  $('animWeight').value = L.weight; $('animMute').checked = L.mute; $('animOverride').checked = L.override; $('animAccum').value = L.accumulation; $('animTangent').value = L.tangent;
  $('animMask').value = L.mask?.regions?.[0] || 'all';
}
const editLayer = patch => { if (S.activeLayer < 0) return; S.animLayers[S.activeLayer] = { ...S.animLayers[S.activeLayer], ...patch }; rebuild(); renderAnimLayers(); };
$('animActive').onchange = e => { S.activeLayer = +e.target.value; renderAnimLayers(); };
$('animAdd').onclick = () => { S.animLayers.push(normalizeLayer({ name: 'Layer ' + (S.animLayers.length + 1) })); S.activeLayer = S.animLayers.length - 1; renderAnimLayers(); };
$('animDel').onclick = () => { if (S.activeLayer < 0) return; S.animLayers.splice(S.activeLayer, 1); S.activeLayer = -1; rebuild(); renderAnimLayers(); };
$('animWeight').onchange = e => editLayer({ weight: Math.min(1, Math.max(0, +e.target.value || 0)) });
$('animMute').onchange = e => editLayer({ mute: e.target.checked });
$('animOverride').onchange = e => editLayer({ override: e.target.checked });
$('animAccum').onchange = e => editLayer({ accumulation: e.target.value });
$('animTangent').onchange = e => editLayer({ tangent: e.target.value });
$('animMask').onchange = e => editLayer({ mask: e.target.value === 'all' ? null : { regions: [e.target.value] } });

const GE = { ed: null, open: false };
function geBones() {
  const L = S.animLayers[S.activeLayer];
  if (!L) return [];
  if (S.selBone >= 0) return [S.body.bones[S.selBone].name];
  return Object.keys(L.bones || {});
}
function geCurves() {
  const L = S.animLayers[S.activeLayer];
  return L ? layerToCurves(normalizeLayer(L), geBones()) : [];
}
function geSetCurves(curves) {
  const L = S.animLayers[S.activeLayer];
  if (!L) return;
  const shown = new Set(geBones()), keep = Object.fromEntries(Object.entries(L.bones || {}).filter(([b]) => !shown.has(b)));
  const next = curvesToLayer({ ...L, bones: { ...(L.bones || {}) } }, curves, S.take?.fps ?? 24);
  S.animLayers[S.activeLayer] = { ...next, bones: { ...next.bones, ...keep } };
  rebuild();
  const sel = $('animActive'), A = S.animLayers[S.activeLayer];
  if (sel.options[S.activeLayer + 1]) sel.options[S.activeLayer + 1].textContent = `${A.name || 'Layer ' + (S.activeLayer + 1)} · ${Object.values(A.bones || {}).reduce((m, k) => m + k.length, 0)} key(s)`;
}
function geRefresh() {
  if (!GE.ed) return;
  GE.ed.refresh();
  const L = S.animLayers[S.activeLayer];
  $('geInfo').textContent = !L ? 'Pick or add an animation layer, or select a joint and press Key joint.' : `${L.name || 'Layer ' + (S.activeLayer + 1)} · ${S.selBone >= 0 ? S.body.bones[S.selBone].name : 'all keyed bones'}`;
}
function geOpen(on = !GE.open) {
  GE.open = on;
  $('gePanel').hidden = !on;
  $('geToggle').classList.toggle('on', on);
  if (on && !GE.ed) GE.ed = createGraphEditor($('geHost'), { getCurves: geCurves, setCurves: geSetCurves, onScrub: f => showFrame(f), fps: () => S.take?.fps ?? 24, frameCount: () => S.take?.n ?? 0, currentFrame: () => S.frame });
  geRefresh();
  requestAnimationFrame(resize);
}
function geKeyBone(i = S.selBone, f = S.frame) {
  if (i < 0 || !S.base) throw new Error('select a joint first');
  if (S.activeLayer < 0 || !S.animLayers[S.activeLayer]) { S.animLayers.push(normalizeLayer({ name: 'Layer ' + (S.animLayers.length + 1) })); S.activeLayer = S.animLayers.length - 1; }
  const nb = S.body.bones.length, below = new THREE.Quaternion().fromArray(S.base.q, (f * nb + i) * 4).multiply(layerDelta(i, f, new THREE.Quaternion())).toArray();
  const cur = new THREE.Quaternion().fromArray(S.out.q, (f * nb + i) * 4).toArray();
  const e = solveLayerKey(S.animLayers, S.animCompiled?.get(i), S.activeLayer, below, cur, f);
  const nm = S.body.bones[i].name;
  let L = setLayerKey(normalizeLayer(S.animLayers[S.activeLayer]), nm, f, e);
  if (i === idx('pelvis')) L = { ...L, bones: { ...L.bones, [nm]: L.bones[nm].map(k => k.p ? k : { ...k, p: [0, 0, 0] }) } };
  S.animLayers[S.activeLayer] = L;
  rebuild(); renderAnimLayers();
  return { layer: S.activeLayer, bone: nm, f, e };
}
$('geToggle').onclick = () => geOpen();
$('geKey').onclick = () => { try { geKeyBone(); } catch (err) { status(err.message); } };
{
  let drag = null;
  $('geGrip').addEventListener('pointerdown', e => { drag = { y: e.clientY, h: $('geHost').getBoundingClientRect().height }; $('geGrip').setPointerCapture(e.pointerId); });
  $('geGrip').addEventListener('pointermove', e => { if (!drag) return; $('geHost').style.height = Math.max(160, Math.min(innerHeight * 0.7, drag.h + drag.y - e.clientY)) + 'px'; });
  $('geGrip').addEventListener('pointerup', () => { drag = null; requestAnimationFrame(resize); });
}
renderAnimLayers();

function storyOpts() { return { story: $('storyOn').checked, curve: $('storyCurve').value, blend: Math.max(1, +$('storyBlend').value || 6) }; }
function doCut(a, b, o = {}) {
  const blend = o.blend ?? 6, j = cutTake(a, b);
  S.cuts = [...(S.cuts || []), { a, b }];
  if (o.story) S.fixes.push({ type: 'story', at: j, pre: blend, post: blend, curve: o.curve ?? 'ease', match: o.match ?? 'gravity' });
  else S.fixes.push({ type: 'join', at: j }, { type: 'smooth', a: Math.max(0, j - blend), b: Math.min(S.take.n - 1, j + blend) });
  rebuild();
  return j;
}
function doInsert(at, a, b, o = {}) {
  const blend = o.blend ?? 6, [j1, j2] = insertTake(at, a, b);
  S.cuts = [...(S.cuts || []), { op: 'insert', at, a, b }];
  if (o.story) S.fixes.push({ type: 'story', at: j1, pre: blend, post: blend, curve: o.curve ?? 'ease', match: o.match ?? 'gravity' }, { type: 'story', at: j2, pre: blend, post: blend, curve: o.curve ?? 'ease', match: o.match ?? 'gravity' });
  else S.fixes.push({ type: 'join', at: j1 }, { type: 'join', at: j2 }, { type: 'smooth', a: Math.max(0, j1 - blend), b: j1 + blend }, { type: 'smooth', a: j2 - blend, b: Math.min(S.take.n - 1, j2 + blend) });
  rebuild();
  return [j1, j2];
}
$('cutSel').onclick = () => { if (!S.take || !S.sel) return status('Select the frames to cut first.'); try { const j = doCut(S.sel[0], S.sel[1], storyOpts()); status(`Cut; the take now joins at f${j}.`); } catch (err) { status(err.message); } };
$('insertSel').onclick = () => { if (!S.take || !S.sel) return status('Select the frames to copy first.'); try { const [j1, j2] = doInsert(S.frame, S.sel[0], S.sel[1], storyOpts()); status(`Inserted f${j1}–${j2 - 1}.`); } catch (err) { status(err.message); } };
$('footstepsSel').onclick = () => {
  if (!S.take || !S.sel) return status('Select the run to drive with footsteps first.');
  S.fixes.push({ type: 'travel', a: S.sel[0], b: S.sel[1], speed: +$('fsSpeed').value || 4, ease: 8, footsteps: true, balance: +$('fsBalance').value || 0 });
  rebuild();
};
for (const id of ['contactSize', 'showMarkers']) $(id).onchange = () => showFrame(S.frame);

function selRange() { return S.sel ? [...S.sel] : [0, S.take.n - 1]; }
function addLock(side) { if (!S.take) return; const [a, b] = selRange(); S.locks.push({ side, a, b }); rebuild(); }
$('lockL').onclick = () => addLock('l');
$('lockR').onclick = () => addLock('r');
$('autoLocks').onclick = () => {
  if (!S.take) return;
  const inSelLock = l => S.sel && l.b >= S.sel[0] && l.a <= S.sel[1];
  S.locks = S.sel ? S.locks.filter(l => !inSelLock(l)) : [];
  S.fixes = S.fixes.filter(x => x.type !== 'roll' || S.locks.some(l => l.side === x.side && l.b >= x.a && l.a <= x.b));
  const plants = detectPlantsNow();
  const inSel = p => !S.sel || (p.b >= S.sel[0] && p.a <= S.sel[1]);
  const keep = plants.filter(p => !p.sliding && inSel(p));
  S.slidingPlants = plants.filter(p => p.sliding && inSel(p));
  for (const p of keep) S.locks.push({ side: p.side, a: p.a, b: p.b });
  rebuild();
  status(`Auto-detected ${keep.length} foot plants (${S.slidingPlants.length} sliding contacts listed, not locked).`);
};
function detectPlantsNow() { return plantsOf(runPass(buildBase(), S.rootOff, { layer: true, ik: false }).W); }
function plantsOf(W) {
  const nb = S.body.bones.length, ct = contactsFromWorld(W);
  const R = S.body.restW, d = (i, j) => new THREE.Vector3().setFromMatrixPosition(R[i]).distanceTo(new THREE.Vector3().setFromMatrixPosition(R[j])) / S.upm * 100;
  const legLength_cm = d(idx('thigh_l'), idx('calf_l')) + d(idx('calf_l'), idx('foot_l'));
  const footLength_cm = idx('ball_l') !== undefined ? Math.max(20, 1.6 * d(idx('foot_l'), idx('ball_l'))) : 26;
  const legs = { l: { foot: idx('foot_l'), ball: firstIdx('ball_l', 'foot_l') }, r: { foot: idx('foot_r'), ball: firstIdx('ball_r', 'foot_r') } };
  return detectFootPlants({ W, nb, n: S.take.n, fps: S.take.fps, upm: S.upm, ground: ct.ground, legs, legLength_cm, footLength_cm });
}
$('footRoll').onclick = () => {
  if (!S.take) return;
  const [a, b] = selRange();
  let added = 0;
  for (const l of S.locks) {
    if (l.b < a || l.a > b) continue;
    const ra = Math.max(a, l.a), rb = Math.min(b, l.b);
    if (rb - ra < 3 || S.fixes.some(x => x.type === 'roll' && x.side === l.side && x.a === ra && x.b === rb)) continue;
    S.fixes.push({ type: 'roll', side: l.side, a: ra, b: rb, start: 30, end: 60 });
    added++;
  }
  if (!added) return status('Foot roll needs a locked foot in the selection. Lock or auto-detect plants first.');
  rebuild();
  status(`Foot roll added on ${added} planted range(s).`);
};
const toggleFix = (type, on, make) => { S.fixes = S.fixes.filter(x => !(x.type === type && (type !== 'floor' || x.whole))); if (on) S.fixes.push(make()); rebuild(); };
$('levelOn').onchange = e => toggleFix('level', e.target.checked, () => ({ type: 'level', side: null }));
$('floorOn').onchange = e => toggleFix('floor', e.target.checked, () => ({ type: 'floor', whole: true, side: null, stiffness: 0, pivot: 'ankle' }));
$('floorRange').onclick = () => { if (!S.take) return; const [a, b] = selRange(); S.fixes.push({ type: 'floor', a, b, side: null, stiffness: 0, pivot: 'ankle' }); rebuild(); };
$('seamOn').onchange = e => toggleFix('seams', e.target.checked, () => ({ type: 'seams', blend: 6, mode: 'carry', force: false }));
const pullFix = () => ({ type: 'pull', pull: Number.isFinite(+$('pullAmt').value) && $('pullAmt').value !== '' ? Math.max(0, +$('pullAmt').value) : 1, gravity: Math.min(1, Math.max(0, +$('pullGravity').value || 0)), iterations: 10 });
$('pullOn').onchange = e => toggleFix('pull', e.target.checked, pullFix);
for (const id of ['pullAmt', 'pullGravity']) $(id).onchange = () => { if ($('pullOn').checked) toggleFix('pull', true, pullFix); };
$('despikeRange').onclick = () => { if (!S.take) return; const [a, b] = selRange(); S.fixes.push({ type: 'despike', a, b, bones: 'all' }); rebuild(); };
$('filterRange').onclick = () => { if (!S.take) return; const [a, b] = selRange(); S.fixes.push({ type: 'filter', a, b, fc: +$('cleanFc').value || 7 }); rebuild(); };
$('medianRange').onclick = () => { if (!S.take) return; const [a, b] = selRange(); S.fixes.push({ type: 'median', a, b, window: 5 }); rebuild(); };
$('pinHand').onclick = () => {
  if (!S.out) return;
  const hl = idx('hand_l'), hr = idx('hand_r');
  if (hl === undefined || hr === undefined) return status('This skeleton has no hand_l / hand_r.');
  const nb = S.body.bones.length, f = S.frame;
  const L = new THREE.Matrix4().fromArray(S.out.W, (f * nb + hl) * 16), R = new THREE.Matrix4().fromArray(S.out.W, (f * nb + hr) * 16);
  const off = R.invert().multiply(L);
  const [a, b] = selRange();
  S.pins.push({ a, b, ref: f, off: off.toArray() });
  rebuild();
  status(`Left hand pinned to its frame-${f} grip over f${a}–${b}.`);
};
$('pinWorld').onclick = () => {
  if (!S.out) return;
  const hl = idx('hand_l');
  if (hl === undefined) return status('This skeleton has no hand_l.');
  const nb = S.body.bones.length, f = S.frame, [a, b] = selRange();
  S.pins.push({ a, b, ref: f, world: Array.from(S.out.W.subarray((f * nb + hl) * 16, (f * nb + hl) * 16 + 16)) });
  rebuild();
  status(`Left hand pinned in the world where it is at frame ${f}, over f${a}–${b}. Pull body to hand now has a fixed goal.`);
};
$('unpinHand').onclick = () => { S.pins = []; rebuild(); };
function editSnapshot(){
  return {project:JSON.parse(JSON.stringify(projectJSON())),take:S.take?{...S.take,q:S.take.q.slice(),p:S.take.p.slice(),shots:S.take.shots.map(x=>({...x}))}:null,srcTake:S.srcTake,srcIdx:S.srcIdx?.slice(),frame:S.frame,sel:S.sel?.slice(),body:S.body?{...S.body,poseQ:S.body.bones.map(b=>b.quaternion.toArray()),poseP:S.body.bones.map(b=>b.position.toArray())}:null};
}
function restoreEdit(row){
  EDITS.restoring=true;
  const P=row.project; S.take=row.take;S.srcTake=row.srcTake;S.srcIdx=row.srcIdx;S.frame=row.frame;S.sel=row.sel;
  S.sourceShots=structuredClone(P.sourceShots||[]);
  S.cuts=structuredClone(P.cuts||[]);S.retimes=structuredClone(P.retimes||[]);
  if(row.body&&S.body){S.body.bones.forEach((b,i)=>{b.quaternion.fromArray(row.body.poseQ[i]);b.position.fromArray(row.body.poseP[i]);});S.body.obj.updateMatrixWorld(true);}
  S.fixes=structuredClone(P.fixes||[]);S.pins=structuredClone(P.pins||[]);S.locks=structuredClone(P.locks||[]);S.cuts=structuredClone(P.cuts||[]);S.props=structuredClone(P.props||[]);S.bookmarks=structuredClone(P.bookmarks||[]);S.animLayers=structuredClone(P.animLayers||[]);S.layer.clear();
  for(const L of P.layer||[]){const i=S.body.bones.findIndex(b=>b.name===L.bone);if(i>=0)S.layer.set(i,structuredClone(L.keys));}
  $('falloff').value=P.falloff;$('mu').value=P.mu;
  try{rebuild();renderAnimLayers();}finally{EDITS.restoring=false;}
}
function recordEdit(previous=null){
  if(EDITS.restoring||EDITS.preview||S.autoFixing||!S.take)return;
  const key=JSON.stringify(projectJSON());
  if(EDITS.rows[EDITS.cursor]?.key===key)return;
  if(previous&&!EDITS.rows.length){EDITS.rows.push({...previous,key:JSON.stringify(previous.project)});EDITS.cursor=0;}
  EDITS.rows.splice(EDITS.cursor+1);EDITS.rows.push({...editSnapshot(),key});
  if(EDITS.rows.length>20)EDITS.rows.shift();EDITS.cursor=EDITS.rows.length-1;
  try{localStorage.setItem('fulcrum.recovery.v2',JSON.stringify({savedAt:new Date().toISOString(),project:projectJSON()}));}catch{}
  $('poseBookmarks').replaceChildren(...S.bookmarks.map((p,i)=>new Option(p.name,String(i))));
}
function beginPreview(label){
  if(EDITS.preview)throw new Error('Accept or reject the current preview first.');
  EDITS.preview={before:editSnapshot(),label};
}
function acceptPreview(){if(!EDITS.preview)return;const preview=EDITS.preview,label=preview.label;EDITS.preview=null;recordEdit(preview.before);$('guidedLog').textContent=label+' accepted. Undo is available.';}
function rejectPreview(){if(!EDITS.preview)return;const before=EDITS.preview.before;EDITS.preview=null;restoreEdit(before);$('guidedLog').textContent='Preview rejected.';}
function posePreviewLayer(name,pose,names,frame,preserve=[]){
  const bones={},falloff=Math.max(1,+$('falloff').value||12),nb=S.body.bones.length;
  names.forEach((bone,k)=>{
    const i=S.body.byKey.get(keyOf(bone));if(i===undefined||preserve.includes(i))return;
    const current=new THREE.Quaternion().fromArray(S.out.q,(frame*nb+i)*4),target=new THREE.Quaternion().fromArray(pose,k*4),delta=current.invert().multiply(target),e=new THREE.Euler().setFromQuaternion(delta);
    const value=[e.x,e.y,e.z].map(v=>v*180/Math.PI),rows=new Map([[0,[0,0,0]],[Math.max(0,frame-falloff),[0,0,0]],[Math.min(S.take.n-1,frame+falloff),[0,0,0]],[S.take.n-1,[0,0,0]],[frame,value]]);
    bones[bone]=[...rows].sort((a,b)=>a[0]-b[0]).map(([f,e])=>({f,e,tangent:'flat'}));
  });
  return normalizeLayer({name,weight:1,override:false,accumulation:'layer',bones,guidedRange:[Math.max(0,frame-falloff),Math.min(S.take.n-1,frame+falloff)]});
}
let learnedPoseModel = null;
fetch('/exports/frt_pose_model_h16-e150.json').then(r => r.ok ? r.json() : null).then(j => { if (j && j.validated) learnedPoseModel = parsePoseModel(j); }).catch(() => {});
function pelvisTrack() {
  const nb = S.body.bones.length, pi = S.body.bones.findIndex(b => b.name === 'pelvis'), n = S.take.n, out = new Float32Array(n * 3);
  for (let f = 0; f < n; f++) out.set(S.take.p.subarray((f * nb + pi) * 3, (f * nb + pi) * 3 + 3), f * 3);
  return out;
}
function learnedSuggestion() {
  if (!learnedPoseModel || !S.take || !S.out) return null;
  const pose = predictNextPose(learnedPoseModel, { p: pelvisTrack(), q: S.out.q, nb: S.body.bones.length, n: S.take.n, f: S.frame, fps: S.take.fps });
  return { pose, sourceFrame: S.frame, guidanceErrorDeg: 0, uncertaintyDeg: 0, confidence: 0, preserved: [], source: 'learned' };
}
function poseSuggestion(index=0){
  if(!S.out)return;
  if(!S.poseSuggestions){
    const guides=[...S.layer.keys()];if(S.selBone>=0)guides.push(S.selBone);
    const preserve=S.pins.length?['hand_l','hand_r'].map(idx).filter(i=>i!==undefined):[];
    S.poseSuggestions=suggestPoses({source:S.take.q,current:S.out.q,nb:S.body.bones.length,n:S.take.n,f:S.frame,guides:[...new Set(guides)],preserve});S.poseSuggestionFrame=S.frame;
  }
  const learned=learnedSuggestion();const rows=learned?[learned,...S.poseSuggestions.suggestions]:S.poseSuggestions.suggestions;if(!rows.length){$('guidedLog').textContent=S.poseSuggestions.reason;return;}
  if(EDITS.preview)rejectPreview();
  beginPreview('Pose suggestion');S.poseSuggestionIndex=index%rows.length;
  const row=rows[S.poseSuggestionIndex];
  S.animLayers.push(posePreviewLayer('Motion pose suggestion',row.pose,S.body.bones.map(b=>b.name),S.poseSuggestionFrame,row.preserved));rebuild();renderAnimLayers();
  $('guidedLog').textContent=`Preview ${S.poseSuggestionIndex+1}/${rows.length}, learned from ${S.poseSuggestions.trainingFrames} frames of this take. Guidance error ${row.guidanceErrorDeg.toFixed(1)}°, confidence ${(row.confidence*100).toFixed(0)}%. Accept or reject. This uses a motion-library statistical prior.`;
}

function setSimpleLayerMask(layer, mask) {
  return { ...layer, mask: mask === 'all' ? null : { regions: [mask] } };
}

function retimeRaw(a,b,scale) {
  if (!S.take) throw new Error('Load a take before retiming.');
  const nb=S.body.bones.length,oldN=S.take.n,oldSrcIdx=S.srcIdx;
  const tracks=retimeTracks({q:S.take.q,p:S.take.p,nb,n:oldN,a,b,scale}),mapF=f=>retimeMapFrame(f,a,b,scale);
  S.take={...S.take,n:tracks.n,q:tracks.q,p:tracks.p,shots:S.take.shots.map(s=>{const start=mapF(s.start),end=mapF(s.start+s.n-1);return {...s,start,n:end-start+1};})};
  if(oldSrcIdx){const mapped=new Int32Array(tracks.n);for(let f=0;f<tracks.n;f++){let sf=f;if(f>=a&&f<=tracks.range[1])sf=a+(f-a)*(b-a)/(tracks.range[1]-a);else if(f>tracks.range[1])sf=f-tracks.delta;const lo=Math.max(0,Math.min(oldN-1,Math.floor(sf))),hi=Math.min(oldN-1,lo+1),w=sf-lo;mapped[f]=Math.round(oldSrcIdx[lo]*(1-w)+oldSrcIdx[hi]*w);}S.srcIdx=mapped;}
  return {oldFrames:oldN,newFrames:tracks.n,scale,range:tracks.range};
}

function retimeSelection(scale) {
  if (!S.take || !S.sel) throw new Error('Select a range to retime.');
  const before=editSnapshot();
  const [a,b]=S.sel,raw=retimeRaw(a,b,scale),mapF=f=>retimeMapFrame(f,a,b,scale);
  S.fixes=S.fixes.map(x=>({...x,...(Number.isInteger(x.a)?{a:mapF(x.a)}:{}),...(Number.isInteger(x.b)?{b:mapF(x.b)}:{}),...(Number.isInteger(x.at)?{at:mapF(x.at)}:{})}));
  S.locks=S.locks.map(x=>({...x,a:mapF(x.a),b:mapF(x.b)}));
  S.pins=S.pins.map(x=>({...x,a:mapF(x.a),b:mapF(x.b),ref:mapF(x.ref)}));
  S.layer=new Map([...S.layer].map(([i,ks])=>[i,ks.map(k=>({...k,f:mapF(k.f)}))]));
  S.animLayers=S.animLayers.map(L=>({...L,guidedRange:L.guidedRange?[mapF(L.guidedRange[0]),mapF(L.guidedRange[1])]:undefined,bones:Object.fromEntries(Object.entries(L.bones||{}).map(([name,ks])=>[name,ks.map(k=>({...k,f:mapF(k.f)}))]))}));
  S.frame=mapF(S.frame);S.sel=raw.range;S.retimes.push({a,b,scale});rebuild();renderShots();recordEdit(before);
  return {...raw,range:S.sel};
}

async function cascadeurPosePreview(settings) {
  if(!S.out||S.selBone==null||S.selBone<0)throw new Error('Select a joint that you have explicitly moved to guide posing.');
  if(!S.take||!S.sel)throw new Error('Select a frame range containing the guided pose.');
  const canonical=await(await fetch('/models/cascadeur_canonical.json')).json(),rig=buildControlRig({THREE,body:S.body,canonical,upm:S.upm});
  const nb=S.body.bones.length,selectedOff=(S.frame*nb+S.selBone)*4,moved=new THREE.Quaternion().fromArray(S.base.q,selectedOff).angleTo(new THREE.Quaternion().fromArray(S.out.q,selectedOff));
  if(moved<0.5*Math.PI/180)throw new Error('Move the selected joint first; pose inference uses that edit as guidance.');
  const guidePoints=rig.controls.map((c,i)=>c.bone===S.selBone?i:-1).filter(i=>i>=0);
  if(!guidePoints.length)throw new Error('The selected joint is not represented by the canonical43 guide rig.');
  const [a,b]=S.sel,frames=Array.from({length:b-a+1},(_,i)=>a+i);
  if(frames.length>256)throw new Error('Cascadeur posing preview is limited to 256 frames; select a smaller local range.');
  const nodes=frames.map(f=>sampleControlRig(rig,S.out.W,f));
  const groundCm=modelGroundY(S.ground,S.upm),normals=frames.map(()=>[[0,1,0],[0,1,0],[0,1,0],[0,1,0]]),planes=frames.map(()=>[[0,groundCm,0],[0,groundCm,0],[0,groundCm,0],[0,groundCm,0]]);
  const guidance=guidanceForFrames({frames,controls:rig.controls,selectedBone:S.selBone,locks:S.locks,pins:S.pins,footBones:{l:idx('foot_l'),r:idx('foot_r')},handBones:{l:idx('hand_l'),r:idx('hand_r')}});
  const request={kind:'pose',mode:'pose',nodes,tposes:canonical.positions,snap_p:settings.snap_p,spine_p:[settings.spine],nodes_s:guidance.masks,dc_s:frames.map(()=>[[1],[1],[1]]),plane_normals:normals,plane_positions:planes};
  $('guidedLog').textContent='Running the recovered Cascadeur posing network on CPU with explicit settings. The native editor defaults are not known.';
  const response=await fetch('/api/cascadeur-model',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(request)}),result=await response.json();
  if(!response.ok||!result.ok)throw new Error(result.error||'Cascadeur pose inference failed.');
  const predicted=result.outputs?.nodes;
  if(!Array.isArray(predicted)||predicted.length!==frames.length||predicted.some(x=>x.length!==43||x.some(p=>p.length!==3||p.some(v=>!Number.isFinite(v)))))throw new Error('Cascadeur model returned invalid guided pose output.');
  const out=preserveGuidedPoints(predicted,nodes,guidance.masks);
  const baselineQ=S.out.q.slice(),baselineP=S.out.p.slice(),baselineW=S.out.W.slice();
  const retargeted=controlMotionToLocalQuaternions({rig,positions:out,sourceQ:baselineQ,sourceP:baselineP,sourceW:baselineW,firstFrame:a,preserveRoot:true});
  const names=S.body.bones.map(x=>x.name),bones={},frame=S.frame;
  for(let bi=0;bi<S.body.bones.length;bi++){
    const rows=[];let changed=false;
    for(let k=0;k<frames.length;k++){
      const f=frames[k];if(f===a||f===b||guidance.preservedBones[k].has(bi))continue;
      const off=(f*S.body.bones.length+bi)*4,src=new THREE.Quaternion().fromArray(baselineQ,off),dst=new THREE.Quaternion().fromArray(retargeted.q,off),d=src.invert().multiply(dst).normalize();
      if(d.angleTo(new THREE.Quaternion())<1e-5)continue;
      const eu=new THREE.Euler().setFromQuaternion(d);rows.push({f,e:[eu.x,eu.y,eu.z].map(x=>x*180/Math.PI),tangent:'linear'});changed=true;
    }
    if(changed)bones[names[bi]]=rows;
  }
  if(!Object.keys(bones).length)throw new Error('Pose network made no editable changes on this range.');
  const baselineFailures=S.motionQuality?.failedConstraints||0;
  beginPreview('Cascadeur neural pose suggestion');
  S.animLayers.push(normalizeLayer({name:'Cascadeur neural pose suggestion',weight:1,override:false,accumulation:'layer',bones,guidedRange:[a,b],guidedModel:'posing-model-0',guidedSettings:{snap_p:settings.snap_p,spine:settings.spine},guidedSettingsSource:'explicit Fulcrum preview settings; native defaults not recovered'}));
  rebuild();renderAnimLayers();
  if((S.motionQuality?.failedConstraints||0)>baselineFailures){rejectPreview();throw new Error('Pose suggestion increased required contact failures; it was reverted.');}
  $('guidedLog').textContent=`Cascadeur model preview: ${Object.keys(bones).length} editable joints; selected guide and active foot/hand contacts held by Fulcrum. Model-space ground ${groundCm.toFixed(2)}; settings are explicit Fulcrum values, not recovered Cascadeur defaults.`;
}
$('cascadeurPose').onclick=async()=>{
  try{
    const snap=$('poseSnap').value.split(',').map(x=>Number(x.trim()));
    if(snap.length!==6||snap.some(x=>!Number.isFinite(x)))throw new Error('Enter six finite snap parameters separated by commas.');
    await cascadeurPosePreview({snap_p:snap,spine:Number($('poseSpine').value)});
  }catch(e){$('guidedLog').textContent='Cascadeur pose preview unavailable: '+e.message;}
};
$('retimeSel').onclick=()=>{try{const r=retimeSelection(Number($('retimeScale').value));$('guidedLog').textContent=`Retimed f${S.sel[0]}–${S.sel[1]} by ${r.scale}× (${r.oldFrames} → ${r.newFrames} frames). Existing keys, locks, pins, and fixes were remapped.`;}catch(e){$('guidedLog').textContent='Retiming failed: '+e.message;}};
async function neuralInbetween(style=0){
  if(!S.take||!S.sel||S.sel[1]-S.sel[0]<2)throw new Error('Select at least three frames bounded by clean key poses.');
  const [a,b]=S.sel,nb=S.body.bones.length,canonical=await(await fetch('/models/cascadeur_canonical.json')).json();
  const rig=buildControlRig({THREE,body:S.body,canonical,upm:S.upm}),frames=[...new Set([Math.max(0,a-1),...Array.from({length:b-a+1},(_,i)=>a+i),Math.min(S.take.n-1,b+1)])].sort((x,y)=>x-y);
  if(frames.length<4)throw new Error('Neural inbetween needs four distinct context/key frames.');
  const positions=frames.map(f=>sampleControlRig(rig,S.out.W,f));
  const response=await fetch('/api/cascadeur-model',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind:'inbetween',positions,frames,tpose:canonical.positions,style,height:182.49})});
  const result=await response.json();if(!response.ok||!result.ok)throw new Error(result.error||'Recovered model inference failed.');
  if(result.first_frame!==frames[1]||result.last_frame!==frames.at(-2)||result.positions.length!==result.last_frame-result.first_frame+1)throw new Error('Recovered model returned an unexpected frame span.');
  const baselineQ=S.out.q.slice(),baselineP=S.out.p.slice(),baselineW=S.out.W.slice(),retargeted=controlMotionToLocalQuaternions({rig,positions:result.positions,sourceQ:baselineQ,sourceP:baselineP,sourceW:baselineW,firstFrame:result.first_frame,preserveRoot:$('guidedRoot').checked});
  const names=S.body.bones.map(x=>x.name),bones={},falloff=Math.max(1,+$('falloff').value||12),n=S.take.n;
  const rootBi=S.body.parentIdx.findIndex(x=>x<0),keepRoot=$('guidedRoot').checked;
  for(let bi=0;bi<nb;bi++){
    const rows=[];let changed=false;
    for(let f=0;f<n;f++){
      let e=[0,0,0],translation=[0,0,0];
      if(f>=result.first_frame&&f<=result.last_frame&&f>a&&f<b){
        const off=(f*nb+bi)*4,src=new THREE.Quaternion().fromArray(baselineQ,off),dst=new THREE.Quaternion().fromArray(retargeted.q,off),d=src.invert().multiply(dst).normalize();
        if(d.angleTo(new THREE.Quaternion())>1e-5){const eu=new THREE.Euler().setFromQuaternion(d);e=[eu.x*180/Math.PI,eu.y*180/Math.PI,eu.z*180/Math.PI];changed=true;}
        if(!keepRoot&&bi===rootBi){const po=(f*nb+bi)*3;translation=[0,1,2].map(k=>retargeted.p[po+k]-baselineP[po+k]);if(Math.hypot(...translation)>1e-6)changed=true;}
      }
      if(f===0||f===n-1||f===a||f===b||f===Math.max(0,a-falloff)||f===Math.min(n-1,b+falloff))rows.push({f,e,...(bi===rootBi?{p:translation}:{}),tangent:'flat'});
    }
    if(changed){
      const map=new Map(rows.map(x=>[x.f,x]));
      for(let f=a+1;f<b;f++){const off=(f*nb+bi)*4,src=new THREE.Quaternion().fromArray(baselineQ,off),dst=new THREE.Quaternion().fromArray(retargeted.q,off),d=src.invert().multiply(dst).normalize(),eu=new THREE.Euler().setFromQuaternion(d),row={f,e:[eu.x*180/Math.PI,eu.y*180/Math.PI,eu.z*180/Math.PI],tangent:'linear'};if(!keepRoot&&bi===rootBi){const po=(f*nb+bi)*3;row.p=[0,1,2].map(k=>retargeted.p[po+k]-baselineP[po+k]);}if(d.angleTo(new THREE.Quaternion())>1e-5||row.p&&Math.hypot(...row.p)>1e-6)map.set(f,row);}
      bones[names[bi]]=[...map.values()].sort((x,y)=>x.f-y.f);
    }
  }
  const preserve=[...new Set(S.pins.flatMap(p=>[p.driver,p.driven].filter(Boolean)))];
  beginPreview('Recovered Cascadeur neural inbetween');
  S.animLayers.push(normalizeLayer({name:'Recovered Cascadeur neural inbetween',weight:Math.max(0,Math.min(1,+$('inbetweenStrength').value||0)),override:false,accumulation:'layer',bones,guidedRange:[a,b]}));
  S.fixes.push({type:'inbetween',a,b,strength:0,preserve,preserveRoot:keepRoot,spacing:'model',model:'casc_generator1234',style,contactTargets:S.locks.filter(l=>l.b>=a&&l.a<=b).map(l=>({side:l.side,a:Math.max(l.a,a),b:Math.min(l.b,b),target:l.target.slice(),targetQ:l.targetQ.slice(),required:true}))});
  rebuild();renderAnimLayers();
  $('guidedLog').textContent=`Recovered Cascadeur MibVelocityLoop preview: ${result.positions.length} frames, CPU inference ${result.seconds.toFixed(2)} s, explicit style label ${style} (label semantics unknown). Retargeted with Fulcrum's canonical43 control rig; inspect and validate before accepting.`;
}
function updateProps(){
  const byId=new Map(S.props.map(x=>[x.id,x]));
  for(const obj of [...propGroup.children])if(!byId.has(obj.userData.propId)){propGroup.remove(obj);obj.traverse(x=>{x.geometry?.dispose();if(x.material&&!Array.isArray(x.material))x.material.dispose();});}
  for(const prop of S.props){
    let obj=propGroup.children.find(x=>x.userData.propId===prop.id);
    if(!obj){obj=new THREE.Mesh(new THREE.BoxGeometry(prop.length*S.upm,.06*S.upm,.10*S.upm),new THREE.MeshStandardMaterial({color:0x596348,roughness:.8}));obj.userData.propId=prop.id;obj.matrixAutoUpdate=false;propGroup.add(obj);}
    const driver=idx(prop.driver||'hand_r');if(driver===undefined){obj.visible=false;continue;}obj.visible=true;obj.matrix.copy(S.body.bones[driver].matrixWorld).multiply(new THREE.Matrix4().fromArray(prop.offset));obj.updateMatrixWorld(true);
  }
}
$('guidedAccept').onclick=acceptPreview;$('guidedReject').onclick=rejectPreview;
$('editUndo').onclick=()=>{if(EDITS.preview)return rejectPreview();if(EDITS.cursor>0)restoreEdit(EDITS.rows[--EDITS.cursor]);};
$('editRedo').onclick=()=>{if(!EDITS.preview&&EDITS.cursor+1<EDITS.rows.length)restoreEdit(EDITS.rows[++EDITS.cursor]);};
$('recoverEdit').onclick=async()=>{const saved=initialRecovery||JSON.parse(localStorage.getItem('fulcrum.recovery.v2')||'null');if(saved){await applyProject(saved.project);$('guidedLog').textContent='Recovered session from '+saved.savedAt;}};
$('compareSource').onchange=e=>{S.compareSource=e.target.checked;showFrame(S.frame);};
$('bookmarkPose').onclick=()=>{if(!S.out)return;const before=editSnapshot(),nb=S.body.bones.length;S.bookmarks.push({name:'Pose f'+S.frame,names:S.body.bones.map(b=>b.name),q:Array.from(S.out.q.subarray(S.frame*nb*4,(S.frame+1)*nb*4))});recordEdit(before);};
$('restorePose').onclick=()=>{const p=S.bookmarks[+$('poseBookmarks').value];if(!p)return;beginPreview('Bookmarked pose');S.animLayers.push(posePreviewLayer(p.name,p.q,p.names,S.frame));rebuild();};
$('nextPose').onclick=()=>poseSuggestion((S.poseSuggestionIndex||0)+1);
$('propIn').onchange=async e=>{
  const file=e.target.files[0];if(!file||!S.body)return;
  const bytes=await file.arrayBuffer(),obj=parseFBX(bytes),box=new THREE.Box3().setFromObject(obj),size=box.getSize(new THREE.Vector3()),length=+$('propLength').value||.8,scale=length*S.upm/Math.max(size.x,size.y,size.z,1e-6);
  const saved=await saveFile('prop_'+file.name,bytes,'application/octet-stream');
  const id=crypto.randomUUID(),prop={id,name:file.name,driver:'hand_r',length,offset:new THREE.Matrix4().identity().toArray(),source:'prop_'+file.name,scale,mass:3.5};S.props.push(prop);obj.scale.multiplyScalar(scale);obj.position.set(0,0,0);const group=new THREE.Group();group.add(obj);group.matrixAutoUpdate=false;group.userData.propId=id;propGroup.add(group);rebuild();$('guidedLog').textContent='Attached '+file.name+' to the right hand. Capture the two-hand grip at the desired pose.';e.target.value='';
};
$('inbetweenSel').onclick=async()=>{try{await neuralInbetween(+$('neuralStyle').value||0);}catch(e){$('guidedLog').textContent='Recovered neural inbetween unavailable: '+e.message;}};
$('validatePose').onclick = () => {
  if (!S.out) return;
  const r=validatePose({q:S.out.q,reference:S.base.q,nb:S.body.bones.length,f:S.frame,names:S.body.bones.map(b=>b.name),limits:{calf:165,lowerarm:165,neck:100}});
  $('guidedLog').textContent=(r.valid?'Pose passes relative-rotation checks.':r.issues.map(x=>`${x.bone}: ${x.angle}° > ${x.limit}°`).join('\n'))+`\nContact error max ${S.motionQuality.constraintMaxCm.toFixed(2)} cm; ${S.motionQuality.failedConstraints} required constraints over 3 cm. Ground/bone findings: ${S.motionQuality.findings.length}.`;
};
$('assistPose').onclick = () => {
  S.poseSuggestions=null;poseSuggestion(0);
};
$('physicsCleanup').onclick = () => {
  if (!S.take) return; const [a,b]=selRange();
  beginPreview('Physics cleanup');
  S.fixes.push({type:'momentum',a,b},{type:'floor',a,b,side:null,stiffness:0,pivot:'ankle'},{type:'smooth',a,b});
  rebuild(); $('guidedLog').textContent=`Physics cleanup applied to f${a}–${b}: momentum, ground contact, and contact-aware smoothing.`;
};
$('secondarySel').onclick = () => {
  if (!S.take) return; const [a,b]=selRange();
  beginPreview('Secondary motion');
  const preserve=S.pins.length?['hand_l','hand_r']:[];
  S.fixes.push({type:'secondary',a,b,bones:['hand','head','weapon','attach'],strength:+$('secondaryStrength').value,damping:+$('secondaryDamping').value,delaySeconds:.05,stiffness:90,maxAngle:15,preserve});
  rebuild(); $('guidedLog').textContent=`Secondary follow-through added to f${a}–${b}.`;
};
$('weaponGrip').onclick = () => {
  if (!S.out) return; const hl=idx('hand_l'),hr=idx('hand_r');
  if(hl===undefined||hr===undefined) return status('Weapon grip needs both mapped hands.');
  const nb=S.body.bones.length,f=S.frame,[a,b]=selRange(),L=new THREE.Matrix4().fromArray(S.out.W,(f*nb+hl)*16),R=new THREE.Matrix4().fromArray(S.out.W,(f*nb+hr)*16);
  const off=R.clone().invert().multiply(L);let prop=S.props.at(-1);
  if(!prop){const wi=firstIdx('weapon_r','ik_hand_gun'),M=wi===undefined?R:new THREE.Matrix4().fromArray(S.out.W,(f*nb+wi)*16);prop={id:crypto.randomUUID(),name:'Rifle grip proxy',driver:'hand_r',length:+$('propLength').value||.8,mass:3.5,offset:R.clone().invert().multiply(M).toArray()};S.props.push(prop);}
  const P=R.clone().multiply(new THREE.Matrix4().fromArray(prop.offset));prop.grips={right:P.clone().invert().multiply(R).toArray(),left:P.clone().invert().multiply(L).toArray()};
  S.pins.push({a,b,ref:f,off:off.toArray(),kind:'weapon',driver:'hand_r',driven:'hand_l',propId:prop.id,required:true});
  rebuild(); $('guidedLog').textContent=`Two-hand weapon grip captured at f${f} and preserved through f${a}–${b}.`;
};
$('fixRange').onclick = () => { if (!S.take) return; const [a, b] = selRange(); S.fixes.push({ type: 'momentum', a, b }); rebuild(); };
$('groundRange').onclick = () => { if (!S.take) return; const [a, b] = selRange(); S.fixes.push({ type: 'ground', a, b }); rebuild(); };
$('smoothRange').onclick = () => { if (!S.take) return; const [a, b] = selRange(); S.fixes.push({ type: 'smooth', a, b }); rebuild(); };
$('clearFix').onclick = () => { S.fixes = []; rebuild(); };
$('ballistic').onclick = () => {
  if (!S.take) return;
  const ct = contactsFromWorld(runPass(buildBase(), null, { layer: true, ik: false }).W);
  let fl = detectFlights(ct.any, 3);
  if (S.sel) fl = fl.filter(x => x.b >= S.sel[0] && x.a <= S.sel[1]);
  if (!fl.length && S.sel) fl = [{ a: S.sel[0], b: S.sel[1] }];
  const g = 9.8 * S.upm, com = comSeries(runPass(buildBase(), null, { layer: true, ik: false }).W);
  const ok = [], rejected = [];
  for (const x of fl) {
    const r = flightReport(fitBallistic(com, x.a, x.b, S.take.fps, g), S.upm);
    (r.airtime_s <= 0.8 && r.apex_m_above_takeoff <= 0.9 ? ok : rejected).push(x);
  }
  for (const x of ok) S.fixes.push({ type: 'ballistic', a: x.a, b: x.b });
  rebuild();
  status(`Snapped ${ok.length} real flight(s) to a ballistic arc.${rejected.length ? ` Skipped ${rejected.length} where the capture's feet hover but no jump is physically possible (${rejected.map(x => 'f' + x.a + '–' + x.b).join(', ')}).` : ''}`);
};
$('fulcrums').onclick = () => {
  if (!S.take) return;
  const W = runPass(buildBase(), null, { layer: true, ik: false }).W, nb = S.body.bones.length;
  const cands = ['foot_l', 'foot_r', 'ball_l', 'ball_r', 'hand_l', 'hand_r'].map(name => ({ name, idx: idx(name) })).filter(c => c.idx !== undefined);
  let runs = detectFulcrums(W, nb, S.take.n, S.take.fps, S.upm, cands, contactsFromWorld(W).ground, 0.25, 0.08, 4);
  if (S.sel) runs = runs.filter(r => r.b >= S.sel[0] && r.a <= S.sel[1]);
  const seen = new Set();
  for (const r of runs) { const k = r.a + ':' + r.b; if (seen.has(k)) continue; seen.add(k); S.fixes.push({ type: 'fulcrum', a: r.a, b: r.b, idx: r.idx, pivot: r.pivot, slip: pivotSlip(W, nb, r.idx, r.a, r.b, S.upm) }); }
  rebuild();
  status(`Added ${seen.size} fulcrum interval(s): the body now pivots around planted hands and feet.`);
};
for (const id of ['mu', 'snapLim', 'gateLim']) $(id).onchange = rebuild;

const tl = $('tl'), tx = tl.getContext('2d');
let TW = 0, TH = 0;
function sizeTL() {
  const r = tl.getBoundingClientRect(), d = devicePixelRatio || 1;
  TW = r.width; TH = r.height; tl.width = TW * d; tl.height = TH * d; tx.setTransform(d, 0, 0, d, 0, 0); drawTL();
}
const fx = f => 8 + f / Math.max(1, (S.take?.n || 2) - 1) * (TW - 16);
const xf = x => Math.round((x - 8) / (TW - 16) * ((S.take?.n || 2) - 1));
function drawTL() {
  if (!TW) return;
  tx.clearRect(0, 0, TW, TH);
  if (!S.take) return;
  const n = S.take.n, mu = css('--muted'), ln = css('--line');
  tx.font = '10px IBM Plex Mono, monospace';
  S.take.shots.forEach((s, i) => { tx.fillStyle = i % 2 ? ln : 'transparent'; tx.globalAlpha = 0.5; tx.fillRect(fx(s.start), 0, fx(s.start + s.n - 1) - fx(s.start), 12); tx.globalAlpha = 1; tx.fillStyle = mu; tx.fillText(s.name.replace(/\.fbx$/i, ''), fx(s.start) + 3, 10); });
  if (S.sel) { tx.fillStyle = css('--accent'); tx.globalAlpha = 0.14; tx.fillRect(fx(S.sel[0]), 0, fx(S.sel[1]) - fx(S.sel[0]) + 1, TH); tx.globalAlpha = 1; }
  const col = { overload: css('--bad'), slide: css('--warn'), float: css('--warn'), snap: css('--bad') };
  for (const fl of S.flags) { tx.fillStyle = col[fl.kind]; tx.fillRect(fx(fl.a), 15, Math.max(2, fx(fl.b) - fx(fl.a)), 7); }
  for (const x of S.fixes) { tx.fillStyle = css('--accent'); tx.fillRect(fx(x.a), 24, Math.max(2, fx(x.b) - fx(x.a)), 4); }
  for (const l of S.locks) { tx.fillStyle = css('--lock'); tx.fillRect(fx(l.a), l.side === 'l' ? 31 : 39, Math.max(2, fx(l.b) - fx(l.a)), 6); }
  for (const p of S.pins) { tx.fillStyle = css('--pin'); tx.fillRect(fx(p.a), 47, Math.max(2, fx(p.b) - fx(p.a)), 5); }
  tx.fillStyle = mu;
  for (const [i, keys] of S.layer) for (const k of keys) {
    const X = fx(k.f), Y = 62, s = i === S.selBone ? 5 : 3;
    tx.fillStyle = i === S.selBone ? css('--com') : mu;
    tx.beginPath(); tx.moveTo(X, Y - s); tx.lineTo(X + s, Y); tx.lineTo(X, Y + s); tx.lineTo(X - s, Y); tx.fill();
  }
  const step = n > 600 ? 60 : n > 200 ? 24 : 10;
  for (let f = 0; f < n; f += step) { tx.fillStyle = mu; tx.fillRect(fx(f), TH - 16, 1, 5); tx.fillText(f, fx(f) + 2, TH - 4); }
  tx.fillStyle = css('--fg'); tx.fillRect(fx(S.frame) - 1, 0, 2, TH - 14);
}
let tlDrag = null;
tl.addEventListener('pointerdown', e => {
  if (!S.take) return;
  tl.setPointerCapture(e.pointerId);
  const f = xf(e.offsetX);
  S.playing = false; setPlay();
  if (e.shiftKey) { tlDrag = { sel: true, a: f }; S.sel = [f, f]; } else { tlDrag = { sel: false }; if (!e.altKey) S.sel = S.sel; }
  showFrame(f);
});
tl.addEventListener('pointermove', e => {
  if (!tlDrag || !S.take) return;
  const f = Math.max(0, Math.min(S.take.n - 1, xf(e.offsetX)));
  if (tlDrag.sel) S.sel = [Math.min(tlDrag.a, f), Math.max(tlDrag.a, f)];
  showFrame(f);
});
tl.addEventListener('pointerup', () => { tlDrag = null; });
tl.addEventListener('dblclick', () => { S.sel = null; drawTL(); showFrame(S.frame); });

function setPlay() { $('play').textContent = S.playing ? 'Pause' : 'Play'; $('play').classList.toggle('on', S.playing); }
$('play').onclick = () => { S.playing = !S.playing; setPlay(); };
$('stepB').onclick = () => showFrame(S.frame - 1);
$('stepF').onclick = () => showFrame(S.frame + 1);
addEventListener('keydown', e => {
  if (/INPUT|SELECT/.test(e.target.tagName)) return;
  if (e.code === 'Space') { e.preventDefault(); $('play').click(); }
  if (e.key === 'ArrowRight') showFrame(S.frame + 1);
  if (e.key === 'ArrowLeft') showFrame(S.frame - 1);
  if (e.key === 'Escape') selectBone(-1);
});

let last = performance.now(), acc = 0;
function loop(t) {
  if (S.playing && S.take) {
    acc += (t - last) / 1000;
    const dt = 1 / S.take.fps;
    if (acc >= dt) { const k = Math.floor(acc / dt); acc -= k * dt; showFrame((S.frame + k) % S.take.n); }
  } else acc = 0;
  last = t;
  orbit.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);

async function preserveImportedSource(file, buf) {
  if (!LOCAL) throw new Error('Local FBX imports need the Fulcrum server to make projects reopenable.');
  const stored = 'source_' + crypto.randomUUID() + '_' + file.name.replace(/[^\w.-]/g, '_');
  const response = await fetch('/api/save/' + encodeURIComponent(stored), { method: 'POST', body: buf });
  if (!response.ok) throw new Error('Could not preserve imported FBX source.');
  return { location: 'exports', name: stored };
}
$('fileIn').onchange = async e => {
  for (const file of e.target.files) {
    const buf = await file.arrayBuffer();
    try {
      await appendBytes(buf, file.name, $('retargetMode').value, await preserveImportedSource(file, buf));
    } catch (err) { status(err.message); }
  }
  e.target.value = '';
};
$('autoRigIn').onchange = async e => {
  const file = e.target.files?.[0];
  if (!file) return;
  try {
    status('Importing and auto-rigging ' + file.name + '…');
    const buf = await file.arrayBuffer();
    const source = await preserveImportedSource(file, buf);
    await loadBodyBytes(buf, file.name, source, true);
    setWorkspace('rig');
  } catch (err) { status('Auto-rig failed: ' + err.message); }
  e.target.value = '';
};
$('loadBody').onclick = async () => { const n = $('sampleList').value; if (!n) return; status('Loading ' + n + '…'); try { await loadBodyBytes(await fetchSample(n), n); } catch (err) { status(err.message); } };
$('appendTake').onclick = async () => { const n = $('sampleList').value; if (!n) return; status('Loading ' + n + '…'); try { await appendBytes(await fetchSample(n), n, $('retargetMode').value); } catch (err) { status(err.message); } };
const FRT = ['CheckpointBody.fbx', 'shot1_f001-101_ue5.fbx', 'shot2_f102-264_ue5.fbx', 'shot3_f270-361_ue5.fbx'];
let frtQueue = Promise.resolve();
function loadFRT() {
  const run = frtQueue.then(loadFRTFresh);
  frtQueue = run.catch(() => {});
  return run;
}
async function loadFRTFresh() {
  status('Loading the Checkpoint soldier and the three FRT shots…');
  await loadBodyBytes(await fetchSample(FRT[0]), FRT[0]);
  for (const s of FRT.slice(1)) await appendBytes(await fetchSample(s), s);
  status(`FRT drill on the Checkpoint soldier: ${S.take.n} frames at ${S.take.fps} fps (${(S.take.n / S.take.fps).toFixed(1)} s), ${S.flags.length} physics flag(s).`);
}
$('loadFRT').onclick = () => loadFRT().catch(err => status(err.message));

function projectJSON() {
  return {
    app: 'fulcrum', version: 1, body: S.body?.name, shots: S.take?.shots.map(s => s.name) || [], fps: S.take?.fps,
    bodySource: S.body?.source,
    ...(S.body?.autorig?.kind === 'generated' ? { bodyAutoRig: true } : {}),
    sourceShots: S.sourceShots,
    ...(S.take?.shots.some(s => s.retarget) ? { retarget: Object.fromEntries(S.take.shots.filter(s => s.retarget).map(s => [s.name, s.retarget])) } : {}),
    falloff: +$('falloff').value, mu: +$('mu').value,
    layer: [...S.layer].map(([i, keys]) => ({ bone: S.body.bones[i].name, keys })),
    ...(S.animLayers.length ? { animLayers: S.animLayers } : {}),
    cuts: S.cuts || [], retimes:S.retimes||[], locks: S.locks.map(({ side, a, b,required }) => ({ side, a, b,required })), pins: S.pins.map(({ a, b, ref, off, world, kind, driver, driven,propId,required }) => ({ a, b, ref, off, world, kind, driver, driven,propId,required })), fixes: S.fixes,props:S.props,bookmarks:S.bookmarks,
  };
}
async function saveFile(name, data, type) {
  if (LOCAL) {
    const r = await fetch('/api/save/' + encodeURIComponent(name), { method: 'POST', body: data });
    const j = await r.json();
    return j.path;
  }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([data], { type }));
  a.download = name; a.click();
  return name;
}
$('saveProj').onclick = async () => {
  if (!S.take) return;
  const p = await saveFile(($('outName').value || 'fulcrum') + '.fulcrum.json', JSON.stringify(projectJSON(), null, 1), 'application/json');
  $('exportLog').textContent = 'Project saved: ' + p;
};
$('projIn').onchange = async e => {
  const f = e.target.files[0]; if (!f) return;
  try {
    const P = JSON.parse(await f.text());
    status('Opening project…');
    await applyProject(P);
    status('Project opened.');
  } catch (err) { status('Project open failed: ' + err.message); }
  finally { e.target.value = ''; }
};

function bakedClip() {
  const B = S.body, nb = B.bones.length, n = S.take.n, fps = S.take.fps;
  const times = Float32Array.from({ length: n }, (_, f) => f / fps);
  const tracks = [];
  B.bones.forEach((b, i) => {
    const q = new Float32Array(n * 4), p = new Float32Array(n * 3);
    for (let f = 0; f < n; f++) { for (let k = 0; k < 4; k++) q[f * 4 + k] = S.out.q[(f * nb + i) * 4 + k]; for (let k = 0; k < 3; k++) p[f * 3 + k] = S.out.p[(f * nb + i) * 3 + k]; }
    tracks.push(new THREE.QuaternionKeyframeTrack(b.name + '.quaternion', times, q));
    tracks.push(new THREE.VectorKeyframeTrack(b.name + '.position', times, p));
  });
  return new THREE.AnimationClip('Fulcrum', (n - 1) / fps, tracks);
}
$('exportGlb').onclick = async () => {
  if (!S.out) return;
  const clip = bakedClip();
  const glb = await new GLTFExporter().parseAsync(S.body.obj, { binary: true, animations: [clip] });
  const p = await saveFile(($('outName').value || 'fulcrum') + '.glb', glb, 'model/gltf-binary');
  showFrame(S.frame);
  $('exportLog').textContent = 'GLB saved: ' + p;
};

async function autoFix({ maxRounds = 4 } = {}) {
  if (!S.out) throw new Error('load a take first');
  const log = $('autoFixLog'), lines = [];
  const say = t => { lines.push(t); log.textContent = lines.join(String.fromCharCode(10)); };
  const strip = x => { const { hits, ...y } = x; return y; };
  const apply = async st => { S.fixes = st.fixes.map(x => ({ ...x })); S.locks = st.locks.map(x => ({ ...x })); rebuild(); };
  const judge = async () => fulcrumFindings({ flags: S.flags, quality: S.motionQuality, slidingPlants: plantsOf(S.out.W).filter(p => p.sliding && !p.touch), contact: S.contact });
  say('Auto Fix: checking the current take…');
  const before = editSnapshot();
  S.autoFixing = true;
  let r;
  try {
    const plants = detectPlantsNow().filter(p => !p.sliding).map(p => ({ side: p.side, a: p.a, b: p.b }));
    r = await autoFixLoop({ judge, apply, maxRounds, fixes: S.fixes.map(strip), locks: S.locks.map(({ side, a, b, required }) => required === false ? { side, a, b, required } : { side, a, b }), n: S.take.n, fps: S.take.fps, plants, log: say });
  } finally { S.autoFixing = false; recordEdit(before); }
  say(`Done (${r.reason}, ${r.rounds} round(s)). Blockers ${r.start.blockers} → ${r.end.blockers}, warnings ${r.start.warnings} → ${r.end.warnings}. Kept ${r.kept.length}, reverted ${r.reverted.length}.`);
  S.autoFixReport = { start: r.start, end: r.end, rounds: r.rounds, reason: r.reason, kept: r.kept, reverted: r.reverted, notes: r.notes };
  return S.autoFixReport;
}
$('autoFix').onclick = () => autoFix().catch(e => { $('autoFixLog').textContent = 'Auto Fix failed: ' + e.message; });

async function saveExportSkeleton() {
  const stem = 'skeleton_' + S.body.name.replace(/[^\w.-]/g, '_');
  if (S.body.autorig?.kind !== 'generated') {
    const name = stem.endsWith('.fbx') ? stem : stem + '.fbx';
    await fetch('/api/save/' + encodeURIComponent(name), { method: 'POST', body: S.body.bytes });
    return name;
  }
  const name = stem + '.glb', frame = S.frame;
  try {
    S.body.bones.forEach((bone, i) => { bone.position.copy(S.body.restP[i]); bone.quaternion.copy(S.body.restQ[i]); });
    S.body.obj.updateMatrixWorld(true);
    const glb = await new GLTFExporter().parseAsync(S.body.obj, { binary: true });
    await saveFile(name, glb, 'model/gltf-binary');
    return name;
  } finally {
    if (S.take && S.out) showFrame(frame);
  }
}

$('exportFbx').onclick = async () => {
  if (!S.out) return;
  const log = $('exportLog');
  if(EDITS.preview){log.textContent='Accept or reject the current preview before exporting.';return;}
  if(S.motionQuality&&!S.motionQuality.valid&&!$('exportAnyway').checked){log.textContent=`Export stopped: ${S.motionQuality.failedConstraints||0} required contact failures or invalid transforms. Inspect pose and contacts, or choose export anyway.`;return;}
  if (!LOCAL) { log.textContent = 'FBX export needs the local Fulcrum server (run server.py). It drives Blender to write the FBX.'; return; }
  const name = ($('outName').value || 'fulcrum').replace(/[^\w.-]/g, '_');
  log.textContent = 'Writing frames, Blender is building the FBX…';
  const skelName = await saveExportSkeleton();
  const nb = S.body.bones.length;
  const frames = [];
  for (let f = 0; f < S.take.n; f++) frames.push(Array.from(S.out.W.subarray(f * nb * 16, (f + 1) * nb * 16), v => +v.toFixed(5)));
  const req = { name, skeleton: skelName, fps: S.take.fps, bones: S.body.bones.map(b => b.name), rest: S.body.restW.map(m => m.toArray()), frames,upm:S.upm,props:S.props.map(prop=>({...prop,driverIndex:idx(prop.driver||'hand_r')})) };
  const r = await fetch('/api/export-fbx', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(req) });
  const j = await r.json();
  log.textContent = (j.ok ? 'FBX written: ' + j.path : 'FBX export failed') + '\n' + j.log.join('\n');
  if (j.ok) {
    const v = await verifyFbx(j.fbx);
    const kr = reduceKeysReport({ W: S.out.W, nb, n: S.take.n, fps: S.take.fps, upm: S.upm });
    log.textContent += `\nKey reduction (report only, export stays per-frame): ${kr.totalKept}/${kr.totalFrames} bone-keys needed at 0.1°/0.05 cm (${(kr.ratio * 100).toFixed(0)}%)`;
    log.textContent += `\nRound-trip check: ${v.frames} frames, ${v.joints} joints, joint position error max ${v.maxCm.toFixed(2)} cm, mean ${v.meanCm.toFixed(2)} cm ${v.maxCm < 1 ? '(PASS)' : '(FAIL)'}`;
  }
};

async function verifyFbx(url) {
  const buf = await (await fetch(url)).arrayBuffer();
  const obj = parseFBX(buf);
  obj.updateMatrixWorld(true);
  const clip = obj.animations[0];
  const bones = bonesOf(obj);
  const pairs = bones.map(b => [b, S.body.byKey.get(keyOf(b.name))]).filter(p => p[1] !== undefined);
  if(!clip||pairs.length!==S.body.bones.length)throw new Error(`Export validation: ${pairs.length}/${S.body.bones.length} joints matched.`);
  const mixer = new THREE.AnimationMixer(obj);
  const act = mixer.clipAction(clip);
  act.setLoop(THREE.LoopOnce);
  act.clampWhenFinished = true;
  act.play();
  const nb = S.body.bones.length, fps = S.take.fps, n = S.take.n;
  const P = [], Q = [];
  const sampleF = Array.from({length:n},(_,f)=>f);
  for (const f of sampleF) {
    mixer.setTime(Math.min(f / fps, clip.duration - 1e-6)); obj.updateMatrixWorld(true);
    for (const [b, i] of pairs) { P.push(new THREE.Vector3().setFromMatrixPosition(b.matrixWorld)); Q.push(new THREE.Vector3().fromArray(S.out.W, (f * nb + i) * 16 + 12)); }
  }
  const err = M => { let mx = 0, sum = 0; P.forEach((p, k) => { const e = (M ? p.clone().applyMatrix4(M) : p).distanceTo(Q[k]) / S.upm * 100; mx = Math.max(mx, e); sum += e; }); return [mx, sum / P.length]; };
  const direct = err(null), alignment=similarity(P.slice(0,pairs.length),Q.slice(0,pairs.length)), fitted = err(alignment);
  const [mx, mean] = direct[0] <= fitted[0] ? direct : fitted, sum = mean * P.length;
  mixer.stopAllAction();
  showFrame(S.frame);
  const result={ frames: sampleF.length, joints: pairs.length, maxCm: mx, meanCm: sum / P.length,directMaxCm:direct[0],alignedMaxCm:fitted[0],alignment:'single transform fitted at frame 0',fps,duration:clip.duration,expectedDuration:(n-1)/fps };
  S.lastExportValidation=result;
  await saveFile(($('outName').value||'fulcrum')+'.validation.json',JSON.stringify(result,null,2),'application/json');
  return result;
}

function similarity(P, Q) {
  const n = P.length, pc = new THREE.Vector3(), qc = new THREE.Vector3();
  P.forEach(p => pc.add(p)); Q.forEach(q => qc.add(q)); pc.divideScalar(n); qc.divideScalar(n);
  const H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  let vp = 0;
  for (let k = 0; k < n; k++) {
    const a = P[k].clone().sub(pc), b = Q[k].clone().sub(qc);
    const av = [a.x, a.y, a.z], bv = [b.x, b.y, b.z];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) H[i][j] += av[i] * bv[j];
    vp += a.lengthSq();
  }
  const [[Sxx, Sxy, Sxz], [Syx, Syy, Syz], [Szx, Szy, Szz]] = H;
  const N = [
    [Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx],
    [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
    [Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy],
    [Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz],
  ];
  const shift = N.reduce((m, r) => m + r.reduce((x, v) => x + Math.abs(v), 0), 0);
  let v = [1, 0.1, 0.1, 0.1];
  for (let it = 0; it < 500; it++) {
    const nv = N.map((r, i) => r.reduce((x, c, j) => x + c * v[j], 0) + shift * v[i]);
    const len = Math.hypot(...nv);
    v = nv.map(x => x / len);
  }
  const q = new THREE.Quaternion(v[1], v[2], v[3], v[0]).normalize();
  const Rm = new THREE.Matrix4().makeRotationFromQuaternion(q);
  let num = 0;
  for (let k = 0; k < n; k++) num += P[k].clone().sub(pc).applyMatrix4(Rm).dot(Q[k].clone().sub(qc));
  const s = num / vp;
  const M = new THREE.Matrix4().makeTranslation(qc.x, qc.y, qc.z).multiply(new THREE.Matrix4().makeScale(s, s, s)).multiply(Rm).multiply(new THREE.Matrix4().makeTranslation(-pc.x, -pc.y, -pc.z));
  return M;
}

async function boot() {
  resize();
  if (!LOCAL) { $('sampleList').innerHTML = '<option value="">Run server.py for the Checkpoint samples</option>'; status('Open FBX files to begin. FBX export needs the local server.'); return; }
  const j = await (await fetch('/api/samples')).json();
  $('sampleList').innerHTML = j.files.map(f => `<option>${f}</option>`).join('');
  if (FRT.every(f => j.files.includes(f))) loadFRT().catch(err => status(err.message));
}
boot();
window.__fulcrum = S;
window.__fulcrumRebuild = rebuild;
window.__fulcrumOpenUrl = async (url, name) => { const buf = await (await fetch(url)).arrayBuffer(); S.take = null; await loadBodyBytes(buf, name); await appendBytes(buf, name); };

const wpos = (W, f, i) => { const o = (f * S.body.bones.length + i) * 16; return [W[o + 12], W[o + 13], W[o + 14]]; };
const cm = v => v.map(x => +(x / S.upm * 100).toFixed(2));
const VIEWS = { side: [1, 0.35, 0], front: [0, 0.35, 1], back: [0, 0.35, -1], top: [0.01, 1, 0], three_quarter: [0.8, 0.45, 0.9] };
async function applyProject(P) {
  if(P.app!=='fulcrum'||P.version>1)throw new Error('Unsupported Fulcrum project version.');
  EDITS.preview=null;S.poseSuggestions=null;
  if (!S.body || S.body.name !== P.body || JSON.stringify(S.body.source) !== JSON.stringify(P.bodySource || { location: 'samples', name: P.body }) || !!P.bodyAutoRig !== (S.body.autorig?.kind === 'generated') || (S.cuts || []).length || (S.retimes || []).length || (S.take?.shots.map(s => s.name).join() !== (P.shots || []).join())) {
    const bodySource = P.bodySource || { location: 'samples', name: P.body };
    await loadBodyBytes(await fetchProjectSource(bodySource), P.body, bodySource, !!P.bodyAutoRig);
    const sources = P.sourceShots?.length ? P.sourceShots : P.clipSources?.length === P.shots?.length ? P.clipSources : P.shots.map(name => ({ name, source: { location: 'samples', name }, clipIndex: 0 }));
    for (const shot of sources) {
      const source = shot.source || { location: 'samples', name: shot.name };
      await appendBytes(await fetchProjectSource(source), shot.name, shot.mode || P.retarget?.[shot.name] || 'auto', source, shot.clipIndex ?? 0, P.fps || null);
      if (shot.clipName != null && S.take.shots.at(-1).clipName !== shot.clipName) throw new Error(`Clip ${shot.clipIndex} in ${source.name} changed from ${shot.clipName} to ${S.take.shots.at(-1).clipName}.`);
    }
  }
  S.cuts = [];
  for (const c of P.cuts || []) { if (c.op === 'insert') insertTake(c.at, c.a, c.b); else cutTake(c.a, c.b); S.cuts.push(c); }
  S.retimes=[];
  for(const r of P.retimes||[]){retimeRaw(r.a,r.b,r.scale);S.retimes.push({...r});}
  if (P.falloff != null) $('falloff').value = P.falloff;
  if (P.mu != null) $('mu').value = P.mu;
  S.layer.clear();
  for (const L of P.layer || []) { const i = S.body.bones.findIndex(b => b.name === L.bone); if (i >= 0) S.layer.set(i, L.keys); }
  S.locks = P.locks || []; S.pins = P.pins || []; S.fixes = P.fixes || []; S.animLayers = P.animLayers || []; S.activeLayer = -1;S.props=P.props||[];S.bookmarks=P.bookmarks||[];
  propGroup.clear();
  for(const prop of S.props)if(prop.source){try{const bytes=await(await fetch('/exports/'+encodeURIComponent(prop.source))).arrayBuffer(),obj=parseFBX(bytes);obj.scale.multiplyScalar(prop.scale||1);obj.position.set(0,0,0);const group=new THREE.Group();group.add(obj);group.matrixAutoUpdate=false;group.userData.propId=prop.id;propGroup.add(group);}catch(e){status('Prop '+prop.name+' unavailable: using grip proxy.');}}
  rebuild();
  renderAnimLayers();
}
function summaryState() {
  if (!S.take) return { loaded: false };
  return {
    loaded: true, body: S.body.name, bones: S.body.bones.length, frames: S.take.n, fps: S.take.fps, upm: S.upm,
    shots: S.take.shots.map(s => ({ name: s.name, start: s.start, n: s.n })), frame: S.frame, selection: S.sel,
    fixes: S.fixes.map((x, i) => ({ index: i, ...x })), locks: S.locks.map(({ side, a, b }) => ({ side, a, b })), pins: S.pins.map(({ a, b, ref, world, kind, driver, driven }) => ({ a, b, ref, world: !!world, kind, driver, driven })),
    gate: S.gate && { pass: S.gate.pass, worst: S.gate.worst, worstBone: S.gate.worstBone, pelvisCm: S.gate.pelvisCm, limit: S.gate.lim },
    flags: (S.flags || []).map(f => ({ kind: f.kind, a: f.a, b: f.b, text: f.text })), floorFrames: S.floorFrames ?? 0, rebuildMs: +($('hud').dataset.ms || 0),
    rigmap: S.body.rig, autoFix: S.autoFixReport || null,
    guided:{preview:EDITS.preview?.label||null,reports:S.advancedReport,props:S.props,history:{undo:EDITS.cursor,redo:EDITS.rows.length-EDITS.cursor-1},quality:S.motionQuality&&{valid:S.motionQuality.valid,constraintMaxCm:S.motionQuality.constraintMaxCm,failedConstraints:S.motionQuality.failedConstraints,findings:S.motionQuality.findings.slice(0,25)}},
    retarget: S.take.shots.filter(s => s.retarget).map(s => ({ name: s.name, mode: s.retarget, slots: s.slots })),
    animLayers: S.animLayers.map(L => ({ name: L.name, weight: L.weight, mute: L.mute, override: L.override, accumulation: L.accumulation, tangent: L.tangent, bones: Object.fromEntries(Object.entries(L.bones || {}).map(([b, k]) => [b, k.length])) })),
    footsteps: S.footstepPlans || [], story: (S.storyReport || []).map(r => ({ ...r, matchOffsetCm: +(Math.hypot(...r.matchOffset) / S.upm * 100).toFixed(2) })),
  };
}
const API = {
  status: async () => summaryState(),
  guided_motion:async({action,options={}})=>{
    const ids={inbetween:'inbetweenSel',suggest_pose:'assistPose',cascadeur_pose:'cascadeurPose',next_pose:'nextPose',physics:'physicsCleanup',secondary:'secondarySel',weapon_grip:'weaponGrip',accept:'guidedAccept',reject:'guidedReject',undo:'editUndo',redo:'editRedo',validate:'validatePose',bookmark:'bookmarkPose'};
    if(!ids[action])throw new Error('Unknown guided motion action: '+action);
    for(const [key,id]of Object.entries({strength:'inbetweenStrength',spacing:'guidedSpacing',secondaryStrength:'secondaryStrength',damping:'secondaryDamping',style:'neuralStyle',poseSnap:'poseSnap',poseSpine:'poseSpine'}))if(options[key]!=null)$(id).value=options[key];
    if(options.preserveRoot!=null)$('guidedRoot').checked=!!options.preserveRoot;
    await $(ids[action]).onclick();
    return {message:$('guidedLog').textContent,state:summaryState()};
  },
  retime: async ({scale}) => { const result=retimeSelection(Number(scale)); return {...result,state:summaryState()}; },
  motion_quality:async()=>S.motionQuality,
  load_frt: async () => { await loadFRT(); return summaryState(); },
  open_project: async ({ name }) => { const P = await (await fetch('/exports/' + encodeURIComponent(name))).json(); await applyProject(P); return summaryState(); },
  save_project: async ({ name }) => ({ path: await saveFile(name.replace(/\.fulcrum\.json$/, '') + '.fulcrum.json', JSON.stringify(projectJSON(), null, 1), 'application/json') }),
  set_fixes: async ({ fixes }) => { S.fixes = fixes; rebuild(); return summaryState(); },
  add_fix: async ({ fix }) => { S.fixes.push(fix); rebuild(); return summaryState(); },
  cut: async ({ a, b, blend = 6, story = false, curve = 'ease', match = 'gravity' }) => { doCut(a, b, { blend, story, curve, match }); return summaryState(); },
  insert: async ({ at, a, b, blend = 6, story = false, curve = 'ease', match = 'gravity' }) => { doInsert(at, a, b, { blend, story, curve, match }); return summaryState(); },
  import_fbx: async ({ name, mode = 'hik', as_body = false, replace = true, realistic, leg_roll }) => {
    const buf = await fetchSample(name);
    if (as_body) { await loadBodyBytes(buf, name); renderShots(); return { loaded: !!S.take, body: S.body.name, bones: S.body.bones.length, rigmap: S.body.rig }; }
    if (replace && S.take) { S.take = null; S.sourceShots = []; S.cuts = []; S.retimes=[];S.fixes = []; S.locks = []; S.pins = []; S.layer.clear(); S.animLayers = []; S.srcTake = null; S.srcIdx = null; }
    if (!S.body) throw new Error('load the Checkpoint body first (load_frt or import_fbx with as_body)');
    const obj = parseFBX(buf), t = sampleAllClips(obj, name, mode === 'hik' || (mode === 'auto' && nameMatchShare(obj) < 0.8) ? 'hik' : 'names');
    if (t.shots?.length > 1) for (const shot of t.shots) appendTake({ ...t, n: shot.n, q: t.q.slice(shot.start * S.body.bones.length * 4, (shot.start + shot.n) * S.body.bones.length * 4), p: t.p.slice(shot.start * S.body.bones.length * 3, (shot.start + shot.n) * S.body.bones.length * 3), shots: [shot] }, shot.name);
    else appendTake(t, name); renderShots(); rebuild();
    return { ...summaryState(), retarget: t.retarget || 'names', slots: t.slots };
  },
  set_anim_layers: async ({ layers }) => { S.animLayers = (layers || []).map(normalizeLayer); S.activeLayer = -1; rebuild(); renderAnimLayers(); return summaryState(); },
  key_anim_layer: async ({ layer = 0, bone, f, euler, q }) => {
    const i = S.body.byKey.get(keyOf(bone)); if (i === undefined) throw new Error('no bone ' + bone);
    if (!S.animLayers[layer]) throw new Error('no animation layer ' + layer);
    let e = euler;
    if (!e) {
      const nb = S.body.bones.length, below = new THREE.Quaternion().fromArray(S.base.q, (f * nb + i) * 4).multiply(layerDelta(i, f, new THREE.Quaternion())).toArray();
      e = solveLayerKey(S.animLayers, S.animCompiled?.get(i), layer, below, q, f);
    }
    S.animLayers[layer] = setLayerKey(normalizeLayer(S.animLayers[layer]), S.body.bones[i].name, f, e);
    rebuild(); renderAnimLayers();
    return { layer: S.animLayers[layer].name, bone: S.body.bones[i].name, f, euler: e, state: summaryState() };
  },
  sole_markers: async ({ f, contact_size = 100 }) => {
    const nb = S.body.bones.length, rig = soleMarkerRig({ W: S.out.W, nb, n: S.take.n, upm: S.upm, ground: S.ground, legs: { l: { foot: idx('foot_l'), ball: firstIdx('ball_l') }, r: { foot: idx('foot_r'), ball: firstIdx('ball_r') } } });
    const mk = soleMarkers(rig, S.out.W, nb, f ?? S.frame, contact_size);
    return Object.fromEntries(Object.entries(mk).map(([k, pts]) => [k, Object.fromEntries(Object.entries(pts).map(([n, p]) => [n, cm(p)]))]));
  },
  remove_fix: async ({ index }) => { S.fixes.splice(index, 1); rebuild(); return summaryState(); },
  set_locks: async ({ locks }) => { S.locks = locks; rebuild(); return summaryState(); },
  set_pins: async ({ pins }) => { S.pins = pins; rebuild(); return summaryState(); },
  set_layer: async ({ bone, keys }) => { const i = S.body.byKey.get(keyOf(bone)); if (i === undefined) throw new Error('no bone ' + bone); keys && keys.length ? S.layer.set(i, keys) : S.layer.delete(i); rebuild(); return summaryState(); },
  select: async ({ a, b }) => { S.sel = a == null ? null : [a, b]; showFrame(a ?? S.frame); return { selection: S.sel }; },
  frame: async ({ f }) => { showFrame(f); return { frame: S.frame }; },
  rebuild: async () => { rebuild(); return summaryState(); },
  auto_fix: async ({ max_rounds = 4 } = {}) => ({ ...(await autoFix({ maxRounds: max_rounds })), state: summaryState() }),
  graph_editor: async ({ open = true, bone, key_frame, layer } = {}) => {
    if (layer != null) { S.activeLayer = layer; renderAnimLayers(); }
    if (bone) { const i = S.body.byKey.get(keyOf(bone)); if (i === undefined) throw new Error('no bone ' + bone); selectBone(i); }
    geOpen(open);
    const keyed = key_frame != null ? geKeyBone(S.selBone, key_frame) : null;
    geRefresh();
    return { open: GE.open, keyed, activeLayer: S.activeLayer, curves: geCurves().map(c => ({ bone: c.bone, channel: c.channel, keys: c.keys.map(k => ({ t: k.t, v: k.v, inType: k.inType, outType: k.outType })) })) };
  },
  pose: async ({ frames, bones }) => {
    const keys = bones && bones.length ? bones : ['pelvis', 'spine_03', 'spine_05', 'head', 'hand_l', 'hand_r', 'thigh_l', 'calf_l', 'foot_l', 'ball_l', 'thigh_r', 'calf_r', 'foot_r', 'ball_r'];
    const out = {};
    for (const f of frames) { const row = {}; for (const k of keys) { const i = idx(k); if (i !== undefined) row[k] = cm(wpos(S.out.W, f, i)); } out[f] = row; }
    return { units: 'cm, Y-up, Fulcrum world', ground_cm: +(S.contact.ground / S.upm * 100).toFixed(2), frames: out };
  },
  screenshot: async ({ f, view = 'side', width = 640, height = 640 }) => {
    if (f != null) showFrame(f);
    const pel = idx('pelvis'), c = new THREE.Vector3(...wpos(S.out.W, S.frame, pel));
    const tl = new THREE.Vector3(...wpos(S.out.W, S.frame, idx('thigh_l'))), tr = new THREE.Vector3(...wpos(S.out.W, S.frame, idx('thigh_r')));
    const right = tr.sub(tl).setY(0).normalize(), fwd = new THREE.Vector3(0, 1, 0).cross(right).normalize(), v = VIEWS[view] || VIEWS.side;
    const d = right.clone().multiplyScalar(v[0]).add(new THREE.Vector3(0, v[1], 0)).add(fwd.multiplyScalar(v[2])).normalize().multiplyScalar(S.upm * 4.2);
    const pc = camera.position.clone(), pt = orbit.target.clone(), size = renderer.getSize(new THREE.Vector2());
    renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix();
    orbit.target.set(c.x, S.ground + S.upm * 0.9, c.z); camera.position.set(c.x + d.x, S.ground + S.upm * 0.9 + d.y, c.z + d.z); camera.lookAt(orbit.target);
    renderer.render(scene, camera);
    const url = renderer.domElement.toDataURL('image/png');
    renderer.setSize(size.x, size.y, false); camera.aspect = size.x / size.y; camera.updateProjectionMatrix(); camera.position.copy(pc); orbit.target.copy(pt); orbit.update();
    return { png: url.split(',')[1], frame: S.frame, view };
  },
  export_fbx: async ({ name, override = false }) => {
    $('outName').value = name; $('exportAnyway').checked = !!override;
    await $('exportFbx').onclick();
    return { log: $('exportLog').textContent };
  },
  eval: async ({ code }) => { const fn = new Function('S', 'THREE', 'api', 'return (async () => {' + code + '})()'); return await fn(S, THREE, { rebuild, showFrame, idx, firstIdx, wpos, sampleTakeHik, appendTake, renderShots, detectFps, bakedClip, geOpen, geKeyBone, geRefresh, GE, selectBone, autoFix, loadBodyBytes, fetchSample, keyOf }); },
};
async function bridgeLoop() {
  if (!LOCAL) return;
  const client=crypto.randomUUID();
  let modern=false;try{modern=(await fetch('/api/bridge/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client})})).ok;}catch{}
  for (;;) {
    let job = null;
    try { job = await (await fetch('/api/bridge/next'+(modern?'?client='+client:''),{cache:'no-store'})).json(); } catch { await new Promise(r => setTimeout(r, 2000)); continue; }
    if(job?.stale)return;
    if (!job || !job.id) continue;
    let reply;
    try {
      const fn = API[job.cmd];
      if (!fn) throw new Error('unknown command ' + job.cmd);
      reply = { id: job.id, ok: true, result: await fn(job.args || {}) };
    } catch (e) { reply = { id: job.id, ok: false, error: String(e && e.stack || e) }; }
    try { await fetch('/api/bridge/result', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(reply) }); } catch {}
  }
}
bridgeLoop();
document.addEventListener('keydown', e => {
  if ((e.key === 'Enter' || e.key === ' ') && e.target.matches('[role="button"][tabindex]')) { e.preventDefault(); e.target.click(); }
});
