import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { BONES, PARENT, NB, B, SIDES, fk, getV, getQ } from './skeleton.js';
import { hikRetargetSetup, hikRetargetFrame, dstWorldRotations } from '../wire/hikretarget.js';

const V3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);
const PALETTE = { body: '#d8d2c8', left: '#b9cde3', right: '#e4c9b4', joint: '#33404d', head: '#e2ddd5' };

function paint(g, bone, color) {
  const n = g.attributes.position.count, c = new THREE.Color(color), col = new Float32Array(n * 3), si = new Uint16Array(n * 4), sw = new Float32Array(n * 4);
  for (let i = 0; i < n; i++) { col.set([c.r, c.g, c.b], i * 3); si[i * 4] = bone; sw[i * 4] = 1; }
  g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  g.setAttribute('skinIndex', new THREE.BufferAttribute(si, 4));
  g.setAttribute('skinWeight', new THREE.BufferAttribute(sw, 4));
  return g;
}

function capsule(a, b, r, bone, color, flat = 1) {
  const d = V3(b).sub(V3(a)), L = Math.max(1e-3, d.length()), g = new THREE.CapsuleGeometry(r, L, 6, 14);
  if (flat !== 1) g.scale(1, 1, flat);
  g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize()));
  g.translate((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2);
  return paint(g, bone, color);
}

function ball(c, r, bone, color, s = [1, 1, 1]) {
  const g = new THREE.SphereGeometry(r, 16, 12);
  g.scale(s[0], s[1], s[2]);
  g.translate(c[0], c[1], c[2]);
  return paint(g, bone, color);
}

export function buildBones(skel) {
  const bones = BONES.map((name, i) => { const b = new THREE.Bone(); b.name = name; b.position.copy(V3(skel.offsets[i])); return b; });
  for (let i = 1; i < NB; i++) bones[PARENT[i]].add(bones[i]);
  return bones;
}

export function buildMannequin(skel) {
  const P = i => skel.rest[i], k = skel.dims.height / 1.73, m = skel.m, parts = [];
  const side = s => s === 'L' ? PALETTE.left : PALETTE.right;
  const end = name => { const o = skel.ends[name]; return [P(B[name])[0] + o[0], P(B[name])[1] + o[1], P(B[name])[2] + o[2]]; };
  const hipsMid = [(P(B.LeftUpLeg)[0] + P(B.RightUpLeg)[0]) / 2, P(B.LeftUpLeg)[1], 0];
  parts.push(ball([0, hipsMid[1] + 0.035 * k, 0], 1, B.Hips, PALETTE.body, [m.hipWidth * 0.85, 0.105 * k, 0.11 * k]));
  parts.push(capsule(P(B.Spine), P(B.Spine1), 0.11 * k, B.Spine, PALETTE.body, 0.85));
  parts.push(capsule(P(B.Spine1), P(B.Spine2), 0.12 * k, B.Spine1, PALETTE.body, 0.8));
  parts.push(ball([0, (P(B.Spine2)[1] + P(B.Neck)[1]) / 2, 0.005 * k], 1, B.Spine2, PALETTE.body, [m.shoulderWidth * 0.46, 0.13 * k, 0.105 * k]));
  parts.push(capsule(P(B.Neck), P(B.Head), 0.042 * k, B.Neck, PALETTE.body));
  parts.push(ball([0, P(B.Head)[1] + 0.085 * k, 0.012 * k], 1, B.Head, PALETTE.head, [0.082 * k, 0.108 * k, 0.098 * k]));
  parts.push(ball([0, P(B.Head)[1] + 0.07 * k, 0.095 * k], 0.022 * k, B.Head, PALETTE.joint));
  for (const s of ['L', 'R']) {
    const S = SIDES[s], c = side(s), hand = s === 'L' ? 'LeftHand' : 'RightHand', toe = s === 'L' ? 'LeftToeBase' : 'RightToeBase';
    parts.push(capsule(P(S.clav), P(S.arm), 0.04 * k, S.clav, PALETTE.body));
    parts.push(ball(P(S.arm), 0.055 * k, S.arm, PALETTE.joint));
    parts.push(capsule(P(S.arm), P(S.fore), 0.047 * k, S.arm, c));
    parts.push(ball(P(S.fore), 0.042 * k, S.fore, PALETTE.joint));
    parts.push(capsule(P(S.fore), P(S.hand), 0.039 * k, S.fore, c));
    parts.push(ball(P(S.hand), 0.032 * k, S.hand, PALETTE.joint));
    parts.push(capsule(P(S.hand), end(hand), 0.034 * k, S.hand, c, 0.55));
    parts.push(ball(P(S.up), 0.068 * k, S.up, PALETTE.joint));
    parts.push(capsule(P(S.up), P(S.knee), 0.07 * k, S.up, c));
    parts.push(ball(P(S.knee), 0.054 * k, S.knee, PALETTE.joint));
    parts.push(capsule(P(S.knee), P(S.foot), 0.052 * k, S.knee, c));
    parts.push(ball(P(S.foot), 0.043 * k, S.foot, PALETTE.joint));
    const heel = [P(S.foot)[0], 0.035 * k, P(S.foot)[2] - skel.dims.heelBack + 0.035 * k], ballP = [P(S.toe)[0], 0.035 * k, P(S.toe)[2]];
    parts.push(capsule(heel, ballP, 0.037 * k, S.foot, c, 1.25));
    parts.push(capsule([ballP[0], 0.026 * k, ballP[2]], [end(toe)[0], 0.026 * k, end(toe)[2] - 0.02 * k], 0.026 * k, S.toe, c, 1.4));
  }
  const geo = mergeGeometries(parts, false);
  parts.forEach(g => g.dispose());
  const mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.62, metalness: 0.04 });
  const mesh = new THREE.SkinnedMesh(geo, mat);
  mesh.name = 'Mannequin';
  mesh.castShadow = true;
  mesh.receiveShadow = false;
  mesh.frustumCulled = false;
  const bones = buildBones(skel);
  const group = new THREE.Group();
  group.name = 'FulcrumCharacter';
  group.add(mesh);
  mesh.add(bones[0]);
  group.updateMatrixWorld(true);
  mesh.bind(new THREE.Skeleton(bones));
  return {
    kind: 'mannequin', group, mesh, bones, skel, height: skel.dims.height,
    applyPose(root, rot) {
      for (let i = 0; i < NB; i++) bones[i].quaternion.set(rot[i * 4], rot[i * 4 + 1], rot[i * 4 + 2], rot[i * 4 + 3]);
      bones[0].position.set(root[0], root[1], root[2]);
    },
    focus: () => bones[0].getWorldPosition(new THREE.Vector3()),
    dispose() { geo.dispose(); mat.dispose(); },
  };
}

export function skeletonLines(skel, color = 0x2f80ed) {
  const pos = new Float32Array((NB - 1 + 5) * 6), geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  const lines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.9 }));
  lines.renderOrder = 10;
  lines.frustumCulled = false;
  const W = { pos: new Float64Array(NB * 3), q: new Float64Array(NB * 4) };
  return {
    object: lines,
    update(root, rot) {
      fk(skel, root, rot, W);
      let o = 0;
      for (let i = 1; i < NB; i++) { pos.set(getV(W.pos, PARENT[i]), o); pos.set(getV(W.pos, i), o + 3); o += 6; }
      for (const name of Object.keys(skel.ends)) {
        const p = getV(W.pos, B[name]), q = getQ(W.q, B[name]), e = new THREE.Vector3(...skel.ends[name]).applyQuaternion(new THREE.Quaternion(q[0], q[1], q[2], q[3]));
        pos.set(p, o); pos.set([p[0] + e.x, p[1] + e.y, p[2] + e.z], o + 3); o += 6;
      }
      geo.attributes.position.needsUpdate = true;
    },
  };
}

function findBones(object) {
  const bones = [];
  object.traverse(o => { if (o.isBone) bones.push(o); });
  return bones;
}

export async function loadCustomCharacter(file, skel) {
  const buf = await file.arrayBuffer(), name = file.name.toLowerCase();
  let object;
  if (name.endsWith('.fbx')) {
    const { FBXLoader } = await import('three/addons/loaders/FBXLoader.js');
    object = new FBXLoader().parse(buf, '');
  } else {
    const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js');
    object = (await new GLTFLoader().parseAsync(buf, '')).scene;
  }
  return makeCustomCharacter(object, skel, file.name);
}

export function makeCustomCharacter(object, skel, label = 'character') {
  object.updateMatrixWorld(true);
  const size0 = new THREE.Box3().setFromObject(object).getSize(new THREE.Vector3()), unit = Math.max(size0.x, size0.y, size0.z) > 20 ? 0.01 : 1;
  if (unit !== 1) object.scale.multiplyScalar(unit);
  object.updateMatrixWorld(true);
  const bones = findBones(object);
  if (bones.length < 10) throw new Error('This file has no humanoid skeleton (found ' + bones.length + ' bones).');
  const index = new Map(bones.map((b, i) => [b, i]));
  const dstBones = bones.map(b => {
    const p = b.getWorldPosition(new THREE.Vector3()), q = b.getWorldQuaternion(new THREE.Quaternion());
    return { name: b.name, parent: index.has(b.parent) ? index.get(b.parent) : -1, p: [p.x, p.y, p.z], q: [q.x, q.y, q.z, q.w] };
  });
  const srcBones = BONES.map((name, i) => ({ name, parent: PARENT[i], p: skel.rest[i].slice(), q: [0, 0, 0, 1] }));
  const setup = hikRetargetSetup(srcBones, dstBones);
  if (!setup.valid) throw new Error('Could not recognise the skeleton. Missing: ' + setup.missing.join(', '));
  const parentInvQ = bones.map(b => index.has(b.parent) ? null : (b.parent ? b.parent.getWorldQuaternion(new THREE.Quaternion()) : new THREE.Quaternion()).invert());
  const hipsIdx = setup.dst.slots.Hips, hipsParent = bones[hipsIdx].parent;
  const parentInv = hipsParent ? hipsParent.matrixWorld.clone().invert() : new THREE.Matrix4();
  const outer = new THREE.Group(), G = setup.G;
  outer.name = 'FulcrumCharacter';
  outer.quaternion.set(-G[0], -G[1], -G[2], G[3]);
  outer.add(object);
  object.traverse(o => { if (o.isMesh) { o.castShadow = true; o.frustumCulled = false; } });
  outer.updateMatrixWorld(true);
  const W = { pos: new Float64Array(NB * 3), q: new Float64Array(NB * 4) }, tq = new THREE.Quaternion(), pq = new THREE.Quaternion(), hp = new THREE.Vector3();
  return {
    kind: 'custom', label, group: outer, object, bones, setup, height: size0.y * unit,
    applyPose(root, rot) {
      fk(skel, root, rot, W);
      const globals = BONES.map((_, i) => ({ p: getV(W.pos, i), q: getQ(W.q, i) }));
      const res = hikRetargetFrame(setup, globals), out = dstWorldRotations(setup, res, dstBones);
      for (let i = 0; i < bones.length; i++) {
        const q = out.q[i], par = dstBones[i].parent;
        tq.set(q[0], q[1], q[2], q[3]);
        if (par >= 0) { const p = out.q[par]; pq.set(p[0], p[1], p[2], p[3]).invert(); }
        else pq.copy(parentInvQ[i]);
        bones[i].quaternion.copy(pq.multiply(tq));
      }
      if (out.hipsP) bones[hipsIdx].position.copy(hp.set(out.hipsP[0], out.hipsP[1], out.hipsP[2]).applyMatrix4(parentInv));
    },
    focus: () => bones[hipsIdx].getWorldPosition(new THREE.Vector3()),
    dispose() { object.traverse(o => { o.geometry?.dispose?.(); }); },
  };
}
