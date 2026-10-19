import test from 'node:test';
import assert from 'node:assert/strict';
import { json, near, nearVec } from './helpers.mjs';
import { skinPoints, setInfluenceWeight, decompose, composeSRT, matMul, matInverse, isNonRigid } from '../re/maya/skin.js';

const fx = json(new URL('./fixtures/maya_skin.json', import.meta.url));
const nr = json(new URL('./fixtures/maya_skin_nonrigid.json', import.meta.url));
const METHOD = ['linear', 'dq', 'blend'];

function maxError(c) {
  const out = skinPoints({ ...c, method: METHOD[c.method], normalize: c.normalize === 'unit' ? 'interactive' : c.normalize });
  let m = 0;
  out.forEach((p, i) => { m = Math.max(m, Math.hypot(p[0] - c.outObject[i][0], p[1] - c.outObject[i][1], p[2] - c.outObject[i][2])); });
  return m;
}

test('linear skinning matches Maya for normalized, post-normalized and raw weights', () => {
  for (const c of fx.cases.filter(c => c.method === 0)) near(maxError(c), 0, 1e-6, `${c.normalize} scaled=${c.scaled}`);
});

test('dual-quaternion skinning matches Maya, rigid and scaled joints, with and without non-rigid support', () => {
  for (const c of fx.cases.filter(c => c.method === 1)) near(maxError(c), 0, 1e-6, `${c.normalize} scaled=${c.scaled} nonRigid=${c.supportNonRigid}`);
});

test('weight-blended skinning is a per-vertex lerp between linear and DQ', () => {
  for (const c of fx.cases.filter(c => c.method === 2)) near(maxError(c), 0, 1e-6, `${c.normalize} scaled=${c.scaled} nonRigid=${c.supportNonRigid}`);
});

test('raw weights + non-rigid DQ: blended non-rigid matrix is scaled by the weight sum', () => {
  for (const c of fx.cases.filter(c => c.method > 0 && c.supportNonRigid && c.normalize === 'none')) near(maxError(c), 0, 1e-6, `scaled=${c.scaled}`);
});

test('non-rigid DQ fixtures: raw/post weights, uniform, non-uniform, parent scale, shear, gate edges', () => {
  for (const c of nr.cases) near(maxError(c), 0, 5e-6, `${c.label} method=${c.method} ${c.normalize} nonRigid=${c.supportNonRigid}`);
});

test('non-rigid gate: any influence matrix with |scale-1| > 1e-10 or |shear| > 1e-4', () => {
  const by = l => nr.cases.find(c => c.label === l).matrix.some(isNonRigid);
  for (const l of ['uniform', 'nonUniform', 'oneJoint', 'parent', 'shear', 'bindScaledKept']) assert.ok(by(l), l);
  for (const l of ['rigid', 'shearBelowGate', 'scaleBelowGate', 'bindScaledReset']) assert.ok(!by(l), l);
});

test('zero-weight vertices: linear keeps the object point, DQ writes the world point', () => {
  const c = fx.cases.find(c => c.method === 1 && c.normalize === 'post');
  const l = skinPoints({ ...c, method: 'linear', normalize: 'post' })[0];
  nearVec(l, c.points[0], 0);
  nearVec(skinPoints({ ...c, method: 'dq', normalize: 'post' })[0], c.outObject[0], 1e-6);
});

test('skinPercent normalization matches Maya, absolute and relative, with locked influences', () => {
  for (const n of fx.normalize) {
    const w = setInfluenceWeight(n.before, n.target, n.value, n.locks, n.relative);
    nearVec(w, n.after, 1e-9, JSON.stringify(n));
  }
});

test('decompose and composeSRT round-trip a scaled, sheared, rotated matrix', () => {
  const m = composeSRT([1.3, 0.7, 2.1], [0.2, -0.1, 0.4], [0.1825742, 0.3651484, 0.5477226, 0.7302967], [3, -2, 5]);
  const d = decompose(m);
  nearVec(composeSRT(d.scale, d.shear, d.rotation, d.translation), m, 1e-12);
  nearVec(matMul(m, matInverse(m)), [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], 1e-12);
});
