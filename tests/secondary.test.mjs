import test from 'node:test';
import assert from 'node:assert/strict';
import { applySpringSecondary } from '../secondary.js';

const fixture = (fps = 30, nb = 2) => {
  const n = fps*2+1, q = new Float64Array(n*nb*4);
  for (let f=0; f<n; f++) for (let i=0; i<nb; i++) {
    const angle = .8 * Math.sin(f/fps*3);
    q.set([0, Math.sin(angle/2), 0, Math.cos(angle/2)], (f*nb+i)*4);
  }
  return { q, nb, n, names: ['head', 'hand_l'], fps };
};
const distance = (a, b) => 2*Math.acos(Math.min(1, Math.abs(a.reduce((s, x, k) => s+x*b[k], 0))))*180/Math.PI;

test('spring motion follows time consistently across 30 and 60 FPS', () => {
  const a=fixture(30,1), b=fixture(60,1);
  applySpringSecondary({ ...a, strength:.7, delaySeconds:.075 });
  applySpringSecondary({ ...b, strength:.7, delaySeconds:.075 });
  for (let f=0; f<a.n; f++) assert.ok(distance(a.q.slice(f*4,f*4+4), b.q.slice(f*8,f*8+4)) < .08);
});

test('range endpoints and every frame outside range remain byte-exact', () => {
  const x=fixture(), before=x.q.slice(), a=12, b=44;
  const report=applySpringSecondary({ ...x, a, b });
  assert.equal(report.frames, b-a-1);
  assert.ok(x.q.some((v,k) => v!==before[k]));
  for (let f=0; f<x.n; f++) if (f<=a || f>=b) assert.deepEqual(x.q.slice(f*8,f*8+8),before.slice(f*8,f*8+8));
});

test('spring output is deterministic, normalized and angle-bounded', () => {
  const a=fixture(), b=fixture(), source=a.q.slice(), options={ strength:1, damping:.1, stiffness:600, maxAngle:3 };
  const report=applySpringSecondary({ ...a, ...options });
  applySpringSecondary({ ...b, ...options });
  assert.deepEqual(a.q,b.q);
  assert.ok(report.maxAppliedAngle <= 3+1e-10);
  for (let k=0; k<a.q.length; k+=4) {
    assert.ok(Math.abs(Math.hypot(...a.q.slice(k,k+4))-1)<1e-12);
    assert.ok(distance(a.q.slice(k,k+4),source.slice(k,k+4))<=3+1e-8);
  }
});

test('preserved bones remain exact and controller-specific disabling works', () => {
  const x=fixture(), before=x.q.slice();
  const report=applySpringSecondary({ ...x, preserve:['hand_l'], controllers:{ head:{ strength:0 } } });
  assert.deepEqual(x.q,before);
  assert.equal(report.joints,0);
  const r=applySpringSecondary({ ...x, preserve:[1], bones:['head'] });
  assert.equal(r.joints,1);
  for(let f=0; f<x.n; f++) assert.deepEqual(x.q.slice(f*8+4,f*8+8),before.slice(f*8+4,f*8+8));
});

test('nonfinite settings use defaults and extreme values stay finite', () => {
  const x=fixture();
  const r=applySpringSecondary({ ...x, strength:NaN, damping:-3, stiffness:Infinity, delaySeconds:-3, maxAngle:999, fps:NaN });
  assert.equal(r.settings.damping,0);
  assert.equal(r.settings.maxAngle,180);
  assert.equal(r.settings.delaySeconds,0);
  assert.ok(x.q.every(Number.isFinite));
});
