import test from 'node:test';
import assert from 'node:assert/strict';
import { json, exists, near, nearVec } from './helpers.mjs';
import { footMarkers } from '../contacts.js';
import { clampContactSize, contactGizmoScale, hikFootMarkers, hikFloorContact, gizmoFootMarkers, CONTACT_SIZE_RANGE } from '../re/hik/contact.js';

const F = 'C:/re_work/hik/results/contact_size.json';

test('contact_size.json: HumanIK 2016 output does not change with ContactSize 0..200 % in any mode', { skip: exists(F) ? false : 'missing contact_size.json' }, () => {
  const rows = json(F);
  assert.equal(rows.length, 5);
  for (const r of rows) {
    nearVec(r.ankle, rows[0].ankle, 0);
    nearVec(r.wrist, rows[0].wrist, 0);
    nearVec(r.marker0, rows[0].marker0, 0);
  }
});

test('hikFootMarkers ignores contactSize, plain footMarkers would scale', () => {
  const ankle = { p: [10, 9, 2], q: [0.1, 0.2, 0, Math.sqrt(1 - 0.05)] };
  const ref = hikFootMarkers(ankle, {});
  for (const cs of [0.01, 50, 200, 1000]) {
    const m = hikFootMarkers(ankle, { contactSize: cs });
    for (const k of Object.keys(ref)) nearVec(m[k], ref[k], 1e-12);
  }
  const scaled = footMarkers(ankle, { contactSize: 50 });
  assert.ok(Math.abs(scaled.frontIn[2] - ref.frontIn[2]) > 1);
});

test('hikFloorContact gives the same foot for any contactSize', () => {
  const foot = { ankle: { p: [10, 6, 0], q: [0.17, 0, 0, Math.sqrt(1 - 0.0289)] } };
  const a = hikFloorContact(foot, { bottom: 8 }), b = hikFloorContact(foot, { bottom: 8, contactSize: 300 });
  nearVec(a.ankle.p, b.ankle.p, 1e-12);
  nearVec(a.ankle.q, b.ankle.q, 1e-12);
});

test('ContactSize is a display scale: clamp to [0.01, 1000] % and value / 100 on all axes', () => {
  assert.deepEqual(CONTACT_SIZE_RANGE, [0.01, 1000]);
  near(clampContactSize(5000), 1000, 0);
  near(clampContactSize(-3), 0.01, 0);
  near(clampContactSize(NaN), 100, 0);
  nearVec(contactGizmoScale(150), [1.5, 1.5, 1.5], 1e-12);
  const ankle = { p: [0, 8, 0], q: [0, 0, 0, 1] };
  const g = gizmoFootMarkers(ankle, { contactSize: 200 }), r = hikFootMarkers(ankle, {});
  for (const k of Object.keys(r)) nearVec(g[k], [r[k][0] * 2, 8 + (r[k][1] - 8) * 2, r[k][2] * 2], 1e-12);
});
