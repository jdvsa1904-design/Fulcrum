import { footMarkers, floorContact, FOOT_MARKERS } from '../../contacts.js';

export const CONTACT_SIZE_RANGE = Object.freeze([0.01, 1000]);
export const CONTACT_SIZE_DEFAULT = 100;

export function clampContactSize(v) {
  const x = Number.isFinite(v) ? v : CONTACT_SIZE_DEFAULT;
  return Math.min(CONTACT_SIZE_RANGE[1], Math.max(CONTACT_SIZE_RANGE[0], x));
}

export function contactGizmoScale(contactSize = CONTACT_SIZE_DEFAULT) {
  const s = clampContactSize(contactSize) / 100;
  return [s, s, s];
}

const solverOptions = o => ({ ...o, contactSize: CONTACT_SIZE_DEFAULT });

export function hikFootMarkers(ankle, o = {}, toe = null) {
  return footMarkers(ankle, solverOptions(o), toe);
}

export function hikFloorContact(foot, o = {}) {
  return floorContact(foot, solverOptions(o));
}

export function gizmoFootMarkers(ankle, o = {}) {
  const s = clampContactSize(o.contactSize ?? CONTACT_SIZE_DEFAULT) / 100;
  const m = { ...FOOT_MARKERS, ...o };
  const scaled = { ...o, contactSize: 100, back: m.back * s, middle: m.middle * s, front: m.front * s, inn: m.inn * s, out: m.out * s, bottom: m.bottom * s };
  return footMarkers(ankle, scaled);
}
