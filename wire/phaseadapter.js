// Thin adapter over Fulcrum's own phase segmentation. tastebridge.js is being renamed to phases.js;
// load whichever exists so facts.js works before and after the rename. Only the phase code is used
// (segmentPhases, phaseSignals); the old taste I/O is not.
let mod = null, source = null;
for (const path of ['../phases.js', '../tastebridge.js']) {
  try { mod = await import(path); if (typeof mod.segmentPhases === 'function') { source = path.slice(3); break; } } catch {}
  mod = null;
}

export const PHASES_SOURCE = source;
export const segmentPhases = mod ? mod.segmentPhases : null;
export const phaseSignals = mod ? mod.phaseSignals ?? null : null;
export const PHASES = mod?.PHASES ?? ['locomotion', 'in_place', 'flight', 'vault', 'plant', 'slide', 'kneel', 'crouch', 'prone', 'rise', 'stand', 'upper_body', 'freeform'];
