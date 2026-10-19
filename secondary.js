const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
const finite = (x, fallback) => typeof x === 'number' && Number.isFinite(x) ? x : fallback;
const norm = q => { const d = Math.hypot(...q); return d > 1e-12 && Number.isFinite(d) ? q.map(x => x / d) : [0, 0, 0, 1]; };
const mul = (a, b) => [a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1], a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0], a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3], a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];
const inverse = q => [-q[0], -q[1], -q[2], q[3]];
const log = q => { const v = q[3] < 0 ? q.map(x => -x) : q, s = Math.hypot(v[0], v[1], v[2]); return s < 1e-12 ? [0, 0, 0] : v.slice(0, 3).map(x => x * 2 * Math.atan2(s, clamp(v[3], -1, 1)) / s); };
const exp = v => { const a = Math.hypot(...v), s = a < 1e-12 ? 0.5 : Math.sin(a / 2) / a; return norm([v[0]*s, v[1]*s, v[2]*s, Math.cos(a / 2)]); };
const slerp = (a, b, t) => norm(mul(exp(log(norm(mul(b, inverse(a)))).map(x => x*t)), a));
const read = (q, nb, f, i) => norm(Array.from(q.slice((f*nb+i)*4, (f*nb+i)*4+4)));
const settings = x => ({ strength: clamp(finite(x.strength, .35), 0, 1), damping: clamp(finite(x.damping, .72), 0, 4), stiffness: clamp(finite(x.stiffness, 90), 0, 1600), delaySeconds: clamp(finite(x.delaySeconds, .04), 0, 2), maxAngle: clamp(finite(x.maxAngle, 20), 0, 180) });

export function applySpringSecondary({ q, nb, n, names = [], a = 0, b = n - 1, bones = [], strength, damping, stiffness, delaySeconds, fps = 30, maxAngle, preserve = [], controllers = {} }) {
  nb = Math.max(0, Math.floor(finite(nb, 0)));
  n = Math.max(0, Math.floor(finite(n, 0)));
  if (!q || q.length < n*nb*4) throw new Error('Secondary motion requires complete quaternion frames.');
  fps = clamp(finite(fps, 30), 1, 1000);
  a = clamp(Math.floor(finite(a, 0)), 0, Math.max(0, n-1));
  b = clamp(Math.floor(finite(b, n-1)), 0, Math.max(0, n-1));
  const defaults = settings({ strength, damping, stiffness, delaySeconds, maxAngle });
  const report = { frames: Math.max(0, b-a-1), joints: 0, maxAppliedAngle: 0, fps, settings: defaults, controllers: [] };
  if (b-a < 2 || !nb) return report;
  const source = new Float64Array(q);
  const pinned = new Set(preserve.map(x => typeof x === 'number' ? x : String(x).toLowerCase()));
  const steps = Math.ceil(240 / fps), h = 1 / fps / steps;
  for (let i = 0; i < nb; i++) {
    const name = String(names[i] ?? i), key = name.toLowerCase();
    if (pinned.has(i) || pinned.has(key)) continue;
    if (bones.length && !bones.some(x => typeof x === 'number' ? x === i : key.includes(String(x).toLowerCase()))) continue;
    const config = settings({ ...defaults, ...(controllers[name] ?? controllers[i] ?? {}) });
    if (!config.strength || !config.stiffness || !config.maxAngle) continue;
    const sample = frame => { const t = clamp(frame, a, b), lo = Math.floor(t), hi = Math.min(b, lo+1); return slerp(read(source, nb, lo, i), read(source, nb, hi, i), t-lo); };
    let state = read(source, nb, a, i), velocity = [0, 0, 0];
    const drag = 2 * config.damping * Math.sqrt(config.stiffness), bound = config.maxAngle * Math.PI / 180;
    for (let f = a+1; f <= b; f++) {
      for (let step = 0; step < steps; step++) {
        const frame = f-1+(step+1)/steps-config.delaySeconds*fps;
        const error = log(norm(mul(sample(frame), inverse(state))));
        velocity = velocity.map((v, k) => (v + config.stiffness * error[k] * h) / (1 + drag * h));
        const speed = Math.hypot(...velocity);
        if (speed > 100) velocity = velocity.map(x => x * 100 / speed);
        state = norm(mul(exp(velocity.map(x => x*h)), state));
      }
      if (f === b) continue;
      const primary = read(source, nb, f, i), error = log(norm(mul(state, inverse(primary))));
      const angle = Math.hypot(...error), taper = Math.sin(Math.PI * (f-a)/(b-a)) ** 2;
      const gain = Math.min(config.strength*taper, angle > 0 ? bound/angle : 1);
      const output = norm(mul(exp(error.map(x => x*gain)), primary));
      if (output.reduce((sum, x, k) => sum+x*primary[k], 0) < 0) for (let k=0; k<4; k++) output[k] *= -1;
      q.set(output, (f*nb+i)*4);
      report.maxAppliedAngle = Math.max(report.maxAppliedAngle, angle*gain*180/Math.PI);
    }
    report.joints++;
    report.controllers.push({ bone: name, ...config });
  }
  return report;
}
