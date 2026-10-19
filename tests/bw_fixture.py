import sys, math, json, random, os
sys.path.insert(0, r'C:\re_work\maya\tests')
import bw_model as M
TPS = M.TPS
FPS = 24.0
def ticks(fr): return round(fr * TPS / FPS)
def lin(keys):
    ts = sorted(keys)
    def f(tk):
        fr = tk * FPS / TPS
        if fr <= ts[0][0]: return ts[0][1]
        if fr >= ts[-1][0]: return ts[-1][1]
        for (a, va), (b, vb) in zip(ts, ts[1:]):
            if a <= fr <= b:
                return va + (vb - va) * (fr - a) / (b - a)
    return f
random.seed(3)
sigs = {
 'impulse': [(i, 1.0 if i == 50 else 0.0) for i in range(101)],
 'step': [(i, 1.0 if i >= 50 else 0.0) for i in range(101)],
 'ramp_noise': [(i, 0.05 * i + random.uniform(-1, 1)) for i in range(101)],
 'short': [(i, (i % 3) * 1.0) for i in range(8)],
}
for hz in (1, 2, 4, 6, 7, 8, 10):
    sigs['sine%d' % hz] = [(i, math.sin(2 * math.pi * hz * i / FPS)) for i in range(241)]
cases = [('cof7_sr24_kof', 7.0, 24.0, True), ('cof3_sr24', 3.0, 24.0, False), ('cof7_sr12_kof', 7.0, 12.0, True), ('cof2_sr30', 2.0, 30.0, False)]
out = {'fps': FPS, 'tps': TPS, 'signals': {k: [list(p) for p in v] for k, v in sigs.items()}, 'cases': [list(c) for c in cases], 'model': {}}
for name, keys in sigs.items():
    for cname, fc, sr, kof in cases:
        pts = M.filt(lin(keys), ticks(keys[0][0]), ticks(keys[-1][0]), fc, sr, kof)
        out['model'][name + ':' + cname] = None if pts is None else [[int(t), v] for t, v in pts]
dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fixtures', 'bw_ref.json')
json.dump(out, open(dst, 'w'))
print('wrote', dst, len(out['model']), 'cases')
