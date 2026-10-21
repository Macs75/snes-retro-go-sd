#!/usr/bin/env python3
"""Two device PERF logs of the same SA-1 replay, window by window (paired by
index): the budget summary of each (windows over 100/110/120% busy, the worst,
mean speed, late frames, frames drawn, the lowest real fps and the windows
under 20 / 28 fps), the mean change of busy and its split after window 1 (the
start-up), then the heaviest windows of either log and every
window that got heavier by more than --noise points, with busy and its
cpu/ppu/sa1 split as % of the frame budget, the 65816 opcodes and SA-1
slices a frame, and the frames drawn of each window (the PPU cost is per
emulated frame, so it rises with the frames drawn).

  prof_sa1_compare.py old.log new.log [--top 12] [--noise 1.0]
"""
import argparse, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prof_sa1_windows import windows   # noqa: E402


def rows(path):
    out = []
    for w in windows(path):
        b = w['budget']
        out.append({'busy': 100.0 * w['busy'] / b, 'cpu': 100.0 * w['cpu'] / b, 'ppu': 100.0 * w['ppu'] / b,
                    'sa1': 100.0 * w['sa1'] / b, 'ops': w.get('ops', 0), 'late': w.get('late', 0),
                    'slices': w.get('slices', 0), 'shown': w.get('shown', 0),
                    'fps': 1000.0 * w.get('shown', 0) / w['wall'] if w.get('wall') else 0,
                    'speed': 100.0 * w['nfr'] / 60.0988 * 1000 / w['wall'] if w.get('wall') else 0})
    return out


def summary(tag, rs):
    busy = [r['busy'] for r in rs]
    fps = [r['fps'] for r in rs]
    low = min(range(len(rs)), key=lambda i: fps[i])
    return ('%s: %d windows, over 100%% %d, over 110%% %d, over 120%% %d, worst %.1f%%, mean speed %.1f%%, late %d, '
            'drawn %d, lowest %.1f fps (window %d), windows under 20 fps %d, under 28 fps %d' % (tag, len(rs), sum(b > 100 for b in busy), sum(b > 110 for b in busy),
                          sum(b > 120 for b in busy), max(busy), sum(r['speed'] for r in rs) / len(rs),
                          sum(r['late'] for r in rs), sum(r['shown'] for r in rs), fps[low], low + 1,
                          sum(f < 20 for f in fps), sum(f < 28 for f in fps)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('old')
    ap.add_argument('new')
    ap.add_argument('--top', type=int, default=12)
    ap.add_argument('--noise', type=float, default=1.0)
    a = ap.parse_args()
    x, y = rows(a.old), rows(a.new)
    n = min(len(x), len(y))
    print(summary('old', x[:n]))
    print(summary('new', y[:n]))
    d = [y[i]['busy'] - x[i]['busy'] for i in range(n)]
    print('windows lighter %d, heavier by more than %.1f points %d, mean change %+.1f points' % (
        sum(v < -a.noise for v in d), a.noise, sum(v > a.noise for v in d), sum(d) / n))
    # window 1 holds the start-up (the replay load is charged to it), so it is left out
    mc = lambda k: sum(y[i][k] - x[i][k] for i in range(1, n)) / max(1, n - 1)
    print('mean change after window 1: busy %+.1f, cpu %+.1f, ppu %+.1f, sa1 %+.1f points' % (
        mc('busy'), mc('cpu'), mc('ppu'), mc('sa1')))
    hdr = (' win   busy old -> new (change)  |  cpu old -> new   ppu old -> new   sa1 old -> new  | ops65 old -> new'
           ' | slices old -> new | drawn')
    fmt = ('%4d  %6.1f -> %6.1f (%+5.1f)  | %5.1f -> %5.1f    %5.1f -> %5.1f    %5.1f -> %5.1f  | %5d -> %5d'
           ' | %5d -> %5d | %3d -> %3d')
    line = lambda i: fmt % (i + 1, x[i]['busy'], y[i]['busy'], d[i], x[i]['cpu'], y[i]['cpu'], x[i]['ppu'],
                            y[i]['ppu'], x[i]['sa1'], y[i]['sa1'], x[i]['ops'], y[i]['ops'], x[i]['slices'],
                            y[i]['slices'], x[i]['shown'], y[i]['shown'])
    print('\nheaviest windows (of either log)')
    print(hdr)
    for i in sorted(range(n), key=lambda i: -max(x[i]['busy'], y[i]['busy']))[:a.top]:
        print(line(i))
    worse = sorted((i for i in range(n) if d[i] > a.noise), key=lambda i: -d[i])
    print('\nheavier by more than %.1f points: %d' % (a.noise, len(worse)))
    if worse:
        print(hdr)
        for i in worse:
            print(line(i))


if __name__ == '__main__':
    main()
