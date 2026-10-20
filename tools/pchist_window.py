#!/usr/bin/env python3
"""The instructions run in a window of frames, from two host PC histograms
(CPU_PCHIST / SA1_PCHIST) of the same replay or attract stopped at its start
and at its end (SNES_FRAMES): their difference, hottest first, per frame, as
"bank:addr per-frame opcode cum% m x e dl taken-per-frame", then the same
summed per opcode.

  pchist_window.py <hist at start> <hist at end> <frames in between> [--top 60]
"""
import argparse


def load(path):
    h = {}
    for line in open(path):
        if line.startswith('#'):
            continue
        f = line.split()
        key = (f[0],) + tuple(f[4:8])
        n, op, taken = int(f[1]), f[2], int(f[8])
        o = h.get(key)
        h[key] = (n + (o[0] if o else 0), op, taken + (o[2] if o else 0))
    return h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('start')
    ap.add_argument('end')
    ap.add_argument('frames', type=int)
    ap.add_argument('--top', type=int, default=60)
    a = ap.parse_args()
    s, e = load(a.start), load(a.end)
    rows = []
    for k, (n, op, t) in e.items():
        n0, _, t0 = s.get(k, (0, op, 0))
        if n > n0:
            rows.append((n - n0, k, op, t - t0))
    rows.sort(reverse=True)
    total = sum(r[0] for r in rows)
    print('# %d instructions in %d frames, %.0f a frame' % (total, a.frames, total / a.frames))
    cum = 0
    for n, k, op, t in rows[:a.top]:
        cum += n
        print('%s %9.1f %s %6.2f %s %s %s %s %9.1f' % (k[0], n / a.frames, op, 100.0 * cum / total, *k[1:],
                                                     t / a.frames))
    ops = {}
    for n, k, op, t in rows:
        ops[op] = ops.get(op, 0) + n
    print('# per opcode')
    for op, n in sorted(ops.items(), key=lambda x: -x[1])[:20]:
        print('%s %9.1f %5.2f%%' % (op, n / a.frames, 100.0 * n / total))


if __name__ == '__main__':
    main()
