#!/usr/bin/env python3
"""Which SA-1 instructions the Thumb-2 engine leaves to C, from host SA1_PCHIST
files (one per game / recording): the dispatch tables of the generated
src/snes/thumb2/sa1_engine.S say, per m / x, which opcodes go to .Lfb. Prints
the opcodes run in C, as % of each file's SA-1 instructions, the largest share
first (--all: every opcode, marked C or asm). Emulation mode (e = 1) always
runs in C and is shown as its own row.

  sa1_engine_coverage.py pch_kss.txt pch_smrpg_rec.txt ... [--min 0.02] [--all]
"""
import argparse
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def tables():
    tab, cur = {}, None
    for line in open(os.path.join(ROOT, 'src/snes/thumb2/sa1_engine.S')):
        m = re.match(r'@ m=(\d) x=(\d)', line)
        if m:
            cur = (int(m[1]), int(m[2]))
            tab[cur] = []
        elif cur and line.strip().startswith('.word'):
            tab[cur].append('.Lfb' in line)
    return tab


def hist(path):
    ops, total = {}, 0
    for line in open(path):
        if line.startswith('#'):
            continue
        f = line.split()
        n, op, m, x, e = int(f[1]), int(f[2], 16), int(f[4]), int(f[5]), int(f[6])
        k = ('e', 0, 0) if e else (op, m, x)
        ops[k] = ops.get(k, 0) + n
        total += n
    return ops, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--min', type=float, default=0.0)
    ap.add_argument('--all', action='store_true')
    a = ap.parse_args()
    tab = tables()
    hs = [hist(p) for p in a.files]
    keys = set().union(*(h[0] for h in hs))
    rows = []
    for k in keys:
        fb = k[0] == 'e' or tab[(k[1], k[2])][k[0]]
        if not fb and not a.all:
            continue
        sh = [100.0 * h[0].get(k, 0) / max(1, h[1]) for h in hs]
        if max(sh) >= a.min:
            rows.append((max(sh), k, fb, sh))
    rows.sort(reverse=True)
    print('op  m x  ' + '  '.join('%10s' % os.path.basename(p)[:10] for p in a.files))
    for _, k, fb, sh in rows:
        name = 'emul ' if k[0] == 'e' else '%02x  %d %d' % k
        print('%s %s ' % (name, 'C  ' if fb else 'asm') + '  '.join('%9.3f%%' % s for s in sh))
    for i, h in enumerate(hs):
        c = sum(n for k, n in h[0].items() if k[0] == 'e' or tab[(k[1], k[2])][k[0]])
        print('%s: %d instructions, %.2f%% in C' % (a.files[i], h[1], 100.0 * c / max(1, h[1])))


if __name__ == '__main__':
    main()
