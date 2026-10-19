#!/usr/bin/env python3
"""One row per PROF window of an SA-1 game's device PERF log (make PERF=1).

Columns, cycles as % of the frame budget: speed (emulated over wall time), busy,
the heaviest frame, the cpu/ppu/apu/dma split and sa1 (the SA-1's cycles: sa1=,
gsu= in older logs), then per emulated frame: 65816 opcodes, SA-1 instructions run and
skipped asleep, SA-1 wakes, the cart-path reads of each CPU (PROF/sa1), the
65816 idle laps skipped, and late frames.

  prof_sa1_windows.py snes_KirbySS_1_..._PERF.log [--sort sa1]
"""
import argparse, re

PATS = [
    (r'PROF/frame: emulated=(\d+) shown=\d+ wall_ms=(\d+)', ('nfr', 'wall')),
    (r'budget=(\d+) busy=(\d+) heaviest=(\d+)', ('budget', 'busy', 'heavy')),
    (r'PROF/emu: total=(\d+) \[cpu=(\d+) ppu=(\d+) apu=(\d+) dma=(\d+) (?:gsu|sa1)=(\d+)\]',
     ('tot', 'cpu', 'ppu', 'apu', 'dma', 'sa1')),
    (r'PROF/ops: ops=(\d+)', ('ops',)),
    (r'PROF/sa1: ops=(\d+) skipped=(\d+) wakes=(\d+) cart_reads cpu=(\d+) sa1=(\d+)',
     ('s_ops', 's_skip', 'wakes', 'rd_cpu', 'rd_sa1')),
    (r'PROF/idle: site=\S+ mem=\d+ laps=(\d+)', ('laps',)),
    (r'PROF/pace: late=(\d+)', ('late',)),
]


def windows(path):
    out, cur = [], {}
    for line in open(path, errors='replace'):
        if line.startswith('PROF/frame:') and cur:
            out.append(cur)
            cur = {}
        for pat, keys in PATS:
            m = re.search(pat, line)
            if m:
                cur.update(zip(keys, map(int, m.groups())))
    if cur:
        out.append(cur)
    return [w for w in out if 'budget' in w and 'tot' in w]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('log')
    ap.add_argument('--sort', help='sort rows by this column (descending)')
    a = ap.parse_args()
    rows = []
    for i, w in enumerate(windows(a.log)):
        b = w['budget']
        pct = lambda k: 100.0 * w.get(k, 0) / b
        r = {'win': i + 1, 'speed': 100.0 * w['nfr'] / 60.0988 * 1000 / w['wall'] if w.get('wall') else 0,
             'busy': pct('busy'), 'heavy': pct('heavy'), 'cpu': pct('cpu'), 'ppu': pct('ppu'),
             'apu': pct('apu'), 'dma': pct('dma'), 'sa1': pct('sa1')}
        for k in ('ops', 's_ops', 's_skip', 'wakes', 'rd_cpu', 'rd_sa1', 'laps', 'late'):
            r[k] = w.get(k, 0)
        rows.append(r)
    if a.sort:
        rows.sort(key=lambda r: -r[a.sort])
    print(' win speed%  busy% heavy%   cpu%  ppu%  apu%  dma%  sa1% |  ops65 sa1ops sa1skip wakes rd_cpu rd_sa1  laps late')
    for r in rows:
        print('%4d %6.1f %6.1f %6.1f %6.1f %5.1f %5.1f %5.1f %5.1f | %6d %6d %7d %5d %6d %6d %5d %4d' % (
            r['win'], r['speed'], r['busy'], r['heavy'], r['cpu'], r['ppu'], r['apu'], r['dma'], r['sa1'],
            r['ops'], r['s_ops'], r['s_skip'], r['wakes'], r['rd_cpu'], r['rd_sa1'], r['laps'], r['late']))
    n = len(rows)
    if n:
        avg = lambda k: sum(r[k] for r in rows) / n
        print('mean %6.1f %6.1f %6.1f %6.1f %5.1f %5.1f %5.1f %5.1f | %6d %6d %7d %5d %6d %6d %5d %4d' % tuple(
            [avg(k) for k in ('speed', 'busy', 'heavy', 'cpu', 'ppu', 'apu', 'dma', 'sa1')] +
            [int(avg(k)) for k in ('ops', 's_ops', 's_skip', 'wakes', 'rd_cpu', 'rd_sa1', 'laps', 'late')]))


if __name__ == '__main__':
    main()
