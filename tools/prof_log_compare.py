#!/usr/bin/env python3
"""Compare device PERF logs of the same recording, window by window and by phase.

Each PERF log (make PERF=1, a replay of a .rec) has one block of PROF/ lines per
120 emulated frames, tagged by the REC/hash line that precedes it. This lines
the windows of two or more logs up by frame and prints, as % of the frame
budget: busy, the heaviest single frame, the cpu/ppu/apu/dma/gsu split, late
frames and the audio step; per phase (frame ranges given with --phase) and for
chosen windows. It also checks the REC hashes agree.

  prof_log_compare.py old.log new.log [more.log ...] \\
      --phase intro:0:12720 --phase title:12960:13560 --phase gameplay:13680:99999 \\
      --frames 240,9360,12960,13920,16080,17520,18240
"""
import argparse, re

PATS = [
    (r'busy=(\d+) heaviest=(\d+)', ('busy', 'heavy')),
    (r'budget=(\d+)', ('budget',)),
    (r'PROF/emu: total=(\d+) \[cpu=(\d+) ppu=(\d+) apu=(\d+) dma=(\d+) gsu=(\d+)\]',
     ('tot', 'cpu', 'ppu', 'apu', 'dma', 'gsu')),
    (r'PROF/pace: late=(\d+)', ('late',)),
    (r'step_avg=(\d+) step_min=(\d+).*underruns=(\d+)', ('savg', 'smin', 'under')),
]


def parse(path):
    rows, cur, hashes = {}, None, {}
    for line in open(path, errors='replace'):
        m = re.search(r'REC/hash frame=(\d+) (.*)', line)
        if m:
            cur = {'f': int(m.group(1))}
            rows[cur['f']] = cur
            hashes[cur['f']] = re.sub(r'\s+$', '', m.group(2))
            continue
        if cur is None:
            continue
        for pat, keys in PATS:
            m = re.search(pat, line)
            if m:
                cur.update(zip(keys, map(int, m.groups())))
    return {f: r for f, r in rows.items() if 'busy' in r}, hashes


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--phase', action='append', default=[], help='name:first:last (window end frames)')
    ap.add_argument('--frames', default='', help='comma-separated windows to list')
    a = ap.parse_args()
    logs = [parse(p) for p in a.logs]
    names = ['L%d' % i for i in range(len(logs))]
    for n, p in zip(names, a.logs):
        print('%s = %s' % (n, p))
    common = sorted(set.intersection(*(set(r) for r, _ in logs)))
    base = logs[0][0][common[0]].get('budget', 5890300) if common else 5890300
    B = base / 100.0
    ref = logs[0][1]
    for n, (_, h) in zip(names[1:], logs[1:]):
        same = sum(1 for f in ref if f in h and h[f] == ref[f])
        print('REC hashes %s vs L0: %d of %d windows equal' % (n, same, len(ref)))

    def agg(rows, frames, key):
        return sum(rows[f].get(key, 0) for f in frames)

    if a.phase:
        print('\nphase (n windows)   ' + '   '.join('%-34s' % n for n in names))
        print(' ' * 20 + '   '.join('%-34s' % 'busy  gsu   cpu   heaviest  late' for _ in names))
        for spec in a.phase:
            name, lo, hi = spec.split(':')
            fr = [f for f in common if int(lo) <= f <= int(hi)]
            if not fr:
                continue
            cells = []
            for rows, _ in logs:
                k = len(fr)
                cells.append('%5.1f %5.1f %5.1f %7.1f %6d' % (
                    agg(rows, fr, 'busy') / k / B, agg(rows, fr, 'gsu') / k / B, agg(rows, fr, 'cpu') / k / B,
                    max(rows[f]['heavy'] for f in fr) / B, agg(rows, fr, 'late')))
            print('%-10s (%3d)     ' % (name, len(fr)) + '   '.join('%-34s' % c for c in cells))
    if a.frames:
        print('\nframe  ' + '   '.join('%-40s' % (n + ': busy  gsu  heaviest late step_min') for n in names))
        for f in (int(x) for x in a.frames.split(',')):
            if f not in common:
                continue
            cells = []
            for rows, _ in logs:
                r = rows[f]
                cells.append('%5.1f %5.1f %7.1f %4d %6d' % (r['busy'] / B, r['gsu'] / B, r['heavy'] / B,
                                                            r.get('late', 0), r.get('smin', 0)))
            print('%5d  ' % f + '   '.join('%-40s' % c for c in cells))


if __name__ == '__main__':
    main()
