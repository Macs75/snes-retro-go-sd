#!/usr/bin/env python3
"""Two device PROF/PERF logs of the same replay, window by window (paired by
index, docs/perf-lessons.md: never median against median): the 65816's
cycles (PROF/emu cpu=), the idle span runner's cycles and calls, the laps it
skipped, busy and the frames shown; then the mean of each over the windows
both have, without the first (the boot).

  prof_pair.py <old.log> <new.log> [--rows]
"""
import argparse, re


def windows(path):
    out, cur = [], None
    for line in open(path, errors='replace'):
        if line.startswith('PROF/emu:'):
            cur = {}
            out.append(cur)
            m = re.search(r'cpu=(\d+) ppu=(\d+) apu=(\d+)', line)
            cur['cpu'], cur['ppu'], cur['apu'] = (int(x) for x in m.groups())
        if cur is None:
            continue
        m = re.search(r'span_runner=(\d+) cycles in (\d+) calls', line)
        if m:
            cur['span'], cur['calls'] = int(m.group(1)), int(m.group(2))
        m = re.search(r'PROF/idle: site=\S+ .*laps=(\d+) skipped_dots=(\d+)', line)
        if m:
            cur['laps'], cur['skipdots'] = int(m.group(1)), int(m.group(2))
        m = re.search(r'PROF/frame: emulated=(\d+) shown=(\d+)', line)
        if m:
            cur['emu'], cur['shown'] = int(m.group(1)), int(m.group(2))
        m = re.search(r'PROF/budget: budget=(\d+) busy=(\d+)', line)
        if m:
            cur['busy'] = 100.0 * int(m.group(2)) / int(m.group(1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('old')
    ap.add_argument('new')
    ap.add_argument('--rows', action='store_true', help='every window, not only the means')
    a = ap.parse_args()
    o, n = windows(a.old), windows(a.new)
    k = min(len(o), len(n))
    keys = ('cpu', 'ppu', 'apu', 'span', 'calls', 'laps', 'shown', 'busy')
    if a.rows:
        print('win ' + ' '.join('%17s' % x for x in keys))
        for i in range(k):
            print('%3d ' % (i + 1) + ' '.join(
                '%8s>%-8s' % (('%.0f' % o[i][x]) if x in o[i] else '-', ('%.0f' % n[i][x]) if x in n[i] else '-')
                for x in keys))
    print('%d and %d windows, %d paired (window 1, the boot, left out of the means)' % (len(o), len(n), k))
    for x in keys:
        pairs = [(o[i][x], n[i][x]) for i in range(1, k) if x in o[i] and x in n[i]]
        if not pairs:
            continue
        mo = sum(p[0] for p in pairs) / len(pairs)
        mn = sum(p[1] for p in pairs) / len(pairs)
        print('  %-6s %12.1f -> %12.1f  (%+.1f%%)' % (x, mo, mn, 100.0 * (mn - mo) / mo if mo else 0.0))
    for name, w in (('old', o), ('new', n)):
        sh = [x['shown'] for x in w[1:k] if 'shown' in x]
        em = [x['emu'] for x in w[1:k] if 'emu' in x]
        if sh:
            print('  %s: shown %d of %d, lowest window %d' % (name, sum(sh), sum(em), min(sh)))


if __name__ == '__main__':
    main()
