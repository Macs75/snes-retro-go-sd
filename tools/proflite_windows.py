#!/usr/bin/env python3
"""One row per PROFLITE window (make PROFLITE=1 device log): speed (emulated
over wall time), late frames, audio silence; with a second log, the two side
by side, paired by window index, and the totals of both.

  proflite_windows.py new.log [old.log]
"""
import re
import sys


def windows(path):
    rows, cur = [], None
    for line in open(path, errors='replace'):
        m = re.search(r'PROFLITE: frames=(\d+) shown=\d+ late=(\d+) of \d+ wall_ms=(\d+)', line)
        if m:
            cur = {'f': int(m[1]), 'late': int(m[2]), 'wall': int(m[3]), 'sil': 0}
            rows.append(cur)
            continue
        m = re.search(r'PROFLITE: audio silence=(\d+) ms', line)
        if m and cur:
            cur['sil'] = int(m[1])
    return rows


def speed(r):
    return 100.0 * r['f'] / 60.0988 * 1000 / r['wall']


def total(rows):
    f = sum(r['f'] for r in rows)
    w = sum(r['wall'] for r in rows)
    late = sum(r['late'] for r in rows)
    return ('speed=%.1f%% late=%d/%d (%.0f%%) silence=%d ms (%d ms after the first window), '
            'windows with silence %d, under 95%% speed %d' %
            (100.0 * f / 60.0988 * 1000 / w, late, f, 100.0 * late / f, sum(r['sil'] for r in rows),
             sum(r['sil'] for r in rows[1:]), sum(1 for r in rows[1:] if r['sil']),
             sum(1 for r in rows if speed(r) < 95)))


def main():
    a = windows(sys.argv[1])
    b = windows(sys.argv[2]) if len(sys.argv) > 2 else None
    print('win  speed%  late silence_ms' + ('  |  old: speed%  late silence_ms' if b else ''))
    for i, r in enumerate(a):
        s = '%3d %7.1f %5d %8d' % (i + 1, speed(r), r['late'], r['sil'])
        if b and i < len(b):
            o = b[i]
            s += '  |  %7.1f %5d %8d' % (speed(o), o['late'], o['sil'])
        print(s)
    print('new: ' + total(a))
    if b:
        print('old: ' + total(b))


if __name__ == '__main__':
    main()
