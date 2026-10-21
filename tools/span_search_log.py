#!/usr/bin/env python3
"""The idle skip's span searches of a host replay: how many pcs, how often
each was searched again, and why its earlier reject did not stop the search
(the slot was empty, held another pc -- a hash collision -- or this pc from
an older 256-frame epoch). With a device log, the searches per window of
both (PROF/span), to show the host replays the device's.

The trace is a temporary host-only fprintf at the end of idle_span_search,
one line per search: "s_frame pc24 slot rejected" -- the frame, the pc
(hex), what its reject slot held before (hex; with the 4-way table, the
newest entry of its set) and 1 if the search ended rejected.

  span_search_log.py <SPAN_LOG file> [--device <PERF log>] [--window 120] [--top 12]
"""
import argparse, collections, re


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--device')
    ap.add_argument('--window', type=int, default=120)
    ap.add_argument('--top', type=int, default=12)
    a = ap.parse_args()
    rows = []
    for l in open(a.log):
        f = l.split()
        if len(f) == 4:
            rows.append((int(f[0]), int(f[1], 16), int(f[2], 16), f[3] == '1'))
    n = len(rows)
    pcs = collections.Counter(r[1] for r in rows)
    print('searches %d, rejected %d, distinct pcs %d' % (n, sum(r[3] for r in rows), len(pcs)))
    why = collections.Counter()
    seen = set()
    for fr, pc, slot, _ in rows:
        if pc not in seen:
            why['first search of this pc'] += 1
        elif slot == 0xffffffff:
            why['again: slot empty'] += 1
        elif slot & 0xffffff == pc:
            why['again: older epoch (256-frame reset)'] += 1
        else:
            why['again: slot held another pc (collision)'] += 1
        seen.add(pc)
    for k, v in why.most_common():
        print('  %-42s %5d  %5.1f%%' % (k, v, 100.0 * v / n))
    print('most searched pcs:')
    for pc, c in pcs.most_common(a.top):
        print('  %06x  %5d' % (pc, c))
    if a.device:
        dev = [int(m.group(1)) for m in re.finditer(r'PROF/span: .*?searches=(\d+) cycles=\d+ in the window', open(a.device, errors='replace').read())]
        host = collections.Counter((fr - 1) // a.window for fr, _, _, _ in rows)
        same = sum(dev[i] == host.get(i, 0) for i in range(len(dev)))
        print('device %d searches in %d windows, host %d in the same frames; windows with the same count %d of %d' % (
            sum(dev), len(dev), sum(host.get(i, 0) for i in range(len(dev))), same, len(dev)))


if __name__ == '__main__':
    main()
