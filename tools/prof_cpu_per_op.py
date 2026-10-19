#!/usr/bin/env python3
"""The 65816's cost per instruction in device PERF logs of the same recording.

Pairs PROF windows by their REC/hash frame, checks the hashes agree, and prints
per window and in total: the cpu figure (cycles per emulated frame), 65816
opcodes run (PROF/ops) and cpu cycles per opcode, for each log. Used to compare
the Thumb-2 engine with the C interpreter (SNES_THUMB2_CPU=0) on one replay.

  prof_cpu_per_op.py engine.log c_only.log [more.log ...]
"""
import re, sys


def windows(path):
    out, cur, h = {}, {}, None
    for line in open(path, errors='replace'):
        m = re.match(r'REC/hash frame=(\d+) (.*)', line)
        if m:
            h = (int(m.group(1)), m.group(2).strip())
        m = re.search(r'budget=(\d+) busy=(\d+)', line)
        if m:
            cur['budget'], cur['busy'] = map(int, m.groups())
        m = re.search(r'PROF/emu: total=(\d+) \[cpu=(\d+)', line)
        if m:
            cur['emu'], cur['cpu'] = map(int, m.groups())
        m = re.search(r'PROF/ops: ops=(\d+)', line)
        if m and h:
            cur['ops'] = int(m.group(1))
            out[h[0]] = dict(cur, hash=h[1])
            cur = {}
    return out


def main():
    logs = [windows(p) for p in sys.argv[1:]]
    frames = sorted(set.intersection(*(set(l) for l in logs)))
    bad = [f for f in frames if len({l[f]['hash'] for l in logs}) > 1]
    print('windows paired: %d, REC/hash mismatches: %d%s' % (len(frames), len(bad), (' first at frame %d' % bad[0]) if bad else ''))
    print('frame  ' + '  '.join('cpu%%  cyc/op  busy%%'.replace('%%', '%') for _ in logs))
    tot = [[0, 0, 0] for _ in logs]
    for f in frames:
        row = []
        for i, l in enumerate(logs):
            w = l[f]
            b = w['budget']
            row.append('%5.1f %7.0f %6.1f' % (100.0 * w['cpu'] / b, w['cpu'] / max(w['ops'], 1), 100.0 * w['busy'] / b))
            tot[i][0] += w['cpu']; tot[i][1] += w['ops']; tot[i][2] += w['busy']
        print('%5d  %s' % (f, '  '.join(row)))
    b = logs[0][frames[0]]['budget']
    n = len(frames)
    print('total  ' + '  '.join('%5.1f %7.0f %6.1f' % (100.0 * c / n / b, c / max(o, 1), 100.0 * bu / n / b) for c, o, bu in tot))


if __name__ == '__main__':
    main()
