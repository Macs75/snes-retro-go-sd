#!/usr/bin/env python3
"""How many of a game's SPC700 opcodes the Thumb-2 engine runs natively.

The device engine (src/snes/thumb2/spc_thumb2.S) handles an opcode itself or
returns it to the C dispatcher (spc_doOpcode) through .L_noteb. This weights
the engine's dispatch table by an APU_HIST histogram (src/snes/apu.c) and lists
the opcodes that fall back to C, hottest first.

  spc_native_coverage.py game.hist [--asm src/snes/thumb2/spc_thumb2.S] [--tracing src/snes/tracing.c]
"""
import argparse, os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spc_hist_report import load_tables


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('hist')
    ap.add_argument('--asm', default='src/snes/thumb2/spc_thumb2.S')
    ap.add_argument('--tracing', default='src/snes/tracing.c')
    a = ap.parse_args()
    src = open(a.asm).read()
    tbl = src.split('.Lspc_dispatch_table:', 1)[1]
    ent = re.findall(r'\.word\s+(\.L\w+)\s*\+\s*1', tbl)[:256]
    assert len(ent) == 256, len(ent)
    native = [e != '.L_noteb' for e in ent]
    names, _ = load_tables(a.tracing)

    per_op, frames = [0] * 256, 1
    for line in open(a.hist):
        if line.startswith('#'):
            m = re.match(r'# frames \d+\.\.\d+ \((\d+)\)', line)
            if m: frames = int(m.group(1))
            continue
        p = line.split()
        per_op[int(p[2], 16)] += int(p[1])
    tot = sum(per_op)
    fb = sum(n for op, n in enumerate(per_op) if not native[op])
    print('%d opcodes (%.0f a frame); native table: %d of 256 opcodes' % (tot, tot / frames, sum(native)))
    print('falling back to C: %d (%.1f%%, %.0f a frame)\n' % (fb, 100.0 * fb / tot, fb / frames))
    print('fallback opcodes, hottest first:')
    for op in sorted(range(256), key=lambda o: -per_op[o]):
        if per_op[op] and not native[op]:
            print('  %02x %-22s %10d %5.1f%%' % (op, names[op].split(' $')[0].split(' #')[0], per_op[op],
                                               100.0 * per_op[op] / tot))


if __name__ == '__main__':
    main()
