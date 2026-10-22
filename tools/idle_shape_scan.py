#!/usr/bin/env python3
"""Which ROMs of a collection hold the wait-loop shapes the skips do not prove
on their own (they write every lap), to judge whether a generic decoder for a
shape pays across games or only one would use it:

  accum    [LDA #0 / TCD] [LDA f / BNE +6] LDA x / ADC y / STA x / BRA head
           (the Konami 65816 vsync wait: x += y each lap; the optional flag
           test is Parodius's)
  counter  INC abs / BRA head, INC dp / BRA head (a lap counter: +1 a lap)

A byte match is only a candidate (data can match too); the BRA must land on
the loop's first byte. Prints one line per ROM with a hit, the ROM offsets.

  idle_shape_scan.py <dir or rom>... [--shape accum|counter|all]
"""
import argparse, os, re, sys

SHAPES = {
    'accum': re.compile(rb'(?:\x22...)?(?:\xa9\x00\x00\x5b)?(?:\xa5.\xd0\x06)?\xa5(.)\x65.\x85\1\x80(.)', re.S),
    'counter': re.compile(rb'(?:\xee..\x80\xfb)|(?:\xe6.\x80\xfc)', re.S),
}


def hits(d, shape):
    out = []
    for m in SHAPES[shape].finditer(d):
        rel = m.group(0)[-1]
        rel = rel - 256 if rel >= 128 else rel
        if m.end() + rel == m.start():          # the BRA goes back to the loop's first byte
            out.append(m.start())
    return out


def roms(paths):
    for p in paths:
        if os.path.isdir(p):
            for root, _, files in os.walk(p):
                for f in sorted(files):
                    if f.lower().endswith(('.sfc', '.smc')):
                        yield os.path.join(root, f)
        else:
            yield p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--shape', default='all', choices=['accum', 'counter', 'all'])
    a = ap.parse_args()
    shapes = list(SHAPES) if a.shape == 'all' else [a.shape]
    n = {s: 0 for s in shapes}
    total = 0
    for r in roms(a.paths):
        d = open(r, 'rb').read()
        if len(d) % 0x8000 == 512:
            d = d[512:]                          # copier header
        total += 1
        for s in shapes:
            h = hits(d, s)
            if h:
                n[s] += 1
                print('%-7s %s: %s' % (s, os.path.basename(r), ' '.join('%06x' % o for o in h[:6])))
    print('%d ROMs: %s' % (total, ', '.join('%s %d' % kv for kv in n.items())), file=sys.stderr)


if __name__ == '__main__':
    main()
