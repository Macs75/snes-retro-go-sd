#!/usr/bin/env python3
"""Write a recording with no input (the pad released every frame), so a ROM's
own intro / attract can be replayed where only replays run (gwemu: make qemu,
host/qemu/run.sh, tools/qemu_vs_host.py). The header needs the ROM's hash as
main_snes.c computes it: replay the ROM once with a recording made with
--hash 0 and pass the hash the "recorded with a different ROM" line names.
--press holds pad bits over a few frames, to get past a screen that waits for
a button (a language select): LakeSnes currentState bits, 0 B, 1 Y, 2 Select,
3 Start, 4-7 Up Down Left Right, 8 A, 9 X, 10 L, 11 R.

  make_idle_rec.py <out.rec> <frames> --hash 0x1234abcd [--pal] [--press frame:count:bits ...]
"""
import argparse
import struct

REC_MAGIC, REC_VERSION = 0x43524E53, 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('frames', type=int)
    ap.add_argument('--hash', type=lambda v: int(v, 0), required=True)
    ap.add_argument('--pal', action='store_true')
    ap.add_argument('--press', action='append', default=[], help='frame:count:bits (0-based frame)')
    a = ap.parse_args()
    pad = [0] * a.frames
    for p in a.press:
        f0, n, bits = (int(v, 0) for v in p.split(':'))
        for i in range(f0, min(f0 + n, a.frames)):
            pad[i] |= bits
    with open(a.out, 'wb') as f:
        f.write(struct.pack('<IHHII', REC_MAGIC, REC_VERSION, 1 if a.pal else 0, a.hash, a.frames))
        f.write(struct.pack('<%dH' % a.frames, *pad))


if __name__ == '__main__':
    main()
