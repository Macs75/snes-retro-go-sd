#!/usr/bin/env python3
"""Write a recording with no input (the pad released every frame), so a ROM's
own intro / attract can be replayed where only replays run (gwemu: make qemu,
host/qemu/run.sh, tools/qemu_vs_host.py). The header needs the ROM's hash as
main_snes.c computes it: replay the ROM once with a recording made with
--hash 0 and pass the hash the "recorded with a different ROM" line names.

  make_idle_rec.py <out.rec> <frames> --hash 0x1234abcd [--pal]
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
    a = ap.parse_args()
    with open(a.out, 'wb') as f:
        f.write(struct.pack('<IHHII', REC_MAGIC, REC_VERSION, 1 if a.pal else 0, a.hash, a.frames))
        f.write(b'\0\0' * a.frames)


if __name__ == '__main__':
    main()
