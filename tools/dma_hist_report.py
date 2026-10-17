#!/usr/bin/env python3
"""What a game's DMA moves per frame, from a host DMA_HIST file (src/snes/dma.c).

One row per kind: general DMA or HDMA, source bank, B-bus target, mode and
(general DMA) the read path dma_doDma takes, with transfers and bytes per
frame, biggest first.

  dma_hist_report.py game.dma
"""
import argparse, re

PATH = {0: 'WRAM direct', 1: 'HiROM base', 2: 'Game Pak RAM', 3: 'snes_read', 4: 'B->A', 5: 'page map'}
BBUS = {0x04: 'OAMDATA', 0x18: 'VMDATAL', 0x19: 'VMDATAH', 0x22: 'CGDATA', 0x80: 'WMDATA',
        0x40: 'APUIO0', 0x41: 'APUIO1', 0x42: 'APUIO2', 0x43: 'APUIO3'}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('hist')
    a = ap.parse_args()
    frames, rows = 1, []
    for line in open(a.hist):
        if line.startswith('#'):
            m = re.match(r'# frames \d+\.\.\d+ \((\d+)\)', line)
            if m: frames = int(m.group(1))
            continue
        kind, bank, badr, mode, path, n, nbytes = line.split()
        rows.append((kind, int(bank, 16), int(badr, 16), int(mode), int(path), int(n), int(nbytes)))
    tot = sum(r[6] for r in rows)
    print('%d frames, %.0f bytes a frame (dma %.0f, hdma %.0f)\n' % (
        frames, tot / frames, sum(r[6] for r in rows if r[0] == 'dma') / frames,
        sum(r[6] for r in rows if r[0] == 'hdma') / frames))
    print('kind  bank  B-bus           mode  path          transfers/f    bytes/f   share')
    for kind, bank, badr, mode, path, n, nbytes in sorted(rows, key=lambda r: -r[6]):
        print('%-5s  $%02x  $21%02x %-9s  %4d  %-12s %10.1f %10.0f  %5.1f%%' % (
            kind, bank, badr, BBUS.get(badr, ''), mode, PATH[path] if kind == 'dma' else '-',
            n / frames, nbytes / frames, 100.0 * nbytes / tot))


if __name__ == '__main__':
    main()
