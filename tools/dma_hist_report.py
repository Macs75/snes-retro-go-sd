#!/usr/bin/env python3
"""What a game's DMA moves per frame, from a host DMA_HIST file (src/snes/dma.c).

One row per kind: general DMA or HDMA, source bank, B-bus target, mode and
(general DMA) the read path dma_doDma takes, with transfers and bytes per
frame, biggest first. Then the VRAM uploads by kind: word uploads whose
low/high byte pairs land on one VRAM word (mode 1/5 to $2118, VMAIN increment
after the high byte, starting on the low byte), the other word uploads, and
byte-only ones.

  dma_hist_report.py game.dma
"""
import argparse, re

PATH = {0: 'WRAM direct', 1: 'HiROM base', 2: 'Game Pak RAM', 3: 'snes_read', 4: 'B->A', 5: 'page map'}
BBUS = {0x04: 'OAMDATA', 0x18: 'VMDATAL', 0x19: 'VMDATAH', 0x22: 'CGDATA', 0x80: 'WMDATA',
        0x40: 'APUIO0', 0x41: 'APUIO1', 0x42: 'APUIO2', 0x43: 'APUIO3'}


def vmain(vm):
    """VMAIN as dma.c recorded it (dma_h_vmain)"""
    return 'inc-after-%s +%d remap%d%s' % ('high' if vm & 1 else 'low', (1, 32, 128, 0)[(vm >> 3) & 3],
                                           (vm >> 1) & 3, ' odd-start' if vm & 0x20 else '')


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
        f = line.split()
        kind, bank, badr, mode, path, n, nbytes = f[:7]
        vm = int(f[7], 16) if len(f) > 7 else 0
        rows.append((kind, int(bank, 16), int(badr, 16), int(mode), int(path), int(n), int(nbytes), vm))
    tot = sum(r[6] for r in rows)
    print('%d frames, %.0f bytes a frame (dma %.0f, hdma %.0f)\n' % (
        frames, tot / frames, sum(r[6] for r in rows if r[0] == 'dma') / frames,
        sum(r[6] for r in rows if r[0] == 'hdma') / frames))
    print('kind  bank  B-bus           mode  path          transfers/f    bytes/f   share  VMAIN')
    for kind, bank, badr, mode, path, n, nbytes, vm in sorted(rows, key=lambda r: -r[6]):
        print('%-5s  $%02x  $21%02x %-9s  %4d  %-12s %10.1f %10.0f  %5.1f%%  %s' % (
            kind, bank, badr, BBUS.get(badr, ''), mode, PATH[path] if kind == 'dma' else '-',
            n / frames, nbytes / frames, 100.0 * nbytes / tot, vmain(vm) if badr in (0x18, 0x19) else ''))
    cls = {}
    for kind, bank, badr, mode, path, n, nbytes, vm in rows:
        if kind != 'dma' or badr not in (0x18, 0x19): continue
        if badr == 0x18 and mode in (1, 5):
            c = 'word, pairable' if (vm & 1) and not (vm & 0x20) else \
                'word, increment after low' if not (vm & 1) else 'word, starts on the high byte'
        elif mode in (0, 2, 6):
            c = 'byte-only $21%02x' % badr
        else:
            c = 'other (mode %d to $21%02x)' % (mode, badr)
        cls[c] = cls.get(c, 0) + nbytes
    dma_tot = sum(r[6] for r in rows if r[0] == 'dma')
    if cls:
        print('\nVRAM uploads                       bytes/f   of all DMA bytes')
        for c, b in sorted(cls.items(), key=lambda x: -x[1]):
            print('  %-30s %8.0f   %5.1f%%' % (c, b / frames, 100.0 * b / dma_tot if dma_tot else 0))


if __name__ == '__main__':
    main()
