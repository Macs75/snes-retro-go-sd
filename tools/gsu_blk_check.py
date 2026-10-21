#!/usr/bin/env python3
"""Can a game's Super FX block code run on this ROM? The blocks
(src/snes/gsu_zsnes/gsu_blocks_<game>.inc, tools/gen_gsu_blocks.py) are used
only when the ROM bytes they were compiled from hash to what they were made
from (gsu_blocks.c gsu_blk_hash): this reads the game's dependency runs and
hash from its .inc, hashes the runs in the ROM as the GSU sees it (banks
$00-$3F 32 KB LoROM pages, $40-$5F 64 KB linear: gsu_zsnes c_fxtable.inc
fill_memtable), and lists the runs whose bytes differ from a reference ROM.

  gsu_blk_check.py <game> <rom.sfc> [--ref <rom the blocks were made from>]
"""
import argparse, os, re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def runs_of(game):
    t = open(os.path.join(ROOT, 'src/snes/gsu_zsnes/gsu_blocks_%s.inc' % game)).read()
    body = re.search(r'gsu_blk_dep_%s\[\] = \{(.*?)\};' % game, t, re.S).group(1)
    runs = [int(x, 16) for x in re.findall(r'0x([0-9A-Fa-f]+)u', body)]
    want = int(re.search(r'GSU_BLK_%s_HASH 0x([0-9A-Fa-f]+)u' % game.upper(), t).group(1), 16)
    return runs, want


def gsu_bytes(rom, bank, a, n):
    if bank < 0x40 or 0x80 <= bank < 0xc0:
        base, mask = ((bank & 0x3f) * 0x8000) % len(rom), 0x7fff
    elif bank < 0x60 or 0xc0 <= bank < 0xe0:
        base, mask = ((bank & 0x1f) * 0x10000) % len(rom), 0xffff
    else:
        raise ValueError('bank %02x is not ROM' % bank)
    return bytes(rom[(base + ((a + k) & mask)) % len(rom)] for k in range(n))


def fnv(rom, runs):
    h = 0x811C9DC5
    for r in runs:
        for b in gsu_bytes(rom, r >> 24, (r >> 8) & 0xffff, r & 0xff):
            h = ((h ^ b) * 0x01000193) & 0xffffffff
    return h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('game')
    ap.add_argument('rom')
    ap.add_argument('--ref')
    a = ap.parse_args()
    runs, want = runs_of(a.game)
    rom = open(a.rom, 'rb').read()
    h = fnv(rom, runs)
    print('%s: %d runs, %d bytes; hash %08X, the blocks want %08X: %s' % (
        a.game, len(runs), sum(r & 0xff for r in runs), h, want, 'MATCH' if h == want else 'no match'))
    if a.ref:
        ref = open(a.ref, 'rb').read()
        bad = 0
        for r in runs:
            bank, adr, n = r >> 24, (r >> 8) & 0xffff, r & 0xff
            x, y = gsu_bytes(ref, bank, adr, n), gsu_bytes(rom, bank, adr, n)
            if x != y:
                bad += 1
                k = next(i for i in range(n) if x[i] != y[i])
                print('  run %02x:%04x+%d differs from byte %d (ref %s, rom %s)' % (
                    bank, adr, n, k, x[k:k + 8].hex(), y[k:k + 8].hex()))
        print('runs differing from the reference: %d of %d' % (bad, len(runs)))


if __name__ == '__main__':
    main()
