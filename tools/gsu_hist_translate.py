#!/usr/bin/env python3
"""A Super FX profile (GSU_PCHIST, "bank:addr count opcode alt ...") of one
cart rewritten in the addresses of another revision of the same game, for
tools/gen_gsu_blocks.py: so the other cart's block code covers the scenes the
first one was profiled on without recording them again.

Each address keeps its count where the other ROM has the same code there (a
window of --window bytes from it, as the GSU sees the ROM); else the window is
looked for in the same bank of the other ROM, and a unique match gives the new
address (code the revision moved); else the line is dropped (code it changed).
A delay-slot line (the opcode is not the ROM's: PC-1 after a jump, see
gen_gsu_blocks.py read_hist) is matched on the code at its target, PC.
The generator compiles the other ROM's own bytes, and the table is enabled
only when they hash to what it was made from: a profile only says where the
hot code is.

  gsu_hist_translate.py <from.sfc> <to.sfc> <in.hist> <out.hist> [--window 8]
"""
import argparse, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gsu_blk_check import gsu_bytes   # noqa: E402


def bank_bytes(rom, bank):
    return gsu_bytes(rom, bank, 0, 0x10000) if bank >= 0x40 else gsu_bytes(rom, bank, 0x8000, 0x8000)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('inp')
    ap.add_argument('out')
    ap.add_argument('--window', type=int, default=8)
    a = ap.parse_args()
    src, dst = open(a.src, 'rb').read(), open(a.dst, 'rb').read()
    W = a.window
    banks, memo = {}, {}

    def where(bank, adr):
        """the other ROM's address of the code at bank:adr, or None"""
        if (bank, adr) in memo:
            return memo[(bank, adr)]
        w = gsu_bytes(src, bank, adr, W)
        r = None
        if gsu_bytes(dst, bank, adr, W) == w:
            r = adr
        else:
            if bank not in banks:
                banks[bank] = bank_bytes(dst, bank)
            d = banks[bank]
            i = d.find(w)
            if i >= 0 and d.find(w, i + 1) < 0:
                r = i if bank >= 0x40 else 0x8000 + i
        memo[(bank, adr)] = r
        return r

    kept = moved = 0
    tot = lost = 0
    lines = []
    head = None
    for line in open(a.inp):
        if line.startswith('#'):
            head = head or line
            continue
        f = line.split()
        bank, adr = (int(x, 16) for x in f[0].split(':'))
        n = int(f[1])
        tot += n
        if bank >= 0x60:
            lost += n
            continue
        slot = not (gsu_bytes(src, bank, adr, 1)[0] == int(f[2], 16) and f[3] == '0')
        t = where(bank, (adr + 1) & 0xffff if slot else adr)
        if t is None:
            lost += n
            continue
        t = (t - 1) & 0xffff if slot else t
        if t == adr:
            kept += n
        else:
            moved += n
        lines.append((n, '%02x:%04x %s' % (bank, t, ' '.join(f[1:4]))))
    with open(a.out, 'w') as o:
        o.write('# %s translated to %s: same address %.1f%%, moved %.1f%%, dropped %.1f%% of %d\n' % (
            os.path.basename(a.inp), os.path.basename(a.dst), 100.0 * kept / tot, 100.0 * moved / tot,
            100.0 * lost / tot, tot))
        for _, l in sorted(lines, key=lambda x: -x[0]):
            o.write(l + '\n')
    print('%s: same address %.1f%%, moved %.1f%%, dropped %.1f%%' % (
        os.path.basename(a.inp), 100.0 * kept / tot, 100.0 * moved / tot, 100.0 * lost / tot))


if __name__ == '__main__':
    main()
