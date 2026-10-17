#!/usr/bin/env python3
"""Where the SPC700 spends its opcodes, grouped into routines and disassembled.

Input: the host's APU_HIST file (src/snes/apu.c) and the ARAM dump written next
to it (<file>.aram). Prints the header (idle skip, DSP voice census), then the
hottest routines -- runs of executed addresses no more than --gap bytes apart --
each with its share of all interpreted opcodes and a disassembly carrying the
per-instruction counts (opcode names from src/snes/tracing.c). A backward branch
inside a routine is marked as a loop with its iteration count.

  SNES_REPLAY=1 SNES_FRAMES=19289 APU_HIST=game.hist APU_HIST_FRAMES=13680:19289 ./snes_host game.sfc
  spc_hist_report.py game.hist --tracing src/snes/tracing.c [--cover 0.9] [--gap 8]
"""
import argparse, re


def load_tables(path):
    src = open(path).read()
    names = re.search(r'opcodeNamesSpc\[256\] = \{(.*?)\};', src, re.S).group(1)
    names = re.findall(r'"((?:[^"\\]|\\.)*)"', names)
    types = re.search(r'opcodeTypeSpc\[256\] = \{(.*?)\};', src, re.S).group(1)
    types = [int(x) for x in re.findall(r'\d+', types)]
    assert len(names) == 256 and len(types) == 256
    return [n.rstrip() for n in names], types


LEN = {0: 1, 1: 2, 2: 3, 3: 2, 4: 3, 5: 3, 6: 3}


def disasm(ram, pc, names, types):
    op = ram[pc]
    b1, b2 = ram[(pc + 1) & 0xffff], ram[(pc + 2) & 0xffff]
    word = b1 | b2 << 8
    t = types[op]
    n = names[op]
    if t == 0: s = n
    elif t == 1: s = n % b1
    elif t == 2: s = n % word
    elif t == 3: s = n % ((pc + 2 + (b1 - 256 if b1 > 127 else b1)) & 0xffff)
    elif t == 4: s = n % (b2, b1)
    elif t == 5: s = n % (b1, (pc + 3 + (b2 - 256 if b2 > 127 else b2)) & 0xffff)
    else: s = n % (word & 0x1fff, word >> 13)
    return LEN[t], s


def branch_target(ram, pc, types):
    """Target of a relative branch at pc, or None."""
    op, t = ram[pc], types[ram[pc]]
    if t == 3:
        b = ram[(pc + 1) & 0xffff]
        return (pc + 2 + (b - 256 if b > 127 else b)) & 0xffff
    if t == 5:
        b = ram[(pc + 2) & 0xffff]
        return (pc + 3 + (b - 256 if b > 127 else b)) & 0xffff
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('hist')
    ap.add_argument('--tracing', default='src/snes/tracing.c')
    ap.add_argument('--cover', type=float, default=0.90, help='list routines up to this share of the opcodes')
    ap.add_argument('--gap', type=int, default=8, help='bytes between addresses of one routine')
    a = ap.parse_args()
    names, types = load_tables(a.tracing)
    ram = open(a.hist + '.aram', 'rb').read()
    cnt, frames = {}, 1
    for line in open(a.hist):
        if line.startswith('#'):
            print(line.rstrip())
            m = re.match(r'# frames \d+\.\.\d+ \((\d+)\)', line)
            if m: frames = int(m.group(1))
            continue
        p = line.split()
        cnt[int(p[0], 16)] = int(p[1])
    total = sum(cnt.values())
    print('%d opcodes interpreted, %.0f a frame\n' % (total, total / frames))

    cl = []
    for pc in sorted(cnt):
        if cl and pc - cl[-1][-1] <= a.gap:
            cl[-1].append(pc)
        else:
            cl.append([pc])
    cl.sort(key=lambda c: -sum(cnt[x] for x in c))
    acc = 0
    for c in cl:
        if acc >= a.cover * total:
            break
        n = sum(cnt[x] for x in c)
        acc += n
        print('== %04x-%04x  %d opcodes (%.1f%%, cum %.1f%%), %.0f a frame'
              % (c[0], c[-1], n, 100.0 * n / total, 100.0 * acc / total, n / frames))
        pc = c[0]
        while pc <= c[-1]:
            ln, s = disasm(ram, pc, names, types)
            k = cnt.get(pc, 0)
            tgt = branch_target(ram, pc, types)
            note = ''
            if tgt is not None and tgt <= pc and tgt >= c[0] and k:
                taken = cnt.get(tgt, 0)
                note = '  <- loop back to %04x' % tgt
            print('  %04x %-10s %-26s %10s%s' % (pc, ' '.join('%02x' % ram[(pc + i) & 0xffff] for i in range(ln)),
                                                 s, k if k else '.', note))
            pc += ln
        print()


if __name__ == '__main__':
    main()
