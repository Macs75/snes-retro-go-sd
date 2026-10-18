#!/usr/bin/env python3
"""Where the SNES core's RAM_EMU image goes: sections, and symbols by source file.

The RAM_EMU pool a game gets is RAM_EMU minus the core image (code + data +
bss). This lists the image by output section and its largest symbols, grouped by
the object they come from, so you can see what a given cart never uses.

Reads `nm -S -l` style output produced with the Docker toolchain, e.g.:
  docker run --rm -v "$PWD:/w" -w /w sylverb/retro-go-sd-builder:v1.5 sh -c \\
    'arm-none-eabi-nm -S -t d --size-sort -l build/core/snes_core.elf' > nm.txt
  docker run ... sh -c 'arm-none-eabi-objdump -h build/core/snes_core.elf' > sections.txt
  core_mem_report.py nm.txt sections.txt [--min 1024]
"""
import argparse, re
from collections import defaultdict

RAM_EMU = (0x2404B000, 0x24100000)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('nm')
    ap.add_argument('sections')
    ap.add_argument('--min', type=int, default=1024, help='smallest symbol to list')
    a = ap.parse_args()

    print('sections in RAM_EMU (VMA):')
    for line in open(a.sections):
        p = line.split()
        if len(p) >= 6 and re.fullmatch(r'\d+', p[0]):
            size, vma = int(p[2], 16), int(p[3], 16)
            if RAM_EMU[0] <= vma < RAM_EMU[1] and size:
                print('  %-22s %8d B  at %08x' % (p[1], size, vma))

    syms = []
    for line in open(a.nm):
        m = re.match(r'(\d+) (\d+) (\w) (\S+)(?:\s+(\S+))?', line)
        if not m:
            continue
        addr, size, typ, name, loc = int(m.group(1)), int(m.group(2)), m.group(3), m.group(4), m.group(5) or ''
        if RAM_EMU[0] <= addr < RAM_EMU[1]:
            src = re.sub(r':\d+$', '', loc).split('/src/')[-1] if loc else '?'
            syms.append((size, typ, name, src))
    byfile = defaultdict(lambda: [0, 0])
    for size, typ, name, src in syms:
        byfile[src][0 if typ in 'tTrR' else 1] += size
    print('\nby source file (code+rodata / data+bss), largest first:')
    for src, (c, d) in sorted(byfile.items(), key=lambda kv: -sum(kv[1]))[:30]:
        print('  %-45s %8d %8d' % (src, c, d))
    print('\nsymbols of at least %d B:' % a.min)
    for size, typ, name, src in sorted(syms, reverse=True):
        if size >= a.min:
            print('  %8d %s %-40s %s' % (size, typ, name, src))


if __name__ == '__main__':
    main()
