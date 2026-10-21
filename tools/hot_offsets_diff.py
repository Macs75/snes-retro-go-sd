#!/usr/bin/env python3
"""Where the hot functions of a gwemu profile sit in the I-cache, in two or
more device ELFs: each function's address and its offset modulo 8 KB (the
I-cache way, docs/perf-lessons.md), and which ELFs differ. Code in ITCM
(below 0x10000000) has no I-cache and is listed only if it moved.

  hot_offsets_diff.py <profile.txt> <ref.elf> <other.elf> [...] [--min 5000]

The profile is tools/qemu_exec_profile.py's output: "<count> <pct>% <function>".
"""
import argparse, subprocess

WAY = 0x2000


def nm(elf):
    out = subprocess.run(['arm-none-eabi-nm', elf], capture_output=True, text=True, check=True).stdout
    return {f[2]: int(f[0], 16) & ~1 for f in (l.split() for l in out.splitlines()) if len(f) == 3 and f[1] in 'tTwW'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('profile')
    ap.add_argument('elfs', nargs='+')
    ap.add_argument('--min', type=int, default=5000)
    a = ap.parse_args()
    syms = [nm(e) for e in a.elfs]
    hot = []
    for l in open(a.profile):
        f = l.split()
        if len(f) == 3 and f[0].isdigit() and int(f[0]) >= a.min:
            hot.append((int(f[0]), f[2]))
    print('%-28s %9s  %s' % ('function', 'count', '  '.join('%-20s' % e.split('/')[-1][-20:] for e in a.elfs)))
    moved = 0
    for n, name in hot:
        at = [s.get(name) for s in syms]
        if at[0] is None:
            continue
        same = all(x == at[0] for x in at) or all(x is not None and x % WAY == at[0] % WAY for x in at)
        if at[0] < 0x10000000 and same:
            continue
        moved += not same
        cols = '  '.join('%-20s' % ('-' if x is None else '%08x (%04x)' % (x, x % WAY)) for x in at)
        print('%-28s %9d  %s%s' % (name[:28], n, cols, '' if same else '   <- offset differs'))
    print('hot functions whose I-cache offset differs from the first ELF: %d' % moved)


if __name__ == '__main__':
    main()
