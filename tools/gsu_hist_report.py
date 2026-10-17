#!/usr/bin/env python3
"""Which GSU code in a histogram runs outside the compiled chains, and where else it runs.

For the per-game block code (tools/gen_gsu_blocks.py): rebuild the chains of a
selection exactly as the generator does, take a histogram of the frames you care
about (e.g. the spike frames of a recording, GSU_PCHIST + GSU_PCHIST_FRAMES=a:b,c:d),
and report the code in it that is not compiled, grouped into routines:

  - instructions it ran in those frames, and outside them (from a whole-run
    histogram);
  - its size in addresses, and the ARM it would cost at the current
    bytes-per-compiled-instruction ratio (--arm-bytes, the linked size of the
    chains);
  - static callers: IWT R15 / IWT R13 immediates anywhere in the program banks
    that point into the routine (jump/call and LOOP targets), with whether a
    LINK precedes them.

Usage:
  gsu_hist_report.py --rom yoshi.sfc --tables src/snes/gsu_zsnes/c_fxtable.inc \\
      --ops src/snes/gsu_zsnes/fx_ops.h \\
      --hist game.hist:0.89 --hist title.hist:0.95 --hist intro.hist:0.75 \\
      --focus spike.hist --all yoshi_all.hist --arm-bytes 49782
"""
import argparse, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_gsu_blocks as G


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--rom', required=True)
    ap.add_argument('--tables', required=True)
    ap.add_argument('--ops', required=True)
    ap.add_argument('--hist', action='append', required=True, help='the selection, as for gen_gsu_blocks.py')
    ap.add_argument('--focus', required=True, help='histogram of the frames to examine')
    ap.add_argument('--all', required=True, help='histogram of the whole run')
    ap.add_argument('--arm-bytes', type=float, required=True, help='linked size of the current chains')
    ap.add_argument('--gap', type=int, default=24, help='bytes between addresses of one routine')
    ap.add_argument('--banks', default='08,09,0a,0b', help='program banks to scan for callers')
    a = ap.parse_args()
    rom = G.Rom(a.rom)
    t = G.parse_tables(a.tables)
    h = G.parse_handlers(a.ops)

    sel, dyn, seen = set(), set(), set()
    for spec in a.hist:
        path, _, cov = spec.partition(':')
        rows, d = G.read_hist(rom, path)
        dyn |= d
        seen |= set(rows)
        tot, acc = sum(rows.values()), 0
        for k, n in sorted(rows.items(), key=lambda r: -r[1]):
            if acc >= float(cov or 0.95) * tot:
                break
            sel.add(k)
            acc += n
    owner, ninstr = {}, 0
    for p in sorted(sel):
        if p in owner:
            continue
        ch = G.Chain(rom, t, h, p[0], p[1], sel, dyn, 256, seen)
        ninstr += len(ch.starts)
        for s, _ in ch.starts:
            owner.setdefault((p[0], s), p)
    bpi = a.arm_bytes / ninstr
    print('chains: %d compiled instructions, %.1f ARM bytes each' % (ninstr, bpi))

    focus, _ = G.read_hist(rom, a.focus)
    whole, _ = G.read_hist(rom, a.all)
    ftot = sum(focus.values())
    miss = {k: n for k, n in focus.items() if k not in owner}
    msum = sum(miss.values())
    print('focus: %d instructions, %.1f%% outside the chains, %d uncompiled addresses'
          % (ftot, 100.0 * msum / ftot, len(miss)))

    cl = []
    for b, x in sorted(miss):
        if cl and cl[-1]['b'] == b and x - cl[-1]['hi'] <= a.gap:
            cl[-1]['hi'] = x
            cl[-1]['addrs'].append(x)
        else:
            cl.append({'b': b, 'lo': x, 'hi': x, 'addrs': [x]})
    for c in cl:
        c['in'] = sum(miss[(c['b'], x)] for x in c['addrs'])
        w = sum(whole.get((c['b'], x), 0) for x in c['addrs'])
        c['out'] = w - c['in'] if w >= c['in'] else None   # None: not (all) in the --all run
    cl.sort(key=lambda c: -c['in'])

    # static references: IWT R15 (FF) and IWT R13 (FD) immediates into a routine
    banks = [int(x, 16) for x in a.banks.split(',')]
    span = 0x8000

    def refs(c):
        out = []
        for b in banks:
            lo = 0x8000 if b < 0x40 else 0
            for x in range(lo, lo + span - 2):
                op = rom.byte(b, x)
                if op not in (0xFF, 0xFD):
                    continue
                tgt = rom.byte(b, x + 1) | rom.byte(b, x + 2) << 8
                if b == c['b'] and c['lo'] <= tgt <= c['hi'] + 8:
                    link = 0x91 <= rom.byte(b, x - 1) <= 0x94
                    ran = (b, x) in whole
                    out.append('%02x:%04x %s #%04x%s%s' % (b, x, 'IWT R15' if op == 0xFF else 'IWT R13', tgt,
                                                           ' (after LINK)' if link else '', ' ran' if ran else ''))
        return out

    print('\nroutine          bytes addrs  in-focus  cum%%  outside-focus  est.ARM  static references (ran = executed in the run)')
    acc = 0
    for c in cl:
        acc += c['in']
        r = refs(c)
        print('%02x:%04x-%04x  %5d %5d %9d %5.1f%% %13s %7.0f B  %s'
              % (c['b'], c['lo'], c['hi'], c['hi'] - c['lo'] + 1, len(c['addrs']), c['in'], 100.0 * acc / msum,
                 'not in --all' if c['out'] is None else str(c['out']), len(c['addrs']) * bpi,
                 '; '.join(r) if r else '-'))
    for frac in (0.8, 0.9, 0.95):
        acc = addrs = k = 0
        for c in cl:
            if acc >= frac * msum:
                break
            acc += c['in']
            addrs += len(c['addrs'])
            k += 1
        print('%.0f%% of the uncompiled focus work: %d routines, %d addresses, ~%.1f KB ARM'
              % (frac * 100, k, addrs, addrs * bpi / 1024))


if __name__ == '__main__':
    main()
