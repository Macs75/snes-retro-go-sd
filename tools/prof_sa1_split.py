#!/usr/bin/env python3
"""Where an SA-1 game's SA-1 time goes, from device PERF logs with PROF/sa1b.

Per log, over all windows and over the heavy ones (SA-1 instructions above
--heavy): the SA-1 figure (sa1= in PROF/emu) split into the cycles inside the
engine's SA-1 loop (in_loop: instructions, memory bridges, C fallback, cache
stalls) and the rest (catch-up calls, sa1_interpret, the proof hand-backs),
per SA-1 instruction and per slice (one sa1_interpret call).

  prof_sa1_split.py A.log B.log [--heavy 15000]
"""
import argparse, re


def windows(path):
    rows, cur = [], {}
    for line in open(path, errors='replace'):
        m = re.search(r'PROF/emu: total=\d+ \[cpu=(\d+) ppu=\d+ apu=\d+ dma=\d+ (?:gsu|sa1)=(\d+)', line)
        if m:
            cur['cpu'], cur['sa1'] = map(int, m.groups())
        m = re.search(r'PROF/ops: ops=(\d+)', line)
        if m:
            cur['ops'] = int(m.group(1))
        m = re.search(r'PROF/sa1: ops=(\d+)', line)
        if m:
            cur['sops'] = int(m.group(1))
        m = re.search(r'PROF/sa1b: slices=(\d+) loops=(\d+) in_loop=(\d+) cycles handbacks=(\d+) c_steps=(\d+)', line)
        if m:
            cur.update(zip(('slices', 'loops', 'inloop', 'hb', 'cst'), map(int, m.groups())))
            rows.append(cur)
            cur = {}
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--heavy', type=int, default=15000)
    a = ap.parse_args()
    for p in a.logs:
        rows = windows(p)
        for name, sel in (('all', rows), ('heavy', [r for r in rows if r['sops'] > a.heavy])):
            if not sel:
                continue
            t = lambda k: sum(r[k] for r in sel)
            out = t('sa1') - t('inloop')
            print('%-40s %-5s n=%2d  SA-1 cyc/instr %5.1f = in loop %5.1f + outside %4.1f | slices/frame %5.0f  outside/slice %5.0f  instr/slice %4.1f  handbacks/frame %4.0f  65816 cyc/instr %5.0f' % (
                p.split('/')[-1][:40], name, len(sel), t('sa1') / t('sops'), t('inloop') / t('sops'), out / t('sops'),
                t('slices') / len(sel), out / max(t('slices'), 1), t('sops') / max(t('slices'), 1), t('hb') / len(sel),
                t('cpu') / t('ops')))


if __name__ == '__main__':
    main()
