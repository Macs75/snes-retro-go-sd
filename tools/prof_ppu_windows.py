#!/usr/bin/env python3
"""One row per window of a PERF_PPU device log (make PERF=1 PERF_PPU=1): the
PPU's cycles per DRAWN frame (a window that keeps up draws more frames, so per
emulated frame it looks heavier), split as PROF/ppu prints them, and per drawn
line; the lines that built a sub-screen, had colour maths, had sprites.

  prof_ppu_windows.py snes_<game>_<n>_..._PERF_PPU.log [--sort ppu] [--top 30]
"""
import argparse, re

PATS = [
    (r'PROF/frame: emulated=(\d+) shown=(\d+)', ('emu', 'shown')),
    (r'PROF/emu: total=\d+ \[cpu=\d+ ppu=(\d+)', ('ppu',)),
    (r'PROF/ppu: eval=(\d+) pre=(\d+) main=(\d+) sub=(\d+) comp=(\d+) blit=(\d+)',
     ('eval', 'pre', 'main', 'sub', 'comp', 'blit')),
    (r'PROF/ppu: comp split cwin=(\d+) span=(\d+) other=(\d+)', ('cwin', 'span', 'cother')),
    (r'PROF/ppu: layers bg4=(\d+) bg2=(\d+) other=(\d+) sprites=(\d+)', ('bg4', 'bg2', 'bgx', 'spr')),
    (r'PROF/ppu: per drawn line main=(\d+) sub=(\d+) comp=(\d+) blit=(\d+); lines drawn=(\d+) sub=(\d+) '
     r'sprites=(\d+) math=(\d+)', ('lmain', 'lsub', 'lcomp', 'lblit', 'lines', 'sublines', 'sprlines', 'mathlines')),
]


def windows(path):
    out, cur = [], None
    for l in open(path, errors='replace'):
        for pat, keys in PATS:
            m = re.search(pat, l)
            if m:
                if keys[0] == 'emu':
                    cur = {}
                    out.append(cur)
                if cur is not None:
                    cur.update(zip(keys, map(int, m.groups())))
    return [w for w in out if 'lines' in w]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--sort', default='')
    ap.add_argument('--top', type=int, default=0)
    a = ap.parse_args()
    ws = windows(a.log)
    for i, w in enumerate(ws):
        w['win'] = i + 1
        k = w['emu'] / max(1, w['shown'])           # per drawn frame
        for f in ('ppu', 'eval', 'main', 'sub', 'comp', 'span', 'bg4', 'bg2', 'spr'):
            w['d' + f] = w.get(f, 0) * k / 1000.0
        w['dlines'] = w['lines'] / max(1, w['shown'])
    rows = sorted(ws, key=lambda w: -w['d' + a.sort]) if a.sort else ws
    if a.top:
        rows = rows[:a.top]
    print('K cycles per DRAWN frame                                      | per drawn line          | lines a drawn frame')
    print(' win shown   ppu  eval  main   sub  comp (span)  bg4  bg2  spr | main  sub comp         | drawn  sub math spr')
    for w in rows:
        n = max(1, w['shown'])
        print('%4d %5d %5.0f %5.0f %5.0f %5.0f %5.0f %5.0f %5.0f %4.0f %4.0f | %4d %4d %4d         | %5.0f %4.0f %4.0f %3.0f' %
              (w['win'], w['shown'], w['dppu'], w['deval'], w['dmain'], w['dsub'], w['dcomp'], w['dspan'], w['dbg4'],
               w['dbg2'], w['dspr'], w['lmain'], w['lsub'], w['lcomp'], w['dlines'], w['sublines'] / n,
               w['mathlines'] / n, w['sprlines'] / n))


if __name__ == '__main__':
    main()
