#!/usr/bin/env python3
"""What each window of a device PERF log would cost with every frame shown,
as % of the frame budget: a window that skips frames looks lighter than it is,
because the PPU and the present run only for the frames it draws. The PPU and
the present are scaled up from their cost per shown frame, the rest (65816,
SA-1 or GSU, APU in the frame and in the PCM loop, DMA, the loop's input and
frame control) is per emulated frame already. The excess over 100% is what a
window must lose to keep up.

  prof_demand.py snes_<game>_<n>_..._PERF.log [--below 120] [--sort demand]
"""
import argparse, re

PATS = [
    (r'PROF/emu: total=\d+ \[cpu=(\d+) ppu=(\d+) apu=(\d+) dma=(\d+) (?:gsu|sa1)=(\d+)',
     ('cpu', 'ppu', 'apu', 'dma', 'cop')),
    (r'PROF/present: present=(\d+) wait=\d+ overlay=(\d+) swap=(\d+)', ('pres', 'ovl', 'swap')),
    (r'PROF/loop: pcm=(\d+) input=(\d+) framectl=(\d+)', ('pcm', 'inp', 'fctl')),
    (r'PROF/frame: emulated=(\d+) shown=(\d+)', ('emu', 'shown')),
    (r'PROF/budget: budget=(\d+)', ('budget',)),
]


def windows(path):
    out, cur = [], None
    for line in open(path, errors='replace'):
        if line.startswith('PROF/frame:'):     # a window's report starts here
            cur = {}
            out.append(cur)
        if cur is None:
            continue
        for pat, keys in PATS:
            m = re.search(pat, line)
            if m:
                cur.update(zip(keys, map(int, m.groups())))
    return [w for w in out if 'budget' in w and 'cpu' in w and w.get('shown')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--below', type=int, default=None, help='only windows that showed fewer frames')
    ap.add_argument('--sort', choices=['demand', 'win'], default='win')
    a = ap.parse_args()
    rows = []
    for i, w in enumerate(windows(a.log), 1):
        if i == 1:
            continue                     # the boot
        b = w['budget'] / 100.0
        k = w['emu'] / w['shown']        # shown -> every frame
        r = dict(win=i, shown=w['shown'], cpu=w['cpu'] / b, cop=w['cop'] / b,
                 apu=(w['apu'] + w['pcm']) / b, ppu=w['ppu'] * k / b,
                 pres=(w['pres'] + w['ovl'] + w['swap']) / b,     # PROF/present is per shown frame
                 rest=(w['dma'] + w['inp'] + w['fctl']) / b)
        r['demand'] = r['cpu'] + r['cop'] + r['apu'] + r['ppu'] + r['pres'] + r['rest']
        rows.append(r)
    sel = [r for r in rows if a.below is None or r['shown'] < a.below]
    if a.sort == 'demand':
        sel.sort(key=lambda r: -r['demand'])
    cols = ('cpu', 'cop', 'apu', 'ppu', 'pres', 'rest', 'demand')
    print('win shown ' + ' '.join('%7s' % c for c in cols) + '  over')
    for r in sel:
        print('%3d %5d ' % (r['win'], r['shown']) + ' '.join('%7.1f' % r[c] for c in cols) +
              '  %+5.1f' % (r['demand'] - 100))
    for name, rs in (('all', rows), ('shown', sel)):
        if rs and (name == 'all' or a.below is not None):
            print('mean %-5s ' % name + ' '.join('%7.1f' % (sum(r[c] for r in rs) / len(rs)) for c in cols))


if __name__ == '__main__':
    main()
