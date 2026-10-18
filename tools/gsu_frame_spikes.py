#!/usr/bin/env python3
"""Find the frames where the GSU does far more work than usual.

Input: the host's GSU_FRAMEOPS file (fx_ops.h): one line per emulated frame,
"frame per-line-ops vblank-catch-up-ops start-ops". Prints the frames above a
threshold grouped into episodes, and a GSU_PCHIST_FRAMES value covering them,
so a histogram of just those frames can be taken:

  SNES_REPLAY=1 SNES_FRAMES=19289 GSU_BLOCKS=0 GSU_FRAMEOPS=frameops.txt ./snes_host game.sfc
  gsu_frame_spikes.py frameops.txt --min 100000 --skip 12895:13700
  SNES_REPLAY=1 ... GSU_PCHIST=spike.hist GSU_PCHIST_FRAMES=<printed ranges> ./snes_host game.sfc
"""
import argparse


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('frameops')
    ap.add_argument('--min', type=int, default=100000, help='GSU instructions in a frame to count as a spike')
    ap.add_argument('--join', type=int, default=3, help='frames apart that still make one episode')
    ap.add_argument('--skip', action='append', default=[],
                    help='a:b frames to leave out (e.g. a title screen that is heavy every few frames)')
    a = ap.parse_args()
    skips = [tuple(int(x) for x in s.split(':')) for s in a.skip]
    tot = {}
    for line in open(a.frameops):
        f, l, v, s = (int(x) for x in line.split()[:4])
        tot[f] = (l + v + s, l, v, s)
    spk = [f for f in sorted(tot) if tot[f][0] >= a.min and not any(lo <= f < hi for lo, hi in skips)]
    eps = []
    for f in spk:
        if eps and f - eps[-1][-1] <= a.join:
            eps[-1].append(f)
        else:
            eps.append([f])
    mean = sum(v[0] for v in tot.values()) / max(len(tot), 1)
    print('%d frames, %.0f GSU instructions a frame on average; %d spike frames in %d episodes'
          % (len(tot), mean, len(spk), len(eps)))
    print('frames        n   peak    total  (per-line / vblank catch-up / start, at the peak)')
    for e in eps:
        pk = max(e, key=lambda f: tot[f][0])
        print('%5d-%-5d %3d %7d %8d  (%d / %d / %d)' % (e[0], e[-1], len(e), tot[pk][0],
                                                        sum(tot[f][0] for f in e), *tot[pk][1:]))
    print('GSU_PCHIST_FRAMES=' + ','.join('%d:%d' % (e[0], e[-1] + 1) for e in eps))


if __name__ == '__main__':
    main()
