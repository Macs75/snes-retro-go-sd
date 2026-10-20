#!/bin/bash
# Two host builds on a ROM's own intro / attract (no recording, no input),
# unthrottled and windowless, for N frames: is the machine state the same every
# frame (SA1_STATELOG, an SA-1 cart: WRAM, I-RAM, BW-RAM, both CPUs), and how
# much did each run (FRAME_STATS: 65816 / SA-1 instructions run per frame,
# summed, and per window of W frames). REPLAY=1: replay the ROM's recording
# (same name, .rec) instead of the attract.
#   [REPLAY=1] tools/host_attract_compare.sh <rom.sfc> <frames> <old snes_host> <new snes_host> [W=120]
set -e
ROM=$1; N=$2; OLD=$3; NEW=$4; W=${5:-120}
T=$(mktemp -d)
for b in old new; do
  BIN=$OLD; [ $b = new ] && BIN=$NEW
  ( cd "$(dirname "$ROM")" && env SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy SNES_FRAMES=$N ${REPLAY:+SNES_REPLAY=1} \
      FRAME_STATS=$T/fs_$b.txt SA1_STATELOG=$T/sl_$b.txt timeout 900 "$BIN" "$(basename "$ROM")" >/dev/null 2>&1 ) || true
done
python3 - "$T" "$W" <<'EOF'
import os, sys
t, W = sys.argv[1], int(sys.argv[2])
def fs(b):
    return [list(map(int, l.split())) for l in open(os.path.join(t, 'fs_%s.txt' % b))]
a, b = fs('old'), fs('new')
sl = [os.path.join(t, 'sl_%s.txt' % x) for x in ('old', 'new')]
if all(os.path.exists(p) for p in sl):
    x, y = open(sl[0]).read().splitlines(), open(sl[1]).read().splitlines()
    first = next((i + 1 for i, (p, q) in enumerate(zip(x, y)) if p != q), None)
    print('state: %d / %d frames logged, %s' % (len(x), len(y), 'identical' if first is None and len(x) == len(y)
                                                else 'FIRST DIFFERENCE at frame %s' % first))
else:
    print('state: no SA1_STATELOG (not an SA-1 cart)')
n = min(len(a), len(b))
print('frames %d; 65816 instructions run: old %d, new %d (%+.1f%%); SA-1: old %d, new %d' % (
    n, sum(r[1] for r in a[:n]), sum(r[1] for r in b[:n]),
    100.0 * (sum(r[1] for r in b[:n]) - sum(r[1] for r in a[:n])) / max(1, sum(r[1] for r in a[:n])),
    sum(r[5] for r in a[:n]), sum(r[5] for r in b[:n])))
print(' window  65816/frame old -> new   SA-1/frame old -> new')
for s in range(0, n, W):
    ca = sum(r[1] for r in a[s:s + W]) / W; cb = sum(r[1] for r in b[s:s + W]) / W
    sa = sum(r[5] for r in a[s:s + W]) / W; sb = sum(r[5] for r in b[s:s + W]) / W
    flag = '  <-- more' if cb > ca * 1.02 + 50 else ''
    print(' %5d   %8.0f -> %8.0f    %8.0f -> %8.0f%s' % (s // W + 1, ca, cb, sa, sb, flag))
EOF
rm -rf "$T"
