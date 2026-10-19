#!/bin/bash
# The REC/hash checkpoints of a host replay, sorted by frame, for comparing two
# builds: the ROM's recording (same name, .rec) runs to its end, unthrottled,
# windowless.
#   tools/host_rec_hashes.sh <rom.sfc> [snes_host]   > hashes.txt
set -e
ROM=$1
BIN=${2:-$(cd "$(dirname "$0")/.." && pwd)/snes_host}
REC="${ROM%.*}.rec"
[ -f "$ROM" ] && [ -f "$REC" ] || { echo "need $ROM and $REC" >&2; exit 1; }
FRAMES=$(python3 -c 'import struct,sys; print(struct.unpack_from("<I", open(sys.argv[1],"rb").read(), 12)[0])' "$REC")
cd "$(dirname "$ROM")"
SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy SNES_REPLAY=1 SNES_FRAMES=$FRAMES \
  timeout 900 "$BIN" "$(basename "$ROM")" 2>/dev/null | grep -o 'REC/hash.*' | sort -u -t= -k2,2n
