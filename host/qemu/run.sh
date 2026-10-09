#!/bin/bash
# Run the QEMU build (make qemu) on gwemu's / QEMU's mps2-an500 board:
#   host/qemu/run.sh <dir with ROM + .rec> <rom file> [frames] [qmp port]
# Prints the REC/hash and REC/pic lines of the replay. The board starts
# halted (gwemu), so it is resumed over QMP. QEMU=... overrides the binary.
set -e
DIR=$1; ROM=$2; FRAMES=${3:-4105}; PORT=${4:-4477}
ELF=$(cd "$(dirname "$0")/../.." && pwd)/snes_qemu.elf
QEMU=${QEMU:-$HOME/retro/gwemu/gwemu.AppImage}
[ -f "$ELF" ] || { echo "no $ELF: run make qemu first" >&2; exit 1; }
cd "$DIR"
"$QEMU" -M mps2-an500 -display none -monitor none -serial none \
  -semihosting-config enable=on,target=native,arg=snes,arg="$ROM",arg="$FRAMES" \
  -qmp tcp:127.0.0.1:$PORT,server,nowait -kernel "$ELF" &
QP=$!
for i in $(seq 50); do (exec 3<>/dev/tcp/127.0.0.1/$PORT) 2>/dev/null && break; sleep 0.1; done
python3 - "$PORT" <<'PY'
import socket, json, sys
s = socket.create_connection(("127.0.0.1", int(sys.argv[1]))); f = s.makefile('rw'); f.readline()
for c in ("qmp_capabilities", "cont"):
    f.write(json.dumps({"execute": c}) + "\n"); f.flush(); f.readline()
PY
wait $QP
