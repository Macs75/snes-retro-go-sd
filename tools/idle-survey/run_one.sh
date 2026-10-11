#!/bin/bash
# run_one.sh <index> : survey one game from games.json.
#
#   1. record a reference run with the UNMODIFIED build (host_base)
#   2. replay the same frames with the IDLE-SKIP + instrumented build
#      (host_survey): idle counters on stderr, a PC histogram -> pch.txt,
#      the NMI-landing PCs -> nmi.txt
#
# Output goes to $SURVEY_DIR/g<NNN>/; analyze.py then reads those folders.
#
# Env:
#   SURVEY_DIR  working dir for the gNNN/ folders   (default: this script's dir)
#   BIN         dir holding host_base + host_survey  (default: this script's dir)
#   FRAMES      emulated frames per run              (default: 4200)
#   PCH_LO/HI   PC-histogram window, in frames       (default: 600 .. FRAMES)
# See README.md for how host_base / host_survey are built.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
SURVEY_DIR="${SURVEY_DIR:-$here}"
BIN="${BIN:-$here}"
FRAMES="${FRAMES:-4200}"
PCH_LO="${PCH_LO:-600}"
PCH_HI="${PCH_HI:-$FRAMES}"

i="$1"; n="g$(printf %03d "$i")"; d="$SURVEY_DIR/$n"; mkdir -p "$d"; cd "$d" || exit 1
[ -f done ] && exit 0

# resolve the game: title / region / chip / rom-path, one per line
python3 -c "import json,sys;g=json.load(open('$here/games.json'))[$i];open('name','w').write(g['title']+'\n'+g['region']+'\n'+g.get('chip','')+'\n'+g['path']+'\n')"
rom="$(sed -n 4p name)"
[ -f "$rom" ] || { echo "load failed: rom not found: $rom" > base.txt; touch done; exit 0; }
cp "$rom" rom.sfc

export SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy
rm -f rom.rec pch.txt nmi.txt

# 1) reference recording (unmodified build)
SNES_RECORD=1 SNES_FRAMES="$FRAMES" timeout 1200 "$BIN/host_base" rom.sfc \
  > base.txt 2> base_err.txt; echo "base_exit=$?" > status

# 2) instrumented idle-skip replay of the same recording
SNES_REPLAY=1 IDLE_STATS=1 PCH_LO="$PCH_LO" PCH_HI="$PCH_HI" SNES_FRAMES="$FRAMES" \
  timeout 1200 "$BIN/host_survey" rom.sfc > sv.txt 2> sv_err.txt; echo "survey_exit=$?" >> status

rm -f rom.sfc
touch done
