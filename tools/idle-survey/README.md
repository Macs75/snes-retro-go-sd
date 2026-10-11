# Idle-skip survey tools

The batch pipeline that histograms every game to find its wait loop and classify
it (does the generic skip already take it, does it need a generic decoder fix, or
a per-game template?). This produced `docs/snes_idle_survey.{md,csv}` and the
category-D draft `docs/snes_idle_skips_draft.txt`. See `docs/idle-skip.md` for
what the idle skip is and how to turn a category-D finding into a template.

Everything here runs on the **host** (no device), from deterministic replays,
and gates every result on a bit-exact state/picture/audio hash against an
unmodified reference build.

## Files

| File | What it is |
|---|---|
| `run_one.sh` | survey one game: reference record + instrumented idle-skip replay |
| `games.json` | the game list (`title`, `region`, `chip`, `path`), one object per entry |
| `analyze.py` | read every `gNNN/` folder → `survey.json` (one row per game: skip %, the wait loop decoded from the ROM, its CPU/NMI share, hash diff) |
| `report.py` | `survey.json` → `report.json` + the A–X classification used in the survey doc |
| `prof_runs.py` | parse `PROF/*` device/host logs into per-run busy%, frameskip, cpu/ppu/apu, skip dots (handy beyond the survey) |
| `survey-instrumentation.md` | the measurement-only host probes (PC histogram + NMI log); **not in the shipping core** |

## How to run it

1. **Build the two host binaries** per `survey-instrumentation.md`:
   `host_base` (unmodified, idle skip OFF — the reference) and `host_survey`
   (idle skip ON + the three probes). Put both in one directory, e.g. `./bin/`.

2. **Point `games.json` at your ROMs** (the `path` field is an absolute ROM
   path). The set that produced the committed survey is this file as-is;
   `docs/snes_top_games_mapping.txt` is the title→file map it came from.

3. **Run a scratch working dir** (not the repo — each game leaves ~a few files,
   and a 16M-entry histogram is dumped per game):

   ```sh
   export SURVEY_DIR=/tmp/idle-survey        # where the gNNN/ folders go
   export BIN=$PWD/bin                        # host_base + host_survey
   N=$(python3 -c 'import json;print(len(json.load(open("tools/idle-survey/games.json"))))')

   # one at a time (or GNU parallel -j6; each game is ~20-40 s on the host)
   for i in $(seq 0 $((N-1))); do tools/idle-survey/run_one.sh "$i"; done
   ```

4. **Analyze + classify:**

   ```sh
   SURVEY_DIR=/tmp/idle-survey python3 tools/idle-survey/analyze.py   # -> survey.json
   SURVEY_DIR=/tmp/idle-survey python3 tools/idle-survey/report.py    # -> report.json + A–X counts
   ```

   `report.json` carries each game's category and the decoded wait loop; the
   human-readable `snes_idle_survey.md/.csv` in `docs/` are produced from it.

## Reading the result

- **Correctness is the gate:** `analyze.py`'s `hash_diff` must be 0 for every
  game. A non-zero diff means the idle build diverged from the reference — that
  game's skip is wrong, not a win.
- **Category** (see `report.py` / `docs/idle-skip.md`): **A** already skips,
  **B** needs a generic decoder fix (no table), **C** needs a traced proof (no
  table), **D** needs a per-game template, **E** no wait loop in this scene,
  **X** unsupported/mismatch. Only **D** touches the template table.
- **Scene caveat:** title + attract only (no input). Real gameplay may wait in a
  different loop, so re-check an interesting game with a gameplay recording.

## Policy

This is a discovery tool, not a per-game optimisation by itself. The generic
skip stays generic; the only per-game artifact it can justify is a **guarded**
template entry (matched on exact ROM bytes), proven bit-exact. The histogram is
for *finding* loops — never make the shipping core's behaviour depend on it.
