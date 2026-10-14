# Changelog

## [v0.0.7]

A targeted follow-up: a generic sub-screen compositing skip that cuts
colour-math-heavy scenes, a platform-invariant APU catch-up so host and device
replays stay bit-for-bit identical, one more idle-skip template, and two
record/replay and host-input fixes.

### Performance

- **PPU sub-screen skip.** The sub-screen is composited every line while colour
  math is on, but the blend reads it only where a main-screen pixel sits on a
  colour-math-enabled layer. A per-line scan of the main z-buffer
  (`PpuLineReadsSubscreen`) skips building the sub-screen when no such pixel
  exists — `ClearBackdrop` has already left it a valid empty buffer, so those
  lines take the no-math fast composite and the output is byte-identical. The
  scan is gated behind active colour math and early-exits on the first match, so
  a plain scene never runs it and a full-screen-translucency scene pays about one
  iteration. The Adventures of Batman & Robin enables OBJ-only colour math but
  its visible sprites are non-math palettes, so the sub-screen is built and never
  read: sub build ~855K -> ~0 cycles/frame, the blend span goes with it, PPU
  total 2.05M -> 1.39M (-32%), the heavy frame off the budget ceiling (96% ->
  79%). Neutral where the sub-screen is genuinely read — Super Punch-Out's 600K
  spikes and Zelda's rain are unchanged. Device-verified; output byte-identical
  across Batman, TMNT, Castlevania and Zelda.

### Added

- **The Adventures of Batman & Robin idle-skip template.** An 80:80ed bare
  accumulator. Unlike the earlier templates it has no in-loop TCD, so the direct
  page is ambient rather than forced; a new per-template DP-guard field gates the
  skip on `cpu->dp` (re-checked each frame) so its WRAM offsets apply only in the
  context they were measured in.
- **International Superstar Soccer (+ Deluxe) idle-skip templates.** Konami's
  Dracula-shape accumulator (in-loop TCD forces DP=0) at `80:80ef` (ISS) and
  `80:80d4` (Deluxe); the two share identical loop bytes, so the head tells them
  apart and the 16-byte guard disambiguates either from the other. Each entry
  covers USA and EUR (byte-identical there). On device, Deluxe is the dominant
  idle wait across the attract/demo (~92k dots/frame skipped): cpu −16%, busy
  −12%, PPU/APU unchanged, worst late window 37→16. (ISS host-verified only.)
- **The Legend of the Mystical Ninja idle-skip template (USA + EUR).** The TMNT
  JSL+accumulator shape at `00:80b4`; the per-lap `JSL $009d27` (USA) / `$009d3e`
  (EUR, byte-identical routine) early-returns writing nothing when `[$001e2c]==0`,
  so the skip carries the same precondition as TMNT. Host-verified bit-exact over
  the attract (both regions) and an 11660-frame USA gameplay recording. It
  reclaims the title/menu/idle phases; in active gameplay the loop busy-waits with
  per-lap JSL work, so the skip is mostly inactive there (that CPU is real work).

### Changed

- **Deterministic APU catch-up.** The owed-APU-cycle accumulation in
  `snes_catchupApu` moved from a `double` — which x86 (host) and ARM (device
  Thumb-2) rounded differently, occasionally tipping the truncation by one cycle
  and desyncing the SPC700 between the two — to an integer Q32 fixed-point
  accumulator with an exact rational cycles-per-master-dot (NTSC 2136/44671, PAL
  1068/22165). Byte-identical rate, platform-invariant: host and device replays
  now match bit-for-bit, which every A/B in this project depends on. Same struct
  size, so the savestate layout is unchanged.
- Host save-state keys are now **F2 (save) / F4 (load)**.

### Fixed

- A failed or invalid state load (wrong state version, truncated file) no longer
  kills an in-progress record/replay: record/replay is closed only after the
  loaded state passes validation, so a bad load leaves the replay running.

## [v0.0.6]

The performance release. A generic idle-loop skip, a page-mapped 65816 bus, a
batch-entry Thumb-2 engine and a reworked PPU together cut frameskip across the
library, and a new deterministic record/replay system was used to verify every
change bit-for-bit.

### Added

- **Record & replay.** Record the controller from power-on and replay it
  deterministically (device pause menu, or on the host / QEMU Cortex-M7 target),
  with per-frame state and audio hashes. Host-made recordings replay
  frame-identically on the device, which is how the optimisations below were
  A/B'd bit-exactly.
- **On-device profiling.** `make PERF=1` cycle profiler; `PERF_CPU=1` breaks the
  65816 cost down by opcode (census runnable on the QEMU Cortex-M7 target);
  `PROFLITE=1` prints a cheap per-frame frameskip/audio report; the PPU
  compositing cost is split into `cwin` / `span` / `other`.
- **Per-game idle-skip templates, with a library-wide survey.** On top of the
  generic skip (below), a guarded template table (`src/snes/idle_skip.c`) covers
  the few wait loops the generic detector can't prove — accumulators (`x += y`
  every lap) and `$4212` H/V polls. Each template is matched on the exact ROM
  bytes, so it can only ever fire on the ROM it describes; a new one is added
  once its loop is found and its skip is verified bit-exact. The overnight
  survey that classifies the wait loop of 134 top games (which already skip,
  which need a generic fix, which need a template), a contributor guide, and the
  survey tooling itself are in `docs/idle-skip.md`, `docs/snes_idle_survey.*` and
  `tools/idle-survey/`.
- **Unthrottled host replays** (`SNES_FRAMES=N`, or `SNES_FAST=1`) — the host
  build drops its two wall-clock pacing gates and runs a headless replay flat
  out (~11x, a 2000-frame replay in ~2.9 s), so a multi-game bit-exact A/B takes
  seconds instead of ~30 minutes. Host-only; it touches only the wall-clock
  waits, so the emulated state/audio hash is unchanged and the device bin is
  unaffected.
- `docs/perf-lessons.md` documenting the core's performance lessons.

### Performance

Frameskip reductions are per-scene, measured on the device from deterministic
replays:

- **Generic idle-loop skip** — detects the main wait loop at the NMI and replays
  whole idle laps in one step; covers many loop shapes ($4210 / $4212 polls,
  accumulators, DP/DB changes, WRAM code, SEP/REP, indexed). Around 60% of
  surveyed games skip about half of the frame.
- **Page-mapped 65816 bus** — WRAM and ROM are served through an 8 KB page map
  instead of the per-access classifier; a HiROM low-half fast path cut the
  Final Fantasy VI intro frameskip by ~62%.
- **HDMA stall taken in one step** — after a line's HDMA transfers the CPU waits
  out the stall; the scheduler stepped it 2 dots at a time (~26,500 do-nothing
  calls/frame in Final Fantasy VI, which runs HDMA every line). Taken whole:
  FF VI busy 95.3% -> 82.8%, late frames 3911 -> 1659.
- **Thumb-2 engine** — a batch-entry `run_dots` loop keeps the dot-timing state
  in registers; cold code and the I/O slow path moved out of ITCM.
- **Fixed, cache-line-aligned code layout.** The hot code (SPC700 engine, DSP,
  PPU line renderer, 65816 bus and C fallback) is pinned in fixed-size,
  32-byte-aligned windows in the linker script, and the Thumb-2 65816 engine at
  a fixed ITCM address, so a change to one group can't shift another's
  instruction-cache sets. Before this, an edit touching only `dsp.c` moved the
  SPC700 engine and swung it ~25k cycles/frame, and bus/PPU set-aliasing cost
  the CPU ~67k cycles/frame — both now designed out, and builds are comparable.
- **PPU renderer** — Mode 7 fast paths for non-rotated lines and a split-out
  tile-cell cache for rotated / large-field lines (Super Mario Kart ~-53%);
  2bpp/4bpp full-tile run functions with a 256-entry row-decode table and
  per-repeat tile-row reuse (4bpp layers -15% in heavy scenes); colour-math
  blend spans rewritten as memoised, adaptively-ordered span functions (inlined
  they ran out of registers and reloaded the blend-table address every pixel —
  the same register-pressure trap as the Mode 7 split; composite 598K -> 437K);
  sprite slivers skip transparent rows (-21%); and the palette and colour-math
  tables are refreshed only for the CGRAM entries that changed, not rebuilt whole
  every line — in gradient/HDMA scenes the table work drops 333K -> 25K
  (PPU -306K/frame).
- **Memory** — cart SRAM and the coprocessor / idle-skip buffers (~48 KB) moved
  from the RAM_EMU pool to the cached AHB heap. This speeds the DMA that writes
  them (into cached SRAM, ~-20% on steady frames) and, because it hands the pool
  back to the hot data, also speeds the 65816 CPU work that stayed there even
  though it barely touches SRAM (Zelda cpu -3.8% vs busy -2.6%). Late frames
  collapse in heavy scenes: Zelda 89 -> 31, Castlevania 140 -> 46.
- **APU CPU time** — the three SPC700 timers advance lazily (closed-form, only
  where the sound program can observe them) instead of stepping after every
  opcode: SPC700 -15% (882K -> 748K cycles/frame), heaviest frame -362K. The
  DSP's idle-voice skip (released voices caught up in one step) is on by default
  (-85K/frame in quiet scenes). Both only became measurable once the code was
  pinned (see the layout note) — unpinned, placement noise hid them.
- **Audio playback quality** — the stretcher now derives its playback rate
  correctly across uneven ISR refills (a ratio of sums, the refill pulled in one
  chunk, folded over the two-frame half-rate pair), so the rate holds 1.0x
  instead of swinging per refill. See the matching Fixed entry for what this
  cured.
- **Display scaling** — the FULL and FIT present modes were rewritten to map
  256x224 into the 320x240 panel with no per-pixel division and 32-bit stores;
  pixel-identical output at ~154k cycles per shown frame (from ~560k).

### Changed

- Structural cleanup of the per-frame hot paths (DMA fast paths, `snes_write`
  early-returns, folded colour-math mask) — bit-identical output, fewer cycles.

### Fixed

- Final Fantasy III (USA) no longer hangs on a black screen at startup: a
  `$4210` (NMI flag) read now sees a vblank that begins inside the reading
  instruction.
- Music no longer plays flat or drops audio while the emulator is keeping time.
  The stretcher's playback-rate estimate was wrong across uneven ISR refills, so
  e.g. Zelda's title played ~3 semitones flat and threw away 12-16% of the sound
  to stay in sync. (Emulation output is unchanged — this is the playback path.)

### Install

- Unzip the release archive onto the SD card root (`cores/snes.bin`).
- Place ROMs under `/roms/snes/`

## [v0.0.5]

### Added

- Pause-menu **Controls** profiles (Auto / Zelda / Mario / Simple). Zelda
  (default with L/R): GAME modifier, GAME+A/B = L/R, GAME+TIME = Select,
  TIME = Start. Simple: no L/R, GAME = Select, TIME = Start. Mario:
  GAME/TIME = Start/Select, GAME+A/B = L/R, TIME+A/B = X/Y.

### Changed

- Nothing.

### Fixed

- Nothing.

### Install

- Unzip the release archive onto the SD card root (`cores/snes.bin`).
- Place ROMs under `/roms/snes/`
