# SNES core — performance lessons learned

Notes from optimising the LakeSNES port on the Game & Watch (Cortex-M7 @ 354 MHz).
Read `../CLAUDE.md` first for the porting/memory model; this file is about making
the core *fast* without breaking determinism.

The single most important lesson is in **[§5.1 Inlining spreads register
pressure](#51-inlining-spreads-register-pressure-contain-a-hot-helper-with-noinline)** —
read it before touching any hot renderer loop.

---

## 1. Platform constraints that shape every decision

- **CPU:** Cortex-M7, in-order, dual-issue, **weak indirect-branch prediction**.
  It hides almost nothing: a dependent load that misses, or a hard-to-predict
  branch, is paid in full, every iteration.
- **Frame budget:** NTSC is **5,890,300 cycles/frame** at 354 MHz. That is the
  only real yardstick — not host FPS, not a PC profile. When a frame overruns,
  the firmware *skips* the next one; frameskip is the metric that matters.
- **Memory tiers (fastest → slowest):**
  - **ITCM (64 KB):** the hand-written Thumb-2 65816 engine lives here (~61 KB).
    Zero-wait code. There is almost no room left — adding ITCM code can overflow
    it (PERF_CPU builds already do).
  - **DTCM:** hot *data*. Pays off only on the *access pattern*, not on being
    "hot" — several hot-table-in-DTCM A/Bs came back neutral-to-worse because the
    D-cache already covered them.
  - **AHB / SRAM:** general working RAM.
  - **VRAM, CGRAM, OAM, WRAM:** on-chip emulator RAM — **cheap** to read. This is
    why the Mode 7 "fewer VRAM fetches" idea is subtle: the fetches it removes are
    nearly free, so the win has to come from breaking a *dependency*, not from
    reducing load count (see §4.3).
  - **External ROM:** memory-mapped flash at `0x90900000` (OCTOSPI XIP),
    HW-cached in **32-byte lines**. A read that misses the line cache is the
    *expensive* one. Count **distinct cache lines touched**, not reads.
- **Build:** uniformly `-O3`, **no LTO**. Device binaries are built with the
  **Docker toolchain** (`sylverb/retro-go-sd-builder`, GCC 15.2.1), never the
  local gcc — the codegen differs and a hand-pushed local build will not match.

---

## 2. What is and isn't the lever

- **The 65816 CPU engine is hand-written Thumb-2 assembly**
  (`src/snes/thumb2/snes_thumb2.S`, ~9 kloc). It handles 255/256 opcodes in ITCM
  and only calls C for the bus (`cpu_thumb2_read/write`) and a rare fallback. It
  is already tight — **it is not the place to look for C-level wins**, and
  rewriting it is not worth it.
- **The C levers are:**
  1. the **bus/read path** (pagemap — see §4.1), and
  2. the **per-scanline PPU renderer** (`ppu.c`, especially Mode 7 — §4.2/§4.3).
- The live renderer is `g_new_ppu` → `PpuDrawWholeLine` → `PpuDrawBackgrounds` →
  `PpuDrawBackground_mode7`. **The old per-pixel path
  (`ppu_getPixelForMode7` / `ppu_handlePixel` / `ppu_getPixel`) is DEAD reference
  code** — a probe placed there never fires. Always confirm you are editing the
  live path.

---

## 3. Measurement discipline (the rules that save days)

1. **Prove the code path actually executes before attributing anything to it.**
   Add a hit counter (a `fasthit`-style `++` in the branch) and confirm it is
   non-zero on the target scene. See the cautionary tale in §4.2.
2. **Never attribute a device A/B delta to the change you intended.** A tiny
   source edit can reshuffle codegen globally (§5.1); the measured delta may be
   that reshuffle, not your mechanism. Mechanism ≠ correlation.
3. **Three measurement surfaces, each for one job:**
   - **Host replay** (`snes_host`, `SNES_REPLAY=1 SNES_FRAMES=N SNES_FAST=1`):
     deterministic, bit-exact **correctness** via the per-120-frame `REC/hash`
     (WRAM/VRAM/CGRAM/OAM/ARAM/CPU/audio). The host build does **not** run the
     Thumb-2 engine, so it proves *output*, never *speed*.
   - **GWEMU** (QEMU `mps2-an500`, real Thumb-2 engine): exact deterministic
     **counts** (opcode census, memops). Cycle reads are 0 — it is not
     cycle-accurate.
   - **Device** (PROFLITE replay): the only source of truth for **cycles /
     frameskip**. The number is `late=` frames summed over the replay.
4. **Host Ir / callgrind *understates* M7 dependent-load wins.** The same ARM7
   loop measured host −6.5% but device −19%, because the host doesn't model the
   in-order dependent-load stall. Promise nothing off host numbers alone.
5. **Judge by frameskip, not averages.** PROF averages are confounded by
   frameskip; PC histograms suspend the idle skip and lie about the top loop.

---

## 4. Landed optimisations (case studies)

§4.1-§4.4 are **generic** (see §5.2); §4.5 is the per-game Super FX exception.
All are verified host-bit-exact + device A/B.

### 4.1 HiROM low-half pagemap fast path — committed `5c253f7`

FF6 spent 13-29% of the frame budget in slow ROM reads (`slow_reads` ~13-18 K/
frame, ~99% of them HiROM low-half ROM, at 77-122 cyc each). The read pagemap
(`g_snes_rdmap`, 2048 × 8 KB pages) only granted the fast base to ROM at
`off >= 0x8000`; HiROM also mirrors ROM in the **low** half of banks `$40+`.
Adding a fast-base grant for `cart->romDmaDirect && off < 0x8000 && (bank & 0x40)`
(mirroring `cart_readHirom`) cut the FF6-intro frameskip **−62%** (2990 → 1149
late). Inert for LoROM (gated on `romDmaDirect`, which needs plain HiROM, no
coprocessor).

### 4.2 Mode 7 `dy==0` fast path — committed `4450e69`

When a Mode 7 line is axis-aligned (`dy==0`, no rotation) and `!m7largeField`,
`ypos` is constant for the whole line, so the tilemap row and in-tile row are
fixed. Fetch the tile index only when the column changes and keep the char-row
base — this breaks the per-pixel dependent map→char load. Bit-identical for this
case (F-Zero / FF6-snowfield ground-plane class).

**⚠️ The cautionary tale.** The commit *appeared* to give Super Mario Kart −36%
on the device. It did not: a `fasthit` counter proved the fast path **never fires
for SMK** (its track sets `m7largeField` every line). The −36% was an *incidental
codegen reshuffle* — adding the guarded `if/else` made GCC re-allocate and
re-schedule the whole inlined `PpuDrawBackgrounds` (842 changed instructions), and
the *rotated fallback* happened to come out faster. Real, device-measured, and
bit-identical — but **accidental and fragile**. This is why §3.1/§3.2 exist.

### 4.3 Mode 7 rotated/large-field cell cache — the headline result

The rotated / large-field fallback refetched the tilemap **every pixel**:
`tile = vram[cell]; pixel = vram[tile*64 + ...]` — a dependent map→char load
pair, the thing the in-order M7 pays for in full. Cache the tile base per 8×8
cell and refetch the map only when `(xpos>>11, ypos>>11)` changes
(≈ every 8 pixels at SMK's scale), which breaks the dependency. Works for any
`dy` and for `m7largeField`, so it covers SMK's rotating track.

The mechanism was right — but **how** it was compiled decided everything:

| Build | SMK late (8022 frames) | vs HEAD |
| --- | --- | --- |
| original (no cell cache) | 782 | — |
| HEAD (`dy==0` + accidental, §4.2) | **499** | baseline |
| cell cache **inlined** | **1137** | **+128 % (regression!)** |
| cell cache in a **`noinline` function** | **233** | **−53 %** |

FF6 intro stayed flat (~1128 → 1131) — its path is untouched. See §5.1 for why
the same mechanism went from a 2.3× *regression* to a −53% *win* purely on the
inline/noinline choice. This is the commit this doc was written for.

### 4.4 PPU sub-screen compositing skip — committed `97f6566`

When colour math is active the renderer builds a second ("sub") screen every
line and the blend reads it — but **only** where a main-screen pixel sits on a
colour-math-enabled layer. On a line with no such pixel the whole sub-screen
build is unobservable. Gate it: scan the just-drawn main z-buffer
(`PpuLineReadsSubscreen`, layer = bits 8–11) against the math mask and skip the
build when nothing matches. `ClearBackdrop` has already left the sub buffer a
valid empty buffer, so those lines take the composite's no-math fast path and the
output is bit-identical.

Self-gating and generic: the scan runs only when colour math is already active
(the lines that would build the sub anyway) and early-exits on the first match —
so a plain scene never runs it, a full-screen-translucency scene pays ~one
iteration, and only a sparse-coverage scene pays the full scan, which is exactly
the scene that then skips the whole build.

| scene | sub-screen usage | PPU/frame | result |
| --- | --- | --- | --- |
| Batman & Robin (heavy) | built, never read (OBJ-only math, sprites are non-math palettes) | 2.05M → **1.39M** | **−32 %**; replay late 37.4 % → 32.5 % |
| TMNT IV | partly read | 1.13M → 1.11M | −2 % |
| Super Punch-Out | genuinely read (600K spikes) | 1.89M → 1.89M | neutral |
| Zelda (rain) | read in the effect | ~1.05M | neutral |

The win was **found by measurement, not guessed** (§5.7). The first cut skipped
only sprite-free lines (`!lineHasSprites`); a throwaway ceiling probe then showed
the sub-screen was ~100 % *unread even on the sprite lines*, because Batman's
visible sprites are non-math palettes (layer 6, exempt). So the correct gate is
"is any main pixel on a math-enabled layer?", which removes almost the entire
build — not the x-range clipping of the tile renderers that was originally
planned (more code, more risk, smaller win). Output bit-identical across Batman,
TMNT, Castlevania and Zelda.

Footnote to §5.1: the scan lives in a helper tagged `NOINLINE`, but that macro is
a gcc no-op here so it inlined anyway (+32 bytes in `ppu_runLine`). Harmless
*this* time — the scan carries no loop-carried per-pixel state, so it added a
self-contained loop without reshuffling the hot pixel loops; the disasm diff
confirmed those stayed byte-identical. The §5.1 blow-up only happens when the
inlined code carries *loop-carried* state that competes for the hot loop's
registers.

**Shared layers between main and sub screen: tried, not kept.** Zelda's sub
screen never draws a layer the main screen draws (0 of 42,191 sub passes),
but Super Mario RPG's is BG1 and the sprites, both also on the main screen.
Handing the main buffer to the sub screen after those layers is exact (every
frame's picture hash unchanged) and took 5-12% off the PPU per drawn frame in
those scenes on the device -- which were not the heavy windows. Its 280 bytes
in `ppu_runLine` (first a 32-byte shift of everything behind the PPU window,
then, re-laid out without it, still ~0.4% on the other scenes) left no net
gain. A renderer change pays only where the frame is PPU-bound.

### 4.5 Super FX (GSU): per-game block code, and the PLOT that called `memcpy` — branch `superfx-poc`

The GSU is the ZSNES 2 core (`src/snes/gsu_zsnes/`): one table dispatch per
opcode, state in globals. Yoshi's Island at the start (device log 3, the 5.89M
budget): title screen 156% busy with the GSU alone at 102%, gameplay 102% busy
with the GSU at 23%.

**Block code** (`tools/gen_gsu_blocks.py` → `gsu_blocks_<game>.inc`, built in
`gsu_blocks.c`): the game's hot GSU code compiled to C, as in the arcade NEC
`nec_hle_build.py`. A host histogram per phase (`GSU_PCHIST`, windows by
`GSU_PCHIST_FRAMES`) selects the hot addresses; the generator decodes them into
*chains* (straight-line code through untaken branches, prefixes resolved,
registers and flags in locals), joins chains that jump into each other into one
C function (shared locals, gotos), and keys entries by (bank, PC, prefetched
opcode). Exactness rests on three things: every template is the ZSNES handler
transcribed, quirks included; the budget is checked per label so a chain never
runs an instruction MainLoop would not have; a ROM-bytes hash enables the table
only for the matching dump. Verified with `GSU_STATELOG` (per-frame hash of the
whole GSU state + Game Pak RAM, blocks on vs `GSU_BLOCKS=0`: byte-identical over
19,219 frames), the 160 device REC hashes, and a negative control.

What the chains had to learn, each found by measuring where control left them
(`GSU_CHAINHIST`): delay slots that hold a *prefix* (TO R14 in a branch delay
slot takes its opcode from the branch target), LOOPs whose delay slot is a prefix
(R13 found by following the `IWT R13` that set it — the histogram can't see the
target), `JMP R11` returns (resolved against `LINK` return addresses). Joining
chains cut gameplay chain calls from 2,047 to 139 per frame.

Budget: ~50 KB of ARM in a ~54 KB pool, so the selection is per phase (gameplay
89%, title 95%, intro 75% of instructions), the lookup table runs at a 2/3 load
factor, and the coldest 5% of functions are `optimize("Os")`.

**The PLOT.** After all that, the title screen still spent ~half its GSU time in
PLOT. Moving the Game Pak RAM from uncached AHB SRAM into DTCM changed nothing
(log 9: −1%), so it was not memory. The disassembly was: `fx_drawpix` read and
wrote each bitplane word through `ld16u`/`st16u` (`memcpy` for unaligned safety),
and with `-mno-unaligned-access` GCC emits a **real `bl memcpy`** for a 2-byte
copy — four library calls per 4bpp pixel, in the interpreter's PLOT handlers as
much as in the chains. The addresses are always even, so `ld16a`/`st16a`
(`__builtin_assume_aligned`) compile to one `ldrh`/`strh`. See §5.8.

| Yoshi's Island, device | log 3 (interpreter) | log 8 (chains) | log 10 (+ aligned PLOT) |
| --- | --- | --- | --- |
| title: busy / GSU | 156% / 102% | 102% / 41% | **87% / 25%**, audio no longer slowed |
| story panel (frame 9480): busy / GSU | — | 81% / 25% | **71% / 14%** |
| gameplay: GSU | 23% | 8.5% | **7.8%** |
| late frames intro / gameplay | 4314 / 4395 | 200 / 2634 | **157 / 2554** |

Gameplay's remaining late frames are an every-other-frame overrun (average busy
86-93%, heaviest frame ~125%); the GSU is now ~8-10% of that frame.

**"late" is not lost time.** `PROF/pace late=` counts frames that reach the
audio-DMA wait after its tick. At 30 fps the drawn frame is the heavy one, and
the undrawn one after it absorbs a sub-period overrun: `wall_ms` stays at 1995
per 120 frames (real time) and the audio at 1.000 with dozens of "late" frames.
Real time is lost only where one frame overruns by more than a period; the
pacing resets its reference every frame, so those ticks are never repaid and the
audio ring drains (`underruns`, `step_min` 0.84-0.89). Read `wall_ms` and
`underruns`, not `late`, to find the frames that matter.

**A spike is often code nobody profiled.** Host `FRAME_STATS` (per-frame 65816,
SPC700, GSU instructions and DMA bytes) put the intro's worst window (229%) on
four frames of the "THE BABIES ARE MINE!" panel, when Kamek flies off: GSU
96K instructions against ~13K, 48% of them interpreted (`GSU_CHAINHIST`), nearly
all one 7-instruction GETC/PLOT loop at `08:9055` run 46,870 times. The intro
histogram had been taken at 75% coverage, which left this rare scene out. A
histogram of the two bursts added to the selection compiled it (2.8% and 0.7%
interpreted after): device, logs 17 → 18, those windows' heaviest frame
229% → 104% and 198% → 94%, no time lost, underruns 590/603 → 0.

**A second game, and one overlay per game.** The chains first lived in the
GSU overlay, so every Super FX cart loaded Yoshi's 85 KB. Each game's blocks
are now their own overlay record (GSGM), linked right behind the GSU overlay
and read with it as one pool allocation for the cart whose internal-header hash
matches; `gsu_blk_select` still checks the ROM bytes before using them
(`tools/overlay_records.py` shows which record a cart loads). Star Fox's chains
came from one device recording split into three phase histograms (intro, title,
gameplay at 95%): 94% of its GSU instructions compiled, state identical with
chains on and off. Device (Star Fox logs 2 → 3): GSU 67% → 20% (intro),
32% → 8% (title), 56% → 16% (gameplay) of the frame budget; the title back to
real time, intro and gameplay from ~1.65x to ~1.25x. What is left there is the
PPU (41-43%) and the 65816 (37-40%), no longer the GSU.

A later recording reached the mission map and the zoom onto Corneria, which
none of the three phases had: 89% of their GSU instructions interpreted, GSU
85-210% of the budget. One routine (01:b674-bbf2, the textured planets) was
86% of that; a fourth histogram of those frames brought them to 98% compiled
and the record to 108 KB (4.5 KB of pool left). Device (logs 6 → 7): those
windows 149% → 72% busy, the worst 268% → 79%, underruns over the replay -86%.
Level 1 was then 92% compiled: the same addition took it to 94% and its GSU
21% → 19.5%. Use `tools/gsu_hist_report.py` on a new scene's frames before
assuming chains will help.

**What an interpreted instruction costs, and level 1's own histogram.**
`PERF_GSU` on the device, the vblank catch-up split out (log 15): level 1's
GSU was 21% of the budget, the chains 13 cycles per GSU instruction (PLOT
apart), the 7.8% left to the interpreter ~100 each (over the 104 windows:
100.6 per interpreted instruction plus 52K a frame) -- 38% of the GSU time
for 8% of its instructions. The catch-up burst (35% of the GSU work: the rest
of a 3D frame, finished at vblank, from the upstream ZSNES fork) has the same
profile. The selection had never had level 1's histogram: adding it at 95%,
the map/zoom one trimmed to 90% and `--hot 0.90`, took the record from 108 to
114 KB (the zoom's own code is 4.7 KB; most of its chains are shared) and level
1's interpreted instructions from 4,228 to 3,433 a frame -- a third less than
the histogram promised. 38% of what stays interpreted runs at compiled
addresses: a chain runs whole loop iterations, so when a scanline's slice ends
mid-loop the interpreter finishes the iteration and the next slice resumes
mid-loop until it meets an entry, about one iteration per slice boundary,
which no histogram shows. With the chain-entry filter (`gsu_blk_filt`, a
miss in the table 27 → 13 instructions) on top, device (logs 14 → 16): level
1 GSU 1,139K → 998K cycles/frame, lower in all 83 windows, busy 99.0% →
96.6%, heaviest frame 241% → 205%, silence halved; the map and zoom +36K
(still 71% busy). Regenerate with:
`gen_gsu_blocks.py --rom starfox.sfc --hist h_game.hist:0.95 --hist h_intro.hist:0.95
--hist h_title.hist:0.95 --hist h_launch.hist:0.90 --hist h_level.hist:0.95
--plot c_FxOp4C1284bzd,c_FxOp4C1284bz --hot 0.90 --name starfox ...`.
`PERF_GSU_PLOT` (the PLOT split) no longer fits Star Fox's record: its probes
are inlined into every chain.

**Then the PPU: offset-per-tile.** Star Fox's 3D scenes are BG mode 2 with
OPT on, which leaves the fast tile walkers for `PpuDrawBackground_opt`: host
callgrind put it at 82% of the PPU's instructions, ~72 per pixel -- every
pixel's 4 bits pulled out one by one, two BG3 tilemap reads per column per layer
(`ppu_handleOPT`). It now decodes each tile once to chunky nibbles (the table
the fast drawers use), skips blank tiles, and reuses the offset words the
per-pass "is OPT on" check already reads. Host: the OPT drawer -60%, PPU -44%.
Device (Star Fox logs 3 → 4): PPU 41-43% → 24-26%, intro and gameplay 1.25x →
~1.07x real time, underruns -90%; CPU, GSU, APU and the hashes unchanged.
Placement cost two pins: the drawer moved behind `ppu_runLine` (only OPT lines
run it) and the 4bpp drawer is pinned, because taking the OPT checks out of
line shrank `PpuDrawBackgrounds` and would have shifted every drawer behind it.

### 4.6 SPC700: batch the opcodes inside the Thumb-2 engine

In Yoshi's Island the SPC700 cost is real driver work, not polling: a host
histogram (`APU_HIST`, `tools/spc_hist_report.py`) put the driver ~97% busy in
gameplay (DSP shadow-register loop, per-voice pitch maths), with the idle skip
firing as designed. So skipping more was not the lever; the cost per opcode was.

`PERF_SPC` measured that cost before changing anything (device log 12, waterfall
room): ~130 cycles per SPC700 opcode, of which ~99 inside `spc_runOpcode` +
engine and ~31 in `apu_run`'s C loop around it (cycle commit, DSP-boundary test,
the call). 3,569 opcodes/frame × 130 ≈ 465K. The fix moves the loop into the
assembly: the engine keeps `Spc*`, ARAM, `Apu*`, `&apu->cycles` and the budget in
registers, commits each opcode's cycles itself, and calls `apu_dsp_ticks` only on
opcodes that cross a 32-cycle boundary (at most ~533 DSP ticks a frame against
3,569 opcodes). It returns to C for exactly
the cases `apu_run` handles differently, so the C loop stays the single source
of those rules (the idle-wait bytes are matched in asm only to stop the batch).

| Yoshi waterfall, device, PERF_SPC | log 12 (per opcode) | log 13 (batch) |
| --- | --- | --- |
| engine calls / frame | 3,569 | 45 |
| SPC700 / frame | 507K | 321K |
| outside the engine (apu_run loop) | ~112K | ~6K |
| inside the engine, per opcode | ~99 | ~88 |
| busy | 103.3% | 100.0% |

Two things to carry over. The probes themselves cost ~42K/frame at per-opcode
granularity: compare a probed build only with a build probed the same way, and
discount it against a normal one. And the per-opcode figure *inside* the engine
also fell (99 → 88) — entry/exit (push/pop, re-deriving bases) was part of every
opcode's cost, invisible until the loop moved. The C fallback (246 opcodes,
~22.5K/frame, ~7% of the SPC700) is now the measured baseline for native
handlers; the DSP (~530K) is the larger half of the APU.

### 4.7 DMA: read the source where `snes_read` would, and write ports as blocks

Yoshi's waterfall room showed `dma=562K` a frame for only ~4,400 bytes moved
(`DMA_HIST`, `tools/dma_hist_report.py`): ~128 device cycles a byte. Not the
volume and not the `dma_cycle` calls (~1,350 a frame); the per-byte path. Every
byte from a LoROM, Super FX or WRAM-mirror source ran the full decode
(`cart_read` → `cart_readLorom` → `cart_gsu_read`, or `snes_read`'s ladder)
although the bank is fixed for the whole transfer. Only WRAM `$7E/$7F` and
plain HiROM banks had a direct pointer, which is why HiROM games never showed it.

The fix reuses what the CPU already trusts: `g_snes_rdmap`, the 65816's 8 KB
page map, gives a host pointer exactly where `snes_read` reads plain memory, so
one table load per byte replaces the decode (NULL pages still go through
`snes_read`); the Super FX Game Pak RAM gets its own flat case. Bytes are still
read one at a time, so a target that writes WRAM is seen. Reading the page map
against `cart_readLorom` found one place they disagree (LoROM under 2 MB with
SRAM, `$70-$7D:$8000+`, noted in `snes_pagemap_base`); DMA keeps the slow path
there.

Device, waterfall (logs 13 → 14, same PERF level): DMA 562K → 320K, busy 100.0%
→ 95.8%, late frames 113 → 60 of 120; CPU, PPU, APU, GSU and the 65816 opcode
count unchanged. The first build overflowed the 9 KB BUS window: the three flat
loops became one (`src[(ofs | adr) & mask]`), and `ppu_write`/`ppu_read` are now
pinned at their measured offset so DMA edits can't move them.

**Then the write side.** Every byte still went `snes_writeBBus` → `ppu_write`'s
switch → its port handler. The four data-port cases ($2104, $2118, $2119, $2122)
moved, unchanged, into always-inline helpers that both `ppu_write` and a new
`ppu_dma_write` use, so the side effects (remap, increments, palette dirty bits,
line-cache bumps) are one piece of code. A transfer whose four B-bus targets
are one port group, from a source without side effects, is read into a 128-byte
buffer and written as blocks; a byte on a page the map does not serve is read
only after the bytes before it are written. `ppu_write`'s instruction stream is
unchanged (612 instructions, diffed). Device (logs 14 → 15): DMA 320K → 238K,
busy 95.8% → 94.4%; late frames stayed at 60 (the overruns are the ~129% frames).

Placement: the A→B transfer became its own function, placed with
`ppu_dma_write` in the padding between the BUS and CPU windows, so `dma_doDma`
shrank (1,016 → 424 bytes) instead of pushing the window; HDMA, `ppu_write` and
`ppu_read` are pinned at their measured offsets. The first try was caught by
that pin (`cannot move location counter backwards`) rather than shipping a
shifted window.

**Then the VRAM word pairs.** Before writing it, `DMA_HIST` recorded VMAIN for
every VRAM transfer to count what would qualify: mode 1 to `$2118` with the
increment after the high byte was 76% of the waterfall's DMA bytes, 53% of the
whole Yoshi replay, 27-31% in Castlevania and Batman, and the cases that cannot
pair (increment after low, byte-only, odd start) ~2 bytes a frame in all six
runs -- a general case, not a per-game one. Such a pair lands on one word, so
it is one remap and one store instead of two read-modify-writes. Device (logs
15 → 16): DMA 238K → 191K, busy 94.4% → 93.8% (the other figures +7K, within
what two logs can tell apart). Over the three steps: DMA 562K → 191K (-66%).

### 4.8 Idle skip: a Super FX wait compares only the bits it tests

Star Fox's 65816 waits for the GSU with `LDA $3030 / AND #$20 / BNE`. The idle
skip knew the loop, but its proof snapshotted the whole status byte, and the
GSU's own flags in that byte (Z/CY/S/OV) change on every slice it runs, so
nearly every span was refused; the 2-per-frame re-proof budget then ran out
and half the frame's opcodes (~4,200 of ~8,250) were the poll, interpreted.

The loop decoder now records an `AND #imm` right after the status read, and
the skip tells three cases apart: the tested bits changed (no skip, the loop is
leaving), the whole byte unchanged (the exact path, as before), only untested
bits changed (skip whole laps from the loop head). The stored mid-lap states
hold the old byte, so that last case does not use them: a span that starts
mid-lap first runs the rest of the lap for real, in a `noinline` helper so the
ITCM code grew only 178 bytes. Generic for any GSU poll of this shape.

Device (Star Fox logs 4 → 5, hashes equal in all 39 windows): gameplay CPU
2.36M → 2.03M cycles/frame, 65816 opcodes 8,447 → 7,146, busy 103% → 97%,
underruns over the replay 8,112 → 3,536. What remains refused (~2,200/frame)
enters the loop with different registers than the proven lap; accepting those
needs to know which registers the loop reads.

### 4.9 Code run from WRAM: the fetch page

The engine's inline fetch serves bytes from one cached page, which only
`snes_cpuRead` installs, and only for ROM. In Star Fox's level 1 half the
65816 opcodes run from WRAM, so every fetched byte there paid the call. The
page map now installs WRAM pages too (`$7E/$7F` and the low mirror). Only
fetches reach `snes_cpuRead` for those pages (data reads are served by the
engine's page-map hit), and the page points at live memory that is never
remapped, so a write to it needs no invalidation. Generic.

Device (`PERF_CPUB`, Star Fox logs 9 → 10, hashes equal in all 105 windows):
level-1 batch engine 1,540K → 1,423K cycles/frame, lower in every window, busy
101.8% → 99.8%. gwemu and host agree on eight recordings
(`tools/qemu_vs_host.py`); a build serving WRAM fetches one byte off diverged.

### 4.10 Audio: pace the frame loop on the playback queue, not on the audio tick

The frame loop waited for the next audio DMA tick (one per frame, 266 samples
at 16 kHz) and then dropped whatever lag the frame had left. Light frames
could not bank time, and a frame that overran a whole tick lost it for good, so
the stretcher's cushion (640 samples) could refill only by playing slower. In
Star Fox's level 1 it sat near empty with the rate at its floor, and every
spike ran it dry.

Two attempts that did not work, measured on the device (Star Fox level 1):

- **Catch-up on the tick** (`SNES_CATCHUP_MAX`, up to 3 ticks of debt repaid by
  not waiting): +1.7 points of speed, but the stretcher had already slowed the
  music for that same lag, so the repaid audio arrived as surplus and its
  overflow drain threw it away: 11,856 samples (0.74 s) against 5,158 without
  catch-up (logs 12, cap 3 / cap 0). It stays for the stretcher path.
- **Direct audio paced on the tick**: the queue never held a spare frame. A
  frame late by less than a whole tick leaves no debt, so nothing repaid it;
  spares came back only after whole-tick overruns, and the queue swung between
  full (frames dropped) and empty (silence) in the same 2 seconds.

What works (`SNES_AUDIO_DIRECT=1`, the default): the samples go straight to the
ISR ring, and the frame wait paces on the ring itself. It waits only while the
playing frame plus `SNES_DIRECT_SPARE` (3) more are queued, so any overrun,
however small, shows as a lower queue and the lighter frames after it refill
it. The pitch and tempo never change; where the core is slower than real time
for longer than the queue covers, the sound has gaps. Device (logs 13 → 14):
the intro and the map 2.9 → 0 ms of silence per 2 s window; level 1's heavy
windows play ~6.8% silence, the time the core loses there. The stretcher's
work in the main loop went too: ~0.6% of the budget.

The lesson: a buffer only moves time. A spike is absorbed only if the frames
around it are far enough under budget to refill it. In Star Fox's level 1 the
3D frame peaks at ~180% every few frames and the frames between barely repay
it (`PROF/qev`: the queue holds one frame before each silence), so what is left
there is CPU time to find, not pacing. `PROF/pace` (`dropped`, `fill_min`),
`PROF/direct`, `PROF/audio3` and `PROF/qev` (`PERF_AUDIO`) measure it, and
`tools/prof_log_compare.py` sums them per phase.

### 4.11 SA-1: two 65816s, one interpreter — branch `sa1-poc`

The SA-1 is a second 65816 at 10.74 MHz beside the main one. Its CPU is ours
(`cpu.c` compiled again as `sa1_cpu.c` on the SA-1's bus); the chip logic is
ZSNES's. What the bring-up and the first device logs taught:

- **Interleave at the instruction, not the scanline.** With the SA-1 run once a
  line, Kirby's Dream Land 3 crashed: when the SA-1 asks the 65816 to run a
  routine, the 65816 waits four `XBA`s before re-reading the message register,
  so a reply one line late ran the routine twice (the second time with the
  SA-1's "list done" index of -1). `sa1_sync` now runs the SA-1 up to the dot
  before every 65816 instruction -- including the one `cpu_tick` runs at an
  event boundary, which had been missed.
- **Most SA-1 instructions are waiting.** snes9x's 36-59K SA-1 instructions a
  frame (the old "not viable" verdict) are 79-99% one polling loop. A jump back
  to a loop head whose next arrival finds the CPU identical, after a lap that
  wrote nothing and read no clock or side effect, with nothing outside having
  happened, proves the loop: the SA-1 sleeps, its clocks accrue, and the next
  65816 write it could see wakes it, charging the skipped laps in one step. The
  65816's idle skip takes loops polling I-RAM/BW-RAM only while the SA-1
  sleeps. Host, both skips on vs off: WRAM, I-RAM, BW-RAM and both CPUs equal
  every frame for 2,400 frames of three games.
- **Make the clock independent of how often you call it.** The first on/off
  check differed: `sa1_run` halved master clocks with `>> 1` per call, so
  fewer, larger catch-ups (the 65816 skipping laps) lost fewer half clocks.
  The odd clock now carries over.
- **A per-byte call into another file is the slow path.** Both CPUs' ROM and
  BW-RAM reads go through 8 KB page maps, rebuilt only on a real change (Mario
  RPG rewrites `$2225` with the same value thousands of times) and only the
  pages that moved.

Device, Kirby Super Star intro + attract (PROFLITE → `PERF`, log 1): 43-69% →
83% of real time on average, the light windows at 100%. What is left is real
work: in the slow windows the SA-1 runs 20-23K instructions a frame at ~216
cycles each (C, from RAM) and the 65816 single-steps at ~460 cycles an
instruction, because the SA-1 is awake. Letting the 65816 batch while the SA-1
sleeps (log 2) gained ~1 point: the windows where it sleeps were already cheap.

- **A second copy of the main CPU's engine bought 17%, and the DWT said why.**
  Assembled for the SA-1, `snes_thumb2.S` ran ~170 cycles an SA-1 instruction
  against C's 204. Stall counters around each instruction (CPI, LSU, fold,
  calibrated on empty windows) gave ~90 ARM instructions at ~1.4 cycles each,
  memory waits 4%, ITCM placement 2.5%. Inlining its bus bridges (2.4 calls an
  instruction fewer) saved only 12 cycles. The main CPU's engine wins over its
  C because that C calls `snes_cpuRead` per byte; the SA-1's C already had its
  bus inline, so the copy kept only the per-instruction bookkeeping: A/X/PC/
  flags loaded and stored in memory, the m/x width tested at run time, the
  loop-head test after every instruction -- 68 instructions for an INX.
- **State in registers, a table per width, small.** `sa1_engine.S` keeps the
  65816 in r4-r11 for the slice, fetches with one `ldrb` from a pointer into
  the ROM page (re-looked-up only when it leaves the page), sets N/Z with one
  `sxtb`/`sxth`, charges cycles with one `subs`, and dispatches from each
  handler's tail; INX is 9 instructions. The wait-loop proof runs only after a
  jump back, on the engine's own snapshot. Generated from templates; opcodes
  under 0.02% of the three games' SA-1 work go to C (≤0.2% of it), which
  halved the code to 23 KB. Device, same replay: 87.8% → 95.7% of real time.
- **BW-RAM is strongly-ordered on the device (AHB SRAM, MPU region 0).** An
  unaligned halfword there faults, so the engine reads data words bytewise and
  takes halfword operand fetches only from ROM pages; I-RAM (in the `Sa1`
  struct, DTCM) takes them.
- **Verify the engine on gwemu, prove the check can fail.** The QEMU harness
  (`make qemu`, RAM moved to the board's 16 MB PSRAM to hold a 4 MB ROM) runs
  the whole 6,494-frame replay in ~36 s; all 54 state hashes equal the host's.
  A deliberately wrong INX for x = 1 changed nothing (the SA-1 runs with 16-bit
  index registers there); wrong for both widths, the hashes split at frame 120.
- **Find the poll before fixing the poll.** The device log pointed at an I-RAM
  wait loop (the idle site); a host PC histogram of just the heavy frames (two
  runs, `SNES_FRAMES` 4800 and 4920, subtracted) showed it at 5% of the
  65816's instructions there and a WRAM loop `BIT $2300 / BEQ` at 59%: the
  65816 waiting for the SA-1's message, ~2,200 laps a frame, each a contact
  that runs the SA-1 seven instructions. The skip refused it twice over: a
  register read, and never the active site (only the NMI picked one).
- **Skipping while the other CPU runs is still exact if you sync it to each
  read.** Per lap the SA-1 is brought to the dot of the lap's one SA-1 read and
  the polled bytes compared; the 65816 is not run. An IRQ that catch-up raises
  must not be seen before that read: the lap then runs up to the read with the
  line held. A loop entered with other registers at each request (Y) needs a
  proof per request: its own budget, 128 a frame. Host: 20.8M laps skipped
  instead of 15.0M, hashes equal; syncing two dots late broke 51 of 54.
- **Run ahead to the write, exact to the clock.** A poll at dot h sees a write
  whose instruction started with debt D when the clocks credited up to h
  exceed (credited to the end) - D. So the engine charges an instruction's
  operand-dependent extra cycles at its end (until then r11 is its start
  debt), records r11 at every C write, and stops after a register write that
  changes `$2300`/`$2301` or the 65816's interrupt lines; the SA-1 is then put
  back to the first lap that sees it. Two traps, both caught by the hashes: in
  C the next instruction overwrote the recorded debt (break right after the
  write), and the polled bytes must still equal the proof's when the run-ahead
  starts (it watches for a change from now). A one-clock error in the recorded
  debt broke 50 of 54 checkpoints on gwemu.
- **SA-1-only code in core RAM costs every game.** The span runner (2.2 KB, its
  proof inlined) pushed Star Fox's GSU blocks out of RAM_EMU by 332 bytes; in
  the SA-1 overlay (`.sa1ovl.text`) it is loaded only for an SA-1 cart.
- **Ask why a skip does not fire before improving it.** Counted on the host
  for the heavy frames: of 809 laps of the `$2300` wait still run, ~450 were
  in spans that began in the line's HDMA stall, which `run_dots` never offered
  to the idle skip -- not a limit of the skip at all. Offering the rest of the
  span once the stall is taken (a check per stall, none per opcode) took
  windows over budget from 9 to 4; FF3 and Super Mario Kart skip more too.
  `tools/host_rec_hashes.sh` replays any ROM with its `.rec` for the checks.
- **An exact skip can still cost: check the other windows and the other games.**
  Offering the idle skip again after a mid-span poll (the `$2300` read setting
  `hclip`) was exact on every recording, and helped the windows it targeted,
  but its extra proofs failed in the lap where the SA-1's message had just
  arrived and used up the proof budget: Kirby Super Star's window 44 went
  102% -> 109%. Removed; the partial-lap step alone left no window over
  budget. Every change since is checked on the six host recordings, on Mario
  RPG and Kirby's Dream Land 3 with `tools/host_attract_compare.sh` (same state
  every frame, no window heavier), and on gwemu, before a device build.
- **A second game finds the next limits.** Super Mario RPG's heavy scenes
  (the scrolling isometric maps) were 65816-bound on loops the skip refused:
  one reading two or three SA-1 addresses a lap, and `$4212` polls. Per lap
  the SA-1 now goes to each read's dot in turn (a change seen at read r: the
  stored lap up to it, then the CPU runs from there). `$4212` is exact over a
  span: vblank and hblank change only on its boundaries, the auto-joypad bit
  at a known dot, so the value at the span's first and last dot must equal the
  proof's. Device: windows over budget 59 -> 54, over 110% 40 -> 22.
- **More proven loops need more site slots.** With the new loops accepted, 8
  idle sites evicted each other and five Kirby Super Star windows got heavier
  on the host (`tools/host_attract_compare.sh` flagged them); 12 sites, none.
- **Count the sync calls that run nothing.** In Mario RPG's heaviest window
  DMA was 12.8% of the budget: 3,616 bytes a frame DMA'd from BW-RAM to the
  PPU, each through `snes_read`, the cart and a sync of the SA-1 to a dot it
  had already reached (3,616 calls, 0 runs). A transfer runs at one dot, so
  one sync then a flat read is the same machine: DMA 12.8% -> 1.5%.
- **An opcode set chosen on one game's histogram misses the next game's.**
  The engine's memory opcodes were picked on Kirby Super Star and two
  attracts; Mario RPG's gameplay sent 9.8% of its SA-1 instructions to C in
  the heavy scenes (MVN alone 6%). Every m/x variant of the missing opcodes
  cost 9.2 KB; only the forms that run (these games keep x = 0), at least 0.1%
  of the instructions each, cost 4.3 KB for the same 0.6% left in C
  (`tools/sa1_engine_coverage.py` reads the engine's dispatch tables against
  `SA1_PCHIST`). The gain was smaller than estimated (SA-1 share -3 to -4
  points, not -10): a C fallback costs less than assumed.
- **gwemu runs only replays: give the attract an empty recording.**
  `tools/make_idle_rec.py` writes one, so a game without a recording (Kirby's
  Dream Land 3) gets its engine paths checked there too.
- **A loop the NMI never lands in is never found.** Mario RPG's intro window
  (124%) was the 65816 polling I-RAM `$3002` in WRAM, 94% of its
  instructions, while its NMI always landed in the frame wait; no sample ever
  looked there (`IDLE_TRACE` showed no decision at that pc). Two spans
  starting within a loop's length of each other mark a loop: search there.
  Two traps, both seen in `tools/host_attract_compare.sh` before any device
  run: a DMA keeps the pc still across spans and used up the searches (the
  same pc is no loop), and arming every loop found let a `$4212` wait take the
  frame's two proofs from the vblank wait the NMI had found (the first windows
  +25%); arming only loops that poll SA-1 memory left no window heavier.
- **A search that fails every frame is a fixed cost: count its failures.** On
  the device the first version took window 8 from 124% to 114% but made 47
  windows 1-3 points heavier with the same instructions, laps and span-runner
  time: it ran its 8 searches a frame and failed (96,681 rejects over the
  recording, against 724), because Mario RPG's own code often starts two
  spans within 32 bytes and 4 remembered rejects could not hold ~200 pcs a
  window. Requiring the pc to repeat one of the 3 span starts before the last,
  and a 64-entry reject table, left 3,327 and no window heavier.
- **Count the ARM instructions before guessing at stalls.** The SA-1 cost
  48-99 device cycles per instruction. `tools/qemu_exec_profile.py` logs every
  block gwemu runs over a few frames and counts SA-1 instructions through the
  engine's dispatch table: 37 ARM instructions each, ~1.9 cycles apiece -- the
  work, not memory. 40% of it was entering, resyncing and leaving the engine,
  and most resyncs came from REP/SEP, which never needed one unless I was
  cleared. Fewer resyncs and inline BW-RAM writes: 30.8 instructions, 4.4%
  fewer device cycles per SA-1 instruction (9% in the heaviest window) -- the
  profiled frames overstated the gain.
- **A loop reading `$4212` has two proven laps, not one.** Its h-blank bit
  moves twice a line and `BIT` copies it into V, so a span in the other half
  of the line is another lap: Mario RPG's vblank wait was refused 241,672
  times by frame 3,720. A second proof per value was not enough by itself
  (65816 instructions +17% on the host): a span starts with V as the last lap
  of the *other* half left it, so its state matches no stored one and every
  attempt to prove from there failed and took the frame's two proofs. Three
  parts made it pay: a proof that comes back with a different flag gets one
  more lap from there; a `$4212` loop's span that matches nothing runs one
  lap for real before it asks for a proof; and the second proofs have their
  own budget. Then -42.8% on the host. The resume step was also applied to
  every loop at first and cost FF3 1.1% in one window -- a loop that reads
  only WRAM has no stale flag, its other state is a new entry -- so it is
  `$4212` loops only. On device the heavy windows lost 2-4 points, the
  skipped laps' ~70 cycles each; they are now SA-1 work (34-41%) and the
  65816's own (38-43%).
- **Room is found by size, not by moving the overlays.** The coprocessor
  overlays are not pinned: they follow the core's bss, and Star Fox's GSU
  blocks end ~1 KB short of RAM_EMU's end, so 2.2 KB of new code broke the
  link. Most of it was codegen, not code: the proof copies' struct loops
  unrolled to 1.1 KB each (word loops with `GCC unroll 1` and no
  `memcpy` recognition: ~130 B), and three copies of the run-one-opcode loop
  (one shared `idle_step`). What runs every span (the swap and its copies)
  sits in ITCM next to the span runner, without a call out to RAM; what runs
  opcodes sits in RAM with the C interpreter it calls; the proof slot is in
  DTCM. Star Fox's margin 468 B, ITCM 3.4 KB free.
- **Make the regression check cheap enough to run every time.** The ten-game
  host check took 30 minutes: four replays per game (hashes and counts in
  separate runs, old and new), the unchanged baseline re-run every round, one
  replay at a time. `tools/host_regress.py` runs each build once per game,
  caches the results by the binary's contents, and runs six at a time: 39 s,
  0.2 s cached.
- **A watch on every store costs every store: give it its own handlers.** A
  run-ahead must stop at the SA-1's first write to the I-RAM bytes the 65816
  polls. A check in the engine's store path ran outside run-aheads too; the
  generator now emits a second copy of the 75 handlers that store to I-RAM,
  with the check, and the engine picks the table at entry (`RUN_WATCH`): 246
  handlers stay the bytes they were. Kirby Super Star still had 24 windows
  over a point heavier on the device, and it was not the stores: counted on gwemu,
  `sa1_watch_changed` ran 7,319 times in 2 frames, once per SA-1 register
  write during a run-ahead. A write to a register that cannot change what is
  watched (the arithmetic unit, the bit reader, the vectors) now skips it.
- **The SA-1's code moves with everything before it: pin it too (§5.5).**
  The overlay follows the core's bss, so any edit moved the engine and its C
  helpers in the I-cache: Kirby Super Star's SA-1 share moved 0.5-1 point
  with the same work. Placed where the build that measured best had them
  (`tools/sa1_overlay_pin.py` writes those lines from an ELF), it recovered
  1.1 points; a compact window -- the helpers 8 KB-aligned in a fixed order,
  the engine at +12 KB -- 2.8 points in the heaviest window, and fixes them
  for later builds. Inside an `OVERLAY` member `.` counts from the member's
  start: an 8 KB alignment must be computed on `__CORE_BSS_END__ + .`;
  `ABSOLUTE(.)` there shifted the later overlays' load addresses (ld warns
  ".gsugame_yoshi lma adjusted").

---

## 5. Cross-cutting lessons

### 5.1 Inlining spreads register pressure; contain a hot helper with `noinline`

`PpuDrawBackground_mode7` is **fully inlined** into `PpuDrawBackgrounds` (it has
no symbol of its own; the other BG modes are separate `.constprop` functions).
So **every** Mode 7 loop variant — the `dy==0` fast path, the mosaic loop, and
the rotated fallback — shares **one register file and one stack frame**.

Consequence: adding two live variables (`cur_cell`, `tilebase`) to *one* branch
forced GCC to re-solve allocation for the *whole* blob. Measured on the inlined
cell cache vs HEAD:

- **778 of ~940 instructions** in `PpuDrawBackgrounds` changed (≈83 %).
- stack frame grew **92 → 100 bytes** (two more spill slots).
- the hot loop got **more** per-pixel spills, not fewer — the opposite of intent.

Result: the inlined cell cache was **+128 %** (SMK 499 → 1137), worse than having
no cell cache at all.

**The fix:** split the fallback into its own `__attribute__((noinline))`
function (`PpuDrawMode7Fallback`). Verified in the binary:

- a new ~218-byte function symbol appears;
- its inner loops are **fully register-allocated with zero per-pixel spills**
  (the 10 args are read from the stack once at entry, then the loop runs entirely
  in registers);
- `PpuDrawBackgrounds` comes out **leaner than HEAD** (892 vs 938 instructions,
  129 vs 136 loads) because the fallback's state no longer perturbs it;
- the per-segment call overhead is paid once per window span per line — negligible
  next to the per-pixel saving.

Result: **−53 %** (SMK 499 → 233), FF6 unchanged, output bit-identical.

Takeaways:
- In a big inlined function, **local state is global pressure.** A change that
  looks local reshuffles the whole function's allocation.
- A win that comes from an accidental reshuffle (§4.2) is luck and can flip to a
  loss on any recompile. **Don't bank it; understand it or contain it.**
- When a hot helper carries its own loop-carried state, give it its own frame
  with `noinline` and *verify in the disassembly* that the inner loop has no
  per-pixel stack traffic. (Note: `types.h`'s `NOINLINE` macro is a no-op under
  gcc — spell out `__attribute__((noinline))`.)
- The frame size of the *caller* can grow for an innocent reason (outgoing
  argument space for a many-arg call) without meaning more spills — read the
  loop, not just `sub sp, #N`.

Related: this generalises the older "per-line state in a hot loop spills" and
"keep the hot body small; a cold-branch CALL costs the hot path" findings — the
resolution is the same: isolate the state-carrying work behind a `noinline` call.

### 5.2 Generic optimisations only — no per-game code

Every optimisation must be generic emulator behaviour: either plain code
efficiency, or eliding work that is *generically unobservable* (produces
bit-identical state/output for any game). **No per-game detection, no per-game
skips.** Per-game data is acceptable only as a guarded, opt-in table — never
branching logic keyed on a title. The three case studies above are all generic:
the pagemap path is "HiROM, no coprocessor"; the Mode 7 paths trigger on PPU
register state, not on which game is running.

**The one exception is the Super FX (§4.5).** Seven titles (plus regional
copies) use the GSU, and a generic speed-up of the interpreter's dispatch did
not pay (the one-switch dispatch lost on the device: `experiments/superfx-gsu-dispatch-switch.md`).
There, per-game compiled block code is allowed — gated on a hash of the exact
ROM bytes it was compiled from, with the generic interpreter as the exact
fallback.

### 5.3 Verify output with state hashes, not pixels

A pixel A/B is blind to cost and to latent divergence — machine state can diverge
~16 s *before* the screen shows it. Use the per-frame `REC/hash` over the full
machine state (WRAM/VRAM/CGRAM/OAM/ARAM/CPU/audio). A bit-exact replay across the
whole recording is the correctness bar; one frame is never enough.

### 5.4 Recordings are deterministic and portable

A recording stores the **SNES pad per frame**, so a host-made `.rec` replays
frame-identically on the device (pause-menu Replay), giving a deterministic A/B.
The `.rec` path is the ROM path with the extension replaced. A button remap
breaks old recordings (they store the post-mapping pad).

### 5.5 Code placement and alignment are measured variables — pin hot code in fixed, aligned windows

The M7's instruction cache is **2-way, 8 KB per way (16 KB total)**; a line's
cache set is decided by its **address modulo 8 KB**. So *where* a hot function
lands — not only what it compiles to — changes what it evicts and how often it
hits. On this core that was not a rounding error:

- With the hot code placed after everything else, **any** change anywhere in the
  core moved the 14 KB SPC700 engine and changed what it evicted from the
  I-cache. The *same* SPC700 code measured **~25,000 cycles/frame apart between
  two builds that differed only in `dsp.c`.** A measurement was untrustworthy —
  the delta could be the change or just code motion.

The fix lives in `snes_core.ld`: each hot group — the SPC700 engine, DSP, the
PPU line renderer, the 65816 bus, the 65816 C fallback — is pinned in a **fixed
window at a fixed offset, padded to a fixed size**, with each function
**32-byte (cache-line) aligned** (64 at a window start), and the Thumb-2 65816
engine pinned in ITCM via `__SNES_ITCM_ENGINE_AT`. Then a change that grows one
group moves nothing in the others: their cache sets stay put, and a measured
difference belongs to the change, not to placement. Because sets repeat every
8 KB, a window that must keep its sets is moved in whole 8 KB steps.

Set-aliasing between windows is real and sweepable:
- DSP is placed **3.5 KB into the 8 KB way** so it does not share sets with the
  start of the SPC700 engine. When it sat at +32 KB (same sets), it cost DSP
  +5K and the SPC700 engine +3.5K cycles/frame.
- The bus runs on **every** CPU memory access; when it shared the PPU window,
  **1.7 KB of renderer growth moved it and cost the CPU +67,000 cycles/frame.**

Takeaways:
- Treat code placement as a variable you control, not noise. A device delta from
  a change that never touched a function can be that function moving in the
  I-cache. Pad the windows so builds stay comparable.
- Align hot functions to the 32-byte cache line; keep the function whose size
  tracks the work (here `ppu_runLine`) *last* in its window so its growth moves
  nothing after it.
- This is the cross-function cousin of §5.1 (register allocation *within* a
  function) and of section-anchor effects (a static's offset from its anchor
  turning a 1-instruction indexed load into 2). All three say the same thing:
  on the M7 the compiler's and linker's incidental choices have first-order
  cost — measure, and remove the degree of freedom when you can.

### 5.6 Moving data out of the hot pool speeds up what *stays* — measure the phases you didn't change

The 65816's hot working set lives in the RAM_EMU pool. Cart save-RAM and the
coprocessor / idle-skip buffers (~48 KB) sat there too, even though the CPU
barely touches them (saves are rare). Moving them to the cached AHB heap did two
things:

1. **Direct:** the DMA that writes cart SRAM now targets cached SRAM (~-20% DMA
   on steady frames), and CPU reads of it hit cache.
2. **Second-order, and bigger here:** the 65816 CPU work that *stayed* in the
   pool got faster — **Zelda cpu -3.8% against busy -2.6%** — although that code
   never touches the moved buffers. Handing the pool back to the hot data (and
   taking the rarely-useful DMA traffic out of it) left more of the cache and bus
   for the accesses that matter.

The tell is that **the CPU figure moved more than the thing that was
"optimised."** When you remove data or memory traffic from a hot region, measure
the phases you did *not* change — the win is often there, not in the moved data.
(Same pattern on the arcade core: moving a per-pixel flash gather out of a hot
loop dropped a completely untouched blit ~-25%.) Corollary: scarce fast memory
(DTCM) spent on data that only *fits* the D-cache buys nothing — spend the move
on an access pattern the cache can't hold, or on evicting traffic from a hot
region, not on "it's hot."

### 5.7 Size the wasted-work ceiling first — the measurement can hand you a better change

Before touching the (very regression-prone) sub-screen renderer for §4.4, a
throwaway host probe measured how much of each built sub-screen line the blend
actually *reads*. The answer reframed the task: the sub-screen was ~100% unread
on the exact lines we were about to clip, so the right move was not "render less
of the sub" (thread an x-range through two shared tile renderers — more code,
more risk) but "don't build it at all on those lines" (a one-line gate). Cheaper,
safer, bigger win.

- **Measure the ceiling before you build.** "How much of this work is provably
  unobservable?" is a cheap host probe, and it both justifies the effort and
  often points at a simpler mechanism than the one you set out to write. The
  first `!lineHasSprites` gate was a guess that caught a third of the waste; the
  probe showed the real figure was ~all of it, and a different predicate.
- **Diff the codegen before the device, every time.** Build the same `.o` both
  ways with the Docker toolchain, disassemble, normalise out addresses and branch
  targets (they shift when the function changes size), and diff the mnemonic
  stream — confirm the hot loops are byte-identical and only the intended gate
  moved, *before* spending a device round-trip. This is what proved §4.4's scan
  didn't perturb the pixel loops, the same lens that caught §5.1's spill blow-up.
  See §6 for the one-liner.

### 5.8 A 2-byte `memcpy` is a library call on this device

The usual portable idiom for an unaligned load — `memcpy(&v, p, 2)` — folds into
one instruction on the host and on most targets. The device build is
`-mno-unaligned-access`, so GCC cannot fold it and emits `bl memcpy`. It hid in
the GSU's PLOT from the port on (§4.5): four calls per pixel, invisible in the C,
obvious in the disassembly (52 `bl memcpy` in the block code alone). When the
address is known to be aligned, say so (`__builtin_assume_aligned`, or a typed
access) and it becomes `ldrh`/`ldr`. **Grep the hot objects for `bl memcpy` /
`bl memset`** — a "free" helper call in a pixel loop is never free here.

### 5.9 A bigger translation unit changes how GCC inlines *unrelated* code

Including 63 KB of generated block code in `gsu_core.c` made GCC inline 19
helpers it had kept out of line before (`fx_lm`, `fx_stw`, `fx_cmprn`, …) into
every interpreter handler: +27 KB of overlay, and the measured interpreter's code
changed silently. GCC's inline-growth limits are *percentages of the unit*, so a
bigger unit raises the budget for everything in it. Pinning those 19 `noinline`
only moved the flip to other helpers (+15 KB). The fix was structural: generated
code in its own unit (`gsu_blocks.c`); the interpreter then compiled exactly as
before (an `nm -S` diff of the linked ELF: 3 symbols changed, all intended).
Diff **linked** symbol sizes, not `.o` sizes — `--gc-sections` drops what is
unreferenced.

---

## 6. Tooling quick reference

```sh
# Host regression, two builds over the ten-recording set (tools/host_regress.set):
# same state (REC/hash, SA-1 every frame), per-window instruction counts; ~40 s
tools/host_regress.py --old <baseline snes_host> --new snes_host -j 6

# Host bit-exact replay (correctness). .rec sits next to the ROM.
make host
SNES_REPLAY=1 SNES_FRAMES=<rec_len> SNES_FAST=1 ./snes_host "<rom>.sfc" \
  2>/dev/null | grep REC/hash

# GWEMU deterministic counts (opcode/bus census; cycles read 0)
make qemu PERF_CPU=1
host/qemu/run.sh <dir> <rom> [frames]       # auto-sets SNES_REPLAY=1

# Device bin, low-overhead frameskip report (the real metric)
docker run --rm -u $(id -u):$(id -g) -v "$PWD:/opt/workdir" -w /opt/workdir \
  sylverb/retro-go-sd-builder:v1.5 make -j$(nproc) PROFLITE=1
# -> sdpush snes.bin /cores/snes.bin ; replay on device ; sum the `late=` frames

# Disassembly diff of one object (prove a codegen claim, e.g. "the hot loop is
# unchanged"). Build the SAME .o with the Docker toolchain for both sides, then
# diff one function's mnemonic stream with addresses/branch targets normalised
# out (they shift whenever the function changes size, else every line "differs"):
norm() { arm-none-eabi-objdump -d --no-show-raw-insn "$1" |
  awk "/<$2>:/{p=1} p{print} p&&/^\$/{exit}" |
  sed -E 's/^ *[0-9a-f]+:\t//; s/;.*$//; s/0x[0-9a-f]+//g; s/<[^>]*>//g'; }
diff <(norm before/ppu.o ppu_runLine) <(norm after/ppu.o ppu_runLine)
# What's left after normalising is real: added/removed instructions, register
# reallocation, spills (new str/ldr [sp,...] inside the loop). Benign: operand-
# order swaps on commutative ops, literal-pool offset shifts.
```

PROFLITE (`PROFLITE=1`) is the cheap "how does it fare" build: per-frame skip +
audio-step tracking only, no per-line/opcode DWT probes. `PERF=1 PERF_CPU=1` adds
the opcode census (and disables opcode batching, so it can overflow ITCM — a
diagnostic build only). Its per-opcode cost is therefore that of a path normal
builds don't run: to split the 65816 figure of a normal build, use
`PERF=1 PERF_CPUB=1` (batch / events / idle / rest, `PROF/cpub`, summed per phase
by `tools/prof_log_compare.py`). Two PERF logs of an SA-1 replay, window by
window: `tools/prof_sa1_compare.py`; the PPU's split per drawn frame of a
`PERF_PPU=1` log: `tools/prof_ppu_windows.py`. A change to the Thumb-2 engines or bridges is
checked with `tools/qemu_vs_host.py` (gwemu against the host, per recording).
