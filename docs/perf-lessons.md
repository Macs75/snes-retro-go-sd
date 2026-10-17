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
copies) use the GSU, and generic speed-ups of the interpreter were exhausted
(the one-switch dispatch lost on the device: `experiments/superfx-gsu-dispatch-switch.md`).
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
diagnostic build only).
