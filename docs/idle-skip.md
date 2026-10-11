# Idle-loop skip — how it works and how to add a per-game template

Most SNES games finish their frame early and then spin in a small loop waiting
for the next NMI. Emulated instruction-by-instruction that wait costs as much as
real work, so the core detects the loop and advances it in whole laps. This is
the single biggest CPU win in the core (around 60% of surveyed games skip about
half of every frame).

Code: `src/snes/idle_skip.c` / `.h`, wired from `main_snes.c`
(`idle_run_span` / `idle_skip_laps`) and `snes.c` (NMI sampling). Build flag
`SNES_IDLE_SKIP=1` (on by default). The header's top comment is the precise
spec; this doc is the working guide, and especially the part the code cannot
explain by itself: **how to add a per-game template when the generic path can't
handle a loop.**

---

## Two layers

### 1. Generic path (no table — the default)

At every NMI the interrupted PC is examined. If it sits in a loop that passes a
**purity check** (decoded from the ROM bytes: only whitelisted instructions that
write nothing, operands in WRAM or ROM, never an I/O register), the loop becomes
the *armed site*. The armed site follows the game through menus and levels; it
is keyed on the loop's address and decode environment, **never on a game name**.

Skipping is then **proven per span**: at the loop head the CPU registers and
every byte the loop reads are compared with the previous lap's. If they are
equal, the machine state has recurred — nothing in the loop writes, and no
interrupt or DMA can occur mid-span — so every remaining lap is provably
identical and is applied natively; the final lap runs for real. A counting loop
(`DEX/BNE`) never recurs, so it is simply executed, never skipped. `WAI` is the
degenerate case (charged 6 dots per step until an interrupt is pending).

This path is fully generic and needs no maintenance. It handles the large
majority of games on its own.

### 2. Per-game template table (the guarded fallback)

Some wait loops **write every lap**, so the recurrence proof correctly rejects
them. The common shape is an **accumulator**:

```
LDA #$0000 / TCD / LDA $dest / ADC $add / STA $dest / BRA self
```

Each lap does `dest += add + carry` in WRAM and then branches back — the state
never recurs, so the generic path cannot skip it, even though the loop does no
*useful* work (it is just burning time until the NMI).

For these, `idle_skip.c` has a small table of **accumulator templates**,
`s_accum_tpl[]`. A template is matched by bank + address **and the exact loop
bytes** (a guard), so it can only ever fire on the one ROM that contains that
code — a different game, revision, or hack will not have those bytes at that
address. When it fires, the span runner measures one lap's charge and applies
the rest of the span as native `dest += n*add` arithmetic.

> **Policy.** This core is otherwise generic-only (no per-game behaviour). The
> template table is the sanctioned exception *because it is a guarded table*:
> it changes nothing about how any other game runs, it is proven bit-exact, and
> a template can never misfire on a ROM it was not written for. Keep it that
> way — a template entry is a byte-pattern match plus a replay proof, never a
> title string or a heuristic.

---

## Finding loops: the library-wide survey

The systematic way to find every skippable (and not-yet-skippable) loop is the
**overnight survey**, already run once and checked into this folder:

- [`snes_idle_survey.md`](snes_idle_survey.md) — the report: 134 loadable games
  from the top lists, each classified **A–X** with its wait-loop address, the
  loop's disassembly, the NMI/CPU share, and the skipped % of frame. Read this
  first — the game you care about is probably already in it.
- [`snes_idle_survey.csv`](snes_idle_survey.csv) — one machine-readable row per
  game (CRC32, category, reason, armed site, loop code, hash check).
- [`snes_idle_skips_draft.txt`](snes_idle_skips_draft.txt) — the draft table
  entries for the category-D games (see below), keyed by CRC32 with guard bytes.
- [`snes_top_games_mapping.txt`](snes_top_games_mapping.txt) — title → ROM file.

**How the survey was produced, and how to re-run it** (do this when the game set
or the decoder changes): the tooling is in [`tools/idle-survey/`](../tools/idle-survey/)
(`run_one.sh` + `analyze.py` + `report.py`, plus `survey-instrumentation.md` for
the two host builds it needs — see that folder's `README.md`). For each ROM, on
the host, no input, from power-on, it records ~4,200 frames with the *unmodified*
build as the reference, then replays with the idle-skip build capturing three
probes — where the NMI interrupts the CPU each frame (`nmi.txt`), a cost-weighted
PC histogram (`pch.txt`), and the skip counters (`IDLE_STATS` on stderr) — and
diffs every `REC/hash` line against the reference. **Zero hash mismatches is the
gate.**
Scene caveat: title + attract only; real play may wait in a different loop, so a
game that looks like category E here can still have a skippable gameplay loop.

The categories, and what each needs:

| Cat | Meaning | Work | Count |
|---|---|---|---|
| **A** | generic skip already takes the wait loop | none | 60 |
| **B** | idle & exact but missed — a **generic** decoder fix, no table | decoder | 32 |
| **C** | lap writes/calls but returns to the same state — **traced** proof, no table | tracer | 18 |
| **D** | lap changes state every lap — needs **per-game arithmetic (the table)** | template | 10 |
| **E** | no dominant wait loop in this scene (check with a gameplay recording) | — | 14 |
| **X** | core can't load it (SA-1/SuperFX) | — | 6 |

Only **category D** needs the per-game template below; B and C are generic
improvements to the decoder/prover that stay table-free (`snes_idle_survey.md`
spells out each fix). The D backlog today is 8 accumulators (mostly Konami) and
2 `$4212` H/V polls — all already captured in `snes_idle_skips_draft.txt`.

### Confirming one game on the device

To check a single title (or one not in the survey), build `make PERF=1`, replay
a recording, and read the per-frame PROF lines:

- **`PROF/idle: … rejects=N`** high — the NMI keeps landing in a loop the
  generic decoder won't arm (a healthy game shows large `laps`/`skipped_dots`,
  `rejects` near zero).
- **`PROF/cpu: c_fallback=… top XX:n`** — the opcodes that most fall to the C
  interpreter; an accumulator shows its `ADC`/`STA` here.
- The game is CPU-bound with heavy frameskip despite a static scene.

Note the address the NMI keeps interrupting, then read the ROM bytes there to
recover the exact loop.

---

## Adding an accumulator template

An entry in `s_accum_tpl[]` (`src/snes/idle_skip.c`):

```c
typedef struct {
  uint8_t  bank;              /* bank of the loop head                        */
  uint16_t head;             /* address of the loop head (the BRA target)     */
  uint8_t  len;              /* bytes from head to the last byte of the BRA   */
  uint16_t acc_dest, acc_add;/* WRAM byte offsets written / read (DP forced 0)*/
  uint8_t  acc_w;            /* accumulator width in bytes (1 or 2)           */
  uint8_t  nguard;           /* number of guard bytes below                   */
  uint8_t  guard[16];        /* the EXACT loop code bytes, head..end          */
} AccumTemplate;
```

Worked example — Castlevania: Vampire's Kiss (E) / Dracula X, loop at `80:80C4`:

```
80C4: A9 00 00   LDA #$0000
80C7: 5B         TCD            ; DP = 0
80C8: A5 46      LDA $46
80CA: 65 42      ADC $42
80CC: 85 46      STA $46        ; $46 += $42 (+carry), 16-bit
80CE: 80 F4      BRA $80C4
```

```c
{ 0x80, 0x80c4, 12, 0x46, 0x42, 2, 12,
  { 0xa9, 0x00, 0x00, 0x5b, 0xa5, 0x46, 0x65, 0x42, 0x85, 0x46, 0x80, 0xf4 } },
```

How to read each field off the disassembly:
- **bank / head** — where the `BRA` lands (the first byte of the loop).
- **len** — bytes from `head` through the last byte of the `BRA` (12 here).
- **acc_dest** — the direct-page byte the loop `STA`s (`$46`). The loop's `TCD`
  after `LDA #0` forces DP = 0, so the DP address is the WRAM byte offset.
- **acc_add** — the direct-page byte it `ADC`s (`$42`).
- **acc_w** — 2 if the add is 16-bit (`REP #$20` / `M` clear before the loop),
  1 if 8-bit.
- **guard** — every byte from `head` to the end of the `BRA`, in order. This is
  what pins the template to exactly this ROM.

The matcher also re-checks the runtime environment before each skip (16-bit `A`,
not emulation mode, binary not decimal). If a new game's loop differs in shape
(a different addressing mode, an 8-bit add, an extra instruction), extend
`idle_accum_match` to parse it rather than forcing it into this template; keep
the guard-byte exactness.

---

## Verify every template bit-exactly

A template must never change emulated behaviour. Prove it:

1. Record the game (device pause menu, or `SNES_RECORD=1` on host), covering the
   scene that was stuck.
2. Replay the recording **with and without** the template, capturing the
   per-frame `REC/hash` (WRAM/VRAM/CGRAM/OAM/ARAM/CPU/audio):

   ```sh
   SNES_REPLAY=1 SNES_FRAMES=<len> SNES_FAST=1 ./snes_host "<rom>.sfc" \
     2>/dev/null | grep REC/hash
   ```
3. The two hash streams must be **identical**. A single differing hash means the
   skip is not equivalent — fix or drop it; do not ship it.
4. Confirm the win on the device (`PROFLITE=1`, sum the `late=` frames before and
   after). Counts can also be read deterministically on the QEMU Cortex-M7
   target (`make qemu PERF_CPU=1`, `host/qemu/run.sh`).

See `docs/perf-lessons.md` §3 for the full measurement discipline.
