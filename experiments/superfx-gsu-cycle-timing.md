# Super FX: per-instruction GSU timing (parked)

Status: designed, NOT applied. Parked until the Super FX core fits on the G&W, so
the device cost of both timing models can be compared there.

## The bug it fixes

Winter Gold (Europe, PAL): the skier's 3D model breaks up during races (two heads,
scattered limb blocks). Reproduced with a host recording of a practice race
(`SNES_REPLAY`, replays bit-exact; glitch from ~presented frame 1140).

The GSU core is a verbatim ZSNES 2 port. Its scheduler (`gsu_zsnes/sched.inc`,
same model as snes9x `fxemu.cpp`) gives the GSU a FLAT budget of instructions per
scanline, whatever the instructions are:

    perLine = 5823405 / (fps * lines)        (370 at NTSC, 373 at PAL)
    CLSR bit 0 (21 MHz): perLine * 5 / 2     (925)
    * SuperFXClockMultiplier / 100

Winter Gold sets CLSR=1, so it gets 925 instructions per line.

A/B on the recording (skier over 10 consecutive race frames):

| run | skier |
|---|---|
| default (100) | broken in most frames |
| `romispal = 1` | no change (state byte-identical at frame 2280) |
| vblank catch-up (`SfxVblankCatchup`) off | still broken |
| GSU_CLOCK=200 | broken |
| GSU_CLOCK=50 | clean in nearly every frame |

So the flat model makes the GSU finish too early relative to the 65816. A snes9x
fork (shanytc, release 1.63.25) documents the same "long-standing garbled-skier bug
at race start" in Winter Gold / FX Skiing (upstream snes9x issue #533): with
cycle-accurate GSU timing the new pose is published one frame later from the CPU's
view, so the game no longer commits a stale arrangement over the new pose tiles.
They verified it frame by frame against a Mesen2 hardware-accurate trace.

Probably the same cause: Stunt Race FX (Europe, PAL) green slabs at the left/right
of the race view; Wild Trax (Japan, NTSC, same game) is clean. Untested.

## The fix

Replace the flat instruction count with a master-clock cycle budget per line and a
real cost per GSU instruction:

- base cost per opcode (table indexed by opcode/ALT state);
- instruction fetch: from the 512-byte cache (cached) vs ROM/RAM (uncached, wait
  states, cache-line fill when the cache is active);
- RAM/ROM data access (LDW/STW/LDB/STB/GETB/ROMB buffering, SBK);
- PLOT / RPIX (pixel cache flushes);
- multiplies: cost depends on CFGR MS0;
- 10.7 vs 21.4 MHz from CLSR.

Source of truth for the costs: Mesen2's GSU (hardware-accurate; GPL-3, reference
only) and the shanytc snes9x fork's cost tables. Keep `SuperFXClockMultiplier` as
a throughput knob (100 = hardware).

Verification:
1. GSU opcode trace still byte-identical to ZSNES (only timing changes).
2. Winter Gold recording: skier clean from frame ~1140.
3. Regression: Star Fox, Yoshi's Island, Doom, Dirt Trax FX, Vortex, Stunt Race FX
   (both releases) at their known-good scenes.

## Cost estimate (not measured)

- Normal SNES games: zero (GSU code only).
- Per GSU instruction: +3-6 Thumb-2 instructions (cost-table load/subtract,
  cache-vs-ROM compare), ~+10-15% over an estimated ~55 device cycles per GSU
  instruction.
- Per frame: the budget is a ceiling. With realistic costs the heavy-scene ceiling
  drops by about half (Winter Gold matched at 50%):
  - flat:      925 x 262 ~ 242K GSU instr/frame x ~55 cyc ~ 13M cycles (2.3x budget)
  - realistic: ~120K x ~62 cyc ~ 7.5M cycles (1.3x budget)
  Light scenes (GSU finishes early) pay ~+10-15% on a small amount of work.

The ~55 cycles per GSU instruction is an estimate, not a device measurement:
measure both models on the device once Super FX fits there.
