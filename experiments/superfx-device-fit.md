# Super FX on the G&W: memory study

Goal: make the superfx-poc GSU core fit the G&W without costing normal SNES games
anything. Speed is a separate problem (see superfx-gsu-cycle-timing.md).

## Device memory today (perf, normal game)

- `RAM_EMU` = 724 KB (0x2404b000-0x24100000). The core image (code + data + bss)
  ends at 594 KB; the remaining ~146 KB is the game's pool
  (`snes: mem free pool=145176` in the device logs, after a normal game's own
  allocations).
- AHB SRAM: ~69-77 KB free at game start.
- ROM: cached into OSPI flash and read in place (`rom=0x90700000`): costs no RAM.

Every byte the Super FX code adds to the core IMAGE comes out of the pool of every
game. So the GSU's code and tables must not live in the image as RAM.

## What the PoC allocates (host build)

| item | size | needed? |
|---|---|---|
| expanded ROM copy (each 32 KB bank mirrored into a 64 KB bank) | 2 x ROM (Doom/Yoshi/Winter Gold 4 MB) | no: fetch with a LoROM mapping from the flash ROM |
| plot line-location LUTs (`sfxlutdata`, 4 x 256 KB) | 1 MB | no: one use site (fx_ops.h ~2056), compute the tile index |
| Game Pak RAM (`kGnwCartSramMax` raised to 256 KB) | 256 KB | yes, but per cart: 32 or 64 KB |
| GSU core static tables (gsu_core.o bss) | 30 KB | partly |
| GSU core code (gsu_core.o text, ARM) | 79 KB | yes, but not in RAM_EMU |
| `sfxnametab` (opcode name strings, rodata) | 9 KB | no (debug) |
| debug scaffolding, e.g. cpu.o `seen[]` (CPUTRACE) | 128 KB bss | no: strip |

gsu_core.o bss detail: FxTable/FxTableb/FxTablec/FxTabled 4 x 4 KB (dispatch per
ALT state, built from 8 x 1 KB const init tables, with the PLOT entry 0x4C patched
at runtime), sfx_rptr/sfx_wptr 2 x 3 KB, fxxand + fxbit01/23/45/67 5 x 1 KB,
SfxMemTable 1 KB, SfxCACHERAM 512 B.

## Game Pak RAM per cart (header $FFBD, dev=$33)

| game | ROM | Game Pak RAM |
|---|---|---|
| Star Fox (GSU-1, old header) | 1 MB | 32 KB |
| Yoshi's Island | 2 MB | 32 KB |
| Vortex | 512 KB | 32 KB |
| Dirt Trax FX | 512 KB | 64 KB |
| Doom | 2 MB | 64 KB |
| Stunt Race FX / Wild Trax | 1 MB | 64 KB |
| Winter Gold | 2 MB | 64 KB |

Real carts mirror their RAM across the 64 KB bank; the GSU's RAM accesses must be
masked to the cart size (the PoC's flat 256 KB hid that: ASan once caught Yoshi's
RPIX reading past 32 KB).

## Pruning steps

1. Strip the debug scaffolding (also needed for the device build to link at all:
   it pulls in libc calls the core cannot link, and grows the ITCM bus-helper slot
   and the 9 KB BUS window past their fixed sizes).
2. LUTs -> computed tile index at the single use site:
   128: (x>>3)*16 + (y>>3); 160: (x>>3)*20 + (y>>3); 192: (x>>3)*24 + (y>>3);
   OBJ: ((y&0x80)<<2)|((x&0x80)<<1)|((y&0x78)<<1)|((x&0x78)>>3). -1 MB.
3. Expanded ROM -> fetch from the original ROM with the LoROM mapping
   (bank*0x8000 + (addr & 0x7fff)); ~20 sites in fx_ops.h + 12 in c_fxtable.inc.
   -2 x ROM.
4. Game Pak RAM sized to the cart (32/64 KB), GSU RAM accesses masked to it;
   restore the normal SRAM cap for non-FX carts.
5. Move the GSU out of the core image: code in flash (or loaded on demand), tables
   allocated from the pool only when a Super FX cart is loaded; const tables in
   flash. Normal games: pool unchanged.

Target for a Super FX game: 32-64 KB Game Pak RAM + ~15-30 KB tables from the
~146 KB pool. Each step is checked against the current host build (Star Fox
14M-op trace vs ZSNES, Yoshi/Doom/Winter Gold recordings bit-exact).
