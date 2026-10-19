/* SA-1: chip state, both CPUs' views of the cartridge, and the scheduler.
 *
 * The chip logic is ZSNES 2's (chips/sa1regs.c, chips/c_sa1regs.c,
 * chips/sa1emu.c, cpu/mem_ops.h, initc.c map_sa1), quirks included: the
 * variable-length reader's two barrels, the SA-1 interrupts taken without the
 * $220A enables, the 0x10 version byte, the one ROM patch. ZSNES's state names
 * are kept on the fields. Its per-game SA-1 idle checks (sa1speedhacks,
 * SA1IdleCharge) are not here: they are speed, not behaviour.
 *
 * What is ours: the SA-1 CPU is cpu.c (sa1_cpu.c), so its interrupts are level
 * lines our CPU samples between instructions, and the 65816's IRQ line is the
 * PPU timer OR'd with the SA-1's request. The SA-1 catches up (sa1_sync)
 * before every 65816 instruction and at every scanline; a wait loop it is
 * proven to be in puts it to sleep until the 65816 does something it could
 * see (sa1_interpret, sa1_event). Both CPUs read ROM and plain BW-RAM through
 * page maps (sa1_maps). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "sa1.h"
#include "snes.h"
#include "cpu.h"
#include "dma.h"
#include "snes_gnw_alloc.h"
#include "../snes_perf.h"

#ifdef SNES_PERF
/* PROF/sa1 (main_snes.c): SA-1 instructions run / skipped asleep, wakes, and
 * the cart reads of each CPU (every one is a call into this file). The SA-1's
 * cycles go to snes_perf_gsu, the coprocessor figure. */
uint32_t snes_perf_sa1_ops, snes_perf_sa1_skip, snes_perf_sa1_wakes;
uint32_t snes_perf_sa1_snes_rd, snes_perf_sa1_cpu_rd;
#define SA1_PERF_INC(c, n) ((c) += (n))
#else
#define SA1_PERF_INC(c, n) do { } while (0)
#endif

Cpu* sa1cpu_init(void* mem, int memType);
void sa1cpu_reset(Cpu* cpu);
int sa1cpu_runOpcode(Cpu* cpu);
void sa1cpu_saveload(Cpu* cpu, SaveLoadFunc* func, void* ctx);

/* ZSNES SA1_DMA_CC1 computes character addresses over this much BW-RAM, then
 * the index wraps at the cart's own size. */
#define SA1_CC1_BWRAM_SPAN 0x40000u

/* The CPU state a wait-loop proof compares: Cpu.a up to (not incl.) irqWanted */
#define SA1_CPU_SNAP (offsetof(Cpu, irqWanted) - offsetof(Cpu, a))

struct Sa1 {
  Snes* snes;
  Cpu* cpu;
  uint8_t* rom;
  uint32_t romSize;
  uint32_t romMask;           /* romSize - 1 for a 2^n image, else 0: fold */
  uint8_t* bwram;
  uint32_t bwramMask;
  uint8_t* rdmap[0x800];      /* the SA-1's read map: see sa1_maps */
  uint32_t romPatch;          /* map_sa1's patch: ROM offset + 1, 0 = none */

  /* --- saved from here (sa1_saveload) --- */
  uint8_t iram[0x800];        /* IRAM */
  uint8_t SA1Control;         /* $2200 */
  uint8_t SA1IRQEnable;       /* $2201 */
  uint8_t SA1IRQData;         /* $2209 (ZSNES SA1IRQData byte 1) */
  uint8_t SA1IRQEn;           /* $220A, stored only, as in ZSNES */
  uint16_t SA1ResetV, SA1NMIV, SA1IRQV;   /* $2203-$2208 */
  uint16_t SNSNMIV, SNSIRQV;              /* $220C-$220F */
  uint8_t SA1Message[2];      /* [0] to the SA-1 ($2200), [1] to the 65816 ($2209) */
  uint8_t SA1IRQExec[3];      /* [0] bit0 SA-1 -> 65816 IRQ, bit1 CC DMA IRQ;
                                 [1] 65816 -> SA-1 IRQ; [2] 65816 -> SA-1 NMI */
  uint8_t SA1BankVal[4];      /* $2220-$2223 */
  uint8_t SNSBMAP;            /* $2224 */
  uint8_t BWUsed2;            /* $2225 */
  uint8_t SA1Overflow[2];     /* [0] $230B overflow, [1] $223F bitmap format */
  uint8_t SA1ARC;             /* $2250 */
  uint16_t SA1AR1, SA1AR2;    /* $2251-$2254 */
  uint32_t SA1ARR1;           /* $2306-$2309 */
  uint8_t SA1ARR2;            /* $230A */
  uint8_t SA1DMAInfo;         /* $2230 */
  uint8_t SA1DMAChar;         /* $2231 */
  uint32_t SA1DMASource;      /* $2232-$2234 */
  uint32_t SA1DMADest;        /* $2235-$2237 */
  uint16_t SA1DMACount;       /* $2238-$2239 */
  uint8_t SA1_in_cc1_dma;
  uint8_t SA1_CC2_line;
  uint8_t SA1_BRF[16];        /* $2240-$224F */
  uint32_t SA1_DMA_ADDR;
  uint32_t VarLenAddr, VarLenAddrB;       /* $2259-$225B */
  uint8_t VarLenBarrel[4];    /* [0] [1] the two bit positions, [2] $2258, [3] its width */
  uint8_t SA1TimerSet;        /* $2210 */
  uint8_t SA1TimerCount[4];   /* $2212-$2215 */
  uint16_t lastV, lastH;      /* where sa1_sync last caught up to */
  uint8_t halfClock;          /* a master clock not yet worth an SA-1 clock */
  int32_t clockDebt;          /* SA-1 clocks owed by sa1_run */

  /* --- the wait-loop skip (sa1_interpret), not saved: it re-proves --- */
  uint8_t idle;               /* asleep at a proven wait loop's head */
  uint8_t lapDirty;           /* this lap wrote, or read a clock / a side effect */
  uint8_t candK;              /* the loop head being proven */
  uint16_t candPc;
  uint8_t snap[SA1_CPU_SNAP]; /* the CPU at the head, from Cpu.a to Cpu.e */
  uint32_t extEvents;         /* things the loop could see: 65816 writes, ... */
  uint32_t lapEvents;         /* extEvents when this lap began */
  int32_t lapCycles;          /* clocks and instructions since the head */
  uint32_t lapInstrs;
  int32_t idleLap;            /* the proven lap */
  uint32_t idleInstrs;
};

#define SA1_SAVE_FIRST iram

static void sa1_wake(Sa1* s);
static void sa1_maps(Sa1* s);
static void sa1_maps_window(Sa1* s);
static void sa1_maps_bwlinear(Sa1* s);

/* Something a sleeping wait loop could see is about to happen: first bring the
 * SA-1 up to now as if it had kept looping, then let it happen. "Now" is the
 * dot the 65816's current instruction started on: the device's engine batch
 * runs the 65816 without a sync per instruction while the SA-1 sleeps
 * (run_dots), storing hPos before each one, so the SA-1 is first synced to it.
 * Awake, the SA-1 needs the per-instruction sync again: hclip makes the batch
 * hand back after this instruction (it is how a moved H-IRQ dot ends a span). */
static inline void sa1_event(Sa1* s) {
  if (s->idle) {
    sa1_sync(s, s->snes->vPos, s->snes->hPos);
    sa1_wake(s);
#ifdef SNES_THUMB2_CPU
    s->snes->dma->hclip = 1;
#endif
  }
  s->extEvents++;
}
#define SA1_SAVE_END   (offsetof(Sa1, clockDebt) + sizeof(int32_t))

/* --- the SA-1 CPU's interrupt lines ---------------------------------------- */

/* ZSNES SA1SwapEnter takes SA1DoIRQ's bits without looking at $220A; the flag
 * stays until $220B clears it. */
static void sa1_update_cpu_irq(Sa1* s) {
  s->cpu->irqWanted = s->SA1IRQExec[1] != 0;
}

/* The 65816's line: the PPU timer flag, or the SA-1's request ($2201 enables;
 * ZSNES's main loop checks SA1IRQEnable 0x80 / 0x20 the same way). */
bool sa1_snes_irq(const Sa1* s) {
  return ((s->SA1IRQEnable & 0x80) && (s->SA1IRQExec[0] & 1)) ||
         ((s->SA1IRQEnable & 0x20) && (s->SA1IRQExec[0] & 2));
}

static void sa1_update_snes_irq(Sa1* s) {
  s->snes->cpu->irqWanted = s->snes->inIrq || sa1_snes_irq(s);
}

/* --- ROM: the MMC ($2220-$2223) -------------------------------------------- */

/* A 3 MB image mirrors its last part like cart.c's cart_fold (bsnes): the
 * same bytes the host's power-of-2 expansion copies there. */
static uint32_t sa1_rom_fold(uint32_t addr, uint32_t size) {
  uint32_t base = 0, mask = 1u << 31;
  while (addr >= size) {
    while (!(addr & mask)) mask >>= 1;
    addr -= mask;
    if (size > mask) { size -= mask; base += mask; }
  }
  return base + addr;
}

/* ZSNES bankswitch(): $00-$3F/$80-$BF:$8000-$FFFF take their 1 MB block from
 * the register when its bit 7 is set, else the fixed one (C=0, D=1, E=2, F=3);
 * $C0-$FF always take it from the register. Banks $40-$7F show the ROM
 * linearly (map_sa1). */
static inline uint32_t sa1_rom_offset(const Sa1* s, uint8_t bank, uint16_t adr) {
  uint32_t off;
  if (bank >= 0xc0) {
    off = ((uint32_t)(s->SA1BankVal[(bank >> 4) & 3] & 7) << 20) | ((uint32_t)(bank & 0x0f) << 16) | adr;
  } else if (bank >= 0x40 && bank < 0x80) {
    off = ((uint32_t)(bank & 0x3f) << 16) | adr;
  } else {
    unsigned idx = ((bank >> 6) & 2) | ((bank >> 5) & 1);
    uint8_t v = s->SA1BankVal[idx];
    uint32_t blk = (v & 0x80) ? (uint32_t)(v & 7) << 20 : (uint32_t)idx << 20;
    off = blk | ((uint32_t)(bank & 0x1f) << 15) | (adr & 0x7fff);
  }
  return s->romMask ? off & s->romMask : sa1_rom_fold(off, s->romSize);
}

static inline uint8_t sa1_rom_read(const Sa1* s, uint8_t bank, uint16_t adr) {
  uint32_t off = sa1_rom_offset(s, bank, adr);
  if (s->romPatch && off + 1 == s->romPatch) return 0;
  return s->rom[off];
}

/* --- BW-RAM ---------------------------------------------------------------- */

/* ZSNES SA1_DMA_CC1: a character-conversion type 1 read. The 65816 DMAs from
 * BW-RAM while the SA-1 hands it the converted character from I-RAM. */
static uint8_t sa1_cc1_read(Sa1* s, uint32_t addr) {
  s->lapDirty = 1;            /* writes I-RAM: not a read a wait loop can repeat */
  const unsigned DMACB = s->SA1DMAChar & 3;
  const unsigned DMASIZE = (s->SA1DMAChar >> 2) & 7;
  const uint32_t DSA = s->SA1DMASource;
  const uint32_t DDA = s->SA1DMADest;
  unsigned charmask = (1u << (6 - DMACB)) - 1;
  s->SA1_DMA_ADDR = (s->SA1_DMA_ADDR & ~0xffffu) | (addr & 0xffffu);

  if ((s->SA1_DMA_ADDR & charmask) == 0) {
    /* buffer next character to I-RAM */
    unsigned bpp = 2u << (2 - DMACB);
    unsigned bpl = (8u << DMASIZE) >> DMACB;
    unsigned bwmask = SA1_CC1_BWRAM_SPAN - 1;
    unsigned tile = ((s->SA1_DMA_ADDR - DSA) & bwmask) >> (6 - DMACB);
    unsigned ty = (tile >> DMASIZE);
    unsigned tx = tile & ((1u << DMASIZE) - 1);
    unsigned bwaddr = DSA + ty * 8 * bpl + tx * bpp;

    for (unsigned y = 0; y < 8; y++) {
      uint64_t data = 0;
      for (unsigned byte = 0; byte < bpp; byte++)
        data |= (uint64_t)s->bwram[((bwaddr + byte) & bwmask) & s->bwramMask] << (byte << 3);
      bwaddr += bpl;

      uint8_t out[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
      for (unsigned x = 0; x < 8; x++) {
        out[0] |= (data & 1) << (7 - x); data >>= 1;
        out[1] |= (data & 1) << (7 - x); data >>= 1;
        if (DMACB == 2) continue;
        out[2] |= (data & 1) << (7 - x); data >>= 1;
        out[3] |= (data & 1) << (7 - x); data >>= 1;
        if (DMACB == 1) continue;
        out[4] |= (data & 1) << (7 - x); data >>= 1;
        out[5] |= (data & 1) << (7 - x); data >>= 1;
        out[6] |= (data & 1) << (7 - x); data >>= 1;
        out[7] |= (data & 1) << (7 - x); data >>= 1;
      }
      for (unsigned byte = 0; byte < bpp; byte++) {
        unsigned p = DDA + (y << 1) + ((byte & 6) << 3) + (byte & 1);
        s->iram[p & 0x07ff] = out[byte];
      }
    }
  }
  return s->iram[(DDA + (s->SA1_DMA_ADDR & charmask)) & 0x07ff];
}

/* Banks $40-$4F: BW-RAM in 64 KB slices, read through the converter during a
 * type 1 character conversion (cpu/mem_ops.h SA1RAMaccessbank*). */
static inline uint8_t sa1_bwram_linear_read(Sa1* s, uint8_t bank, uint16_t adr) {
  if (s->SA1_in_cc1_dma) return sa1_cc1_read(s, adr);
  return s->bwram[(((uint32_t)(bank & 0x0f) << 16) | adr) & s->bwramMask];
}

static inline void sa1_bwram_linear_write(Sa1* s, uint8_t bank, uint16_t adr, uint8_t val) {
  s->bwram[(((uint32_t)(bank & 0x0f) << 16) | adr) & s->bwramMask] = val;
}

/* Bit-map views: a pixel per address, 4 bits, or 2 when $223F bit 7 is set. */
static inline bool sa1_bm_2bit(const Sa1* s) { return (s->SA1Overflow[1] & 0x80) != 0; }

/* Banks $60-$6F (SA-1 only), ZSNES SA1RAMaccessbank*b. */
static uint32_t sa1_bm_index(const Sa1* s, uint8_t bank, uint16_t adr, unsigned* sh) {
  if (sa1_bm_2bit(s)) {
    *sh = (adr & 3u) << 1;
    return (((uint32_t)(bank & 0x0f) << 14) + (adr >> 2)) & s->bwramMask;
  }
  *sh = (adr & 1u) << 2;
  return (((uint32_t)(bank & 0x07) << 15) + (adr >> 1)) & s->bwramMask;
}

static uint8_t sa1_bm_read(Sa1* s, uint8_t bank, uint16_t adr) {
  unsigned sh;
  uint32_t i = sa1_bm_index(s, bank, adr, &sh);
  return (s->bwram[i] >> sh) & (sa1_bm_2bit(s) ? 3 : 0x0f);
}

static void sa1_bm_write(Sa1* s, uint8_t bank, uint16_t adr, uint8_t val) {
  unsigned sh;
  uint32_t i = sa1_bm_index(s, bank, adr, &sh);
  unsigned mask = sa1_bm_2bit(s) ? 3u : 0x0fu;
  s->bwram[i] = (uint8_t)((s->bwram[i] & ~(mask << sh)) | ((val & mask) << sh));
}

/* $6000-$7FFF for the SA-1: the $2225 block, or with its bit 7 the bit map
 * (ZSNES c_sa12225w + mem_bw_*). */
static uint32_t sa1_bw_window_index(const Sa1* s, uint16_t adr, unsigned* sh, unsigned* mask) {
  uint32_t off = adr - 0x6000u;
  uint8_t al = s->BWUsed2;
  if (!(al & 0x80)) {
    *sh = 0;
    *mask = 0xff;
    return (((uint32_t)(al & 0x1f) << 13) + off) & s->bwramMask;
  }
  if (sa1_bm_2bit(s)) {
    *sh = (off & 3u) << 1;
    *mask = 3;
    return (((uint32_t)(al & 0x7f) << 11) + (off >> 2)) & s->bwramMask;
  }
  *sh = (off & 1u) << 2;
  *mask = 0x0f;
  return (((uint32_t)(al & 0x3f) << 12) + (off >> 1)) & s->bwramMask;
}

static uint8_t sa1_bw_window_read(Sa1* s, uint16_t adr) {
  unsigned sh, mask;
  uint32_t i = sa1_bw_window_index(s, adr, &sh, &mask);
  return (s->bwram[i] >> sh) & mask;
}

static void sa1_bw_window_write(Sa1* s, uint16_t adr, uint8_t val) {
  unsigned sh, mask;
  uint32_t i = sa1_bw_window_index(s, adr, &sh, &mask);
  s->bwram[i] = (uint8_t)((s->bwram[i] & ~(mask << sh)) | ((val & mask) << sh));
}

/* --- arithmetic, DMA, character conversion (ZSNES c_sa1regs.c) ------------- */

static void sa1_arith(Sa1* s) {
  if (s->SA1ARC & 0x02) { /* cumulative sum */
    int32_t const product = (int16_t)s->SA1AR1 * (int16_t)s->SA1AR2;
    int64_t const sum = ((int64_t)((uint64_t)s->SA1ARR2 << 32 | (uint64_t)s->SA1ARR1) << 24 >> 24) + product;
    int64_t const sign = sum >> 39;
    s->SA1ARR1 = (uint32_t)sum;
    s->SA1ARR2 = (uint8_t)(sum >> 32 & 0xff);
    s->SA1Overflow[0] = sign != 0 && sign != -1 ? 0x80 : 0x00;
  } else if (s->SA1ARC & 0x01) { /* division */
    int16_t const dividend = (int16_t)s->SA1AR1;
    uint16_t const divisor = s->SA1AR2;
    if (divisor != 0) {
      int16_t const quotient = dividend / divisor;
      uint16_t const reminder = dividend % divisor;
      s->SA1ARR1 = (uint32_t)reminder << 16 | (uint16_t)quotient;
    } else {
      s->SA1ARR1 = 0;
    }
  } else { /* multiplication */
    s->SA1ARR1 = (uint32_t)((int32_t)(int16_t)s->SA1AR1 * (int16_t)s->SA1AR2);
  }
}

static uint8_t sa1_dma_source_byte(Sa1* s, uint32_t a) {
  if (s->SA1DMAInfo & 0x01) return s->bwram[a & s->bwramMask];
  if (s->SA1DMAInfo & 0x02) return s->iram[a & 0x7ff];
  return sa1_rom_read(s, (uint8_t)(a >> 16), (uint16_t)a);
}

/* executesa1dma: a memcpy of SA1DMACount bytes, done on the spot */
static void sa1_dma(Sa1* s, bool toBwram) {
  uint32_t src = s->SA1DMASource, dst = s->SA1DMADest;
  for (uint32_t i = 0; i < s->SA1DMACount; i++) {
    uint8_t v = sa1_dma_source_byte(s, (s->SA1DMAInfo & 3) ? src + i
                                        : (src & 0xff0000u) | ((src + i) & 0xffffu));
    if (toBwram) s->bwram[(dst + i) & s->bwramMask] = v;
    else s->iram[(dst + i) & 0x7ff] = v;
  }
}

/* SA1_DMA_CC2: one line of the bitmap register file into I-RAM */
static void sa1_cc2(Sa1* s) {
  const unsigned DMACB = s->SA1DMAChar & 3;
  const uint8_t* brf = &s->SA1_BRF[(s->SA1_CC2_line & 1) << 3];
  unsigned bpp = 2u << (2 - DMACB);
  unsigned addr = s->SA1DMADest & 0x07ff;
  addr &= ~((1u << (7 - DMACB)) - 1);
  addr += (s->SA1_CC2_line & 8) * bpp;
  addr += (s->SA1_CC2_line & 7) * 2;

  for (unsigned byte = 0; byte < bpp; byte++) {
    uint8_t output = 0;
    for (unsigned bit = 0; bit < 8; bit++)
      output |= ((brf[bit] >> byte) & 1) << (7 - bit);
    s->iram[(addr + ((byte & 6) << 3) + (byte & 1)) & 0x7ff] = output;
  }
  s->SA1_CC2_line = (s->SA1_CC2_line + 1) & 15;
}

/* --- the variable-length reader ($2258-$225B, $230C/$230D), ZSNES's way ---- */

static uint32_t sa1_varlen_peek(Sa1* s, uint32_t a) {
  uint8_t bank = (uint8_t)(a >> 16);
  uint16_t adr = (uint16_t)a;
  return (uint32_t)sa1_rom_read(s, bank, adr) |
         (uint32_t)sa1_rom_read(s, bank, (uint16_t)(adr + 1)) << 8 |
         (uint32_t)sa1_rom_read(s, bank, (uint16_t)(adr + 2)) << 16 |
         (uint32_t)sa1_rom_read(s, bank, (uint16_t)(adr + 3)) << 24;
}

static uint8_t sa1_read_230c(Sa1* s) {
  uint32_t val = sa1_varlen_peek(s, s->VarLenAddr) >> s->VarLenBarrel[1];
  if (s->VarLenBarrel[2] & 0x80) {
    s->VarLenBarrel[0] += s->VarLenBarrel[3];
    if (s->VarLenBarrel[0] > 16) {
      s->VarLenBarrel[0] -= 16;
      s->VarLenAddr = (s->VarLenAddr + 2) & 0xffffff;
    }
  }
  return (uint8_t)val;
}

static uint8_t sa1_read_230d(Sa1* s) {
  uint32_t val = sa1_varlen_peek(s, s->VarLenAddrB) >> s->VarLenBarrel[1];
  if (s->VarLenBarrel[2] & 0x80) {
    s->VarLenBarrel[1] += s->VarLenBarrel[3];
    if (s->VarLenBarrel[1] > 16) {
      s->VarLenBarrel[1] -= 16;
      s->VarLenAddrB = (s->VarLenAddrB + 2) & 0xffffff;
    }
  }
  return (uint8_t)(val >> 8);
}

/* --- registers ------------------------------------------------------------- */

static uint8_t sa1_reg_read(Sa1* s, uint16_t adr) {
  /* the H/V counters change with time, the bit-stream reads advance it: a lap
   * that reads them proves nothing */
  if ((adr >= 0x2302 && adr <= 0x2305) || adr == 0x230c || adr == 0x230d) s->lapDirty = 1;
  switch (adr) {
    case 0x2300: {
      uint8_t al = s->SA1Message[1];
      if (s->SA1IRQExec[0] & 1) al |= 0x80;
      if (s->SA1IRQExec[0] & 2) al |= 0x20;
      return al | (s->SA1IRQData & 0x50);
    }
    case 0x2301: {
      uint8_t al = s->SA1Message[0];
      if (s->SA1IRQExec[1] & 1) al |= 0x80;
      return al;
    }
    /* H/V counters: ZSNES derives H from its slot count; ours is the PPU's */
    case 0x2302:
      if (s->SA1TimerSet & 0x80) return s->SA1TimerCount[0];
      return (uint8_t)(s->snes->hPos >> 2);
    case 0x2303:
      if (s->SA1TimerSet & 0x80) return s->SA1TimerCount[1] & 1;
      return (uint8_t)(s->snes->hPos >> 10);
    case 0x2304:
      if (s->SA1TimerSet & 0x80)
        return (uint8_t)((s->SA1TimerCount[1] | s->SA1TimerCount[2] << 8) >> 1);
      return (uint8_t)s->snes->vPos;
    case 0x2305:
      if (s->SA1TimerSet & 0x80)
        return (uint8_t)(((s->SA1TimerCount[2] | s->SA1TimerCount[3] << 8) >> 1) & 1);
      return (uint8_t)(s->snes->vPos >> 8);
    case 0x2306: return (uint8_t)s->SA1ARR1;
    case 0x2307: return (uint8_t)(s->SA1ARR1 >> 8);
    case 0x2308: return (uint8_t)(s->SA1ARR1 >> 16);
    case 0x2309: return (uint8_t)(s->SA1ARR1 >> 24);
    case 0x230a: return s->SA1ARR2;
    case 0x230b: return s->SA1Overflow[0];
    case 0x230c: return sa1_read_230c(s);
    case 0x230d: return sa1_read_230d(s);
    case 0x230e: return 0x10;
  }
  return s->snes->openBus;
}

static void sa1_reg_write(Sa1* s, uint16_t adr, uint8_t al) {
  switch (adr) {
    case 0x2200: { /* SA-1 CPU control */
      uint8_t oldctrl = s->SA1Control;
      s->SA1Message[0] = al & 0x0f;
      if (al & 0x80) s->SA1IRQExec[1] = 1;
      if (al & 0x10) { s->SA1IRQExec[2] = 1; s->cpu->nmiWanted = true; }
      s->SA1Control = al;
      if ((oldctrl & 0x20) && !(al & 0x20)) /* leaving reset: start at $2203 */
        sa1cpu_reset(s->cpu);
      sa1_update_cpu_irq(s);
      return;
    }
    case 0x2201: s->SA1IRQEnable = al; sa1_update_snes_irq(s); return;
    case 0x2202: /* IRQ clear */
      if (al & 0x80) s->SA1IRQExec[0] &= 0xfe;
      if (al & 0x20) s->SA1IRQExec[0] &= 0xfd;
      sa1_update_snes_irq(s);
      return;
    case 0x2203: s->SA1ResetV = (s->SA1ResetV & 0xff00) | al; return;
    case 0x2204: s->SA1ResetV = (s->SA1ResetV & 0x00ff) | al << 8; return;
    case 0x2205: s->SA1NMIV = (s->SA1NMIV & 0xff00) | al; return;
    case 0x2206: s->SA1NMIV = (s->SA1NMIV & 0x00ff) | al << 8; return;
    case 0x2207: s->SA1IRQV = (s->SA1IRQV & 0xff00) | al; return;
    case 0x2208: s->SA1IRQV = (s->SA1IRQV & 0x00ff) | al << 8; return;
    case 0x2209: /* the 65816's IRQ from the SA-1, vector selects */
      s->SA1IRQData = al;
      if (al & 0x80) s->SA1IRQExec[0] |= 1;
      s->SA1Message[1] = al & 0x0f;
      sa1_update_snes_irq(s);
      return;
    case 0x220a: s->SA1IRQEn = al; return;
    case 0x220b: /* SA-1 IRQ clear */
      if (al & 0x80) s->SA1IRQExec[1] = 0;
      if (al & 0x10) s->SA1IRQExec[2] = 0;
      sa1_update_cpu_irq(s);
      return;
    case 0x220c: s->SNSNMIV = (s->SNSNMIV & 0xff00) | al; return;
    case 0x220d: s->SNSNMIV = (s->SNSNMIV & 0x00ff) | al << 8; return;
    case 0x220e: s->SNSIRQV = (s->SNSIRQV & 0xff00) | al; return;
    case 0x220f: s->SNSIRQV = (s->SNSIRQV & 0x00ff) | al << 8; return;
    case 0x2210: s->SA1TimerSet = al; return;
    case 0x2211: return;   /* timer clear: ZSNES's counter is never read back */
    case 0x2212: case 0x2213: case 0x2214: case 0x2215:
      s->SA1TimerCount[adr - 0x2212] = al;
      return;
    case 0x2220: case 0x2221: case 0x2222: case 0x2223:
      if (s->SA1BankVal[adr - 0x2220] == al) return;
      s->SA1BankVal[adr - 0x2220] = al;
      sa1_maps(s);
      return;
    case 0x2224: if (s->SNSBMAP != al) { s->SNSBMAP = al; sa1_maps_window(s); } return;
    case 0x2225: if (s->BWUsed2 != al) { s->BWUsed2 = al; sa1_maps_window(s); } return;
    case 0x2230:
      if (!(al & 0x80)) s->SA1_CC2_line = 0;
      s->SA1DMAInfo = al;
      return;
    case 0x2231:
      s->SA1DMAChar = al;
      if ((al & 0x80) && s->SA1_in_cc1_dma) { s->SA1_in_cc1_dma = 0; sa1_maps_bwlinear(s); }
      return;
    case 0x2232: s->SA1DMASource = (s->SA1DMASource & 0xffff00) | al; return;
    case 0x2233: s->SA1DMASource = (s->SA1DMASource & 0xff00ff) | al << 8; return;
    case 0x2234: s->SA1DMASource = (s->SA1DMASource & 0x00ffff) | (uint32_t)al << 16; return;
    case 0x2235: s->SA1DMADest = (s->SA1DMADest & 0xffff00) | al; return;
    case 0x2236: /* dest mid byte: starts the I-RAM DMA, or character conversion 1 */
      s->SA1DMADest = (s->SA1DMADest & 0xff00ff) | al << 8;
      if (s->SA1DMAInfo & 0x10) {
        /* ZSNES sa1chconv: the converter itself is off upstream; this raises
         * the 65816's IRQ and puts BW-RAM reads through SA1_DMA_CC1 */
        s->SA1IRQExec[0] |= 2;
        if (!s->SA1_in_cc1_dma) { s->SA1_in_cc1_dma = 1; sa1_maps_bwlinear(s); }
        sa1_update_snes_irq(s);
        return;
      }
      if (!(s->SA1DMAInfo & 4)) sa1_dma(s, false);
      return;
    case 0x2237: /* dest high byte: starts the BW-RAM DMA */
      s->SA1DMADest = (s->SA1DMADest & 0x00ffff) | (uint32_t)al << 16;
      if (!(s->SA1DMAInfo & 0x10) && (s->SA1DMAInfo & 4)) sa1_dma(s, true);
      return;
    case 0x2238: s->SA1DMACount = (s->SA1DMACount & 0xff00) | al; return;
    case 0x2239: s->SA1DMACount = (s->SA1DMACount & 0x00ff) | al << 8; return;
    case 0x223f: s->SA1Overflow[1] = al; return;
    case 0x2240: case 0x2241: case 0x2242: case 0x2243:
    case 0x2244: case 0x2245: case 0x2246: case 0x2247:
    case 0x2248: case 0x2249: case 0x224a: case 0x224b:
    case 0x224c: case 0x224d: case 0x224e: case 0x224f:
      s->SA1_BRF[adr - 0x2240] = al;
      /* $2247 and $224F end a register-file line: character conversion 2 */
      if ((adr == 0x2247 || adr == 0x224f) &&
          (s->SA1DMAInfo & 0xa0) && !(s->SA1DMAInfo & 0x10))
        sa1_cc2(s);
      return;
    case 0x2250:
      s->SA1ARC = al;
      if (al & 2) { s->SA1ARR1 = 0; s->SA1ARR2 = 0; }   /* cumulative-sum reset */
      return;
    case 0x2251: s->SA1AR1 = (s->SA1AR1 & 0xff00) | al; return;
    case 0x2252: s->SA1AR1 = (s->SA1AR1 & 0x00ff) | al << 8; return;
    case 0x2253: s->SA1AR2 = (s->SA1AR2 & 0xff00) | al; return;
    case 0x2254: s->SA1AR2 = (s->SA1AR2 & 0x00ff) | al << 8; sa1_arith(s); return;
    case 0x2258: {
      uint8_t bl = al & 0x0f;
      if (bl == 0) bl = 16;
      s->VarLenBarrel[2] = al;
      s->VarLenBarrel[3] = bl;
      if (al & 0x80) { s->VarLenBarrel[0] = bl; s->VarLenBarrel[1] = bl; }
      return;
    }
    case 0x2259: case 0x225a: case 0x225b: {
      unsigned sh = (adr - 0x2259) * 8;
      s->VarLenAddr = (s->VarLenAddr & ~(0xffu << sh)) | (uint32_t)al << sh;
      s->VarLenAddrB = (s->VarLenAddrB & ~(0xffu << sh)) | (uint32_t)al << sh;
      s->VarLenBarrel[0] = 0;
      s->VarLenBarrel[1] = 0;
      return;
    }
  }
}

/* --- the 65816's view (cart_read / cart_write) ----------------------------- */

uint8_t sa1_snes_read(Sa1* s, uint8_t bank, uint16_t adr) {
  uint8_t b = bank & 0x7f;
  SA1_PERF_INC(snes_perf_sa1_snes_rd, 1);
  /* the reads that change SA-1 state: the bit-stream reader, the converter */
  if (adr == 0x230c || adr == 0x230d || (s->SA1_in_cc1_dma && b >= 0x40 && b < 0x50))
    sa1_event(s);
  if (bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if (adr >= 0x8000) {
      /* $2209 bits 4 and 6: the SA-1 supplies the 65816's NMI / IRQ vector */
      if (bank == 0x00 && adr >= 0xffea && adr <= 0xffef) {
        if ((adr == 0xffea || adr == 0xffeb) && (s->SA1IRQData & 0x10))
          return (uint8_t)(s->SNSNMIV >> ((adr & 1) * 8));
        if ((adr == 0xffee || adr == 0xffef) && (s->SA1IRQData & 0x40))
          return (uint8_t)(s->SNSIRQV >> ((adr & 1) * 8));
      }
      return sa1_rom_read(s, bank, adr);
    }
    if (adr >= 0x6000)
      return s->bwram[(((uint32_t)(s->SNSBMAP & 0x1f) << 13) | (adr & 0x1fff)) & s->bwramMask];
    if (adr >= 0x3000 && adr < 0x3800) return s->iram[adr - 0x3000];
    if (adr >= 0x2200 && adr < 0x2400) return sa1_reg_read(s, adr);
    return s->snes->openBus;
  }
  if (bank >= 0xc0) return sa1_rom_read(s, bank, adr);
  if (b >= 0x40 && b < 0x50) return sa1_bwram_linear_read(s, b, adr);
  return s->snes->openBus;
}

void sa1_snes_write(Sa1* s, uint8_t bank, uint16_t adr, uint8_t val) {
  sa1_event(s);
  if (bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if (adr >= 0x8000) return;
    if (adr >= 0x6000) {
      s->bwram[(((uint32_t)(s->SNSBMAP & 0x1f) << 13) | (adr & 0x1fff)) & s->bwramMask] = val;
      return;
    }
    if (adr >= 0x3000 && adr < 0x3800) { s->iram[adr - 0x3000] = val; return; }
    if (adr >= 0x2200 && adr < 0x2400) sa1_reg_write(s, adr, val);
    return;
  }
  if (bank >= 0x40 && bank < 0x50) sa1_bwram_linear_write(s, bank, adr, val);
}

/* --- read maps -------------------------------------------------------------- */

uint8_t** g_sa1_rdmap;

/* A ROM page through the MMC: an 8 KB page never straddles a 32 KB LoROM bank
 * or a 1 MB block, so it is linear in the image. Not the bank $00 page with
 * the vectors (both CPUs' are replaced there) nor the page map_sa1 patched. */
static uint8_t* sa1_rom_page(Sa1* s, uint8_t bank, uint16_t page) {
  if (bank == 0x00 && page >= 0xe000) return NULL;
  uint32_t off = sa1_rom_offset(s, bank, page);
  if (s->romPatch && s->romPatch - 1 - off < 0x2000) return NULL;
  return s->rom + off;
}

/* BW-RAM read straight from memory: banks $40-$4F unless a type 1 character
 * conversion owns them, and the $6000 window of the given block register when
 * it is not the bit map (bit 7). Writes always take the slow path: they wake
 * the SA-1 or mark its lap (sa1_event, lapDirty). */
static uint8_t* sa1_bw_page(Sa1* s, uint8_t bank, uint16_t page, uint8_t blockReg) {
  uint8_t b = bank & 0x7f;
  if (b >= 0x40 && b < 0x50 && bank < 0x80)
    return s->SA1_in_cc1_dma ? NULL : s->bwram + ((((uint32_t)(b & 0x0f) << 16) | page) & s->bwramMask);
  if ((bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) && page == 0x6000 && !(blockReg & 0x80))
    return s->bwram + (((uint32_t)(blockReg & 0x1f) << 13) & s->bwramMask);
  return NULL;
}

uint8_t* sa1_snes_page(Sa1* s, uint8_t bank, uint16_t adr) {
  uint16_t page = adr & 0xe000;
  if (bank >= 0xc0 || ((bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) && page >= 0x8000))
    return sa1_rom_page(s, bank, page);
  if (s->bwramMask + 1 < 0x2000) return NULL;
  return sa1_bw_page(s, bank, page, s->SNSBMAP & 0x1f);   /* the 65816's window is never a bit map */
}

/* The $6000-$7FFF page of banks $00-$3F/$80-$BF in both maps: a BW-RAM block
 * register moved ($2224 the 65816's, $2225 the SA-1's). Only BW-RAM pages
 * change, never the fetch page (ROM / WRAM), so no refresh of that. */
static void sa1_maps_window(Sa1* s) {
  const bool bw = s->bwramMask + 1 >= 0x2000;
  for (uint32_t bank = 0; bank < 0xc0; bank++) {
    if (bank >= 0x40 && bank < 0x80) continue;
    const uint32_t p = (bank << 3) | 3;
    s->rdmap[p] = bw ? sa1_bw_page(s, (uint8_t)bank, 0x6000, s->BWUsed2) : NULL;
#if SNES_PAGEMAP
    g_snes_rdmap[p] = sa1_snes_page(s, (uint8_t)bank, 0x6000);
#endif
  }
}

/* Banks $40-$4F in both maps: a type 1 character conversion started or ended */
static void sa1_maps_bwlinear(Sa1* s) {
  for (uint32_t p = 0x40 << 3; p < (0x50 << 3); p++) {
    s->rdmap[p] = s->bwramMask + 1 >= 0x2000 ? sa1_bw_page(s, (uint8_t)(p >> 3), (uint16_t)(p << 13), 0) : NULL;
#if SNES_PAGEMAP
    g_snes_rdmap[p] = sa1_snes_page(s, (uint8_t)(p >> 3), (uint16_t)(p << 13));
#endif
  }
}

/* Rebuild both CPUs' maps whole: a ROM block moved, a reset, a load. */
static void sa1_maps(Sa1* s) {
  for (uint32_t p = 0; p < 0x800; p++) {
    uint8_t bank = (uint8_t)(p >> 3);
    uint16_t page = (uint16_t)(p << 13);
    uint8_t* m = NULL;
    if (bank >= 0xc0 || ((bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) && page >= 0x8000))
      m = sa1_rom_page(s, bank, page);
    else if (s->bwramMask + 1 >= 0x2000)
      m = sa1_bw_page(s, bank, page, s->BWUsed2);
    s->rdmap[p] = m;
  }
  g_sa1_rdmap = s->rdmap;
  if (s->snes) snes_pagemap_refresh(s->snes);
}

bool sa1_asleep(const Sa1* s) {
  return s->idle && !s->SA1_in_cc1_dma;
}

uint8_t sa1_peek(Sa1* s, uint8_t bank, uint16_t adr) {
  uint8_t b = bank & 0x7f;
  if (bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if (adr >= 0x8000) return sa1_rom_read(s, bank, adr);
    if (adr >= 0x6000)
      return s->bwram[(((uint32_t)(s->SNSBMAP & 0x1f) << 13) | (adr & 0x1fff)) & s->bwramMask];
    if (adr >= 0x3000 && adr < 0x3800) return s->iram[adr - 0x3000];
    return 0;
  }
  if (bank >= 0xc0) return sa1_rom_read(s, bank, adr);
  if (b >= 0x40 && b < 0x50) return s->bwram[(((uint32_t)(b & 0x0f) << 16) | adr) & s->bwramMask];
  return 0;
}

/* --- the SA-1 CPU's view --------------------------------------------------- */

uint8_t sa1_cpu_read(Sa1* s, uint32_t a) {
  uint8_t bank = (uint8_t)(a >> 16);
  uint16_t adr = (uint16_t)a;
  SA1_PERF_INC(snes_perf_sa1_cpu_rd, 1);
  if (bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if (adr >= 0x8000) {
      /* its own vectors come from $2203-$2208 */
      if (bank == 0x00 && adr >= 0xffea) {
        switch (adr) {
          case 0xffea: return (uint8_t)s->SA1NMIV;
          case 0xffeb: return (uint8_t)(s->SA1NMIV >> 8);
          case 0xffee: return (uint8_t)s->SA1IRQV;
          case 0xffef: return (uint8_t)(s->SA1IRQV >> 8);
          case 0xfffc: return (uint8_t)s->SA1ResetV;
          case 0xfffd: return (uint8_t)(s->SA1ResetV >> 8);
        }
      }
      return sa1_rom_read(s, bank, adr);
    }
    if (adr >= 0x6000) return sa1_bw_window_read(s, adr);
    if (adr < 0x0800) return s->iram[adr];
    if (adr >= 0x3000 && adr < 0x3800) return s->iram[adr - 0x3000];
    if (adr >= 0x2200 && adr < 0x2400) return sa1_reg_read(s, adr);
    return 0;
  }
  if (bank >= 0xc0) return sa1_rom_read(s, bank, adr);
  if (bank >= 0x40 && bank < 0x50) return sa1_bwram_linear_read(s, bank, adr);
  if (bank >= 0x60 && bank < 0x70) return sa1_bm_read(s, bank, adr);
  return 0;
}

void sa1_cpu_write(Sa1* s, uint32_t a, uint8_t val) {
  uint8_t bank = (uint8_t)(a >> 16);
  uint16_t adr = (uint16_t)a;
  s->lapDirty = 1;
  if (bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if (adr >= 0x8000) return;
    if (adr >= 0x6000) { sa1_bw_window_write(s, adr, val); return; }
    if (adr < 0x0800) { s->iram[adr] = val; return; }
    if (adr >= 0x3000 && adr < 0x3800) { s->iram[adr - 0x3000] = val; return; }
    if (adr >= 0x2200 && adr < 0x2400) sa1_reg_write(s, adr, val);
    return;
  }
  if (bank >= 0x40 && bank < 0x50) { sa1_bwram_linear_write(s, bank, adr, val); return; }
  if (bank >= 0x60 && bank < 0x70) sa1_bm_write(s, bank, adr, val);
}

/* --- life cycle ------------------------------------------------------------ */

Sa1* sa1_alloc(void) {
  Sa1* s = snes_zalloc(sizeof(Sa1));
  if (!s) return NULL;
  s->cpu = sa1cpu_init(s, 1);
  if (!s->cpu) { snes_zfree(s); return NULL; }
  return s;
}

void sa1_link(Sa1* s, Snes* snes, uint8_t* rom, uint32_t romSize, uint8_t* bwram, uint32_t bwramSize) {
  static const uint8_t test[] = { 0xa9, 0x10, 0xcf, 0xad };
  s->snes = snes;
  s->rom = rom;
  s->romSize = romSize;
  s->romMask = (romSize & (romSize - 1)) == 0 ? romSize - 1 : 0;
  s->bwram = bwram;
  s->bwramMask = bwramSize ? bwramSize - 1 : 0;
  /* map_sa1: a ROM with this sequence at $B95 gets $B96 zeroed */
  s->romPatch = (romSize > 0xb99 && !memcmp(rom + 0xb95, test, 4)) ? 0xb96 + 1 : 0;
  sa1_maps(s);
}

/* ZSNES SA1Reset: the SA-1 held in reset, the 65816 vectors its own, the MMC
 * on blocks 0-3 */
void sa1_reset(Sa1* s) {
  memset((uint8_t*)s + offsetof(Sa1, SA1_SAVE_FIRST), 0, SA1_SAVE_END - offsetof(Sa1, SA1_SAVE_FIRST));
  s->SA1Control = 0x20;
  for (int i = 0; i < 4; i++) s->SA1BankVal[i] = (uint8_t)i;
  sa1cpu_reset(s->cpu);
  s->cpu->sp = 0x01ff;
  s->idle = 0;
  s->candPc = 0xffff;
  s->candK = 0xff;
  s->extEvents++;
  sa1_maps(s);
}

void sa1_saveload(Sa1* s, SaveLoadFunc* func, void* ctx) {
  sa1_event(s);               /* the state as if the SA-1 had kept looping */
  func(ctx, (uint8_t*)s + offsetof(Sa1, SA1_SAVE_FIRST), SA1_SAVE_END - offsetof(Sa1, SA1_SAVE_FIRST));
  sa1cpu_saveload(s->cpu, func, ctx);
  sa1_maps(s);                /* a load brings its own MMC and BW-RAM windows */
}

/* --- scheduling ------------------------------------------------------------ */

void sa1_sync(Sa1* s, uint16_t v, uint16_t h) {
  int32_t lines = (int32_t)v - s->lastV;
  if (lines < 0) lines += s->snes->pal ? 312 : 262;
  int32_t clocks = lines * 1364 + (int32_t)h - s->lastH;
  s->lastV = v;
  s->lastH = h;
  if (clocks > 0) sa1_run(s, (uint32_t)clocks);
}

#ifdef HOST_BUILD
uint32_t sa1_fs_skip;         /* FRAME_STATS: SA-1 instructions the skip did not run */
static int sa1_skip_on(void) {
  static int on = -1;
  if (on < 0) { const char* e = getenv("SA1_IDLE"); on = !(e && e[0] == '0'); }
  return on;
}
#else
#define sa1_skip_on() 1
#endif

/* Run the SA-1 while it owes clocks. Each instruction charges its opcode-table
 * cycles, one SA-1 clock each (no bus waits modelled).
 *
 * The wait-loop skip: a jump back (same bank, at most 64 bytes) lands on a
 * loop head. If the next arrival at the same head finds the CPU exactly as the
 * last one left it -- registers, flags, modes -- and the lap in between wrote
 * nothing, read no clock and no side-effecting register, and nothing outside
 * happened (extEvents), every further lap is that same lap, until something
 * outside does happen. So the SA-1 sleeps there: its clocks keep accruing in
 * clockDebt, and the next event (sa1_event: a 65816 write to I-RAM, BW-RAM or
 * the registers, a 65816 read with a side effect, a save state) first wakes it
 * (sa1_wake). Proven on the CPU state, not on a table of known loops. */
static void sa1_interpret(Sa1* s) {
  Cpu* c = s->cpu;
  while (s->clockDebt > 0) {
    if (c->stopped || (c->waiting && !c->irqWanted && !c->nmiWanted)) {
      s->clockDebt = 0;
      break;
    }
    const uint16_t pc0 = c->pc;
    const uint8_t k0 = c->k;
    const int cyc = sa1cpu_runOpcode(c);
    SA1_PERF_INC(snes_perf_sa1_ops, 1);
    s->clockDebt -= cyc;
    s->lapCycles += cyc;
    s->lapInstrs++;
    if (c->k != k0 || c->pc >= pc0 || pc0 - c->pc > 64) continue;
    /* at a loop head */
    if (c->pc == s->candPc && c->k == s->candK && !s->lapDirty && s->lapEvents == s->extEvents &&
        !memcmp(s->snap, &c->a, SA1_CPU_SNAP) && sa1_skip_on()) {
      s->idle = 1;
      s->idleLap = s->lapCycles;
      s->idleInstrs = s->lapInstrs;
      return;
    }
    s->candPc = c->pc;
    s->candK = c->k;
    memcpy(s->snap, &c->a, SA1_CPU_SNAP);
    s->lapDirty = 0;
    s->lapEvents = s->extEvents;
    s->lapCycles = 0;
    s->lapInstrs = 0;
  }
}

/* Out of the wait loop: the laps it would have run while asleep are whole
 * laps of a known length that change nothing, so charge every lap that would
 * have started and finished (an instruction runs while clockDebt > 0), then
 * interpret the last, partial one. The SA-1 ends exactly where running every
 * lap would have left it. */
static void sa1_wake(Sa1* s) {
  SNES_PERF_BEGIN(perf_t);
  s->idle = 0;
  SA1_PERF_INC(snes_perf_sa1_wakes, 1);
  if (s->clockDebt > 0 && s->idleLap > 0) {
    const int32_t laps = (s->clockDebt - 1) / s->idleLap;
    s->clockDebt -= laps * s->idleLap;
#ifdef HOST_BUILD
    sa1_fs_skip += (uint32_t)laps * s->idleInstrs;
#endif
    SA1_PERF_INC(snes_perf_sa1_skip, (uint32_t)laps * s->idleInstrs);
  }
  s->candPc = 0xffff;         /* the rest of the lap must not re-prove it */
  sa1_interpret(s);
  SNES_PERF_END(snes_perf_gsu, perf_t);
}

/* Held in reset or wait ($2200 bits 5/6), or in WAI with nothing pending, the
 * slice is dropped. */
void sa1_run(Sa1* s, uint32_t masterClocks) {
  if (s->SA1Control & 0x60) { s->clockDebt = 0; return; }
  /* the odd master clock carries over: the SA-1's time must not depend on
   * how often it is called (the 65816's idle skip calls it less) */
  masterClocks += s->halfClock;
  s->halfClock = masterClocks & 1;
  s->clockDebt += (int32_t)(masterClocks >> 1);
  if (!s->idle) {
    SNES_PERF_BEGIN(perf_t);
    sa1_interpret(s);
    SNES_PERF_END(snes_perf_gsu, perf_t);
  }
}

#ifdef HOST_BUILD
/* SA1_STATELOG=file (host): one line per frame hashing everything the skip
 * must leave as running every lap would -- WRAM, I-RAM, BW-RAM, both CPUs.
 * The SA-1 is first brought up to this dot (the 65816's idle skip leaves it
 * behind until the next instruction) and woken (sa1_event), both exact, so the
 * log is the same with SA1_IDLE=0, which turns both skips off. */
static uint32_t sa1_fnv(uint32_t h, const void* p, size_t n) {
  const uint8_t* b = p;
  while (n--) h = (h ^ *b++) * 16777619u;
  return h;
}

void sa1_statelog(Sa1* s, unsigned long frame) {
  static FILE* f;
  static int on = -1;
  if (on < 0) { const char* e = getenv("SA1_STATELOG"); f = e ? fopen(e, "w") : NULL; on = f != NULL; }
  if (!on) return;
  sa1_sync(s, s->snes->vPos, s->snes->hPos);   /* both runs observed at the same dot */
  sa1_event(s);
  const uint32_t h0 = 2166136261u;
  fprintf(f, "%lu wram=%08x iram=%08x bwram=%08x sa1=%08x cpu=%08x debt=%d sa1pc=%02x:%04x cpupc=%02x:%04x\n", frame,
          (unsigned)sa1_fnv(h0, s->snes->ram, 0x20000), (unsigned)sa1_fnv(h0, s->iram, sizeof s->iram),
          (unsigned)sa1_fnv(h0, s->bwram, s->bwramMask + 1), (unsigned)sa1_fnv(h0, &s->cpu->a, SA1_CPU_SNAP),
          (unsigned)sa1_fnv(h0, &s->snes->cpu->a, SA1_CPU_SNAP), (int)s->clockDebt,
          s->cpu->k, s->cpu->pc, s->snes->cpu->k, s->snes->cpu->pc);
}
#endif
