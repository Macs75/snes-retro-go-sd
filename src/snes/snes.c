
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "../snes_perf.h"
#include <stddef.h>
#include <assert.h>
#include "snes.h"
#ifdef SNES_IDLE_SKIP
#include "idle_skip.h"
#endif
#include "cpu.h"
#include "apu.h"
#include "dma.h"
#include "ppu.h"
#include "cart.h"
#include "sdd1.h"
#include "spc7110.h"
#include "gsu.h"
#include "sa1.h"
#include "input.h"
#include "tracing.h"
#include "snes_gnw_alloc.h"

extern bool g_is_turbo;

#if SNES_PAGEMAP && SNES_ROMPAGE_LOW
#error "SNES_PAGEMAP reproduces the fast paths with SNES_ROMPAGE_LOW off; add the low-ROM rule first"
#endif
#if SNES_PAGEMAP
/* The 65816 read page map: one entry per 8 KB page of the 24-bit bus, a host
 * pointer for a page snes_cpuRead serves directly, NULL for one that goes to
 * snes_read. The rules are exactly the fast paths the compare chain had --
 * WRAM in banks $7E/$7F, its low 8 KB mirror in banks $00-$3F/$80-$BF, and
 * ROM at offsets >= $8000 where the cart allows the ROM page cache, outside
 * the DSP-1 window -- each condition falls on an 8 KB boundary, so a page is
 * uniformly one or the other. The SNES maps memory in 8 KB granules (low
 * WRAM) and 32 KB LoROM banks, hence the page size (the arcade cores' map is
 * 64 KB pages). Host pointers only: never in a savestate; rebuilt wherever
 * the ROM page cache is dropped (init, reset after every load, savestates). */
uint8_t *g_snes_rdmap[0x800];
/* SA-1 carts (sa1_link): its I-RAM and where it last caught up to (lastV,
 * lastH), for the 65816's I-RAM fast path in snes_cpuRead/snes_cpuWrite */
uint8_t *g_sa1_iram;
const uint16_t *g_sa1_vh;
const uint8_t *g_sa1_bwwin;   /* the 65816's $6000-$7FFF BW-RAM block */
const uint8_t *g_sa1_bwlin;   /* BW-RAM for banks $40-$4F, NULL during CC1 */
uint32_t g_sa1_bwmask;        /* (bank & $0F) << 16 | address, wrapped to BW-RAM */
/* the write map: the WRAM pages only, exactly snes_cpuWrite's fast paths */
uint8_t *g_snes_wrmap[0x800];

static uint8_t *snes_pagemap_base(Snes *snes, uint32_t adr) {
  uint8_t bank = adr >> 16;
  uint16_t off = (uint16_t)adr;
  if (bank == 0x7e || bank == 0x7f) return &snes->ram[((bank & 1) << 16) | off];
  if (off < 0x2000 && (bank < 0x40 || (bank >= 0x80 && bank < 0xc0))) return &snes->ram[off];
  Cart *cart = snes->cart;
  /* SA-1: its MMC's ROM pages and plain BW-RAM (sa1_snes_page), rebuilt by
   * snes_pagemap_refresh when the mapping moves */
  if (cart && cart->sa1) return sa1_snes_page(cart->sa1, bank, off);
  /* Super FX: exactly the ROM that cart_gsu_read serves -- $00-$3F/$80-$BF
   * upper halves as LoROM, $40-$5F/$C0-$DF as linear 64 KB banks. Game Pak RAM,
   * the RAM mirror and the GSU registers keep the slow path. (romPageOk is off
   * for these carts; the CPU is never locked out of ROM in this port.) */
  if (cart && cart->gsu) {
    if (!cart->romMask) return NULL;   /* Super FX ROMs are 512 KB-2 MB, all 2^n */
    uint8_t b = bank & 0x7f;
    uint32_t page = adr & ~(uint32_t)0x1fff;
    if (b <= 0x3f && off >= 0x8000)
      return cart->rom + ((((uint32_t)b << 15) | (page & 0x7fff)) & cart->romMask);
    if (b >= 0x40 && b <= 0x5f)
      return cart->rom + ((((uint32_t)(b & 0x1f) << 16) | (page & 0xffff)) & cart->romMask);
    return NULL;
  }
  /* NOT the same as cart_readLorom in one place: a LoROM cart under 2 MB with
   * SRAM has its SRAM across the whole of banks $70-$7D/$F0-$FF (dinkc64
   * e9e1139, PGA Tour), but this maps their $8000-$FFFF to ROM. So the 65816's
   * data reads there get ROM while snes_read (and DMA) get SRAM, and the PGA
   * Tour fix holds only for those. dma_doDma excludes these banks from its
   * page-map path to stay with snes_read. Making this agree means NULL pages
   * there (or SRAM pointers); it changes what the CPU reads on such carts. */
  if (cart && off >= 0x8000 && SNES_ROM_PAGE_OK(cart) && !SNES_DSP_LOROM_WINDOW(cart, bank)) {
    uint32_t page = adr & ~(uint32_t)0x1fff;
#if SNES_ROMPAGE_FOLD
    return (cart->type == 1)
      ? cart->bankBase[bank & 0x7f] + (page & 0x7fff)
      : cart->bankBase[bank & 0x3f] + (page & 0xffff);
#else
    uint32_t pidx = (cart->type == 1)
      ? (((uint32_t)((page >> 16) & 0x7f) << 15) | (page & 0x7fff))  /* LoROM */
      : (((uint32_t)((page >> 16) & 0x3f) << 16) | (page & 0xffff)); /* HiROM */
    return cart->rom + (pidx & cart->romMask);
#endif
  }
  /* HiROM low half ($40-$7D/$C0-$FF, $0000-$7FFF) is ROM too: cart_readHirom
   * serves `adr>=0x8000 || (bank&0x7f)>=0x40` from cart_romIndex, so the whole
   * of those banks maps flat through the same fold-baked bankBase the high half
   * uses. The $8000 gate above (correct for LoROM, where the low half is
   * WRAM/SRAM/MMIO) left this on the slow path -- the bulk of a HiROM game's ROM
   * data reads. romDmaDirect is exactly "plain HiROM, romPageOk, no coprocessor
   * intercepting" (spc7110 $40-4F/$C0-FF, sdd1 $C0-FF, cx4 $70-77), so it is the
   * right guard; bit 6 of the raw bank selects the $40-7F/$C0-FF window. */
#if SNES_ROMPAGE_FOLD
  if (cart && cart->romDmaDirect && off < 0x8000 && (bank & 0x40)) {
    uint32_t page = adr & ~(uint32_t)0x1fff;
    return cart->bankBase[bank & 0x3f] + (page & 0xffff);
  }
#endif
  return NULL;
}

static void snes_build_pagemap(Snes *snes);
/* The cartridge mapping moved (the SA-1's MMC or BW-RAM windows): rebuild the
 * map and drop the fetch page, which may point at the old mapping. */
void snes_pagemap_refresh(Snes *snes) {
  snes->romPageTag = ~(uint32_t)0;
  snes->romPageBase = NULL;
  snes_build_pagemap(snes);
}

static void snes_build_pagemap(Snes *snes) {
  for (uint32_t p = 0; p < 0x800; p++) {
    uint32_t adr = p << 13;
    uint8_t bank = adr >> 16;
    uint16_t off = (uint16_t)adr;
    g_snes_rdmap[p] = snes_pagemap_base(snes, adr);
    g_snes_wrmap[p] = (bank == 0x7e || bank == 0x7f ||
                       (off < 0x2000 && (bank < 0x40 || (bank >= 0x80 && bank < 0xc0))))
                      ? g_snes_rdmap[p] : NULL;
  }
}
#endif
int snes_frame_counter;
static const double apuCyclesPerMaster = (32040 * 32) / (1364 * 262 * 60.0);
extern uint8_t g_ram[0x20000];

extern void RtlApuWrite(uint32_t adr, uint8_t val);

static uint8_t snes_readReg(Snes* snes, uint16_t adr);
static void snes_writeReg(Snes* snes, uint16_t adr, uint8_t val);

#ifdef HOST_BUILD
static int host_boot_mmio_trace_on(void) {
  static int t = -1;
  if (t < 0) t = getenv("HOST_BOOT_MMIO") ? 1 : 0;
  return t;
}
#define HOST_BOOT_MMIO_LOG(addr, val) do { \
  if (host_boot_mmio_trace_on()) \
    printf("bootmmio: %04x=%02x @%02x:%04x\n", \
           (unsigned)(addr), (unsigned)(val), \
           (unsigned)snes->cpu->k, (unsigned)snes->cpu->pc); \
} while (0)
#else
#define HOST_BOOT_MMIO_LOG(addr, val) do {} while (0)
#endif

Snes* snes_init(uint8_t *ram) {
  Snes* snes = snes_zalloc(sizeof(Snes));
  if (!snes) return NULL;
  snes->ram = ram;
  snes->debug_cycles = false;
  snes->debug_apu_cycles = false;
  snes->runningWhichVersion = 0;
  /* NEVER initialized anywhere else, yet gated on at snes_handle_pos_stuff()
   * (hPos==512): a malloc'd Snes leaves it garbage. On a host the heap happens to
   * be zero so the line renders; on the device the DTCM heap is non-zero, so
   * disableRender reads true, ppu_runLine is skipped every scanline, and the
   * screen is black while the game runs fine (cb=0, cgram populated). */
  snes->disableRender = false;

  /* snes_init() only malloc()s and assigns fields -- it never memsets the
   * struct -- so every new field starts as heap garbage. On a host that heap
   * reads back as zeros and nothing shows; on the device's DTCM heap it does
   * not, and a garbage tag that happened to match a page would dereference a
   * garbage pointer. Set it here as well as in snes_reset(). */
  snes->romPageTag = ~(uint32_t)0;
  snes->romPageBase = NULL;
  g_sa1_iram = NULL;          /* until an SA-1 cart links it (sa1_link) */
  g_sa1_vh = NULL;
  g_sa1_bwwin = g_sa1_bwlin = NULL;
#if SNES_PAGEMAP
  snes_build_pagemap(snes);
#endif
  snes->pal = false;
  snes->vcount = 262;
  snes->apuDotsAccum = 0;
  snes->cpu = cpu_init(snes, 0);
#if defined(TARGET_GNW) && !defined(GNW_SNES_CORE)
  /* The Super Metroid port has no reference emulator on the device: the SPC700
   * emulator (66 KB, incl. 64 KB of ARAM) is dead weight because g_use_my_apu_code
   * routes audio through spc_player, and the second PPU exists only to diff
   * against. The standalone SNES core (GNW_SNES_CORE) needs both back — it has no
   * reimplementation to lean on. */
  snes->apu = NULL;
  snes->dma = dma_init(snes);
  snes->my_ppu = ppu_init(snes);
  snes->snes_ppu = snes->my_ppu;
  snes->ppu = snes->my_ppu;
#elif defined(GNW_SNES_CORE)
  snes->apu = apu_init();
  snes->dma = dma_init(snes);
  snes->my_ppu = ppu_init(snes);   /* one PPU: nothing to compare against */
  snes->snes_ppu = snes->my_ppu;
  snes->ppu = snes->my_ppu;
#else
  snes->apu = apu_init();
  snes->dma = dma_init(snes);
  snes->my_ppu = ppu_init(snes);
  snes->snes_ppu = ppu_init(snes);
  snes->ppu = snes->snes_ppu;
#endif
  snes->cart = cart_init(snes);
  snes->input1 = input_init(snes);
  snes->input2 = input_init(snes);
  return snes;
}

void snes_free(Snes* snes) {
  cpu_free(snes->cpu);
  apu_free(snes->apu);
  dma_free(snes->dma);
  ppu_free(snes->ppu);
  cart_free(snes->cart);
  input_free(snes->input1);
  input_free(snes->input2);
  snes_zfree(snes);
}

void snes_saveload(Snes *snes, SaveLoadFunc *func, void *ctx) {
  cpu_saveload(snes->cpu, func, ctx);
  apu_saveload(snes->apu, func, ctx);
  dma_saveload(snes->dma, func, ctx);
  ppu_saveload(snes->ppu, func, ctx);
  cart_saveload(snes->cart, func, ctx);

  func(ctx, &snes->hPos, offsetof(Snes, openBus) + 1 - offsetof(Snes, hPos));
  func(ctx, snes->ram, 0x20000);
  func(ctx, &snes->ramAdr, 4);

  snes->runningWhichVersion = 0;
  /* The fetch-page cache is deliberately not part of the range above (a host
   * pointer must never be written to or read from a savestate). Drop it on
   * both save and load so a load can never resume on a pointer built before
   * it -- cheap, and it removes the whole question. */
  snes->romPageTag = ~(uint32_t)0;
  snes->romPageBase = NULL;
#if SNES_PAGEMAP
  snes_build_pagemap(snes);
#endif
}

void snes_reset(Snes* snes, bool hard) {
  /* Drop the fetch-page cache BEFORE anything reads through snes_cpuRead():
   * cart_reset() may map a different ROM, and cpu_reset() immediately below
   * fetches the reset vector. The tag is always page-aligned inside a 24-bit
   * bus, so ~0 can never match a real address. */
  snes->romPageTag = ~(uint32_t)0;
  snes->romPageBase = NULL;
  cart_reset(snes->cart); // reset cart first, because resetting cpu will read from it (reset vector)
#if SNES_PAGEMAP
  snes_build_pagemap(snes);   /* after the cart, before the first read (the reset vector) */
#endif
  cpu_reset(snes->cpu);
  apu_reset(snes->apu);
  dma_reset(snes->dma);
  ppu_reset(snes->my_ppu);
  ppu_reset(snes->snes_ppu);
  input_reset(snes->input1);
  input_reset(snes->input2);
  if (hard)
    memset(snes->ram, 0x55, 0x20000);
  snes->ramAdr = 0;
  snes->hPos = 0;
  snes->vPos = 0;
  snes->frames = 0;
  snes->cpuCyclesLeft = 52; // 5 reads (8) + 2 IntOp (6)
  snes->cpuMemOps = 0;
  snes->apuCycleAcc = 0;
  snes->apuDotsAccum = 0;
  snes->hIrqEnabled = false;
  snes->vIrqEnabled = false;
  snes->nmiEnabled = false;
  snes->hTimer = 0x1ff;
  snes->vTimer = 0x1ff;
  snes->inNmi = false;
  snes->inIrq = false;
  snes->inVblank = false;
  memset(snes->portAutoReadX, 0, sizeof(snes->portAutoReadX));
  snes->autoJoyRead = false;
  snes->autoJoyTimer = 0;
  snes->ppuLatch = true; // $4201 powers up as 0xff: the IO line idles high (dinkc64 70b26c3)
  snes->multiplyA = 0xff;
  snes->multiplyResult = 0xfe01;
  snes->divideA = 0xffff;
  snes->divideResult = 0x101;
  snes->fastMem = false;
  snes->openBus = 0;
  snes->nmiReadEarly = false;
  snes_set_region(snes, snes->pal);
}

void snes_handle_pos_stuff(Snes *snes) {
  // handle positional stuff
  // TODO: better timing? (especially Hpos)
  if (snes->hPos == 0) {
    // end of hblank, do most vPos-tests
    bool startingVblank = false;
    if (snes->vPos == 0) {
      // end of vblank
      snes->inVblank = false;
      snes->inNmi = false;
      dma_initHdma(snes->dma);
    } else if (snes->vPos == 225) {
      // ask the ppu if we start vblank now or at vPos 240 (overscan)
      startingVblank = !ppu_checkOverscan(snes->ppu);
    } else if (snes->vPos == 240) {
      // if we are not yet in vblank, we had an overscan frame, set startingVblank
      if (!snes->inVblank) startingVblank = true;
    }
    if (startingVblank) {
      // if we are starting vblank
      ppu_handleVblank(snes->ppu);
      snes->inVblank = true;
      snes->inNmi = !snes->nmiReadEarly;   /* already read and cleared: see $4210 */
      snes->nmiReadEarly = false;
#ifdef GNW_SNES_CORE
      /* Only if the game asked for it. Super Metroid always has NMI on, so its
       * reimplementation could take the shortcut of firing unconditionally — but
       * an arbitrary game gets an NMI before it has set up its vector, and dies.
       * (This alone was 100 of 125 games booting to a black screen.) */
#ifdef SNES_IDLE_SKIP
      idle_nmi_sample(snes);   /* where the CPU waits: see idle_skip.h */
#endif
      if (snes->nmiEnabled)
        snes->cpu->nmiWanted = true;
#else
      snes->cpu->nmiWanted = true; // request NMI on CPU
#endif
      if (snes->autoJoyRead) {
        // TODO: this starts a little after start of vblank
        snes->autoJoyTimer = snes->vPos;   /* busy flag: see $4212 */
        /* After 16 hardware clocks the leftover serial bits are 1. */
        snes->input1->latchedState = 0xffff;
        snes->input2->latchedState = 0xffff;
      }
      if (snes->cart && snes->cart->spc7110)
        spc7110_tick(snes->cart->spc7110);
      if (snes->cart && snes->cart->gsu)
        gsu_vblank(snes->cart->gsu);   /* finish the frame's GSU work */
    }
    if (snes->cart && snes->cart->gsu)
      gsu_runLine(snes->cart->gsu);    /* one scanline's GSU slice */
    if (snes->cart && snes->cart->sa1)
      sa1_sync(snes->cart->sa1, snes->vPos, snes->hPos);  /* catch up to this line */
  } else if (snes->hPos == 512) {
    // render the line halfway of the screen for better compatibility
    if (!snes->inVblank && !snes->disableRender) {
      SNES_PERF_BEGIN(perf_t);
      ppu_runLine(snes->ppu, snes->vPos);
      SNES_PERF_END(snes_perf_ppu, perf_t);
#ifdef SNES_PERF
      snes_perf_ppu_lines++;
#endif
    }
  } else if (snes->hPos == 1024) {
    // start of hblank
    if (!snes->inVblank && snes->dma->hdmaEnabled)   /* no HDMA armed -> skip the 8-channel scan */
      dma_doHdma(snes->dma);
  }
#ifdef GNW_SNES_CORE
  /* H/V timer IRQ. Raster splits, mid-frame scroll changes and mode switches all
   * hang off this; without it a great many games either freeze or render one
   * frozen layer. Super Metroid's reimplementation raised its own IRQ, so the
   * emulator never needed to. */
  if (snes->hIrqEnabled || snes->vIrqEnabled) {
    bool match = true;
    if (snes->vIrqEnabled && snes->vPos != snes->vTimer) match = false;
    if (snes->hIrqEnabled) {
      if (snes->hPos != snes->hTimer * 4) match = false;
    } else if (snes->hPos != 0) {
      /* V-IRQ only: once at the start of the line, not on every dot. */
      match = false;
    }
    if (match) {
      snes->inIrq = true;
      snes->cpu->irqWanted = true;
#ifdef SNES_IDLE_SKIP
      idle_irq_sample(snes);   /* where the CPU waits for it */
#endif
    }
  }
#endif
  // increment position
  // TODO: exact frame timing (line 240 on odd frame is 4 cycles shorter,
  //   even frames in interlace is 1 extra line)
  snes->hPos += 2;
  if (snes->hPos == 1364) {
    snes->hPos = 0;
    snes->vPos++;
    if (snes->vPos == snes->vcount) {
      snes->vPos = 0;
      snes->frames++;
//      snes_catchupApu(snes); // catch up the apu at the end of the frame
    }
  }
}

/* One scanline per call, instead of one dot-pair per call.
 *
 * snes_handle_pos_stuff() steps the dot clock two dots at a time, so a frame
 * costs 178,684 calls of it — and all but ~800 of those do nothing except
 * increment a counter and fall through every branch. On a desktop that is 14% of
 * the run; on an in-order 280 MHz Cortex-M7 with no branch predictor to speak of
 * it is a large part of the frame budget, spent on nothing.
 *
 * Only three dot positions in a line do any work (0, 512, 1024), plus an H-timer
 * IRQ, which can land on any dot. Super Metroid never arms one — it uses the
 * V-timer — so take the dot loop only when one is actually armed. The events,
 * their order, and the state they leave behind are identical either way. */
void snes_run_line(Snes *snes) {
#if defined(GNW_SNES_CORE) || !SNES_LINE_HIRQ
  if (snes->hIrqEnabled || snes->hPos != 0) {
#else
  /* Native SM/Zelda ports: snes_handle_pos_stuff's H-timer is #ifdef
   * GNW_SNES_CORE, so the dot loop cannot raise that IRQ. Split at hPos==0. */
  if (snes->hPos != 0) {
#endif
    do { snes_handle_pos_stuff(snes); } while (snes->hPos != 0);
    return;
  }

  /* hPos == 0: end of hblank, the vPos tests */
  bool startingVblank = false;
  if (snes->vPos == 0) {
    snes->inVblank = false;
    snes->inNmi = false;
    dma_initHdma(snes->dma);
  } else if (snes->vPos == 225) {
    startingVblank = !ppu_checkOverscan(snes->ppu);
  } else if (snes->vPos == 240) {
    if (!snes->inVblank) startingVblank = true;
  }
  if (startingVblank) {
    ppu_handleVblank(snes->ppu);
    snes->inVblank = true;
    snes->inNmi = !snes->nmiReadEarly;   /* already read and cleared: see $4210 */
    snes->nmiReadEarly = false;
#ifdef SNES_IDLE_SKIP
    idle_nmi_sample(snes);   /* where the CPU waits: see idle_skip.h */
#endif
    if (snes->nmiEnabled)
      snes->cpu->nmiWanted = true;
    if (snes->autoJoyRead) {
      snes->autoJoyTimer = snes->vPos;   /* busy flag: see $4212 */
      snes->input1->latchedState = 0xffff;
      snes->input2->latchedState = 0xffff;
    }
    if (snes->cart && snes->cart->spc7110)
      spc7110_tick(snes->cart->spc7110);
    if (snes->cart && snes->cart->gsu)
      gsu_vblank(snes->cart->gsu);     /* finish the frame's GSU work */
  }
  if (snes->cart && snes->cart->gsu)
    gsu_runLine(snes->cart->gsu);      /* one scanline's GSU slice */
  if (snes->cart && snes->cart->sa1)
    sa1_sync(snes->cart->sa1, snes->vPos, snes->hPos);    /* catch up to this line */

  /* The V-timer matches on every dot of its line; setting the flag once is the
   * same thing to everyone who reads it. */
  if (snes->vIrqEnabled && snes->vPos == snes->vTimer) {
    snes->inIrq = true;
    snes->cpu->irqWanted = true;
#ifdef SNES_IDLE_SKIP
    idle_irq_sample(snes);   /* where the CPU waits for it */
#endif
  }

  /* hPos == 512: the line is rendered halfway across, for compatibility */
  if (!snes->inVblank && !snes->disableRender) {
    SNES_PERF_BEGIN(perf_t);
    ppu_runLine(snes->ppu, snes->vPos);
    SNES_PERF_END(snes_perf_ppu, perf_t);
#ifdef SNES_PERF
    snes_perf_ppu_lines++;
#endif
  }

  /* hPos == 1024: start of hblank */
  if (!snes->inVblank)
    dma_doHdma(snes->dma);

  /* end of line */
  snes->vPos++;
  if (snes->vPos == snes->vcount) {
    snes->vPos = 0;
    snes->frames++;
  }
}

#define IS_ADR(x) (x == 0xfffff)

/* APU cycles per master dot. NTSC: (32040*32)/(1364*262*60), PAL: over 312
 * lines at 50 Hz. Both reduce to exact fractions -- NTSC 2136/44671, PAL
 * 1068/22165 -- pre-scaled here to a Q32 fixed-point multiplier so the catch-up
 * can accumulate in integers. */
#define APU_CYC_SHIFT 32
#define APU_CYC_MUL_NTSC ((((uint64_t)2136u << APU_CYC_SHIFT) + 44671u / 2) / 44671u)
#define APU_CYC_MUL_PAL  ((((uint64_t)1068u << APU_CYC_SHIFT) + 22165u / 2) / 22165u)

void snes_set_region(Snes *snes, bool pal) {
  snes->pal = pal;
  snes->vcount = pal ? 312 : 262;
  if (snes->apu && snes->apu->dsp)
    snes->apu->dsp->frameSamples = pal ? DSP_SAMPLES_PAL : DSP_SAMPLES_NTSC;
}

/* Integer accumulation is associative, so the APU cycles run depend only on the
 * cumulative dot count, not on how the dots were split across catch-ups. The
 * old double accumulator did not have this property: x86 (host) and ARM (device
 * Thumb-2) round the double differently, so the same inputs could land on
 * opposite sides of the (int) truncation -- host and device then emulated
 * divergent state, and neither could be used to debug the other. apuCycleAcc
 * holds owed APU cycles in Q32 fixed point, identical on every platform. */
void snes_catchupApu(Snes* snes) {
  if (snes->apu == NULL)
    return;
  if (snes->apuDotsAccum) {
    uint64_t mul = snes->pal ? APU_CYC_MUL_PAL : APU_CYC_MUL_NTSC;
    snes->apuCycleAcc += (uint64_t)snes->apuDotsAccum * mul;
    snes->apuDotsAccum = 0;
  }
  int catchupCycles = (int)(snes->apuCycleAcc >> APU_CYC_SHIFT);
  if (catchupCycles > 10000) {            /* runaway guard: run 10000, drop the rest */
    catchupCycles = 10000;
    snes->apuCycleAcc = 0;
  } else {
    snes->apuCycleAcc -= (uint64_t)catchupCycles << APU_CYC_SHIFT;   /* keep the fraction */
  }
  apu_run(snes->apu, catchupCycles);
}

uint8_t snes_readBBus(Snes* snes, uint8_t adr) {
  if(adr < 0x40) {
    return ppu_read(snes->ppu, adr);
  }
  if(adr < 0x80) {
    if (snes->apu == NULL)
      return snes->openBus;   /* no SPC700 here: spc_player is the sound chip */
    /* Catch the APU up to *now*, which is what apuCatchupCycles has been counting.
     * Overwriting it with a flat 32 first (as this did) starves the SPC700: the
     * boot handshake every game does through these ports never completes and the
     * screen stays black. Dead code in the Super Metroid port — it returns above,
     * with no APU at all — and wrong everywhere else. */
    snes_catchupApu(snes); // catch up the apu before reading
    return snes->apu->outPorts[adr & 0x3];
  }
  if(adr == 0x80) {
    uint8_t ret = snes->ram[snes->ramAdr++];
    snes->ramAdr &= 0x1ffff;
    return ret;
  }

#ifndef GNW_SNES_CORE
  /* dev builds: an unhandled B-bus read is a porting bug worth stopping on.
   * General core: games read write-only $21xx registers in normal play and
   * real hardware answers with open bus -- fall through. */
  assert(0);
#endif
  return snes->openBus;
}


#define is_uploading_apu (*(uint16_t*)(g_ram+0x617))

void snes_writeBBus(Snes* snes, uint8_t adr, uint8_t val) {
  if(adr < 0x40) {
    ppu_write(snes->ppu, adr, val);
    return;
  }
  if(adr < 0x80) {
    RtlApuWrite(0x2100 + adr, val);
    return;
  }
  switch(adr) {
    case 0x80: {
      snes->ram[snes->ramAdr++] = val;
      snes->ramAdr &= 0x1ffff;
      break;
    }
    case 0x81: {
      snes->ramAdr = (snes->ramAdr & 0x1ff00) | val;
      break;
    }
    case 0x82: {
      snes->ramAdr = (snes->ramAdr & 0x100ff) | (val << 8);
      break;
    }
    case 0x83: {
      snes->ramAdr = (snes->ramAdr & 0x0ffff) | ((val & 1) << 16);
      break;
    }
  }
}

static uint16_t SwapInputBits(uint16_t x) {
  uint16_t r = 0;
  for (int i = 0; i < 16; i++, x >>= 1)
    r = r * 2 + (x & 1);
  return r;
}

/* $4212 bit 0: the auto-joypad read is in progress, for 4224 master clocks
 * from the start of vblank (the line autoJoyTimer holds). Computed when read
 * rather than counted down every dot. Star Fox's briefing waits for it to SET
 * before reading the pad, and hung while it never did. Out of line, placed
 * after the BUS window (snes_core.ld), so snes_read keeps its size. */
/* $4212 as a read at dot h of this line returns it, with no side effect: the
 * idle skip (main_snes.c) holds a loop polling it only over a span where this
 * is the same at its first and last dot -- vblank and hblank change only on
 * span boundaries (dots 0 and 1024), the auto-joypad busy bit at most once. */
uint8_t snes_hvbjoy_at(const Snes* snes, uint16_t h) {
  uint8_t val = snes->autoJoyTimer &&
                (uint32_t)(uint16_t)(snes->vPos - snes->autoJoyTimer) * 1364 + h < 4224;
  val |= (h >= 1024) << 6;
  val |= snes->inVblank << 7;
  return val | (snes->openBus & 0x3e);
}

__attribute__((noinline)) static uint8_t snes_autoJoyBusy(const Snes* snes) {
  return snes->autoJoyTimer &&
         (uint32_t)(uint16_t)(snes->vPos - snes->autoJoyTimer) * 1364 + snes->hPos < 4224;
}

static uint8_t snes_readReg(Snes* snes, uint16_t adr) {
  switch(adr) {
    case 0x4210: {
#ifdef HOST_BUILD
      if (host_boot_mmio_trace_on())
        printf("bootmmio-r: 4210 inNmi=%d @%02x:%04x\n",
               (int)snes->inNmi,
               (unsigned)snes->cpu->k, (unsigned)snes->cpu->pc);
#endif
      uint8_t val = 0x2; // CPU version (4 bit)
      bool nmi = snes->inNmi;
      /* The read cycle is the LAST cycle of the instruction, but this core
       * runs an instruction whole at its start. If vblank begins inside the
       * instruction, real hardware reads the flag already set -- and that is
       * the only way a main loop polling $4210 with NMI enabled ever sees it,
       * because the NMI handler reads $4210 too and clears it first.
       * Final Fantasy VI's event "play song" waits exactly like that and hung
       * on a black screen. The read lands ~8 dots per cycle after the start. */
      if (!nmi && !snes->inVblank &&
          snes->vPos + 1 == ((snes->ppu && snes->ppu->overscan) ? 240 : 225)) {
        const int read_at = snes->hPos + (snes->cpu->cyclesUsed ? (snes->cpu->cyclesUsed - 1) * 8 : 0);
        if (read_at >= 1364) { nmi = true; snes->nmiReadEarly = true; }
      }
      val |= nmi << 7;
      /* RDNMI bit 7 clears on read (and at end of vblank). */
      snes->inNmi = false;
      return val | (snes->openBus & 0x70);
    }
    case 0x4211: {
      uint8_t val = snes->inIrq << 7;
      snes->inIrq = false;
      /* the SA-1's request shares the line */
      snes->cpu->irqWanted = snes->cart->sa1 && sa1_snes_irq(snes->cart->sa1);
      return val | (snes->openBus & 0x7f);
    }
    case 0x4212: {
#ifdef HOST_BUILD
      if (host_boot_mmio_trace_on())
        printf("bootmmio-r: 4212 vblank=%d hblank=%d @%02x:%04x\n",
               (int)snes->inVblank, (int)(snes->hPos >= 1024),
               (unsigned)snes->cpu->k, (unsigned)snes->cpu->pc);
#endif
      uint8_t val = snes_autoJoyBusy(snes);
      val |= (snes->hPos >= 1024) << 6;
      val |= snes->inVblank << 7;
      return val | (snes->openBus & 0x3e);
    }
    case 0x4213:
      return snes->ppuLatch << 7; // IO-port
    case 0x4214:
      return snes->divideResult & 0xff;
    case 0x4215:
      return snes->divideResult >> 8;
    case 0x4216:
      return snes->multiplyResult & 0xff;
    case 0x4217:
      return snes->multiplyResult >> 8;
    case 0x4218:
      return SwapInputBits(snes->input1->currentState) & 0xff;
    case 0x4219:
      return SwapInputBits(snes->input1->currentState) >> 8;
    case 0x421a:
      return SwapInputBits(snes->input2->currentState) & 0xff;
    case 0x421b:
      return SwapInputBits(snes->input2->currentState) >> 8;
    case 0x421c:
    case 0x421e:
    case 0x421d:
    case 0x421f:
      return 0;

    default: {
      return snes->openBus;
    }
  }
}

/* $4207/$4208 (HTIME), out of line: written a few times a frame, and
 * snes_write fills its fixed BUS window (snes_core.ld). */
__attribute__((noinline))
static void snes_write_htime(Snes* snes, uint16_t adr, uint8_t val) {
  if (adr == 0x4207) snes->hTimer = (snes->hTimer & 0x100) | val;
  else               snes->hTimer = (snes->hTimer & 0x0ff) | ((val & 1) << 8);
  snes->dma->hclip = 1;   /* the H-IRQ dot moved: end the span there */
}

static void snes_writeReg(Snes* snes, uint16_t adr, uint8_t val) {
  switch(adr) {
    case 0x4200: {
      HOST_BOOT_MMIO_LOG(adr, val);
      bool oldNmiEnabled = snes->nmiEnabled;
      bool oldVirqEnabled = snes->vIrqEnabled;
      snes->autoJoyRead = val & 0x1;
      if(!snes->autoJoyRead) snes->autoJoyTimer = 0;
      snes->hIrqEnabled = val & 0x10;
      snes->vIrqEnabled = val & 0x20;
      snes->dma->hclip = 1;   /* the H-IRQ may now match inside this span */
      snes->nmiEnabled = val & 0x80;
      if(!snes->hIrqEnabled && !snes->vIrqEnabled) {
        snes->inIrq = false;
        snes->cpu->irqWanted = snes->cart->sa1 && sa1_snes_irq(snes->cart->sa1);
      }
      /* SNES quirk: enabling NMI during vblank can assert NMI immediately.
       * Gate on the RDNMI latch (inNmi / $4210.7): if the game already
       * acknowledged vblank by reading $4210, a later NMITIMEN 0→1 must not
       * fire a second NMI in the same blank. Soul Blazer (France) does
       * LDA $4210 / LDA #$A1 / STA $4200 every frame; an ungated edge gave
       * two NMIs/frame and visible multi-Hz image jumps. */
      if (!oldNmiEnabled && snes->nmiEnabled && snes->inVblank && snes->inNmi) {
        snes->cpu->nmiWanted = true;
      }
      /* Similar behavior for V-IRQ enable on the matching line (without H-IRQ). */
      if (!oldVirqEnabled && snes->vIrqEnabled && !snes->hIrqEnabled &&
          snes->vPos == snes->vTimer) {
        snes->inIrq = true;
        snes->cpu->irqWanted = true;
      }
      break;
    }
    case 0x4201: {
      if(!(val & 0x80) && snes->ppuLatch) {
        // latch the ppu
        ppu_read(snes->ppu, 0x37);
      }
      snes->ppuLatch = val & 0x80;
      break;
    }
    case 0x4202: {
      snes->multiplyA = val;
      break;  
    }
    case 0x4203: {
      snes->multiplyResult = snes->multiplyA * val;
      break;
    }
    case 0x4204: {
      snes->divideA = (snes->divideA & 0xff00) | val;
      break;
    }
    case 0x4205: {
      snes->divideA = (snes->divideA & 0x00ff) | (val << 8);
      break;
    }
    case 0x4206: {
      if(val == 0) {
        snes->divideResult = 0xffff;
        snes->multiplyResult = snes->divideA;
      } else {
        snes->divideResult = snes->divideA / val;
        snes->multiplyResult = snes->divideA % val;
      }
      break;
    }
    case 0x4207:
    case 0x4208:
      snes_write_htime(snes, adr, val);
      break;
    case 0x4209: {
      snes->vTimer = (snes->vTimer & 0x100) | val;
      break;
    }
    case 0x420a: {
      snes->vTimer = (snes->vTimer & 0x0ff) | ((val & 1) << 8);
      break;
    }
    case 0x420b: {
      HOST_BOOT_MMIO_LOG(adr, val);
      if (val == 2) {
        uint32_t t = snes->dma->channel[1].aBank << 16 | snes->dma->channel[1].aAdr;
        int data = snes_read(snes, t) | snes_read(snes, t + 1) << 8;

        if (0)printf("DMA: 0x%x -> 0x%x (ppu 0x%x), 0x%x bytes, data 0x%x\n",
               t, snes->dma->channel[1].bAdr,
               snes->ppu->vramPointer, snes->dma->channel[1].size, data);
      }
      {
        /* The data moves now, but the CPU is halted for the transfer's bus time
         * on hardware: 8 master cycles a byte plus 8 a channel. Leave that in
         * dmaTimer with dmaBusy set; the scheduler runs it out as a DMA stall,
         * so interrupts raised meanwhile are taken only when it ends (Dirt Trax
         * FX's WAI after a 14 KB VRAM upload relies on the NMI landing first). */
        Dma *dma = snes->dma;
        uint32_t stall = 0;
        for (int c = 0; c < 8; c++)
          if (val & (1 << c)) stall += 8 * (dma->channel[c].size ? dma->channel[c].size : 0x10000u) + 8;
        dma_startDma(dma, val, false);
        SNES_PERF_DMA_BEGIN(perf_t, perf_a);
        for (;;) {
          int c = 0;
          while (c < 8 && !dma->channel[c].dmaActive) c++;
          if (c == 8) break;
          dma_cycle(dma);
        }
        SNES_PERF_DMA_END(perf_t, perf_a);
        if (dma->dmaBusy) dma->dmaTimer += stall;
      }
      break;
    }
    case 0x420c: {
      HOST_BOOT_MMIO_LOG(adr, val);
      dma_startDma(snes->dma, val, true);
      break;
    }
    case 0x420d: {
      snes->fastMem = val & 0x1;
      break;
    }
    default: {
      break;
    }
  }
}

/* noinline: snes_cpuRead's slow path calls it, and inlined there it put
 * ~700 bytes of I/O decode into ITCM for the accesses that need it least */
__attribute__((noinline))
uint8_t snes_read(Snes* snes, uint32_t adr) {
  uint8_t bank = adr >> 16;
  adr &= 0xffff;
  if(bank == 0x7e || bank == 0x7f) {
    return snes->ram[((bank & 1) << 16) | adr]; // ram
  }
  if(bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if(adr < 0x2000) {
      return snes->ram[adr]; // ram mirror
    }
    if(adr >= 0x2100 && adr < 0x2200) {
      return snes_readBBus(snes, adr & 0xff); // B-bus
    }
    if (adr == 0x4016)
      return input_read(snes->input1) | (snes->openBus & 0xfc);
    if (adr == 0x4017)
      return input_read(snes->input2) | 0x1c;
    if(adr >= 0x4200 && adr < 0x4220) {
      return snes_readReg(snes, adr); // internal registers
    }
    if(adr >= 0x4300 && adr < 0x4380) {
      return dma_read(snes->dma, adr); // dma registers
    }
    if(snes->cart->sdd1 && adr >= 0x4800 && adr < 0x4808) {
      return sdd1_mmio_read(snes->cart->sdd1, adr);
    }
    if(snes->cart->spc7110 && adr >= 0x4800 && adr <= 0x4842) {
      return spc7110_mmio_read(snes->cart->spc7110, adr, snes->cart->rom, snes->cart->romSize);
    }
  }
#if SNES_ROMPAGE_LOW == 2
  {
    Cart* c = snes->cart;
    if(c->bankLowRom[bank])
      return (c->type == 1) ? c->bankBase[bank & 0x7f][adr & 0x7fff]
                            : c->bankBase[bank & 0x3f][adr];
  }
#endif
  // read from cart
  return cart_read(snes->cart, bank, adr);
}

void LogWrite(Snes *snes, uint32_t adr, uint8_t val) {
  printf("@%d: Write to 0x%x = 0x%.2x: 0x%x: r18=0x%x: r20=0x%x: a = 0x%x, x = 0x%x, y = 0x%x, c = %d\n",
         snes_frame_counter, adr, val, snes->cpu->k << 16 | snes->cpu->pc,
         snes->ram[0x12] | snes->ram[0x13] << 8, 
          snes->ram[0x14] | snes->ram[0x15] << 8,
         snes->cpu->a, snes->cpu->x, snes->cpu->y, snes->cpu->c);
}

void snes_write(Snes* snes, uint32_t adr, uint8_t val) {
  uint8_t bank = adr >> 16;
  adr &= 0xffff;
  /* Early-return after each matched region: the ranges are disjoint and none of
   * them also needs the trailing cart_write (WRAM, B-bus, pads, internal regs,
   * DMA regs, sdd1/spc7110 MMIO are never cart targets), so a matched write
   * stops re-testing the rest and skips the no-op cart_write dispatch. The
   * $2000-20ff / $4220-42ff / $4380-47ff gaps still fall through to cart_write. */
  if(bank == 0x7e || bank == 0x7f) {
    uint32_t addr = ((bank & 1) << 16) | adr;
    snes->ram[addr] = val; // ram
    if (IS_ADR(addr)) LogWrite(snes, adr, val);
    return;
  }
  if(bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if(adr < 0x2000) {
      snes->ram[adr] = val; // ram mirror
      if (IS_ADR(adr)) LogWrite(snes, adr, val);
      return;
    }
    if(adr >= 0x2100 && adr < 0x2200) {
      if (adr == 0x2100) HOST_BOOT_MMIO_LOG(adr, val);
      snes_writeBBus(snes, adr & 0xff, val); // B-bus
      return;
    }
    if(adr == 0x4016) {
      input_latch(snes->input1, val & 1);
      input_latch(snes->input2, val & 1);
      return;
    }
    if(adr >= 0x4200 && adr < 0x4220) {
      snes_writeReg(snes, adr, val); // internal registers
      return;
    }
    if(adr >= 0x4300 && adr < 0x4380) {
      dma_write(snes->dma, adr, val); // dma registers
      if(snes->cart->sdd1)
        sdd1_mmio_write(snes->cart->sdd1, adr, val);
      return;
    }
    if(snes->cart->sdd1 && adr >= 0x4800 && adr < 0x4808) {
      sdd1_mmio_write(snes->cart->sdd1, adr, val);
      return;
    }
    if(snes->cart->spc7110 && adr >= 0x4800 && adr <= 0x4842) {
      spc7110_mmio_write(snes->cart->spc7110, adr, val, snes->cart->rom, snes->cart->romSize);
      return;
    }
  }
  cart_write(snes->cart, bank, adr, val);
}


/* WRAM fast paths: the DP/stack/data accesses that dominate CPU traffic resolve
 * to one array index, skipping the snes_read -> cart_read -> cart_readLorom call
 * chain. Anything with a side effect (B-bus/MMIO) or in ROM/SRAM keeps the slow
 * path, so behaviour is unchanged (state hash identical). Standard emulator
 * page-fast-path, minus the page table. */
/* Put the bus accessors in ITCM beside the engine that calls them. ITCM is at
 * 0x00000000 and the overlay at 0x24000000 -- past BL's +-16 MB -- so every read
 * from the Thumb-2 engine went through a linker veneer, an extra jump on the
 * hottest path in the emulator. The device profile showed the veneer alone at
 * 2.7% of the frame. The two functions are 244 bytes and ITCM has ~7 KB spare. */
#ifdef SNES_BUS_IN_ITCM
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
uint8_t snes_cpuRead(Snes* snes, uint32_t adr) {
  /* Count the access; do NOT charge for it here.
   *
   * Every bus access used to do cpuCyclesLeft += 8 as well, and the caller then
   * did cpuCyclesLeft += (cycles - memOps) * 6 once the opcode was over. The sum
   * of those is 6*cycles + 2*memOps, which the caller can compute on its own --
   * so the load, add and store on every single memory access were buying a
   * number nobody read until the opcode ended. Nothing inside an opcode reads
   * cpuCyclesLeft (the scheduler only looks at it between opcodes), and it
   * cannot overflow the uint8 on the way: it starts each opcode at 0 and the
   * worst opcode charges about 62.
   *
   * This is a removal, not a test that skips work -- the shape that keeps
   * losing on this chip. See docs/SNES_LAST_MILE.md. */
  snes->cpuMemOps++;
#ifdef RIG_CALL_PROFILE
  extern uint64_t g_cpuRead_calls, g_win_cpuRead_calls, g_cpuRead_slow, g_cpuRead_romhit, g_cpuRead_wram;
  g_cpuRead_calls++; g_win_cpuRead_calls++;
#endif
  /* Fetch-page cache. The ROM fast path below already collapsed the mapper to
   * one AND, but every single byte still re-ran the whole classification
   * chain above it -- two bank compares, the WRAM range test, the >=0x8000
   * test, the LoROM/HiROM select -- even though an opcode fetch walks the
   * same 8 KB of ROM for long stretches (fetch is ~77% of CPU reads). Cache
   * the host base of the last ROM page and serve a hit with one compare and
   * one index.
   *
   * Without the page map only the ROM branch installs a tag, so a tag match
   * implies ROM: WRAM, SRAM and MMIO all live below 0x8000 (or in banks
   * 7e/7f) and keep the slow path, which is also why no write path has to
   * invalidate this -- ROM does not change under us. (The page map also
   * installs WRAM pages; see there.) An 8 KB page cannot straddle a LoROM 32 KB
   * mapping boundary, and cannot straddle a romMask wrap either (romMask is
   * 2^n-1 with n >= 13 for any real cart), so base+offset stays linear for
   * the whole page. The tag holds the page-aligned address, so the sentinel
   * below (low bits set, and beyond the 24-bit bus) can never collide. */
#if SNES_PAGEMAP
  {
    /* One table load instead of the compare chain below. A ROM page, or a WRAM
     * page ($7E/$7F, and the $0000-$1FFF mirror in banks $00-$3F/$80-$BF),
     * also becomes the engine's inline fetch page (ROM_FETCH_FAST): only
     * fetches arrive here for those pages (cpu_thumb2_read serves the data
     * reads), and code run from WRAM otherwise paid this call per byte. The
     * page is a pointer to live memory and WRAM is never remapped, so a write
     * to it -- self-modifying code, DMA into WRAM -- needs no invalidation. */
    uint8_t *base = g_snes_rdmap[(adr >> 13) & 0x7ff];
    if (base) {
      if ((adr & 0x8000) || (adr >> 17) == 0x3f || !(adr & 0x40e000)) {
        uint32_t page = adr & ~(uint32_t)0x1fff;
        if (page != snes->romPageTag) {
          snes->romPageBase = base;
          snes->romPageTag = page;
          SNES_PERF_CPU_INC(snes_perf_cpu_rd_page);
        }
      }
      return base[adr & 0x1fff];
    }
#if SNES_SA1
    /* SA-1 cart memory the 65816 shares with the SA-1, served here instead of
     * through snes_read, cart_read and sa1_snes_read: I-RAM ($3000-$37FF of
     * banks $00-$3F/$80-$BF, its direct page in these games), the BW-RAM
     * window at $6000-$7FFF ($2224's block) and BW-RAM banks $40-$4F (not
     * during a type 1 character conversion: g_sa1_bwlin is NULL then). Each is
     * a contact (sa1.h): the SA-1 first catches up, unless it already has to
     * this dot (the other bytes of the same instruction). g_sa1_iram is NULL
     * without an SA-1. */
    if (g_sa1_iram) {
      const uint8_t *p = NULL;
      if ((adr & 0x40f800) == 0x003000) p = g_sa1_iram + (adr & 0x7ff);
      else if ((adr & 0x40e000) == 0x006000) p = g_sa1_bwwin + (adr & 0x1fff);
      else if ((adr & 0xf00000) == 0x400000 && g_sa1_bwlin) p = g_sa1_bwlin + (adr & g_sa1_bwmask);
      if (p) {
        if (g_sa1_vh[0] != snes->vPos || g_sa1_vh[1] != snes->hPos)
          sa1_sync(snes->cart->sa1, snes->vPos, snes->hPos);
        return *p;
      }
    }
#endif
    SNES_PERF_CPU_INC(snes_perf_cpu_rd_slow);
    return snes_read(snes, adr);
  }
#else
  if((adr & ~(uint32_t)0x1fff) == snes->romPageTag) {
#ifdef RIG_CALL_PROFILE
    g_cpuRead_romhit++;
#endif
#ifdef SNES_ROMPAGE_VERIFY
    {
      uint8_t want = snes_read(snes, adr);
      uint8_t got  = snes->romPageBase[adr & 0x1fff];
      if (want != got) {
        static int nh = 0;
        if (nh++ < 40)
          printf("ROMPAGE HIT MISMATCH adr=%06lx fast=%02x slow=%02x tag=%06lx\n",
                 (unsigned long)adr, got, want, (unsigned long)snes->romPageTag);
      }
    }
#endif
    return snes->romPageBase[adr & 0x1fff];
  }
  uint8_t bank = adr >> 16;
  uint16_t off = (uint16_t)adr;
  if(bank == 0x7e || bank == 0x7f) {
#ifdef RIG_CALL_PROFILE
    g_cpuRead_wram++;
#endif
    return snes->ram[((bank & 1) << 16) | off];
  }
  if(off < 0x2000 && (bank < 0x40 || (bank >= 0x80 && bank < 0xc0))) {
#ifdef RIG_CALL_PROFILE
    g_cpuRead_wram++;
#endif
    return snes->ram[off];
  }
  /* ROM fast path — the opcode/operand fetch that is ~77% of CPU reads. adr>=0x8000
   * is always ROM in LoROM/HiROM (SRAM/MMIO are all <0x8000), so only the mapper
   * index differs. Power-of-2 ROMs (romMask set = the common case) index with one
   * AND; odd sizes fall to the folding slow path. */
  Cart* cart = snes->cart;
  if(off >= 0x8000 && SNES_ROM_PAGE_OK(cart) && !SNES_DSP_LOROM_WINDOW(cart, bank)) {
    uint32_t page = adr & ~(uint32_t)0x1fff;
#if SNES_ROMPAGE_FOLD
    uint8_t* base = (cart->type == 1)
      ? cart->bankBase[bank & 0x7f] + (page & 0x7fff)
      : cart->bankBase[bank & 0x3f] + (page & 0xffff);
#else
    uint32_t pidx = (cart->type == 1)
      ? (((uint32_t)((page >> 16) & 0x7f) << 15) | (page & 0x7fff))  /* LoROM */
      : (((uint32_t)((page >> 16) & 0x3f) << 16) | (page & 0xffff)); /* HiROM */
    uint8_t* base = cart->rom + (pidx & cart->romMask);
#endif
    snes->romPageBase = base;
    snes->romPageTag = page;
    SNES_PERF_CPU_INC(snes_perf_cpu_rd_page);
#ifdef RIG_CALL_PROFILE
    g_cpuRead_romhit++;
#endif
#ifdef SNES_ROMPAGE_VERIFY
    {
      uint8_t want = snes_read(snes, adr);
      uint8_t got  = base[adr & 0x1fff];
      if (want != got) {
        static int n = 0;
        if (n++ < 40)
          printf("ROMPAGE MISMATCH adr=%06lx bank=%02x off=%04x fast=%02x slow=%02x "
                 "type=%d romSize=%lu ramSize=%lu lowRom=%d\n",
                 (unsigned long)adr, bank, off, got, want, cart->type,
                 (unsigned long)cart->romSize, (unsigned long)cart->ramSize,
                 (int)cart->bankLowRom[bank]);
      }
    }
#endif
    return base[adr & 0x1fff];
  }
#if SNES_ROMPAGE_LOW
  if(SNES_BANK_LOW_ROM(cart, bank)) {
#ifdef RIG_CALL_PROFILE
    g_cpuRead_romhit++;
#endif
    uint8_t got = (cart->type == 1) ? cart->bankBase[bank & 0x7f][off & 0x7fff]
                                    : cart->bankBase[bank & 0x3f][off];
#ifdef SNES_ROMPAGE_VERIFY
    {
      uint8_t want = snes_read(snes, adr);
      if (want != got) {
        static int nl = 0;
        if (nl++ < 40)
          printf("ROMPAGE LOW MISMATCH adr=%06lx bank=%02x off=%04x fast=%02x "
                 "slow=%02x type=%d ramSize=%lu\n",
                 (unsigned long)adr, bank, off, got, want, cart->type,
                 (unsigned long)cart->ramSize);
      }
    }
#endif
    return got;
  }
#endif
#ifdef RIG_CALL_PROFILE
  g_cpuRead_slow++;
#endif
  SNES_PERF_CPU_INC(snes_perf_cpu_rd_slow);
  return snes_read(snes, adr);
#endif
}

#ifdef SNES_BUS_IN_ITCM
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
void snes_cpuWrite(Snes* snes, uint32_t adr, uint8_t val) {
  snes->cpuMemOps++;   /* charged once per opcode by the caller; see snes_cpuRead */
#ifdef RIG_CALL_PROFILE
  extern uint64_t g_cpuWrite_calls, g_cpuWrite_slow;
  g_cpuWrite_calls++;
#endif
  uint8_t bank = adr >> 16;
  uint16_t off = (uint16_t)adr;
  if(bank == 0x7e || bank == 0x7f) { snes->ram[((bank & 1) << 16) | off] = val; return; }
  if(off < 0x2000 && (bank < 0x40 || (bank >= 0x80 && bank < 0xc0))) { snes->ram[off] = val; return; }
#ifdef RIG_CALL_PROFILE
  g_cpuWrite_slow++;
#endif
#if SNES_SA1
  /* SA-1 cart I-RAM and BW-RAM, as in snes_cpuRead: a write is an event (it
   * may wake the SA-1), so it goes to the SA-1's own write, skipping only the
   * bus decode */
  if (g_sa1_iram) {
    if ((adr & 0x40f800) == 0x003000) { sa1_iram_write(snes->cart->sa1, adr, val); return; }
    if ((adr & 0x40e000) == 0x006000 || (adr & 0xf00000) == 0x400000) {
      sa1_snes_write(snes->cart->sa1, (uint8_t)(adr >> 16), (uint16_t)adr, val);
      return;
    }
  }
#endif
  SNES_PERF_CPU_INC(snes_perf_cpu_wr_slow);
  snes_write(snes, adr, val);
}

