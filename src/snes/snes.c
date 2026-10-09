
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "../snes_perf.h"
#include <stddef.h>
#include <assert.h>
#include "snes.h"
#include "cpu.h"
#include "apu.h"
#include "dma.h"
#include "ppu.h"
#include "cart.h"
#include "sdd1.h"
#include "spc7110.h"
#include "input.h"
#include "tracing.h"
#include "snes_gnw_alloc.h"

extern bool g_is_turbo;
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
}

void snes_reset(Snes* snes, bool hard) {
  /* Drop the fetch-page cache BEFORE anything reads through snes_cpuRead():
   * cart_reset() may map a different ROM, and cpu_reset() immediately below
   * fetches the reset vector. The tag is always page-aligned inside a 24-bit
   * bus, so ~0 can never match a real address. */
  snes->romPageTag = ~(uint32_t)0;
  snes->romPageBase = NULL;
  cart_reset(snes->cart); // reset cart first, because resetting cpu will read from it (reset vector)
  cpu_reset(snes->cpu);
  apu_reset(snes->apu);
  dma_reset(snes->dma);
  ppu_reset(snes->my_ppu);
  ppu_reset(snes->snes_ppu);
  input_reset(snes->input1);
  input_reset(snes->input2);
  if (hard)
    memset(snes->ram, 0, 0x20000);
  snes->ramAdr = 0;
  snes->hPos = 0;
  snes->vPos = 0;
  snes->frames = 0;
  snes->cpuCyclesLeft = 52; // 5 reads (8) + 2 IntOp (6)
  snes->cpuMemOps = 0;
  snes->apuCatchupCycles = 0.0;
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
  snes->ppuLatch = false;
  snes->multiplyA = 0xff;
  snes->multiplyResult = 0xfe01;
  snes->divideA = 0xffff;
  snes->divideResult = 0x101;
  snes->fastMem = false;
  snes->openBus = 0;
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
      snes->inNmi = true;
#ifdef GNW_SNES_CORE
      /* Only if the game asked for it. Super Metroid always has NMI on, so its
       * reimplementation could take the shortcut of firing unconditionally — but
       * an arbitrary game gets an NMI before it has set up its vector, and dies.
       * (This alone was 100 of 125 games booting to a black screen.) */
      if (snes->nmiEnabled)
        snes->cpu->nmiWanted = true;
#else
      snes->cpu->nmiWanted = true; // request NMI on CPU
#endif
      if (snes->autoJoyRead) {
        // TODO: this starts a little after start of vblank
        snes->autoJoyTimer = 0;
        /* After 16 hardware clocks the leftover serial bits are 1. */
        snes->input1->latchedState = 0xffff;
        snes->input2->latchedState = 0xffff;
      }
      if (snes->cart && snes->cart->spc7110)
        spc7110_tick(snes->cart->spc7110);
    }
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
    if (!snes->inVblank)
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
    }
  }
#endif
  // handle autoJoyRead-timer
  //if (snes->autoJoyTimer > 0) snes->autoJoyTimer -= 2;
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
    snes->inNmi = true;
    if (snes->nmiEnabled)
      snes->cpu->nmiWanted = true;
    if (snes->autoJoyRead) {
      snes->autoJoyTimer = 0;
      snes->input1->latchedState = 0xffff;
      snes->input2->latchedState = 0xffff;
    }
    if (snes->cart && snes->cart->spc7110)
      spc7110_tick(snes->cart->spc7110);
  }

  /* The V-timer matches on every dot of its line; setting the flag once is the
   * same thing to everyone who reads it. */
  if (snes->vIrqEnabled && snes->vPos == snes->vTimer) {
    snes->inIrq = true;
    snes->cpu->irqWanted = true;
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

/* NTSC: (32040*32)/(1364*262*60). PAL: same APU clock over 312 lines at 50 Hz. */
#define APU_CYCLES_PER_MASTER_NTSC ((32040.0 * 32.0) / (1364.0 * 262.0 * 60.0))
#define APU_CYCLES_PER_MASTER_PAL  ((32040.0 * 32.0) / (1364.0 * 312.0 * 50.0))

void snes_set_region(Snes *snes, bool pal) {
  snes->pal = pal;
  snes->vcount = pal ? 312 : 262;
  if (snes->apu && snes->apu->dsp)
    snes->apu->dsp->frameSamples = pal ? DSP_SAMPLES_PAL : DSP_SAMPLES_NTSC;
}

void snes_catchupApu(Snes* snes) {
  if (snes->apu == NULL)
    return;
  /* Flush accumulated integer dots to double. By distributivity this is
   * mathematically identical to per-dot accumulation; the <1 ULP rounding
   * difference is far below the integer truncation below. snes_run_line (sm
   * core) still accumulates apuCatchupCycles directly — its apuDotsAccum is
   * always 0, so this is a no-op for sm. */
  if (snes->apuDotsAccum) {
    double ratio = snes->pal ? APU_CYCLES_PER_MASTER_PAL : APU_CYCLES_PER_MASTER_NTSC;
    snes->apuCatchupCycles += (double)snes->apuDotsAccum * ratio;
    snes->apuDotsAccum = 0;
  }
  if (snes->apuCatchupCycles > 10000)
    snes->apuCatchupCycles = 10000;

  int catchupCycles = (int) snes->apuCatchupCycles;

  apu_run(snes->apu, catchupCycles);
  snes->apuCatchupCycles -= (double) catchupCycles;
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
      val |= snes->inNmi << 7;
      /* RDNMI bit 7 clears on read (and at end of vblank). */
      snes->inNmi = false;
      return val | (snes->openBus & 0x70);
    }
    case 0x4211: {
      uint8_t val = snes->inIrq << 7;
      snes->inIrq = false;
      snes->cpu->irqWanted = false;
      return val | (snes->openBus & 0x7f);
    }
    case 0x4212: {
#ifdef HOST_BUILD
      if (host_boot_mmio_trace_on())
        printf("bootmmio-r: 4212 vblank=%d hblank=%d @%02x:%04x\n",
               (int)snes->inVblank, (int)(snes->hPos >= 1024),
               (unsigned)snes->cpu->k, (unsigned)snes->cpu->pc);
#endif
      uint8_t val = (snes->autoJoyTimer > 0);
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
      snes->nmiEnabled = val & 0x80;
      if(!snes->hIrqEnabled && !snes->vIrqEnabled) {
        snes->inIrq = false;
        snes->cpu->irqWanted = false;
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
    case 0x4207: {
      snes->hTimer = (snes->hTimer & 0x100) | val;
      break;
    }
    case 0x4208: {
      snes->hTimer = (snes->hTimer & 0x0ff) | ((val & 1) << 8);
      break;
    }
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
      dma_startDma(snes->dma, val, false);
      {
        SNES_PERF_DMA_BEGIN(perf_t, perf_a);
        while (dma_cycle(snes->dma)) {}
        SNES_PERF_DMA_END(perf_t, perf_a);
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
    if(adr >= 0x4200 && adr < 0x4220 || adr >= 0x4218 && adr < 0x4220) {
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
  if(bank == 0x7e || bank == 0x7f) {
    uint32_t addr = ((bank & 1) << 16) | adr;
    snes->ram[addr] = val; // ram
    if (IS_ADR(addr)) {
      LogWrite(snes, adr, val);
    }
  }
  if(bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if(adr < 0x2000) {
      snes->ram[adr] = val; // ram mirror
      if (IS_ADR(adr)) {
        LogWrite(snes, adr, val);
      }
    }
    if(adr >= 0x2100 && adr < 0x2200) {
      if (adr == 0x2100) HOST_BOOT_MMIO_LOG(adr, val);
      snes_writeBBus(snes, adr & 0xff, val); // B-bus
    }
    if(adr == 0x4016) {
      input_latch(snes->input1, val & 1); // strobe both controller ports
      input_latch(snes->input2, val & 1);
    }
    if(adr >= 0x4200 && adr < 0x4220) {
      snes_writeReg(snes, adr, val); // internal registers
    }
    if(adr >= 0x4300 && adr < 0x4380) {
      dma_write(snes->dma, adr, val); // dma registers
      if(snes->cart->sdd1)
        sdd1_mmio_write(snes->cart->sdd1, adr, val);
    }
    if(snes->cart->sdd1 && adr >= 0x4800 && adr < 0x4808) {
      sdd1_mmio_write(snes->cart->sdd1, adr, val);
    }
    if(snes->cart->spc7110 && adr >= 0x4800 && adr <= 0x4842) {
      spc7110_mmio_write(snes->cart->spc7110, adr, val, snes->cart->rom, snes->cart->romSize);
    }
  }
  // write to cart
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
   * Only the ROM branch ever installs a tag, so a tag match implies ROM:
   * WRAM, SRAM and MMIO all live below 0x8000 (or in banks 7e/7f) and keep
   * the slow path, which is also why no write path has to invalidate this --
   * ROM does not change under us. An 8 KB page cannot straddle a LoROM 32 KB
   * mapping boundary, and cannot straddle a romMask wrap either (romMask is
   * 2^n-1 with n >= 13 for any real cart), so base+offset stays linear for
   * the whole page. The tag holds the page-aligned address, so the sentinel
   * below (low bits set, and beyond the 24-bit bus) can never collide. */
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
  SNES_PERF_CPU_INC(snes_perf_cpu_wr_slow);
  snes_write(snes, adr, val);
}

