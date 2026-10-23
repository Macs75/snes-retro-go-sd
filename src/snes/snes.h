
#ifndef SNES_H
#define SNES_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Snes Snes;

#include "cpu.h"
#include "apu.h"
#include "dma.h"
#include "ppu.h"
#include "cart.h"
#include "input.h"
#include "saveload.h"

/* A LoROM DSP-1 board decodes the chip at banks $30-$3f (mirrored at $b0-$bf),
 * $8000-$ffff. snes_cpuRead's ROM fast path and its fetch-page cache claim
 * everything at $8000 and above, so that window has to come out of them.
 * SNES_DSP_FASTPATH=1 takes it out with one test at page-install time. */
#ifndef SNES_DSP_FASTPATH
#define SNES_DSP_FASTPATH 1
#endif
#ifndef SNES_LINE_HIRQ
#define SNES_LINE_HIRQ 1
#endif
/* Fetch-page cache for carts whose size is not a power of two (3 MB, etc.). */
#ifndef SNES_ROMPAGE_FOLD
#define SNES_ROMPAGE_FOLD 1
#endif
#if SNES_ROMPAGE_FOLD
#define SNES_ROM_PAGE_OK(cart) ((cart)->romPageOk)
#else
#define SNES_ROM_PAGE_OK(cart) ((cart)->romMask)
#endif
/* OFF: serving ROM below $8000 from ITCM costs LoROM carts that never collect.
 * =2 would put the same serve in snes_read(); still taxes uncacheable dumps. */
#ifndef SNES_ROMPAGE_LOW
#define SNES_ROMPAGE_LOW 0
#endif
#if SNES_ROMPAGE_LOW == 1 && SNES_ROMPAGE_FOLD
#define SNES_BANK_LOW_ROM(cart, bank) ((cart)->bankLowRom[(bank)])
#else
#define SNES_BANK_LOW_ROM(cart, bank) 0
#endif
#if SNES_DSP_FASTPATH
#define SNES_DSP_LOROM_WINDOW(cart, bank) \
  ((cart)->dsp1 && (cart)->type == 1 && (uint8_t)(((bank) & 0x7f) - 0x30) < 0x10u)
#else
#define SNES_DSP_LOROM_WINDOW(cart, bank) 0
#endif

struct Snes {
  Cpu* cpu;
  Apu* apu;
  Ppu* ppu, *snes_ppu, *my_ppu;
  Dma* dma;
  Cart* cart;
  Input *input1;
  Input *input2;
  // input
  bool debug_cycles;
  bool debug_apu_cycles;
  bool disableRender;
  uint8_t runningWhichVersion;

  // ram
  uint32_t ramAdr;
  uint8_t *ram;
  uint8_t padx[4];

  // frame timing
  uint16_t hPos;
  uint16_t vPos;
  uint32_t frames;
  // cpu handling
  uint8_t cpuCyclesLeft;
  uint8_t cpuMemOps;
  uint8_t padpad[2];
  uint64_t apuCycleAcc;    /* owed APU cycles, Q32 fixed-point (see snes_catchupApu) */
  uint32_t apuDotsAccum;   /* integer master-dot accumulator since the last catch-up */
  // nmi / irq
  bool hIrqEnabled;
  bool vIrqEnabled;
  bool nmiEnabled;
  uint16_t hTimer;
  uint16_t vTimer;
  bool inNmi;
  bool inIrq;
  bool inVblank;
  // joypad handling
  uint16_t portAutoReadX[4]; // as read by auto-joypad read
  bool autoJoyRead;
  uint16_t autoJoyTimer; // line the auto-joypad read started on, 0 = none
  bool ppuLatch;
  // multiplication/division
  uint8_t multiplyA;
  uint16_t multiplyResult;
  uint16_t divideA;
  uint16_t divideResult;
  // misc
  bool fastMem;
  uint8_t openBus;
  /* ROM fetch-page cache for snes_cpuRead(). Deliberately declared AFTER
   * openBus: snes_saveload() serializes the byte range hPos..openBus, so
   * anything past it stays out of the savestate -- which is what we want,
   * these are a derived host pointer and its tag, not emulated state, and a
   * savestate must never carry a host address across a load. */
  const uint8_t *romPageBase;
  uint32_t romPageTag;
  /* PAL (header $ffd9): 312 lines / 50 Hz. After romPageTag so Thumb-2
   * SNES_ROMPAGE* offsets stay valid. Not in the savestate range. */
  bool pal;
  uint16_t vcount; /* 262 NTSC, 312 PAL */
  /* $4210 was read in the last dots before vblank, inside an instruction
   * whose read cycle falls after vblank starts (see snes_readReg): it saw
   * the NMI flag set and cleared it, so the vblank event must not set it
   * again. Lives a few dots at most; not in the savestate range. */
  bool nmiReadEarly;
};

Snes* snes_init(uint8_t *ram);
void snes_free(Snes* snes);
void snes_reset(Snes* snes, bool hard);
void snes_set_region(Snes* snes, bool pal);
void snes_runFrame(Snes* snes);
// used by dma, cpu
uint8_t snes_readBBus(Snes* snes, uint8_t adr);
void snes_writeBBus(Snes* snes, uint8_t adr, uint8_t val);
uint8_t snes_read(Snes* snes, uint32_t adr);
uint8_t snes_hvbjoy_at(const Snes* snes, uint16_t h);   /* $4212 at dot h, no side effect */
void snes_write(Snes* snes, uint32_t adr, uint8_t val);
uint8_t snes_cpuRead(Snes* snes, uint32_t adr);
#ifndef SNES_PAGEMAP
#define SNES_PAGEMAP 1
#endif
#if SNES_PAGEMAP
/* the 65816 page maps (snes.c): 8 KB pages, host pointer or NULL = slow path */
extern uint8_t *g_snes_rdmap[0x800];   /* WRAM + cacheable ROM */
extern uint8_t *g_snes_wrmap[0x800];   /* WRAM only */
void snes_pagemap_refresh(Snes *snes);   /* after a cartridge mapping change */
extern uint8_t *g_sa1_iram;          /* SA-1 cart I-RAM, else NULL (snes_cpuRead) */
extern const uint16_t *g_sa1_vh;     /* the SA-1's last catch-up: line, dot */
extern const uint8_t *g_sa1_bwwin;   /* SA-1 cart: the 65816's BW-RAM window block */
extern const uint8_t *g_sa1_bwlin;   /* SA-1 cart: BW-RAM banks $40-$4F, NULL in CC1 */
extern uint32_t g_sa1_bwmask;
extern const uint8_t *g_sa1_idle;    /* SA-1 cart: asleep in a wait loop 1, a lap counter 2 */
extern uint8_t *const *g_sa1_ctr;    /* SA-1 cart: that counter's bytes (lo, hi or NULL) */
/* A catch-up of the SA-1 that nothing observes -- the 65816's run loop at an
 * event, the line catch-ups -- can be left for later while it sits in a lap
 * counter (sa1.c's sa1_ctr_sleep): there it changes nothing but the counter
 * and raises nothing. The time stays owed in lastV / lastH, not in its debt,
 * so the next sa1_sync runs it all in one go, with the same clocks and laps
 * as in steps; and every contact that can see the counter makes one: a 65816
 * read of its bytes (snes_cpuRead), a register or DMA read, sa1_event before
 * a wake. Up to 128 lines, so sa1_sync's line count never wraps a frame.
 * Synced at each event instead, Parodius spent ~1,100 catch-ups a frame
 * folding laps no one read. */
static inline bool sa1_time_owed(const Snes *snes) {
  if (*g_sa1_idle != 2) return false;
  int lines = (int)snes->vPos - g_sa1_vh[0];
  if (lines < 0) lines += snes->pal ? 312 : 262;
  return lines < 128;
}
#endif
void snes_cpuWrite(Snes* snes, uint32_t adr, uint8_t val);
// debugging
void snes_debugCycle(Snes* snes, bool* cpuNext, bool* spcNext);

void snes_handle_pos_stuff(Snes *snes);

// snes_other.c functions:

bool snes_loadRom(Snes* snes, const uint8_t* data, int length);
/* The coprocessor snes_loadRom attaches for this image (same header pick). */
enum { SNES_COPRO_NONE, SNES_COPRO_DSP1, SNES_COPRO_CX4, SNES_COPRO_SDD1,
       SNES_COPRO_SPC7110, SNES_COPRO_GSU, SNES_COPRO_SA1 };
int snes_rom_coprocessor(const uint8_t* data, int length);
/* Whether the code for coprocessor `kind` is present (device: its overlay is
   loaded). cart_attach* refuse the chip otherwise. Weak default: true. */
bool snes_copro_overlay_ready(int kind);
void snes_setPixels(Snes* snes, uint8_t* pixelData);
void snes_setSamples(Snes* snes, int16_t* sampleData, int samplesPerFrame);
void snes_saveload(Snes *snes, SaveLoadFunc *func, void *ctx);
/* One scanline of dot-clock events in one call — see snes.c. */
void snes_run_line(Snes *snes);
uint8_t snes_readBBusOrg(Snes *snes, uint8_t adr);
void snes_catchupApu(Snes *snes);

extern int snes_frame_counter;
#endif
