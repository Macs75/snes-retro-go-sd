/* Super FX (GSU) glue — Phase 1. Drives the verbatim ZSNES core in
 * gsu_zsnes/gsu_core.c: owns the plot-LUT buffer, points the core at our ROM
 * (cart->rom) and shared Game Pak RAM (cart->ram), routes CPU MMIO, steps the
 * GSU per scanline / vblank, and relays the GSU's STOP IRQ to the 65816. */

#include <stdlib.h>
#include <string.h>
#include "gsu.h"
#include "snes.h"
#include "cpu.h"
#include "gsu_zsnes/gsu_zsnes_core.h"

#ifdef GNW_SNES_CORE
#include "gw_malloc.h"
#endif
#include "snes_gnw_alloc.h"
#include "../snes_perf.h"
#include "idle_skip.h"

#define GSU_VERSION 1

Gsu *gsu_alloc(void) {
  Gsu *g = (Gsu *)snes_zalloc(sizeof(Gsu));
  if (!g) return NULL;
  g->version = GSU_VERSION;
  return g;
}

void gsu_saveload(Gsu *g, SaveLoadFunc *func, void *ctx) {
  (void)g;
  gsu_zsnes_saveload(func, ctx);
}

void gsu_link(Gsu *g, Snes *snes, uint8_t *rom, uint32_t rom_size,
              uint8_t *ram, uint32_t ram_size) {
  g->snes = snes;
  romdata     = rom;          /* read in place: fill_memtable maps the banks */
  romdatasize = rom_size;
  sfxgameram = ram;
  SfxRamMask = ram_size ? ram_size - 1 : 0;   /* a power of two: 32 or 64 KB */
  romispal   = 0;                  /* TODO: wire region; budget diff is minor */
  SuperFXClockMultiplier = 100;
#ifdef HOST_BUILD
  /* host-only throughput knob for timing experiments (50..800, 100 = default) */
  { const char *cm = getenv("GSU_CLOCK"); if (cm) SuperFXClockMultiplier = (uint32_t)atoi(cm); }
#endif
  if (!g->built) {
    gsu_zsnes_build_tables();
    g->built = 1;
  }
  gsu_zsnes_reset_state();
}

void gsu_reset(Gsu *g) {
  (void)g;
  gsu_zsnes_reset_state();
}

/* Relay a GSU STOP interrupt to the 65816. */
/* after a GSU slice: point the idle skip at the loop the CPU is in */
static inline void gsu_idle_select(Gsu *g) {
#ifdef SNES_IDLE_SKIP
  if (g->snes) idle_line_select(g->snes);
#else
  (void)g;
#endif
}

static void gsu_service_irq(Gsu *g) {
  if (doirqnext) {
    doirqnext = 0;
    if (g->snes && g->snes->cpu) {
      g->snes->cpu->irqWanted = true;
#ifdef SNES_IDLE_SKIP
      idle_irq_sample(g->snes);   /* where the CPU waits for the GSU to stop */
#endif
    }
  }
}

uint8_t gsu_mmio_read(Gsu *g, uint16_t addr) {
#ifdef SNES_PERF
  snes_perf_gsu_reads++;
#endif
#ifdef SNES_IDLE_SKIP
  /* the CPU polls the status while the GSU works: offer the loop to the idle
   * skip (a no-op when it is already the active site) */
  if (addr == 0x3030 && (SfxSFR & 0x20) && g->snes) idle_gsu_poll_sample(g->snes);
#endif
  return gsu_zsnes_reg_read(addr);
}

void gsu_mmio_write(Gsu *g, uint16_t addr, uint8_t val) {
  SNES_PERF_BEGIN(perf_t);
  gsu_zsnes_reg_write(addr, val);   /* writing R15 hi runs the first slice */
  SNES_PERF_END(snes_perf_gsu, perf_t);
  gsu_service_irq(g);
}

uint16_t gsu_sfr_peek(void) { return gsu_zsnes_sfr_peek(); }

bool gsu_cpu_ram_blocked(Gsu *g) { (void)g; return false; }
bool gsu_cpu_rom_blocked(Gsu *g) { (void)g; return false; }

void gsu_runLine(Gsu *g) {
  SNES_PERF_BEGIN(perf_t);
  gsu_zsnes_startline();
  SNES_PERF_END(snes_perf_gsu, perf_t);
  gsu_idle_select(g);
  gsu_service_irq(g);
}

#ifdef HOST_BUILD
long gsu_host_frame;   /* emulated frames, for the host-only GSU tools in fx_ops.h */
void gsu_state_frame(long frame);
#endif
void gsu_vblank(Gsu *g) {
#ifdef HOST_BUILD
  gsu_state_frame(gsu_host_frame);
  gsu_host_frame++;
#endif
  SNES_PERF_BEGIN(perf_t);
  gsu_zsnes_vblank();
  SNES_PERF_END(snes_perf_gsu, perf_t);
  gsu_idle_select(g);
  gsu_service_irq(g);
}

