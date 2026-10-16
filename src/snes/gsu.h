#ifndef GSU_H
#define GSU_H

#include <stdint.h>
#include <stdbool.h>

/* Super FX (GSU) coprocessor — Phase 1 proof of concept.
 *
 * Unlike our other coprocessors this is a full LLE interpreter; rather than
 * re-derive it, we run the proven ZSNES 2 GSU core verbatim (see
 * gsu_zsnes/gsu_core.c) behind this thin, struct-based API. The ZSNES core is
 * a single global instance, so `Gsu` is just a handle that holds the things
 * our tree owns (a back-pointer for IRQs). The game RAM
 * is cart->ram, shared with the 65816.
 *
 * Save states: NOT supported for Super FX carts (the GSU state lives in the
 * ZSNES core's globals, outside our serialized Cart) — gsu_size() returns 0.
 * This is a deliberate Phase-1 trade; see docs / memory.
 *
 * CPU-visible map: see cart.c (cart_gsu_read) and snes.c (MMIO $3000-$32FF). */

typedef struct Snes Snes;

typedef struct Gsu {
  uint32_t version;
  uint8_t  built;          /* tables initialised */
  Snes    *snes;           /* for raising the 65816 IRQ on STOP */
} Gsu;

Gsu     *gsu_alloc(void);
uint32_t gsu_size(void);                 /* 0: FX carts don't save state */
void     gsu_reset(Gsu *g);
void     gsu_link(Gsu *g, Snes *snes, uint8_t *rom, uint32_t rom_size,
                  uint8_t *ram, uint32_t ram_size);

/* CPU-side MMIO: registers $3000-$303F and cache $3100-$32FF */
uint8_t  gsu_mmio_read(Gsu *g, uint16_t addr);
void     gsu_mmio_write(Gsu *g, uint16_t addr, uint8_t val);

/* Whether the CPU is locked out of the shared ROM/RAM (PoC: never) */
bool     gsu_cpu_ram_blocked(Gsu *g);
bool     gsu_cpu_rom_blocked(Gsu *g);

/* the GSU status word ($3030/$3031) without the read's side effect */
uint16_t gsu_sfr_peek(void);

/* scheduling hooks from snes.c */
void     gsu_runLine(Gsu *g);            /* one scanline's slice */
void     gsu_vblank(Gsu *g);             /* finish the frame before render */

#endif /* GSU_H */
