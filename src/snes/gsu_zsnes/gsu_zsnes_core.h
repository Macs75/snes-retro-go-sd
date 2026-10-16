#ifndef GSU_ZSNES_CORE_H
#define GSU_ZSNES_CORE_H

/* Boundary between the verbatim ZSNES GSU core (gsu_core.c) and the glue
 * (gsu.c). The glue sets the three memory pointers + region, builds the
 * tables once, then drives the core per scanline / vblank and routes MMIO. */

#include <stdint.h>

/* memory the core operates on — set by the glue before gsu_zsnes_build_tables */
extern uint8_t *romdata;      /* = cart->rom */
extern uint32_t romdatasize;
extern uint32_t SfxRamMask;
uint16_t gsu_zsnes_sfr_peek(void);
extern uint8_t *sfxgameram;   /* = cart->ram (shared Game Pak RAM) */
extern uint8_t  romispal;     /* 0 = NTSC, 1 = PAL */
extern uint32_t SuperFXClockMultiplier;  /* 100 = stock GSU speed */

/* the 65816 IRQ request the GSU raises on STOP (polled by the glue) */
extern uint8_t  doirqnext;

/* status/control the glue peeks for bring-up diagnostics */
extern uint32_t SfxSFR, SfxSCMR, SfxPBR, SFXProc;

void    gsu_zsnes_build_tables(void);
void    gsu_zsnes_reset_state(void);
uint8_t gsu_zsnes_reg_read(uint16_t addr);
void    gsu_zsnes_reg_write(uint16_t addr, uint8_t val);
void    gsu_zsnes_startline(void);
void    gsu_zsnes_vblank(void);

#endif
