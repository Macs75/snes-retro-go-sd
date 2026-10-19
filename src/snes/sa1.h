#ifndef SA1_H
#define SA1_H

/* SA-1 (Super Mario RPG, Kirby Super Star, Kirby's Dream Land 3...): a second
 * 65816 at 10.74 MHz with its own I-RAM, the cartridge's BW-RAM, a ROM mapper,
 * DMA with character conversion, a multiplier/divider and a bit-stream reader.
 *
 * The SA-1's CPU is our own 65816 (cpu.c compiled again as sa1_cpu.c, with
 * this file's bus). The chip logic -- registers, arithmetic, DMA, character
 * conversion, the variable-length reader -- follows ZSNES 2 (xyproto C
 * rewrite: chips/sa1regs.c, c_sa1regs.c, sa1emu.c, cpu/mem_ops.h), with
 * ZSNES's names kept for the state so the two can be read side by side.
 * Interrupts are level lines as on the chip; ZSNES takes them as one-shot
 * events from its own scheduler. */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "saveload.h"

typedef struct Sa1 Sa1;
typedef struct Snes Snes;
typedef struct Cpu Cpu;

Sa1* sa1_alloc(void);
void sa1_link(Sa1* s, Snes* snes, uint8_t* rom, uint32_t romSize, uint8_t* bwram, uint32_t bwramSize);
void sa1_reset(Sa1* s);
void sa1_saveload(Sa1* s, SaveLoadFunc* func, void* ctx);

/* The 65816's view: everything cart_read/cart_write get on an SA-1 cart. */
uint8_t sa1_snes_read(Sa1* s, uint8_t bank, uint16_t adr);
void sa1_snes_write(Sa1* s, uint8_t bank, uint16_t adr, uint8_t val);

/* The SA-1 CPU's bus (cpu.c under CPU_SA1). */
uint8_t sa1_cpu_read(Sa1* s, uint32_t adr);
void sa1_cpu_write(Sa1* s, uint32_t adr, uint8_t val);

/* Run the SA-1 for the master clocks the 65816 side has advanced. */
void sa1_run(Sa1* s, uint32_t masterClocks);
/* Run the SA-1 up to line v, master clock h (from where it last stopped). */
void sa1_sync(Sa1* s, uint16_t v, uint16_t h);

#ifdef HOST_BUILD
void sa1_statelog(Sa1* s, unsigned long frame);   /* SA1_STATELOG=file */
#endif

/* For the 65816's idle skip (idle_skip.c): the SA-1 sleeps in a proven wait
 * loop, so nothing but a 65816 event changes I-RAM, BW-RAM or its registers;
 * and a 65816 read of I-RAM / BW-RAM / ROM without any side effect (0 for
 * anything else). */
bool sa1_asleep(const Sa1* s);
/* The 65816's page map entry for SA-1 cart space (snes.c snes_pagemap_base):
 * a ROM or plain BW-RAM page as a host pointer, NULL for the slow path. */
uint8_t* sa1_snes_page(Sa1* s, uint8_t bank, uint16_t adr);
/* The SA-1 CPU's read map (cpu.c under CPU_SA1): 8 KB pages, NULL = slow path */
extern uint8_t** g_sa1_rdmap;
uint8_t sa1_peek(Sa1* s, uint8_t bank, uint16_t adr);

/* The SA-1's request on the 65816's IRQ line ($2201/$2300 bits 7 and 5). */
bool sa1_snes_irq(const Sa1* s);

#endif
