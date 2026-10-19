
#ifndef CART_H
#define CART_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Cart Cart;

#include "snes.h"

typedef struct Dsp1 Dsp1;
typedef struct Cx4 Cx4;
typedef struct Sdd1 Sdd1;
typedef struct Spc7110 Spc7110;
typedef struct Gsu Gsu;
typedef struct Sa1 Sa1;

struct Cart {
  Snes* snes;
  uint8_t type;

  uint8_t* rom;
  uint32_t romSize;
  uint32_t romMask;   /* nonzero only when romSize is a power of 2 (fast path) */
  /* Whether snes_cpuRead's fetch-page cache can serve this cart at all.
   * Decided once at load so the hot path never re-asks; see cart_setRomSize. */
  uint8_t romPageOk;
  /* The DMA ROM-source fast path's run-fixed precondition, folded to one byte:
   * HiROM, fetch-cache-eligible, and no coprocessor intercepting the A-bus.
   * None of these change during a run, so cart_reset sets it once (after any
   * coprocessor is attached) and the per-transfer path adds only the flat-bank
   * check. See dma_doDma. */
  uint8_t romDmaDirect;
  /* Host base of each mapped bank: LoROM indexes it by bank&0x7f in 32 KB
   * steps, HiROM by bank&0x3f in 64 KB. Built once at load so the page install
   * in snes_cpuRead is a table load and an add. */
  uint8_t* bankBase[128];
  /* Per RAW bank: is $0000-$7fff of it ROM? Indexed by the unmasked bank
   * because SRAM decode is written against the unmasked bank. */
  uint8_t bankLowRom[256];
  uint8_t* ram;
  uint32_t ramSize;

  /* DSP-1 coprocessor (Mario Kart / Pilotwings class). NULL for normal carts —
   * every added branch below is behind this test, so plain games cost nothing. */
  Dsp1* dsp1;
  /* Capcom Cx4 (LoROM $00-$3F:$6000-$7FFF). NULL for normal carts. */
  Cx4* cx4;
  /* S-DD1 decompression (HiROM, Star Ocean / SFA2). NULL for normal carts. */
  Sdd1* sdd1;
  /* SPC7110 (compressed data ROM + mapper + optional RTC). */
  Spc7110* spc7110;
  /* Super FX / GSU (Star Fox, Yoshi's Island...). LoROM boards; NULL otherwise. */
  Gsu* gsu;
  /* SA-1 (Mario RPG, Kirby...): its own 65816, MMC and BW-RAM = cart->ram.
   * cart_read/cart_write hand the whole cart region to it. NULL otherwise. */
  Sa1* sa1;
};

// TODO: how to handle reset & load? (especially where to init ram)

Cart* cart_init(Snes* snes);
void cart_setRomSize(Cart* cart, int size);
void cart_attachDsp1(Cart* cart);   /* call after cart_load for DSP carts */
void cart_attachCx4(Cart* cart);    /* call after cart_load for Cx4 carts */
void cart_attachSdd1(Cart* cart);   /* call after cart_load for S-DD1 carts */
void cart_attachSpc7110(Cart* cart);/* call after cart_load for SPC7110 carts */
void cart_attachGsu(Cart* cart);    /* call after cart_load for Super FX carts */
void cart_attachSa1(Cart* cart);    /* call after cart_load for SA-1 carts */
void cart_free(Cart* cart);
void cart_reset(Cart* cart); // will reset special chips etc, general reading is set up in load
void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize); // TODO: figure out how to handle (battery, cart-chips etc)
uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr);
/* Super FX: the Game Pak RAM a DMA from `bank` reads, if cart_gsu_read serves
 * the whole bank from it ($70-$73 and mirrors): byte adr is
 * ram[(*ofs | adr) & *mask]. NULL for any other bank or cart. */
const uint8_t* cart_gsu_dma_ram(Cart* cart, uint8_t bank, uint32_t* ofs, uint32_t* mask);
void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
void cart_saveload(Cart *cart, SaveLoadFunc *func, void *ctx);
#endif
