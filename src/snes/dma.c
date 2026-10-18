
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

#include "dma.h"
#include "snes.h"
#include "cart.h"
#include "snes_gnw_alloc.h"

static const int bAdrOffsets[8][4] = {
  {0, 0, 0, 0},
  {0, 1, 0, 1},
  {0, 0, 0, 0},
  {0, 0, 1, 1},
  {0, 1, 2, 3},
  {0, 1, 0, 1},
  {0, 0, 0, 0},
  {0, 0, 1, 1}
};

static const int transferLength[8] = {
  1, 2, 2, 4, 4, 4, 2, 4
};

static void dma_transferByte(Dma* dma, uint16_t aAdr, uint8_t aBank, uint8_t bAdr, bool fromB);

#if defined(HOST_BUILD) && !defined(QEMU_BUILD)   /* not in the 4 MB QEMU harness */
#define DMA_HIST 1
/* DMA_HIST=file (host): what the DMA moves. Per general-DMA kind (direction,
 * source bank, B-bus target, mode, the read path dma_doDma takes) the
 * transfers and bytes, and per HDMA channel target the bytes; written at exit
 * with the frames seen (tools/dma_hist_report.py). */
extern uint32_t apu_hist_frame;   /* counted by main_snes.c at each frame end */
uint32_t snes_fs_dma;             /* FRAME_STATS (main_snes.c): general-DMA bytes this frame */
#define DMA_H_N 256
static struct { uint32_t key; unsigned long n, bytes; } dma_h[DMA_H_N];
static int dma_h_on = -1;
static uint32_t dma_h_f0 = 0xffffffffu, dma_h_f1;
static unsigned long dma_h_cycles, dma_h_cycles_hdma, dma_h_cycles_dma, dma_h_doDma;   /* dma_cycle calls */
static void dma_h_write(void) {
  FILE *f = fopen(getenv("DMA_HIST"), "w");
  if (!f) return;
  fprintf(f, "# frames %u..%u (%u)\n", dma_h_f0, dma_h_f1, dma_h_f1 - dma_h_f0 + 1);
  fprintf(f, "# kind bank badr mode path transfers bytes vmain\n");
  fprintf(f, "# dma_cycle calls %lu: hdma stall %lu, dma busy %lu (dma_doDma %lu)\n",
          dma_h_cycles, dma_h_cycles_hdma, dma_h_cycles_dma, dma_h_doDma);
  for (int i = 0; i < DMA_H_N; i++) {
    uint32_t k = dma_h[i].key;
    if (!dma_h[i].n) continue;
    fprintf(f, "%s %02x %02x %u %u %lu %lu %02x\n", (k >> 31) ? "hdma" : "dma", (k >> 16) & 0xff, (k >> 8) & 0xff,
            (k >> 4) & 7, k & 0xf, dma_h[i].n, dma_h[i].bytes, (k >> 24) & 0x3f);
  }
  fclose(f);
}
/* VRAM transfers ($2118/$2119 targets): VMAIN at the start, the low 6 bits of
 * the key's top byte -- bit 0 increment after the high byte, bits 1-2 remap
 * mode, bits 3-4 increment (1, 32, 128), bit 5 the transfer starts on an odd
 * offset index. 0 for every other target. */
static unsigned dma_h_vmain(const Snes *sn, uint8_t badr, unsigned oi) {
  if (badr != 0x18 && badr != 0x19) return 0;
  const Ppu *p = sn->ppu;
  unsigned inc = p->vramIncrement == 1 ? 0 : p->vramIncrement == 32 ? 1 : p->vramIncrement == 128 ? 2 : 3;
  return (p->vramIncrementOnHigh ? 1u : 0u) | ((p->vramRemapMode & 3u) << 1) | (inc << 3) | ((oi & 1u) << 5);
}
/* hdma: 1/0; path: 0 WRAM, 1 HiROM base, 2 Game Pak RAM, 3 snes_read, 4 B->A, 5 page map */
static void dma_h_add(int hdma, uint8_t bank, uint8_t badr, uint8_t mode, int path, unsigned long bytes, unsigned vm) {
  if (dma_h_on < 0) { dma_h_on = getenv("DMA_HIST") != NULL; if (dma_h_on) atexit(dma_h_write); }
  if (!dma_h_on) return;
  if (dma_h_f0 == 0xffffffffu) dma_h_f0 = apu_hist_frame;
  dma_h_f1 = apu_hist_frame;
  uint32_t key = ((uint32_t)hdma << 31) | ((uint32_t)(vm & 0x3f) << 24) | ((uint32_t)bank << 16) | ((uint32_t)badr << 8) | ((uint32_t)(mode & 7) << 4) | (uint32_t)path;
  uint32_t h = (key * 2654435761u) >> 24;
  for (;; h = (h + 1) & (DMA_H_N - 1))
    if (!dma_h[h].n || dma_h[h].key == key) break;
  dma_h[h].key = key; dma_h[h].n++; dma_h[h].bytes += bytes;
}
#endif

Dma* dma_init(Snes* snes) {
  Dma* dma = snes_zalloc(sizeof(Dma));
  if (!dma) return NULL;
  dma->snes = snes;
  return dma;
}

void dma_free(Dma* dma) {
  snes_zfree(dma);
}

void dma_reset(Dma* dma) {
  for(int i = 0; i < 8; i++) {
    dma->channel[i].bAdr = 0xff;
    dma->channel[i].aAdr = 0xffff;
    dma->channel[i].aBank = 0xff;
    dma->channel[i].size = 0xffff;
    dma->channel[i].indBank = 0xff;
    dma->channel[i].tableAdr = 0xffff;
    dma->channel[i].repCount = 0xff;
    dma->channel[i].unusedByte = 0xff;
    dma->channel[i].dmaActive = false;
    dma->channel[i].hdmaActive = false;
    dma->channel[i].mode = 7;
    dma->channel[i].fixed = true;
    dma->channel[i].decrement = true;
    dma->channel[i].indirect = true;
    dma->channel[i].fromB = true;
    dma->channel[i].unusedBit = true;
    dma->channel[i].doTransfer = false;
    dma->channel[i].terminated = false;
    dma->channel[i].offIndex = 0;
  }
  dma->hdmaTimer = 0;
  dma->dmaTimer = 0;
  dma->dmaBusy = false;
  dma->hdmaEnabled = 0;
}

void dma_saveload(Dma *dma, SaveLoadFunc *func, void *ctx) {
  func(ctx, &dma->channel, offsetof(Dma, pad) + 7 - offsetof(Dma, channel));
}

uint8_t dma_read(Dma* dma, uint16_t adr) {
  uint8_t c = (adr & 0x70) >> 4;
  switch(adr & 0xf) {
    case 0x0: {
      uint8_t val = dma->channel[c].mode;
      val |= dma->channel[c].fixed << 3;
      val |= dma->channel[c].decrement << 4;
      val |= dma->channel[c].unusedBit << 5;
      val |= dma->channel[c].indirect << 6;
      val |= dma->channel[c].fromB << 7;
      return val;
    }
    case 0x1: {
      return dma->channel[c].bAdr;
    }
    case 0x2: {
      return dma->channel[c].aAdr & 0xff;
    }
    case 0x3: {
      return dma->channel[c].aAdr >> 8;
    }
    case 0x4: {
      return dma->channel[c].aBank;
    }
    case 0x5: {
      return dma->channel[c].size & 0xff;
    }
    case 0x6: {
      return dma->channel[c].size >> 8;
    }
    case 0x7: {
      return dma->channel[c].indBank;
    }
    case 0x8: {
      return dma->channel[c].tableAdr & 0xff;
    }
    case 0x9: {
      return dma->channel[c].tableAdr >> 8;
    }
    case 0xa: {
      return dma->channel[c].repCount;
    }
    case 0xb:
    case 0xf: {
      return dma->channel[c].unusedByte;
    }
    default: {
#ifndef GNW_SNES_CORE
      /* dev builds only: $43xC-$43xE are unmapped on hardware (open bus);
       * stray reads there are normal for commercial games. */
      assert(0);
#endif
      return dma->snes->openBus;
    }
  }
}

void dma_write(Dma* dma, uint16_t adr, uint8_t val) {
  uint8_t c = (adr & 0x70) >> 4;
  switch(adr & 0xf) {
    case 0x0: {
      dma->channel[c].mode = val & 0x7;
      dma->channel[c].fixed = val & 0x8;
      dma->channel[c].decrement = val & 0x10;
      dma->channel[c].unusedBit = val & 0x20;
      dma->channel[c].indirect = val & 0x40;
      dma->channel[c].fromB = val & 0x80;
      break;
    }
    case 0x1: {
      dma->channel[c].bAdr = val;
      break;
    }
    case 0x2: {
      dma->channel[c].aAdr = (dma->channel[c].aAdr & 0xff00) | val;
      break;
    }
    case 0x3: {
      dma->channel[c].aAdr = (dma->channel[c].aAdr & 0xff) | (val << 8);
      break;
    }
    case 0x4: {
      dma->channel[c].aBank = val;
      break;
    }
    case 0x5: {
      dma->channel[c].size = (dma->channel[c].size & 0xff00) | val;
      break;
    }
    case 0x6: {
      dma->channel[c].size = (dma->channel[c].size & 0xff) | (val << 8);
      break;
    }
    case 0x7: {
      dma->channel[c].indBank = val;
      break;
    }
    case 0x8: {
      dma->channel[c].tableAdr = (dma->channel[c].tableAdr & 0xff00) | val;
      break;
    }
    case 0x9: {
      dma->channel[c].tableAdr = (dma->channel[c].tableAdr & 0xff) | (val << 8);
      break;
    }
    case 0xa: {
      dma->channel[c].repCount = val;
      break;
    }
    case 0xb:
    case 0xf: {
      dma->channel[c].unusedByte = val;
      break;
    }
    default: {
      break;
    }
  }
}

extern bool g_fail;

/* A->B into the PPU's data ports (ppu_dma_ports) from a source without side
 * effects: a flat array (src[(ofs | adr) & mask]) or the page map's plain
 * pages. The bytes are read into a buffer and written as blocks by
 * ppu_dma_write; a byte on a page the map does not serve is read through
 * snes_read only after the bytes before it are written, so every read and
 * write keeps its order. Writes to these ports touch nothing on the A bus.
 * Returns aAdr after the transfer. Out of the BUS window: per transfer. */
__attribute__((noinline))
static uint16_t dma_to_ppu(Snes* sn, const uint8_t regs[4], unsigned oi, const uint8_t* src,
                           uint32_t ofs, uint32_t mask, uint8_t aBank, uint16_t aAdr, int step, uint32_t n) {
  uint8_t buf[128];
#if SNES_PAGEMAP
  uint8_t* const* map = &g_snes_rdmap[(uint32_t)aBank << 3];
#endif
  while (n) {
    uint32_t k = 0, lim = n < sizeof buf ? n : sizeof buf;
    if (src) {
      for (; k < lim; k++, aAdr += step) buf[k] = src[(ofs | aAdr) & mask];
    } else {
#if SNES_PAGEMAP
      for (; k < lim; k++, aAdr += step) {
        const uint8_t* pg = map[aAdr >> 13];
        if (!pg) break;
        buf[k] = pg[aAdr & 0x1fff];
      }
#endif
    }
    if (k) { ppu_dma_write(sn->ppu, regs, oi, buf, k); oi += k; n -= k; }
    if (k < lim) {   /* a page with no host pointer: this byte as before */
      uint8_t v = snes_read(sn, ((uint32_t)aBank << 16) | aAdr);
      ppu_dma_write(sn->ppu, regs, oi, &v, 1);
      oi++; n--; aAdr += step;
    }
  }
  return aAdr;
}

/* dma_doDma's A->B transfer, out of line so the BUS window keeps only the
 * channel scan: it runs once a transfer (snes_core.ld pins it after the
 * window). */
__attribute__((noinline))
static void dma_runAtoB(Dma* dma, DmaChannel* ch) {
  /* A->B (the common VRAM/CGRAM/OAM upload): every write goes to the B-bus
   * (snes_writeBBus, $2100-$21ff) and can never reach $420B, so there is NO
   * re-entrancy, and nothing observes the intermediate per-byte channel state
   * -- the CPU is stalled inside $420B's synchronous `while (dma_cycle(...)) {}`,
   * the only caller that drives a general DMA (dma_cycle's other callers in
   * main_snes.c's event loop only run when dmaBusy, which $420B always drains to
   * false before returning; dmaBusy/dmaTimer are referenced nowhere outside
   * dma.c and never feed the scanline scheduler). So drain the whole channel in
   * one call with identical per-byte address math. Host CPU only: a 32 KB VRAM
   * upload drops from ~40*N inner iterations (per-byte timer walk + an 8-way
   * active-channel rescan every byte) to N transfers. The dma_transferByte
   * sequence is byte-identical to the old loop (standalone equivalence test). */
  const int* off = bAdrOffsets[ch->mode];
  uint16_t aAdr = ch->aAdr;
  const uint8_t aBank = ch->aBank;
  const uint8_t bAdr = ch->bAdr;
  const int step = ch->fixed ? 0 : (ch->decrement ? -1 : 1);
  unsigned oi = ch->offIndex;
  uint16_t size = ch->size;   // 0 means 0x10000, matching the old size-- wrap

  /* The A-bus source bank is invariant across the transfer; only aAdr steps
   * (16-bit wrap). Instead of snes_read's classification ladder per byte, the
   * source is read from where snes_read would read it, byte-identical. Flat
   * sources are one host array, byte adr at src[(ofs | adr) & mask]:
   *   WRAM $7E/$7F        -> snes->ram + ((bank&1)<<16), mask 0xffff
   *   plain HiROM bank    -> bankBase[bank&0x3f], mask 0xffff (romDmaDirect)
   *   Super FX $70-$73    -> the Game Pak RAM (cart_gsu_dma_ram)
   * Anything else reads through the CPU's page map, g_snes_rdmap: per 8 KB page
   * a host pointer where snes_read reads plain memory (WRAM and its mirror,
   * ROM), else NULL, and that byte goes through snes_read as before. Except
   * where the page map serves ROM and cart_readLorom serves SRAM (banks
   * $70-$7D/$F0-$FF:$8000-$FFFF of a LoROM cart under 2 MB with SRAM, see
   * snes_pagemap_base): such a source keeps the per-byte snes_read.
   * Dest stays snes_writeBBus (VRAM/CGRAM/OAM side effects). Values are read
   * byte by byte as before, so a target that writes WRAM ($2180) is seen. */
  Snes* sn = dma->snes;
  Cart* cart = sn->cart;
  const uint8_t* src = NULL;
  uint32_t ofs = 0, mask = 0xffff;
  bool paged = false;
  if (aBank == 0x7e || aBank == 0x7f) {
    src = &sn->ram[(aBank & 1) << 16];
  } else if (cart->romDmaDirect && (aBank & 0x40) &&
             ((((uint32_t)(aBank & 0x3f)) << 16) + 0x10000u) <= cart->romSize) {
    /* a plain HiROM bank $40-$7D/$C0-$FF maps flat (no fold, no coprocessor,
     * no SRAM): cart_read for every adr, wrap included, is one base pointer.
     * The base is cart->bankBase[bank&0x3f], fold-baked at load. */
    src = cart->bankBase[aBank & 0x3f];
  } else {
    src = cart_gsu_dma_ram(cart, aBank, &ofs, &mask);
#if SNES_PAGEMAP
    if (!src)
      paged = !(cart->type == 1 && cart->ramSize > 0 && cart->romSize < 0x200000 &&
                ((aBank >= 0x70 && aBank < 0x7e) || aBank >= 0xf0));
#endif
  }
#ifdef DMA_HIST
  snes_fs_dma += size ? size : 0x10000u;
  dma_h_add(0, aBank, bAdr, ch->mode, (aBank == 0x7e || aBank == 0x7f) ? 0 : src ? (cart->gsu ? 2 : 1) : paged ? 5 : 3,
            size ? size : 0x10000u, dma_h_vmain(sn, bAdr, oi));
#endif
  const uint8_t regs[4] = { (uint8_t)(bAdr + off[0]), (uint8_t)(bAdr + off[1]),
                            (uint8_t)(bAdr + off[2]), (uint8_t)(bAdr + off[3]) };
  if ((src || paged) && ppu_dma_ports(regs)) {
    aAdr = dma_to_ppu(sn, regs, oi, src, ofs, mask, aBank, aAdr, step, size ? size : 0x10000u);
  } else if (src) {
    do {
      snes_writeBBus(sn, bAdr + off[oi & 3], src[(ofs | aAdr) & mask]);
      oi++; aAdr += step; size--;
    } while(size != 0);
#if SNES_PAGEMAP
  } else if (paged) {
    uint8_t* const* map = &g_snes_rdmap[(uint32_t)aBank << 3];
    do {
      const uint8_t* pg = map[aAdr >> 13];
      uint8_t v = pg ? pg[aAdr & 0x1fff] : snes_read(sn, ((uint32_t)aBank << 16) | aAdr);
      snes_writeBBus(sn, bAdr + off[oi & 3], v);
      oi++; aAdr += step; size--;
    } while(size != 0);
#endif
  } else {
    do {
      dma_transferByte(dma, aAdr, aBank, bAdr + off[oi & 3], false);
      oi++; aAdr += step; size--;
    } while(size != 0);
  }

  ch->offIndex = 0; // reset offset index
  ch->aAdr = aAdr;
  ch->size = 0;
  ch->dmaActive = false;
}

void dma_doDma(Dma* dma) {
#ifdef RIG_CALL_PROFILE
  extern uint64_t g_dma_doDma_calls;
  g_dma_doDma_calls++;
#endif
#ifdef DMA_HIST
  if (dma_h_on > 0) dma_h_doDma++;
#endif
  // figure out first channel that is active
  int i = 0;
  for(i = 0; i < 8; i++) {
    if(dma->channel[i].dmaActive) {
      break;
    }
  }
  if(i == 8) {
    // no active channels: the data has moved, but the CPU stays halted for the
    // transfer's bus time ($420B charges it to dmaTimer), 2 dots per call
    if(dma->dmaTimer >= 2) {
      dma->dmaTimer -= 2;
      return;
    }
    dma->dmaBusy = false;
    dma->dmaTimer = 0;
    return;
  }

  DmaChannel* ch = &dma->channel[i];

  /* SMW/Zelda homebrew used this as a "DMA from FastROM $00-$7FFF" canary
   * (those windows are WRAM/MMIO on LoROM $80-$BF). On HiROM, $C0-$FF is a
   * full 64 KB of ROM, so Secret of Evermore's $C6:2053 is a normal cart
   * upload — not a fault. Same for LoROM $40-$7F/$C0-$FF. Only warn when
   * the A-bus source is actually the system area. */
  if (!ch->fromB && (ch->aBank & 0x80) && !(ch->aAdr & 0x8000) && !g_fail) {
    uint8_t b7 = ch->aBank & 0x7f;
    int cart_type = (dma->snes && dma->snes->cart) ? dma->snes->cart->type : 0;
    bool a_is_rom = (cart_type == 1 || cart_type == 2) &&
                    (ch->aAdr >= 0x8000 || b7 >= 0x40);
    if (!a_is_rom) {
      printf("Warning! DMA from addr 0x%x\n", ch->aBank << 16 | ch->aAdr);
      g_fail = true;
    }
  }

  if (ch->fromB) {
    /* B->A: the transfer writes the A-bus through snes_write(), which can
     * decode a mirrored $420B and RE-ENTER dma_startDma (the emulator does not
     * block that A-bus target -- see dma_transferByte's TODO). Keep the exact
     * original one-byte-per-call form here so a re-entrant DMA re-reads channel
     * state from the struct between bytes, byte-for-byte as before. This
     * direction (reading PPU/APU back into RAM) is rare; the hot path is A->B. */
#ifdef DMA_HIST
    snes_fs_dma++;
    dma_h_add(0, ch->aBank, ch->bAdr, ch->mode, 4, 1, 0);
#endif
    dma_transferByte(
      dma, ch->aAdr, ch->aBank,
      ch->bAdr + bAdrOffsets[ch->mode][ch->offIndex++], true);
    ch->offIndex &= 3;
    if(!ch->fixed) ch->aAdr += ch->decrement ? -1 : 1;
    ch->size--;
    if(ch->size == 0) { ch->offIndex = 0; ch->dmaActive = false; }
    return;
  }

  dma_runAtoB(dma, ch);
}

void dma_initHdma(Dma* dma) {
  dma->hdmaTimer = 0;
  bool hdmaHappened = false;
  for(int i = 0; i < 8; i++) {
    if(dma->channel[i].hdmaActive) {
      hdmaHappened = true;
      // terminate any dma
      dma->channel[i].dmaActive = false;
      dma->channel[i].offIndex = 0;
      // load address, repCount, and indirect address if needed
      dma->channel[i].tableAdr = dma->channel[i].aAdr;
      dma->channel[i].repCount = snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++);
      if (dma->channel[i].repCount == 0) {
        dma->channel[i].terminated = true;
        continue;
      }
      dma->hdmaTimer += 8; // 8 cycle overhead for each active channel
      if(dma->channel[i].indirect) {
        dma->channel[i].size = snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++);
        dma->channel[i].size |= snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++) << 8;
        dma->hdmaTimer += 16; // another 16 cycles for indirect (total 24)
      }
      dma->channel[i].doTransfer = true;
    } else {
      dma->channel[i].doTransfer = false;
    }
    dma->channel[i].terminated = false;
  }
  if(hdmaHappened) dma->hdmaTimer += 16; // 18 cycles overhead, -2 for this cycle
}

void dma_doHdma(Dma* dma) {
#ifdef RIG_CALL_PROFILE
  extern uint64_t g_dma_doHdma_calls;
  g_dma_doHdma_calls++;
#endif
  dma->hdmaTimer = 0;
  bool hdmaHappened = false;
  for(int i = 0; i < 8; i++) {
    if(dma->channel[i].hdmaActive && !dma->channel[i].terminated) {
//      printf("DMA %d: 0x%x\n", i, dma->channel[i].bAdr);
      hdmaHappened = true;
      // terminate any dma
      dma->channel[i].dmaActive = false;
      dma->channel[i].offIndex = 0;
      // do the hdma
      dma->hdmaTimer += 8; // 8 cycles overhead for each active channel

      
      if(dma->channel[i].doTransfer) {
#ifdef DMA_HIST
        dma_h_add(1, dma->channel[i].indirect ? dma->channel[i].indBank : dma->channel[i].aBank,
                  dma->channel[i].bAdr, dma->channel[i].mode, 0, transferLength[dma->channel[i].mode], 0);
#endif
        for(int j = 0; j < transferLength[dma->channel[i].mode]; j++) {
          dma->hdmaTimer += 8; // 8 cycles for each byte transferred
          if(dma->channel[i].indirect) {
            dma_transferByte(
              dma, dma->channel[i].size++, dma->channel[i].indBank,
              dma->channel[i].bAdr + bAdrOffsets[dma->channel[i].mode][j], dma->channel[i].fromB
            );
          } else {
            dma_transferByte(
              dma, dma->channel[i].tableAdr++, dma->channel[i].aBank,
              dma->channel[i].bAdr + bAdrOffsets[dma->channel[i].mode][j], dma->channel[i].fromB
            );
          }
        }
      }
      dma->channel[i].repCount--;
      dma->channel[i].doTransfer = dma->channel[i].repCount & 0x80;
      if((dma->channel[i].repCount & 0x7f) == 0) {
        dma->channel[i].repCount = snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++);
        if(dma->channel[i].indirect) {
          // TODO: oddness with not fetching high byte if last active channel and reCount is 0
          dma->channel[i].size = snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++);
          dma->channel[i].size |= snes_read(dma->snes, (dma->channel[i].aBank << 16) | dma->channel[i].tableAdr++) << 8;
          dma->hdmaTimer += 16; // 16 cycles for new indirect address
        }
        if(dma->channel[i].repCount == 0) dma->channel[i].terminated = true;
        dma->channel[i].doTransfer = true;
      }
    }
  }
  if(hdmaHappened) dma->hdmaTimer += 16; // 18 cycles overhead, -2 for this cycle
}

static void dma_transferByte(Dma* dma, uint16_t aAdr, uint8_t aBank, uint8_t bAdr, bool fromB) {
  // TODO: invalid writes:
  //   accesing b-bus via a-bus gives open bus,
  //   $2180-$2183 while accessing ram via a-bus open busses $2180-$2183
  //   cannot access $4300-$437f (dma regs), or $420b / $420c
  if(fromB) {
    snes_write(dma->snes, (aBank << 16) | aAdr, snes_readBBus(dma->snes, bAdr));
  } else {
    snes_writeBBus(dma->snes, bAdr, snes_read(dma->snes, (aBank << 16) | aAdr));
  }
}

#ifdef SNES_BUS_IN_ITCM
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
bool dma_cycle(Dma* dma) {
#ifdef RIG_CALL_PROFILE
  extern uint64_t g_dma_cycle_calls, g_dma_cycle_true, g_win_dma_cycle_calls;
  g_dma_cycle_calls++; g_win_dma_cycle_calls++;
#endif
#ifdef DMA_HIST
  if (dma_h_on > 0) {
    dma_h_cycles++;
    if (dma->hdmaTimer > 0) dma_h_cycles_hdma++;
    else if (dma->dmaBusy) dma_h_cycles_dma++;
  }
#endif
  if(dma->hdmaTimer > 0) {
    dma->hdmaTimer -= 2;
#ifdef RIG_CALL_PROFILE
    g_dma_cycle_true++;
#endif
    return true;
  } else if(dma->dmaBusy) {
    dma_doDma(dma);
#ifdef RIG_CALL_PROFILE
    g_dma_cycle_true++;
#endif
    return true;
  }
  return false;
}

void dma_startDma(Dma* dma, uint8_t val, bool hdma) {
  for(int i = 0; i < 8; i++) {
    if(hdma) {
      dma->channel[i].hdmaActive = val & (1 << i);
    } else {
      dma->channel[i].dmaActive = val & (1 << i);
    }
  }
  if(hdma) dma->hdmaEnabled = val;   /* skip the per-line 8-scan when no HDMA armed */
  if(!hdma) {
    dma->dmaBusy = val;
    dma->dmaTimer += dma->dmaBusy ? 16 : 0; // 12-24 cycle overhead for entire dma transfer
  }
}
