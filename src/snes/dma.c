
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

void dma_doDma(Dma* dma) {
#ifdef RIG_CALL_PROFILE
  extern uint64_t g_dma_doDma_calls;
  g_dma_doDma_calls++;
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
    dma_transferByte(
      dma, ch->aAdr, ch->aBank,
      ch->bAdr + bAdrOffsets[ch->mode][ch->offIndex++], true);
    ch->offIndex &= 3;
    if(!ch->fixed) ch->aAdr += ch->decrement ? -1 : 1;
    ch->size--;
    if(ch->size == 0) { ch->offIndex = 0; ch->dmaActive = false; }
    return;
  }

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
   * (16-bit wrap). When the source is plain memory, read it directly instead of
   * re-running snes_read's full classification ladder per byte. The value is
   * byte-identical to snes_read for the ranges selected:
   *   WRAM $7E/$7F        -> snes->ram[((bank&1)<<16)|aAdr]  (flat 64K, wrap-safe)
   *   non-mirror banks    -> cart_read (snes_read routes $40-7D/$C0-FF straight
   *                          to cart_read for every adr, wrap included)
   *   mirror $00-3F/$80-BF-> cart_read ONLY if the whole run stays >= $8000 and
   *                          does not 16-bit-wrap (else it could hit the WRAM
   *                          mirror / MMIO ladder); otherwise fall back.
   * Dest stays snes_writeBBus (VRAM/CGRAM/OAM side effects). */
  Snes* sn = dma->snes;
  const uint32_t n = size ? size : 0x10000u;
  const uint8_t* wram = NULL;
  bool cart_fast = false;
  if (aBank == 0x7e || aBank == 0x7f) {
    wram = &sn->ram[(aBank & 1) << 16];
  } else {
    const bool mirror = aBank < 0x40 || (aBank >= 0x80 && aBank < 0xc0);
    if (!mirror) {
      cart_fast = true;                 /* every adr in the bank is cart */
    } else if (step == 0) {
      cart_fast = (aAdr >= 0x8000);
    } else if (step == 1) {
      cart_fast = (aAdr >= 0x8000) && ((uint32_t)aAdr + (n - 1) <= 0xffffu);
    } else { /* step == -1 */
      cart_fast = ((int32_t)aAdr - (int32_t)(n - 1) >= 0x8000);
    }
  }
  if (wram) {
    do {
      snes_writeBBus(sn, bAdr + off[oi & 3], wram[aAdr]);
      oi++; aAdr += step; size--;
    } while(size != 0);
  } else if (cart_fast) {
    Cart* cart = sn->cart;
    /* Hoist the ROM mapping out of the per-byte loop. The source bank is fixed,
     * so when it is a plain HiROM bank that maps flat (no fold, no coprocessor,
     * no SRAM), cart_read's fold + chip-dispatch ladder collapses to one base
     * pointer indexed by aAdr -- bit-identical to cart_read for that case. The
     * base is cart->bankBase[bank&0x3f] (already fold-baked at load); the flat
     * check (whole 64K bank below romSize) guarantees base[aAdr] == cart_read
     * across the full 16-bit wrap. Anything else keeps the per-byte cart_read. */
    const uint8_t* base = NULL;
    if (cart->romDmaDirect &&
        ((((uint32_t)(aBank & 0x3f)) << 16) + 0x10000u) <= cart->romSize) {
      base = cart->bankBase[aBank & 0x3f];   /* run-fixed preconditions in romDmaDirect */
    }
    if (base) {
      do {
        snes_writeBBus(sn, bAdr + off[oi & 3], base[aAdr]);
        oi++; aAdr += step; size--;
      } while(size != 0);
    } else {
      do {
        snes_writeBBus(sn, bAdr + off[oi & 3], cart_read(cart, aBank, aAdr));
        oi++; aAdr += step; size--;
      } while(size != 0);
    }
  } else
  do {
    dma_transferByte(dma, aAdr, aBank, bAdr + off[oi & 3], false);
    oi++;
    aAdr += step;
    size--;
  } while(size != 0);

  ch->offIndex = 0; // reset offset index
  ch->aAdr = aAdr;
  ch->size = 0;
  ch->dmaActive = false;
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
