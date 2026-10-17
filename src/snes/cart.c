#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "types.h"
#include "cart.h"
#include "snes.h"
#include "dsp1_hle.h"
#include "cx4_hle.h"
#include "sdd1.h"
#include "spc7110.h"
#include "gsu.h"
#include "snes_gnw_alloc.h"

/* Weak fallbacks so build scripts that list snes/*.c files explicitly and
 * predate dsp1_hle.c still link; without the strong definitions a DSP cart
 * just behaves as before (status reads 0). Harness globs pick up the real
 * implementation automatically. */
/* main_snes.c overrides this on the device, where each coprocessor's code is an
   overlay loaded only for its carts; elsewhere all of it is linked in. */
__attribute__((weak)) bool snes_copro_overlay_ready(int kind) { (void)kind; return true; }

/* A chip whose code is not loaded is not attached: cart->x stays NULL, and
   every call into its code is behind that pointer. */
static bool cart_copro_ready(int kind, const char *name) {
  if (snes_copro_overlay_ready(kind)) return true;
  printf("snes: %s code not loaded, chip not attached\n", name);
  return false;
}

__attribute__((weak)) void dsp1_reset(Dsp1* d) { (void)d; }
__attribute__((weak)) uint8_t dsp1_readDR(Dsp1* d) { (void)d; return 0; }
__attribute__((weak)) void dsp1_writeDR(Dsp1* d, uint8_t v) { (void)d; (void)v; }
__attribute__((weak)) uint8_t dsp1_readSR(Dsp1* d) { (void)d; return 0; }

/* sizeof(Dsp1) is unknown when only weak stubs are linked, so allocation lives
 * here behind the same weak mechanism: the strong version in dsp1_hle.c
 * allocates for real. */
__attribute__((weak)) Dsp1* dsp1_alloc(void) { return NULL; }
__attribute__((weak)) uint32_t dsp1_size(void) { return 0; }

__attribute__((weak)) void cx4_init(Cx4* c) { (void)c; }
__attribute__((weak)) uint8_t cx4_read(Cx4* c, uint16_t a) { (void)c; (void)a; return 0; }
__attribute__((weak)) void cx4_write(Cx4* c, uint16_t a, uint8_t v,
                                    const uint8_t* rom, uint32_t rom_size) {
  (void)c; (void)a; (void)v; (void)rom; (void)rom_size;
}
__attribute__((weak)) Cx4* cx4_alloc(void) { return NULL; }
__attribute__((weak)) uint32_t cx4_size(void) { return 0; }

__attribute__((weak)) void sdd1_reset(Sdd1* s) { (void)s; }
__attribute__((weak)) uint8_t sdd1_mmio_read(Sdd1* s, uint16_t a) { (void)s; (void)a; return 0; }
__attribute__((weak)) void sdd1_mmio_write(Sdd1* s, uint16_t a, uint8_t v) { (void)s; (void)a; (void)v; }
__attribute__((weak)) uint8_t sdd1_read(Sdd1* s, uint32_t a, const uint8_t* rom, uint32_t sz) {
  (void)s; (void)a; (void)rom; (void)sz; return 0;
}
__attribute__((weak)) Sdd1* sdd1_alloc(void) { return NULL; }
__attribute__((weak)) uint32_t sdd1_size(void) { return 0; }

__attribute__((weak)) void spc7110_reset(Spc7110* s) { (void)s; }
__attribute__((weak)) uint8_t spc7110_mmio_read(Spc7110* s, uint16_t a, const uint8_t* rom, uint32_t sz) {
  (void)s; (void)a; (void)rom; (void)sz; return 0;
}
__attribute__((weak)) void spc7110_mmio_write(Spc7110* s, uint16_t a, uint8_t v, const uint8_t* rom, uint32_t sz) {
  (void)s; (void)a; (void)v; (void)rom; (void)sz;
}
__attribute__((weak)) uint8_t spc7110_read(Spc7110* s, uint32_t a, const uint8_t* rom, uint32_t sz) {
  (void)s; (void)a; (void)rom; (void)sz; return 0;
}
__attribute__((weak)) Spc7110* spc7110_alloc(void) { return NULL; }
__attribute__((weak)) uint32_t spc7110_size(void) { return 0; }

__attribute__((weak)) Gsu* gsu_alloc(void) { return NULL; }
__attribute__((weak)) uint32_t gsu_size(void) { return 0; }
__attribute__((weak)) void gsu_reset(Gsu* g) { (void)g; }
__attribute__((weak)) void gsu_link(Gsu* g, Snes* s, uint8_t* rom, uint32_t rs,
                                    uint8_t* ram, uint32_t as) {
  (void)g; (void)s; (void)rom; (void)rs; (void)ram; (void)as;
}
__attribute__((weak)) uint8_t gsu_mmio_read(Gsu* g, uint16_t a) { (void)g; (void)a; return 0; }
__attribute__((weak)) void gsu_mmio_write(Gsu* g, uint16_t a, uint8_t v) { (void)g; (void)a; (void)v; }
__attribute__((weak)) bool gsu_cpu_ram_blocked(Gsu* g) { (void)g; return false; }
__attribute__((weak)) bool gsu_cpu_rom_blocked(Gsu* g) { (void)g; return false; }
__attribute__((weak)) void gsu_runLine(Gsu* g) { (void)g; }
__attribute__((weak)) void gsu_vblank(Gsu* g) { (void)g; }

#ifdef GNW_SNES_CORE
#include <assert.h>
#include "gw_malloc.h"
/* Device save-RAM. Writable, so it can't be XIP'd from flash like the ROM is,
 * and the ~81 KB DTCM heap is full. Games touch it rarely (save slots), so it
 * lives on the AHB heap, sized to what the header declares (most games 0-8 KB),
 * leaving the RAM_EMU pool to the hot data. The RAM_EMU pool is the fallback
 * when AHB is short. Allocated once (the core exits by switching app, nothing
 * is freed); a later cart_load reuses it. */
/* 64 KB: the largest Super FX Game Pak RAM (Doom, Stunt Race FX, Winter
 * Gold); normal carts request their header size, at most 32 KB in practice. */
enum { kGnwCartSramMax = 0x10000 };
static uint8_t *gnw_cart_sram;
static int gnw_cart_sram_size;
#endif


/* SNES carts mirror their ROM across the address space. A power-of-2 image needs
 * only a mask, but 1.5 MB / 3 MB / 6 MB images are common (37% of a real library)
 * and for those the top of the range folds back onto the last power-of-2 chunk —
 * exactly what the cart's chip-select decoding does on hardware. Precompute a
 * mask when we can and fall back to bsnes's fold otherwise, so the hot path stays
 * a single AND for most games. */
static uint32_t cart_fold(uint32_t addr, uint32_t size) {
  if (size == 0) return 0;
  uint32_t base = 0, mask = 1u << 31;
  while (addr >= size) {
    while (!(addr & mask)) mask >>= 1;
    addr -= mask;
    if (size > mask) { size -= mask; base += mask; }
  }
  return base + addr;
}

static inline uint32_t cart_romIndex(Cart* cart, uint32_t addr) {
  if (cart->romMask) return addr & cart->romMask;      /* power of 2: one AND */
  return cart_fold(addr, (uint32_t)cart->romSize);
}

/* snes_cpuRead's fetch-page cache used to require cart->romMask -- a
 * power-of-two ROM. "Every other size" is not exotic: Super Metroid is 3 MB.
 * cart_romIndex() already folds those; bake each bank's host base once. */
static void cart_buildBankLowRom(Cart* cart) {
  for(int raw = 0; raw < 256; raw++) {
    uint8_t b7 = (uint8_t)(raw & 0x7f);
    uint8_t ok = 0;
    if(raw != 0x7e && raw != 0x7f) {
      if(cart->type == 1) {
        int isSram = (((raw >= 0x70 && raw < 0x7e) || raw >= 0xf0) &&
                      cart->ramSize > 0);
        ok = (!isSram && b7 >= 0x40) ? 1 : 0;
      } else if(cart->type == 2) {
        ok = (b7 >= 0x40) ? 1 : 0;
      }
    }
    cart->bankLowRom[raw] = cart->romPageOk ? ok : 0;
  }
}

static void cart_buildBankBases(Cart* cart) {
  for(int b = 0; b < 128; b++) cart->bankBase[b] = cart->rom;
  if(cart->romPageOk) {
    if(cart->type == 1) {
      for(int b = 0; b < 128; b++)
        cart->bankBase[b] = cart->rom + cart_romIndex(cart, (uint32_t)b << 15);
    } else {
      for(int b = 0; b < 64; b++)
        cart->bankBase[b] = cart->rom + cart_romIndex(cart, (uint32_t)b << 16);
    }
  }
  cart_buildBankLowRom(cart);
}

void cart_setRomSize(Cart* cart, int size) {
  cart->romSize = size;
  cart->romMask = (size > 0 && (size & (size - 1)) == 0) ? (uint32_t)(size - 1) : 0;
  /* A HiROM bank is 64 KB and a LoROM bank is 32 KB. Demanding 64 KB of both
   * took the cache away from a 32 KB LoROM cart that HAD one via romMask. */
  const uint32_t bankGran = (cart->type == 2) ? 0xffffu : 0x7fffu;
  cart->romPageOk = (cart->rom != NULL && size > 0 &&
                     ((uint32_t)size & bankGran) == 0 &&
                     (cart->type == 1 || cart->type == 2)) ? 1 : 0;
  cart_buildBankBases(cart);
}

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);

Cart* cart_init(Snes* snes) {
  Cart* cart = snes_zalloc(sizeof(Cart));
  if (!cart) return NULL;
  cart->snes = snes;
  cart->type = 0;
  cart->rom = NULL;
  cart->romSize = 0;
  cart->romMask = 0;
  cart->ram = NULL;
  cart->ramSize = 0;
  cart->dsp1 = NULL;
  cart->cx4 = NULL;
  cart->sdd1 = NULL;
  cart->spc7110 = NULL;
  cart->gsu = NULL;
  cart->romPageOk = 0;
  memset(cart->bankLowRom, 0, sizeof(cart->bankLowRom));
  for(int b = 0; b < 128; b++) cart->bankBase[b] = NULL;
  return cart;
}

void cart_attachDsp1(Cart* cart) {
  if (!cart_copro_ready(SNES_COPRO_DSP1, "DSP-1")) return;
  if (cart->dsp1 == NULL) cart->dsp1 = dsp1_alloc();
  if (cart->dsp1) {
    dsp1_reset(cart->dsp1);
    /* LoROM boards decode the DSP at banks $30-$3f, $8000-$ffff — inside the
     * range snes_cpuRead's ROM fast path claims. snes_cpuRead excludes the
     * window at page-install time; SNES_DSP_FASTPATH=0 restores the old
     * "clear romMask for the whole cart" behaviour. */
#if !SNES_DSP_FASTPATH
    if (cart->type == 1) cart->romMask = 0;
#endif
  }
}

void cart_attachCx4(Cart* cart) {
  if (!cart_copro_ready(SNES_COPRO_CX4, "Cx4")) return;
  if (cart->cx4 == NULL) cart->cx4 = cx4_alloc();
  if (cart->cx4) {
    cx4_init(cart->cx4);
#ifdef GNW_SNES_CORE
    printf("snes: Cx4 HLE attached (map=%s, %d KB ROM)\n",
           cart->type == 1 ? "LoROM" : "HiROM",
           (int)(cart->romSize / 1024));
#endif
  }
#ifdef GNW_SNES_CORE
  else printf("snes: Cx4 alloc failed\n");
#endif
}

void cart_attachSdd1(Cart* cart) {
  if (!cart_copro_ready(SNES_COPRO_SDD1, "S-DD1")) return;
  if (cart->sdd1 == NULL) cart->sdd1 = sdd1_alloc();
  if (cart->sdd1) {
    sdd1_reset(cart->sdd1);
    /* S-DD1 uses its own MMC for banks $C0-$FF, so disable the ROM page cache
     * for those — cart_readHirom will route through sdd1_read instead. */
    cart->romPageOk = 0;
    cart_buildBankBases(cart);
#ifdef GNW_SNES_CORE
    printf("snes: S-DD1 attached (map=%s, %d KB ROM)\n",
           cart->type == 1 ? "LoROM" : "HiROM",
           (int)(cart->romSize / 1024));
#endif
  }
#ifdef GNW_SNES_CORE
  else printf("snes: S-DD1 alloc failed\n");
#endif
}

void cart_attachSpc7110(Cart* cart) {
  if (!cart_copro_ready(SNES_COPRO_SPC7110, "SPC7110")) return;
  if (cart->spc7110 == NULL) cart->spc7110 = spc7110_alloc();
  if (cart->spc7110) {
    spc7110_reset(cart->spc7110);
    cart->romPageOk = 0;
    cart_buildBankBases(cart);
#ifdef GNW_SNES_CORE
    printf("snes: SPC7110 attached (map=%s, %d KB ROM, %d KB SRAM%s)\n",
           cart->type == 1 ? "LoROM" : "HiROM",
           (int)(cart->romSize / 1024),
           (int)(cart->ramSize / 1024),
           cart->romSize > 0x600000 ? ", $40-$4F expansion" : "");
#endif
  }
#ifdef GNW_SNES_CORE
  else printf("snes: SPC7110 alloc failed\n");
#endif
}

void cart_attachGsu(Cart* cart) {
  if (!cart_copro_ready(SNES_COPRO_GSU, "Super FX")) return;
  if (cart->gsu == NULL) cart->gsu = gsu_alloc();
  if (cart->gsu) {
    /* The GSU shares ROM and Game Pak RAM with the 65816, so the fetch-page
     * cache and DMA ROM fast path must not bypass it. */
    cart->romPageOk = 0;
    cart_buildBankBases(cart);
    gsu_link(cart->gsu, cart->snes, cart->rom, cart->romSize,
             cart->ram, cart->ramSize);
    gsu_reset(cart->gsu);
#ifdef GNW_SNES_CORE
    printf("snes: Super FX (GSU) attached (%d KB ROM, %d KB RAM)\n",
           (int)(cart->romSize / 1024), (int)(cart->ramSize / 1024));
#endif
  }
#ifdef GNW_SNES_CORE
  else printf("snes: GSU alloc failed\n");
#endif
}

void cart_free(Cart* cart) {
  snes_zfree(cart);
}

void cart_reset(Cart* cart) {
  //if(cart->ramSize > 0 && cart->ram != NULL) memset(cart->ram, 0, cart->ramSize); // for now
  if (cart->dsp1) dsp1_reset(cart->dsp1);
  if (cart->cx4) cx4_init(cart->cx4);
  if (cart->sdd1) sdd1_reset(cart->sdd1);
  if (cart->spc7110) spc7110_reset(cart->spc7110);
  if (cart->gsu) gsu_reset(cart->gsu);
  /* Fold the DMA ROM fast path's run-fixed conditions now that coprocessors
   * (if any) are attached. */
  cart->romDmaDirect = (cart->type == 2 && cart->romPageOk &&
                        !cart->dsp1 && !cart->cx4 && !cart->sdd1 &&
                        !cart->spc7110 && !cart->gsu);
}

void cart_saveload(Cart *cart, SaveLoadFunc *func, void *ctx) {
  func(ctx, cart->ram, cart->ramSize);
  /* DSP / Cx4 carts append chip state (plain data, versioned via the first
   * field). Normal carts write exactly what they always did, so existing
   * savestates stay byte-compatible. */
  if (cart->dsp1) func(ctx, cart->dsp1, dsp1_size());
  if (cart->cx4) func(ctx, cart->cx4, cx4_size());
  if (cart->sdd1) func(ctx, cart->sdd1, sdd1_size());
  if (cart->spc7110) func(ctx, cart->spc7110, spc7110_size());
  /* Super FX (GSU): no state saved — the ZSNES core keeps its registers in
   * globals, outside this serialized Cart (Phase-1 PoC limitation). The shared
   * Game Pak RAM above (cart->ram) is still saved. */
}

void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize) {
  cart->type = type;
#ifdef GNW_SNES_CORE
  // Device: neither the ROM nor the SRAM touches the ~81 KB heap. `rom` is the
  // flash-mapped image used in place (never freed); the SRAM comes from the
  // AHB heap (see gnw_cart_sram) and is kept for the core's lifetime.
  cart->rom = rom;
  cart_setRomSize(cart, romSize);
  if(ramSize > 0) {
    /* A header can claim more SRAM than the 32 KB static buffer (64/128 KB
     * carts exist). Clamp instead of assert-BSOD: the power-of-2 clamp keeps
     * every `& (ramSize-1)` mask valid, so oversized carts see a mirrored
     * 32 KB -- degraded but running, exactly what an undersized SRAM chip
     * does on a repro board. */
    if(ramSize > kGnwCartSramMax) ramSize = kGnwCartSramMax;
    if (!gnw_cart_sram || gnw_cart_sram_size < ramSize) {
      gnw_cart_sram = (uint8_t *)ahb_calloc(1, (size_t)ramSize);
      if (!gnw_cart_sram) {
        printf("cart: SRAM %d bytes from the RAM_EMU pool (AHB full)\n", ramSize);
        gnw_cart_sram = (uint8_t *)ram_calloc(1, (size_t)ramSize);
      }
      gnw_cart_sram_size = gnw_cart_sram ? ramSize : 0;
    }
    if (!gnw_cart_sram) {
      printf("cart: no room for %d bytes of SRAM, running without it\n", ramSize);
      ramSize = 0;
    }
    cart->ram = gnw_cart_sram;
    if (cart->ram) memset(cart->ram, 0, ramSize);
  } else {
    cart->ram = NULL;
  }
  cart->ramSize = ramSize;
  /* AFTER ramSize: bankLowRom has to know whether $70-$7d is SRAM. */
  cart_buildBankBases(cart);
#else
  if(cart->rom != NULL) free(cart->rom);
  if(cart->ram != NULL) free(cart->ram);
  cart->rom = malloc(romSize);
  cart_setRomSize(cart, romSize);
  if(ramSize > 0) {
    cart->ram = malloc(ramSize);
    memset(cart->ram, 0, ramSize);
  } else {
    cart->ram = NULL;
  }
  cart->ramSize = ramSize;
  memcpy(cart->rom, rom, romSize);
  cart_buildBankBases(cart);   /* AFTER ramSize -- see the device branch above */
#endif
}

uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr) {
  switch(cart->type) {
    case 0: 
      assert(0);
      return cart->snes->openBus;
    case 1: return cart_readLorom(cart, bank, adr);
    case 2: return cart_readHirom(cart, bank, adr);
  }
  assert(0);
  return cart->snes->openBus;
}

void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  switch(cart->type) {
    case 0: break;
    case 1: cart_writeLorom(cart, bank, adr, val); break;
    case 2: cart_writeHirom(cart, bank, adr, val); break;
  }
}

void DumpCpuHistory();

/* Super FX CPU-side map: ROM at $00-$3F/$80-$BF:$8000-$FFFF (LoROM halves) and
 * $40-$5F/$C0-$DF:$0000-$FFFF (64 KB-linear view of the same image); Game Pak
 * RAM at $70-$71:$0000-$FFFF and $00-$3F/$80-$BF:$6000-$7FFF. While the GSU owns
 * the shared bus the CPU sees open bus. Returns 1 and sets *out if it handled
 * the address. */
/* Out of line: cart_read sits in the fixed BUS window (snes_core.ld), and
 * normal carts pay only the cart->gsu test. */
__attribute__((noinline))
static int cart_gsu_read(Cart* cart, uint8_t bank, uint16_t adr, uint8_t* out) {
  uint8_t b = bank & 0x7f;                  /* fold $80-$FF mirrors */
  /* GSU registers and cache $3000-$32FF: here rather than in snes_read, which
   * sits in the fixed BUS window with every other game's hot handlers */
  if(b <= 0x3f && adr >= 0x3000 && adr < 0x3300) {
    *out = gsu_mmio_read(cart->gsu, adr);
    return 1;
  }
  if(cart->ramSize > 0) {
    if(b >= 0x70 && b <= 0x73) {            /* $70-$73 Game Pak RAM (4x64KB) */
      *out = gsu_cpu_ram_blocked(cart->gsu) ? cart->snes->openBus
           : cart->ram[(((b & 3) << 16) | adr) & (cart->ramSize - 1)];
      return 1;
    }
  }
  /* $00-$3F:$6000-$7FFF mirrors the first 8 KB of Game Pak RAM into every bank
   * (the Super FX MMC window the CPU and GSU share as scratch). bsnes maps this
   * to Game Pak RAM, not WRAM; ZSNES's map_sfx routes it to WRAM, which is an
   * acknowledged inaccuracy and corrupted Yoshi's story text + title transition
   * here. The fixed first-8KB mirror (not a banked b*8KB window) is what renders
   * correctly. */
  if(b <= 0x3f && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    *out = cart->ram[(adr & 0x1fff) & (cart->ramSize - 1)];
    return 1;
  }
  if(b <= 0x3f && adr >= 0x8000) {          /* LoROM halves */
    *out = gsu_cpu_rom_blocked(cart->gsu) ? cart->snes->openBus
         : cart->rom[cart_romIndex(cart, ((uint32_t)b << 15) | (adr & 0x7fff))];
    return 1;
  }
  if(b >= 0x40 && b <= 0x5f) {              /* 64 KB-linear */
    *out = gsu_cpu_rom_blocked(cart->gsu) ? cart->snes->openBus
         : cart->rom[cart_romIndex(cart, ((uint32_t)(b & 0x1f) << 16) | adr)];
    return 1;
  }
  return 0;
}

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr) {
  if(cart->gsu) {
    uint8_t v;
    if(cart_gsu_read(cart, bank, adr, &v)) return v;
    return cart->snes->openBus;
  }
  /* S-DD1 MMC/decompress window on upper banks. Keep full 24-bit address
   * for DMA source address matching in sdd1_read(). */
  if(cart->sdd1 && bank >= 0xc0) {
    uint32_t full = ((uint32_t)bank << 16) | adr;
    return sdd1_read(cart->sdd1, full, cart->rom, cart->romSize);
  }
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) &&
     ((cart->romSize >= 0x200000 && adr < 0x8000) || cart->romSize < 0x200000) &&
     cart->ramSize > 0) {
    // banks 70-7d and f0-ff: adr 0000-7fff for rom >= 2MB, else full 0000-ffff
    // (small LoROM carts mirror SRAM across the whole bank -- dinkc64 e9e1139,
    // fixes PGA Tour)
    return cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)];
  }
  bank &= 0x7f;
  /* Cx4 boards are LoROM: ROM at $8000-$FFFF, MMIO at $00-$3F:$6000-$7FFF. */
  if(cart->cx4 && bank < 0x40 && adr >= 0x6000 && adr < 0x8000) {
    return cx4_read(cart->cx4, adr);
  }
  // DSP-1 on LoROM boards: banks 30-3f, DR 8000-bfff / SR c000-ffff
  if(cart->dsp1 && bank >= 0x30 && bank < 0x40 && adr >= 0x8000) {
    return adr < 0xc000 ? dsp1_readDR(cart->dsp1) : dsp1_readSR(cart->dsp1);
  }
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[cart_romIndex(cart, ((uint32_t)bank << 15) | (adr & 0x7fff))];
  }
#ifdef GNW_SNES_CORE
  /* General-purpose core: an unmapped cart-region read is OPEN BUS on real
   * hardware (last value on the bus), and commercial games do stray reads
   * there in normal play. Crashing here is a homebrew-development aid, not
   * emulation -- keep it for the sm/zelda3 dev builds below, never here. */
  return cart->snes->openBus;
#else
  printf("While trying to read from 0x%x\n", bank << 16 | adr);
  DumpCpuHistory();
  Die("The game crashed in cart_readLorom");
  return cart->snes->openBus;
#endif
}

/* Super FX CPU-side writes: Game Pak RAM only (see cart_gsu_read). Out of
 * line for the same reason. */
__attribute__((noinline))
static void cart_gsu_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  uint8_t b = bank & 0x7f;
  if(b <= 0x3f && adr >= 0x3000 && adr < 0x3300) {   /* GSU registers / cache */
    gsu_mmio_write(cart->gsu, adr, val);
    return;
  }
  if(b <= 0x3f && adr >= 0x6000 && adr < 0x8000) {
    if(cart->ramSize > 0)            /* first-8KB Game Pak RAM mirror; see cart_gsu_read */
      cart->ram[(adr & 0x1fff) & (cart->ramSize - 1)] = val;
  } else if(cart->ramSize > 0 && !gsu_cpu_ram_blocked(cart->gsu) &&
            b >= 0x70 && b <= 0x73) {
    cart->ram[(((b & 3) << 16) | adr) & (cart->ramSize - 1)] = val;
  }
  /* ROM region is not writable */
}

static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  if(cart->gsu) {
    cart_gsu_write(cart, bank, adr, val);
    return;
  }
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) &&
     ((cart->romSize >= 0x200000 && adr < 0x8000) || cart->romSize < 0x200000) &&
     cart->ramSize > 0) {
    // banks 70-7d and f0-ff: adr 0000-7fff for rom >= 2MB, else full 0000-ffff
    // (small LoROM carts mirror SRAM across the whole bank -- dinkc64 e9e1139,
    // fixes PGA Tour)
    cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)] = val;
    return;
  }
  bank &= 0x7f;
  /* Cx4 on LoROM boards: banks 00-3f, adr 6000-7fff */
  if(cart->cx4 && bank < 0x40 && adr >= 0x6000 && adr < 0x8000) {
    cx4_write(cart->cx4, adr, val, cart->rom, cart->romSize);
    return;
  }
  // DSP-1 on LoROM boards: banks 30-3f, adr 8000-ffff
  if(cart->dsp1 && bank >= 0x30 && bank < 0x40 && adr >= 0x8000) {
    if (adr < 0xc000) dsp1_writeDR(cart->dsp1, val);
  }
}

/* SPC7110 $4830 bit 7 enables cart SRAM. ares: disabled reads return 0,
 * writes are ignored. The TMZ check program fills 8 KB with $FF only while
 * that bit is set, then Mode 2 reads it back the same way. */
static int cart_spc7110_sram_en(const Cart* cart) {
  return !cart->spc7110 || (cart->spc7110->r4830 & 0x80);
}

static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  if (cart->spc7110) {
    if (bank == 0x50 || (bank >= 0x40 && bank <= 0x4f) ||
        (bank >= 0xc0 && bank <= 0xff)) {
      uint32_t full = ((uint32_t)bank << 16) | adr;
      return spc7110_read(cart->spc7110, full, cart->rom, cart->romSize);
    }
    /* Program ROM is only $00-$0F / $80-$8F:$8000-$FFFF. $10-$3F would
     * otherwise decode as HiROM into data ROM and execute garbage. */
    if ((bank & 0x7f) >= 0x10 && (bank & 0x7f) < 0x40 && adr >= 0x8000)
      return cart->snes->openBus;
  }
  /* S-DD1 MMC: banks $C0-$FF (after mask: $40-$7F) go through the chip's
   * memory-map controller and possibly the decompressor. */
  if(cart->sdd1 && (bank & 0xc0) == 0xc0) {
    /* $C0-$FF:$0000-$FFFF window: keep the full 24-bit CPU address for
     * matching DMA source addresses exactly like S-DD1 hardware. */
    uint32_t full = ((uint32_t)bank << 16) | adr;
    return sdd1_read(cart->sdd1, full, cart->rom, cart->romSize);
  }
  bank &= 0x7f;
  /* Cx4 owns $00-$3F/$80-$BF:$6000-$7FFF. Battery SRAM on those boards is
   * LoROM-style at $70-$77:$0000-$7FFF. */
  if(cart->cx4 && bank < 0x40 && adr >= 0x6000 && adr < 0x8000) {
    return cx4_read(cart->cx4, adr);
  }
  if(cart->cx4 && cart->ramSize > 0 && bank >= 0x70 && bank < 0x78 && adr < 0x8000) {
    return cart->ram[(((bank & 7) << 15) | adr) & (cart->ramSize - 1)];
  }
  // DSP-1 on HiROM boards: banks 00-1f, DR 6000-6fff / SR 7000-7fff.
  // SRAM decode moves up to banks 20-3f (matching real HiROM+DSP boards).
  if(cart->dsp1 && bank < 0x20 && adr >= 0x6000 && adr < 0x8000) {
    return adr < 0x7000 ? dsp1_readDR(cart->dsp1) : dsp1_readSR(cart->dsp1);
  }
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    if (!cart_spc7110_sram_en(cart)) return 0x00;
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[cart_romIndex(cart, (((uint32_t)(bank & 0x3f)) << 16) | adr)];
  }
#ifdef GNW_SNES_CORE
  /* General-purpose core: unmapped reads are open bus, same reasoning as
   * cart_readLorom above. Mario Kart reads $80:4A78 (bank 0 after masking,
   * $4400-$5FFF system-area hole) ~40 frames into the title screen -- real
   * hardware shrugs; this assert was a device BSOD (host builds compile
   * asserts out with -DNDEBUG, which is why every host run sailed past it). */
  return cart->snes->openBus;
#else
  assert(0);
  return cart->snes->openBus;
#endif
}

static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  bank &= 0x7f;
  if(cart->cx4 && bank < 0x40 && adr >= 0x6000 && adr < 0x8000) {
    cx4_write(cart->cx4, adr, val, cart->rom, cart->romSize);
    return;
  }
  if(cart->cx4 && cart->ramSize > 0 && bank >= 0x70 && bank < 0x78 && adr < 0x8000) {
    cart->ram[(((bank & 7) << 15) | adr) & (cart->ramSize - 1)] = val;
    return;
  }
  if(cart->dsp1 && bank < 0x20 && adr >= 0x6000 && adr < 0x8000) {
    if (adr < 0x7000) dsp1_writeDR(cart->dsp1, val);
    return;
  }
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    if (!cart_spc7110_sram_en(cart)) return;
    cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)] = val;
  }
}
