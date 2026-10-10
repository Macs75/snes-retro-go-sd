/* SPC7110 coprocessor emulation.
 *
 * Decompression + MMIO originally by byuu and neviksti (spc7110emu 0.03,
 * 2008-08-10). Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted.
 *
 * Clean C port for LakeSnes / Retro-Go SD. */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "spc7110.h"
#include "snes_gnw_alloc.h"
#ifdef TARGET_GNW
#include "gw_malloc.h"
extern void wdog_refresh(void);
#endif
#ifdef HOST_BUILD
#include <stdio.h>
#include <stdlib.h>
#endif

#define SPC7110_VERSION 1

enum {
  RTCS_Inactive = 0,
  RTCS_ModeSelect = 1,
  RTCS_IndexSelect = 2,
  RTCS_Write = 3
};

/* 6 KB, only for SPC7110 carts: allocated by spc7110_alloc (AHB first on
 * the device, so the RAM_EMU pool is left to the hot data). */
static uint32_t (*morton16)[256];
static uint32_t (*morton32)[256];
static int morton_ready;

static int morton_init(void) {
  if (morton_ready) return 1;
  const size_t n = sizeof(uint32_t) * 256 * (2 + 4);
#ifdef TARGET_GNW
  uint32_t (*t)[256] = (uint32_t (*)[256])ahb_calloc(1, n);
  if (!t) t = (uint32_t (*)[256])ram_calloc(1, n);
#else
  uint32_t (*t)[256] = (uint32_t (*)[256])snes_zalloc(n);
#endif
  if (!t) return 0;
  morton16 = t; morton32 = t + 2;
  for (unsigned i = 0; i < 256; i++) {
#define MAP(x, y) (((i >> (x)) & 1u) << (y))
    morton16[1][i] = MAP(7, 15) + MAP(6, 7) + MAP(5, 14) + MAP(4, 6)
                   + MAP(3, 13) + MAP(2, 5) + MAP(1, 12) + MAP(0, 4);
    morton16[0][i] = MAP(7, 11) + MAP(6, 3) + MAP(5, 10) + MAP(4, 2)
                   + MAP(3, 9)  + MAP(2, 1) + MAP(1, 8)  + MAP(0, 0);
    morton32[3][i] = MAP(7, 31) + MAP(6, 23) + MAP(5, 15) + MAP(4, 7)
                   + MAP(3, 30) + MAP(2, 22) + MAP(1, 14) + MAP(0, 6);
    morton32[2][i] = MAP(7, 29) + MAP(6, 21) + MAP(5, 13) + MAP(4, 5)
                   + MAP(3, 28) + MAP(2, 20) + MAP(1, 12) + MAP(0, 4);
    morton32[1][i] = MAP(7, 27) + MAP(6, 19) + MAP(5, 11) + MAP(4, 3)
                   + MAP(3, 26) + MAP(2, 18) + MAP(1, 10) + MAP(0, 2);
    morton32[0][i] = MAP(7, 25) + MAP(6, 17) + MAP(5, 9)  + MAP(4, 1)
                   + MAP(3, 24) + MAP(2, 16) + MAP(1, 8)  + MAP(0, 0);
#undef MAP
  }
  morton_ready = 1;
  return 1;
}

#ifdef HOST_BUILD
static int host_spc7110_log(void) {
  static int t = -1;
  if (t < 0) {
    const char *e = getenv("HOST_SPC7110_LOG");
    t = (e && e[0]) ? 1 : 0;
  }
  return t;
}
#endif

static uint32_t datarom_addr(uint32_t addr, uint32_t romSize) {
  uint32_t size = romSize > 0x500000u ? romSize - 0x200000u : romSize - 0x100000u;
  if (size == 0) return 0x100000u;
  while (addr >= size) addr -= size;
  return addr + 0x100000u;
}

static uint8_t rom_get(const uint8_t *rom, uint32_t romSize, uint32_t addr) {
  if (addr >= romSize) return 0;
  return rom[addr];
}

/* ========== Decompression (byuu / neviksti) ========== */

static const uint8_t evolution_table[53][4] = {
  { 0x5a,  1,  1, 1 },
  { 0x25,  6,  2, 0 },
  { 0x11,  8,  3, 0 },
  { 0x08, 10,  4, 0 },
  { 0x03, 12,  5, 0 },
  { 0x01, 15,  5, 0 },
  { 0x5a,  7,  7, 1 },
  { 0x3f, 19,  8, 0 },
  { 0x2c, 21,  9, 0 },
  { 0x20, 22, 10, 0 },
  { 0x17, 23, 11, 0 },
  { 0x11, 25, 12, 0 },
  { 0x0c, 26, 13, 0 },
  { 0x09, 28, 14, 0 },
  { 0x07, 29, 15, 0 },
  { 0x05, 31, 16, 0 },
  { 0x04, 32, 17, 0 },
  { 0x03, 34, 18, 0 },
  { 0x02, 35,  5, 0 },
  { 0x5a, 20, 20, 1 },
  { 0x48, 39, 21, 0 },
  { 0x3a, 40, 22, 0 },
  { 0x2e, 42, 23, 0 },
  { 0x26, 44, 24, 0 },
  { 0x1f, 45, 25, 0 },
  { 0x19, 46, 26, 0 },
  { 0x15, 25, 27, 0 },
  { 0x11, 26, 28, 0 },
  { 0x0e, 26, 29, 0 },
  { 0x0b, 27, 30, 0 },
  { 0x09, 28, 31, 0 },
  { 0x08, 29, 32, 0 },
  { 0x07, 30, 33, 0 },
  { 0x05, 31, 34, 0 },
  { 0x04, 33, 35, 0 },
  { 0x04, 33, 36, 0 },
  { 0x03, 34, 37, 0 },
  { 0x02, 35, 38, 0 },
  { 0x02, 36,  5, 0 },
  { 0x58, 39, 40, 1 },
  { 0x4d, 47, 41, 0 },
  { 0x43, 48, 42, 0 },
  { 0x3b, 49, 43, 0 },
  { 0x34, 50, 44, 0 },
  { 0x2e, 51, 45, 0 },
  { 0x29, 44, 46, 0 },
  { 0x25, 45, 24, 0 },
  { 0x56, 47, 48, 1 },
  { 0x4f, 47, 49, 0 },
  { 0x47, 48, 50, 0 },
  { 0x41, 49, 51, 0 },
  { 0x3c, 50, 52, 0 },
  { 0x37, 51, 43, 0 },
};

static const uint8_t mode2_context_table[32][2] = {
  {  1,  2 },
  {  3,  8 },
  { 13, 14 },
  { 15, 16 },
  { 17, 18 },
  { 19, 20 },
  { 21, 22 },
  { 23, 24 },
  { 25, 26 },
  { 25, 26 },
  { 25, 26 },
  { 25, 26 },
  { 25, 26 },
  { 27, 28 },
  { 29, 30 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
  { 31, 31 },
};

static uint8_t probability(Spc7110 *s, unsigned n) { return evolution_table[s->context[n].index][0]; }
static uint8_t next_lps(Spc7110 *s, unsigned n) { return evolution_table[s->context[n].index][1]; }
static uint8_t next_mps(Spc7110 *s, unsigned n) { return evolution_table[s->context[n].index][2]; }
static uint8_t toggle_invert(Spc7110 *s, unsigned n) { return evolution_table[s->context[n].index][3]; }

static uint32_t morton_2x8(uint32_t data) {
  return morton16[0][(data >> 0) & 255] + morton16[1][(data >> 8) & 255];
}

static uint32_t morton_4x8(uint32_t data) {
  return morton32[0][(data >> 0) & 255] + morton32[1][(data >> 8) & 255]
       + morton32[2][(data >> 16) & 255] + morton32[3][(data >> 24) & 255];
}

static void decomp_write(Spc7110 *s, uint8_t data) {
  s->decomp_buffer[s->decomp_buffer_wroffset++] = data;
  s->decomp_buffer_wroffset &= SPC7110_DECOMP_BUFFER_SIZE - 1;
  s->decomp_buffer_length++;
}

static uint8_t decomp_dataread(Spc7110 *s, const uint8_t *rom, uint32_t romSize) {
  uint32_t size = romSize > 0x500000u ? romSize - 0x200000u : romSize - 0x100000u;
  if (size == 0) return 0;
  while (s->decomp_offset >= size) s->decomp_offset -= size;
  return rom_get(rom, romSize, 0x100000u + s->decomp_offset++);
}

static void mode0(Spc7110 *s, bool init, const uint8_t *rom, uint32_t romSize) {
  if (init) {
    s->d_out = s->d_inverts = s->d_lps = 0;
    s->d_span = 0xff;
    s->d_val = decomp_dataread(s, rom, romSize);
    s->d_in = decomp_dataread(s, rom, romSize);
    s->d_in_count = 8;
    return;
  }

  while (s->decomp_buffer_length < (SPC7110_DECOMP_BUFFER_SIZE >> 1)) {
    unsigned bit;
    for (bit = 0; bit < 8; bit++) {
      uint8_t mask = (uint8_t)((1u << (bit & 3)) - 1);
      uint8_t con = (uint8_t)(mask + ((s->d_inverts & mask) ^ (s->d_lps & mask)));
      unsigned prob, mps, flag_lps, shift;
      if (bit > 3) con = (uint8_t)(con + 15);

      prob = probability(s, con);
      mps = (((s->d_out >> 15) & 1) ^ s->context[con].invert);

      if (s->d_val <= (uint8_t)(s->d_span - prob)) {
        s->d_span = (uint8_t)(s->d_span - prob);
        s->d_out = (s->d_out << 1) + (int32_t)mps;
        flag_lps = 0;
      } else {
        s->d_val = (uint8_t)(s->d_val - (s->d_span - (prob - 1)));
        s->d_span = (uint8_t)(prob - 1);
        s->d_out = (s->d_out << 1) + 1 - (int32_t)mps;
        flag_lps = 1;
      }

      shift = 0;
      while (s->d_span < 0x7f) {
        shift++;
        s->d_span = (uint8_t)((s->d_span << 1) + 1);
        s->d_val = (uint8_t)((s->d_val << 1) + (s->d_in >> 7));
        s->d_in <<= 1;
        if (--s->d_in_count == 0) {
          s->d_in = decomp_dataread(s, rom, romSize);
          s->d_in_count = 8;
        }
      }

      s->d_lps = (s->d_lps << 1) + (int32_t)flag_lps;
      s->d_inverts = (s->d_inverts << 1) + s->context[con].invert;
      if (flag_lps & toggle_invert(s, con)) s->context[con].invert ^= 1;
      if (flag_lps) s->context[con].index = next_lps(s, con);
      else if (shift) s->context[con].index = next_mps(s, con);
    }
    decomp_write(s, (uint8_t)s->d_out);
  }
}

static void mode1(Spc7110 *s, bool init, const uint8_t *rom, uint32_t romSize) {
  unsigned realorder[4];
  if (init) {
    unsigned i;
    for (i = 0; i < 4; i++) s->d_pixelorder[i] = i;
    s->d_out = s->d_inverts = s->d_lps = 0;
    s->d_span = 0xff;
    s->d_val = decomp_dataread(s, rom, romSize);
    s->d_in = decomp_dataread(s, rom, romSize);
    s->d_in_count = 8;
    return;
  }

  while (s->decomp_buffer_length < (SPC7110_DECOMP_BUFFER_SIZE >> 1)) {
    unsigned pixel;
    uint32_t packed;
    for (pixel = 0; pixel < 8; pixel++) {
      unsigned a = (s->d_out >> (1 * 2)) & 3;
      unsigned b = (s->d_out >> (7 * 2)) & 3;
      unsigned c = (s->d_out >> (8 * 2)) & 3;
      unsigned con = (a == b) ? (b != c) : (b == c) ? 2 : 4 - (a == c);
      unsigned m, n, bit;

      for (m = 0; m < 4; m++) if (s->d_pixelorder[m] == a) break;
      for (n = m; n > 0; n--) s->d_pixelorder[n] = s->d_pixelorder[n - 1];
      s->d_pixelorder[0] = a;

      for (m = 0; m < 4; m++) realorder[m] = s->d_pixelorder[m];
      for (m = 0; m < 4; m++) if (realorder[m] == c) break;
      for (n = m; n > 0; n--) realorder[n] = realorder[n - 1];
      realorder[0] = c;
      for (m = 0; m < 4; m++) if (realorder[m] == b) break;
      for (n = m; n > 0; n--) realorder[n] = realorder[n - 1];
      realorder[0] = b;
      for (m = 0; m < 4; m++) if (realorder[m] == a) break;
      for (n = m; n > 0; n--) realorder[n] = realorder[n - 1];
      realorder[0] = a;

      for (bit = 0; bit < 2; bit++) {
        unsigned prob = probability(s, con);
        unsigned flag_lps, shift;
        if (s->d_val <= (uint8_t)(s->d_span - prob)) {
          s->d_span = (uint8_t)(s->d_span - prob);
          flag_lps = 0;
        } else {
          s->d_val = (uint8_t)(s->d_val - (s->d_span - (prob - 1)));
          s->d_span = (uint8_t)(prob - 1);
          flag_lps = 1;
        }
        shift = 0;
        while (s->d_span < 0x7f) {
          shift++;
          s->d_span = (uint8_t)((s->d_span << 1) + 1);
          s->d_val = (uint8_t)((s->d_val << 1) + (s->d_in >> 7));
          s->d_in <<= 1;
          if (--s->d_in_count == 0) {
            s->d_in = decomp_dataread(s, rom, romSize);
            s->d_in_count = 8;
          }
        }
        s->d_lps = (s->d_lps << 1) + (int32_t)flag_lps;
        s->d_inverts = (s->d_inverts << 1) + s->context[con].invert;
        if (flag_lps & toggle_invert(s, con)) s->context[con].invert ^= 1;
        if (flag_lps) s->context[con].index = next_lps(s, con);
        else if (shift) s->context[con].index = next_mps(s, con);
        con = 5 + (con << 1) + ((s->d_lps ^ s->d_inverts) & 1);
      }
      b = realorder[(s->d_lps ^ s->d_inverts) & 3];
      s->d_out = (s->d_out << 2) + (int32_t)b;
    }
    packed = morton_2x8((uint32_t)s->d_out);
    decomp_write(s, (uint8_t)(packed >> 8));
    decomp_write(s, (uint8_t)(packed >> 0));
  }
}

static void mode2(Spc7110 *s, bool init, const uint8_t *rom, uint32_t romSize) {
  unsigned realorder[16];
  if (init) {
    unsigned i;
    for (i = 0; i < 16; i++) s->d_pixelorder[i] = i;
    s->d_buffer_index = 0;
    s->d_out0 = s->d_out1 = s->d_inverts = s->d_lps = 0;
    s->d_span = 0xff;
    s->d_val = decomp_dataread(s, rom, romSize);
    s->d_in = decomp_dataread(s, rom, romSize);
    s->d_in_count = 8;
    return;
  }

  while (s->decomp_buffer_length < (SPC7110_DECOMP_BUFFER_SIZE >> 1)) {
    unsigned pixel;
    uint32_t packed;
    for (pixel = 0; pixel < 8; pixel++) {
      unsigned a = (s->d_out0 >> (0 * 4)) & 15;
      unsigned b = (s->d_out0 >> (7 * 4)) & 15;
      unsigned c = (s->d_out1 >> (0 * 4)) & 15;
      unsigned con = 0;
      unsigned refcon = (a == b) ? (b != c) : (b == c) ? 2 : 4 - (a == c);
      unsigned m, n, bit;

      for (m = 0; m < 16; m++) if (s->d_pixelorder[m] == a) break;
      for (n = m; n > 0; n--) s->d_pixelorder[n] = s->d_pixelorder[n - 1];
      s->d_pixelorder[0] = a;

      for (m = 0; m < 16; m++) realorder[m] = s->d_pixelorder[m];
      for (m = 0; m < 16; m++) if (realorder[m] == c) break;
      for (n = m; n > 0; n--) realorder[n] = realorder[n - 1];
      realorder[0] = c;
      for (m = 0; m < 16; m++) if (realorder[m] == b) break;
      for (n = m; n > 0; n--) realorder[n] = realorder[n - 1];
      realorder[0] = b;
      for (m = 0; m < 16; m++) if (realorder[m] == a) break;
      for (n = m; n > 0; n--) realorder[n] = realorder[n - 1];
      realorder[0] = a;

      for (bit = 0; bit < 4; bit++) {
        unsigned prob = probability(s, con);
        unsigned flag_lps, shift, invertbit;
        if (s->d_val <= (uint8_t)(s->d_span - prob)) {
          s->d_span = (uint8_t)(s->d_span - prob);
          flag_lps = 0;
        } else {
          s->d_val = (uint8_t)(s->d_val - (s->d_span - (prob - 1)));
          s->d_span = (uint8_t)(prob - 1);
          flag_lps = 1;
        }
        shift = 0;
        while (s->d_span < 0x7f) {
          shift++;
          s->d_span = (uint8_t)((s->d_span << 1) + 1);
          s->d_val = (uint8_t)((s->d_val << 1) + (s->d_in >> 7));
          s->d_in <<= 1;
          if (--s->d_in_count == 0) {
            s->d_in = decomp_dataread(s, rom, romSize);
            s->d_in_count = 8;
          }
        }
        s->d_lps = (s->d_lps << 1) + (int32_t)flag_lps;
        invertbit = s->context[con].invert;
        s->d_inverts = (s->d_inverts << 1) + (int32_t)invertbit;
        if (flag_lps & toggle_invert(s, con)) s->context[con].invert ^= 1;
        if (flag_lps) s->context[con].index = next_lps(s, con);
        else if (shift) s->context[con].index = next_mps(s, con);
        con = mode2_context_table[con][flag_lps ^ invertbit] + (con == 1 ? refcon : 0);
      }
      b = realorder[(s->d_lps ^ s->d_inverts) & 0x0f];
      s->d_out1 = (s->d_out1 << 4) + ((s->d_out0 >> 28) & 0x0f);
      s->d_out0 = (s->d_out0 << 4) + (int32_t)b;
    }
    packed = morton_4x8((uint32_t)s->d_out0);
    decomp_write(s, (uint8_t)(packed >> 24));
    decomp_write(s, (uint8_t)(packed >> 16));
    s->d_bitplanebuffer[s->d_buffer_index++] = (uint8_t)(packed >> 8);
    s->d_bitplanebuffer[s->d_buffer_index++] = (uint8_t)(packed >> 0);
    if (s->d_buffer_index == 16) {
      unsigned i;
      for (i = 0; i < 16; i++) decomp_write(s, s->d_bitplanebuffer[i]);
      s->d_buffer_index = 0;
    }
  }
}

static uint8_t decomp_read(Spc7110 *s, const uint8_t *rom, uint32_t romSize) {
  uint8_t data;
  if (s->decomp_buffer_length == 0) {
    switch (s->decomp_mode) {
      case 0: mode0(s, false, rom, romSize); break;
      case 1: mode1(s, false, rom, romSize); break;
      case 2: mode2(s, false, rom, romSize); break;
      default: return 0x00;
    }
  }
  data = s->decomp_buffer[s->decomp_buffer_rdoffset++];
  s->decomp_buffer_rdoffset &= SPC7110_DECOMP_BUFFER_SIZE - 1;
  s->decomp_buffer_length--;
  return data;
}

static void decomp_init(Spc7110 *s, unsigned mode, unsigned offset, unsigned index,
                        const uint8_t *rom, uint32_t romSize) {
  unsigned i;
  s->decomp_mode = mode;
  s->decomp_offset = offset;
  s->decomp_buffer_rdoffset = 0;
  s->decomp_buffer_wroffset = 0;
  s->decomp_buffer_length = 0;
  for (i = 0; i < 32; i++) {
    s->context[i].index = 0;
    s->context[i].invert = 0;
  }
  switch (s->decomp_mode) {
    case 0: mode0(s, true, rom, romSize); break;
    case 1: mode1(s, true, rom, romSize); break;
    case 2: mode2(s, true, rom, romSize); break;
  }
  while (index--) {
#ifdef TARGET_GNW
    if ((index & 0x3ff) == 0) wdog_refresh();
#endif
    decomp_read(s, rom, romSize);
  }
}

static void decomp_reset(Spc7110 *s) {
  s->decomp_mode = 3;
  s->decomp_buffer_rdoffset = 0;
  s->decomp_buffer_wroffset = 0;
  s->decomp_buffer_length = 0;
}

/* ========== Data-port helpers ========== */

static uint32_t data_pointer(const Spc7110 *s) {
  return (uint32_t)s->r4811 | ((uint32_t)s->r4812 << 8) | ((uint32_t)s->r4813 << 16);
}
static uint32_t data_adjust(const Spc7110 *s) {
  return (uint32_t)s->r4814 | ((uint32_t)s->r4815 << 8);
}
static uint32_t data_increment(const Spc7110 *s) {
  return (uint32_t)s->r4816 | ((uint32_t)s->r4817 << 8);
}
static void set_data_pointer(Spc7110 *s, uint32_t addr) {
  s->r4811 = (uint8_t)addr;
  s->r4812 = (uint8_t)(addr >> 8);
  s->r4813 = (uint8_t)(addr >> 16);
}
static void set_data_adjust(Spc7110 *s, uint32_t addr) {
  s->r4814 = (uint8_t)addr;
  s->r4815 = (uint8_t)(addr >> 8);
}

/* ========== Lifecycle ========== */

Spc7110 *spc7110_alloc(void) {
  if (!morton_init()) return NULL;
#ifdef TARGET_GNW
  return (Spc7110 *)ram_calloc(1, sizeof(Spc7110));
#else
  return (Spc7110 *)snes_zalloc(sizeof(Spc7110));
#endif
}

uint32_t spc7110_size(void) {
  return (uint32_t)sizeof(Spc7110);
}

/* RTC-4513 power-on: 00:00:00 01/01/00 weekday 0, CR0=STOP, CR1=$F, CR2=$6.
 * Tengai Makyou Zero's self-test compares a dump against this exact image. */
static const uint8_t rtc_poweron[16] = {
  0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0x0f, 0x06
};
static const uint8_t rtc_month_days[12] = {
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static unsigned rtc_get2(const Spc7110 *s, int ones, int tens) {
  return (unsigned)(s->rtc_ram[ones] & 15) + (unsigned)(s->rtc_ram[tens] & 15) * 10u;
}

static void rtc_set2(Spc7110 *s, int ones, int tens, unsigned v) {
  s->rtc_ram[ones] = (uint8_t)(v % 10u);
  s->rtc_ram[tens] = (uint8_t)(v / 10u);
}

static void rtc_add_seconds(Spc7110 *s, unsigned add) {
  unsigned second = rtc_get2(s, 0, 1);
  unsigned minute = rtc_get2(s, 2, 3);
  unsigned hour = rtc_get2(s, 4, 5);
  unsigned day = rtc_get2(s, 6, 7);
  unsigned month = rtc_get2(s, 8, 9);
  unsigned year = rtc_get2(s, 10, 11);
  unsigned weekday = s->rtc_ram[12] & 7;
  if (month < 1u) month = 1;
  if (day < 1u) day = 1;
  hour %= 24u; /* strip AM/PM flag in the tens nibble if present */
  second += add;
  while (second >= 60u) {
    second -= 60u;
    minute++;
    if (minute < 60u) continue;
    minute = 0;
    hour++;
    if (hour < 24u) continue;
    hour = 0;
    day++;
    weekday = (weekday + 1u) % 7u;
    {
      unsigned md = rtc_month_days[(month - 1u) % 12u];
      if (md == 28u) {
        unsigned y = year + (year >= 90u ? 1900u : 2000u);
        if ((y % 4u) == 0u && ((y % 100u) != 0u || (y % 400u) == 0u)) md++;
      }
      if (day <= md) continue;
    }
    day = 1;
    month++;
    if (month <= 12u) continue;
    month = 1;
    year = (year + 1u) % 100u;
  }
  rtc_set2(s, 0, 1, second);
  rtc_set2(s, 2, 3, minute);
  rtc_set2(s, 4, 5, hour);
  rtc_set2(s, 6, 7, day);
  rtc_set2(s, 8, 9, month);
  rtc_set2(s, 10, 11, year);
  s->rtc_ram[12] = (uint8_t)(weekday % 7u);
}

void spc7110_reset(Spc7110 *s) {
  memset(s, 0, sizeof(*s));
  s->version = SPC7110_VERSION;
  decomp_reset(s);
  s->r4831 = 0;
  s->dx_offset = 0x100000u;
  s->r4832 = 1;
  s->ex_offset = 0x200000u;
  s->r4833 = 2;
  s->fx_offset = 0x300000u;
  memcpy(s->rtc_ram, rtc_poweron, sizeof(rtc_poweron));
}

void spc7110_tick(Spc7110 *s) {
  /* CR0 bit 0 and CR2 bits 0-1 stop the oscillator (Epson RTC-4513). */
  if ((s->rtc_ram[13] & 1) || (s->rtc_ram[15] & 3)) return;
  s->rtc_subsec++;
  if (s->rtc_subsec >= 60) {
    s->rtc_subsec = 0;
    rtc_add_seconds(s, 1);
  }
}

/* ========== MMIO ========== */

uint8_t spc7110_mmio_read(Spc7110 *s, uint16_t addr, const uint8_t *rom, uint32_t romSize) {
#ifdef HOST_BUILD
  if (host_spc7110_log()) {
    static unsigned n;
    if (n < 64) {
      fprintf(stderr, "spc7110: r %04x (#%u)\n", addr, n);
      n++;
    }
  }
#endif
  switch (addr) {
    case 0x4800: {
      uint16_t counter = (uint16_t)(s->r4809 | (s->r480a << 8));
      counter--;
      s->r4809 = (uint8_t)counter;
      s->r480a = (uint8_t)(counter >> 8);
      return decomp_read(s, rom, romSize);
    }
    case 0x4801: return s->r4801;
    case 0x4802: return s->r4802;
    case 0x4803: return s->r4803;
    case 0x4804: return s->r4804;
    case 0x4805: return s->r4805;
    case 0x4806: return s->r4806;
    case 0x4807: return s->r4807;
    case 0x4808: return s->r4808;
    case 0x4809: return s->r4809;
    case 0x480a: return s->r480a;
    case 0x480b: return s->r480b;
    case 0x480c: {
      uint8_t status = s->r480c;
      s->r480c &= 0x7f;
      return status;
    }

    case 0x4810: {
      uint32_t addr24, adjust, adjustaddr, increment;
      uint8_t data;
      if (s->r481x != 0x07) return 0x00;
      addr24 = data_pointer(s);
      adjust = data_adjust(s);
      if (s->r4818 & 8) adjust = (uint32_t)(int32_t)(int16_t)adjust;
      adjustaddr = addr24;
      if (s->r4818 & 2) {
        adjustaddr += adjust;
        set_data_adjust(s, adjust + 1);
      }
      data = rom_get(rom, romSize, datarom_addr(adjustaddr, romSize));
      if (!(s->r4818 & 2)) {
        increment = (s->r4818 & 1) ? data_increment(s) : 1;
        if (s->r4818 & 4) increment = (uint32_t)(int32_t)(int16_t)increment;
        if ((s->r4818 & 16) == 0) set_data_pointer(s, addr24 + increment);
        else set_data_adjust(s, adjust + increment);
      }
      return data;
    }
    case 0x4811: return s->r4811;
    case 0x4812: return s->r4812;
    case 0x4813: return s->r4813;
    case 0x4814: return s->r4814;
    case 0x4815: return s->r4815;
    case 0x4816: return s->r4816;
    case 0x4817: return s->r4817;
    case 0x4818: return s->r4818;
    case 0x481a: {
      uint32_t addr24, adjust;
      uint8_t data;
      if (s->r481x != 0x07) return 0x00;
      addr24 = data_pointer(s);
      adjust = data_adjust(s);
      if (s->r4818 & 8) adjust = (uint32_t)(int32_t)(int16_t)adjust;
      data = rom_get(rom, romSize, datarom_addr(addr24 + adjust, romSize));
      if ((s->r4818 & 0x60) == 0x60) {
        if ((s->r4818 & 16) == 0) set_data_pointer(s, addr24 + adjust);
        else set_data_adjust(s, adjust + adjust);
      }
      return data;
    }

    case 0x4820: return s->r4820;
    case 0x4821: return s->r4821;
    case 0x4822: return s->r4822;
    case 0x4823: return s->r4823;
    case 0x4824: return s->r4824;
    case 0x4825: return s->r4825;
    case 0x4826: return s->r4826;
    case 0x4827: return s->r4827;
    case 0x4828: return s->r4828;
    case 0x4829: return s->r4829;
    case 0x482a: return s->r482a;
    case 0x482b: return s->r482b;
    case 0x482c: return s->r482c;
    case 0x482d: return s->r482d;
    case 0x482e: return s->r482e;
    case 0x482f: {
      uint8_t status = s->r482f;
      s->r482f &= 0x7f;
      return status;
    }

    case 0x4830: return s->r4830;
    case 0x4831: return s->r4831;
    case 0x4832: return s->r4832;
    case 0x4833: return s->r4833;
    case 0x4834: return s->r4834;

    case 0x4840: return s->r4840;
    case 0x4841: {
      uint8_t data;
      if (s->rtc_state == RTCS_Inactive || s->rtc_state == RTCS_ModeSelect) return 0x00;
      s->r4842 = 0x80;
      data = s->rtc_ram[s->rtc_index & 15];
      s->rtc_index = (s->rtc_index + 1) & 15;
      return data;
    }
    case 0x4842: {
      uint8_t status = s->r4842;
      s->r4842 &= 0x7f;
      return status;
    }
  }
  return 0;
}

static void apply_adjust_write(Spc7110 *s) {
  uint32_t increment;
  if (!s->r4814_latch || !s->r4815_latch) return;
  if (!(s->r4818 & 2)) return;
  if (s->r4818 & 0x10) return;
  if ((s->r4818 & 0x60) == 0x20) {
    increment = data_adjust(s) & 0xff;
    if (s->r4818 & 8) increment = (uint32_t)(int32_t)(int8_t)increment;
    set_data_pointer(s, data_pointer(s) + increment);
  } else if ((s->r4818 & 0x60) == 0x40) {
    increment = data_adjust(s);
    if (s->r4818 & 8) increment = (uint32_t)(int32_t)(int16_t)increment;
    set_data_pointer(s, data_pointer(s) + increment);
  }
}

void spc7110_mmio_write(Spc7110 *s, uint16_t addr, uint8_t val, const uint8_t *rom, uint32_t romSize) {
#ifdef HOST_BUILD
  if (host_spc7110_log()) {
    static unsigned n;
    if (n < 64) {
      fprintf(stderr, "spc7110: w %04x=%02x (#%u)\n", addr, val, n);
      n++;
    }
  }
#endif
  switch (addr) {
    case 0x4801: s->r4801 = val; break;
    case 0x4802: s->r4802 = val; break;
    case 0x4803: s->r4803 = val; break;
    case 0x4804: s->r4804 = val; break;
    case 0x4805: s->r4805 = val; break;
    case 0x4806: {
      uint32_t table, index, e, offset, skip;
      uint8_t mode;
      s->r4806 = val;
      table = (uint32_t)s->r4801 | ((uint32_t)s->r4802 << 8) | ((uint32_t)s->r4803 << 16);
      index = (uint32_t)s->r4804 << 2;
      e = datarom_addr(table + index, romSize);
      mode = rom_get(rom, romSize, e + 0);
      offset = ((uint32_t)rom_get(rom, romSize, e + 1) << 16)
             | ((uint32_t)rom_get(rom, romSize, e + 2) << 8)
             | ((uint32_t)rom_get(rom, romSize, e + 3) << 0);
      skip = ((uint32_t)s->r4805 | ((uint32_t)s->r4806 << 8)) << (mode & 3);
#ifdef HOST_BUILD
      if (host_spc7110_log())
        fprintf(stderr, "spc7110: decomp mode=%u offset=%06x skip=%u table=%06x idx=%u\n",
                mode, (unsigned)offset, (unsigned)skip, (unsigned)table, s->r4804);
#endif
      decomp_init(s, mode, offset, skip, rom, romSize);
      s->r480c = 0x80;
    } break;
    case 0x4807: s->r4807 = val; break;
    case 0x4808: s->r4808 = val; break;
    case 0x4809: s->r4809 = val; break;
    case 0x480a: s->r480a = val; break;
    case 0x480b: s->r480b = val; break;

    case 0x4811: s->r4811 = val; s->r481x |= 0x01; break;
    case 0x4812: s->r4812 = val; s->r481x |= 0x02; break;
    case 0x4813: s->r4813 = val; s->r481x |= 0x04; break;
    case 0x4814:
      s->r4814 = val;
      s->r4814_latch = 1;
      apply_adjust_write(s);
      break;
    case 0x4815:
      s->r4815 = val;
      s->r4815_latch = 1;
      apply_adjust_write(s);
      break;
    case 0x4816: s->r4816 = val; break;
    case 0x4817: s->r4817 = val; break;
    case 0x4818:
      if (s->r481x != 0x07) break;
      s->r4818 = val;
      s->r4814_latch = s->r4815_latch = 0;
      break;

    case 0x4820: s->r4820 = val; break;
    case 0x4821: s->r4821 = val; break;
    case 0x4822: s->r4822 = val; break;
    case 0x4823: s->r4823 = val; break;
    case 0x4824: s->r4824 = val; break;
    case 0x4825: {
      uint32_t result;
      s->r4825 = val;
      if (s->r482e & 1) {
        int16_t r0 = (int16_t)(s->r4824 | (s->r4825 << 8));
        int16_t r1 = (int16_t)(s->r4820 | (s->r4821 << 8));
        result = (uint32_t)(int32_t)(r0 * r1);
      } else {
        uint16_t r0 = (uint16_t)(s->r4824 | (s->r4825 << 8));
        uint16_t r1 = (uint16_t)(s->r4820 | (s->r4821 << 8));
        result = (uint32_t)r0 * (uint32_t)r1;
      }
      s->r4828 = (uint8_t)result;
      s->r4829 = (uint8_t)(result >> 8);
      s->r482a = (uint8_t)(result >> 16);
      s->r482b = (uint8_t)(result >> 24);
      s->r482f = 0x80;
    } break;
    case 0x4826: s->r4826 = val; break;
    case 0x4827: {
      uint32_t quotient;
      uint16_t remainder;
      s->r4827 = val;
      if (s->r482e & 1) {
        int32_t dividend = (int32_t)((uint32_t)s->r4820 | ((uint32_t)s->r4821 << 8)
                         | ((uint32_t)s->r4822 << 16) | ((uint32_t)s->r4823 << 24));
        int16_t divisor = (int16_t)(s->r4826 | (s->r4827 << 8));
        if (divisor) {
          quotient = (uint32_t)(dividend / divisor);
          remainder = (uint16_t)(dividend % divisor);
        } else {
          quotient = 0;
          remainder = (uint16_t)(dividend & 0xffff);
        }
      } else {
        uint32_t dividend = (uint32_t)s->r4820 | ((uint32_t)s->r4821 << 8)
                          | ((uint32_t)s->r4822 << 16) | ((uint32_t)s->r4823 << 24);
        uint16_t divisor = (uint16_t)(s->r4826 | (s->r4827 << 8));
        if (divisor) {
          quotient = dividend / divisor;
          remainder = (uint16_t)(dividend % divisor);
        } else {
          quotient = 0;
          remainder = (uint16_t)(dividend & 0xffff);
        }
      }
      s->r4828 = (uint8_t)quotient;
      s->r4829 = (uint8_t)(quotient >> 8);
      s->r482a = (uint8_t)(quotient >> 16);
      s->r482b = (uint8_t)(quotient >> 24);
      s->r482c = (uint8_t)remainder;
      s->r482d = (uint8_t)(remainder >> 8);
      s->r482f = 0x80;
    } break;
    case 0x482e:
      s->r4820 = s->r4821 = s->r4822 = s->r4823 = 0;
      s->r4824 = s->r4825 = s->r4826 = s->r4827 = 0;
      s->r4828 = s->r4829 = s->r482a = s->r482b = 0;
      s->r482c = s->r482d = 0;
      s->r482e = val;
      break;

    case 0x4830: s->r4830 = val & 0x87; break;
    case 0x4831:
      s->r4831 = val;
      s->dx_offset = datarom_addr((uint32_t)(val & 7) * 0x100000u, romSize);
      break;
    case 0x4832:
      s->r4832 = val;
      s->ex_offset = datarom_addr((uint32_t)(val & 7) * 0x100000u, romSize);
      break;
    case 0x4833:
      s->r4833 = val;
      s->fx_offset = datarom_addr((uint32_t)(val & 7) * 0x100000u, romSize);
      break;
    case 0x4834: s->r4834 = val; break;

    case 0x4840:
      s->r4840 = val;
      if (!(s->r4840 & 1)) {
        s->rtc_state = RTCS_Inactive;
      } else {
        s->r4842 = 0x80;
        s->rtc_state = RTCS_ModeSelect;
      }
      break;
    case 0x4841:
      s->r4841 = val;
      if (s->rtc_state == RTCS_ModeSelect) {
        if (val == 0x03 || val == 0x0c) {
          s->r4842 = 0x80;
          s->rtc_state = RTCS_IndexSelect;
          s->rtc_mode = val;
          s->rtc_index = 0;
        }
      } else if (s->rtc_state == RTCS_IndexSelect) {
        s->r4842 = 0x80;
        s->rtc_index = val & 15;
        if (s->rtc_mode == 0x03) s->rtc_state = RTCS_Write;
      } else if (s->rtc_state == RTCS_Write) {
        s->r4842 = 0x80;
        /* CR0: bit1 = +1 second, bit3 = 30-second adjust (RTC-4513). */
        if (s->rtc_index == 13) {
          if (val & 2) rtc_add_seconds(s, 1);
          if (val & 8) {
            unsigned second = rtc_get2(s, 0, 1);
            rtc_set2(s, 0, 1, 0);
            if (second >= 30u) rtc_add_seconds(s, 60);
          }
        }
        /* CR2: rising STOP/HOLD bits freeze and may clear seconds. */
        if (s->rtc_index == 15) {
          if ((val & 1) && !(s->rtc_ram[15] & 1))
            rtc_set2(s, 0, 1, 0);
        }
        s->rtc_ram[s->rtc_index & 15] = val & 15;
        s->rtc_index = (s->rtc_index + 1) & 15;
      }
      break;
  }
}

uint8_t spc7110_read(Spc7110 *s, uint32_t addr, const uint8_t *rom, uint32_t romSize) {
  uint8_t bank = (uint8_t)(addr >> 16);
  uint16_t ofs = (uint16_t)addr;

  if (bank == 0x50)
    return spc7110_mmio_read(s, 0x4800, rom, romSize);

  /* 7 MB fan translation: last 1 MB is expansion ROM at $40-$4F (HiROM). */
  if (bank >= 0x40 && bank <= 0x4f) {
    if (romSize > 0x600000u) {
      uint32_t i = 0x600000u + (((uint32_t)(bank & 0x0f) << 16) | ofs);
      return rom_get(rom, romSize, i);
    }
    return 0;
  }

  if (bank >= 0xc0 && bank <= 0xcf) {
    uint32_t i = ((uint32_t)(bank - 0xc0) << 16) | ofs;
    return rom_get(rom, romSize, i);
  }
  if (bank >= 0xd0 && bank <= 0xdf)
    return rom_get(rom, romSize, s->dx_offset + (((uint32_t)(bank & 0x0f) << 16) | ofs));
  if (bank >= 0xe0 && bank <= 0xef)
    return rom_get(rom, romSize, s->ex_offset + (((uint32_t)(bank & 0x0f) << 16) | ofs));
  if (bank >= 0xf0)
    return rom_get(rom, romSize, s->fx_offset + (((uint32_t)(bank & 0x0f) << 16) | ofs));
  return 0;
}
