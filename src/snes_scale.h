/* FULL and FIT presents of the 256x224 SNES picture into the 320x240 panel.
 *
 * Output pixel for pixel identical to the nearest-neighbour loops they
 * replace (column x reads source (x * 256) / width, row y reads source
 * (y * 224) / 240), which divided once per pixel and stored one 16-bit pixel
 * at a time into the uncached LCD buffer: ~560,000 cycles a presented frame
 * on the device. Here every store is a 32-bit word and no pixel divides:
 *
 *   FULL, 256 -> 320: output x = 5k + j reads source 4k + {0,0,1,2,3}[j], so
 *     8 source pixels (4 words) give 10 output pixels (5 words):
 *       s0 s0 | s1 s2 | s3 s4 | s4 s5 | s6 s7
 *   FIT, 256 -> 274 between 23-pixel black borders: a column table built
 *     once; the borders are odd, so the two pairs at the image edges mix a
 *     border pixel with an image pixel.
 *
 * `frame` is the 320-wide SNES frame buffer and `top`/`left` the picture's
 * position in it (the margins it is centred with); `dst` the 320x240
 * panel buffer, 4-byte aligned. */
#pragma once
#include <stdint.h>

/* the frame and the panel are uint16_t arrays read and written here as
 * words: may_alias, or -O2 and up (strict aliasing; the build only silences
 * the warning) may reorder these accesses against the 16-bit ones */
typedef uint32_t __attribute__((may_alias)) snes_scale_u32;

#define SNES_SCALE_SRC_W   256
#define SNES_SCALE_SRC_H   224
#define SNES_SCALE_DST_W   320
#define SNES_SCALE_DST_H   240
#define SNES_SCALE_FIT_W   ((SNES_SCALE_SRC_W * SNES_SCALE_DST_H + SNES_SCALE_SRC_H / 2) / SNES_SCALE_SRC_H)  /* 274 */
#define SNES_SCALE_FIT_PAD ((SNES_SCALE_DST_W - SNES_SCALE_FIT_W) / 2)                                      /* 23 */

static inline const uint16_t *snes_scale_src_row(const uint16_t *frame, int y, int top, int left)
{
  int sy = (y * SNES_SCALE_SRC_H) / SNES_SCALE_DST_H;
  return frame + (sy + top) * SNES_SCALE_DST_W + left;
}

static void snes_scale_full(uint16_t *dst, const uint16_t *frame, int top, int left)
{
  for (int y = 0; y < SNES_SCALE_DST_H; y++) {
    const snes_scale_u32 *s = (const snes_scale_u32 *)(const void *)snes_scale_src_row(frame, y, top, left);
    snes_scale_u32 *d = (snes_scale_u32 *)(void *)(dst + y * SNES_SCALE_DST_W);
    for (int i = 0; i < SNES_SCALE_SRC_W / 8; i++, s += 4, d += 5) {
      uint32_t a = s[0], b = s[1], c = s[2], e = s[3];   /* s0s1 s2s3 s4s5 s6s7 */
      d[0] = (a & 0xffffu) * 0x10001u;                   /* s0 s0 */
      d[1] = (a >> 16) | (b << 16);                      /* s1 s2 */
      d[2] = (b >> 16) | (c << 16);                      /* s3 s4 */
      d[3] = c;                                          /* s4 s5 */
      d[4] = e;                                          /* s6 s7 */
    }
  }
}

/* source column of each output column of the FIT picture */
static uint8_t snes_scale_fit_col[SNES_SCALE_FIT_W];
static int snes_scale_fit_ready;

static void snes_scale_fit(uint16_t *dst, const uint16_t *frame, int top, int left)
{
  enum { W = SNES_SCALE_FIT_W, PAD = SNES_SCALE_FIT_PAD };
  if (!snes_scale_fit_ready) {
    for (int x = 0; x < W; x++) snes_scale_fit_col[x] = (uint8_t)((x * SNES_SCALE_SRC_W) / W);
    snes_scale_fit_ready = 1;
  }
  /* output pairs: [0, PAD/2) black, PAD/2 = black | image 0 (PAD is odd),
   * then image pairs, then image W-1 | black, then black */
  for (int y = 0; y < SNES_SCALE_DST_H; y++) {
    const uint16_t *s = snes_scale_src_row(frame, y, top, left);
    snes_scale_u32 *d = (snes_scale_u32 *)(void *)(dst + y * SNES_SCALE_DST_W);
    int p = 0;
    for (; p < PAD / 2; p++) d[p] = 0;
    d[p++] = (uint32_t)s[snes_scale_fit_col[0]] << 16;
    const uint8_t *col = snes_scale_fit_col + 1;
    for (int k = 0; k < (W - 2) / 2; k++, col += 2)
      d[p++] = (uint32_t)s[col[0]] | ((uint32_t)s[col[1]] << 16);
    d[p++] = s[snes_scale_fit_col[W - 1]];
    for (; p < SNES_SCALE_DST_W / 2; p++) d[p] = 0;
  }
}
