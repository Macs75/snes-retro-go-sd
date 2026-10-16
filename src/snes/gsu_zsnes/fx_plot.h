/* PLOT's pixel write, from fx_ops.h: one copy for the interpreter (fx_plot in
 * gsu_core.c) and the block code (gsu_blocks.c). Textual include; the includer
 * declares sfxgameram, SfxRamMask, SCBRrel, sfxclineloc, SfxCOLR and
 * fxbit01pcal..67pcal (and unaligned.h's ld16u / st16u). */
#ifndef FX_PLOT_H
#define FX_PLOT_H

/* The tile holding pixel (x, y) = (index >> 8, index & 0xFF) in screen mode
 * `mode` (see fx_lineloc), or 0xFFFFFFFF below the screen. The GSU numbers the
 * bitmap modes' tiles column by column; object mode interleaves the high bits
 * of x and y. This is what ZSNES's 1 MB of line-location tables held. */
static inline __attribute__((always_inline)) u4 fx_tile(zreg const mode, u4 const index)
{
    u4 const x = (index >> 8) & 0xFFu, y = index & 0xFFu;
    if (mode == 3)
        return ((x & 0x80u) << 1) + ((y & 0x80u) << 2) + ((x & 0x78u) >> 3) + ((y & 0x78u) << 1);
    u4 const rows = mode == 0 ? 16u : mode == 1 ? 20u : 24u;
    if (y >= rows * 8u)
        return 0xFFFFFFFFu;
    return (x >> 3) * rows + (y >> 3);
}

enum { FX_PLOT_2BPP,
    FX_PLOT_4BPP,
    FX_PLOT_8BPP };

static inline void* fx_plane(zreg const addr, u4 const n)
{
    return (void*)(uintptr_t)(addr + n * 16u);
}

/* Write one pixel: `bit` is the pixel's bit in both bytes of each bitplane
   pair (0x0101 << (7 - x&7)). The dithered form just uses the other pair of
   colour lookups, which COLOR loaded with the high nibble. */
static inline __attribute__((always_inline))
void fx_drawpix(zreg const addr, u4 const bit, int const depth, int const dither)
{
    u4 const b01 = dither ? fxbit45pcal : fxbit01pcal;
    u4 const b23 = dither ? fxbit67pcal : fxbit23pcal;
    u4 const b45 = dither ? fxbit01pcal : fxbit45pcal;
    u4 const b67 = dither ? fxbit23pcal : fxbit67pcal;
    u4 const planes = depth == FX_PLOT_2BPP ? 1 : depth == FX_PLOT_4BPP ? 2
                                                                        : 4;
    u4 const col[4] = { b01, b23, b45, b67 };

    /* One 16-bit read-modify-write a plane pair: the asm cleared the bit in
       every plane, then ORed the colour in, in two passes of 32-bit accesses;
       the planes are 16 bytes apart and only the low 16 bits ever changed (on
       the last tile of the Game Pak RAM the upper two ran past its end).
       The address is always even (SCBR * 1024 + tile << 4.. + 2 * (y & 7),
       masked to the RAM size), hence the aligned accesses. */
    for (u4 i = 0; i < planes; i++) {
        u2 const v = ld16a(fx_plane(addr, i));
        st16a(fx_plane(addr, i), (u2)((v & ~bit) | (col[i] & bit)));
    }
}

/* The pixel PLOT draws at (R1, R2), in the colour register; the caller
   advances R1. Skips pixels below the screen and, with zcheck, colour 0. */
static inline __attribute__((always_inline))
void fx_plot_pixel(u4 const r1, u4 const r2, int const depth, u4 const zmask, int const zcheck, int const dither)
{
    u4 const shift = depth == FX_PLOT_2BPP ? 4 : depth == FX_PLOT_4BPP ? 5
                                                                       : 6;
    u4 const index = (r2 & 0xFFFF00FFu) | ((r1 & 0xFFu) << 8);
    u4 const tile = fx_tile(sfxclineloc, index);
#ifdef SNES_PERF_GSU
    u4 const perf_t = snes_perf_now();
#endif

    if (tile != 0xFFFFFFFFu && (!zcheck || ((u1)SfxCOLR & zmask))) {
        zreg const addr = (uintptr_t)sfxgameram
            + (((uintptr_t)SCBRrel - (uintptr_t)sfxgameram + (tile << shift) + ((index & 7u) * 2u))
                  & SfxRamMask);
        u4 const bit = 0x0101u << ((r1 & 7u) ^ 7u);   /* = ~fxxand[x] */

        fx_drawpix(addr, bit, depth, dither && (((r1 ^ r2) & 1u) != 0));
    }
#ifdef HOST_BUILD
    { extern unsigned long gsu_plotstat[64]; gsu_plotstat[(depth << 4) | (zcheck << 3) | (dither << 2) | (u4)sfxclineloc]++; }
#endif
#ifdef SNES_PERF_GSU
    snes_perf_gsu_plot += snes_perf_now() - perf_t;
    snes_perf_gsu_plots++;
#endif
}

#endif
