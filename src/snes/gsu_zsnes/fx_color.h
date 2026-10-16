/* COLOR / GETC's colour-register update, from fx_ops.h: one copy for the
 * interpreter (gsu_core.c) and the block code (gsu_blocks.c). Textual include;
 * the includer declares SfxCOLR, SfxPOR, fxbit01..67 and fxbit01pcal..67pcal. */
#ifndef FX_COLOR_H
#define FX_COLOR_H

/* Setting the colour register also refreshes the four per-bitplane lookups the
   plot routines use, but only when the value actually changes. */
static inline void fx_set_color(u1 const col)
{
    if ((u1)SfxCOLR == col) {
        return;
    }
    *(u1*)&SfxCOLR = col; /* byte store: the upper three are kept */
    fxbit01pcal = fxbit01[col];
    fxbit23pcal = fxbit23[col];
    fxbit45pcal = fxbit45[col];
    fxbit67pcal = fxbit67[col];
}

/* POR bit 2 duplicates the high nibble down; bit 3 keeps the colour register's
   existing high nibble instead. */
static inline void fx_color(u4 const raw)
{
    u1 col = (u1)raw;

    if (SfxPOR & 0x04u) {
        col = (u1)((col & 0xF0u) | (col >> 4));
    }
    if (SfxPOR & 0x08u) {
        col = (u1)((col & 0x0Fu) | ((u1)SfxCOLR & 0xF0u));
    }
    fx_set_color(col);
}

#endif
