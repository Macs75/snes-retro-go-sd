/* ============================================================================
 * gsu_blocks_game.h — what a game's generated block code is compiled with.
 *
 * Each gsu_blocks_<game>.c includes this, then its gsu_blocks_<game>.inc
 * (tools/gen_gsu_blocks.py), then GSU_BLK_GAME_DEFINE(<game>, <GAME>). On the
 * device that unit is the game's own overlay (snes_core.ld), loaded only for
 * its cart; on the host every game is linked and gsu_blocks.c lists them.
 * ========================================================================== */
#ifndef GSU_BLOCKS_GAME_H
#define GSU_BLOCKS_GAME_H

#include "gsu_types.h"
#include "gsu_endmem.h"
#include "gsu_blocks.h"

/* the GSU state the blocks touch (defined in gsu_core.c) */
extern u4 SfxR0[16], SfxSignZero, SfxCarry, SfxOverflow, SfxCBR, SfxCacheActive;
extern u4 SfxRAMBR, SfxROMBR, SfxnRamBanks, SfxPOR, SfxCOLR, SfxRamMask, SfxCROMMask;
extern u4 fxbit01pcal, fxbit23pcal, fxbit45pcal, fxbit67pcal;
extern zreg SfxRAMMem, SfxLastRamAdr, SfxRomBuffer, SfxCROM;
extern u4 FxSeamPC, FxSeamCX, FxCodeMask;
extern u1 const *FxCodeBase;
extern u1 *sfxgameram, *SCBRrel;
extern zreg sfxclineloc;

#ifdef SNES_PERF_GSU
#include "../../snes_perf.h"
#endif
#include "unaligned.h"
#include "fx_color.h"
#include "fx_plot.h"

/* The game's header: first in its overlay (section .gsu_game_hdr). */
#define GSU_BLK_GAME_DEFINE(name, NAME)                                              \
    __attribute__((section(".gsu_game_hdr"), used))                                  \
    struct gsu_blk_game const gsu_blk_game_##name = {                                \
        GSU_BLK_GAME_MAGIC, GSU_BLK_##NAME##_ID, GSU_BLK_##NAME##_HASH,              \
        gsu_blk_dep_##name, sizeof gsu_blk_dep_##name / sizeof gsu_blk_dep_##name[0], \
        gsu_blk_tab_##name, sizeof gsu_blk_tab_##name / sizeof gsu_blk_tab_##name[0] - 1 \
    };

#endif
