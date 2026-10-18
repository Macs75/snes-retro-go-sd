#ifndef GSU_UI_H
#define GSU_UI_H

/* Shim for the two ZSNES ui.h globals the GSU core needs, plus the region
 * flag. Set by the glue in gsu.c:
 *   romdata     = cart->rom          (the ROM image)
 *   sfxgameram  = cart->ram          (shared Game Pak RAM; was sfxramdata)
 */

#include "gsu_types.h"

extern u1 *romdata;
extern u4  romdatasize;
extern u4  SfxRamMask;
extern u1 *sfxgameram;
extern u1  romispal;      /* 0 = NTSC, 1 = PAL — scanline opcode budget */

#endif
