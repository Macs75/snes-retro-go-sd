#ifndef GSU_ENDMEM_H
#define GSU_ENDMEM_H

/* GSU scratch tables (was endmem.c's inline-asm block, GSU slice only, in
 * plain C). The one load-bearing fact: each dispatch table is four adjacent
 * 256-entry ALT sub-tables, so FxDispatch can index table[(ALT<<8)|op] across
 * all four. We make that a single [1024] and name the sub-regions. */

#include "gsu_types.h"

extern zreg FxTable[1024], FxTableb[1024], FxTablec[1024], FxTabled[1024];

#define FxTableA1   (FxTable  + 256)
#define FxTableA2   (FxTable  + 512)
#define FxTableA3   (FxTable  + 768)
#define FxTablebA1  (FxTableb + 256)
#define FxTablebA2  (FxTableb + 512)
#define FxTablebA3  (FxTableb + 768)
#define FxTablecA1  (FxTablec + 256)
#define FxTablecA2  (FxTablec + 512)
#define FxTablecA3  (FxTablec + 768)
#define FxTabledA1  (FxTabled + 256)
#define FxTabledA2  (FxTabled + 512)
#define FxTabledA3  (FxTabled + 768)

extern zreg SfxMemTable[256];
extern u4   SfxMemMask[256];
extern u4   fxxand[256], fxbit01[256], fxbit23[256], fxbit45[256], fxbit67[256];
extern zreg PLOTJmpa[64], PLOTJmpb[64];
extern zreg sfx128lineloc, sfx160lineloc, sfx192lineloc, sfxobjlineloc;

#endif
