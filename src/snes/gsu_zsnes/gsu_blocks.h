/* Per-game Super FX block code: the table MainLoop (fx_ops.h) looks blocks up
 * in, and its selection. The blocks and their tables are in gsu_blocks.c. */
#ifndef GSU_BLOCKS_H
#define GSU_BLOCKS_H

#include "gsu_types.h"

/* key = program bank (7 bits) << 24 | PC << 8 | prefetched opcode, 0 = empty;
   fn(N, entry) runs the chain from that entry, see gsu_blk_run in fx_ops.h */
struct gsu_blk {
    u4 key;
    u4 entry;
    u4 (*fn)(s4, u4);
};

extern struct gsu_blk const *gsu_blk_tab; /* NULL: no blocks for this game */
extern u4 gsu_blk_mask;                   /* table size - 1 */

/* Point gsu_blk_tab at the table of the loaded game, if its ROM matches. Needs
   the bank tables (InitFxTables) filled. */
void gsu_blk_select(void);

#endif
