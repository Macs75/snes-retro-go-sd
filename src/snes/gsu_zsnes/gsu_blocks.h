/* Per-game Super FX block code: the table MainLoop (fx_ops.h) looks blocks up
 * in, and its selection (gsu_blocks.c). Each game's blocks and table are in
 * gsu_blocks_<game>.c. */
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

/* A game's block code, one per gsu_blocks_<game>.c: the header its overlay
   starts with on the device (snes_core.ld, the GSGM records), which the loader
   picks by cart id and gsu_blk_select enables only if the ROM bytes the chains
   were compiled from hash to `hash`. */
#define GSU_BLK_GAME_MAGIC 0x4D475347u   /* "GSGM" */
struct gsu_blk_game {
    u4 magic;
    u4 id;                       /* gsu_blk_cart_id of the cart it was made for */
    u4 hash;                     /* FNV-1a of the dep runs' ROM bytes */
    u4 const *dep;               /* bank << 24 | addr << 8 | count */
    u4 ndep;
    struct gsu_blk const *tab;
    u4 mask;                     /* table size - 1 */
};

/* The cart a game's blocks are for: FNV-1a of the 32-byte internal header
   ($7FC0, LoROM, as every Super FX cart), copier header already removed.
   tools/gen_gsu_blocks.py computes the same. */
static inline u4 gsu_blk_cart_id(u1 const *const rom, u4 const len)
{
    u4 h = 0x811C9DC5u;
    if (len < 0x7FE0u) return 0;
    for (u4 i = 0x7FC0u; i != 0x7FE0u; ++i) h = (h ^ rom[i]) * 0x01000193u;
    return h;
}

/* The game whose block code a cart takes: its own, or that of a cart whose
   Super FX code it has byte for byte at every run the blocks were made from
   (tools/gsu_blk_check.py). gsu_blk_select still checks those bytes. */
static inline u4 gsu_blk_game_id(u4 const cart_id)
{
    switch (cart_id) {
    case 0xF7A389D0u: return 0x3902A841u;   /* Starwing (Europe): Star Fox (USA) */
    }
    return cart_id;
}

extern struct gsu_blk const *gsu_blk_tab; /* NULL: no blocks for this game */
extern u4 gsu_blk_mask;                   /* table size - 1 */

/* One bit per (program bank, PC) hash, set for every key in the table: a clear
   bit means no chain starts there, so MainLoop skips the table probe. That is
   what every interpreted instruction asks, and a miss in the table costs a
   multiply and two or more probes. A set bit only means "maybe". */
#define GSU_BLK_FILT_BITS 4096u
extern u4 gsu_blk_filt[GSU_BLK_FILT_BITS / 32u];
static inline __attribute__((always_inline)) u4 gsu_blk_filt_bit(u4 const bank, u4 const pc)
{
    return (pc ^ (bank << 7)) & (GSU_BLK_FILT_BITS - 1u);
}

/* Point gsu_blk_tab at the table of the loaded game, if its ROM matches. Needs
   the bank tables (InitFxTables) filled. */
void gsu_blk_select(void);

#endif
