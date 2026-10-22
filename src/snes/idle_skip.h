/* Idle-loop skip: generic, found at run time, exact.
 *
 * Almost every SNES game finishes its frame early and then waits for the next
 * NMI in a small loop. Seven ROMs surveyed (attract, and the two real-play
 * recordings) spend 50-90% of their 65816 instructions in one. The emulator
 * runs those laps instruction by instruction, so the wait costs as much as
 * real work.
 *
 * FINDING THE LOOP. When the NMI is raised, the CPU is almost always inside the
 * wait loop: 98.4% of NMIs in Castlevania's real play, 93.3% in Zelda's. So at
 * every NMI the interrupted pc is examined, and if it lies in a loop that
 * passes the purity check it becomes the armed site. No table and no game
 * names: the site follows the game through menus, levels and play.
 *
 * PURITY, decided from the code bytes. The loop (head .. closing backward
 * branch, at most IDLE_MAX_LEN bytes, in ROM) may contain only instructions
 * from a whitelist that write nothing: loads, compares, BIT, AND/ORA/EOR,
 * register transfers and inc/dec, flag set/clear, branches. Their memory
 * operands use direct, absolute or long addressing only, and each must be WRAM
 * or ROM -- never an I/O register, whose value moves with time. Lengths and
 * operand addresses depend on M/X/E, DB and DP, so those are recorded at decode
 * time and must match whenever the loop is replayed.
 *
 * SKIPPING, proven per span. At an opcode boundary on the loop head the
 * registers and every polled WRAM byte are compared with the previous lap's.
 * Equal means the machine state recurred: nothing in the loop writes, no
 * interrupt or DMA can occur inside a span (checked), so every further lap of
 * this span is the same lap again, with the same charge. The rest of the span
 * is then advanced in whole laps and the last lap runs for real. A loop that
 * counts (DEX/BNE) never recurs, so it is never skipped -- only executed.
 *
 * WAI is the degenerate case: a waiting CPU is charged 6 dots per call until
 * an interrupt is pending, so the rest of the span is consumed in 6-dot steps.
 */
#ifndef SNES_IDLE_SKIP_H
#define SNES_IDLE_SKIP_H

#include <stdint.h>
#include <stdbool.h>

#include "snes.h"
#include "cpu.h"

#define IDLE_MAX_LEN  32
/* a read-set offset with this bit is an SA-1 bus address (I-RAM / BW-RAM),
 * read with sa1_peek; without it, an offset into Snes.ram */
#define IDLE_MEM_SA1 0x80000000u
#define IDLE_MAX_MEM   4

#define IDLE_MAX_STATES 16
#define IDLE_SA1_READS  3

/* The CPU state at one instruction boundary of the loop: a raw copy of Cpu
 * from `a` to `stopped`. Those 27 bytes are A X Y SP PC DP K DB, every flag
 * (E M X included) and the IRQ/NMI/WAI/STP state, contiguous in the struct
 * (asserted in idle_skip.c), so recurrence is a handful of word compares. */
#define IDLE_IMG_LEN 27
typedef struct {
  uint16_t pc;
  uint8_t  img[IDLE_IMG_LEN + 1];   /* at offset 2: img+14 (the flags) is word-aligned */
} __attribute__((aligned(4))) IdleState;

#define IDLE_SITES 12
#define IDLE_MAX_IDX 4

typedef struct {
  /* the site; head 0xffff with len 0 when the slot is empty */
  uint16_t head;          /* pc of the loop head                              */
  uint16_t len;           /* bytes from head to the closing branch's last byte */
  uint8_t  bank, bank_alt;
  /* the decode environment, and which parts of it the loop depends on */
  bool     e, mf, xf;
  bool     uses_db, uses_dp;
  uint8_t  db;
  uint16_t dp;
  /* reads: fixed WRAM bytes found at decode, and indexed operands whose
   * address the proof fixes (the loop never changes that index register) */
  uint8_t  nmem;
  uint32_t mem_off[IDLE_MAX_MEM];
  uint8_t  nidx;
  struct { uint8_t bank, reg, w, dpmode; uint16_t base; } idx[IDLE_MAX_IDX];
  bool     rdnmi;         /* reads $4210: idle only while the NMI flag is clear */
  /* reads $4212 (vblank, hblank, auto-joypad busy): idle only in a span over
   * which it holds, and holds what the proof read (hvb_val) */
  bool     hvbjoy;
  uint8_t  hvb_val;
  /* Super FX: the loop polls the GSU status word ($3030/$3031, one read).
   * It changes only when the GSU runs (line events, span boundaries), so the
   * proof snapshots the bits the polling instruction can observe and laps are
   * skipped while those bits hold (see idle_prove / idle_skip_laps). */
  bool     gsu_poll, gsu_bit;
  uint8_t  gsu_w;         /* bytes read */
  uint16_t gsu_pc, gsu_base, gsu_mask, gsu_snap;
  /* An LDA of the status followed by AND #imm keeps only the imm bits of it
   * beyond that AND, so the CPU state at the loop head depends on those bits
   * alone (gsu_and, 0 = no such AND). The GSU's own flags in the status move
   * every slice while it runs; with the head bits unchanged, whole laps from
   * the head are still the same laps (idle_skip_laps). */
  uint16_t gsu_and, gsu_hmask, gsu_hsnap;
  /* SA-1: the loop's instructions that read memory the SA-1 shares, and how
   * many (0x80 set: an indexed one, address known only at proof time, or more
   * than IDLE_SA1_READS). With 1..IDLE_SA1_READS, laps are skipped while the
   * SA-1 runs too (idle_sa1_awake_span in main_snes.c). */
  uint16_t sa1_pc[IDLE_SA1_READS];
  uint8_t  sa1_rd;
  bool     code_ram;      /* the loop's code is in WRAM: compared every span    */
  uint8_t  ncode;
  uint8_t  code[IDLE_MAX_LEN];
  uint32_t used;          /* frame it was last made active (LRU)                */
  /* the proven lap: the CPU state at each of its instruction boundaries, the
   * WRAM it reads (the same all lap: the loop writes nothing), and its charge */
  bool     snap_ok;
  uint8_t  nst;
  /* a $4212 loop: the shared second proof (idle_hvb_proof in main_snes.c)
   * holds this site's lap for the other value it reads */
  bool     alt;
  IdleState st[IDLE_MAX_STATES];
  uint8_t  ch[IDLE_MAX_STATES];   /* charge of the opcode at each boundary */
  uint8_t  npmem;
  uint32_t pmem_off[IDLE_MAX_MEM + IDLE_MAX_IDX * 2];
  uint8_t  smem[IDLE_MAX_MEM + IDLE_MAX_IDX * 2];
  uint32_t lap;
  /* kind: 0 = generic recurrence loop (everything above); 1 = accumulator loop
   * matched from the per-game template table (idle_skip.c). An accumulator is
   * LDA #0 / TCD / LDA x / ADC y / STA x / BRA: each lap does x += y + carry in
   * WRAM, so it never recurs and the recurrence path rejects it. The accum path
   * (idle_accum_span) runs one lap for real to measure its charge, then applies
   * the remaining whole laps natively. acc_dest/acc_add are WRAM byte offsets
   * (DP forced to 0 by the loop's TCD), acc_w the operand width in bytes. */
  uint8_t  kind;
  uint16_t acc_dest, acc_add;
  uint8_t  acc_w;
  /* optional WRAM precondition: the native fast-skip engages only while the
   * value at acc_pre_off (width acc_pre_w, 1 or 2) equals acc_pre_val. Disabled
   * when acc_pre_w == 0. Used when the loop body calls a subroutine that is idle
   * only in one state -- e.g. TMNT's per-lap JSL, which falls straight through
   * to an RTL (no writes, no I/O) iff [$001e2c] == 0, and does real work
   * otherwise. Re-checked each span; it cannot change mid-span (no NMI, and the
   * idle path writes nothing). acc_dest/acc_add stay the pure x += y the native
   * laps apply, so the precondition is what proves those laps are the whole lap. */
  uint32_t acc_pre_off;
  uint16_t acc_pre_val;
  uint8_t  acc_pre_w;
  /* a decoded accumulator (idle_skip.c accum_decode): its instructions in lap
   * order from the head (IDLE_ACC_*), where each starts (pc - head), and the
   * instruction starting at each pc - head (0xff: none). Their charges are
   * ch[], measured on its first lap, so a span can start and end anywhere in
   * the lap natively (idle_accum_native). acc_n 0: a template's loop, whose
   * per-lap call is skipped only in whole laps from the head. */
  uint8_t  acc_n;
  uint8_t  acc_op[8];
  uint8_t  acc_off[8];
  uint8_t  acc_at[16];
} IdleSkip;

enum { IDLE_KIND_RECUR = 0, IDLE_KIND_ACCUM = 1 };
enum { IDLE_ACC_LDA0, IDLE_ACC_TCD, IDLE_ACC_LDAF, IDLE_ACC_BNE, IDLE_ACC_LDAX, IDLE_ACC_ADC, IDLE_ACC_STA,
       IDLE_ACC_BRA };

/* The second proof of a $4212 loop, for the other value it reads, kept for
 * one site at a time (owner, whose alt is set): see idle_hvb_proof in
 * main_snes.c. In DTCM with the sites. */
typedef struct {
  IdleSkip *owner;
  uint8_t  hvb_val, nst, npmem;
  uint32_t lap;
  IdleState st[IDLE_MAX_STATES];
  uint8_t  ch[IDLE_MAX_STATES];
  uint32_t pmem_off[IDLE_MAX_MEM + IDLE_MAX_IDX * 2];
  uint8_t  smem[IDLE_MAX_MEM + IDLE_MAX_IDX * 2];
} IdleProof;

#ifndef SNES_IDLE_ACCUM
#define SNES_IDLE_ACCUM 1
#endif

/* after try_head fails: match the interrupted pc against the accumulator
 * template table (guard-byte verified). Fills `out` and returns true on a hit. */
bool idle_accum_match(Snes *s, uint8_t bank, uint16_t pc, IdleSkip *out);

typedef struct {
  uint32_t laps, ff_dots, wai_dots, rearms, rejects;
  uint32_t cyc, calls;    /* PERF: the span runner's own cycles and calls */
  uint32_t sa1_laps;      /* of laps: skipped with the SA-1 at work */
} IdleStats;

/* The sites live in DTCM on the device (idle_alloc): the span runner reads
 * the active one up to ~600 times a frame between PPU lines that sweep the
 * 16 KB D-cache. Until idle_alloc runs, and on the host, they are static.
 * g_idle is the ACTIVE site: the one the last NMI landed in. Several are kept
 * because some games alternate between two identical wait loops. */
extern IdleSkip *g_idle_sites;
extern IdleSkip *g_idlep;
extern IdleProof *g_idle_alt;
#define g_idle (*g_idlep)
extern IdleStats g_idle_stats;
extern uint8_t g_idle_budget;   /* proving laps left this frame, refilled at the NMI */
extern uint8_t g_idle_gsu_budget;   /* the same for GSU poll sites, so they never
                                     * starve the frame's wait loop of its proofs */
extern uint8_t g_idle_sa1_budget;   /* and for a loop polling the SA-1 while it works,
                                     * entered with other registers at each request */
/* what the span loop reads of the span search on every span outside the known
 * sites (main_snes.c's idle_span_line): in DTCM, allocated by idle_alloc() */
typedef struct {
  uint32_t pc[4];     /* the last spans' pcs, [0] the last */
  uint8_t  budget;    /* idle_span_search's site searches left this frame */
} IdleSpan;
extern IdleSpan *g_idle_span;
#define g_idle_span_budget (g_idle_span->budget)
extern uint8_t g_idle_hvb_budget;   /* and the second proofs of $4212 loops (main_snes.c) */

/* once, after snes_init: move the state (sites, second proof) to DTCM (no-op
 * off the device) */
void idle_alloc(void);

void idle_reset(void);
/* at the NMI request: examine the interrupted pc, arm a site if it passes */
void idle_nmi_sample(Snes *s);
/* Super FX: the CPU read the GSU status while the GSU is busy -- the same
 * site search at the current pc (no per-frame bookkeeping) */
void idle_gsu_poll_sample(Snes *s);
/* the CPU polls the SA-1's messages ($2300/$2301) while it works: the same */
void idle_sa1_poll_sample(Snes *s);
/* at an IRQ request: the same search at the interrupted pc. A loop waiting for
 * an IRQ (Yoshi's Island's frame wait ends on the GSU's STOP IRQ, mid-frame)
 * is never where the CPU is at the NMI, so the NMI sample alone never finds it. */
void idle_irq_sample(Snes *s);
/* Super FX, once a line: activate whichever known site holds the pc. Such a
 * game waits in two loops a frame -- its frame wait and the GSU poll -- and
 * only the NMI and the poll would otherwise switch between them. */
bool idle_line_select(Snes *s);   /* true: the pc is in a known site, now the active one */
/* SA-1 cart, a span outside every known site: an SA-1 cart's 65816 also waits
 * in loops no sample lands in (Mario RPG: the NMI in its frame wait, then an
 * I-RAM flag, which no register read offers). When this span starts at the
 * pc one of the 3 spans before the last started at, and within IDLE_MAX_LEN
 * bytes of the last one's pc, not at it -- a short loop, most likely -- the
 * site search runs at the pc (not a pc it rejected lately), at most
 * g_idle_span_budget times a frame, and arms only a loop that reads memory the
 * SA-1 shares: the others are the NMI sample's (taking the frame's proofs
 * from the loop it finds cost more than it saved). The pc history and that
 * first test are the caller's, on every such span (main_snes.c's
 * idle_span_line, in ITCM: up to ~800 spans a frame fail it); idle_span_search
 * does the rest when it passes. */
void idle_span_search(Snes *s, uint32_t pc24);
#ifdef SNES_PERF
extern uint32_t g_span_perf_calls, g_span_perf_searches, g_span_perf_cyc;   /* PROF/span */
#endif
/* before proving: resolve the read set (indexed operands with the current
 * X/Y) into pmem_off; false if any of it is not WRAM or ROM */
bool idle_prepare_reads(Snes *s, IdleSkip *g);

/* the decode environment still holds (only what the loop depends on) */
static inline bool idle_env_ok(const Cpu *cpu, const IdleSkip *g) {
  return cpu->e == g->e && cpu->mf == g->mf && cpu->xf == g->xf &&
         (!g->uses_db || cpu->db == g->db) && (!g->uses_dp || cpu->dp == g->dp);
}
/* a WRAM-resident loop's code is still the code that was decoded */
static inline bool idle_code_ok(const Snes *s, const IdleSkip *g) {
  if (!g->code_ram) return true;
  const uint8_t *ram = s->ram;
  uint32_t base = (g->bank == 0x7e || g->bank == 0x7f) ? ((uint32_t)(g->bank & 1) << 16) | g->head : g->head;
  for (int i = 0; i < g->ncode; i++) if (ram[(base + i) & 0x1ffff] != g->code[i]) return false;
  return true;
}

#endif
