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
#define IDLE_MAX_MEM   4

#define IDLE_MAX_STATES 16

/* The CPU state at one instruction boundary of the loop: a raw copy of Cpu
 * from `a` to `stopped`. Those 27 bytes are A X Y SP PC DP K DB, every flag
 * (E M X included) and the IRQ/NMI/WAI/STP state, contiguous in the struct
 * (asserted in idle_skip.c), so recurrence is a handful of word compares. */
#define IDLE_IMG_LEN 27
typedef struct {
  uint16_t pc;
  uint8_t  img[IDLE_IMG_LEN + 1];   /* at offset 2: img+14 (the flags) is word-aligned */
} __attribute__((aligned(4))) IdleState;

typedef struct {
  /* the armed site; head 0xffff with len 0 when nothing is armed */
  uint16_t head;          /* pc of the loop head                              */
  uint16_t len;           /* bytes from head to the closing branch's last byte */
  uint8_t  bank, bank_alt;
  /* the decode environment the loop was checked under */
  bool     e, mf, xf;
  uint8_t  db;
  uint16_t dp;
  /* WRAM bytes the loop reads (offsets into Snes.ram) */
  uint8_t  nmem;
  uint32_t mem_off[IDLE_MAX_MEM];
  uint8_t  mem_w[IDLE_MAX_MEM];
  /* the proven lap: the CPU state at each of its instruction boundaries, the
   * polled WRAM (the same all lap: the loop writes nothing), and its charge */
  bool     snap_ok;
  uint8_t  nst;
  IdleState st[IDLE_MAX_STATES];
  uint8_t  ch[IDLE_MAX_STATES];   /* charge of the opcode at each boundary */
  uint8_t  smem[IDLE_MAX_MEM];
  uint32_t lap;
  uint8_t  proof_budget;  /* proving laps left this frame, refilled at the NMI */
  /* statistics, per report window */
  uint32_t laps, ff_dots, wai_dots, rearms, rejects;
  uint32_t cyc, calls;    /* PERF: the span runner's own cycles and calls */
} IdleSkip;

/* The state lives in DTCM on the device (idle_alloc): the span runner reads it
 * up to ~600 times a frame between PPU lines that sweep the 16 KB D-cache, and
 * in cached RAM that measured 422 cycles a call, mostly misses. Until
 * idle_alloc runs, and on the host, it is a static. */
extern IdleSkip *g_idlep;
#define g_idle (*g_idlep)

/* once, after snes_init: move the state to DTCM (no-op off the device) */
void idle_alloc(void);

void idle_reset(void);
/* at the NMI request: examine the interrupted pc, arm a site if it passes */
void idle_nmi_sample(Snes *s);

#endif
