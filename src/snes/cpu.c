
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "cpu.h"
#include "snes.h"
#include "types.h"
#include "snes_gnw_alloc.h"
#ifdef HOST_BUILD
static int host_boot_trace_on(void) {
  static int t = -1;
  if (t < 0) t = getenv("HOST_BOOT_TRACE") ? 1 : 0;
  return t;
}
static int host_hang_pc_on(void) {
  static int t = -1;
  if (t < 0) t = getenv("HOST_HANG_PC") ? 1 : 0;
  return t;
}
#endif
#ifndef GNW_SNES_CORE
#include "../ida_types.h"
#include "../enemy_types.h"
#include "../variables.h"
#endif
#include "rc_dispatch.h"
#define SNES_CYCLES_PER_OPCODE \
  7, 6, 7, 4, 5, 3, 5, 6, 3, 2, 2, 4, 6, 4, 6, 5, \
  2, 5, 5, 7, 5, 4, 6, 6, 2, 4, 2, 2, 6, 4, 7, 5, \
  6, 6, 8, 4, 3, 3, 5, 6, 4, 2, 2, 5, 4, 4, 6, 5, \
  2, 5, 5, 7, 4, 4, 6, 6, 2, 4, 2, 2, 4, 4, 7, 5, \
  6, 6, 2, 4, 7, 3, 5, 6, 3, 2, 2, 3, 3, 4, 6, 5, \
  2, 5, 5, 7, 7, 4, 6, 6, 2, 4, 3, 2, 4, 4, 7, 5, \
  6, 6, 6, 4, 3, 3, 5, 6, 4, 2, 2, 6, 5, 4, 6, 5, \
  2, 5, 5, 7, 4, 4, 6, 6, 2, 4, 4, 2, 6, 4, 7, 5, \
  3, 6, 4, 4, 3, 3, 3, 6, 2, 2, 2, 3, 4, 4, 4, 5, \
  2, 6, 5, 7, 4, 4, 4, 6, 2, 5, 2, 2, 4, 5, 5, 5, \
  2, 6, 2, 4, 3, 3, 3, 6, 2, 2, 2, 4, 4, 4, 4, 5, \
  2, 5, 5, 7, 4, 4, 4, 6, 2, 4, 2, 2, 4, 4, 4, 5, \
  2, 6, 3, 4, 3, 3, 5, 6, 2, 2, 2, 3, 4, 4, 6, 5, \
  2, 5, 5, 7, 6, 4, 6, 6, 2, 4, 3, 3, 6, 4, 7, 5, \
  2, 6, 3, 4, 3, 3, 5, 6, 2, 2, 2, 3, 4, 4, 6, 5, \
  2, 5, 5, 7, 5, 4, 6, 6, 2, 4, 4, 2, 8, 4, 7, 5
static const int cyclesPerOpcode[256] = {
  SNES_CYCLES_PER_OPCODE
};
#ifdef SNES_THUMB2_CPU
const uint8_t snes_cycles_per_opcode[256] = {
  SNES_CYCLES_PER_OPCODE
};
_Static_assert(sizeof(snes_cycles_per_opcode) == 256, "snes_cycles_per_opcode must be 256 bytes");
#endif

static uint8_t cpu_read(Cpu* cpu, uint32_t adr);
static void cpu_write(Cpu* cpu, uint32_t adr, uint8_t val);
static uint8_t cpu_readOpcode(Cpu* cpu);
static uint16_t cpu_readOpcodeWord(Cpu* cpu);
static void cpu_setZN(Cpu* cpu, uint16_t value, bool byte);
static void cpu_doBranch(Cpu* cpu, uint8_t value, bool check);
static uint8_t cpu_pullByte(Cpu* cpu);
static void cpu_pushByte(Cpu* cpu, uint8_t value);
static uint16_t cpu_pullWord(Cpu* cpu);
static void cpu_pushWord(Cpu* cpu, uint16_t value);
static uint16_t cpu_readWord(Cpu* cpu, uint32_t adrl, uint32_t adrh);
static void cpu_writeWord(Cpu* cpu, uint32_t adrl, uint32_t adrh, uint16_t value, bool reversed);
static void cpu_doInterrupt(Cpu* cpu, bool irq);
static void cpu_doOpcode(Cpu* cpu, uint8_t opcode);

#ifdef HOST_BUILD
/* CPU_PCHIST=file (host): 65816 instructions executed per (bank, pc, mode) --
 * after the idle skip, so the work that really runs. Written at exit, hottest
 * first, as "bank:addr count opcode cum% m x e dl taken": the mode the
 * instruction ran in (m, x, e flags; dl = the direct page's low byte is not 0)
 * and how many times the next instruction was not 1-4 bytes after it (a
 * branch taken, a jump). The SA-1's instance reads SA1_PCHIST. */
#include <stdio.h>
#ifndef CPU_PCH_ENV
#define CPU_PCH_ENV "CPU_PCHIST"
#endif
#define CPU_PCH_N 131072u
static struct { uint32_t key, n; uint8_t op; uint32_t taken; } cpu_pch[CPU_PCH_N];
static int32_t cpu_pch_prev = -1;   /* the last instruction's entry */
static int cpu_pch_on = -1;
static unsigned long cpu_pch_total;
static int cpu_pch_cmp(const void *a, const void *b) {
  uint32_t x = ((const uint32_t *)a)[1], y = ((const uint32_t *)b)[1];
  return x < y ? 1 : x > y ? -1 : 0;
}
static void cpu_pch_write(void) {
  FILE *f = fopen(getenv(CPU_PCH_ENV), "w");
  if (!f) return;
  qsort(cpu_pch, CPU_PCH_N, sizeof cpu_pch[0], cpu_pch_cmp);
  unsigned long cum = 0;
  fprintf(f, "# total %lu 65816 instructions\n", cpu_pch_total);
  for (uint32_t i = 0; i < CPU_PCH_N && cpu_pch[i].n; i++) {
    cum += cpu_pch[i].n;
    const uint32_t k = cpu_pch[i].key;
    fprintf(f, "%02x:%04x %u %02x %.2f %u %u %u %u %u\n", (unsigned)((k >> 16) & 0xff), (unsigned)(k & 0xffff),
            (unsigned)cpu_pch[i].n, cpu_pch[i].op, 100.0 * cum / cpu_pch_total, (unsigned)(k >> 24) & 1,
            (unsigned)(k >> 25) & 1, (unsigned)(k >> 26) & 1, (unsigned)(k >> 27) & 1, (unsigned)cpu_pch[i].taken);
  }
  fclose(f);
}
static void cpu_pchist(const Cpu* cpu, uint32_t key, uint8_t op) {
  if (cpu_pch_on < 0) { cpu_pch_on = getenv(CPU_PCH_ENV) != NULL; if (cpu_pch_on) atexit(cpu_pch_write); }
  if (!cpu_pch_on) return;
  if (cpu_pch_prev >= 0) {
    const uint32_t pk = cpu_pch[cpu_pch_prev].key, d = (key - pk) & 0xffff;
    if (((pk ^ key) & 0xff0000) || d == 0 || d > 4) cpu_pch[cpu_pch_prev].taken++;
  }
  key |= (uint32_t)cpu->mf << 24 | (uint32_t)cpu->xf << 25 | (uint32_t)cpu->e << 26 |
         (uint32_t)((cpu->dp & 0xff) != 0) << 27;
  uint32_t h = (key * 2654435761u) >> 15;
  for (;; h++) {
    h &= CPU_PCH_N - 1;
    if (!cpu_pch[h].n || cpu_pch[h].key == key) break;
  }
  if (!cpu_pch[h].n) { cpu_pch[h].key = key; cpu_pch[h].op = op; }
  cpu_pch[h].n++;
  cpu_pch_total++;
  cpu_pch_prev = (int32_t)h;
}
#endif

extern Snes *g_snes; // for debugging

// addressing modes and opcode functions not declared, only used after defintions

/* The unhooked path, taken by every read that goes through cpu_readOpcode():
 * the opcode byte itself and the operand bytes each addressing mode consumes to
 * build its address. spin_hook_read() discards any address within ±6 bytes of
 * the PC as the fetch it is (`if (adr - (pcb - 6) <= 12) return;`), and those
 * are always inside that window, so classifying one was guaranteed to be a
 * no-op. The branch was not free — a device 3-arm A/B (0722, Zelda, gate OFF)
 * put the whole read/write hook tax at 240,186 cycles/frame, 3.61% of wall.
 *
 * NOT every fetch, to be exact: cpu_adrImm() only advances the pc and hands the
 * address to the ordinary cpu_read()/cpu_readWord() the data modes use, so an
 * immediate operand's bytes still take the hooked path. They still classify to
 * nothing (same ±6 window), they just still pay the branch. Splitting them out
 * would mean threading "this operand is immediate" through every cpu_xxx(low,
 * high) consumer, which is not worth it for the branch. */
#ifdef CPU_SA1
/* The SA-1's instance (sa1_cpu.c): mem is the Sa1, and its bus is the SA-1's
 * view of the cartridge (sa1.c). None of the main CPU's hooks apply. */
static inline uint8_t cpu_read(Cpu* cpu, uint32_t adr) {
  const uint8_t* p = g_sa1_rdmap[(adr >> 13) & 0x7ff];   /* ROM and plain BW-RAM */
  if (p) return p[adr & 0x1fff];
  /* I-RAM ($0000-$07FF and $3000-$37FF of banks $00-$3F/$80-$BF), where the
   * SA-1 keeps its direct page and stack: 2 KB, so not in the 8 KB page map */
  const uint32_t a = adr & 0x40f800;
  if (a == 0 || a == 0x3000) return g_sa1_iram[adr & 0x7ff];
  return sa1_cpu_read((Sa1*) cpu->mem, adr);
}

static uint8_t cpu_read_raw(Cpu* cpu, uint32_t adr) {
  return cpu_read(cpu, adr);
}

static inline void cpu_write(Cpu* cpu, uint32_t adr, uint8_t val) {
  const uint32_t a = adr & 0x40f800;
  if (a == 0 || a == 0x3000) {        /* I-RAM, as in cpu_read: a write that changes a byte marks the lap */
    uint8_t* p = &g_sa1_iram[adr & 0x7ff];
    if (*p != val) {
      *g_sa1_lapdirty = 1;
      *p = val;
    }
    return;
  }
  sa1_cpu_write((Sa1*) cpu->mem, adr, val);
}
#else
static uint8_t cpu_read_raw(Cpu* cpu, uint32_t adr) {
  // assume mem is a pointer to a Snes
  return snes_cpuRead((Snes*) cpu->mem, adr);
}

#ifdef SNES_SPIN_SKIP
/* Spin-skip purity hooks (spin_skip.h): every data read is classified (IO reads
 * break an iteration's purity), every write bumps the write sequence. The SAME
 * hooks feed the host gate harness (tools/snes_spin) — one implementation,
 * proven bit-identical there, running here. */
#include "spin_skip.h"
static uint8_t cpu_read(Cpu* cpu, uint32_t adr) {
  if (g_spin.phase) spin_hook_read(cpu, adr);
  return snes_cpuRead((Snes*) cpu->mem, adr);
}

static void cpu_write(Cpu* cpu, uint32_t adr, uint8_t val) {
  if (g_spin.phase) spin_hook_write();
  snes_cpuWrite((Snes*) cpu->mem, adr, val);
}
#else
static uint8_t cpu_read(Cpu* cpu, uint32_t adr) {
  return cpu_read_raw(cpu, adr);
}

static void cpu_write(Cpu* cpu, uint32_t adr, uint8_t val) {
  // assume mem is a pointer to a Snes
  snes_cpuWrite((Snes*) cpu->mem, adr, val);
}
#endif
#endif /* CPU_SA1 */

/* Stage 3D -- non-static data-access bridges for the Thumb-2 direct-page
 * handlers (snes_thumb2.S). These are byte-identical to the static cpu_read /
 * cpu_write above: same spin-skip hooks (when SNES_SPIN_SKIP), same raw bus
 * call. They exist only so the assembly can BL a data read/write that takes the
 * hooked path -- operand fetches stay raw (snes_cpuRead via the engine's
 * IMM_FETCH_BYTE), exactly as cpu_adrDp uses cpu_readOpcode (cpu_read_raw).
 * Non-static + unreferenced from C: the BL from .S resolves the symbol at link;
 * -Wunused-function does not fire for extern linkage. */
#ifndef CPU_SA1
#ifdef SNES_BUS_IN_ITCM
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
uint8_t cpu_thumb2_read(Cpu* cpu, uint32_t adr) {
#if defined(SNES_SPIN_SKIP) && !SNES_SPIN_NO_BUS_HOOKS
  if(g_spin.phase) spin_hook_read(cpu, adr);
#endif
#if SNES_PAGEMAP
  /* The page map's fast path right here, where the engine's data reads
   * arrive: a WRAM or ROM byte never enters snes_cpuRead. The engine has
   * 280 bus call sites and ITCM no room to inline this at each, so the
   * bridge they all call is where it goes. snes_cpuRead counts the access
   * itself on the slow path, so it is counted exactly once either way. A
   * data read no longer retargets the engine's opcode-fetch page (only
   * fetches do): that changes speed, never a result. */
  {
    Snes *snes = (Snes*) cpu->mem;
    uint8_t *base = g_snes_rdmap[(adr >> 13) & 0x7ff];
    if (base) {
      snes->cpuMemOps++;
      return base[adr & 0x1fff];
    }
    return snes_cpuRead(snes, adr);
  }
#else
  return snes_cpuRead((Snes*) cpu->mem, adr);
#endif
}

#ifdef SNES_BUS_IN_ITCM
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
void cpu_thumb2_write(Cpu* cpu, uint32_t adr, uint8_t val) {
#if defined(SNES_SPIN_SKIP) && !SNES_SPIN_NO_BUS_HOOKS
  if(g_spin.phase) spin_hook_write();
#endif
#if SNES_PAGEMAP
  /* the write twin: WRAM pages straight from the map, the rest (and the
   * access count) through snes_cpuWrite */
  {
    Snes *snes = (Snes*) cpu->mem;
    uint8_t *base = g_snes_wrmap[(adr >> 13) & 0x7ff];
    if (base) {
      snes->cpuMemOps++;
      base[adr & 0x1fff] = val;
      return;
    }
    snes_cpuWrite(snes, adr, val);
  }
#else
  snes_cpuWrite((Snes*) cpu->mem, adr, val);
#endif
}
#endif /* !CPU_SA1 */

Cpu* cpu_init(void* mem, int memType) {
  Cpu* cpu = snes_zalloc(sizeof(Cpu));
  if (!cpu) return NULL;
  cpu->mem = mem;
  cpu->memType = memType;
  return cpu;
}

void cpu_free(Cpu* cpu) {
  snes_zfree(cpu);
}

void cpu_reset(Cpu* cpu) {
  cpu->a = 0;
  cpu->x = 0;
  cpu->y = 0;
  cpu->sp = 0x100;
  {
    uint8_t rv_lo = cpu_read(cpu, 0xfffc);
    uint8_t rv_hi = cpu_read(cpu, 0xfffd);
    cpu->pc = rv_lo | (rv_hi << 8);
#ifdef HOST_BUILD
    if (host_boot_trace_on()) {
      printf("boot: resetvec [%02x %02x] -> %02x:%04x\n",
             rv_lo, rv_hi, 0, cpu->pc);
    }
#endif
  }
  cpu->dp = 0;
  cpu->k = 0;
  cpu->db = 0;
  cpu->c = false;
  cpu->z = false;
  cpu->v = false;
  cpu->n = false;
  cpu->i = true;
  cpu->d = false;
  cpu->xf = true;
  cpu->mf = true;
  cpu->e = true;
  cpu->irqWanted = false;
  cpu->nmiWanted = false;
  cpu->waiting = false;
  cpu->stopped = false;
  cpu->cyclesUsed = 0;
}

void cpu_saveload(Cpu *cpu, SaveLoadFunc *func, void *ctx) {
  func(ctx, &cpu->a, offsetof(Cpu, cyclesUsed) - offsetof(Cpu, a));
  cpu->spBreakpoint = 0x0;
}

#ifndef SNES_SPIN_NO_BUS_HOOKS
#define SNES_SPIN_NO_BUS_HOOKS 0
#endif
/* SNES_THUMB2_CPU: split the interpreter into an oracle symbol the Thumb-2
 * dispatcher can call as a fallback, while keeping flag-off byte-identical to
 * upstream (the macro collapses to the plain name, so the definition token and
 * thus the object are unchanged). Stage 0's dispatcher is a pure C passthrough;
 * Stage 1 routes per-opcode into the assembly engine, falling back here for
 * anything not yet ported. Never introduce a recursive alias -- the public
 * contract cpu_runOpcode() always resolves to exactly one implementation. */
#ifdef SNES_THUMB2_CPU
#  define SNES_RUNOPCODE_IMPL cpu_runOpcode_c
#else
#  define SNES_RUNOPCODE_IMPL cpu_runOpcode
#endif
/* Also in ITCM: this is the wrapper that calls the engine, and it lives in the
 * overlay while the engine lives in ITCM, so the BL between them went through a
 * veneer -- 2.1% of the frame by device profile, on every opcode. 188 bytes.
 * With the Thumb-2 engine this is the C oracle cpu_runOpcode_c, which nothing
 * calls: it stays out of ITCM. */
#if defined(SNES_BUS_IN_ITCM) && !defined(SNES_THUMB2_CPU)
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
#ifdef HOST_BUILD
uint32_t snes_fs_cpu;   /* FRAME_STATS (main_snes.c): 65816 instructions this frame */
#endif
int SNES_RUNOPCODE_IMPL(Cpu* cpu) {
  cpu->cyclesUsed = 0;
  if(cpu->stopped) return 1;
#ifdef HOST_BUILD
  snes_fs_cpu++;
#endif

  if(cpu->waiting) {
    /* WAI holds the CPU until an interrupt is pending — including one the I flag
     * will not let it take, which still wakes it. */
    if(!(cpu->irqWanted || cpu->nmiWanted)) return 1;
    cpu->waiting = false;
  }

  // not stopped or waiting, execute a opcode or go to interrupt
  if((!cpu->i && cpu->irqWanted) || cpu->nmiWanted) {
    cpu->cyclesUsed = 7; // interrupt: at least 7 cycles
    if(cpu->nmiWanted) {
      cpu->nmiWanted = false;
      cpu_doInterrupt(cpu, false);
    } else {
      // must be irq
      cpu_doInterrupt(cpu, true);
    }
  }
  /* rc static recompiler: if active, dispatch to native site (−42% insn on SMW).
   * One never-taken branch when inactive (g_rc_active defaults false). */
#ifndef CPU_SA1
  if (g_rc_active) {
    uint16_t id = rc_dispatch_lookup(cpu->k, cpu->pc);
    if (id) { rc_dispatch_call(id, cpu); return cpu->cyclesUsed; }
  }
#endif
#ifdef HOST_BUILD
  const uint32_t pch_key = ((uint32_t)cpu->k << 16) | cpu->pc;
#endif
  uint8_t opcode = cpu_readOpcode(cpu);
#ifdef HOST_BUILD
  cpu_pchist(cpu, pch_key, opcode);
#endif
  cpu->cyclesUsed = cyclesPerOpcode[opcode];
  cpu_doOpcode(cpu, opcode);
  return cpu->cyclesUsed;
}

#ifdef SNES_THUMB2_CPU
/* Public contract, Stage 2. Mirrors cpu_runOpcode_c's pre-work (stopped / WAI /
 * IRQ / NMI / rc) and then delegates the single-opcode fetch+dispatch to the
 * Thumb-2 engine via snes_thumb2_step. The engine fetches EXACTLY ONE opcode by
 * calling the real snes_cpuRead(mem, (k<<16)|pc), increments the 16-bit pc once,
 * charges cyclesUsed from the exported byte cycle table, and runs the handler
 * directly. It returns -1 if handled, or the opcode byte (0..255) if unsupported
 * -- in which case C calls cpu_doOpcode on the SAME already-fetched byte: no
 * second fetch, no double cycle/bus charge. The oracle cpu_runOpcode_c is kept
 * intact as the differential reference and is the sole code path when flag off. */
int cpu_runOpcode(Cpu* cpu) {
  cpu->cyclesUsed = 0;
  if(cpu->stopped) return 1;

  if(cpu->waiting) {
    if(!(cpu->irqWanted || cpu->nmiWanted)) return 1;
    cpu->waiting = false;
  }

  if((!cpu->i && cpu->irqWanted) || cpu->nmiWanted) {
    cpu->cyclesUsed = 7;
    if(cpu->nmiWanted) {
      cpu->nmiWanted = false;
      cpu_doInterrupt(cpu, false);
    } else {
      cpu_doInterrupt(cpu, true);
    }
  }
  if (g_rc_active) {
    uint16_t id = rc_dispatch_lookup(cpu->k, cpu->pc);
    if (id) { rc_dispatch_call(id, cpu); return cpu->cyclesUsed; }
  }
  int r = snes_thumb2_step(cpu);
  if(r != -1)
    cpu_doOpcode(cpu, (uint8_t)r);
  return cpu->cyclesUsed;
}

/* Finish an opcode the engine's run-mode entry (snes_thumb2_run) fetched but
 * cannot execute. cpu_doOpcode is static, so the assembly needs a door; this is
 * the same shape as the cpu_thumb2_read/write bridges above. The byte comes in
 * as an argument precisely because it is ALREADY fetched and pc already
 * advanced -- refetching here would run the operand as an opcode. */
#if SNES_OP_CENSUS
uint32_t g_op_total, g_op_fallback, g_op_fbhist[256];
#endif
#ifdef SNES_BUS_IN_ITCM
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
int cpu_thumb2_fallback(Cpu* cpu, uint32_t opcode) {
#if SNES_OP_CENSUS
  g_op_fallback++;
  g_op_fbhist[(uint8_t)opcode]++;
#endif
  cpu_doOpcode(cpu, (uint8_t)opcode);
  return cpu->cyclesUsed;
}
#endif

#if defined(CPU_SA1) && defined(CPU_SA1_T2)
/* The SA-1's engine (thumb2/sa1_engine.S) finishes here an opcode it leaves
 * to C, on this copy's bus: the same door as above. A run-ahead looks once at
 * what the opcode wrote, not at each write: a check inlined in every write of
 * this interpreter would slow it down outside run-aheads too. */
int cpu_thumb2_fallback(Cpu* cpu, uint32_t opcode) {
  cpu_doOpcode(cpu, (uint8_t)opcode);
  sa1_ran_opcode((Sa1*) cpu->mem);
  return cpu->cyclesUsed;
}
#endif

static uint8_t cpu_readOpcode(Cpu* cpu) {
  return cpu_read_raw(cpu, (cpu->k << 16) | cpu->pc++);
}

static uint16_t cpu_readOpcodeWord(Cpu* cpu) {
  uint8_t low = cpu_readOpcode(cpu);
  return low | (cpu_readOpcode(cpu) << 8);
}

uint8_t cpu_getFlags(Cpu* cpu) {
  uint8_t val = cpu->n << 7;
  val |= cpu->v << 6;
  val |= cpu->mf << 5;
  val |= cpu->xf << 4;
  val |= cpu->d << 3;
  val |= cpu->i << 2;
  val |= cpu->z << 1;
  val |= cpu->c;
  return val;
}

void cpu_setFlags(Cpu* cpu, uint8_t val) {
  cpu->n = val & 0x80;
  cpu->v = val & 0x40;
  cpu->mf = val & 0x20;
  cpu->xf = val & 0x10;
  cpu->d = val & 8;
  cpu->i = val & 4;
  cpu->z = val & 2;
  cpu->c = val & 1;
  if(cpu->e) {
    cpu->mf = true;
    cpu->xf = true;
    cpu->sp = (cpu->sp & 0xff) | 0x100;
  }
  if(cpu->xf) {
    cpu->x &= 0xff;
    cpu->y &= 0xff;
  }
}

static void cpu_setZN(Cpu* cpu, uint16_t value, bool byte) {
  if(byte) {
    cpu->z = (value & 0xff) == 0;
    cpu->n = value & 0x80;
  } else {
    cpu->z = value == 0;
    cpu->n = value & 0x8000;
  }
}

static void cpu_doBranch(Cpu* cpu, uint8_t value, bool check) {
  if(check) {
    cpu->cyclesUsed++; // taken branch: 1 extra cycle
    cpu->pc += (int8_t) value;
  }
}

static uint8_t cpu_pullByte(Cpu* cpu) {
  cpu->sp++;
  if(cpu->e) cpu->sp = (cpu->sp & 0xff) | 0x100;
  return cpu_read(cpu, cpu->sp);
}

static void cpu_pushByte(Cpu* cpu, uint8_t value) {
  cpu_write(cpu, cpu->sp, value);
  cpu->sp--;
  if(cpu->e) cpu->sp = (cpu->sp & 0xff) | 0x100;
}

static uint16_t cpu_pullWord(Cpu* cpu) {
  uint8_t value = cpu_pullByte(cpu);
  return value | (cpu_pullByte(cpu) << 8);
}

static void cpu_pushWord(Cpu* cpu, uint16_t value) {
  cpu_pushByte(cpu, value >> 8);
  cpu_pushByte(cpu, value & 0xff);
}

static uint16_t cpu_readWord(Cpu* cpu, uint32_t adrl, uint32_t adrh) {
  uint8_t value = cpu_read(cpu, adrl);
  return value | (cpu_read(cpu, adrh) << 8);
}

static void cpu_writeWord(Cpu* cpu, uint32_t adrl, uint32_t adrh, uint16_t value, bool reversed) {
  if(reversed) {
    cpu_write(cpu, adrh, value >> 8);
    cpu_write(cpu, adrl, value & 0xff);
  } else {
    cpu_write(cpu, adrl, value & 0xff);
    cpu_write(cpu, adrh, value >> 8);
  }
}

static void cpu_doInterrupt(Cpu* cpu, bool irq) {
  cpu_pushByte(cpu, cpu->k);
  cpu_pushWord(cpu, cpu->pc);
  cpu_pushByte(cpu, cpu_getFlags(cpu));
  cpu->cyclesUsed++; // native mode: 1 extra cycle
  cpu->i = true;
  cpu->d = false;
  cpu->k = 0;
  if(irq) {
    cpu->pc = cpu_readWord(cpu, 0xffee, 0xffef);
  } else {
    // nmi
    cpu->pc = cpu_readWord(cpu, 0xffea, 0xffeb);
  }
}

// addressing modes

static uint32_t cpu_adrImm(Cpu* cpu, uint32_t* low, bool xFlag) {
  if((xFlag && cpu->xf) || (!xFlag && cpu->mf)) {
    *low = (cpu->k << 16) | cpu->pc++;
    return 0;
  } else {
    *low = (cpu->k << 16) | cpu->pc++;
    return (cpu->k << 16) | cpu->pc++;
  }
}

static uint32_t cpu_adrDp(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  *low = (cpu->dp + adr) & 0xffff;
  return (cpu->dp + adr + 1) & 0xffff;
}

static uint32_t cpu_adrDpx(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  *low = (cpu->dp + adr + cpu->x) & 0xffff;
  return (cpu->dp + adr + cpu->x + 1) & 0xffff;
}

static uint32_t cpu_adrDpy(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  *low = (cpu->dp + adr + cpu->y) & 0xffff;
  return (cpu->dp + adr + cpu->y + 1) & 0xffff;
}

static uint32_t cpu_adrIdp(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint16_t pointer = cpu_readWord(cpu, (cpu->dp + adr) & 0xffff, (cpu->dp + adr + 1) & 0xffff);
  *low = (cpu->db << 16) + pointer;
  return ((cpu->db << 16) + pointer + 1) & 0xffffff;
}

static uint32_t cpu_adrIdx(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint16_t pointer = cpu_readWord(cpu, (cpu->dp + adr + cpu->x) & 0xffff, (cpu->dp + adr + cpu->x + 1) & 0xffff);
  *low = (cpu->db << 16) + pointer;
  return ((cpu->db << 16) + pointer + 1) & 0xffffff;
}

static uint32_t cpu_adrIdy(Cpu* cpu, uint32_t* low, bool write) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint16_t pointer = cpu_readWord(cpu, (cpu->dp + adr) & 0xffff, (cpu->dp + adr + 1) & 0xffff);
  if(write && (!cpu->xf || ((pointer >> 8) != ((pointer + cpu->y) >> 8)))) cpu->cyclesUsed++;
  // x = 0 or page crossed, with writing opcode: 1 extra cycle
  *low = ((cpu->db << 16) + pointer + cpu->y) & 0xffffff;
  return ((cpu->db << 16) + pointer + cpu->y + 1) & 0xffffff;
}

static uint32_t cpu_adrIdl(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint32_t pointer = cpu_readWord(cpu, (cpu->dp + adr) & 0xffff, (cpu->dp + adr + 1) & 0xffff);
  pointer |= cpu_read(cpu, (cpu->dp + adr + 2) & 0xffff) << 16;
  *low = pointer;
  return (pointer + 1) & 0xffffff;
}

static uint32_t cpu_adrIly(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  if(cpu->dp & 0xff) cpu->cyclesUsed++; // dpr not 0: 1 extra cycle
  uint32_t pointer = cpu_readWord(cpu, (cpu->dp + adr) & 0xffff, (cpu->dp + adr + 1) & 0xffff);
  pointer |= cpu_read(cpu, (cpu->dp + adr + 2) & 0xffff) << 16;
  *low = (pointer + cpu->y) & 0xffffff;
  return (pointer + cpu->y + 1) & 0xffffff;
}

static uint32_t cpu_adrSr(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  *low = (cpu->sp + adr) & 0xffff;
  return (cpu->sp + adr + 1) & 0xffff;
}

static uint32_t cpu_adrIsy(Cpu* cpu, uint32_t* low) {
  uint8_t adr = cpu_readOpcode(cpu);
  uint16_t pointer = cpu_readWord(cpu, (cpu->sp + adr) & 0xffff, (cpu->sp + adr + 1) & 0xffff);
  *low = ((cpu->db << 16) + pointer + cpu->y) & 0xffffff;
  return ((cpu->db << 16) + pointer + cpu->y + 1) & 0xffffff;
}

static uint32_t cpu_adrAbs(Cpu* cpu, uint32_t* low) {
  uint16_t adr = cpu_readOpcodeWord(cpu);
  *low = (cpu->db << 16) + adr;
  return ((cpu->db << 16) + adr + 1) & 0xffffff;
}

static uint32_t cpu_adrAbx(Cpu* cpu, uint32_t* low, bool write) {
  uint16_t adr = cpu_readOpcodeWord(cpu);
  if(write && (!cpu->xf || ((adr >> 8) != ((adr + cpu->x) >> 8)))) cpu->cyclesUsed++;
  // x = 0 or page crossed, with writing opcode: 1 extra cycle
  *low = ((cpu->db << 16) + adr + cpu->x) & 0xffffff;
  return ((cpu->db << 16) + adr + cpu->x + 1) & 0xffffff;
}

static uint32_t cpu_adrAby(Cpu* cpu, uint32_t* low, bool write) {
  uint16_t adr = cpu_readOpcodeWord(cpu);
  if(write && (!cpu->xf || ((adr >> 8) != ((adr + cpu->y) >> 8)))) cpu->cyclesUsed++;
  // x = 0 or page crossed, with writing opcode: 1 extra cycle
  *low = ((cpu->db << 16) + adr + cpu->y) & 0xffffff;
  return ((cpu->db << 16) + adr + cpu->y + 1) & 0xffffff;
}

static uint32_t cpu_adrAbl(Cpu* cpu, uint32_t* low) {
  uint32_t adr = cpu_readOpcodeWord(cpu);
  adr |= cpu_readOpcode(cpu) << 16;
  *low = adr;
  return (adr + 1) & 0xffffff;
}

static uint32_t cpu_adrAlx(Cpu* cpu, uint32_t* low) {
  uint32_t adr = cpu_readOpcodeWord(cpu);
  adr |= cpu_readOpcode(cpu) << 16;
  *low = (adr + cpu->x) & 0xffffff;
  return (adr + cpu->x + 1) & 0xffffff;
}

static uint16_t cpu_adrIax(Cpu* cpu) {
  uint16_t adr = cpu_readOpcodeWord(cpu);
  return cpu_readWord(cpu, (cpu->k << 16) | ((adr + cpu->x) & 0xffff), (cpu->k << 16) | ((adr + cpu->x + 1) & 0xffff));
}

// opcode functions

static void cpu_and(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->a = (cpu->a & 0xff00) | ((cpu->a & value) & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->a &= value;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_ora(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->a = (cpu->a & 0xff00) | ((cpu->a | value) & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->a |= value;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_eor(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->a = (cpu->a & 0xff00) | ((cpu->a ^ value) & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->a ^= value;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_adc(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    int result = 0;
    if(cpu->d) {
      result = (cpu->a & 0xf) + (value & 0xf) + cpu->c;
      if(result > 0x9) result = ((result + 0x6) & 0xf) + 0x10;
      result = (cpu->a & 0xf0) + (value & 0xf0) + result;
    } else {
      result = (cpu->a & 0xff) + value + cpu->c;
    }
    cpu->v = (cpu->a & 0x80) == (value & 0x80) && (value & 0x80) != (result & 0x80);
    if(cpu->d && result > 0x9f) result += 0x60;
    cpu->c = result > 0xff;
    cpu->a = (cpu->a & 0xff00) | (result & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    int result = 0;
    if(cpu->d) {
      result = (cpu->a & 0xf) + (value & 0xf) + cpu->c;
      if(result > 0x9) result = ((result + 0x6) & 0xf) + 0x10;
      result = (cpu->a & 0xf0) + (value & 0xf0) + result;
      if(result > 0x9f) result = ((result + 0x60) & 0xff) + 0x100;
      result = (cpu->a & 0xf00) + (value & 0xf00) + result;
      if(result > 0x9ff) result = ((result + 0x600) & 0xfff) + 0x1000;
      result = (cpu->a & 0xf000) + (value & 0xf000) + result;
    } else {
      result = cpu->a + value + cpu->c;
    }
    cpu->v = (cpu->a & 0x8000) == (value & 0x8000) && (value & 0x8000) != (result & 0x8000);
    if(cpu->d && result > 0x9fff) result += 0x6000;
    cpu->c = result > 0xffff;
    cpu->a = result;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_sbc(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low) ^ 0xff;
    int result = 0;
    if(cpu->d) {
      result = (cpu->a & 0xf) + (value & 0xf) + cpu->c;
      if(result < 0x10) result = (result - 0x6) & ((result - 0x6 < 0) ? 0xf : 0x1f);
      result = (cpu->a & 0xf0) + (value & 0xf0) + result;
    } else {
      result = (cpu->a & 0xff) + value + cpu->c;
    }
    cpu->v = (cpu->a & 0x80) == (value & 0x80) && (value & 0x80) != (result & 0x80);
    if(cpu->d && result < 0x100) result -= 0x60;
    cpu->c = result > 0xff;
    cpu->a = (cpu->a & 0xff00) | (result & 0xff);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high) ^ 0xffff;
    int result = 0;
    if(cpu->d) {
      result = (cpu->a & 0xf) + (value & 0xf) + cpu->c;
      if(result < 0x10) result = (result - 0x6) & ((result - 0x6 < 0) ? 0xf : 0x1f);
      result = (cpu->a & 0xf0) + (value & 0xf0) + result;
      if(result < 0x100) result = (result - 0x60) & ((result - 0x60 < 0) ? 0xff : 0x1ff);
      result = (cpu->a & 0xf00) + (value & 0xf00) + result;
      if(result < 0x1000) result = (result - 0x600) & ((result - 0x600 < 0) ? 0xfff : 0x1fff);
      result = (cpu->a & 0xf000) + (value & 0xf000) + result;
    } else {
      result = cpu->a + value + cpu->c;
    }
    cpu->v = (cpu->a & 0x8000) == (value & 0x8000) && (value & 0x8000) != (result & 0x8000);
    if(cpu->d && result < 0x10000) result -= 0x6000;
    cpu->c = result > 0xffff;
    cpu->a = result;
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_cmp(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low) ^ 0xff;
    result = (cpu->a & 0xff) + value + 1;
    cpu->c = result > 0xff;
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high) ^ 0xffff;
    result = cpu->a + value + 1;
    cpu->c = result > 0xffff;
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_cpx(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->xf) {
    uint8_t value = cpu_read(cpu, low) ^ 0xff;
    result = (cpu->x & 0xff) + value + 1;
    cpu->c = result > 0xff;
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high) ^ 0xffff;
    result = cpu->x + value + 1;
    cpu->c = result > 0xffff;
  }
  cpu_setZN(cpu, result, cpu->xf);
}

static void cpu_cpy(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->xf) {
    uint8_t value = cpu_read(cpu, low) ^ 0xff;
    result = (cpu->y & 0xff) + value + 1;
    cpu->c = result > 0xff;
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high) ^ 0xffff;
    result = cpu->y + value + 1;
    cpu->c = result > 0xffff;
  }
  cpu_setZN(cpu, result, cpu->xf);
}

static void cpu_bit(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    uint8_t result = (cpu->a & 0xff) & value;
    cpu->z = result == 0;
    cpu->n = value & 0x80;
    cpu->v = value & 0x40;
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    uint16_t value = cpu_readWord(cpu, low, high);
    uint16_t result = cpu->a & value;
    cpu->z = result == 0;
    cpu->n = value & 0x8000;
    cpu->v = value & 0x4000;
  }
}

static void cpu_lda(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    cpu->a = (cpu->a & 0xff00) | cpu_read(cpu, low);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    cpu->a = cpu_readWord(cpu, low, high);
  }
  cpu_setZN(cpu, cpu->a, cpu->mf);
}

static void cpu_ldx(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->xf) {
    cpu->x = cpu_read(cpu, low);
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    cpu->x = cpu_readWord(cpu, low, high);
  }
  cpu_setZN(cpu, cpu->x, cpu->xf);
}

static void cpu_ldy(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->xf) {
    cpu->y = cpu_read(cpu, low);
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    cpu->y = cpu_readWord(cpu, low, high);
  }
  cpu_setZN(cpu, cpu->y, cpu->xf);
}

static void cpu_sta(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    cpu_write(cpu, low, (uint8_t)cpu->a);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    cpu_writeWord(cpu, low, high, cpu->a, false);
  }
}

static void cpu_stx(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->xf) {
    cpu_write(cpu, low, (uint8_t)cpu->x);
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    cpu_writeWord(cpu, low, high, cpu->x, false);
  }
}

static void cpu_sty(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->xf) {
    cpu_write(cpu, low, (uint8_t)cpu->y);
  } else {
    cpu->cyclesUsed++; // x = 0: 1 extra cycle
    cpu_writeWord(cpu, low, high, cpu->y, false);
  }
}

static void cpu_stz(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    cpu_write(cpu, low, 0);
  } else {
    cpu->cyclesUsed++; // m = 0: 1 extra cycle
    cpu_writeWord(cpu, low, high, 0, false);
  }
}

static void cpu_ror(Cpu* cpu, uint32_t low, uint32_t high) {
  bool carry = false;
  int result = 0;
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    carry = value & 1;
    result = (value >> 1) | (cpu->c << 7);
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    uint16_t value = cpu_readWord(cpu, low, high);
    carry = value & 1;
    result = (value >> 1) | (cpu->c << 15);
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
  cpu->c = carry;
}

static void cpu_rol(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    result = (cpu_read(cpu, low) << 1) | cpu->c;
    cpu->c = result & 0x100;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    result = (cpu_readWord(cpu, low, high) << 1) | cpu->c;
    cpu->c = result & 0x10000;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_lsr(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->c = value & 1;
    result = value >> 1;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->c = value & 1;
    result = value >> 1;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_asl(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    result = cpu_read(cpu, low) << 1;
    cpu->c = result & 0x100;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    result = cpu_readWord(cpu, low, high) << 1;
    cpu->c = result & 0x10000;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_inc(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    result = cpu_read(cpu, low) + 1;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    result = cpu_readWord(cpu, low, high) + 1;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_dec(Cpu* cpu, uint32_t low, uint32_t high) {
  int result = 0;
  if(cpu->mf) {
    result = cpu_read(cpu, low) - 1;
    cpu_write(cpu, low, result);
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    result = cpu_readWord(cpu, low, high) - 1;
    cpu_writeWord(cpu, low, high, result, true);
  }
  cpu_setZN(cpu, result, cpu->mf);
}

static void cpu_tsb(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->z = ((cpu->a & 0xff) & value) == 0;
    cpu_write(cpu, low, value | (cpu->a & 0xff));
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->z = (cpu->a & value) == 0;
    cpu_writeWord(cpu, low, high, value | cpu->a, true);
  }
}

static void cpu_trb(Cpu* cpu, uint32_t low, uint32_t high) {
  if(cpu->mf) {
    uint8_t value = cpu_read(cpu, low);
    cpu->z = ((cpu->a & 0xff) & value) == 0;
    cpu_write(cpu, low, value & ~(cpu->a & 0xff));
  } else {
    cpu->cyclesUsed += 2; // m = 0: 2 extra cycles
    uint16_t value = cpu_readWord(cpu, low, high);
    cpu->z = (cpu->a & value) == 0;
    cpu_writeWord(cpu, low, high, value & ~cpu->a, true);
  }
}


extern int CpuOpcodeHook(uint32 addr);

uint32_t pc_hist[8], pc_hist_ctr;

uint32_t pc_bp = 0;

void DumpCpuHistory() {
  for (int i = 0; i < 8; i++) {
    printf("PC history: 0x%x\n", pc_hist[(pc_hist_ctr + i) & 7]);
  }
}

static void cpu_doOpcode(Cpu* cpu, uint8_t opcode) {
#ifdef HOST_BUILD
  if (host_boot_trace_on()) {
    static unsigned boot_trace_left = 256;
    if (boot_trace_left) {
      printf("boot: pc=%02x:%04x op=%02x a=%04x x=%04x y=%04x sp=%04x dp=%04x\n",
             cpu->k, (uint16_t)(cpu->pc - 1), opcode,
             cpu->a, cpu->x, cpu->y, cpu->sp, cpu->dp);
      boot_trace_left--;
    }
  }
#ifndef CPU_SA1
  if (host_hang_pc_on()) {
    static unsigned n;
    static uint32_t last_pc;
    static unsigned stuck;
    uint32_t pc = ((uint32_t)cpu->k << 16) | (uint16_t)(cpu->pc - 1);
    n++;
    if (pc == last_pc) stuck++;
    else { stuck = 0; last_pc = pc; }
    if (n <= 8 || n == 1000 || (n % 200000u) == 0u || stuck == 80000u) {
      Snes *s = (Snes *)cpu->mem;
      unsigned apu0 = 0, apu1 = 0;
      if (s && s->apu) {
        apu0 = s->apu->outPorts[0];
        apu1 = s->apu->outPorts[1];
      }
      printf("hang: n=%u pc=%02x:%04x op=%02x a=%04x stuck=%u nmi=%d 2140=%02x 2141=%02x v=%u\n",
             n, cpu->k, (uint16_t)(cpu->pc - 1), opcode, cpu->a, stuck,
             s ? (int)s->cpu->nmiWanted : 0, apu0, apu1,
             s ? s->vPos : 0);
      fflush(stdout);
    }
    if (n >= 3000000u) {
      printf("hang: abort after %u opcodes at %02x:%04x\n", n, cpu->k, (uint16_t)(cpu->pc - 1));
      fflush(stdout);
      exit(2);
    }
  }
#endif /* !CPU_SA1 */
#endif
#ifdef SNES_PC_HISTOGRAM
  /* Per-ROM execution map: count every executed opcode by its 24-bit address, so
   * the hot banks/routines stand out. The histogram (16M entries) lives in the
   * harness; this just bumps it. Measurement build only. */
  extern uint32_t *g_pchist;
  if (g_pchist) g_pchist[(((uint32_t)cpu->k << 16) | (uint16_t)(cpu->pc - 1)) & 0xffffff]++;
#endif
#ifdef SNES_CPU_DEBUG
  /* PC-history ring + breakpoint compare — a debug facility (DumpCpuHistory).
   * It was running unguarded on every opcode, i.e. in release; the device pays
   * ~6 instructions/opcode for a feature only a debugger uses. Guarded off. */
  uint32 cur_pc = ((cpu->k << 16) | cpu->pc - 1);
  pc_hist[pc_hist_ctr] = cur_pc;
  pc_hist_ctr = (pc_hist_ctr + 1) & 7;
  if (cur_pc == pc_bp) {
    opcode += 0;
  }
#endif

restart:
  switch(opcode) {
    case 0x00: { // brk imp
      uint32_t addr = (cpu->k << 16) | cpu->pc | 0x800000;
      switch (opcode = CpuOpcodeHook(addr - 1)) {
      case 0:
#ifdef GNW_SNES_CORE
        /* CpuOpcodeHook is a Super Metroid leftover: return 0 meant "NOP".
         * Soul Blazer uses BRK as a real software interrupt — skip the
         * signature, push PBR/PC/P, take $00:FFE6. Same shape as COP. */
        cpu_readOpcode(cpu);
        if (!cpu->e) cpu_pushByte(cpu, cpu->k);
        cpu_pushWord(cpu, cpu->pc);
        cpu_pushByte(cpu, cpu_getFlags(cpu));
        if (!cpu->e) cpu->cyclesUsed++;
        cpu->i = true;
        cpu->d = false;
        cpu->k = 0;
        cpu->pc = cpu->e ? cpu_readWord(cpu, 0xfffe, 0xffff)
                         : cpu_readWord(cpu, 0xffe6, 0xffe7);
        break;
#else
        break;
#endif
      case 1: // rts
        cpu->pc = cpu_pullWord(cpu) + 1;
        break;
      case 2: // rtl
        cpu->pc = cpu_pullWord(cpu) + 1;
        cpu->k = cpu_pullByte(cpu);
        break;
      case 0xe5:
      case 0xe9:
      case 0xed: cpu->c = 1; goto restart;
      case 0x65:
      case 0x6d: 
      case 0x69:
      case 0x7d:
      case 0x7f: cpu->c = 0; goto restart;
      default:
//        printf("Unk op 0x%x!\n", opcode);
        goto restart;
      }
      break;
    }
    case 0x01: { // ora idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x02: { // cop imm(s)
      cpu_readOpcode(cpu);
      if (!cpu->e) cpu_pushByte(cpu, cpu->k);
      cpu_pushWord(cpu, cpu->pc);
      cpu_pushByte(cpu, cpu_getFlags(cpu));
      if (!cpu->e) cpu->cyclesUsed++; // native mode: 1 extra cycle
      cpu->i = true;
      cpu->d = false;
      cpu->k = 0;
      cpu->pc = cpu->e ? cpu_readWord(cpu, 0xfff4, 0xfff5)
                       : cpu_readWord(cpu, 0xffe4, 0xffe5);
      break;
    }
    case 0x03: { // ora sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x04: { // tsb dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_tsb(cpu, low, high);
      break;
    }
    case 0x05: { // ora dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x06: { // asl dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_asl(cpu, low, high);
      break;
    }
    case 0x07: { // ora idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x08: { // php imp
      cpu_pushByte(cpu, cpu_getFlags(cpu));
      break;
    }
    case 0x09: { // ora imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x0a: { // asla imp
      if(cpu->mf) {
        cpu->c = cpu->a & 0x80;
        cpu->a = (cpu->a & 0xff00) | ((cpu->a << 1) & 0xff);
      } else {
        cpu->c = cpu->a & 0x8000;
        cpu->a <<= 1;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x0b: { // phd imp
      cpu_pushWord(cpu, cpu->dp);
      break;
    }
    case 0x0c: { // tsb abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_tsb(cpu, low, high);
      break;
    }
    case 0x0d: { // ora abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x0e: { // asl abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_asl(cpu, low, high);
      break;
    }
    case 0x0f: { // ora abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x10: { // bpl rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), !cpu->n);
      break;
    }
    case 0x11: { // ora idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x12: { // ora idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x13: { // ora isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x14: { // trb dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_trb(cpu, low, high);
      break;
    }
    case 0x15: { // ora dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x16: { // asl dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_asl(cpu, low, high);
      break;
    }
    case 0x17: { // ora ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x18: { // clc imp
      cpu->c = false;
      break;
    }
    case 0x19: { // ora aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x1a: { // inca imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | ((cpu->a + 1) & 0xff);
      } else {
        cpu->a++;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x1b: { // tcs imp
      cpu->sp = cpu->e ? ((cpu->a & 0xff) | 0x100) : cpu->a;
      break;
    }
    case 0x1c: { // trb abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_trb(cpu, low, high);
      break;
    }
    case 0x1d: { // ora abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x1e: { // asl abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_asl(cpu, low, high);
      break;
    }
    case 0x1f: { // ora alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_ora(cpu, low, high);
      break;
    }
    case 0x20: { // jsr abs
      uint16_t value = cpu_readOpcodeWord(cpu);
      cpu_pushWord(cpu, cpu->pc - 1);
      cpu->pc = value;
      break;
    }
    case 0x21: { // and idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x22: { // jsl abl
      uint16_t value = cpu_readOpcodeWord(cpu);
      uint8_t newK = cpu_readOpcode(cpu);
      cpu_pushByte(cpu, cpu->k);
      cpu_pushWord(cpu, cpu->pc - 1);
      cpu->pc = value;
      cpu->k = newK;
      break;
    }
    case 0x23: { // and sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x24: { // bit dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_bit(cpu, low, high);
      break;
    }
    case 0x25: { // and dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x26: { // rol dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_rol(cpu, low, high);
      break;
    }
    case 0x27: { // and idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x28: { // plp imp
      cpu_setFlags(cpu, cpu_pullByte(cpu));
      break;
    }
    case 0x29: { // and imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x2a: { // rola imp
      int result = (cpu->a << 1) | cpu->c;
      if(cpu->mf) {
        cpu->c = result & 0x100;
        cpu->a = (cpu->a & 0xff00) | (result & 0xff);
      } else {
        cpu->c = result & 0x10000;
        cpu->a = result;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x2b: { // pld imp
      cpu->dp = cpu_pullWord(cpu);
      cpu_setZN(cpu, cpu->dp, false);
      break;
    }
    case 0x2c: { // bit abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_bit(cpu, low, high);
      break;
    }
    case 0x2d: { // and abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x2e: { // rol abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_rol(cpu, low, high);
      break;
    }
    case 0x2f: { // and abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x30: { // bmi rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), cpu->n);
      break;
    }
    case 0x31: { // and idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x32: { // and idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x33: { // and isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x34: { // bit dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_bit(cpu, low, high);
      break;
    }
    case 0x35: { // and dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x36: { // rol dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_rol(cpu, low, high);
      break;
    }
    case 0x37: { // and ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x38: { // sec imp
      cpu->c = true;
      break;
    }
    case 0x39: { // and aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x3a: { // deca imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | ((cpu->a - 1) & 0xff);
      } else {
        cpu->a--;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x3b: { // tsc imp
      cpu->a = cpu->sp;
      cpu_setZN(cpu, cpu->a, false);
      break;
    }
    case 0x3c: { // bit abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_bit(cpu, low, high);
      break;
    }
    case 0x3d: { // and abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x3e: { // rol abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_rol(cpu, low, high);
      break;
    }
    case 0x3f: { // and alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_and(cpu, low, high);
      break;
    }
    case 0x40: { // rti imp
      cpu_setFlags(cpu, cpu_pullByte(cpu));
      cpu->cyclesUsed++; // native mode: 1 extra cycle
      cpu->pc = cpu_pullWord(cpu);
      cpu->k = cpu_pullByte(cpu);
      break;
    }
    case 0x41: { // eor idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x42: { // wdm imm(s)
      cpu_readOpcode(cpu);
      break;
    }
    case 0x43: { // eor sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x44: { // mvp bm
      uint8_t dest = cpu_readOpcode(cpu);
      uint8_t src = cpu_readOpcode(cpu);
      cpu->db = dest;
      cpu_write(cpu, (dest << 16) | cpu->y, cpu_read(cpu, (src << 16) | cpu->x));
      cpu->a--;
      cpu->x--;
      cpu->y--;
      if(cpu->a != 0xffff) {
        cpu->pc -= 3;
      }
      if(cpu->xf) {
        cpu->x &= 0xff;
        cpu->y &= 0xff;
      }
      break;
    }
    case 0x45: { // eor dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x46: { // lsr dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_lsr(cpu, low, high);
      break;
    }
    case 0x47: { // eor idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x48: { // pha imp
      if(cpu->mf) {
        cpu_pushByte(cpu, (uint8_t)cpu->a);
      } else {
        cpu->cyclesUsed++; // m = 0: 1 extra cycle
        cpu_pushWord(cpu, cpu->a);
      }
      break;
    }
    case 0x49: { // eor imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x4a: { // lsra imp
      cpu->c = cpu->a & 1;
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | ((cpu->a >> 1) & 0x7f);
      } else {
        cpu->a >>= 1;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x4b: { // phk imp
      cpu_pushByte(cpu, cpu->k);
      break;
    }
    case 0x4c: { // jmp abs
      cpu->pc = cpu_readOpcodeWord(cpu);
      break;
    }
    case 0x4d: { // eor abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x4e: { // lsr abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_lsr(cpu, low, high);
      break;
    }
    case 0x4f: { // eor abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x50: { // bvc rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), !cpu->v);
      break;
    }
    case 0x51: { // eor idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x52: { // eor idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x53: { // eor isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x54: { // mvn bm
      uint8_t dest = cpu_readOpcode(cpu);
      uint8_t src = cpu_readOpcode(cpu);
      cpu->db = dest;
      cpu_write(cpu, (dest << 16) | cpu->y, cpu_read(cpu, (src << 16) | cpu->x));
      cpu->a--;
      cpu->x++;
      cpu->y++;
      if(cpu->a != 0xffff) {
        cpu->pc -= 3;
      }
      if(cpu->xf) {
        cpu->x &= 0xff;
        cpu->y &= 0xff;
      }
      break;
    }
    case 0x55: { // eor dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x56: { // lsr dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_lsr(cpu, low, high);
      break;
    }
    case 0x57: { // eor ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x58: { // cli imp
      cpu->i = false;
      break;
    }
    case 0x59: { // eor aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x5a: { // phy imp
      if(cpu->xf) {
        cpu_pushByte(cpu, (uint8_t)cpu->y);
      } else {
        cpu->cyclesUsed++; // m = 0: 1 extra cycle
        cpu_pushWord(cpu, cpu->y);
      }
      break;
    }
    case 0x5b: { // tcd imp
      cpu->dp = cpu->a;
      cpu_setZN(cpu, cpu->dp, false);
      break;
    }
    case 0x5c: { // jml abl
      uint16_t value = cpu_readOpcodeWord(cpu);
      uint8_t new_k = cpu_readOpcode(cpu);
#ifndef GNW_SNES_CORE
      /* sm-specific crash trap: Super Metroid's own code funnels fatal states
       * to $80:8573, so jumping there means the PORT broke. On the general
       * core $80:8573 is ordinary ROM any game may legitimately JML to. */
      if (new_k == 0x80 && value == 0x8573) {
        printf("Current PC = 0x%x\n", cpu->k << 16 | cpu->pc);
        DumpCpuHistory();
        Die("The game has crashed!\n");
      }
#endif
      cpu->k = new_k;
      cpu->pc = value;
      break;
    }
    case 0x5d: { // eor abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x5e: { // lsr abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_lsr(cpu, low, high);
      break;
    }
    case 0x5f: { // eor alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_eor(cpu, low, high);
      break;
    }
    case 0x60: { // rts imp
      if (cpu->sp >= cpu->spBreakpoint && cpu->spBreakpoint) {
        int delta = cpu->sp - cpu->spBreakpoint;
        assert(delta == 0 || delta == 2);
        cpu->spBreakpoint = 0;
        if (HookedFunctionRts(delta == 2))
          return;
      }
      cpu->pc = cpu_pullWord(cpu) + 1;
      break;
    }
    case 0x61: { // adc idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x62: { // per rll
      uint16_t value = cpu_readOpcodeWord(cpu);
      cpu_pushWord(cpu, cpu->pc + (int16_t) value);
      break;
    }
    case 0x63: { // adc sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x64: { // stz dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_stz(cpu, low, high);
      break;
    }
    case 0x65: { // adc dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x66: { // ror dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_ror(cpu, low, high);
      break;
    }
    case 0x67: { // adc idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x68: { // pla imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | cpu_pullByte(cpu);
      } else {
        cpu->cyclesUsed++; // 16-bit m: 1 extra cycle
        cpu->a = cpu_pullWord(cpu);
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x69: { // adc imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x6a: { // rora imp
      bool carry = cpu->a & 1;
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | ((cpu->a >> 1) & 0x7f) | (cpu->c << 7);
      } else {
        cpu->a = (cpu->a >> 1) | (cpu->c << 15);
      }
      cpu->c = carry;
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x6b: { // rtl imp
      if (cpu->sp >= cpu->spBreakpoint && cpu->spBreakpoint) {
        assert(cpu->sp == cpu->spBreakpoint);
        cpu->spBreakpoint = 0;
        if (HookedFunctionRts(0))
          return;
      }

      cpu->pc = cpu_pullWord(cpu) + 1;
      cpu->k = cpu_pullByte(cpu);
      break;
    }
    case 0x6c: { // jmp ind
      uint16_t adr = cpu_readOpcodeWord(cpu);
      cpu->pc = cpu_readWord(cpu, adr, (adr + 1) & 0xffff);
      break;
    }
    case 0x6d: { // adc abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x6e: { // ror abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_ror(cpu, low, high);
      break;
    }
    case 0x6f: { // adc abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x70: { // bvs rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), cpu->v);
      break;
    }
    case 0x71: { // adc idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x72: { // adc idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x73: { // adc isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x74: { // stz dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_stz(cpu, low, high);
      break;
    }
    case 0x75: { // adc dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x76: { // ror dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_ror(cpu, low, high);
      break;
    }
    case 0x77: { // adc ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x78: { // sei imp
      cpu->i = true;
      break;
    }
    case 0x79: { // adc aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x7a: { // ply imp
      if(cpu->xf) {
        cpu->y = cpu_pullByte(cpu);
      } else {
        cpu->cyclesUsed++; // 16-bit x: 1 extra cycle
        cpu->y = cpu_pullWord(cpu);
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0x7b: { // tdc imp
      cpu->a = cpu->dp;
      cpu_setZN(cpu, cpu->a, false);
      break;
    }
    case 0x7c: { // jmp iax
      cpu->pc = cpu_adrIax(cpu);
      break;
    }
    case 0x7d: { // adc abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x7e: { // ror abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_ror(cpu, low, high);
      break;
    }
    case 0x7f: { // adc alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_adc(cpu, low, high);
      break;
    }
    case 0x80: { // bra rel
      cpu->pc += (int8_t) cpu_readOpcode(cpu);
      break;
    }
    case 0x81: { // sta idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x82: { // brl rll
      cpu->pc += (int16_t) cpu_readOpcodeWord(cpu);
      break;
    }
    case 0x83: { // sta sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x84: { // sty dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_sty(cpu, low, high);
      break;
    }
    case 0x85: { // sta dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x86: { // stx dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_stx(cpu, low, high);
      break;
    }
    case 0x87: { // sta idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x88: { // dey imp
      if(cpu->xf) {
        cpu->y = (cpu->y - 1) & 0xff;
      } else {
        cpu->y--;
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0x89: { // biti imm(m)
      if(cpu->mf) {
        uint8_t result = (cpu->a & 0xff) & cpu_readOpcode(cpu);
        cpu->z = result == 0;
      } else {
        cpu->cyclesUsed++; // m = 0: 1 extra cycle
        uint16_t result = cpu->a & cpu_readOpcodeWord(cpu);
        cpu->z = result == 0;
      }
      break;
    }
    case 0x8a: { // txa imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | (cpu->x & 0xff);
      } else {
        cpu->a = cpu->x;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x8b: { // phb imp
      cpu_pushByte(cpu, cpu->db);
      break;
    }
    case 0x8c: { // sty abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_sty(cpu, low, high);
      break;
    }
    case 0x8d: { // sta abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x8e: { // stx abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_stx(cpu, low, high);
      break;
    }
    case 0x8f: { // sta abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x90: { // bcc rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), !cpu->c);
      break;
    }
    case 0x91: { // sta idy
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, true);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x92: { // sta idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x93: { // sta isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x94: { // sty dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_sty(cpu, low, high);
      break;
    }
    case 0x95: { // sta dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x96: { // stx dpy
      uint32_t low = 0;
      uint32_t high = cpu_adrDpy(cpu, &low);
      cpu_stx(cpu, low, high);
      break;
    }
    case 0x97: { // sta ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x98: { // tya imp
      if(cpu->mf) {
        cpu->a = (cpu->a & 0xff00) | (cpu->y & 0xff);
      } else {
        cpu->a = cpu->y;
      }
      cpu_setZN(cpu, cpu->a, cpu->mf);
      break;
    }
    case 0x99: { // sta aby
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, true);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x9a: { // txs imp
      cpu->sp = cpu->e ? ((cpu->x & 0xff) | 0x100) : cpu->x;
      break;
    }
    case 0x9b: { // txy imp
      if(cpu->xf) {
        cpu->y = cpu->x & 0xff;
      } else {
        cpu->y = cpu->x;
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0x9c: { // stz abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_stz(cpu, low, high);
      break;
    }
    case 0x9d: { // sta abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0x9e: { // stz abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_stz(cpu, low, high);
      break;
    }
    case 0x9f: { // sta alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_sta(cpu, low, high);
      break;
    }
    case 0xa0: { // ldy imm(x)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, true);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xa1: { // lda idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xa2: { // ldx imm(x)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, true);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xa3: { // lda sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xa4: { // ldy dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xa5: { // lda dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xa6: { // ldx dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xa7: { // lda idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xa8: { // tay imp
      if(cpu->xf) {
        cpu->y = cpu->a & 0xff;
      } else {
        cpu->y = cpu->a;
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0xa9: { // lda imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xaa: { // tax imp
      if(cpu->xf) {
        cpu->x = cpu->a & 0xff;
      } else {
        cpu->x = cpu->a;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xab: { // plb imp
      cpu->db = cpu_pullByte(cpu);
      cpu_setZN(cpu, cpu->db, true);
      break;
    }
    case 0xac: { // ldy abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xad: { // lda abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xae: { // ldx abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xaf: { // lda abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb0: { // bcs rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), cpu->c);
      break;
    }
    case 0xb1: { // lda idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb2: { // lda idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb3: { // lda isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb4: { // ldy dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xb5: { // lda dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb6: { // ldx dpy
      uint32_t low = 0;
      uint32_t high = cpu_adrDpy(cpu, &low);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xb7: { // lda ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xb8: { // clv imp
      cpu->v = false;
      break;
    }
    case 0xb9: { // lda aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xba: { // tsx imp
      if(cpu->xf) {
        cpu->x = cpu->sp & 0xff;
      } else {
        cpu->x = cpu->sp;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xbb: { // tyx imp
      if(cpu->xf) {
        cpu->x = cpu->y & 0xff;
      } else {
        cpu->x = cpu->y;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xbc: { // ldy abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_ldy(cpu, low, high);
      break;
    }
    case 0xbd: { // lda abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xbe: { // ldx aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_ldx(cpu, low, high);
      break;
    }
    case 0xbf: { // lda alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_lda(cpu, low, high);
      break;
    }
    case 0xc0: { // cpy imm(x)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, true);
      cpu_cpy(cpu, low, high);
      break;
    }
    case 0xc1: { // cmp idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xc2: { // rep imm(s)
      cpu_setFlags(cpu, cpu_getFlags(cpu) & ~cpu_readOpcode(cpu));
      break;
    }
    case 0xc3: { // cmp sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xc4: { // cpy dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_cpy(cpu, low, high);
      break;
    }
    case 0xc5: { // cmp dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xc6: { // dec dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_dec(cpu, low, high);
      break;
    }
    case 0xc7: { // cmp idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xc8: { // iny imp
      if(cpu->xf) {
        cpu->y = (cpu->y + 1) & 0xff;
      } else {
        cpu->y++;
      }
      cpu_setZN(cpu, cpu->y, cpu->xf);
      break;
    }
    case 0xc9: { // cmp imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xca: { // dex imp
      if(cpu->xf) {
        cpu->x = (cpu->x - 1) & 0xff;
      } else {
        cpu->x--;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xcb: { // wai imp
      /* Halt until an interrupt. Super Metroid's reimplementation never runs a
       * 65816 instruction, so this was left as assert(0) — and with -DNDEBUG that
       * is a no-op, which means WAI silently did nothing. Games that idle their
       * main loop on WAI (most of them) then run their vblank code at full tilt
       * against a screen that is not in vblank. */
      cpu->waiting = true;
      cpu->cyclesUsed = 3;
      break;
    }
    case 0xcc: { // cpy abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_cpy(cpu, low, high);
      break;
    }
    case 0xcd: { // cmp abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xce: { // dec abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_dec(cpu, low, high);
      break;
    }
    case 0xcf: { // cmp abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd0: { // bne rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), !cpu->z);
      break;
    }
    case 0xd1: { // cmp idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd2: { // cmp idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd3: { // cmp isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd4: { // pei dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_pushWord(cpu, cpu_readWord(cpu, low, high));
      break;
    }
    case 0xd5: { // cmp dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd6: { // dec dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_dec(cpu, low, high);
      break;
    }
    case 0xd7: { // cmp ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xd8: { // cld imp
      cpu->d = false;
      break;
    }
    case 0xd9: { // cmp aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xda: { // phx imp
      if(cpu->xf) {
        cpu_pushByte(cpu, (uint8_t)cpu->x);
      } else {
        cpu->cyclesUsed++; // m = 0: 1 extra cycle
        cpu_pushWord(cpu, cpu->x);
      }
      break;
    }
    case 0xdb: { // stp imp
      cpu->stopped = true;
      break;
    }
    case 0xdc: { // jml ial
      uint16_t adr = cpu_readOpcodeWord(cpu);
      cpu->pc = cpu_readWord(cpu, adr, (adr + 1) & 0xffff);
      cpu->k = cpu_read(cpu, (adr + 2) & 0xffff);
      break;
    }
    case 0xdd: { // cmp abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xde: { // dec abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_dec(cpu, low, high);
      break;
    }
    case 0xdf: { // cmp alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_cmp(cpu, low, high);
      break;
    }
    case 0xe0: { // cpx imm(x)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, true);
      cpu_cpx(cpu, low, high);
      break;
    }
    case 0xe1: { // sbc idx
      uint32_t low = 0;
      uint32_t high = cpu_adrIdx(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xe2: { // sep imm(s)
      cpu_setFlags(cpu, cpu_getFlags(cpu) | cpu_readOpcode(cpu));
      break;
    }
    case 0xe3: { // sbc sr
      uint32_t low = 0;
      uint32_t high = cpu_adrSr(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xe4: { // cpx dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_cpx(cpu, low, high);
      break;
    }
    case 0xe5: { // sbc dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xe6: { // inc dp
      uint32_t low = 0;
      uint32_t high = cpu_adrDp(cpu, &low);
      cpu_inc(cpu, low, high);
      break;
    }
    case 0xe7: { // sbc idl
      uint32_t low = 0;
      uint32_t high = cpu_adrIdl(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xe8: { // inx imp
      if(cpu->xf) {
        cpu->x = (cpu->x + 1) & 0xff;
      } else {
        cpu->x++;
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xe9: { // sbc imm(m)
      uint32_t low = 0;
      uint32_t high = cpu_adrImm(cpu, &low, false);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xea: { // nop imp
      // no operation
      break;
    }
    case 0xeb: { // xba imp
      uint8_t low = cpu->a & 0xff;
      uint8_t high = cpu->a >> 8;
      cpu->a = (low << 8) | high;
      cpu_setZN(cpu, high, true);
      break;
    }
    case 0xec: { // cpx abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_cpx(cpu, low, high);
      break;
    }
    case 0xed: { // sbc abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xee: { // inc abs
      uint32_t low = 0;
      uint32_t high = cpu_adrAbs(cpu, &low);
      cpu_inc(cpu, low, high);
      break;
    }
    case 0xef: { // sbc abl
      uint32_t low = 0;
      uint32_t high = cpu_adrAbl(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf0: { // beq rel
      cpu_doBranch(cpu, cpu_readOpcode(cpu), cpu->z);
      break;
    }
    case 0xf1: { // sbc idy(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrIdy(cpu, &low, false);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf2: { // sbc idp
      uint32_t low = 0;
      uint32_t high = cpu_adrIdp(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf3: { // sbc isy
      uint32_t low = 0;
      uint32_t high = cpu_adrIsy(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf4: { // pea imm(l)
      uint16_t w = cpu_readOpcodeWord(cpu);
      if (w == 0xC2AE)
        cpu->y = 0;
      cpu_pushWord(cpu, w);
  
      break;
    }
    case 0xf5: { // sbc dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf6: { // inc dpx
      uint32_t low = 0;
      uint32_t high = cpu_adrDpx(cpu, &low);
      cpu_inc(cpu, low, high);
      break;
    }
    case 0xf7: { // sbc ily
      uint32_t low = 0;
      uint32_t high = cpu_adrIly(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xf8: { // sed imp
      cpu->d = true;
      break;
    }
    case 0xf9: { // sbc aby(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAby(cpu, &low, false);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xfa: { // plx imp
      if(cpu->xf) {
        cpu->x = cpu_pullByte(cpu);
      } else {
        cpu->cyclesUsed++; // 16-bit x: 1 extra cycle
        cpu->x = cpu_pullWord(cpu);
      }
      cpu_setZN(cpu, cpu->x, cpu->xf);
      break;
    }
    case 0xfb: { // xce imp
      bool temp = cpu->c;
      cpu->c = cpu->e;
      cpu->e = temp;
      cpu_setFlags(cpu, cpu_getFlags(cpu)); // updates x and m flags, clears upper half of x and y if needed
      break;
    }
    case 0xfc: { // jsr iax
      uint16_t value = cpu_adrIax(cpu);
      cpu_pushWord(cpu, cpu->pc - 1);
      cpu->pc = value;
      break;
    }
    case 0xfd: { // sbc abx(r)
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, false);
      cpu_sbc(cpu, low, high);
      break;
    }
    case 0xfe: { // inc abx
      uint32_t low = 0;
      uint32_t high = cpu_adrAbx(cpu, &low, true);
      cpu_inc(cpu, low, high);
      break;
    }
    case 0xff: { // sbc alx
      uint32_t low = 0;
      uint32_t high = cpu_adrAlx(cpu, &low);
      cpu_sbc(cpu, low, high);
      break;
    }
  }
}