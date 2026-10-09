
#ifndef CPU_H
#define CPU_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "saveload.h"

typedef struct Cpu Cpu;

struct Cpu {
  // reference to memory handler, for reading//writing
  void* mem;
  uint8_t memType; // used to define which type mem is
  // registers
  uint16_t a;
  uint16_t x;
  uint16_t y;
  uint16_t sp;
  uint16_t pc;
  uint16_t dp; // direct page (D)
  uint8_t k; // program bank (PB)
  uint8_t db; // data bank (B)
  // flags
  bool c;
  bool z;
  bool v;
  bool n;
  bool i;
  bool d;
  bool xf;
  bool mf;
  bool e;
  // interrupts
  bool irqWanted;
  bool nmiWanted;
  // power state (WAI/STP)
  bool waiting;
  bool stopped;
  // internal use
  uint8_t cyclesUsed; // indicates how many cycles an opcode used
  uint16_t spBreakpoint;
  bool in_emu;
};

extern struct Cpu *g_cpu;
bool HookedFunctionRts(int is_long);

Cpu* cpu_init(void* mem, int memType);
void cpu_free(Cpu* cpu);
void cpu_reset(Cpu* cpu);
#ifdef SNES_BUS_IN_ITCM
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
int cpu_runOpcode(Cpu* cpu);
#ifdef SNES_THUMB2_CPU
/* The C interpreter exposed as oracle/fallback for the Thumb-2 dispatcher.
 * Nothing on the Thumb-2 path calls it, so it is not in ITCM. */
int cpu_runOpcode_c(Cpu* cpu);
/* Thumb-2 fast path. The caller (try path) has already fetched the opcode and
   charged cyclesPerOpcode[opcode]; the step path fetches it itself. Either way,
   on entry pc points just past the opcode byte. Each native handler performs
   exactly the operand/data fetches and extra cycle charges its opcode semantics
   require (e.g. branches, immediate ALU/loads, and REP/SEP consume their
   operands at pc), so a handler is correct on BOTH paths: pc consistently
   points at the operand. Returns 1 if handled (cpu state mutated in place), 0
   to fall back to C, which then runs cpu_doOpcode on the already-fetched byte. */
int snes_thumb2_try(Cpu* cpu, uint8_t opcode);
/* Stage 2 fetch-dispatch entry. Fetches EXACTLY ONE opcode by calling the real
   snes_cpuRead(mem, (k<<16)|pc), increments the 16-bit pc once, charges
   cyclesUsed from snes_cycles_per_opcode, and runs the handler. Returns -1 if
   handled, or the opcode byte (0..255) if unsupported — the caller then calls
   cpu_doOpcode on that already-fetched byte with no second fetch or cycle charge. */
int snes_thumb2_step(Cpu* cpu);
/* Stage 4 whole-opcode entry: cpu_runOpcode's pre-work AND the fetch-dispatch
   in one frame, returning cyclesUsed exactly as cpu_runOpcode does. Anything
   the fast path does not model tail-branches to cpu_runOpcode before the fetch,
   so the two are interchangeable at every call site. Use CPU_RUN_OPCODE rather
   than naming it: a build without the engine has no such symbol. */
int snes_thumb2_run(Cpu* cpu);
int cpu_thumb2_fallback(Cpu* cpu, uint32_t opcode);
#ifndef SNES_OP_CENSUS
#define SNES_OP_CENSUS 0
#endif
#if SNES_OP_CENSUS
/* How much of the guest's instruction stream does the Thumb-2 engine actually
 * execute, and what is left? snes_thumb2_run is the single entry per opcode and
 * cpu_thumb2_fallback the single exit to C, so counting both gives the coverage
 * rate exactly, and a 256-entry histogram of the fallbacks names what to add. */
extern uint32_t g_op_total;
extern uint32_t g_op_fallback;
extern uint32_t g_op_fbhist[256];
static inline int snes_thumb2_run_censused(Cpu* cpu) { g_op_total++; return snes_thumb2_run(cpu); }
#define CPU_RUN_OPCODE(cpu) snes_thumb2_run_censused(cpu)
#else
#define CPU_RUN_OPCODE(cpu) snes_thumb2_run(cpu)
#endif
#else
#define CPU_RUN_OPCODE(cpu) cpu_runOpcode(cpu)
#endif
uint8_t cpu_getFlags(Cpu *cpu);
void cpu_setFlags(Cpu *cpu, uint8_t val);
void cpu_saveload(Cpu *cpu, SaveLoadFunc *func, void *ctx);
#endif
