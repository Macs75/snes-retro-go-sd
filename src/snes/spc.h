#ifndef SPC_H
#define SPC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Spc Spc;

#include "apu.h"
#include "saveload.h"

struct Spc {
  Apu* apu;
  // registers
  uint8_t a;
  uint8_t x;
  uint8_t y;
  uint8_t sp;
  uint16_t pc;
  // flags
  bool c;
  bool z;
  bool v;
  bool n;
  bool i;
  bool h;
  bool p;
  bool b;
  // stopping
  bool stopped;
  // internal use
  uint8_t cyclesUsed; // indicates how many cycles an opcode used
};

Spc* spc_init(Apu* apu);
void spc_free(Spc* spc);
void spc_reset(Spc* spc);
int spc_runOpcode(Spc* spc);
void spc_saveload(Spc *spc, SaveLoadFunc *func, void *ctx);

// Thumb-2 SPC700 engine (defined in thumb2/spc_thumb2.S when SPC_THUMB2_SPC=1).
// Runs whole opcodes from apu->cycles, with the DSP ticks they span, while
// the budget lasts; returns the budget left. It stops early at the idle-wait
// pattern, after a STOP/SLEEP, or at an opcode longer than the budget left
// (left in apu->cpuCyclesLeft). budget 0 runs exactly one opcode and commits
// nothing: its cycles are in cyclesUsed. See the .S header.
int spc_thumb2_run(Spc* spc, int budget);
// The engine's C fallback: spc_doOpcode on an already fetched, charged opcode.
void spc_fallback(Spc* spc, uint8_t opcode);

#endif
