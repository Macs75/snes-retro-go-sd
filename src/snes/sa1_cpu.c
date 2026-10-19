/* The SA-1's 65816: cpu.c compiled a second time, on the SA-1's bus (sa1.c).
 * One interpreter, two instances -- every opcode fix lands in both. The main
 * CPU's machinery stays out of this copy: the Thumb-2 engine and its bridges,
 * the ITCM sections, the spin skip, the recompiler, the census. */
#ifdef SNES_SA1_T2
#define CPU_SA1_T2 1          /* the device: the SA-1's engine calls the fallback below */
#endif
#undef SNES_THUMB2_CPU
#undef SNES_BUS_IN_ITCM
#undef SNES_SPIN_SKIP
#undef SNES_PC_HISTOGRAM
#undef SNES_CPU_DEBUG
#undef SNES_OP_CENSUS
#define SNES_OP_CENSUS 0
#define CPU_SA1 1
#define CPU_PCH_ENV "SA1_PCHIST"

#define cpu_init sa1cpu_init
#define cpu_free sa1cpu_free
#define cpu_reset sa1cpu_reset
#define cpu_saveload sa1cpu_saveload
#define cpu_runOpcode sa1cpu_runOpcode
#define cpu_getFlags sa1cpu_getFlags
#define cpu_setFlags sa1cpu_setFlags
#define snes_fs_cpu sa1_fs_cpu
#define pc_hist sa1cpu_pc_hist
#define pc_hist_ctr sa1cpu_pc_hist_ctr
#define pc_bp sa1cpu_pc_bp
#define DumpCpuHistory sa1cpu_DumpCpuHistory
#define cpu_thumb2_fallback sa1cpu_thumb2_fallback

#include "sa1.h"
#include "cpu.c"

#ifdef CPU_SA1_T2
/* the SA-1's engine charges an opcode it leaves to C from this (the main
 * engine's table exists only with SNES_THUMB2_CPU) */
const uint8_t sa1_cycles_per_opcode[256] = { SNES_CYCLES_PER_OPCODE };
#endif
