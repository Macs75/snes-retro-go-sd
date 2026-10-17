/* Compile-time guard: fail the SPC_THUMB2_SPC=1 build if Spc/Apu's layout
 * drifts from the numeric constants the Thumb-2 SPC700 engine bakes in.
 * No runtime cost -- this TU is all _Static_assert, so it lowers to an
 * empty object.  Kept in its own TU so the asserts survive even when the
 * engine's own objects are excluded by conditional compilation. */
#include <stddef.h>
#include "snes/spc.h"
#include "snes/apu.h"
#include "spc_thumb2_offsets.h"

#define SPC_CK(field, val) \
    _Static_assert(offsetof(Spc, field) == (val), \
                   "Spc::" #field " offset drift -- update spc_thumb2_offsets.h")

SPC_CK(apu,          SPC_OFF_APU);
SPC_CK(a,            SPC_OFF_A);
SPC_CK(x,            SPC_OFF_X);
SPC_CK(y,            SPC_OFF_Y);
SPC_CK(sp,           SPC_OFF_SP);
SPC_CK(pc,           SPC_OFF_PC);
SPC_CK(c,            SPC_OFF_C);
SPC_CK(z,            SPC_OFF_Z);
SPC_CK(v,            SPC_OFF_V);
SPC_CK(n,            SPC_OFF_N);
SPC_CK(i,            SPC_OFF_I);
SPC_CK(h,            SPC_OFF_H);
SPC_CK(p,            SPC_OFF_P);
SPC_CK(b,            SPC_OFF_B);
SPC_CK(stopped,      SPC_OFF_STOPPED);
SPC_CK(cyclesUsed,   SPC_OFF_CYCLESUSED);

#define APU_CK(field, val) \
    _Static_assert(offsetof(Apu, field) == (val), \
                   "Apu::" #field " offset drift -- update spc_thumb2_offsets.h")

APU_CK(spc,          APU_OFF_SPC);
APU_CK(dsp,          APU_OFF_DSP);
APU_CK(ram,          APU_OFF_RAM);
APU_CK(romReadable,  APU_OFF_ROMREADABLE);
APU_CK(cycles,       APU_OFF_CYCLES);
APU_CK(cpuCyclesLeft, APU_OFF_CPUCYCLESLEFT);
_Static_assert(APU_OFF_CYCLES_PAGE + APU_OFF_CYCLES_IN_PAGE == APU_OFF_CYCLES, "APU_OFF_CYCLES split");
