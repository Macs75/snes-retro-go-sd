#ifndef SPC_THUMB2_OFFSETS_H
#define SPC_THUMB2_OFFSETS_H
/*
 * Compile-checked Spc/Apu field offsets for the Thumb-2 SPC700 engine.
 *
 * Values are for the DEVICE compiler (arm-none-eabi-gcc -mcpu=cortex-m7
 * -mthumb) layout: pointers are 4 bytes, _Bool is 1 byte.  A 64-bit host
 * gives different sizes, so these must come from the device compiler.
 *
 * spc_thumb2_offsets_check.c _Static_asserts every entry against
 * offsetof(Spc, field) / offsetof(Apu, field), so any struct drift fails
 * the SPC_THUMB2_SPC=1 build at compile time.  The .S engine #includes this
 * file; never hard-code these in assembly.
 *
 * Constants use the SPC_OFF_/APU_OFF_ prefix because SPC_H is the spc.h
 * include guard and several other short names risk colliding with the
 * SNES codebase's macros.
 */

/* ---- Spc struct (spc.h) ---- */
#define SPC_OFF_APU          0    /* Apu* apu          */
#define SPC_OFF_A            4    /* uint8_t a         */
#define SPC_OFF_X            5    /* uint8_t x         */
#define SPC_OFF_Y            6    /* uint8_t y         */
#define SPC_OFF_SP           7    /* uint8_t sp        */
#define SPC_OFF_PC           8    /* uint16_t pc       */
#define SPC_OFF_C           10    /* bool c            */
#define SPC_OFF_Z           11    /* bool z            */
#define SPC_OFF_V           12    /* bool v            */
#define SPC_OFF_N           13    /* bool n            */
#define SPC_OFF_I           14    /* bool i            */
#define SPC_OFF_H           15    /* bool h            */
#define SPC_OFF_P           16    /* bool p            */
#define SPC_OFF_B           17    /* bool b            */
#define SPC_OFF_STOPPED     18    /* bool stopped      */
#define SPC_OFF_CYCLESUSED  19    /* uint8_t cyclesUsed*/

/* ---- Apu struct (apu.h) ---- */
#define APU_OFF_SPC           0          /* Spc* spc          */
#define APU_OFF_DSP           4          /* Dsp* dsp          */
#define APU_OFF_RAM           8          /* uint8_t ram[0x10000] -- base offset */
#define APU_OFF_ROMREADABLE   0x10008    /* bool romReadable  */
#define APU_OFF_CYCLES        0x1000C    /* uint32_t cycles   */
#define APU_OFF_CYCLES_PAGE   0x10000    /* APU_OFF_CYCLES in two Thumb-2 immediates */
#define APU_OFF_CYCLES_IN_PAGE 0xC
#define APU_OFF_CPUCYCLESLEFT 0x10029    /* uint8_t cpuCyclesLeft */
/* ram occupies [8 .. 0x10007] */

#endif
