#ifndef CPU_THUMB2_OFFSETS_H
#define CPU_THUMB2_OFFSETS_H
/*
 * Compile-checked Cpu field offsets for the Thumb-2 65816 engine.
 *
 * Values were emitted by the device compiler (arm-none-eabi-gcc -mcpu=cortex-m7
 * -mthumb) via offsetof(), see tools/probe at the bottom. They are NOT host
 * layout: a 64-bit host gives void* offset 0 size 8, this device gives size 4.
 *
 * cpu_thumb2_offsets_check.c _Static_asserts every entry against offsetof(Cpu,
 * field), so any struct drift fails the SNES_THUMB2_CPU=1 build at compile
 * time. The .S engine #includes this file; never hard-code these in assembly.
 */
#define CPU_MEM          0
#define CPU_MEMTYPE      4
#define CPU_A            6
#define CPU_X            8
#define CPU_Y           10
#define CPU_SP          12
#define CPU_PC          14
#define CPU_DP          16
#define CPU_K           18
#define CPU_DB          19
#define CPU_C           20
#define CPU_Z           21
#define CPU_V           22
#define CPU_N           23
#define CPU_I           24
#define CPU_D           25
#define CPU_XF          26
#define CPU_MF          27
#define CPU_E           28
#define CPU_IRQWANTED   29
#define CPU_NMIWANTED   30
#define CPU_WAITING     31
#define CPU_STOPPED     32
#define CPU_CYCLESUSED  33
#define CPU_SPBREAKPOINT 34
#define CPU_IN_EMU      36

/* ---- Snes struct offsets (for the ROM fetch-page cache inline fast-path) ----
 * Verified by the device compiler (arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb)
 * via offsetof(Snes, field). The Thumb-2 engine reads these through the Cpu.mem
 * pointer (which is Snes*). The rig's RigBus places romPageBase/romPageTag at
 * these exact offsets (sentinel-initialized) so the inline check always misses
 * and falls through to the rig's snes_cpuRead. */
#define SNES_CPUCYCLESLEFT  60
#define SNES_CPUMEMOPS      61
#define SNES_ROMPAGEBASE   112
#define SNES_ROMPAGETAG    116
#define SNES_RAM            44   /* uint8_t *ram, for the WRAM inline path */
/* the batch entry (snes_thumb2_batch): the scheduler state it keeps */
#define SNES_DMA            20   /* Dma *dma */
#define SNES_HPOS           52
#define SNES_VPOS           54
#define SNES_APUDOTS        72   /* uint32_t apuDotsAccum */
#define SNES_HIRQEN         76
#define SNES_VIRQEN         77
#define SNES_HTIMER         80
#define SNES_VTIMER         82
#define SNES_INIRQ          85
#define DMA_HDMATIMER      212   /* uint16_t */
#define DMA_BUSY           220   /* bool */

#endif
