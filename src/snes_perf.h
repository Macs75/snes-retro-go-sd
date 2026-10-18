/* SNES_PERF (make PERF=1): per-component cycle profiler for the device.
 *
 * Every SNES_PERF_FRAMES emulated frames the main loop logs PROF/... lines:
 * where an emulated frame's cycles go (65816 CPU, PPU, APU split into SPC700
 * and DSP, DMA) and the rest of the loop (present, overlay, swap, audio
 * submit, input, pacing), against the frame budget at the measured clock.
 *
 * The probes read DWT_CYCCNT directly (one load) and sit at scanline / APU
 * catch-up / DMA-burst granularity, never per pixel or per instruction.
 * CPU time is the remainder of the emulation after PPU, APU and DMA, so it
 * includes the event scheduler and the bus. DWT stops in __WFI, so the
 * pacing wait is measured in milliseconds of HAL_GetTick, not cycles.
 *
 * Without SNES_PERF every macro compiles to nothing. */
#pragma once

#ifdef SNES_PERF

#include <stdint.h>

#define SNES_PERF_FRAMES 120u

#define SNES_PERF_DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)
static inline uint32_t snes_perf_now(void) { return SNES_PERF_DWT_CYCCNT; }

/* per-window accumulators (cycles), reset after each report */
extern uint32_t snes_perf_ppu, snes_perf_ppu_lines;
extern uint32_t snes_perf_apu, snes_perf_apu_calls;   /* apu_run, inclusive (SPC700 + DSP) */
extern uint32_t snes_perf_dsp;                        /* DSP ticks inside apu_run */
extern uint32_t snes_perf_dma, snes_perf_dma_calls;   /* DMA, minus any APU work inside it */
extern uint32_t snes_perf_gsu;                        /* Super FX: every GSU slice (line, vblank, start) */
extern uint32_t snes_perf_ops, snes_perf_gsu_reads;   /* 65816 opcodes run; CPU reads of GSU registers */

/* the PPU split (cycles, both inside snes_perf_ppu): sprite evaluation runs
 * on every line of every frame; the rest only on drawn lines. main/sub are
 * the two PpuDrawBackgrounds passes, which contain bg4/bg2/bgx/spr. comp
 * contains cwin (the colour window and line setup, before the first pixel)
 * and span (the colour-math blend spans, one probe pair per window region). */
enum { SNES_PERF_PPU_EVAL, SNES_PERF_PPU_PRE, SNES_PERF_PPU_MAIN, SNES_PERF_PPU_SUB,
       SNES_PERF_PPU_COMP, SNES_PERF_PPU_BLIT,
       SNES_PERF_PPU_BG4, SNES_PERF_PPU_BG2, SNES_PERF_PPU_BGX, SNES_PERF_PPU_SPR,
       SNES_PERF_PPU_CWIN, SNES_PERF_PPU_SPAN,
       SNES_PERF_PPU_N };
extern uint32_t snes_perf_ppu_part[SNES_PERF_PPU_N];
/* line counts: drawn, with a subscreen pass, with sprites, with colour math
 * on (the per-pixel compose path), and drawn lines per BG mode */
extern uint32_t snes_perf_ppu_drawn, snes_perf_ppu_sub_lines, snes_perf_ppu_spr_lines;
extern uint32_t snes_perf_ppu_math_lines, snes_perf_ppu_mode[8];
/* line-start table updates: lines with CGRAM entries updated, full colour-math rebuilds */
extern uint32_t snes_perf_ppu_cgram_lines, snes_perf_ppu_math_rebuilds;
/* PERF_CPU=1, a diagnostic for the 65816 side, distorting by design (two
 * timer reads per opcode): the "cpu" remainder split into the engine
 * itself (per opcode, minus any APU/DMA/PPU work it triggered), the spin
 * bake, the scheduler's event handler (snes_handle_pos_stuff minus the
 * PPU/DMA/APU inside it) and the rest (the scheduler loops). Plus counts:
 * opcodes, memory accesses, bus accesses that leave the ITCM fast paths,
 * ROM page-cache refills, opcodes that fall back to the C interpreter. */
#ifdef SNES_PERF_CPU
extern uint32_t snes_perf_cpu_eng, snes_perf_cpu_ops, snes_perf_cpu_memops;
extern uint32_t snes_perf_cpu_bake, snes_perf_cpu_events, snes_perf_cpu_dmaloop;
extern uint32_t snes_perf_cpu_rd_slow, snes_perf_cpu_rd_page, snes_perf_cpu_wr_slow;
#define SNES_PERF_CPU_INC(c) ((c)++)
#else
#define SNES_PERF_CPU_INC(c) do { } while (0)
#endif

/* PERF_GSU=1, a diagnostic for the Super FX figure (two timer reads per chain
 * call): cycles inside the compiled chains, GSU top-level instructions run by
 * the chains / in all, and the same for the vblank catch-up alone (gsu.c).
 * PERF_GSU_PLOT=1 adds the cycles inside PLOT and the PLOT count (two timer
 * reads per pixel). */
#ifdef SNES_PERF_GSU
extern uint32_t snes_perf_gsu_plot, snes_perf_gsu_plots, snes_perf_gsu_chain;
extern uint32_t snes_perf_gsu_chain_ops, snes_perf_gsu_ops;
extern uint32_t snes_perf_gsu_vbl, snes_perf_gsu_vbl_ops, snes_perf_gsu_vbl_plot, snes_perf_gsu_vbl_chain, snes_perf_gsu_vbl_chain_ops;
#endif

/* PERF_SPC=1, a diagnostic for the SPC700 figure (two timer reads per call
 * into the Thumb-2 engine): cycles inside the engine (the batches of
 * spc_thumb2_run net of their DSP ticks, and the single steps of
 * spc_runOpcode) and the number of those calls; cycles inside the C fallback
 * (spc_fallback) and its count; the idle skip's count. spc700 minus "step" is
 * apu_run's own loop. */
#ifdef SNES_PERF_SPC
extern uint32_t snes_perf_spc_step, snes_perf_spc_ops, snes_perf_spc_fb, snes_perf_spc_fbs;
extern uint32_t snes_perf_spc_skips;
#endif

/* the split costs probes on every drawn line: only with PERF_PPU=1 */
#ifdef SNES_PERF_PPU
#define SNES_PERF_PPU_BEGIN(t)     uint32_t t = snes_perf_now()
#define SNES_PERF_PPU_END(part, t) (snes_perf_ppu_part[(part)] += snes_perf_now() - (t))
#define SNES_PERF_COUNT(c)         ((c)++)
#else
#define SNES_PERF_PPU_BEGIN(t)     do { } while (0)
#define SNES_PERF_PPU_END(part, t) do { } while (0)
#define SNES_PERF_COUNT(c)         do { } while (0)
#endif

#define SNES_PERF_BEGIN(t)        uint32_t t = snes_perf_now()
#define SNES_PERF_END(acc, t)     ((acc) += snes_perf_now() - (t))
/* a DMA burst may reach the APU ports (catch-up): keep DMA exclusive */
#define SNES_PERF_DMA_BEGIN(t, a) uint32_t t = snes_perf_now(), a = snes_perf_apu
#define SNES_PERF_DMA_END(t, a)   do { snes_perf_dma += snes_perf_now() - (t) - (snes_perf_apu - (a)); \
                                       snes_perf_dma_calls++; } while (0)

#else

#define SNES_PERF_BEGIN(t)        do { } while (0)
#define SNES_PERF_END(acc, t)     do { } while (0)
#define SNES_PERF_DMA_BEGIN(t, a) do { } while (0)
#define SNES_PERF_DMA_END(t, a)   do { } while (0)
#define SNES_PERF_CPU_INC(c)      do { } while (0)
#define SNES_PERF_PPU_BEGIN(t)     do { } while (0)
#define SNES_PERF_PPU_END(part, t) do { } while (0)
#define SNES_PERF_COUNT(c)         do { } while (0)

#endif
