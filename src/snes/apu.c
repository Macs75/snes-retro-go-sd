
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "../snes_perf.h"
#include <stddef.h>


#include "apu.h"
#include "snes.h"
#include "spc.h"
#include "dsp.h"
#include "tracing.h"
#ifdef TARGET_GNW
#include "gw_malloc.h"
#endif

static const uint8_t bootRom[0x40] = {
  0xcd, 0xef, 0xbd, 0xe8, 0x00, 0xc6, 0x1d, 0xd0, 0xfc, 0x8f, 0xaa, 0xf4, 0x8f, 0xbb, 0xf5, 0x78,
  0xcc, 0xf4, 0xd0, 0xfb, 0x2f, 0x19, 0xeb, 0xf4, 0xd0, 0xfc, 0x7e, 0xf4, 0xd0, 0x0b, 0xe4, 0xf5,
  0xcb, 0xf4, 0xd7, 0x00, 0xfc, 0xd0, 0xf3, 0xab, 0x01, 0x10, 0xef, 0x7e, 0xf4, 0x10, 0xeb, 0xba,
  0xf6, 0xda, 0x00, 0xba, 0xf4, 0xc4, 0xf4, 0xdd, 0x5d, 0xd0, 0xdb, 0x1f, 0x00, 0x00, 0xc0, 0xff
};

Apu* apu_init(void) {
#ifdef TARGET_GNW
  /* jshsakura put Apu (~66 KB, mostly ARAM) on uncached AHB so SPC traffic
   * did not thrash the 16 KiB D-cache against WRAM/VRAM. This firmware's AHB
   * heap is only ~56 KiB, so DTCM is the replacement: zero-wait, not cached.
   * ABI ALLOC already zeroes (dtc_calloc). */
  Apu* apu = dtc_malloc(sizeof(Apu));
  if (!apu || (uintptr_t)apu == (uintptr_t)-1)
    return NULL;
#else
  Apu* apu = malloc(sizeof(Apu));
#endif
  apu->spc = spc_init(apu);
  apu->dsp = dsp_init(apu->ram);
  return apu;
}

void apu_free(Apu* apu) {
#ifndef TARGET_GNW
  spc_free(apu->spc);
  dsp_free(apu->dsp);
  free(apu);
#else
  /* Apu + Spc/Dsp live on the DTCM bump — no free. */
  (void)apu;
#endif
}

/* LAZY SPC TIMERS. The three timers are a pure function of elapsed SPC
 * cycles; the sound program only observes them through $FD-$FF (counter
 * read, which also clears it) and changes them through $F1 (enable) and
 * $FA-$FC (target). So instead of advancing them after every opcode,
 * apu_timers_sync() advances them by the cycles elapsed since the last sync,
 * just before each of those accesses, the idle skip's Timer 0 test and a
 * savestate. A register access inside an opcode sees the state as of the
 * opcode's start in both versions (apu->cycles moves after the opcode), so
 * the result is the per-step update's, exactly.
 *
 * One closed form covers every case of a tick: the counter steps after
 * s0 = ((target - divider - 1) & 0xff) + 1 ticks (divider climbs, wrapping at
 * 256, until it equals target), then every `target` ticks (256 for target 0).
 * For divider < target that is the old total/target, total%target.
 *
 * The sync point is a static, not an Apu field: the struct layout (savestate,
 * the Thumb-2 engine's offsets) stays as it is. */
static uint32_t apu_timer_sync;

static void apu_timer_advance(Timer* t, int R, uint32_t n)
{
  uint32_t C = t->cycles;
  uint32_t ticks = (C < n) ? ((n - 1 - C) / (uint32_t)R + 1) : 0;
  if (!ticks) { t->cycles = (uint8_t)(C - n); return; }
  if (t->enabled) {
    uint32_t s0 = (((uint32_t)t->target - t->divider - 1) & 0xffu) + 1;
    if (ticks < s0) {
      t->divider = (uint8_t)(t->divider + ticks);
    } else {
      uint32_t period = t->target ? t->target : 256u;
      uint32_t rest = ticks - s0;
      t->counter = (uint8_t)((t->counter + 1 + rest / period) & 0xf);
      t->divider = (uint8_t)(rest % period);
    }
  }
  uint32_t lastK = C + (ticks - 1) * (uint32_t)R;
  t->cycles = (uint8_t)((uint32_t)R + lastK - n);   /* value after the last reload */
}

static inline void apu_timers_sync(Apu* apu)
{
  uint32_t n = apu->cycles - apu_timer_sync;
  if (!n) return;
  apu_timer_sync = apu->cycles;
  apu_timer_advance(&apu->timer[0], 128, n);
  apu_timer_advance(&apu->timer[1], 128, n);
  apu_timer_advance(&apu->timer[2], 16, n);
}

void apu_reset(Apu* apu) {
  apu->romReadable = true; // before resetting spc, because it reads reset vector from it
  spc_reset(apu->spc);
  dsp_reset(apu->dsp);
  memset(apu->ram, 0, sizeof(apu->ram));
  apu->dspAdr = 0;
  apu->cycles = 0;
  memset(apu->inPorts, 0, sizeof(apu->inPorts));
  memset(apu->outPorts, 0, sizeof(apu->outPorts));
  for(int i = 0; i < 3; i++) {
    apu->timer[i].cycles = 0;
    apu->timer[i].divider = 0;
    apu->timer[i].target = 0;
    apu->timer[i].counter = 0;
    apu->timer[i].enabled = false;
  }
  apu->cpuCyclesLeft = 7;
  apu->hist.count = 0;
  apu_timer_sync = apu->cycles;
}

void apu_saveload(Apu *apu, SaveLoadFunc *func, void *ctx) {
  apu_timers_sync(apu);                 /* a save writes the timers as of now */
  func(ctx, apu->ram, offsetof(Apu, pad) + 6 - offsetof(Apu, ram));
  apu_timer_sync = apu->cycles;         /* a load: the loaded timers are as of its cycles */
  dsp_saveload(apu->dsp, func, ctx);
  spc_saveload(apu->spc, func, ctx);
}

bool g_debug_apu_cycles;

#ifndef SNES_ABLATE_APU
#define SNES_ABLATE_APU 0
#endif
#ifndef SNES_ABLATE_DSP
#define SNES_ABLATE_DSP 0
#endif
#ifndef SNES_ABLATE_SPC
#define SNES_ABLATE_SPC 0
#endif
#if SNES_ABLATE_DSP || SNES_ABLATE_SPC
/* SNES_ABLATE_APU=1 cannot be measured: without the SPC700 the game never gets
 * past the boot handshake and never runs. These two split the chain into the
 * halves that CAN be measured with a game on screen.
 *
 * SNES_ABLATE_DSP deletes only the sample generation. The SPC700 still runs, so
 * it still answers the ports the game writes to, and the game keeps playing --
 * silently. That is the DSP's own price.
 *
 * SNES_ABLATE_SPC deletes only the opcode execution and leaves the DSP and the
 * ports. Most games hang on this; it is here so the pair brackets the chain.
 * WRONG OUTPUT, both. Diagnostic. */
#endif
void apu_cycle(Apu* apu) {
#if SNES_ABLATE_APU
  /* ABLATION, WRONG OUTPUT ON PURPOSE. Deletes the SPC700 and the DSP so the
   * device can price the whole APU chain the same way the background draw was
   * priced. Audio is silence and the emulated machine loses its sound CPU; the
   * frame counter is the only valid reading. Diagnostic, never shippable. */
  (void)apu;
  return;
#endif
  if(apu->cpuCyclesLeft == 0) {
    if (g_debug_apu_cycles) {
      char line[80];
      getProcessorStateSpc(apu, line);
      puts(line);
    }
#if SNES_ABLATE_SPC
    apu->cpuCyclesLeft = 2;
#else
    apu->cpuCyclesLeft = spc_runOpcode(apu->spc);
#endif
  }
  apu->cpuCyclesLeft--;

  if((apu->cycles & 0x1f) == 0) {
    // every 32 cycles
#if !SNES_ABLATE_DSP
    dsp_cycle(apu->dsp);
#endif
  }

  /* timers: lazy, apu_timers_sync() at their register accesses */

  apu->cycles++;
}

/* SNES_SPC_IDLE_SKIP=0 compiles the idle-wait skip out, for A/B measurement.
 * It is on by default because it changes no state at all -- see below. */
#ifndef SNES_SPC_IDLE_SKIP
#define SNES_SPC_IDLE_SKIP 1
#endif

#if SNES_SPC_IDLE_SKIP
/* The N-SPC sound driver's main wait, verbatim in both ALTTP ($0873) and SMW
 * ($0549) and 40% of every SPC opcode they execute:
 *
 *     EC FD 00   MOV A,$00FD     ; timer 0's counter -- the read CLEARS it
 *     F0 FB      BEQ -5          ; loop while it is still zero
 *
 * Every iteration before the timer ticks reads zero and clears an already-zero
 * counter: no observable state changes, only cycles pass. So charge those cycles
 * in one step instead of dispatching two opcodes per eight of them. apu_run's
 * existing closed-form bulk update advances the timers and the 32-cycle DSP tick
 * across the skipped span exactly as it would have anyway, which is why the
 * audio comes out bit-identical rather than merely close.
 *
 * Returns whole 8-cycle iterations only, and stops one short of the tick, so the
 * iteration that actually reads a non-zero counter is still interpreted, at the
 * cycle it would have run at. Everything unusual -- timer disabled, a target of
 * 0, a divider already past its target, a counter that is already non-zero --
 * returns 0 and runs the interpreter.
 */
static int apu_idleSkipCycles(Apu* apu, int budget) {
  uint16_t pc = apu->spc->pc;
  const uint8_t* r = apu->ram;

  if (r[pc] != 0xec || r[(uint16_t)(pc + 1)] != 0xfd || r[(uint16_t)(pc + 2)] != 0x00 ||
      r[(uint16_t)(pc + 3)] != 0xf0 || r[(uint16_t)(pc + 4)] != 0xfb)
    return 0;
  if (apu->romReadable && pc >= 0xffbc)
    return 0;                     /* those bytes would come from the boot ROM */

  apu_timers_sync(apu);
  Timer* t = &apu->timer[0];
  if (!t->enabled || t->counter != 0 || t->target == 0 || t->divider >= t->target)
    return 0;

  int ticks  = t->target - t->divider;              /* ticks until counter++ */
  int toInc  = (int)t->cycles + (ticks - 1) * 128;  /* cycles until that tick */
  int cycles = ((toInc - 1) / 8) * 8;               /* iterations strictly before it */
  if (cycles > budget) cycles = (budget / 8) * 8;   /* never past the caller's budget */
  return cycles >= 8 ? cycles : 0;
}
#endif

#if defined(HOST_BUILD) && !defined(QEMU_BUILD)   /* not in the 4 MB QEMU harness */
#define APU_HIST 1
#endif
#ifdef APU_HIST
/* APU_HIST=file (host; window APU_HIST_FRAMES=a:b, emulated frames since power-on):
 * where the APU's time goes. SPC700 opcodes interpreted per pc (after the idle
 * skip), the cycles the idle skip took, and a DSP census per 32 kHz tick: voices
 * playing, playing at zero volume, the echo unit's state. Written at exit; the
 * pc rows are "pc count opcode cum% bytes" with the 6 ARAM bytes at pc then
 * (tools/spc_hist_report.py groups them into loops). */
uint32_t apu_hist_frame;   /* counted by main_snes.c at each frame end */
static int apu_h_on = -1;
static uint32_t apu_h_f0, apu_h_f1 = 0xffffffffu, apu_h_fn0 = 0xffffffffu, apu_h_fn1;
static uint32_t apu_h_pc[0x10000];
static unsigned long long apu_h_ops, apu_h_opcyc, apu_h_skips, apu_h_skipcyc;
static unsigned long long apu_h_ticks, apu_h_nvoice[9], apu_h_v_idle, apu_h_v_silentenv,
    apu_h_v_zerovol, apu_h_v_play, apu_h_echo_mix, apu_h_echo_wr, apu_h_v_echo, apu_h_mute;
static const Apu *apu_h_apu;
static void apu_h_write(void) {
  FILE *f = fopen(getenv("APU_HIST"), "w");
  if (!f) return;
  unsigned long long vt = apu_h_ticks * 8;
  fprintf(f, "# frames %u..%u (%u)\n", apu_h_fn0, apu_h_fn1, apu_h_fn1 - apu_h_fn0 + 1);
  fprintf(f, "# spc opcodes %llu cycles %llu; idle skip %llu times, %llu cycles\n",
          apu_h_ops, apu_h_opcyc, apu_h_skips, apu_h_skipcyc);
  fprintf(f, "# dsp ticks %llu; voice-ticks: released+gain0 %llu, gain0 not released %llu,"
          " playing at volume 0 %llu, playing %llu (of which echo-enabled %llu)\n",
          apu_h_ticks, apu_h_v_idle, apu_h_v_silentenv, apu_h_v_zerovol, apu_h_v_play, apu_h_v_echo);
  fprintf(f, "# dsp ticks: echo FIR+mix run %llu, echo writes %llu, mute %llu\n",
          apu_h_echo_mix, apu_h_echo_wr, apu_h_mute);
  fprintf(f, "# voices not idle per tick:");
  for (int i = 0; i <= 8; i++) fprintf(f, " %d=%.1f%%", i, apu_h_ticks ? 100.0 * apu_h_nvoice[i] / apu_h_ticks : 0.0);
  fprintf(f, "\n# voice-ticks total %llu\n", vt);
  static uint32_t order[0x10000];
  uint32_t n = 0;
  for (uint32_t pc = 0; pc < 0x10000; pc++) if (apu_h_pc[pc]) order[n++] = pc;
  for (uint32_t i = 1; i < n; i++) {   /* insertion sort, hottest first (n is small) */
    uint32_t x = order[i], j = i;
    while (j && apu_h_pc[order[j - 1]] < apu_h_pc[x]) { order[j] = order[j - 1]; j--; }
    order[j] = x;
  }
  unsigned long long cum = 0;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t pc = order[i];
    cum += apu_h_pc[pc];
    fprintf(f, "%04x %u %02x %.2f", pc, apu_h_pc[pc], apu_h_apu->ram[pc], 100.0 * cum / apu_h_ops);
    for (int k = 0; k < 6; k++) fprintf(f, "%c%02x", k ? ' ' : '\t', apu_h_apu->ram[(uint16_t)(pc + k)]);
    fputc('\n', f);
  }
  fclose(f);
  char path[1024];   /* the whole ARAM at exit, for the report's disassembly */
  snprintf(path, sizeof path, "%s.aram", getenv("APU_HIST"));
  if ((f = fopen(path, "wb"))) { fwrite(apu_h_apu->ram, 1, 0x10000, f); fclose(f); }
}
static int apu_h_live(const Apu *apu) {
  if (apu_h_on < 0) {
    apu_h_on = getenv("APU_HIST") != NULL;
    const char *e = getenv("APU_HIST_FRAMES");
    if (e) sscanf(e, "%u:%u", &apu_h_f0, &apu_h_f1);
    if (apu_h_on) atexit(apu_h_write);
  }
  if (!apu_h_on || apu_hist_frame < apu_h_f0 || apu_hist_frame >= apu_h_f1) return 0;
  apu_h_apu = apu;
  if (apu_h_fn0 == 0xffffffffu) apu_h_fn0 = apu_hist_frame;
  apu_h_fn1 = apu_hist_frame;
  return 1;
}
static void apu_h_dsp(const Dsp *d) {
  int nv = 0;
  apu_h_ticks++;
  for (int i = 0; i < 8; i++) {
    const DspChannel *c = &d->channel[i];
    if (c->gain == 0 && c->adsrState == 4) { apu_h_v_idle++; continue; }
    nv++;
    if (c->gain == 0) apu_h_v_silentenv++;
    else if (c->volumeL == 0 && c->volumeR == 0 && !(c->echoEnable && (d->echoWrites || d->echoVolumeL || d->echoVolumeR)))
      apu_h_v_zerovol++;
    else { apu_h_v_play++; if (c->echoEnable) apu_h_v_echo++; }
  }
  apu_h_nvoice[nv]++;
  if (d->echoWrites || d->echoVolumeL || d->echoVolumeR) apu_h_echo_mix++;
  if (d->echoWrites) apu_h_echo_wr++;
  if (d->mute) apu_h_mute++;
}
#endif

#if defined(SPC_THUMB2_SPC) && !SNES_ABLATE_SPC && !SNES_ABLATE_DSP
#define APU_BATCH 1
/* The DSP ticks of an opcode spanning [start, end): one for every multiple of
 * 32 in it -- apu_run's own loop below, called by the Thumb-2 engine's batch
 * (spc_thumb2_run) when an opcode crosses one. */
void apu_dsp_ticks(Apu* apu, uint32_t start, uint32_t end) {
#ifdef SNES_PERF
  SNES_PERF_BEGIN(perf_d);
#endif
  for (uint32_t m = (start + 31u) & ~31u; m < end; m += 32u)
    dsp_cycle(apu->dsp);
#ifdef SNES_PERF
  SNES_PERF_END(snes_perf_dsp, perf_d);
#endif
}
#endif

/* Advance the APU by `cyclesToRun` SPC cycles. Identical machine to calling
 * apu_cycle() that many times, but the per-cycle DSP tick and the three timers are
 * folded into closed-form bulk updates between opcode boundaries — the SPC700's idle
 * cycles charged in one step, exactly as the main CPU's dot loop was collapsed
 * (snes_run_line). apu_cycle() called this ~17,000x/frame doing a 3-timer loop and a
 * DSP branch every single cycle; an opcode already told us its whole cost. Cycle-exact:
 * the framebuffer/WRAM/SRAM state hash is bit-identical to the per-cycle loop. */
void apu_run(Apu* apu, int cyclesToRun) {
#ifdef SNES_PERF
  SNES_PERF_BEGIN(perf_t);
  snes_perf_apu_calls++;
#endif
  while (cyclesToRun > 0) {
    int step;
    bool idle = false;

#ifdef APU_BATCH
    /* Whole opcodes and their DSP ticks inside the Thumb-2 engine, which
     * keeps its registers across them. It hands back here only for what this
     * loop does differently: the idle-wait pattern (the skip below), an
     * opcode longer than the budget left (in cpuCyclesLeft), a stopped SPC. */
    if (apu->cpuCyclesLeft == 0 && !apu->spc->stopped) {
#ifdef SNES_PERF_SPC
      uint32_t perf_s = snes_perf_now(), dsp0 = snes_perf_dsp;
      cyclesToRun = spc_thumb2_run(apu->spc, cyclesToRun);
      snes_perf_spc_step += snes_perf_now() - perf_s - (snes_perf_dsp - dsp0);
      snes_perf_spc_ops++;
#else
      cyclesToRun = spc_thumb2_run(apu->spc, cyclesToRun);
#endif
      if (cyclesToRun <= 0) break;
    }
#endif

    if (apu->cpuCyclesLeft == 0) {
#if SNES_SPC_IDLE_SKIP
      int skip = apu_idleSkipCycles(apu, cyclesToRun);
      if (skip > 0) {
        step = skip;             /* the SPC does nothing observable for this long */
        idle = true;
#ifdef SNES_PERF_SPC
        snes_perf_spc_skips++;
#endif
      } else
#endif
#if SNES_ABLATE_SPC
        apu->cpuCyclesLeft = 2;
#else
      {
#ifdef APU_HIST
        uint16_t h_pc = apu->spc->pc;
#endif
#ifdef SNES_PERF_SPC
        uint32_t perf_s = snes_perf_now();
        apu->cpuCyclesLeft = spc_runOpcode(apu->spc);
        snes_perf_spc_step += snes_perf_now() - perf_s;
        snes_perf_spc_ops++;
#else
        apu->cpuCyclesLeft = spc_runOpcode(apu->spc);
#endif
#ifdef APU_HIST
        if (apu_h_live(apu)) { apu_h_pc[h_pc]++; apu_h_ops++; apu_h_opcyc += apu->cpuCyclesLeft; }
#endif
      }
#endif
    }
#ifdef APU_HIST
    if (idle && apu_h_live(apu)) { apu_h_skips++; apu_h_skipcyc += step; }
#endif

    if (!idle) {
      step = apu->cpuCyclesLeft < cyclesToRun ? apu->cpuCyclesLeft : cyclesToRun;
      if (step <= 0) step = 1; /* an opcode charging 0: step one and wrap like the ref */
    }

    /* DSP fires when (cycles & 0x1f)==0, tested before the increment — so once for
     * every multiple of 32 in [cycles, cycles+step). */
    uint32_t start = apu->cycles, end = start + (uint32_t)step;
#if !SNES_ABLATE_DSP
    {
      uint32_t m = (start + 31u) & ~31u;
#ifdef SNES_PERF
      if (m < end) {
        SNES_PERF_BEGIN(perf_d);
        for (; m < end; m += 32u)
          dsp_cycle(apu->dsp);
        SNES_PERF_END(snes_perf_dsp, perf_d);
      }
#else
      for (; m < end; m += 32u) {
#ifdef APU_HIST
        if (apu_h_live(apu)) apu_h_dsp(apu->dsp);
#endif
        dsp_cycle(apu->dsp);
      }
#endif
    }
#endif

    /* timers: lazy, apu_timers_sync() at their register accesses */

    apu->cycles = end;
    if (!idle) apu->cpuCyclesLeft -= (uint8_t)step;  /* an idle skip runs no opcode */
    cyclesToRun -= step;
  }
#ifdef SNES_PERF
  SNES_PERF_END(snes_perf_apu, perf_t);
#endif
}

uint8_t apu_cpuRead(Apu* apu, uint16_t adr) {
  switch(adr) {
    case 0xf0:
    case 0xf1:
    case 0xfa:
    case 0xfb:
    case 0xfc: {
      return 0;
    }
    case 0xf2: {
      return apu->dspAdr;
    }
    case 0xf3: {
      return dsp_read(apu->dsp, apu->dspAdr & 0x7f);
    }
    case 0xf4:
    case 0xf5:
    case 0xf6:
    case 0xf7:
    case 0xf8:
    case 0xf9: {
      return apu->inPorts[adr - 0xf4];
    }
    case 0xfd:
    case 0xfe:
    case 0xff: {
      apu_timers_sync(apu);
      uint8_t ret = apu->timer[adr - 0xfd].counter;
      apu->timer[adr - 0xfd].counter = 0;
      return ret;
    }
  }
  if(apu->romReadable && adr >= 0xffc0) {
    return bootRom[adr - 0xffc0];
  }
  return apu->ram[adr];
}

void apu_cpuWrite(Apu* apu, uint16_t adr, uint8_t val) {
  switch(adr) {
    case 0xf0: {
      break; // test register
    }
    case 0xf1: {
      apu_timers_sync(apu);
      for(int i = 0; i < 3; i++) {
        if(!apu->timer[i].enabled && (val & (1 << i))) {
          apu->timer[i].divider = 0;
          apu->timer[i].counter = 0;
        }
        apu->timer[i].enabled = val & (1 << i);
      }
      if(val & 0x10) {
        apu->inPorts[0] = 0;
        apu->inPorts[1] = 0;
      }
      if(val & 0x20) {
        apu->inPorts[2] = 0;
        apu->inPorts[3] = 0;
      }
      apu->romReadable = val & 0x80;
      break;
    }
    case 0xf2: {
      apu->dspAdr = val;
      break;
    }
    case 0xf3: {
      int i = apu->hist.count;
      if (i != 256) {
        apu->hist.count = i + 1;
        apu->hist.addr[i] = (uint8_t)apu->dspAdr;
        apu->hist.val[i] = val;
      }
      if(apu->dspAdr < 0x80) dsp_write(apu->dsp, apu->dspAdr, val);
      break;
    }
    case 0xf4:
    case 0xf5:
    case 0xf6:
    case 0xf7: {
      apu->outPorts[adr - 0xf4] = val;
      break;
    }
    case 0xf8:
    case 0xf9: {
      apu->inPorts[adr - 0xf4] = val;
      break;
    }
    case 0xfa:
    case 0xfb:
    case 0xfc: {
      apu_timers_sync(apu);
      apu->timer[adr - 0xfa].target = val;
      break;
    }
  }
  apu->ram[adr] = val;
}
