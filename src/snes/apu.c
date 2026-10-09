
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
}

void apu_saveload(Apu *apu, SaveLoadFunc *func, void *ctx) {
  func(ctx, apu->ram, offsetof(Apu, pad) + 6 - offsetof(Apu, ram));
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

  // handle timers
  for(int i = 0; i < 3; i++) {
    if(apu->timer[i].cycles == 0) {
      apu->timer[i].cycles = i == 2 ? 16 : 128;
      if(apu->timer[i].enabled) {
        apu->timer[i].divider++;
        if(apu->timer[i].divider == apu->timer[i].target) {
          apu->timer[i].divider = 0;
          apu->timer[i].counter++;
          apu->timer[i].counter &= 0xf;
        }
      }
    }
    apu->timer[i].cycles--;
  }

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

    if (apu->cpuCyclesLeft == 0) {
#if SNES_SPC_IDLE_SKIP
      int skip = apu_idleSkipCycles(apu, cyclesToRun);
      if (skip > 0) {
        step = skip;             /* the SPC does nothing observable for this long */
        idle = true;
      } else
#endif
#if SNES_ABLATE_SPC
        apu->cpuCyclesLeft = 2;
#else
        apu->cpuCyclesLeft = spc_runOpcode(apu->spc);
#endif
    }

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
      for (; m < end; m += 32u)
        dsp_cycle(apu->dsp);
#endif
    }
#endif

    /* Each timer counts down; when it passes 0 it reloads to R and, if enabled,
     * advances divider->counter. Over `step` cycles the zero-crossings land at
     * k = C, C+R, C+2R, ... for k in [0,step). step <= cpuCyclesLeft (<=255), so the
     * fallback loops are tiny. */
    for (int i = 0; i < 3; i++) {
      Timer* t = &apu->timer[i];
      int R = (i == 2) ? 16 : 128;
      int C = t->cycles;
      int ticks = (C < step) ? ((step - 1 - C) / R + 1) : 0;
      if (ticks) {
        if (t->enabled) {
          if (t->target && t->divider < t->target) {
            int total = t->divider + ticks;
            t->counter = (uint8_t)((t->counter + total / t->target) & 0xf);
            t->divider = (uint8_t)(total % t->target);
          } else {
            /* target 0 (==test only on the 256-wrap) or divider>=target: step it */
            for (int k = 0; k < ticks; k++) {
              t->divider++;
              if (t->divider == t->target) { t->divider = 0; t->counter = (t->counter + 1) & 0xf; }
            }
          }
        }
        int lastK = C + (ticks - 1) * R;
        t->cycles = (uint8_t)(R + lastK - step);   /* value after the last reload */
      } else {
        t->cycles = (uint8_t)(C - step);
      }
    }

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
      apu->timer[adr - 0xfa].target = val;
      break;
    }
  }
  apu->ram[adr] = val;
}
