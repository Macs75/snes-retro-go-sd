/* Generic SNES core (LakeSnes interpreter, src/snes) as a launcher
 * system — EXPERIMENTAL baseline.
 *
 * Milestone 1: interpreter + the PPU line-renderer optimizations, nothing else.
 * The measured M7-rig figure for this exact configuration is 49.2 fps (Zelda,
 * gameplay); the first real-hardware number from this build calibrates the
 * rig's insn-count against device cycles for every further lever (spin-skip,
 * translator/XIP, audio HLE), which land separately.
 *
 * The frame loop below (dots_to_next_event / run_dots / run_frame_events) is
 * the parity-proven event loop from tools/snes_harness/snes_main.c — it and
 * the per-dot reference walk produce bit-identical state hashes there. Do not
 * "improve" it here without re-running that oracle.
 *
 * LoROM/HiROM only. Enhancement-chip carts (SA-1, SuperFX, DSP-1, ...) are
 * rejected at load with a message instead of a mid-game Hardfault.
 */
/*
 * External Retro-Go SD core — adapted from jshsakura
 * Core/Src/porting/snes/main_snes.c for the standalone ABI bridge.
 */
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "odroid_system.h"
#include "gw_lcd.h"
#include "rom_manager.h"
#include "common.h"
#include "gw_malloc.h"
#include "rg_storage.h"
#include "odroid_overlay.h"
#include "crc32.h"

#ifndef HOST_BUILD
#include "gw_core_bridge.h"
#include "gw_core_i18n.h"
#else
#include "host_compat.h"
#include "gw_core_i18n.h"
#endif

/* Firmware ABI (objcopy → core_get_ofw_is_mario). Host stub in host_emu.c. */
bool get_ofw_is_mario(void);

void draw_error_screen(const char *main_line, const char *line_1, const char *line_2);

/* Block until any pad edge after an error screen (draw_error_screen itself
 * does not wait — pairing it with switch_app(0) otherwise soft-resets
 * instantly and looks like a mysterious reboot). */
static void snes_wait_button(void)
{
  odroid_gamepad_state_t pad;
  odroid_input_read_gamepad(&pad);
  uint32_t prev = 0;
  for (int i = 0; i < ODROID_INPUT_MAX; i++)
    if (pad.values[i])
      prev |= (1u << i);
  for (;;) {
    wdog_refresh();
    odroid_input_read_gamepad(&pad);
    uint32_t now = 0;
    for (int i = 0; i < ODROID_INPUT_MAX; i++)
      if (pad.values[i])
        now |= (1u << i);
    if (now && now != prev)
      return;
    prev = now;
  }
}

#include "snes/snes.h"
#include "snes/cart.h"
#include "snes/spc7110.h"
#include "snes/ppu.h"
#include "snes/apu.h"
#include "snes/dsp.h"
#include "snes/cpu.h"
#include "snes/dma.h"
#include "snes/input.h"
#include "snes/saveload.h"
#include "snes/spin_skip.h"
#ifdef SNES_SPIN_BAKE
#include "snes/spin_bake.h"
#endif
#include "snes_audio_stretch.h"
#include "snes_dsp_variant.h"
#include "snes/rc_dispatch.h"
#include "snes_profile.h"
#include "snes_perf.h"
#include "snes_scale.h"

#define APPID_SNES 25

bool snes_loadRom(Snes *snes, const uint8_t *data, int length);   /* snes_other.c */

#define SNES_FPS_NTSC       60
#define SNES_FPS_PAL        50
#define SNES_WIDTH          256
#define SNES_HEIGHT         224
#define SNES_AUDIO_RATE     16000
#define SNES_AUDIO_SAMPLES_NTSC  (SNES_AUDIO_RATE / SNES_FPS_NTSC) /* 266 */
#define SNES_AUDIO_SAMPLES_PAL   (SNES_AUDIO_RATE / SNES_FPS_PAL)  /* 320 */
#define SNES_AUDIO_SAMPLES_MAX   SNES_AUDIO_SAMPLES_PAL

/* Savestate stamp: a raw struct dump must refuse files this build didn't
 * write (project rule — a stale state "loads" and restores nonsense). */
#define SNES_STATE_MAGIC    0x31534E53u   /* "SNS1" */
#define SNES_STATE_VERSION  3   /* v3: PAL DSP sampleBuffer 640 */

/* ---- hooks the snes lib links against ------------------------------------
 * Super Metroid RTL hooks; a generic core has no reimplementation so they
 * are inert (same shims as the host harness / M7 rig). */
int  CpuOpcodeHook(uint32_t addr) { (void)addr; return 0; }
bool HookedFunctionRts(int level) { (void)level; return false; }
bool g_fail;
bool g_new_ppu = true;                 /* the fast line renderer, as measured */

static Snes *g_the_snes;

#ifdef SNES_SMW_HLE_PRODUCT
extern int g_wire_on;
void wire_apu_write(Snes *snes, uint32_t adr, uint8_t val);
int wire_try_swap(Snes *snes, int frame);
void wire_frame_audio(int16_t *buf, int n);
bool wire_configure_rom(const uint8_t *rom, uint32_t len);
void wire_prepare_save(void);
void wire_restore_after_load(Snes *snes);
static int smw_hle_frame;
#endif

/* The lib routes $2140-43 writes through this (snes.c). Catch the APU up and
 * write the CPU-visible mailbox — NOT apu_cpuWrite(), which is the SPC's own
 * bus and would leave inPorts stale (boot then spins on the port echo). */
void RtlApuWrite(uint32_t adr, uint8_t val) {
#ifdef SNES_SMW_HLE_PRODUCT
  wire_apu_write(g_the_snes, adr, val);
#else
  snes_catchupApu(g_the_snes);
  g_the_snes->apu->inPorts[adr & 0x3] = val;
#endif
}

void Die(const char *s) {
  printf("SNES Die: %s\n", s);
  assert(!"snes core died");
}
void Warning(const char *s) { (void)s; }

extern bool g_ppu_skip_render;   /* ppu.c: skip compositing on dropped frames */

/* ---- state ---------------------------------------------------------------- */
static Snes *snes;
/* input record/replay (further below) */
static void rec_close(void);
static bool rec_sram_blocked;
static uint32_t rec_audio_h = 2166136261u;   /* FNV-1a of every frame's samples since the record/replay start */
#ifdef HOST_BUILD
/* host only: a running hash of the rendered picture, every frame. The state
 * hashes cannot see a renderer change (palette tables, compositing); this can.
 * Too slow for the device, where it would distort the PERF numbers. */
static uint32_t rec_pic_h = 2166136261u;
#endif
static int rec_mode;                        /* REC_OFF / REC_RECORD / REC_REPLAY (further below) */
/* 128 KB WRAM lives in the overlay BSS (like the SM port's g_ram): the AHB
 * pool is only 120 KB total and the Apu (~66 KB) already comes from it. */
static uint8_t snes_wram[0x20000];
static int16_t audio_buf[SNES_AUDIO_SAMPLES_MAX];
static int snes_fps = SNES_FPS_NTSC;
static int snes_audio_samples = SNES_AUDIO_SAMPLES_NTSC;

/* ---- event loop (verbatim from tools/snes_harness/snes_main.c) ------------ */
#ifdef SNES_BUS_IN_ITCM
#define SNES_ITCM_SCHED __attribute__((section(".itcm_snes_interp.thumb2.bus")))
#else
#define SNES_ITCM_SCHED
#endif

/* Keep the scheduler in ITCM with cpu_runOpcode. At -O3 gcc already inlines
 * these into app_main_snes (also ITCM); the section is the safety net if a
 * call is not inlined. -ffunction-sections drops a section attribute that is
 * only on the definition, so it is on the prototype too. */
static int dots_to_next_event(Snes *s) SNES_ITCM_SCHED;
static void apply_irq_match(Snes *s) SNES_ITCM_SCHED;
static int run_one_opcode(Snes *s) SNES_ITCM_SCHED;
static void cpu_tick(Snes *s) SNES_ITCM_SCHED;
static void run_dots(Snes *s, int dots) SNES_ITCM_SCHED;
static void run_frame_events(Snes *s) SNES_ITCM_SCHED;

#ifdef SNES_PERF
static bool snes_half_lock_active(void);
uint32_t snes_perf_ppu, snes_perf_ppu_lines;
uint32_t snes_perf_apu, snes_perf_apu_calls;
uint32_t snes_perf_dsp;
uint32_t snes_perf_dma, snes_perf_dma_calls;
uint32_t snes_perf_ppu_part[SNES_PERF_PPU_N];
uint32_t snes_perf_ppu_drawn, snes_perf_ppu_sub_lines, snes_perf_ppu_spr_lines;
uint32_t snes_perf_ppu_math_lines, snes_perf_ppu_mode[8];
uint32_t snes_perf_ppu_cgram_lines, snes_perf_ppu_math_rebuilds;
#ifdef SNES_PERF_CPU
uint32_t snes_perf_cpu_eng, snes_perf_cpu_ops, snes_perf_cpu_memops;
uint32_t snes_perf_cpu_bake, snes_perf_cpu_events, snes_perf_cpu_dmaloop;
uint32_t snes_perf_cpu_rd_slow, snes_perf_cpu_rd_page, snes_perf_cpu_wr_slow;
extern uint32_t g_op_total, g_op_fallback, g_op_fbhist[256];
#endif
/* main-loop phases, cycles per window */
enum { SP_FRAMECTL, SP_INPUT, SP_EMU, SP_KICK, SP_PCM, SP_WAIT, SP_OVERLAY, SP_SWAP, SP_N };
static uint32_t sp_phase[SP_N];
static uint32_t sp_apu_emu, sp_apu_pcm;            /* APU cycles inside emu / inside pcm submit */
static uint32_t sp_frames, sp_shown, sp_late, sp_pace_ms, sp_t0_ms;
static uint32_t sp_mhz;
static uint32_t sp_frame_max;                      /* the heaviest emulated frame (busy cycles) */
static uint32_t sp_last, sp_frame_busy;
/* the audio stretcher: playback step (65536 = normal speed; below it the
 * sound is slowed to follow a slow emulation), sampled once a frame */
static uint32_t sp_step_min = 0xffffffffu;
static uint64_t sp_step_sum;
static uint32_t sp_under0, sp_ins0;
extern uint32_t g_stretch_ins;
#define SP_MARK(i) do { uint32_t now_ = snes_perf_now(); sp_phase[(i)] += now_ - sp_last; \
                        sp_frame_busy += now_ - sp_last; sp_last = now_; } while (0)

/* DWT on (it is on whenever the firmware profiles, but do not assume),
 * then the clock: DWT cycles over 100 ms of HAL_GetTick, busy-waited. */
static void snes_perf_init(void)
{
  *(volatile uint32_t *)0xE000EDFCu |= (1u << 24);   /* DEMCR.TRCENA */
  *(volatile uint32_t *)0xE0001000u |= 1u;           /* DWT_CTRL.CYCCNTENA */
  uint32_t t = HAL_GetTick();
  while (HAL_GetTick() == t) { }
  uint32_t c0 = snes_perf_now(), t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 100u) wdog_refresh();
  uint32_t c1 = snes_perf_now();
  sp_mhz = (c1 - c0 + 50000u) / 100000u;
  sp_t0_ms = HAL_GetTick();
  printf("PROF/clock: %lu MHz\n", (unsigned long)sp_mhz);
}

#ifdef SNES_PERF_AUDIO
extern uint32_t g_sa_used, g_sa_drop, g_sa_pulls, g_sa_pulled, g_sa_pull_max;
extern uint32_t g_sa_floor, g_sa_ceil, g_sa_base_min, g_sa_base_max;
extern uint64_t g_sa_next_sum, g_sa_base_sum;
/* per window: samples pushed by the core, taken by the ISR (ring tail),
 * audio DMA periods, frames whose pump found the ring full (no pull) */
static uint32_t sp_au_push, sp_au_isr, sp_au_dma0, sp_au_nopull;
static uint16_t sp_au_tail;
static volatile uint16_t snes_pcm_tail;   /* defined with the ring below */
#endif
static void snes_perf_reset(void);
static void snes_perf_report(void)
{
  const char *scal[] = { "off", "fit", "full", "custom" };
  unsigned sm = (unsigned)odroid_display_get_scaling_mode();
  uint32_t n = sp_frames ? sp_frames : 1;
  uint32_t wall = HAL_GetTick() - sp_t0_ms;
  /* the frame budget at the PCB rate: 60.0988 Hz NTSC, 50.0070 Hz PAL */
  uint32_t budget = (uint32_t)((uint64_t)sp_mhz * 1000000u * 10000u / (snes_fps == SNES_FPS_PAL ? 500070u : 600988u));
  uint32_t ppu = snes_perf_ppu / n, apu_e = sp_apu_emu / n, dma = snes_perf_dma / n;
  uint32_t emu = sp_phase[SP_EMU] / n;
  uint32_t cpu = emu > ppu + apu_e + dma ? emu - ppu - apu_e - dma : 0;
  uint32_t dsp = snes_perf_dsp / n, apu_all = snes_perf_apu / n;
  uint32_t spc = apu_all > dsp ? apu_all - dsp : 0;
  uint32_t busy = 0;
  for (int i = 0; i < SP_N; i++) busy += sp_phase[i] / n;
  uint32_t shown = sp_shown ? sp_shown : 1;
  printf("PROF/frame: emulated=%lu shown=%lu wall_ms=%lu (half-rate %s, scaling %s, %s)\n",
         (unsigned long)sp_frames, (unsigned long)sp_shown, (unsigned long)wall,
         snes_half_lock_active() ? "on" : "off", sm < 4 ? scal[sm] : "?", snes_fps == SNES_FPS_PAL ? "PAL" : "NTSC");
  printf("PROF/budget: budget=%lu busy=%lu heaviest=%lu (cycles per emulated frame, %lu MHz)\n",
         (unsigned long)budget, (unsigned long)busy, (unsigned long)sp_frame_max, (unsigned long)sp_mhz);
  printf("PROF/emu: total=%lu [cpu=%lu ppu=%lu apu=%lu dma=%lu] per emulated frame, ppu lines=%lu\n",
         (unsigned long)emu, (unsigned long)cpu, (unsigned long)ppu, (unsigned long)apu_e, (unsigned long)dma,
         (unsigned long)(snes_perf_ppu_lines / n));
  printf("PROF/apu: total=%lu [spc700=%lu dsp=%lu] in_emu=%lu in_pcm=%lu calls=%lu per emulated frame\n",
         (unsigned long)apu_all, (unsigned long)spc, (unsigned long)dsp, (unsigned long)apu_e,
         (unsigned long)(sp_apu_pcm / n), (unsigned long)(snes_perf_apu_calls / n));
  printf("PROF/present: present=%lu wait=%lu overlay=%lu swap=%lu per SHOWN frame\n",
         (unsigned long)(sp_phase[SP_KICK] / shown), (unsigned long)(sp_phase[SP_WAIT] / shown),
         (unsigned long)(sp_phase[SP_OVERLAY] / shown), (unsigned long)(sp_phase[SP_SWAP] / shown));
  printf("PROF/loop: pcm=%lu input=%lu framectl=%lu per emulated frame\n",
         (unsigned long)(sp_phase[SP_PCM] / n), (unsigned long)(sp_phase[SP_INPUT] / n),
         (unsigned long)(sp_phase[SP_FRAMECTL] / n));
  printf("PROF/pace: late=%lu of %lu frames (already behind the audio clock), wait_ms=%lu\n",
         (unsigned long)sp_late, (unsigned long)sp_frames, (unsigned long)sp_pace_ms);
  printf("PROF/audio: step_avg=%lu step_min=%lu (65536 = normal speed) underruns=%lu inserts=%lu fill=%u\n",
         (unsigned long)(sp_step_sum / n), (unsigned long)sp_step_min,
         (unsigned long)(snes_stretch_underruns() - sp_under0), (unsigned long)(g_stretch_ins - sp_ins0),
         (unsigned)snes_stretch_fill());
#ifdef SNES_PERF_AUDIO
  {
    uint32_t w = wall ? wall : 1, pl = g_sa_pulls ? g_sa_pulls : 1;
    printf("PROF/audio2: pushed=%lu used=%lu dropped=%lu pulled=%lu isr=%lu dma_periods=%lu speed_x10000=%lu\n",
           (unsigned long)sp_au_push, (unsigned long)g_sa_used, (unsigned long)g_sa_drop,
           (unsigned long)g_sa_pulled, (unsigned long)sp_au_isr,
           (unsigned long)(dma_counter - sp_au_dma0),
           (unsigned long)(g_sa_pulled ? (uint64_t)g_sa_used * 10000u / g_sa_pulled : 0));
    printf("PROF/audio2: pulls=%lu nopull_frames=%lu pull_avg=%lu pull_max=%lu floor_pulls=%lu ceil_pulls=%lu asked_avg=%lu base avg=%lu min=%lu max=%lu isr_hz=%lu\n",
           (unsigned long)g_sa_pulls, (unsigned long)sp_au_nopull, (unsigned long)(g_sa_pulled / pl),
           (unsigned long)g_sa_pull_max, (unsigned long)g_sa_floor, (unsigned long)g_sa_ceil,
           (unsigned long)(g_sa_next_sum / pl), (unsigned long)(g_sa_base_sum / pl),
           (unsigned long)g_sa_base_min, (unsigned long)g_sa_base_max,
           (unsigned long)((uint64_t)sp_au_isr * 1000u / w));
  }
#endif
#ifdef SNES_PERF_CPU
  {
    uint32_t ops = snes_perf_cpu_ops ? snes_perf_cpu_ops : 1;
    uint32_t eng = snes_perf_cpu_eng / n, bake = snes_perf_cpu_bake / n, ev = snes_perf_cpu_events / n;
    uint32_t rest = cpu > eng + bake + ev ? cpu - eng - bake - ev : 0;
    printf("PROF/cpu: engine=%lu bake=%lu events=%lu rest=%lu (of cpu=%lu) per emulated frame\n",
           (unsigned long)eng, (unsigned long)bake, (unsigned long)ev, (unsigned long)rest, (unsigned long)cpu);
    printf("PROF/cpu: opcodes=%lu memops=%lu engine_per_op=%lu dma_loop_steps=%lu per emulated frame\n",
           (unsigned long)(snes_perf_cpu_ops / n), (unsigned long)(snes_perf_cpu_memops / n),
           (unsigned long)(snes_perf_cpu_eng / ops), (unsigned long)(snes_perf_cpu_dmaloop / n));
    printf("PROF/cpu: bus slow_reads=%lu rom_page_refills=%lu slow_writes=%lu per emulated frame\n",
           (unsigned long)(snes_perf_cpu_rd_slow / n), (unsigned long)(snes_perf_cpu_rd_page / n),
           (unsigned long)(snes_perf_cpu_wr_slow / n));
    /* the five opcodes that fall back to the C interpreter most */
    char top[96]; int tl = 0; top[0] = 0;
    for (int k = 0; k < 5; k++) {
      int best = -1; uint32_t bv = 0;
      for (int o = 0; o < 256; o++) if (g_op_fbhist[o] > bv) { bv = g_op_fbhist[o]; best = o; }
      if (best < 0) break;
      tl += snprintf(top + tl, sizeof top - tl, " %02x:%lu", best, (unsigned long)(bv / n));
      g_op_fbhist[best] = 0;
    }
    printf("PROF/cpu: c_fallback=%lu of %lu opcodes per emulated frame, top%s\n",
           (unsigned long)(g_op_fallback / n), (unsigned long)(g_op_total / n), top);
  }
#endif
#ifdef SNES_PERF_PPU
  {
    uint32_t d = snes_perf_ppu_drawn ? snes_perf_ppu_drawn : 1;
    const uint32_t *pp = snes_perf_ppu_part;
    printf("PROF/ppu: eval=%lu pre=%lu main=%lu sub=%lu comp=%lu blit=%lu per emulated frame\n",
           (unsigned long)(pp[SNES_PERF_PPU_EVAL] / n), (unsigned long)(pp[SNES_PERF_PPU_PRE] / n),
           (unsigned long)(pp[SNES_PERF_PPU_MAIN] / n), (unsigned long)(pp[SNES_PERF_PPU_SUB] / n),
           (unsigned long)(pp[SNES_PERF_PPU_COMP] / n), (unsigned long)(pp[SNES_PERF_PPU_BLIT] / n));
    printf("PROF/ppu: layers bg4=%lu bg2=%lu other=%lu sprites=%lu per emulated frame\n",
           (unsigned long)(pp[SNES_PERF_PPU_BG4] / n), (unsigned long)(pp[SNES_PERF_PPU_BG2] / n),
           (unsigned long)(pp[SNES_PERF_PPU_BGX] / n), (unsigned long)(pp[SNES_PERF_PPU_SPR] / n));
    printf("PROF/ppu: per drawn line main=%lu sub=%lu comp=%lu blit=%lu; lines drawn=%lu sub=%lu sprites=%lu math=%lu\n",
           (unsigned long)(pp[SNES_PERF_PPU_MAIN] / d), (unsigned long)(pp[SNES_PERF_PPU_SUB] / d),
           (unsigned long)(pp[SNES_PERF_PPU_COMP] / d), (unsigned long)(pp[SNES_PERF_PPU_BLIT] / d),
           (unsigned long)snes_perf_ppu_drawn, (unsigned long)snes_perf_ppu_sub_lines,
           (unsigned long)snes_perf_ppu_spr_lines, (unsigned long)snes_perf_ppu_math_lines);
    printf("PROF/ppu: modes 0=%lu 1=%lu 2=%lu 3=%lu 4=%lu 5=%lu 6=%lu 7=%lu (drawn lines)\n",
           (unsigned long)snes_perf_ppu_mode[0], (unsigned long)snes_perf_ppu_mode[1],
           (unsigned long)snes_perf_ppu_mode[2], (unsigned long)snes_perf_ppu_mode[3],
           (unsigned long)snes_perf_ppu_mode[4], (unsigned long)snes_perf_ppu_mode[5],
           (unsigned long)snes_perf_ppu_mode[6], (unsigned long)snes_perf_ppu_mode[7]);
    printf("PROF/ppu: tables cgram_update_lines=%lu math_rebuilds=%lu (of %lu drawn lines)\n",
           (unsigned long)snes_perf_ppu_cgram_lines, (unsigned long)snes_perf_ppu_math_rebuilds,
           (unsigned long)snes_perf_ppu_drawn);
  }
#endif
  snes_perf_reset();
}

/* a new window from now: after each report, and at the start of an input
 * replay so its windows line up frame for frame from one run to the next */
static void snes_perf_reset(void)
{
  sp_step_min = 0xffffffffu; sp_step_sum = 0;
  sp_under0 = snes_stretch_underruns(); sp_ins0 = g_stretch_ins;
  for (int i = 0; i < SP_N; i++) sp_phase[i] = 0;
  snes_perf_ppu = snes_perf_ppu_lines = snes_perf_apu = snes_perf_apu_calls = 0;
  snes_perf_dsp = snes_perf_dma = snes_perf_dma_calls = 0;
  memset(snes_perf_ppu_part, 0, sizeof(snes_perf_ppu_part));
  memset(snes_perf_ppu_mode, 0, sizeof(snes_perf_ppu_mode));
  snes_perf_ppu_drawn = snes_perf_ppu_sub_lines = snes_perf_ppu_spr_lines = snes_perf_ppu_math_lines = 0;
  snes_perf_ppu_cgram_lines = snes_perf_ppu_math_rebuilds = 0;
#ifdef SNES_PERF_CPU
  snes_perf_cpu_eng = snes_perf_cpu_ops = snes_perf_cpu_memops = 0;
  snes_perf_cpu_bake = snes_perf_cpu_events = snes_perf_cpu_dmaloop = 0;
  snes_perf_cpu_rd_slow = snes_perf_cpu_rd_page = snes_perf_cpu_wr_slow = 0;
  g_op_total = g_op_fallback = 0;
  memset(g_op_fbhist, 0, sizeof(g_op_fbhist));
#endif
#ifdef SNES_PERF_AUDIO
  g_sa_used = g_sa_drop = g_sa_pulls = g_sa_pulled = g_sa_pull_max = g_sa_floor = g_sa_ceil = 0;
  g_sa_next_sum = g_sa_base_sum = 0; g_sa_base_min = 0xffffffffu; g_sa_base_max = 0;
  sp_au_push = sp_au_isr = sp_au_nopull = 0; sp_au_dma0 = dma_counter; sp_au_tail = snes_pcm_tail;
#endif
  sp_apu_emu = sp_apu_pcm = sp_frames = sp_shown = sp_late = sp_pace_ms = sp_frame_max = 0;
  sp_t0_ms = HAL_GetTick();
}
#endif

static int SNES_ITCM_SCHED dots_to_next_event(Snes *s) {
  int h = s->hPos;
  if (h == 0 || h == 512 || h == 1024) return 0;
  if (s->hIrqEnabled && h == s->hTimer * 4) return 0;
  int next = 1362;
  if (h < 512)       next = 512;
  else if (h < 1024) next = 1024;
  if (s->hIrqEnabled) {
    int t = s->hTimer * 4;
    if (t > h && t < next) next = t;
  }
  return next - h;
}

static void SNES_ITCM_SCHED apply_irq_match(Snes *s) {
  if (!(s->hIrqEnabled || s->vIrqEnabled)) return;
  if (s->vIrqEnabled && s->vPos != s->vTimer) return;
  if (s->hIrqEnabled) {
    if (s->hPos != s->hTimer * 4) return;
  } else {
    /* V-IRQ only is an edge at the start of vTimer (snes_handle_pos_stuff
     * at hPos==0). Re-asserting every opcode holds TIMEUP after the handler
     * reads $4211, so RTI immediately re-enters IRQ. TMZ's message box uses
     * that pair (F80E then F89A on $4209) and collapsed to one scanline. */
    return;
  }
  s->inIrq = true;
  s->cpu->irqWanted = true;
}

/* One real interpreter call, with the spin learner watching (harness-identical:
 * tools/snes_spin compiles the same spin_skip.c and gates skip-off vs skip-on to
 * bit-identical state+audio hashes). */
static int SNES_ITCM_SCHED run_one_opcode(Snes *s) {
  Cpu *cpu = s->cpu;
#if defined(SNES_SPIN_SKIP) && !SNES_SPIN_REPLAY_ONLY
  const bool learn = spin_engaged();   /* sample before the opcode; see header */
  uint32_t pc24 = 0;
  int disp = 0;
  if (learn) {
    pc24 = ((uint32_t)cpu->k << 16) | cpu->pc;
    disp = (cpu->nmiWanted || (cpu->irqWanted && !cpu->i) || cpu->waiting) && !cpu->stopped;
  }
#endif
  s->cpuMemOps = 0;
#ifdef SNES_PERF_CPU
  uint32_t pc_t0 = snes_perf_now(), pc_a0 = snes_perf_apu, pc_d0 = snes_perf_dma, pc_p0 = snes_perf_ppu;
#endif
  int cycles = SNES_PROF_CPU_CALL(CPU_RUN_OPCODE(cpu));
#ifdef SNES_PERF_CPU
  snes_perf_cpu_eng += snes_perf_now() - pc_t0 - (snes_perf_apu - pc_a0) - (snes_perf_dma - pc_d0)
                       - (snes_perf_ppu - pc_p0);
  snes_perf_cpu_ops++;
  snes_perf_cpu_memops += s->cpuMemOps;
#endif
  /* 6*cycles + 2*memOps -- identical to the old 8-per-access charge plus
   * (cycles - memOps)*6, but paid once instead of on every bus access. */
  s->cpuCyclesLeft += cycles * 6 + s->cpuMemOps * 2;
#if defined(SNES_SPIN_SKIP) && !SNES_SPIN_REPLAY_ONLY
  if (learn) SNES_PROF_SPIN_CALL(spin_note_real(cpu, pc24, (uint8_t)s->cpuCyclesLeft, disp));
#endif
  return cycles;
}

static void SNES_ITCM_SCHED cpu_tick(Snes *s) {
#ifdef SNES_PERF
  {
    SNES_PERF_DMA_BEGIN(perf_t, perf_a);
    bool busy = dma_cycle(s->dma);
    SNES_PERF_DMA_END(perf_t, perf_a);
    if (busy) return;
  }
#else
  if (dma_cycle(s->dma)) return;
#endif
  if (s->cpuCyclesLeft == 0) run_one_opcode(s);
  s->cpuCyclesLeft -= 2;
}

static void SNES_ITCM_SCHED run_dots(Snes *s, int dots) {
  Cpu *cpu = s->cpu;
#ifdef SNES_SPIN_BAKE
  /* One test per SPAN, outside the loop below. See spin_bake.h. */
  if ((uint16_t)(s->cpu->pc - g_bake.pc_load) <= 2u)
  {
#ifdef SNES_PERF_CPU
    uint32_t pb_t0 = snes_perf_now(), pb_d0 = snes_perf_dma;
#endif
    dots = spin_bake_run_span(s, s->cpu, dots);
#ifdef SNES_PERF_CPU
    snes_perf_cpu_bake += snes_perf_now() - pb_t0 - (snes_perf_dma - pb_d0);
#endif
  }
#endif
  bool dma_active = s->dma->dmaBusy || s->dma->hdmaTimer > 0;
  while (dots > 0) {
    if (dma_active) {
#ifdef SNES_PERF
      { SNES_PERF_DMA_BEGIN(perf_t, perf_a); dma_cycle(s->dma); SNES_PERF_DMA_END(perf_t, perf_a); }
#else
      dma_cycle(s->dma);
#endif
      s->apuDotsAccum += 2;
      s->hPos += 2; dots -= 2;
      dma_active = s->dma->dmaBusy || s->dma->hdmaTimer > 0;
      SNES_PERF_CPU_INC(snes_perf_cpu_dmaloop);
      continue;
    }
    bool started_dma = false;
    if (s->cpuCyclesLeft == 0) {
#ifdef SNES_SPIN_SKIP
      if (g_spin.on &&
          !cpu->nmiWanted && !cpu->irqWanted && !cpu->waiting && !cpu->stopped &&
          !s->hIrqEnabled &&
          !(s->vIrqEnabled && s->vPos == s->vTimer) &&
          (((uint32_t)cpu->k << 16) | cpu->pc) == g_spin.pc[g_spin.idx]) {
        s->cpuCyclesLeft += g_spin.charge[g_spin.idx];
        g_spin.idx = (g_spin.idx + 1) % g_spin.len;
        cpu->k  = (uint8_t)(g_spin.pc[g_spin.idx] >> 16);
        cpu->pc = (uint16_t)g_spin.pc[g_spin.idx];
        g_spin.ops_virtual++;
      } else
#endif
      {
        apply_irq_match(s);
        run_one_opcode(s);
        started_dma = s->dma->dmaBusy || s->dma->hdmaTimer > 0;
      }
    }
    int step;
    if (s->cpuCyclesLeft >= 2 && !started_dma) {
      step = s->cpuCyclesLeft;
      if (step > dots) step = dots;
      s->cpuCyclesLeft -= (uint8_t)step;
    } else {
      step = 2;
      s->cpuCyclesLeft -= 2;
    }
    s->apuDotsAccum += step;
    s->hPos += step; dots -= step;
    dma_active = started_dma;
  }
}

static void SNES_ITCM_SCHED run_frame_events(Snes *s) {
  extern void wdog_refresh(void);
  uint16_t last_v = 0xffff;
  for (;;) {
    if (s->vPos != last_v) {
      last_v = s->vPos;
      wdog_refresh();
    }
    s->apuDotsAccum += 2;
#ifdef SNES_PERF_CPU
    {
      uint32_t pe_t0 = snes_perf_now(), pe_a0 = snes_perf_apu, pe_d0 = snes_perf_dma, pe_p0 = snes_perf_ppu;
      snes_handle_pos_stuff(s);
      snes_perf_cpu_events += snes_perf_now() - pe_t0 - (snes_perf_apu - pe_a0) - (snes_perf_dma - pe_d0)
                              - (snes_perf_ppu - pe_p0);
    }
#else
    snes_handle_pos_stuff(s);
#endif
    cpu_tick(s);
    if (s->hPos == 0 && s->vPos == 0) break;
    run_dots(s, dots_to_next_event(s));
  }
  wdog_refresh();
  snes_catchupApu(s);
  spin_frame_tick();   /* auto-gate: park the learner on non-spinning carts */
#ifdef SNES_SPIN_BAKE
  spin_bake_frame_tick();
#endif
#ifdef SNES_SMW_HLE_PRODUCT
  wire_try_swap(s, smw_hle_frame++);
#endif
}

/* ---- input ----------------------------------------------------------------
 * LakeSnes currentState bits: 0=B 1=Y 2=Select 3=Start 4=Up 5=Down 6=Left
 * 7=Right 8=A 9=X 10=L 11=R.
 *
 * Firmware odroid_input maps hardware as:
 *   START=GAME, SELECT=TIME, X=phys Start (Zelda only), Y=phys Select (Zelda),
 *   VOLUME=PAUSE/SET. PAUSE combos stay with the launcher — ignore VOLUME.
 *
 * Profiles (pause menu "Controls"; Auto → Zelda or Mario via get_ofw_is_mario):
 *   Zelda  — GAME modifier: GAME+A/B = R/L, GAME+TIME = Select, TIME = Start
 *   Mario  — GAME/TIME = Start/Select; GAME+A/B = R/L; TIME+A/B = X/Y
 *   Simple — no L/R: GAME = Select, TIME = Start; phys Start/Select = X/Y
 */

typedef enum {
  SNES_PAD_AUTO = 0,
  SNES_PAD_ZELDA = 1,
  SNES_PAD_MARIO = 2,
  SNES_PAD_SIMPLE = 3,
  SNES_PAD_PROFILE_COUNT
} snes_pad_profile_t;

static int snes_pad_profile = SNES_PAD_AUTO;
static char snes_pad_map_value[12];

static snes_pad_profile_t snes_pad_profile_effective(void)
{
  if (snes_pad_profile == SNES_PAD_AUTO)
    return get_ofw_is_mario() ? SNES_PAD_MARIO : SNES_PAD_ZELDA;
  return (snes_pad_profile_t)snes_pad_profile;
}

static uint16_t read_snes_pad(odroid_gamepad_state_t *joy) {
  uint16_t s = 0;
  /* D-pad always. */
  if (joy->values[ODROID_INPUT_UP])    s |= 1u << 4;
  if (joy->values[ODROID_INPUT_DOWN])  s |= 1u << 5;
  if (joy->values[ODROID_INPUT_LEFT])  s |= 1u << 6;
  if (joy->values[ODROID_INPUT_RIGHT]) s |= 1u << 7;

  /* PAUSE/SET held: launcher owns the chord; emit no face/shoulder. */
  if (joy->values[ODROID_INPUT_VOLUME])
    return s;

  const bool game = joy->values[ODROID_INPUT_START] != 0;   /* GAME */
  const bool time = joy->values[ODROID_INPUT_SELECT] != 0;  /* TIME */
  const bool a = joy->values[ODROID_INPUT_A] != 0;
  const bool b = joy->values[ODROID_INPUT_B] != 0;
  const bool phys_start = joy->values[ODROID_INPUT_X] != 0;  /* Zelda Start */
  const bool phys_select = joy->values[ODROID_INPUT_Y] != 0; /* Zelda Select */
  const snes_pad_profile_t profile = snes_pad_profile_effective();

  if (profile == SNES_PAD_MARIO) {
    /* GAME+A/B = R/L; TIME+A/B = X/Y; else face + GAME=Start, TIME=Select. */
    if (game && (a || b)) {
      if (a) s |= 1u << 11; /* R */
      if (b) s |= 1u << 10; /* L */
    } else if (time && (a || b)) {
      if (a) s |= 1u << 9; /* X */
      if (b) s |= 1u << 1; /* Y */
    } else {
      if (a) s |= 1u << 8; /* A */
      if (b) s |= 1u << 0; /* B */
      if (game) s |= 1u << 3; /* Start */
      if (time) s |= 1u << 2; /* Select */
    }
  } else if (profile == SNES_PAD_SIMPLE) {
    /* No L/R: GAME = Select, TIME = Start. */
    if (a) s |= 1u << 8;
    if (b) s |= 1u << 0;
    if (game) s |= 1u << 2; /* Select */
    if (time) s |= 1u << 3; /* Start */
    if (phys_select) s |= 1u << 1; /* Y */
    if (phys_start) s |= 1u << 9;  /* X */
  } else {
    /* Zelda (default): GAME alone does nothing (modifier). */
    if (game && (a || b)) {
      if (a) s |= 1u << 11; /* R */
      if (b) s |= 1u << 10; /* L */
    } else if (game && time) {
      s |= 1u << 2; /* Select */
    } else {
      if (a) s |= 1u << 8; /* A */
      if (b) s |= 1u << 0; /* B */
      if (time) s |= 1u << 3; /* Start */
    }
    if (phys_select) s |= 1u << 1; /* Y */
    if (phys_start) s |= 1u << 9;  /* X */
  }
  return s;
}

/* ---- video ----------------------------------------------------------------
 * Render into a PRIVATE persistent framebuffer, then copy the complete visible
 * 320x240 image to whichever LCD buffer is active.  This keeps both LCD buffers
 * complete across swaps/overlays.  SNES_DIRECT_VIDEO lets the PPU write its RGB565
 * scanlines directly into that private framebuffer, removing the old 512-byte
 * scratch-to-frame memcpy on every visible line. */
#define SNES_TOP_MARGIN  ((GW_LCD_HEIGHT - SNES_HEIGHT) / 2)   /* (240-224)/2 = 8 */
#define SNES_LEFT_MARGIN ((GW_LCD_WIDTH - SNES_WIDTH) / 2)     /* (320-256)/2 = 32 */
#ifdef SNES_DIRECT_VIDEO
/* Overscan can produce 240 lines.  Starting at row 8 therefore needs eight
 * hidden tail rows; present_frame still copies only the visible 240 rows. */
#define SNES_FRAME_ROWS (GW_LCD_HEIGHT + SNES_TOP_MARGIN)
#else
#define SNES_FRAME_ROWS GW_LCD_HEIGHT
static uint16_t snes_line[256];
#endif
static uint16_t snes_frame[GW_LCD_WIDTH * SNES_FRAME_ROWS] __attribute__((aligned(8)));

#ifdef SNES_LOAD_DIAG
/* DWT profiling accumulators for the SNES_LOAD_DIAG block further below.
 * Defined here rather than in the generated apu.c (where they used to live)
 * because SNES_APU_SOURCE replaces that file with the audio HLE's apu_wire.c
 * whenever SNES_SMW_HLE=1 / SNES_NSPC_HLE=1 -- so the definitions dropped out
 * of the link precisely when HLE was enabled, and SNES_LOAD_DIAG=1 could not
 * be combined with it. That combination is the interesting one (HLE is what
 * ships), and it was never profiled on device for exactly this reason.
 * main_snes.c always compiles, so the counters always exist; the instrumented
 * apu.c/dsp.c reach them as externs through the forced -include of
 * snes_diag_accum.h. With HLE active nothing increments the spc counter --
 * a 0 there is the correct reading, not a missing probe: the SPC700
 * interpreter genuinely no longer runs. */
uint64_t g_diag_spc_cycles = 0;
uint64_t g_diag_dsp_cycles = 0;
uint64_t g_diag_dsp_echo_cycles = 0;
#endif

#ifndef SNES_DIRECT_VIDEO
static void snes_blit_line(unsigned y, const uint16_t *line) {
  if (y < 1) return;   /* y is 1-based */
  unsigned row = (y - 1) + SNES_TOP_MARGIN;
  if (row >= GW_LCD_HEIGHT) return;   /* clip overscan past the panel */
  memcpy(snes_frame + row * GW_LCD_WIDTH + SNES_LEFT_MARGIN, line, sizeof(snes_line));
}
#endif

static void render_frame_into_active_buffer(void) {
#ifdef SNES_DIRECT_VIDEO
  g_ppu_line_cb = NULL;
  PpuBeginDrawing(snes->ppu,
                  (uint8_t *)(snes_frame + SNES_TOP_MARGIN * GW_LCD_WIDTH +
                              SNES_LEFT_MARGIN),
                  GW_LCD_WIDTH * sizeof(uint16_t), 0);
#else
  g_ppu_line_cb = &snes_blit_line;
  PpuBeginDrawing(snes->ppu, (uint8_t *)snes_line, 0, 0);  /* pitch 0: every line here */
#endif
}

/* Async RGB565 M2M present via firmware dma2d_m2m_rgb565_start / dma2d_poll
 * (objcopy remaps to core_* ABI trampolines). Same flow as integrated
 * jshsakura SNES: kick blit, run audio, then poll. Cache clean stays local
 * (CMSIS); HAL lives only in the firmware. */
#ifdef SNES_PRESENT_DMA2D
static bool snes_dma2d_pending;

static void present_frame_wait(void) {
  if (!snes_dma2d_pending) return;
  for (int i = 0; i < 10; i++) {
    wdog_refresh();
    /* 3 == HAL_TIMEOUT — keep polling until done or hard error. */
    if (dma2d_poll(10) != 3u)
      break;
  }
  wdog_refresh();
  snes_dma2d_pending = false;
}
#else
static void present_frame_wait(void) {}
#endif

static void present_frame(void) {
  uint16_t *dst = lcd_get_active_buffer();
  odroid_display_scaling_t scaling = odroid_display_get_scaling_mode();

  if (scaling == ODROID_DISPLAY_SCALING_OFF) {
    /* 1:1 centred (256x224 in 320x240): snes_frame already has black borders
     * baked in (cleared once at init). DMA2D moves the 320x240 straight copy
     * into the background so the CPU can spend that time on audio/pacing
     * instead of blocked on an uncached memcpy — the caller MUST reach
     * present_frame_wait() before touching dst again. */
#ifdef SNES_PRESENT_DMA2D
    /* snes_frame lives in cacheable RAM_EMU; SNES_DIRECT_VIDEO's PPU write
     * this frame's pixels via normal cached stores. DMA2D is a bus master
     * and cache-blind, so any dirty line not yet evicted would be read as
     * stale — clean (not invalidate: the next frame's PPU render is about
     * to overwrite this same buffer through the cache again) before
     * handing the address to hardware. */
    SCB_CleanDCache_by_Addr((uint32_t *)snes_frame, sizeof(snes_frame));
    if (dma2d_m2m_rgb565_start((uint32_t)(uintptr_t)snes_frame,
                               (uint32_t)(uintptr_t)dst,
                               GW_LCD_WIDTH, GW_LCD_HEIGHT) == 0) {
      snes_dma2d_pending = true;
      return;
    }
    /* DMA2D unavailable for some reason — fall back to the plain CPU copy. */
#endif
    /* Revert switch: drop -DSNES_PRESENT_DMA2D to force this path
     * unconditionally (present_frame_wait() stays a safe no-op either way). */
    memcpy(dst, snes_frame, GW_LCD_WIDTH * GW_LCD_HEIGHT * sizeof(uint16_t));
    return;
  }

  /* FULL (256x224 stretched to 320x240) and FIT / CUSTOM (aspect kept:
   * 274x240 between 23-pixel black borders): src/snes_scale.h, the same
   * nearest-neighbour output as the per-pixel loops it replaced, with 32-bit
   * stores and no division per pixel. */
  if (scaling == ODROID_DISPLAY_SCALING_FULL)
    snes_scale_full(dst, snes_frame, SNES_TOP_MARGIN, SNES_LEFT_MARGIN);
  else
    snes_scale_fit(dst, snes_frame, SNES_TOP_MARGIN, SNES_LEFT_MARGIN);
}

/* Present the last rendered frame and draw the in-game overlay on top. Used both
 * as the normal per-frame present and as the overlay's repaint callback, so the
 * pause menu keeps the game behind it instead of a stale/black background.
 * Fully synchronous (waits out any DMA2D copy immediately) — callers that want
 * the async overlap split present_frame()/present_frame_wait() themselves; see
 * the main loop below. */
static void blit(void) {
  present_frame();
  present_frame_wait();
  common_ingame_overlay();
}

/* Locked half-rate present: emulate every frame (CPU/APU/audio), composite
 * and swap only every other one. Regular frameskip drops whichever frames
 * the integrator is late on, so the cadence jitters; this is a steady 30 Hz
 * (25 Hz PAL) picture. On by default — most titles cannot hold 60 Hz present
 * and the locked 30 Hz cadence is smoother than adaptive skip. */
static bool snes_half_render = true;
static uint8_t snes_half_ctr;
static char snes_half_value[2];
/* Physical G&W pad → SNES port 1 (0) or port 2 (1). MMX2's Cx4 self-test
 * and a few 2P titles read port 2. */
static int snes_pad_port;
static char snes_pad_value[2];

static const gw_i18n_entry_t i18n_half_render[] = {
  { "en", "Half-rate render" },
  { "fr", "Rendu 1 image / 2" },
  { "es", "Render 1 de cada 2" },
  { "de", "Jedes 2. Bild" },
  GW_I18N_END
};

static const gw_i18n_entry_t i18n_pad_port[] = {
  { "en", "Controller" },
  { "fr", "Manette" },
  { "es", "Mando" },
  { "de", "Controller" },
  GW_I18N_END
};

static const gw_i18n_entry_t i18n_pad_map[] = {
  { "en", "Controls" },
  { "fr", "Commandes" },
  { "es", "Controles" },
  { "de", "Steuerung" },
  GW_I18N_END
};

static const gw_i18n_entry_t i18n_record[] = {
  { "en", "Record input" },
  { "fr", "Enregistrer" },
  { "es", "Grabar entrada" },
  { "de", "Eingabe aufnehmen" },
  GW_I18N_END
};

static const gw_i18n_entry_t i18n_replay[] = {
  { "en", "Replay input" },
  { "fr", "Rejouer" },
  { "es", "Reproducir entrada" },
  { "de", "Eingabe abspielen" },
  GW_I18N_END
};

static const gw_i18n_entry_t i18n_record_stop[] = {
  { "en", "Stop recording" },
  { "fr", "Arrêter l'enregistrement" },
  { "es", "Detener grabación" },
  { "de", "Aufnahme beenden" },
  GW_I18N_END
};

static const gw_i18n_entry_t i18n_replay_stop[] = {
  { "en", "Stop replay" },
  { "fr", "Arrêter la relecture" },
  { "es", "Detener reproducción" },
  { "de", "Wiedergabe beenden" },
  GW_I18N_END
};

static const gw_i18n_entry_t i18n_reset[] = {
  { "en", "Reset" },
  { "fr", "Réinitialiser" },
  { "es", "Reiniciar" },
  { "de", "Zurücksetzen" },
  GW_I18N_END
};

/* Locked half-rate: present at snes_fps/2. The panel PLL only knows 50/60
 * (lcd_set_refresh_rate(30) is a no-op), so vsync stays native. The pause
 * bar's FPS is odroid_system_tick via common_emu_frame_loop — one count per
 * call — so we only call it on present slots or the bar keeps saying 60. */
static bool snes_half_lock_active(void)
{
  if (!snes_half_render)
    return false;
  rg_app_desc_t *app = odroid_system_get_app();
  return !app || app->speedupEnabled == SPEEDUP_1x;
}

static void snes_apply_frame_time(void)
{
  int fps = snes_fps;
  if (snes_half_lock_active() && fps > 1)
    fps /= 2;
  common_emu_state.frame_time_10us = (uint16_t)(100000 / fps + 0.5f);
}

static bool snes_half_render_cb(odroid_dialog_choice_t *option,
                               odroid_dialog_event_t event, uint32_t repeat)
{
  (void)repeat;
  if (event == ODROID_DIALOG_PREV || event == ODROID_DIALOG_NEXT) {
    snes_half_render = !snes_half_render;
    odroid_settings_app_int32_set("HalfRender", snes_half_render ? 1 : 0);
    snes_apply_frame_time();
  }
  strcpy(option->value, snes_half_render ? "\x06" : "\x05");
  return event == ODROID_DIALOG_ENTER;
}

static bool snes_pad_port_cb(odroid_dialog_choice_t *option,
                            odroid_dialog_event_t event, uint32_t repeat)
{
  (void)repeat;
  if (event == ODROID_DIALOG_PREV || event == ODROID_DIALOG_NEXT) {
    snes_pad_port = snes_pad_port ? 0 : 1;
    odroid_settings_app_int32_set("PadPort", snes_pad_port);
  }
  strcpy(option->value, snes_pad_port ? "2" : "1");
  return event == ODROID_DIALOG_ENTER;
}

static void snes_pad_map_label(char *dst, size_t dst_sz)
{
  static const char *const names[] = { "Auto", "Zelda", "Mario", "Simple" };
  int p = snes_pad_profile;
  if (p < 0 || p >= SNES_PAD_PROFILE_COUNT)
    p = SNES_PAD_AUTO;
  snprintf(dst, dst_sz, "%s", names[p]);
}

static bool snes_pad_map_cb(odroid_dialog_choice_t *option,
                           odroid_dialog_event_t event, uint32_t repeat)
{
  (void)repeat;
  if (event == ODROID_DIALOG_PREV || event == ODROID_DIALOG_NEXT) {
    if (event == ODROID_DIALOG_NEXT)
      snes_pad_profile = (snes_pad_profile + 1) % SNES_PAD_PROFILE_COUNT;
    else
      snes_pad_profile = (snes_pad_profile + SNES_PAD_PROFILE_COUNT - 1) %
                         SNES_PAD_PROFILE_COUNT;
    odroid_settings_app_int32_set("PadMap", snes_pad_profile);
  }
  snes_pad_map_label(option->value, sizeof(snes_pad_map_value));
  return event == ODROID_DIALOG_ENTER;
}

/* ---- audio ----------------------------------------------------------------
 * Top the DSP up to one frame of samples (534 stereo pairs internally) and
 * downmix to 16 kHz mono, exactly like the harness/rig.
 *
 * DMA is filled by the firmware SAI ISR via pcm_attach(), not from this
 * loop. At ~30 Hz (half-rate present, or a slow title) a main-loop emit
 * always lands on the same dma_state selector, so one half-buffer is
 * rewritten and the other plays stale/short contents — the "buffer never
 * quite fills" sound. The ISR writes every completed half from this ring. */
#define SNES_PCM_RING 2048u
_Static_assert((SNES_PCM_RING & (SNES_PCM_RING - 1u)) == 0u,
               "pcm ring must be a power of two");
static int16_t snes_pcm_ring[SNES_PCM_RING];
static volatile uint16_t snes_pcm_head;
static volatile uint16_t snes_pcm_tail;

static void snes_do_console_reset(void) {
  if (!snes) return;
#ifdef HOST_BUILD
  if (getenv("HOST_SPC7110_LOG") && snes->cart && snes->cart->ram &&
      snes->cart->ramSize > 0) {
    unsigned ff = 0, n11 = 0;
    uint32_t n = snes->cart->ramSize;
    uint32_t i;
    for (i = 0; i < n; i++) {
      if (snes->cart->ram[i] == 0xff) ff++;
      if (snes->cart->ram[i] == 0x11) n11++;
    }
    fprintf(stderr, "spc7110: SRAM %uB before reset: ff=%u eleven=%u last16=",
            (unsigned)n, ff, n11);
    for (i = n - 16; i < n; i++)
      fprintf(stderr, "%02x", snes->cart->ram[i]);
    fprintf(stderr, "\n");
  }
#endif
  wdog_refresh();
  snes_reset(snes, true);
  snes_stretch_reset();
  snes_pcm_head = 0;
  snes_pcm_tail = 0;
  snes_half_ctr = 0;
  wdog_refresh();
}

#ifdef HOST_BUILD
void snes_console_reset(void) {
  snes_do_console_reset();
}
#endif

static bool snes_reset_cb(odroid_dialog_choice_t *option,
                         odroid_dialog_event_t event, uint32_t repeat)
{
  (void)option;
  (void)repeat;
  if (event == ODROID_DIALOG_ENTER)
    snes_do_console_reset();
  return event == ODROID_DIALOG_ENTER;
}

static void snes_pcm_pump(void)
{
  const uint16_t mask = (uint16_t)(SNES_PCM_RING - 1u);
  /* Keep ~3 DMA periods queued so a slow PPU frame (~2 periods) cannot
   * drain the ring down to pcm_fill writing zeros for the rest of a half. */
  uint16_t target = (uint16_t)(snes_audio_samples * 3);
  if (target > (uint16_t)(SNES_PCM_RING / 2u))
    target = (uint16_t)(SNES_PCM_RING / 2u);

  uint16_t tail = snes_pcm_tail;
  uint16_t head = snes_pcm_head;
#ifdef SNES_PERF_AUDIO
  sp_au_isr += (uint16_t)((tail - sp_au_tail) & mask);
  sp_au_tail = tail;
#endif
  uint16_t used = (uint16_t)((head - tail) & mask);
  if (used >= target) {
#ifdef SNES_PERF_AUDIO
    sp_au_nopull++;
#endif
    return;
  }

  uint16_t need = (uint16_t)(target - used);
  uint16_t space = (uint16_t)((tail - head - 1u) & mask);
  if (need > space)
    need = space;

  /* ONE pull for the whole refill. The stretcher sets its playback rate per
   * pull from what was pushed since the previous one; after an overrun frame
   * the refill is two periods, and pulled in two chunks (320 + 212 on NTSC)
   * the rate swung about +-1.2% on every refill -- 30 times a second, which
   * is heard as roughness, not as vibrato. In one piece every pull sees a
   * whole refill and the rate holds still. need <= target <= SNES_PCM_RING/2. */
  static int16_t pump_buf[SNES_PCM_RING / 2u];
  snes_stretch_pull(pump_buf, need);
  for (uint16_t i = 0; i < need; i++) {
    snes_pcm_ring[head] = pump_buf[i];
    head = (uint16_t)((head + 1u) & mask);
  }
  snes_pcm_head = head;
}

static void snes_pcm_submit(void) {
  if (snes->apu) {
#ifdef SNES_SMW_HLE_PRODUCT
    if (g_wire_on) {
      wire_frame_audio(audio_buf, snes_audio_samples);
    } else
#endif
    {
    /* Ledger B, APU-exclusive scope: this whole block is what an exact wire
     * replaces -- the SPC700/DSP top-up AND the sample extraction. Bracketing
     * only the apu_cycle loop would undercount the recoverable cost. One scope
     * per frame, so the probe is noise. */
    uint32_t apu_t0 = SNES_PROF_APU_SCOPE_ENTER();
    /* Bulk apu_run — not 17k× apu_cycle. Music makes each DSP tick expensive;
     * the old per-cycle top-up starved WWDG (~0.4 s) and soft-reset at song
     * start. Feed the dog between chunks. */
    {
      extern void wdog_refresh(void);
      int guard = 0;
      int dsp_need = snes->apu->dsp->frameSamples;
      if (dsp_need <= 0 || dsp_need > DSP_SAMPLES_MAX)
        dsp_need = DSP_SAMPLES_NTSC;
      while (snes->apu->dsp->sampleOffset < dsp_need) {
        int need = dsp_need - snes->apu->dsp->sampleOffset;
        int cycles = need * 32;
        if (cycles > 2048)
          cycles = 2048;
        apu_run(snes->apu, cycles);
        if ((++guard & 3) == 0)
          wdog_refresh();
        if (guard > 512)
          break; /* never spin forever if sampleOffset stalls */
      }
    }
    dsp_getSamples(snes->apu->dsp, audio_buf, snes_audio_samples, 1);
    if (rec_mode) {   /* the record/replay audio hash: the samples the DSP produced */
      const uint8_t *b = (const uint8_t *)audio_buf;
      uint32_t h = rec_audio_h;
      for (int i = 0; i < snes_audio_samples * 2; i++) { h ^= b[i]; h *= 16777619u; }
      rec_audio_h = h;
    }
    SNES_PROF_APU_SCOPE_EXIT(apu_t0);
    }
  } else {
    memset(audio_buf, 0, sizeof(audio_buf));
  }

  /* Mute via pcm_silent so the ISR writes zeros and does not drain the
   * ring. Skip push+pump or a long pause fills the stretcher. */
  int vol = (int)common_emu_sound_get_volume();
  int play = !common_emu_sound_loop_is_muted();
  pcm_audio_set(vol, play);
  if (!play)
    return;

  /* Hand the frame's emulated samples to the stretcher, then top the ISR
   * ring up. The stretcher always writes a full pull; pcm_fill only zeros
   * when the ring is empty. */
  snes_stretch_push(audio_buf, snes_audio_samples);
#ifdef SNES_PERF_AUDIO
  sp_au_push += (uint32_t)snes_audio_samples;
#endif
  snes_pcm_pump();
}

/* ---- savestate -------------------------------------------------------------
 * snes_saveload() streams every subsystem (cpu/apu+dsp/dma/ppu/cart-sram/wram)
 * through one SaveLoadFunc; we stream it straight to the file behind a stamp.
 * ppu_saveload rebuilds its derived caches (palette, sprite-line cache) on
 * load, so no manual invalidation is needed here. */
static FILE *state_file;
static uint32_t state_bytes;

static void state_write(void *ctx, void *data, size_t size) {
  (void)ctx;
  wdog_refresh();
  if (state_file)
    fwrite(data, 1, size, state_file);
  state_bytes += size;
}

static void state_read(void *ctx, void *data, size_t size) {
  (void)ctx;
  wdog_refresh();
  size_t got = state_file ? fread(data, 1, size, state_file) : 0;
  if (got < size)
    memset((uint8_t *)data + got, 0, size - got);
  state_bytes += size;
}

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t length;    /* payload bytes after this header */
} snes_state_header_t;

/* The lib chain (snes_saveload) covers cpu/apu+dsp/dma/ppu/cart/wram but NOT
 * the controller shift registers (Input.latchLine/latchedState) — SM never
 * reads a port serially so the lib never needed them. Real games do: DKC
 * manual-reads $4016 and a cold resume handed it a zeroed shift register
 * where the live machine returns 1s after the auto-joy shift-out (found by
 * the two-process cold-resume proof, tools/snes_save_test). Serialize them
 * here, after the lib stream, without touching the lib. */
static void state_io_input(SaveLoadFunc *func) {
  Input *pads[2] = { snes->input1, snes->input2 };
  for (int i = 0; i < 2; i++) {
    func(NULL, &pads[i]->latchLine, sizeof(pads[i]->latchLine));
    func(NULL, &pads[i]->latchedState, sizeof(pads[i]->latchedState));
  }
}

static void state_stream(SaveLoadFunc *func) {
  snes_saveload(snes, func, NULL);
  state_io_input(func);
}

static bool snes_SaveState(const char *pathName) {
#ifdef SNES_SMW_HLE_PRODUCT
  wire_prepare_save();
#endif
  /* Pass 1: count. Pass 2: write behind an accurate header. */
  state_file = NULL; state_bytes = 0;
  state_stream(&state_write);
  uint32_t payload = state_bytes;

  FILE *f = fopen(pathName, "wb");
  if (!f) return false;
  snes_state_header_t h = { SNES_STATE_MAGIC, SNES_STATE_VERSION, payload };
  if (fwrite(&h, 1, sizeof(h), f) != sizeof(h)) { fclose(f); return false; }
  state_file = f; state_bytes = 0;
  state_stream(&state_write);
  fclose(f);
  state_file = NULL;
  return state_bytes == payload;
}

static bool snes_LoadState(const char *pathName) {
  rec_close();   /* a loaded state is not the recording's machine any more */
  FILE *f = fopen(pathName, "rb");
  if (!f) return false;
  snes_state_header_t h;
  if (fread(&h, 1, sizeof(h), f) != sizeof(h) ||
      h.magic != SNES_STATE_MAGIC || h.version != SNES_STATE_VERSION) {
    /* Not ours / other build: refuse rather than restore nonsense. */
    fclose(f);
    return false;
  }
  /* Refuse BEFORE touching the machine, not after: (a) the payload must be
   * exactly the size this build streams (a lying length would otherwise be
   * caught only after the machine is clobbered), (b) the file must actually
   * contain it (state_read zero-fills past EOF, so a truncated file would
   * "load" a half-zeroed machine and report success — proven by the refusal
   * test before this check existed). */
  /* Dry run with the WRITE counter (file==NULL: counts, reads nothing into
   * the machine — state_read would zero-fill the live machine here). */
  state_file = NULL; state_bytes = 0;
  state_stream(&state_write);          /* what this build expects, in bytes */
  uint32_t expected = state_bytes;
  fseek(f, 0, SEEK_END);
  long fsize = ftell(f);
  if (h.length != expected ||
      fsize != (long)(sizeof(h) + expected)) {
    fclose(f);
    return false;
  }
  fseek(f, sizeof(h), SEEK_SET);
  state_file = f; state_bytes = 0;
  state_stream(&state_read);
  fclose(f);
  state_file = NULL;
  lcd_clear_active_buffer();
  /* A load replaces the whole machine: a learned spin pattern (and its purity
   * sequence history) now describes a machine that no longer exists. Relearn. */
  spin_reset();
#ifdef SNES_SMW_HLE_PRODUCT
  wire_restore_after_load(snes);
#endif
  return state_bytes == h.length;
}

static void *snes_Screenshot(void) {
  // Todo : rerender frame in active buffer to remove HUD
  lcd_wait_for_vblank();
  return lcd_get_active_buffer();
}

#ifdef SNES_SMW_HLE_PRODUCT
/* One-shot, quit-time-only flush of the audio-HLE swap status. Registered
 * as shutdown_cb (odroid_system_shutdown / power-off) and also chained from
 * snes_SramSave so switch_app / sleep still dump it. Both call sites are
 * single quit-time events, never the frame loop.
 * The load-time probe further below can only ever say the gate was ARMED
 * (wire_try_swap() needs ~180+ live frames it doesn't have yet at load) --
 * this is what actually answers whether the swap happened, and if not, the
 * concrete reason, without needing a second device round-trip to find out.
 * Appends a second line to the same /snes_diag.txt so one file covers both
 * halves. */
static void snes_wire_diag_flush(void) {
  extern int g_wire_enable, g_wire_on;
  FILE *df = fopen("/snes_diag.txt", "a");
  if (!df) return;
  /* g_wire_attempt_count/g_wire_swap_frame/g_wire_last_fail_reason are
   * smw_exact_wire.c-specific (its own per-attempt diagnostics) -- they
   * only exist when SMW's exact backend is actually compiled
   * (SNES_SMW_HLE_PRESENT). A generic-only build (SNES_NSPC_HLE=1,
   * SNES_SMW_HLE=0) has neither smw_exact_wire.c nor those globals; an
   * unguarded extern here would be a link error in that configuration. */
#ifdef SNES_SMW_HLE_PRESENT
  extern int g_wire_attempt_count, g_wire_swap_frame;
  extern const char *g_wire_last_fail_reason;
  fprintf(df, "SNES wire at quit: g_wire_enable=%d g_wire_on=%d attempts=%d "
              "swap_frame=%d last_fail=[%s]\n",
          g_wire_enable, g_wire_on, g_wire_attempt_count, g_wire_swap_frame,
          g_wire_last_fail_reason);
#else
  fprintf(df, "SNES wire at quit: g_wire_enable=%d g_wire_on=%d "
              "(generic-only build, no SMW-exact attempt/fail diagnostics)\n",
          g_wire_enable, g_wire_on);
#endif
  fclose(df);
}
#endif

/* Cart battery SRAM as a sidecar /saves/<rom>.sram, same contract as the
 * SMW / Zelda 3 / GBA ports. Savestates still snapshot cart RAM; the sidecar
 * is what in-game Save writes persist across a cold launch. Firmware calls
 * sram_save on app switch and sleep. Load the sidecar after ROM init and
 * before a resumed savestate so the slot's cart RAM wins.
 *
 * SPC7110: 8 KB SRAM, then 16 bytes of RTC-4513. Tengai Makyou Zero's
 * self-test dumps the clock against the power-on image, then writes
 * "SPC7110 CHECK OK" at SRAM $1FF0 only after the player sets date,
 * time and birthday. Restore the RTC solely when that stamp is present
 * (setup is done, check is skipped). Otherwise keep power-on so a
 * mid-check quit still passes the dump. A file that is only ramSize
 * bytes leaves the power-on RTC. */
static void snes_SramSave(void)
{
  Cart *cart;
  size_t wrote = 0;

  rec_close();
  if (rec_sram_blocked) {
    /* a record/replay cleared the cartridge RAM for this session */
    printf("snes: SRAM save skipped (record/replay session)\n");
    return;
  }
  if (!snes || !snes->cart)
    return;
  cart = snes->cart;
  if (!cart->ram || cart->ramSize == 0) {
    printf("snes: SRAM save skipped (ram=%p size=%lu)\n",
           (void *)cart->ram, (unsigned long)cart->ramSize);
    return;
  }
  if (!ACTIVE_FILE || !ACTIVE_FILE->path[0]) {
    printf("snes: SRAM save skipped (no ACTIVE_FILE)\n");
    return;
  }

  wdog_refresh();
  char *path = odroid_system_get_path(ODROID_PATH_SAVE_SRAM, ACTIVE_FILE->path);
  if (path) {
    FILE *f = fopen(path, "wb");
    if (f) {
      wrote = fwrite(cart->ram, 1, cart->ramSize, f);
      if (cart->spc7110)
        wrote += fwrite(cart->spc7110->rtc_ram, 1, 16, f);
      fclose(f);
      printf("snes: SRAM saved %s (%lu bytes)\n", path, (unsigned long)wrote);
    } else {
      printf("snes: SRAM save failed fopen %s\n", path);
    }
    free(path);
  } else {
    printf("snes: SRAM save skipped (no path)\n");
  }
  wdog_refresh();
#ifdef SNES_SMW_HLE_PRODUCT
  snes_wire_diag_flush();
#endif
}

static void snes_SramLoad(void)
{
  Cart *cart;
  size_t got = 0;

  if (!snes || !snes->cart)
    return;
  cart = snes->cart;
  if (!cart->ram || cart->ramSize == 0)
    return;
  if (!ACTIVE_FILE || !ACTIVE_FILE->path[0])
    return;

  wdog_refresh();
  char *path = odroid_system_get_path(ODROID_PATH_SAVE_SRAM, ACTIVE_FILE->path);
  if (path) {
    FILE *f = fopen(path, "rb");
    if (f) {
      got = fread(cart->ram, 1, cart->ramSize, f);
      if (cart->spc7110 && got == cart->ramSize) {
        uint8_t rtc[16];
        if (fread(rtc, 1, 16, f) == 16) {
          static const char check_ok[16] = "SPC7110 CHECK OK";
          if (cart->ramSize >= 16 &&
              memcmp(cart->ram + (cart->ramSize - 16), check_ok, 16) == 0)
            memcpy(cart->spc7110->rtc_ram, rtc, 16);
          got += 16;
        }
      }
      fclose(f);
      printf("snes: SRAM loaded %s (%lu bytes)\n", path, (unsigned long)got);
    } else {
      printf("snes: SRAM none at %s\n", path);
    }
    free(path);
  }
  wdog_refresh();
}

/* ---- ROM loading -----------------------------------------------------------
 * The cart stays memory-mapped in external flash (flash-cache machinery); the
 * core only reads it. Copier headers (512 bytes) are skipped in place. */
static const uint8_t *snes_rom;
static uint32_t snes_rom_len;

/* ---- input record / replay ---------------------------------------------
 * A recording is the SNES pad of every emulated frame (port 1 or 2 as
 * configured, AFTER the G&W button mapping, so a mapping change does not
 * break it), from a power-on with the cartridge RAM cleared, in
 * "<rom path without extension>.rec" next to the ROM:
 *   header (16 bytes): "SNRC", version 1, flags (bit 0 PAL), FNV-1a of the
 *   ROM, frame count (0 until the recording is closed); then one
 *   little-endian uint16 per frame.
 * Record on the host (SNES_RECORD=1) or the device (pause menu), replay on
 * either (SNES_REPLAY=1 / pause menu): the same file. Every 120 frames both
 * log "REC/hash", a hash of work RAM, VRAM, CGRAM, OAM, the APU RAM and the
 * CPU registers: two runs of one recording must log the same lines.
 * The cartridge RAM is cleared for the session, so the SRAM save is skipped
 * from the first record/replay until the game is restarted. */
#define REC_MAGIC   0x43524e53u   /* "SNRC" */
#define REC_VERSION 1u
#define REC_BUF     1024u
typedef struct { uint32_t magic; uint16_t version, flags; uint32_t rom_hash, frames; } rec_header_t;
enum { REC_OFF = 0, REC_RECORD, REC_REPLAY };
static int rec_mode;
static FILE *rec_f;
static uint32_t rec_frame, rec_rom_hash;
static uint16_t rec_buf[REC_BUF];
static uint32_t rec_n, rec_pos;
static odroid_dialog_choice_t *rec_opt_record, *rec_opt_replay;   /* their pause-menu entries */
static void rec_labels(void);

static uint32_t rec_fnv(const void *p, uint32_t n, uint32_t h)
{
  const uint8_t *b = (const uint8_t *)p;
  for (uint32_t i = 0; i < n; i++) { h ^= b[i]; h *= 16777619u; }
  return h;
}

static void rec_path(char *out, size_t cap)
{
  snprintf(out, cap, "%s", ACTIVE_FILE ? ACTIVE_FILE->path : "snes");
  char *dot = strrchr(out, '.'), *slash = strrchr(out, '/');
  if (dot && (!slash || dot > slash)) *dot = '\0';
  strncat(out, ".rec", cap - strlen(out) - 1);
}

static void rec_flush(void)
{
  if (rec_f && rec_n) fwrite(rec_buf, sizeof(uint16_t), rec_n, rec_f);
  rec_n = 0;
}

static void rec_close(void)
{
  if (!rec_f) { rec_mode = REC_OFF; return; }
  if (rec_mode == REC_RECORD) {
    rec_flush();
    rec_header_t h = { REC_MAGIC, REC_VERSION, (uint16_t)(snes->pal ? 1 : 0), rec_rom_hash, rec_frame };
    fseek(rec_f, 0, SEEK_SET);
    fwrite(&h, sizeof h, 1, rec_f);
    printf("REC: recorded %lu frames\n", (unsigned long)rec_frame);
  } else {
    printf("REC: replay stopped at frame %lu\n", (unsigned long)rec_frame);
  }
  fclose(rec_f);
  rec_f = NULL;
  rec_mode = REC_OFF;
  rec_labels();
}

/* power-on, cartridge RAM cleared, the spin learner and the bake as after
 * a ROM load: the same machine every time */
static void rec_power_on(void)
{
  snes_do_console_reset();
  if (snes->cart && snes->cart->ram && snes->cart->ramSize)
    memset(snes->cart->ram, 0, snes->cart->ramSize);
#ifdef SNES_SPIN_BAKE
  spin_bake_reset();
  spin_bake_scan(snes);
#endif
  spin_reset();
  rec_sram_blocked = true;
  rec_frame = 0;
  rec_audio_h = 2166136261u;
#ifdef HOST_BUILD
  rec_pic_h = 2166136261u;
#endif
  rec_n = rec_pos = 0;
#ifdef SNES_PERF
  snes_perf_reset();
#endif
}

static bool rec_start(int mode)
{
  char path[300];
  rec_close();
  rec_path(path, sizeof path);
  if (!rec_rom_hash) rec_rom_hash = rec_fnv(snes_rom, snes_rom_len, 2166136261u);
  if (mode == REC_RECORD) {
    rec_f = fopen(path, "wb");
    if (!rec_f) { printf("REC: cannot create %s\n", path); return false; }
    rec_header_t h = { REC_MAGIC, REC_VERSION, (uint16_t)(snes->pal ? 1 : 0), rec_rom_hash, 0 };
    fwrite(&h, sizeof h, 1, rec_f);
  } else {
    rec_header_t h;
    rec_f = fopen(path, "rb");
    if (!rec_f) { printf("REC: no recording %s\n", path); return false; }
    if (fread(&h, sizeof h, 1, rec_f) != 1 || h.magic != REC_MAGIC || h.version != REC_VERSION) {
      printf("REC: %s is not a recording (version %u)\n", path, 1u);
      fclose(rec_f); rec_f = NULL; return false;
    }
    if (h.rom_hash != rec_rom_hash || (h.flags & 1) != (snes->pal ? 1 : 0)) {
      printf("REC: %s was recorded with a different ROM (%08lx, this one %08lx)\n", path,
             (unsigned long)h.rom_hash, (unsigned long)rec_rom_hash);
      fclose(rec_f); rec_f = NULL; return false;
    }
    printf("REC: replaying %lu frames from %s\n", (unsigned long)h.frames, path);
  }
  rec_mode = mode;
  rec_labels();
  rec_power_on();
  printf("REC: %s from power-on, %s, ROM %08lx\n", mode == REC_RECORD ? "recording" : "replay",
         snes->pal ? "PAL" : "NTSC", (unsigned long)rec_rom_hash);
  return true;
}

/* the pad the emulated frame gets: recorded, or replayed in place of the live one */
static uint16_t rec_pad(uint16_t live)
{
  if (rec_mode == REC_RECORD) {
    rec_buf[rec_n++] = live;
    if (rec_n == REC_BUF) rec_flush();
    return live;
  }
  if (rec_mode == REC_REPLAY) {
    if (rec_pos == rec_n) {
      rec_n = (uint32_t)fread(rec_buf, sizeof(uint16_t), REC_BUF, rec_f);
      rec_pos = 0;
      if (!rec_n) {
        printf("REC: replay finished at frame %lu, live input from here\n", (unsigned long)rec_frame);
        rec_close();
        return live;
      }
    }
    return rec_buf[rec_pos++];
  }
  return live;
}

static void rec_frame_done(void)
{
  if (rec_mode == REC_OFF) return;
  rec_frame++;
#ifdef HOST_BUILD
  {
    const Ppu *p = snes->ppu;
    uint32_t h = rec_pic_h;
    for (int row = 0; row < 224; row++) {
      const uint32_t *px = (const uint32_t *)(const void *)(p->renderBuffer + row * p->renderPitch);
      for (int i = 0; i < 128; i++) { h ^= px[i]; h *= 16777619u; }
    }
    rec_pic_h = h;
  }
#endif
  if (rec_frame % 120u) return;
#ifdef HOST_BUILD
  printf("REC/pic frame=%lu pic=%08lx\n", (unsigned long)rec_frame, (unsigned long)rec_pic_h);
#endif
  const Cpu *cpu = snes->cpu;
  uint32_t cpu_h = rec_fnv(&cpu->a, sizeof cpu->a, 2166136261u);
  cpu_h = rec_fnv(&cpu->x, sizeof cpu->x, cpu_h);
  cpu_h = rec_fnv(&cpu->sp, sizeof cpu->sp, cpu_h);
  cpu_h = rec_fnv(&cpu->pc, sizeof cpu->pc, cpu_h);
  cpu_h = rec_fnv(&cpu->dp, sizeof cpu->dp, cpu_h);
  cpu_h = rec_fnv(&cpu->k, sizeof cpu->k, cpu_h);
  cpu_h = rec_fnv(&cpu->db, sizeof cpu->db, cpu_h);
  printf("REC/hash frame=%lu wram=%08lx vram=%08lx cgram=%08lx oam=%08lx aram=%08lx cpu=%08lx audio=%08lx\n",
         (unsigned long)rec_frame,
         (unsigned long)rec_fnv(snes->ram, 0x20000, 2166136261u),
         (unsigned long)rec_fnv(snes->ppu->vram, 0x8000 * sizeof(uint16_t), 2166136261u),   /* a pointer under TARGET_GNW */
         (unsigned long)rec_fnv(snes->ppu->cgram, sizeof snes->ppu->cgram, 2166136261u),
         (unsigned long)rec_fnv(snes->ppu->oam, sizeof snes->ppu->oam, rec_fnv(snes->ppu->highOam, sizeof snes->ppu->highOam, 2166136261u)),
         (unsigned long)rec_fnv(snes->apu->ram, sizeof snes->apu->ram, 2166136261u),
         (unsigned long)cpu_h, (unsigned long)rec_audio_h);
}

/* actions, not toggles: the label says what selecting the entry does */
static void rec_labels(void)
{
  if (rec_opt_record) rec_opt_record->label = gw_i18n(rec_mode == REC_RECORD ? i18n_record_stop : i18n_record);
  if (rec_opt_replay) rec_opt_replay->label = gw_i18n(rec_mode == REC_REPLAY ? i18n_replay_stop : i18n_replay);
}

static bool rec_record_cb(odroid_dialog_choice_t *option, odroid_dialog_event_t event, uint32_t repeat)
{
  (void)option; (void)repeat;
  if (event == ODROID_DIALOG_ENTER) {
    if (rec_mode == REC_RECORD) rec_close();
    else rec_start(REC_RECORD);
  }
  rec_labels();
  return event == ODROID_DIALOG_ENTER;
}

static bool rec_replay_cb(odroid_dialog_choice_t *option, odroid_dialog_event_t event, uint32_t repeat)
{
  (void)option; (void)repeat;
  if (event == ODROID_DIALOG_ENTER) {
    if (rec_mode == REC_REPLAY) rec_close();
    else rec_start(REC_REPLAY);
  }
  rec_labels();
  return event == ODROID_DIALOG_ENTER;
}

#ifdef HOST_BUILD
/* SNES_RECORD / SNES_REPLAY at start-up; the recording is closed at exit */
static void rec_host_start(void)
{
  if (getenv("SNES_RECORD")) rec_start(REC_RECORD);
  else if (getenv("SNES_REPLAY")) rec_start(REC_REPLAY);
  atexit(rec_close);
}
#endif

/* $ffd6 (ROM type): 0=ROM 1=ROM+RAM 2=ROM+RAM+battery; 3+ = coprocessor
 * (DSP-x/SA-1/SuperFX/...). The DSP-1 family (high nibble 0, low nibble 3-5)
 * has HLE support in dsp1_hle.c, and Cx4 (romType $F0-$FF with $FFBF=$10)
 * in cx4_hle.c. Everything else is rejected. */
static bool cart_needs_coprocessor(const uint8_t *rom, uint32_t len) {
  static const uint32_t offs[2] = { 0x7fb0, 0xffb0 };   /* LoROM, HiROM */
  for (int i = 0; i < 2; i++) {
    if (offs[i] + 0x30 > len) continue;
    const uint8_t *h = rom + offs[i];
    uint16_t cks  = h[0x2e] | (h[0x2f] << 8);
    uint16_t icks = h[0x2c] | (h[0x2d] << 8);
    if ((cks ^ icks) == 0xffff) {
      uint8_t romType = h[0x26];   /* $ffd6 = header+0x26 */
      /* high nibble 0 = DSP family (HLE in dsp1_hle.c).
       * romType 0xF0-0xFF with exCoprocessor 0x10 = Cx4 (HLE in cx4_hle.c).
       * anything else with romType >= 3 is a coprocessor we don't support yet */
      if (romType >= 0x03 && (romType >> 4) != 0) {
        if ((romType >> 4) == 0x0F && h[0x0f] == 0x10) {
          /* Cx4 is supported */
        } else if ((romType >> 4) == 0x04) {
          /* S-DD1 is supported (Star Ocean, SFA2) */
        } else if ((romType >> 4) == 0x0F && (romType & 0x0f) == 0x09) {
          /* SPC7110 is supported (Far East of Eden Zero / Momotaro / SPL4) */
        } else {
          return true;
        }
      }

      /* The DSP family is four chips behind one encoding; only DSP-1 is HLE'd.
       * See snes_dsp_variant.h. Cx4 is high nibble $F — skip this title check. */
      if (romType >= 0x03 && (romType >> 4) == 0) {
        char name[22];
        for (int c = 0; c < 21; c++) {
          uint8_t ch = h[c];
          name[c] = (ch >= 0x20 && ch < 0x7f) ? (char)ch : '.';
        }
        name[21] = 0;
        if (snes_title_needs_unsupported_dsp(name))
          return true;
      }
    }
  }
  return false;
}

/* ---- rc SMW native optimization ---------------------------------------------
 * Per-ROM static recompilation: SMW's 270 hottest 65816 sites translated to C.
 * The overlay's cpu_runOpcode fast path (g_rc_active) dispatches to sites
 * instead of interpreting. Rig-measured: -42.3% insn/frame on SMW.
 *
 * ITCM-resident: the rc hot subset (270 sites, ~12 KB) is linked at ITCM VMA
 * and copied there by run_internal_emu BEFORE app_main runs. Zero wait-state
 * execution — no QSPI cache, no sentinel patching, no XIP thrash.
 * rc takes priority over spin-skip (rc replaces the interpreter entirely). */
#define RC_SMW_MAGIC     0x4D534352u   /* 'RCSM' little-endian — blob header */

/* Diag: code-region hash computed at activation time, reported in profile2 so
 * a mismatch (user's dump has different code at translated PCs) is visible
 * without a debugger. */
static uint32_t s_rc_diag_crc = 0;   /* computed code hash (name kept for diag fmt) */
static uint32_t s_rc_diag_exp = 0;   /* expected code hash from blob header */

/* Blob header layout (matches rc_smw_sites.c's rc_smw_header). Pointer fields
 * are linked at ITCM VMA directly — no sentinel patching needed. */
typedef struct {
  uint32_t magic;
  uint32_t nsites;
  uint32_t code_hash;   /* FNV-1a of consumed bytes (opcode+operands) at all site PCs */
  const uint32_t *addrs;   /* ITCM VMA of rc_addrs[] */
  const void **fns;        /* ITCM VMA of rc_fns[] */
  const uint8_t *lens;     /* ITCM VMA of rc_site_lens[] */
} rc_smw_header_t;

/* The header symbol — defined in rc_smw_sites.c, linked into .itcm_rc_hot
 * (ITCM VMA). Available as a direct extern because run_internal_emu copies the
 * ITC blob to ITCM before app_main runs. */
extern const rc_smw_header_t rc_smw_header;

/* SNES-overlay-resident hash storage (defined in rc_smw_sites.c, linked into
 * .overlay_snes_bss). Passed to rc_dispatch_init() so it never touches the
 * 81 KB DTCM heap. */
extern rc_entry_t rc_hash_storage[];
extern uint32_t rc_bank_off[];
extern uint32_t rc_bank_mask[];

/* SMW title hash (FNV-1a of 21-byte internal title at LoROM 0x7FC0).
 * Same value the spin-skip whitelist uses — quick reject for non-SMW ROMs. */
#define RCSMW_TITLE_HASH  0xFB0BD0ECu

static bool rc_smw_activate(const uint8_t *rom, uint32_t len) {
#if RCSMW
  if (len == 0 || rom == NULL) return false;

  /* Quick reject: SMW internal title hash (21 bytes at LoROM 0x7FC0).
   * Same FNV-1a the spin-skip whitelist uses — cheap reject for non-SMW ROMs. */
  if (len < 0x7FD5) return false;
  uint32_t th = 0x811C9DC5u;
  for (int i = 0; i < 21; i++) {
    th ^= rom[0x7FC0 + i];
    th *= 0x01000193u;
  }
  if (th != RCSMW_TITLE_HASH) {
    printf("rc_smw: not SMW (title hash %08lX != %08lX)\n",
           (unsigned long)th, (unsigned long)RCSMW_TITLE_HASH);
    return false;
  }

  /* The header is a direct symbol at ITCM VMA — code already copied by
   * run_internal_emu. No fopen/QSPI cache/sentinel patching. */
  const rc_smw_header_t *hdr = &rc_smw_header;
  if (hdr->magic != RC_SMW_MAGIC || hdr->nsites == 0) {
    printf("rc_smw: bad header (magic=0x%08lX nsites=%lu)\n",
           (unsigned long)hdr->magic, (unsigned long)hdr->nsites);
    return false;
  }

  /* Code-region hash gate: FNV-1a of consumed bytes (opcode + operands) at
   * every translated site PC. "The bytes are identity" — same principle as
   * GBA M4A HLE. Accepts any dump/patch/hack whose CODE at the translated PCs
   * is byte-identical to the reference dump; rejects any code change.
   * Text/graphics hacks (different data, same code) pass correctly. */
  uint32_t rom_mask = len - 1;
  uint32_t ch = 0x811C9DC5u;
  for (uint32_t i = 0; i < hdr->nsites; i++) {
    uint32_t a = hdr->addrs[i];
    uint8_t bank = a >> 16;
    uint16_t off = a & 0xFFFF;
    uint32_t idx = ((uint32_t)(bank & 0x7F) << 15) | (off & 0x7FFF);
    int nbytes = 1 + hdr->lens[i];   /* opcode + operand bytes */
    for (int j = 0; j < nbytes; j++) {
      ch ^= rom[(idx + j) & rom_mask];
      ch *= 0x01000193u;
    }
  }
  s_rc_diag_crc = ch;
  s_rc_diag_exp = hdr->code_hash;
  if (ch != hdr->code_hash) {
    printf("rc_smw: code hash mismatch (got %08lX want %08lX) — not activating\n",
           (unsigned long)ch, (unsigned long)hdr->code_hash);
    return false;
  }

  /* Build the dispatch table. rc_fns[] pointers are at ITCM VMA (linked
   * directly, no patching). The hash storage lives in SNES overlay BSS
   * (rc_smw_sites.c) — NOT the DTCM heap. After this call, g_rc_active is
   * true and cpu_runOpcode uses the rc fast path. */
  rc_dispatch_init(rc_hash_storage, rc_bank_off, rc_bank_mask,
                   hdr->addrs, hdr->nsites, (void (**)(Cpu *))hdr->fns);
  printf("rc_smw: activated — %lu sites, ITCM header at %p\n",
         (unsigned long)hdr->nsites, hdr);
  return true;
#else
  (void)rom; (void)len;
  return false;
#endif
}

/* In ITCM with the engine it drives — same as jshsakura monolithic overlay
 * (avoids BL veneers to cpu_runOpcode / snes_cpuRead every opcode). */
#ifdef SNES_BUS_IN_ITCM
__attribute__((section(".itcm_snes_interp.thumb2.bus")))
#endif
void app_main_snes(uint8_t load_state, uint8_t start_paused, int8_t save_slot)
{
  gw_core_bridge_init();
  /* Firmware / host shim seeds ram_start before entry; only rewind the bump. */
  ram_init();
  /* Firmware already dtc_init()'d in emulator_start() (ACTIVE_FILE was copied
   * to AHB first). Rewind again so this core owns a known-empty DTCM bump —
   * mem_ctl(GW_MEM_OP_INIT, GW_MEM_DTC). Do not call this after dtc_* allocs. */
  dtc_init();

  SystemClock_Config(3);

  odroid_gamepad_state_t joystick;
  odroid_dialog_choice_t options[] = {
      {100, gw_i18n(i18n_half_render), snes_half_value, 1, &snes_half_render_cb},
      {101, gw_i18n(i18n_pad_port), snes_pad_value, 1, &snes_pad_port_cb},
      {103, gw_i18n(i18n_pad_map), snes_pad_map_value, 1, &snes_pad_map_cb},
      {102, gw_i18n(i18n_reset), NULL, 1, &snes_reset_cb},
      {104, gw_i18n(i18n_record), NULL, 1, &rec_record_cb},
      {105, gw_i18n(i18n_replay), NULL, 1, &rec_replay_cb},
      ODROID_DIALOG_CHOICE_LAST
  };
  for (odroid_dialog_choice_t *o = options; o->id != 0x0F0F0F0F; o++) {   /* ODROID_DIALOG_CHOICE_LAST */
    if (o->id == 104) rec_opt_record = o;
    if (o->id == 105) rec_opt_replay = o;
  }
  rec_labels();

  if (start_paused) {
    common_emu_state.pause_after_frames = 2;
    odroid_audio_mute(true);
  } else {
    common_emu_state.pause_after_frames = 0;
  }
  common_emu_state.frame_time_10us = (uint16_t)(100000 / SNES_FPS_NTSC + 0.5f);
  lcd_set_refresh_rate(SNES_FPS_NTSC);

  odroid_system_init(APPID_SNES, SNES_AUDIO_RATE);
  snes_half_render = odroid_settings_app_int32_get("HalfRender", 1) != 0;
  snes_pad_port = odroid_settings_app_int32_get("PadPort", 0) != 0 ? 1 : 0;
  {
    int pm = odroid_settings_app_int32_get("PadMap", SNES_PAD_AUTO);
    if (pm < 0 || pm >= SNES_PAD_PROFILE_COUNT)
      pm = SNES_PAD_AUTO;
    snes_pad_profile = pm;
  }
  snes_apply_frame_time();
#ifdef SNES_SMW_HLE_PRODUCT
  odroid_system_emu_init(&snes_LoadState, &snes_SaveState, &snes_Screenshot,
                         &snes_wire_diag_flush, NULL, &snes_SramSave, NULL);
#else
  odroid_system_emu_init(&snes_LoadState, &snes_SaveState, &snes_Screenshot,
                         NULL, NULL, &snes_SramSave, NULL);
#endif

  /* Defer audio_start_playing until after ROM load: SAI DMA running during
   * the multi-100KB snes_reset memsets has been a HardFault suspect. */

  memset(snes_wram, 0, sizeof(snes_wram));

  if (!ACTIVE_FILE || !ACTIVE_FILE->path[0]) {
    printf("snes: ACTIVE_FILE missing\n");
    draw_error_screen("SNES ROM path missing", "ACTIVE_FILE unset", "Press a button");
    lcd_swap();
    snes_wait_button();
    odroid_system_switch_app(0);
    return;
  }
  printf("snes: cache '%s'\n", ACTIVE_FILE->path);

  uint32_t sz = 0;
  const uint8_t *rom = odroid_overlay_cache_file_in_flash(ACTIVE_FILE->path, &sz, false);
  printf("snes: cache done rom=%p sz=%lu\n", rom, (unsigned long)sz);
  if (rom && sz > 512 && (sz % 1024) == 512) {   /* copier header */
    rom += 512;
    sz  -= 512;
  }
  snes_rom = rom; snes_rom_len = sz;

  /* Flash program may leave stale D-cache lines for this window — drop them
   * before the CPU walks the image (header score / reset vector / XIP fetch). */
  if (rom && sz) {
    SCB_InvalidateDCache_by_Addr((void *)(uintptr_t)rom, (int32_t)sz);
    wdog_refresh();
  }

#ifdef SNES_SMW_HLE_PRODUCT
  smw_hle_frame = 0;
  bool wire_armed = wire_configure_rom(rom, sz);
  printf("SNES SMW audio HLE: %s\n", wire_armed ? "armed (exact ROM)" : "LLE fallback");
#endif

  printf("snes: snes_init\n");
  snes = snes_init(snes_wram);
  g_the_snes = snes;
  if (!snes || !snes->apu || !snes->apu->spc || !snes->apu->dsp ||
      !snes->cpu || !snes->ppu || !snes->cart || !snes->dma ||
      !snes->input1 || !snes->input2) {
    printf("snes: snes_init failed snes=%p apu=%p spc=%p dsp=%p cpu=%p dma=%p\n",
           snes,
           snes ? snes->apu : NULL,
           (snes && snes->apu) ? snes->apu->spc : NULL,
           (snes && snes->apu) ? snes->apu->dsp : NULL,
           snes ? snes->cpu : NULL,
           snes ? snes->dma : NULL);
    draw_error_screen("SNES init failed", "(out of memory?)", "Press a button");
    lcd_swap();
    snes_wait_button();
    odroid_system_switch_app(0);
    return;
  }
  printf("snes: snes_init ok apu=%p spc=%p dsp=%p\n",
         (void *)snes->apu,
         snes->apu ? (void *)snes->apu->spc : NULL,
         snes->apu ? (void *)snes->apu->dsp : NULL);
  /* rc SMW activation takes priority over spin-skip. If the ITCM metadata or
   * ROM identity check fails, fall back to the spin-skip whitelist. */
  printf("snes: whitelist\n");
  if (!rc_smw_activate(rom, sz)) {
    if (rom)
      spin_whitelist_set(rom, sz);   /* enable spin-skip only for high-spin ROMs */
  }
#ifdef SNES_SPIN_FORCE_ON
  /* DIAGNOSTIC ARM. spin_table turns Zelda 3 off by name on the strength of a
   * 25.0% skip rate against a ~50% break-even -- and BOTH of those numbers are
   * QEMU rig instruction counts. This tree's own ledger says that rig is blind
   * to roughly 70% of hardware time, and a replayed opcode skips the cache
   * misses and flash stalls as well as the interpreter work, so the DEVICE
   * break-even should sit below the rig's. Nobody has measured it. This forces
   * the learner on regardless of the table so the third arm of a three-way
   * device A/B exists at all; it is not a shipping path. */
  extern bool g_spin_whitelist;
  g_spin_whitelist = true;
#endif
  printf("snes: spin_reset\n");
  spin_reset();   /* clean slate either way (spin-skip learner / rc dispatch) */
  snes_stretch_reset();   /* and the audio stretcher: a new ROM starts empty */
  snes_pcm_head = 0;
  snes_pcm_tail = 0;

  printf("snes: loadRom enter\n");
  wdog_refresh();
  bool copro = (rom != NULL) && cart_needs_coprocessor(rom, sz);
  printf("snes: copro=%d\n", (int)copro);
  wdog_refresh();
  bool loaded = (rom != NULL) && !copro && snes_loadRom(snes, rom, (int)sz);
  printf("snes: loadRom done loaded=%d\n", (int)loaded);
  if (!loaded) {
    /* draw_error_screen does not wait — without a button gate, switch_app(0)
     * soft-resets immediately (boot_magic stays BOOT_MAGIC_RESET), which looks
     * like a mysterious reboot right after emu_init. */
    const char *why = !rom ? "ROM cache failed"
                    : copro ? "coprocessor/mapper"
                    : "snes_loadRom failed";
    printf("snes: load failed: %s (sz=%lu)\n", why, (unsigned long)sz);
    draw_error_screen("Unsupported SNES cartridge", why, "Press a button");
    lcd_swap();
    snes_wait_button();
    odroid_system_switch_app(0);
    return;
  }
#ifdef SNES_SPIN_BAKE
  spin_bake_scan(snes);
  printf("snes: bake on=%d sites=%lu pc=%02x:%04x dp=$%02x\n", (int)g_bake.on,
         (unsigned long)g_bake.sites, g_bake.bank, g_bake.pc_load, g_bake.dp_off);
#endif
  snes_SramLoad();
  snes_fps = snes->pal ? SNES_FPS_PAL : SNES_FPS_NTSC;
  snes_audio_samples = SNES_AUDIO_RATE / snes_fps;
  snes_apply_frame_time();
  lcd_set_refresh_rate((uint32_t)snes_fps);
  printf("snes: %s %dHz (%u lines)\n",
         snes->pal ? "PAL" : "NTSC", snes_fps, (unsigned)snes->vcount);
  /* Defer SAI until the loop: starting DMA during/right after the 64 KB ARAM
   * wipe raced the bus on earlier bring-up builds. */
  printf("snes: ROM loaded, entering loop\n");
  int audio_started = 0;
  /* The loader (GNW_SNES_CORE build) already points cart->rom straight at the
   * flash-cached image — no malloc'd copy exists to free. Do NOT free cart->rom
   * here: it is read-only flash, not heap, and the header-skip may have offset
   * it past a copier header (snes_rom + 0x200). */

  /* The load-time /snes_diag.txt probe lived here: ROM title, 64K checksum,
   * dsp1/ramSize and the HLE wire flags, written on every ROM load. It was
   * bring-up instrumentation for the device-only black screen and the Mario
   * Kart BSOD, both of which are closed, and it wrote a file to the card on
   * every single launch to say so. Removed; git history has it if a device-
   * only load fault ever needs it again (git log -S snes_diag.txt). */

  /* (Baseline profile, on device, pre-levers: 312 MHz, budget 5.2M cyc/frame;
   * interpreter+APU 7.12M, PPU +2.82M, audio 0.71M → ~29 fps raw. Rig insn ≈
   * device cycle ~1:1.) */

#ifdef SNES_LOAD_DIAG
  /* Headless 500-frame DWT profile (~11-18 s stall). Gated OFF by default;
   * build with -DSNES_LOAD_DIAG (the Makefile's SNES_LOAD_DIAG=1 flag wires
   * it up and swaps in DWT-instrumented apu.c/dsp.c copies that feed the
   * three g_diag_* buckets below). Post-lever frame-cost PROFILE -- same
   * DWT probe, now with spin-skip stats AND an APU split (SPC700 vs DSP,
   * DSP further split into echo FIR vs the rest), so the next lever is
   * aimed at a measured number, not a guess. Headless, before the
   * interactive loop, SD idle, wdog kicked.
   *
   * The g_diag_* counters are defined in the generated apu.c and incremented
   * at every spc_runOpcode / dsp_cycle / dsp_handleEcho call site. They are
   * read ONCE after the 250-frame draw loop and divided by 250 for a per-
   * frame average; DWT_CYCCNT is reset per frame (preserving the existing
   * cyc_skip/cyc_draw measurement), and the per-call deltas the
   * instrumentation accumulates are independent of that reset. */
  {
    extern uint32_t SystemCoreClock;
    extern uint64_t g_diag_spc_cycles, g_diag_dsp_cycles,
                    g_diag_dsp_echo_cycles;
    common_emu_enable_dwt_cycles();
    uint64_t cyc_skip = 0, cyc_draw = 0, cyc_audio = 0;

    for (int i = 0; i < 250; i++) {           /* interpreter+APU only */
      wdog_refresh();
      g_ppu_skip_render = true;
      render_frame_into_active_buffer();
      common_emu_clear_dwt_cycles();
      run_frame_events(snes);
      cyc_skip += common_emu_get_dwt_cycles();
    }
    uint64_t spin_v0 = g_spin.ops_virtual, spin_r0 = g_spin.ops_real;

    /* Zero the APU buckets AFTER the skip loop (which also runs the APU) so
     * the read below captures the draw loop only -- the path the device
     * actually takes when PPU is on and audio is streaming. */
    g_diag_spc_cycles = 0;
    g_diag_dsp_cycles = 0;
    g_diag_dsp_echo_cycles = 0;

    memset(snes_frame, 0, sizeof(snes_frame));
    for (int i = 0; i < 250; i++) {           /* + PPU line renderer + audio */
      wdog_refresh();
      g_ppu_skip_render = false;
      render_frame_into_active_buffer();
      common_emu_clear_dwt_cycles();
      run_frame_events(snes);
      cyc_draw += common_emu_get_dwt_cycles();

      common_emu_clear_dwt_cycles();
      snes_pcm_submit();
      cyc_audio += common_emu_get_dwt_cycles();
    }
    uint64_t vd = g_spin.ops_virtual - spin_v0, rd = g_spin.ops_real - spin_r0;

    /* 250-frame sums -> per-frame averages. skeleton = interpreter+memory
     * alone (cyc_skip minus the APU work that the skip loop also ran);
     * PPU = cyc_draw - cyc_skip (the line renderer + mode 7 the skip loop
     * skipped); DSP non-echo = dsp - echo (the channel mix path a 16 kHz
     * mono decimation would NOT touch); echo = the FIR filter work it
     * WOULD halve. audio = snes_pcm_submit (the DMA buffer handoff). */
    uint64_t skel_pf = cyc_skip / 250 - g_diag_spc_cycles / 250
                     - g_diag_dsp_cycles / 250;
    uint64_t ppu_pf  = (cyc_draw - cyc_skip) / 250;
    uint64_t spc_pf  = g_diag_spc_cycles / 250;
    uint64_t dsp_pf  = g_diag_dsp_cycles / 250;
    uint64_t echo_pf = g_diag_dsp_echo_cycles / 250;
    uint64_t nonecho_pf = dsp_pf - echo_pf;
    uint64_t audio_pf = cyc_audio / 250;
    uint64_t total_pf = skel_pf + ppu_pf + spc_pf + dsp_pf + audio_pf;

    FILE *df = fopen("/snes_diag.txt", "a");
    if (df) {
      fprintf(df, "SNES profile3: clk=%lu skel=%lu(%lu%%) ppu=%lu(%lu%%) "
                  "spc=%lu(%lu%%) dsp=%lu(%lu%%) [echo=%lu(%lu%%) non_echo=%lu(%lu%%)] "
                  "audio=%lu(%lu%%) total=%lu spin=%lu%%(gate=%d) rc=%d "
                  "hash=%08lX/exp=%08lX\n",
              (unsigned long)SystemCoreClock,
              (unsigned long)skel_pf,  (unsigned long)(total_pf ? 100*skel_pf/total_pf : 0),
              (unsigned long)ppu_pf,   (unsigned long)(total_pf ? 100*ppu_pf/total_pf : 0),
              (unsigned long)spc_pf,   (unsigned long)(total_pf ? 100*spc_pf/total_pf : 0),
              (unsigned long)dsp_pf,   (unsigned long)(total_pf ? 100*dsp_pf/total_pf : 0),
              (unsigned long)echo_pf,  (unsigned long)(dsp_pf ? 100*echo_pf/dsp_pf : 0),
              (unsigned long)nonecho_pf,(unsigned long)(dsp_pf ? 100*nonecho_pf/dsp_pf : 0),
              (unsigned long)audio_pf, (unsigned long)(total_pf ? 100*audio_pf/total_pf : 0),
              (unsigned long)total_pf,
              (unsigned long)((vd + rd) ? (100 * vd / (vd + rd)) : 0),
              (int)g_spin.gate_on,
              (int)g_rc_active,
              (unsigned long)s_rc_diag_crc, (unsigned long)s_rc_diag_exp);
      fclose(df);
    }
  }
#endif /* SNES_LOAD_DIAG */

  if (load_state) {
    odroid_system_emu_load_state(save_slot);
  } else {
    lcd_clear_buffers();
  }
  memset(snes_frame, 0, sizeof(snes_frame));   /* borders start black and stay black */

#ifdef SNES_DEVICE_PROFILE
  /* Arm AFTER snes_init()/load_state so the AHB pre-flight sees the Apu's
   * 66 KB already taken, and after the last boot-time SD write. */
  snes_profile_init(SNES_AUDIO_RATE, snes_audio_samples);
  uint32_t prof_wall_prev = snes_prof_wall_now();
  uint32_t prof_dma_prev  = dma_counter;
#endif

#ifdef SNES_PERF
  snes_perf_init();
#endif
#ifdef HOST_BUILD
  rec_host_start();
#endif
  while (1) {
#ifdef SNES_DEVICE_PROFILE
    /* ONE DWT base per iteration. Every SNES_PROF_MARK below is a CUMULATIVE
     * read from it and nothing in the loop may re-clear CYCCNT — a nested
     * clear would silently reset the frame's zero point, so
     * snes_profile_record() checks the marks are monotonic and fails the whole
     * run if they are not. */
    uint32_t prof_base       = common_emu_get_dwt_cycles();
    uint32_t prof_apu_in_emu = 0;
    uint32_t prof_wall_pace  = 0;
    uint32_t prof_dma_before = 0;
    uint32_t prof_wfi        = 0;
#endif
#ifdef SNES_PERF
    sp_last = snes_perf_now();
    sp_frame_busy = 0;
#endif
    wdog_refresh();

    bool half_lock = snes_half_lock_active();
    bool present_slot = true;
    if (half_lock) {
      present_slot = (snes_half_ctr & 1) == 0;
      snes_half_ctr++;
    }
    snes_apply_frame_time();

    bool drawFrame;
    if (half_lock && !present_slot) {
      /* Skip frame_loop so the HUD counts presented frames (~30/25), not
       * emulated ones. CPU/APU still run below. */
      drawFrame = false;
    } else {
      if (half_lock)
        common_emu_state.skip_frames = 0;
      drawFrame = common_emu_frame_loop();
      if (half_lock)
        drawFrame = true;
    }
    SNES_PROF_MARK(SNES_PROF_M_FRAMECTL);
#ifdef SNES_PERF
      SP_MARK(SP_FRAMECTL);
#endif

    odroid_input_read_gamepad(&joystick);
    common_emu_input_loop(&joystick, options, &blit);
    common_emu_input_loop_handle_turbo(&joystick);

    uint16_t pad = rec_pad(read_snes_pad(&joystick));
    if (snes_pad_port) {
      snes->input1->currentState = 0;
      snes->input2->currentState = pad;
    } else {
      snes->input1->currentState = pad;
      snes->input2->currentState = 0;
    }
    SNES_PROF_MARK(SNES_PROF_M_INPUT);
#ifdef SNES_PERF
      SP_MARK(SP_INPUT);
#endif

    snes->disableRender = false;
    g_ppu_skip_render = !drawFrame;
    render_frame_into_active_buffer();
    SNES_PROF_MARK(SNES_PROF_M_RENDER_ARM);
#ifdef SNES_PERF
      SP_MARK(SP_FRAMECTL);
#endif

#ifdef SNES_PERF
    uint32_t sp_apu0 = snes_perf_apu;
#endif
    run_frame_events(snes);
    SNES_PROF_MARK(SNES_PROF_M_EMU);
#ifdef SNES_PERF
    SP_MARK(SP_EMU);
    sp_apu_emu += snes_perf_apu - sp_apu0;
#endif
#ifdef SNES_DEVICE_PROFILE
    /* Split the Ledger B APU accumulator at the emu/pcm boundary: core_rem is
     * emu_outer minus the APU work that happened INSIDE emu_outer, and the pcm
     * top-up further down must not be subtracted from it as well. */
    prof_apu_in_emu = snes_prof_b_apu_cyc;
#endif

    if (drawFrame)
      present_frame();
    SNES_PROF_MARK(SNES_PROF_M_PRESENT_KICK);
#ifdef SNES_PERF
      SP_MARK(SP_KICK);
#endif

    if (!audio_started) {
      /* start_playing clears pcm_owns — attach/enable must come after. */
      audio_start_playing(snes_audio_samples);
      pcm_attach(snes_pcm_ring, (int)SNES_PCM_RING, &snes_pcm_head, &snes_pcm_tail);
      pcm_audio_set((int)common_emu_sound_get_volume(), 1);
      audio_started = 1;
      wdog_refresh();
    }
#ifdef SNES_PERF
    uint32_t sp_apu1 = snes_perf_apu;
#endif
    snes_pcm_submit();
    pcm_audio_enable(1);
    SNES_PROF_MARK(SNES_PROF_M_PCM);
#ifdef SNES_PERF
    SP_MARK(SP_PCM);
    sp_apu_pcm += snes_perf_apu - sp_apu1;
#endif

    if (drawFrame) {
      present_frame_wait();
      SNES_PROF_MARK(SNES_PROF_M_PRESENT_TAIL);
#ifdef SNES_PERF
      SP_MARK(SP_WAIT);
#endif
      common_ingame_overlay();
      SNES_PROF_MARK(SNES_PROF_M_OVERLAY);
#ifdef SNES_PERF
      SP_MARK(SP_OVERLAY);
#endif
      lcd_swap();
      SNES_PROF_MARK(SNES_PROF_M_SWAP);
#ifdef SNES_PERF
      SP_MARK(SP_SWAP);
#endif
    }
#ifdef SNES_DEVICE_PROFILE
    else {
      /* Skipped frame: none of the three drawn-only phases ran. Collapse their
       * marks onto the pcm mark so each delta is exactly 0 -- leaving them at
       * last frame's values would make the deltas garbage AND break the
       * monotonicity gate. */
      snes_prof_mark[SNES_PROF_M_PRESENT_TAIL] =
      snes_prof_mark[SNES_PROF_M_OVERLAY]      =
      snes_prof_mark[SNES_PROF_M_SWAP]         = snes_prof_mark[SNES_PROF_M_PCM];
    }
    uint32_t prof_pace_w0 = snes_prof_wall_now();
#endif

    /* Pace the loop by the audio DMA UNCONDITIONALLY (WS pattern,
     * main_wswan.c:348-366). common_emu_sound_sync skips this wait when
     * skip_frames>0, which let heavy SNES games run ahead of real time and
     * play audio at fast-forward speed (SMW 55fps = 92% speed, SM 31fps =
     * half speed). Waiting for one DMA tick every frame caps the emulator at
     * real time so the tempo is correct; on frames that genuinely overran,
     * the DMA has already advanced so this passes through with no delay. */
    /* Pace only once SAI is running — otherwise dma_counter never moves and
     * the 100k-iter guard just burns time (and can still trip WWDG under load). */
    if (odroid_system_get_app()->speedupEnabled == SPEEDUP_1x) {
        static uint32_t snes_last_dma = 0;
        if (snes_last_dma == 0) snes_last_dma = dma_counter;
#ifdef SNES_PERF
        if (dma_counter != snes_last_dma) sp_late++;   /* the audio clock is already ahead */
        uint32_t sp_w0 = HAL_GetTick();
#endif
#ifdef SNES_DEVICE_PROFILE
        /* How many audio periods had ALREADY elapsed when we got here. 0 means
         * we arrived before the deadline and are about to wait; >=1 means the
         * deadline had already passed, the wait below falls straight through,
         * and LLE never recovers that period. Reading a near-zero wait as
         * "we overran" without this number is the mistake the review flagged. */
        prof_dma_before = dma_counter - snes_last_dma;
#endif
        /* Feed the watchdog and give up eventually. This wait blocks until the
         * audio DMA advances, and it had neither guard: if dma_counter stops
         * moving the loop spins forever, WWDG fires, and a watchdog reset
         * leaves no BSOD and no log -- the device simply drops out.
         *
         * That is what killed SMW after closing the pause menu. The comment
         * above is the reason it was SMW-only: a frame that overran finds the
         * DMA already advanced and passes straight through, so Zelda (35 fps)
         * and Super Metroid (30 fps) never actually wait here. Only SMW with
         * the audio HLE is fast enough to arrive before the DMA ticks -- and
         * if the menu left the DMA stopped, that arrival never ends. Turning
         * the HLE off "fixed" it by making the emulator too slow to reach the
         * wait, which is why every other explanation fit some of the evidence
         * and none of it fit all.
         *
         * The bound is generous (~100 ms, several audio periods): a frame that
         * legitimately waits is far under it, so pacing is unchanged. */
        {
            uint32_t spin_guard = 0;
            while (dma_counter == snes_last_dma && spin_guard < 100000u) {
                wdog_refresh();
                cpumon_sleep();
                spin_guard++;
            }
#ifdef SNES_DEVICE_PROFILE
            prof_wfi = spin_guard;   /* __WFI() round trips actually executed */
#endif
        }
#ifdef SNES_PERF
        sp_pace_ms += HAL_GetTick() - sp_w0;
#endif
        uint32_t elapsed = dma_counter - snes_last_dma;
        snes_last_dma = dma_counter;
        /* LLE: the SAI ISR fills every DMA half from the pcm ring, so a
         * slow frame no longer leaves one half playing stale samples.
         * Catch-up only for HLE — wire_frame_audio does not advance the
         * SPC700, extra batches stay tempo-correct. */
#ifdef SNES_SMW_HLE_PRODUCT
        if (g_wire_on) {
            /* Catch-up: replay the audio batches the DMA advanced past on a slow
             * frame. But `elapsed` is unbounded -- after the pause menu it counts
             * every audio period spent in the menu, so this tried to regenerate
             * seconds of audio at once, and the inner DMA wait below had no
             * watchdog feed or ceiling. That is the real SMW-menu death (Codex
             * adversarial review of 15dd53c8, which had guarded the wrong wait).
             *
             * Backlog past a couple of frames is not worth replaying -- the audio
             * for time spent paused is gone regardless -- so cap it and resync
             * rather than grind through it. */
            if (elapsed > 4) {
                snes_last_dma = dma_counter;   /* drop the backlog, don't replay it */
                elapsed = 0;
            }
            while (elapsed > 1) {
                snes_pcm_submit();
                uint32_t guard = 0;
                while (dma_counter == snes_last_dma && guard < 20000u) {
                    wdog_refresh();
                    cpumon_sleep();
                    guard++;
                }
                snes_last_dma = dma_counter;
                elapsed--;
            }
        }
#endif
    }
#ifdef SNES_DEVICE_PROFILE
    prof_wall_pace = snes_prof_wall_now() - prof_pace_w0;
    SNES_PROF_MARK(SNES_PROF_M_PACING);
    {
      /* wall_frame is measured end-of-iteration to end-of-iteration, so it
       * covers the WHOLE period including the previous snes_profile_record()
       * call. Measuring it from the top of this iteration instead would leave
       * the recorder's own cost outside every frame, and the sum would then
       * disagree with the audio-DMA reference for a reason that has nothing to
       * do with the emulator -- i.e. it would break the wall_vs_dma gate that
       * exists to catch real problems. */
      uint32_t wall_now = snes_prof_wall_now();
      uint32_t dma_now  = dma_counter;
      snes_profile_record(drawFrame, prof_base, prof_apu_in_emu,
                          wall_now - prof_wall_prev, prof_wall_pace,
                          prof_dma_before, dma_now - prof_dma_prev, prof_wfi);
      prof_wall_prev = wall_now;
      prof_dma_prev  = dma_now;
    }
#endif
#ifdef SNES_PERF
    sp_frames++;
    if (drawFrame) sp_shown++;
    if (sp_frame_busy > sp_frame_max) sp_frame_max = sp_frame_busy;
    {
      uint32_t st = snes_stretch_step_q16();
      sp_step_sum += st;
      if (st < sp_step_min) sp_step_min = st;
    }
    if (sp_frames >= SNES_PERF_FRAMES) snes_perf_report();
#endif
    rec_frame_done();
#ifdef HOST_BUILD
    {   /* SNES_FRAMES=N: exit normally after N emulated frames (closes a recording) */
      static long host_frames_left = -2;
      if (host_frames_left == -2) { const char *e = getenv("SNES_FRAMES"); host_frames_left = e ? atol(e) : -1; }
      if (host_frames_left > 0 && --host_frames_left == 0) exit(0);
    }
#endif
  }
}
