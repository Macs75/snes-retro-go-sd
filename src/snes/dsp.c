
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "dsp.h"
#include "apu.h"
#include "snes_gnw_alloc.h"

#define MY_CHANGES 1

#ifdef RIG_DSP_KEYON_PROBE
#include <stddef.h>
int g_keyon_probe_count = 0;
int g_keyon_probe_filter[4] = {0,0,0,0};
int g_keyon_probe_old_nonzero = 0;
int g_keyon_probe_older_nonzero = 0;
bool g_keyon_pending[8] = {0,0,0,0,0,0,0,0};
void dsp_keyonProbeReport(void) {
    printf("\n=== DSP KEY-ON FIRST-BLOCK FILTER PROBE ===\n");
    printf("Total key-ons observed: %d\n", g_keyon_probe_count);
    printf("First-block filter distribution:\n");
    for (int i = 0; i < 4; i++)
        printf("  filter=%d: %d (%.1f%%)\n", i, g_keyon_probe_filter[i],
               g_keyon_probe_count ? 100.0*g_keyon_probe_filter[i]/g_keyon_probe_count : 0.0);
    printf("old != 0 at key-on first decode: %d (%.1f%%)\n", g_keyon_probe_old_nonzero,
           g_keyon_probe_count ? 100.0*g_keyon_probe_old_nonzero/g_keyon_probe_count : 0.0);
    printf("older != 0 at key-on first decode: %d (%.1f%%)\n", g_keyon_probe_older_nonzero,
           g_keyon_probe_count ? 100.0*g_keyon_probe_older_nonzero/g_keyon_probe_count : 0.0);
    if (g_keyon_probe_filter[0] == g_keyon_probe_count)
        printf("VERDICT: ALL first blocks use filter=0 -> old/older freeze is SAFE\n");
    else
        printf("VERDICT: SOME first blocks use filter!=0 -> freeze would corrupt those\n");
    printf("=== END DSP KEY-ON PROBE ===\n\n");
}
#endif

static const int rateValues[32] = {
  0, 2048, 1536, 1280, 1024, 768, 640, 512,
  384, 320, 256, 192, 160, 128, 96, 80,
  64, 48, 40, 32, 24, 20, 16, 12,
  10, 8, 6, 5, 4, 3, 2, 1
};

static const uint16_t gaussValues[512] = {
  0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
  0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x002, 0x002, 0x002, 0x002, 0x002,
  0x002, 0x002, 0x003, 0x003, 0x003, 0x003, 0x003, 0x004, 0x004, 0x004, 0x004, 0x004, 0x005, 0x005, 0x005, 0x005,
  0x006, 0x006, 0x006, 0x006, 0x007, 0x007, 0x007, 0x008, 0x008, 0x008, 0x009, 0x009, 0x009, 0x00A, 0x00A, 0x00A,
  0x00B, 0x00B, 0x00B, 0x00C, 0x00C, 0x00D, 0x00D, 0x00E, 0x00E, 0x00F, 0x00F, 0x00F, 0x010, 0x010, 0x011, 0x011,
  0x012, 0x013, 0x013, 0x014, 0x014, 0x015, 0x015, 0x016, 0x017, 0x017, 0x018, 0x018, 0x019, 0x01A, 0x01B, 0x01B,
  0x01C, 0x01D, 0x01D, 0x01E, 0x01F, 0x020, 0x020, 0x021, 0x022, 0x023, 0x024, 0x024, 0x025, 0x026, 0x027, 0x028,
  0x029, 0x02A, 0x02B, 0x02C, 0x02D, 0x02E, 0x02F, 0x030, 0x031, 0x032, 0x033, 0x034, 0x035, 0x036, 0x037, 0x038,
  0x03A, 0x03B, 0x03C, 0x03D, 0x03E, 0x040, 0x041, 0x042, 0x043, 0x045, 0x046, 0x047, 0x049, 0x04A, 0x04C, 0x04D,
  0x04E, 0x050, 0x051, 0x053, 0x054, 0x056, 0x057, 0x059, 0x05A, 0x05C, 0x05E, 0x05F, 0x061, 0x063, 0x064, 0x066,
  0x068, 0x06A, 0x06B, 0x06D, 0x06F, 0x071, 0x073, 0x075, 0x076, 0x078, 0x07A, 0x07C, 0x07E, 0x080, 0x082, 0x084,
  0x086, 0x089, 0x08B, 0x08D, 0x08F, 0x091, 0x093, 0x096, 0x098, 0x09A, 0x09C, 0x09F, 0x0A1, 0x0A3, 0x0A6, 0x0A8,
  0x0AB, 0x0AD, 0x0AF, 0x0B2, 0x0B4, 0x0B7, 0x0BA, 0x0BC, 0x0BF, 0x0C1, 0x0C4, 0x0C7, 0x0C9, 0x0CC, 0x0CF, 0x0D2,
  0x0D4, 0x0D7, 0x0DA, 0x0DD, 0x0E0, 0x0E3, 0x0E6, 0x0E9, 0x0EC, 0x0EF, 0x0F2, 0x0F5, 0x0F8, 0x0FB, 0x0FE, 0x101,
  0x104, 0x107, 0x10B, 0x10E, 0x111, 0x114, 0x118, 0x11B, 0x11E, 0x122, 0x125, 0x129, 0x12C, 0x130, 0x133, 0x137,
  0x13A, 0x13E, 0x141, 0x145, 0x148, 0x14C, 0x150, 0x153, 0x157, 0x15B, 0x15F, 0x162, 0x166, 0x16A, 0x16E, 0x172,
  0x176, 0x17A, 0x17D, 0x181, 0x185, 0x189, 0x18D, 0x191, 0x195, 0x19A, 0x19E, 0x1A2, 0x1A6, 0x1AA, 0x1AE, 0x1B2,
  0x1B7, 0x1BB, 0x1BF, 0x1C3, 0x1C8, 0x1CC, 0x1D0, 0x1D5, 0x1D9, 0x1DD, 0x1E2, 0x1E6, 0x1EB, 0x1EF, 0x1F3, 0x1F8,
  0x1FC, 0x201, 0x205, 0x20A, 0x20F, 0x213, 0x218, 0x21C, 0x221, 0x226, 0x22A, 0x22F, 0x233, 0x238, 0x23D, 0x241,
  0x246, 0x24B, 0x250, 0x254, 0x259, 0x25E, 0x263, 0x267, 0x26C, 0x271, 0x276, 0x27B, 0x280, 0x284, 0x289, 0x28E,
  0x293, 0x298, 0x29D, 0x2A2, 0x2A6, 0x2AB, 0x2B0, 0x2B5, 0x2BA, 0x2BF, 0x2C4, 0x2C9, 0x2CE, 0x2D3, 0x2D8, 0x2DC,
  0x2E1, 0x2E6, 0x2EB, 0x2F0, 0x2F5, 0x2FA, 0x2FF, 0x304, 0x309, 0x30E, 0x313, 0x318, 0x31D, 0x322, 0x326, 0x32B,
  0x330, 0x335, 0x33A, 0x33F, 0x344, 0x349, 0x34E, 0x353, 0x357, 0x35C, 0x361, 0x366, 0x36B, 0x370, 0x374, 0x379,
  0x37E, 0x383, 0x388, 0x38C, 0x391, 0x396, 0x39B, 0x39F, 0x3A4, 0x3A9, 0x3AD, 0x3B2, 0x3B7, 0x3BB, 0x3C0, 0x3C5,
  0x3C9, 0x3CE, 0x3D2, 0x3D7, 0x3DC, 0x3E0, 0x3E5, 0x3E9, 0x3ED, 0x3F2, 0x3F6, 0x3FB, 0x3FF, 0x403, 0x408, 0x40C,
  0x410, 0x415, 0x419, 0x41D, 0x421, 0x425, 0x42A, 0x42E, 0x432, 0x436, 0x43A, 0x43E, 0x442, 0x446, 0x44A, 0x44E,
  0x452, 0x455, 0x459, 0x45D, 0x461, 0x465, 0x468, 0x46C, 0x470, 0x473, 0x477, 0x47A, 0x47E, 0x481, 0x485, 0x488,
  0x48C, 0x48F, 0x492, 0x496, 0x499, 0x49C, 0x49F, 0x4A2, 0x4A6, 0x4A9, 0x4AC, 0x4AF, 0x4B2, 0x4B5, 0x4B7, 0x4BA,
  0x4BD, 0x4C0, 0x4C3, 0x4C5, 0x4C8, 0x4CB, 0x4CD, 0x4D0, 0x4D2, 0x4D5, 0x4D7, 0x4D9, 0x4DC, 0x4DE, 0x4E0, 0x4E3,
  0x4E5, 0x4E7, 0x4E9, 0x4EB, 0x4ED, 0x4EF, 0x4F1, 0x4F3, 0x4F5, 0x4F6, 0x4F8, 0x4FA, 0x4FB, 0x4FD, 0x4FF, 0x500,
  0x502, 0x503, 0x504, 0x506, 0x507, 0x508, 0x50A, 0x50B, 0x50C, 0x50D, 0x50E, 0x50F, 0x510, 0x511, 0x511, 0x512,
  0x513, 0x514, 0x514, 0x515, 0x516, 0x516, 0x517, 0x517, 0x517, 0x518, 0x518, 0x518, 0x518, 0x518, 0x519, 0x519
};

#ifdef SNES_DSP_MONO
static void dsp_cycleChannel(Dsp* dsp, int ch, bool needSample);
#else
static void dsp_cycleChannel(Dsp* dsp, int ch);
#endif
static void dsp_handleEcho(Dsp* dsp, int* outputL, int* outputR);
static void dsp_handleGain(Dsp* dsp, int ch);
static void dsp_decodeBrr(Dsp* dsp, int ch);
static void dsp_decodeBrrIdle(Dsp* dsp, int ch);
static int16_t dsp_getSample(Dsp* dsp, int ch, int sampleNum, int offset);
static void dsp_handleNoise(Dsp* dsp);

Dsp* dsp_init(uint8_t *ram) {
  Dsp* dsp = snes_zalloc(sizeof(Dsp));
  if (!dsp) return NULL;
  dsp->apu_ram = ram;
  return dsp;
}

void dsp_free(Dsp* dsp) {
  snes_zfree(dsp);
}

#ifndef SNES_DSP_IDLE_SKIP_VOICE
#define SNES_DSP_IDLE_SKIP_VOICE 0
#endif
#if SNES_DSP_IDLE_SKIP_VOICE
/* Idle voices leave the loop entirely.
 *
 * Census, 700 frames of ALttP: 2,031,589 of 2,990,400 voice-ticks are idle --
 * 68% -- and ZERO idle voices use pitch modulation. Hoisting the idle test out
 * of the call (SNES_DSP_IDLE_HOIST) removed the call frame and measured nothing,
 * because the work itself stayed: the voice is still visited, still loads gain
 * and adsrState to be told it is idle, and still adds to its pitch counter, 2,902
 * times a frame.
 *
 * All an idle voice does per tick is `pitchCounter += pitch` and `sampleOut = 0`.
 * It never reaches the overflow test, so it never decodes BRR, so over N ticks
 * the counter is exactly `pitchCounter + pitch * N` -- and `pitch` cannot move
 * while idle, because pitch modulation is what would move it and no idle voice
 * has it. So the voice can be dropped from the loop and reconciled when it wakes.
 *
 * The mask is maintained conservatively: a voice can only BECOME idle inside
 * dsp_cycleChannel, and can only LEAVE idle through a register write or a reset,
 * both of which clear the whole mask. Register writes are rare against 4,272
 * voice-ticks a frame, so clearing all eight costs nothing and cannot be wrong.
 *
 * Derived state, so it lives in statics rather than in Dsp -- a savestate is a
 * raw struct dump. dsp_saveload flushes the pending advances first, so what is
 * written is the same state the per-tick version would have written. */
static uint8_t g_dsp_idle_mask;
static uint32_t g_dsp_idle_n[8];
static inline void dsp_flushIdle(Dsp* dsp, int ch) {
  uint32_t n = g_dsp_idle_n[ch];
  if (n) {
    dsp->channel[ch].pitchCounter += (uint16_t)(dsp->channel[ch].pitch * n);
    g_dsp_idle_n[ch] = 0;
  }
}
static void dsp_flushIdleAll(Dsp* dsp) {
  for (int i = 0; i < 8; i++) dsp_flushIdle(dsp, i);
  g_dsp_idle_mask = 0;
}
#endif
void dsp_reset(Dsp* dsp) {
#if SNES_DSP_IDLE_SKIP_VOICE
  dsp_flushIdleAll(dsp);
#endif
  memset(dsp->ram, 0, sizeof(dsp->ram));
  dsp->ram[0x7c] = 0xff; // set ENDx
  for(int i = 0; i < 8; i++) {
    dsp->channel[i].pitch = 0;
    dsp->channel[i].pitchCounter = 0;
    dsp->channel[i].pitchModulation = false;
    memset(dsp->channel[i].decodeBuffer, 0, sizeof(dsp->channel[i].decodeBuffer));
    dsp->channel[i].srcn = 0;
    dsp->channel[i].decodeOffset = 0;
    dsp->channel[i].previousFlags = 0;
    dsp->channel[i].old = 0;
    dsp->channel[i].older = 0;
    dsp->channel[i].useNoise = false;
    memset(dsp->channel[i].adsrRates, 0, sizeof(dsp->channel[i].adsrRates));
    dsp->channel[i].rateCounter = 0;
    dsp->channel[i].adsrState = 0;
    dsp->channel[i].sustainLevel = 0;
    dsp->channel[i].useGain = false;
    dsp->channel[i].gainMode = 0;
    dsp->channel[i].directGain = false;
    dsp->channel[i].gainValue = 0;
    dsp->channel[i].gain = 0;
    dsp->channel[i].keyOn = false;
    dsp->channel[i].keyOff = false;
    dsp->channel[i].sampleOut = 0;
    dsp->channel[i].volumeL = 0;
    dsp->channel[i].volumeR = 0;
    dsp->channel[i].echoEnable = false;
  }
  dsp->dirPage = 0;
  dsp->evenCycle = false;
  dsp->mute = true;
  dsp->reset = true;
  dsp->masterVolumeL = 0;
  dsp->masterVolumeR = 0;
  dsp->noiseSample = -0x4000;
  dsp->noiseRate = 0;
  dsp->noiseCounter = 0;
  dsp->echoWrites = false;
  dsp->echoVolumeL = 0;
  dsp->echoVolumeR = 0;
  dsp->feedbackVolume = 0;
  dsp->echoBufferAdr = 0;
  dsp->echoDelay = 1;
  dsp->echoRemain = 1;
  dsp->echoBufferIndex = 0;
  dsp->firBufferIndex = 0;
  memset(dsp->firValues, 0, sizeof(dsp->firValues));
  memset(dsp->firBufferL, 0, sizeof(dsp->firBufferL));
  memset(dsp->firBufferR, 0, sizeof(dsp->firBufferR));
  memset(dsp->sampleBuffer, 0, sizeof(dsp->sampleBuffer));
  dsp->sampleOffset = 0;
  dsp->frameSamples = DSP_SAMPLES_NTSC;
}

void dsp_saveload(Dsp *dsp, SaveLoadFunc *func, void *ctx) {
#if SNES_DSP_IDLE_SKIP_VOICE
  dsp_flushIdleAll(dsp);
#endif
  func(ctx, &dsp->ram, sizeof(Dsp) - offsetof(Dsp, ram));
}

#ifndef SNES_ABLATE_DSP_VOICES
#define SNES_ABLATE_DSP_VOICES 0
#endif
#ifndef SNES_ABLATE_DSP_GAIN
#define SNES_ABLATE_DSP_GAIN 0
#endif
#ifndef SNES_ABLATE_DSP_INTERP
#define SNES_ABLATE_DSP_INTERP 0
#endif
#ifndef SNES_ABLATE_DSP_BRR
#define SNES_ABLATE_DSP_BRR 0
#endif
#ifndef SNES_ABLATE_DSP_ECHO
#define SNES_ABLATE_DSP_ECHO 0
#endif
#ifndef SNES_DSP_CENSUS
#define SNES_DSP_CENSUS 0
#endif
#ifndef SNES_DSP_IDLE_HOIST
#define SNES_DSP_IDLE_HOIST 0
#endif
#if SNES_DSP_CENSUS
uint32_t g_dsp_ticks, g_dsp_idle, g_dsp_active, g_dsp_pm, g_dsp_brr;
#endif
void dsp_cycle(Dsp* dsp) {
  int totalL = 0;
  int totalR = 0;
#if SNES_ABLATE_DSP_VOICES
  /* ABLATION, WRONG OUTPUT (silence). The eight voices -- BRR decode, Gaussian
   * interpolation, ADSR, pitch, echo -- do nothing, but dsp_cycle still runs and
   * still EMITS a sample every tick.
   *
   * That last part is why the coarser ablations were unmeasurable. Deleting the
   * APU stops the boot handshake and the ROM never starts (22.4 fps, no ROM
   * running). Deleting dsp_cycle outright keeps the SPC700 answering ports but
   * stops sample production, and the frame loop -- which paces on the audio
   * DMA -- spins in its 100,000-WFI guard instead: same 22.4 fps, same dead ROM.
   * Audio is load-bearing for pacing here, so the only measurable cut is one
   * that keeps producing samples. */
#endif
#ifdef SNES_DSP_MONO
  /* The Game & Watch consumes 266 mono samples per 60 Hz frame. Keep source
   * ticks 0,2,...530; the 32 kHz DSP state and echo delay line still advance
   * on every tick, but the discarded main output is never synthesized. */
  bool emitSample = dsp->sampleOffset < 532 && (dsp->sampleOffset & 1) == 0;
#endif
#if !SNES_ABLATE_DSP_VOICES
  for(int i = 0; i < 8; i++) {
#if SNES_DSP_IDLE_SKIP_VOICE
    if (g_dsp_idle_mask & (1u << i)) { g_dsp_idle_n[i]++; continue; }
    dsp_flushIdle(dsp, i);
#endif
#if SNES_DSP_IDLE_HOIST
    /* The idle test, moved from the callee to the caller.
     *
     * dsp_cycleChannel already opens with exactly this test and returns after
     * advancing the pitch counter -- but by then the call frame is built and, in
     * the mono build, `needSample` has been computed from three loads and two
     * branches for a voice that will not produce a sample. Census, 700 frames of
     * ALttP: 2,031,589 of 2,990,400 voice-ticks are idle -- 68% -- and ZERO of
     * them have pitch modulation, so the copied test needs no `ch > 0` arm and
     * the fold is exact.
     *
     * Identical code, one level up: same condition, same pitch advance, same
     * sampleOut. It removes a call and a needSample for two thirds of all
     * voice-ticks and changes nothing about what the DSP computes.
     *
     * MEASURED, AND IT IS NOTHING: 56.75 against a 57.00 baseline, three runs.
     * The rig confirms it removes 57,959 instructions a frame (-1.46%) with
     * STATEHASH and AUDIOHASH both bit-identical -- and that buys nothing. The
     * DSP voice loop transfers instructions to time at about 0.43 (deleting the
     * voices outright is -9.76% instructions for +2.37 fps), so 1.46% was worth
     * ~0.35 fps in theory and did not clear the noise floor. Left off. */
    {
      DspChannel *c_ = &dsp->channel[i];
      if (c_->gain == 0 && c_->adsrState == 4 && !dsp->reset &&
          !(i > 0 && c_->pitchModulation)) {
        c_->pitchCounter += c_->pitch;
        c_->sampleOut = 0;
        continue;
      }
    }
#endif
#ifdef SNES_DSP_MONO
    /* A discarded sample remains live if the next voice uses it for pitch
     * modulation or the full-rate echo path writes it back to ARAM. */
    bool needSample = emitSample ||
      (i < 7 && dsp->channel[i + 1].pitchModulation) ||
      (dsp->echoWrites && dsp->channel[i].echoEnable);
    dsp_cycleChannel(dsp, i, needSample);
    if (!emitSample)
      continue;
    int monoVolume = (int)dsp->channel[i].volumeL + dsp->channel[i].volumeR;
    totalL += (dsp->channel[i].sampleOut * monoVolume) >> 7;
    totalL = totalL < -0x8000 ? -0x8000 : (totalL > 0x7fff ? 0x7fff : totalL);
#else
    dsp_cycleChannel(dsp, i);
    totalL += (dsp->channel[i].sampleOut * dsp->channel[i].volumeL) >> 6;
    totalR += (dsp->channel[i].sampleOut * dsp->channel[i].volumeR) >> 6;
    totalL = totalL < -0x8000 ? -0x8000 : (totalL > 0x7fff ? 0x7fff : totalL); // clamp 16-bit
    totalR = totalR < -0x8000 ? -0x8000 : (totalR > 0x7fff ? 0x7fff : totalR); // clamp 16-bit
#endif
  }
#endif
#ifdef SNES_DSP_MONO
  if (emitSample) {
    int monoMaster = (int)dsp->masterVolumeL + dsp->masterVolumeR;
    totalL = (totalL * monoMaster) >> 8;
    totalL = totalL < -0x8000 ? -0x8000 : (totalL > 0x7fff ? 0x7fff : totalL);
    totalR = totalL;
  }
#else
  totalL = (totalL * dsp->masterVolumeL) >> 7;
  totalR = (totalR * dsp->masterVolumeR) >> 7;
  totalL = totalL < -0x8000 ? -0x8000 : (totalL > 0x7fff ? 0x7fff : totalL); // clamp 16-bit
  totalR = totalR < -0x8000 ? -0x8000 : (totalR > 0x7fff ? 0x7fff : totalR); // clamp 16-bit
#endif
  /* Echo remains full-rate: its FIR history, feedback and ARAM writes are
   * updated even on main-output ticks that SNES_DSP_MONO discards. */
#if !SNES_ABLATE_DSP_VOICES && !SNES_ABLATE_DSP_ECHO
  dsp_handleEcho(dsp, &totalL, &totalR);
#endif
  if(dsp->mute) {
    totalL = 0;
    totalR = 0;
  }
  dsp_handleNoise(dsp);
  // put it in the samplebuffer
  if (dsp->sampleOffset < dsp->frameSamples) {
#ifdef SNES_DSP_MONO
    if (emitSample)
      dsp->sampleBuffer[dsp->sampleOffset >> 1] =
        (int16_t)((totalL + totalR) / 2);
#else
    dsp->sampleBuffer[dsp->sampleOffset * 2] = totalL;
    dsp->sampleBuffer[dsp->sampleOffset * 2 + 1] = totalR;
#endif
    // prevent sampleOffset from going past the current frame (NTSC 534 / PAL 640)
    dsp->sampleOffset++;
  }
  dsp->evenCycle = !dsp->evenCycle;
}

static void dsp_handleEcho(Dsp* dsp, int* outputL, int* outputR) {
  // get value out of ram
  uint16_t adr = dsp->echoBufferAdr + dsp->echoBufferIndex * 4;
  dsp->firBufferL[dsp->firBufferIndex] = (
    dsp->apu_ram[adr] + (dsp->apu_ram[(adr + 1) & 0xffff] << 8)
  );
  dsp->firBufferL[dsp->firBufferIndex] >>= 1;
  dsp->firBufferR[dsp->firBufferIndex] = (
    dsp->apu_ram[(adr + 2) & 0xffff] + (dsp->apu_ram[(adr + 3) & 0xffff] << 8)
  );
  dsp->firBufferR[dsp->firBufferIndex] >>= 1;
  /* If echo cannot affect this output or ARAM, the FIR sum and eight-channel
   * feedback mix are dead work. Loading the current delay-line values and
   * advancing both indexes preserves the exact history for a later enable. */
  if (!dsp->echoWrites && dsp->echoVolumeL == 0 && dsp->echoVolumeR == 0)
    goto handle_indexes;
  // calculate FIR-sum
  int sumL = 0, sumR = 0;
  for(int i = 0; i < 8; i++) {
    sumL += (dsp->firBufferL[(dsp->firBufferIndex + i + 1) & 0x7] * dsp->firValues[i]) >> 6;
    sumR += (dsp->firBufferR[(dsp->firBufferIndex + i + 1) & 0x7] * dsp->firValues[i]) >> 6;
    if(i == 6) {
      // clip to 16-bit before last addition
      sumL = ((int16_t) (sumL & 0xffff)); // clip 16-bit
      sumR = ((int16_t) (sumR & 0xffff)); // clip 16-bit
    }
  }
  sumL = sumL < -0x8000 ? -0x8000 : (sumL > 0x7fff ? 0x7fff : sumL); // clamp 16-bit
  sumR = sumR < -0x8000 ? -0x8000 : (sumR > 0x7fff ? 0x7fff : sumR); // clamp 16-bit
  // modify output with sum
  int outL = *outputL + ((sumL * dsp->echoVolumeL) >> 7);
  int outR = *outputR + ((sumR * dsp->echoVolumeR) >> 7);
  *outputL = outL < -0x8000 ? -0x8000 : (outL > 0x7fff ? 0x7fff : outL); // clamp 16-bit
  *outputR = outR < -0x8000 ? -0x8000 : (outR > 0x7fff ? 0x7fff : outR); // clamp 16-bit
  // get echo input
  int inL = 0, inR = 0;
  for(int i = 0; i < 8; i++) {
    if(dsp->channel[i].echoEnable) {
      inL += (dsp->channel[i].sampleOut * dsp->channel[i].volumeL) >> 6;
      inR += (dsp->channel[i].sampleOut * dsp->channel[i].volumeR) >> 6;
      inL = inL < -0x8000 ? -0x8000 : (inL > 0x7fff ? 0x7fff : inL); // clamp 16-bit
      inR = inR < -0x8000 ? -0x8000 : (inR > 0x7fff ? 0x7fff : inR); // clamp 16-bit
    }
  }
  // write this to ram
  inL += (sumL * dsp->feedbackVolume) >> 7;
  inR += (sumR * dsp->feedbackVolume) >> 7;
  inL = inL < -0x8000 ? -0x8000 : (inL > 0x7fff ? 0x7fff : inL); // clamp 16-bit
  inR = inR < -0x8000 ? -0x8000 : (inR > 0x7fff ? 0x7fff : inR); // clamp 16-bit
  inL &= 0xfffe;
  inR &= 0xfffe;
  if(dsp->echoWrites) {
    dsp->apu_ram[adr] = inL & 0xff;
    dsp->apu_ram[(adr + 1) & 0xffff] = inL >> 8;
    dsp->apu_ram[(adr + 2) & 0xffff] = inR & 0xff;
    dsp->apu_ram[(adr + 3) & 0xffff] = inR >> 8;
  }
handle_indexes:
  // handle indexes
  dsp->firBufferIndex++;
  dsp->firBufferIndex &= 7;
  dsp->echoBufferIndex++;
  dsp->echoRemain--;
  if(dsp->echoRemain == 0) {
    dsp->echoRemain = dsp->echoDelay;
    dsp->echoBufferIndex = 0;
  }
}

#ifdef SNES_DSP_MONO
static void dsp_cycleChannel(Dsp* dsp, int ch, bool needSample) {
#else
static void dsp_cycleChannel(Dsp* dsp, int ch) {
#endif
  /* Fast path: truly idle voice (released + gain 0). Output is 0 regardless of
   * decode state. Key-on (handled in dsp_writeReg for MY_CHANGES=1) resets
   * decodeOffset/previousFlags/decodeBuffer, so freezing BRR state while idle
   * is harmless. Pitch counter must still advance for correct sample timing
   * when key-on fires. sampleOut=0 keeps pitch-modulation for ch+1 correct. */
#if SNES_DSP_CENSUS
  g_dsp_ticks++;
  if (dsp->channel[ch].gain == 0 && dsp->channel[ch].adsrState == 4 && !dsp->reset) {
    g_dsp_idle++;
    if (ch > 0 && dsp->channel[ch].pitchModulation) g_dsp_pm++;
  } else g_dsp_active++;
#endif
  if (dsp->channel[ch].gain == 0 && dsp->channel[ch].adsrState == 4 && !dsp->reset) {
#if SNES_DSP_IDLE_SKIP_VOICE
    /* Only voices without pitch modulation may leave the loop -- theirs is the
     * pitch that cannot move while idle. The census says that is all of them
     * here; the test keeps it true when it is not. */
    if (!(ch > 0 && dsp->channel[ch].pitchModulation))
      g_dsp_idle_mask |= (uint8_t)(1u << ch);
#endif
    uint16_t pitch = dsp->channel[ch].pitch;
    if (ch > 0 && dsp->channel[ch].pitchModulation) {
      int factor = (dsp->channel[ch - 1].sampleOut >> 4) + 0x400;
      pitch = (pitch * factor) >> 10;
      if (pitch > 0x3fff) pitch = 0x3fff;
    }
    dsp->channel[ch].pitchCounter += pitch;
    dsp->channel[ch].sampleOut = 0;
    return;
  }

  // handle pitch counter
  uint16_t pitch = dsp->channel[ch].pitch;
  if(ch > 0 && dsp->channel[ch].pitchModulation) {
    int factor = (dsp->channel[ch - 1].sampleOut >> 4) + 0x400;
    pitch = (pitch * factor) >> 10;
    if(pitch > 0x3fff) pitch = 0x3fff;
  }
  int newCounter = dsp->channel[ch].pitchCounter + pitch;
  if(newCounter > 0xffff) {
#ifdef SNES_DSP_BRR_IDLE_SKIP
    if (dsp->channel[ch].gain == 0 && dsp->channel[ch].adsrState == 4)
      dsp_decodeBrrIdle(dsp, ch);
    else
#endif
    dsp_decodeBrr(dsp, ch);
#if SNES_DSP_CENSUS
    g_dsp_brr++;
#endif
  }
  dsp->channel[ch].pitchCounter = newCounter;
  int16_t sample = 0;
#ifdef SNES_DSP_MONO
  if (needSample) {
#endif
  if(dsp->channel[ch].useNoise) {
    sample = dsp->noiseSample;
  } else if(dsp->channel[ch].gain == 0 && dsp->channel[ch].adsrState == 4) {
    /* Released and silent: this sample is multiplied by gain (0) below, so the
     * Gaussian interpolation's result is zeroed anyway. Skip it — dsp_getSample is
     * a pure read (no ENDx/decode side effects, those already ran above), so the
     * output is bit-identical. Saves interpolation for every idle voice. */
    sample = 0;
  } else {
#if SNES_ABLATE_DSP_INTERP
    /* ABLATION, WRONG OUTPUT. The four-tap Gaussian interpolation alone. */
    sample = 0;
#else
    sample = dsp_getSample(dsp, ch, dsp->channel[ch].pitchCounter >> 12, (dsp->channel[ch].pitchCounter >> 4) & 0xff);
#endif
  }
#ifdef SNES_DSP_MONO
  }
#endif
#if !MY_CHANGES
  if(dsp->evenCycle) {
    // handle keyon/off (every other cycle)
    if(dsp->channel[ch].keyOff) {
      // go to release
      dsp->channel[ch].adsrState = 4;
    } else if(dsp->channel[ch].keyOn) {
      dsp->channel[ch].keyOn = false;
      // restart current sample
      dsp->channel[ch].previousFlags = 0;
      uint16_t samplePointer = dsp->dirPage + 4 * dsp->channel[ch].srcn;
      dsp->channel[ch].decodeOffset = dsp->apu_ram[samplePointer];
      dsp->channel[ch].decodeOffset |= dsp->apu_ram[(samplePointer + 1) & 0xffff] << 8;
      memset(dsp->channel[ch].decodeBuffer, 0, sizeof(dsp->channel[ch].decodeBuffer));
      dsp->channel[ch].gain = 0;
      dsp->channel[ch].adsrState = dsp->channel[ch].useGain ? 3 : 0;
    }
  }
#endif
  // handle reset
  if(dsp->reset) {
    dsp->channel[ch].adsrState = 4;
    dsp->channel[ch].gain = 0;
  }
  // handle envelope/adsr
  bool doingDirectGain = dsp->channel[ch].adsrState != 4 && dsp->channel[ch].useGain && dsp->channel[ch].directGain;
  uint16_t rate = dsp->channel[ch].adsrState == 4 ? 0 : dsp->channel[ch].adsrRates[dsp->channel[ch].adsrState];
  if(dsp->channel[ch].adsrState != 4 && !doingDirectGain && rate != 0) {
    dsp->channel[ch].rateCounter++;
  }
  if(dsp->channel[ch].adsrState == 4 || (!doingDirectGain && dsp->channel[ch].rateCounter >= rate && rate != 0)) {
    if(dsp->channel[ch].adsrState != 4) dsp->channel[ch].rateCounter = 0;
    dsp_handleGain(dsp, ch);
  }
  if(doingDirectGain) dsp->channel[ch].gain = dsp->channel[ch].gainValue;
  // set outputs
  dsp->ram[(ch << 4) | 8] = dsp->channel[ch].gain >> 4;
#ifdef SNES_DSP_MONO
  if (needSample) {
#endif
  sample = (sample * dsp->channel[ch].gain) >> 11;
  dsp->ram[(ch << 4) | 9] = sample >> 7;
  dsp->channel[ch].sampleOut = sample;
#ifdef SNES_DSP_MONO
  }
#endif
}

static void dsp_handleGain(Dsp* dsp, int ch) {
#if SNES_ABLATE_DSP_GAIN
  /* ABLATION, WRONG OUTPUT. The ADSR/gain envelope alone. It runs for every
   * voice on every one of the 534 DSP ticks a frame, and is the closed-form
   * candidate -- the SPC's three timers were folded exactly this way. */
  (void)dsp; (void)ch; return;
#endif
  switch(dsp->channel[ch].adsrState) {
    case 0: { // attack
      uint16_t rate = dsp->channel[ch].adsrRates[dsp->channel[ch].adsrState];
      dsp->channel[ch].gain += rate == 1 ? 1024 : 32;
      if(dsp->channel[ch].gain >= 0x7e0) dsp->channel[ch].adsrState = 1;
      if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0x7ff;
      break;
    }
    case 1: { // decay
      dsp->channel[ch].gain -= ((dsp->channel[ch].gain - 1) >> 8) + 1;
      if(dsp->channel[ch].gain < dsp->channel[ch].sustainLevel) dsp->channel[ch].adsrState = 2;
      break;
    }
    case 2: { // sustain
      dsp->channel[ch].gain -= ((dsp->channel[ch].gain - 1) >> 8) + 1;
      break;
    }
    case 3: { // gain
      switch(dsp->channel[ch].gainMode) {
        case 0: { // linear decrease
          dsp->channel[ch].gain -= 32;
          // decreasing below 0 will underflow to above 0x7ff
          if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0;
          break;
        }
        case 1: { // exponential decrease
          dsp->channel[ch].gain -= ((dsp->channel[ch].gain - 1) >> 8) + 1;
          break;
        }
        case 2: { // linear increase
          dsp->channel[ch].gain += 32;
          if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0x7ff;
          break;
        }
        case 3: { // bent increase
          dsp->channel[ch].gain += dsp->channel[ch].gain < 0x600 ? 32 : 8;
          if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0x7ff;
          break;
        }
      }
      break;
    }
    case 4: { // release
      dsp->channel[ch].gain -= 8;
      // decreasing below 0 will underflow to above 0x7ff
      if(dsp->channel[ch].gain > 0x7ff) dsp->channel[ch].gain = 0;
      break;
    }
  }
}

static int16_t dsp_getSample(Dsp* dsp, int ch, int sampleNum, int offset) {
#if defined(SNES_LINEAR_INTERP) || defined(GNW_SNES_CORE)
  /* Reduced-accuracy: 2-point linear instead of the 4-tap Gaussian. One lerp vs
   * four table[512] lookups + four MACs — and no 1 KB LUT competing for the M7's
   * 16 KB D-cache. Loses the SNES's characteristic HF muffling (many prefer the
   * crisper linear). A toggle; the accurate path stays the default. */
  int16_t olds = dsp->channel[ch].decodeBuffer[sampleNum + 2];
  int16_t news = dsp->channel[ch].decodeBuffer[sampleNum + 3];
  return (int16_t)(olds + (((news - olds) * offset) >> 8));
#else
  int16_t news = dsp->channel[ch].decodeBuffer[sampleNum + 3];
  int16_t olds = dsp->channel[ch].decodeBuffer[sampleNum + 2];
  int16_t olders = dsp->channel[ch].decodeBuffer[sampleNum + 1];
  int16_t oldests = dsp->channel[ch].decodeBuffer[sampleNum];
  int out = (gaussValues[0xff - offset] * oldests) >> 10;
  out += (gaussValues[0x1ff - offset] * olders) >> 10;
  out += (gaussValues[0x100 + offset] * olds) >> 10;
  out = ((int16_t) (out & 0xffff)); // clip 16-bit
  out += (gaussValues[offset] * news) >> 10;
  out = out < -0x8000 ? -0x8000 : (out > 0x7fff ? 0x7fff : out); // clamp 16-bit
  return out >> 1;
#endif
}

#ifdef SNES_DSP_BRR_IDLE_SKIP
/* Lightweight BRR advance for idle voices (gain==0 && adsrState==4).
 * Reads header for previousFlags, advances decodeOffset past the 9-byte block,
 * handles loop/end flags — but skips the expensive 16-sample decode + filter.
 * old/older are FROZEN. Safe because probe shows first BRR block after every
 * key-on uses filter=0 (ALttP: 341/341 = 100%), so old/older are never read
 * before they'd be overwritten by a real decode. */
static void dsp_decodeBrrIdle(Dsp* dsp, int ch) {
  dsp->channel[ch].decodeBuffer[0] = dsp->channel[ch].decodeBuffer[16];
  dsp->channel[ch].decodeBuffer[1] = dsp->channel[ch].decodeBuffer[17];
  dsp->channel[ch].decodeBuffer[2] = dsp->channel[ch].decodeBuffer[18];
  if(dsp->channel[ch].previousFlags == 1 || dsp->channel[ch].previousFlags == 3) {
    uint16_t samplePointer = dsp->dirPage + 4 * dsp->channel[ch].srcn;
    dsp->channel[ch].decodeOffset = dsp->apu_ram[(samplePointer + 2) & 0xffff];
    dsp->channel[ch].decodeOffset |= dsp->apu_ram[(samplePointer + 3) & 0xffff] << 8;
    if(dsp->channel[ch].previousFlags == 1) {
      dsp->channel[ch].adsrState = 4;
      dsp->channel[ch].gain = 0;
    }
    dsp->ram[0x7c] |= 1 << ch;
  }
  uint8_t header = dsp->apu_ram[dsp->channel[ch].decodeOffset++];
  dsp->channel[ch].previousFlags = header & 0x3;
  dsp->channel[ch].decodeOffset += 8; /* skip 8 data bytes */
  /* old/older intentionally NOT updated */
}
#endif

static void dsp_decodeBrr(Dsp* dsp, int ch) {
#if SNES_ABLATE_DSP_BRR
  /* ABLATION, WRONG OUTPUT. The BRR block decode alone -- sixteen nibbles and
   * the two-tap filter. It fires whenever a voice's pitch counter overflows,
   * which for a voice playing near 32 kHz is every tick. */
  (void)dsp; (void)ch; return;
#endif
  // copy last 3 samples (16-18) to first 3 for interpolation
  dsp->channel[ch].decodeBuffer[0] = dsp->channel[ch].decodeBuffer[16];
  dsp->channel[ch].decodeBuffer[1] = dsp->channel[ch].decodeBuffer[17];
  dsp->channel[ch].decodeBuffer[2] = dsp->channel[ch].decodeBuffer[18];
  // handle flags from previous block
  if(dsp->channel[ch].previousFlags == 1 || dsp->channel[ch].previousFlags == 3) {
    // loop sample
    uint16_t samplePointer = dsp->dirPage + 4 * dsp->channel[ch].srcn;
    dsp->channel[ch].decodeOffset = dsp->apu_ram[(samplePointer + 2) & 0xffff];
    dsp->channel[ch].decodeOffset |= (dsp->apu_ram[(samplePointer + 3) & 0xffff]) << 8;
    if(dsp->channel[ch].previousFlags == 1) {
      // also release and clear gain
      dsp->channel[ch].adsrState = 4;
      dsp->channel[ch].gain = 0;
    }
    dsp->ram[0x7c] |= 1 << ch; // set ENDx
  }
  uint8_t header = dsp->apu_ram[dsp->channel[ch].decodeOffset++];
  int shift = header >> 4;
  int filter = (header & 0xc) >> 2;
#ifdef RIG_DSP_KEYON_PROBE
  if (g_keyon_pending[ch]) {
    g_keyon_pending[ch] = false;
    g_keyon_probe_count++;
    if (filter >= 0 && filter <= 3) g_keyon_probe_filter[filter]++;
    if (dsp->channel[ch].old != 0) g_keyon_probe_old_nonzero++;
    if (dsp->channel[ch].older != 0) g_keyon_probe_older_nonzero++;
  }
#endif
  dsp->channel[ch].previousFlags = header & 0x3;
  uint8_t curByte = 0;
  int old = dsp->channel[ch].old;
  int older = dsp->channel[ch].older;
  for(int i = 0; i < 16; i++) {
    int s = 0;
    if(i & 1) {
      s = curByte & 0xf;
    } else {
      curByte = dsp->apu_ram[dsp->channel[ch].decodeOffset++];
      s = curByte >> 4;
    }
    if(s > 7) s -= 16;
    if(shift <= 0xc) {
      s = (s << shift) >> 1;
    } else {
      s = (s >> 3) << 12;
    }
    switch(filter) {
      case 1: s += old + (-old >> 4); break;
      case 2: s += 2 * old + ((3 * -old) >> 5) - older + (older >> 4); break;
      case 3: s += 2 * old + ((13 * -old) >> 6) - older + ((3 * older) >> 4); break;
    }
    s = s < -0x8000 ? -0x8000 : (s > 0x7fff ? 0x7fff : s); // clamp 16-bit
    s = ((int16_t) ((s & 0x7fff) << 1)) >> 1; // clip 15-bit
    older = old;
    old = s;
    dsp->channel[ch].decodeBuffer[i + 3] = s;
  }
  dsp->channel[ch].older = older;
  dsp->channel[ch].old = old;
}

static void dsp_handleNoise(Dsp* dsp) {
  if(dsp->noiseRate != 0) {
    dsp->noiseCounter++;
  }
  if(dsp->noiseCounter >= dsp->noiseRate && dsp->noiseRate != 0) {
    int bit = (dsp->noiseSample & 1) ^ ((dsp->noiseSample >> 1) & 1);
    dsp->noiseSample = ((dsp->noiseSample >> 1) & 0x3fff) | (bit << 14);
    dsp->noiseSample = ((int16_t) ((dsp->noiseSample & 0x7fff) << 1)) >> 1;
    dsp->noiseCounter = 0;
  }
}

uint8_t dsp_read(Dsp* dsp, uint8_t adr) {
  return dsp->ram[adr];
}

void dsp_write(Dsp* dsp, uint8_t adr, uint8_t val) {
#if SNES_DSP_IDLE_SKIP_VOICE
  dsp_flushIdleAll(dsp);
#endif
  int ch = adr >> 4;
  switch(adr) {
    case 0x00: case 0x10: case 0x20: case 0x30: case 0x40: case 0x50: case 0x60: case 0x70: {
      dsp->channel[ch].volumeL = val;
      break;
    }
    case 0x01: case 0x11: case 0x21: case 0x31: case 0x41: case 0x51: case 0x61: case 0x71: {
      dsp->channel[ch].volumeR = val;
      break;
    }
    case 0x02: case 0x12: case 0x22: case 0x32: case 0x42: case 0x52: case 0x62: case 0x72: {
      dsp->channel[ch].pitch = (dsp->channel[ch].pitch & 0x3f00) | val;
      break;
    }
    case 0x03: case 0x13: case 0x23: case 0x33: case 0x43: case 0x53: case 0x63: case 0x73: {
      dsp->channel[ch].pitch = ((dsp->channel[ch].pitch & 0x00ff) | (val << 8)) & 0x3fff;
      break;
    }
    case 0x04: case 0x14: case 0x24: case 0x34: case 0x44: case 0x54: case 0x64: case 0x74: {
      dsp->channel[ch].srcn = val;
      break;
    }
    case 0x05: case 0x15: case 0x25: case 0x35: case 0x45: case 0x55: case 0x65: case 0x75: {
      dsp->channel[ch].adsrRates[0] = rateValues[(val & 0xf) * 2 + 1];
      dsp->channel[ch].adsrRates[1] = rateValues[((val & 0x70) >> 4) * 2 + 16];
      dsp->channel[ch].useGain = (val & 0x80) == 0;
      break;
    }
    case 0x06: case 0x16: case 0x26: case 0x36: case 0x46: case 0x56: case 0x66: case 0x76: {
      dsp->channel[ch].adsrRates[2] = rateValues[val & 0x1f];
      dsp->channel[ch].sustainLevel = (((val & 0xe0) >> 5) + 1) * 0x100;
      break;
    }
    case 0x07: case 0x17: case 0x27: case 0x37: case 0x47: case 0x57: case 0x67: case 0x77: {
      dsp->channel[ch].directGain = (val & 0x80) == 0;
      if(val & 0x80) {
        dsp->channel[ch].gainMode = (val & 0x60) >> 5;
        dsp->channel[ch].adsrRates[3] = rateValues[val & 0x1f];
      } else {
        dsp->channel[ch].gainValue = (val & 0x7f) * 16;
      }
      break;
    }
    case 0x0c: {
      dsp->masterVolumeL = val;
      break;
    }
    case 0x1c: {
      dsp->masterVolumeR = val;
      break;
    }
    case 0x2c: {
      dsp->echoVolumeL = val;
      break;
    }
    case 0x3c: {
      dsp->echoVolumeR = val;
      break;
    }
    case 0x4c: {
      for(int ch = 0; ch < 8; ch++) {
        dsp->channel[ch].keyOn = val & (1 << ch);
#if MY_CHANGES
        if (dsp->channel[ch].keyOn) {
          dsp->channel[ch].keyOn = false;
          // restart current sample
          dsp->channel[ch].previousFlags = 0;
          uint16_t samplePointer = dsp->dirPage + 4 * dsp->channel[ch].srcn;
          dsp->channel[ch].decodeOffset = dsp->apu_ram[samplePointer];
          dsp->channel[ch].decodeOffset |= dsp->apu_ram[(samplePointer + 1) & 0xffff] << 8;
          memset(dsp->channel[ch].decodeBuffer, 0, sizeof(dsp->channel[ch].decodeBuffer));
          dsp->channel[ch].gain = 0;
          dsp->channel[ch].adsrState = dsp->channel[ch].useGain ? 3 : 0;
          dsp->ram[0x7c] &= ~(1 << ch); // key-on clears this voice's ENDx (dinkc64 b6f5839)
#ifdef RIG_DSP_KEYON_PROBE
          g_keyon_pending[ch] = true;
#endif
        }
#endif
      }
      break;
    }
    case 0x5c: {
      for(int ch = 0; ch < 8; ch++) {
        dsp->channel[ch].keyOff = val & (1 << ch);
#if MY_CHANGES
        if (dsp->channel[ch].keyOff) {
          // go to release
          dsp->channel[ch].adsrState = 4;
        }
#endif
      }
      break;
    }
    case 0x6c: {
      dsp->reset = val & 0x80;
      dsp->mute = val & 0x40;
      dsp->echoWrites = (val & 0x20) == 0;
      dsp->noiseRate = rateValues[val & 0x1f];
      break;
    }
    case 0x7c: {
      val = 0; // any write clears ENDx
      break;
    }
    case 0x0d: {
      dsp->feedbackVolume = val;
      break;
    }
    case 0x2d: {
      for(int i = 0; i < 8; i++) {
        dsp->channel[i].pitchModulation = val & (1 << i);
      }
      break;
    }
    case 0x3d: {
      for(int i = 0; i < 8; i++) {
        dsp->channel[i].useNoise = val & (1 << i);
      }
      break;
    }
    case 0x4d: {
      for(int i = 0; i < 8; i++) {
        dsp->channel[i].echoEnable = val & (1 << i);
      }
      break;
    }
    case 0x5d: {
      dsp->dirPage = val << 8;
      break;
    }
    case 0x6d: {
      dsp->echoBufferAdr = val << 8;
      break;
    }
    case 0x7d: {
      dsp->echoDelay = (val & 0xf) * 512; // 2048-byte steps, stereo sample is 4 bytes
      if(dsp->echoDelay == 0) dsp->echoDelay = 1;
      break;
    }
    case 0x0f: case 0x1f: case 0x2f: case 0x3f: case 0x4f: case 0x5f: case 0x6f: case 0x7f: {
      dsp->firValues[ch] = val;
      break;
    }
  }
  dsp->ram[adr] = val;
}

void dsp_getSamples(Dsp* dsp, int16_t* sampleData, int samplesPerFrame, int numChannels) {
  const int src = dsp->frameSamples ? dsp->frameSamples : DSP_SAMPLES_NTSC;
#ifdef SNES_DSP_MONO
  /* The device-rate mono samples were emitted directly by dsp_cycle(). */
  const int available = src / 2;
  if (numChannels == 1) {
    for (int i = 0; i < samplesPerFrame; i++)
      sampleData[i] = dsp->sampleBuffer[i < available ? i : available - 1];
  } else {
    for (int i = 0; i < samplesPerFrame; i++) {
      int16_t sample = dsp->sampleBuffer[i < available ? i : available - 1];
      sampleData[i * 2] = sample;
      sampleData[i * 2 + 1] = sample;
    }
  }
  dsp->sampleOffset = 0;
#else
  // resample from native DSP samples/frame (534 NTSC / 640 PAL) to wanted value
  double adder = (double)src / samplesPerFrame;
  double location = 0.0;
  if (numChannels == 1) {
    // The Game & Watch's SAI is mono. Downmix rather than writing two samples per
    // frame into a buffer that only has room for one — which is what a stereo
    // write into a mono buffer does, and it runs off the end of it.
    //
    // And average the samples we pass over instead of picking one and dropping the
    // rest. The DSP runs at 32 kHz and the SAI at 16, so point-sampling folds
    // everything above 8 kHz back down into the audible band — which is heard as a
    // buzz on bright material (the title theme) and on nothing else. Averaging the
    // source samples that fall in each output sample is a one-pole-cheap box filter
    // and it is the difference between "aliased" and "clean" here.
    for(int i = 0; i < samplesPerFrame; i++) {
      int start = (int)location;
      location += adder;
      int end = (int)location;
      if (end <= start) end = start + 1;
      if (end > src) end = src;

      int32_t sum = 0;
      for (int s = start; s < end; s++)
        sum += dsp->sampleBuffer[s * 2] + dsp->sampleBuffer[s * 2 + 1];

      sampleData[i] = (int16_t)(sum / (2 * (end - start)));
    }
  } else {
    for(int i = 0; i < samplesPerFrame; i++) {
      sampleData[i * 2] = dsp->sampleBuffer[((int) location) * 2];
      sampleData[i * 2 + 1] = dsp->sampleBuffer[((int) location) * 2 + 1];
      location += adder;
    }
  }
  dsp->sampleOffset = 0;
#endif
}
