/* Adaptive audio stretcher for the SNES core.
 *
 * The problem it exists for
 * -------------------------
 * The core produces exactly SNES_AUDIO_SAMPLES (266 = 16000/60) samples per
 * EMULATED frame, and the audio DMA consumes one buffer every 16.625 ms of
 * REAL time. Those two only balance at 60 emulated fps. Mario Kart runs at
 * ~44, so the DMA drains 1.35 buffers per frame while the core refills one,
 * and main_snes.c's pacing block says what happens next in as many words:
 * "LLE accepts the underrun (stale audio for one DMA period on slow frames)".
 * That stale period IS the audible gap. The device profile shows it exactly:
 * deadline advance was 1 on 34 frames and 2 on 30, never 0.
 *
 * Why not just emulate more APU
 * -----------------------------
 * Because the same comment explains why that was rejected: extra apu_cycle()
 * calls advance the SPC700's timer relative to the 65816 and change the music
 * tempo. This module never touches the SPC700. It takes the samples the core
 * already produced and resamples them across the real time that actually
 * elapsed. The emulated machine is untouched; only the playback rate moves.
 *
 * What it may and may not do
 * --------------------------
 * It first shipped following the deficit all the way down — at 44 fps the
 * music played at 0.74x — on the argument that the game is ALREADY running at
 * 0.74x and the audio should agree with the video. The device disagreed. On
 * Zelda, Super Mario World and Mario Kart the same verdict came back in the
 * same word: flat. A transposition down a fifth is not the video's slowness
 * made audible, it is a second fault on top of it, and unlike a gap it never
 * stops.
 *
 * So the playback rate is clamped to ±1% (~17 cents, under the ~20-25
 * cents at which a detune registers). Inside that band the module still does
 * the job it was written for: it absorbs the jitter of a pull landing part of
 * a frame early or late, which is what turns into a gap. Outside it the ring
 * would run dry — and that is what PICOLA insertion prevents.
 *
 * PICOLA pitch-preserving insertion
 * ---------------------------------
 * When the push/pull deficit accumulates to at least one waveform period
 * (time_error >= pitch_est), the pull emits a crossfaded copy of the most
 * recent period from ring history WITHOUT advancing rd. This produces extra
 * output samples at the correct pitch — the waveform repeats exactly, so the
 * splice is seamless for periodic content and crossfaded for the rest.
 *
 * At 44 fps the deficit is ~97 samples/frame and the pitch period is ~64, so
 * ~1.5 insertions per frame keep the ring balanced. The listener hears the
 * game's own audio at the correct pitch, with a slight stutter on sustained
 * notes — which is what a slow scene sounds like, not a transposed one.
 *
 * A core at 68% speed has an fps problem; PICOLA makes the audio continuous
 * and in-tune instead of gap-ridden or flat.
 *
 * At full speed the loop converges to step == 1.0 and the pull is a straight
 * copy, so a 60 fps scene is unaffected.
 *
 * ISR-fed DMA (pcm_attach)
 * ------------------------
 * The SAI ISR must not jump into RAM_EMU (no emu_audio_register from a
 * core). Firmware pcm_fill() in gw_audio.c copies from a core-owned ring
 * registered with pcm_attach(). Main loop: stretch_push the emulated frame,
 * then stretch_pull into that ring. The ISR fills EVERY completed DMA half
 * from the ring — that is what kills the half-buffer resonance: at fps where
 * frame_time is an integer multiple of the DMA period (30 fps = 2 periods),
 * a main-loop emit always wrote the same half (dma_state landed on the same
 * selector every frame) and the other half played stale/short contents.
 *
 * Concurrency: stretcher push and pull both run in the main loop. The pcm
 * ring is SPSC (main writes head, ISR writes tail).
 */
#pragma once

#include <stdint.h>

/* Ring capacity in samples. 2048 = ~7.7 frames of emulated audio, which is
 * far more headroom than the +-1 buffer of jitter the pacing block produces;
 * the control loop holds the level near STRETCH_TARGET, not near full. */
#define SNES_STRETCH_RING 2048u

void     snes_stretch_reset(void);

/* Push one emulated frame's worth of samples (producer side). */
void     snes_stretch_push(const int16_t *src, uint16_t n);

/* Fill dst with n samples (consumer side). ALWAYS writes n samples — that is
 * the whole point; it never zero-pads a short frame the way the old path did. */
void     snes_stretch_pull(int16_t *dst, uint16_t n);

/* Fold what was pushed and pulled since the last call into the rate
 * measurement. Called every second emulated frame, at the end of a
 * drawn+undrawn pair, so a drawn frame that the undrawn one repays is on time. */
void     snes_stretch_commit(void);

/* Diagnostics for the device profiler / tests. */
uint16_t snes_stretch_fill(void);
uint32_t snes_stretch_step_q16(void);
uint32_t snes_stretch_underruns(void);

/* 0 = keep pitch (splice the deficit), 1 = gap-free (follow the rate).
 * Runtime, because only a listener can choose, and switching should not need a
 * rebuild. See the block comment in the .c for what each costs. */
extern uint8_t g_snes_audio_gapfree;
