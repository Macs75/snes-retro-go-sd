/* host_platform API for the QEMU (mps2-an500) build: no window, no audio,
 * no input -- a replay supplies the pad. Time is a counter so nothing waits. */
#include <stdint.h>
#include <string.h>
#include "host_platform.h"
static uint32_t s_ms;
int host_platform_init(const char *title, int scale) { (void)title; (void)scale; return 0; }
void host_platform_shutdown(void) {}
int host_platform_poll(host_pad_t *pad) { memset(pad, 0, sizeof *pad); return 1; }
void host_platform_present_rgb565(const uint16_t *fb, int width, int height) { (void)fb; (void)width; (void)height; }
void host_platform_delay_ms(uint32_t ms) { s_ms += ms; }
uint32_t host_platform_ticks_ms(void) { return s_ms += 1; }
int host_platform_audio_start(int rate, int half_len) { (void)rate; (void)half_len; return 0; }
void host_platform_audio_queue(const int16_t *mono, int samples) { (void)mono; (void)samples; }
void host_platform_audio_stop(void) {}
int host_platform_audio_queued_samples(void) { return 0; }
