/* Shared host platform API (SDL2/SDL3 backends). */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t values[16]; /* enough for ODROID_INPUT_MAX */
    uint8_t want_save;  /* edge: F1 pressed this poll */
    uint8_t want_load;  /* edge: F2 pressed this poll */
    uint8_t want_reset; /* edge: F5 / R pressed this poll */
} host_pad_t;

int host_platform_init(const char *title, int scale);
void host_platform_shutdown(void);
int host_platform_poll(host_pad_t *pad); /* 0 = quit requested */
uint16_t host_extra_snes_bits(void);     /* host-only direct keys: Ctrl=Select, Q=L, W=R */
void host_platform_present_rgb565(const uint16_t *fb, int width, int height);
void host_platform_delay_ms(uint32_t ms);
uint32_t host_platform_ticks_ms(void);

int host_platform_audio_start(int sample_rate, int half_len);
void host_platform_audio_queue(const int16_t *mono, int samples);
void host_platform_audio_stop(void);
int host_platform_audio_queued_samples(void);

#ifdef __cplusplus
}
#endif
