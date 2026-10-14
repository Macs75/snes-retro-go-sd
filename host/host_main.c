/*
 * Desktop entry: init SDL, then jump into the same app_main_snes() as on device.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "host_compat.h"
#include "host_platform.h"

#ifndef HOST_SCALE
#define HOST_SCALE 2
#endif

extern void app_main_snes(uint8_t load_state, uint8_t start_paused, int8_t save_slot);

int main(int argc, char **argv)
{
    const char *title = "SNES LakeSnes (host)";
    const char *rom = getenv("HOST_ROM");
    int scale = HOST_SCALE;

    /* SNES_SCALE env sets the default window zoom; -scale/-zoom N overrides. */
    {
        const char *e = getenv("SNES_SCALE");
        if (e && e[0]) scale = atoi(e);
    }
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!a || !a[0]) continue;
        if ((!strcmp(a, "-scale") || !strcmp(a, "-zoom")) && i + 1 < argc) {
            scale = atoi(argv[++i]);
        } else if (!rom) {
            rom = a;  /* first non-flag arg is the ROM path */
        }
    }
    if (scale < 1) scale = 1;
    if (scale > 8) scale = 8;

    if (host_platform_init(title, scale) != 0)
        return 1;

    gw_core_bridge_init();
    if (rom)
        host_set_rom_path(rom);

    printf("host: Esc or close window to quit (window scale %d; -scale/-zoom N or SNES_SCALE=N)\n", scale);
    printf("host: Arrows=D-pad  Z=B  X=A  Enter=Start  Shift=Select  A/S=Y/X\n");
    printf("host: F2=save state  F4=load state  F5/R=reset  Esc=quit (writes .sram to ./host_saves/)\n");
    if (rom)
        printf("host: ROM %s\n", rom);
    else
        printf("host: no ROM — pass path or set HOST_ROM\n");

    app_main_snes(0, 0, -1);

    host_platform_shutdown();
    return 0;
}
