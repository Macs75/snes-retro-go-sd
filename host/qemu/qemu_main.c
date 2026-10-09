/* QEMU (mps2-an500) entry: the same app_main_snes() as the host and the device,
 * with the device's Thumb-2 65816/SPC700 engines. argv (semihosting): ROM [frames]. */
#include <stdio.h>
#include <stdlib.h>
#include "host_compat.h"
#include "host_platform.h"
extern void app_main_snes(uint8_t load_state, uint8_t start_paused, int8_t save_slot);
int main(int argc, char **argv) {
  const char *rom = argc > 1 ? argv[1] : "cv.sfc";
  setenv("SNES_REPLAY", "1", 1);
  setenv("SNES_FRAMES", argc > 2 ? argv[2] : "1210", 1);
  host_platform_init("qemu", 1);
  gw_core_bridge_init();
  host_set_rom_path(rom);
  printf("qemu: ROM %s, Thumb-2 engines\n", rom);
  app_main_snes(0, 0, -1);
  return 0;
}
