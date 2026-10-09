/* mps2-an500 reset: FPU on, .data copied, .bss cleared, semihosting I/O. */
#include <stdint.h>
#include <string.h>
extern uint32_t __data_load, __data_start, __data_end, __bss_start__, __bss_end__, __stack_top;
extern int main(int, char **);
extern void __libc_init_array(void);
extern void initialise_monitor_handles(void);
extern void exit(int);
void Reset_Handler(void);
void Default_Handler(void) { for (;;) __asm__ volatile ("bkpt 0x00"); }
__attribute__((section(".vectors"), used)) const void *vectors[16] = {
  &__stack_top, Reset_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
};
/* semihosting argv */
static char s_cmd[256]; static char *s_argv[8];
static int semi(int op, void *arg) { register int r0 __asm__("r0") = op; register void *r1 __asm__("r1") = arg;
  __asm__ volatile ("bkpt 0xab" : "+r"(r0) : "r"(r1) : "memory"); return r0; }
void Reset_Handler(void) {
  *(volatile uint32_t *)0xE000ED88u |= 0xFu << 20;   /* CPACR: CP10/CP11 full access */
  __asm__ volatile ("dsb; isb");
  memcpy(&__data_start, &__data_load, (char *)&__data_end - (char *)&__data_start);
  memset(&__bss_start__, 0, (char *)&__bss_end__ - (char *)&__bss_start__);
  initialise_monitor_handles();
  __libc_init_array();
  uint32_t blk[2] = { (uint32_t)s_cmd, sizeof s_cmd - 1 };
  int argc = 0;
  if (semi(0x15, blk) == 0) {           /* SYS_GET_CMDLINE */
    char *p = s_cmd;
    while (*p && argc < 7) { while (*p == ' ') *p++ = 0; if (!*p) break; s_argv[argc++] = p; while (*p && *p != ' ') p++; }
  }
  exit(main(argc, s_argv));
}
/* newlib/semihosting gaps: no directories, no crti/crtn */
int mkdir(const char *path, unsigned mode) { (void)path; (void)mode; return 0; }
void _init(void) {}
void _fini(void) {}
