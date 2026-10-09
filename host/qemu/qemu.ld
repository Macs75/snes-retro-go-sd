MEMORY { CODE (rx) : ORIGIN = 0x00000000, LENGTH = 4M
         RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 4M }
ENTRY(Reset_Handler)
SECTIONS {
  .text : { KEEP(*(.vectors)) *(.text* .itcm* .core_entry .gw_core_bridge_probe) *(.rodata*)
            . = ALIGN(4); __preinit_array_start = .; KEEP(*(.preinit_array*)) __preinit_array_end = .;
            __init_array_start = .; KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) __init_array_end = .;
            __fini_array_start = .; KEEP(*(.fini_array*)) __fini_array_end = .;
            *(.ARM.extab* .gnu.linkonce.armextab.*) } > CODE
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > CODE
  __data_load = LOADADDR(.data);
  .data : { __data_start = .; *(.data*) . = ALIGN(4); __data_end = .; } > RAM AT > CODE
  .bss (NOLOAD) : { __bss_start__ = .; *(.bss* COMMON) . = ALIGN(8); __bss_end__ = .; end = .; __end__ = .; } > RAM
  __stack_top = ORIGIN(RAM) + LENGTH(RAM);
  __HeapLimit = __stack_top - 0x10000;
}
