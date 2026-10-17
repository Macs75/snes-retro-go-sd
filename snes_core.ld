/*
 * SNES (LakeSnes) core — RAM_EMU + ITCM for the Thumb-2 65816 engine.
 *
 * Match the monolithic firmware: 65816 Thumb-2 (~58KB) owns ITCM; SPC Thumb-2
 * (~14KB) stays in RAM_EMU — both do not fit in 64KB ITCM together.
 */

INCLUDE gnw_ram_emu.ld
INCLUDE gnw_itcm_core.ld

MEMORY
{
  RAM_EMU (xrw) : ORIGIN = __RAM_EMU_START__, LENGTH = __RAM_EMU_LENGTH__
  ITCM    (xrw) : ORIGIN = __ITCM_CORE_START__, LENGTH = __ITCM_CORE_LENGTH__
}

PHDRS
{
  text PT_LOAD FLAGS(5);
  data PT_LOAD FLAGS(6);
  itcm PT_LOAD FLAGS(5);
}

SECTIONS
{
  /* Segment 0 first so ITCM's AT>RAM_EMU LMA lands after RAM_EMU code/data
   * (not overlapping __RAM_EMU_START__ — same ordering as cores/pce). */
  .core_entry __RAM_EMU_START__ :
  {
    KEEP(*(.core_entry))
    KEEP(*(.gw_core_bridge_probe))
    /* The APU's hot code, PINNED: first, in a fixed order, on cache-line
     * boundaries. It runs from cached RAM_EMU (the 65816 engine owns ITCM),
     * and when it sat after all other code, any change anywhere in the core
     * moved the 14 KB SPC700 engine and changed what it evicts from the
     * 16 KB I-cache: the same SPC code measured ~25K cycles a frame apart
     * between two builds that differed only in dsp.c. */
    . = ALIGN(64);
    __APU_HOT_START__ = .;
    *(.itcm_snes_interp.thumb2_spc .itcm_snes_interp.thumb2_spc.*)
    . = ALIGN(32);
    *(.text.spc_runOpcode)
    . = ALIGN(32);
    *(.text.apu_run)
    . = ALIGN(32);
    *(.text.apu_cpuRead)
    . = ALIGN(32);
    *(.text.apu_cpuWrite)
    . = ALIGN(32);
    *(.text.snes_catchupApu)
    . = ALIGN(32);
    *(.text.spc_adcm .text.spc_sbcm .text.spc_doOpcode .text.spc_fallback)
    . = ALIGN(32);
    *(.text.apu_dsp_ticks)
    __APU_HOT_END__ = .;
    ASSERT(__APU_HOT_END__ <= __APU_HOT_START__ + 0x8000, "SNES: APU window (32K) overflows");

    /* The other hot groups sit in FIXED WINDOWS at fixed offsets from
     * __APU_HOT_START__, padded to size: a change that grows one group
     * moves nothing in the others, so their I-cache sets stay where they
     * were and a measured difference belongs to the change, not to
     * placement. The I-cache is 2-way, 8K a way: what decides the sets is an
     * address modulo 8K, so a window that must keep its sets moves in whole
     * 8K steps. Windows:
     *   +0K     APU  32K   SPC700 engine + C fallback, apu_*
     *   +35.5K  DSP  6.5K  dsp_cycle/read/write; 3.5K into the 8K way, so it
     *                      does not share sets with the start of the SPC700
     *                      engine (at +32K it did: DSP +5K, SPC700 +3.5K a frame)
     *   +42K    PPU  40.4K the line renderer only, ppu_runLine last
     *   +82.4K  BUS  9K    the 65816's bus: snes_read/write, B bus, cart,
     *                      input, DMA/HDMA, PPU registers. It runs on every
     *                      CPU memory access; when it shared the PPU window,
     *                      1.7K of renderer growth moved it and cost the CPU
     *                      +67K a frame. Same sets as when it measured best.
     *   +94K    CPU  24K   65816 C fallback (cpu_doOpcode + helpers), same sets
     *   +118K   everything else */
    . = __APU_HOT_START__ + 0x8E00;
    __DSP_HOT_START__ = .;
    *(.text.dsp_cycle)
    . = ALIGN(32);
    *(.text.dsp_read .text.dsp_write)
    __DSP_HOT_END__ = .;
    ASSERT(__DSP_HOT_END__ <= __DSP_HOT_START__ + 0x1A00, "SNES: DSP window (6.5K) overflows");

    . = __APU_HOT_START__ + 0xA800;
    __PPU_HOT_START__ = .;
    *(.text.PpuDrawBackgrounds)
    . = ALIGN(32);
    *(.text.PpuDrawBackground_4bpp*)
    . = ALIGN(32);
    *(.text.PpuDrawBackground_2bpp*)
    . = ALIGN(32);
    *(.text.PpuWindows_Calc* .text.ppu_getWindowState*)
    . = ALIGN(32);
    *(.text.PpuDrawSprites*)
    . = ALIGN(32);
    *(.text.ppu_getPixel*)
    . = ALIGN(32);
    *(.text.PpuDrawBackground_8bpp* .text.PpuDrawBackground_hires* .text.PpuDrawBackground_opt*)
    *(.text.PpuDrawBackground_mosaic* .text.ppu_handleOPT*)
    /* last: ppu_runLine inlines the line-start table updates and the sprite
     * evaluation, so it changes size with most PPU work -- here that moves
     * nothing else */
    . = ALIGN(32);
    *(.text.PpuBg4FullTiles)
    . = ALIGN(32);
    *(.text.PpuBg2FullTiles)
    . = ALIGN(32);
    *(.text.PpuBlendSpan*)
    . = ALIGN(32);
    *(.text.ppu_runLine)
    __PPU_HOT_END__ = .;
    ASSERT(__PPU_HOT_END__ <= __APU_HOT_START__ + 0x149C0, "SNES: PPU window (40.4K) overflows");

    . = __APU_HOT_START__ + 0x149C0;
    __BUS_HOT_START__ = .;
    *(.text.snes_handle_pos_stuff .text.snes_read .text.snes_write .text.snes_readBBus .text.snes_writeBBus)
    . = ALIGN(32);
    *(.text.cart_read .text.cart_write .text.input_read)
    . = ALIGN(32);
    *(.text.dma_doDma .text.dma_read .text.dma_write)
    /* HDMA (every line) and the PPU registers at fixed offsets, where they
     * measured: a DMA change no longer moves them (ld stops if the code
     * before grows past them) */
    . = __BUS_HOT_START__ + 0x15D4;
    *(.text.dma_doHdma .text.dma_startDma .text.dma_initHdma)
    . = __BUS_HOT_START__ + 0x18A0;
    *(.text.ppu_write .text.ppu_read)
    __BUS_HOT_END__ = .;
    ASSERT(__BUS_HOT_END__ <= __BUS_HOT_START__ + 0x2400, "SNES: BUS window (9K) overflows");
    /* A general DMA's transfer, once a transfer: in the padding before the CPU
     * window (2.6K), so it has a fixed place without growing the BUS window */
    . = ALIGN(32);
    *(.text.dma_runAtoB .text.dma_to_ppu)
    . = ALIGN(32);
    *(.text.ppu_dma_write .text.ppu_dma_ports)

    . = __APU_HOT_START__ + 0x17800;
    __CPU_C_START__ = .;
    *(.text.cpu_doOpcode)
    . = ALIGN(32);
    *(.text.cpu_*)
    __CPU_C_END__ = .;
    ASSERT(__CPU_C_END__ <= __CPU_C_START__ + 0x6000, "SNES: 65816 C window (24K) overflows");

    . = __APU_HOT_START__ + 0x1D800;
    . = ALIGN(4);
    /* The coprocessors' code is not part of the image: see the OVERLAY below. */
    *(EXCLUDE_FILE(*gsu_core.o *gsu_blocks.o *gsu_blocks_*.o *cx4_hle.o *sdd1.o *spc7110.o *dsp1_hle.o) .text*)
    *(EXCLUDE_FILE(*gsu_core.o *gsu_blocks.o *gsu_blocks_*.o *cx4_hle.o *sdd1.o *spc7110.o *dsp1_hle.o) .rodata*)
    *(.glue_7)
    *(.glue_7t)

    . = ALIGN(4);
    __CORE_INIT_ARRAY_START__ = .;
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    __CORE_INIT_ARRAY_END__ = .;
  } >RAM_EMU :text

  .data :
  {
    *(EXCLUDE_FILE(*gsu_core.o *gsu_blocks.o *gsu_blocks_*.o) .data*)
  } >RAM_EMU :data

  __CORE_CODE_END__ = .;

  .bss (NOLOAD) :
  {
    . = ALIGN(8);
    *(EXCLUDE_FILE(*gsu_core.o *gsu_blocks.o *gsu_blocks_*.o) .bss*)
    *(EXCLUDE_FILE(*gsu_core.o *gsu_blocks.o *gsu_blocks_*.o) COMMON)
    . = ALIGN(8);
  } >RAM_EMU :data

  __CORE_BSS_END__ = .;

  ASSERT(__CORE_BSS_END__ <= __RAM_EMU_END__,
         "Error: SNES core code+data+bss overflows RAM_EMU")

  .core_itcm __ITCM_CORE_START__ :
  {
    __CORE_ITCM_CODE_START__ = .;
    /* Fixed order and a FIXED ENGINE ADDRESS, like the RAM_EMU windows.
     * ITCM has no cache, yet where the 65816 engine sits measured +-50-60K
     * cycles a frame of CPU time: engine at 0x1E04 was the best of three
     * positions measured (0x1F74 and 0x0000 were both 50-60K worse,
     * Castlevania and Zelda alike). It now sits at 0x0E04, exactly 4 KB
     * lower: every address keeps its low 12 bits, so its branch targets
     * keep their alignment in the fetch words, and the 4 KB that the cold
     * main loop gave back (it moved to RAM) are free after the engine for
     * it to grow into. In front of it: the bus helpers, then the scheduler
     * (run_frame_events). If they outgrow their slot the link fails here
     * instead of silently moving the engine. */
    *cpu.o(.itcm_snes_interp.thumb2.bus)
    *dma.o(.itcm_snes_interp.thumb2.bus)
    */snes.o(.itcm_snes_interp.thumb2.bus)
    /* the scheduler at a fixed offset too: a change in the bus helpers in
     * front must not move it */
    __ITCM_BUS_END__ = .;
    ASSERT(__ITCM_BUS_END__ <= __CORE_ITCM_CODE_START__ + 0x564,
           "SNES: ITCM bus helpers outgrew their slot in front of the scheduler (0x564)");
    . = __CORE_ITCM_CODE_START__ + 0x564;
    *main_snes.o(.itcm_snes_interp.thumb2.bus)
    __ITCM_MAIN_END__ = .;
    /* a diagnostic build (PERF_CPU=1) may move the engine with
     * --defsym=__SNES_ITCM_ENGINE_AT=...; every other build keeps 0x0E04 */
    __ITCM_ENGINE_OFF = DEFINED(__SNES_ITCM_ENGINE_AT) ? __SNES_ITCM_ENGINE_AT : 0x0E04;
    ASSERT(__ITCM_MAIN_END__ <= __CORE_ITCM_CODE_START__ + __ITCM_ENGINE_OFF,
           "SNES: ITCM bus+scheduler outgrew their slot in front of the 65816 engine");
    . = __CORE_ITCM_CODE_START__ + __ITCM_ENGINE_OFF;
    __ITCM_ENGINE_START__ = .;
    *(.itcm_snes_interp.thumb2)
    *(.itcm_snes_interp.thumb2.*)
    *(.itcm_snes_interp*)
    . = ALIGN(4);
    __CORE_ITCM_CODE_END__ = .;
  } >ITCM AT>RAM_EMU :itcm

  __CORE_ITCM_BSS_END__ = __CORE_ITCM_CODE_END__;

  ASSERT(__CORE_ITCM_BSS_END__ <= __ITCM_CORE_START__ + __ITCM_CORE_LENGTH__,
         "Error: SNES core ITCM (65816 Thumb-2) overflows 64K ITCM")

  /* Coprocessor overlays: the code (and constant tables) of the Super FX
   * (gsu_core.o + gsu_blocks.o; each game's blocks are below), Cx4, S-DD1,
   * SPC7110 and DSP-1 is linked to
   * run at the start of the RAM_EMU pool (ram_start = __CORE_BSS_END__, where
   * the firmware seeds the bump) and NOT loaded with the core. pack appends
   * each section to snes.bin (tools/append_overlays.py); for a cart with one
   * of these chips the core's first pool allocation reserves the range and
   * copro_overlay_load() reads that chip's code in. Other games keep the whole
   * pool. The chips' small .data/.bss stay in the image (above). NOCROSSREFS:
   * no overlay may call into another. The LMAs are dummies outside every
   * region (the bytes are only ever taken with objcopy --only-section). */
  OVERLAY __CORE_BSS_END__ : NOCROSSREFS AT(0xD0000000)
  {
    .gsu_overlay
    {
      __GSU_OVL_START__ = .;
      *gsu_core.o(.text* .rodata* .data*)
      *gsu_blocks.o(.text* .rodata* .data*)
      . = ALIGN(4);
      __GSU_OVL_CODE_END__ = .;
    }
    .cx4_overlay
    {
      __CX4_OVL_START__ = .;
      *cx4_hle.o(.text* .rodata*)
      . = ALIGN(4);
      __CX4_OVL_END__ = .;
    }
    .sdd1_overlay
    {
      __SDD1_OVL_START__ = .;
      *sdd1.o(.text* .rodata*)
      . = ALIGN(4);
      __SDD1_OVL_END__ = .;
    }
    .spc7110_overlay
    {
      __SPC7110_OVL_START__ = .;
      *spc7110.o(.text* .rodata*)
      . = ALIGN(4);
      __SPC7110_OVL_END__ = .;
    }
    .dsp1_overlay
    {
      __DSP1_OVL_START__ = .;
      *dsp1_hle.o(.text* .rodata*)
      . = ALIGN(4);
      __DSP1_OVL_END__ = .;
    }
  }

  /* Right behind the code: without an address ld would put it at the RAM_EMU
   * location counter, which has also counted the ITCM segment's load image. */
  .gsu_overlay_bss ALIGN(__GSU_OVL_CODE_END__, 8) (NOLOAD) :
  {
    *gsu_core.o(.bss* COMMON)
    *gsu_blocks.o(.bss* COMMON)
    . = ALIGN(4);
    __GSU_OVL_END__ = .;
  }

  ASSERT(ADDR(.gsu_overlay_bss) >= __GSU_OVL_CODE_END__,
         "SNES: Super FX overlay bss overlaps its code")
  ASSERT(__GSU_OVL_END__ <= __RAM_EMU_END__,
         "SNES: Super FX overlay does not fit the RAM_EMU pool")

  /* Super FX block code, one overlay per game (gsu_blocks_<game>.c), all linked
   * right behind the GSU overlay's bss: the loader reads the GSU code and the
   * cart's game, if it has one, as one pool allocation. Each starts with its
   * header (.gsu_game_hdr, struct gsu_blk_game in gsu_blocks.h) and is appended
   * to snes.bin as a GSGM record (tools/append_overlays.py finds them by their
   * __GSUGAME_<game>_START__/_END__ symbols). A new game: one section here. */
  __GSU_GAME_OVL_START__ = ALIGN(__GSU_OVL_END__, 32);
  OVERLAY __GSU_GAME_OVL_START__ : NOCROSSREFS AT(0xD8000000)
  {
    .gsugame_yoshi
    {
      __GSUGAME_yoshi_START__ = .;
      KEEP(*gsu_blocks_yoshi.o(.gsu_game_hdr))
      *gsu_blocks_yoshi.o(.text* .rodata* .data* .bss* COMMON)
      . = ALIGN(4);
      __GSUGAME_yoshi_END__ = .;
    }
    .gsugame_starfox
    {
      __GSUGAME_starfox_START__ = .;
      KEEP(*gsu_blocks_starfox.o(.gsu_game_hdr))
      *gsu_blocks_starfox.o(.text* .rodata* .data* .bss* COMMON)
      . = ALIGN(4);
      __GSUGAME_starfox_END__ = .;
    }
  }
  ASSERT(__GSUGAME_yoshi_END__ <= __RAM_EMU_END__,
         "SNES: Yoshi's Island's GSU blocks do not fit the RAM_EMU pool")
  ASSERT(__GSUGAME_starfox_END__ <= __RAM_EMU_END__,
         "SNES: Star Fox's GSU blocks do not fit the RAM_EMU pool")

  __exidx_start = .;
  __exidx_end = .;

  /DISCARD/ :
  {
    *(.fini_array*)
    *(.preinit_array*)
    *(.ARM.exidx*)
    *(.ARM.extab*)
    *(.comment)
    *(.ARM.attributes)
  }
}
