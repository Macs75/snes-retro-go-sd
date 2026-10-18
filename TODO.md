# TODO

## DMA2D for the 1:1 present (firmware ABI change)

The SNES framebuffer (`snes_frame`, `src/main_snes.c`) holds only the 256x240
picture, so the 1:1 present has to place it at (32, 8) in the 320x240 LCD
buffer. The firmware's DMA2D entry, `dma2d_m2m_rgb565_start(src, dst, width,
height)`, copies contiguous blocks only (no line offsets), so 1:1 is a CPU copy
today (rows + black borders, every presented frame). FULL and FIT are CPU
scalers anyway and are unaffected.

- Firmware: add an ABI entry with line offsets, e.g.
  `dma2d_m2m_rgb565_start_ex(src, dst, width, height, src_offset, dst_offset)`
  (DMA2D FGOR / OOR), plus a register-to-memory fill for the borders, and bump
  the ABI version.
- Core: in `present_frame()` (1:1), use it when the firmware offers it: the
  picture to `dst + 8 * 320 + 32` with output offset 64, the borders by fill,
  the CPU copy kept as the fallback; the async `present_frame_wait()` path
  (`SNES_PRESENT_DMA2D`) is still in place for it.
