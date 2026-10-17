#ifndef PPU_H
#define PPU_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Ppu Ppu;

#include "snes.h"

#ifdef TARGET_GNW
/* The device framebuffer is RGB565. This used to be defined in ppu.c *after* it
 * included this header, which meant every #ifdef PPU_RGB565 in the struct was
 * silently false while the ones in the code were true. */
#ifndef PPU_RGB565
#define PPU_RGB565 1
#endif
#endif

typedef struct BgLayer {
  uint16_t hScroll;
  uint16_t vScroll;
  bool tilemapWider;
  bool tilemapHigher;
  uint16_t tilemapAdr;
  uint16_t tileAdr;
  bool bigTiles;
  bool mosaicEnabled;
} BgLayer;

enum {
  kPpuXPixels = 256,
  kPpuExtraLeftRight = 0,
};

/* Frameskip. The launcher drops video frames when it cannot hold 60 Hz, but the
 * SNES renders line by line from inside the frame's timing loop, so there is no
 * outer draw call to skip. Set this and the line renderer does the timing and
 * the sprite evaluation (the game can read range/time-over) but not the pixels,
 * which is where the work is. */
extern bool g_ppu_skip_render;

#ifdef TARGET_GNW
/* Per-line hand-off. There is no room on the device for a 256x224 staging buffer,
 * so this PPU has always rendered straight into the LCD framebuffer at its native
 * size — which is exactly why the launcher's scaling options did nothing for this
 * core: there was nothing between the renderer and the screen to scale.
 *
 * Point renderBuffer at a one-line buffer (renderPitch 0, so every line lands in
 * the same place) and set this, and the port gets each finished line to place or
 * stretch as it likes. y is 1-based, as ppu_runLine counts. */
extern void (*g_ppu_line_cb)(unsigned y, const uint16_t *line);
#endif

typedef uint16_t PpuZbufType;

// ClearBackdrop() fills these through a *(uint64*) cast. On ARM that compiles to
// STRD, which faults unless the address is word-aligned — and PpuZbufType only
// asks for 2. Left to the compiler these land at offset 0x702 inside Ppu, which
// is 2 mod 4, and the device takes a UsageFault on the first rendered line. The
// alignment the cast assumes has to be stated, not hoped for.
typedef struct PpuPixelPrioBufs {
  // This holds the prio in the upper 8 bits and the color in the lower 8 bits.
  PpuZbufType data[kPpuXPixels];
} __attribute__((aligned(8))) PpuPixelPrioBufs;

enum {
  kPpuRenderFlags_NewRenderer = 1,
  // Render mode7 upsampled by 4x4
  kPpuRenderFlags_4x4Mode7 = 2,
  // Use 240 height instead of 224
  kPpuRenderFlags_Height240 = 4,
  // Disable sprite render limits
  kPpuRenderFlags_NoSpriteLimits = 8,
};



typedef struct Layer {
  bool mainScreenEnabled;
  bool subScreenEnabled;
  bool mainScreenWindowed;
  bool subScreenWindowed;
} Layer;

typedef struct WindowLayer {
  bool window1enabled;
  bool window2enabled;
  bool window1inversed;
  bool window2inversed;
  uint8_t maskLogic;
} WindowLayer;

struct Ppu {
  Snes* snes;
  // vram access
#ifdef TARGET_GNW
  /* 64 KB of VRAM would dominate this struct, and the struct has to live in the
   * RAM_EMU overlay pool that also holds the game's code. Point at ITC RAM
   * instead (see ppu_init) — same trick the zelda3 G&W port uses. */
  uint16_t *vram;
#else
  uint16_t vram[0x8000];
#endif
  uint16_t vramPointer;
  bool vramIncrementOnHigh;
  uint16_t vramIncrement;
  uint8_t vramRemapMode;
  uint16_t vramReadBuffer;
  // cgram access
  uint16_t cgram[0x100];
  uint8_t cgramPointer;
  bool cgramSecondWrite;
  uint8_t cgramBuffer;
  // oam access
  uint16_t oam[0x100];
  uint8_t highOam[0x20];
  uint8_t oamAdr;
  uint8_t oamAdrWritten;
  bool oamInHigh;
  bool oamInHighWritten;
  bool oamSecondWrite;
  uint8_t oamBuffer;
  // object/sprites
  bool objPriority;
  uint16_t objTileAdr1;
  uint16_t objTileAdr2;
  uint8_t objSize;
  uint8_t objPixelBufferXX[256]; // line buffers
  uint8_t objPriorityBufferXX[256];
  bool timeOver;
  bool rangeOver;
  bool objInterlace;
  // background layers
  BgLayer bgLayer[4];
  uint8_t scrollPrev;
  uint8_t scrollPrev2;
  uint8_t mosaicSize;
  uint8_t mosaicStartLine;
  // layers
  Layer layer[5];
  // mode 7
  int16_t m7matrix[8]; // a, b, c, d, x, y, h, v
  uint8_t m7prev;
  bool m7largeField;
  bool m7charFill;
  bool m7xFlip;
  bool m7yFlip;
  bool m7extBg;
  // mode 7 internal
  int32_t m7startX;
  int32_t m7startY;
  // windows
  WindowLayer windowLayer[6];
  uint8_t window1left;
  uint8_t window1right;
  uint8_t window2left;
  uint8_t window2right;
  // color math
  uint8_t clipMode;
  uint8_t preventMathMode;
  bool addSubscreen;
  bool subtractColor;
  bool halfColor;
  bool mathEnabled[6];
  uint8_t fixedColorR;
  uint8_t fixedColorG;
  uint8_t fixedColorB;
  // settings
  bool forcedBlank;
  uint8_t brightness;
  uint8_t mode;
  bool bg3priority;
  bool evenFrame;
  bool pseudoHires;
  bool overscan;
  bool frameOverscan; // if we are overscanning this frame (determined at 0,225)
  bool interlace;
  bool frameInterlace; // if we are interlacing this frame (determined at start vblank)
  bool directColor;
  // latching
  uint16_t hCount;
  uint16_t vCount;
  bool hCountSecond;
  bool vCountSecond;
  bool countersLatched;
  uint8_t ppu1openBus;
  uint8_t ppu2openBus;
  // pixel buffer (xbgr)
  // times 2 for even and odd frame
  uint8_t pixelbuffer_placeholder;
  /* OR of mathEnabled[i]<<i, maintained at CGADSUB write; after
   * pixelbuffer_placeholder so it is never serialized (rebuilt on load). */
  uint8_t mathEnabledMask;

  uint32_t windowsel;
  uint8_t extraLeftCur, extraRightCur, extraLeftRight;
  uint8_t screenEnabled[2];
  uint8_t screenWindowed[2];
  uint8_t mosaicEnabled;
  uint8_t lastBrightnessMult;
  bool lineHasSprites;
  PpuPixelPrioBufs bgBuffers[2];
  PpuPixelPrioBufs objBuffer;
  uint32_t renderPitch;
  uint8_t *renderBuffer;
  uint8_t brightnessMult[32 + 31];
  uint8_t brightnessMultHalf[32 * 2];
  uint8_t mosaicModulo[kPpuXPixels];

#ifdef PPU_RGB565
  /* The composite loop's inner statement was four table lookups and a pack, per
   * pixel: cgram[idx], then brightnessMult[] three times for R, G and B. But the
   * result is a function of cgram and brightness alone — not of the pixel — so it
   * is the same answer 57,344 times a frame. Bake it once, look it up once.
   * Rebuilt when either input changes, which is a handful of times a frame. */
  uint16_t palette565[256];
  bool paletteDirty;

  /* Derived fixed-color math cache. The slow composite path used to split the
   * same CGRAM color into RGB, add/subtract the same fixed color, clamp through
   * brightness tables and repack RGB565 for every pixel. Cache that pure result
   * for both color-window clip states and all six SNES layers. Subscreen pixels
   * still use the exact per-pixel path because their second color varies.
   * Kept after pixelbuffer_placeholder, so savestates never serialize it. */
  /* 8 rows, not 6, and the two extra are load-bearing: the layer nibble in a z
   * word reaches 6 (BG 0-2, backdrop 5, sprites 4 or 6), so padding to a power
   * of two lets the compositing loops index with one AND and drop the
   * `layer < 6` range test they used to need per pixel. Rows 6 and 7 are filled
   * with the no-math content, which is what those loops computed by hand. */
  uint16_t mathFixed565[2][8][256];
  uint32_t mathFixedKey;
#endif

  /* Sprite line-candidacy cache — derived state, deliberately past
   * pixelbuffer_placeholder so it is never part of the savestate stream.
   * Bit s of objLineCand[line][s>>5] says sprite s covers that scanline
   * (y position and size only; x range and the 32-sprite/34-tile limits are
   * still evaluated per line, in the same order as the full scan). A write
   * that can move a sprite vertically (OAM, OBSEL, SETINI) clears
   * objCacheValid, and so does a savestate load. */
  uint32_t objLineCand[240][4];
  uint8_t objCacheValid;
  /* objBuffer is known to still be all-backdrop when the previous line
   * fetched no sprite tiles; lets ppu_runLine skip the 512-byte ClearBackdrop.
   * Zeroed (= not clean) by ppu_reset's memset. */
  uint8_t objBufferClean;
};

Ppu* ppu_init(Snes* snes);
void ppu_free(Ppu* ppu);
void ppu_copy(Ppu *ppu, Ppu *ppu_src);
void ppu_reset(Ppu* ppu);
bool ppu_checkOverscan(Ppu* ppu);
void ppu_handleVblank(Ppu* ppu);
void ppu_runLine(Ppu* ppu, int line);
uint8_t ppu_read(Ppu* ppu, uint8_t adr);
void ppu_write(Ppu* ppu, uint8_t adr, uint8_t val);
/* A general DMA into the PPU's data ports, without ppu_write's dispatch:
 * ppu_dma_ports says whether the four B-bus targets of a transfer (bAdr plus
 * the mode's offsets) are all OAMDATA, all CGDATA or all VMDATAL/VMDATAH;
 * ppu_dma_write then does exactly ppu_write(regs[(oi + i) & 3], buf[i]) for
 * each byte, in order. */
bool ppu_dma_ports(const uint8_t regs[4]);
void ppu_dma_write(Ppu* ppu, const uint8_t regs[4], unsigned oi, const uint8_t* buf, uint32_t n);
void ppu_saveload(Ppu *ppu, SaveLoadFunc *func, void *ctx);
void PpuBeginDrawing(Ppu *ppu, uint8_t *pixels, size_t pitch, uint32_t render_flags);
#ifdef SNES_LINE_REUSE_PROBE
void ppu_lineReuseProbeReport(void);
#endif
#ifdef SNES_LINE_CACHE
void ppu_lineCacheReport(void);
void ppu_lineCacheInvalidate(void);
#endif

int PpuGetCurrentRenderScale(Ppu *ppu, uint32_t render_flags);

#endif
