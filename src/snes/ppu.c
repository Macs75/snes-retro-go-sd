
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ppu.h"
#include "snes.h"
#include "types.h"
#ifdef TARGET_GNW
#include "gw_malloc.h"
#include "../snes_perf.h"
/* The device framebuffer is RGB565. */
#ifndef PPU_RGB565
#define PPU_RGB565 1
#endif
#endif
typedef uint64_t uint64;
typedef uint32_t uint32;
#ifndef TARGET_GNW
typedef uint32_t uint;
#endif
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint8_t uint8;

extern bool g_new_ppu;
static void PpuDrawWholeLine(Ppu *ppu, uint y);

// array for layer definitions per mode:
//   0-7: mode 0-7; 8: mode 1 + l3prio; 9: mode 7 + extbg

//   0-3; layers 1-4; 4: sprites; 5: nonexistent
static const int layersPerMode[10][12] = {
  {4, 0, 1, 4, 0, 1, 4, 2, 3, 4, 2, 3},
  {4, 0, 1, 4, 0, 1, 4, 2, 4, 2, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 4, 0, 4, 5, 5, 5, 5, 5, 5},
  {4, 4, 4, 0, 4, 5, 5, 5, 5, 5, 5, 5},
  {2, 4, 0, 1, 4, 0, 1, 4, 4, 2, 5, 5},
  {4, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5, 5}
};

static const int prioritysPerMode[10][12] = {
  {3, 1, 1, 2, 0, 0, 1, 1, 1, 0, 0, 0},
  {3, 1, 1, 2, 0, 0, 1, 1, 0, 0, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 0, 0, 5, 5, 5, 5, 5, 5},
  {3, 2, 1, 0, 0, 5, 5, 5, 5, 5, 5, 5},
  {1, 3, 1, 1, 2, 0, 0, 1, 0, 0, 5, 5},
  {3, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5, 5}
};

static const int layerCountPerMode[10] = {
  12, 10, 8, 8, 8, 8, 6, 5, 10, 7
};

static const int bitDepthsPerMode[10][4] = {
  {2, 2, 2, 2},
  {4, 4, 2, 5},
  {4, 4, 5, 5},
  {8, 4, 5, 5},
  {8, 2, 5, 5},
  {4, 2, 5, 5},
  {4, 5, 5, 5},
  {8, 5, 5, 5},
  {4, 4, 2, 5},
  {8, 7, 5, 5}
};

static const int spriteSizes[8][2] = {
  {8, 16}, {8, 32}, {8, 64}, {16, 32},
  {16, 64}, {32, 64}, {16, 32}, {16, 32}
};

static void ppu_handlePixel(Ppu* ppu, int x, int y);
static int ppu_getPixel(Ppu* ppu, int x, int y, bool sub, int* r, int* g, int* b);
static uint16_t ppu_getOffsetValue(Ppu* ppu, int col, int row);
static bool PpuMode4HasOpt(Ppu *ppu);
static bool PpuMode2HasOpt(Ppu *ppu);
static int ppu_getPixelForBgLayer(Ppu* ppu, int x, int y, int layer, bool priority);
static void ppu_handleOPT(Ppu* ppu, int layer, int* lx, int* ly);
static void ppu_calculateMode7Starts(Ppu* ppu, int y);
static int ppu_getPixelForMode7(Ppu* ppu, int x, int layer, bool priority);
static bool ppu_getWindowState(Ppu* ppu, int layer, int x);
static bool ppu_evaluateSprites(Ppu* ppu, int line);
static uint16_t ppu_getVramRemap(Ppu* ppu);
#define SPRITE_PRIO_TO_PRIO(prio, level6) (((prio) * 4 + 2) * 16 + 4 + (level6 ? 2 : 0))
#define SPRITE_PRIO_TO_PRIO_HI(prio) ((prio) * 4 + 2)


#define IS_SCREEN_ENABLED(ppu, sub, layer) (ppu->screenEnabled[sub] & (1 << layer))
#define IS_SCREEN_WINDOWED(ppu, sub, layer) (ppu->screenWindowed[sub] & (1 << layer))
#define IS_MOSAIC_ENABLED(ppu, layer) ((ppu->mosaicEnabled & (1 << layer)))
#define GET_WINDOW_FLAGS(ppu, layer) (ppu->windowsel >> (layer * 4))
enum {
  kWindow1Inversed = 1,
  kWindow1Enabled = 2,
  kWindow2Inversed = 4,
  kWindow2Enabled = 8,
};

Ppu* ppu_init(Snes* snes) {
#ifdef TARGET_GNW
  /* One PPU on the device (there is no reference emulator to run alongside), so
   * a static instance beats a malloc from the overlay pool. */
  static Ppu g_ppu;
  Ppu* ppu = &g_ppu;
#if defined(GNW_SNES_CORE)
  /* SNES overlay only: VRAM in overlay BSS (RAM_EMU) as a static array — frees
   * 64 KB of ITCM for the rc hot subset. The GNW_SNES_CORE guard keeps this
   * array out of the SM overlay (which also compiles ppu.o for shared symbols
   * but has its own PPU and its own tight BSS budget). */
  static uint16_t g_ppu_vram[0x8000];
  if (ppu->vram == NULL) {
#if SNES_VRAM_IN_DTCM
    /* VRAM is the only thing measured as expensive on this part: 64 KB in AXI
     * SRAM, read at an address the tilemap picks, behind a 16 KB D-cache.
     * Ablating those reads is worth +4.33 fps, and nothing that keeps them in
     * that memory has recovered any of it -- not a memo at an 80% hit rate, not
     * a prefetch, not removing the framebuffer's cache pollution. So move the
     * memory instead of trying to cache it better.
     *
     * DTCM is zero-wait and needs no cache at all. It is not free space: the
     * stdlib heap takes whatever DTCM is left over, and it is shared with the
     * launcher and every other core. But the heap's high-water mark measured on
     * the device during SNES play is 11,336 B of 90,336 -- 79 KB idle -- so 64 KB
     * fits with room to spare, and taking it from the heap at run time costs the
     * other cores nothing because they never run this line.
     *
     * If the allocation fails the static below is still there and nothing
     * changes; a slower emulator is a better failure than one that does not
     * start.
     *
     * MEASURED: NOTHING. 55.43 fps against a 55.46 baseline, four runs each back
     * to back, with ppu->vram read over SWD as 0x20006a70 to prove the
     * allocation succeeded rather than falling back.
     *
     * That closes the memory theory entirely. Four separate attacks on VRAM read
     * cost -- an 80%-hit memo, a prefetch, removing the framebuffer's cache
     * pollution, and finally putting the whole 64 KB in zero-wait DTCM -- all
     * measure zero, while deleting the layer draw outright is worth +4.33 fps.
     * The cost is not the reads. It is the loop that makes them: the tilemap
     * walk, the window-span arithmetic and the per-layer call, 68 times a line.
     * Left off; it costs the shared DTCM heap 64 KB for no return. */
    extern void *malloc(size_t);
    uint16_t *dtcm = (uint16_t *)malloc(0x8000 * sizeof(uint16_t));
    if (dtcm) {
      memset(dtcm, 0, 0x8000 * sizeof(uint16_t));
      ppu->vram = dtcm;
    } else
#endif
    ppu->vram = g_ppu_vram;
  }
#else
  /* Other GNW overlays (SM etc.): VRAM in ITCM as before. */
  if (ppu->vram == NULL)
    ppu->vram = (uint16_t *)itc_calloc(1, 0x10000);
#endif
#else
  Ppu* ppu = malloc(sizeof(Ppu));
#endif
  ppu->snes = snes;
  return ppu;
}

void ppu_free(Ppu* ppu) {
#ifndef TARGET_GNW
  free(ppu);
#endif
}

#ifndef TARGET_GNW
void ppu_copy(Ppu *ppu, Ppu *ppu_src) {
  Snes *snes = ppu->snes;
  size_t pitch = ppu->renderPitch;
  uint8_t *renderBuffer = ppu->renderBuffer;
  memcpy(ppu, ppu_src, sizeof(*ppu));
  ppu->renderBuffer = renderBuffer;
  ppu->renderPitch = (uint32_t)pitch;
  ppu->snes = snes;
}
#endif  /* !TARGET_GNW */

void ppu_reset(Ppu* ppu) {
  {
    Snes *snes = ppu->snes;
    size_t pitch = ppu->renderPitch;
    uint8_t *renderBuffer = ppu->renderBuffer;
#ifdef TARGET_GNW
    /* vram is a pointer now: the wholesale memset below would throw it away. */
    uint16_t *vram = ppu->vram;
#endif
    memset(ppu, 0, sizeof(*ppu));
#ifdef TARGET_GNW
    ppu->vram = vram;
    memset(ppu->vram, 0, 0x10000);
#endif
    ppu->renderBuffer = renderBuffer;
    ppu->renderPitch = (uint32_t)pitch;
    ppu->snes = snes;
  }
  ppu->vramPointer = 0;
  ppu->vramIncrementOnHigh = false;
  ppu->vramIncrement = 1;
  ppu->vramRemapMode = 0;
  ppu->vramReadBuffer = 0;
  memset(ppu->cgram, 0, sizeof(ppu->cgram));
#ifdef PPU_RGB565
  ppu->paletteDirty = true;   /* the derived tables (some static) describe the old cgram */
#endif
  ppu->cgramPointer = 0;
  ppu->cgramSecondWrite = false;
  ppu->cgramBuffer = 0;
  memset(ppu->oam, 0, sizeof(ppu->oam));
  memset(ppu->highOam, 0, sizeof(ppu->highOam));
  ppu->oamAdr = 0;
  ppu->oamAdrWritten = 0;
  ppu->oamInHigh = false;
  ppu->oamInHighWritten = false;
  ppu->oamSecondWrite = false;
  ppu->oamBuffer = 0;
  ppu->objPriority = false;
  ppu->objTileAdr1 = 0;
  ppu->objTileAdr2 = 0;
  ppu->objSize = 0;
  ppu->timeOver = false;
  ppu->rangeOver = false;
  ppu->objInterlace = false;
  for(int i = 0; i < 4; i++) {
    ppu->bgLayer[i].hScroll = 0;
    ppu->bgLayer[i].vScroll = 0;
    ppu->bgLayer[i].tilemapWider = false;
    ppu->bgLayer[i].tilemapHigher = false;
    ppu->bgLayer[i].tilemapAdr = 0;
    ppu->bgLayer[i].tileAdr = 0;
    ppu->bgLayer[i].bigTiles = false;
    ppu->bgLayer[i].mosaicEnabled = false;
  }
  ppu->scrollPrev = 0;
  ppu->scrollPrev2 = 0;
  ppu->mosaicSize = 1;
  ppu->mosaicStartLine = 1;
  for(int i = 0; i < 5; i++) {
    ppu->layer[i].mainScreenEnabled = false;
    ppu->layer[i].subScreenEnabled = false;
    ppu->layer[i].mainScreenWindowed = false;
    ppu->layer[i].subScreenWindowed = false;
  }
  memset(ppu->m7matrix, 0, sizeof(ppu->m7matrix));
  ppu->m7prev = 0;
  ppu->m7largeField = false;
  ppu->m7charFill = false;
  ppu->m7xFlip = false;
  ppu->m7yFlip = false;
  ppu->m7extBg = false;
  ppu->m7startX = 0;
  ppu->m7startY = 0;
  for(int i = 0; i < 6; i++) {
    ppu->windowLayer[i].window1enabled = false;
    ppu->windowLayer[i].window2enabled = false;
    ppu->windowLayer[i].window1inversed = false;
    ppu->windowLayer[i].window2inversed = false;
    ppu->windowLayer[i].maskLogic = 0;
  }
  ppu->window1left = 0;
  ppu->window1right = 0;
  ppu->window2left = 0;
  ppu->window2right = 0;
  ppu->clipMode = 0;
  ppu->preventMathMode = 0;
  ppu->addSubscreen = false;
  ppu->subtractColor = false;
  ppu->halfColor = false;
  memset(ppu->mathEnabled, 0, sizeof(ppu->mathEnabled));
  ppu->fixedColorR = 0;
  ppu->fixedColorG = 0;
  ppu->fixedColorB = 0;
  ppu->forcedBlank = true;
  ppu->brightness = 0;
  ppu->mode = 0;
  ppu->bg3priority = false;
  ppu->evenFrame = false;
  ppu->pseudoHires = false;
  ppu->overscan = false;
  ppu->frameOverscan = false;
  ppu->interlace = false;
  ppu->frameInterlace = false;
  ppu->directColor = false;
  ppu->hCount = 0;
  ppu->vCount = 0;
  ppu->hCountSecond = false;
  ppu->vCountSecond = false;
  ppu->countersLatched = false;
  ppu->ppu1openBus = 0;
  ppu->ppu2openBus = 0;
#ifdef SNES_LINE_CACHE
  ppu_lineCacheInvalidate();
#endif
}

/* ppu_write() stores every screen-enable and window register TWICE: unpacked into
 * layer[]/windowLayer[], and packed into screenEnabled/screenWindowed/windowsel —
 * the byte the game actually wrote. The renderer reads only the packed copies:
 *
 *     #define IS_SCREEN_ENABLED(ppu, sub, layer) (ppu->screenEnabled[sub] & (1 << layer))
 *
 * and the packed copies sit past pixelbuffer_placeholder, so the savestate does
 * not carry them. They are caches, exactly like palette565 — and like palette565
 * a load has to rebuild them, because the unpacked originals it DID restore are
 * not what anything looks at.
 *
 * Left alone, screenEnabled stays whatever it was. Load into a PPU that was just
 * ppu_reset() — which is every "resume from a savestate" on the G&W, because the
 * launcher boots the core and loads second — and it is zero: no BG, no sprites,
 * every line composited as bare backdrop. A black screen that still runs at full
 * speed on almost no CPU, because there is nothing left to draw. */
static void ppu_rebuild_packed_registers(Ppu *ppu) {
  uint8_t tm = 0, ts = 0, tmw = 0, tsw = 0;

  for (int i = 0; i < 5; i++) {   /* BG1..BG4, OBJ — $212C/$212D/$212E/$212F */
    if (ppu->layer[i].mainScreenEnabled)  tm  |= 1 << i;
    if (ppu->layer[i].subScreenEnabled)   ts  |= 1 << i;
    if (ppu->layer[i].mainScreenWindowed) tmw |= 1 << i;
    if (ppu->layer[i].subScreenWindowed)  tsw |= 1 << i;
  }
  ppu->screenEnabled[0] = tm;
  ppu->screenEnabled[1] = ts;
  ppu->screenWindowed[0] = tmw;
  ppu->screenWindowed[1] = tsw;

  /* $2106 lives past pixelbuffer_placeholder too. ppu_write() packs the
   * layer enable bits into mosaicEnabled; a load only restores
   * bgLayer[].mosaicEnabled. */
  {
    uint8_t mos = 0;
    for (int i = 0; i < 4; i++)
      if (ppu->bgLayer[i].mosaicEnabled)
        mos |= (uint8_t)(1 << i);
    ppu->mosaicEnabled = mos;
  }

  /* windowsel is six 4-bit fields, one per layer, in the order GET_WINDOW_FLAGS
   * indexes them — the same nibble ppu_write() packs from $2123..$2125. */
  uint32_t sel = 0;
  for (int i = 0; i < 6; i++) {
    uint32_t flags = 0;
    if (ppu->windowLayer[i].window1inversed) flags |= kWindow1Inversed;
    if (ppu->windowLayer[i].window1enabled)  flags |= kWindow1Enabled;
    if (ppu->windowLayer[i].window2inversed) flags |= kWindow2Inversed;
    if (ppu->windowLayer[i].window2enabled)  flags |= kWindow2Enabled;
    sel |= flags << (i * 4);
  }
  ppu->windowsel = sel;
}

void ppu_saveload(Ppu *ppu, SaveLoadFunc *func, void *ctx) {
#ifdef PPU_RGB565
  /* Everything the PPU derives from cgram and brightness lives outside the saved
   * region — the RGB565 palette and the brightness table are caches, not state.
   * A load restores cgram underneath them and nothing tells them so: the screen
   * then draws the scene you loaded with the colours of the scene you left, or,
   * on a PPU that has drawn nothing yet, with no colours at all. Invalidate them.
   * (0xff is not a brightness, so the table rebuilds on the next line.) */
  ppu->paletteDirty = true;
  ppu->lastBrightnessMult = 0xff;
#endif
  /* Same rule for the sprite cache: a load restores OAM underneath it. And the
   * objBuffer contents aren't part of the stream either, so stop trusting them. */
  ppu->objCacheValid = 0;
  ppu->objBufferClean = 0;
#ifdef TARGET_GNW
  /* vram lives in ITC RAM now, so it is no longer contiguous with the rest of
   * the struct. Emit the identical byte stream — VRAM first, then everything
   * from vramPointer on — so savestates stay compatible with the PC build. */
  func(ctx, ppu->vram, 0x8000 * sizeof(uint16_t));
  func(ctx, &ppu->vramPointer,
       offsetof(Ppu, pixelbuffer_placeholder) - offsetof(Ppu, vramPointer));
#else
  func(ctx, &ppu->vram, offsetof(Ppu, pixelbuffer_placeholder) - offsetof(Ppu, vram));
#endif

  /* After the stream, so a load rebuilds from what it just read. On a save this
   * recomputes the values it already had — the two copies agree by construction,
   * ppu_write() writes both — so it is a no-op there rather than a special case. */
  ppu_rebuild_packed_registers(ppu);
#ifdef SNES_LINE_CACHE
  /* The framebuffer is not part of the savestate stream.  Whether this call
   * saved or loaded, stop trusting its pixels until every line is redrawn. */
  ppu_lineCacheInvalidate();
#endif
}

void PpuBeginDrawing(Ppu *ppu, uint8_t *pixels, size_t pitch, uint32_t render_flags) {
  ppu->renderPitch = (uint)pitch;
  ppu->renderBuffer = pixels;
}

bool ppu_checkOverscan(Ppu* ppu) {
  // called at (0,225)
  ppu->frameOverscan = ppu->overscan; // set if we have a overscan-frame
  return ppu->frameOverscan;
}

void ppu_handleVblank(Ppu* ppu) {
  // called either right after ppu_checkOverscan at (0,225), or at (0,240)
  if(!ppu->forcedBlank) {
    ppu->oamAdr = ppu->oamAdrWritten;
    ppu->oamInHigh = ppu->oamInHighWritten;
    ppu->oamSecondWrite = false;
  }
  ppu->frameInterlace = ppu->interlace; // set if we have a interlaced frame
}

#ifndef SNES_SPRITE_CENSUS
#define SNES_SPRITE_CENSUS 0
#endif
bool g_ppu_skip_render;
#ifndef SNES_ABLATE_OBJWIPE
#define SNES_ABLATE_OBJWIPE 0
#endif
#ifndef SNES_ABLATE_SPRITE_MERGE
#define SNES_ABLATE_SPRITE_MERGE 0
#endif
#ifndef SNES_ABLATE_SPRITE_PIX
#define SNES_ABLATE_SPRITE_PIX 0
#endif
#ifndef SNES_ABLATE_SPRITES
#define SNES_ABLATE_SPRITES 0
#endif
#ifndef SNES_ABLATE_MATHFIXED
#define SNES_ABLATE_MATHFIXED 0
#endif
#ifndef SNES_MATHFIXED_CENSUS
#define SNES_MATHFIXED_CENSUS 0
#endif
#if SNES_MATHFIXED_CENSUS
uint32_t g_mathfixed_lines, g_mathfixed_rebuilds;
uint32_t g_objcache_rebuilds, g_objeval_lines, g_sprite_visits, g_sprite_cols;
#endif
#ifndef SNES_SKIP_SPRITE_EVAL_ON_SKIP
#define SNES_SKIP_SPRITE_EVAL_ON_SKIP 0
#endif
#if SNES_SKIP_SPRITE_EVAL_ON_SKIP
/* Three frames in four are thrown away by the overload guard, and all three of
 * them were evaluating sprites for a buffer nobody reads.
 *
 * Everything the evaluation leaves behind on a skipped line is either
 * unobservable or recomputed before its next use: objBuffer is consumed only by
 * the compositing, which is already behind the g_ppu_skip_render return;
 * lineHasSprites is written before every use on a drawn line; objBufferClean
 * still describes the buffer correctly, because a skipped line does not write
 * it. That leaves rangeOver/timeOver, whose only reader in the entire emulator
 * is $213E -- so gate the skip on whether this game has ever read it.
 *
 * Measured on the DRAW RATE, not on fps: fps counts emulated frames, and making
 * a skipped frame cheaper lets the pacing guard draw more, which moves the
 * player's picture without moving that number. */
static bool g_stat77_read;
#endif

#if SNES_RENDER_CENSUS
/* Declared here because ppu_runLine reads them and it comes before the rest of
 * the census block. */
uint32_t g_lc_hit, g_lc_miss;
/* Is the tile fetch repetitive? The render's whole remaining cost is reading
 * VRAM -- pixel arithmetic ablates to nothing, and the line-cache tracking that
 * rode along with the reads is gone. Two halfwords per 4bpp tile, 68 tiles a
 * line, random access into 64 KB behind a 16 KB cache. If consecutive tiles in a
 * row repeat, a one-entry memo skips both the read and the decode. The
 * subscreen's layer is fully opaque and 33 tiles a line; flat areas repeat.
 * Count it before writing the memo. */
uint32_t g_tile_same, g_tile_diff;
#endif

#ifdef TARGET_GNW
void (*g_ppu_line_cb)(unsigned y, const uint16_t *line);
#endif

_Static_assert(_Alignof(PpuPixelPrioBufs) >= 8,
               "ClearBackdrop writes 64 bits at a time; on ARM that is STRD, which "
               "faults on an unaligned address. Keep the aligned(8) on the struct.");

#if defined(SNES_LINE_REUSE_PROBE) || defined(SNES_LINE_CACHE)
enum { kLineHistoryLines = 240, kLineHistoryVramPages = 512 };
#endif

#ifdef PPU_RGB565
/* CGRAM entries written with a NEW value since the last drawn line. The
 * line-start rebuild updates only these in the palette, colour-math and blend
 * tables; ppu->paletteDirty still means "rebuild all of it" (brightness, load).
 * HDMA colour gradients rewrite one or two entries a line, and the full
 * rebuild that used to follow was ~3,000 cycles a line on the device. Derived
 * state, so a static: never in a savestate. */
static uint32_t g_cgram_dirty[8];
static bool g_cgram_dirty_any;
#endif

static inline void ClearBackdrop(PpuPixelPrioBufs *buf) {
  for (size_t i = 0; i != arraysize(buf->data); i += 4)
    *(uint64*)&buf->data[i] = 0x0500050005000500;
}

#ifdef SNES_LINE_REUSE_PROBE
/* Observation only: predict from inputs, still render, then compare the exact
 * RGB565 line against the previous frame. None of this state is emulated. */
typedef struct PpuLineProbeState {
  BgLayer bgLayer[4];
  int16_t m7matrix[8];
  uint32_t windowsel;
  uint16_t objTileAdr1, objTileAdr2;
  uint8_t objPriority, objSize, objInterlace, oamAdr;
  uint8_t mosaicSize, mosaicStartLine, mosaicEnabled;
  uint8_t window1left, window1right, window2left, window2right;
  uint8_t clipMode, preventMathMode, addSubscreen, subtractColor, halfColor;
  uint8_t mathEnabled[6];
  uint8_t fixedColorR, fixedColorG, fixedColorB;
  uint8_t forcedBlank, brightness, mode, bg3priority;
  uint8_t pseudoHires, directColor, m7largeField, m7charFill, m7xFlip, m7yFlip, m7extBg;
  uint8_t screenEnabled[2], screenWindowed[2];
  uint8_t extraLeftCur, extraRightCur, extraLeftRight;
  uint8_t lineHasSprites, evenFrameWhenObjInterlace;
} PpuLineProbeState;

typedef struct PpuLineProbeStats {
  uint64_t total, actualSame, predicted, falsePositive, falseNegative;
} PpuLineProbeStats;

enum { kProbeLines = kLineHistoryLines, kProbeBuckets = 4, kProbeVariants = 12,
       kProbeVramPages = kLineHistoryVramPages };
static PpuLineProbeState g_probe_prev_state[kProbeLines];
static uint32_t g_probe_prev_vram[kProbeLines], g_probe_prev_cgram[kProbeLines], g_probe_prev_oam[kProbeLines];
static uint8_t g_probe_prev_line[kProbeLines][kPpuXPixels * sizeof(uint16_t)];
static uint8_t g_probe_valid[kProbeLines];
static uint8_t g_probe_pending[kProbeVariants];
static uint32_t g_probe_vram_gen, g_probe_cgram_gen, g_probe_oam_gen;
static uint32_t g_probe_vram_page_gen[kProbeVramPages], g_probe_oam_entry_gen[128];
static uint32_t g_probe_cgram_entry_gen[256];
static uint32_t g_probe_prev_vram_page_gen[kProbeLines][kProbeVramPages];
static uint32_t g_probe_prev_oam_entry_gen[kProbeLines][128];
static uint32_t g_probe_prev_cgram_entry_gen[kProbeLines][256];
static uint32_t g_probe_prev_vram_mask[kProbeLines][16], g_probe_cur_vram_mask[16];
static uint32_t g_probe_prev_oam_mask[kProbeLines][4];
static uint32_t g_probe_prev_cgram_mask[kProbeLines][8];
static uint32_t g_probe_frame;
static PpuLineProbeStats g_probe_stats[kProbeBuckets + 1][kProbeVariants];

static inline void PpuLineProbeVram(uint32_t adr) {
  g_probe_cur_vram_mask[(adr >> 11) & 15] |= 1u << ((adr >> 6) & 31);
}

static inline uint16_t PpuLineProbeVramPtr(Ppu *ppu, const uint16_t *ptr) {
  PpuLineProbeVram((uint32_t)(ptr - ppu->vram) & 0x7fff);
  return *ptr;
}

static void PpuLineProbeCapture(PpuLineProbeState *s, const Ppu *ppu) {
  memset(s, 0, sizeof(*s));
  memcpy(s->bgLayer, ppu->bgLayer, sizeof(s->bgLayer));
  memcpy(s->m7matrix, ppu->m7matrix, sizeof(s->m7matrix));
  memcpy(s->mathEnabled, ppu->mathEnabled, sizeof(s->mathEnabled));
  memcpy(s->screenEnabled, ppu->screenEnabled, sizeof(s->screenEnabled));
  memcpy(s->screenWindowed, ppu->screenWindowed, sizeof(s->screenWindowed));
  s->windowsel = ppu->windowsel;
  s->objTileAdr1 = ppu->objTileAdr1; s->objTileAdr2 = ppu->objTileAdr2;
  s->objPriority = ppu->objPriority; s->objSize = ppu->objSize;
  s->objInterlace = ppu->objInterlace; s->oamAdr = ppu->oamAdr;
  s->mosaicSize = ppu->mosaicSize; s->mosaicStartLine = ppu->mosaicStartLine;
  s->mosaicEnabled = ppu->mosaicEnabled;
  s->window1left = ppu->window1left; s->window1right = ppu->window1right;
  s->window2left = ppu->window2left; s->window2right = ppu->window2right;
  s->clipMode = ppu->clipMode; s->preventMathMode = ppu->preventMathMode;
  s->addSubscreen = ppu->addSubscreen; s->subtractColor = ppu->subtractColor;
  s->halfColor = ppu->halfColor;
  s->fixedColorR = ppu->fixedColorR; s->fixedColorG = ppu->fixedColorG;
  s->fixedColorB = ppu->fixedColorB;
  s->forcedBlank = ppu->forcedBlank; s->brightness = ppu->brightness;
  s->mode = ppu->mode; s->bg3priority = ppu->bg3priority;
  s->pseudoHires = ppu->pseudoHires; s->directColor = ppu->directColor;
  s->m7largeField = ppu->m7largeField; s->m7charFill = ppu->m7charFill;
  s->m7xFlip = ppu->m7xFlip; s->m7yFlip = ppu->m7yFlip; s->m7extBg = ppu->m7extBg;
  s->extraLeftCur = ppu->extraLeftCur; s->extraRightCur = ppu->extraRightCur;
  s->extraLeftRight = ppu->extraLeftRight; s->lineHasSprites = ppu->lineHasSprites;
  s->evenFrameWhenObjInterlace = ppu->objInterlace ? ppu->evenFrame : 0;
}

static void PpuLineProbeFieldDiff(const PpuLineProbeState *cur, const PpuLineProbeState *prev, uint32_t bucket);

static bool PpuLineProbeRegsMatchExcl(const PpuLineProbeState *a, const PpuLineProbeState *b) {
  const uint8_t *pa = (const uint8_t *)a, *pb = (const uint8_t *)b;
  size_t sz = sizeof(PpuLineProbeState);
  bool skip_m7 = (a->mode != 7 && b->mode != 7);
  size_t m7_off = offsetof(PpuLineProbeState, m7matrix);
  size_t m7_end = m7_off + sizeof(a->m7matrix);
  size_t oam_off = offsetof(PpuLineProbeState, oamAdr);
  for (size_t i = 0; i < sz; i++) {
    if (i == oam_off) continue;
    if (skip_m7 && i >= m7_off && i < m7_end) continue;
    if (pa[i] != pb[i]) return false;
  }
  return true;
}

static void PpuLineProbeBefore(Ppu *ppu, int line) {
  PpuLineProbeState cur;
  PpuLineProbeCapture(&cur, ppu);
  int y = line - 1;
  bool regs_raw = g_probe_valid[y] && memcmp(&cur, &g_probe_prev_state[y], sizeof(cur)) == 0;
  bool regs = regs_raw;
  if (g_probe_valid[y] && !regs_raw) {
    uint32_t bucket = (g_probe_frame - 1) / 300;
    if (bucket >= kProbeBuckets) bucket = kProbeBuckets - 1;
    PpuLineProbeFieldDiff(&cur, &g_probe_prev_state[y], bucket);
  }
  bool vr = g_probe_vram_gen == g_probe_prev_vram[y];
  bool cg = g_probe_cgram_gen == g_probe_prev_cgram[y];
  bool oa = g_probe_oam_gen == g_probe_prev_oam[y];
  bool vr_pages = g_probe_valid[y];
  for (int page = 0; page < kProbeVramPages && vr_pages; page++)
    if ((g_probe_prev_vram_mask[y][page >> 5] & (1u << (page & 31))) &&
        g_probe_vram_page_gen[page] != g_probe_prev_vram_page_gen[y][page])
      vr_pages = false;
  const uint32_t *cur_oam_mask = ppu->objLineCand[y];
  bool oa_line = g_probe_valid[y];
  for (int s = 0; s < 128 && oa_line; s++)
    if (((cur_oam_mask[s >> 5] | g_probe_prev_oam_mask[y][s >> 5]) & (1u << (s & 31))) &&
        g_probe_oam_entry_gen[s] != g_probe_prev_oam_entry_gen[y][s])
      oa_line = false;
  bool cg_line = g_probe_valid[y];
  for (int index = 0; index < 256 && cg_line; index++)
    if ((g_probe_prev_cgram_mask[y][index >> 5] & (1u << (index & 31))) &&
        g_probe_cgram_entry_gen[index] != g_probe_prev_cgram_entry_gen[y][index])
      cg_line = false;
  g_probe_pending[0] = regs && vr && cg && oa; /* conservative */
  g_probe_pending[1] = regs && vr && cg;       /* omit OAM */
  g_probe_pending[2] = regs && cg && oa;       /* omit VRAM */
  g_probe_pending[3] = regs && vr && oa;       /* omit CGRAM */
  g_probe_pending[4] = regs;                   /* registers only */
  g_probe_pending[5] = regs && vr_pages && cg && oa;
  g_probe_pending[6] = regs && vr_pages && cg && oa_line;
  g_probe_pending[7] = regs && vr_pages && cg_line && oa_line;
  g_probe_pending[8] = regs && vr_pages && cg_line && (!ppu->lineHasSprites || oa_line);
  bool regs_excl = g_probe_valid[y] && PpuLineProbeRegsMatchExcl(&cur, &g_probe_prev_state[y]);
  g_probe_pending[9] = regs_excl && vr_pages && cg && oa_line;
  g_probe_pending[10] = regs_excl && vr_pages && cg_line && oa_line;
  g_probe_pending[11] = regs_excl && vr_pages && cg_line && (!ppu->lineHasSprites || oa_line);
  g_probe_prev_state[y] = cur;
  g_probe_prev_vram[y] = g_probe_vram_gen;
  g_probe_prev_cgram[y] = g_probe_cgram_gen;
  g_probe_prev_oam[y] = g_probe_oam_gen;
  memcpy(g_probe_prev_vram_page_gen[y], g_probe_vram_page_gen, sizeof(g_probe_vram_page_gen));
  memcpy(g_probe_prev_oam_entry_gen[y], g_probe_oam_entry_gen, sizeof(g_probe_oam_entry_gen));
  memcpy(g_probe_prev_cgram_entry_gen[y], g_probe_cgram_entry_gen, sizeof(g_probe_cgram_entry_gen));
  memcpy(g_probe_prev_oam_mask[y], cur_oam_mask, sizeof(g_probe_prev_oam_mask[y]));
}

static void PpuLineProbeAfter(Ppu *ppu, int line) {
  int y = line - 1;
  const uint8_t *cur = ppu->renderBuffer + y * ppu->renderPitch;
  bool same = g_probe_valid[y] && memcmp(cur, g_probe_prev_line[y], sizeof(g_probe_prev_line[y])) == 0;
  if (g_probe_valid[y]) {
    uint32_t bucket = (g_probe_frame - 1) / 300;
    if (bucket >= kProbeBuckets) bucket = kProbeBuckets - 1;
    for (int v = 0; v < kProbeVariants; v++) {
      PpuLineProbeStats *all = &g_probe_stats[kProbeBuckets][v];
      PpuLineProbeStats *part = &g_probe_stats[bucket][v];
#define ADD_STAT(field, value) do { all->field += (value); part->field += (value); } while (0)
      ADD_STAT(total, 1);
      ADD_STAT(actualSame, same);
      ADD_STAT(predicted, g_probe_pending[v]);
      ADD_STAT(falsePositive, g_probe_pending[v] && !same);
      ADD_STAT(falseNegative, !g_probe_pending[v] && same);
#undef ADD_STAT
    }
  }
  memcpy(g_probe_prev_line[y], cur, sizeof(g_probe_prev_line[y]));
  memset(g_probe_prev_cgram_mask[y], 0, sizeof(g_probe_prev_cgram_mask[y]));
  uint32_t math_enabled = 0;
  for (int layer = 0; layer < 6; layer++) math_enabled |= ppu->mathEnabled[layer] << layer;
  bool uses_subscreen = ppu->preventMathMode != 3 && ppu->addSubscreen &&
      math_enabled && ppu->screenEnabled[1] != 0;
  for (int x = 0; x < kPpuXPixels; x++) {
    uint8_t main_index = ppu->bgBuffers[0].data[x] & 0xff;
    g_probe_prev_cgram_mask[y][main_index >> 5] |= 1u << (main_index & 31);
    if (uses_subscreen) {
      uint8_t sub_index = ppu->bgBuffers[1].data[x] & 0xff;
      g_probe_prev_cgram_mask[y][sub_index >> 5] |= 1u << (sub_index & 31);
    }
  }
  memcpy(g_probe_prev_vram_mask[y], g_probe_cur_vram_mask, sizeof(g_probe_cur_vram_mask));
  g_probe_valid[y] = 1;
}

void ppu_lineReuseProbeReport(void) {
  static const char *const names[kProbeVariants] = {
    "full", "no_oam", "no_vram", "no_cgram", "regs", "vram_pages", "vram_pages_oam_line",
    "pages_oam_cgram_line", "pages_cgram_no_sprite",
    "excl_oamadr_pages", "excl_oamadr_cgram_line", "excl_oamadr_no_sprite"
  };
  for (int b = 0; b <= kProbeBuckets; b++) {
    for (int v = 0; v < kProbeVariants; v++) {
      const PpuLineProbeStats *s = &g_probe_stats[b][v];
      printf("[line-reuse] bucket=%s variant=%s total=%llu actual=%llu predicted=%llu fp=%llu fn=%llu pred_x10000=%llu fn_x10000=%llu\n",
          b == kProbeBuckets ? "all" : (b == 0 ? "0-299" : b == 1 ? "300-599" : b == 2 ? "600-899" : "900-1199"), names[v],
          (unsigned long long)s->total, (unsigned long long)s->actualSame,
          (unsigned long long)s->predicted, (unsigned long long)s->falsePositive,
          (unsigned long long)s->falseNegative,
          (unsigned long long)(s->total ? s->predicted * 10000 / s->total : 0),
          (unsigned long long)(s->actualSame ? s->falseNegative * 10000 / s->actualSame : 0));
    }
  }
}

/* Field-level diff tracker: when regs memcmp fails, identify which field(s) differ. */
static uint32_t g_probe_fdiff[kProbeBuckets + 1][sizeof(PpuLineProbeState)];
static uint64_t g_probe_fdiff_calls[kProbeBuckets + 1];

static void PpuLineProbeFieldDiff(const PpuLineProbeState *cur, const PpuLineProbeState *prev, uint32_t bucket) {
  const uint8_t *pa = (const uint8_t *)cur, *pb = (const uint8_t *)prev;
  g_probe_fdiff_calls[bucket]++;
  g_probe_fdiff_calls[kProbeBuckets]++;
  for (size_t i = 0; i < sizeof(PpuLineProbeState); i++)
    if (pa[i] != pb[i]) {
      g_probe_fdiff[bucket][i]++;
      g_probe_fdiff[kProbeBuckets][i]++;
    }
}

/* Field name lookup for reporting */
static const char *ppuProbeFieldName(size_t off) {
#define OFF(field) offsetof(PpuLineProbeState, field)
  if (off < OFF(m7matrix))              return "bgLayer";
  if (off < OFF(windowsel))             return "m7matrix";
  if (off < OFF(objTileAdr1))           return "windowsel";
  if (off < OFF(objPriority))           return "objTileAdr";
  if (off < OFF(mosaicSize))            return "objCfg(oamAdr etc)";
  if (off < OFF(window1left))           return "mosaic";
  if (off < OFF(clipMode))              return "window";
  if (off < OFF(mathEnabled))           return "clipMath(addSub/sub/half)";
  if (off < OFF(fixedColorR))           return "mathEnabled";
  if (off < OFF(forcedBlank))           return "fixedColor";
  if (off < OFF(pseudoHires))           return "blankBrightMode";
  if (off < OFF(screenEnabled))         return "bg3prio/hires/direct/m7opts";
  if (off < OFF(extraLeftCur))          return "screen";
  if (off < OFF(lineHasSprites))        return "extraLeftRight";
  if (off < sizeof(PpuLineProbeState))  return "sprites/evenFrame";
  return "?";
#undef OFF
}

void ppu_lineProbeFieldReport(void) {
  for (int b = 0; b <= kProbeBuckets; b++) {
    const char *bname = b == kProbeBuckets ? "all" : (b == 0 ? "0-299" : b == 1 ? "300-599" : b == 2 ? "600-899" : "900-1199");
    /* Aggregate by field name, not byte offset, for readability */
    uint64_t agg[20]; memset(agg, 0, sizeof(agg));
    const char *names[20];
    int nfields = 0;
    for (size_t i = 0; i < sizeof(PpuLineProbeState); i++) {
      if (g_probe_fdiff[b][i] == 0) continue;
      const char *fn = ppuProbeFieldName(i);
      int idx = -1;
      for (int j = 0; j < nfields; j++) if (names[j] == fn) { idx = j; break; }
      if (idx < 0) { idx = nfields++; names[idx] = fn; }
      agg[idx] += g_probe_fdiff[b][i];
    }
    uint64_t calls = g_probe_fdiff_calls[b];
    printf("[field-diff] bucket=%s regs_false_calls=%llu\n", bname, (unsigned long long)calls);
    for (int j = 0; j < nfields; j++)
      printf("[field-diff]   field=%-24s changes=%llu (%llu%% of regs_false)\n",
             names[j], (unsigned long long)agg[j],
             (unsigned long long)(calls ? agg[j] * 100 / calls : 0));
  }
  /* Also dump raw byte offsets for top noisy fields in gameplay bucket */
  int b = 3; /* 900-1199 */
  printf("[field-diff] raw byte offsets (bucket 900-1199, top 20):\n");
  uint32_t sorted_idx[sizeof(PpuLineProbeState)];
  for (size_t i = 0; i < sizeof(PpuLineProbeState); i++) sorted_idx[i] = (uint32_t)i;
  /* simple insertion sort by count desc */
  for (size_t i = 1; i < sizeof(PpuLineProbeState); i++) {
    for (size_t j = i; j > 0 && g_probe_fdiff[b][sorted_idx[j]] > g_probe_fdiff[b][sorted_idx[j-1]]; j--) {
      uint32_t tmp = sorted_idx[j]; sorted_idx[j] = sorted_idx[j-1]; sorted_idx[j-1] = tmp;
    }
  }
  int shown = 0;
  for (size_t i = 0; i < sizeof(PpuLineProbeState) && shown < 20; i++) {
    uint32_t off = sorted_idx[i];
    if (g_probe_fdiff[b][off] == 0) break;
    printf("[field-diff]   offset=%3u count=%-8u field=%s\n",
           off, g_probe_fdiff[b][off], ppuProbeFieldName(off));
    shown++;
  }
}
#endif

#ifdef SNES_LINE_CACHE
typedef struct PpuLineCacheState {
  BgLayer bgLayer[4];
  int16_t m7matrix[8];
  uint32_t windowsel;
  uint16_t objTileAdr1, objTileAdr2;
  uint8_t objPriority, objSize, objInterlace, oamAdr;
  uint8_t mosaicSize, mosaicStartLine, mosaicEnabled;
  uint8_t window1left, window1right, window2left, window2right;
  uint8_t clipMode, preventMathMode, addSubscreen, subtractColor, halfColor;
  uint8_t mathEnabled[6];
  uint8_t fixedColorR, fixedColorG, fixedColorB;
  uint8_t forcedBlank, brightness, mode, bg3priority;
  uint8_t pseudoHires, directColor, m7largeField, m7charFill, m7xFlip, m7yFlip, m7extBg;
  uint8_t screenEnabled[2], screenWindowed[2];
  uint8_t extraLeftCur, extraRightCur, extraLeftRight;
  uint8_t lineHasSprites, evenFrameWhenObjInterlace;
} PpuLineCacheState;

enum { kLineCacheBuckets = 4, kLineCacheVramShift = 8,
       kLineCacheVramPages = 0x8000 >> kLineCacheVramShift,
       kLineCacheVramWords = kLineCacheVramPages / 32 };
typedef struct PpuLineCacheStats {
  uint32_t total, hits;
} PpuLineCacheStats;

static PpuLineCacheState g_line_cache_state[kLineHistoryLines], g_line_cache_current;
static uint32_t g_line_cache_vram_dep[kLineHistoryLines][kLineCacheVramWords], g_line_cache_cgram_dep[kLineHistoryLines][8];
static uint32_t g_line_cache_oam_dep[kLineHistoryLines][4], g_line_cache_cur_vram[kLineCacheVramWords];
static uint32_t g_line_cache_vram_last[kLineCacheVramPages], g_line_cache_cgram_last[256];
static uint32_t g_line_cache_oam_last[128];
static uint32_t g_line_cache_vram_serial, g_line_cache_cgram_serial, g_line_cache_oam_serial;
static uint32_t g_line_cache_vram_at[kLineHistoryLines], g_line_cache_cgram_at[kLineHistoryLines];
static uint32_t g_line_cache_oam_at[kLineHistoryLines], g_line_cache_frame;
static uint8_t g_line_cache_valid[kLineHistoryLines];
static uint8_t g_line_cache_cooldown[kLineHistoryLines];
static bool g_line_cache_tracking;
static PpuLineCacheStats g_line_cache_stats[kLineCacheBuckets + 1];

static inline bool PpuLineCacheBeginLine(int y) {
  if (g_line_cache_cooldown[y]) {
    g_line_cache_cooldown[y]--;
    return false;
  }
  return true;
}

static inline void PpuLineCacheMiss(int y) {
  g_line_cache_valid[y] = 0;
}

static void PpuLineCacheCapture(PpuLineCacheState *s, const Ppu *ppu) {
  memset(s, 0, sizeof(*s));
  memcpy(s->bgLayer, ppu->bgLayer, sizeof(s->bgLayer));
  memcpy(s->m7matrix, ppu->m7matrix, sizeof(s->m7matrix));
  memcpy(s->mathEnabled, ppu->mathEnabled, sizeof(s->mathEnabled));
  memcpy(s->screenEnabled, ppu->screenEnabled, sizeof(s->screenEnabled));
  memcpy(s->screenWindowed, ppu->screenWindowed, sizeof(s->screenWindowed));
  s->windowsel = ppu->windowsel;
  s->objTileAdr1 = ppu->objTileAdr1; s->objTileAdr2 = ppu->objTileAdr2;
  s->objPriority = ppu->objPriority; s->objSize = ppu->objSize;
  s->objInterlace = ppu->objInterlace; s->oamAdr = ppu->oamAdr;
  s->mosaicSize = ppu->mosaicSize; s->mosaicStartLine = ppu->mosaicStartLine;
  s->mosaicEnabled = ppu->mosaicEnabled;
  s->window1left = ppu->window1left; s->window1right = ppu->window1right;
  s->window2left = ppu->window2left; s->window2right = ppu->window2right;
  s->clipMode = ppu->clipMode; s->preventMathMode = ppu->preventMathMode;
  s->addSubscreen = ppu->addSubscreen; s->subtractColor = ppu->subtractColor;
  s->halfColor = ppu->halfColor;
  s->fixedColorR = ppu->fixedColorR; s->fixedColorG = ppu->fixedColorG;
  s->fixedColorB = ppu->fixedColorB;
  s->forcedBlank = ppu->forcedBlank; s->brightness = ppu->brightness;
  s->mode = ppu->mode; s->bg3priority = ppu->bg3priority;
  s->pseudoHires = ppu->pseudoHires; s->directColor = ppu->directColor;
  s->m7largeField = ppu->m7largeField; s->m7charFill = ppu->m7charFill;
  s->m7xFlip = ppu->m7xFlip; s->m7yFlip = ppu->m7yFlip; s->m7extBg = ppu->m7extBg;
  s->extraLeftCur = ppu->extraLeftCur; s->extraRightCur = ppu->extraRightCur;
  s->extraLeftRight = ppu->extraLeftRight; s->lineHasSprites = ppu->lineHasSprites;
  s->evenFrameWhenObjInterlace = ppu->objInterlace ? ppu->evenFrame : 0;
}

static inline void PpuLineCacheBump(uint32_t *serial, uint32_t *last) {
  uint32_t next = *serial + 1;
  if (next == 0) {
    memset(g_line_cache_valid, 0, sizeof(g_line_cache_valid));
    memset(g_line_cache_vram_last, 0, sizeof(g_line_cache_vram_last));
    memset(g_line_cache_cgram_last, 0, sizeof(g_line_cache_cgram_last));
    memset(g_line_cache_oam_last, 0, sizeof(g_line_cache_oam_last));
    next = 1;
  }
  *serial = *last = next;
}

static inline void PpuLineCacheVram(uint32_t adr) {
  if (g_line_cache_tracking)
    g_line_cache_cur_vram[(adr >> (kLineCacheVramShift + 5)) & (kLineCacheVramWords - 1)] |=
        1u << ((adr >> kLineCacheVramShift) & 31);
}

static bool PpuLineCacheChanged(const uint32_t *dep, int words,
                                const uint32_t *last, uint32_t at) {
  for (int w = 0; w < words; w++) {
    uint32_t bits = dep[w];
    while (bits) {
      int bit = __builtin_ctz(bits);
      if (last[w * 32 + bit] > at) return true;
      bits &= bits - 1;
    }
  }
  return false;
}

/* Compare PpuLineCacheState excluding oamAdr (write-only pointer, no visual
 * effect) and m7matrix (only relevant in mode 7; games write dummy values
 * every frame in other modes).  Probe-validated: this unlocks ~96.6% cache
 * hit rate during ALttP gameplay with 0 false positives.
 *
 * Word-wise compare (uint32_t stride). Safe because Capture() memsets the
 * struct to 0 before field assignment, so padding bytes are zero on both
 * sides. m7matrix spans whole words (int16[8] = 16 B = 4 words) and is
 * skipped entirely when mode!=7. oamAdr is a single byte inside a word that
 * also holds objPriority/objSize/objInterlace — masked out, not skipped. */
_Static_assert(sizeof(PpuLineCacheState) % 4 == 0,
               "PpuLineCacheState must be word-sized for word-wise compare");
static bool PpuLineCacheRegsMatch(const PpuLineCacheState *a, const PpuLineCacheState *b) {
  const uint32_t *wa = (const uint32_t *)a;
  const uint32_t *wb = (const uint32_t *)b;
  size_t m7_w0 = offsetof(PpuLineCacheState, m7matrix) / 4;
  size_t m7_wn = m7_w0 + sizeof(a->m7matrix) / 4;
  size_t oam_w = offsetof(PpuLineCacheState, oamAdr) / 4;
  uint32_t oam_mask = ~(0xFFu << (8 * (offsetof(PpuLineCacheState, oamAdr) % 4)));
  size_t nwords = sizeof(PpuLineCacheState) / 4;
  bool skip_m7 = (a->mode != 7 && b->mode != 7);
  for (size_t i = 0; i < nwords; i++) {
    if (skip_m7 && i >= m7_w0 && i < m7_wn) continue;
    uint32_t xa = wa[i], xb = wb[i];
    if (i == oam_w) { xa &= oam_mask; xb &= oam_mask; }
    if (xa != xb) return false;
  }
  return true;
}

/* Direct field-by-field comparison of live PPU state against the stored cache
 * entry, WITHOUT capturing into a temp PpuLineCacheState first.
 *
 * On cache-HIT lines (96.6%), this avoids:
 *   - memset(s, 0, sizeof(*s))        — 120 bytes zeroed
 *   - 5 memcpy()s for bgLayer/m7matrix/mathEnabled/screenEnabled/screenWindowed
 *   - ~30 scalar field assignments
 * Total ~250 bytes of memory writes skipped per hit line.
 *
 * Compares the SAME fields as PpuLineCacheCapture + PpuLineCacheRegsMatch, with
 * the SAME exclusions: oamAdr (write-only OAM pointer, no visual effect) is
 * never compared, m7matrix is skipped when mode!=7 (ALttP writes dummy values
 * every frame). Probe-validated exclusion — see the word-wise variant above.
 *
 * Field order mirrors Capture() exactly. Returns true if all compared fields
 * match. */
static bool PpuLineCacheMatchPpu(const Ppu *p, const PpuLineCacheState *s) {
  /* bgLayer[4] */
  if (memcmp(p->bgLayer, s->bgLayer, sizeof(s->bgLayer)) != 0) return false;
  /* m7matrix[8] — skip when mode!=7 (game writes dummy values every frame) */
  if (p->mode == 7 && memcmp(p->m7matrix, s->m7matrix, sizeof(s->m7matrix)) != 0) return false;
  /* mathEnabled[6] */
  if (memcmp(p->mathEnabled, s->mathEnabled, sizeof(s->mathEnabled)) != 0) return false;
  /* screenEnabled[6] */
  if (memcmp(p->screenEnabled, s->screenEnabled, sizeof(s->screenEnabled)) != 0) return false;
  /* screenWindowed[6] */
  if (memcmp(p->screenWindowed, s->screenWindowed, sizeof(s->screenWindowed)) != 0) return false;
  /* Scalar fields — grouped for readability, order matches Capture */
  if (p->windowsel != s->windowsel) return false;
  if (p->objTileAdr1 != s->objTileAdr1) return false;
  if (p->objTileAdr2 != s->objTileAdr2) return false;
  if (p->objPriority != s->objPriority) return false;
  if (p->objSize != s->objSize) return false;
  if (p->objInterlace != s->objInterlace) return false;
  /* oamAdr: SKIPPED — write-only OAM write pointer, PPU reads all 128 entries
   * via ppu_evaluateSprites, oamAdr is irrelevant to rendered pixels. */
  if (p->mosaicSize != s->mosaicSize) return false;
  if (p->mosaicStartLine != s->mosaicStartLine) return false;
  if (p->mosaicEnabled != s->mosaicEnabled) return false;
  if (p->window1left != s->window1left) return false;
  if (p->window1right != s->window1right) return false;
  if (p->window2left != s->window2left) return false;
  if (p->window2right != s->window2right) return false;
  if (p->clipMode != s->clipMode) return false;
  if (p->preventMathMode != s->preventMathMode) return false;
  if (p->addSubscreen != s->addSubscreen) return false;
  if (p->subtractColor != s->subtractColor) return false;
  if (p->halfColor != s->halfColor) return false;
  if (p->fixedColorR != s->fixedColorR) return false;
  if (p->fixedColorG != s->fixedColorG) return false;
  if (p->fixedColorB != s->fixedColorB) return false;
  if (p->forcedBlank != s->forcedBlank) return false;
  if (p->brightness != s->brightness) return false;
  if (p->mode != s->mode) return false;
  if (p->bg3priority != s->bg3priority) return false;
  if (p->pseudoHires != s->pseudoHires) return false;
  if (p->directColor != s->directColor) return false;
  if (p->m7largeField != s->m7largeField) return false;
  if (p->m7charFill != s->m7charFill) return false;
  if (p->m7xFlip != s->m7xFlip) return false;
  if (p->m7yFlip != s->m7yFlip) return false;
  if (p->m7extBg != s->m7extBg) return false;
  if (p->extraLeftCur != s->extraLeftCur) return false;
  if (p->extraRightCur != s->extraRightCur) return false;
  if (p->extraLeftRight != s->extraLeftRight) return false;
  if (p->lineHasSprites != s->lineHasSprites) return false;
  if (p->objInterlace && p->evenFrame != s->evenFrameWhenObjInterlace) return false;
  return true;
}

static bool PpuLineCacheCanReuse(Ppu *ppu, int line, bool eligible) {
  int y = line - 1;
  uint32_t bucket = (g_line_cache_frame - 1) / 300;
  if (bucket >= kLineCacheBuckets) bucket = kLineCacheBuckets - 1;
  g_line_cache_stats[bucket].total++;
  g_line_cache_stats[kLineCacheBuckets].total++;
  if (!eligible)
    return false;
  PpuLineCacheCapture(&g_line_cache_current, ppu);
  if (!g_line_cache_valid[y])
    return false;
  if (ppu->renderPitch == 0
#ifdef TARGET_GNW
      || g_ppu_line_cb != NULL
#endif
      ||
      !PpuLineCacheRegsMatch(&g_line_cache_current, &g_line_cache_state[y])) {
    PpuLineCacheMiss(y);
    return false;
  }
  if (PpuLineCacheChanged(g_line_cache_vram_dep[y], kLineCacheVramWords,
                          g_line_cache_vram_last,
                          g_line_cache_vram_at[y])) {
    PpuLineCacheMiss(y);
    return false;
  }
  if (PpuLineCacheChanged(g_line_cache_cgram_dep[y], 8, g_line_cache_cgram_last,
                          g_line_cache_cgram_at[y])) {
    PpuLineCacheMiss(y);
    return false;
  }
  uint32_t oam_union[4];
  for (int w = 0; w < 4; w++) oam_union[w] = g_line_cache_oam_dep[y][w] | ppu->objLineCand[y][w];
  if (PpuLineCacheChanged(oam_union, 4, g_line_cache_oam_last,
                          g_line_cache_oam_at[y])) {
    PpuLineCacheMiss(y);
    return false;
  }
  g_line_cache_stats[bucket].hits++;
  g_line_cache_stats[kLineCacheBuckets].hits++;
  return true;
}

static void PpuLineCacheCommit(Ppu *ppu, int line) {
  int y = line - 1;
  g_line_cache_state[y] = g_line_cache_current;
  memcpy(g_line_cache_vram_dep[y], g_line_cache_cur_vram, sizeof(g_line_cache_cur_vram));
  memcpy(g_line_cache_oam_dep[y], ppu->objLineCand[y], sizeof(g_line_cache_oam_dep[y]));
  memset(g_line_cache_cgram_dep[y], 0, sizeof(g_line_cache_cgram_dep[y]));
  uint32_t math_enabled = 0;
  for (int layer = 0; layer < 6; layer++) math_enabled |= ppu->mathEnabled[layer] << layer;
  bool uses_subscreen = ppu->preventMathMode != 3 && ppu->addSubscreen &&
      math_enabled && ppu->screenEnabled[1] != 0;
  for (int x = 0; x < kPpuXPixels; x++) {
    uint8_t index = ppu->bgBuffers[0].data[x] & 0xff;
    g_line_cache_cgram_dep[y][index >> 5] |= 1u << (index & 31);
    if (uses_subscreen) {
      index = ppu->bgBuffers[1].data[x] & 0xff;
      g_line_cache_cgram_dep[y][index >> 5] |= 1u << (index & 31);
    }
  }
  g_line_cache_vram_at[y] = g_line_cache_vram_serial;
  g_line_cache_cgram_at[y] = g_line_cache_cgram_serial;
  g_line_cache_oam_at[y] = g_line_cache_oam_serial;
  g_line_cache_valid[y] = 1;
}

void ppu_lineCacheReport(void) {
  static const char *const names[kLineCacheBuckets + 1] = {
    "0-299", "300-599", "600-899", "900-1199", "all"
  };
  for (int b = 0; b <= kLineCacheBuckets; b++) {
    const PpuLineCacheStats *s = &g_line_cache_stats[b];
    printf("[line-cache] bucket=%s total=%llu hits=%llu hit_x10000=%llu metadata=%u\n",
      names[b], (unsigned long long)s->total, (unsigned long long)s->hits,
      (unsigned long long)(s->total ? (uint64_t)s->hits * 10000 / s->total : 0),
      (unsigned)(sizeof(g_line_cache_state) + sizeof(g_line_cache_current) +
                 sizeof(g_line_cache_vram_dep) +
                 sizeof(g_line_cache_cgram_dep) + sizeof(g_line_cache_oam_dep) +
                 sizeof(g_line_cache_cur_vram) +
                 sizeof(g_line_cache_vram_last) + sizeof(g_line_cache_cgram_last) +
                 sizeof(g_line_cache_oam_last) + sizeof(g_line_cache_vram_at) +
                 sizeof(g_line_cache_cgram_at) + sizeof(g_line_cache_oam_at) +
                 sizeof(g_line_cache_valid) + sizeof(g_line_cache_cooldown) +
                 sizeof(g_line_cache_vram_serial) + sizeof(g_line_cache_cgram_serial) +
                 sizeof(g_line_cache_oam_serial) + sizeof(g_line_cache_frame) +
                 sizeof(g_line_cache_tracking) + sizeof(g_line_cache_stats)));
  }
}

void ppu_lineCacheInvalidate(void) {
  memset(g_line_cache_valid, 0, sizeof(g_line_cache_valid));
  memset(g_line_cache_cooldown, 0, sizeof(g_line_cache_cooldown));
}
#endif

#if defined(SNES_LINE_REUSE_PROBE) || defined(SNES_LINE_CACHE)
static inline void PpuTrackVramAdr(uint32_t adr) {
#ifdef SNES_LINE_REUSE_PROBE
  PpuLineProbeVram(adr);
#endif
#ifdef SNES_LINE_CACHE
  PpuLineCacheVram(adr);
#endif
}
static inline uint16_t PpuTrackVramPtr(Ppu *ppu, const uint16_t *ptr) {
  PpuTrackVramAdr((uint32_t)(ptr - ppu->vram) & 0x7fff);
  return *ptr;
}
#define PPU_PROBE_VRAM_ADR(adr) PpuTrackVramAdr((adr) & 0x7fff)
#define PPU_PROBE_VRAM_PTR(ppu, ptr) PpuTrackVramPtr((ppu), (ptr))
#else
#define PPU_PROBE_VRAM_ADR(adr) ((void)0)
#define PPU_PROBE_VRAM_PTR(ppu, ptr) (*(ptr))
#endif

void ppu_runLine(Ppu* ppu, int line) {
  if(line == 0) {
#ifdef SNES_LINE_REUSE_PROBE
    g_probe_frame++;
#endif
#ifdef SNES_LINE_CACHE
    g_line_cache_frame++;
#endif
    // pre-render line
    // TODO: this now happens halfway into the first line
    ppu->mosaicStartLine = 1;
    ppu->rangeOver = false;
    ppu->timeOver = false;
    ppu->evenFrame = !ppu->evenFrame;
  } else {  
    // Cache the brightness computation
    if (ppu->brightness != ppu->lastBrightnessMult) {
      uint8_t ppu_brightness = ppu->brightness;
      ppu->lastBrightnessMult = ppu_brightness;
      for (int i = 0; i < 32; i++)
        ppu->brightnessMultHalf[i * 2] = ppu->brightnessMultHalf[i * 2 + 1] = ppu->brightnessMult[i] =
        ((i << 3) | (i >> 2)) * ppu_brightness / 15;
      // Store 31 extra entries to remove the need for clamping to 31.
      memset(&ppu->brightnessMult[32], ppu->brightnessMult[31], 31);
#ifdef PPU_RGB565
      ppu->paletteDirty = true;
#endif
    }

    // evaluate sprites. The buffer only needs wiping if the previous line put
    // something in it — most lines of most frames have no sprites at all.
#ifdef SNES_LINE_REUSE_PROBE
    memset(g_probe_cur_vram_mask, 0, sizeof(g_probe_cur_vram_mask));
#endif
#ifdef SNES_LINE_CACHE
    bool cache_eligible = ppu->mode != 7 && PpuLineCacheBeginLine(line - 1);
    g_line_cache_tracking = cache_eligible;
    if (cache_eligible)
      memset(g_line_cache_cur_vram, 0, sizeof(g_line_cache_cur_vram));
#endif
#if SNES_ABLATE_SPRITES
    /* ABLATION, WRONG OUTPUT -- AND NOT THE ONE ITS NAME CLAIMS. This `return`
     * leaves ppu_runLine before the rendering as well as before the sprites, so
     * its 60.08 fps against 56.93 prices THE WHOLE REMAINING RENDER at 3.15 fps,
     * not the sprite path. Kept, renamed in spirit, because that is a useful
     * number: it is the ceiling on everything the renderer has left.
     *
     * The sprite path itself was then priced piece by piece and is nearly free:
     * pixel emission 0 (SNES_ABLATE_SPRITE_PIX, 56.67), objBuffer wipe 0
     * (SNES_ABLATE_OBJWIPE, 56.90), merge into the bg buffer +0.33
     * (SNES_ABLATE_SPRITE_MERGE, 57.26), and the scan visits 0.93 sprites and
     * 1.64 columns PER LINE by census -- there is nothing there to remove. */
    ppu->lineHasSprites = false;
    return;
#endif
#if SNES_SKIP_SPRITE_EVAL_ON_SKIP
    if (g_ppu_skip_render && !g_stat77_read)
      return;
#endif
#if SNES_ABLATE_SKIPSPR
    /* ABLATION. On a frameskipped line, return BEFORE the sprite work instead of
     * after it. The overload guard draws one frame in four, so three quarters of
     * every second's OAM scans (128 entries x 224 lines) and objBuffer wipes are
     * done for pixels that are thrown away.
     *
     * WRONG OUTPUT is possible here in a way the rig cannot see: the rig renders
     * every frame, so it never takes this path at all. ppu_evaluateSprites also
     * sets the range/time-over flags a game can read at $213E, and lineHasSprites
     * feeds the next drawn line. This prices the idea; it does not implement it. */
    if (g_ppu_skip_render)
      return;
#endif
#if SNES_SPRITE_SKIP_DRAW
    /* With the sprite pixel emission compiled out on a frameskipped line,
     * nothing writes objBuffer on that line -- so wiping it is a wipe of
     * something already clean, and objBufferClean must not be updated either or
     * the next drawn line would trust a flag describing a line that never drew.
     * 512 bytes a line, three lines in four. */
    const bool obj_untouched = g_ppu_skip_render;
#else
    const bool obj_untouched = false;
#endif
#if SNES_ABLATE_OBJWIPE
    /* ABLATION, WRONG OUTPUT. 512 bytes a line to erase what 5.46 slivers -- about
     * 44 pixels -- actually dirtied. */
    (void)0;
#else
    if (!ppu->objBufferClean && !obj_untouched)
      ClearBackdrop(&ppu->objBuffer);
#endif
#ifdef SNES_LINE_CACHE
    if (cache_eligible && ppu->objCacheValid &&
        (ppu->objLineCand[line - 1][0] | ppu->objLineCand[line - 1][1] |
         ppu->objLineCand[line - 1][2] | ppu->objLineCand[line - 1][3]) == 0) {
      ppu->lineHasSprites = false;
    } else
#endif
    {
#if SNES_MATHFIXED_CENSUS
      g_objeval_lines++;
#endif
      SNES_PERF_PPU_BEGIN(perf_e);
      ppu->lineHasSprites = !ppu->forcedBlank && ppu_evaluateSprites(ppu, line - 1);
      SNES_PERF_PPU_END(SNES_PERF_PPU_EVAL, perf_e);
      if (!obj_untouched)
        ppu->objBufferClean = !ppu->lineHasSprites;
    }

    if (g_ppu_skip_render)
      return;   /* frameskip: the flags above still matter, the pixels below do not */

    if (g_new_ppu) {
#ifdef SNES_LINE_CACHE
      bool reused = PpuLineCacheCanReuse(ppu, line, cache_eligible);
#if SNES_RENDER_CENSUS
      /* Does the line cache earn the tracking it charges? Every VRAM access in
       * the tile loops calls PpuLineCacheVram -- a global load, a branch, two
       * shifts and a read-modify-write on a bitmap -- roughly 200 times a line.
       * That is paid whether or not a line is ever reused, and the reuse rate
       * has only ever been measured on the attract screen. */
      if (reused) g_lc_hit++; else g_lc_miss++;
#endif
      if (!reused) {
#endif
#ifdef SNES_LINE_REUSE_PROBE
        PpuLineProbeBefore(ppu, line);
#endif
        PpuDrawWholeLine(ppu, line);
#ifdef SNES_LINE_REUSE_PROBE
        PpuLineProbeAfter(ppu, line);
#endif
#ifdef SNES_LINE_CACHE
        if (cache_eligible && !g_line_cache_cooldown[line - 1])
          PpuLineCacheCommit(ppu, line);
      }
      g_line_cache_tracking = false;
#endif
    } else {
      // actual line
      if (ppu->mode == 7) ppu_calculateMode7Starts(ppu, line);
      for (int x = 0; x < 256; x++) {
        ppu_handlePixel(ppu, x, line);
      }
    }
  }
}

typedef struct PpuWindows {
  int16 edges[6];
  uint8 nr;
  uint8 bits;
} PpuWindows;

static void PpuWindows_Clear(PpuWindows *win, Ppu *ppu, uint layer) {
  win->edges[0] = -(layer != 2 ? ppu->extraLeftCur : 0);
  win->edges[1] = 256 + (layer != 2 ? ppu->extraRightCur : 0);
  win->nr = 1;
  win->bits = 0;
}

#ifndef SNES_WINDOW_CENSUS
#define SNES_WINDOW_CENSUS 0
#endif
#if SNES_WINDOW_CENSUS
/* Is the duplicate real? PpuWindows_Calc takes no `sub` argument, so a layer
 * windowed on BOTH screens computes the identical result twice per line. That is
 * a removal if it happens, and worth nothing if it does not -- so count before
 * writing a cache.
 *
 * COUNTED, AND IT DOES NOT HAPPEN. Zelda 3 rain, 28,690 rendered lines: 30,319
 * Calc calls (1.06 per line), of which the subscreen pass accounts for ZERO --
 * no layer is ever windowed on the sub screen here, so there are no duplicates
 * to remove. The call is also not a hotspot to begin with: at 1.06 per line it
 * is essentially just the colour window. Lever closed by a count, before the
 * cache that would have implemented it was written. */
uint32_t g_win_calc, g_win_calc_sub, g_win_dup, g_win_lines;
#endif

static void PpuWindows_Calc(PpuWindows *win, Ppu *ppu, uint layer) {
#if SNES_WINDOW_CENSUS
  g_win_calc++;
#endif
  // Evaluate which spans to render based on the window settings.
  // There are at most 5 windows.
  // Algorithm from Snes9x
  uint32 winflags = GET_WINDOW_FLAGS(ppu, layer);
  uint nr = 1;
  int window_right = 256 + (layer != 2 ? ppu->extraRightCur : 0);
  win->edges[0] = - (layer != 2 ? ppu->extraLeftCur : 0);
  win->edges[1] = window_right;
  uint i, j;
  int t;
  bool w1_ena = (winflags & kWindow1Enabled) && ppu->window1left <= ppu->window1right;
  if (w1_ena) {
    if (ppu->window1left > win->edges[0]) {
      win->edges[nr] = ppu->window1left;
      win->edges[++nr] = window_right;
    }
    if (ppu->window1right + 1 < window_right) {
      win->edges[nr] = ppu->window1right + 1;
      win->edges[++nr] = window_right;
    }
  }
  bool w2_ena = (winflags & kWindow2Enabled) && ppu->window2left <= ppu->window2right;
  if (w2_ena) {
    for (i = 0; i <= nr && (t = ppu->window2left) != win->edges[i]; i++) {
      if (t < win->edges[i]) {
        for (j = nr++; j >= i; j--)
          win->edges[j + 1] = win->edges[j];
        win->edges[i] = t;
        break;
      }
    }
    for (; i <= nr && (t = ppu->window2right + 1) != win->edges[i]; i++) {
      if (t < win->edges[i]) {
        for (j = nr++; j >= i; j--)
          win->edges[j + 1] = win->edges[j];
        win->edges[i] = t;
        break;
      }
    }
  }
  win->nr = nr;
  /* Per-span mask — must match ppu_getWindowState (maskLogic XOR/AND/OR). The
   * Snes9x w1_bits|w2_bits OR alone is wrong for logic != OR. */
  win->bits = 0;
  for (i = 0; i < nr; i++) {
    int x = win->edges[i];
    if (x < 0)
      x = 0;
    else if (x > 255)
      x = 255;
    if (ppu_getWindowState(ppu, (int)layer, x))
      win->bits |= (uint8)(1 << i);
  }
}

static inline uint32 PpuSpreadByteToNibbles(uint32 x) {
  /* Insert three zero bits between each source bit. Four spread bitplanes OR
   * directly into eight chunky 4bpp pixels, avoiding four extracts per pixel. */
  x = (x | x << 12) & 0x000f000f;
  x = (x | x << 6) & 0x03030303;
  return (x | x << 3) & 0x11111111;
}

/* PpuSpreadByteToNibbles(b) for every byte, as a table: the decode of a 4bpp
 * row was ~35 shift-and-mask instructions, the table makes it 4 loads and 3
 * ORs (the shifts fold into them). Generated from the function above, so the
 * two cannot disagree; PpuDecodeTableCheck() confirms it on the host. */
static const uint32 kPpuSpreadNib[256] = {
  0x00000000u, 0x00000001u, 0x00000010u, 0x00000011u, 0x00000100u, 0x00000101u, 0x00000110u, 0x00000111u,
  0x00001000u, 0x00001001u, 0x00001010u, 0x00001011u, 0x00001100u, 0x00001101u, 0x00001110u, 0x00001111u,
  0x00010000u, 0x00010001u, 0x00010010u, 0x00010011u, 0x00010100u, 0x00010101u, 0x00010110u, 0x00010111u,
  0x00011000u, 0x00011001u, 0x00011010u, 0x00011011u, 0x00011100u, 0x00011101u, 0x00011110u, 0x00011111u,
  0x00100000u, 0x00100001u, 0x00100010u, 0x00100011u, 0x00100100u, 0x00100101u, 0x00100110u, 0x00100111u,
  0x00101000u, 0x00101001u, 0x00101010u, 0x00101011u, 0x00101100u, 0x00101101u, 0x00101110u, 0x00101111u,
  0x00110000u, 0x00110001u, 0x00110010u, 0x00110011u, 0x00110100u, 0x00110101u, 0x00110110u, 0x00110111u,
  0x00111000u, 0x00111001u, 0x00111010u, 0x00111011u, 0x00111100u, 0x00111101u, 0x00111110u, 0x00111111u,
  0x01000000u, 0x01000001u, 0x01000010u, 0x01000011u, 0x01000100u, 0x01000101u, 0x01000110u, 0x01000111u,
  0x01001000u, 0x01001001u, 0x01001010u, 0x01001011u, 0x01001100u, 0x01001101u, 0x01001110u, 0x01001111u,
  0x01010000u, 0x01010001u, 0x01010010u, 0x01010011u, 0x01010100u, 0x01010101u, 0x01010110u, 0x01010111u,
  0x01011000u, 0x01011001u, 0x01011010u, 0x01011011u, 0x01011100u, 0x01011101u, 0x01011110u, 0x01011111u,
  0x01100000u, 0x01100001u, 0x01100010u, 0x01100011u, 0x01100100u, 0x01100101u, 0x01100110u, 0x01100111u,
  0x01101000u, 0x01101001u, 0x01101010u, 0x01101011u, 0x01101100u, 0x01101101u, 0x01101110u, 0x01101111u,
  0x01110000u, 0x01110001u, 0x01110010u, 0x01110011u, 0x01110100u, 0x01110101u, 0x01110110u, 0x01110111u,
  0x01111000u, 0x01111001u, 0x01111010u, 0x01111011u, 0x01111100u, 0x01111101u, 0x01111110u, 0x01111111u,
  0x10000000u, 0x10000001u, 0x10000010u, 0x10000011u, 0x10000100u, 0x10000101u, 0x10000110u, 0x10000111u,
  0x10001000u, 0x10001001u, 0x10001010u, 0x10001011u, 0x10001100u, 0x10001101u, 0x10001110u, 0x10001111u,
  0x10010000u, 0x10010001u, 0x10010010u, 0x10010011u, 0x10010100u, 0x10010101u, 0x10010110u, 0x10010111u,
  0x10011000u, 0x10011001u, 0x10011010u, 0x10011011u, 0x10011100u, 0x10011101u, 0x10011110u, 0x10011111u,
  0x10100000u, 0x10100001u, 0x10100010u, 0x10100011u, 0x10100100u, 0x10100101u, 0x10100110u, 0x10100111u,
  0x10101000u, 0x10101001u, 0x10101010u, 0x10101011u, 0x10101100u, 0x10101101u, 0x10101110u, 0x10101111u,
  0x10110000u, 0x10110001u, 0x10110010u, 0x10110011u, 0x10110100u, 0x10110101u, 0x10110110u, 0x10110111u,
  0x10111000u, 0x10111001u, 0x10111010u, 0x10111011u, 0x10111100u, 0x10111101u, 0x10111110u, 0x10111111u,
  0x11000000u, 0x11000001u, 0x11000010u, 0x11000011u, 0x11000100u, 0x11000101u, 0x11000110u, 0x11000111u,
  0x11001000u, 0x11001001u, 0x11001010u, 0x11001011u, 0x11001100u, 0x11001101u, 0x11001110u, 0x11001111u,
  0x11010000u, 0x11010001u, 0x11010010u, 0x11010011u, 0x11010100u, 0x11010101u, 0x11010110u, 0x11010111u,
  0x11011000u, 0x11011001u, 0x11011010u, 0x11011011u, 0x11011100u, 0x11011101u, 0x11011110u, 0x11011111u,
  0x11100000u, 0x11100001u, 0x11100010u, 0x11100011u, 0x11100100u, 0x11100101u, 0x11100110u, 0x11100111u,
  0x11101000u, 0x11101001u, 0x11101010u, 0x11101011u, 0x11101100u, 0x11101101u, 0x11101110u, 0x11101111u,
  0x11110000u, 0x11110001u, 0x11110010u, 0x11110011u, 0x11110100u, 0x11110101u, 0x11110110u, 0x11110111u,
  0x11111000u, 0x11111001u, 0x11111010u, 0x11111011u, 0x11111100u, 0x11111101u, 0x11111110u, 0x11111111u,
};

static inline uint32 PpuDecode4bpp(uint32 bits) {
  return kPpuSpreadNib[bits & 0xff] |
         kPpuSpreadNib[bits >> 8 & 0xff] << 1 |
         kPpuSpreadNib[bits >> 16 & 0xff] << 2 |
         kPpuSpreadNib[bits >> 24] << 3;
}

static inline uint32 PpuDecode2bpp(uint32 bits) {
  return kPpuSpreadNib[bits & 0xff] |
         kPpuSpreadNib[bits >> 8 & 0xff] << 1;
}

#ifdef HOST_BUILD
/* the table is the function, entry for entry */
__attribute__((constructor)) static void PpuDecodeTableCheck(void) {
  for (uint32 b = 0; b < 256; b++)
    if (kPpuSpreadNib[b] != PpuSpreadByteToNibbles(b)) {
      fprintf(stderr, "kPpuSpreadNib[%u] is wrong\n", (unsigned)b);
      abort();
    }
}
#endif

#ifndef SNES_ABLATE_BG
#define SNES_ABLATE_BG 0
#endif
#ifndef SNES_PPU_BLEND_LUT
#define SNES_PPU_BLEND_LUT 0
#endif
#if SNES_PPU_BLEND_LUT && defined(PPU_RGB565)
/* The blend is 97.4% of compositing pixels here, and it was paying for six
 * component extracts and three shifts it does not need.
 *
 * Counted on the device, Zelda 3 rain, in the same window everything else is
 * measured in: 8,547,757 blended pixels against 227,411 bypassed -- 249.4 of
 * every 256 on a colour-math line. The source comment above the pair experiment
 * assumed the opposite ("most pixels on a colour-math line still take the
 * one-lookup bypass"); that was true of the scene it was written for, not this
 * one. The same census also says brightness is 15 on 100% of those lines and the
 * subscreen pixel is NEVER backdrop, so `color_map` inside the blend is
 * loop-invariant -- always the half map.
 *
 * So: keep each CGRAM colour pre-split into three 11-bit-spaced fields. Two
 * spread entries ADD in one instruction with no carry between fields (62 < 2048),
 * and three tables turn each channel sum straight into its positioned RGB565
 * bits, clamping included -- the same clamp brightnessMult already does by
 * holding 31 extra entries. Component extraction, the three shifts and the
 * per-channel clamp all go.
 *
 * Derived state only, rebuilt from cgram/brightness/halfColor, so it lives in
 * statics rather than in Ppu -- a savestate is a raw struct dump and this must
 * not be in it. */
static uint32_t g_cgram_spread[256];
static uint32_t g_sub_spread[256];
/* One array, so the blend loops reach all three channel tables from a single
 * base register (three separate arrays cost a literal-pool load each, per
 * pixel, once the compositing ran out of registers). */
static uint16_t g_blend565[3][64];
#define g_blend_r565 (g_blend565[0])
#define g_blend_g565 (g_blend565[1])
#define g_blend_b565 (g_blend565[2])
static uint32_t g_blend_key = 0xffffffffu;
/* Three 5-bit channels at bits 0, 11 and 22 leave bits 28-31 free, and the two
 * conditions that send a pixel down the bypass are folded into them:
 *
 *   sub index 0  -> g_sub_spread[0] carries kBlendBypass
 *   layer >= 6   -> tested once, on the main z, and OR'd in the same way
 *
 * so the add that combines the two colours also combines the two tests, and the
 * loop asks one question instead of two. Bit 22 + 62 tops out at bit 27, so
 * nothing the colours do can reach the flag. */
enum { kBlendBypass = 1u << 30 };
#define PPU_SPREAD(c) (((c) & 0x1f) | ((((c) >> 5) & 0x1f) << 11) | ((((c) >> 10) & 0x1f) << 22))
static void PpuRebuildBlendSpread(Ppu *ppu) {
  for (int i = 0; i < 256; i++) {
    uint32 c = ppu->cgram[i];
    g_cgram_spread[i] = PPU_SPREAD(c);
    g_sub_spread[i] = PPU_SPREAD(c);
  }
  /* A subscreen pixel of index 0 is the backdrop: the SNES does not blend it, it
   * falls back to the fixed colour, which is what math_fixed already holds. */
  g_sub_spread[0] |= kBlendBypass;
}

/* the channel tables: brightness and halfColor only, not cgram */
static void PpuRebuildBlendLut(Ppu *ppu) {
  const uint8_t *map = ppu->halfColor ? ppu->brightnessMultHalf : ppu->brightnessMult;
  for (int x = 0; x < 64; x++) {
    uint32 v = map[x];
    g_blend_r565[x] = (uint16_t)((v >> 3) << 11);
    g_blend_g565[x] = (uint16_t)((v >> 2) << 5);
    g_blend_b565[x] = (uint16_t)(v >> 3);
  }
}

/* The two colour-math blend loops of the compositing, as their own functions.
 * Inline in PpuDrawWholeLine they ran out of registers and reloaded two table
 * addresses from the literal pool on every pixel; here every base stays in a
 * register for the whole span. Same arithmetic, same order: bit-identical. */
__attribute__((noinline))
static uint16_t *PpuBlendSpanAll(uint16_t *dst, const PpuZbufType *mrow, const PpuZbufType *srow,
                                 uint32 i, uint32 right, const uint16_t *math_fixed) {
  const uint32_t *cs = g_cgram_spread, *ss = g_sub_spread;
  const uint16_t *br = g_blend565[0], *bg = g_blend565[1], *bb = g_blend565[2];
  uint32 last_key = ~0u, last_out = 0;   /* see PpuBlendSpanPart */
  while (i < right) {
    uint32 main_z = mrow[i], sub_z = srow[i];
    uint32 key = (main_z & 0xff) | (sub_z & 0xff) << 8;
    uint32 sum = cs[main_z & 0xff] + ss[sub_z & 0xff];
    if ((sum & kBlendBypass) || ((main_z >> 8) & 0xf) >= 6) {
      dst[0] = math_fixed[main_z & 0x7ff];
    } else {
      if (key != last_key) {
        last_key = key;
        last_out = br[sum & 63] | bg[(sum >> 11) & 63] | bb[sum >> 22];
      }
      dst[0] = (uint16_t)last_out;
    }
    dst++, i++;
  }
  return dst;
}

__attribute__((noinline))
static uint16_t *PpuBlendSpanPart(uint16_t *dst, const PpuZbufType *mrow, const PpuZbufType *srow,
                                  uint32 i, uint32 right, const uint16_t *math_fixed,
                                  uint32 math_enabled_cur) {
  const uint32_t *cs = g_cgram_spread;
  const uint16_t *br = g_blend565[0], *bg = g_blend565[1], *bb = g_blend565[2];
  /* A blended pixel is a function of its main and sub colour indices alone
   * (the tables cannot change inside a span), and neighbours repeat: in
   * Zelda's title 86% of blended pixels have the same pair as the pixel
   * before. Remember the last pair and its result. */
  uint32 last_key = ~0u, last_out = 0;
  while (i < right) {
    uint32 main_z = mrow[i], sub_z = srow[i];
    if (!(math_enabled_cur & (1u << ((main_z >> 8) & 0xf))) || (sub_z & 0xff) == 0) {
      dst[0] = math_fixed[main_z & 0x7ff];
    } else {
      uint32 key = (main_z & 0xff) | (sub_z & 0xff) << 8;
      if (key != last_key) {
        last_key = key;
        uint32 sum = cs[main_z & 0xff] + cs[sub_z & 0xff];
        last_out = br[sum & 63] | bg[(sum >> 11) & 63] | bb[(sum >> 22) & 63];
      }
      dst[0] = (uint16_t)last_out;
    }
    dst++, i++;
  }
  return dst;
}
#endif
#ifndef SNES_COMP_CENSUS
#define SNES_COMP_CENSUS 0
#endif
#if SNES_COMP_CENSUS
/* Which way do the compositing pixels actually go in the scene being measured?
 * The bypass is one table lookup; the blend is two palette loads, six extracts,
 * three clamped adds and three brightness lookups. Everything about how to
 * attack this loop depends on the ratio, and the source comment's guess ("most
 * pixels on a colour-math line still take the bypass") was made for a different
 * scene than the one the device is benchmarked in. */
uint32_t g_comp_bypass, g_comp_blend, g_comp_subzero, g_comp_lines, g_comp_bright;
#endif
#ifndef SNES_ABLATE_SKIPSPR
#define SNES_ABLATE_SKIPSPR 0
#endif
#ifndef SNES_ABLATE_COMPOSITE
#define SNES_ABLATE_COMPOSITE 0
#endif
#ifndef SNES_PPU_VIRGIN_Z
#define SNES_PPU_VIRGIN_Z 0
#endif
#if SNES_PPU_VIRGIN_Z
/* The first layer written into a z-buffer cannot lose the z test.
 *
 * ClearBackdrop fills both bg buffers with 0x0500 at the top of every drawn
 * line, so until something else has written to that buffer, `z > dstz[i]` is
 * true for every pixel of any layer whose z floor is above 0x0500 -- every
 * BG layer in modes 0-1 and 3-6 (Mode 0 BG4 prio 0 is 0x13xx, not 0x03xx).
 * The load, the
 * compare and its branch are then three instructions per pixel spent proving
 * something already known.
 *
 * This is worth having because of what the buffer looks like: the main screen
 * runs 2.29 layer passes a line and the subscreen exactly 1.00, so roughly
 * three of every 3.29 passes... no: exactly one pass per screen is the first
 * one, which is 2 of 3.29 -- 61% of all passes, and the first pass is also the
 * one with the fewest transparent pixels, since it is the base layer.
 *
 * The 2bpp drawer already had this store, for a different reason: `top_mask`
 * marks mode 1's BG3 as above every other priority, so its high-priority tiles
 * skip the compare. The virgin case just widens when that path is legal.
 *
 * Conservative in the safe direction: any pass that is entered marks the buffer
 * dirty whether or not it writes a pixel, so the flag can only ever say "not
 * virgin" too early, never too late. */
static uint8_t g_bg_dirty;
#define PPU_BUF_MARK(sub)      (g_bg_dirty |= 1u << (sub))
#define PPU_BUF_CLEAN(sub)     (g_bg_dirty &= ~(1u << (sub)))
#define PPU_BUF_VIRGIN(sub)    (!(g_bg_dirty & (1u << (sub))))
enum { kPpuBackdropZ = 0x0500 };
/* Store two z-buffer entries in one word. The address may be odd-halfword
 * aligned -- a window edge decides it -- which Cortex-M7 permits for word
 * accesses; only 64-bit ones trap (CLAUDE.md, the Super Metroid STRD). memcpy
 * says that in C without letting the compiler assume alignment it does not
 * have -- but gcc does NOT fold a 4-byte memcpy with an unknown-alignment
 * destination into a str. It emits a CALL, and seven of them landed inside
 * PpuDrawBackground_4bpp. The branch they sit in never executes in Zelda 3, and
 * it still cost **4.7 fps on hardware** (52.38 against 57.10, three runs each):
 * a call in the hottest loop makes the compiler treat every caller-saved
 * register as clobbered across it, and the loop that has to survive that is the
 * one that runs tens of thousands of times a frame. An aligned(1) may_alias
 * store says the same thing to the compiler and compiles to one str. */
#else
#define PPU_BUF_MARK(sub)      ((void)0)
#define PPU_BUF_CLEAN(sub)     ((void)0)
#define PPU_BUF_VIRGIN(sub)    0
#endif
#ifndef SNES_PPU_COARSE_SKIP
#define SNES_PPU_COARSE_SKIP 0
#endif
#if SNES_PPU_COARSE_SKIP
/* One test that drops four pixels, instead of four tests that drop one each.
 *
 * SNES_ABLATE_BG=6 prices the pixel work at 4.4 fps, and SNES_PPU_SIMD_PIXELS
 * proved that most of it is not arithmetic: doing all eight pixels
 * unconditionally, two lanes at a time and branchless, LOSES 7.5 fps, because
 * the per-pixel test was skipping constantly. So keep the skip and make it
 * coarser.
 *
 * `chunky` holds eight 4-bit pixels, nibble i for pixel i. Its low halfword is
 * pixels 0-3 and its high halfword pixels 4-7, so a single AND says whether four
 * consecutive pixels are all transparent -- and when they are, four nibble
 * extracts, four tests and four branches collapse into one. The flipped drawer
 * takes its pixels in reverse nibble order, so its halves swap.
 *
 * The shape that keeps winning on this chip is a test that skips something
 * LARGE. Four pixels is large; one pixel was not. */
#define PPU_IF_LOW4   if (chunky & 0x0000ffffu)
#define PPU_IF_HIGH4  if (chunky & 0xffff0000u)
#else
#define PPU_IF_LOW4
#define PPU_IF_HIGH4
#endif
#ifndef SNES_PPU_SIMD_PIXELS
#define SNES_PPU_SIMD_PIXELS 0
#endif
#if SNES_PPU_SIMD_PIXELS
/* Two z-buffer pixels at a time, branchless, on the M7's halfword SIMD.
 *
 * The per-pixel rule is `if (pixel && z > dstz[i]) dstz[i] = z + pixel;` on
 * uint16 -- a compare-and-select, which is what USUB16 (GE flags per halfword)
 * and SEL (pick per halfword by those flags) do two lanes at a time. The
 * transparent case folds into the same compare: substitute 0 for z where the
 * pixel nibble is zero, and `d >= 0` is always true, so that lane keeps its old
 * value with no extra test.
 *
 * USUB16 and SEL have to stay in one asm block: the GE flags live in APSR and
 * nothing guarantees the compiler will not schedule an instruction between two
 * separate statements.
 *
 * MEASURED, AND IT LOSES BADLY: 48.06 against a 55.57 baseline, -7.5 fps, with
 * the rig hashes bit-identical so it is the same picture. The reason is the
 * thing it deleted. `if (pixel && z > dstz[i])` is a test that SKIPS, and on
 * this scene it skips constantly -- 46% of tiles are blank outright and a great
 * many pixels inside the rest are transparent. Doing two lanes unconditionally
 * pays for every transparent pixel in the frame. The pair version is not slower
 * per pixel drawn; it draws pixels that the branch version never touched.
 *
 * So the 4.4 fps is not "eight pixels of arithmetic to vectorise". Much of it is
 * the skipping machinery itself, and anything that replaces a skip with
 * unconditional work loses. A coarser skip -- one test that drops four pixels at
 * once, e.g. `(chunky & 0xffff) == 0` -- is the shape that could still win, and
 * it is the next thing to build. (An unaligned 32-bit dstz access may be part of
 * this loss too; not separated, and it cannot account for 7.5 fps on its own.)
 *
 * dstz is uint16* and a window edge can be odd, so the 32-bit accesses go
 * through memcpy -- gcc emits a plain LDR/STR, and ARMv7-M handles an unaligned
 * word access in hardware. Only 64-bit accesses trap on this core. */
static inline uint32 PpuRd32(const void *p) { uint32 v; memcpy(&v, p, 4); return v; }
static inline void PpuWr32(void *p, uint32 v) { memcpy(p, &v, 4); }
#define PPU_SIMD_PAIR(i, s0, s1) do {                                          \
    uint32 pp_ = ((chunky >> (s0)) & 0xf) | (((chunky >> (s1)) & 0xf) << 16);  \
    uint32 dd_ = PpuRd32(dstz + (i)), vv_, zp_, rr_;                           \
    __asm volatile ("uadd16 %0, %1, %2" : "=r"(vv_) : "r"(zz), "r"(pp_));      \
    __asm volatile ("usub16 %0, %1, %2\n\tsel %0, %3, %4"                      \
                    : "=&r"(zp_) : "r"(pp_), "r"(kOne16), "r"(zz), "r"(0)      \
                    : "cc");                                                   \
    __asm volatile ("usub16 %0, %1, %2\n\tsel %0, %3, %4"                      \
                    : "=&r"(rr_) : "r"(dd_), "r"(zp_), "r"(dd_), "r"(vv_)      \
                    : "cc");                                                   \
    PpuWr32(dstz + (i), rr_);                                                  \
  } while (0)
#define PPU_SIMD_TILE_4BPP() do {                                              \
    uint32 zz = (uint32)z | (uint32)z << 16;                                   \
    if (tile & 0x4000) {                                                       \
      PPU_SIMD_PAIR(0, 0, 4);   PPU_SIMD_PAIR(2, 8, 12);                       \
      PPU_SIMD_PAIR(4, 16, 20); PPU_SIMD_PAIR(6, 24, 28);                      \
    } else {                                                                   \
      PPU_SIMD_PAIR(0, 28, 24); PPU_SIMD_PAIR(2, 20, 16);                      \
      PPU_SIMD_PAIR(4, 12, 8);  PPU_SIMD_PAIR(6, 4, 0);                        \
    }                                                                          \
  } while (0)
enum { kOne16 = 0x00010001 };
#endif
#ifndef SNES_PPU_PIPELINE
#define SNES_PPU_PIPELINE 0
#endif
#ifndef SNES_ABLATE_WALK
#define SNES_ABLATE_WALK 0
#endif
#ifndef SNES_ABLATE_FETCH
#define SNES_ABLATE_FETCH 0
#endif
#ifndef SNES_ABLATE_ADDR
#define SNES_ABLATE_ADDR 0
#endif
#if SNES_ABLATE_BG == 6
#define PPU_ABLATE_KEEP_BITS(b) do { g_ppu_ablate_sink = (b); } while (0)
#else
#define PPU_ABLATE_KEEP_BITS(b) do { } while (0)
#endif
#if SNES_ABLATE_BG == 4 || SNES_ABLATE_BG == 6
/* The setup ablation computes the per-call setup and then throws it away, which
 * is exactly the shape gcc deletes. Everything it wants to keep is summed into
 * this volatile, so the arm measures the setup rather than an empty call. */
volatile uint32 g_ppu_ablate_sink;
#endif
#ifndef SNES_PPU_PREFETCH
#define SNES_PPU_PREFETCH 0
#endif
#ifndef SNES_VRAM_IN_DTCM
#define SNES_VRAM_IN_DTCM 0
#endif
#ifndef SNES_PPU_TILE_MEMO
#define SNES_PPU_TILE_MEMO 0
#endif
#ifndef SNES_RENDER_CENSUS
#define SNES_RENDER_CENSUS 0
#endif
#if SNES_RENDER_CENSUS
/* What the 17.65 ms of render is actually made of. The frame histogram gives the
 * total; the PC profile gives symbols; neither says how many layer passes a line
 * runs, how many of those are the SUB screen (the pass that only exists because
 * colour math is on), or how many tiles in a pass are transparent and skipped.
 * Count all four before designing anything. Read over SWD.
 *
 * COUNTED, Zelda 3 rain, 339,259 rendered lines:
 *
 *                     main      sub
 *   layer passes/line  2.29     1.00     sub runs on 100% of lines
 *   tiles/line        65.1     33.0
 *   blank tiles        46%       0%      main skips nearly half for free
 *   decoded/line      35.2     33.0      <- SUB IS 48% OF ALL TILE DECODE
 *
 * The subscreen exists only because colour math is on, draws a single layer,
 * and that layer is fully opaque -- not one tile of it is transparent, so
 * nothing is skipped. It costs as much decoding as the entire main screen.
 * That is the render's shape, and any structural work on the renderer should
 * start there rather than in the compositing loop, which pairing and range-test
 * deletion have already been through. */
uint32_t g_bg_pass[2], g_bg_tile[2], g_bg_tile_blank[2], g_spr_pass, g_render_lines, g_sub_lines;
/* Of the tiles that are NOT blank, how many are FULLY opaque -- every one of the
 * eight nibbles non-zero. On such a tile the per-pixel `if (pixel)` is provably
 * true eight times out of eight, so it is work that is ALWAYS useless, which is
 * the only shape that has ever won on this loop. Two SIMD attempts lost because
 * they removed a skip that was paying; this asks how often there is nothing to
 * skip. Counted for the 4bpp drawer, which is where the pixels are. */
uint32_t g_tile_full[2], g_tile_mixed[2];
uint32_t g_tile_flat[2], g_tile_opq_z[2];
uint32_t g_t2_full[2], g_t2_mixed[2];
uint64_t g_tile_opaque_px[2];
/* The one that decides whether a shared decode is even possible: when the sub
 * pass draws layer N, was layer N also drawn on the main screen? hScroll and
 * vScroll live on the layer, not the screen, so the same layer fetches and
 * decodes IDENTICAL tiles in both passes -- only the destination buffer and the
 * window differ. If this is high, half the tile decode is literally duplicate
 * work. If it is zero, the sub screen draws something the main screen does not
 * and there is nothing to share.
 *
 * COUNTED: 42,191 sub passes, of which **0** draw a layer the main screen also
 * draws. The sub screen is never a second pass over the same content -- it is a
 * different layer entirely. So the "single-pass main+sub" idea has nothing to
 * collapse, and the subscreen's 33 tiles per line are irreducible work in this
 * design, not duplication. Closed before it was written. */
uint32_t g_sub_also_main, g_sub_only;
#endif

/* 16×16 tilemap entries cover four 8×8 characters: +1 for the right half
 * (swapped if hflip), +0x10 for the bottom half (swapped if vflip). Same
 * addressing as ppu_getPixelForBgLayer, so the fast drawers can keep their
 * 8-pixel walk. */
static inline uint32 PpuBgCharFromMap(uint32 tile, uint x, uint y, bool big) {
  uint32 n = tile & 0x3ff;
  if (big) {
    if (((bool)(x & 8)) ^ ((bool)(tile & 0x4000))) n += 1;
    if (((bool)(y & 8)) ^ ((bool)(tile & 0x8000))) n += 0x10;
  }
  return n;
}

/* Mosaic ($2106): sample the top-left of each N×N screen block and splat into
 * the z-buffer. X is aligned to 0, Y to mosaicStartLine — same as
 * ppu_getPixel. Palette encoding matches the 2/4/8bpp fast drawers (Mode 0
 * CGRAM windows live in zhi/zlo). Called only when mosaicSize > 1. */
static void PpuDrawBackground_mosaic(Ppu *ppu, uint y, bool sub, uint layer,
                                     PpuZbufType zhi, PpuZbufType zlo, int bpp) {
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer)
                                      : PpuWindows_Clear(&win, ppu, layer);
  PPU_BUF_MARK(sub);

  int size = ppu->mosaicSize;
  int sy = (int)y - ((int)y - (int)ppu->mosaicStartLine) % size;
  BgLayer *bglayer = &ppu->bgLayer[layer];
  int ly = (sy + bglayer->vScroll) & 0x3ff;
  bool big = bglayer->bigTiles;
  bool wide = big || ppu->mode == 5 || ppu->mode == 6;
  int pal_shift = (bpp == 4) ? 6 : 8;
  int tile_pitch = 4 * bpp;

  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;
    int x = win.edges[windex];
    int x2 = win.edges[windex + 1];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + x + kPpuExtraLeftRight;
    int w = size - (x % size);
    do {
      w = IntMin(w, x2 - x);
      int lx = (x - (x % size) + bglayer->hScroll) & 0x3ff;

      int tileBitsX = wide ? 4 : 3;
      int tileHighBitX = wide ? 0x200 : 0x100;
      int tileBitsY = big ? 4 : 3;
      int tileHighBitY = big ? 0x200 : 0x100;
      uint16_t tilemapAdr = (uint16_t)(bglayer->tilemapAdr
          + (((ly >> tileBitsY) & 0x1f) << 5 | ((lx >> tileBitsX) & 0x1f)));
      if ((lx & tileHighBitX) && bglayer->tilemapWider)
        tilemapAdr += 0x400;
      if ((ly & tileHighBitY) && bglayer->tilemapHigher)
        tilemapAdr += bglayer->tilemapWider ? 0x800 : 0x400;
      uint16_t tile = ppu->vram[tilemapAdr & 0x7fff];

      int row = (tile & 0x8000) ? 7 - (ly & 7) : (ly & 7);
      int col = (tile & 0x4000) ? (lx & 7) : 7 - (lx & 7);
      uint32 tileNum = PpuBgCharFromMap(tile, (uint)lx, (uint)ly, big);
      unsigned adr = (bglayer->tileAdr + tileNum * (unsigned)tile_pitch + (unsigned)row) & 0x7fff;
      uint16_t plane1 = ppu->vram[adr];
      int pixel = (plane1 >> col) & 1;
      pixel |= ((plane1 >> (8 + col)) & 1) << 1;
      if (bpp >= 4) {
        uint16_t plane2 = ppu->vram[(adr + 8) & 0x7fff];
        pixel |= ((plane2 >> col) & 1) << 2;
        pixel |= ((plane2 >> (8 + col)) & 1) << 3;
      }
      if (bpp >= 8) {
        uint16_t plane3 = ppu->vram[(adr + 16) & 0x7fff];
        pixel |= ((plane3 >> col) & 1) << 4;
        pixel |= ((plane3 >> (8 + col)) & 1) << 5;
        uint16_t plane4 = ppu->vram[(adr + 24) & 0x7fff];
        pixel |= ((plane4 >> col) & 1) << 6;
        pixel |= ((plane4 >> (8 + col)) & 1) << 7;
      }

      if (pixel) {
        PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
        if (bpp < 8)
          z += (tile & 0x1c00) >> pal_shift;
        PpuZbufType zv = z + (PpuZbufType)pixel;
        int i = 0;
        do {
          if (z > dstz[i])
            dstz[i] = zv;
        } while (++i != w);
      }
    } while (x += w, dstz += w, w = size, x < x2);
  }
}

/* Mode 2/4 offset-per-tile: each 8px screen column can replace H and/or V
 * scroll from BG3. One tile fetch per column. Mode 4 packs H/V in one word
 * (ppu_handleOPT). column 0 never applies OPT. */
static void PpuDrawBackground_opt(Ppu *ppu, uint y, bool sub, uint layer,
                                  PpuZbufType zhi, PpuZbufType zlo, int bpp) {
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer)
                                      : PpuWindows_Clear(&win, ppu, layer);
  PPU_BUF_MARK(sub);
  BgLayer *bglayer = &ppu->bgLayer[layer];
  bool big = bglayer->bigTiles;
  int pal_shift = (bpp == 4) ? 6 : 8;
  unsigned pitch = (unsigned)(4 * bpp);

  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;
    int x = win.edges[windex];
    int x2 = win.edges[windex + 1];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + x + kPpuExtraLeftRight;
    int run;
    do {
      run = IntMin(8 - (x & 7), x2 - x);
      int lx = x + bglayer->hScroll;
      int ly = (int)y + bglayer->vScroll;
      ppu_handleOPT(ppu, (int)layer, &lx, &ly);
      lx &= 0x3ff;
      ly &= 0x3ff;

      int tileBitsX = big ? 4 : 3;
      int tileHighBitX = big ? 0x200 : 0x100;
      int tileBitsY = big ? 4 : 3;
      int tileHighBitY = big ? 0x200 : 0x100;
      uint16_t tilemapAdr = (uint16_t)(bglayer->tilemapAdr
          + (((ly >> tileBitsY) & 0x1f) << 5 | ((lx >> tileBitsX) & 0x1f)));
      if ((lx & tileHighBitX) && bglayer->tilemapWider)
        tilemapAdr += 0x400;
      if ((ly & tileHighBitY) && bglayer->tilemapHigher)
        tilemapAdr += bglayer->tilemapWider ? 0x800 : 0x400;
      uint16_t tile = ppu->vram[tilemapAdr & 0x7fff];

      int row = (tile & 0x8000) ? 7 - (ly & 7) : (ly & 7);
      uint32 tileNum = PpuBgCharFromMap(tile, (uint)lx, (uint)ly, big);
      unsigned adr = (bglayer->tileAdr + tileNum * pitch + (unsigned)row) & 0x7fff;
      uint32 bits = ppu->vram[adr];
      uint32 bits_hi = 0;
      if (bpp >= 4)
        bits |= (uint32)ppu->vram[(adr + 8) & 0x7fff] << 16;
      if (bpp >= 8)
        bits_hi = ppu->vram[(adr + 16) & 0x7fff]
                | (uint32)ppu->vram[(adr + 24) & 0x7fff] << 16;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      if (bpp < 8)
        z += (tile & 0x1c00) >> pal_shift;

      for (int i = 0; i < run; i++) {
        int col = (tile & 0x4000) ? ((lx + i) & 7) : 7 - ((lx + i) & 7);
        int pixel = ((bits >> col) & 1) | ((bits >> (7 + col)) & 2);
        if (bpp >= 4)
          pixel |= ((bits >> (14 + col)) & 4) | ((bits >> (21 + col)) & 8);
        if (bpp >= 8)
          pixel |= (((bits_hi >> col) & 1) | ((bits_hi >> (7 + col)) & 2)
                 | ((bits_hi >> (14 + col)) & 4) | ((bits_hi >> (21 + col)) & 8)) << 4;
        if (pixel && z > dstz[i])
          dstz[i] = z + (PpuZbufType)pixel;
      }
    } while (x += run, dstz += run, x < x2);
  }
}

/* Mode 5/6: 512-wide layer, 256-wide output. LakeSnes samples
 * lx = (x + hofs)*2 + (sub ? 0 : 1) so the main screen keeps the odd
 * hires pixels (the RGB565 path never averages the pair). Tiles are
 * always 16 pixels wide in that 512-space; 16×16 if bigTiles. */
static void PpuDrawBackground_hires(Ppu *ppu, uint y, bool sub, uint layer,
                                    PpuZbufType zhi, PpuZbufType zlo, int bpp) {
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer)
                                      : PpuWindows_Clear(&win, ppu, layer);
  PPU_BUF_MARK(sub);
  BgLayer *bglayer = &ppu->bgLayer[layer];
  bool big = bglayer->bigTiles;
  bool mosaic = IS_MOSAIC_ENABLED(ppu, layer) && ppu->mosaicSize > 1;
  int phase = (sub || mosaic) ? 0 : 1;
  int pal_shift = (bpp == 4) ? 6 : 8;
  unsigned tile_pitch = (unsigned)(4 * bpp);
  bool do_opt = ppu->mode == 6;

  /* Streaming path for Mode 5 without mosaic, OPT or interlace: walk the
   * tilemap sequentially (like the 4bpp drawer) and subsample every other
   * hires pixel via chunky decode instead of per-pixel bit extraction. */
  if (!mosaic && !do_opt && !ppu->interlace && !big) {
    int ly = ((int)y + (int)bglayer->vScroll) & 0x3ff;
    int sc_row = (ly >> 3) & 0x1f;
    int sc_y_hi = (ly & 0x100) && bglayer->tilemapHigher;
    int tile_row_n = bglayer->tilemapAdr + (sc_row << 5);
    int tile_row_w = sc_y_hi ? (bglayer->tilemapWider ? 0x800 : 0x400) : 0;

    for (size_t windex = 0; windex < win.nr; windex++) {
      if (win.bits & (1 << windex))
        continue;
      int x = win.edges[windex];
      int x2 = win.edges[windex + 1];
      PpuZbufType *dstz = ppu->bgBuffers[sub].data + x + kPpuExtraLeftRight;

      while (x < x2) {
        int lx = ((x + (int)bglayer->hScroll) * 2 + phase) & 0x3ff;
        int sc_col = (lx >> 4) & 0x1f;
        uint16_t tma = (uint16_t)(tile_row_n + sc_col);
        if ((lx & 0x200) && bglayer->tilemapWider)
          tma += 0x400;
        tma += tile_row_w;
        uint16_t tile = ppu->vram[tma & 0x7fff];

        int row = (tile & 0x8000) ? 7 - (ly & 7) : (ly & 7);
        uint32 tileNum = tile & 0x3ff;
        if (((bool)(lx & 8)) ^ ((bool)(tile & 0x4000))) tileNum += 1;
        unsigned adr = (bglayer->tileAdr + tileNum * tile_pitch + (unsigned)row) & 0x7fff;
        uint32 bits = ppu->vram[adr];
        if (bpp >= 4)
          bits |= (uint32)ppu->vram[(adr + 8) & 0x7fff] << 16;

        int run = IntMin((9 - (lx & 7)) / 2, x2 - x);

        if (bits) {
          uint32 chunky = (bpp >= 4) ? PpuDecode4bpp(bits) : PpuDecode2bpp(bits);
          PpuZbufType z = ((tile & 0x2000) ? zhi : zlo)
                        + ((tile & 0x1c00) >> pal_shift);
          if (tile & 0x4000) {
            for (int i = 0; i < run; i++) {
              int pixel = (chunky >> (4 * ((lx + 2 * i) & 7))) & 0xf;
              if (pixel && z > dstz[i])
                dstz[i] = z + (PpuZbufType)pixel;
            }
          } else {
            for (int i = 0; i < run; i++) {
              int pixel = (chunky >> (4 * (7 - ((lx + 2 * i) & 7)))) & 0xf;
              if (pixel && z > dstz[i])
                dstz[i] = z + (PpuZbufType)pixel;
            }
          }
        }

        x += run;
        dstz += run;
      }
    }
    return;
  }

  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;
    int x = win.edges[windex];
    int x2 = win.edges[windex + 1];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + x + kPpuExtraLeftRight;
    int run;
    do {
      int sx = x, sy = (int)y;
      if (mosaic) {
        sx -= sx % ppu->mosaicSize;
        sy -= (sy - (int)ppu->mosaicStartLine) % ppu->mosaicSize;
      }
      int lx = (sx + (int)bglayer->hScroll) * 2 + phase;
      int ly = sy;
      if (ppu->interlace) {
        ly *= 2;
        ly += (ppu->evenFrame || mosaic) ? 0 : 1;
      }
      ly += (int)bglayer->vScroll;
      if (do_opt)
        ppu_handleOPT(ppu, (int)layer, &lx, &ly);
      lx &= 0x3ff;
      ly &= 0x3ff;

      int tileBitsY = big ? 4 : 3;
      int tileHighBitY = big ? 0x200 : 0x100;
      uint16_t tilemapAdr = (uint16_t)(bglayer->tilemapAdr
          + (((ly >> tileBitsY) & 0x1f) << 5 | ((lx >> 4) & 0x1f)));
      if ((lx & 0x200) && bglayer->tilemapWider)
        tilemapAdr += 0x400;
      if ((ly & tileHighBitY) && bglayer->tilemapHigher)
        tilemapAdr += bglayer->tilemapWider ? 0x800 : 0x400;
      uint16_t tile = ppu->vram[tilemapAdr & 0x7fff];

      int row = (tile & 0x8000) ? 7 - (ly & 7) : (ly & 7);
      uint32 tileNum = tile & 0x3ff;
      if (((bool)(lx & 8)) ^ ((bool)(tile & 0x4000))) tileNum += 1;
      if (big && (((bool)(ly & 8)) ^ ((bool)(tile & 0x8000)))) tileNum += 0x10;
      unsigned adr = (bglayer->tileAdr + tileNum * tile_pitch + (unsigned)row) & 0x7fff;
      uint32 bits = ppu->vram[adr];
      if (bpp >= 4)
        bits |= (uint32)ppu->vram[(adr + 8) & 0x7fff] << 16;

      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      z += (tile & 0x1c00) >> pal_shift;

      if (mosaic) {
        run = IntMin(ppu->mosaicSize - (x % ppu->mosaicSize), x2 - x);
        int col = (tile & 0x4000) ? (lx & 7) : 7 - (lx & 7);
        int pixel = ((bits >> col) & 1) | ((bits >> (7 + col)) & 2);
        if (bpp >= 4)
          pixel |= ((bits >> (14 + col)) & 4) | ((bits >> (21 + col)) & 8);
        if (pixel) {
          PpuZbufType zv = z + (PpuZbufType)pixel;
          for (int i = 0; i < run; i++) {
            if (z > dstz[i])
              dstz[i] = zv;
          }
        }
      } else {
        run = IntMin((9 - (lx & 7)) / 2, x2 - x);
        for (int i = 0; i < run; i++) {
          int col = (tile & 0x4000) ? ((lx + 2 * i) & 7) : 7 - ((lx + 2 * i) & 7);
          int pixel = ((bits >> col) & 1) | ((bits >> (7 + col)) & 2);
          if (bpp >= 4)
            pixel |= ((bits >> (14 + col)) & 4) | ((bits >> (21 + col)) & 8);
          if (pixel && z > dstz[i])
            dstz[i] = z + (PpuZbufType)pixel;
        }
      }
    } while (x += run, dstz += run, x < x2);
  }
}

/* The full-tile middle of a 4bpp layer pass, 8x8 tiles, as its own function.
 * Inside PpuDrawBackground_4bpp it shared registers with the window loop, the
 * clipped edges and the 16x16 path, and reloaded and stored about nine values
 * on the stack per tile; here the walk state, the two tile bases and the two
 * z bases stay in registers for the whole run. The body is the production
 * path of the loop it replaces (OPAQUE_TILE on, every ablation and census
 * off), expression for expression, so the output is the same. The tilemap
 * walk state goes in and comes back through `tw`. */
#if SNES_PPU_OPAQUE_TILE && !SNES_ABLATE_BG && !SNES_PPU_PIPELINE && !SNES_PPU_TILE_MEMO && \
    !SNES_PPU_PREFETCH && !SNES_RENDER_CENSUS && !SNES_PPU_SIMD_PIXELS && !SNES_ABLATE_WALK && \
    !SNES_ABLATE_ADDR && !SNES_ABLATE_FETCH && !SNES_PPU_COARSE_SKIP && \
    !defined(SNES_LINE_CACHE) && !defined(SNES_LINE_REUSE_PROBE)
#define SNES_PPU_BG4_RUN 1
typedef struct { const uint16 *tp, *last, *next; } PpuTileWalk;

__attribute__((noinline))
static PpuZbufType *PpuBg4FullTiles(const uint16 *vram, PpuTileWalk *tw, PpuZbufType *dstz, uint n,
                                    uint32 tileadrs, uint32 zs, bool no_ztest) {
  /* tileadrs = tileadr1 << 16 | tileadr0 and zs = zhi << 16 | zlo: one
   * register each instead of two stack slots. The pixel steps consume the
   * decoded word one nibble at a time (c >>= 4 / c <<= 4), so one value is
   * live instead of eight precomputed extracts that spilled. */
  const uint16 *tp = tw->tp, *tp_last = tw->last, *tp_next = tw->next;
#define R_STEP(i, nib, adv, TEST, STORE) do { PpuZbufType *d_ = &dstz[i]; uint32 px_ = (nib); adv; if (TEST) *d_ = STORE; } while (0)
#define R_RUN(nib, adv, TEST, STORE) do { \
    R_STEP(0, nib, adv, TEST, STORE); R_STEP(1, nib, adv, TEST, STORE); \
    R_STEP(2, nib, adv, TEST, STORE); R_STEP(3, nib, adv, TEST, STORE); \
    R_STEP(4, nib, adv, TEST, STORE); R_STEP(5, nib, adv, TEST, STORE); \
    R_STEP(6, nib, adv, TEST, STORE); R_STEP(7, nib, adv, TEST, STORE); } while (0)
/* non-flipped tiles (bit 0x4000) read nibble i = low first; flipped read 7-i = high first */
#define R_LO c & 0xf
#define R_LO_ADV c >>= 4
#define R_HI c >> 28
#define R_HI_ADV c <<= 4
  do {
    uint32 tile = *tp;
    if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31;
    int ta = (tile & 0x8000) ? (int)(tileadrs >> 16) : (int)(tileadrs & 0xffff);
    PpuZbufType z = (tile & 0x2000) ? (PpuZbufType)(zs >> 16) : (PpuZbufType)zs;
    const uint16 *addr = &vram[(ta + (tile & 0x3ff) * 16) & 0x7fff];
    uint32 bits = addr[0] | addr[8] << 16;
    if (bits) {
      uint32 c = PpuDecode4bpp(bits);
      z += ((tile & 0x1c00) >> 6);
      const uint32 nz = (c | c >> 1 | c >> 2 | c >> 3) & 0x11111111u;
      if (nz == 0x11111111u) {
        if (no_ztest) {
          if (tile & 0x4000) R_RUN(R_LO, R_LO_ADV, 1, z + px_);
          else               R_RUN(R_HI, R_HI_ADV, 1, z + px_);
        } else if (tile & 0x4000) {
          R_RUN(R_LO, R_LO_ADV, z > *d_, z + px_);
        } else {
          R_RUN(R_HI, R_HI_ADV, z > *d_, z + px_);
        }
      } else if (no_ztest) {
        if (tile & 0x4000) R_RUN(R_LO, R_LO_ADV, px_, z + px_);
        else               R_RUN(R_HI, R_HI_ADV, px_, z + px_);
      } else if (tile & 0x4000) {
        R_RUN(R_LO, R_LO_ADV, px_ && z > *d_, z + px_);
      } else {
        R_RUN(R_HI, R_HI_ADV, px_ && z > *d_, z + px_);
      }
    }
    dstz += 8;
  } while (--n);
#undef R_STEP
#undef R_RUN
#undef R_LO
#undef R_LO_ADV
#undef R_HI
#undef R_HI_ADV
  tw->tp = tp, tw->last = tp_last, tw->next = tp_next;
  return dstz;
}
#else
#define SNES_PPU_BG4_RUN 0
#endif

// Draw a whole line of a 4bpp background layer into bgBuffers
static void PpuDrawBackground_4bpp(Ppu *ppu, uint y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo) {
/* SNES_ABLATE_BG=2: keep the tilemap walk and the VRAM fetch, delete only the
 * decode, the z-compare and the store. That is exactly the part a Thumb-2
 * rewrite of this loop could reach -- =1 also deletes the walk and the fetch,
 * which it could not -- so the two ablations bracket what the project is worth.
 * With the pixel macros empty, `chunky` is dead and gcc removes the decode too.
 * WRONG OUTPUT, frame counter only. */
#if SNES_ABLATE_BG == 6
/* ABLATION, WRONG OUTPUT ON PURPOSE, and the one =2 was supposed to be.
 *
 * =2 empties the pixel macros and calls itself "decode, z-compare and store
 * deleted, walk and fetch kept". In the middle loop that is not what it
 * compiles to: DO_CHUNKY_PIXEL touches neither `chunky` nor `bits`, so `bits`
 * is dead, so READ_BITS is dead, and BOTH VRAM loads and the tilemap load go
 * with them -- =2 quietly becomes =1. Measured today, and that is exactly what
 * it does: 59.94, against =1's 59.80 and =4's 59.73.
 *
 * Here the fetch is kept alive by one volatile store per tile, the cheapest
 * thing a compiler may not delete. So this really is the pixel work alone. */
#define DO_PIXEL(i)              do { g_ppu_ablate_sink = bits; } while (0)
#define DO_PIXEL_HFLIP(i)        do { g_ppu_ablate_sink = bits; } while (0)
#define DO_CHUNKY_PIXEL(i)       do { } while (0)
#define DO_CHUNKY_PIXEL_HFLIP(i) do { } while (0)
#elif SNES_ABLATE_BG == 2
#define DO_PIXEL(i)              do { (void)bits; } while (0)
#define DO_PIXEL_HFLIP(i)        do { (void)bits; } while (0)
#define DO_CHUNKY_PIXEL(i)       do { } while (0)
#define DO_CHUNKY_PIXEL_HFLIP(i) do { } while (0)
#else
#define DO_PIXEL(i) do { \
  pixel = (bits >> i) & 1 | (bits >> (7 + i)) & 2 | (bits >> (14 + i)) & 4 | (bits >> (21 + i)) & 8; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_PIXEL_HFLIP(i) do { \
  pixel = (bits >> (7 - i)) & 1 | (bits >> (14 - i)) & 2 | (bits >> (21 - i)) & 4 | (bits >> (28 - i)) & 8; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_CHUNKY_PIXEL(i) do { \
  pixel = (chunky >> (4 * i)) & 0xf; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_CHUNKY_PIXEL_HFLIP(i) do { \
  pixel = (chunky >> (4 * (7 - i))) & 0xf; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_TOP_CHUNKY_PIXEL(i) do { \
  pixel = (chunky >> (4 * i)) & 0xf; \
  if (pixel) dstz[i] = z + pixel; } while (0)
#define DO_TOP_CHUNKY_PIXEL_HFLIP(i) do { \
  pixel = (chunky >> (4 * (7 - i))) & 0xf; \
  if (pixel) dstz[i] = z + pixel; } while (0)
/* A FULLY OPAQUE tile: every nibble non-zero, so `if (pixel)` is provably true
 * eight times out of eight. Onto a virgin buffer the z compare cannot fail
 * either, and the pixel becomes an unconditional store -- no test, no load, no
 * branch. Counted on the rig, Zelda 3: 38% of main-screen tiles and 86% of
 * SUB-screen ones, and the subscreen is 65% of all tile decode and runs exactly
 * one pass a line, so it is virgin whenever the line has no sprites.
 *
 * This is the shape that has won on this loop every time and the one that lost
 * every time it was violated: SNES_PPU_SIMD_PIXELS removed a skip that was
 * paying (-7.5 fps) and SNES_PPU_COARSE_SKIP tried to predict which pixels were
 * transparent (+0.22, noise). Here nothing is predicted -- the tile has been
 * measured, once, in four ALU ops. */
#define DO_FLAT_CHUNKY_PIXEL(i)       do { \
  dstz[i] = z + ((chunky >> (4 * i)) & 0xf); } while (0)
#define DO_FLAT_CHUNKY_PIXEL_HFLIP(i) do { \
  dstz[i] = z + ((chunky >> (4 * (7 - i))) & 0xf); } while (0)
#define DO_OPAQUE_CHUNKY_PIXEL(i) do { \
  pixel = (chunky >> (4 * i)) & 0xf; \
  if (z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_OPAQUE_CHUNKY_PIXEL_HFLIP(i) do { \
  pixel = (chunky >> (4 * (7 - i))) & 0xf; \
  if (z > dstz[i]) dstz[i] = z + pixel; } while (0)
#endif
#ifndef SNES_PPU_OPAQUE_TILE
#define SNES_PPU_OPAQUE_TILE 0
#endif
/* CLOSED, with the number: the same opaque-tile path in the 2bpp drawer (BG3 in
 * mode 1) measured **56.90 against 56.95** on hardware, three runs each -- fully
 * inside the spread. The rig had already said why: BG3 is not enabled in the
 * window it runs and every 2bpp census counter reads zero, so there was no
 * stimulus to price. In a loop where an unexecuted branch costs 0.8 fps, a path
 * that measures nothing is a liability rather than a neutral, so it is not
 * carried. g_t2_full/g_t2_mixed remain under SNES_RENDER_CENSUS for whoever
 * finds a scene that does drive BG3. */
#if SNES_PPU_OPAQUE_TILE && (SNES_ABLATE_BG || SNES_PPU_SIMD_PIXELS)
#error "SNES_PPU_OPAQUE_TILE changes the pixel loop an ablation is trying to hold still"
#endif
#if SNES_ABLATE_ADDR
/* ABLATION, WRONG OUTPUT ON PURPOSE. Both loads still happen, at scattered
 * addresses across the same 64 KB, at the same rate -- but the address no longer
 * comes from the tilemap word, so the CHASE is broken while the memory traffic
 * is not. Read against SNES_ABLATE_FETCH, which deletes the loads outright:
 * equal means the cost is the dependency, not the loads; zero means the cost is
 * the loads, and the DTCM result needs explaining. */
#define READ_BITS(ta, tile) (addr = &ppu->vram[(((uintptr_t)dstz * 37u) >> 1) & 0x7ff0], addr[0] | addr[8] << 16)
#elif SNES_ABLATE_FETCH
/* ABLATION, WRONG OUTPUT ON PURPOSE. The bitplane read is a POINTER CHASE: the
 * tilemap word is loaded, its low bits pick an address, and that address is
 * loaded. Moving all of VRAM into zero-wait DTCM proved the second load never
 * waits on memory -- but a load that hits L1 still has a use latency, and the
 * chain is two of them deep per tile with the pixel work hanging off the end.
 * This keeps `bits` a function of the same inputs, so nothing hoists and every
 * downstream branch still varies, and computes it in the ALU instead. */
#define READ_BITS(ta, tile) (addr = ppu->vram, (((uint32)(ta) + (uint32)(tile) * 16u) * 0x00010001u))
#else
#define READ_BITS(ta, tile) (PPU_PROBE_VRAM_ADR((ta) + (tile) * 16), addr = &ppu->vram[((ta) + (tile) * 16) & 0x7fff], addr[0] | addr[8] << 16)
#endif
#if SNES_PPU_PIPELINE && (SNES_PPU_TILE_MEMO || SNES_PPU_PREFETCH || SNES_RENDER_CENSUS)
#error "SNES_PPU_PIPELINE does not carry the memo/prefetch/census paths -- all three measured zero and were not duplicated into it"
#endif
#define PPU_PIPE_DRAW_4BPP(tile_, bits_) do {                                  \
    if (bits_) {                                                               \
      PPU_ABLATE_KEEP_BITS(bits_);                                             \
      uint32 chunky = PpuDecode4bpp(bits_);                                    \
      PpuZbufType z = (((tile_) & 0x2000) ? zhi : zlo)                         \
                    + (((tile_) & 0x1c00) >> kPaletteShift);                   \
      if ((tile_) & 0x4000) {                                                  \
        DO_CHUNKY_PIXEL(0); DO_CHUNKY_PIXEL(1); DO_CHUNKY_PIXEL(2); DO_CHUNKY_PIXEL(3); \
        DO_CHUNKY_PIXEL(4); DO_CHUNKY_PIXEL(5); DO_CHUNKY_PIXEL(6); DO_CHUNKY_PIXEL(7); \
      } else {                                                                 \
        DO_CHUNKY_PIXEL_HFLIP(0); DO_CHUNKY_PIXEL_HFLIP(1); DO_CHUNKY_PIXEL_HFLIP(2); DO_CHUNKY_PIXEL_HFLIP(3); \
        DO_CHUNKY_PIXEL_HFLIP(4); DO_CHUNKY_PIXEL_HFLIP(5); DO_CHUNKY_PIXEL_HFLIP(6); DO_CHUNKY_PIXEL_HFLIP(7); \
      }                                                                        \
    }                                                                          \
  } while (0)
  enum { kPaletteShift = 6 };
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  if (IS_MOSAIC_ENABLED(ppu, layer) && ppu->mosaicSize > 1) {
    PpuDrawBackground_mosaic(ppu, y, sub, layer, zhi, zlo, 4);
    return;
  }
#if SNES_PPU_VIRGIN_Z
  const bool no_ztest = PPU_BUF_VIRGIN(sub) && (uint32)zlo > (uint32)kPpuBackdropZ;
#else
  const bool no_ztest = false;
#endif
  PPU_BUF_MARK(sub);
#if SNES_ABLATE_BG == 1
  /* ABLATION, WRONG OUTPUT ON PURPOSE. Not an optimisation -- it deletes the
   * entire background layer draw (tilemap walk, VRAM fetch, decode, z-compare,
   * store) so the device can price the ceiling of ever rewriting that inner
   * loop. If "BG rendering is free" is worth little, the hand-written assembly
   * project behind it is worth less, and nobody has to spend days finding out.
   * The screen shows backdrop; the frame counter is the only valid reading.
   *
   * MEASURED: **59.54 fps against a 52.36 baseline -- +7.18, +13.7%.** Zelda 3
   * rain, 900 deterministic frames, three runs. That is the ceiling of making
   * background tile rendering free, and it is 3.5x everything else won on this
   * core in a day of A/Bs.
   *
   * Note what this says about the profile: PC sampling scored
   * PpuDrawBackground_4bpp at 6.1% of the frame, and deleting it is worth 13.7%.
   * Sampling puts a stall on whichever instruction is retiring, so work that is
   * mostly waiting on memory reads lighter than it is. Price a candidate by
   * ablation, not by its share of the histogram.
   *
   * What a rewrite could actually capture is less than 7.18: this deletes the
   * tilemap walk and VRAM fetch too, which SIMD does not touch. Say a third to a
   * half of it. Still the largest number on the board. */
  return;
#endif
#if SNES_RENDER_CENSUS
  g_bg_pass[sub ? 1 : 0]++;
  if (sub) { if (IS_SCREEN_ENABLED(ppu, 0, layer)) g_sub_also_main++; else g_sub_only++; }
#endif
  PpuWindows win;
#if SNES_WINDOW_CENSUS
  if (sub && IS_SCREEN_WINDOWED(ppu, 1, layer)) {
    g_win_calc_sub++;
    if (IS_SCREEN_WINDOWED(ppu, 0, layer)) g_win_dup++;
  }
#endif
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  BgLayer *bglayer = &ppu->bgLayer[layer];
  y += bglayer->vScroll;
  const bool big = bglayer->bigTiles;
  const int y_shift = big ? 4 : 3;
  const int y_screen = big ? 0x200 : 0x100;
  int sc_offs = bglayer->tilemapAdr + (((y >> y_shift) & 0x1f) << 5);
  if ((y & y_screen) && bglayer->tilemapHigher)
    sc_offs += bglayer->tilemapWider ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (bglayer->tilemapWider ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = ppu->bgLayer[layer].tileAdr, pixel;
  int tileadr1 = tileadr + 7 - (y & 0x7), tileadr0 = tileadr + (y & 0x7);
#if SNES_ABLATE_BG == 4
  /* ABLATION, WRONG OUTPUT ON PURPOSE. Everything above this line stays -- the
   * enabled/windowed tests, PpuWindows_Calc or _Clear, the tilemap base and the
   * two row addresses -- and everything below it goes. It prices the PER-CALL
   * SETUP on its own, against =1 (the whole draw) and =2 (the setup and the walk
   * and the fetch, without the pixel work). 737 calls a frame at 3.29 layer
   * passes a line; if that is where the 4.33 fps lives, the target is the caller,
   * not the inner loop.
   *
   * The sink is what keeps it honest: with the results unused, gcc deletes the
   * setup and the arm measures nothing but the call. */
  g_ppu_ablate_sink = (uint32)win.nr + (uint32)win.edges[0]
                    + (uint32)(uintptr_t)tps[0] + (uint32)(uintptr_t)tps[1]
                    + (uint32)tileadr1 + (uint32)tileadr0;
  return;
#endif
  const uint16 *addr;
#if SNES_PPU_TILE_MEMO
  /* One-entry memo on the tile fetch.
   *
   * Ablation says what this loop costs and what it does not: deleting the
   * decode, the z-compare and the store is worth nothing, while deleting the
   * VRAM reads as well is worth +4.33 fps. So the read IS the loop. Each 4bpp
   * tile takes two halfwords sixteen bytes apart -- one 32-byte line -- chosen
   * by a tilemap entry, i.e. a random index into 64 KB behind a 16 KB cache.
   *
   * Consecutive tilemap entries in flat areas repeat, and the subscreen here
   * draws one fully opaque layer of 33 tiles a line with not a single
   * transparent tile. When the key repeats, both loads and the decode go.
   *
   * This is a test-to-skip, the shape that usually loses on this part -- but
   * what it skips is two cache-missing loads, not three instructions, which is
   * the same reason the DSP idle fast paths were worth keeping. The device
   * decides.
   *
   * IT DECIDED: NOTHING. Six runs each against six baseline runs, back to back
   * -- memo 55.60, baseline 55.58. And the 80% hit rate explains why rather than
   * contradicting it: consecutive identical tilemap entries read the SAME 32-byte
   * cache line again immediately, which the D-cache already serves at full speed.
   * The memo removes an L1 hit, not a miss. Whatever the ablation's +4.33 fps is,
   * it lives in the 20% of tiles that are genuinely different -- cold lines -- and
   * in the tilemap walk, and no memo reaches either. Left off. */
  uint32 memo_key = 0x10000u, memo_bits = 0;   /* 0x10000 is not a 16-bit tile word */
#endif
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    uint x = win.edges[windex] + bglayer->hScroll;
    uint w = win.edges[windex + 1] - win.edges[windex];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + win.edges[windex] + kPpuExtraLeftRight;
    const int x_shift = big ? 4 : 3;
    const int x_screen = big ? 9 : 8;
    const uint16 *tp = tps[x >> x_screen & 1] + ((x >> x_shift) & 0x1f);
    const uint16 *tp_last = tps[x >> x_screen & 1] + 31;
    const uint16 *tp_next = tps[(x >> x_screen & 1) ^ 1];
#if SNES_ABLATE_WALK
/* ABLATION, WRONG OUTPUT ON PURPOSE. `tp` never advances, so every tile in the
 * span is the same tilemap entry: the decode, the z-compare, the store and both
 * VRAM loads all still happen, and only the WALK -- the pointer bump, the
 * end-of-screen compare and its branch -- is gone. Against =2 (which keeps the
 * walk and deletes the pixel work, and measured nothing) this is the other half
 * of the same question, and one of the two has to hold the 4.33 fps.
 *
 * It does contaminate the read: the same address every time is a guaranteed
 * cache hit. That is acceptable here only because four independent experiments
 * have already priced the reads at zero, up to and including moving all 64 KB of
 * VRAM into zero-wait DTCM. */
#define NEXT_TP() do { } while (0)
#else
#define NEXT_TP() if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31;
#endif
#define NEXT_MAP() do { if (!big || (x & 8)) { NEXT_TP(); } } while (0)
    // Handle clipped pixels on left side
    if (x & 7) {
      int curw = IntMin(8 - (x & 7), w);
      int clip = curw;
      w -= curw;
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_MAP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
#if SNES_PPU_TILE_MEMO
      /* Compare the raw tilemap word, not a constructed key. `ta` is chosen by
       * bit 15 of that same word and `tile & 0x3ff` is its low bits, so equal
       * words mean the same row of the same tile -- and the word is already in a
       * register. One compare, no shift, no or. */
      uint32 bits;
      if (tile == memo_key) {
        bits = memo_bits;
      } else {
        bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
        memo_key = tile, memo_bits = bits;
      }
#else
      uint32 bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
#endif
#if SNES_RENDER_CENSUS
      { static uint32 prev_key; uint32 key = (ta << 10) | (tile & 0x3ff);
        if (key == prev_key) g_tile_same++; else g_tile_diff++;
        prev_key = key; }
      g_bg_tile[sub ? 1 : 0]++;
      if (!bits) g_bg_tile_blank[sub ? 1 : 0]++;
#endif
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          bits >>= (x & 7);
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --curw);
        } else {
          bits <<= (x & 7);
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --curw);
        }
      } else {
        dstz += clip;
      }
      x += clip;
    }
    // Handle full tiles in the middle
#if SNES_PPU_PIPELINE
    /* RETRACTED DIAGNOSIS, kept because the arms are worth more than the wrong
     * conclusion was. These were built to break a "pointer chase" that turned
     * out not to be the cost:
     *
     *   depth 1  55.53   depth 2  55.33   two-pass batched  55.46
     *   against a 55.57 baseline -- all three nothing.
     *
     * They measure nothing because the fetch they reorder is nearly free. What
     * misled them was SNES_ABLATE_FETCH (+3.04) and SNES_ABLATE_ADDR (+2.09),
     * and both are contaminated: they change `bits`, which changes how many
     * tiles are blank and how many pixels are non-zero, so they move the PIXEL
     * work while claiming to price the fetch.
     *
     * SNES_ABLATE_BG=6 settles it -- fetch kept alive by a volatile store, only
     * the decode and the eight compare-and-stores deleted: 59.96, the whole
     * 4.4 fps. The rig agrees to the instruction: the pixel work is 663,322 of
     * the layer draw's 685,758 instructions a frame. It is not stalled, it is
     * not a chain, it is ordinary work in ordinary quantity. */
    if (w >= 8) {
      uint n = w >> 3;
      w -= n << 3;
#if SNES_PPU_PIPELINE == 1
      /* Depth 1: the next tile's whole chain is issued before this tile's pixel
       * work. MEASURED 55.53 against a 55.57 baseline -- NOTHING, and the reason
       * is that it moved the chain without shortening it: the tilemap load and
       * the bitplane load that depends on it are still back to back, so the
       * second still waits on the first. Kept for the record. */
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_TP();
      uint32 bits = READ_BITS((tile & 0x8000) ? tileadr1 : tileadr0, tile & 0x3ff);
      while (--n) {
        uint32 ntile = PPU_PROBE_VRAM_PTR(ppu, tp);
        NEXT_TP();
        uint32 nbits = READ_BITS((ntile & 0x8000) ? tileadr1 : tileadr0, ntile & 0x3ff);
        PPU_PIPE_DRAW_4BPP(tile, bits);
        dstz += 8;
        tile = ntile, bits = nbits;
      }
      PPU_PIPE_DRAW_4BPP(tile, bits);
      dstz += 8;
#elif SNES_PPU_PIPELINE == 3
      /* Two passes over the span instead of one interleaved loop.
       *
       * Pass 1 is nothing but the chase, and -- this is the whole point -- each
       * iteration is independent of the last. Tile i's tilemap read does not
       * wait on tile i-1's bitplane read, so three or four chases can be in
       * flight at once with only two live values to keep, which is few enough
       * that the register allocator has no reason to spill. Pass 2 then touches
       * no VRAM at all.
       *
       * This is also the decisive probe for the hand-written Thumb-2 loop: if
       * independent chases cannot overlap even with the pressure removed, then
       * scheduling them by hand cannot help either, and that project is dead
       * before it starts.
       *
       * The buffers are sized by the span: kPpuXPixels is 256 and
       * kPpuExtraLeftRight is 0, so a window span is at most 256 pixels and n is
       * at most 32. */
      uint32 bitsbuf[34];
      uint16 tilebuf[34];
      for (uint i = 0; i < n; i++) {
        uint32 t = PPU_PROBE_VRAM_PTR(ppu, tp);
        NEXT_TP();
        tilebuf[i] = t;
        bitsbuf[i] = READ_BITS((t & 0x8000) ? tileadr1 : tileadr0, t & 0x3ff);
      }
      for (uint i = 0; i < n; i++) {
        uint32 t = tilebuf[i];
        PPU_PIPE_DRAW_4BPP(t, bitsbuf[i]);
        dstz += 8;
      }
#else
      /* Depth 2: every link of the chase separated by a full iteration. It
       * cost 5,186 instructions a frame in spills and bought nothing, for the
       * reason above -- the chase was never the cost. */
      uint32 tA = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_TP();
      uint32 tB = tA;
      if (n > 1) { tB = PPU_PROBE_VRAM_PTR(ppu, tp); NEXT_TP(); }
      uint32 bA = READ_BITS((tA & 0x8000) ? tileadr1 : tileadr0, tA & 0x3ff);
      while (n > 2) {
        uint32 tC = PPU_PROBE_VRAM_PTR(ppu, tp);
        NEXT_TP();
        uint32 bB = READ_BITS((tB & 0x8000) ? tileadr1 : tileadr0, tB & 0x3ff);
        PPU_PIPE_DRAW_4BPP(tA, bA);
        dstz += 8;
        tA = tB, bA = bB, tB = tC;
        n--;
      }
      if (n > 1) {
        uint32 bB = READ_BITS((tB & 0x8000) ? tileadr1 : tileadr0, tB & 0x3ff);
        PPU_PIPE_DRAW_4BPP(tA, bA);
        dstz += 8;
        tA = tB, bA = bB;
      }
      PPU_PIPE_DRAW_4BPP(tA, bA);
      dstz += 8;
#endif
    }
#else
#if SNES_PPU_BG4_RUN
    if (!big && w >= 8) {
      PpuTileWalk tw = { tp, tp_last, tp_next };
      uint n = w >> 3;
      dstz = PpuBg4FullTiles(ppu->vram, &tw, dstz, n,
                             (uint32)(uint16)tileadr1 << 16 | (uint16)tileadr0,
                             (uint32)(uint16)zhi << 16 | (uint16)zlo, no_ztest);
      tp = tw.tp, tp_last = tw.last, tp_next = tw.next;
      w -= n << 3, x += n << 3;
    }
#endif
    while (w >= 8) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_MAP();
#if SNES_PPU_PREFETCH
      /* Start the NEXT tile's bitplane read now, and process this one while it
       * is in flight.
       *
       * Ablation says where this loop's time goes: deleting the decode, the
       * z-compare and the store is worth nothing (52.23 vs 52.36), while
       * deleting the tilemap walk and the VRAM fetch as well is worth +7.18 fps.
       * The arithmetic in here is free; the reads are the frame. VRAM is 64 KB
       * of AXI SRAM behind a 16 KB D-cache, and a 4bpp tile needs two halfwords
       * sixteen bytes apart -- two lines, both likely cold.
       *
       * `tp` already points at the next tilemap entry after NEXT_TP(), so the
       * next tile word is one dependent load away and its bitplane address
       * follows. PLD it and let the pixel work below overlap the miss. This
       * removes stall rather than instructions, which is the only kind of
       * removal that has paid on this part. */
      {
        uint32 ntile = *tp;
        const uint16 *na = &ppu->vram[((ntile & 0x8000 ? tileadr1 : tileadr0)
                                       + (ntile & 0x3ff) * 16) & 0x7fff];
        __builtin_prefetch(na);
        __builtin_prefetch(na + 8);
      }
#endif
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
#if SNES_PPU_TILE_MEMO
      /* Compare the raw tilemap word, not a constructed key. `ta` is chosen by
       * bit 15 of that same word and `tile & 0x3ff` is its low bits, so equal
       * words mean the same row of the same tile -- and the word is already in a
       * register. One compare, no shift, no or. */
      uint32 bits;
      if (tile == memo_key) {
        bits = memo_bits;
      } else {
        bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
        memo_key = tile, memo_bits = bits;
      }
#else
      uint32 bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
#endif
#if SNES_RENDER_CENSUS
      { static uint32 prev_key; uint32 key = (ta << 10) | (tile & 0x3ff);
        if (key == prev_key) g_tile_same++; else g_tile_diff++;
        prev_key = key; }
      g_bg_tile[sub ? 1 : 0]++;
      if (!bits) g_bg_tile_blank[sub ? 1 : 0]++;
      else {
        /* A nibble is opaque iff any of its four bits is set. OR the word down
         * by 1,2,3 and keep bit 0 of each nibble: eight flags in one register. */
        uint32 ch = PpuDecode4bpp(bits);
        uint32 nz = (ch | ch >> 1 | ch >> 2 | ch >> 3) & 0x11111111u;
        int opaque = __builtin_popcount(nz);
        g_tile_opaque_px[sub ? 1 : 0] += opaque;
        if (opaque == 8) g_tile_full[sub ? 1 : 0]++; else g_tile_mixed[sub ? 1 : 0]++;
      }
#endif
      if (bits) {
        PPU_ABLATE_KEEP_BITS(bits);
        uint32 chunky = PpuDecode4bpp(bits);
        z += ((tile & 0x1c00) >> kPaletteShift);
#if SNES_PPU_SIMD_PIXELS
        PPU_SIMD_TILE_4BPP();
#else
#if SNES_PPU_OPAQUE_TILE
        /* Four ALU ops decide it: OR the word down by 1, 2 and 3 and keep bit 0
         * of every nibble. All eight set means no nibble is zero. */
        const uint32 nz = (chunky | chunky >> 1 | chunky >> 2 | chunky >> 3) & 0x11111111u;
        if (nz == 0x11111111u) {
#if SNES_RENDER_CENSUS
          if (no_ztest) g_tile_flat[sub ? 1 : 0]++; else g_tile_opq_z[sub ? 1 : 0]++;
#endif
          if (no_ztest) {
            if (tile & 0x4000) {
              DO_FLAT_CHUNKY_PIXEL(0); DO_FLAT_CHUNKY_PIXEL(1); DO_FLAT_CHUNKY_PIXEL(2); DO_FLAT_CHUNKY_PIXEL(3);
              DO_FLAT_CHUNKY_PIXEL(4); DO_FLAT_CHUNKY_PIXEL(5); DO_FLAT_CHUNKY_PIXEL(6); DO_FLAT_CHUNKY_PIXEL(7);
            } else {
              DO_FLAT_CHUNKY_PIXEL_HFLIP(0); DO_FLAT_CHUNKY_PIXEL_HFLIP(1); DO_FLAT_CHUNKY_PIXEL_HFLIP(2); DO_FLAT_CHUNKY_PIXEL_HFLIP(3);
              DO_FLAT_CHUNKY_PIXEL_HFLIP(4); DO_FLAT_CHUNKY_PIXEL_HFLIP(5); DO_FLAT_CHUNKY_PIXEL_HFLIP(6); DO_FLAT_CHUNKY_PIXEL_HFLIP(7);
            }
          } else if (tile & 0x4000) {
            DO_OPAQUE_CHUNKY_PIXEL(0); DO_OPAQUE_CHUNKY_PIXEL(1); DO_OPAQUE_CHUNKY_PIXEL(2); DO_OPAQUE_CHUNKY_PIXEL(3);
            DO_OPAQUE_CHUNKY_PIXEL(4); DO_OPAQUE_CHUNKY_PIXEL(5); DO_OPAQUE_CHUNKY_PIXEL(6); DO_OPAQUE_CHUNKY_PIXEL(7);
          } else {
            DO_OPAQUE_CHUNKY_PIXEL_HFLIP(0); DO_OPAQUE_CHUNKY_PIXEL_HFLIP(1); DO_OPAQUE_CHUNKY_PIXEL_HFLIP(2); DO_OPAQUE_CHUNKY_PIXEL_HFLIP(3);
            DO_OPAQUE_CHUNKY_PIXEL_HFLIP(4); DO_OPAQUE_CHUNKY_PIXEL_HFLIP(5); DO_OPAQUE_CHUNKY_PIXEL_HFLIP(6); DO_OPAQUE_CHUNKY_PIXEL_HFLIP(7);
          }
        } else
#endif
        if (no_ztest) {
          /* Buffer still holds nothing but backdrop, and this layer's floor is
           * above it: the compare cannot fail, so do not make it. The branch is
           * loop-invariant and perfectly predicted; what it drops is eight loads
           * and eight compares. */
          if (tile & 0x4000) {
            PPU_IF_LOW4  { DO_TOP_CHUNKY_PIXEL(0); DO_TOP_CHUNKY_PIXEL(1); DO_TOP_CHUNKY_PIXEL(2); DO_TOP_CHUNKY_PIXEL(3); }
            PPU_IF_HIGH4 { DO_TOP_CHUNKY_PIXEL(4); DO_TOP_CHUNKY_PIXEL(5); DO_TOP_CHUNKY_PIXEL(6); DO_TOP_CHUNKY_PIXEL(7); }
          } else {
            PPU_IF_HIGH4 { DO_TOP_CHUNKY_PIXEL_HFLIP(0); DO_TOP_CHUNKY_PIXEL_HFLIP(1); DO_TOP_CHUNKY_PIXEL_HFLIP(2); DO_TOP_CHUNKY_PIXEL_HFLIP(3); }
            PPU_IF_LOW4  { DO_TOP_CHUNKY_PIXEL_HFLIP(4); DO_TOP_CHUNKY_PIXEL_HFLIP(5); DO_TOP_CHUNKY_PIXEL_HFLIP(6); DO_TOP_CHUNKY_PIXEL_HFLIP(7); }
          }
        } else if (tile & 0x4000) {
          PPU_IF_LOW4  { DO_CHUNKY_PIXEL(0); DO_CHUNKY_PIXEL(1); DO_CHUNKY_PIXEL(2); DO_CHUNKY_PIXEL(3); }
          PPU_IF_HIGH4 { DO_CHUNKY_PIXEL(4); DO_CHUNKY_PIXEL(5); DO_CHUNKY_PIXEL(6); DO_CHUNKY_PIXEL(7); }
        } else {
          PPU_IF_HIGH4 { DO_CHUNKY_PIXEL_HFLIP(0); DO_CHUNKY_PIXEL_HFLIP(1); DO_CHUNKY_PIXEL_HFLIP(2); DO_CHUNKY_PIXEL_HFLIP(3); }
          PPU_IF_LOW4  { DO_CHUNKY_PIXEL_HFLIP(4); DO_CHUNKY_PIXEL_HFLIP(5); DO_CHUNKY_PIXEL_HFLIP(6); DO_CHUNKY_PIXEL_HFLIP(7); }
        }
#endif
      }
      dstz += 8, w -= 8, x += 8;
    }
#endif
    // Handle remaining clipped part
    if (w) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
#if SNES_PPU_TILE_MEMO
      /* Compare the raw tilemap word, not a constructed key. `ta` is chosen by
       * bit 15 of that same word and `tile & 0x3ff` is its low bits, so equal
       * words mean the same row of the same tile -- and the word is already in a
       * register. One compare, no shift, no or. */
      uint32 bits;
      if (tile == memo_key) {
        bits = memo_bits;
      } else {
        bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
        memo_key = tile, memo_bits = bits;
      }
#else
      uint32 bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
#endif
#if SNES_RENDER_CENSUS
      { static uint32 prev_key; uint32 key = (ta << 10) | (tile & 0x3ff);
        if (key == prev_key) g_tile_same++; else g_tile_diff++;
        prev_key = key; }
      g_bg_tile[sub ? 1 : 0]++;
      if (!bits) g_bg_tile_blank[sub ? 1 : 0]++;
#endif
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --w);
        } else {
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --w);
        }
      }
    }
  }
#undef PPU_PIPE_DRAW_4BPP
#undef DO_TOP_CHUNKY_PIXEL_HFLIP
#undef DO_TOP_CHUNKY_PIXEL
#undef READ_BITS
#undef DO_CHUNKY_PIXEL_HFLIP
#undef DO_CHUNKY_PIXEL
#undef DO_PIXEL
#undef DO_PIXEL_HFLIP
#undef NEXT_MAP
#undef NEXT_TP
}

/* The full-tile middle of a 2bpp layer pass, 8x8 tiles: the 2bpp twin of
 * PpuBg4FullTiles, for the same reason (registers), with the same streaming
 * nibble steps. 2bpp rows are one VRAM word, pixels are the low two bits of
 * each nibble, the palette shift is 8, and a tile whose bits meet top_mask
 * (or a virgin buffer) skips the z test, exactly as the loop it replaces. */
#if !SNES_ABLATE_BG && !SNES_PPU_TILE_MEMO && !SNES_RENDER_CENSUS && !SNES_ABLATE_WALK && \
    !SNES_ABLATE_FETCH && !SNES_PPU_COARSE_SKIP && \
    !defined(SNES_LINE_CACHE) && !defined(SNES_LINE_REUSE_PROBE)
#define SNES_PPU_BG2_RUN 1
#if !SNES_PPU_BG4_RUN
typedef struct { const uint16 *tp, *last, *next; } PpuTileWalk;
#endif
__attribute__((noinline))
static PpuZbufType *PpuBg2FullTiles(const uint16 *vram, PpuTileWalk *tw, PpuZbufType *dstz, uint n,
                                    uint32 tileadrs, uint32 zs, uint32 top_mask, bool no_ztest) {
  const uint16 *tp = tw->tp, *tp_last = tw->last, *tp_next = tw->next;
#define R_STEP(i, nib, adv, TEST) do { PpuZbufType *d_ = &dstz[i]; uint32 px_ = (nib); adv; if (TEST) *d_ = z + px_; } while (0)
#define R_RUN(nib, adv, TEST) do { \
    R_STEP(0, nib, adv, TEST); R_STEP(1, nib, adv, TEST); R_STEP(2, nib, adv, TEST); R_STEP(3, nib, adv, TEST); \
    R_STEP(4, nib, adv, TEST); R_STEP(5, nib, adv, TEST); R_STEP(6, nib, adv, TEST); R_STEP(7, nib, adv, TEST); } while (0)
  do {
    uint32 tile = *tp;
    if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31;
    int ta = (tile & 0x8000) ? (int)(tileadrs >> 16) : (int)(tileadrs & 0xffff);
    PpuZbufType z = (tile & 0x2000) ? (PpuZbufType)(zs >> 16) : (PpuZbufType)zs;
    uint32 bits = vram[(ta + (tile & 0x3ff) * 8) & 0x7fff];
    if (bits) {
      uint32 c = PpuDecode2bpp(bits);
      z += ((tile & 0x1c00) >> 8);
      if (tile & 0x4000) {   /* nibble i = low first */
        if ((tile & top_mask) || no_ztest) R_RUN(c & 3, c >>= 4, px_);
        else                               R_RUN(c & 3, c >>= 4, px_ && z > *d_);
      } else {               /* nibble 7-i = high first */
        if ((tile & top_mask) || no_ztest) R_RUN((c >> 28) & 3, c <<= 4, px_);
        else                               R_RUN((c >> 28) & 3, c <<= 4, px_ && z > *d_);
      }
    }
    dstz += 8;
  } while (--n);
#undef R_STEP
#undef R_RUN
  tw->tp = tp, tw->last = tp_last, tw->next = tp_next;
  return dstz;
}
#else
#define SNES_PPU_BG2_RUN 0
#endif

// Draw a whole line of a 2bpp background layer into bgBuffers.
// top_mask: 0x2000 lets priority-set tiles take the unconditional-store fast
// path -- valid only when this layer's high priority tops every z drawn so
// far (mode 1 BG3). Pass 0 when it does not (mode 0), forcing the z test.
static void PpuDrawBackground_2bpp(Ppu *ppu, uint y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo, uint16 top_mask) {
#if SNES_ABLATE_BG == 6
/* See the 4bpp drawer. */
#define DO_PIXEL(i)                  do { g_ppu_ablate_sink = bits; } while (0)
#define DO_PIXEL_HFLIP(i)            do { g_ppu_ablate_sink = bits; } while (0)
#define DO_CHUNKY_PIXEL(i)           do { } while (0)
#define DO_CHUNKY_PIXEL_HFLIP(i)     do { } while (0)
#define DO_TOP_CHUNKY_PIXEL(i)       do { } while (0)
#define DO_TOP_CHUNKY_PIXEL_HFLIP(i) do { } while (0)
#elif SNES_ABLATE_BG == 2
/* Same bracket for the 2bpp layers; see the 4bpp block. */
#define DO_PIXEL(i)                  do { (void)bits; } while (0)
#define DO_PIXEL_HFLIP(i)            do { (void)bits; } while (0)
#define DO_CHUNKY_PIXEL(i)           do { } while (0)
#define DO_CHUNKY_PIXEL_HFLIP(i)     do { } while (0)
#define DO_TOP_CHUNKY_PIXEL(i)       do { } while (0)
#define DO_TOP_CHUNKY_PIXEL_HFLIP(i) do { } while (0)
#else
#define DO_PIXEL(i) do { \
  pixel = (bits >> i) & 1 | (bits >> (7 + i)) & 2; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_PIXEL_HFLIP(i) do { \
  pixel = (bits >> (7 - i)) & 1 | (bits >> (14 - i)) & 2; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_CHUNKY_PIXEL(i) do { \
  pixel = (chunky >> (4 * i)) & 3; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_CHUNKY_PIXEL_HFLIP(i) do { \
  pixel = (chunky >> (4 * (7 - i))) & 3; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_TOP_CHUNKY_PIXEL(i) do { \
  pixel = (chunky >> (4 * i)) & 3; \
  if (pixel) dstz[i] = z + pixel; } while (0)
#define DO_TOP_CHUNKY_PIXEL_HFLIP(i) do { \
  pixel = (chunky >> (4 * (7 - i))) & 3; \
  if (pixel) dstz[i] = z + pixel; } while (0)
#endif
#if SNES_ABLATE_FETCH
/* See the 4bpp drawer: the ALU stands in for the second load of the chase. */
#define READ_BITS(ta, tile) (addr = ppu->vram, (uint32)(ta) + (uint32)(tile) * 8u)
#else
#define READ_BITS(ta, tile) (PPU_PROBE_VRAM_ADR((ta) + (tile) * 8), addr = &ppu->vram[((ta) + (tile) * 8) & 0x7fff], addr[0])
#endif
  enum { kPaletteShift = 8 };
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  if (IS_MOSAIC_ENABLED(ppu, layer) && ppu->mosaicSize > 1) {
    PpuDrawBackground_mosaic(ppu, y, sub, layer, zhi, zlo, 2);
    return;
  }
#if SNES_PPU_VIRGIN_Z
  const bool no_ztest = PPU_BUF_VIRGIN(sub) && (uint32)zlo > (uint32)kPpuBackdropZ;
#else
  const bool no_ztest = false;
#endif
  PPU_BUF_MARK(sub);
#if SNES_ABLATE_BG == 1
  /* ABLATION, WRONG OUTPUT ON PURPOSE. Not an optimisation -- it deletes the
   * entire background layer draw (tilemap walk, VRAM fetch, decode, z-compare,
   * store) so the device can price the ceiling of ever rewriting that inner
   * loop. If "BG rendering is free" is worth little, the hand-written assembly
   * project behind it is worth less, and nobody has to spend days finding out.
   * The screen shows backdrop; the frame counter is the only valid reading.
   *
   * MEASURED: **59.54 fps against a 52.36 baseline -- +7.18, +13.7%.** Zelda 3
   * rain, 900 deterministic frames, three runs. That is the ceiling of making
   * background tile rendering free, and it is 3.5x everything else won on this
   * core in a day of A/Bs.
   *
   * Note what this says about the profile: PC sampling scored
   * PpuDrawBackground_4bpp at 6.1% of the frame, and deleting it is worth 13.7%.
   * Sampling puts a stall on whichever instruction is retiring, so work that is
   * mostly waiting on memory reads lighter than it is. Price a candidate by
   * ablation, not by its share of the histogram.
   *
   * What a rewrite could actually capture is less than 7.18: this deletes the
   * tilemap walk and VRAM fetch too, which SIMD does not touch. Say a third to a
   * half of it. Still the largest number on the board. */
  return;
#endif
#if SNES_RENDER_CENSUS
  g_bg_pass[sub ? 1 : 0]++;
  if (sub) { if (IS_SCREEN_ENABLED(ppu, 0, layer)) g_sub_also_main++; else g_sub_only++; }
#endif
  PpuWindows win;
#if SNES_WINDOW_CENSUS
  if (sub && IS_SCREEN_WINDOWED(ppu, 1, layer)) {
    g_win_calc_sub++;
    if (IS_SCREEN_WINDOWED(ppu, 0, layer)) g_win_dup++;
  }
#endif
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  BgLayer *bglayer = &ppu->bgLayer[layer];
  y += bglayer->vScroll;
  const bool big = bglayer->bigTiles;
  const int y_shift = big ? 4 : 3;
  const int y_screen = big ? 0x200 : 0x100;
  int sc_offs = bglayer->tilemapAdr + (((y >> y_shift) & 0x1f) << 5);
  if ((y & y_screen) && bglayer->tilemapHigher)
    sc_offs += bglayer->tilemapWider ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (bglayer->tilemapWider ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = ppu->bgLayer[layer].tileAdr, pixel;
  int tileadr1 = tileadr + 7 - (y & 0x7), tileadr0 = tileadr + (y & 0x7);
#if SNES_ABLATE_BG == 4
  /* ABLATION, WRONG OUTPUT ON PURPOSE. Everything above this line stays -- the
   * enabled/windowed tests, PpuWindows_Calc or _Clear, the tilemap base and the
   * two row addresses -- and everything below it goes. It prices the PER-CALL
   * SETUP on its own, against =1 (the whole draw) and =2 (the setup and the walk
   * and the fetch, without the pixel work). 737 calls a frame at 3.29 layer
   * passes a line; if that is where the 4.33 fps lives, the target is the caller,
   * not the inner loop.
   *
   * The sink is what keeps it honest: with the results unused, gcc deletes the
   * setup and the arm measures nothing but the call. */
  g_ppu_ablate_sink = (uint32)win.nr + (uint32)win.edges[0]
                    + (uint32)(uintptr_t)tps[0] + (uint32)(uintptr_t)tps[1]
                    + (uint32)tileadr1 + (uint32)tileadr0;
  return;
#endif

  const uint16 *addr;
#if SNES_PPU_TILE_MEMO
  /* Same one-entry tile memo as the 4bpp drawer; see the comment there. */
  uint32 memo_key = 0x10000u, memo_bits = 0;   /* 0x10000 is not a 16-bit tile word */
#endif
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    uint x = win.edges[windex] + bglayer->hScroll;
    uint w = win.edges[windex + 1] - win.edges[windex];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + win.edges[windex] + kPpuExtraLeftRight;
    const int x_shift = big ? 4 : 3;
    const int x_screen = big ? 9 : 8;
    const uint16 *tp = tps[x >> x_screen & 1] + ((x >> x_shift) & 0x1f);
    const uint16 *tp_last = tps[x >> x_screen & 1] + 31;
    const uint16 *tp_next = tps[(x >> x_screen & 1) ^ 1];

#if SNES_ABLATE_WALK
/* ABLATION, WRONG OUTPUT ON PURPOSE. `tp` never advances, so every tile in the
 * span is the same tilemap entry: the decode, the z-compare, the store and both
 * VRAM loads all still happen, and only the WALK -- the pointer bump, the
 * end-of-screen compare and its branch -- is gone. Against =2 (which keeps the
 * walk and deletes the pixel work, and measured nothing) this is the other half
 * of the same question, and one of the two has to hold the 4.33 fps.
 *
 * It does contaminate the read: the same address every time is a guaranteed
 * cache hit. That is acceptable here only because four independent experiments
 * have already priced the reads at zero, up to and including moving all 64 KB of
 * VRAM into zero-wait DTCM. */
#define NEXT_TP() do { } while (0)
#else
#define NEXT_TP() if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31;
#endif
#define NEXT_MAP() do { if (!big || (x & 8)) { NEXT_TP(); } } while (0)
    // Handle clipped pixels on left side
    if (x & 7) {
      int curw = IntMin(8 - (x & 7), w);
      int clip = curw;
      w -= curw;
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_MAP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
#if SNES_PPU_TILE_MEMO
      /* Compare the raw tilemap word, not a constructed key. `ta` is chosen by
       * bit 15 of that same word and `tile & 0x3ff` is its low bits, so equal
       * words mean the same row of the same tile -- and the word is already in a
       * register. One compare, no shift, no or. */
      uint32 bits;
      if (tile == memo_key) {
        bits = memo_bits;
      } else {
        bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
        memo_key = tile, memo_bits = bits;
      }
#else
      uint32 bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
#endif
#if SNES_RENDER_CENSUS
      { static uint32 prev_key; uint32 key = (ta << 10) | (tile & 0x3ff);
        if (key == prev_key) g_tile_same++; else g_tile_diff++;
        prev_key = key; }
      g_bg_tile[sub ? 1 : 0]++;
      if (!bits) g_bg_tile_blank[sub ? 1 : 0]++;
#endif
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          bits >>= (x & 7);
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --curw);
        } else {
          bits <<= (x & 7);
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --curw);
        }
      } else {
        dstz += clip;
      }
      x += clip;
    }
    // Handle full tiles in the middle
#if SNES_PPU_BG2_RUN
    if (!big && w >= 8) {
      PpuTileWalk tw = { tp, tp_last, tp_next };
      uint n = w >> 3;
      dstz = PpuBg2FullTiles(ppu->vram, &tw, dstz, n,
                             (uint32)(uint16)tileadr1 << 16 | (uint16)tileadr0,
                             (uint32)(uint16)zhi << 16 | (uint16)zlo, top_mask, no_ztest);
      tp = tw.tp, tp_last = tw.last, tp_next = tw.next;
      w -= n << 3, x += n << 3;
    }
#endif
    while (w >= 8) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_MAP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
#if SNES_PPU_TILE_MEMO
      /* Compare the raw tilemap word, not a constructed key. `ta` is chosen by
       * bit 15 of that same word and `tile & 0x3ff` is its low bits, so equal
       * words mean the same row of the same tile -- and the word is already in a
       * register. One compare, no shift, no or. */
      uint32 bits;
      if (tile == memo_key) {
        bits = memo_bits;
      } else {
        bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
        memo_key = tile, memo_bits = bits;
      }
#else
      uint32 bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
#endif
#if SNES_RENDER_CENSUS
      { static uint32 prev_key; uint32 key = (ta << 10) | (tile & 0x3ff);
        if (key == prev_key) g_tile_same++; else g_tile_diff++;
        prev_key = key; }
      g_bg_tile[sub ? 1 : 0]++;
      if (!bits) g_bg_tile_blank[sub ? 1 : 0]++;
      else {
        uint32 c2 = PpuDecode2bpp(bits), n2 = (c2 | c2 >> 1) & 0x11111111u;
        if (n2 == 0x11111111u) g_t2_full[sub ? 1 : 0]++; else g_t2_mixed[sub ? 1 : 0]++;
      }
#endif
      if (bits) {
        PPU_ABLATE_KEEP_BITS(bits);
        uint32 chunky = PpuDecode2bpp(bits);
        z += ((tile & 0x1c00) >> kPaletteShift);
        /* In mode 1 this renderer is BG3, whose high priority (0xf2) is above
         * every BG1/BG2/OBJ priority, so the z test is always true and the
         * TOP store can skip it (top_mask = 0x2000). Mode 0 layers have
         * sprites above them at every priority, so they pass top_mask = 0. */
        if (tile & 0x4000) {
          if ((tile & top_mask) || no_ztest) {
            PPU_IF_LOW4  { DO_TOP_CHUNKY_PIXEL(0); DO_TOP_CHUNKY_PIXEL(1); DO_TOP_CHUNKY_PIXEL(2); DO_TOP_CHUNKY_PIXEL(3); }
            PPU_IF_HIGH4 { DO_TOP_CHUNKY_PIXEL(4); DO_TOP_CHUNKY_PIXEL(5); DO_TOP_CHUNKY_PIXEL(6); DO_TOP_CHUNKY_PIXEL(7); }
          } else {
            PPU_IF_LOW4  { DO_CHUNKY_PIXEL(0); DO_CHUNKY_PIXEL(1); DO_CHUNKY_PIXEL(2); DO_CHUNKY_PIXEL(3); }
            PPU_IF_HIGH4 { DO_CHUNKY_PIXEL(4); DO_CHUNKY_PIXEL(5); DO_CHUNKY_PIXEL(6); DO_CHUNKY_PIXEL(7); }
          }
        } else {
          if ((tile & top_mask) || no_ztest) {
            PPU_IF_HIGH4 { DO_TOP_CHUNKY_PIXEL_HFLIP(0); DO_TOP_CHUNKY_PIXEL_HFLIP(1); DO_TOP_CHUNKY_PIXEL_HFLIP(2); DO_TOP_CHUNKY_PIXEL_HFLIP(3); }
            PPU_IF_LOW4  { DO_TOP_CHUNKY_PIXEL_HFLIP(4); DO_TOP_CHUNKY_PIXEL_HFLIP(5); DO_TOP_CHUNKY_PIXEL_HFLIP(6); DO_TOP_CHUNKY_PIXEL_HFLIP(7); }
          } else {
            PPU_IF_HIGH4 { DO_CHUNKY_PIXEL_HFLIP(0); DO_CHUNKY_PIXEL_HFLIP(1); DO_CHUNKY_PIXEL_HFLIP(2); DO_CHUNKY_PIXEL_HFLIP(3); }
            PPU_IF_LOW4  { DO_CHUNKY_PIXEL_HFLIP(4); DO_CHUNKY_PIXEL_HFLIP(5); DO_CHUNKY_PIXEL_HFLIP(6); DO_CHUNKY_PIXEL_HFLIP(7); }
          }
        }
      }
      dstz += 8, w -= 8, x += 8;
    }
    // Handle remaining clipped part
    if (w) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
#if SNES_PPU_TILE_MEMO
      /* Compare the raw tilemap word, not a constructed key. `ta` is chosen by
       * bit 15 of that same word and `tile & 0x3ff` is its low bits, so equal
       * words mean the same row of the same tile -- and the word is already in a
       * register. One compare, no shift, no or. */
      uint32 bits;
      if (tile == memo_key) {
        bits = memo_bits;
      } else {
        bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
        memo_key = tile, memo_bits = bits;
      }
#else
      uint32 bits = READ_BITS(ta, PpuBgCharFromMap(tile, x, y, big));
#endif
#if SNES_RENDER_CENSUS
      { static uint32 prev_key; uint32 key = (ta << 10) | (tile & 0x3ff);
        if (key == prev_key) g_tile_same++; else g_tile_diff++;
        prev_key = key; }
      g_bg_tile[sub ? 1 : 0]++;
      if (!bits) g_bg_tile_blank[sub ? 1 : 0]++;
#endif
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --w);
        } else {
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --w);
        }
      }
    }
  }
#undef NEXT_TP
#undef NEXT_MAP
#undef READ_BITS
#undef DO_TOP_CHUNKY_PIXEL_HFLIP
#undef DO_TOP_CHUNKY_PIXEL
#undef DO_CHUNKY_PIXEL_HFLIP
#undef DO_CHUNKY_PIXEL
#undef DO_PIXEL
#undef DO_PIXEL_HFLIP
}

/* Mode 3/4 BG1: 8bpp, 64 bytes per 8×8 character (four bitplanes). Colour is
 * the pixel itself (no tilemap palette). Same 8-pixel walk as 4bpp, including
 * 16×16 map entries. */
static void PpuDrawBackground_8bpp(Ppu *ppu, uint y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo) {
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;
  if (IS_MOSAIC_ENABLED(ppu, layer) && ppu->mosaicSize > 1) {
    PpuDrawBackground_mosaic(ppu, y, sub, layer, zhi, zlo, 8);
    return;
  }
  PPU_BUF_MARK(sub);
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  BgLayer *bglayer = &ppu->bgLayer[layer];
  y += bglayer->vScroll;
  const bool big = bglayer->bigTiles;
  const int y_shift = big ? 4 : 3;
  const int y_screen = big ? 0x200 : 0x100;
  int sc_offs = bglayer->tilemapAdr + (((y >> y_shift) & 0x1f) << 5);
  if ((y & y_screen) && bglayer->tilemapHigher)
    sc_offs += bglayer->tilemapWider ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (bglayer->tilemapWider ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = bglayer->tileAdr;
  int tileadr1 = tileadr + 7 - (y & 7), tileadr0 = tileadr + (y & 7);
  int pixel;
#define LOAD_8BPP(ta, ch, lo, hi) do { \
    unsigned a = ((unsigned)(ta) + (unsigned)(ch) * 32u) & 0x7fff; \
    PPU_PROBE_VRAM_ADR(a); \
    (lo) = ppu->vram[a] | (uint32)ppu->vram[(a + 8) & 0x7fff] << 16; \
    (hi) = ppu->vram[(a + 16) & 0x7fff] | (uint32)ppu->vram[(a + 24) & 0x7fff] << 16; \
  } while (0)
#define DO_PIX8() do { \
    pixel = (lo & 1) | (lo >> 7) & 2 | (lo >> 14) & 4 | (lo >> 21) & 8 \
          | ((hi & 1) | (hi >> 7) & 2 | (hi >> 14) & 4 | (hi >> 21) & 8) << 4; \
    if (pixel && z > dstz[0]) dstz[0] = z + pixel; \
  } while (0)
#define DO_PIX8_HFLIP() do { \
    pixel = (lo >> 7) & 1 | (lo >> 14) & 2 | (lo >> 21) & 4 | (lo >> 28) & 8 \
          | ((hi >> 7) & 1 | (hi >> 14) & 2 | (hi >> 21) & 4 | (hi >> 28) & 8) << 4; \
    if (pixel && z > dstz[0]) dstz[0] = z + pixel; \
  } while (0)
#define DO_PIX8_I(i) do { \
    pixel = (lo >> (i)) & 1 | (lo >> (7 + (i))) & 2 | (lo >> (14 + (i))) & 4 | (lo >> (21 + (i))) & 8 \
          | ((hi >> (i)) & 1 | (hi >> (7 + (i))) & 2 | (hi >> (14 + (i))) & 4 | (hi >> (21 + (i))) & 8) << 4; \
    if (pixel && z > dstz[i]) dstz[i] = z + pixel; \
  } while (0)
#define DO_PIX8_HFLIP_I(i) do { \
    pixel = (lo >> (7 - (i))) & 1 | (lo >> (14 - (i))) & 2 | (lo >> (21 - (i))) & 4 | (lo >> (28 - (i))) & 8 \
          | ((hi >> (7 - (i))) & 1 | (hi >> (14 - (i))) & 2 | (hi >> (21 - (i))) & 4 | (hi >> (28 - (i))) & 8) << 4; \
    if (pixel && z > dstz[i]) dstz[i] = z + pixel; \
  } while (0)
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;
    uint x = win.edges[windex] + bglayer->hScroll;
    uint w = win.edges[windex + 1] - win.edges[windex];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + win.edges[windex] + kPpuExtraLeftRight;
    const int x_shift = big ? 4 : 3;
    const int x_screen = big ? 9 : 8;
    const uint16 *tp = tps[x >> x_screen & 1] + ((x >> x_shift) & 0x1f);
    const uint16 *tp_last = tps[x >> x_screen & 1] + 31;
    const uint16 *tp_next = tps[(x >> x_screen & 1) ^ 1];
#define NEXT_TP() if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31
#define NEXT_MAP() do { if (!big || (x & 8)) { NEXT_TP(); } } while (0)
    if (x & 7) {
      int curw = IntMin(8 - (x & 7), w);
      int clip = curw;
      w -= curw;
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_MAP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 lo, hi;
      LOAD_8BPP(ta, PpuBgCharFromMap(tile, x, y, big), lo, hi);
      if (lo | hi) {
        if (tile & 0x4000) {
          lo >>= (x & 7); hi >>= (x & 7);
          do { DO_PIX8(); } while (lo >>= 1, hi >>= 1, dstz++, --curw);
        } else {
          lo <<= (x & 7); hi <<= (x & 7);
          do { DO_PIX8_HFLIP(); } while (lo <<= 1, hi <<= 1, dstz++, --curw);
        }
      } else {
        dstz += clip;
      }
      x += clip;
    }
    while (w >= 8) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_MAP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 lo, hi;
      LOAD_8BPP(ta, PpuBgCharFromMap(tile, x, y, big), lo, hi);
      if (lo | hi) {
        if (tile & 0x4000) {
          DO_PIX8_I(0); DO_PIX8_I(1); DO_PIX8_I(2); DO_PIX8_I(3);
          DO_PIX8_I(4); DO_PIX8_I(5); DO_PIX8_I(6); DO_PIX8_I(7);
        } else {
          DO_PIX8_HFLIP_I(0); DO_PIX8_HFLIP_I(1); DO_PIX8_HFLIP_I(2); DO_PIX8_HFLIP_I(3);
          DO_PIX8_HFLIP_I(4); DO_PIX8_HFLIP_I(5); DO_PIX8_HFLIP_I(6); DO_PIX8_HFLIP_I(7);
        }
      }
      dstz += 8, w -= 8, x += 8;
    }
    if (w) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 lo, hi;
      LOAD_8BPP(ta, PpuBgCharFromMap(tile, x, y, big), lo, hi);
      if (lo | hi) {
        if (tile & 0x4000) {
          do { DO_PIX8(); } while (lo >>= 1, hi >>= 1, dstz++, --w);
        } else {
          do { DO_PIX8_HFLIP(); } while (lo <<= 1, hi <<= 1, dstz++, --w);
        }
      }
    }
#undef NEXT_MAP
#undef NEXT_TP
  }
#undef DO_PIX8_HFLIP_I
#undef DO_PIX8_I
#undef DO_PIX8_HFLIP
#undef DO_PIX8
#undef LOAD_8BPP
}

// Assumes it's drawn on an empty backdrop
static void PpuDrawBackground_mode7(Ppu *ppu, uint y, bool sub, PpuZbufType z) {
  int layer = 0;
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PPU_BUF_MARK(sub);
#if SNES_ABLATE_BG == 1
  /* ABLATION, WRONG OUTPUT ON PURPOSE. Not an optimisation -- it deletes the
   * entire background layer draw (tilemap walk, VRAM fetch, decode, z-compare,
   * store) so the device can price the ceiling of ever rewriting that inner
   * loop. If "BG rendering is free" is worth little, the hand-written assembly
   * project behind it is worth less, and nobody has to spend days finding out.
   * The screen shows backdrop; the frame counter is the only valid reading.
   *
   * MEASURED: **59.54 fps against a 52.36 baseline -- +7.18, +13.7%.** Zelda 3
   * rain, 900 deterministic frames, three runs. That is the ceiling of making
   * background tile rendering free, and it is 3.5x everything else won on this
   * core in a day of A/Bs.
   *
   * Note what this says about the profile: PC sampling scored
   * PpuDrawBackground_4bpp at 6.1% of the frame, and deleting it is worth 13.7%.
   * Sampling puts a stall on whichever instruction is retiring, so work that is
   * mostly waiting on memory reads lighter than it is. Price a candidate by
   * ablation, not by its share of the histogram.
   *
   * What a rewrite could actually capture is less than 7.18: this deletes the
   * tilemap walk and VRAM fetch too, which SIMD does not touch. Say a third to a
   * half of it. Still the largest number on the board. */
  return;
#endif
#if SNES_RENDER_CENSUS
  g_bg_pass[sub ? 1 : 0]++;
  if (sub) { if (IS_SCREEN_ENABLED(ppu, 0, layer)) g_sub_also_main++; else g_sub_only++; }
#endif
  PpuWindows win;
#if SNES_WINDOW_CENSUS
  if (sub && IS_SCREEN_WINDOWED(ppu, 1, layer)) {
    g_win_calc_sub++;
    if (IS_SCREEN_WINDOWED(ppu, 0, layer)) g_win_dup++;
  }
#endif
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);

  // expand 13-bit values to signed values
  int hScroll = ((int16_t)(ppu->m7matrix[6] << 3)) >> 3;
  int vScroll = ((int16_t)(ppu->m7matrix[7] << 3)) >> 3;
  int xCenter = ((int16_t)(ppu->m7matrix[4] << 3)) >> 3;
  int yCenter = ((int16_t)(ppu->m7matrix[5] << 3)) >> 3;
  int clippedH = hScroll - xCenter;
  int clippedV = vScroll - yCenter;
  clippedH = (clippedH & 0x2000) ? (clippedH | ~1023) : (clippedH & 1023);
  clippedV = (clippedV & 0x2000) ? (clippedV | ~1023) : (clippedV & 1023);
  bool mosaic_enabled = IS_MOSAIC_ENABLED(ppu, 0) && ppu->mosaicSize > 1;
  if (mosaic_enabled)
    y -= (y - ppu->mosaicStartLine) % ppu->mosaicSize;
  uint32 ry = ppu->m7yFlip ? 255 - y : y;
  uint32 m7startX = (ppu->m7matrix[0] * clippedH & ~63) + (ppu->m7matrix[1] * ry & ~63) +
    (ppu->m7matrix[1] * clippedV & ~63) + (xCenter << 8);
  uint32 m7startY = (ppu->m7matrix[2] * clippedH & ~63) + (ppu->m7matrix[3] * ry & ~63) +
    (ppu->m7matrix[3] * clippedV & ~63) + (yCenter << 8);
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int x = win.edges[windex], x2 = win.edges[windex + 1], tile;
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + x + kPpuExtraLeftRight;
    PpuZbufType *dstz_end = ppu->bgBuffers[sub].data + x2 + kPpuExtraLeftRight;
    uint32 rx = ppu->m7xFlip ? 255 - x : x;
    uint32 xpos = m7startX + ppu->m7matrix[0] * rx;
    uint32 ypos = m7startY + ppu->m7matrix[2] * rx;
    uint32 dx = ppu->m7xFlip ? -ppu->m7matrix[0] : ppu->m7matrix[0];
    uint32 dy = ppu->m7xFlip ? -ppu->m7matrix[2] : ppu->m7matrix[2];
    uint32 outside_value = ppu->m7largeField ? 0x3ffff : 0xffffffff;
    bool char_fill = ppu->m7charFill;
    if (mosaic_enabled) {
      int w = ppu->mosaicSize - (x % ppu->mosaicSize);
      do {
        w = IntMin(w, dstz_end - dstz);
        if ((uint32)(xpos | ypos) > outside_value) {
          if (!char_fill)
            continue;
          tile = 0;
        } else {
          uint32_t map_adr = (ypos >> 11 & 0x7f) * 128 + (xpos >> 11 & 0x7f);
          tile = PPU_PROBE_VRAM_PTR(ppu, &ppu->vram[map_adr]) & 0xff;
        }
        uint8 pixel = PPU_PROBE_VRAM_PTR(ppu, &ppu->vram[tile * 64 + (ypos >> 8 & 7) * 8 + (xpos >> 8 & 7)]) >> 8;
        if (pixel) {
          int i = 0;
          do dstz[i] = pixel + z; while (++i != w);
        }
      } while (xpos += dx * w, ypos += dy * w, dstz += w, w = ppu->mosaicSize, dstz_end - dstz != 0);
    } else {
      do {
        if ((uint32)(xpos | ypos) > outside_value) {
          if (!char_fill)
            continue;
          tile = 0;
        } else {
          uint32_t map_adr = (ypos >> 11 & 0x7f) * 128 + (xpos >> 11 & 0x7f);
          tile = PPU_PROBE_VRAM_PTR(ppu, &ppu->vram[map_adr]) & 0xff;
        }
        uint8 pixel = PPU_PROBE_VRAM_PTR(ppu, &ppu->vram[tile * 64 + (ypos >> 8 & 7) * 8 + (xpos >> 8 & 7)]) >> 8;
        if (pixel)
          dstz[0] = pixel + z;
      } while (xpos += dx, ypos += dy, ++dstz != dstz_end);
    }
  }
}


/* SNES_PPU_SPLIT=1: keep the render's three big pieces as separate symbols so
 * the device's PC sampler can tell them apart. types.h defines NOINLINE only
 * for MSVC -- under gcc it expands to nothing, so PpuDrawWholeLine,
 * PpuDrawBackgrounds and PpuDrawSprites all fold into ppu_runLine, and the
 * profile can only report that 5.6 KB blob as one 14.7% line. Diagnostic builds
 * only: forcing the calls costs a little, and what it buys is attribution. */
#if defined(SNES_PPU_SPLIT) && SNES_PPU_SPLIT
#define PPU_SPLIT_NOINLINE __attribute__((noinline))
#else
#define PPU_SPLIT_NOINLINE
#endif

PPU_SPLIT_NOINLINE static void PpuDrawSprites(Ppu *ppu, uint y, uint sub, bool clear_backdrop) {
#if SNES_ABLATE_SPRITE_MERGE
  /* ABLATION, WRONG OUTPUT. Only the merge of objBuffer into the bg z-buffer.
   * With clear_backdrop it is a 512-byte memcpy per line per screen -- over a
   * buffer ClearBackdrop just filled with the same backdrop value the objBuffer
   * holds everywhere a sprite is not. */
  (void)y; (void)sub; (void)clear_backdrop; return;
#endif
  int layer = 4;
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PPU_BUF_MARK(sub);
#if SNES_ABLATE_BG == 1
  /* ABLATION, WRONG OUTPUT ON PURPOSE. Not an optimisation -- it deletes the
   * entire background layer draw (tilemap walk, VRAM fetch, decode, z-compare,
   * store) so the device can price the ceiling of ever rewriting that inner
   * loop. If "BG rendering is free" is worth little, the hand-written assembly
   * project behind it is worth less, and nobody has to spend days finding out.
   * The screen shows backdrop; the frame counter is the only valid reading.
   *
   * MEASURED: **59.54 fps against a 52.36 baseline -- +7.18, +13.7%.** Zelda 3
   * rain, 900 deterministic frames, three runs. That is the ceiling of making
   * background tile rendering free, and it is 3.5x everything else won on this
   * core in a day of A/Bs.
   *
   * Note what this says about the profile: PC sampling scored
   * PpuDrawBackground_4bpp at 6.1% of the frame, and deleting it is worth 13.7%.
   * Sampling puts a stall on whichever instruction is retiring, so work that is
   * mostly waiting on memory reads lighter than it is. Price a candidate by
   * ablation, not by its share of the histogram.
   *
   * What a rewrite could actually capture is less than 7.18: this deletes the
   * tilemap walk and VRAM fetch too, which SIMD does not touch. Say a third to a
   * half of it. Still the largest number on the board. */
  return;
#endif
#if SNES_RENDER_CENSUS
  g_bg_pass[sub ? 1 : 0]++;
  if (sub) { if (IS_SCREEN_ENABLED(ppu, 0, layer)) g_sub_also_main++; else g_sub_only++; }
#endif
  PpuWindows win;
#if SNES_WINDOW_CENSUS
  if (sub && IS_SCREEN_WINDOWED(ppu, 1, layer)) {
    g_win_calc_sub++;
    if (IS_SCREEN_WINDOWED(ppu, 0, layer)) g_win_dup++;
  }
#endif
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int left = win.edges[windex];
    int width = win.edges[windex + 1] - left;
    PpuZbufType *src = ppu->objBuffer.data + left + kPpuExtraLeftRight;
    PpuZbufType *dst = ppu->bgBuffers[sub].data + left + kPpuExtraLeftRight;
    if (clear_backdrop) {
      memcpy(dst, src, width * sizeof(uint16));
    } else {
      do {
        if (src[0] > dst[0])
          dst[0] = src[0];
      } while (src++, dst++, --width);
    }
  }
}

/* SNES_PERF: time one layer or sprite pass into its PPU part */
#define PPU_PT(part, call) do { SNES_PERF_PPU_BEGIN(t_pt); call; SNES_PERF_PPU_END(part, t_pt); } while (0)

PPU_SPLIT_NOINLINE static void PpuDrawBackgrounds(Ppu *ppu, int y, bool sub) {
  // Top 4 bits contain the prio level, and bottom 4 bits the layer type.
  // SPRITE_PRIO_TO_PRIO can be used to convert from obj prio to this prio.
  //  15: BG3 tiles with priority 1 if bit 3 of $2105 is set
  //  14: Sprites with priority 3 (4 * sprite_prio + 2)
  //  12: BG1 tiles with priority 1
  //  11: BG2 tiles with priority 1
  //  10: Sprites with priority 2 (4 * sprite_prio + 2)
  //  8: BG1 tiles with priority 0
  //  7: BG2 tiles with priority 0
  //  6: Sprites with priority 1 (4 * sprite_prio + 2)
  //  3: BG3 tiles with priority 1 if bit 3 of $2105 is clear
  //  2: Sprites with priority 0 (4 * sprite_prio + 2)
  //  1: BG3 tiles with priority 0
  //  0: backdrop

  if (ppu->mode == 1) {
    if (ppu->lineHasSprites)
      PPU_PT(SNES_PERF_PPU_SPR, PpuDrawSprites(ppu, y, sub, true));

#ifdef GNW_SNES_CORE
    /* Mosaic ($2106) is handled inside the 4bpp/2bpp drawers. */
    PPU_PT(SNES_PERF_PPU_BG4, PpuDrawBackground_4bpp(ppu, y, sub, 0, 0xc000, 0x8000));
    PPU_PT(SNES_PERF_PPU_BG4, PpuDrawBackground_4bpp(ppu, y, sub, 1, 0xb100, 0x7100));
    /* $2105 bit 3 raises BG3 to rank 15. Without it, BG3 prio-1 is rank 3
     * (under BG1/BG2). Hardcoding the Zelda-3 constants put Pilotwings' HUD
     * under BG3's 2bpp tiles. */
    PPU_PT(SNES_PERF_PPU_BG2, PpuDrawBackground_2bpp(ppu, y, sub, 2,
                           ppu->bg3priority ? 0xf200 : 0x3200,
                           0x1200,
                           ppu->bg3priority ? 0x2000 : 0));
#else
    if (IS_MOSAIC_ENABLED(ppu, 0))
      assert(0);
    else
      PPU_PT(SNES_PERF_PPU_BG4, PpuDrawBackground_4bpp(ppu, y, sub, 0, 0xc000, 0x8000));

    if (IS_MOSAIC_ENABLED(ppu, 1))
      assert(0);
    else
      PPU_PT(SNES_PERF_PPU_BG4, PpuDrawBackground_4bpp(ppu, y, sub, 1, 0xb100, 0x7100));

    if (IS_MOSAIC_ENABLED(ppu, 2))
      assert(0);
    else
      PPU_PT(SNES_PERF_PPU_BG2, PpuDrawBackground_2bpp(ppu, y, sub, 2,
                             ppu->bg3priority ? 0xf200 : 0x3200,
                             0x1200,
                             ppu->bg3priority ? 0x2000 : 0));
#endif
  } else if (ppu->mode == 0) {
    /* Mode 0: four 2bpp layers, each with its own 32-colour CGRAM window
     * (BG2 +32, BG3 +64, BG4 +96 -- folded into the z parameters, whose low
     * byte is the CGRAM index). Priority ranks interleave with the sprite
     * ranks (4*prio+2 = 2/6/10/14) in the hardware order
     * S3 BG1p1 BG2p1 S2 BG1p0 BG2p0 S1 BG3p1 BG4p1 S0 BG3p0 BG4p0.
     * Sprites are never below any layer's fast path here, so top_mask = 0.
     * (Mario Kart's whole menu flow -- driver select included -- is mode 0;
     * this used to fall through to the mode-7 renderer and drew garbage.) */
    if (ppu->lineHasSprites)
      PPU_PT(SNES_PERF_PPU_SPR, PpuDrawSprites(ppu, y, sub, true));
    /* z is [prio:4][layer:4][cgram:8]. Backdrop is 0x0500 (layer 5), so a
     * prio-0 layer-3 word (0x03xx) loses the z test and BG4 vanishes — the
     * Burn-in Test Cartridge Mode 0 screen is exactly that: TM=BG4, prio 0.
     * Keep layer nibbles 2/3 and sit BG4p0 above 0x0500, BG3p0 above BG4p0,
     * both still below sprite prio 0 (0x24xx). */
    PPU_PT(SNES_PERF_PPU_BG2, PpuDrawBackground_2bpp(ppu, y, sub, 0, 0xd000,      0x9000,      0));
    PPU_PT(SNES_PERF_PPU_BG2, PpuDrawBackground_2bpp(ppu, y, sub, 1, 0xc100 + 32, 0x8100 + 32, 0));
    PPU_PT(SNES_PERF_PPU_BG2, PpuDrawBackground_2bpp(ppu, y, sub, 2, 0x5200 + 64, 0x2200 + 64, 0));
    PPU_PT(SNES_PERF_PPU_BG2, PpuDrawBackground_2bpp(ppu, y, sub, 3, 0x4300 + 96, 0x1300 + 96, 0));
  } else if (ppu->mode == 2) {
    /* Mode 2: BG1+BG2 4bpp. Streaming walk when OPT is unused; per-column
     * ppu_handleOPT otherwise. Hardware order S3 BG1p1 S2 BG2p1 S1 BG1p0 S0 BG2p0. */
    if (ppu->lineHasSprites)
      PPU_PT(SNES_PERF_PPU_SPR, PpuDrawSprites(ppu, y, sub, true));
    if (PpuMode2HasOpt(ppu)) {
      PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_opt(ppu, y, sub, 0, 0xc000, 0x4000, 4));
      PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_opt(ppu, y, sub, 1, 0x9100, 0x1100, 4));
    } else {
      PPU_PT(SNES_PERF_PPU_BG4, PpuDrawBackground_4bpp(ppu, y, sub, 0, 0xc000, 0x4000));
      PPU_PT(SNES_PERF_PPU_BG4, PpuDrawBackground_4bpp(ppu, y, sub, 1, 0x9100, 0x1100));
    }
  } else if (ppu->mode == 4) {
    /* Mode 4: BG1 8bpp + BG2 2bpp. Hardware order
     * S3 BG1p1 S2 BG2p1 S1 BG1p0 S0 BG2p0. Sprite ranks are 14/10/6/2 in
     * the high byte; the z constants sit in the gaps. */
    if (ppu->lineHasSprites)
      PPU_PT(SNES_PERF_PPU_SPR, PpuDrawSprites(ppu, y, sub, true));
    if (PpuMode4HasOpt(ppu)) {
      PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_opt(ppu, y, sub, 0, 0xc000, 0x4000, 8));
      PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_opt(ppu, y, sub, 1, 0x9100, 0x1100, 2));
    } else {
      PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_8bpp(ppu, y, sub, 0, 0xc000, 0x4000));
      PPU_PT(SNES_PERF_PPU_BG2, PpuDrawBackground_2bpp(ppu, y, sub, 1, 0x9100, 0x1100, 0));
    }
  } else if (ppu->mode == 3) {
    /* Mode 3: BG1 8bpp + BG2 4bpp. Same priority ranks as Mode 2/4
     * (S3 BG1p1 S2 BG2p1 S1 BG1p0 S0 BG2p0). No OPT. */
    if (ppu->lineHasSprites)
      PPU_PT(SNES_PERF_PPU_SPR, PpuDrawSprites(ppu, y, sub, true));
    PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_8bpp(ppu, y, sub, 0, 0xc000, 0x4000));
    PPU_PT(SNES_PERF_PPU_BG4, PpuDrawBackground_4bpp(ppu, y, sub, 1, 0x9100, 0x1100));
  } else if (ppu->mode == 5) {
    /* Mode 5: BG1 4bpp + BG2 2bpp, 512-wide tiles shown at 256.
     * Same priority ranks as Mode 2/3/4. */
    if (ppu->lineHasSprites)
      PPU_PT(SNES_PERF_PPU_SPR, PpuDrawSprites(ppu, y, sub, true));
    PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_hires(ppu, y, sub, 0, 0xc000, 0x4000, 4));
    PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_hires(ppu, y, sub, 1, 0x9100, 0x1100, 2));
  } else if (ppu->mode == 6) {
    /* Mode 6: BG1 4bpp hires + OPT. Hardware has no BG2.
     * PpuDrawBackground_hires already calls ppu_handleOPT. */
    if (ppu->lineHasSprites)
      PPU_PT(SNES_PERF_PPU_SPR, PpuDrawSprites(ppu, y, sub, true));
    PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_hires(ppu, y, sub, 0, 0xc000, 0x4000, 4));
  } else {
    /* Mode 7 only. */
    PPU_PT(SNES_PERF_PPU_BGX, PpuDrawBackground_mode7(ppu, y, sub, 0x5000));
    if (ppu->lineHasSprites)
      PPU_PT(SNES_PERF_PPU_SPR, PpuDrawSprites(ppu, y, sub, false));
  }
}

#ifdef PPU_RGB565
static inline uint16_t PpuPalette565(const Ppu *ppu, uint32 color) {
  return (uint16_t)(
      (ppu->brightnessMult[color & 0x1f] >> 3) << 11 |
      (ppu->brightnessMult[(color >> 5) & 0x1f] >> 2) << 5 |
      (ppu->brightnessMult[(color >> 10) & 0x1f] >> 3));
}

static void PpuRebuildPalette(Ppu *ppu) {
  for (int i = 0; i < 256; i++)
    ppu->palette565[i] = PpuPalette565(ppu, ppu->cgram[i]);
  ppu->paletteDirty = false;
}

static uint32_t PpuMathFixedKey(Ppu *ppu) {
  uint32_t key = ppu->fixedColorR | ppu->fixedColorG << 5 | ppu->fixedColorB << 10;
  key |= (uint32_t)ppu->subtractColor << 15;
  key |= (uint32_t)ppu->halfColor << 16;
  key |= (uint32_t)ppu->addSubscreen << 17;
  for (int layer = 0; layer < 6; layer++)
    key |= (uint32_t)ppu->mathEnabled[layer] << (18 + layer);
  return key;
}

static inline uint16_t PpuMathFixed565(const Ppu *ppu, uint32_t color, uint32_t mask,
                                       uint32_t r2, uint32_t g2, uint32_t b2,
                                       const uint8_t *color_map) {
  uint32_t r = color & mask, g = color >> 5 & mask, b = color >> 10 & mask;
  if (ppu->subtractColor) {
    r = r >= r2 ? r - r2 : 0;
    g = g >= g2 ? g - g2 : 0;
    b = b >= b2 ? b - b2 : 0;
  } else {
    r += r2, g += g2, b += b2;
  }
  return (uint16_t)((color_map[b] >> 3) | (color_map[g] >> 2) << 5 | (color_map[r] >> 3) << 11);
}

static void PpuRebuildMathFixed(Ppu *ppu, uint32_t key) {
  uint32_t fixed = ppu->fixedColorR | ppu->fixedColorG << 5 | ppu->fixedColorB << 10;
  uint32_t r2 = fixed & 0x1f, g2 = fixed >> 5 & 0x1f, b2 = fixed >> 10 & 0x1f;
  /* With addSubscreen enabled, a transparent subscreen pixel falls back to the
   * fixed color but is deliberately NOT halved (SNES rule, matching the old
   * loop). Otherwise fixed-color half math uses brightnessMultHalf. */
  uint8_t *math_map = ppu->halfColor && !ppu->addSubscreen ?
      ppu->brightnessMultHalf : ppu->brightnessMult;
  /* 3,072 entries, but only FOUR of the twelve rows are ever distinct: `layer`
   * enters the body solely through `do_math = mathEnabled[layer]`, and
   * everything else is a function of (clip, index). Two of those four rows are
   * free besides:
   *
   *   [clip=1][do_math=0] is bit-for-bit palette565[] -- same cgram, same
   *                       brightnessMult, same packing as PpuRebuildPalette
   *   [clip=0][do_math=0] is all zeros: mask 0 sends every component to
   *                       brightnessMult[0], which is 0
   *
   * So compute 512 entries and copy the rest. This matters because the rebuild
   * is not rare: PpuMathFixedKey includes fixedColorR/G/B, so every $2132
   * COLDATA write triggers it, and COLDATA is a routine HDMA target (gradient
   * skies, fades, Zelda 3's rain). Worst case that is once per scanline.
   *
   * A removal, not a test that skips work -- the shape that keeps losing on
   * this chip. Bit-identical by construction. */
  for (int clip = 0; clip < 2; clip++) {
    uint32_t mask = clip ? 0x1f : 0;
    int math_row = -1;
    for (int layer = 0; layer < 8; layer++) {
      uint16_t *row = ppu->mathFixed565[clip][layer];
      /* Layers 6 and 7 never have colour math -- mathEnabled has six entries
       * and the compositing loops rely on those rows holding the plain result. */
      if (layer >= 6 || !ppu->mathEnabled[layer]) {
        if (clip)
          memcpy(row, ppu->palette565, sizeof(ppu->palette565));
        else
          memset(row, 0, 256 * sizeof(uint16_t));
        continue;
      }
      if (math_row >= 0) {
        memcpy(row, ppu->mathFixed565[clip][math_row], 256 * sizeof(uint16_t));
        continue;
      }
      math_row = layer;
      for (int index = 0; index < 256; index++)
        row[index] = PpuMathFixed565(ppu, ppu->cgram[index], mask, r2, g2, b2, math_map);
    }
  }
  ppu->mathFixedKey = key;
}

/* Bring the tables up to date for the CGRAM entries written since the last
 * line. Each entry is computed by the same function the full rebuild uses, so
 * the result is the full rebuild's, entry for entry. `math_too` is false when
 * the colour-math key also changed: the full math rebuild that follows reads
 * palette565 and recomputes every row anyway. */
static void PpuUpdateCgramEntries(Ppu *ppu, bool math_too) {
  uint32_t fixed = ppu->fixedColorR | ppu->fixedColorG << 5 | ppu->fixedColorB << 10;
  uint32_t r2 = fixed & 0x1f, g2 = fixed >> 5 & 0x1f, b2 = fixed >> 10 & 0x1f;
  uint8_t *math_map = ppu->halfColor && !ppu->addSubscreen ?
      ppu->brightnessMultHalf : ppu->brightnessMult;
  for (int w = 0; w < 8; w++) {
    uint32_t bits = g_cgram_dirty[w];
    g_cgram_dirty[w] = 0;
    while (bits) {
      int i = w * 32 + __builtin_ctz(bits);
      bits &= bits - 1;
      uint32 color = ppu->cgram[i];
      uint16_t c565 = PpuPalette565(ppu, color);
      ppu->palette565[i] = c565;
#if SNES_PPU_BLEND_LUT
      g_cgram_spread[i] = PPU_SPREAD(color);
      g_sub_spread[i] = PPU_SPREAD(color) | (i == 0 ? kBlendBypass : 0);
#endif
      if (math_too) {
        /* clip=0 rows do not depend on cgram (mask 0); clip=1 rows are either
         * the palette (no math on that layer) or the math value */
        uint16_t m = PpuMathFixed565(ppu, color, 0x1f, r2, g2, b2, math_map);
        for (int layer = 0; layer < 8; layer++)
          ppu->mathFixed565[1][layer][i] =
              (layer >= 6 || !ppu->mathEnabled[layer]) ? c565 : m;
      }
    }
  }
  g_cgram_dirty_any = false;
}

#endif

PPU_SPLIT_NOINLINE static NOINLINE void PpuDrawWholeLine(Ppu *ppu, uint y) {
  SNES_PERF_PPU_BEGIN(perf_p);
  SNES_PERF_COUNT(snes_perf_ppu_drawn);
  SNES_PERF_COUNT(snes_perf_ppu_mode[ppu->mode & 7]);
  if (ppu->lineHasSprites) SNES_PERF_COUNT(snes_perf_ppu_spr_lines);
#ifdef PPU_RGB565
  bool palette_was_dirty = ppu->paletteDirty;   /* brightness or a load: everything */
  uint32_t math_fixed_key = PpuMathFixedKey(ppu);
  if (palette_was_dirty) {
    PpuRebuildPalette(ppu);
#if SNES_PPU_BLEND_LUT
    PpuRebuildBlendSpread(ppu);
#endif
    for (int w = 0; w < 8; w++) g_cgram_dirty[w] = 0;
    g_cgram_dirty_any = false;
  } else if (g_cgram_dirty_any) {
    /* only the entries written since the last line */
    PpuUpdateCgramEntries(ppu, math_fixed_key == ppu->mathFixedKey && !SNES_ABLATE_MATHFIXED);
    SNES_PERF_COUNT(snes_perf_ppu_cgram_lines);
#ifdef SNES_PPU_TABLE_CHECK
    /* host self-check: the incremental tables must equal a full rebuild */
    if (math_fixed_key == ppu->mathFixedKey) {
      static uint16_t pal[256], mf[2][8][256];
      static uint32_t cs[256], ss[256];
      static unsigned long checks, bad;
      memcpy(pal, ppu->palette565, sizeof pal); memcpy(mf, ppu->mathFixed565, sizeof mf);
      memcpy(cs, g_cgram_spread, sizeof cs); memcpy(ss, g_sub_spread, sizeof ss);
      PpuRebuildPalette(ppu); PpuRebuildMathFixed(ppu, math_fixed_key); PpuRebuildBlendSpread(ppu);
      checks++;
      if (memcmp(pal, ppu->palette565, sizeof pal) || memcmp(mf, ppu->mathFixed565, sizeof mf) ||
          memcmp(cs, g_cgram_spread, sizeof cs) || memcmp(ss, g_sub_spread, sizeof ss))
        fprintf(stderr, "TABLECHECK MISMATCH line %u (%lu)\n", y, ++bad);
      if ((checks & 0x3ff) == 0) fprintf(stderr, "TABLECHECK %lu checks, %lu bad\n", checks, bad);
    }
#endif
  }
#if SNES_MATHFIXED_CENSUS
  g_mathfixed_lines++;
  if (palette_was_dirty || math_fixed_key != ppu->mathFixedKey) g_mathfixed_rebuilds++;
#endif
#if !SNES_ABLATE_MATHFIXED
  if (palette_was_dirty || math_fixed_key != ppu->mathFixedKey) {
    PpuRebuildMathFixed(ppu, math_fixed_key);
    SNES_PERF_COUNT(snes_perf_ppu_math_rebuilds);
  }
#else
  /* ABLATION, WRONG OUTPUT. The table is 3,072 entries and its key includes
   * fixedColorR/G/B, so every $2132 COLDATA write invalidates it -- and COLDATA
   * is a routine HDMA target (gradient skies, fades, Zelda 3's rain), which the
   * comment on PpuRebuildMathFixed says can mean once per scanline. With
   * SNES_PPU_BLEND_LUT in, only the 2.6% of pixels that bypass the blend still
   * read the table, so a per-line rebuild would be almost pure waste.
   *
   * COUNTED, AND IT DOES NOT HAPPEN HERE: 27 rebuilds in 124,768 rendered lines
   * (SNES_MATHFIXED_CENSUS). The worry is real for some scene; it is not this
   * one. Lever closed by a count before anything was built for it. */
  ppu->mathFixedKey = math_fixed_key;
#endif
#if SNES_PPU_BLEND_LUT
  { uint32_t bk = (uint32_t)ppu->brightness << 1 | (ppu->halfColor ? 1u : 0u);
    if (palette_was_dirty || bk != g_blend_key) { PpuRebuildBlendLut(ppu); g_blend_key = bk; } }
#endif
#endif
  SNES_PERF_PPU_END(SNES_PERF_PPU_PRE, perf_p);
  if (ppu->forcedBlank) {
    uint8 *dst = &ppu->renderBuffer[(y - 1) * ppu->renderPitch];
#ifdef PPU_RGB565
    size_t n = sizeof(uint16_t) * (256 + ppu->extraLeftRight * 2);
#else
    size_t n = sizeof(uint32) * (256 + ppu->extraLeftRight * 2);
#endif
    memset(dst, 0, n);
#ifdef TARGET_GNW
    if (g_ppu_line_cb)
      g_ppu_line_cb(y, (const uint16_t *)dst);
#endif
    return;
  }

#ifdef HOST_BUILD
  if (getenv("HOST_PPU_LOG") && y == 1) {
    static uint32 prev;
    uint32 key = (uint32)ppu->mode
      | ((uint32)ppu->bgLayer[0].bigTiles << 3)
      | ((uint32)ppu->bgLayer[1].bigTiles << 4)
      | ((uint32)ppu->mosaicEnabled << 8)
      | ((uint32)ppu->screenEnabled[0] << 16)
      | ((uint32)ppu->screenEnabled[1] << 24);
    if (key != prev) {
      prev = key;
      fprintf(stderr,
              "ppu: mode=%u blank=%d bright=%u clip=%u prevent=%u "
              "big=%d%d%d%d mosaic=%02x size=%u start=%u bgmos=%d%d%d%d "
              "tm=%02x ts=%02x tw=%02x tsw=%02x "
              "math=%d%d%d%d%d%d addsub=%d hires=%d sprites=%d direct=%d "
              "win=%u-%u/%u-%u extra=%u "
              "cgram0=%03x cgram1=%03x c32=%03x c64=%03x c96=%03x\n",
              ppu->mode, (int)ppu->forcedBlank, ppu->brightness,
              ppu->clipMode, ppu->preventMathMode,
              (int)ppu->bgLayer[0].bigTiles, (int)ppu->bgLayer[1].bigTiles,
              (int)ppu->bgLayer[2].bigTiles, (int)ppu->bgLayer[3].bigTiles,
              ppu->mosaicEnabled, ppu->mosaicSize, ppu->mosaicStartLine,
              (int)ppu->bgLayer[0].mosaicEnabled, (int)ppu->bgLayer[1].mosaicEnabled,
              (int)ppu->bgLayer[2].mosaicEnabled, (int)ppu->bgLayer[3].mosaicEnabled,
              ppu->screenEnabled[0], ppu->screenEnabled[1],
              ppu->screenWindowed[0], ppu->screenWindowed[1],
              (int)ppu->mathEnabled[0], (int)ppu->mathEnabled[1],
              (int)ppu->mathEnabled[2], (int)ppu->mathEnabled[3],
              (int)ppu->mathEnabled[4], (int)ppu->mathEnabled[5],
              (int)ppu->addSubscreen, (int)ppu->pseudoHires,
              (int)ppu->lineHasSprites, (int)ppu->directColor,
              ppu->window1left, ppu->window1right,
              ppu->window2left, ppu->window2right,
              ppu->extraLeftRight,
              ppu->cgram[0], ppu->cgram[1],
              ppu->cgram[32], ppu->cgram[64], ppu->cgram[96]);
    }
  }
  if (getenv("HOST_PPU_SLOW")) {
    for (int x = 0; x < 256; x++)
      ppu_handlePixel(ppu, x, (int)y);
#ifdef TARGET_GNW
    if (g_ppu_line_cb)
      g_ppu_line_cb(y, (const uint16_t *)&ppu->renderBuffer[(y - 1) * ppu->renderPitch]);
#endif
    return;
  }
#endif

#if SNES_WINDOW_CENSUS
  g_win_lines++;
#endif
#if SNES_RENDER_CENSUS
  g_render_lines++;
#endif
  // Default background is backdrop
#if SNES_ABLATE_BG == 3
  /* ABLATION, WRONG OUTPUT. ClearBackdrop fell through BOTH earlier ablations --
   * =1 returns at the top of the layer drawer and =2 empties the pixel macros,
   * and this call is in neither -- so its 1 KB of stores per line (two buffers,
   * 224 lines, every drawn frame) has never been priced. Diagnostic only.
   *
   * PRICED: nothing. 55.59 against a 55.58 baseline. A kilobyte of sequential
   * stores per line does not show up in this frame budget -- which is the same
   * lesson the tile memo taught from the other side. On this part sequential
   * writes are cheap; what costs is reading a cold line at a random address. */
  PPU_BUF_MARK(0);   /* the ablation below leaves the buffer dirty */
#else
  ClearBackdrop(&ppu->bgBuffers[0]);
  PPU_BUF_CLEAN(0);
#endif

  // Render main screen
  SNES_PERF_PPU_BEGIN(perf_m);
  PpuDrawBackgrounds(ppu, y, false);
  SNES_PERF_PPU_END(SNES_PERF_PPU_MAIN, perf_m);

  // The 6:th bit is automatically zero, math is never applied to the first half of the sprites.
  uint32 math_enabled = 0;
  for(int i = 0; i < 6; i++)
    math_enabled |= ppu->mathEnabled[i] << i;

  // Render also the subscreen?
  bool rendered_subscreen = false;
  if (ppu->preventMathMode != 3 && ppu->addSubscreen && math_enabled) {
#if SNES_ABLATE_BG != 3
    ClearBackdrop(&ppu->bgBuffers[1]);
    PPU_BUF_CLEAN(1);
#else
    PPU_BUF_MARK(1);
#endif
    if (ppu->screenEnabled[1] != 0) {
#if SNES_RENDER_CENSUS
      g_sub_lines++;
#endif
      SNES_PERF_PPU_BEGIN(perf_s);
      PpuDrawBackgrounds(ppu, y, true);
      SNES_PERF_PPU_END(SNES_PERF_PPU_SUB, perf_s);
      SNES_PERF_COUNT(snes_perf_ppu_sub_lines);
      rendered_subscreen = true;
    }
  }

  // Color window affects the drawing mode in each region
  SNES_PERF_PPU_BEGIN(perf_c);
  if (ppu->preventMathMode != 3 && math_enabled) SNES_PERF_COUNT(snes_perf_ppu_math_lines);
  PpuWindows cwin;
  PpuWindows_Calc(&cwin, ppu, 5);
  static const uint8 kCwBitsMod[8] = {
    0x00, 0xff, 0xff, 0x00,
    0xff, 0x00, 0xff, 0x00,
  };
  uint32 cw_clip_math = ((cwin.bits & kCwBitsMod[ppu->clipMode]) ^ kCwBitsMod[ppu->clipMode + 4]) |
    ((cwin.bits & kCwBitsMod[ppu->preventMathMode]) ^ kCwBitsMod[ppu->preventMathMode + 4]) << 8;

#ifdef PPU_RGB565
  uint16_t *dst = (uint16_t*)&ppu->renderBuffer[(y - 1) * ppu->renderPitch], *dst_org = dst;
#else
  uint32 *dst = (uint32*)&ppu->renderBuffer[(y - 1) * ppu->renderPitch], *dst_org = dst;
#endif

  dst += (ppu->extraLeftRight - ppu->extraLeftCur);
  SNES_PERF_PPU_END(SNES_PERF_PPU_CWIN, perf_c);

#if SNES_ABLATE_COMPOSITE
  /* ABLATION, WRONG OUTPUT ON PURPOSE. Deletes the whole compositing pass --
   * the colour window walk, the per-pixel main/sub selection, the colour maths
   * and the palette lookup -- and writes a flat line instead.
   *
   * Why it needs pricing: the layer draw is 4.4 fps, which at 1 frame drawn in 4
   * is about 5.4 ms of the 17.65 ms a drawn frame costs. The other 12 ms has
   * never been ablated, and this pass is most of it: 256 pixels a line with a
   * palette lookup each, against the layer draw's 65 tiles. */
  { size_t n = (size_t)(256 + ppu->extraLeftRight * 2);
    for (size_t i = 0; i < n; i++) dst[i] = (uint16_t)(y * 3);
  }
#else
  uint32 windex = 0;
  do {
    uint32 left = cwin.edges[windex] + kPpuExtraLeftRight, right = cwin.edges[windex + 1] + kPpuExtraLeftRight;
    // If clip is set, then zero out the rgb values from the main screen.
    uint32 clip_color_mask = (cw_clip_math & 1) ? 0x1f : 0;
    uint32 math_enabled_cur = (cw_clip_math & 0x100) ? math_enabled : 0;
    uint32 fixed_color = ppu->fixedColorR | ppu->fixedColorG << 5 | ppu->fixedColorB << 10;
    if (math_enabled_cur == 0 || fixed_color == 0 && !ppu->halfColor && !rendered_subscreen) {
      // Math is disabled (or has no effect), so can avoid the per-pixel maths check
      uint32 i = left;
#ifdef PPU_RGB565
      if (clip_color_mask == 0x1f) {
        const uint16_t *pal = ppu->palette565;
        const PpuZbufType *src = ppu->bgBuffers[0].data;
        /* Two pixels per iteration: one 32-bit load of two z-entries, one 32-bit
         * store of two RGB565 pixels (little-endian pairing, like the 64-bit fill
         * in ClearBackdrop). src and dst advance in lockstep, so when they are
         * co-aligned one odd head pixel word-aligns both; when they are not
         * (odd render pitch), the plain tail loop does the whole span. */
        if ((((uintptr_t)dst ^ (uintptr_t)&src[i]) & 3) == 0) {
          if ((uintptr_t)dst & 3)
            dst[0] = pal[src[i] & 0xff], dst++, i++;
          for (; i + 1 < right; i += 2, dst += 2) {
            uint32 zz = *(const uint32 *)&src[i];
            *(uint32 *)dst = pal[zz & 0xff] | (uint32)pal[(zz >> 16) & 0xff] << 16;
          }
        }
        for (; i < right; i++, dst++)
          dst[0] = pal[src[i] & 0xff];
      } else {
        /* clip: every component masks to index 0, and brightnessMult[0] is 0 */
        do {
          dst[0] = 0;
        } while (dst++, ++i < right);
      }
#else
      do {
        uint32 color = ppu->cgram[ppu->bgBuffers[0].data[i] & 0xff];
        dst[0] = ppu->brightnessMult[color & clip_color_mask] << 16 |
          ppu->brightnessMult[(color >> 5) & clip_color_mask] << 8 |
          ppu->brightnessMult[(color >> 10) & clip_color_mask];
      } while (dst++, ++i < right);
#endif
    } else {
#if defined(PPU_RGB565) && defined(SNES_PPU_DIRECT_MATH)
      /* No subscreen means the result is solely a function of the main z/color
       * word and clip state.  Its low 12 bits are already laid out exactly as
       * mathFixed565[layer][index]. One lookup replaces math-enable branches
       * and the per-pixel fixed/subscreen test. Layer 6 (OBJ palettes exempt
       * from color math) falls back to the already-built plain palette. */
      if (!ppu->addSubscreen) {
        const uint16_t *direct = &ppu->mathFixed565[clip_color_mask != 0][0][0];
        const PpuZbufType *src = ppu->bgBuffers[0].data;
        uint32 i = left;
        /* One masked load. Rows 6/7 exist precisely so the old `layer < 6`
         * test and its alternate arm can go: they hold the same plain-palette
         * (clip) or zero (clipped) values that arm produced. */
        do {
          dst[0] = direct[src[i] & 0x7ff];
        } while (dst++, ++i < right);
        continue;
      }
#endif
#if SNES_COMP_CENSUS
      g_comp_lines++;
      if (ppu->brightness == 15) g_comp_bright++;
#endif
      uint8 *half_color_map = ppu->halfColor ? ppu->brightnessMultHalf : ppu->brightnessMult;
      /* The z word already stores [layer:4][CGRAM index:8] in its low 12 bits,
       * exactly matching the last two dimensions of mathFixed565. */
      const uint16_t *math_fixed = &ppu->mathFixed565[clip_color_mask != 0][0][0];
      // Store this in locals
      math_enabled_cur |= ppu->addSubscreen << 8 | ppu->subtractColor << 9;
      // Need to check for each pixel whether to use math or not based on the main screen layer.
      uint32 i = left;
#if SNES_PPU_BLEND_LUT && defined(PPU_RGB565)
      /* The shape this scene actually is: add-subscreen, no subtract, no clip.
       * Everything variable inside the blend collapses to two table reads, one
       * add and three positioned lookups. The bypass test stays -- it is 2.6% of
       * pixels here but it is not free to be wrong about. */
      if ((math_enabled_cur & 0x300) == 0x100 && clip_color_mask == 0x1f) {
        const PpuZbufType *mrow = ppu->bgBuffers[0].data;
        const PpuZbufType *srow = ppu->bgBuffers[1].data;
        /* When every layer has maths enabled -- which is what a full-screen
         * translucency looks like -- the per-pixel layer test is a shift, an AND
         * and a branch that can never fail. Hoist it: the buffers only ever hold
         * layers 0-5 plus the backdrop's own 0x05, and 0x05's bit is inside the
         * mask too, so the whole test is redundant and only the subscreen
         * emptiness check remains. */
        if ((math_enabled_cur & 0x3f) == 0x3f) {
          /* Pairing the loads and the store was tried here and LOSES: 57.03
           * against 57.60 on the device, the same -0.6 the older SNES_PPU_PAIR
           * experiment cost, and the rig said so too (+4,706 instructions a
           * frame). It does not remove work, it reshapes it, and this loop does
           * not reward that. Everything that has won today removed work that
           * was provably unnecessary. */
          /* Folding both bypass tests into a sentinel bit carried by the
           * sub table was tried and measured 57.53 against 57.60: it does not
           * delete either test, it adds an OR. The sentinel stays in the table
           * because it costs nothing there; the loop asks plainly.
           * The blue field needs no mask -- it is the top field, and 62 at
           * bit 22 cannot reach bit 28. */
          PPU_PT(SNES_PERF_PPU_SPAN, dst = PpuBlendSpanAll(dst, mrow, srow, i, right, math_fixed));
          continue;
        }
        /* Field order follows the SNES word: bits 0-4 red, 5-9 green,
         * 10-14 blue -- so the spread keeps red at 0, green at 11, blue at
         * 22, and the three tables are read in that same order. */
        PPU_PT(SNES_PERF_PPU_SPAN, dst = PpuBlendSpanPart(dst, mrow, srow, i, right, math_fixed, math_enabled_cur));
        continue;
      }
#endif
/* SNES_PPU_PAIR: OFF, by device measurement, and the reason is worth keeping.
 *
 * The no-math branch above pairs its pixels and wins; doing the same here for
 * the pixels that bypass the blend LOSES: Zelda 3's rain, the deterministic
 * 900-frame window from a savestate, three runs each -- 50.39 fps without,
 * 47.99 with. -4.8%%. No pixel changes either way (framebuffer, state and audio
 * hashes identical on the rig), so this is purely what the extra per-pixel
 * tests cost when the pair does NOT qualify, which in a translucent scene is
 * most of them.
 *
 * The host rig had already said so -- +1.1%% instructions on ALttP's first 400
 * frames -- and I discounted it because that scene has little colour math. The
 * rig was measuring the right thing: this loop is sensitive to added branches,
 * not to store width. Any future attempt here should REMOVE work rather than
 * add a test that has to be paid before it can pay off. */
#ifndef SNES_PPU_PAIR
#define SNES_PPU_PAIR 0
#endif
#if defined(PPU_RGB565) && defined(SNES_PPU_DIRECT_MATH) && SNES_PPU_PAIR
      /* Two pixels per iteration while BOTH of them bypass the blend.
       *
       * The no-math branch above already pairs its pixels -- one 32-bit load of
       * two z-entries, one 32-bit store of two RGB565 pixels -- and this branch
       * did not, although the pixels reaching it are overwhelmingly the same
       * shape: a translucency effect covers part of a scanline, not all of it,
       * so most pixels on a colour-math line still take the one-lookup bypass
       * below. That is not a guess about typical content -- on the device, in
       * Zelda 3's rain, PpuDrawWholeLine is 7.4%% of the frame and this is the
       * loop it is in.
       *
       * A pair is taken only when both pixels qualify. The moment one does not,
       * the scalar body handles that single pixel and pairing resumes on the
       * next -- so the fast path never has to reproduce the blend, and a fully
       * translucent line pays one extra test per pixel instead. */
      const PpuZbufType *mrow = ppu->bgBuffers[0].data;
      const PpuZbufType *srow = ppu->bgBuffers[1].data;
      const uint32 pair_sub = math_enabled_cur & 0x100;   /* addSubscreen */
      /* dst and both z rows must agree in their low two address bits or a
       * 32-bit access would be unaligned on one of them. Checked once, not per
       * pixel; when it fails the scalar body does the whole span as before. */
      const bool pair_aligned =
          ((((uintptr_t)dst ^ (uintptr_t)&mrow[i]) |
            ((uintptr_t)&mrow[i] ^ (uintptr_t)&srow[i])) & 3) == 0;
#endif
      while (i < right) {
#if defined(PPU_RGB565) && defined(SNES_PPU_DIRECT_MATH) && SNES_PPU_PAIR
        if (pair_aligned && i + 1 < right && !((uintptr_t)dst & 3)) {
          uint32 mm = *(const uint32 *)&mrow[i];
          uint32 l0 = (mm >> 8) & 0xf, l1 = (mm >> 24) & 0xf;
          if (l0 < 6 && l1 < 6) {
            uint32 ss = pair_sub ? *(const uint32 *)&srow[i] : 0;
            if ((!(math_enabled_cur & (1u << l0)) || !pair_sub || !(ss & 0xff)) &&
                (!(math_enabled_cur & (1u << l1)) || !pair_sub || !(ss & 0xff0000))) {
              *(uint32 *)dst = (uint32)math_fixed[mm & 0xfff] |
                               (uint32)math_fixed[(mm >> 16) & 0xfff] << 16;
              dst += 2, i += 2;
              continue;
            }
          }
        }
#endif
        PpuZbufType main_z = ppu->bgBuffers[0].data[i];
        uint8 main_layer = (main_z >> 8) & 0xf;
        /* Fixed-color, transparent-subscreen AND math-disabled-layer pixels are
         * all a pure function of clip state, main layer and CGRAM index. When
         * this layer's mathEnabled bit is off, PpuRebuildMathFixed() built its
         * table entry with do_math=false — the exact same brightnessMult-only
         * formula the manual path below falls through to when the per-pixel
         * `math_enabled_cur & (1 << main_layer)` test fails. So a bypassing
         * pixel needs neither the real subscreen value nor the manual
         * extract/blend/repack below; it needs the same one lookup the
         * fixed-color case already uses. One lookup replaces component
         * extraction, layer test, add/subtract, clamp and RGB565 packing. */
        /* No `main_layer < 6` here either: math_enabled_cur only ever has bits
         * 0-5, so a layer-6 pixel fails the enable test and takes this bypass,
         * and row 6 now holds the value it needs. */
        if (!(math_enabled_cur & (1 << main_layer)) ||
            !ppu->addSubscreen || (ppu->bgBuffers[1].data[i] & 0xff) == 0) {
#if SNES_COMP_CENSUS
          g_comp_bypass++;
          if (ppu->addSubscreen && (math_enabled_cur & (1 << main_layer)) &&
              (ppu->bgBuffers[1].data[i] & 0xff) == 0) g_comp_subzero++;
#endif
          dst[0] = math_fixed[main_z & 0x7ff];
          dst++, i++;
          continue;
        }
#if SNES_COMP_CENSUS
        g_comp_blend++;
#endif
        uint32 color = ppu->cgram[main_z & 0xff], color2;
        uint32 r = color & clip_color_mask;
        uint32 g = (color >> 5) & clip_color_mask;
        uint32 b = (color >> 10) & clip_color_mask;
        uint8 *color_map = ppu->brightnessMult;
        if (math_enabled_cur & (1 << main_layer)) {
          if (math_enabled_cur & 0x100) {  // addSubscreen ?
            if ((ppu->bgBuffers[1].data[i] & 0xff) != 0)
              color2 = ppu->cgram[ppu->bgBuffers[1].data[i] & 0xff], color_map = half_color_map;
            else  // Don't halve if ppu->addSubscreen && backdrop
              color2 = fixed_color;
          } else {
            color2 = fixed_color, color_map = half_color_map;
          }
          uint32 r2 = (color2 & 0x1f), g2 = ((color2 >> 5) & 0x1f), b2 = ((color2 >> 10) & 0x1f);
          if (math_enabled_cur & 0x200) {  // subtractColor?
            r = (r >= r2) ? r - r2 : 0;
            g = (g >= g2) ? g - g2 : 0;
            b = (b >= b2) ? b - b2 : 0;
          } else {
            r += r2;
            g += g2;
            b += b2;
          }
        }
#ifdef PPU_RGB565
        dst[0] = (color_map[b] >> 3) | (color_map[g] >> 2) << 5 | (color_map[r] >> 3) << 11;
#else
        dst[0] = color_map[b] | color_map[g] << 8 | color_map[r] << 16;
#endif
        dst++, i++;
      }
    }
  } while (cw_clip_math >>= 1, ++windex < cwin.nr);
#endif
  SNES_PERF_PPU_END(SNES_PERF_PPU_COMP, perf_c);

#ifdef TARGET_GNW
  if (g_ppu_line_cb) {
    SNES_PERF_PPU_BEGIN(perf_b);
    g_ppu_line_cb(y, dst_org);
    SNES_PERF_PPU_END(SNES_PERF_PPU_BLIT, perf_b);
  }
#endif
}


static void ppu_handlePixel(Ppu* ppu, int x, int y) {
  int r = 0, r2 = 0;
  int g = 0, g2 = 0;
  int b = 0, b2 = 0;
  if(!ppu->forcedBlank) {
    int mainLayer = ppu_getPixel(ppu, x, y, false, &r, &g, &b);
    bool colorWindowState = ppu_getWindowState(ppu, 5, x);
    if(
      ppu->clipMode == 3 ||
      (ppu->clipMode == 2 && colorWindowState) ||
      (ppu->clipMode == 1 && !colorWindowState)
    ) {
      r = 0;
      g = 0;
      b = 0;
    }
    int secondLayer = 5; // backdrop
    bool mathEnabled = mainLayer < 6 && ppu->mathEnabled[mainLayer] && !(
      ppu->preventMathMode == 3 ||
      (ppu->preventMathMode == 2 && colorWindowState) ||
      (ppu->preventMathMode == 1 && !colorWindowState)
    );
    if((mathEnabled && ppu->addSubscreen) || ppu->pseudoHires || ppu->mode == 5 || ppu->mode == 6) {
      secondLayer = ppu_getPixel(ppu, x, y, true, &r2, &g2, &b2);
    }
    // TODO: subscreen pixels can be clipped to black as well
    // TODO: math for subscreen pixels (add/sub sub to main)
    if(mathEnabled) {
      if(ppu->subtractColor) {
        r -= (ppu->addSubscreen && secondLayer != 5) ? r2 : ppu->fixedColorR;
        g -= (ppu->addSubscreen && secondLayer != 5) ? g2 : ppu->fixedColorG;
        b -= (ppu->addSubscreen && secondLayer != 5) ? b2 : ppu->fixedColorB;
      } else {
        r += (ppu->addSubscreen && secondLayer != 5) ? r2 : ppu->fixedColorR;
        g += (ppu->addSubscreen && secondLayer != 5) ? g2 : ppu->fixedColorG;
        b += (ppu->addSubscreen && secondLayer != 5) ? b2 : ppu->fixedColorB;
      }
      if(ppu->halfColor && (secondLayer != 5 || !ppu->addSubscreen)) {
        r >>= 1;
        g >>= 1;
        b >>= 1;
      }
      if(r > 31) r = 31;
      if(g > 31) g = 31;
      if(b > 31) b = 31;
      if(r < 0) r = 0;
      if(g < 0) g = 0;
      if(b < 0) b = 0;
    }
    if(!(ppu->pseudoHires || ppu->mode == 5 || ppu->mode == 6)) {
      r2 = r; g2 = g; b2 = b;
    }
  }
  int row = y - 1;
#ifdef PPU_RGB565
  uint8 *pixelBuffer = (uint8*) &ppu->renderBuffer[row * ppu->renderPitch + (x + ppu->extraLeftRight) * 2];
  uint32 r8 = ((r << 3) | (r >> 2)) * ppu->brightness / 15;
  uint32 g8 = ((g << 3) | (g >> 2)) * ppu->brightness / 15;
  uint32 b8 = ((b << 3) | (b >> 2)) * ppu->brightness / 15;
  uint16_t px = (uint16_t)(((r8 >> 3) << 11) | ((g8 >> 2) << 5) | (b8 >> 3));
  pixelBuffer[0] = (uint8)px;
  pixelBuffer[1] = (uint8)(px >> 8);
#else
  uint8 *pixelBuffer = (uint8*) &ppu->renderBuffer[row * ppu->renderPitch + (x + ppu->extraLeftRight) * 4];
  pixelBuffer[0] = ((b << 3) | (b >> 2)) * ppu->brightness / 15;
  pixelBuffer[1] = ((g << 3) | (g >> 2)) * ppu->brightness / 15;
  pixelBuffer[2] = ((r << 3) | (r >> 2)) * ppu->brightness / 15;
  pixelBuffer[3] = 0;
#endif
}

static int ppu_getPixel(Ppu* ppu, int x, int y, bool sub, int* r, int* g, int* b) {
  // figure out which color is on this location on main- or subscreen, sets it in r, g, b
  // returns which layer it is: 0-3 for bg layer, 4 or 6 for sprites (depending on palette), 5 for backdrop
  int actMode = ppu->mode == 1 && ppu->bg3priority ? 8 : ppu->mode;
  actMode = ppu->mode == 7 && ppu->m7extBg ? 9 : actMode;
  int layer = 5;
  int pixel = 0;
  for(int i = 0; i < layerCountPerMode[actMode]; i++) {
    int curLayer = layersPerMode[actMode][i];
    int curPriority = prioritysPerMode[actMode][i];
    bool layerActive = false;
    if(!sub) {
      layerActive = ppu->layer[curLayer].mainScreenEnabled && (
        !ppu->layer[curLayer].mainScreenWindowed || !ppu_getWindowState(ppu, curLayer, x)
      );
    } else {
      layerActive = ppu->layer[curLayer].subScreenEnabled && (
        !ppu->layer[curLayer].subScreenWindowed || !ppu_getWindowState(ppu, curLayer, x)
      );
    }
    if(layerActive) {
      if(curLayer < 4) {
        // bg layer
        int lx = x;
        int ly = y;
        if(ppu->bgLayer[curLayer].mosaicEnabled && ppu->mosaicSize > 1) {
          lx -= lx % ppu->mosaicSize;
          ly -= (ly - ppu->mosaicStartLine) % ppu->mosaicSize;
        }
        if(ppu->mode == 7) {
          pixel = ppu_getPixelForMode7(ppu, lx, curLayer, curPriority);
        } else {
          lx += ppu->bgLayer[curLayer].hScroll;
          if(ppu->mode == 5 || ppu->mode == 6) {
            lx *= 2;
            lx += (sub || ppu->bgLayer[curLayer].mosaicEnabled) ? 0 : 1;
            if(ppu->interlace) {
              ly *= 2;
              ly += (ppu->evenFrame || ppu->bgLayer[curLayer].mosaicEnabled) ? 0 : 1;
            }
          }
          ly += ppu->bgLayer[curLayer].vScroll;
          if(ppu->mode == 2 || ppu->mode == 4 || ppu->mode == 6) {
            ppu_handleOPT(ppu, curLayer, &lx, &ly);
          }
          pixel = ppu_getPixelForBgLayer(
            ppu, lx & 0x3ff, ly & 0x3ff,
            curLayer, curPriority
          );
        }
      } else {
        // get a pixel from the sprite buffer
        pixel = 0;
        if ((ppu->objBuffer.data[x + kPpuExtraLeftRight] >> 12) == SPRITE_PRIO_TO_PRIO_HI(curPriority))
          pixel = ppu->objBuffer.data[x + kPpuExtraLeftRight] & 0xff;
      }
    }
    if(pixel > 0) {
      layer = curLayer;
      break;
    }
  }
  if(ppu->directColor && layer < 4 && bitDepthsPerMode[actMode][layer] == 8) {
    *r = ((pixel & 0x7) << 2) | ((pixel & 0x100) >> 7);
    *g = ((pixel & 0x38) >> 1) | ((pixel & 0x200) >> 8);
    *b = ((pixel & 0xc0) >> 3) | ((pixel & 0x400) >> 8);
  } else {
    uint16_t color = ppu->cgram[pixel & 0xff];
    *r = color & 0x1f;
    *g = (color >> 5) & 0x1f;
    *b = (color >> 10) & 0x1f;
  }
  if(layer == 4 && pixel < 0xc0) layer = 6; // sprites with palette color < 0xc0
  return layer;
}

static void ppu_handleOPT(Ppu* ppu, int layer, int* lx, int* ly) {
  int x = *lx;
  int y = *ly;
  int column = 0;
  if(ppu->mode == 6) {
    column = ((x - (x & 0xf)) - ((ppu->bgLayer[layer].hScroll * 2) & 0xfff0)) >> 4;
  } else {
    column = ((x - (x & 0x7)) - (ppu->bgLayer[layer].hScroll & 0xfff8)) >> 3;
  }
  if(column > 0) {
    // fetch offset values from layer 3 tilemap
    int valid = layer == 0 ? 0x2000 : 0x4000;
    uint16_t hOffset = ppu_getOffsetValue(ppu, column - 1, 0);
    uint16_t vOffset = 0;
    if(ppu->mode == 4) {
      if(hOffset & 0x8000) {
        vOffset = hOffset;
        hOffset = 0;
      }
    } else {
      vOffset = ppu_getOffsetValue(ppu, column - 1, 1);
    }
    if(ppu->mode == 6) {
      // TODO: not sure if correct
      if(hOffset & valid) *lx = (((hOffset & 0x3f8) + (column * 8)) * 2) | (x & 0xf);
    } else {
      if(hOffset & valid) *lx = ((hOffset & 0x3f8) + (column * 8)) | (x & 0x7);
    }
    // TODO: not sure if correct for interlace
    if(vOffset & valid) *ly = (vOffset & 0x3ff) + (y - ppu->bgLayer[layer].vScroll);
  }
}

static uint16_t ppu_getOffsetValue(Ppu* ppu, int col, int row) {
  int x = col * 8 + ppu->bgLayer[2].hScroll;
  int y = row * 8 + ppu->bgLayer[2].vScroll;
  int tileBits = ppu->bgLayer[2].bigTiles ? 4 : 3;
  int tileHighBit = ppu->bgLayer[2].bigTiles ? 0x200 : 0x100;
  uint16_t tilemapAdr = ppu->bgLayer[2].tilemapAdr + (((y >> tileBits) & 0x1f) << 5 | ((x >> tileBits) & 0x1f));
  if((x & tileHighBit) && ppu->bgLayer[2].tilemapWider) tilemapAdr += 0x400;
  if((y & tileHighBit) && ppu->bgLayer[2].tilemapHigher) tilemapAdr += ppu->bgLayer[2].tilemapWider ? 0x800 : 0x400;
  return ppu->vram[tilemapAdr & 0x7fff];
}

static bool PpuMode4HasOpt(Ppu *ppu) {
  for (int c = 0; c < 32; c++) {
    if (ppu_getOffsetValue(ppu, c, 0) & 0x6000)
      return true;
  }
  return false;
}

static bool PpuMode2HasOpt(Ppu *ppu) {
  for (int c = 0; c < 32; c++) {
    if ((ppu_getOffsetValue(ppu, c, 0) | ppu_getOffsetValue(ppu, c, 1)) & 0x6000)
      return true;
  }
  return false;
}

static int ppu_getPixelForBgLayer(Ppu* ppu, int x, int y, int layer, bool priority) {
  // figure out address of tilemap word and read it
  bool wideTiles = ppu->bgLayer[layer].bigTiles || ppu->mode == 5 || ppu->mode == 6;
  int tileBitsX = wideTiles ? 4 : 3;
  int tileHighBitX = wideTiles ? 0x200 : 0x100;
  int tileBitsY = ppu->bgLayer[layer].bigTiles ? 4 : 3;
  int tileHighBitY = ppu->bgLayer[layer].bigTiles ? 0x200 : 0x100;
  uint16_t tilemapAdr = ppu->bgLayer[layer].tilemapAdr + (((y >> tileBitsY) & 0x1f) << 5 | ((x >> tileBitsX) & 0x1f));
  if((x & tileHighBitX) && ppu->bgLayer[layer].tilemapWider) tilemapAdr += 0x400;
  if((y & tileHighBitY) && ppu->bgLayer[layer].tilemapHigher) tilemapAdr += ppu->bgLayer[layer].tilemapWider ? 0x800 : 0x400;
  uint16_t tile = ppu->vram[tilemapAdr & 0x7fff];
  // check priority, get palette
  if(((bool) (tile & 0x2000)) != priority) return 0; // wrong priority
  int paletteNum = (tile & 0x1c00) >> 10;
  // figure out position within tile
  int row = (tile & 0x8000) ? 7 - (y & 0x7) : (y & 0x7);
  int col = (tile & 0x4000) ? (x & 0x7) : 7 - (x & 0x7);
  int tileNum = tile & 0x3ff;
  if(wideTiles) {
    // if unflipped right half of tile, or flipped left half of tile
    if(((bool) (x & 8)) ^ ((bool) (tile & 0x4000))) tileNum += 1;
  }
  if(ppu->bgLayer[layer].bigTiles) {
    // if unflipped bottom half of tile, or flipped upper half of tile
    if(((bool) (y & 8)) ^ ((bool) (tile & 0x8000))) tileNum += 0x10;
  }
  // read tiledata, ajust palette for mode 0
  int bitDepth = bitDepthsPerMode[ppu->mode][layer];
  if(ppu->mode == 0) paletteNum += 8 * layer;
  // plane 1 (always)
  int paletteSize = 4;
  uint16_t plane1 = ppu->vram[(ppu->bgLayer[layer].tileAdr + ((tileNum & 0x3ff) * 4 * bitDepth) + row) & 0x7fff];
  int pixel = (plane1 >> col) & 1;
  pixel |= ((plane1 >> (8 + col)) & 1) << 1;
  // plane 2 (for 4bpp, 8bpp)
  if(bitDepth > 2) {
    paletteSize = 16;
    uint16_t plane2 = ppu->vram[(ppu->bgLayer[layer].tileAdr + ((tileNum & 0x3ff) * 4 * bitDepth) + 8 + row) & 0x7fff];
    pixel |= ((plane2 >> col) & 1) << 2;
    pixel |= ((plane2 >> (8 + col)) & 1) << 3;
  }
  // plane 3 & 4 (for 8bpp)
  if(bitDepth > 4) {
    paletteSize = 256;
    uint16_t plane3 = ppu->vram[(ppu->bgLayer[layer].tileAdr + ((tileNum & 0x3ff) * 4 * bitDepth) + 16 + row) & 0x7fff];
    pixel |= ((plane3 >> col) & 1) << 4;
    pixel |= ((plane3 >> (8 + col)) & 1) << 5;
    uint16_t plane4 = ppu->vram[(ppu->bgLayer[layer].tileAdr + ((tileNum & 0x3ff) * 4 * bitDepth) + 24 + row) & 0x7fff];
    pixel |= ((plane4 >> col) & 1) << 6;
    pixel |= ((plane4 >> (8 + col)) & 1) << 7;
  }
  // return cgram index, or 0 if transparent, palette number in bits 10-8 for 8-color layers
  return pixel == 0 ? 0 : paletteSize * paletteNum + pixel;
}

static void ppu_calculateMode7Starts(Ppu* ppu, int y) {
  // expand 13-bit values to signed values
  int hScroll = ((int16_t) (ppu->m7matrix[6] << 3)) >> 3;
  int vScroll = ((int16_t) (ppu->m7matrix[7] << 3)) >> 3;
  int xCenter = ((int16_t) (ppu->m7matrix[4] << 3)) >> 3;
  int yCenter = ((int16_t) (ppu->m7matrix[5] << 3)) >> 3;
  // do calculation
  int clippedH = hScroll - xCenter;
  int clippedV = vScroll - yCenter;
  clippedH = (clippedH & 0x2000) ? (clippedH | ~1023) : (clippedH & 1023);
  clippedV = (clippedV & 0x2000) ? (clippedV | ~1023) : (clippedV & 1023);
  if(ppu->bgLayer[0].mosaicEnabled && ppu->mosaicSize > 1) {
    y -= (y - ppu->mosaicStartLine) % ppu->mosaicSize;
  }
  uint8_t ry = ppu->m7yFlip ? 255 - y : y;
  ppu->m7startX = (
    ((ppu->m7matrix[0] * clippedH) & ~63) +
    ((ppu->m7matrix[1] * ry) & ~63) +
    ((ppu->m7matrix[1] * clippedV) & ~63) +
    (xCenter << 8)
  );
  ppu->m7startY = (
    ((ppu->m7matrix[2] * clippedH) & ~63) +
    ((ppu->m7matrix[3] * ry) & ~63) +
    ((ppu->m7matrix[3] * clippedV) & ~63) +
    (yCenter << 8)
  );
}

static int ppu_getPixelForMode7(Ppu* ppu, int x, int layer, bool priority) {
  uint8_t rx = ppu->m7xFlip ? 255 - x : x;
  int xPos = (ppu->m7startX + ppu->m7matrix[0] * rx) >> 8;
  int yPos = (ppu->m7startY + ppu->m7matrix[2] * rx) >> 8;
  bool outsideMap = xPos < 0 || xPos >= 1024 || yPos < 0 || yPos >= 1024;
  xPos &= 0x3ff;
  yPos &= 0x3ff;
  if(!ppu->m7largeField) outsideMap = false;
  uint8_t tile = outsideMap ? 0 : ppu->vram[(yPos >> 3) * 128 + (xPos >> 3)] & 0xff;
  uint8_t pixel = outsideMap && !ppu->m7charFill ? 0 : ppu->vram[tile * 64 + (yPos & 7) * 8 + (xPos & 7)] >> 8;
  if(layer == 1) {
    if(((bool) (pixel & 0x80)) != priority) return 0;
    return pixel & 0x7f;
  }
  return pixel;
}

static bool ppu_getWindowState(Ppu* ppu, int layer, int x) {
  if(!ppu->windowLayer[layer].window1enabled && !ppu->windowLayer[layer].window2enabled) {
    return false;
  }
  if(ppu->windowLayer[layer].window1enabled && !ppu->windowLayer[layer].window2enabled) {
    bool test = x >= ppu->window1left && x <= ppu->window1right;
    return ppu->windowLayer[layer].window1inversed ? !test : test;
  }
  if(!ppu->windowLayer[layer].window1enabled && ppu->windowLayer[layer].window2enabled) {
    bool test = x >= ppu->window2left && x <= ppu->window2right;
    return ppu->windowLayer[layer].window2inversed ? !test : test;
  }
  bool test1 = x >= ppu->window1left && x <= ppu->window1right;
  bool test2 = x >= ppu->window2left && x <= ppu->window2right;
  if(ppu->windowLayer[layer].window1inversed) test1 = !test1;
  if(ppu->windowLayer[layer].window2inversed) test2 = !test2;
  switch(ppu->windowLayer[layer].maskLogic) {
    case 0: return test1 || test2;
    case 1: return test1 && test2;
    case 2: return test1 != test2;
    case 3: return test1 == test2;
  }
  return false;
}

/* One pass over OAM builds, for every scanline, the set of sprites whose
 * y-range covers it — so the per-line evaluation only visits candidates
 * instead of rescanning all 128 entries 224 times a frame. Candidacy is a
 * function of OAM y bytes, the highOam size bits, OBSEL and SETINI alone;
 * writes to any of those clear objCacheValid. x, priority rotation and the
 * hardware's 32-sprite/34-tile limits are still applied per line, in the
 * exact order of the full scan, so the output is bit-identical. */
static void ppu_rebuildSpriteLineCache(Ppu *ppu) {
#if SNES_MATHFIXED_CENSUS
  g_objcache_rebuilds++;
#endif
  memset(ppu->objLineCand, 0, sizeof(ppu->objLineCand));
  for (int s = 0; s < 128; s++) {
    uint8_t index = (uint8_t)(s * 2);
    uint8_t y = ppu->oam[index] >> 8;
    int spriteSize = spriteSizes[ppu->objSize][(ppu->highOam[index >> 3] >> ((index & 7) + 1)) & 1];
    int spriteHeight = ppu->objInterlace ? spriteSize / 2 : spriteSize;
    for (int row = 0; row < spriteHeight; row++) {
      uint8_t l = (uint8_t)(y + row);   /* same wraparound as (uint8)(line - y) < height */
      if (l < 240)
        ppu->objLineCand[l][s >> 5] |= 1u << (s & 31);
    }
  }
  ppu->objCacheValid = 1;
}

#if SNES_SPRITE_CENSUS || SNES_RENDER_CENSUS
/* How much sprite work a scene actually has, read over SWD. The reverse-draw
 * lever removes one load per sprite PIXEL, so "is this scene sprite-heavy" is
 * not a matter of opinion -- it is slivers per frame, and nobody had counted. */
uint32_t g_sprite_slivers, g_sprite_lines, g_sprite_over32, g_sprite_over34;
#endif

static bool ppu_evaluateSprites(Ppu* ppu, int line) {
  // TODO: iterate over oam normally to determine in-range sprites,
  //   then iterate those in-range sprites in reverse for tile-fetching
  // TODO: rectangular sprites, wierdness with sprites at -256
  if (!ppu->objCacheValid)
    ppu_rebuildSpriteLineCache(ppu);
  const uint32_t *cand = ppu->objLineCand[line];
  int spritesFound = 0;
  int tilesFound = 0;
  /* the full scan started at this sprite and wrapped through all 128;
   * visit the candidates in that same order: s0..127, then 0..s0-1 */
  int s0 = ppu->objPriority ? ((ppu->oamAdr & 0xfe) >> 1) : 0;
  for (int half = 0; half != 2; half++) {
    int lo = half ? 0 : s0, hi = half ? s0 : 128;
    for (int w = lo >> 5; w * 32 < hi; w++) {
      uint32_t bits = cand[w];
      if (w == lo >> 5)
        bits &= ~0u << (lo & 31);
      if (hi - w * 32 < 32)
        bits &= (1u << (hi & 31)) - 1;
      while (bits) {
        int s = w * 32 + __builtin_ctz(bits);
        bits &= bits - 1;
#if SNES_MATHFIXED_CENSUS
        g_sprite_visits++;
#endif
        uint8_t index = (uint8_t)(s * 2);
        uint8_t y = ppu->oam[index] >> 8;
        // check if the sprite is on this line and get the sprite size
        uint8_t row = line - y;
        int spriteSize = spriteSizes[ppu->objSize][(ppu->highOam[index >> 3] >> ((index & 7) + 1)) & 1];
        {
          // in y-range, get the x location, using the high bit as well
          int x = ppu->oam[index] & 0xff;
          x |= ((ppu->highOam[index >> 3] >> (index & 7)) & 1) << 8;
          if(x > 255) x -= 512;
          // if in x-range
          if(x > -spriteSize) {
            // break if we found 32 sprites already
            spritesFound++;
            if(spritesFound > 32) {
              ppu->rangeOver = true;
#if SNES_SPRITE_CENSUS || SNES_RENDER_CENSUS
              g_sprite_over32++;
#endif
              goto done;
            }
            // update row according to obj-interlace
            if(ppu->objInterlace) row = row * 2 + (ppu->evenFrame ? 0 : 1);
            // get some data for the sprite and y-flip row if needed
            int oam1 = ppu->oam[index + 1];
            int objAdr = (oam1 & 0x100) ? ppu->objTileAdr2 : ppu->objTileAdr1;
            if(oam1 & 0x8000) row = spriteSize - 1 - row;
            // fetch all tiles in x-range
            int paletteBase = 0x80 + 16 * ((oam1 & 0xe00) >> 9);
            int prio = SPRITE_PRIO_TO_PRIO((oam1 & 0x3000) >> 12, (oam1 & 0x800) == 0);
            PpuZbufType z = paletteBase + (prio << 8);

            for(int col = 0; col < spriteSize; col += 8) {
#if SNES_MATHFIXED_CENSUS
              g_sprite_cols++;
#endif
              if(col + x > -8 && col + x < 256) {
                // break if we found 34 8*1 slivers already
                tilesFound++;
                if(tilesFound > 34) {
                  ppu->timeOver = true;
#if SNES_SPRITE_CENSUS || SNES_RENDER_CENSUS
                  g_sprite_over34++;
#endif
                  goto done;
                }
#if SNES_SPRITE_CENSUS || SNES_RENDER_CENSUS
                g_sprite_slivers++;
#endif
                // figure out which tile this uses, looping within 16x16 pages, and get it's data
                int usedCol = oam1 & 0x4000 ? spriteSize - 1 - col : col;
                int usedTile = ((((oam1 & 0xff) >> 4) + (row >> 3)) << 4) | (((oam1 & 0xf) + (usedCol >> 3)) & 0xf);
                uint16 *addr = &ppu->vram[(objAdr + usedTile * 16 + (row & 0x7)) & 0x7fff];
                PPU_PROBE_VRAM_ADR(objAdr + usedTile * 16 + (row & 0x7));
                /* On a frameskipped frame nobody ever reads objBuffer -- the
                 * compositing that consumes it is behind the g_ppu_skip_render
                 * return in ppu_runLine, whose comment says "the pixels below do
                 * not [matter]". These pixels are ABOVE that line and did not
                 * matter either: three frames in four were decoding and writing
                 * sprite pixels into a buffer that was then thrown away.
                 *
                 * The scan itself must still run in full -- the 32-sprite and
                 * 34-sliver limits set rangeOver/timeOver, which games read at
                 * $213E -- and so must the VRAM probe above, which feeds the line
                 * cache's dependency tracking. Only the decode and the pixel
                 * loops go. One test per sliver (5.46 per line, measured) removes
                 * eight pixel iterations each.
                 *
                 * DEFAULT OFF: implemented and rig-verified (hashes identical on
                 * ALttP 400f; +626 insn/frame there, which is the test being paid
                 * with nothing to skip because the rig draws every frame), but
                 * MEASURED, AND IT IS NOTHING: 52.29 / 52.36 / 52.21 fps against
                 * a 52.36 baseline, Zelda 3 rain, 900 deterministic frames. The
                 * arithmetic said ~1.4% of instructions; at this chip's observed
                 * transfer ratio that is ~0.25 fps, and it did not show. Sprite
                 * pixels are simply not enough of the frame here -- the census
                 * counts 5.46 slivers per line against a limit of 34. Left off. */
#if SNES_ABLATE_SPRITE_PIX
                /* ABLATION, WRONG OUTPUT. Keep the whole scan -- the candidate
                 * walk, the 32/34 limits, rangeOver/timeOver -- and delete only
                 * the tile decode and the eight pixel writes. Splits the sprite
                 * path's 3.15 fps into "finding them" and "drawing them". */
                continue;
#endif
#if SNES_SPRITE_SKIP_DRAW
                if (g_ppu_skip_render)
                  continue;
#endif
                uint32 plane = addr[0] | addr[8] << 16;
                /* a fully transparent sliver writes nothing: no decode, no loop */
                if (!plane)
                  continue;
                uint32 chunky = PpuDecode4bpp(plane);
                /* Screen pixel px is nibble px of `chunky` with bit 0x4000 set,
                 * nibble 7-px without; reverse the nibbles once so both cases
                 * read nibble px. Then visit only the opaque pixels inside the
                 * visible range [px_left, px_right): each is still written at
                 * most once and only onto a free objBuffer slot, exactly as the
                 * pixel-by-pixel loops did, so the result is the same. */
                if (!(oam1 & 0x4000)) {
                  chunky = __builtin_bswap32(chunky);
                  chunky = ((chunky >> 4) & 0x0f0f0f0fu) | ((chunky & 0x0f0f0f0fu) << 4);
                }
                int px_left = IntMax(-(col + x + kPpuExtraLeftRight), 0);
                int px_right = IntMin(256 + kPpuExtraLeftRight - (col + x), 8);
                uint32 opaque = (chunky | chunky >> 1 | chunky >> 2 | chunky >> 3) & 0x11111111u;
                opaque &= (px_right >= 8 ? ~0u : (1u << (px_right * 4)) - 1u) & (~0u << (px_left * 4));
                PpuZbufType *dst = ppu->objBuffer.data + col + x + kPpuExtraLeftRight;
                while (opaque) {
                  uint32 b = (uint32)__builtin_ctz(opaque);
                  opaque &= opaque - 1;
                  PpuZbufType *d = dst + (b >> 2);
                  if ((d[0] & 0xff) == 0)
                    d[0] = z + ((chunky >> b) & 0xf);
                }

              }
            }
          }
        }
      }
    }
  }
done:
#if SNES_SPRITE_CENSUS || SNES_RENDER_CENSUS
  g_sprite_lines++;
#endif
  return tilesFound != 0;
}

static uint16_t ppu_getVramRemap(Ppu* ppu) {
  uint16_t adr = ppu->vramPointer;
  switch(ppu->vramRemapMode) {
    case 0: return adr;
    case 1: return (adr & 0xff00) | ((adr & 0xe0) >> 5) | ((adr & 0x1f) << 3);
    case 2: return (adr & 0xfe00) | ((adr & 0x1c0) >> 6) | ((adr & 0x3f) << 3);
    case 3: return (adr & 0xfc00) | ((adr & 0x380) >> 7) | ((adr & 0x7f) << 3);
  }
  return adr;
}

uint8_t ppu_read(Ppu* ppu, uint8_t adr) {
  switch(adr) {
    case 0x04: case 0x14: case 0x24:
    case 0x05: case 0x15: case 0x25:
    case 0x06: case 0x16: case 0x26:
    case 0x08: case 0x18: case 0x28:
    case 0x09: case 0x19: case 0x29:
    case 0x0a: case 0x1a: case 0x2a: {
      return ppu->ppu1openBus;
    }
    case 0x34:
    case 0x35:
    case 0x36: {
      int result = ppu->m7matrix[0] * (ppu->m7matrix[1] >> 8);
      ppu->ppu1openBus = (result >> (8 * (adr - 0x34))) & 0xff;
      return ppu->ppu1openBus;
    }
    case 0x37: {
      // TODO: only when ppulatch is set
      ppu->hCount = ppu->snes->hPos / 4;
      ppu->vCount = ppu->snes->vPos;
      ppu->countersLatched = true;
      return ppu->snes->openBus;
    }
    case 0x38: {
      uint8_t ret = 0;
      if(ppu->oamInHigh) {
        ret = ppu->highOam[((ppu->oamAdr & 0xf) << 1) | ppu->oamSecondWrite];
        if(ppu->oamSecondWrite) {
          ppu->oamAdr++;
          if(ppu->oamAdr == 0) ppu->oamInHigh = false;
        }
      } else {
        if(!ppu->oamSecondWrite) {
          ret = ppu->oam[ppu->oamAdr] & 0xff;
        } else {
          ret = ppu->oam[ppu->oamAdr++] >> 8;
          if(ppu->oamAdr == 0) ppu->oamInHigh = true;
        }
      }
      ppu->oamSecondWrite = !ppu->oamSecondWrite;
      ppu->ppu1openBus = ret;
      return ret;
    }
    case 0x39: {
      uint16_t val = ppu->vramReadBuffer;
      if(!ppu->vramIncrementOnHigh) {
        ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
        ppu->vramPointer += ppu->vramIncrement;
      }
      ppu->ppu1openBus = val & 0xff;
      return val & 0xff;
    }
    case 0x3a: {
      uint16_t val = ppu->vramReadBuffer;
      if(ppu->vramIncrementOnHigh) {
        ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
        ppu->vramPointer += ppu->vramIncrement;
      }
      ppu->ppu1openBus = val >> 8;
      return val >> 8;
    }
    case 0x3b: {
      uint8_t ret = 0;
      if(!ppu->cgramSecondWrite) {
        ret = ppu->cgram[ppu->cgramPointer] & 0xff;
      } else {
        ret = ((ppu->cgram[ppu->cgramPointer++] >> 8) & 0x7f) | (ppu->ppu2openBus & 0x80);
      }
      ppu->cgramSecondWrite = !ppu->cgramSecondWrite;
      ppu->ppu2openBus = ret;
      return ret;
    }
    case 0x3c: {
      uint8_t val = 0;
      if(ppu->hCountSecond) {
        val = ((ppu->hCount >> 8) & 1) | (ppu->ppu2openBus & 0xfe);
      } else {
        val = ppu->hCount & 0xff;
      }
      ppu->hCountSecond = !ppu->hCountSecond;
      ppu->ppu2openBus = val;
      return val;
    }
    case 0x3d: {
      uint8_t val = 0;
      if(ppu->vCountSecond) {
        val = ((ppu->vCount >> 8) & 1) | (ppu->ppu2openBus & 0xfe);
      } else {
        val = ppu->vCount & 0xff;
      }
      ppu->vCountSecond = !ppu->vCountSecond;
      ppu->ppu2openBus = val;
      return val;
    }
    case 0x3e: {
      uint8_t val = 0x1; // ppu1 version (4 bit)
      val |= ppu->ppu1openBus & 0x10;
#if SNES_SKIP_SPRITE_EVAL_ON_SKIP
      /* The only reader of rangeOver/timeOver in the whole emulator. Until a
       * game has asked for them, sprite evaluation on a frameskipped line has no
       * observable effect at all and can be skipped outright; from the first read
       * onward this latches and the evaluation is exact again, for ever. */
      g_stat77_read = true;
#endif
      val |= ppu->rangeOver << 6;
      val |= ppu->timeOver << 7;
      ppu->ppu1openBus = val;
      return val;
    }
    case 0x3f: {
      uint8_t val = 0x3; // ppu2 version (low 4 bits)
      if (ppu->snes && ppu->snes->pal)
        val |= 0x10; /* bit 4: PAL */
      val |= ppu->ppu2openBus & 0x20;
      val |= ppu->countersLatched << 6;
      val |= ppu->evenFrame << 7;
      ppu->countersLatched = false; // TODO: only when ppulatch is set
      ppu->hCountSecond = false;
      ppu->vCountSecond = false;
      ppu->ppu2openBus = val;
      return val;
    }
    default: {
      return ppu->snes->openBus;
    }
  }
}

void ppu_write(Ppu* ppu, uint8_t adr, uint8_t val) {
//  if (adr != 24 && adr != 25)
//    printf("ppu_write(%d, %d)\n", adr, val);
  switch(adr) {
    case 0x00: {
      // TODO: oam address reset when written on first line of vblank, (and when forced blank is disabled?)
      ppu->brightness = val & 0xf;
      ppu->forcedBlank = val & 0x80;
      break;
    }
    case 0x01: {
      ppu->objSize = val >> 5;
      ppu->objTileAdr1 = (val & 7) << 13;
      ppu->objTileAdr2 = ppu->objTileAdr1 + (((val & 0x18) + 8) << 9);
      ppu->objCacheValid = 0;   /* sprite sizes moved */
      break;
    }
    case 0x02: {
      ppu->oamAdr = val;
      ppu->oamAdrWritten = ppu->oamAdr;
      ppu->oamInHigh = ppu->oamInHighWritten;
      ppu->oamSecondWrite = false;
      break;
    }
    case 0x03: {
      ppu->objPriority = val & 0x80;
      ppu->oamInHigh = val & 1;
      ppu->oamInHighWritten = ppu->oamInHigh;
      ppu->oamAdr = ppu->oamAdrWritten;
      ppu->oamSecondWrite = false;
      break;
    }
    case 0x04: {
      if(ppu->oamInHigh) {
        uint32_t high_index = ((ppu->oamAdr & 0xf) << 1) | ppu->oamSecondWrite;
        uint8_t *dst = &ppu->highOam[high_index];
#ifdef SNES_LINE_REUSE_PROBE
        if (*dst != val) {
          g_probe_oam_gen++;
          for (int s = high_index * 4; s < high_index * 4 + 4; s++)
            g_probe_oam_entry_gen[s]++;
        }
#endif
#ifdef SNES_LINE_CACHE
        if (*dst != val) {
          int first = high_index * 4;
          PpuLineCacheBump(&g_line_cache_oam_serial, &g_line_cache_oam_last[first]);
          for (int s = first + 1; s < first + 4; s++)
            g_line_cache_oam_last[s] = g_line_cache_oam_serial;
        }
#endif
        *dst = val;
        ppu->objCacheValid = 0;   /* size / x-high bits moved */
        if(ppu->oamSecondWrite) {
          ppu->oamAdr++;
          if(ppu->oamAdr == 0) ppu->oamInHigh = false;
        }
      } else {
        if(!ppu->oamSecondWrite) {
          ppu->oamBuffer = val;
        } else {
          uint16_t value = (val << 8) | ppu->oamBuffer;
#ifdef SNES_LINE_REUSE_PROBE
          if (ppu->oam[ppu->oamAdr] != value) {
            g_probe_oam_gen++;
            g_probe_oam_entry_gen[ppu->oamAdr >> 1]++;
          }
#endif
#ifdef SNES_LINE_CACHE
          if (ppu->oam[ppu->oamAdr] != value)
            PpuLineCacheBump(&g_line_cache_oam_serial,
                             &g_line_cache_oam_last[ppu->oamAdr >> 1]);
#endif
          ppu->oam[ppu->oamAdr++] = value;
          ppu->objCacheValid = 0;   /* a sprite may have moved vertically */
          if(ppu->oamAdr == 0) ppu->oamInHigh = true;
        }
      }
      ppu->oamSecondWrite = !ppu->oamSecondWrite;
      break;
    }
    case 0x05: {
      ppu->mode = val & 0x7;
      ppu->bg3priority = val & 0x8;
      ppu->bgLayer[0].bigTiles = val & 0x10;
      ppu->bgLayer[1].bigTiles = val & 0x20;
      ppu->bgLayer[2].bigTiles = val & 0x40;
      ppu->bgLayer[3].bigTiles = val & 0x80;
      break;
    }
    case 0x06: {
      // TODO: mosaic line reset specifics
      ppu->bgLayer[0].mosaicEnabled = val & 0x1;
      ppu->bgLayer[1].mosaicEnabled = val & 0x2;
      ppu->bgLayer[2].mosaicEnabled = val & 0x4;
      ppu->bgLayer[3].mosaicEnabled = val & 0x8;
      ppu->mosaicEnabled = val & 0x0f;
      ppu->mosaicSize = (val >> 4) + 1;
      ppu->mosaicStartLine = 0;// ppu->snes->vPos;
      break;
    }
    case 0x07:
    case 0x08:
    case 0x09:
    case 0x0a: {
      ppu->bgLayer[adr - 7].tilemapWider = val & 0x1;
      ppu->bgLayer[adr - 7].tilemapHigher = val & 0x2;
      ppu->bgLayer[adr - 7].tilemapAdr = (val & 0xfc) << 8;
      break;
    }
    case 0x0b: {
      ppu->bgLayer[0].tileAdr = (val & 0xf) << 12;
      ppu->bgLayer[1].tileAdr = (val & 0xf0) << 8;
      break;
    }
    case 0x0c: {
      ppu->bgLayer[2].tileAdr = (val & 0xf) << 12;
      ppu->bgLayer[3].tileAdr = (val & 0xf0) << 8;
      break;
    }
    case 0x0d: {
      ppu->m7matrix[6] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      // fallthrough to normal layer BG-HOFS
    }
    case 0x0f:
    case 0x11:
    case 0x13: {
      ppu->bgLayer[(adr - 0xd) / 2].hScroll = ((val << 8) | (ppu->scrollPrev & 0xf8) | (ppu->scrollPrev2 & 0x7)) & 0x3ff;
      ppu->scrollPrev = val;
      ppu->scrollPrev2 = val;
      break;
    }
    case 0x0e: {
      ppu->m7matrix[7] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      // fallthrough to normal layer BG-VOFS
    }
    case 0x10:
    case 0x12:
    case 0x14: {
      ppu->bgLayer[(adr - 0xe) / 2].vScroll = ((val << 8) | ppu->scrollPrev) & 0x3ff;
      ppu->scrollPrev = val;
      break;
    }
    case 0x15: {
      if((val & 3) == 0) {
        ppu->vramIncrement = 1;
      } else if((val & 3) == 1) {
        ppu->vramIncrement = 32;
      } else {
        ppu->vramIncrement = 128;
      }
      ppu->vramRemapMode = (val & 0xc) >> 2;
      ppu->vramIncrementOnHigh = val & 0x80;
      break;
    }
    case 0x16: {
      ppu->vramPointer = (ppu->vramPointer & 0xff00) | val;
      ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
      break;
    }
    case 0x17: {
      ppu->vramPointer = (ppu->vramPointer & 0x00ff) | (val << 8);
      ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
      break;
    }
    case 0x18: {
      // TODO: vram access during rendering (also cgram and oam)
      uint16_t vramAdr = ppu_getVramRemap(ppu);
      uint16_t *dst = &ppu->vram[vramAdr & 0x7fff];
      uint16_t value = (*dst & 0xff00) | val;
#ifdef SNES_LINE_REUSE_PROBE
      if (*dst != value) {
        g_probe_vram_gen++;
        g_probe_vram_page_gen[(vramAdr & 0x7fff) >> 6]++;
      }
#endif
#ifdef SNES_LINE_CACHE
      if (*dst != value)
        PpuLineCacheBump(&g_line_cache_vram_serial,
                         &g_line_cache_vram_last[(vramAdr & 0x7fff) >> kLineCacheVramShift]);
#endif
      *dst = value;
      if(!ppu->vramIncrementOnHigh) ppu->vramPointer += ppu->vramIncrement;
      break;
    }
    case 0x19: {
      uint16_t vramAdr = ppu_getVramRemap(ppu);
      uint16_t *dst = &ppu->vram[vramAdr & 0x7fff];
      uint16_t value = (*dst & 0x00ff) | (val << 8);
#ifdef SNES_LINE_REUSE_PROBE
      if (*dst != value) {
        g_probe_vram_gen++;
        g_probe_vram_page_gen[(vramAdr & 0x7fff) >> 6]++;
      }
#endif
#ifdef SNES_LINE_CACHE
      if (*dst != value)
        PpuLineCacheBump(&g_line_cache_vram_serial,
                         &g_line_cache_vram_last[(vramAdr & 0x7fff) >> kLineCacheVramShift]);
#endif
      *dst = value;
      if(ppu->vramIncrementOnHigh) ppu->vramPointer += ppu->vramIncrement;
      break;
    }
    case 0x1a: {
      ppu->m7largeField = val & 0x80;
      ppu->m7charFill = val & 0x40;
      ppu->m7yFlip = val & 0x2;
      ppu->m7xFlip = val & 0x1;
      break;
    }
    case 0x1b:
    case 0x1c:
    case 0x1d:
    case 0x1e: {
      ppu->m7matrix[adr - 0x1b] = (val << 8) | ppu->m7prev;
      ppu->m7prev = val;
      break;
    }
    case 0x1f:
    case 0x20: {
      ppu->m7matrix[adr - 0x1b] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      break;
    }
    case 0x21: {
      ppu->cgramPointer = val;
      ppu->cgramSecondWrite = false;
      break;
    }
    case 0x22: {
      if(!ppu->cgramSecondWrite) {
        ppu->cgramBuffer = val;
      } else {
        uint16_t value = (val << 8) | ppu->cgramBuffer;
#ifdef SNES_LINE_REUSE_PROBE
        if (ppu->cgram[ppu->cgramPointer] != value) {
          g_probe_cgram_gen++;
          g_probe_cgram_entry_gen[ppu->cgramPointer]++;
        }
#endif
#ifdef SNES_LINE_CACHE
        if (ppu->cgram[ppu->cgramPointer] != value)
          PpuLineCacheBump(&g_line_cache_cgram_serial,
                           &g_line_cache_cgram_last[ppu->cgramPointer]);
#endif
#ifdef PPU_RGB565
        if (ppu->cgram[ppu->cgramPointer] != value) {
          g_cgram_dirty[ppu->cgramPointer >> 5] |= 1u << (ppu->cgramPointer & 31);
          g_cgram_dirty_any = true;
        }
#ifdef SNES_PPU_FULL_REBUILD_REF
        ppu->paletteDirty = true;   /* A/B reference: the old full rebuild on every write */
#endif
#endif
        ppu->cgram[ppu->cgramPointer++] = value;
      }
      ppu->cgramSecondWrite = !ppu->cgramSecondWrite;
      break;
    }
    case 0x23:
    case 0x24:
    case 0x25: {

      if (adr == 0x23)
        ppu->windowsel = (ppu->windowsel & ~0xff) | val;
      else if (adr == 0x24)
        ppu->windowsel = (ppu->windowsel & ~0xff00) | (val << 8);
      else if (adr == 0x25)
        ppu->windowsel = (ppu->windowsel & ~0xff0000) | (val << 16);

      ppu->windowLayer[(adr - 0x23) * 2].window1inversed = val & 0x1;
      ppu->windowLayer[(adr - 0x23) * 2].window1enabled = val & 0x2;
      ppu->windowLayer[(adr - 0x23) * 2].window2inversed = val & 0x4;
      ppu->windowLayer[(adr - 0x23) * 2].window2enabled = val & 0x8;
      ppu->windowLayer[(adr - 0x23) * 2 + 1].window1inversed = val & 0x10;
      ppu->windowLayer[(adr - 0x23) * 2 + 1].window1enabled = val & 0x20;
      ppu->windowLayer[(adr - 0x23) * 2 + 1].window2inversed = val & 0x40;
      ppu->windowLayer[(adr - 0x23) * 2 + 1].window2enabled = val & 0x80;
      break;
    }
    case 0x26: {
      ppu->window1left = val;
      break;
    }
    case 0x27: {
      ppu->window1right = val;
      break;
    }
    case 0x28: {
      ppu->window2left = val;
      break;
    }
    case 0x29: {
      ppu->window2right = val;
      break;
    }
    case 0x2a: {
      ppu->windowLayer[0].maskLogic = val & 0x3;
      ppu->windowLayer[1].maskLogic = (val >> 2) & 0x3;
      ppu->windowLayer[2].maskLogic = (val >> 4) & 0x3;
      ppu->windowLayer[3].maskLogic = (val >> 6) & 0x3;
      break;
    }
    case 0x2b: {
      ppu->windowLayer[4].maskLogic = val & 0x3;
      ppu->windowLayer[5].maskLogic = (val >> 2) & 0x3;
      break;
    }
    case 0x2c: {
      ppu->screenEnabled[0] = val;
      ppu->layer[0].mainScreenEnabled = val & 0x1;
      ppu->layer[1].mainScreenEnabled = val & 0x2;
      ppu->layer[2].mainScreenEnabled = val & 0x4;
      ppu->layer[3].mainScreenEnabled = val & 0x8;
      ppu->layer[4].mainScreenEnabled = val & 0x10;
      break;
    }
    case 0x2d: {
      ppu->screenEnabled[1] = val;
      ppu->layer[0].subScreenEnabled = val & 0x1;
      ppu->layer[1].subScreenEnabled = val & 0x2;
      ppu->layer[2].subScreenEnabled = val & 0x4;
      ppu->layer[3].subScreenEnabled = val & 0x8;
      ppu->layer[4].subScreenEnabled = val & 0x10;
      break;
    }
    case 0x2e: {
      ppu->screenWindowed[0] = val;
      ppu->layer[0].mainScreenWindowed = val & 0x1;
      ppu->layer[1].mainScreenWindowed = val & 0x2;
      ppu->layer[2].mainScreenWindowed = val & 0x4;
      ppu->layer[3].mainScreenWindowed = val & 0x8;
      ppu->layer[4].mainScreenWindowed = val & 0x10;
      break;
    }
    case 0x2f: {
      ppu->screenWindowed[1] = val;
      ppu->layer[0].subScreenWindowed = val & 0x1;
      ppu->layer[1].subScreenWindowed = val & 0x2;
      ppu->layer[2].subScreenWindowed = val & 0x4;
      ppu->layer[3].subScreenWindowed = val & 0x8;
      ppu->layer[4].subScreenWindowed = val & 0x10;
      break;
    }
    case 0x30: {
      ppu->directColor = val & 0x1;
      ppu->addSubscreen = val & 0x2;
      ppu->preventMathMode = (val & 0x30) >> 4;
      ppu->clipMode = (val & 0xc0) >> 6;
      break;
    }
    case 0x31: {
      ppu->subtractColor = val & 0x80;
      ppu->halfColor = val & 0x40;
      for(int i = 0; i < 6; i++) {
        ppu->mathEnabled[i] = val & (1 << i);
      }
      break;
    }
    case 0x32: {
      if(val & 0x80) ppu->fixedColorB = val & 0x1f;
      if(val & 0x40) ppu->fixedColorG = val & 0x1f;
      if(val & 0x20) ppu->fixedColorR = val & 0x1f;
      break;
    }
    case 0x33: {
      ppu->interlace = val & 0x1;
      if (ppu->objInterlace != (bool)(val & 0x2))
        ppu->objCacheValid = 0;   /* sprite heights halve/double */
      ppu->objInterlace = val & 0x2;
      ppu->overscan = val & 0x4;
      ppu->pseudoHires = val & 0x8;
      ppu->m7extBg = val & 0x40;
      break;
    }
    default: {
      break;
    }
  }
}

int PpuGetCurrentRenderScale(Ppu *ppu, uint32_t render_flags) {
  return 1;
}
