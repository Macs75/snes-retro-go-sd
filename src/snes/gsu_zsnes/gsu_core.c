/* ============================================================================
 * gsu_core.c — the ZSNES 2 Super FX (GSU) core, ported verbatim into one TU.
 *
 * Source: ZSNES 2 (xyproto C rewrite), GPLv2. Opcode logic (fx_ops.h), the
 * dispatch/plot/memory tables (c_fxtable.inc) and the MMIO register handlers
 * (sfxproc.inc) are byte-for-byte ZSNES. Only the DATA definitions that ZSNES
 * laid out via an inline-asm block (c_fxdata.c + the GSU slice of endmem.c) are
 * reimplemented here in plain C — there is no save-state layout to preserve
 * (FX carts don't save state in this PoC), and the one load-bearing fact (the
 * four adjacent ALT dispatch sub-tables) is kept as [1024] arrays, see
 * gsu_endmem.h.
 *
 * Memory split (PoC): game RAM = cart->ram via `sfxgameram`; the plot
 * line-location LUTs are computed (fx_tile in fx_ops.h). Set by gsu.c.
 * ========================================================================== */

#include <string.h>
#include "gsu_types.h"
#include "unaligned.h"
#include "gsu_endmem.h"
#include "gsu_ui.h"
#include "memseam.h"
#include "gsu_zsnes_core.h"
#include "../saveload.h"

/* ---- GSU register/state block (was chips/c_fxdata.c inline asm) ---------- */
u4 SfxR0[16];
u4 SfxSFR, SfxBRAMR, SfxPBR, SfxROMBR, SfxCFGR, SfxSCBR, SfxCLSR, SfxSCMR, SfxVCR;
u4 SfxRAMBR, SfxCBR, SfxCOLR, SfxPOR, SfxCacheFlags, SfxLastRamAdrSt, SfxDREG, SfxSREG;
u4 SfxRomBufferSt, SfxPIPE, SfxPipeAdr, SfxnRamBanks = 4, SfxnRomBanks;
u4 SfxvScreenHeight, SfxvScreenSize, SfxCacheActive, SfxCarry, SfxSignZero, SfxB;
u4 SfxOverflow, SfxAC;
u1 SfxCACHERAM[512];

/* pointer-typed state (ZSNES PTRSYM slots) */
zreg SfxCPB, SfxCROM, SfxRAMMem, SfxLastRamAdr, SfxRomBuffer, sfxclineloc;
u1  *SCBRrel;                    /* c_fxops.c declares this one as u1* */
u4 withr15sk, fxbit01pcal, fxbit23pcal, fxbit45pcal, fxbit67pcal;

u4 NumberOfOpcodes, NumberOfOpcodesBU, NumberOfOpcodes2, ChangeOps, SFXProc, flagnz;
u1 sfxwarningb, fxtrace;

/* ---- GSU scratch tables (was endmem.c's GSU slice) ----------------------- */
zreg FxTable[1024], FxTableb[1024], FxTablec[1024], FxTabled[1024];
zreg SfxMemTable[256];
u4   fxxand[256], fxbit01[256], fxbit23[256], fxbit45[256], fxbit67[256];
zreg PLOTJmpa[64], PLOTJmpb[64];
zreg sfx128lineloc, sfx160lineloc, sfx192lineloc, sfxobjlineloc;

/* ---- opcode seam (was the head of chips/c_fxops.c) ----------------------- */
u4  FxSeamPC;          /* program counter, bank-relative (bytes via FX_PC_BYTE) */
u1 const *FxCodeBase;  /* program bank's base in the ROM / Game Pak RAM */
u4  FxCodeMask;        /* 0x7FFF for banks $00-$3F (both halves mirror), else 0xFFFF */
u4  SfxCROMMask;       /* the same for the ROM bank R14 reads through */
u4  SfxMemMask[256];   /* per-bank address mask, beside SfxMemTable */
u4 *FxSeamSrc;
u4 *FxSeamDst;
u4  FxSeamCX;
u4  FxLoopDone;

/* ---- memory seam globals (was defined elsewhere in ZSNES) ---------------- */
uintptr_t MemSeamB, MemSeamC, MemSeamA, MemSeamD, MemSeamS, MemSeamDI;

/* ---- ui.h shim + glue-visible knobs -------------------------------------- */
u1 *romdata;
u4  romdatasize;
u1 *sfxgameram;
u4  SfxRamMask = 0x3FFFF;  /* Game Pak RAM size - 1: GSU RAM addresses wrap to it */
u1  romispal;
u4  SuperFXClockMultiplier = 100;
u1  doirqnext;

/* ---- the verbatim ZSNES units -------------------------------------------- */
#include "fx_ops.h"      /* opcode bodies + MainLoop */
#include "sched.inc"     /* StartSFX / SfxVblankCatchup / UpdatePORSCMR / ... */
#include "sfxproc.inc"   /* $3000-$303F register handlers + RAM access */
#include "c_fxtable.inc" /* InitFxTables + the dispatch/plot/mem tables */

/* ---- register install (was chips/c_sfxproc.c), into local tables --------- */
static eop *sfx_rptr[0x300];
static eop *sfx_wptr[0x300];
#define REGPTR(a) sfx_rptr[(a) - 0x3000]
#define REGPTW(a) sfx_wptr[(a) - 0x3000]

static void gsu_install_regs(void)
{
    REGPTR(0x3000) = reg3000r; REGPTW(0x3000) = reg3000w;
    REGPTR(0x3001) = reg3001r; REGPTW(0x3001) = reg3001w;
    REGPTR(0x3002) = reg3002r; REGPTW(0x3002) = reg3002w;
    REGPTR(0x3003) = reg3003r; REGPTW(0x3003) = reg3003w;
    REGPTR(0x3004) = reg3004r; REGPTW(0x3004) = reg3004w;
    REGPTR(0x3005) = reg3005r; REGPTW(0x3005) = reg3005w;
    REGPTR(0x3006) = reg3006r; REGPTW(0x3006) = reg3006w;
    REGPTR(0x3007) = reg3007r; REGPTW(0x3007) = reg3007w;
    REGPTR(0x3008) = reg3008r; REGPTW(0x3008) = reg3008w;
    REGPTR(0x3009) = reg3009r; REGPTW(0x3009) = reg3009w;
    REGPTR(0x300A) = reg300Ar; REGPTW(0x300A) = reg300Aw;
    REGPTR(0x300B) = reg300Br; REGPTW(0x300B) = reg300Bw;
    REGPTR(0x300C) = reg300Cr; REGPTW(0x300C) = reg300Cw;
    REGPTR(0x300D) = reg300Dr; REGPTW(0x300D) = reg300Dw;
    REGPTR(0x300E) = reg300Er; REGPTW(0x300E) = reg300Ew;
    REGPTR(0x300F) = reg300Fr; REGPTW(0x300F) = reg300Fw;
    REGPTR(0x3010) = reg3010r; REGPTW(0x3010) = reg3010w;
    REGPTR(0x3011) = reg3011r; REGPTW(0x3011) = reg3011w;
    REGPTR(0x3012) = reg3012r; REGPTW(0x3012) = reg3012w;
    REGPTR(0x3013) = reg3013r; REGPTW(0x3013) = reg3013w;
    REGPTR(0x3014) = reg3014r; REGPTW(0x3014) = reg3014w;
    REGPTR(0x3015) = reg3015r; REGPTW(0x3015) = reg3015w;
    REGPTR(0x3016) = reg3016r; REGPTW(0x3016) = reg3016w;
    REGPTR(0x3017) = reg3017r; REGPTW(0x3017) = reg3017w;
    REGPTR(0x3018) = reg3018r; REGPTW(0x3018) = reg3018w;
    REGPTR(0x3019) = reg3019r; REGPTW(0x3019) = reg3019w;
    REGPTR(0x301A) = reg301Ar; REGPTW(0x301A) = reg301Aw;
    REGPTR(0x301B) = reg301Br; REGPTW(0x301B) = reg301Bw;
    REGPTR(0x301C) = reg301Cr; REGPTW(0x301C) = reg301Cw;
    REGPTR(0x301D) = reg301Dr; REGPTW(0x301D) = reg301Dw;
    REGPTR(0x301E) = reg301Er; REGPTW(0x301E) = reg301Ew;
    REGPTR(0x301F) = reg301Fr; REGPTW(0x301F) = reg301Fw;
    REGPTR(0x3030) = reg3030r; REGPTW(0x3030) = reg3030w;
    REGPTR(0x3031) = reg3031r; REGPTW(0x3031) = reg3031w;
    REGPTR(0x3032) = reg3032r; REGPTW(0x3032) = reg3032w;
    REGPTR(0x3033) = reg3033r; REGPTW(0x3033) = reg3033w;
    REGPTR(0x3034) = reg3034r; REGPTW(0x3034) = reg3034w;
    REGPTR(0x3035) = reg3035r; REGPTW(0x3035) = reg3035w;
    REGPTR(0x3036) = reg3036r; REGPTW(0x3036) = reg3036w;
    REGPTR(0x3037) = reg3037r; REGPTW(0x3037) = reg3037w;
    REGPTR(0x3038) = reg3038r; REGPTW(0x3038) = reg3038w;
    REGPTR(0x3039) = reg3039r; REGPTW(0x3039) = reg3039w;
    REGPTR(0x303A) = reg303Ar; REGPTW(0x303A) = reg303Aw;
    REGPTR(0x303B) = reg303Br; REGPTW(0x303B) = reg303Bw;
    REGPTR(0x303C) = reg303Cr; REGPTW(0x303C) = reg303Cw;
    REGPTR(0x303D) = reg303Dr; REGPTW(0x303D) = reg303Dw;
    REGPTR(0x303E) = reg303Er; REGPTW(0x303E) = reg303Ew;
    REGPTR(0x303F) = reg303Fr; REGPTW(0x303F) = reg303Fw;
    for (int a = 0x3100; a < 0x3300; a++) {
        REGPTR(a) = cacheregr;
        REGPTW(a) = cacheregw;
    }
}

/* ============================================================================
 * Entry points for the glue (gsu.c)
 * ========================================================================== */

void gsu_zsnes_build_tables(void)
{
    InitFxTables();
    gsu_install_regs();
    gsu_blk_select();
}

uint8_t gsu_zsnes_reg_read(uint16_t addr)
{
    if (addr < 0x3000 || addr >= 0x3300) return 0;
    eop *fn = REGPTR(addr);
    if (!fn) return 0;
    fn();                            /* trampoline: mem_set_al(c_regXX()) */
    return (uint8_t)MemSeamA;
}

void gsu_zsnes_reg_write(uint16_t addr, uint8_t val)
{
    if (addr < 0x3000 || addr >= 0x3300) return;
    eop *fn = REGPTW(addr);
    if (!fn) return;
#ifdef HOST_BUILD
    { extern unsigned long gsu_hs[5]; gsu_hs[4]++; }
#endif
    mem_set_al(val);                 /* the value the handler consumes */
    MemSeamD = 0;                    /* for the one _DX handler (reg3030w) */
    fn();
}

/* Only while GO is set: ZSNES gates StartSFX in its caller (c_execloop.c). */
void gsu_zsnes_startline(void) { if (SfxSFR & 0x20) StartSFX(); }
void gsu_zsnes_vblank(void)    { SfxVblankCatchup(); }

/* Clear the register/state block on reset (keeps tables + pointers). */
void gsu_zsnes_reset_state(void)
{
    memset(SfxR0, 0, sizeof SfxR0);
    SfxSFR = SfxBRAMR = SfxPBR = SfxROMBR = SfxCFGR = SfxSCBR = SfxCLSR = 0;
    SfxSCMR = SfxVCR = SfxRAMBR = SfxCBR = SfxCOLR = SfxPOR = 0;
    SfxCacheFlags = SfxLastRamAdrSt = SfxDREG = SfxSREG = 0;
    SfxRomBufferSt = SfxPIPE = SfxPipeAdr = 0;
    SfxCacheActive = SfxCarry = SfxSignZero = SfxB = SfxOverflow = SfxAC = 0;
    memset(SfxCACHERAM, 0, sizeof SfxCACHERAM);
    SFXProc = ChangeOps = NumberOfOpcodes = 0;
    doirqnext = 0;
    fx_set_codebank(0);              /* PBR 0 */
    fx_set_rombank(0);
    SfxRAMMem = (uintptr_t)sfxgameram;
    UpdateCLSR();                    /* seed NumberOfOpcodes2 from CLSR */
    UpdatePORSCMR();
    UpdateSCBRCOLR();
}

/* ---- save state ----------------------------------------------------------
 * The GSU's state is these globals (the ZSNES core's), not a struct. One
 * snapshot covers what ZSNES's zstate.c saves (R0-R15 .. cache RAM, the ROM
 * buffer and last RAM address as offsets) plus what it rebuilds only
 * approximately after a load: the colour-plane masks, the opcode counters and
 * the IRQ flag, and the scheduler's per-line bookkeeping (sched.inc). Between
 * MainLoop calls, the program counter, opcode byte, ALT mode and the TO/FROM
 * registers live in R15, PIPE, SFR and SREG/DREG, all in here. The same
 * gather / stream / scatter serves counting, saving and loading. */
typedef struct {
    u4 r[16];
    u4 sfr, bramr, pbr, rombr, cfgr, scbr, clsr, scmr, vcr, rambr, cbr, colr, por;
    u4 cacheflags, dreg, sreg, pipe, pipeadr, nrambanks, nrombanks;
    u4 vscreenheight, vscreensize, cacheactive, carry, signzero, b, overflow, ac;
    u4 rombuf_off, lastram_off;   /* 0xFFFFFFFF: the pointer was never set */
    u4 withr15sk, pcal[4], opcodes, opcodes2, changeops, sfxproc, flagnz;
    u4 doirqnext, ran_this_line, owed_ops;
    u1 cacheram[512];
} gsu_zsnes_state;

void gsu_zsnes_saveload(SaveLoadFunc *func, void *ctx)
{
    gsu_zsnes_state st;
    memcpy(st.r, SfxR0, sizeof st.r);
    st.sfr = SfxSFR; st.bramr = SfxBRAMR; st.pbr = SfxPBR; st.rombr = SfxROMBR;
    st.cfgr = SfxCFGR; st.scbr = SfxSCBR; st.clsr = SfxCLSR; st.scmr = SfxSCMR;
    st.vcr = SfxVCR; st.rambr = SfxRAMBR; st.cbr = SfxCBR; st.colr = SfxCOLR; st.por = SfxPOR;
    st.cacheflags = SfxCacheFlags; st.dreg = SfxDREG; st.sreg = SfxSREG; st.pipe = SfxPIPE;
    st.pipeadr = SfxPipeAdr; st.nrambanks = SfxnRamBanks; st.nrombanks = SfxnRomBanks;
    st.vscreenheight = SfxvScreenHeight; st.vscreensize = SfxvScreenSize;
    st.cacheactive = SfxCacheActive; st.carry = SfxCarry; st.signzero = SfxSignZero;
    st.b = SfxB; st.overflow = SfxOverflow; st.ac = SfxAC;
    st.rombuf_off = SfxRomBuffer ? (u4)(SfxRomBuffer - SfxCROM) : 0xFFFFFFFFu;
    st.lastram_off = SfxLastRamAdr ? (u4)(SfxLastRamAdr - SfxRAMMem) : 0xFFFFFFFFu;
    st.withr15sk = withr15sk;
    st.pcal[0] = fxbit01pcal; st.pcal[1] = fxbit23pcal; st.pcal[2] = fxbit45pcal; st.pcal[3] = fxbit67pcal;
    st.opcodes = NumberOfOpcodes; st.opcodes2 = NumberOfOpcodes2; st.changeops = ChangeOps;
    st.sfxproc = SFXProc; st.flagnz = flagnz; st.doirqnext = doirqnext;
    st.ran_this_line = SfxRanThisLine; st.owed_ops = SfxOwedOps;
    memcpy(st.cacheram, SfxCACHERAM, sizeof st.cacheram);

    func(ctx, &st, sizeof st);

    memcpy(SfxR0, st.r, sizeof st.r);
    SfxSFR = st.sfr; SfxBRAMR = st.bramr; SfxPBR = st.pbr; SfxROMBR = st.rombr;
    SfxCFGR = st.cfgr; SfxSCBR = st.scbr; SfxCLSR = st.clsr; SfxSCMR = st.scmr;
    SfxVCR = st.vcr; SfxRAMBR = st.rambr; SfxCBR = st.cbr; SfxCOLR = st.colr; SfxPOR = st.por;
    SfxCacheFlags = st.cacheflags; SfxDREG = st.dreg; SfxSREG = st.sreg; SfxPIPE = st.pipe;
    SfxPipeAdr = st.pipeadr; SfxnRamBanks = st.nrambanks; SfxnRomBanks = st.nrombanks;
    SfxvScreenHeight = st.vscreenheight; SfxvScreenSize = st.vscreensize;
    SfxCacheActive = st.cacheactive; SfxCarry = st.carry; SfxSignZero = st.signzero;
    SfxB = st.b; SfxOverflow = st.overflow; SfxAC = st.ac;
    withr15sk = st.withr15sk;
    fxbit01pcal = st.pcal[0]; fxbit23pcal = st.pcal[1]; fxbit45pcal = st.pcal[2]; fxbit67pcal = st.pcal[3];
    NumberOfOpcodes = st.opcodes; NumberOfOpcodes2 = st.opcodes2; ChangeOps = st.changeops;
    SFXProc = st.sfxproc; flagnz = st.flagnz; doirqnext = (u1)st.doirqnext;
    SfxRanThisLine = (u1)st.ran_this_line; SfxOwedOps = st.owed_ops;
    memcpy(SfxCACHERAM, st.cacheram, sizeof st.cacheram);

    /* the host pointers, from the bank registers (as zstate.c) */
    fx_set_codebank(SfxPBR & 0xFFu);
    fx_set_rombank(SfxROMBR & 0xFFu);
    SfxRAMMem = (SfxRAMBR << 16) + (uintptr_t)sfxgameram;   /* as MainLoop */
    SfxRomBuffer = st.rombuf_off == 0xFFFFFFFFu ? 0 : SfxCROM + st.rombuf_off;
    SfxLastRamAdr = st.lastram_off == 0xFFFFFFFFu ? 0 : SfxRAMMem + st.lastram_off;
    SCBRrel = sfxgameram + SfxSCBR * 1024u;
    UpdatePORSCMR();   /* the screen height and the PLOT variant in the tables */
}

/* The 16-bit status word the CPU would read at $3030/$3031, without the
 * $3031 read's side effect (clearing the IRQ flag). For the idle skip. */
uint16_t gsu_zsnes_sfr_peek(void)
{
    return (uint16_t)(c_reg3030r() | ((SfxSFR >> 8) & 0xFFu) << 8);
}
