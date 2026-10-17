/* GSU scheduling — the SuperFX slice of ZSNES cpu/c_execute.c, verbatim except
 * SCBRrel now bases on sfxgameram (our shared cart RAM) and the 65816 IRQ is
 * raised through the glue flag `doirqnext` (polled in gsu.c). Included into the
 * gsu_core.c translation unit after fx_ops.h (which defines MainLoop). */

/* Raise a 65816 IRQ only when the GSU has stopped (Go clear, IRQ flag set,
 * IRQs unmasked); an IRQ while merely suspended mid-task makes games flip
 * buffers before rendering is done. */
static inline void SfxIRQpoll(void)
{
    if ((SfxSFR & 0x8020) == 0x8000 && !(SfxCFGR & 0x80)) {
        doirqnext = 1;
    }
}

/* Per-scanline GSU opcode budget: ~5.82M instr/s at 10.7 MHz, 2.5x at 21.4 MHz
 * (CLSR bit 0). Unlimited would run games too fast. */
static u4 SfxOpcodesPerLine(void)
{
    u4 const perLine = 5823405u / (romispal ? 50u * 312u : 60u * 262u);
    u4 const budget = SfxCLSR & 0x01 ? perLine * 5 / 2 : perLine;
    u4 percent = SuperFXClockMultiplier;

    if (percent < 50 || percent > 800) {
        percent = 100;
    }
    return budget * percent / 100;
}

void UpdatePORSCMR(void)
{
    {
        zreg eax; /* a host address: one of the line-location tables */
        if (SfxPOR & 0x10)
            goto objmode;
        switch (SfxSCMR & 0x24) { /* 4 + 32 */
        default:
            eax = sfx128lineloc;
            break;
        case 0x04:
            eax = sfx160lineloc;
            break;
        case 0x20:
            eax = sfx192lineloc;
            break;
        objmode:
        case 0x24:
            eax = sfxobjlineloc;
            break;
        }
        sfxclineloc = eax;
    }

    u4 const eax_ = (SfxPOR & 0x0F) << 2 | (SfxSCMR & 0x03);
    zreg const ebx = PLOTJmpb[eax_];
    zreg const eax = PLOTJmpa[eax_];
    FxTable[0x4C] = eax;
    FxTableb[0x4C] = eax;
    FxTablec[0x4C] = eax;
    FxTabled[0x4C] = ebx;
}

void UpdateSCBRCOLR(void)
{
    SCBRrel = sfxgameram + SfxSCBR * 1024;
    u4 const eax = SfxCOLR;
    fxbit01pcal = fxbit01[eax];
    fxbit23pcal = fxbit23[eax];
    fxbit45pcal = fxbit45[eax];
    fxbit67pcal = fxbit67[eax];
}

void UpdateCLSR(void)
{
    NumberOfOpcodes2 = SfxOpcodesPerLine();
}

static u1 SfxRanThisLine; /* GSU already got its slice this scanline */
static u4 SfxOwedOps;     /* budget banked while the bus is the 65816's */

void StartSFX(void)
{
    if (SfxRanThisLine) {
        SfxRanThisLine = 0;
        return;
    }
    if (SfxSCMR & ((SfxPBR & 0x7F) < 0x70 ? /* noram */ 0x10 : /* ram */ 0x08)) {
        NumberOfOpcodes = NumberOfOpcodes2 + SfxOwedOps;
        SfxOwedOps = 0;
#ifdef HOST_BUILD
        { extern int gsu_src; gsu_src = 0; }
#endif
        MainLoop();
        SfxIRQpoll();
    } else {
        SfxOwedOps += NumberOfOpcodes2;
        if (SfxOwedOps > NumberOfOpcodes2 * 240) {
            SfxOwedOps = NumberOfOpcodes2 * 240;
        }
    }
}

/* Run a slice as soon as the game starts the GSU, so short tasks finish before
 * the game polls the Go flag. */
void SfxExecOnStart(void)
{
    if (SfxSCMR & ((SfxPBR & 0x7F) < 0x70 ? /* noram */ 0x10 : /* ram */ 0x08)) {
        NumberOfOpcodes = NumberOfOpcodes2;
#ifdef HOST_BUILD
        { extern int gsu_src; gsu_src = 2; }
#endif
        MainLoop();
        SfxIRQpoll();
        SfxRanThisLine = 1;
    }
}

/* Finish the in-flight GSU task when vblank starts (bounded to ~a frame), so
 * the NMI handler never copies a half-drawn buffer. */
void SfxVblankCatchup(void)
{
    if ((SfxSFR & 0x20) &&
        (SfxSCMR & ((SfxPBR & 0x7F) < 0x70 ? /* noram */ 0x10 : /* ram */ 0x08))) {
        NumberOfOpcodes = NumberOfOpcodes2 * 240;
#ifdef HOST_BUILD
        { extern int gsu_src; gsu_src = 1; }
#endif
        MainLoop();
        SfxIRQpoll();
    }
}
