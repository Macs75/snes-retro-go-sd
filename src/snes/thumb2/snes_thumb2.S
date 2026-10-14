@ Cortex-M7 Thumb-2 65816 engine -- Stage 2 direct fetch/dispatch bridge.
@
@ snes_thumb2_try(Cpu *cpu, uint8_t opcode) -> int
@   r0 = Cpu*, r1 = opcode
@   returns 1 if the opcode was handled (Cpu state updated in place), 0 if not.
@
@ Contract: the caller has ALREADY performed the single opcode fetch and charged
@ cyclesPerOpcode[opcode]; on entry pc points just past the opcode byte. Each
@ native handler performs exactly the operand/data fetches and extra cycle
@ charges its opcode semantics require, so a handler is correct on BOTH the
@ snes_thumb2_try path (caller-owned fetch) and the snes_thumb2_step path
@ (self-fetch): pc consistently points at the operand. Handled families:
@   no-operand: CLC SEC CLI SEI CLV CLD SED NOP
@              TAX TAY TXA TYA TSX TXS TCD TDC TCS TSC TXY TYX
@              INA INX INY DEA DEX DEY XBA
@   accumulator shifts: ASL A LSR A ROL A ROR A
@   relative branches (one operand byte at pc, taken-cycle per cpu_doBranch):
@              BPL BMI BVC BVS BRA BCC BCS BNE BEQ
@   immediate ALU/load: ORA AND EOR LDY LDX LDA
@              ADC SBC CMP CPX CPY BIT #imm (ADC/SBC binary only -- decimal
@              D-flag path bails to C before operand fetch; BIT #imm is Z-only)
@   status masks: REP SEP
@   direct page (bank 0x00, 16-bit wrap, dp&0xff cycle): LDA LDX LDY dp,
@              AND ORA EOR dp, CMP dp, STA STX STY STZ dp, ADC/SBC dp (binary
@              only -- decimal D-flag path bails to C before operand fetch).
@   absolute (data bank db, 24-bit address, 24-bit high-wrap, no addr cycle):
@              LDA LDX LDY abs, AND ORA EOR abs, CMP abs, STA STX STY STZ abs,
@              ADC/SBC abs (binary only -- decimal bails to C before operand fetch).
@              RMW abs (ASL LSR ROL ROR INC DEC TSB TRB -- word write-back is
@              REVERSED high-then-low and charges +2 cycles, mirroring
@              cpu_writeWord(...,true)), BIT abs (N/V from the memory value),
@              CPX/CPY abs (XF width, not MF).
@   absolute-long (bank FROM OPERAND, 24-bit address, 24-bit high-wrap, no addr
@              cycle, no page-cross penalty): LDA AND ORA EOR CMP STA abl,
@              ADC/SBC abl (binary only -- decimal bails to C before operand
@              fetch); abl,X (alx) -- same 8 ops with +X 24-bit wrap (NOT uxth).
@   dp,Y (bank 0x00, 16-bit wrap, dp&0xff cycle): STX LDX dpy (XF width).
@   stack-relative (Family D, bank 0x00, NO addr cycle): sr,S -- ORA AND EOR
@              ADC STA LDA CMP SBC (16-bit wrap like DP, base CPU_SP not CPU_DP,
@              no dp&0xff charge); (sr,S),Y -- same 8 ops with 2 HOOKED pointer
@              reads at (sp+off)/+1 bank 0, db-banked 24-bit effective addr
@              ((db<<16)+ptr+Y)&0xffffff, NO write penalty (even STA isy 0x93).
@   stack push/pull (implied): PHP PHD PHA PHK PHY PHB PHX (push) and
@              PLP PLD PLA PLY PLB PLX (pull).  All via HOOKED bridges;
@              e-mode SP clamp (sp&0xff|0x100) on every byte.  PLP runs the
@              full setFlags re-normalization (e-force M/X, SP clamp, XF X/Y
@              truncation) -- same tail as REP/SEP.
@ Every width-sensitive opcode honours the M/X/E flags exactly as cpu_doOpcode
@ does. Unsupported opcodes return 0 and leave *cpu untouched, so the C
@ dispatcher falls through to cpu_doOpcode(cpu, opcode) on the fetched byte.
@
@ Build path: Makefile.common snes_asm_obj_prereq_gen runs this file through
@ the C preprocessor (capital .S), so #include "cpu_thumb2_offsets.h" resolves
@ next to this source; CORE_C_INCLUDES adds the wider search path. AAPCS:
@ caller-saved r0-r3 may be clobbered; callee-saved r4-r11 are preserved.

#include "cpu_thumb2_offsets.h"

        .syntax unified
        .cpu    cortex-m7
        .thumb

@ ---------------------------------------------------------------------------
@ setZN helper. Expects r4 = Cpu*, r2 = value already masked to the relevant
@ width (byte: 0..0xff, word: 0..0xffff), r3 = 1 for byte mode, 0 for word.
@ Writes CPU_Z (0/1) and CPU_N (0/1) -- normalised _Bool, matching what the C
@ compiler emits for `cpu->z = ... ; cpu->n = ...`.
        .macro SETZN
        cmp     r2, #0
        ite     eq
        moveq   r3, #1
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        cmp     r1, #0                  @ r1 holds byte-mode flag at call sites
        bne     8f
        lsr     r3, r2, #15             @ word mode: n = bit15 (value < 0x10000)
        strb    r3, [r4, #CPU_N]
        b       9f
8:      lsr     r3, r2, #7              @ byte mode: n = bit7 (value < 0x100)
        strb    r3, [r4, #CPU_N]
9:
        .endm

@ Load a bool field (0/1) into r3 for width tests. Field offset in \off.
        .macro LDB r, off
        ldrb    \r, [r4, #(\off)]
        .endm

@ ROM fetch-page cache inline fast-path (mirrors snes_cpuRead's cache in snes.c).
@ Expects: r0=Snes*, r1=fetch_addr. Clobbers r2,r3. Returns byte in r0.
@ On HIT: serves directly from romPageBase (skips the BL + function prologue).
@ On MISS: falls through to bl snes_cpuRead (installs cache + updates counters).
@ The NULL-romPageBase guard rejects the un-installed state (base is NULL until
@ the C path first installs a real page pointer + tag).
        .macro ROM_FETCH_FAST
#ifdef SNES_T2_NO_ROMCACHE
        b       .Lrom_miss_\@                    @ measurement arm: never serve inline
#endif
        ldr     r3, [r0, #SNES_ROMPAGEBASE]     @ base ptr
        cbz     r3, .Lrom_miss_\@                @ NULL → cache not installed → full BL
        ldr     r3, [r0, #SNES_ROMPAGETAG]
        eor     r2, r1, r3
        lsrs    r2, r2, #13                     @ Z iff addr and tag share bits 13+
        bne     .Lrom_miss_\@                    @ different page → full BL
        ldrb    r2, [r0, #SNES_CPUMEMOPS]       @ HIT: cpuMemOps++
        adds    r2, r2, #1
        strb    r2, [r0, #SNES_CPUMEMOPS]
        @ No cpuCyclesLeft charge here: the caller charges 6*cycles + 2*memOps
        @ once per opcode, which is the same total. Mirrors snes_cpuRead.
        ldr     r3, [r0, #SNES_ROMPAGEBASE]
        ubfx    r2, r1, #0, #13                 @ adr & 0x1fff
        ldrb    r0, [r3, r2]                    @ byte from cached ROM page
        b       .Lrom_done_\@
.Lrom_miss_\@:
        bl      snes_cpuRead
.Lrom_done_\@:
        .endm

@ Inline one raw operand fetch.  This deliberately is a macro, not a callable
@ local helper: the contained BL may overwrite LR because the outer entry saved
@ its caller LR in the common prologue.  A BL-called helper containing this BL
@ would instead lose its own return address.  r6 carries all live state across
@ the C call (AAPCS callee-saved).
        .macro IMM_FETCH_BYTE
        ldr     r0, [r4, #CPU_MEM]
        ldrb    r1, [r4, #CPU_K]
        ldrh    r2, [r4, #CPU_PC]
        lsl     r1, r1, #16
        orr     r1, r1, r2
        adds    r3, r2, #1
        strh    r3, [r4, #CPU_PC]       @ uint16 PC wrap; K never carries
        ROM_FETCH_FAST
        uxtb    r0, r0
        .endm

@ --- Stage 3D direct-page helpers.  Like IMM_FETCH_BYTE these are macros, not
@ subroutines: each contains a BL, and a BL-called helper would lose its own
@ return address (the outer entry already saved the caller LR).  r6 carries the
@ only live state across the C calls (AAPCS callee-saved). ---

@ DP_ADDR: fetch the DP offset operand (RAW -- matches cpu_adrDp's
@ cpu_readOpcode = cpu_read_raw, NO spin hook), compute r6 = addr_low =
@ (dp + offset) & 0xffff.  Direct page is ALWAYS bank 0x00 with 16-bit wrap
@ (it does NOT use db), matching cpu_adrDp.  Charges +1 cycle when (dp & 0xff)
@ != 0.  Clobbers r0,r1,r2,r3; leaves r6 = addr_low.
        .macro DP_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte; pc++ (uint16 wrap)
        ldrh    r2, [r4, #CPU_DP]
        add     r6, r2, r0              @ dp + offset (may carry into bit 16)
        uxth    r6, r6                  @ addr_low = (dp+offset) & 0xffff, bank 0
        ldrb    r2, [r4, #CPU_DP]       @ dp low byte (little-endian field base)
        lsls    r2, r2, #24             @ Z set iff dp low byte == 0
        beq     82f                     @ dp & 0xff == 0: no extra cycle
        ldrb    r2, [r4, #CPU_CYCLESUSED]
        adds    r2, r2, #1
        strb    r2, [r4, #CPU_CYCLESUSED]
82:
        .endm

@ DP_READ_VALUE wid: given r6 = addr_low, read the data operand into r2 (byte:
@ 0..0xff, word: 0..0xffff) honouring the \wid flag (CPU_MF or CPU_XF).  Word
@ mode charges +1 cycle and reads low@addr_low then high@addr_high where
@ addr_high = (addr_low + 1) & 0xffff -- the high address is derived into r1
@ (the arg register) just before r6 is reused to carry the low byte, so no
@ second callee-saved register is needed.  r6 holds the low byte in word mode
@ (addr_low is spent by then).  Clobbers r0,r1,r3 (and r6 in word mode).
        .macro DP_READ_VALUE wid
        ldrb    r1, [r4, #\wid]
        cbz     r1, .Ldprw_\@
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ byte value
        b       .Ldprd_\@
.Ldprw_\@:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read         @ r0 = low byte; r6 still addr_low (callee-saved)
        add     r1, r6, #1
        uxth    r1, r1                  @ r1 = addr_high (formed before r6 is reused)
        mov     r6, r0                  @ r6 = low byte (addr_low no longer needed)
        mov     r0, r4                  @ r1 still holds addr_high -> arg
        bl      cpu_thumb2_read         @ r0 = high byte
        orr     r2, r6, r0, lsl #8      @ word value (low | high<<8)
.Ldprd_\@:
        .endm

@ DPX_ADDR: fetch the DP offset operand (RAW -- matches cpu_adrDpx's
@ cpu_readOpcode = cpu_read_raw, NO spin hook), compute r6 = addr_low =
@ (dp + offset + X) & 0xffff.  Like DP_ADDR but adds the X index register,
@ matching cpu_adrDpx (cpu.c:385-389).  Direct page is ALWAYS bank 0x00 with
@ 16-bit wrap (it does NOT use db), matching cpu_adrDpx -- the +X term wraps
@ within bank 0 the same way (dp+off+X)&0xffff, NOT a 24-bit wrap.  Charges
@ +1 cycle when (dp & 0xff) != 0 (SAME penalty as DP_ADDR).  Clobbers
@ r0,r1,r2,r3; leaves r6 = addr_low.
        .macro DPX_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte; pc++ (uint16 wrap)
        ldrh    r2, [r4, #CPU_DP]
        ldrh    r3, [r4, #CPU_X]        @ X index
        add     r6, r2, r0              @ dp + offset
        add     r6, r6, r3              @ dp + offset + X
        uxth    r6, r6                  @ addr_low = (dp+off+X) & 0xffff, bank 0
        ldrb    r2, [r4, #CPU_DP]       @ dp low byte (little-endian field base)
        lsls    r2, r2, #24             @ Z set iff dp low byte == 0
        beq     86f                     @ dp & 0xff == 0: no extra cycle
        ldrb    r2, [r4, #CPU_CYCLESUSED]
        adds    r2, r2, #1
        strb    r2, [r4, #CPU_CYCLESUSED]
86:
        .endm

@ DPY_ADDR: fetch the DP offset operand (RAW -- matches cpu_adrDpo's
@ cpu_readOpcode = cpu_read_raw, NO spin hook), compute r6 = addr_low =
@ (dp + offset + Y) & 0xffff.  Like DPX_ADDR but adds the Y index register,
@ matching cpu_adrDpo (cpu.c:392-397).  Direct page is ALWAYS bank 0x00 with
@ 16-bit wrap (it does NOT use db), matching cpu_adrDpo -- the +Y term wraps
@ within bank 0 the same way (dp+off+Y)&0xffff, NOT a 24-bit wrap.  Charges
@ +1 cycle when (dp & 0xff) != 0 (SAME penalty as DP_ADDR/DPX_ADDR).
@ Clobbers r0,r1,r2,r3; leaves r6 = addr_low.
        .macro DPY_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte; pc++ (uint16 wrap)
        ldrh    r2, [r4, #CPU_DP]
        ldrh    r3, [r4, #CPU_Y]        @ Y index (NOT X)
        add     r6, r2, r0              @ dp + offset
        add     r6, r6, r3              @ dp + offset + Y
        uxth    r6, r6                  @ addr_low = (dp+off+Y) & 0xffff, bank 0
        ldrb    r2, [r4, #CPU_DP]       @ dp low byte (little-endian field base)
        lsls    r2, r2, #24             @ Z set iff dp low byte == 0
        beq     88f                     @ dp & 0xff == 0: no extra cycle
        ldrb    r2, [r4, #CPU_CYCLESUSED]
        adds    r2, r2, #1
        strb    r2, [r4, #CPU_CYCLESUSED]
88:
        .endm

@ --- Stage 3E absolute helpers.  Same macro-not-subroutine rule as DP: each
@ contains a BL, and r6 carries the only live state across the C calls. ---

@ ABS_ADDR: fetch the two absolute operand bytes (RAW -- matches cpu_adrAbs's
@ cpu_readOpcodeWord = cpu_read_raw, NO spin hook), compute r6 = addr_low =
@ (db << 16) | (lo | hi<<8).  Absolute addressing uses the data-bank register db
@ (NOT bank 0 like DP) and a 24-bit effective address, matching cpu_adrAbs.
@ Unlike cpu_adrDp, cpu_adrAbs charges NO cycle, so neither do we.  Clobbers
@ r0,r1,r2,r3; leaves r6 = addr_low (24-bit).
        .macro ABS_ADDR
        IMM_FETCH_BYTE                  @ r0 = addr_lo; pc++ (uint16 wrap)
        mov     r6, r0                  @ preserve lo across 2nd fetch (r6 callee-saved)
        IMM_FETCH_BYTE                  @ r0 = addr_hi; pc++
        ldrb    r1, [r4, #CPU_DB]
        lsl     r1, r1, #16             @ r1 = db << 16
        orr     r6, r6, r0, lsl #8      @ r6 = lo | (hi << 8) = adr (0..0xffff)
        orr     r6, r1, r6              @ r6 = (db<<16) | adr = addr_low (24-bit)
        .endm

@ ABS_READ_VALUE wid: given r6 = addr_low (24-bit), read the data operand into r2
@ (byte: 0..0xff, word: 0..0xffff) honouring the \wid flag (CPU_MF or CPU_XF).
@ Word mode charges +1 cycle and reads low@addr_low then high@addr_high where
@ addr_high = (addr_low + 1) & 0xffffff -- 24-bit bank wrap (NOT 16-bit uxth like
@ DP), matching cpu_adrAbs's high = ((db<<16)+adr+1) & 0xffffff.  Clobbers
@ r0,r1,r3 (and r6 in word mode).
        .macro ABS_READ_VALUE wid
        ldrb    r1, [r4, #\wid]
        cbz     r1, .Labrw_\@
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ byte value
        b       .Labrd_\@
.Labrw_\@:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read         @ r0 = low byte; r6 still addr_low
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        mov     r6, r0                  @ r6 = low byte (addr_low no longer needed)
        mov     r0, r4                  @ r1 still holds addr_high -> arg
        bl      cpu_thumb2_read         @ r0 = high byte
        orr     r2, r6, r0, lsl #8      @ word value (low | high<<8)
.Labrd_\@:
        .endm

@ --- Stage 3J absolute-long helpers.  Same macro-not-subroutine rule as ABS:
@ each contains BLs (IMM_FETCH_BYTE x3), and r6 carries the only live state
@ across the C calls (AAPCS callee-saved). ---

@ ABL_ADDR: fetch the THREE absolute-long operand bytes (RAW -- matches
@ cpu_adrAbl's cpu_readOpcodeWord + cpu_readOpcode = cpu_read_raw, NO spin
@ hook), compute r6 = lo | (hi<<8) | (bank<<16) = 24-bit addr.  Absolute-long
@ addressing takes the BANK FROM THE OPERAND (3rd byte), NOT from cpu->db,
@ matching cpu_adrAbl (cpu.c:478-483).  NO address-computation cycle (unlike
@ DP), NO page-cross penalty (long addressing never page-crosses).  Clobbers
@ r0-r3; leaves r6 = addr_low (24-bit, bank from operand).
        .macro ABL_ADDR
        IMM_FETCH_BYTE                  @ r0 = lo; pc++ (uint16 wrap)
        mov     r6, r0                  @ r6 = lo (callee-saved, survives next fetch)
        IMM_FETCH_BYTE                  @ r0 = hi; pc++
        orr     r6, r6, r0, lsl #8      @ r6 = lo | (hi << 8)
        IMM_FETCH_BYTE                  @ r0 = bank; pc++
        orr     r6, r6, r0, lsl #16     @ r6 = lo|(hi<<8)|(bank<<16) = 24-bit addr
        .endm

@ ALX_ADDR: ABL_ADDR then add X index with 24-bit wrap, matching cpu_adrAlx
@ (cpu.c:485-490).  The +X term wraps within the FULL 24-bit address
@ ((adr+X)&0xffffff), NOT 16-bit uxth like dpx -- X can carry into the bank
@ byte.  NO page-cross penalty (long addressing never penalizes).  Clobbers
@ r0-r3; leaves r6 = addr_low (24-bit, bank from operand).
        .macro ALX_ADDR
        ABL_ADDR
        ldrh    r3, [r4, #CPU_X]
        add     r6, r6, r3
        ubfx    r6, r6, #0, #24         @ (adr + X) & 0xffffff, 24-bit wrap (NOT uxth!)
        .endm

@ --- Stage 3F absolute,X / absolute,Y helpers.  Same macro-not-subroutine rule
@ as ABS_ADDR: each contains BLs (IMM_FETCH_BYTE), and r6 carries the only live
@ state across the C calls (AAPCS callee-saved). ---

@ ABXY_ADDR write idxoff: fetch the two operand bytes (RAW -- matches
@ cpu_adrAbx/Aby's cpu_readOpcodeWord = cpu_read_raw, NO spin hook), compute
@ r6 = addr_low = ((db<<16) + adr + idx) & 0xffffff with 24-bit wrap, matching
@ cpu_adrAbx/Aby (cpu.c:462-476).  \idxoff = CPU_X (abx) or CPU_Y (aby).
@ Page-cross penalty (cpu.c:464/472): ONLY writes (\write=1) pay +1 cycle when
@ EITHER xf==0 (16-bit index, short-circuit -- the || skips the page test) OR
@ (adr>>8) != ((adr+idx)>>8) (256-byte page cross on the 16-bit operand adr).
@ Reads (\write=0) pay ZERO penalty always.  Clobbers r0-r3; r6 = addr_low.
        .macro ABXY_ADDR write idxoff
        IMM_FETCH_BYTE                  @ r0 = lo; pc++ (uint16 wrap)
        mov     r6, r0                  @ preserve lo across 2nd fetch
        IMM_FETCH_BYTE                  @ r0 = hi; pc++
        orr     r6, r6, r0, lsl #8     @ r6 = adr (0..0xffff, zero-extended)
        ldrh    r2, [r4, #\idxoff]     @ r2 = idx (always needed for addr_low)
        .if \write
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, 83f                 @ xf==0 -> always penalty (short-circuit)
        mov     r3, r6                  @ r3 = adr
        lsr     r3, r3, #8              @ r3 = adr>>8
        add     r0, r6, r2              @ r0 = adr+idx (0..0x1fffe, 32-bit like C)
        lsr     r0, r0, #8              @ r0 = (adr+idx)>>8
        cmp     r3, r0
        beq     84f                     @ same page -> no penalty
83:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
84:
        .endif
        ldrb    r1, [r4, #CPU_DB]
        lsl     r1, r1, #16             @ r1 = db<<16
        add     r6, r6, r2              @ r6 = adr + idx
        add     r6, r6, r1              @ r6 = db<<16 + adr + idx
        ubfx    r6, r6, #0, #24         @ r6 = addr_low & 0xffffff
        .endm

@ --- Stage 3G DP-indirect helpers.  Same macro-not-subroutine rule as the
@ abs/abx macros: each contains BLs (IMM_FETCH_BYTE + cpu_thumb2_read), and r6
@ carries the only live state across the C calls (AAPCS callee-saved).  The
@ pointer fetch is TWO HOOKED cpu_thumb2_read calls at bank-0 16-bit-wrapped
@ addresses (cpu.c cpu_adrIdp/Idx/Idy -> cpu_readWord -> cpu_read, NOT raw),
@ matching cpu.c:399-423.  The single operand byte IS raw (IMM_FETCH_BYTE =
@ cpu_readOpcode).  Register pressure: two values must survive across the two
@ pointer-fetch BLs -- ptr_lo_addr (for the high-byte address) and ptr_lo_byte
@ (for the final pointer).  r0-r3 are caller-saved and clobbered by each BL.
@ ptr_lo_addr goes in r6 (callee-saved); ptr_lo_byte is stack-spilled across
@ the high-byte BL (push {r0}/pop {rN}, balanced -- same pattern as the 3E
@ RMW word handlers).  r6 ends = addr_low (24-bit). ---

@ IDP_ADDR ($dp): IMM_FETCH_BYTE -> dp&0xff penalty -> 2 HOOKED reads at
@ (dp+off)&0xffff / +1 -> pointer -> r6 = (db<<16)|pointer.  Pointer fetch
@ addresses 16-bit-wrap bank 0 (uxth, NOT ubfx #0,#24).  Clobbers r0-r3;
@ r6 = addr_low (24-bit).  cpu.c:399-404.
        .macro IDP_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte; pc++ (uint16 wrap)
        ldrh    r2, [r4, #CPU_DP]
        add     r2, r2, r0              @ dp + offset (0..0x100fe)
        uxth    r6, r2                  @ r6 = ptr_lo_addr = (dp+off)&0xffff (callee-saved)
        ldrb    r2, [r4, #CPU_DP]       @ dp low byte (little-endian field base)
        lsls    r2, r2, #24             @ Z set iff dp low byte == 0
        beq     85f                     @ dp & 0xff == 0: no extra cycle
        ldrb    r2, [r4, #CPU_CYCLESUSED]
        adds    r2, r2, #1
        strb    r2, [r4, #CPU_CYCLESUSED]
85:
        mov     r0, r4
        mov     r1, r6                  @ ptr_lo_addr (0..0xffff, bank 0)
        bl      cpu_thumb2_read         @ r0 = ptr_lo byte (HOOKED); r6 survives
        push    {r0}                    @ stash ptr_lo byte across high-byte BL
        add     r1, r6, #1              @ r6 still = ptr_lo_addr (callee-saved)
        uxth    r1, r1                  @ ptr_hi_addr = (ptr_lo_addr+1) & 0xffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_hi byte (HOOKED)
        pop     {r1}                    @ r1 = ptr_lo byte
        orr     r6, r1, r0, lsl #8      @ r6 = pointer (0..0xffff)
        ldrb    r1, [r4, #CPU_DB]
        lsl     r1, r1, #16             @ r1 = db << 16
        orr     r6, r1, r6              @ r6 = (db<<16) | pointer = addr_low
        .endm

@ IDX_ADDR ($dp,X): same as IDP but pointer at (dp+off+X)&0xffff / +1.
@ cpu.c:407-412.
        .macro IDX_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte
        ldrh    r2, [r4, #CPU_DP]
        add     r2, r2, r0              @ dp + offset
        ldrh    r3, [r4, #CPU_X]        @ X index
        add     r2, r2, r3              @ dp + offset + X (0..0x200fd)
        uxth    r6, r2                  @ r6 = ptr_lo_addr = (dp+off+X)&0xffff
        ldrb    r2, [r4, #CPU_DP]       @ dp low byte
        lsls    r2, r2, #24
        beq     85f
        ldrb    r2, [r4, #CPU_CYCLESUSED]
        adds    r2, r2, #1
        strb    r2, [r4, #CPU_CYCLESUSED]
85:
        mov     r0, r4
        mov     r1, r6                  @ ptr_lo_addr (bank 0)
        bl      cpu_thumb2_read         @ r0 = ptr_lo byte (HOOKED); r6 survives
        push    {r0}                    @ stash ptr_lo byte
        add     r1, r6, #1              @ r6 still = ptr_lo_addr
        uxth    r1, r1                  @ ptr_hi_addr
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_hi byte (HOOKED)
        pop     {r1}                    @ r1 = ptr_lo byte
        orr     r6, r1, r0, lsl #8      @ r6 = pointer (0..0xffff)
        ldrb    r1, [r4, #CPU_DB]
        lsl     r1, r1, #16
        orr     r6, r1, r6              @ r6 = (db<<16) | pointer = addr_low
        .endm

@ IDY_ADDR write ($dp),Y: IMM_FETCH_BYTE -> dp&0xff penalty -> 2 HOOKED reads
@ for pointer -> .if \write: penalty block (!xf || ptr page-cross) -> r6 =
@ ((db<<16)+pointer+Y)&0xffffff (SINGLE 24-bit add, Y carries into bank).
@ cpu.c:415-422.  Write penalty (0x91 STA idy ONLY): +1 if xf==0 OR
@ (pointer>>8)!=((pointer+Y)>>8).  Reads pay nothing.
        .macro IDY_ADDR write
        IMM_FETCH_BYTE                  @ r0 = offset byte
        ldrh    r2, [r4, #CPU_DP]
        add     r2, r2, r0              @ dp + offset (0..0x100fe)
        uxth    r6, r2                  @ r6 = ptr_lo_addr = (dp+off)&0xffff
        ldrb    r2, [r4, #CPU_DP]       @ dp low byte
        lsls    r2, r2, #24
        beq     85f
        ldrb    r2, [r4, #CPU_CYCLESUSED]
        adds    r2, r2, #1
        strb    r2, [r4, #CPU_CYCLESUSED]
85:
        mov     r0, r4
        mov     r1, r6                  @ ptr_lo_addr (bank 0)
        bl      cpu_thumb2_read         @ r0 = ptr_lo byte (HOOKED); r6 survives
        push    {r0}                    @ stash ptr_lo byte
        add     r1, r6, #1              @ r6 still = ptr_lo_addr
        uxth    r1, r1                  @ ptr_hi_addr
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_hi byte (HOOKED)
        pop     {r1}                    @ r1 = ptr_lo byte
        orr     r6, r1, r0, lsl #8      @ r6 = pointer (0..0xffff)
        .if \write
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, 86f                 @ xf==0 -> always penalty (short-circuit)
        ldrh    r1, [r4, #CPU_Y]
        lsr     r2, r6, #8              @ pointer>>8 (0..0xff)
        add     r3, r6, r1              @ pointer + Y (0..0x1fffe, 32-bit like C)
        lsr     r3, r3, #8             @ (pointer+Y)>>8 (0..0x1ff)
        cmp     r2, r3
        beq     87f                     @ same page -> no penalty
86:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
87:
        .endif
        ldrh    r1, [r4, #CPU_Y]
        ldrb    r2, [r4, #CPU_DB]
        lsl     r2, r2, #16             @ r2 = db<<16
        add     r6, r6, r1              @ pointer + Y
        add     r6, r6, r2              @ (db<<16) + pointer + Y
        ubfx    r6, r6, #0, #24        @ r6 = addr_low & 0xffffff (single 24-bit add)
        .endm

@ --- Stage 3G Pass 2 -- DP-indirect-long ([dp] / [dp],Y).  Like the Pass-1
@ indirect macros but the pointer is THREE bytes (lo, hi, bank) and the bank
@ comes from MEMORY (the 3rd pointer byte), NOT db, NOT k.  This means three
@ HOOKED pointer-fetch BLs instead of two.  Register pressure: ptr_lo and
@ ptr_hi bytes must BOTH survive across the ptr_bank BL.  r6 holds ptr_base
@ (callee-saved) across all three BLs; ptr_lo and ptr_hi are stack-spilled
@ (push {r0} x2 / pop {rN} x2, balanced -- 2-deep, same proven pattern as the
@ 3E RMW word handlers).  r6 ends = addr_low (24-bit, bank from memory).

@ IDL_ADDR [dp]: IMM_FETCH_BYTE -> dp&0xff penalty -> 3 HOOKED reads at
@ (dp+off)&0xffff / +1 / +2 -> pointer = lo|(hi<<8)|(bank<<16) -> r6 = pointer
@ (24-bit, BANK FROM MEMORY).  Pointer-fetch addresses 16-bit-wrap bank 0
@ (uxth, NOT ubfx).  NO write penalty (even STA 0x87).  Clobbers r0-r3;
@ r6 = addr_low (24-bit).  cpu.c:425-431.
        .macro IDL_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte; pc++ (uint16 wrap)
        ldrh    r2, [r4, #CPU_DP]
        add     r2, r2, r0              @ dp + offset (0..0x100fe)
        uxth    r6, r2                  @ r6 = ptr_base = (dp+off)&0xffff (callee-saved)
        ldrb    r2, [r4, #CPU_DP]       @ dp low byte (little-endian field base)
        lsls    r2, r2, #24             @ Z set iff dp low byte == 0
        beq     85f                     @ dp & 0xff == 0: no extra cycle
        ldrb    r2, [r4, #CPU_CYCLESUSED]
        adds    r2, r2, #1
        strb    r2, [r4, #CPU_CYCLESUSED]
85:
        mov     r0, r4
        mov     r1, r6                  @ ptr_lo_addr (bank 0)
        bl      cpu_thumb2_read         @ r0 = ptr_lo byte (HOOKED); r6 survives
        push    {r0}                    @ stash ptr_lo byte across ptr_hi BL
        add     r1, r6, #1              @ r6 still = ptr_base
        uxth    r1, r1                  @ ptr_hi_addr = (ptr_base+1) & 0xffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_hi byte (HOOKED)
        push    {r0}                    @ stash ptr_hi byte across ptr_bank BL
        add     r1, r6, #2              @ r6 still = ptr_base
        uxth    r1, r1                  @ ptr_bank_addr = (ptr_base+2) & 0xffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_bank byte (HOOKED)
        lsl     r0, r0, #16             @ bank << 16
        pop     {r1}                    @ r1 = ptr_hi byte
        orr     r0, r0, r1, lsl #8      @ (bank<<16) | (ptr_hi<<8)
        pop     {r1}                    @ r1 = ptr_lo byte
        orr     r6, r1, r0              @ r6 = pointer (24-bit, bank from memory)
        .endm

@ ILY_ADDR [dp],Y: same 3-byte pointer fetch as IDL_ADDR, then r6 =
@ (pointer + Y) & 0xffffff (SINGLE 24-bit add, Y carries into bank -- NOT
@ db-relative).  NO write penalty (even STA 0x97).  cpu.c:434-440.
        .macro ILY_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte; pc++ (uint16 wrap)
        ldrh    r2, [r4, #CPU_DP]
        add     r2, r2, r0              @ dp + offset (0..0x100fe)
        uxth    r6, r2                  @ r6 = ptr_base = (dp+off)&0xffff (callee-saved)
        ldrb    r2, [r4, #CPU_DP]       @ dp low byte (little-endian field base)
        lsls    r2, r2, #24             @ Z set iff dp low byte == 0
        beq     85f                     @ dp & 0xff == 0: no extra cycle
        ldrb    r2, [r4, #CPU_CYCLESUSED]
        adds    r2, r2, #1
        strb    r2, [r4, #CPU_CYCLESUSED]
85:
        mov     r0, r4
        mov     r1, r6                  @ ptr_lo_addr (bank 0)
        bl      cpu_thumb2_read         @ r0 = ptr_lo byte (HOOKED); r6 survives
        push    {r0}                    @ stash ptr_lo byte across ptr_hi BL
        add     r1, r6, #1              @ r6 still = ptr_base
        uxth    r1, r1                  @ ptr_hi_addr = (ptr_base+1) & 0xffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_hi byte (HOOKED)
        push    {r0}                    @ stash ptr_hi byte across ptr_bank BL
        add     r1, r6, #2              @ r6 still = ptr_base
        uxth    r1, r1                  @ ptr_bank_addr = (ptr_base+2) & 0xffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_bank byte (HOOKED)
        lsl     r0, r0, #16             @ bank << 16
        pop     {r1}                    @ r1 = ptr_hi byte
        orr     r0, r0, r1, lsl #8      @ (bank<<16) | (ptr_hi<<8)
        pop     {r1}                    @ r1 = ptr_lo byte
        orr     r6, r1, r0              @ r6 = pointer (24-bit)
        ldrh    r1, [r4, #CPU_Y]
        add     r6, r6, r1              @ pointer + Y (32-bit add, Y carries into bank)
        ubfx    r6, r6, #0, #24         @ r6 = (pointer + Y) & 0xffffff
        .endm

@ --- Stage 3M stack-relative helpers (Family D).  Same macro-not-subroutine
@ rule as DP/IDY: each contains BLs, and r6 carries the only live state across
@ the C calls (AAPCS callee-saved).  Unlike DP_ADDR/IDY_ADDR these use CPU_SP
@ (NOT CPU_DP) for the base and charge NO dp&0xff cycle (cpu_adrSr/Isy in
@ cpu.c:443-454 add no cycle). ---

@ SR_ADDR (sr,S): IMM_FETCH_BYTE (raw offset byte) -> r6 = (sp+offset)&0xffff.
@ Stack-relative is ALWAYS bank 0x00 with 16-bit wrap (it does NOT use db),
@ matching cpu_adrSr (cpu.c:443-447).  NO address-computation cycle (unlike
@ DP_ADDR's dp&0xff charge).  Clobbers r0,r3; leaves r6 = addr_low.
        .macro SR_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte; pc++ (uint16 wrap)
        ldrh    r3, [r4, #CPU_SP]
        add     r6, r3, r0              @ sp + offset (may carry into bit 16)
        uxth    r6, r6                  @ addr_low = (sp+offset) & 0xffff, bank 0
        .endm

@ ISY_ADDR ((sr,S),Y): IMM_FETCH_BYTE (raw offset) -> 2 HOOKED reads at
@ (sp+off)&0xffff / +1 (bank 0, uxth 16-bit wrap) -> pointer = lo|(hi<<8) ->
@ r6 = ((db<<16)+pointer+Y)&0xffffff (SINGLE 24-bit add, Y carries into bank,
@ like IDY_ADDR).  Pointer base uses CPU_SP (NOT CPU_DP).  NO dp&0xff cycle.
@ NO write penalty (even STA isy 0x93 pays nothing -- cpu_adrIsy has no write
@ param, unlike cpu_adrIdy's).  Clobbers r0-r3; r6 = addr_low (24-bit).
@ cpu.c:449-454.
        .macro ISY_ADDR
        IMM_FETCH_BYTE                  @ r0 = offset byte
        ldrh    r2, [r4, #CPU_SP]
        add     r2, r2, r0              @ sp + offset (0..0x100fe)
        uxth    r6, r2                  @ r6 = ptr_lo_addr = (sp+off)&0xffff
        mov     r0, r4
        mov     r1, r6                  @ ptr_lo_addr (bank 0)
        bl      cpu_thumb2_read         @ r0 = ptr_lo byte (HOOKED); r6 survives
        push    {r0}                    @ stash ptr_lo byte across high-byte BL
        add     r1, r6, #1              @ r6 still = ptr_lo_addr (callee-saved)
        uxth    r1, r1                  @ ptr_hi_addr = (ptr_lo_addr+1) & 0xffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_hi byte (HOOKED)
        pop     {r1}                    @ r1 = ptr_lo byte
        orr     r6, r1, r0, lsl #8      @ r6 = pointer (0..0xffff)
        ldrh    r1, [r4, #CPU_Y]
        ldrb    r2, [r4, #CPU_DB]
        lsl     r2, r2, #16             @ r2 = db<<16
        add     r6, r6, r1              @ pointer + Y
        add     r6, r6, r2              @ (db<<16) + pointer + Y
        ubfx    r6, r6, #0, #24         @ r6 = addr_low & 0xffffff (single 24-bit add)
        .endm

@ --- stack push/pull primitives (Stage 3K) ---
@ All stack accesses use the HOOKED cpu_thumb2_read/write bridges.
@ r4=Cpu* throughout.  r0-r3 are clobbered; r6 (callee-saved) survives.
@ Emulation-mode SP clamp: (sp & 0xff) | 0x100, applied AFTER inc/dec.

@ PUSH_BYTE: caller sets r2 = byte value.  Writes [SP], SP--, e-clamp.
        .macro PUSH_BYTE
        ldrh    r1, [r4, #CPU_SP]
        mov     r0, r4                  @ Cpu*
        bl      cpu_thumb2_write        @ [SP] = r2
        ldrh    r3, [r4, #CPU_SP]
        subs    r3, r3, #1
        strh    r3, [r4, #CPU_SP]
        ldrb    r0, [r4, #CPU_E]
        cbz     r0, 81f
        uxtb    r3, r3
        add.w   r3, r3, #0x100
        strh    r3, [r4, #CPU_SP]
81:
        .endm

@ PULL_BYTE: SP++, e-clamp, reads [SP] -> r0 (byte).  Clobbers r0-r3.
        .macro PULL_BYTE
        ldrh    r3, [r4, #CPU_SP]
        adds    r3, r3, #1
        strh    r3, [r4, #CPU_SP]
        ldrb    r0, [r4, #CPU_E]
        cbz     r0, 82f
        uxtb    r3, r3
        add.w   r3, r3, #0x100
        strh    r3, [r4, #CPU_SP]
82:
        ldrh    r1, [r4, #CPU_SP]
        mov     r0, r4
        bl      cpu_thumb2_read
        uxtb    r0, r0
        .endm

@ PUSH_WORD: caller sets r6 = word value.  HIGH byte first, then LOW.
@ r6 survives both PUSH_BYTE BLs (callee-saved).
        .macro PUSH_WORD
        lsr     r2, r6, #8              @ HIGH byte
        PUSH_BYTE
        uxtb    r2, r6                  @ LOW byte (r6 intact)
        PUSH_BYTE
        .endm

@ PULL_WORD: returns word in r6.  LOW first, then HIGH.
        .macro PULL_WORD
        PULL_BYTE                        @ r0 = low
        mov     r6, r0
        PULL_BYTE                        @ r0 = high
        lsl     r0, r0, #8
        orr     r6, r6, r0
        .endm

@ SETFLAGS_TAIL: caller provides r0 = flags byte.  Unpacks all 8 flag bits
@ into their Cpu fields (each stored as 0/1, matching cpu_setFlags's bool
@ assignment of val&mask), then runs the setFlags re-normalization that
@ cpu_setFlags applies: e-check (emulation pins M=X=1 and clamps SP to
@ (sp&0xff)|0x100) then xf-check (truncates X and Y to 8 bits).  Clobbers
@ r0,r1.  REUSED by PLP (0x28) and RTI (0x40) -- both call cpu_setFlags on
@ a pulled byte.  Numeric labels 89/90 are unused elsewhere in this file.
        .macro SETFLAGS_TAIL
        ubfx    r1, r0, #7, #1          @ N
        strb    r1, [r4, #CPU_N]
        ubfx    r1, r0, #6, #1          @ V
        strb    r1, [r4, #CPU_V]
        ubfx    r1, r0, #5, #1          @ MF
        strb    r1, [r4, #CPU_MF]
        ubfx    r1, r0, #4, #1          @ XF
        strb    r1, [r4, #CPU_XF]
        ubfx    r1, r0, #3, #1          @ D
        strb    r1, [r4, #CPU_D]
        ubfx    r1, r0, #2, #1          @ I
        strb    r1, [r4, #CPU_I]
        ubfx    r1, r0, #1, #1          @ Z
        strb    r1, [r4, #CPU_Z]
        ubfx    r1, r0, #0, #1          @ C
        strb    r1, [r4, #CPU_C]
        @ setFlags re-normalization (mirrors cpu_setFlags cpu.c:285-293)
        ldrb    r0, [r4, #CPU_E]
        cbz     r0, 89f
        movs    r0, #1
        strb    r0, [r4, #CPU_MF]       @ emulation pins M=X=1
        strb    r0, [r4, #CPU_XF]
        ldrh    r1, [r4, #CPU_SP]
        uxtb    r1, r1
        add.w   r1, r1, #0x100
        strh    r1, [r4, #CPU_SP]
89:
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, 90f
        ldrh    r1, [r4, #CPU_X]
        uxtb    r1, r1
        strh    r1, [r4, #CPU_X]
        ldrh    r1, [r4, #CPU_Y]
        uxtb    r1, r1
        strh    r1, [r4, #CPU_Y]
90:
        .endm

@ Dense O(1) opcode dispatch.  Table entries carry the Thumb bit because the
@ local handler labels are not function symbols and therefore do not acquire
@ it automatically.
        .macro DISPATCH_ENTRY op
        .if \op == 0x18
        .word   .Lop18 + 1
        .elseif \op == 0x38
        .word   .Lop38 + 1
        .elseif \op == 0x58
        .word   .Lop58 + 1
        .elseif \op == 0x78
        .word   .Lop78 + 1
        .elseif \op == 0xb8
        .word   .Lopb8 + 1
        .elseif \op == 0xd8
        .word   .Lopd8 + 1
        .elseif \op == 0xf8
        .word   .Lopf8 + 1
        .elseif \op == 0xea
        .word   .Lopea + 1
        .elseif \op == 0x9a
        .word   .Lop9a + 1
        .elseif \op == 0x1b
        .word   .Lop1b + 1
        .elseif \op == 0x5b
        .word   .Lop5b + 1
        .elseif \op == 0x7b
        .word   .Lop7b + 1
        .elseif \op == 0x3b
        .word   .Lop3b + 1
        .elseif \op == 0x8a
        .word   .Lop8a + 1
        .elseif \op == 0x98
        .word   .Lop98 + 1
        .elseif \op == 0xaa
        .word   .Lopaa + 1
        .elseif \op == 0xa8
        .word   .Lopa8 + 1
        .elseif \op == 0xba
        .word   .Lopba + 1
        .elseif \op == 0x9b
        .word   .Lop9b + 1
        .elseif \op == 0xbb
        .word   .Lopbb + 1
        .elseif \op == 0x1a
        .word   .Lop1a + 1
        .elseif \op == 0x3a
        .word   .Lop3a + 1
        .elseif \op == 0xe8
        .word   .Lope8 + 1
        .elseif \op == 0xca
        .word   .Lopca + 1
        .elseif \op == 0xc8
        .word   .Lopc8 + 1
        .elseif \op == 0x88
        .word   .Lop88 + 1
        .elseif \op == 0xeb
        .word   .Lopeb + 1
        .elseif \op == 0x0a
        .word   .Lop0a + 1
        .elseif \op == 0x4a
        .word   .Lop4a + 1
        .elseif \op == 0x2a
        .word   .Lop2a + 1
        .elseif \op == 0x6a
        .word   .Lop6a + 1
        .elseif \op == 0x10
        .word   .Lop10 + 1
        .elseif \op == 0x30
        .word   .Lop30 + 1
        .elseif \op == 0x50
        .word   .Lop50 + 1
        .elseif \op == 0x70
        .word   .Lop70 + 1
        .elseif \op == 0x80
        .word   .Lop80 + 1
        .elseif \op == 0x90
        .word   .Lop90 + 1
        .elseif \op == 0xb0
        .word   .Lopb0 + 1
        .elseif \op == 0xd0
        .word   .Lopd0 + 1
        .elseif \op == 0xf0
        .word   .Lopf0 + 1
        .elseif \op == 0x09
        .word   .Lop09 + 1
        .elseif \op == 0x29
        .word   .Lop29 + 1
        .elseif \op == 0x49
        .word   .Lop49 + 1
        .elseif \op == 0xa0
        .word   .Lopa0 + 1
        .elseif \op == 0xa2
        .word   .Lopa2 + 1
        .elseif \op == 0xa9
        .word   .Lopa9 + 1
@ --- Stage 3I: immediate ALU family (ADC/SBC/CMP/CPX/CPY/BIT #imm) ---
        .elseif \op == 0x69
        .word   .Lop69 + 1
        .elseif \op == 0x89
        .word   .Lop89 + 1
        .elseif \op == 0xc0
        .word   .Lopc0 + 1
        .elseif \op == 0xc9
        .word   .Lopc9 + 1
        .elseif \op == 0xe0
        .word   .Lope0 + 1
        .elseif \op == 0xe9
        .word   .Lope9 + 1
        .elseif \op == 0xc2
        .word   .Lopc2 + 1
        .elseif \op == 0xe2
        .word   .Lope2 + 1
        .elseif \op == 0x05
        .word   .Lop05 + 1
        .elseif \op == 0x25
        .word   .Lop25 + 1
        .elseif \op == 0x45
        .word   .Lop45 + 1
        .elseif \op == 0x85
        .word   .Lop85 + 1
        .elseif \op == 0xa4
        .word   .Lopa4 + 1
        .elseif \op == 0xa5
        .word   .Lopa5 + 1
        .elseif \op == 0xa6
        .word   .Lopa6 + 1
        .elseif \op == 0xc5
        .word   .Lopc5 + 1
        .elseif \op == 0x64
        .word   .Lop64 + 1
        .elseif \op == 0x84
        .word   .Lop84 + 1
        .elseif \op == 0x86
        .word   .Lop86 + 1
        .elseif \op == 0x65
        .word   .Lop65 + 1
        .elseif \op == 0xe5
        .word   .Lope5 + 1
@ --- Stage 3H Pass 1: dp RMW / TSB / TRB / BIT / CPX / CPY ---
        .elseif \op == 0x06
        .word   .Lop06 + 1
        .elseif \op == 0x26
        .word   .Lop26 + 1
        .elseif \op == 0x46
        .word   .Lop46 + 1
        .elseif \op == 0x66
        .word   .Lop66 + 1
        .elseif \op == 0xe6
        .word   .Lope6 + 1
        .elseif \op == 0xc6
        .word   .Lopc6 + 1
        .elseif \op == 0x04
        .word   .Lop04 + 1
        .elseif \op == 0x14
        .word   .Lop14 + 1
        .elseif \op == 0x24
        .word   .Lop24 + 1
        .elseif \op == 0xe4
        .word   .Lope4 + 1
        .elseif \op == 0xc4
        .word   .Lopc4 + 1
@ --- Stage 3H Pass 2: dp,X (reads/ALU/writes/RMW) ---
        .elseif \op == 0x15
        .word   .Lop15 + 1
        .elseif \op == 0x34
        .word   .Lop34 + 1
        .elseif \op == 0x35
        .word   .Lop35 + 1
        .elseif \op == 0x55
        .word   .Lop55 + 1
        .elseif \op == 0x75
        .word   .Lop75 + 1
        .elseif \op == 0xb4
        .word   .Lopb4 + 1
        .elseif \op == 0xb5
        .word   .Lopb5 + 1
        .elseif \op == 0xd5
        .word   .Lopd5 + 1
        .elseif \op == 0xf5
        .word   .Lopf5 + 1
        .elseif \op == 0x95
        .word   .Lop95 + 1
        .elseif \op == 0x94
        .word   .Lop94 + 1
        .elseif \op == 0x74
        .word   .Lop74 + 1
        .elseif \op == 0x16
        .word   .Lop16 + 1
        .elseif \op == 0x36
        .word   .Lop36 + 1
        .elseif \op == 0x56
        .word   .Lop56 + 1
        .elseif \op == 0x76
        .word   .Lop76 + 1
        .elseif \op == 0xd6
        .word   .Lopd6 + 1
        .elseif \op == 0xf6
        .word   .Lopf6 + 1
        .elseif \op == 0x0d
        .word   .Lop0d + 1
        .elseif \op == 0x2d
        .word   .Lop2d + 1
        .elseif \op == 0x4d
        .word   .Lop4d + 1
        .elseif \op == 0x6d
        .word   .Lop6d + 1
        .elseif \op == 0x8c
        .word   .Lop8c + 1
        .elseif \op == 0x8d
        .word   .Lop8d + 1
        .elseif \op == 0x8e
        .word   .Lop8e + 1
        .elseif \op == 0x9c
        .word   .Lop9c + 1
        .elseif \op == 0xac
        .word   .Lopac + 1
        .elseif \op == 0xad
        .word   .Lopad + 1
        .elseif \op == 0xae
        .word   .Lopae + 1
        .elseif \op == 0xcd
        .word   .Lopcd + 1
        .elseif \op == 0xed
        .word   .Loped + 1
@ --- Stage 3E Pass 2: RMW / BIT / CPX / CPY abs ---
        .elseif \op == 0x0c
        .word   .Lop0c + 1
        .elseif \op == 0x0e
        .word   .Lop0e + 1
        .elseif \op == 0x1c
        .word   .Lop1c + 1
        .elseif \op == 0x2c
        .word   .Lop2c + 1
        .elseif \op == 0x2e
        .word   .Lop2e + 1
        .elseif \op == 0x4e
        .word   .Lop4e + 1
        .elseif \op == 0x6e
        .word   .Lop6e + 1
        .elseif \op == 0xcc
        .word   .Lopcc + 1
        .elseif \op == 0xce
        .word   .Lopce + 1
        .elseif \op == 0xec
        .word   .Lopec + 1
        .elseif \op == 0xee
        .word   .Lopee + 1
@ --- Stage 3F Pass 1: absolute,X / absolute,Y ---
        .elseif \op == 0x19
        .word   .Lop19 + 1
        .elseif \op == 0x1d
        .word   .Lop1d + 1
        .elseif \op == 0x39
        .word   .Lop39 + 1
        .elseif \op == 0x3d
        .word   .Lop3d + 1
        .elseif \op == 0x59
        .word   .Lop59 + 1
        .elseif \op == 0x5d
        .word   .Lop5d + 1
        .elseif \op == 0x79
        .word   .Lop79 + 1
        .elseif \op == 0x7d
        .word   .Lop7d + 1
        .elseif \op == 0x99
        .word   .Lop99 + 1
        .elseif \op == 0x9d
        .word   .Lop9d + 1
        .elseif \op == 0x9e
        .word   .Lop9e + 1
        .elseif \op == 0xb9
        .word   .Lopb9 + 1
        .elseif \op == 0xbc
        .word   .Lopbc + 1
        .elseif \op == 0xbd
        .word   .Lopbd + 1
        .elseif \op == 0xbe
        .word   .Lopbe + 1
        .elseif \op == 0xd9
        .word   .Lopd9 + 1
        .elseif \op == 0xdd
        .word   .Lopdd + 1
        .elseif \op == 0xf9
        .word   .Lopf9 + 1
        .elseif \op == 0xfd
        .word   .Lopfd + 1
@ --- Stage 3F Pass 2: absolute,X RMW / BIT ---
        .elseif \op == 0x1e
        .word   .Lop1e + 1
        .elseif \op == 0x3e
        .word   .Lop3e + 1
        .elseif \op == 0x3c
        .word   .Lop3c + 1
        .elseif \op == 0x5e
        .word   .Lop5e + 1
        .elseif \op == 0x7e
        .word   .Lop7e + 1
        .elseif \op == 0xde
        .word   .Lopde + 1
        .elseif \op == 0xfe
        .word   .Lopfe + 1
@ --- Stage 3G Pass 1: DP-indirect (IDX/IDP/IDY) ---
        .elseif \op == 0x01
        .word   .Lop01 + 1
        .elseif \op == 0x11
        .word   .Lop11 + 1
        .elseif \op == 0x12
        .word   .Lop12 + 1
        .elseif \op == 0x21
        .word   .Lop21 + 1
        .elseif \op == 0x31
        .word   .Lop31 + 1
        .elseif \op == 0x32
        .word   .Lop32 + 1
        .elseif \op == 0x41
        .word   .Lop41 + 1
        .elseif \op == 0x51
        .word   .Lop51 + 1
        .elseif \op == 0x52
        .word   .Lop52 + 1
        .elseif \op == 0x61
        .word   .Lop61 + 1
        .elseif \op == 0x71
        .word   .Lop71 + 1
        .elseif \op == 0x72
        .word   .Lop72 + 1
        .elseif \op == 0x81
        .word   .Lop81 + 1
        .elseif \op == 0x91
        .word   .Lop91 + 1
        .elseif \op == 0x92
        .word   .Lop92 + 1
        .elseif \op == 0xa1
        .word   .Lopa1 + 1
        .elseif \op == 0xb1
        .word   .Lopb1 + 1
        .elseif \op == 0xb2
        .word   .Lopb2 + 1
        .elseif \op == 0xc1
        .word   .Lopc1 + 1
        .elseif \op == 0xd1
        .word   .Lopd1 + 1
        .elseif \op == 0xd2
        .word   .Lopd2 + 1
        .elseif \op == 0xe1
        .word   .Lope1 + 1
        .elseif \op == 0xf1
        .word   .Lopf1 + 1
        .elseif \op == 0xf2
        .word   .Lopf2 + 1
@ --- Stage 3G Pass 2: DP-indirect-long (IDL [dp] / ILY [dp],Y) ---
        .elseif \op == 0x07
        .word   .Lop07 + 1
        .elseif \op == 0x17
        .word   .Lop17 + 1
        .elseif \op == 0x27
        .word   .Lop27 + 1
        .elseif \op == 0x37
        .word   .Lop37 + 1
        .elseif \op == 0x47
        .word   .Lop47 + 1
        .elseif \op == 0x57
        .word   .Lop57 + 1
        .elseif \op == 0x67
        .word   .Lop67 + 1
        .elseif \op == 0x77
        .word   .Lop77 + 1
        .elseif \op == 0x87
        .word   .Lop87 + 1
        .elseif \op == 0x97
        .word   .Lop97 + 1
        .elseif \op == 0xa7
        .word   .Lopa7 + 1
        .elseif \op == 0xb7
        .word   .Lopb7 + 1
        .elseif \op == 0xc7
        .word   .Lopc7 + 1
        .elseif \op == 0xd7
        .word   .Lopd7 + 1
        .elseif \op == 0xe7
        .word   .Lope7 + 1
        .elseif \op == 0xf7
        .word   .Lopf7 + 1
@ --- Stage 3J: absolute-long (abl/alx) + dp,Y (dpy) ---
        .elseif \op == 0x0f
        .word   .Lop0f + 1
        .elseif \op == 0x2f
        .word   .Lop2f + 1
        .elseif \op == 0x4f
        .word   .Lop4f + 1
        .elseif \op == 0xcf
        .word   .Lopcf + 1
        .elseif \op == 0xaf
        .word   .Lopaf + 1
        .elseif \op == 0x8f
        .word   .Lop8f + 1
        .elseif \op == 0x6f
        .word   .Lop6f + 1
        .elseif \op == 0xef
        .word   .Lopef + 1
        .elseif \op == 0x1f
        .word   .Lop1f + 1
        .elseif \op == 0x3f
        .word   .Lop3f + 1
        .elseif \op == 0x5f
        .word   .Lop5f + 1
        .elseif \op == 0xdf
        .word   .Lopdf + 1
        .elseif \op == 0xbf
        .word   .Lopbf + 1
        .elseif \op == 0x9f
        .word   .Lop9f + 1
        .elseif \op == 0x7f
        .word   .Lop7f + 1
        .elseif \op == 0xff
        .word   .Lopff + 1
        .elseif \op == 0x96
        .word   .Lop96 + 1
        .elseif \op == 0xb6
        .word   .Lopb6 + 1
        .elseif \op == 0x08
        .word   .Lop08 + 1
        .elseif \op == 0x0b
        .word   .Lop0b + 1
        .elseif \op == 0x28
        .word   .Lop28 + 1
        .elseif \op == 0x2b
        .word   .Lop2b + 1
        .elseif \op == 0x48
        .word   .Lop48 + 1
        .elseif \op == 0x4b
        .word   .Lop4b + 1
        .elseif \op == 0x5a
        .word   .Lop5a + 1
        .elseif \op == 0x68
        .word   .Lop68 + 1
        .elseif \op == 0x7a
        .word   .Lop7a + 1
        .elseif \op == 0x8b
        .word   .Lop8b + 1
        .elseif \op == 0xab
        .word   .Lopab + 1
        .elseif \op == 0xda
        .word   .Lopda + 1
        .elseif \op == 0xfa
        .word   .Lopfa + 1
@ --- Stage 3L: jump / return / subroutine (Family B) ---
        .elseif \op == 0x4c
        .word   .Lop4c + 1
        .elseif \op == 0x82
        .word   .Lop82 + 1
        .elseif \op == 0x5c
        .word   .Lop5c + 1
        .elseif \op == 0x20
        .word   .Lop20 + 1
        .elseif \op == 0x02
        .word   .Lop02 + 1
        .elseif \op == 0x22
        .word   .Lop22 + 1
        .elseif \op == 0x6c
        .word   .Lop6c + 1
        .elseif \op == 0xdc
        .word   .Lopdc + 1
        .elseif \op == 0x7c
        .word   .Lop7c + 1
        .elseif \op == 0xfc
        .word   .Lopfc + 1
        .elseif \op == 0x60
        .word   .Lop60 + 1
        .elseif \op == 0x6b
        .word   .Lop6b + 1
        .elseif \op == 0x40
        .word   .Lop40 + 1
@ --- Stage 3M: stack-relative (Family D) sr,S + (sr,S),Y ---
        .elseif \op == 0x03
        .word   .Lop03 + 1
        .elseif \op == 0x13
        .word   .Lop13 + 1
        .elseif \op == 0x23
        .word   .Lop23 + 1
        .elseif \op == 0x33
        .word   .Lop33 + 1
        .elseif \op == 0x43
        .word   .Lop43 + 1
        .elseif \op == 0x53
        .word   .Lop53 + 1
        .elseif \op == 0x63
        .word   .Lop63 + 1
        .elseif \op == 0x73
        .word   .Lop73 + 1
        .elseif \op == 0x83
        .word   .Lop83 + 1
        .elseif \op == 0x93
        .word   .Lop93 + 1
        .elseif \op == 0xa3
        .word   .Lopa3 + 1
        .elseif \op == 0xb3
        .word   .Lopb3 + 1
        .elseif \op == 0xc3
        .word   .Lopc3 + 1
        .elseif \op == 0xd3
        .word   .Lopd3 + 1
        .elseif \op == 0xe3
        .word   .Lope3 + 1
        .elseif \op == 0xf3
        .word   .Lopf3 + 1
@ --- Stage 3N: misc native (WDM WAI STP XCE MVP MVN PER PEI PEA) ---
        .elseif \op == 0x42
        .word   .Lop42 + 1
        .elseif \op == 0x44
        .word   .Lop44 + 1
        .elseif \op == 0x54
        .word   .Lop54 + 1
        .elseif \op == 0x62
        .word   .Lop62 + 1
        .elseif \op == 0xcb
        .word   .Lopcb + 1
        .elseif \op == 0xd4
        .word   .Lopd4 + 1
        .elseif \op == 0xdb
        .word   .Lopdb + 1
        .elseif \op == 0xf4
        .word   .Lopf4 + 1
        .elseif \op == 0xfb
        .word   .Lopfb + 1
        .else
        .word   .L_noteb + 1
        .endif
        .endm

@ A named placement section lets the linker collect this engine into ITCM only
@ when SNES_THUMB2_CPU=1 actually supplies the object as a link input.  Do not
@ name the object explicitly in the linker script: that breaks clean flag-off
@ links and can pull a stale flag-on object into an incremental flag-off build.
        .section .itcm_snes_interp.thumb2,"ax",%progbits
        .align  2

        .global snes_thumb2_try
        .type   snes_thumb2_try,%function
        .thumb_func
snes_thumb2_try:
        push    {r4, r5, r6, lr}
        mov     r4, r0                  @ r4 = Cpu* (callee-saved across handlers)
        movs    r5, #0                  @ return contract: try (1/0)
        uxtb    r6, r1
        b       .Ldispatch

@ Full single-opcode entry used by cpu_runOpcode().  Keep the public C pre-work
@ (stop/WAI/IRQ/NMI/RC) outside; this leaf owns exactly one opcode fetch, the
@ 16-bit PC increment, base-cycle charge, and direct/fallback decision.
        .global snes_thumb2_step
        .type   snes_thumb2_step,%function
        .thumb_func
snes_thumb2_step:
        push    {r4, r5, r6, lr}
        mov     r4, r0
        ldr     r0, [r4, #CPU_MEM]
        ldrb    r1, [r4, #CPU_K]
        ldrh    r2, [r4, #CPU_PC]
        lsl     r1, r1, #16
        orr     r1, r1, r2
        adds    r3, r2, #1
        strh    r3, [r4, #CPU_PC]       @ uint16_t wrap matches cpu->pc++
        ROM_FETCH_FAST
        uxtb    r6, r0                  @ preserve opcode across dispatch
        movw    r2, #:lower16:snes_cycles_per_opcode
        movt    r2, #:upper16:snes_cycles_per_opcode
        ldrb    r3, [r2, r6]
        strb    r3, [r4, #CPU_CYCLESUSED]
        movs    r5, #1                  @ return contract: step (-1/opcode)
        b       .Ldispatch

@ Whole-opcode entry: everything cpu_runOpcode() does, in one frame.
@
@ cpu_runOpcode is a wrapper -- five never-taken checks and a BL -- yet the
@ device profile charged it 10.0% of the frame, because "per opcode" is tens of
@ thousands of times per frame and a C frame is not free: its own push/pop of
@ four registers, then immediately the engine's push/pop of the same four, then
@ a BL and a return, to reach a body that was always going to run. This entry
@ keeps the checks (they are semantics, not overhead) and drops the frame: the
@ caller lands directly in the fetch.
@
@ The checks mirror cpu_runOpcode exactly, including that an IRQ the I flag
@ masks is NOT a reason to leave the fast path -- `irqWanted & ~i`, both fields
@ being normalised _Bool. Anything the fast path does not model (stopped, WAI,
@ a taken interrupt, an active rc translation) tail-branches to the C function,
@ which then runs the full original path from the top; nothing is half-done at
@ the branch, because the branch is before the fetch.
@
@ Returns cyclesUsed, like cpu_runOpcode.
        .global snes_thumb2_run
        .type   snes_thumb2_run,%function
        .thumb_func
snes_thumb2_run:
        ldrb    r1, [r0, #CPU_STOPPED]
        ldrb    r2, [r0, #CPU_WAITING]
        orrs    r1, r1, r2
        ldrb    r2, [r0, #CPU_NMIWANTED]
        orrs    r1, r1, r2
        ldrb    r2, [r0, #CPU_IRQWANTED]
        ldrb    r3, [r0, #CPU_I]
        bics    r2, r2, r3              @ irqWanted && !i
        orrs    r1, r1, r2
        bne     .Lrun_slow
        movw    r2, #:lower16:g_rc_active
        movt    r2, #:upper16:g_rc_active
        ldrb    r2, [r2]
        cbnz    r2, .Lrun_slow
        push    {r4, r5, r6, lr}
        mov     r4, r0
        ldr     r0, [r4, #CPU_MEM]
        ldrb    r1, [r4, #CPU_K]
        ldrh    r2, [r4, #CPU_PC]
        lsl     r1, r1, #16
        orr     r1, r1, r2
        adds    r3, r2, #1
        strh    r3, [r4, #CPU_PC]       @ uint16_t wrap matches cpu->pc++
        ROM_FETCH_FAST
        uxtb    r6, r0                  @ preserve opcode across dispatch
        movw    r2, #:lower16:snes_cycles_per_opcode
        movt    r2, #:upper16:snes_cycles_per_opcode
        ldrb    r3, [r2, r6]
        strb    r3, [r4, #CPU_CYCLESUSED]
        movs    r5, #2                  @ return contract: run (cyclesUsed)
        b       .Ldispatch
.Lrun_slow:
        b       cpu_runOpcode           @ tail call: LR is still the caller's

.Ldispatch:
        adr     r2, .Ldispatch_table
        ldr     r3, [r2, r6, lsl #2]
        bx      r3

        .align  2
.Ldispatch_table:
        .set    .Ldispatch_op, 0
        .rept   256
        DISPATCH_ENTRY .Ldispatch_op
        .set    .Ldispatch_op, .Ldispatch_op + 1
        .endr

        @ --- flag operations (no width dependence, no ZN update) ---
        cmp     r1, #0x18               @ CLC
        bne     .L_not18
.Lop18:
        movs    r2, #0
        strb    r2, [r4, #CPU_C]
        b       .L_handled
.L_not18:
        cmp     r1, #0x38               @ SEC
        bne     .L_not38
.Lop38:
        movs    r2, #1
        strb    r2, [r4, #CPU_C]
        b       .L_handled
.L_not38:
        cmp     r1, #0x58               @ CLI
        bne     .L_not58
.Lop58:
        movs    r2, #0
        strb    r2, [r4, #CPU_I]
        b       .L_handled
.L_not58:
        cmp     r1, #0x78               @ SEI
        bne     .L_not78
.Lop78:
        movs    r2, #1
        strb    r2, [r4, #CPU_I]
        b       .L_handled
.L_not78:
        cmp     r1, #0xb8               @ CLV
        bne     .L_notb8
.Lopb8:
        movs    r2, #0
        strb    r2, [r4, #CPU_V]
        b       .L_handled
.L_notb8:
        cmp     r1, #0xd8               @ CLD
        bne     .L_notd8
.Lopd8:
        movs    r2, #0
        strb    r2, [r4, #CPU_D]
        b       .L_handled
.L_notd8:
        cmp     r1, #0xf8               @ SED
        bne     .L_notf8
.Lopf8:
        movs    r2, #1
        strb    r2, [r4, #CPU_D]
        b       .L_handled
.L_notf8:
        cmp     r1, #0xea               @ NOP
        bne     .L_notea
.Lopea:
        b       .L_handled
.L_notea:

        @ --- transfers with no width masking, no ZN (TXS, TCS) ---
        cmp     r1, #0x9a               @ TXS: sp = e ? (x&0xff)|0x100 : x
        bne     .L_not9a
.Lop9a:
        ldrh    r2, [r4, #CPU_X]
        ldrb    r3, [r4, #CPU_E]
        cbz     r3, .Lop9a_store
        and     r2, r2, #0xff
        orr     r2, r2, #0x100
.Lop9a_store:
        strh    r2, [r4, #CPU_SP]
        b       .L_handled
.L_not9a:
        cmp     r1, #0x1b               @ TCS: sp = e ? (a&0xff)|0x100 : a
        bne     .L_not1b
.Lop1b:
        ldrh    r2, [r4, #CPU_A]
        ldrb    r3, [r4, #CPU_E]
        cbz     r3, .Lop1b_store
        and     r2, r2, #0xff
        orr     r2, r2, #0x100
.Lop1b_store:
        strh    r2, [r4, #CPU_SP]
        b       .L_handled
.L_not1b:

        @ --- always-16-bit transfers (TCD, TDC, TSC): setZN(value, false) ---
        cmp     r1, #0x5b               @ TCD: dp = a ; ZN(a, word)
        bne     .L_not5b
.Lop5b:
        ldrh    r2, [r4, #CPU_A]
        strh    r2, [r4, #CPU_DP]
        movs    r1, #0                  @ byte-mode = 0
        SETZN
        b       .L_handled
.L_not5b:
        cmp     r1, #0x7b               @ TDC: a = dp ; ZN(dp, word)
        bne     .L_not7b
.Lop7b:
        ldrh    r2, [r4, #CPU_DP]
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled
.L_not7b:
        cmp     r1, #0x3b               @ TSC: a = sp ; ZN(sp, word)
        bne     .L_not3b
.Lop3b:
        ldrh    r2, [r4, #CPU_SP]
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled
.L_not3b:

        @ --- accumulator <-> A transfers gated on M flag ---
        @ TXA (0x8a): mf ? a=(a&ff00)|(x&ff) : a=x ; ZN(a, mf)
        cmp     r1, #0x8a
        bne     .L_not8a
.Lop8a:
        LDB     r1, CPU_MF             @ r1 = mf (byte-mode for ZN)
        ldrh    r3, [r4, #CPU_A]        @ r3 = current A (need high byte if mf)
        ldrh    r2, [r4, #CPU_X]
        cmp     r1, #0
        bne     .L8a_m
        @ word: a = x
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2                  @ already 16-bit clean for ZN
        b       .L8a_zn
.L8a_m: @ byte: a = (a&ff00) | (x&ff)
        and     r3, r3, #0xff00
        uxtb    r2, r2                  @ x & 0xff
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ ZN on byte
.L8a_zn:
        SETZN
        b       .L_handled
.L_not8a:

        @ TYA (0x98): mf ? a=(a&ff00)|(y&ff) : a=y ; ZN(a, mf)
        cmp     r1, #0x98
        bne     .L_not98
.Lop98:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        ldrh    r2, [r4, #CPU_Y]
        cmp     r1, #0
        bne     .L98_m
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        b       .L98_zn
.L98_m:
        and     r3, r3, #0xff00
        uxtb    r2, r2
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L98_zn:
        SETZN
        b       .L_handled
.L_not98:

        @ --- A -> X/Y transfers gated on X flag ---
        @ TAX (0xaa): xf ? x=a&ff : x=a ; ZN(x, xf)
        cmp     r1, #0xaa
        bne     .L_notaa
.Lopaa:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_A]
        cmp     r1, #0
        bne     .Laa_x
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Laa_zn
.Laa_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Laa_zn:
        SETZN
        b       .L_handled
.L_notaa:

        @ TAY (0xa8): xf ? y=a&ff : y=a ; ZN(y, xf)
        cmp     r1, #0xa8
        bne     .L_nota8
.Lopa8:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_A]
        cmp     r1, #0
        bne     .La8_x
        strh    r2, [r4, #CPU_Y]
        uxth    r2, r2
        b       .La8_zn
.La8_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_Y]
.La8_zn:
        SETZN
        b       .L_handled
.L_nota8:

        @ TSX (0xba): xf ? x=sp&ff : x=sp ; ZN(x, xf)
        cmp     r1, #0xba
        bne     .L_notba
.Lopba:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_SP]
        cmp     r1, #0
        bne     .Lba_x
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Lba_zn
.Lba_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Lba_zn:
        SETZN
        b       .L_handled
.L_notba:

        @ TXY (0x9b): xf ? y=x&ff : y=x ; ZN(y, xf)
        cmp     r1, #0x9b
        bne     .L_not9b
.Lop9b:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_X]
        cmp     r1, #0
        bne     .L9b_x
        strh    r2, [r4, #CPU_Y]
        uxth    r2, r2
        b       .L9b_zn
.L9b_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_Y]
.L9b_zn:
        SETZN
        b       .L_handled
.L_not9b:

        @ TYX (0xbb): xf ? x=y&ff : x=y ; ZN(x, xf)
        cmp     r1, #0xbb
        bne     .L_notbb
.Lopbb:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_Y]
        cmp     r1, #0
        bne     .Lbb_x
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Lbb_zn
.Lbb_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Lbb_zn:
        SETZN
        b       .L_handled
.L_notbb:

        @ --- accumulator inc/dec gated on M flag ---
        @ INA (0x1a): mf ? a=(a&ff00)|((a+1)&ff) : a++ ; ZN(a, mf)
        cmp     r1, #0x1a
        bne     .L_not1a
.Lop1a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        cmp     r1, #0
        bne     .L1a_m
        adds    r2, r3, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        b       .L1a_zn
.L1a_m:
        @ r3 = A. Keep high byte in r3, compute (low+1)&0xff separately.
        mov     r2, r3                  @ r2 = A
        and     r3, r3, #0xff00         @ r3 = high byte
        uxtb    r2, r2                  @ r2 = low byte
        adds    r2, r2, #1
        uxtb    r2, r2
        orr     r2, r3, r2             @ a = high | new_low
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L1a_zn:
        SETZN
        b       .L_handled
.L_not1a:

        @ DEA (0x3a): mf ? a=(a&ff00)|((a-1)&ff) : a-- ; ZN(a, mf)
        cmp     r1, #0x3a
        bne     .L_not3a
.Lop3a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        cmp     r1, #0
        bne     .L3a_m
        subs    r2, r3, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        b       .L3a_zn
.L3a_m:
        @ r3 = A. Keep high byte in r3, compute (low-1)&0xff separately.
        mov     r2, r3                  @ r2 = A
        and     r3, r3, #0xff00         @ r3 = high byte
        uxtb    r2, r2                  @ r2 = low byte
        subs    r2, r2, #1
        uxtb    r2, r2
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L3a_zn:
        SETZN
        b       .L_handled
.L_not3a:

        @ --- index inc/dec gated on X flag ---
        @ INX (0xe8): xf ? x=(x+1)&ff : x++ ; ZN(x, xf)
        cmp     r1, #0xe8
        bne     .L_note8
.Lope8:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_X]
        cmp     r1, #0
        bne     .Le8_x
        adds    r2, r2, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Le8_zn
.Le8_x:
        adds    r2, r2, #1
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Le8_zn:
        SETZN
        b       .L_handled
.L_note8:

        @ DEX (0xca): xf ? x=(x-1)&ff : x-- ; ZN(x, xf)
        cmp     r1, #0xca
        bne     .L_notca
.Lopca:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_X]
        cmp     r1, #0
        bne     .Lca_x
        subs    r2, r2, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Lca_zn
.Lca_x:
        subs    r2, r2, #1
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Lca_zn:
        SETZN
        b       .L_handled
.L_notca:

        @ INY (0xc8): xf ? y=(y+1)&ff : y++ ; ZN(y, xf)
        cmp     r1, #0xc8
        bne     .L_notc8
.Lopc8:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_Y]
        cmp     r1, #0
        bne     .Lc8_x
        adds    r2, r2, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_Y]
        uxth    r2, r2
        b       .Lc8_zn
.Lc8_x:
        adds    r2, r2, #1
        uxtb    r2, r2
        strh    r2, [r4, #CPU_Y]
.Lc8_zn:
        SETZN
        b       .L_handled
.L_notc8:

        @ DEY (0x88): xf ? y=(y-1)&ff : y-- ; ZN(y, xf)
        cmp     r1, #0x88
        bne     .L_not88
.Lop88:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_Y]
        cmp     r1, #0
        bne     .L88_x
        subs    r2, r2, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_Y]
        uxth    r2, r2
        b       .L88_zn
.L88_x:
        subs    r2, r2, #1
        uxtb    r2, r2
        strh    r2, [r4, #CPU_Y]
.L88_zn:
        SETZN
        b       .L_handled
.L_not88:

        @ --- XBA (0xeb): swap A bytes ; ZN(new low = old high, byte) ---
        @ C: low=a&ff; high=a>>8; a=(low<<8)|high; setZN(high, true)
        @    i.e. n/z reflect the NEW low byte (which is the old high byte).
        cmp     r1, #0xeb
        bne     .L_noteb
.Lopeb:
        ldrh    r2, [r4, #CPU_A]       @ r2 = old a, e.g. 0x0000AABB
        rev16   r2, r2                 @ swap bytes in halfword: 0x0000BBAA
        uxth    r2, r2                 @ new a = 0xBBAA (old low promoted to high)
        strh    r2, [r4, #CPU_A]
        @ ZN reflects new low byte = old high byte (0xAA here).
        uxtb    r2, r2                 @ r2 = new low byte
        movs    r1, #1                 @ byte mode
        SETZN
        b       .L_handled

        @ --- accumulator shifts (implied form: no operand, no bus access) ---
        @ Gate on M flag exactly as cpu_doOpcode. r1 holds mf (=ZN byte-mode),
        @ r2 the ZN value, r0/r3 scratch. Carry extracted via ubfx (0/1), so the
        @ stored _Bool matches the C compiler's normalisation.
        @ ASL A (0x0a): mf? c=bit7,a=(a&ff00)|((a<<1)&ff) : c=bit15,a=(a<<1)&ffff
.Lop0a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        cmp     r1, #0
        bne     .L0a_m
        ubfx    r2, r3, #15, #1        @ word carry = bit15
        strb    r2, [r4, #CPU_C]
        lsls    r2, r3, #1
        uxth    r2, r2                 @ a = (a<<1)&0xffff
        strh    r2, [r4, #CPU_A]
        b       .L0a_zn
.L0a_m:
        ubfx    r2, r3, #7, #1         @ byte carry = bit7
        strb    r2, [r4, #CPU_C]
        lsls    r0, r3, #1
        uxtb    r0, r0                 @ (a<<1)&0xff
        and     r3, r3, #0xff00
        orr     r2, r3, r0             @ a = (a&ff00)|newlow
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2                 @ ZN byte value
.L0a_zn:
        SETZN
        b       .L_handled

        @ LSR A (0x4a): c=a&1; mf? a=(a&ff00)|((a>>1)&7f) : a=a>>1
.Lop4a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        ubfx    r2, r3, #0, #1         @ carry = bit0 (same both widths)
        strb    r2, [r4, #CPU_C]
        cmp     r1, #0
        bne     .L4a_m
        lsrs    r2, r3, #1
        strh    r2, [r4, #CPU_A]
        b       .L4a_zn
.L4a_m:
        uxtb    r0, r3                 @ low byte
        lsrs    r0, r0, #1
        and     r3, r3, #0xff00
        orr     r2, r3, r0
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L4a_zn:
        SETZN
        b       .L_handled

        @ ROL A (0x2a): result=(a<<1)|c; mf? c=bit7,a=(a&ff00)|(result&ff)
        @                                   : c=bit15,a=result&ffff.  bit7/15 of
        @ a == bit8/16 of result (c only sets bit0), so carry comes from a.
.Lop2a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        ldrb    r0, [r4, #CPU_C]       @ old carry (shift-in)
        cmp     r1, #0
        bne     .L2a_m
        ubfx    r2, r3, #15, #1        @ word carry = bit15 of a
        strb    r2, [r4, #CPU_C]
        lsls    r2, r3, #1
        orr     r2, r2, r0             @ (a<<1)|c
        uxth    r2, r2                 @ a = result&0xffff
        strh    r2, [r4, #CPU_A]
        b       .L2a_zn
.L2a_m:
        ubfx    r2, r3, #7, #1         @ byte carry = bit7 of a
        strb    r2, [r4, #CPU_C]
        lsls    r2, r3, #1
        orr     r2, r2, r0
        uxtb    r2, r2                 @ result&0xff
        and     r3, r3, #0xff00
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L2a_zn:
        SETZN
        b       .L_handled

        @ ROR A (0x6a): newc=a&1; mf? a=(a&ff00)|((a>>1)&7f)|(c<<7)
        @                                   : a=(a>>1)|(c<<15).  a uses OLD c.
.Lop6a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        ldrb    r0, [r4, #CPU_C]       @ old carry (shift-in)
        ubfx    r2, r3, #0, #1         @ new carry = bit0
        strb    r2, [r4, #CPU_C]       @ store now; a below uses r0, not CPU_C
        cmp     r1, #0
        bne     .L6a_m
        lsrs    r2, r3, #1
        lsls    r0, r0, #15
        orr     r2, r2, r0            @ a = (a>>1)|(c<<15)
        strh    r2, [r4, #CPU_A]
        b       .L6a_zn
.L6a_m:
        uxtb    r2, r3                 @ low byte
        lsrs    r2, r2, #1
        lsls    r0, r0, #7             @ c<<7
        orr     r2, r2, r0
        and     r3, r3, #0xff00
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L6a_zn:
        SETZN
        b       .L_handled

@ --- immediate ALU/load (Stage 3C) ---------------------------------------
@ r6 packing survives each BL snes_cpuRead:
@   bits  7:0  original opcode
@   bit     8  byte mode (MF for A ops, XF for X/Y ops)
@   bits 23:16 low operand byte
@   bits 31:24 high operand byte (word mode only)
.Lop09:                                 @ ORA #imm(m)
.Lop29:                                 @ AND #imm(m)
.Lop49:                                 @ EOR #imm(m)
.Lopa9:                                 @ LDA #imm(m)
        ldrb    r0, [r4, #CPU_MF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Lopa0:                                 @ LDY #imm(x)
.Lopa2:                                 @ LDX #imm(x)
        ldrb    r0, [r4, #CPU_XF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Lop69:                                 @ ADC #imm(m)  -- decimal bail before fetch
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ldrb    r0, [r4, #CPU_MF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Lope9:                                 @ SBC #imm(m)  -- decimal bail before fetch
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ldrb    r0, [r4, #CPU_MF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Lopc9:                                 @ CMP #imm(m)
        ldrb    r0, [r4, #CPU_MF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Lope0:                                 @ CPX #imm(x)
        ldrb    r0, [r4, #CPU_XF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Lopc0:                                 @ CPY #imm(x)
        ldrb    r0, [r4, #CPU_XF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Lop89:                                 @ BIT #imm(m)  -- Z-ONLY quirk
        ldrb    r0, [r4, #CPU_MF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Limm_common:
        IMM_FETCH_BYTE                  @ low operand
        orr     r6, r6, r0, lsl #16
        tst     r6, #0x100
        bne     .Limm_value_ready
        IMM_FETCH_BYTE                  @ high operand; rebuild K:PC after wrap
        orr     r6, r6, r0, lsl #24
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1              @ 16-bit M/X: one dynamic extra cycle
        strb    r0, [r4, #CPU_CYCLESUSED]

.Limm_value_ready:
        ubfx    r2, r6, #16, #8
        ubfx    r3, r6, #24, #8
        orr     r2, r2, r3, lsl #8      @ operand value (high is zero in byte mode)
        uxtb    r0, r6                   @ original opcode
        ubfx    r1, r6, #8, #1          @ SETZN byte-mode flag

        cmp     r0, #0x29               @ AND is the measured hot ALU case
        beq     .Limm_and
        cmp     r0, #0xa9
        beq     .Limm_lda
        cmp     r0, #0xa0
        beq     .Limm_ldy
        cmp     r0, #0xa2
        beq     .Limm_ldx
        cmp     r0, #0x09
        beq     .Limm_ora
        cmp     r0, #0x69
        beq     .Limm_adc
        cmp     r0, #0xe9
        beq     .Limm_sbc
        cmp     r0, #0xc9
        beq     .Limm_cmp
        cmp     r0, #0xe0
        beq     .Limm_cpx
        cmp     r0, #0xc0
        beq     .Limm_cpy
        cmp     r0, #0x89
        beq     .Limm_bit
        @ only remaining shared entry is EOR # (0x49)
        ldrh    r3, [r4, #CPU_A]
        eor     r0, r3, r2
        b       .Limm_store_a

.Limm_and:
        ldrh    r3, [r4, #CPU_A]
        and     r0, r3, r2
        b       .Limm_store_a
.Limm_ora:
        ldrh    r3, [r4, #CPU_A]
        orr     r0, r3, r2
        b       .Limm_store_a
.Limm_lda:
        mov     r0, r2

.Limm_store_a:
        cmp     r1, #0
        beq     .Limm_store_a_word
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r0, #0, #8          @ byte A ops preserve A[15:8]
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r0                  @ SETZN requires width-masked value
        SETZN
        b       .L_handled
.Limm_store_a_word:
        uxth    r2, r0
        strh    r2, [r4, #CPU_A]
        SETZN
        b       .L_handled

.Limm_ldx:
        strh    r2, [r4, #CPU_X]        @ byte mode zero-extends; no high preserve
        SETZN
        b       .L_handled
.Limm_ldy:
        strh    r2, [r4, #CPU_Y]
        SETZN
        b       .L_handled

@ --- immediate ALU family (Stage 3I): CMP/CPX/CPY/ADC/SBC/BIT #imm ---
@ Mirrors the dp templates (.Lopc5/.Lope4/.Lopc4/.Lop65/.Lope5) but r2=value
@ and r1=byte-mode are already extracted by .Limm_common.  r6 is dead here.
.Limm_cmp:
        cbz     r1, .Limm_cmp_w
        @ byte: result = (a&0xff) + (value^0xff) + 1
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3
        mvn     r2, r2
        uxtb    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8              @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Limm_cmp_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16             @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Limm_cpx:
        cbz     r1, .Limm_cpx_w
        ldrh    r3, [r4, #CPU_X]
        uxtb    r3, r3
        mvn     r2, r2
        uxtb    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Limm_cpx_w:
        ldrh    r3, [r4, #CPU_X]
        mvn     r2, r2
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Limm_cpy:
        cbz     r1, .Limm_cpy_w
        ldrh    r3, [r4, #CPU_Y]
        uxtb    r3, r3
        mvn     r2, r2
        uxtb    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Limm_cpy_w:
        ldrh    r3, [r4, #CPU_Y]
        mvn     r2, r2
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Limm_adc:
        cbz     r1, .Limm_adc_w
        @ byte: r2=value; result=(a&0xff)+value+c; V=((value^result)&~(a^value))>>7
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                  @ preserve value for V XOR
        uxtb    r0, r3                  @ a & 0xff
        add     r2, r0, r6              @ (a&0xff) + value
        add     r2, r2, r1              @ + c
        lsr     r1, r2, #8              @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6              @ a ^ value
        eor     r1, r6, r2              @ value ^ result
        bic     r0, r1, r0              @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1          @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8          @ a = (a&0xff00)|(result&0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Limm_adc_w:
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                  @ preserve value
        add     r2, r3, r0              @ a + value
        add     r2, r2, r1              @ + c
        lsr     r1, r2, #16             @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0              @ a ^ value
        eor     r1, r0, r2              @ value ^ result
        bic     r0, r1, r3              @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1         @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Limm_sbc:
        cbz     r1, .Limm_sbc_w
        @ byte: complement value first
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                  @ preserve complemented value
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Limm_sbc_w:
        @ word: complement value first
        mvn     r2, r2
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Limm_bit:
        cbz     r1, .Limm_bit_w
        @ byte: z = ((a&0xff) & value) == 0
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3
        tst     r3, r2
        b       .Limm_bit_z
.Limm_bit_w:
        @ word: z = (a & value) == 0
        ldrh    r3, [r4, #CPU_A]
        tst     r3, r2
.Limm_bit_z:
        mov     r0, #0
        it      eq
        moveq   r0, #1                  @ z=1 if AND result was zero
        strb    r0, [r4, #CPU_Z]
        b       .L_handled

@ REP/SEP update only mask-selected status fields, then reproduce every
@ cpu_setFlags side effect.  Shifting the mask exposes old bit0 in APSR.C;
@ field order is P = N V M X D I Z C.
.Lopc2:                                 @ REP #imm: selected bits become 0
        movs    r6, #0
        b       .Lflags_imm
.Lope2:                                 @ SEP #imm: selected bits become 1
        movs    r6, #1
.Lflags_imm:
        IMM_FETCH_BYTE
        mov     r2, r6                  @ store value; survives no further BL
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_C]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_Z]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_I]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_D]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_XF]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_MF]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_V]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_N]

        ldrb    r0, [r4, #CPU_E]
        cbz     r0, .Lflags_xf
        movs    r0, #1
        strb    r0, [r4, #CPU_MF]       @ emulation mode pins M=X=1
        strb    r0, [r4, #CPU_XF]
        ldrh    r1, [r4, #CPU_SP]
        uxtb    r1, r1
        add.w   r1, r1, #0x100
        strh    r1, [r4, #CPU_SP]
.Lflags_xf:
        ldrb    r0, [r4, #CPU_XF]
        cmp     r0, #0
        beq     .L_handled
        ldrh    r1, [r4, #CPU_X]
        uxtb    r1, r1
        strh    r1, [r4, #CPU_X]
        ldrh    r1, [r4, #CPU_Y]
        uxtb    r1, r1
        strh    r1, [r4, #CPU_Y]
        b       .L_handled

@ --- relative branches (Stage 3B): one operand byte at pc, taken-cycle gate ---
@ r6 = 0 (not taken) / 1 (taken, +1 cycle) / 2 (BRA, taken, no cycle).
@ r6 is callee-saved (pushed in prologue), so it survives bl snes_cpuRead.
.Lop10:                                 @ BPL  taken if n==0
        ldrb    r6, [r4, #CPU_N]
        eors    r6, r6, #1
        b       .Lbr_common
.Lop50:                                 @ BVC  taken if v==0
        ldrb    r6, [r4, #CPU_V]
        eors    r6, r6, #1
        b       .Lbr_common
.Lop90:                                 @ BCC  taken if c==0
        ldrb    r6, [r4, #CPU_C]
        eors    r6, r6, #1
        b       .Lbr_common
.Lopd0:                                 @ BNE  taken if z==0
        ldrb    r6, [r4, #CPU_Z]
        eors    r6, r6, #1
        b       .Lbr_common
.Lop30:                                 @ BMI  taken if n==1
        ldrb    r6, [r4, #CPU_N]
        b       .Lbr_common
.Lop70:                                 @ BVS  taken if v==1
        ldrb    r6, [r4, #CPU_V]
        b       .Lbr_common
.Lopb0:                                 @ BCS  taken if c==1
        ldrb    r6, [r4, #CPU_C]
        b       .Lbr_common
.Lopf0:                                 @ BEQ  taken if z==1
        ldrb    r6, [r4, #CPU_Z]
        b       .Lbr_common
.Lop80:                                 @ BRA  always, no taken-cycle
        movs    r6, #2
        b       .Lbr_common

 .Lbr_common:
        @ raw operand fetch -- mirrors the step opcode fetch (lines 155-162).
        ldr     r0, [r4, #CPU_MEM]
        ldrb    r1, [r4, #CPU_K]
        ldrh    r2, [r4, #CPU_PC]
        lsl     r1, r1, #16
        orr     r1, r1, r2
        adds    r3, r2, #1
        strh    r3, [r4, #CPU_PC]       @ pc++ (uint16 wrap)
        ROM_FETCH_FAST                   @ r0 = operand byte (cached)
        sxtb    r0, r0                  @ (int8_t)operand
        cbz     r6, .L_handled          @ not taken: operand consumed, done
        ldrh    r2, [r4, #CPU_PC]       @ pc (already +1)
        adds    r2, r2, r0              @ pc += offset (uint16 wrap via strh)
        strh    r2, [r4, #CPU_PC]
        cmp     r6, #2
        beq     .L_handled              @ BRA: no taken-cycle
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        b       .L_handled

.L_noteb:
        @ --- unsupported: run and batch finish the byte in C (.L_noteb_c, at
        @ the end of the engine so that this block stays the size it was and no
        @ handler after it moves); try returns 0; step returns the fetched
        @ opcode for C to finish.  Reachable at runtime, not just for the one
        @ opcode with no table entry: decimal-mode ADC/SBC bail here before
        @ touching any operand. ---
        cmp     r5, #2
        blo     .L_noteb_legacy
        b.w     .L_noteb_c
.L_noteb_legacy:
        cmp     r5, #0
        ite     eq
        moveq   r0, #0
        movne   r0, r6
        pop     {r4, r5, r6, pc}

.L_handled:
        @ batch continues its loop; run returns cyclesUsed; try returns 1; step
        @ returns -1.  Batch and run are tested first: the shipping paths, and
        @ this is the tail of every opcode.
        cmp     r5, #3
        beq     .Lb_after
        cmp     r5, #2
        bne     .L_handled_legacy
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        pop     {r4, r5, r6, pc}
.L_handled_legacy:
        cmp     r5, #0
        ite     eq
        moveq   r0, #1
        movne   r0, #-1
        pop     {r4, r5, r6, pc}




@ ===========================================================================
@ Stage 3D -- direct-page opcodes (bank 0x00, 16-bit wrap, dp&0xff cycle).
@ Read/load/ALU opcodes inline the data read via the cpu_thumb2_read bridge
@ (spin-hook path); store opcodes inline the write via cpu_thumb2_write (gate 1:
@ every write takes the hooked path).  ADC/SBC (0x65/0xe5) inline only the
@ BINARY path; the decimal (D-flag) path bails to .L_noteb before any operand
@ fetch so C handles it -- operand fetch happens in the C fallback, pc stays
@ correct.  V flag uses the identity  V = ((~(a^value)) & (value^result)) & sign
@ which is provably equal to the C form (a&sign)==(value&sign) && sign differs.
@ ===========================================================================

.Lopa5:                                 @ LDA dp
        DP_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_dp_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_dp_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop25:                                 @ AND dp
        DP_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_dp_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (already <= 0xff: value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a & value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2                  @ SETZN value = (a & value) & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_dp_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (word)
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop05:                                 @ ORA dp
        DP_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_dp_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2              @ a | value (low byte; value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a | value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_dp_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop45:                                 @ EOR dp
        DP_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_dp_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2              @ a ^ value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a ^ value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_dp_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopc5:                                 @ CMP dp  (sets C/Z/N; A unchanged)
        DP_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_dp_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff (result>>8 is 0/1)
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_dp_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff (result>>16 is 0/1)
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopa6:                                 @ LDX dp  (XF width)
        DP_ADDR
        DP_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lldx_dp_w
        strh    r2, [r4, #CPU_X]        @ byte zero-extends into x
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lldx_dp_w:
        strh    r2, [r4, #CPU_X]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopa4:                                 @ LDY dp  (XF width)
        DP_ADDR
        DP_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lldy_dp_w
        strh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lldy_dp_w:
        strh    r2, [r4, #CPU_Y]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop85:                                 @ STA dp  (MF width; writes via bridge)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_dp_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_dp_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1                  @ addr_high
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop86:                                 @ STX dp  (XF width; writes via bridge)
        DP_ADDR
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lstx_dp_w
        ldrh    r2, [r4, #CPU_X]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lstx_dp_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_X]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1
        ldrh    r2, [r4, #CPU_X]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop84:                                 @ STY dp  (XF width; writes via bridge)
        DP_ADDR
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lsty_dp_w
        ldrh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lsty_dp_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1
        ldrh    r2, [r4, #CPU_Y]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop64:                                 @ STZ dp  (MF width; stores zero)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lstz_dp_w
        movs    r2, #0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lstz_dp_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        movs    r2, #0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1
        movs    r2, #0
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

@ --- ADC dp (0x65) / SBC dp (0xe5): binary path only.  Decimal mode (D flag
@ set) bails to .L_noteb BEFORE the operand fetch, so C's cpu_adc/cpu_sbc
@ re-dispatch and fetch the operand themselves (pc is still post-opcode).  The
@ binary maths are identical for ADC and SBC once SBC's operand is one's-
@ complemented (value ^= 0xff/0xffff); only the complement step differs, so the
@ two handlers share the same ALU shape.  V flag identity (proven equal to the
@ C form):  V = ((~(a^value)) & (value^result)) & sign_bit.
.Lop65:                                 @ ADC dp  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        DP_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_dp_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (16-bit; high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 of low byte is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_dp_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Lope5:                                 @ SBC dp  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        DP_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_dp_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_dp_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled


@ ===========================================================================
@ Stage 3H Pass 1 -- direct-page RMW / TSB / TRB / BIT / CPX / CPY (11 opcodes).
@ Each is the VERBATIM abs RMW/TSB/TRB/BIT/CPX/CPY handler with ABS_ADDR replaced
@ by DP_ADDR: the ALU maths, cyclesUsed+=2 (word), reversed word write-back, and
@ SETZN tails are byte-identical because cpu_adrDp feeds the SAME cpu_asl/cpu_tsb/
@ ... helpers as cpu_adrAbs.  The ONLY new semantics are in DP_ADDR itself: bank-0
@ addressing (not db) and the 16-bit high-wrap (uxth, NOT the abs 24-bit ubfx
@ #0,#24).  Word-mode high address uses (addr_low+1)&0xffff -- DP_READ_VALUE and
@ the existing DP STA already do this; the RMW inline paths are converted to match.
@ ===========================================================================

.Lop06:                                 @ ASL dp  (RMW; reversed word write-back)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lasl_dp_w
        @ --- byte: result = value << 1; c = result & 0x100 ---
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value (0..0xff)
        lsl     r2, r2, #1              @ result = value << 1 (0..0x1fe)
        lsr     r3, r2, #8              @ c = bit8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ result & 0xff
        push    {r2}                    @ stash for setZN (write clobbers r2)
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Lasl_dp_w:
        @ --- word: cyclesUsed += 2; read word; result = value << 1; c = bit16 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read         @ r0 = low byte
        push    {r0}                    @ stash low byte across high read
        add     r1, r6, #1
        uxth    r1, r1                  @ addr_high = (addr_low+1) & 0xffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = high byte
        pop     {r3}                    @ r3 = low byte
        orr     r2, r3, r0, lsl #8      @ word value
        lsl     r2, r2, #1              @ result = value << 1 (0..0x1fffe)
        lsr     r3, r2, #16             @ c = bit16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ result & 0xffff
        push    {r2}                    @ stash result for low write + setZN
        lsr     r2, r2, #8              @ high byte
        add     r1, r6, #1
        uxth    r1, r1                  @ addr_high
        mov     r0, r4
        bl      cpu_thumb2_write        @ write HIGH first (reversed)
        ldr     r2, [sp]                @ reload result (still pushed)
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write        @ write LOW second
        pop     {r2}                    @ r2 = result (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop46:                                 @ LSR dp  (RMW; reversed word write-back)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llsr_dp_w
        @ --- byte: c = value & 1 (capture first); result = value >> 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ubfx    r3, r2, #0, #1          @ c = value & 1
        strb    r3, [r4, #CPU_C]
        lsr     r2, r2, #1              @ result = value >> 1
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Llsr_dp_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ubfx    r3, r2, #0, #1          @ c = value & 1
        strb    r3, [r4, #CPU_C]
        lsr     r2, r2, #1              @ result = value >> 1
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop26:                                 @ ROL dp  (RMW; reversed word write-back)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lrol_dp_w
        @ --- byte: result = (value<<1) | c; new c = result & 0x100 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ldrb    r3, [r4, #CPU_C]        @ old c (shifted-in bit)
        lsl     r2, r2, #1
        orr     r2, r2, r3              @ result = (value<<1) | c (0..0x1ff)
        lsr     r3, r2, #8              @ c = bit8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ result & 0xff
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Lrol_dp_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ldrb    r3, [r4, #CPU_C]        @ old c
        lsl     r2, r2, #1
        orr     r2, r2, r3              @ result = (value<<1) | c (0..0x1ffff)
        lsr     r3, r2, #16             @ c = bit16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop66:                                 @ ROR dp  (RMW; reversed word write-back)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lror_dp_w
        @ --- byte: carry = value & 1; result = (value>>1)|(c<<7); c set LAST ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ubfx    r3, r2, #0, #1          @ carry = value & 1 (new c)
        push    {r3}                    @ stash carry across write+setZN
        ldrb    r1, [r4, #CPU_C]        @ old c (shifted-in bit)
        lsr     r2, r2, #1
        lsl     r1, r1, #7              @ c << 7
        orr     r2, r2, r1             @ result = (value>>1) | (c<<7)
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}                    @ result byte
        movs    r1, #1
        SETZN
        pop     {r3}                    @ carry
        strb    r3, [r4, #CPU_C]        @ c = carry (set after setZN, per cpu.c)
        b       .L_handled
.Lror_dp_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ubfx    r3, r2, #0, #1          @ carry = value & 1
        push    {r3}                    @ stash carry
        ldrb    r1, [r4, #CPU_C]
        lsr     r2, r2, #1
        lsl     r1, r1, #15             @ c << 15
        orr     r2, r2, r1             @ result = (value>>1) | (c<<15)
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]                @ reload result (top of stack)
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}                    @ result
        movs    r1, #0
        SETZN
        pop     {r3}                    @ carry
        strb    r3, [r4, #CPU_C]
        b       .L_handled

.Lope6:                                 @ INC dp  (RMW; reversed word write-back; no carry change)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Linc_dp_w
        @ --- byte: result = value + 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0
        adds    r2, r2, #1              @ result (0..0x100)
        uxtb    r2, r2
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Linc_dp_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8
        adds    r2, r2, #1              @ result (0..0x10000)
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopc6:                                 @ DEC dp  (RMW; reversed word write-back; no carry change)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ldec_dp_w
        @ --- byte: result = value - 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0
        subs    r2, r2, #1              @ result (-1..0xfe); uxtb wraps 0->0xff
        uxtb    r2, r2
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Ldec_dp_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8
        subs    r2, r2, #1
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop04:                                 @ TSB dp  (RMW; sets Z only, N/V untouched; reversed word write-back)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ltsb_dp_w
        @ --- byte: z = ((a&0xff)&value)==0; write value | (a&0xff) ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        tst     r3, r2                  @ (a&0xff) & value
        ite     eq
        moveq   r0, #1                  @ z = 1 if zero
        movne   r0, #0
        strb    r0, [r4, #CPU_Z]
        orr     r2, r2, r3             @ value | (a&0xff)
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Ltsb_dp_w:
        @ --- word: cyclesUsed += 2; z = (a&value)==0; write value | a ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ldrh    r3, [r4, #CPU_A]        @ a (full word)
        tst     r3, r2                  @ a & value
        ite     eq
        moveq   r0, #1
        movne   r0, #0
        strb    r0, [r4, #CPU_Z]
        orr     r2, r2, r3             @ value | a
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        b       .L_handled

.Lop14:                                 @ TRB dp  (RMW; sets Z only, N/V untouched; reversed word write-back)
        DP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ltrb_dp_w
        @ --- byte: z = ((a&0xff)&value)==0; write value & ~(a&0xff) ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        tst     r3, r2                  @ (a&0xff) & value
        ite     eq
        moveq   r0, #1
        movne   r0, #0
        strb    r0, [r4, #CPU_Z]
        bic     r2, r2, r3             @ value & ~(a&0xff)
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Ltrb_dp_w:
        @ --- word: cyclesUsed += 2; z = (a&value)==0; write value & ~a ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ldrh    r3, [r4, #CPU_A]        @ a
        tst     r3, r2                  @ a & value
        ite     eq
        moveq   r0, #1
        movne   r0, #0
        strb    r0, [r4, #CPU_Z]
        bic     r2, r2, r3             @ value & ~a
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        b       .L_handled

.Lop24:                                 @ BIT dp  (read-only; N/V from memory value, Z from AND)
        DP_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lbit_dp_w
        @ --- byte: r2 = value; z=((a&0xff)&value)==0; n=value&0x80; v=value&0x40 ---
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        tst     r3, r2                  @ (a&0xff) & value
        ite     eq
        moveq   r3, #1                  @ z = 1 if zero
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        ubfx    r3, r2, #7, #1          @ n = value & 0x80 -> bit7
        strb    r3, [r4, #CPU_N]
        ubfx    r3, r2, #6, #1          @ v = value & 0x40 -> bit6
        strb    r3, [r4, #CPU_V]
        b       .L_handled
.Lbit_dp_w:
        @ --- word: r2 = value; z=(a&value)==0; n=value&0x8000; v=value&0x4000 ---
        ldrh    r3, [r4, #CPU_A]
        tst     r3, r2                  @ a & value
        ite     eq
        moveq   r3, #1
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        ubfx    r3, r2, #15, #1         @ n = value & 0x8000 -> bit15
        strb    r3, [r4, #CPU_N]
        ubfx    r3, r2, #14, #1         @ v = value & 0x4000 -> bit14
        strb    r3, [r4, #CPU_V]
        b       .L_handled

@ --- 0x34 BIT dp,X: VERBATIM .Lop24 (BIT dp) with DP_ADDR -> DPX_ADDR.
@ cpu_bit (cpu.c, fed by cpu_adrDpx == cpu_adrDp + X) is the same helper for
@ both, so the flag extraction (N/V from the raw memory value, Z from A&value)
@ is byte-identical -- only the effective-address computation differs. ---
.Lop34:                                 @ BIT dp,X  (read-only; N/V from memory value, Z from AND)
        DPX_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lbit_dpx_w
        @ --- byte: r2 = value; z=((a&0xff)&value)==0; n=value&0x80; v=value&0x40 ---
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        tst     r3, r2                  @ (a&0xff) & value
        ite     eq
        moveq   r3, #1                  @ z = 1 if zero
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        ubfx    r3, r2, #7, #1          @ n = value & 0x80 -> bit7
        strb    r3, [r4, #CPU_N]
        ubfx    r3, r2, #6, #1          @ v = value & 0x40 -> bit6
        strb    r3, [r4, #CPU_V]
        b       .L_handled
.Lbit_dpx_w:
        @ --- word: r2 = value; z=(a&value)==0; n=value&0x8000; v=value&0x4000 ---
        ldrh    r3, [r4, #CPU_A]
        tst     r3, r2                  @ a & value
        ite     eq
        moveq   r3, #1
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        ubfx    r3, r2, #15, #1         @ n = value & 0x8000 -> bit15
        strb    r3, [r4, #CPU_N]
        ubfx    r3, r2, #14, #1         @ v = value & 0x4000 -> bit14
        strb    r3, [r4, #CPU_V]
        b       .L_handled

.Lope4:                                 @ CPX dp  (XF width; compare X; sets C/Z/N; X unchanged)
        DP_ADDR
        DP_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lcpx_dp_w
        @ --- byte (xf=1): r2 = value; result = (x&0xff)+(value^0xff)+1 ---
        mvn     r2, r2                  @ value ^ 0xff
        uxtb    r2, r2
        ldrh    r3, [r4, #CPU_X]
        uxtb    r3, r3                  @ x & 0xff
        add     r2, r3, r2             @ (x&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcpx_dp_w:
        @ --- word (xf=0): r2 = value; result = x+(value^0xffff)+1 ---
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_X]
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopc4:                                 @ CPY dp  (XF width; compare Y; sets C/Z/N; Y unchanged)
        DP_ADDR
        DP_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lcpy_dp_w
        @ --- byte (xf=1): r2 = value; result = (y&0xff)+(value^0xff)+1 ---
        mvn     r2, r2
        uxtb    r2, r2
        ldrh    r3, [r4, #CPU_Y]
        uxtb    r3, r3                  @ y & 0xff
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcpy_dp_w:
        @ --- word (xf=0): r2 = value; result = y+(value^0xffff)+1 ---
        mvn     r2, r2
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_Y]
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled


@ ===========================================================================
@ Stage 3H Pass 2 -- direct-page,X (17 opcodes).  READS/ALU/WRITES are the
@ VERBATIM dp handler with DP_ADDR replaced by DPX_ADDR: the ALU maths, SETZN,
@ and write tails are byte-identical because cpu_adrDpx feeds the SAME
@ cpu_lda/cpu_and/... helpers as cpu_adrDp.  The 6 RMW members mirror the abs
@ RMW handlers (ASL LSR ROL ROR INC DEC) with ABS_ADDR->DPX_ADDR and the 16-bit
@ high-wrap (uxth).  ADC/SBC (0x75/0xf5) inline only BINARY; decimal D-flag
@ bails to .L_noteb BEFORE any operand fetch, exactly as the DP ADC/SBC handlers.
@ ===========================================================================

.Lop15:                                 @ ORA dp,X
        DPX_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_dpx_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2              @ a | value (low byte; value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a | value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_dpx_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop35:                                 @ AND dp,X
        DPX_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_dpx_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (already <= 0xff: value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a & value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2                  @ SETZN value = (a & value) & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_dpx_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (word)
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop55:                                 @ EOR dp,X
        DPX_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_dpx_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2              @ a ^ value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a ^ value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_dpx_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopd5:                                 @ CMP dp,X  (sets C/Z/N; A unchanged)
        DPX_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_dpx_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff (result>>8 is 0/1)
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_dpx_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff (result>>16 is 0/1)
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopb5:                                 @ LDA dp,X
        DPX_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_dpx_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_dpx_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lopb4:                                 @ LDY dp,X  (XF width)
        DPX_ADDR
        DP_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lldy_dpx_w
        strh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lldy_dpx_w:
        strh    r2, [r4, #CPU_Y]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop75:                                 @ ADC dp,X  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        DPX_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_dpx_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (16-bit; high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 of low byte is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_dpx_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Lopf5:                                 @ SBC dp,X  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        DPX_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_dpx_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_dpx_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop95:                                 @ STA dp,X  (MF width; writes via bridge)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_dpx_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_dpx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1                  @ addr_high
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop94:                                 @ STY dp,X  (XF width; writes via bridge)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lsty_dpx_w
        ldrh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lsty_dpx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1
        ldrh    r2, [r4, #CPU_Y]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop74:                                 @ STZ dp,X  (MF width; stores zero)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lstz_dpx_w
        movs    r2, #0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lstz_dpx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        movs    r2, #0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1
        movs    r2, #0
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop16:                                 @ ASL dp,X  (RMW; reversed word write-back)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lasl_dpx_w
        @ --- byte: result = value << 1; c = result & 0x100 ---
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value (0..0xff)
        lsl     r2, r2, #1              @ result = value << 1 (0..0x1fe)
        lsr     r3, r2, #8              @ c = bit8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ result & 0xff
        push    {r2}                    @ stash for setZN (write clobbers r2)
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Lasl_dpx_w:
        @ --- word: cyclesUsed += 2; read word; result = value << 1; c = bit16 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read         @ r0 = low byte
        push    {r0}                    @ stash low byte across high read
        add     r1, r6, #1
        uxth    r1, r1                  @ addr_high = (addr_low+1) & 0xffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = high byte
        pop     {r3}                    @ r3 = low byte
        orr     r2, r3, r0, lsl #8      @ word value
        lsl     r2, r2, #1              @ result = value << 1 (0..0x1fffe)
        lsr     r3, r2, #16             @ c = bit16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ result & 0xffff
        push    {r2}                    @ stash result for low write + setZN
        lsr     r2, r2, #8              @ high byte
        add     r1, r6, #1
        uxth    r1, r1                  @ addr_high
        mov     r0, r4
        bl      cpu_thumb2_write        @ write HIGH first (reversed)
        ldr     r2, [sp]                @ reload result (still pushed)
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write        @ write LOW second
        pop     {r2}                    @ r2 = result (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop36:                                 @ ROL dp,X  (RMW; reversed word write-back)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lrol_dpx_w
        @ --- byte: result = (value<<1) | c; new c = result & 0x100 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ldrb    r3, [r4, #CPU_C]        @ old c (shifted-in bit)
        lsl     r2, r2, #1
        orr     r2, r2, r3              @ result = (value<<1) | c (0..0x1ff)
        lsr     r3, r2, #8              @ c = bit8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ result & 0xff
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Lrol_dpx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ldrb    r3, [r4, #CPU_C]        @ old c
        lsl     r2, r2, #1
        orr     r2, r2, r3              @ result = (value<<1) | c (0..0x1ffff)
        lsr     r3, r2, #16             @ c = bit16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop56:                                 @ LSR dp,X  (RMW; reversed word write-back)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llsr_dpx_w
        @ --- byte: c = value & 1 (capture first); result = value >> 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ubfx    r3, r2, #0, #1          @ c = value & 1
        strb    r3, [r4, #CPU_C]
        lsr     r2, r2, #1              @ result = value >> 1
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Llsr_dpx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ubfx    r3, r2, #0, #1          @ c = value & 1
        strb    r3, [r4, #CPU_C]
        lsr     r2, r2, #1              @ result = value >> 1
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop76:                                 @ ROR dp,X  (RMW; reversed word write-back)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lror_dpx_w
        @ --- byte: carry = value & 1; result = (value>>1)|(c<<7); c set LAST ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ubfx    r3, r2, #0, #1          @ carry = value & 1 (new c)
        push    {r3}                    @ stash carry across write+setZN
        ldrb    r1, [r4, #CPU_C]        @ old c (shifted-in bit)
        lsr     r2, r2, #1
        lsl     r1, r1, #7              @ c << 7
        orr     r2, r2, r1             @ result = (value>>1) | (c<<7)
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}                    @ result byte
        movs    r1, #1
        SETZN
        pop     {r3}                    @ carry
        strb    r3, [r4, #CPU_C]        @ c = carry (set after setZN, per cpu.c)
        b       .L_handled
.Lror_dpx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ubfx    r3, r2, #0, #1          @ carry = value & 1
        push    {r3}                    @ stash carry
        ldrb    r1, [r4, #CPU_C]
        lsr     r2, r2, #1
        lsl     r1, r1, #15             @ c << 15
        orr     r2, r2, r1             @ result = (value>>1) | (c<<15)
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]                @ reload result (top of stack)
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}                    @ result
        movs    r1, #0
        SETZN
        pop     {r3}                    @ carry
        strb    r3, [r4, #CPU_C]
        b       .L_handled

.Lopd6:                                 @ DEC dp,X  (RMW; reversed word write-back; no carry change)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ldec_dpx_w
        @ --- byte: result = value - 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0
        subs    r2, r2, #1              @ result (-1..0xfe); uxtb wraps 0->0xff
        uxtb    r2, r2
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Ldec_dpx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8
        subs    r2, r2, #1
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopf6:                                 @ INC dp,X  (RMW; reversed word write-back; no carry change)
        DPX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Linc_dpx_w
        @ --- byte: result = value + 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0
        adds    r2, r2, #1              @ result (0..0x100)
        uxtb    r2, r2
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Linc_dpx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8
        adds    r2, r2, #1              @ result (0..0x10000)
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        uxth    r1, r1
        mov     r0, r4
        bl      cpu_thumb2_write
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled


@ ===========================================================================
@ Stage 3E -- absolute opcodes (data bank db, 24-bit effective address, 24-bit
@ high-wrap, NO address-computation cycle).  The ALU tails are byte-identical to
@ the Stage 3D direct-page handlers: cpu_adrAbs feeds the SAME cpu_lda/cpu_and/
@ ... helpers as cpu_adrDp, so only the address source differs.  Each handler =
@ ABS_ADDR + (ABS_READ_VALUE | inline write) + the verbatim DP ALU tail.  Word-
@ mode store/RMW high address uses 24-bit wrap (ubfx #0,#24), NOT DP's uxth.
@ ADC/SBC (0x6d/0xed) inline only BINARY; decimal D-flag bails to .L_noteb
@ BEFORE any operand fetch, exactly as the DP ADC/SBC handlers do.
@ ===========================================================================

.Lopad:                                 @ LDA abs
        ABS_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_abs_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_abs_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop2d:                                 @ AND abs
        ABS_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_abs_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a & value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_abs_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (word)
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop0d:                                 @ ORA abs
        ABS_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_abs_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2              @ a | value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a | value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_abs_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop4d:                                 @ EOR abs
        ABS_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_abs_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2              @ a ^ value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a ^ value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_abs_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopcd:                                 @ CMP abs  (sets C/Z/N; A unchanged)
        ABS_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_abs_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_abs_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopae:                                 @ LDX abs  (XF width)
        ABS_ADDR
        ABS_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lldx_abs_w
        strh    r2, [r4, #CPU_X]        @ byte zero-extends into x
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lldx_abs_w:
        strh    r2, [r4, #CPU_X]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopac:                                 @ LDY abs  (XF width)
        ABS_ADDR
        ABS_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lldy_abs_w
        strh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lldy_abs_w:
        strh    r2, [r4, #CPU_Y]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop8d:                                 @ STA abs  (MF width; writes via bridge)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_abs_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_abs_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop8e:                                 @ STX abs  (XF width; writes via bridge)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lstx_abs_w
        ldrh    r2, [r4, #CPU_X]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lstx_abs_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_X]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        ldrh    r2, [r4, #CPU_X]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop8c:                                 @ STY abs  (XF width; writes via bridge)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lsty_abs_w
        ldrh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lsty_abs_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        ldrh    r2, [r4, #CPU_Y]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop9c:                                 @ STZ abs  (MF width; stores zero)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lstz_abs_w
        movs    r2, #0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lstz_abs_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        movs    r2, #0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        movs    r2, #0
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

@ --- ADC abs (0x6d) / SBC abs (0xed): binary path only.  Decimal mode (D flag
@ set) bails to .L_noteb BEFORE the operand fetch, so C's cpu_adc/cpu_sbc
@ re-dispatch and fetch the operand themselves (pc is still post-opcode).  Maths
@ identical to the DP ADC/SBC handlers; only the address source differs.
.Lop6d:                                 @ ADC abs  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ABS_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_abs_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_abs_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Loped:                                 @ SBC abs  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ABS_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_abs_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_abs_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled


@ ===========================================================================
@ Stage 3E Pass 2 -- RMW / BIT / CPX / CPY absolute opcodes.  Same ABS_ADDR +
@ ABS_READ_VALUE front end as Pass 1.  Three oracle-critical rules, all from
@ cpu.c (cpu_asl..cpu_trb at lines 739-855, cpu_bit 656, cpu_cpx/cpy 626/641):
@   (1) RMW word mode charges cyclesUsed += 2 (NOT +1) -- see every `adds,#2`.
@   (2) RMW word write-back is REVERSED: HIGH byte at addr_high FIRST, then LOW
@       at addr_low.  This is cpu_writeWord(...,true).  Pass-1 STA/STX/STY/STZ
@       write LOW-then-HIGH (reversed=false); RMW is the OPPOSITE.  The high
@       write precedes the low write in the bus trace.
@   (3) TSB/TRB set ONLY the Z flag (no setZN -- N and V are left untouched).
@ BIT takes N/V from the MEMORY value (not the AND result).  CPX/CPY use the XF
@ flag (not MF) and ABS_READ_VALUE CPU_XF already charges the word-mode +1 cycle
@ that cpu_cpx/cpy charge, so no extra cycle code is needed there.
@
@ Register convention: r4 = Cpu* (callee-saved), r6 = addr_low (callee-saved,
@ preserved across every BL).  Because RMW word mode must keep addr_low alive
@ across two reads AND two writes while also carrying the read low byte and the
@ computed result, the result/low-byte are spilled to the stack across the BL
@ calls (r0-r3 are caller-saved).  Stack use is balanced (push/pop pairs) and
@ stays one-deep, so sp returns to its entry value before .L_handled.
@ ===========================================================================

.Lop0e:                                 @ ASL abs  (RMW; reversed word write-back)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lasl_abs_w
        @ --- byte: result = value << 1; c = result & 0x100 ---
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value (0..0xff)
        lsl     r2, r2, #1              @ result = value << 1 (0..0x1fe)
        lsr     r3, r2, #8              @ c = bit8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ result & 0xff
        push    {r2}                    @ stash for setZN (write clobbers r2)
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Lasl_abs_w:
        @ --- word: cyclesUsed += 2; read word; result = value << 1; c = bit16 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read         @ r0 = low byte
        push    {r0}                    @ stash low byte across high read
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = high byte
        pop     {r3}                    @ r3 = low byte
        orr     r2, r3, r0, lsl #8      @ word value
        lsl     r2, r2, #1              @ result = value << 1 (0..0x1fffe)
        lsr     r3, r2, #16             @ c = bit16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ result & 0xffff
        push    {r2}                    @ stash result for low write + setZN
        lsr     r2, r2, #8              @ high byte
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high
        mov     r0, r4
        bl      cpu_thumb2_write        @ write HIGH first (reversed)
        ldr     r2, [sp]                @ reload result (still pushed)
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write        @ write LOW second
        pop     {r2}                    @ r2 = result (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop4e:                                 @ LSR abs  (RMW; reversed word write-back)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llsr_abs_w
        @ --- byte: c = value & 1 (capture first); result = value >> 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ubfx    r3, r2, #0, #1          @ c = value & 1
        strb    r3, [r4, #CPU_C]
        lsr     r2, r2, #1              @ result = value >> 1
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Llsr_abs_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ubfx    r3, r2, #0, #1          @ c = value & 1
        strb    r3, [r4, #CPU_C]
        lsr     r2, r2, #1              @ result = value >> 1
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop2e:                                 @ ROL abs  (RMW; reversed word write-back)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lrol_abs_w
        @ --- byte: result = (value<<1) | c; new c = result & 0x100 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ldrb    r3, [r4, #CPU_C]        @ old c (shifted-in bit)
        lsl     r2, r2, #1
        orr     r2, r2, r3              @ result = (value<<1) | c (0..0x1ff)
        lsr     r3, r2, #8              @ c = bit8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ result & 0xff
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Lrol_abs_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ldrb    r3, [r4, #CPU_C]        @ old c
        lsl     r2, r2, #1
        orr     r2, r2, r3              @ result = (value<<1) | c (0..0x1ffff)
        lsr     r3, r2, #16             @ c = bit16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop6e:                                 @ ROR abs  (RMW; reversed word write-back)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lror_abs_w
        @ --- byte: carry = value & 1; result = (value>>1)|(c<<7); c set LAST ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ubfx    r3, r2, #0, #1          @ carry = value & 1 (new c)
        push    {r3}                    @ stash carry across write+setZN
        ldrb    r1, [r4, #CPU_C]        @ old c (shifted-in bit)
        lsr     r2, r2, #1
        lsl     r1, r1, #7              @ c << 7
        orr     r2, r2, r1             @ result = (value>>1) | (c<<7)
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}                    @ result byte
        movs    r1, #1
        SETZN
        pop     {r3}                    @ carry
        strb    r3, [r4, #CPU_C]        @ c = carry (set after setZN, per cpu.c)
        b       .L_handled
.Lror_abs_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ubfx    r3, r2, #0, #1          @ carry = value & 1
        push    {r3}                    @ stash carry
        ldrb    r1, [r4, #CPU_C]
        lsr     r2, r2, #1
        lsl     r1, r1, #15             @ c << 15
        orr     r2, r2, r1             @ result = (value>>1) | (c<<15)
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]                @ reload result (top of stack)
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}                    @ result
        movs    r1, #0
        SETZN
        pop     {r3}                    @ carry
        strb    r3, [r4, #CPU_C]
        b       .L_handled

.Lopee:                                 @ INC abs  (RMW; reversed word write-back; no carry change)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Linc_abs_w
        @ --- byte: result = value + 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0
        adds    r2, r2, #1              @ result (0..0x100)
        uxtb    r2, r2
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Linc_abs_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8
        adds    r2, r2, #1              @ result (0..0x10000)
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopce:                                 @ DEC abs  (RMW; reversed word write-back; no carry change)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ldec_abs_w
        @ --- byte: result = value - 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0
        subs    r2, r2, #1              @ result (-1..0xfe); uxtb wraps 0->0xff
        uxtb    r2, r2
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Ldec_abs_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8
        subs    r2, r2, #1
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop0c:                                 @ TSB abs  (RMW; sets Z only, N/V untouched; reversed word write-back)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ltsb_abs_w
        @ --- byte: z = ((a&0xff)&value)==0; write value | (a&0xff) ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        tst     r3, r2                  @ (a&0xff) & value
        ite     eq
        moveq   r0, #1                  @ z = 1 if zero
        movne   r0, #0
        strb    r0, [r4, #CPU_Z]
        orr     r2, r2, r3             @ value | (a&0xff)
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Ltsb_abs_w:
        @ --- word: cyclesUsed += 2; z = (a&value)==0; write value | a ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ldrh    r3, [r4, #CPU_A]        @ a (full word)
        tst     r3, r2                  @ a & value
        ite     eq
        moveq   r0, #1
        movne   r0, #0
        strb    r0, [r4, #CPU_Z]
        orr     r2, r2, r3             @ value | a
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        b       .L_handled

.Lop1c:                                 @ TRB abs  (RMW; sets Z only, N/V untouched; reversed word write-back)
        ABS_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ltrb_abs_w
        @ --- byte: z = ((a&0xff)&value)==0; write value & ~(a&0xff) ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        tst     r3, r2                  @ (a&0xff) & value
        ite     eq
        moveq   r0, #1
        movne   r0, #0
        strb    r0, [r4, #CPU_Z]
        bic     r2, r2, r3             @ value & ~(a&0xff)
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Ltrb_abs_w:
        @ --- word: cyclesUsed += 2; z = (a&value)==0; write value & ~a ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ldrh    r3, [r4, #CPU_A]        @ a
        tst     r3, r2                  @ a & value
        ite     eq
        moveq   r0, #1
        movne   r0, #0
        strb    r0, [r4, #CPU_Z]
        bic     r2, r2, r3             @ value & ~a
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        b       .L_handled

.Lop2c:                                 @ BIT abs  (read-only; N/V from memory value, Z from AND)
        ABS_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lbit_abs_w
        @ --- byte: r2 = value; z=((a&0xff)&value)==0; n=value&0x80; v=value&0x40 ---
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        tst     r3, r2                  @ (a&0xff) & value
        ite     eq
        moveq   r3, #1                  @ z = 1 if zero
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        ubfx    r3, r2, #7, #1          @ n = value & 0x80 -> bit7
        strb    r3, [r4, #CPU_N]
        ubfx    r3, r2, #6, #1          @ v = value & 0x40 -> bit6
        strb    r3, [r4, #CPU_V]
        b       .L_handled
.Lbit_abs_w:
        @ --- word: r2 = value; z=(a&value)==0; n=value&0x8000; v=value&0x4000 ---
        ldrh    r3, [r4, #CPU_A]
        tst     r3, r2                  @ a & value
        ite     eq
        moveq   r3, #1
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        ubfx    r3, r2, #15, #1         @ n = value & 0x8000 -> bit15
        strb    r3, [r4, #CPU_N]
        ubfx    r3, r2, #14, #1         @ v = value & 0x4000 -> bit14
        strb    r3, [r4, #CPU_V]
        b       .L_handled

.Lopec:                                 @ CPX abs  (XF width; compare X; sets C/Z/N; X unchanged)
        ABS_ADDR
        ABS_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lcpx_abs_w
        @ --- byte (xf=1): r2 = value; result = (x&0xff)+(value^0xff)+1 ---
        mvn     r2, r2                  @ value ^ 0xff
        uxtb    r2, r2
        ldrh    r3, [r4, #CPU_X]
        uxtb    r3, r3                  @ x & 0xff
        add     r2, r3, r2             @ (x&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcpx_abs_w:
        @ --- word (xf=0): r2 = value; result = x+(value^0xffff)+1 ---
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_X]
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopcc:                                 @ CPY abs  (XF width; compare Y; sets C/Z/N; Y unchanged)
        ABS_ADDR
        ABS_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lcpy_abs_w
        @ --- byte (xf=1): r2 = value; result = (y&0xff)+(value^0xff)+1 ---
        mvn     r2, r2
        uxtb    r2, r2
        ldrh    r3, [r4, #CPU_Y]
        uxtb    r3, r3                  @ y & 0xff
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcpy_abs_w:
        @ --- word (xf=0): r2 = value; result = y+(value^0xffff)+1 ---
        mvn     r2, r2
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_Y]
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled


@ ===========================================================================
@ Stage 3F Pass 1 -- absolute,X / absolute,Y opcodes.  Each handler is the
@ verbatim abs handler with ABS_ADDR replaced by ABXY_ADDR: the ALU tails,
@ SETZN, and write tails are byte-identical because cpu_adrAbx/Aby feed the SAME
@ cpu_lda/cpu_and/... helpers as cpu_adrAbs.  The ONLY new semantics are in
@ ABXY_ADDR itself: the +idx term in the effective address and the write-only
@ page-cross penalty (cpu.c:464/472).  ADC/SBC (0x7d/0xfd abx, 0x79/0xf9 aby)
@ keep the decimal-bail BEFORE ABXY_ADDR, exactly as the abs ADC/SBC handlers
@ bail before ABS_ADDR (pc correctness for the C fallback).  LDY abx (0xbc) and
@ LDX aby (0xbe) gate on XF, mirroring abs LDX/LDY.  STA/STZ/STA writes pass
@ write=1 to ABXY_ADDR so the page-cross penalty is charged.
@ ===========================================================================

@ ---------------- absolute,X (idx = X, idxoff = CPU_X) ----------------

.Lopbd:                                 @ LDA abx  (read)
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_abx_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_abx_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop3d:                                 @ AND abx  (read)
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_abx_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a & value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_abx_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (word)
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop1d:                                 @ ORA abx  (read)
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_abx_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2              @ a | value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a | value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_abx_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop5d:                                 @ EOR abx  (read)
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_abx_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2              @ a ^ value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a ^ value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_abx_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopdd:                                 @ CMP abx  (read; sets C/Z/N; A unchanged)
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_abx_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_abx_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopbc:                                 @ LDY abx  (read; XF width)
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lldy_abx_w
        strh    r2, [r4, #CPU_Y]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lldy_abx_w:
        strh    r2, [r4, #CPU_Y]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop9d:                                 @ STA abx  (WRITE; MF width; writes via bridge)
        ABXY_ADDR 1, CPU_X
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_abx_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_abx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop9e:                                 @ STZ abx  (WRITE; MF width; stores zero)
        ABXY_ADDR 1, CPU_X
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lstz_abx_w
        movs    r2, #0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lstz_abx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        movs    r2, #0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        movs    r2, #0
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

@ --- ADC abx (0x7d) / SBC abx (0xfd): binary path only.  Decimal mode (D flag
@ set) bails to .L_noteb BEFORE any operand fetch, so C's cpu_adc/cpu_sbc
@ re-dispatch and fetch the operand themselves (pc is still post-opcode).  Maths
@ identical to the abs ADC/SBC handlers; only the address source differs.
.Lop7d:                                 @ ADC abx  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_abx_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_abx_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Lopfd:                                 @ SBC abx  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_abx_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_abx_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

@ ---------------- absolute,Y (idx = Y, idxoff = CPU_Y) ----------------

.Lopb9:                                 @ LDA aby  (read)
        ABXY_ADDR 0, CPU_Y
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_aby_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_aby_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop39:                                 @ AND aby  (read)
        ABXY_ADDR 0, CPU_Y
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_aby_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a & value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_aby_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (word)
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop19:                                 @ ORA aby  (read)
        ABXY_ADDR 0, CPU_Y
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_aby_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2              @ a | value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a | value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_aby_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop59:                                 @ EOR aby  (read)
        ABXY_ADDR 0, CPU_Y
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_aby_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2              @ a ^ value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a ^ value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_aby_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopd9:                                 @ CMP aby  (read; sets C/Z/N; A unchanged)
        ABXY_ADDR 0, CPU_Y
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_aby_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_aby_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopbe:                                 @ LDX aby  (read; XF width)
        ABXY_ADDR 0, CPU_Y
        ABS_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lldx_aby_w
        strh    r2, [r4, #CPU_X]        @ byte zero-extends into x
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lldx_aby_w:
        strh    r2, [r4, #CPU_X]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop99:                                 @ STA aby  (WRITE; MF width; writes via bridge)
        ABXY_ADDR 1, CPU_Y
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_aby_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_aby_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

@ --- ADC aby (0x79) / SBC aby (0xf9): binary path only.  Decimal mode (D flag
@ set) bails to .L_noteb BEFORE any operand fetch, so C's cpu_adc/cpu_sbc
@ re-dispatch and fetch the operand themselves (pc is still post-opcode).  Maths
@ identical to the abs ADC/SBC handlers; only the address source differs.
.Lop79:                                 @ ADC aby  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ABXY_ADDR 0, CPU_Y
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_aby_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_aby_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Lopf9:                                 @ SBC aby  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ABXY_ADDR 0, CPU_Y
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_aby_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_aby_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled


@ ===========================================================================
@ Stage 3F Pass 2 -- absolute,X RMW / BIT opcodes.  Each handler is the VERBATIM
@ abs ancestor (Stage 3E Pass 2) with ABS_ADDR replaced by ABXY_ADDR.  The ALU
@ tails, SETZN, reversed word write-back and BIT flag extraction are byte-identical
@ because cpu_adrAbx feeds the SAME cpu_asl/cpu_lsr/cpu_rol/cpu_ror/cpu_inc/
@ cpu_dec/cpu_bit helpers as cpu_adrAbs (cpu.c:1096 ASL abx, 1280 BIT abx,
@ 1292 ROL abx, 1512 LSR abx, 1719 ROR abx, 2314 DEC abx, 2509 INC abx -- all
@ call cpu_adrAbx(,,write)).  The ONLY new semantics are in ABXY_ADDR: the +X
@ term in the effective address and the write-only page-cross penalty.  BIT abx
@ (0x3c) is a READ (write=0, no page-cross penalty); the 6 RMW opcodes are
@ WRITES (write=1, penalty applies when xf==0 OR page crossed).
@
@ RMW CYCLE STACKING (oracle-critical): RMW abx word mode with a page-cross
@ charges TWO independent cycle increments that MUST both accumulate:
@   (1) +1 from the page-cross penalty inside ABXY_ADDR 1, CPU_X (xf==0 OR
@       (adr>>8)!=((adr+X)>>8)) -- cpu.c:464, mirrored in the ABXY_ADDR macro.
@   (2) +2 from the RMW word-mode tail (the `adds r0,r0,#2` copied verbatim from
@       the abs handler) -- cpu.c cpu_asl..cpu_dec charge cyclesUsed += 2.
@ Total = +3 worst case.  The two charges are in different code regions (macro
@ vs tail), both read-modify-write [r4,#CPU_CYCLESUSED] (ldrb/adds/strb), so they
@ stack -- DO NOT remove or merge either.  BIT abx word mode pays only the +1
@ from ABS_READ_VALUE (read, no ABXY_ADDR penalty, no RMW tail).
@ ===========================================================================

.Lop1e:                                 @ ASL abx  (RMW; reversed word write-back)
        ABXY_ADDR 1, CPU_X
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lasl_abx_w
        @ --- byte: result = value << 1; c = result & 0x100 ---
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value (0..0xff)
        lsl     r2, r2, #1              @ result = value << 1 (0..0x1fe)
        lsr     r3, r2, #8              @ c = bit8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ result & 0xff
        push    {r2}                    @ stash for setZN (write clobbers r2)
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Lasl_abx_w:
        @ --- word: cyclesUsed += 2; read word; result = value << 1; c = bit16 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_read         @ r0 = low byte
        push    {r0}                    @ stash low byte across high read
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = high byte
        pop     {r3}                    @ r3 = low byte
        orr     r2, r3, r0, lsl #8      @ word value
        lsl     r2, r2, #1              @ result = value << 1 (0..0x1fffe)
        lsr     r3, r2, #16             @ c = bit16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ result & 0xffff
        push    {r2}                    @ stash result for low write + setZN
        lsr     r2, r2, #8              @ high byte
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high
        mov     r0, r4
        bl      cpu_thumb2_write        @ write HIGH first (reversed)
        ldr     r2, [sp]                @ reload result (still pushed)
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write        @ write LOW second
        pop     {r2}                    @ r2 = result (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop5e:                                 @ LSR abx  (RMW; reversed word write-back)
        ABXY_ADDR 1, CPU_X
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llsr_abx_w
        @ --- byte: c = value & 1 (capture first); result = value >> 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ubfx    r3, r2, #0, #1          @ c = value & 1
        strb    r3, [r4, #CPU_C]
        lsr     r2, r2, #1              @ result = value >> 1
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Llsr_abx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ubfx    r3, r2, #0, #1          @ c = value & 1
        strb    r3, [r4, #CPU_C]
        lsr     r2, r2, #1              @ result = value >> 1
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop3e:                                 @ ROL abx  (RMW; reversed word write-back)
        ABXY_ADDR 1, CPU_X
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lrol_abx_w
        @ --- byte: result = (value<<1) | c; new c = result & 0x100 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ldrb    r3, [r4, #CPU_C]        @ old c (shifted-in bit)
        lsl     r2, r2, #1
        orr     r2, r2, r3              @ result = (value<<1) | c (0..0x1ff)
        lsr     r3, r2, #8              @ c = bit8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ result & 0xff
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Lrol_abx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ldrb    r3, [r4, #CPU_C]        @ old c
        lsl     r2, r2, #1
        orr     r2, r2, r3              @ result = (value<<1) | c (0..0x1ffff)
        lsr     r3, r2, #16             @ c = bit16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop7e:                                 @ ROR abx  (RMW; reversed word write-back)
        ABXY_ADDR 1, CPU_X
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lror_abx_w
        @ --- byte: carry = value & 1; result = (value>>1)|(c<<7); c set LAST ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0                  @ value
        ubfx    r3, r2, #0, #1          @ carry = value & 1 (new c)
        push    {r3}                    @ stash carry across write+setZN
        ldrb    r1, [r4, #CPU_C]        @ old c (shifted-in bit)
        lsr     r2, r2, #1
        lsl     r1, r1, #7              @ c << 7
        orr     r2, r2, r1             @ result = (value>>1) | (c<<7)
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}                    @ result byte
        movs    r1, #1
        SETZN
        pop     {r3}                    @ carry
        strb    r3, [r4, #CPU_C]        @ c = carry (set after setZN, per cpu.c)
        b       .L_handled
.Lror_abx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8      @ word value
        ubfx    r3, r2, #0, #1          @ carry = value & 1
        push    {r3}                    @ stash carry
        ldrb    r1, [r4, #CPU_C]
        lsr     r2, r2, #1
        lsl     r1, r1, #15             @ c << 15
        orr     r2, r2, r1             @ result = (value>>1) | (c<<15)
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write        @ HIGH first
        ldr     r2, [sp]                @ reload result (top of stack)
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write        @ LOW second
        pop     {r2}                    @ result
        movs    r1, #0
        SETZN
        pop     {r3}                    @ carry
        strb    r3, [r4, #CPU_C]
        b       .L_handled

.Lopfe:                                 @ INC abx  (RMW; reversed word write-back; no carry change)
        ABXY_ADDR 1, CPU_X
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Linc_abx_w
        @ --- byte: result = value + 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0
        adds    r2, r2, #1              @ result (0..0x100)
        uxtb    r2, r2
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Linc_abx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8
        adds    r2, r2, #1              @ result (0..0x10000)
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopde:                                 @ DEC abx  (RMW; reversed word write-back; no carry change)
        ABXY_ADDR 1, CPU_X
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ldec_abx_w
        @ --- byte: result = value - 1 ---
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        uxtb    r2, r0
        subs    r2, r2, #1              @ result (-1..0xfe); uxtb wraps 0->0xff
        uxtb    r2, r2
        push    {r2}
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #1
        SETZN
        b       .L_handled
.Ldec_abx_w:
        @ --- word: cyclesUsed += 2 ---
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #2
        strb    r0, [r4, #CPU_CYCLESUSED]
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read
        push    {r0}
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_read
        pop     {r3}
        orr     r2, r3, r0, lsl #8
        subs    r2, r2, #1
        uxth    r2, r2
        push    {r2}
        lsr     r2, r2, #8
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        mov     r0, r4
        bl      cpu_thumb2_write
        ldr     r2, [sp]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        pop     {r2}
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop3c:                                 @ BIT abx  (read-only; N/V from memory value, Z from AND)
        ABXY_ADDR 0, CPU_X
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lbit_abx_w
        @ --- byte: r2 = value; z=((a&0xff)&value)==0; n=value&0x80; v=value&0x40 ---
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        tst     r3, r2                  @ (a&0xff) & value
        ite     eq
        moveq   r3, #1                  @ z = 1 if zero
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        ubfx    r3, r2, #7, #1          @ n = value & 0x80 -> bit7
        strb    r3, [r4, #CPU_N]
        ubfx    r3, r2, #6, #1          @ v = value & 0x40 -> bit6
        strb    r3, [r4, #CPU_V]
        b       .L_handled
.Lbit_abx_w:
        @ --- word: r2 = value; z=(a&value)==0; n=value&0x8000; v=value&0x4000 ---
        ldrh    r3, [r4, #CPU_A]
        tst     r3, r2                  @ a & value
        ite     eq
        moveq   r3, #1
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        ubfx    r3, r2, #15, #1         @ n = value & 0x8000 -> bit15
        strb    r3, [r4, #CPU_N]
        ubfx    r3, r2, #14, #1         @ v = value & 0x4000 -> bit14
        strb    r3, [r4, #CPU_V]
        b       .L_handled


@ ===========================================================================
@ Stage 3G Pass 1 -- DP-indirect opcodes (IDX $dp,X / IDP $dp / IDY $dp,Y).
@ Each handler is the VERBATIM abs ancestor (Stage 3E Pass 1) with ABS_ADDR
@ replaced by the corresponding indirect macro (IDX_ADDR / IDP_ADDR / IDY_ADDR).
@ The ALU tails, SETZN, STA write-back and ADC/SBC maths are byte-identical
@ because cpu_adrIdp/Idx/Idy feed the SAME cpu_ora/cpu_and/cpu_eor/cpu_adc/
@ cpu_sbc/cpu_cmp/cpu_lda/cpu_sta helpers as cpu_adrAbs (cpu.c:918 ORA idx,
@ 1015 ORA idy, 1021 ORA idp, ... 1836 STA idy, 1842 STA idp, ...).  The ONLY
@ new semantics live in the three indirect macros: the 1-raw-operand-byte
@ fetch, the dp&0xff cycle charge, the 2-HOOKED-read pointer fetch at bank-0
@ 16-bit-wrapped addresses (uxth), and (IDY write=0x91 STA ONLY) the
@ page-cross penalty.  ADC/SBC bail to .L_noteb BEFORE the operand fetch when
@ the D flag is set, so C re-dispatches with pc still post-opcode.
@ ===========================================================================

@ ---------------- ($dp,X) IDX : 0x01 ORA 0x21 AND 0x41 EOR 0x61 ADC 0x81 STA 0xa1 LDA 0xc1 CMP 0xe1 SBC

.Lop01:                                 @ ORA idx  (read)
        IDX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_idx_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2              @ a | value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a | value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_idx_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop21:                                 @ AND idx  (read)
        IDX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_idx_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a & value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_idx_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (word)
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop41:                                 @ EOR idx  (read)
        IDX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_idx_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2              @ a ^ value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a ^ value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_idx_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopa1:                                 @ LDA idx  (read)
        IDX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_idx_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_idx_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lopc1:                                 @ CMP idx  (read; sets C/Z/N; A unchanged)
        IDX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_idx_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_idx_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop81:                                 @ STA idx  (MF width; writes via bridge)
        IDX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_idx_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_idx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop61:                                 @ ADC idx  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        IDX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_idx_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_idx_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Lope1:                                 @ SBC idx  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        IDX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_idx_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_idx_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

@ ---------------- ($dp) IDP : 0x12 ORA 0x32 AND 0x52 EOR 0x72 ADC 0x92 STA 0xb2 LDA 0xd2 CMP 0xf2 SBC

.Lop12:                                 @ ORA idp  (read)
        IDP_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_idp_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_idp_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop32:                                 @ AND idp  (read)
        IDP_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_idp_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_idp_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop52:                                 @ EOR idp  (read)
        IDP_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_idp_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_idp_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopb2:                                 @ LDA idp  (read)
        IDP_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_idp_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Llda_idp_w:
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopd2:                                 @ CMP idp  (read; sets C/Z/N; A unchanged)
        IDP_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_idp_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3
        mvn     r2, r2
        uxtb    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_idp_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop92:                                 @ STA idp  (MF width; writes via bridge)
        IDP_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_idp_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_idp_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        ldrh    r2, [r4, #CPU_A]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop72:                                 @ ADC idp  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        IDP_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_idp_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Ladc_idp_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopf2:                                 @ SBC idp  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        IDP_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_idp_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_idp_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

@ ---------------- ($dp),Y IDY : 0x11 ORA 0x31 AND 0x51 EOR 0x71 ADC 0x91 STA(W) 0xb1 LDA 0xd1 CMP 0xf1 SBC

.Lop11:                                 @ ORA idy  (read)
        IDY_ADDR 0
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_idy_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_idy_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop31:                                 @ AND idy  (read)
        IDY_ADDR 0
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_idy_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_idy_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop51:                                 @ EOR idy  (read)
        IDY_ADDR 0
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_idy_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_idy_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopb1:                                 @ LDA idy  (read)
        IDY_ADDR 0
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_idy_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Llda_idy_w:
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopd1:                                 @ CMP idy  (read; sets C/Z/N; A unchanged)
        IDY_ADDR 0
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_idy_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3
        mvn     r2, r2
        uxtb    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_idy_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop91:                                 @ STA idy  (WRITE penalty; MF width; writes via bridge)
        IDY_ADDR 1
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_idy_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_idy_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        ldrh    r2, [r4, #CPU_A]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop71:                                 @ ADC idy  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        IDY_ADDR 0
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_idy_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Ladc_idy_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopf1:                                 @ SBC idy  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        IDY_ADDR 0
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_idy_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_idy_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled


@ ===========================================================================
@ Stage 3G Pass 2 -- DP-indirect-long opcodes (IDL [dp] / ILY [dp],Y).
@ Each handler is the VERBATIM abs ancestor (Stage 3E) with ABS_ADDR replaced
@ by the corresponding indirect-long macro (IDL_ADDR / ILY_ADDR).  The ALU
@ tails, SETZN, STA write-back and ADC/SBC maths are byte-identical because
@ cpu_adrIdl/cpu_adrIly feed the SAME cpu_ora/cpu_and/cpu_eor/cpu_adc/
@ cpu_sbc/cpu_cmp/cpu_lda/cpu_sta helpers as cpu_adrAbs (cpu.c:918+).  The ONLY
@ new semantics live in the two indirect-long macros: the 1-raw-operand-byte
@ fetch, the dp&0xff cycle charge, the 3-HOOKED-read pointer fetch at bank-0
@ 16-bit-wrapped addresses (uxth), and the bank-from-memory 3rd pointer byte.
@ NO write penalty for either mode (even STA idl 0x87 / STA ily 0x97).
@ ADC/SBC bail to .L_noteb BEFORE the operand fetch when the D flag is set.
@ ===========================================================================

@ ---------------- [dp] IDL : 0x07 ORA 0x27 AND 0x47 EOR 0x67 ADC 0x87 STA 0xa7 LDA 0xc7 CMP 0xe7 SBC

.Lop07:                                 @ ORA idl  (read)
        IDL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_idl_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_idl_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop27:                                 @ AND idl  (read)
        IDL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_idl_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_idl_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop47:                                 @ EOR idl  (read)
        IDL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_idl_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_idl_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopa7:                                 @ LDA idl  (read)
        IDL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_idl_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Llda_idl_w:
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopc7:                                 @ CMP idl  (read; sets C/Z/N; A unchanged)
        IDL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_idl_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3
        mvn     r2, r2
        uxtb    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_idl_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop87:                                 @ STA idl  (MF width; NO write penalty; writes via bridge)
        IDL_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_idl_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_idl_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        ldrh    r2, [r4, #CPU_A]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop67:                                 @ ADC idl  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        IDL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_idl_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Ladc_idl_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lope7:                                 @ SBC idl  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        IDL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_idl_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_idl_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

@ ---------------- [dp],Y ILY : 0x17 ORA 0x37 AND 0x57 EOR 0x77 ADC 0x97 STA 0xb7 LDA 0xd7 CMP 0xf7 SBC

.Lop17:                                 @ ORA ily  (read)
        ILY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_ily_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_ily_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop37:                                 @ AND ily  (read)
        ILY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_ily_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_ily_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop57:                                 @ EOR ily  (read)
        ILY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_ily_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_ily_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopb7:                                 @ LDA ily  (read)
        ILY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_ily_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Llda_ily_w:
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopd7:                                 @ CMP ily  (read; sets C/Z/N; A unchanged)
        ILY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_ily_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3
        mvn     r2, r2
        uxtb    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_ily_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop97:                                 @ STA ily  (MF width; NO write penalty; writes via bridge)
        ILY_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_ily_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_ily_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        ldrh    r2, [r4, #CPU_A]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop77:                                 @ ADC ily  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ILY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_ily_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Ladc_ily_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopf7:                                 @ SBC ily  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ILY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_ily_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_ily_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled


@ ===========================================================================
@ Stage 3J -- absolute-long (abl/alx) + dp,Y (dpy) opcodes.
@ Each abl/alx handler is the VERBATIM abs ancestor (Stage 3E) with ABS_ADDR
@ replaced by ABL_ADDR/ALX_ADDR.  Each dpy handler is the VERBATIM dp ancestor
@ (Stage 3D/3H) with DP_ADDR replaced by DPY_ADDR.  The ALU tails, SETZN,
@ STA/STX write-back and ADC/SBC maths are byte-identical because cpu_adrAbl/
@ cpu_adrAlx feed the SAME cpu_ora/cpu_and/cpu_eor/cpu_adc/cpu_sbc/cpu_cmp/
@ cpu_lda/cpu_sta helpers as cpu_adrAbs (cpu.c:918+), and cpu_adrDpo feeds the
@ same helpers as cpu_adrDpx (cpu.c:392-397).  The ONLY new semantics live in
@ the three new address macros:
@   ABL_ADDR -- 3 RAW operand bytes, bank FROM OPERAND (NOT db), 24-bit addr,
@               NO cycle, NO page-cross penalty (cpu.c:478-483).
@   ALX_ADDR -- ABL_ADDR + X with 24-bit wrap ((adr+X)&0xffffff, NOT uxth),
@               NO page-cross penalty (cpu.c:485-490).
@   DPY_ADDR -- 1 RAW operand byte, dp+off+Y with 16-bit wrap (uxth, bank 0),
@               dp&0xff cycle charge (cpu.c:392-397).
@ ADC/SBC bail to .L_noteb BEFORE the operand fetch when the D flag is set.
@ ===========================================================================

@ ---------------- abl : 0x0f ORA 0x2f AND 0x4f EOR 0xcf CMP 0xaf LDA 0x8f STA 0x6f ADC 0xef SBC

.Lop0f:                                 @ ORA abl  (read)
        ABL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_abl_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2              @ a | value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a | value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_abl_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop2f:                                 @ AND abl  (read)
        ABL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_abl_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a & value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_abl_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (word)
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop4f:                                 @ EOR abl  (read)
        ABL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_abl_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2              @ a ^ value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a ^ value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_abl_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopcf:                                 @ CMP abl  (sets C/Z/N; A unchanged)
        ABL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_abl_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_abl_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopaf:                                 @ LDA abl
        ABL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_abl_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_abl_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop8f:                                 @ STA abl  (MF width; writes via bridge)
        ABL_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_abl_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_abl_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop6f:                                 @ ADC abl  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ABL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_abl_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_abl_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Lopef:                                 @ SBC abl  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ABL_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_abl_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_abl_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

@ ---------------- alx : 0x1f ORA 0x3f AND 0x5f EOR 0xdf CMP 0xbf LDA 0x9f STA 0x7f ADC 0xff SBC

.Lop1f:                                 @ ORA alx  (read)
        ALX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_alx_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2              @ a | value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a | value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_alx_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop3f:                                 @ AND alx  (read)
        ALX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_alx_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (value is a byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a & value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_alx_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2              @ a & value (word)
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop5f:                                 @ EOR alx  (read)
        ALX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_alx_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2              @ a ^ value (low byte)
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | (a ^ value)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_alx_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopdf:                                 @ CMP alx  (sets C/Z/N; A unchanged)
        ALX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_alx_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_alx_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopbf:                                 @ LDA alx
        ALX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_alx_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_alx_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lop9f:                                 @ STA alx  (MF width; writes via bridge)
        ALX_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_alx_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_alx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24         @ addr_high = (addr_low+1) & 0xffffff
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop7f:                                 @ ADC alx  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ALX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_alx_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_alx_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Lopff:                                 @ SBC alx  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ALX_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_alx_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_alx_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

@ ---------------- dpy : 0x96 STX 0xb6 LDX

.Lop96:                                 @ STX dpy  (XF width; writes via bridge)
        DPY_ADDR
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lstx_dpy_w
        ldrh    r2, [r4, #CPU_X]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lstx_dpy_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_X]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1
        ldrh    r2, [r4, #CPU_X]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lopb6:                                 @ LDX dpy  (XF width)
        DPY_ADDR
        DP_READ_VALUE CPU_XF
        ldrb    r1, [r4, #CPU_XF]
        cbz     r1, .Lldx_dpy_w
        strh    r2, [r4, #CPU_X]        @ byte zero-extends into x
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lldx_dpy_w:
        strh    r2, [r4, #CPU_X]
        movs    r1, #0
        SETZN
        b       .L_handled

@ ===== Stage 3K: stack push/pull (13 opcodes) =====
@ All stack R/W via HOOKED cpu_thumb2_read/write.  e-mode SP clamp in macros.
@ r4=Cpu*, r5=contract (preserved), r6=callee-saved scratch.

@ --- 0x08 PHP: push flags byte ---
.Lop08:
        ldrb    r2, [r4, #CPU_N]
        lsl     r2, r2, #7
        ldrb    r1, [r4, #CPU_V]
        lsl     r1, r1, #6
        orr     r2, r1
        ldrb    r1, [r4, #CPU_MF]
        lsl     r1, r1, #5
        orr     r2, r1
        ldrb    r1, [r4, #CPU_XF]
        lsl     r1, r1, #4
        orr     r2, r1
        ldrb    r1, [r4, #CPU_D]
        lsl     r1, r1, #3
        orr     r2, r1
        ldrb    r1, [r4, #CPU_I]
        lsl     r1, r1, #2
        orr     r2, r1
        ldrb    r1, [r4, #CPU_Z]
        lsl     r1, r1, #1
        orr     r2, r1
        ldrb    r1, [r4, #CPU_C]
        orr     r2, r1
        PUSH_BYTE
        b       .L_handled

@ --- 0x0b PHD: push DP word ---
.Lop0b:
        ldrh    r6, [r4, #CPU_DP]
        PUSH_WORD
        b       .L_handled

@ --- 0x28 PLP: pull byte -> setFlags (full re-normalization) ---
.Lop28:
        PULL_BYTE                       @ r0 = flags byte
        SETFLAGS_TAIL                   @ unpack 8 bits + e/xf renormalization
        b       .L_handled

@ --- 0x2b PLD: pull word -> DP, setZN(word) ---
.Lop2b:
        PULL_WORD                       @ r6 = word
        strh    r6, [r4, #CPU_DP]
        mov     r2, r6
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

@ --- 0x48 PHA: push A (MF width) ---
.Lop48:
        ldrb    r0, [r4, #CPU_MF]
        cbz     r0, .Lpha_w
        ldrb    r2, [r4, #CPU_A]        @ byte: push A low
        PUSH_BYTE
        b       .L_handled
.Lpha_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r6, [r4, #CPU_A]
        PUSH_WORD
        b       .L_handled

@ --- 0x4b PHK: push K byte ---
.Lop4b:
        ldrb    r2, [r4, #CPU_K]
        PUSH_BYTE
        b       .L_handled

@ --- 0x5a PHY: push Y (XF width) ---
.Lop5a:
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, .Lphy_w
        ldrb    r2, [r4, #CPU_Y]
        PUSH_BYTE
        b       .L_handled
.Lphy_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r6, [r4, #CPU_Y]
        PUSH_WORD
        b       .L_handled

@ --- 0x68 PLA: pull -> A (MF width), setZN ---
.Lop68:
        ldrb    r0, [r4, #CPU_MF]
        cbz     r0, .Lpla_w
        PULL_BYTE                       @ r0 = byte
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r0, #0, #8         @ A = (A & 0xff00) | byte
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        ldrb    r1, [r4, #CPU_MF]       @ byte-mode flag
        SETZN
        b       .L_handled
.Lpla_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        PULL_WORD                       @ r6 = word
        strh    r6, [r4, #CPU_A]
        mov     r2, r6
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

@ --- 0x7a PLY: pull -> Y (XF width), setZN ---
.Lop7a:
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, .Lply_w
        PULL_BYTE                       @ r0 = byte
        uxtb    r6, r0                  @ zero-extend (high = 0)
        strh    r6, [r4, #CPU_Y]
        mov     r2, r6
        ldrb    r1, [r4, #CPU_XF]       @ byte-mode flag
        SETZN
        b       .L_handled
.Lply_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        PULL_WORD                       @ r6 = word
        strh    r6, [r4, #CPU_Y]
        mov     r2, r6
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

@ --- 0x8b PHB: push DB byte ---
.Lop8b:
        ldrb    r2, [r4, #CPU_DB]
        PUSH_BYTE
        b       .L_handled

@ --- 0xab PLB: pull -> DB, setZN(byte) ---
.Lopab:
        PULL_BYTE                       @ r0 = byte
        strb    r0, [r4, #CPU_DB]
        mov     r2, r0
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled

@ --- 0xda PHX: push X (XF width) ---
.Lopda:
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, .Lphx_w
        ldrb    r2, [r4, #CPU_X]
        PUSH_BYTE
        b       .L_handled
.Lphx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r6, [r4, #CPU_X]
        PUSH_WORD
        b       .L_handled

@ --- 0xfa PLX: pull -> X (XF width), setZN ---
.Lopfa:
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, .Lplx_w
        PULL_BYTE                       @ r0 = byte
        uxtb    r6, r0                  @ zero-extend (high = 0)
        strh    r6, [r4, #CPU_X]
        mov     r2, r6
        ldrb    r1, [r4, #CPU_XF]       @ byte-mode flag
        SETZN
        b       .L_handled
.Lplx_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        PULL_WORD                       @ r6 = word
        strh    r6, [r4, #CPU_X]
        mov     r2, r6
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

@ ===========================================================================
@ Stage 3L -- jump / return / subroutine opcodes (Family B).
@
@ Twelve opcodes that redirect control flow.  All operand bytes are RAW
@ (IMM_FETCH_BYTE = cpu_readOpcode = cpu_read_raw, NO spin hook); indirect
@ pointer bytes and all stack accesses are HOOKED (cpu_thumb2_read/write =
@ cpu_read/cpu_write WITH spin hook).  Banks: JMP/JSR abs use NO bank (raw
@ word fetch); 0x6c/0xdc indirect pointers are bank 0; 0x7c/0xfc indirect
@ pointers are bank K.  JSR/JSL push pc-1; RTS/RTL add +1; RTI adds NO +1.
@ RTS/RTL gate on spBreakpoint (bail to C before any pull); RTI does not.
@ ===========================================================================

@ --- 0x4c JMP abs: raw 2-byte word fetch, pc = word.  No push, no DB. ---
.Lop4c:
        IMM_FETCH_BYTE                  @ r0 = low byte; pc++ (uint16 wrap)
        mov     r6, r0                  @ stash low (callee-saved)
        IMM_FETCH_BYTE                  @ r0 = high byte; pc++
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = target word
        strh    r6, [r4, #CPU_PC]       @ pc = word
        b       .L_handled

@ --- 0x82 BRL rll: raw 2-byte fetch, sign-extend 16-bit, pc += disp.  No
@ taken-cycle (always-taken branch, like BRA 0x80).  K unchanged. ---
.Lop82:
        IMM_FETCH_BYTE                  @ r0 = low byte
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = high byte
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = unsigned word
        sxth    r6, r6                  @ r6 = (int16_t)word
        ldrh    r2, [r4, #CPU_PC]       @ pc (past both operand bytes)
        add     r6, r6, r2             @ pc += disp (signed; uint16 wrap via strh)
        strh    r6, [r4, #CPU_PC]
        b       .L_handled

@ --- 0x5c JML abl: raw 3-byte fetch (word + byte), set k then pc.  NO push.
@ The Die() trap at cpu.c:1497-1501 (target 0x80:0x8573) is debug-only and
@ intentionally NOT replicated -- the rig pre-seeds operand bytes to a
@ non-magic target (run_case_jml_safe), and on the device the crash
@ sentinel is handled at a higher level. ---
.Lop5c:
        IMM_FETCH_BYTE                  @ r0 = low byte
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = high byte
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = value (target pc)
        IMM_FETCH_BYTE                  @ r0 = new_k byte; r6 survives (callee-saved)
        strb    r0, [r4, #CPU_K]        @ k = new_k
        strh    r6, [r4, #CPU_PC]       @ pc = value
        b       .L_handled

@ --- 0x20 JSR abs: raw word -> target, push pc-1, pc = target.
@ After fetching 2 operand bytes pc points past them; pc-1 = last operand
@ byte addr.  Target is stashed on the ARM stack across PUSH_WORD (which
@ needs r6 = the word to push). ---
.Lop20:
        IMM_FETCH_BYTE                  @ r0 = low byte
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = high byte
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = target
        push    {r6}                    @ stash target across PUSH_WORD
        ldrh    r6, [r4, #CPU_PC]       @ pc (past both operands)
        subs    r6, r6, #1              @ r6 = pc - 1 (return addr word)
        PUSH_WORD                       @ push (pc-1): HIGH then LOW
        pop     {r6}                    @ restore target
        strh    r6, [r4, #CPU_PC]       @ pc = target
        b       .L_handled

@ --- 0x22 JSL abl: raw 3 bytes (word->target, byte->newK).  Push K byte
@ FIRST, then push pc-1 word.  Set pc=target, k=newK.  Target and newK are
@ stashed together (8-byte push, stays aligned) across both pushes. ---
.Lop22:
        IMM_FETCH_BYTE                  @ r0 = low byte
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = high byte
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = target (value)
        IMM_FETCH_BYTE                  @ r0 = newK byte
        push    {r0, r6}                @ stash newK + target (8 bytes, aligned)
        ldrb    r2, [r4, #CPU_K]        @ r2 = current k
        PUSH_BYTE                       @ push(k) FIRST
        ldrh    r6, [r4, #CPU_PC]       @ pc (past 3 operands)
        subs    r6, r6, #1              @ pc - 1
        PUSH_WORD                       @ push(pc-1)
        pop     {r0, r6}                @ r0 = newK, r6 = target
        strh    r6, [r4, #CPU_PC]       @ pc = value
        strb    r0, [r4, #CPU_K]        @ k = newK
        b       .L_handled

@ --- 0x6c JMP (abs): raw word -> adr (bank 0).  2 HOOKED reads at bank-0
@ 16-bit-wrapped addresses (6502 page-wrap): lo@adr, hi@(adr+1)&0xffff.
@ pc = lo|(hi<<8).  Stash adr in r6 (callee-saved), spill low byte to stack. ---
.Lop6c:
        IMM_FETCH_BYTE                  @ r0 = low byte of adr
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = high byte of adr
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = adr (16-bit, bank 0)
        mov     r0, r4
        mov     r1, r6                  @ lo addr = adr (bank 0)
        bl      cpu_thumb2_read         @ r0 = lo byte (HOOKED); r6 survives
        push    {r0}                    @ stash lo byte across hi-byte BL
        add     r1, r6, #1
        uxth    r1, r1                  @ hi addr = (adr+1) & 0xffff (bank 0)
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = hi byte (HOOKED)
        lsl     r0, r0, #8
        pop     {r1}                    @ r1 = lo byte
        orr     r6, r1, r0             @ r6 = lo|(hi<<8)
        strh    r6, [r4, #CPU_PC]       @ pc = target
        b       .L_handled

@ --- 0xdc JML [abs] indirect long: raw word -> adr.  3 HOOKED reads at
@ bank-0 16-bit-wrapped addresses: pc-lo@adr, pc-hi@(adr+1)&0xffff,
@ k@(adr+2)&0xffff.  2-deep stack spill (IDL_ADDR pattern). ---
.Lopdc:
        IMM_FETCH_BYTE                  @ r0 = low byte of adr
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = high byte of adr
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = adr (16-bit)
        mov     r0, r4
        mov     r1, r6                  @ pc-lo addr (bank 0)
        bl      cpu_thumb2_read         @ r0 = pc-lo; r6 survives
        push    {r0}                    @ stash pc-lo
        add     r1, r6, #1
        uxth    r1, r1                  @ pc-hi addr (bank 0)
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = pc-hi
        push    {r0}                    @ stash pc-hi
        add     r1, r6, #2
        uxth    r1, r1                  @ k addr (bank 0)
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = k byte
        strb    r0, [r4, #CPU_K]        @ k = new bank
        pop     {r1}                    @ r1 = pc-hi
        lsl     r1, r1, #8
        pop     {r0}                    @ r0 = pc-lo
        orr     r0, r0, r1             @ r0 = pc-lo|(pc-hi<<8)
        strh    r0, [r4, #CPU_PC]       @ pc = target
        b       .L_handled

@ --- 0x7c JMP (abs,X): raw word -> adr, base=(adr+X)&0xffff.  2 HOOKED
@ reads at BANK K: lo@(k<<16)|base, hi@(k<<16)|((base+1)&0xffff).
@ (adr+X+1)&0xffff == (base+1)&0xffff (proven: single 16-bit wrap).  r6
@ holds base (callee-saved); (k<<16) is recomputed before each BL (r0-r3
@ are caller-saved and clobbered by cpu_thumb2_read). ---
.Lop7c:
        IMM_FETCH_BYTE                  @ r0 = low byte of adr
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = high byte of adr
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = adr
        ldrh    r2, [r4, #CPU_X]
        add     r2, r2, r6             @ adr + X
        uxth    r6, r2                  @ r6 = base = (adr+X)&0xffff
        ldrb    r1, [r4, #CPU_K]
        lsl     r1, r1, #16
        orr     r1, r1, r6             @ lo addr = (k<<16)|base
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = lo byte; r6 survives
        push    {r0}                    @ stash lo
        ldrb    r1, [r4, #CPU_K]
        lsl     r1, r1, #16
        add     r2, r6, #1
        uxth    r2, r2                  @ (base+1)&0xffff
        orr     r1, r1, r2             @ hi addr = (k<<16)|((base+1)&0xffff)
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = hi byte
        lsl     r0, r0, #8
        pop     {r1}                    @ r1 = lo byte
        orr     r6, r1, r0             @ r6 = lo|(hi<<8)
        strh    r6, [r4, #CPU_PC]       @ pc = target
        b       .L_handled

@ --- 0xfc JSR (abs,X): same bank-K pointer fetch as 0x7c -> target, then
@ push pc-1 word, then pc=target.  Target fetched FIRST (before push). ---
.Lopfc:
        @ --- pointer fetch (identical to 0x7c) ---
        IMM_FETCH_BYTE                  @ r0 = low byte of adr
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = high byte of adr
        lsl     r0, r0, #8
        orr     r6, r6, r0              @ r6 = adr
        ldrh    r2, [r4, #CPU_X]
        add     r2, r2, r6
        uxth    r6, r2                  @ r6 = base = (adr+X)&0xffff
        ldrb    r1, [r4, #CPU_K]
        lsl     r1, r1, #16
        orr     r1, r1, r6
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = lo byte
        push    {r0}                    @ stash lo
        ldrb    r1, [r4, #CPU_K]
        lsl     r1, r1, #16
        add     r2, r6, #1
        uxth    r2, r2
        orr     r1, r1, r2
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = hi byte
        lsl     r0, r0, #8
        pop     {r1}                    @ r1 = lo byte
        orr     r6, r1, r0             @ r6 = target
        @ --- push pc-1, set pc=target ---
        push    {r6}                    @ stash target across PUSH_WORD
        ldrh    r6, [r4, #CPU_PC]       @ pc (past 2 operands)
        subs    r6, r6, #1              @ pc - 1
        PUSH_WORD                       @ push(pc-1)
        pop     {r6}                    @ restore target
        strh    r6, [r4, #CPU_PC]       @ pc = target
        b       .L_handled

@ --- 0x60 RTS: GATE on spBreakpoint -- the oracle's real condition
@ (cpu.c:1525) is `sp >= spBreakpoint && spBreakpoint`, NOT merely
@ `spBreakpoint != 0`: a breakpoint armed for a DEEPER, not-yet-reached
@ stack frame must not force every intervening RTS to C. Bail to C
@ (BEFORE any pull -- the C fallback runs the HookedFunctionRts gate)
@ only when both hold; otherwise pull word -> pc, pc+1 (uint16 wrap via
@ strh), same as before. sp/spBreakpoint are uint16_t promoted to a
@ non-negative int in C, so an unsigned ARM compare matches exactly. ---
.Lop60:
        ldrh    r0, [r4, #CPU_SPBREAKPOINT]
        cbz     r0, .Lop60_native       @ spBreakpoint == 0: never gates
        ldrh    r1, [r4, #CPU_SP]
        cmp     r1, r0                  @ sp - spBreakpoint (unsigned)
        blo     .Lop60_native           @ sp < spBreakpoint: not reached yet
        b       .L_noteb                @ sp >= spBreakpoint && spBreakpoint: gate fires
.Lop60_native:
        PULL_WORD                       @ r6 = return pc
        adds    r6, r6, #1              @ pc + 1
        strh    r6, [r4, #CPU_PC]
        b       .L_handled

@ --- 0x6b RTL: GATE on spBreakpoint (same two-condition test as RTS --
@ see its comment). Then pull word -> pc (+1), pull byte -> k. ---
.Lop6b:
        ldrh    r0, [r4, #CPU_SPBREAKPOINT]
        cbz     r0, .Lop6b_native       @ spBreakpoint == 0: never gates
        ldrh    r1, [r4, #CPU_SP]
        cmp     r1, r0                  @ sp - spBreakpoint (unsigned)
        blo     .Lop6b_native           @ sp < spBreakpoint: not reached yet
        b       .L_noteb                @ sp >= spBreakpoint && spBreakpoint: gate fires
.Lop6b_native:
        PULL_WORD                       @ r6 = return pc
        adds    r6, r6, #1              @ pc + 1
        strh    r6, [r4, #CPU_PC]
        PULL_BYTE                       @ r0 = k byte
        strb    r0, [r4, #CPU_K]
        b       .L_handled

@ --- 0x40 RTI: pull byte -> setFlags (SETFLAGS_TAIL reuses the PLP
@ renormalization), +1 cycle (native-mode extra), pull word -> pc (NO +1),
@ pull byte -> k.  No spBreakpoint gate (RTI has none in the oracle). ---
.Lop40:
        PULL_BYTE                       @ r0 = flags byte
        SETFLAGS_TAIL                   @ unpack + e/xf renormalization
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1              @ native mode: 1 extra cycle
        strb    r0, [r4, #CPU_CYCLESUSED]
        PULL_WORD                       @ r6 = return pc (NO +1)
        strh    r6, [r4, #CPU_PC]
        PULL_BYTE                       @ r0 = k byte
        strb    r0, [r4, #CPU_K]
        b       .L_handled

@ --- 0x02 COP: real software interrupt (unlike 0x00 BRK, which is a
@ CpuOpcodeHook redirect -- see the note by the dispatch table's missing
@ 0x00 entry; BRK is deliberately left in C, it can jump to an arbitrary
@ OTHER opcode's case body at runtime via goto restart, which a fixed
@ native handler cannot safely replicate).
@ COP (cpu.c:0x02): consume signature byte (RAW, discarded -- cpu_readOpcode),
@ push k (NATIVE only -- e-mode skips the PBR push), push pc (post-signature,
@ NOT pc-1), push flags (PHP's pack, .Lop08), +1 cycle (NATIVE only), i=1, d=0,
@ k=0, pc = HOOKED word at bank-0 0xffe4/0xffe5 native, 0xfff4/0xfff5 emulation
@ (cpu_readWord -> cpu_read, hooked -- matches JMP-ind's bank-0 pattern). ---
.Lop02:
        IMM_FETCH_BYTE                  @ r0 = signature byte, discarded; pc++ (raw)
        ldrb    r3, [r4, #CPU_E]
        cbnz    r3, .Lop02_pushpc       @ e-mode: skip PBR (k) push
        ldrb    r2, [r4, #CPU_K]
        PUSH_BYTE                       @ push(k)  (native only)
.Lop02_pushpc:
        ldrh    r6, [r4, #CPU_PC]       @ pc (post-signature; no -1)
        PUSH_WORD                       @ push(pc)
        @ --- pack flags (verbatim PHP, .Lop08) ---
        ldrb    r2, [r4, #CPU_N]
        lsl     r2, r2, #7
        ldrb    r1, [r4, #CPU_V]
        lsl     r1, r1, #6
        orr     r2, r1
        ldrb    r1, [r4, #CPU_MF]
        lsl     r1, r1, #5
        orr     r2, r1
        ldrb    r1, [r4, #CPU_XF]
        lsl     r1, r1, #4
        orr     r2, r1
        ldrb    r1, [r4, #CPU_D]
        lsl     r1, r1, #3
        orr     r2, r1
        ldrb    r1, [r4, #CPU_I]
        lsl     r1, r1, #2
        orr     r2, r1
        ldrb    r1, [r4, #CPU_Z]
        lsl     r1, r1, #1
        orr     r2, r1
        ldrb    r1, [r4, #CPU_C]
        orr     r2, r1
        PUSH_BYTE                       @ push(getFlags())
        ldrb    r3, [r4, #CPU_E]
        cbnz    r3, .Lop02_noextra      @ e-mode: no extra (PBR push) cycle
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1              @ native mode: 1 extra cycle
        strb    r0, [r4, #CPU_CYCLESUSED]
.Lop02_noextra:
        movs    r0, #1
        strb    r0, [r4, #CPU_I]        @ i = true
        movs    r0, #0
        strb    r0, [r4, #CPU_D]        @ d = false
        strb    r0, [r4, #CPU_K]        @ k = 0
        @ vector: native 0xffe4/0xffe5, emulation 0xfff4/0xfff5
        ldrb    r3, [r4, #CPU_E]
        movw    r6, #0xffe4
        cbz     r3, .Lop02_vec
        movw    r6, #0xfff4
.Lop02_vec:
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read         @ r0 = vector low byte (HOOKED, bank 0)
        push    {r0}
        mov     r0, r4
        adds    r1, r6, #1
        bl      cpu_thumb2_read         @ r0 = vector high byte
        lsl     r0, r0, #8
        pop     {r1}
        orr     r0, r0, r1
        strh    r0, [r4, #CPU_PC]       @ pc = vector
        b       .L_handled


@ ===========================================================================
@ Stage 3M -- stack-relative opcodes (Family D): sr,S (8 ops) + (sr,S),Y (8 ops).
@
@ sr,S handlers are the VERBATIM DP ancestor (Stage 3D) with DP_ADDR replaced
@ by SR_ADDR: the ALU maths, SETZN, STA write-back and ADC/SBC maths are byte-
@ identical because cpu_adrSr feeds the SAME cpu_ora/cpu_and/... helpers as
@ cpu_adrDp (cpu.c:936+).  sr data reads use DP_READ_VALUE (16-bit high-wrap
@ via uxth, bank 0 -- like DP).  The ONLY new semantics live in SR_ADDR: base
@ is CPU_SP (NOT CPU_DP) and NO dp&0xff cycle charge (cpu_adrSr adds none).
@
@ (sr,S),Y handlers are the VERBATIM IDY ancestor (Stage 3G Pass 1) with
@ IDY_ADDR replaced by ISY_ADDR: the ALU maths, SETZN, STA write-back and
@ ADC/SBC maths are byte-identical because cpu_adrIsy feeds the SAME helpers
@ as cpu_adrIdy (cpu.c:1031+).  isy data reads use ABS_READ_VALUE (24-bit
@ high-wrap via ubfx -- like IDY).  The new semantics live in ISY_ADDR:
@ pointer base is CPU_SP (NOT CPU_DP), NO dp&0xff cycle, and NO write penalty
@ (even STA isy 0x93 pays nothing -- cpu_adrIsy has no write param, unlike
@ cpu_adrIdy's).  ADC/SBC bail to .L_noteb BEFORE the operand fetch when D set.
@ ===========================================================================

@ ---------------- sr,S : 0x03 ORA 0x23 AND 0x43 EOR 0x63 ADC 0x83 STA 0xa3 LDA 0xc3 CMP 0xe3 SBC

.Lop03:                                 @ ORA sr  (read)
        SR_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_sr_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_sr_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop23:                                 @ AND sr  (read)
        SR_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_sr_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_sr_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop43:                                 @ EOR sr  (read)
        SR_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_sr_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_sr_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopa3:                                 @ LDA sr
        SR_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_sr_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8          @ a = (a & 0xff00) | value
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                  @ SETZN value = a & 0xff
        movs    r1, #1                  @ byte mode
        SETZN
        b       .L_handled
.Llda_sr_w:
        strh    r2, [r4, #CPU_A]        @ a = value (word)
        movs    r1, #0                  @ word mode
        SETZN
        b       .L_handled

.Lopc3:                                 @ CMP sr  (sets C/Z/N; A unchanged)
        SR_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_sr_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3                  @ a & 0xff
        mvn     r2, r2                  @ value ^ 0xff (low byte)
        uxtb    r2, r2
        add     r2, r3, r2              @ (a&0xff) + (value^0xff)
        adds    r2, r2, #1             @ +1 -> result (0..0x1ff)
        lsr     r3, r2, #8             @ c = result > 0xff (result>>8 is 0/1)
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2                  @ SETZN value = result & 0xff
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_sr_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2                  @ value ^ 0xffff
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1             @ result (0..0x1ffff)
        lsr     r3, r2, #16            @ c = result > 0xffff (result>>16 is 0/1)
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2                  @ SETZN value = result & 0xffff
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop83:                                 @ STA sr  (MF width; writes via bridge)
        SR_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_sr_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ a & 0xff
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_sr_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ low byte
        mov     r0, r4
        mov     r1, r6                  @ addr_low
        bl      cpu_thumb2_write
        add     r1, r6, #1
        uxth    r1, r1                  @ addr_high (16-bit wrap, bank 0)
        ldrh    r2, [r4, #CPU_A]        @ re-read a (clobbered by the BL)
        lsr     r2, r2, #8             @ high byte
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop63:                                 @ ADC sr  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        SR_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_sr_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a (16-bit; high byte kept for writeback)
        ldrb    r1, [r4, #CPU_C]         @ r1 = c (0/1)
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3                   @ r0 = a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ (full a) ^ value -- bit7 of low byte is the sign
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7 (0/1)
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3                   @ SETZN value = a & 0xff
        movs    r1, #1                   @ byte mode
        SETZN
        b       .L_handled
.Ladc_sr_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]         @ r3 = a
        ldrb    r1, [r4, #CPU_C]         @ r1 = c
        mov     r0, r2                   @ r0 = value (preserve for V XOR)
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15 (0/1)
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2                   @ SETZN value = result & 0xffff
        movs    r1, #0                   @ word mode
        SETZN
        b       .L_handled

.Lope3:                                 @ SBC sr  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        SR_ADDR
        DP_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_sr_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = complemented value
        uxtb    r0, r3                   @ a & 0xff
        add     r2, r0, r6               @ (a&0xff) + value
        add     r2, r2, r1               @ + c -> result (0..0x1ff)
        lsr     r1, r2, #8               @ c = result > 0xff
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6               @ a ^ value
        eor     r1, r6, r2               @ value ^ result
        bic     r0, r1, r0               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #7, #1           @ V = bit7
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8           @ a = (a & 0xff00) | (result & 0xff)
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_sr_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2                   @ ~read; low 16 bits = read ^ 0xffff
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2                   @ r0 = complemented value
        add     r2, r3, r0               @ a + value
        add     r2, r2, r1               @ + c -> result (0..0x1ffff)
        lsr     r1, r2, #16              @ c = result > 0xffff
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0               @ a ^ value
        eor     r1, r0, r2               @ value ^ result
        bic     r0, r1, r3               @ (value^result) & ~(a^value)
        ubfx    r0, r0, #15, #1          @ V = bit15
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]         @ a = result (16-bit)
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled


@ ---------------- (sr,S),Y : 0x13 ORA 0x33 AND 0x53 EOR 0x73 ADC 0x93 STA 0xb3 LDA 0xd3 CMP 0xf3 SBC

.Lop13:                                 @ ORA isy  (read)
        ISY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lora_isy_w
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lora_isy_w:
        ldrh    r3, [r4, #CPU_A]
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop33:                                 @ AND isy  (read)
        ISY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Land_isy_w
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Land_isy_w:
        ldrh    r3, [r4, #CPU_A]
        and     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop53:                                 @ EOR isy  (read)
        ISY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Leor_isy_w
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Leor_isy_w:
        ldrh    r3, [r4, #CPU_A]
        eor     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopb3:                                 @ LDA isy  (read)
        ISY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Llda_isy_w
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Llda_isy_w:
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopd3:                                 @ CMP isy  (read; sets C/Z/N; A unchanged)
        ISY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lcmp_isy_w
        ldrh    r3, [r4, #CPU_A]
        uxtb    r3, r3
        mvn     r2, r2
        uxtb    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #8
        strb    r3, [r4, #CPU_C]
        uxtb    r2, r2
        movs    r1, #1
        SETZN
        b       .L_handled
.Lcmp_isy_w:
        ldrh    r3, [r4, #CPU_A]
        mvn     r2, r2
        uxth    r2, r2
        add     r2, r3, r2
        adds    r2, r2, #1
        lsr     r3, r2, #16
        strb    r3, [r4, #CPU_C]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lop93:                                 @ STA isy  (MF width; NO write penalty; writes via bridge)
        ISY_ADDR
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsta_isy_w
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        b       .L_handled
.Lsta_isy_w:
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        ldrh    r2, [r4, #CPU_A]
        uxtb    r2, r2
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_write
        add     r1, r6, #1
        ubfx    r1, r1, #0, #24
        ldrh    r2, [r4, #CPU_A]
        lsr     r2, r2, #8
        mov     r0, r4
        bl      cpu_thumb2_write
        b       .L_handled

.Lop73:                                 @ ADC isy  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ISY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Ladc_isy_w
        @ --- byte: r2 = value (0..0xff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2                   @ r6 = value (preserve for V XOR)
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Ladc_isy_w:
        @ --- word: r2 = value (0..0xffff) ---
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

.Lopf3:                                 @ SBC isy  (binary; decimal -> C fallback)
        ldrb    r0, [r4, #CPU_D]
        cmp     r0, #0
        bne     .L_noteb
        ISY_ADDR
        ABS_READ_VALUE CPU_MF
        ldrb    r1, [r4, #CPU_MF]
        cbz     r1, .Lsbc_isy_w
        @ --- byte: complement operand first (value = read ^ 0xff) ---
        eor     r2, r2, #0xff
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r6, r2
        uxtb    r0, r3
        add     r2, r0, r6
        add     r2, r2, r1
        lsr     r1, r2, #8
        strb    r1, [r4, #CPU_C]
        eor     r0, r3, r6
        eor     r1, r6, r2
        bic     r0, r1, r0
        ubfx    r0, r0, #7, #1
        strb    r0, [r4, #CPU_V]
        bfi     r3, r2, #0, #8
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r3
        movs    r1, #1
        SETZN
        b       .L_handled
.Lsbc_isy_w:
        @ --- word: complement operand (value = read ^ 0xffff) ---
        mvn     r2, r2
        uxth    r2, r2
        ldrh    r3, [r4, #CPU_A]
        ldrb    r1, [r4, #CPU_C]
        mov     r0, r2
        add     r2, r3, r0
        add     r2, r2, r1
        lsr     r1, r2, #16
        strb    r1, [r4, #CPU_C]
        eor     r3, r3, r0
        eor     r1, r0, r2
        bic     r0, r1, r3
        ubfx    r0, r0, #15, #1
        strb    r0, [r4, #CPU_V]
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        movs    r1, #0
        SETZN
        b       .L_handled

@ ===========================================================================
@ Stage 3N: 9 remaining native opcodes (WDM WAI STP XCE MVP MVN PER PEI PEA).
@ Operand bytes are RAW (IMM_FETCH_BYTE = cpu_readOpcode = cpu_read_raw, NO
@ spin hook); data and stack accesses are HOOKED (cpu_thumb2_read/write =
@ cpu_read/cpu_write).  r6 is the only state carried across BLs (callee-saved).
@ ===========================================================================

@ --- 0x42 WDM: 2-byte NOP.  Read & discard 1 operand byte (RAW).  cpu.c:1317.
.Lop42:
        IMM_FETCH_BYTE                  @ read & discard operand; pc++
        b       .L_handled

@ --- 0xcb WAI: set waiting, overwrite cyclesUsed=3 (NOT add).  cpu.c:2199.
@ The runOpcode pre-work (cpu.c:226) has already cleared waiting before the
@ opcode is fetched; the handler sets it again so the NEXT runOpcode blocks.
.Lopcb:
        movs    r0, #1
        strb    r0, [r4, #CPU_WAITING]
        movs    r0, #3
        strb    r0, [r4, #CPU_CYCLESUSED]  @ OVERWRITE base cycle, not add
        b       .L_handled

@ --- 0xdb STP: set stopped (freeze).  cpu.c:2298.
.Lopdb:
        movs    r0, #1
        strb    r0, [r4, #CPU_STOPPED]
        b       .L_handled

@ --- 0xfb XCE: swap C and E, then cpu_setFlags(cpu, cpu_getFlags(cpu)).
@ getFlags packs N<<7|V<<6|MF<<5|XF<<4|D<<3|I<<2|Z<<1|C (each field 0/1).
@ SETFLAGS_TAIL unpacks r0 and re-normalises using the POST-swap E (read fresh),
@ matching cpu_setFlags.  cpu.c:2490.
.Lopfb:
        ldrb    r0, [r4, #CPU_C]
        ldrb    r1, [r4, #CPU_E]
        strb    r1, [r4, #CPU_C]          @ c = old e
        strb    r0, [r4, #CPU_E]          @ e = old c
        @ build flags byte into r0 from current (post-swap) fields
        ldrb    r1, [r4, #CPU_N]
        lsl     r0, r1, #7
        ldrb    r1, [r4, #CPU_V]
        lsl     r1, r1, #6
        orr     r0, r0, r1
        ldrb    r1, [r4, #CPU_MF]
        lsl     r1, r1, #5
        orr     r0, r0, r1
        ldrb    r1, [r4, #CPU_XF]
        lsl     r1, r1, #4
        orr     r0, r0, r1
        ldrb    r1, [r4, #CPU_D]
        lsl     r1, r1, #3
        orr     r0, r0, r1
        ldrb    r1, [r4, #CPU_I]
        lsl     r1, r1, #2
        orr     r0, r0, r1
        ldrb    r1, [r4, #CPU_Z]
        lsl     r1, r1, #1
        orr     r0, r0, r1
        ldrb    r1, [r4, #CPU_C]          @ post-swap c
        orr     r0, r0, r1
        SETFLAGS_TAIL                    @ unpack + renormalise (reads E fresh)
        b       .L_handled

@ --- 0x44 MVP: block move (src->dest), a/x/y--.  cpu.c:1327.
@ dest=readOpcode, src=readOpcode (RAW); db=dest; write[(dest<<16)|y] =
@ read[(src<<16)|x] (HOOKED); a--,x--,y--; if(a!=0xffff) pc-=3; if(xf) x&=0xff,
@ y&=0xff.  r6 carries dest across both BLs; the read value is spilled to the
@ ARM stack across the write-addr computation.
.Lop44:
        IMM_FETCH_BYTE                  @ r0 = dest
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = src
        strb    r6, [r4, #CPU_DB]       @ db = dest
        lsl     r3, r0, #16             @ src << 16
        ldrh    r1, [r4, #CPU_X]
        orr     r1, r3, r1             @ read_addr = (src<<16) | x
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = value (HOOKED); r6=dest survives
        push    {r0}                    @ save value
        lsl     r3, r6, #16             @ dest << 16
        ldrh    r1, [r4, #CPU_Y]
        orr     r1, r3, r1             @ write_addr = (dest<<16) | y
        mov     r0, r4
        pop     {r2}                    @ r2 = value
        bl      cpu_thumb2_write        @ write (HOOKED)
        ldrh    r0, [r4, #CPU_A]
        subs    r0, r0, #1
        strh    r0, [r4, #CPU_A]
        ldrh    r0, [r4, #CPU_X]
        subs    r0, r0, #1
        strh    r0, [r4, #CPU_X]
        ldrh    r0, [r4, #CPU_Y]
        subs    r0, r0, #1
        strh    r0, [r4, #CPU_Y]
        ldrh    r0, [r4, #CPU_A]        @ reload truncated 16-bit a
        movw    r1, #0xffff
        cmp     r0, r1
        beq     91f                     @ a == 0xffff: no rewind
        ldrh    r0, [r4, #CPU_PC]
        subs    r0, r0, #3
        strh    r0, [r4, #CPU_PC]
91:
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, 92f
        ldrh    r0, [r4, #CPU_X]
        uxtb    r0, r0
        strh    r0, [r4, #CPU_X]
        ldrh    r0, [r4, #CPU_Y]
        uxtb    r0, r0
        strh    r0, [r4, #CPU_Y]
92:
        b       .L_handled

@ --- 0x54 MVN: block move (src->dest), a--, x++, y++.  cpu.c:1435.
@ Identical to MVP except X and Y are INCREMENTED (and the high half of the
@ pointer banks still come from the operand bytes, not db).
.Lop54:
        IMM_FETCH_BYTE                  @ r0 = dest
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = src
        strb    r6, [r4, #CPU_DB]       @ db = dest
        lsl     r3, r0, #16             @ src << 16
        ldrh    r1, [r4, #CPU_X]
        orr     r1, r3, r1             @ read_addr = (src<<16) | x
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = value (HOOKED); r6=dest survives
        push    {r0}                    @ save value
        lsl     r3, r6, #16             @ dest << 16
        ldrh    r1, [r4, #CPU_Y]
        orr     r1, r3, r1             @ write_addr = (dest<<16) | y
        mov     r0, r4
        pop     {r2}                    @ r2 = value
        bl      cpu_thumb2_write        @ write (HOOKED)
        ldrh    r0, [r4, #CPU_A]
        subs    r0, r0, #1
        strh    r0, [r4, #CPU_A]
        ldrh    r0, [r4, #CPU_X]
        adds    r0, r0, #1             @ MVN: x++
        strh    r0, [r4, #CPU_X]
        ldrh    r0, [r4, #CPU_Y]
        adds    r0, r0, #1             @ MVN: y++
        strh    r0, [r4, #CPU_Y]
        ldrh    r0, [r4, #CPU_A]        @ reload truncated 16-bit a
        movw    r1, #0xffff
        cmp     r0, r1
        beq     93f                     @ a == 0xffff: no rewind
        ldrh    r0, [r4, #CPU_PC]
        subs    r0, r0, #3
        strh    r0, [r4, #CPU_PC]
93:
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, 94f
        ldrh    r0, [r4, #CPU_X]
        uxtb    r0, r0
        strh    r0, [r4, #CPU_X]
        ldrh    r0, [r4, #CPU_Y]
        uxtb    r0, r0
        strh    r0, [r4, #CPU_Y]
94:
        b       .L_handled

@ --- 0x62 PER: push (pc + sign-extended operand word).  cpu.c:1541.
@ operand word is RAW (2 IMM_FETCH_BYTE); pc has advanced past both bytes, so
@ CPU_PC holds pc+2.  sxth sign-extends the displacement; PUSH_WORD pushes the
@ low 16 bits of the sum (HIGH byte first).
.Lop62:
        IMM_FETCH_BYTE                  @ r0 = lo
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = hi
        lsl     r0, r0, #8
        orr     r6, r6, r0             @ r6 = word
        sxth    r6, r6                  @ (int16_t) disp
        ldrh    r0, [r4, #CPU_PC]       @ pc (past both operand bytes)
        add     r6, r0, r6             @ pc + disp (low 16 bits pushed)
        PUSH_WORD
        b       .L_handled

@ --- 0xd4 PEI: push indirect-pointer word from direct page.  cpu.c:2255.
@ DP_ADDR computes r6 = (dp+off)&0xffff (bank 0) and charges the dp&0xff cycle.
@ Two HOOKED reads at r6 and (r6+1)&0xffff build the pointer; PUSH_WORD pushes it.
.Lopd4:
        DP_ADDR                         @ r6 = (dp+off)&0xffff, bank 0
        mov     r0, r4
        mov     r1, r6
        bl      cpu_thumb2_read         @ r0 = ptr_lo (HOOKED); r6 survives
        push    {r0}                    @ save ptr_lo
        add     r1, r6, #1
        uxth    r1, r1                  @ ptr_hi addr = (r6+1)&0xffff (bank 0)
        mov     r0, r4
        bl      cpu_thumb2_read         @ r0 = ptr_hi (HOOKED)
        lsl     r0, r0, #8
        pop     {r1}                    @ r1 = ptr_lo
        orr     r6, r1, r0             @ r6 = pointer
        PUSH_WORD
        b       .L_handled

@ --- 0xf4 PEA: push effective address (raw operand word).  cpu.c:2444.
@ operand word is RAW; if it equals the 0xC2AE sentinel Y is cleared first
@ (matches the C hack); then PUSH_WORD.
.Lopf4:
        IMM_FETCH_BYTE                  @ r0 = lo
        mov     r6, r0
        IMM_FETCH_BYTE                  @ r0 = hi
        lsl     r0, r0, #8
        orr     r6, r6, r0             @ r6 = word
        movw    r0, #0xc2ae
        cmp     r6, r0
        bne     95f
        movs    r0, #0
        strh    r0, [r4, #CPU_Y]       @ y = 0 hack
95:
        PUSH_WORD
        b       .L_handled

        .size   snes_thumb2_try, .-snes_thumb2_try
        .size   snes_thumb2_step, .-snes_thumb2_step

@ Keep the Stage 0 probe: it is still a build-path smoke test (offset header
@ resolves inside assembly). Unreferenced under release, so GC'd at link.
@ ===========================================================================
@ Batch entry: run_dots' per-opcode loop, inside the engine.
@
@ int snes_thumb2_batch(Cpu *cpu, SnesBatch *b)   b = {dots, apud, hpos, Snes*}
@
@ run_dots ran ONE opcode per call: the C loop's own bookkeeping, a BL into
@ snes_thumb2_run, its push/pop, its return -- every opcode.  This entry keeps
@ the loop here for as long as consecutive opcodes complete strictly inside
@ the span, doing per opcode exactly what run_dots does, in the same order:
@   store hPos and apuDotsAccum (an opcode can read both through the bus),
@   the H-IRQ timer match (apply_irq_match),
@   the entry checks of snes_thumb2_run (STP, WAI, NMI, a takeable IRQ, rc),
@   cpuMemOps = 0, fetch, dispatch, the handler (or the C fallback),
@   charge = cycles*6 + memOps*2 as a uint8, reload apuDotsAccum,
@   then continue only if no DMA started, the charge is >= 2 and it ends
@   before the span does (hPos/apuDotsAccum += charge, dots -= charge).
@ It returns -1 with nothing run when an entry check fails (C runs that
@ opcode the ordinary way; the IRQ match it repeats is idempotent), or the
@ charge of the last opcode run when that one starts a DMA or does not end
@ inside the span: C then applies its step exactly as before.  The span
@ state is written back to *b either way.
@
@ Registers across the loop (callee-saved, untouched by every handler and by
@ every C function they call): r7 = Snes*, r8 = dots, r9 = hPos, r10 =
@ apuDotsAccum, r11 = Dma*; r4 = Cpu* and r5 = 3 (the batch contract) as the
@ handlers expect.
@ ===========================================================================
        .global snes_thumb2_batch
        .type   snes_thumb2_batch,%function
        .thumb_func
snes_thumb2_batch:
        push    {r1, r4, r5, r6, r7, r8, r9, r10, r11, lr}   @ 40 bytes, 8-aligned; b at [sp]
        mov     r4, r0
        ldr     r8, [r1, #0]            @ dots
        ldr     r10, [r1, #4]           @ apuDotsAccum
        ldrh    r9, [r1, #8]            @ hPos
        ldr     r7, [r1, #12]           @ Snes*
        ldr     r11, [r7, #SNES_DMA]    @ Dma*
        movs    r5, #3                  @ return contract: batch
.Lb_top:
        strh    r9, [r7, #SNES_HPOS]
        str     r10, [r7, #SNES_APUDOTS]
        @ apply_irq_match: needs H-IRQ on, the V test passing if V-IRQ is on,
        @ and hPos == hTimer*4; then inIrq = irqWanted = true
        ldrb    r0, [r7, #SNES_HIRQEN]
        ldrb    r1, [r7, #SNES_VIRQEN]
        cbz     r0, .Lb_noirq           @ no H-IRQ: nothing to match (V alone returns too)
        cbz     r1, .Lb_hmatch
        ldrh    r2, [r7, #SNES_VPOS]
        ldrh    r3, [r7, #SNES_VTIMER]
        cmp     r2, r3
        bne     .Lb_noirq
.Lb_hmatch:
        ldrh    r2, [r7, #SNES_HTIMER]
        cmp     r9, r2, lsl #2
        bne     .Lb_noirq
        movs    r2, #1
        strb    r2, [r7, #SNES_INIRQ]
        strb    r2, [r4, #CPU_IRQWANTED]
.Lb_noirq:
        @ the entry checks of snes_thumb2_run
        ldrb    r1, [r4, #CPU_STOPPED]
        ldrb    r2, [r4, #CPU_WAITING]
        orrs    r1, r1, r2
        ldrb    r2, [r4, #CPU_NMIWANTED]
        orrs    r1, r1, r2
        ldrb    r2, [r4, #CPU_IRQWANTED]
        ldrb    r3, [r4, #CPU_I]
        bics    r2, r2, r3              @ irqWanted && !i
        orrs    r1, r1, r2
        bne     .Lb_exit_noop
        movw    r2, #:lower16:g_rc_active
        movt    r2, #:upper16:g_rc_active
        ldrb    r2, [r2]
        cmp     r2, #0
        bne     .Lb_exit_noop
        movs    r0, #0
        strb    r0, [r7, #SNES_CPUMEMOPS]   @ cpuMemOps = 0
        mov     r0, r7                  @ fetch, exactly as snes_thumb2_run
        ldrb    r1, [r4, #CPU_K]
        ldrh    r2, [r4, #CPU_PC]
        lsl     r1, r1, #16
        orr     r1, r1, r2
        adds    r3, r2, #1
        strh    r3, [r4, #CPU_PC]       @ uint16_t wrap matches cpu->pc++
        ROM_FETCH_FAST
        uxtb    r6, r0
        movw    r2, #:lower16:snes_cycles_per_opcode
        movt    r2, #:upper16:snes_cycles_per_opcode
        ldrb    r3, [r2, r6]
        strb    r3, [r4, #CPU_CYCLESUSED]
        b       .Ldispatch

.Lb_after:                              @ every handler of a batch opcode ends here
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        ldrb    r1, [r7, #SNES_CPUMEMOPS]
        add     r0, r0, r0, lsl #1      @ cycles*3
        add     r0, r0, r1              @ + memOps
        lsls    r0, r0, #1              @ cycles*6 + memOps*2
        uxtb    r0, r0                  @ the uint8 cpuCyclesLeft
        ldr     r10, [r7, #SNES_APUDOTS]    @ the opcode's APU catch-up may have zeroed it
        ldrb    r1, [r11, #DMA_BUSY]
        cbnz    r1, .Lb_out             @ DMA started: C takes the step
        ldrh    r1, [r11, #DMA_HDMATIMER]
        cbnz    r1, .Lb_out
        cmp     r0, #2
        blo     .Lb_out                 @ (never: a charge is >= 6) C's else-branch
        cmp     r0, r8
        bge     .Lb_out                 @ does not end strictly inside the span
        add     r10, r10, r0
        add     r9, r9, r0
        uxth    r9, r9
        sub     r8, r8, r0
        b       .Lb_top
.Lb_exit_noop:
        mov     r0, #-1
.Lb_out:
        ldr     r1, [sp]
        str     r8, [r1, #0]
        str     r10, [r1, #4]
        strh    r9, [r1, #8]
        pop     {r1, r4, r5, r6, r7, r8, r9, r10, r11, pc}

@ The C finish of an opcode the engine does not handle (run and batch).
.L_noteb_c:
        mov     r0, r4
        mov     r1, r6                  @ the byte already fetched; do NOT refetch
        bl      cpu_thumb2_fallback     @ r5 is callee-saved: still the contract
        cmp     r5, #3
        beq     .Lb_after
        pop     {r4, r5, r6, pc}

        .section .text.snes_thumb2_probe,"ax",%progbits
        .align  2
        .global snes_thumb2_probe
        .type   snes_thumb2_probe,%function
        .thumb_func
snes_thumb2_probe:
        ldrh    r1, [r0, #CPU_PC]
        movs    r0, #0
        bx      lr
        .size   snes_thumb2_probe, .-snes_thumb2_probe

        .section        .note.GNU-stack,"",%progbits
