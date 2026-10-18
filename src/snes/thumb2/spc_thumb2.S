@ Cortex-M7 Thumb-2 SPC700 engine -- native interpreter for the APU's spc700.
@
@ spc_thumb2_run(Spc *spc, int budget) -> int (the budget left)
@   Runs whole SPC700 opcodes from apu->cycles, the per-opcode part of
@   apu_run (apu.c) moved inside the engine so it keeps its registers across
@   opcodes. After each opcode of n cycles started at apu->cycles:
@     n > budget left   -> apu->cpuCyclesLeft = n, return (apu_run consumes it)
@     otherwise         -> the DSP ticks of [start, start+n) (apu_dsp_ticks),
@                          apu->cycles += n, budget -= n; loop while budget > 0
@   It also returns, BEFORE running it, at the idle-wait pattern
@   (EC FD 00 F0 FB, apu_idleSkipCycles), and after a STOP/SLEEP.
@   budget == 0 runs exactly one opcode and leaves it to the caller
@   (spc_runOpcode, the single step apu_run takes at those exits): its cycles
@   in cyclesUsed and apu->cpuCyclesLeft, nothing committed.
@
@ Contract: the caller has checked spc->stopped. Each opcode: one fetch
@ (pc++), the base-cycle charge from spc_cycles_per_opcode into cyclesUsed,
@ the handler, which performs exactly the operand/data fetches and extra
@ cycle charges of its opcode. apu->cycles holds the opcode's start while it
@ runs, which is what the lazy timers read (apu_timers_sync).
@
@ Memory model: apu->ram is a flat 64 KB array at Apu offset 8.  Addresses
@ outside [0xf0,0xff] (IO) and [0xff00,0xffff] (boot ROM range) are served
@ directly from ram -- no function call.  IO and boot-ROM accesses fall
@ through to bl apu_cpuRead / apu_cpuWrite.
@
@ Every handler is bit-exact with spc.c's spc_doOpcode; opcodes without one
@ call spc_fallback (spc_doOpcode) from inside the loop.  The hybrid model
@ guarantees STATEHASH / AUDIOHASH bit-identity.
@
@ AAPCS: r0-r3, r12, lr caller-saved (clobbered by BL); r4-r11 callee-saved.
@ Register assignment: r4 = Spc*, r5 = ram_base (apu->ram), r6 = opcode,
@ r7 = Apu* (for C bridge args), r8/r9 = scratch callee-saved temps,
@ r10 = &apu->cycles, r11 = budget left (0 = single opcode).

#include "spc_thumb2_offsets.h"

        .syntax unified
        .cpu    cortex-m7
        .thumb

@ ===========================================================================
@ Macros
@ ===========================================================================

@ setZN for an 8-bit value in r0. Writes SPC_OFF_Z (0/1) and SPC_OFF_N (0/1),
@ matching the C _Bool normalisation of z = (value==0); n = (value&0x80).
        .macro SETZN
        cmp     r0, #0
        ite     eq
        moveq   r3, #1
        movne   r3, #0
        strb    r3, [r4, #SPC_OFF_Z]
        lsrs    r3, r0, #7
        strb    r3, [r4, #SPC_OFF_N]
        .endm

@ Fast memory read.  Expects r1 = address (uint16_t).  Returns r0 = byte.
@ Clobbers r0, r3 (and r0-r3 in the BL slow path).  r5 = ram_base, r7 = Apu*.
@ IO region [0x00f0,0x00ff] and boot-ROM [0xffc0,0xffff] go through C bridge.
@ Everything else is a direct ldrb from ram.
        .macro READ_FAST
        cmp     r1, #0x100
        blo     .Lrf_io\@
        cmp     r1, #0xff00
        blo     .Lrf_ram\@
        b       .Lrf_slow\@
.Lrf_io\@:
        cmp     r1, #0xf0
        blo     .Lrf_ram\@
.Lrf_slow\@:
        mov     r0, r7
        bl      apu_cpuRead
        b       .Lrf_done\@
.Lrf_ram\@:
        ldrb    r0, [r5, r1]
.Lrf_done\@:
        .endm

@ Fast memory write.  Expects r1 = address, r2 = value.  Clobbers r0, r3.
@ IO writes MUST go through C (register side effects); non-IO writes store
@ directly to ram (which is all apu_cpuWrite does for those addresses).
        .macro WRITE_FAST
        cmp     r1, #0x100
        blo     .Lwf_io\@
        cmp     r1, #0xff00
        blo     .Lwf_ram\@
        b       .Lwf_slow\@
.Lwf_io\@:
        cmp     r1, #0xf0
        blo     .Lwf_ram\@
.Lwf_slow\@:
        mov     r0, r7
        bl      apu_cpuWrite
        b       .Lwf_done\@
.Lwf_ram\@:
        strb    r2, [r5, r1]
.Lwf_done\@:
        .endm

@ Fetch one byte from [pc], increment pc (uint16_t wrap).  Returns r0 = byte.
        .macro FETCH_BYTE
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r3, r1, #1
        strh    r3, [r4, #SPC_OFF_PC]
        READ_FAST
        .endm

@ Fetch a 16-bit word from [pc], pc += 2.  Returns r0 = word.  Uses r8.
        .macro FETCH_WORD
        FETCH_BYTE
        mov     r8, r0
        FETCH_BYTE
        orr     r0, r8, r0, lsl #8
        .endm

@ Direct-page addressing: fetch offset byte, addr = offset | (p << 8).
@ Returns r1 = address.
        .macro DP_ADDR
        FETCH_BYTE
        ldrb    r3, [r4, #SPC_OFF_P]
        lsl     r3, r3, #8
        orr     r1, r0, r3
        .endm

@ Absolute addressing: fetch word operand.  Returns r1 = address.
        .macro ABS_ADDR
        FETCH_WORD
        mov     r1, r0
        .endm

@ Direct-page + X: ((fetch + x) & 0xff) | (p << 8).  Returns r1 = address.
@ Spc.c: offset byte wraps in 0..0xff range before OR with page.
        .macro DPX_ADDR
        FETCH_BYTE
        ldrb    r2, [r4, #SPC_OFF_X]
        adds    r0, r0, r2
        ldrb    r2, [r4, #SPC_OFF_P]
        lsl     r2, r2, #8
        orr     r1, r0, r2
        .endm

@ Absolute + X: (fetchWord + x) & 0xffff.  Returns r1 = address.
        .macro ABX_ADDR
        FETCH_WORD
        ldrb    r2, [r4, #SPC_OFF_X]
        adds    r1, r0, r2
        uxth    r1, r1
        .endm

@ Absolute + Y: (fetchWord + y) & 0xffff.  Returns r1 = address.
        .macro ABY_ADDR
        FETCH_WORD
        ldrb    r2, [r4, #SPC_OFF_Y]
        adds    r1, r0, r2
        uxth    r1, r1
        .endm

@ Indirect + Y: pointer = fetch; adr = readWord(pointer|p<<8, ((pointer+1)&0xff)|p<<8);
@              return (adr + y) & 0xffff.  Returns r1 = address.  Clobbers r0,r2,r3,r8,r9.
@ r8/r9 are callee-saved so they survive READ_FAST's BL.
        .macro IDY_ADDR
        FETCH_BYTE                       @ r0 = pointer
        ldrb    r3, [r4, #SPC_OFF_P]
        lsl     r3, r3, #8               @ r3 = p << 8
        orr     r8, r0, r3               @ r8 = low-byte read address
        adds    r2, r0, #1
        uxtb    r2, r2                   @ (pointer+1) & 0xff
        orr     r9, r2, r3               @ r9 = high-byte read address
        mov     r1, r8
        READ_FAST                        @ r0 = low byte
        mov     r8, r0                   @ r8 = low byte (park in callee-saved)
        mov     r1, r9
        READ_FAST                        @ r0 = high byte
        lsl     r0, r0, #8
        orr     r0, r0, r8               @ r0 = word
        ldrb    r2, [r4, #SPC_OFF_Y]
        adds    r1, r0, r2
        uxth    r1, r1
        .endm

@ ADC with value in r8.  Clobbers r0-r3, r9.  Updates v/h/c flags, a, setZN(a).
@ result = a + value + c
@ v = (~(a^value) & (value^result)) & 0x80
@ h = ((a&0xf) + (value&0xf) + c_orig) > 0xf
@ c = result > 0xff
        .macro ADC_SPC
        ldrb    r0, [r4, #SPC_OFF_A]
        ldrb    r1, [r4, #SPC_OFF_C]
        add     r3, r0, r8
        add     r3, r3, r1
        lsr     r2, r3, #8
        strb    r2, [r4, #SPC_OFF_C]
        and     r9, r0, #0xf
        and     r2, r8, #0xf
        add     r9, r9, r2
        add     r9, r9, r1
        cmp     r9, #0xf
        ite     hi
        movhi   r9, #1
        movls   r9, #0
        strb    r9, [r4, #SPC_OFF_H]
        eor     r9, r0, r8
        mvn     r9, r9
        eor     r2, r8, r3
        and     r9, r9, r2
        tst     r9, #0x80
        ite     ne
        movne   r9, #1
        moveq   r9, #0
        strb    r9, [r4, #SPC_OFF_V]
        uxtb    r0, r3
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        .endm

@ CMP with value in r8, register offset \off.  Clobbers r0-r3.
@ value ^= 0xff; result = reg + value + 1; c = result > 0xff; setZN(result).
@ Does NOT affect v or h.
        .macro CMP_REG off
        ldrb    r0, [r4, #\off]
        eor     r8, r8, #0xff
        add     r3, r0, r8
        add     r3, r3, #1
        lsr     r1, r3, #8
        strb    r1, [r4, #SPC_OFF_C]
        uxtb    r0, r3
        SETZN
        .endm

@ Stack push/pull.  Stack is always at 0x100|sp which is [0x100,0x1ff] --
@ always plain RAM, never IO or boot ROM, so direct ldrb/strb is safe.
@ pushByte: value in r0.  write(0x100|sp, val); sp--.
        .macro PUSH_BYTE
        ldrb    r2, [r4, #SPC_OFF_SP]
        mov     r1, #0x100
        orr     r1, r1, r2
        strb    r0, [r5, r1]
        ldrb    r2, [r4, #SPC_OFF_SP]
        subs    r2, r2, #1
        strb    r2, [r4, #SPC_OFF_SP]
        .endm

@ pullByte: returns r0 = byte.  sp++; read(0x100|sp).
        .macro PULL_BYTE
        ldrb    r2, [r4, #SPC_OFF_SP]
        adds    r2, r2, #1
        strb    r2, [r4, #SPC_OFF_SP]
        mov     r1, #0x100
        orr     r1, r1, r2
        ldrb    r0, [r5, r1]
        .endm

@ pushWord: value in r0.  Push high then low.
        .macro PUSH_WORD
        mov     r8, r0
        lsr     r0, r8, #8
        PUSH_BYTE
        mov     r0, r8
        PUSH_BYTE
        .endm

@ pullWord: returns r0 = word.  Pull low then high.
        .macro PULL_WORD
        PULL_BYTE
        mov     r8, r0
        PULL_BYTE
        orr     r0, r8, r0, lsl #8
        .endm

@ ===========================================================================
@ Entry point
@ ===========================================================================

        .section .itcm_snes_interp.thumb2_spc,"ax",%progbits
        .align  2
        .global spc_thumb2_run
        .type   spc_thumb2_run,%function
        .thumb_func
spc_thumb2_run:
        push    {r3, r4, r5, r6, r7, r8, r9, r10, r11, lr}  @ 10 regs: sp stays 8-aligned for the C calls
        mov     r4, r0
        mov     r11, r1
        ldr     r7, [r4, #SPC_OFF_APU]
        add     r5, r7, #APU_OFF_RAM
        add     r10, r7, #APU_OFF_CYCLES_PAGE
        add     r10, r10, #APU_OFF_CYCLES_IN_PAGE

.Lrun_loop:
        FETCH_BYTE
        uxtb    r6, r0
        cmp     r6, #0xec
        beq     .Lrun_ec
.Lrun_dispatch:
        movw    r2, #:lower16:spc_cycles_per_opcode
        movt    r2, #:upper16:spc_cycles_per_opcode
        ldrb    r3, [r2, r6]
        strb    r3, [r4, #SPC_OFF_CYCLESUSED]

        adr     r2, .Lspc_dispatch_table
        ldr     r3, [r2, r6, lsl #2]
        bx      r3

@ MOV Y,!$00FD; BEQ -5 (EC FD 00 F0 FB) is the idle wait apu_run may skip:
@ hand it back unrun. Bytes compared in ram exactly as apu_idleSkipCycles
@ does (which also rejects the boot-ROM range), so a non-match here would
@ also be a non-match there. A single step (budget 0) just runs it.
.Lrun_ec:
        cmp     r11, #0
        beq     .Lrun_dispatch
        ldrh    r1, [r4, #SPC_OFF_PC]
        subs    r1, r1, #1
        uxth    r1, r1                   @ the opcode's own pc
        adds    r2, r1, #1
        uxth    r2, r2
        ldrb    r3, [r5, r2]
        cmp     r3, #0xfd
        bne     .Lrun_dispatch
        adds    r2, r1, #2
        uxth    r2, r2
        ldrb    r3, [r5, r2]
        cmp     r3, #0x00
        bne     .Lrun_dispatch
        adds    r2, r1, #3
        uxth    r2, r2
        ldrb    r3, [r5, r2]
        cmp     r3, #0xf0
        bne     .Lrun_dispatch
        adds    r2, r1, #4
        uxth    r2, r2
        ldrb    r3, [r5, r2]
        cmp     r3, #0xfb
        bne     .Lrun_dispatch
        strh    r1, [r4, #SPC_OFF_PC]    @ unfetch: apu_run takes it from here
        b       .Lrun_exit


@ ===========================================================================
@ Dispatch table (256 entries, Thumb-bit-set addresses)
@ ===========================================================================

        .align  2
.Lspc_dispatch_table:
        .word   .Lop00 + 1
        .word   .L_noteb + 1
        .word   .Lop02 + 1
        .word   .Lop03 + 1
        .word   .Lop04 + 1
        .word   .Lop05 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop08 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop0b + 1
        .word   .Lop0c + 1
        .word   .Lop0d + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop10 + 1
        .word   .L_noteb + 1
        .word   .Lop12 + 1
        .word   .Lop13 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop1c + 1
        .word   .Lop1d + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop20 + 1
        .word   .L_noteb + 1
        .word   .Lop22 + 1
        .word   .Lop23 + 1
        .word   .Lop24 + 1
        .word   .Lop25 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop28 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop2b + 1
        .word   .Lop2c + 1
        .word   .Lop2d + 1
        .word   .L_noteb + 1
        .word   .Lop2f + 1
        .word   .Lop30 + 1
        .word   .L_noteb + 1
        .word   .Lop32 + 1
        .word   .Lop33 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop3c + 1
        .word   .Lop3d + 1
        .word   .L_noteb + 1
        .word   .Lop3f + 1
        .word   .Lop40 + 1
        .word   .L_noteb + 1
        .word   .Lop42 + 1
        .word   .Lop43 + 1
        .word   .Lop44 + 1
        .word   .Lop45 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop48 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop4b + 1
        .word   .Lop4c + 1
        .word   .Lop4d + 1
        .word   .L_noteb + 1
        .word   .Lop4f + 1
        .word   .Lop50 + 1
        .word   .L_noteb + 1
        .word   .Lop52 + 1
        .word   .Lop53 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop5c + 1
        .word   .Lop5d + 1
        .word   .Lop5e + 1
        .word   .Lop5f + 1
        .word   .Lop60 + 1
        .word   .L_noteb + 1
        .word   .Lop62 + 1
        .word   .Lop63 + 1
        .word   .Lop64 + 1
        .word   .Lop65 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop68 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop6b + 1
        .word   .Lop6c + 1
        .word   .Lop6d + 1
        .word   .L_noteb + 1
        .word   .Lop6f + 1
        .word   .Lop70 + 1
        .word   .L_noteb + 1
        .word   .Lop72 + 1
        .word   .Lop73 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop7c + 1
        .word   .Lop7d + 1
        .word   .Lop7e + 1
        .word   .Lop7f + 1
        .word   .Lop80 + 1
        .word   .L_noteb + 1
        .word   .Lop82 + 1
        .word   .Lop83 + 1
        .word   .Lop84 + 1
        .word   .Lop85 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop88 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop8b + 1
        .word   .Lop8c + 1
        .word   .Lop8d + 1
        .word   .Lop8e + 1
        .word   .L_noteb + 1
        .word   .Lop90 + 1
        .word   .L_noteb + 1
        .word   .Lop92 + 1
        .word   .Lop93 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lop9b + 1
        .word   .Lop9c + 1
        .word   .Lop9d + 1
        .word   .L_noteb + 1
        .word   .Lop9f + 1
        .word   .Lopa0 + 1
        .word   .L_noteb + 1
        .word   .Lopa2 + 1
        .word   .Lopa3 + 1
        .word   .Lopa4 + 1
        .word   .Lopa5 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopa8 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopab + 1
        .word   .Lopac + 1
        .word   .Lopad + 1
        .word   .Lopae + 1
        .word   .L_noteb + 1
        .word   .Lopb0 + 1
        .word   .L_noteb + 1
        .word   .Lopb2 + 1
        .word   .Lopb3 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopbc + 1
        .word   .Lopbd + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopc0 + 1
        .word   .L_noteb + 1
        .word   .Lopc2 + 1
        .word   .Lopc3 + 1
        .word   .Lopc4 + 1
        .word   .Lopc5 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopc8 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopcb + 1
        .word   .Lopcc + 1
        .word   .Lopcd + 1
        .word   .Lopce + 1
        .word   .Lopcf + 1
        .word   .Lopd0 + 1
        .word   .L_noteb + 1
        .word   .Lopd2 + 1
        .word   .Lopd3 + 1
        .word   .Lopd4 + 1
        .word   .Lopd5 + 1
        .word   .Lopd6 + 1
        .word   .Lopd7 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopda + 1
        .word   .Lopdb + 1
        .word   .Lopdc + 1
        .word   .Lopdd + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lope0 + 1
        .word   .L_noteb + 1
        .word   .Lope2 + 1
        .word   .Lope3 + 1
        .word   .Lope4 + 1
        .word   .Lope5 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lope8 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopeb + 1
        .word   .Lopec + 1
        .word   .Loped + 1
        .word   .Lopee + 1
        .word   .Lopef + 1
        .word   .Lopf0 + 1
        .word   .L_noteb + 1
        .word   .Lopf2 + 1
        .word   .Lopf3 + 1
        .word   .Lopf4 + 1
        .word   .Lopf5 + 1
        .word   .Lopf6 + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .L_noteb + 1
        .word   .Lopfc + 1
        .word   .Lopfd + 1
        .word   .L_noteb + 1
        .word   .Lopff + 1

@ ===========================================================================
@ Opcode handlers
@ ===========================================================================

.Lop00:
        b       .L_handled


.Lop02:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        orr     r0, r0, #1
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop03:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #1
        bne     .Lbb_03_taken
        b       .L_handled
.Lbb_03_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop04:
        DP_ADDR
        READ_FAST
        ldrb    r3, [r4, #SPC_OFF_A]
        orrs    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop05:
        ABS_ADDR
        READ_FAST
        ldrb    r3, [r4, #SPC_OFF_A]
        orrs    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop08:
        FETCH_BYTE
        ldrb    r3, [r4, #SPC_OFF_A]
        orrs    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop0b:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        lsrs    r9, r0, #7
        lsls    r0, r0, #1
        strb    r9, [r4, #SPC_OFF_C]
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop0c:
        ABS_ADDR
        mov     r8, r1
        READ_FAST
        lsrs    r9, r0, #7
        lsls    r0, r0, #1
        strb    r9, [r4, #SPC_OFF_C]
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop0d:
        ldrb    r0, [r4, #SPC_OFF_N]
        lsls    r0, r0, #7
        ldrb    r1, [r4, #SPC_OFF_V]
        lsls    r1, r1, #6
        orrs    r0, r0, r1
        ldrb    r1, [r4, #SPC_OFF_P]
        lsls    r1, r1, #5
        orrs    r0, r0, r1
        ldrb    r1, [r4, #SPC_OFF_B]
        lsls    r1, r1, #4
        orrs    r0, r0, r1
        ldrb    r1, [r4, #SPC_OFF_H]
        lsls    r1, r1, #3
        orrs    r0, r0, r1
        ldrb    r1, [r4, #SPC_OFF_I]
        lsls    r1, r1, #2
        orrs    r0, r0, r1
        ldrb    r1, [r4, #SPC_OFF_Z]
        lsls    r1, r1, #1
        orrs    r0, r0, r1
        ldrb    r1, [r4, #SPC_OFF_C]
        orrs    r0, r0, r1
        uxtb    r0, r0
        PUSH_BYTE
        b       .L_handled


.Lop10:
        FETCH_BYTE
        mov     r8, r0
        ldrb    r1, [r4, #SPC_OFF_N]
        cmp     r1, #0
        beq     .Lbr10_taken
        b       .L_handled
.Lbr10_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r8
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop12:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        bic     r0, r0, #1
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop13:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #1
        beq     .Lbb_13_taken
        b       .L_handled
.Lbb_13_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop1c:
        ldrb    r0, [r4, #SPC_OFF_A]
        lsrs    r3, r0, #7
        strb    r3, [r4, #SPC_OFF_C]
        lsls    r0, r0, #1
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop1d:
        ldrb    r0, [r4, #5]
        adds    r0, r0, #-1
        uxtb    r0, r0
        strb    r0, [r4, #5]
        SETZN
        b       .L_handled


.Lop20:
        movs    r0, #0
        strb    r0, [r4, #SPC_OFF_P]
        b       .L_handled


.Lop22:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        orr     r0, r0, #2
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop23:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #2
        bne     .Lbb_23_taken
        b       .L_handled
.Lbb_23_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop24:
        DP_ADDR
        READ_FAST
        ldrb    r3, [r4, #SPC_OFF_A]
        ands    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop25:
        ABS_ADDR
        READ_FAST
        ldrb    r3, [r4, #SPC_OFF_A]
        ands    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop28:
        FETCH_BYTE
        ldrb    r3, [r4, #SPC_OFF_A]
        ands    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop2b:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        lsrs    r9, r0, #7
        ldrb    r3, [r4, #SPC_OFF_C]
        lsls    r0, r0, #1
        orr     r0, r0, r3
        strb    r9, [r4, #SPC_OFF_C]
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop2c:
        ABS_ADDR
        mov     r8, r1
        READ_FAST
        lsrs    r9, r0, #7
        ldrb    r3, [r4, #SPC_OFF_C]
        lsls    r0, r0, #1
        orr     r0, r0, r3
        strb    r9, [r4, #SPC_OFF_C]
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop2d:
        ldrb    r0, [r4, #SPC_OFF_A]
        PUSH_BYTE
        b       .L_handled


.Lop2f:
        FETCH_BYTE
        sxtb    r0, r0
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop30:
        FETCH_BYTE
        mov     r8, r0
        ldrb    r1, [r4, #SPC_OFF_N]
        cmp     r1, #0
        bne     .Lbr30_taken
        b       .L_handled
.Lbr30_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r8
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop32:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        bic     r0, r0, #2
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop33:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #2
        beq     .Lbb_33_taken
        b       .L_handled
.Lbb_33_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop3c:
        ldrb    r0, [r4, #SPC_OFF_A]
        lsrs    r9, r0, #7
        ldrb    r3, [r4, #SPC_OFF_C]
        lsls    r0, r0, #1
        orr     r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        strb    r9, [r4, #SPC_OFF_C]
        SETZN
        b       .L_handled


.Lop3d:
        ldrb    r0, [r4, #5]
        adds    r0, r0, #1
        uxtb    r0, r0
        strb    r0, [r4, #5]
        SETZN
        b       .L_handled


.Lop3f:
        FETCH_WORD
        mov     r9, r0
        ldrh    r0, [r4, #SPC_OFF_PC]
        PUSH_WORD
        strh    r9, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop40:
        movs    r0, #1
        strb    r0, [r4, #SPC_OFF_P]
        b       .L_handled


.Lop42:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        orr     r0, r0, #4
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop43:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #4
        bne     .Lbb_43_taken
        b       .L_handled
.Lbb_43_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop44:
        DP_ADDR
        READ_FAST
        ldrb    r3, [r4, #SPC_OFF_A]
        eors    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop45:
        ABS_ADDR
        READ_FAST
        ldrb    r3, [r4, #SPC_OFF_A]
        eors    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop48:
        FETCH_BYTE
        ldrb    r3, [r4, #SPC_OFF_A]
        eors    r0, r0, r3
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop4b:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        ands    r9, r0, #1
        lsrs    r0, r0, #1
        strb    r9, [r4, #SPC_OFF_C]
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop4c:
        ABS_ADDR
        mov     r8, r1
        READ_FAST
        ands    r9, r0, #1
        lsrs    r0, r0, #1
        strb    r9, [r4, #SPC_OFF_C]
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop4d:
        ldrb    r0, [r4, #SPC_OFF_X]
        PUSH_BYTE
        b       .L_handled


.Lop4f:
        FETCH_BYTE
        mov     r9, r0
        ldrh    r0, [r4, #SPC_OFF_PC]
        PUSH_WORD
        mov     r0, #0xff00
        orr     r0, r0, r9
        strh    r0, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop50:
        FETCH_BYTE
        mov     r8, r0
        ldrb    r1, [r4, #SPC_OFF_V]
        cmp     r1, #0
        beq     .Lbr50_taken
        b       .L_handled
.Lbr50_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r8
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop52:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        bic     r0, r0, #4
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop53:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #4
        beq     .Lbb_53_taken
        b       .L_handled
.Lbb_53_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop5c:
        ldrb    r0, [r4, #SPC_OFF_A]
        ands    r3, r0, #1
        strb    r3, [r4, #SPC_OFF_C]
        lsrs    r0, r0, #1
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop5d:
        ldrb    r0, [r4, #SPC_OFF_A]
        strb    r0, [r4, #SPC_OFF_X]
        SETZN
        b       .L_handled


.Lop5e:
        ABS_ADDR
        READ_FAST
        eor     r0, r0, #0xff
        ldrb    r2, [r4, #SPC_OFF_Y]
        adds    r3, r0, r2
        adds    r3, r3, #1
        lsrs    r2, r3, #8
        strb    r2, [r4, #SPC_OFF_C]
        uxtb    r0, r3
        SETZN
        b       .L_handled


.Lop5f:
        FETCH_WORD
        strh    r0, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop60:
        movs    r0, #0
        strb    r0, [r4, #SPC_OFF_C]
        b       .L_handled


.Lop62:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        orr     r0, r0, #8
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop63:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #8
        bne     .Lbb_63_taken
        b       .L_handled
.Lbb_63_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop64:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        CMP_REG SPC_OFF_A
        b       .L_handled


.Lop65:
        ABS_ADDR
        READ_FAST
        mov     r8, r0
        CMP_REG SPC_OFF_A
        b       .L_handled


.Lop68:
        FETCH_BYTE
        mov     r8, r0
        CMP_REG SPC_OFF_A
        b       .L_handled


.Lop6b:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        ands    r9, r0, #1
        ldrb    r3, [r4, #SPC_OFF_C]
        lsrs    r0, r0, #1
        tst     r3, #1
        it      ne
        orrne   r0, r0, #0x80
        strb    r9, [r4, #SPC_OFF_C]
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop6c:
        ABS_ADDR
        mov     r8, r1
        READ_FAST
        ands    r9, r0, #1
        ldrb    r3, [r4, #SPC_OFF_C]
        lsrs    r0, r0, #1
        tst     r3, #1
        it      ne
        orrne   r0, r0, #0x80
        strb    r9, [r4, #SPC_OFF_C]
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop6d:
        ldrb    r0, [r4, #SPC_OFF_Y]
        PUSH_BYTE
        b       .L_handled


.Lop6f:
        PULL_WORD
        strh    r0, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop70:
        FETCH_BYTE
        mov     r8, r0
        ldrb    r1, [r4, #SPC_OFF_V]
        cmp     r1, #0
        bne     .Lbr70_taken
        b       .L_handled
.Lbr70_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r8
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop72:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        bic     r0, r0, #8
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop73:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #8
        beq     .Lbb_73_taken
        b       .L_handled
.Lbb_73_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop7c:
        ldrb    r0, [r4, #SPC_OFF_A]
        ands    r9, r0, #1
        ldrb    r3, [r4, #SPC_OFF_C]
        lsrs    r0, r0, #1
        tst     r3, #1
        it      ne
        orrne   r0, r0, #0x80
        strb    r0, [r4, #SPC_OFF_A]
        strb    r9, [r4, #SPC_OFF_C]
        SETZN
        b       .L_handled


.Lop7d:
        ldrb    r0, [r4, #SPC_OFF_X]
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lop7e:
        DP_ADDR
        READ_FAST
        eor     r0, r0, #0xff
        ldrb    r2, [r4, #SPC_OFF_Y]
        adds    r3, r0, r2
        adds    r3, r3, #1
        lsrs    r2, r3, #8
        strb    r2, [r4, #SPC_OFF_C]
        uxtb    r0, r3
        SETZN
        b       .L_handled


.Lop7f:
        PULL_BYTE
        @ setFlags(r0): n=bit7 v=bit6 p=bit5 b=bit4 h=bit3 i=bit2 z=bit1 c=bit0
        mov     r8, r0
        ubfx    r1, r8, #7, #1
        strb    r1, [r4, #SPC_OFF_N]
        ubfx    r1, r8, #6, #1
        strb    r1, [r4, #SPC_OFF_V]
        ubfx    r1, r8, #5, #1
        strb    r1, [r4, #SPC_OFF_P]
        ubfx    r1, r8, #4, #1
        strb    r1, [r4, #SPC_OFF_B]
        ubfx    r1, r8, #3, #1
        strb    r1, [r4, #SPC_OFF_H]
        ubfx    r1, r8, #2, #1
        strb    r1, [r4, #SPC_OFF_I]
        ubfx    r1, r8, #1, #1
        strb    r1, [r4, #SPC_OFF_Z]
        and     r1, r8, #1
        strb    r1, [r4, #SPC_OFF_C]
        PULL_WORD
        strh    r0, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop80:
        movs    r0, #1
        strb    r0, [r4, #SPC_OFF_C]
        b       .L_handled


.Lop82:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        orr     r0, r0, #16
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop83:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #16
        bne     .Lbb_83_taken
        b       .L_handled
.Lbb_83_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop84:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        ADC_SPC
        b       .L_handled


.Lop85:
        ABS_ADDR
        READ_FAST
        mov     r8, r0
        ADC_SPC
        b       .L_handled


.Lop88:
        FETCH_BYTE
        mov     r8, r0
        ADC_SPC
        b       .L_handled


.Lop8b:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        subs    r0, r0, #1
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop8c:
        ABS_ADDR
        mov     r8, r1
        READ_FAST
        subs    r0, r0, #1
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop8d:
        FETCH_BYTE
        strb    r0, [r4, #SPC_OFF_Y]
        SETZN
        b       .L_handled


.Lop8e:
        PULL_BYTE
        mov     r8, r0
        ubfx    r1, r8, #7, #1
        strb    r1, [r4, #SPC_OFF_N]
        ubfx    r1, r8, #6, #1
        strb    r1, [r4, #SPC_OFF_V]
        ubfx    r1, r8, #5, #1
        strb    r1, [r4, #SPC_OFF_P]
        ubfx    r1, r8, #4, #1
        strb    r1, [r4, #SPC_OFF_B]
        ubfx    r1, r8, #3, #1
        strb    r1, [r4, #SPC_OFF_H]
        ubfx    r1, r8, #2, #1
        strb    r1, [r4, #SPC_OFF_I]
        ubfx    r1, r8, #1, #1
        strb    r1, [r4, #SPC_OFF_Z]
        and     r1, r8, #1
        strb    r1, [r4, #SPC_OFF_C]
        b       .L_handled


.Lop90:
        FETCH_BYTE
        mov     r8, r0
        ldrb    r1, [r4, #SPC_OFF_C]
        cmp     r1, #0
        beq     .Lbr90_taken
        b       .L_handled
.Lbr90_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r8
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop92:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        bic     r0, r0, #16
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lop93:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #16
        beq     .Lbb_93_taken
        b       .L_handled
.Lbb_93_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lop9b:
        DPX_ADDR
        mov     r8, r1
        READ_FAST
        subs    r0, r0, #1
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lop9c:
        ldrb    r0, [r4, #4]
        adds    r0, r0, #-1
        uxtb    r0, r0
        strb    r0, [r4, #4]
        SETZN
        b       .L_handled


.Lop9d:
        ldrb    r0, [r4, #SPC_OFF_SP]
        strb    r0, [r4, #SPC_OFF_X]
        SETZN
        b       .L_handled


.Lop9f:
        ldrb    r0, [r4, #SPC_OFF_A]
        lsls    r1, r0, #4
        lsrs    r0, r0, #4
        orrs    r0, r0, r1
        uxtb    r0, r0
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lopa0:
        movs    r0, #1
        strb    r0, [r4, #SPC_OFF_I]
        b       .L_handled


.Lopa2:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        orr     r0, r0, #32
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lopa3:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #32
        bne     .Lbb_a3_taken
        b       .L_handled
.Lbb_a3_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lopa4:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        eor     r8, r8, #0xff
        ADC_SPC
        b       .L_handled


.Lopa5:
        ABS_ADDR
        READ_FAST
        mov     r8, r0
        eor     r8, r8, #0xff
        ADC_SPC
        b       .L_handled


.Lopa8:
        FETCH_BYTE
        mov     r8, r0
        eor     r8, r8, #0xff
        ADC_SPC
        b       .L_handled


.Lopab:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        adds    r0, r0, #1
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lopac:
        ABS_ADDR
        mov     r8, r1
        READ_FAST
        adds    r0, r0, #1
        uxtb    r0, r0
        mov     r9, r0
        mov     r1, r8
        mov     r2, r0
        WRITE_FAST
        mov     r0, r9
        SETZN
        b       .L_handled


.Lopad:
        FETCH_BYTE
        mov     r8, r0
        CMP_REG SPC_OFF_Y
        b       .L_handled


.Lopae:
        PULL_BYTE
        strb    r0, [r4, #SPC_OFF_A]
        b       .L_handled


.Lopb0:
        FETCH_BYTE
        mov     r8, r0
        ldrb    r1, [r4, #SPC_OFF_C]
        cmp     r1, #0
        bne     .Lbrb0_taken
        b       .L_handled
.Lbrb0_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r8
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lopb2:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        bic     r0, r0, #32
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lopb3:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #32
        beq     .Lbb_b3_taken
        b       .L_handled
.Lbb_b3_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lopbc:
        ldrb    r0, [r4, #4]
        adds    r0, r0, #1
        uxtb    r0, r0
        strb    r0, [r4, #4]
        SETZN
        b       .L_handled


.Lopbd:
        ldrb    r0, [r4, #SPC_OFF_X]
        strb    r0, [r4, #SPC_OFF_SP]
        b       .L_handled


.Lopc0:
        movs    r0, #0
        strb    r0, [r4, #SPC_OFF_I]
        b       .L_handled


.Lopc2:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        orr     r0, r0, #64
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lopc3:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #64
        bne     .Lbb_c3_taken
        b       .L_handled
.Lbb_c3_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lopc4:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_A]
        WRITE_FAST
        b       .L_handled


.Lopc5:
        ABS_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_A]
        WRITE_FAST
        b       .L_handled


.Lopc8:
        FETCH_BYTE
        mov     r8, r0
        CMP_REG SPC_OFF_X
        b       .L_handled


.Lopcb:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_Y]
        WRITE_FAST
        b       .L_handled


.Lopcc:
        ABS_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_Y]
        WRITE_FAST
        b       .L_handled


.Lopcd:
        FETCH_BYTE
        strb    r0, [r4, #SPC_OFF_X]
        SETZN
        b       .L_handled


.Lopce:
        PULL_BYTE
        strb    r0, [r4, #SPC_OFF_X]
        b       .L_handled


.Lopcf:
        ldrb    r0, [r4, #SPC_OFF_A]
        ldrb    r1, [r4, #SPC_OFF_Y]
        mul     r3, r0, r1
        strb    r3, [r4, #SPC_OFF_A]
        lsrs    r0, r3, #8
        strb    r0, [r4, #SPC_OFF_Y]
        SETZN
        b       .L_handled


.Lopd0:
        FETCH_BYTE
        mov     r8, r0
        ldrb    r1, [r4, #SPC_OFF_Z]
        cmp     r1, #0
        beq     .Lbrd0_taken
        b       .L_handled
.Lbrd0_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r8
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lopd2:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        bic     r0, r0, #64
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lopd3:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #64
        beq     .Lbb_d3_taken
        b       .L_handled
.Lbb_d3_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lopd4:
        DPX_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_A]
        WRITE_FAST
        b       .L_handled


.Lopd5:
        ABX_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_A]
        WRITE_FAST
        b       .L_handled


.Lopd6:
        ABY_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_A]
        WRITE_FAST
        b       .L_handled


.Lopd7:
        IDY_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_A]
        WRITE_FAST
        b       .L_handled


.Lopda:
        FETCH_BYTE
        ldrb    r3, [r4, #SPC_OFF_P]
        lsl     r3, r3, #8
        orr     r8, r0, r3
        adds    r2, r0, #1
        uxtb    r2, r2
        orr     r9, r2, r3
        mov     r1, r8
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_A]
        WRITE_FAST
        mov     r1, r9
        ldrb    r2, [r4, #SPC_OFF_Y]
        WRITE_FAST
        b       .L_handled


.Lopdb:
        DPX_ADDR
        mov     r8, r1
        READ_FAST
        mov     r1, r8
        ldrb    r2, [r4, #SPC_OFF_Y]
        WRITE_FAST
        b       .L_handled


.Lopdc:
        ldrb    r0, [r4, #6]
        adds    r0, r0, #-1
        uxtb    r0, r0
        strb    r0, [r4, #6]
        SETZN
        b       .L_handled


.Lopdd:
        ldrb    r0, [r4, #SPC_OFF_Y]
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lope0:
        movs    r0, #0
        strb    r0, [r4, #SPC_OFF_V]
        strb    r0, [r4, #SPC_OFF_H]
        b       .L_handled


.Lope2:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        orr     r0, r0, #128
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lope3:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #128
        bne     .Lbb_e3_taken
        b       .L_handled
.Lbb_e3_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lope4:
        DP_ADDR
        READ_FAST
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lope5:
        ABS_ADDR
        READ_FAST
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lope8:
        FETCH_BYTE
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lopeb:
        DP_ADDR
        READ_FAST
        strb    r0, [r4, #SPC_OFF_Y]
        SETZN
        b       .L_handled


.Lopec:
        ABS_ADDR
        READ_FAST
        strb    r0, [r4, #SPC_OFF_Y]
        SETZN
        b       .L_handled


.Loped:
        ldrb    r0, [r4, #SPC_OFF_C]
        eors    r0, r0, #1
        strb    r0, [r4, #SPC_OFF_C]
        b       .L_handled


.Lopee:
        PULL_BYTE
        strb    r0, [r4, #SPC_OFF_Y]
        b       .L_handled


.Lopef:
        movs    r0, #1
        strb    r0, [r4, #SPC_OFF_STOPPED]
        b       .L_handled_stop


.Lopf0:
        FETCH_BYTE
        mov     r8, r0
        ldrb    r1, [r4, #SPC_OFF_Z]
        cmp     r1, #0
        bne     .Lbrf0_taken
        b       .L_handled
.Lbrf0_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r8
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lopf2:
        DP_ADDR
        mov     r8, r1
        READ_FAST
        bic     r0, r0, #128
        uxtb    r2, r0
        mov     r1, r8
        WRITE_FAST
        b       .L_handled


.Lopf3:
        DP_ADDR
        READ_FAST
        mov     r8, r0
        FETCH_BYTE
        mov     r9, r0
        mov     r0, r8
        tst     r0, #128
        beq     .Lbb_f3_taken
        b       .L_handled
.Lbb_f3_taken:
        ldrb    r1, [r4, #SPC_OFF_CYCLESUSED]
        adds    r1, r1, #2
        strb    r1, [r4, #SPC_OFF_CYCLESUSED]
        sxtb    r0, r9
        ldrh    r1, [r4, #SPC_OFF_PC]
        adds    r1, r1, r0
        strh    r1, [r4, #SPC_OFF_PC]
        b       .L_handled


.Lopf4:
        DPX_ADDR
        READ_FAST
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lopf5:
        ABX_ADDR
        READ_FAST
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lopf6:
        ABY_ADDR
        READ_FAST
        strb    r0, [r4, #SPC_OFF_A]
        SETZN
        b       .L_handled


.Lopfc:
        ldrb    r0, [r4, #6]
        adds    r0, r0, #1
        uxtb    r0, r0
        strb    r0, [r4, #6]
        SETZN
        b       .L_handled


.Lopfd:
        ldrb    r0, [r4, #SPC_OFF_A]
        strb    r0, [r4, #SPC_OFF_Y]
        SETZN
        b       .L_handled


.Lopff:
        movs    r0, #1
        strb    r0, [r4, #SPC_OFF_STOPPED]
        b       .L_handled_stop


@ ===========================================================================
@ Exit labels
@ ===========================================================================

@ The opcode is done; its cycles are in cyclesUsed. apu_run's step, inside.
        .macro COMMIT_OPCODE next
        ldrb    r0, [r4, #SPC_OFF_CYCLESUSED]
        cmp     r0, r11
        bgt     .Lrun_partial            @ runs past the budget (always, for a single step)
        ldr     r1, [r10]                @ start
        add     r8, r1, r0               @ end
        sub     r11, r11, r0
        add     r3, r1, #31
        bic     r3, r3, #31              @ the first multiple of 32 at or after start
        cmp     r3, r8
        bhs     1f
        mov     r0, r7                   @ a DSP tick in [start, end)
        mov     r2, r8
        bl      apu_dsp_ticks
1:
        str     r8, [r10]
        cmp     r11, #0
        bgt     \next
        b       .Lrun_exit
        .endm

.L_handled:
        COMMIT_OPCODE .Lrun_loop

@ STOP / SLEEP: commit, then leave (spc->stopped is set; apu_run takes over)
.L_handled_stop:
        COMMIT_OPCODE .Lrun_exit

.L_noteb:
        mov     r0, r4
        mov     r1, r6
        bl      spc_fallback            @ spc_doOpcode on the fetched opcode
        b       .L_handled

.Lrun_partial:
        strb    r0, [r10, #APU_OFF_CPUCYCLESLEFT - APU_OFF_CYCLES]
.Lrun_exit:
        mov     r0, r11
        pop     {r3, r4, r5, r6, r7, r8, r9, r10, r11, pc}

        .size   spc_thumb2_run, .-spc_thumb2_run
