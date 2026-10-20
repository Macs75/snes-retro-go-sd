#!/usr/bin/env python3
"""Generate src/snes/thumb2/sa1_engine.S, the SA-1's 65816 in Thumb-2.

The SA-1 runs in this engine on the device (SNES_SA1_T2), the C copy
(sa1_cpu.c) on the host and for what the engine leaves to it. Unlike the main
CPU's engine it keeps the 65816 in ARM registers for a whole slice:

  r4  Cpu*                 r8  X                  r11 SA-1 clocks owed (debt)
  r5  code page limit      r9  Y                  lr  dispatch table (m, x)
  r6  code pointer         r10 N and Z            r0-r3, ip scratch
  r7  A

r6 points into the 8 KB read-map page of the code (ROM), so a fetch is one
ldrb; the page is looked up again only when the pc leaves it (r6 > r5, the
page's last 4 bytes, or a jump). N is bit 31 of r10, Z is set when bits 0-30
are all 0 (sxtb / sxth of a result gives both). C, V, D, I, m, x and the rest
stay in the Cpu struct, as do SP, DP, DB and K. A handler is written for one
m/x width: there is a dispatch table per combination (REP, SEP, PLP switch it).
Every handler ends with its own dispatch, charging the opcode's cycles the way
cpu.c does (cyclesPerOpcode plus the same extras).

The bus is the SA-1's cpu_read / cpu_write (cpu.c under CPU_SA1): the read
map, then I-RAM, then C (sa1.c's sa1e_* helpers). Data reads of the map are
bytewise: BW-RAM is in the device's AHB SRAM, strongly-ordered, where an
unaligned halfword faults; I-RAM (DTCM) and the code page (ROM) take them.

The wait-loop proof (sa1.c, sa1_loop_head) is done here after every jump back
of 1-64 bytes in the bank, on this engine's own snapshot (Sa1Run.asnap):
return 3 when a lap recurred exactly. Returns 0 debt spent, 1 C runs the next
instruction (an interrupt, WAI / STP, emulation mode, code outside the map),
3 proven. Opcodes left to C (BRK, COP, RTI, WAI, STP, XCE, decimal ADC / SBC,
the memory opcode forms not in ASM_MEM / ASM_FORMS) go through sa1cpu_thumb2_fallback with the
state flushed.

  tools/gen_sa1_engine.py            (writes the file; run from the repo root)
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'src/snes/thumb2/sa1_engine.S')


def cycles_table():
    src = open(os.path.join(ROOT, 'src/snes/cpu.c')).read()
    m = re.search(r'#define SNES_CYCLES_PER_OPCODE \\\n((?:.*\\\n)*.*\n)', src)
    t = [int(v) for v in re.findall(r'\d+', m.group(1))]
    assert len(t) == 256
    return t


CYC = cycles_table()
out = []
_n = [0]


def L(prefix='L'):
    _n[0] += 1
    return '.L%s%d' % (prefix, _n[0])


def code(lines):
    """Format: labels flush left, instructions indented."""
    res = []
    for s in lines:
        if not s:
            continue
        if s.endswith(':') or s.startswith('#'):
            res.append(s)
        else:
            res.append('        ' + s)
    return res


COUNT = '#ifdef SA1E_COUNT\n        ldr     ip, [sp, #F_OPS]\n        add     ip, ip, #1\n        str     ip, [sp, #F_OPS]\n#endif'


def dispatch():
    return [COUNT, 'ldrb r0, [r6], #1', 'ldr.w pc, [lr, r0, lsl #2]']


def NEXT(c):
    """Charge c clocks, then the next instruction."""
    s = ['subs r11, r11, #%d' % c] if c else ['cmp r11, #0']
    return s + ['ble.w .Lx0', 'cmp r6, r5', 'bhi.w .Lpage'] + dispatch()


# --- memory ---------------------------------------------------------------
# r1 = address (kept), r0 = value. 'b0': bank 0, 16-bit wrap (dp, stack);
# 'l': 24-bit; 'k': bank K with a 16-bit wrap (JMP (a,x)).

def cold(lines):
    return ['.subsection 1'] + lines + ['.subsection 0']


def iram_test(miss):
    return ['ands r3, r1, #0xf800', 'it ne', 'cmpne r3, #0x3000', 'bne.w %s' % miss]


def RD8(kind):
    m, d = L('m'), L('d')
    if kind == 'b0':
        s = iram_test(m) + ['ldr r2, [sp, #F_IRAM]', 'ubfx r3, r1, #0, #11', 'ldrb r0, [r2, r3]']
    else:
        s = ['ldr r2, [sp, #F_SA1]', 'ubfx r3, r1, #13, #11', 'ldr r3, [r2, r3, lsl #2]', 'cmp r3, #0',
             'beq.w %s' % m, 'ubfx r2, r1, #0, #13', 'ldrb r0, [r3, r2]']
    return s + [d + ':'] + cold([m + ':', 'bl sa1e_rd8_s', 'b.w %s' % d])


def RD16(kind):
    m, d = L('m'), L('d')
    if kind == 'b0':
        s = iram_test(m) + ['add r2, r1, #1', 'lsls r2, r2, #21', 'beq.w %s' % m,
                            'ldr r2, [sp, #F_IRAM]', 'ubfx r3, r1, #0, #11', 'ldrh r0, [r2, r3]']
    else:
        s = ['ldr r2, [sp, #F_SA1]', 'ubfx r3, r1, #13, #11', 'ldr r3, [r2, r3, lsl #2]', 'cmp r3, #0',
             'beq.w %s' % m, 'add r2, r1, #1', 'lsls r2, r2, #19', 'beq.w %s' % m,
             'ubfx r2, r1, #0, #13', 'add r3, r3, r2', 'ldrb r0, [r3]', 'ldrb r2, [r3, #1]',
             'orr r0, r0, r2, lsl #8']
    return s + [d + ':'] + cold([m + ':', 'bl sa1e_rd16_s_%s' % kind, 'b.w %s' % d])


def RD24_b0():
    m, d = L('m'), L('d')
    s = iram_test(m) + ['ubfx r3, r1, #0, #11', 'add r2, r3, #2', 'lsrs r2, r2, #11', 'bne.w %s' % m,
                        'ldr r2, [sp, #F_IRAM]', 'add r2, r2, r3', 'ldrh r0, [r2]', 'ldrb r3, [r2, #2]',
                        'orr r0, r0, r3, lsl #16']
    return s + [d + ':'] + cold([m + ':', 'bl sa1e_rd24_s', 'b.w %s' % d])


def WR8(kind):
    m, d = L('m'), L('d')
    s = ['tst r1, #0x400000', 'bne.w %s' % m] if kind == 'l' else []
    s += iram_test(m) + ['ldr r2, [sp, #F_IRAM]', 'ubfx r3, r1, #0, #11', 'strb r0, [r2, r3]',
                         'str r2, [r2, #SA1E_WDIRTY]']
    return s + [d + ':'] + cold([m + ':', 'bl sa1e_wr8_s', 'b.w %s' % d])


def WR16(kind, rev):
    m, d = L('m'), L('d')
    s = ['tst r1, #0x400000', 'bne.w %s' % m] if kind == 'l' else []
    s += iram_test(m) + ['add r2, r1, #1', 'lsls r2, r2, #21', 'beq.w %s' % m,
                         'ldr r2, [sp, #F_IRAM]', 'ubfx r3, r1, #0, #11', 'strh r0, [r2, r3]',
                         'str r2, [r2, #SA1E_WDIRTY]']
    return s + [d + ':'] + cold([m + ':', 'bl sa1e_wr16_s_%s%s' % (kind, 'r' if rev else ''), 'b.w %s' % d])


# --- addressing modes: r1 = the low address; returns (code, kind) -----------

def am_dp(idx=None):
    s = ['ldrb r1, [r6], #1', 'ldrh r2, [r4, #CPU_DP]', 'add r1, r1, r2']
    if idx:
        s.append('add r1, r1, %s' % idx)
    s += ['uxth r1, r1', '@@DPX']        # dp & 0xff: 1 extra cycle, charged at the end (TAIL)
    return s, 'b0'


def am_sr():
    return ['ldrb r1, [r6], #1', 'ldrh r2, [r4, #CPU_SP]', 'add r1, r1, r2', 'uxth r1, r1'], 'b0'


def with_db(src='r1'):
    return ['ldrb r2, [r4, #CPU_DB]', 'add r1, %s, r2, lsl #16' % src]


def cross(base, idx):
    """write modes, x = 1: an extra cycle when base + index crosses a page --
    the flags kept to the end of the address (cross_mark), then bit 31 of r1
    (the bus ignores it) to the end of the instruction (TAIL)"""
    return ['add r3, %s, %s' % (base, idx), 'eor r3, r3, %s' % base, 'cmp r3, #0xff']


def cross_mark():
    return ['it hi', 'orrhi r1, r1, #0x80000000', '@@XC']


# Extra cycles that depend on the operand are charged at the end of the
# instruction, just before its own clocks: until then r11 is the debt the
# instruction started with, which the run-ahead (sa1_ahead) reads at a write.
TAIL = {'@@DPX': ['ldrh r2, [r4, #CPU_DP]', 'tst r2, #0xff', 'it ne', 'subne r11, r11, #1'],
        '@@XC': ['tst r1, #0x80000000', 'it ne', 'subne r11, r11, #1']}


def am(mode, x, write):
    """-> (code, kind, static extra cycles)"""
    if mode == 'dp':
        c, k = am_dp(); return c, k, 0
    if mode == 'dpx':
        c, k = am_dp('r8'); return c, k, 0
    if mode == 'dpy':
        c, k = am_dp('r9'); return c, k, 0
    if mode == 'sr':
        c, k = am_sr(); return c, k, 0
    if mode == 'abs':
        return ['ldrh r1, [r6], #2'] + with_db(), 'l', 0
    if mode in ('abx', 'aby'):
        idx = 'r8' if mode == 'abx' else 'r9'
        s, extra = ['ldrh r1, [r6], #2'], 0
        if write:
            if x:
                s += cross('r1', idx)
            else:
                extra = 1
        s += ['add r1, r1, %s' % idx] + with_db() + ['bic r1, r1, #0xff000000']
        if write and x:
            s += cross_mark()
        return s, 'l', extra
    if mode == 'abl':
        return ['ldrh r1, [r6], #2', 'ldrb r2, [r6], #1', 'orr r1, r1, r2, lsl #16'], 'l', 0
    if mode == 'alx':
        return ['ldrh r1, [r6], #2', 'ldrb r2, [r6], #1', 'orr r1, r1, r2, lsl #16', 'add r1, r1, r8',
                'bic r1, r1, #0xff000000'], 'l', 0
    if mode in ('idp', 'idx'):
        c, _ = am_dp('r8' if mode == 'idx' else None)
        return c + RD16('b0') + with_db('r0'), 'l', 0
    if mode == 'idy':
        c, _ = am_dp()
        s, extra = c + RD16('b0'), 0
        if write:
            if x:
                s += cross('r0', 'r9')
            else:
                extra = 1
        s += ['add r1, r0, r9'] + with_db() + ['bic r1, r1, #0xff000000']
        if write and x:
            s += cross_mark()
        return s, 'l', extra
    if mode in ('idl', 'ily'):
        c, _ = am_dp()
        s = c + RD24_b0()
        s += ['mov r1, r0'] if mode == 'idl' else ['add r1, r0, r9', 'bic r1, r1, #0xff000000']
        return s, 'l', 0
    if mode == 'isy':
        c, _ = am_sr()
        return c + RD16('b0') + ['add r1, r0, r9'] + with_db() + ['bic r1, r1, #0xff000000'], 'l', 0
    raise ValueError(mode)


# The opcodes with memory operands the engine runs: those that are at least
# 0.02% of the SA-1's instructions in Kirby Super Star (the replay of a
# recording), Kirby's Dream Land 3 or Super Mario RPG (intro and attract, host
# SA1_PCHIST). The rest -- with them under 0.2% of the work -- go to the C copy,
# as do BRK, COP, RTI, WAI, STP, XCE; every opcode without a memory operand
# (flags, transfers, branches) is here regardless, being small.
ASM_MEM = {int(o, 16) for o in """
    04 05 06 08 09 0a 0b 10 18 1a 20 22 25 26 28 29 2a 2b 2c 30
    38 3a 3c 45 48 49 4a 4b 4c 5a 5c 60 64 65 68 69 6b 6c 6d
    70 74 79 7a 7b 7c 7d 7f 80 82 84 85 86 87 88 89 8a 8b 8c 8d
    8e 8f 90 95 98 99 9b 9c 9d 9e 9f a0 a2 a4 a5 a6 a7 a8 a9 aa
    ab ac ad ae b0 b5 b7 b9 bb bc bd be bf c0 c2 c5 c6 c8 c9 ca
    cd d0 d9 da dc dd de e0 e2 e4 e5 e6 e8 e9 ea eb ec ed ee f0
    f4 fa fc
""".split()}

# And these in one m / x form only (opcode/mx, as it ran): the forms that are
# at least 0.1% of the SA-1's instructions, those same runs and the replay of
# a Super Mario RPG recording (tools/sa1_engine_coverage.py), each worth some
# 0.1% of the frame where it runs; a handler per form, not per opcode, since
# these games keep x = 0. Their other forms go to C.
ASM_FORMS = {(int(f[:2], 16), int(f[3]), int(f[4])) for f in """
    0d/00 14/10 24/00 44/00 54/00 66/10 75/00 91/00 94/10 af/00 b4/00 c4/00
    f6/10
""".split()}


GROUP1 = {1: 'idx', 3: 'sr', 5: 'dp', 7: 'idl', 9: 'imm', 0xd: 'abs', 0xf: 'abl', 0x11: 'idy', 0x12: 'idp',
          0x13: 'isy', 0x15: 'dpx', 0x17: 'ily', 0x19: 'aby', 0x1d: 'abx', 0x1f: 'alx'}

handlers = {}      # (op, m, x) -> label
bodies = []        # (label, lines)
FB = '.Lfb'


def add(op, m, x, lines, name=None):
    lab = '.Lh%02x_%d%d' % (op, m, x)
    tails = [t for t in TAIL if t in lines]
    lines = [l for l in lines if l not in TAIL]
    if tails:
        tail = sum((TAIL[t] for t in tails), [])
        out = []
        for l in lines:
            if l.startswith('subs r11, r11, #'):
                out += tail
            out.append(l)
        assert len(out) > len(lines)
        lines = out
    bodies.append((lab, lines))
    return lab


def setzn8(r):
    return ['sxtb r10, %s' % r]


def setzn16(r):
    return ['sxth r10, %s' % r]


def carry_from_arm():
    return ['mov.w r3, #0', 'adc r3, r3, #0', 'strb r3, [r4, #CPU_C]']


def alu(name, op, m, x, mode):
    """group 1: ORA AND EOR ADC STA LDA CMP SBC"""
    write = name == 'STA' and mode in ('abx', 'aby', 'idy')
    s = []
    if name in ('ADC', 'SBC'):
        s += ['ldrb r3, [r4, #CPU_D]', 'cmp r3, #0', 'bne.w .Lfb']        # decimal mode: C
    extra = 0 if m else 1
    if mode == 'imm':
        s += ['ldrb r0, [r6], #1' if m else 'ldrh r0, [r6], #2']
        kind = None
    else:
        c, kind, e = am(mode, x, write)
        s += c
        extra += e
        if name != 'STA':
            s += RD8(kind) if m else RD16(kind)
    w = 8 if m else 16
    if name == 'STA':
        s += ['mov r0, r7'] + (WR8(kind) if m else WR16(kind, False))
    elif name == 'LDA':
        s += (['bfi r7, r0, #0, #8'] + setzn8('r0')) if m else (['mov r7, r0'] + setzn16('r0'))
    elif name in ('ORA', 'AND', 'EOR'):
        ins = {'ORA': 'orr', 'AND': 'and', 'EOR': 'eor'}[name]
        if m:
            s += ['%s r2, r7, r0' % ins, 'bfi r7, r2, #0, #8'] + setzn8('r2')
        else:
            s += ['%s r7, r7, r0' % ins] + setzn16('r7')
    elif name == 'CMP':
        s += (['uxtb r2, r7', 'subs r2, r2, r0'] if m else ['subs r2, r7, r0']) + carry_from_arm() + \
             (setzn8('r2') if m else setzn16('r2'))
    elif name in ('ADC', 'SBC'):
        if name == 'SBC':
            s += ['eor r0, r0, #0xff'] if m else ['mvn r0, r0', 'uxth r0, r0']
        if m:
            s += ['ldrb r3, [r4, #CPU_C]', 'lsl r2, r7, #24', 'orn r2, r2, #0xff000000', 'lsrs r3, r3, #1',
                  'adcs r2, r2, r0, lsl #24', 'mrs r3, APSR', 'lsr r2, r2, #24', 'bfi r7, r2, #0, #8'] + setzn8('r2')
        else:
            s += ['ldrb r3, [r4, #CPU_C]', 'lsl r2, r7, #16', 'orr r2, r2, #0xff00', 'orr r2, r2, #0xff',
                  'lsrs r3, r3, #1', 'adcs r2, r2, r0, lsl #16', 'mrs r3, APSR', 'lsr r7, r2, #16'] + setzn16('r7')
        s += ['ubfx r0, r3, #29, #1', 'strb r0, [r4, #CPU_C]', 'ubfx r0, r3, #28, #1', 'strb r0, [r4, #CPU_V]']
    return s + NEXT(CYC[op] + extra)


def rmw(name, op, m, x, mode):
    """ASL ROL LSR ROR DEC INC TSB TRB on memory"""
    write = mode == 'abx'
    c, kind, e = am(mode, x, write)
    s = c + (RD8(kind) if m else RD16(kind))
    top = 8 if m else 16
    zn = setzn8 if m else setzn16
    if name == 'ASL':
        s += ['lsl r0, r0, #1', 'ubfx r3, r0, #%d, #1' % top, 'strb r3, [r4, #CPU_C]'] + zn('r0')
    elif name == 'LSR':
        s += ['and r3, r0, #1', 'strb r3, [r4, #CPU_C]', 'lsr r0, r0, #1'] + zn('r0')
    elif name == 'ROL':
        s += ['ldrb r3, [r4, #CPU_C]', 'orr r0, r3, r0, lsl #1', 'ubfx r3, r0, #%d, #1' % top,
              'strb r3, [r4, #CPU_C]'] + zn('r0')
    elif name == 'ROR':
        s += ['ldrb r3, [r4, #CPU_C]', 'and ip, r0, #1', 'lsr r0, r0, #1', 'orr r0, r0, r3, lsl #%d' % (top - 1),
              'strb ip, [r4, #CPU_C]'] + zn('r0')
    elif name == 'INC':
        s += ['add r0, r0, #1'] + zn('r0')
    elif name == 'DEC':
        s += ['sub r0, r0, #1'] + zn('r0')
    elif name in ('TSB', 'TRB'):
        s += (['uxtb r2, r7', 'ands r2, r2, r0'] if m else ['ands r2, r7, r0']) + \
             ['and r10, r10, #0x80000000', 'it ne', 'orrne r10, r10, #1']    # Z only, N kept
        s += ['orr r0, r0, r7'] if name == 'TSB' else ['bic r0, r0, r7']
    s += WR8(kind) if m else WR16(kind, True)
    return s + NEXT(CYC[op] + (0 if m else 2) + e)


def ldxy(reg, op, x, mode):
    if mode == 'imm':
        s = ['ldrb %s, [r6], #1' % reg] if x else ['ldrh %s, [r6], #2' % reg]
        e = 0
    else:
        c, kind, e = am(mode, x, False)
        s = c + (RD8(kind) if x else RD16(kind)) + ['mov %s, r0' % reg]
    s += setzn8(reg) if x else setzn16(reg)
    return s + NEXT(CYC[op] + e + (0 if x else 1))


def stxy(reg, op, m, x, mode, width_flag):
    write = mode in ('abx',)
    c, kind, e = am(mode, x, write)
    small = width_flag
    s = c + ['mov r0, %s' % reg if reg != '0' else 'mov.w r0, #0'] + (WR8(kind) if small else WR16(kind, False))
    return s + NEXT(CYC[op] + e + (0 if small else 1))


def cpxy(reg, op, x, mode):
    if mode == 'imm':
        s = ['ldrb r0, [r6], #1'] if x else ['ldrh r0, [r6], #2']
        e = 0
    else:
        c, kind, e = am(mode, x, False)
        s = c + (RD8(kind) if x else RD16(kind))
    s += ['subs r2, %s, r0' % reg] + carry_from_arm() + (setzn8('r2') if x else setzn16('r2'))
    return s + NEXT(CYC[op] + e + (0 if x else 1))


def bit(op, m, x, mode):
    test = ['uxtb r2, r7', 'ands r2, r2, r0'] if m else ['ands r2, r7, r0']
    if mode == 'imm':                       # Z only, N kept
        s = ['ldrb r0, [r6], #1' if m else 'ldrh r0, [r6], #2'] + test + \
            ['and r10, r10, #0x80000000', 'it ne', 'orrne r10, r10, #1']
        return s + NEXT(CYC[op] + (0 if m else 1))
    c, kind, e = am(mode, x, False)
    top = 7 if m else 15
    s = c + (RD8(kind) if m else RD16(kind)) + test
    s += ['and r10, r0, #0x%x' % (1 << top), 'lsl r10, r10, #%d' % (31 - top), 'it ne', 'orrne r10, r10, #1',
          'ubfx r3, r0, #%d, #1' % (top - 1), 'strb r3, [r4, #CPU_V]']
    return s + NEXT(CYC[op] + e + (0 if m else 1))


def mvx(op, x, d):
    """MVN (d = 'add') / MVP ('sub'): one byte per instruction, as cpu.c --
    DB = the destination bank, [src:X] to [dest:Y], A - 1, X and Y step; the
    instruction again (pc back to its opcode) until A wraps to $FFFF. Flags
    untouched."""
    s = ['ldrb r2, [r6], #1', 'strb r2, [r4, #CPU_DB]', 'ldrb r1, [r6], #1', 'orr r1, r8, r1, lsl #16']
    s += RD8('l') + ['ldrb r2, [r4, #CPU_DB]', 'orr r1, r9, r2, lsl #16'] + WR8('l')
    w = 'uxtb' if x else 'uxth'
    s += ['%s r8, r8, #1' % d, '%s r8, r8' % w, '%s r9, r9, #1' % d, '%s r9, r9' % w,
          'subs r7, r7, #1', 'it pl', 'subpl r6, r6, #3', 'uxth r7, r7']
    return s + NEXT(CYC[op])


# --- stack (bank 0, native mode: no page-1 clamp) ----------------------------

def push8():          # r0 = byte
    return ['ldrh r1, [r4, #CPU_SP]'] + WR8('b0') + ['sub r3, r1, #1', 'strh r3, [r4, #CPU_SP]']


def push16():         # r0 = word: high at sp, low at sp-1, the high one first
    return ['ldrh r1, [r4, #CPU_SP]', 'sub r1, r1, #1', 'uxth r1, r1'] + WR16('b0', True) + \
           ['sub r3, r1, #1', 'strh r3, [r4, #CPU_SP]']


def pull8():          # -> r0
    return ['ldrh r1, [r4, #CPU_SP]', 'add r1, r1, #1', 'uxth r1, r1'] + RD8('b0') + ['strh r1, [r4, #CPU_SP]']


def pull16():         # -> r0
    return ['ldrh r1, [r4, #CPU_SP]', 'add r1, r1, #1', 'uxth r1, r1'] + RD16('b0') + \
           ['add r3, r1, #1', 'strh r3, [r4, #CPU_SP]']


def pc_after(dst):
    """dst = the 16-bit pc of the next byte to fetch"""
    return ['ldr r3, [sp, #F_BIAS]', 'sub %s, r6, r3' % dst, 'uxth %s, %s' % (dst, dst)]


def save_pc0():
    """F_PC0 = the pc of this instruction's opcode (r6 is past it)"""
    return ['ldr r3, [sp, #F_BIAS]', 'sub r2, r6, r3', 'sub r2, r2, #1', 'uxth r2, r2', 'str r2, [sp, #F_PC0]']


def jump16(c):
    """r1 = new pc in the same bank, F_PC0 = this opcode's pc: charge c, the
    loop-head test (back 1..64 bytes), on to the target"""
    return ['subs r11, r11, #%d' % c, 'ldr r3, [sp, #F_BIAS]', 'add r6, r3, r1', 'ldr r2, [sp, #F_PC0]',
            'sub r0, r2, r1', 'sub r0, r0, #1', 'cmp r0, #64', 'blo.w .Lhead', 'b.w .Lcont']


def jumpk(c):
    """r1 = new pc, r2 = new K (already stored), r3 = old K: as jump16, but a
    new bank is never a loop head and always a new page"""
    lab = L('k')
    return ['cmp r2, r3', 'bne %s' % lab] + jump16(c) + [lab + ':', 'subs r11, r11, #%d' % c,
                                                       'ble.w .Lx0_r1', 'b.w .Lsync']


def gen():
    tab = {}
    for m in (0, 1):
        for x in (0, 1):
            tab[(m, x)] = ['.Lfb'] * 256

    def put(op, fn, dep, mem=True):
        """dep: which flags the handler's code depends on ('', 'm', 'x', 'mx');
        mem: the opcode has a memory operand (then only if in ASM_MEM)"""
        made = {}
        for m in (0, 1):
            for x in (0, 1):
                key = (m if 'm' in dep else 0, x if 'x' in dep else 0)
                if mem and op not in ASM_MEM and not any(
                        o == op and (fm if 'm' in dep else 0, fx if 'x' in dep else 0) == key
                        for o, fm, fx in ASM_FORMS):
                    continue
                if key not in made:
                    made[key] = add(op, key[0], key[1], fn(key[0] if 'm' in dep else 1, key[1] if 'x' in dep else 1))
                tab[(m, x)][op] = made[key]

    names = ['ORA', 'AND', 'EOR', 'ADC', 'STA', 'LDA', 'CMP', 'SBC']
    for i, name in enumerate(names):
        for off, mode in GROUP1.items():
            op = (i << 5) + off
            if name == 'STA' and mode == 'imm':
                continue
            write = name == 'STA' and mode in ('abx', 'aby', 'idy')
            put(op, lambda m, x, name=name, op=op, mode=mode: alu(name, op, m, x, mode), 'mx' if write else 'm')

    for name, base in (('ASL', 0x00), ('ROL', 0x20), ('LSR', 0x40), ('ROR', 0x60), ('DEC', 0xc0), ('INC', 0xe0)):
        for off, mode in ((6, 'dp'), (0xe, 'abs'), (0x16, 'dpx'), (0x1e, 'abx')):
            op = base + off
            put(op, lambda m, x, name=name, op=op, mode=mode: rmw(name, op, m, x, mode), 'mx' if mode == 'abx' else 'm')
    for name, ops in (('TSB', ((0x04, 'dp'), (0x0c, 'abs'))), ('TRB', ((0x14, 'dp'), (0x1c, 'abs')))):
        for op, mode in ops:
            put(op, lambda m, x, name=name, op=op, mode=mode: rmw(name, op, m, x, mode), 'm')

    for op, mode in ((0xa2, 'imm'), (0xa6, 'dp'), (0xae, 'abs'), (0xb6, 'dpy'), (0xbe, 'aby')):
        put(op, lambda m, x, op=op, mode=mode: ldxy('r8', op, x, mode), 'x')
    for op, mode in ((0xa0, 'imm'), (0xa4, 'dp'), (0xac, 'abs'), (0xb4, 'dpx'), (0xbc, 'abx')):
        put(op, lambda m, x, op=op, mode=mode: ldxy('r9', op, x, mode), 'x')
    for op, mode in ((0x86, 'dp'), (0x8e, 'abs'), (0x96, 'dpy')):
        put(op, lambda m, x, op=op, mode=mode: stxy('r8', op, m, x, mode, x), 'x')
    for op, mode in ((0x84, 'dp'), (0x8c, 'abs'), (0x94, 'dpx')):
        put(op, lambda m, x, op=op, mode=mode: stxy('r9', op, m, x, mode, x), 'x')
    for op, mode in ((0x64, 'dp'), (0x74, 'dpx'), (0x9c, 'abs'), (0x9e, 'abx')):
        put(op, lambda m, x, op=op, mode=mode: stxy('0', op, m, x, mode, m), 'mx' if mode == 'abx' else 'm')
    for op, mode in ((0xe0, 'imm'), (0xe4, 'dp'), (0xec, 'abs')):
        put(op, lambda m, x, op=op, mode=mode: cpxy('r8', op, x, mode), 'x')
    for op, mode in ((0xc0, 'imm'), (0xc4, 'dp'), (0xcc, 'abs')):
        put(op, lambda m, x, op=op, mode=mode: cpxy('r9', op, x, mode), 'x')
    for op, mode in ((0x24, 'dp'), (0x2c, 'abs'), (0x34, 'dpx'), (0x3c, 'abx'), (0x89, 'imm')):
        put(op, lambda m, x, op=op, mode=mode: bit(op, m, x, mode), 'm')
    put(0x54, lambda m, x: mvx(0x54, x, 'add'), 'x')
    put(0x44, lambda m, x: mvx(0x44, x, 'sub'), 'x')

    # --- accumulator / implied -------------------------------------------
    def acc(op, m8, m16):
        put(op, lambda m, x: (m8 if m else m16) + NEXT(CYC[op]), 'm', False)

    acc(0x0a, ['ubfx r3, r7, #7, #1', 'strb r3, [r4, #CPU_C]', 'lsl r2, r7, #1', 'bfi r7, r2, #0, #8'] + setzn8('r7'),
        ['ubfx r3, r7, #15, #1', 'strb r3, [r4, #CPU_C]', 'lsl r7, r7, #1', 'uxth r7, r7'] + setzn16('r7'))
    acc(0x4a, ['and r3, r7, #1', 'strb r3, [r4, #CPU_C]', 'ubfx r2, r7, #1, #7', 'bfi r7, r2, #0, #8'] + setzn8('r7'),
        ['and r3, r7, #1', 'strb r3, [r4, #CPU_C]', 'lsr r7, r7, #1'] + setzn16('r7'))
    acc(0x2a, ['ldrb r3, [r4, #CPU_C]', 'orr r2, r3, r7, lsl #1', 'ubfx r3, r2, #8, #1', 'strb r3, [r4, #CPU_C]',
               'bfi r7, r2, #0, #8'] + setzn8('r7'),
        ['ldrb r3, [r4, #CPU_C]', 'orr r2, r3, r7, lsl #1', 'ubfx r3, r2, #16, #1', 'strb r3, [r4, #CPU_C]',
         'uxth r7, r2'] + setzn16('r7'))
    acc(0x6a, ['ldrb r3, [r4, #CPU_C]', 'and r2, r7, #1', 'strb r2, [r4, #CPU_C]', 'ubfx r0, r7, #1, #7',
               'orr r0, r0, r3, lsl #7', 'bfi r7, r0, #0, #8'] + setzn8('r7'),
        ['ldrb r3, [r4, #CPU_C]', 'and r2, r7, #1', 'strb r2, [r4, #CPU_C]', 'lsr r7, r7, #1',
         'orr r7, r7, r3, lsl #15'] + setzn16('r7'))
    acc(0x1a, ['add r2, r7, #1', 'bfi r7, r2, #0, #8'] + setzn8('r7'), ['add r7, r7, #1', 'uxth r7, r7'] + setzn16('r7'))
    acc(0x3a, ['sub r2, r7, #1', 'bfi r7, r2, #0, #8'] + setzn8('r7'), ['sub r7, r7, #1', 'uxth r7, r7'] + setzn16('r7'))
    acc(0x8a, ['bfi r7, r8, #0, #8'] + setzn8('r7'), ['mov r7, r8'] + setzn16('r7'))     # TXA
    acc(0x98, ['bfi r7, r9, #0, #8'] + setzn8('r7'), ['mov r7, r9'] + setzn16('r7'))     # TYA

    def idx(op, x8, x16):
        put(op, lambda m, x: (x8 if x else x16) + NEXT(CYC[op]), 'x', False)

    for op, r, d in ((0xe8, 'r8', 'add'), (0xc8, 'r9', 'add'), (0xca, 'r8', 'sub'), (0x88, 'r9', 'sub')):
        idx(op, ['%s %s, %s, #1' % (d, r, r), 'uxtb %s, %s' % (r, r)] + setzn8(r),
            ['%s %s, %s, #1' % (d, r, r), 'uxth %s, %s' % (r, r)] + setzn16(r))
    idx(0xaa, ['uxtb r8, r7'] + setzn8('r8'), ['mov r8, r7'] + setzn16('r8'))           # TAX
    idx(0xa8, ['uxtb r9, r7'] + setzn8('r9'), ['mov r9, r7'] + setzn16('r9'))           # TAY
    idx(0x9b, ['mov r9, r8'] + setzn8('r9'), ['mov r9, r8'] + setzn16('r9'))            # TXY
    idx(0xbb, ['mov r8, r9'] + setzn8('r8'), ['mov r8, r9'] + setzn16('r8'))            # TYX
    idx(0xba, ['ldrb r8, [r4, #CPU_SP]'] + setzn8('r8'), ['ldrh r8, [r4, #CPU_SP]'] + setzn16('r8'))   # TSX

    def imp(op, lines, mem=False):
        put(op, lambda m, x: lines + NEXT(CYC[op]), '', mem)

    imp(0x9a, ['strh r8, [r4, #CPU_SP]'])                                    # TXS
    imp(0x1b, ['strh r7, [r4, #CPU_SP]'])                                    # TCS
    imp(0x3b, ['ldrh r7, [r4, #CPU_SP]'] + setzn16('r7'))                    # TSC
    imp(0x5b, ['strh r7, [r4, #CPU_DP]'] + setzn16('r7'))                    # TCD
    imp(0x7b, ['ldrh r7, [r4, #CPU_DP]'] + setzn16('r7'))                    # TDC
    imp(0xeb, ['rev16 r7, r7'] + setzn8('r7'))                               # XBA
    imp(0xea, [])                                                            # NOP
    imp(0x42, ['add r6, r6, #1'])                                            # WDM
    for op, fld, v in ((0x18, 'CPU_C', 0), (0x38, 'CPU_C', 1), (0xd8, 'CPU_D', 0), (0xf8, 'CPU_D', 1),
                       (0xb8, 'CPU_V', 0), (0x78, 'CPU_I', 1)):
        imp(op, ['mov.w r0, #%d' % v, 'strb r0, [r4, #%s]' % fld])
    imp(0x58, ['mov.w r0, #0', 'strb r0, [r4, #CPU_I]', 'mov.w r5, #0'])  # CLI: an IRQ may be due

    # REP / SEP / PLP / PHP: the P byte
    imp_raw = lambda op, lines, mem=False: put(op, lambda m, x: lines, '', mem)
    imp_raw(0xc2, ['ldrb r1, [r6], #1', 'bl .Lgetp', 'bic r0, r0, r1', 'subs r11, r11, #%d' % CYC[0xc2], 'b.w .Lsetp'])
    imp_raw(0xe2, ['ldrb r1, [r6], #1', 'bl .Lgetp', 'orr r0, r0, r1', 'subs r11, r11, #%d' % CYC[0xe2], 'b.w .Lsetp'])
    imp_raw(0x28, pull8() + ['subs r11, r11, #%d' % CYC[0x28], 'b.w .Lsetp'], True)
    imp_raw(0x08, ['bl .Lgetp', 'ldr lr, [sp, #F_TAB]'] + push8() + NEXT(CYC[0x08]), True)

    # --- stack -------------------------------------------------------------
    put(0x48, lambda m, x: ['mov r0, r7'] + (push8() if m else push16()) + NEXT(CYC[0x48] + (0 if m else 1)), 'm')
    put(0xda, lambda m, x: ['mov r0, r8'] + (push8() if x else push16()) + NEXT(CYC[0xda] + (0 if x else 1)), 'x')
    put(0x5a, lambda m, x: ['mov r0, r9'] + (push8() if x else push16()) + NEXT(CYC[0x5a] + (0 if x else 1)), 'x')
    put(0x68, lambda m, x: ((pull8() + ['bfi r7, r0, #0, #8'] + setzn8('r0')) if m else
                            (pull16() + ['mov r7, r0'] + setzn16('r0'))) + NEXT(CYC[0x68] + (0 if m else 1)), 'm')
    for op, r in ((0xfa, 'r8'), (0x7a, 'r9')):
        put(op, lambda m, x, op=op, r=r: ((pull8() + ['mov %s, r0' % r] + setzn8(r)) if x else
                                          (pull16() + ['mov %s, r0' % r] + setzn16(r))) + NEXT(CYC[op] + (0 if x else 1)), 'x')
    imp(0x8b, ['ldrb r0, [r4, #CPU_DB]'] + push8(), True)                          # PHB
    imp(0x4b, ['ldrb r0, [r4, #CPU_K]'] + push8(), True)                           # PHK
    imp(0x0b, ['ldrh r0, [r4, #CPU_DP]'] + push16(), True)                         # PHD
    imp(0xab, pull8() + ['strb r0, [r4, #CPU_DB]'] + setzn8('r0'), True)           # PLB
    imp(0x2b, pull16() + ['strh r0, [r4, #CPU_DP]'] + setzn16('r0'), True)         # PLD
    imp(0xf4, ['ldrh r0, [r6], #2', 'movw r2, #0xc2ae', 'cmp r0, r2', 'it eq', 'moveq r9, #0'] + push16(), True)   # PEA (cpu.c's y = 0)
    imp(0xd4, am_dp()[0] + RD16('b0') + push16(), True)                            # PEI
    imp(0x62, ['ldrh r0, [r6], #2', 'sxth r0, r0'] + pc_after('r2') + ['add r0, r0, r2', 'uxth r0, r0'] + push16(), True)  # PER

    # --- branches ------------------------------------------------------------
    conds = {0x10: ('cmp r10, #0', 'bmi'), 0x30: ('cmp r10, #0', 'bpl'),
             0x50: ('ldrb r0, [r4, #CPU_V]\n        cmp r0, #0', 'bne'), 0x70: ('ldrb r0, [r4, #CPU_V]\n        cmp r0, #0', 'beq'),
             0x90: ('ldrb r0, [r4, #CPU_C]\n        cmp r0, #0', 'bne'), 0xb0: ('ldrb r0, [r4, #CPU_C]\n        cmp r0, #0', 'beq'),
             0xd0: ('lsls r0, r10, #1', 'beq'), 0xf0: ('lsls r0, r10, #1', 'bne')}    # (test, not-taken branch)
    for op, (test, nt) in conds.items():
        lab = L('nt')
        put(op, lambda m, x, op=op, test=test, nt=nt, lab=lab:
            ['ldrsb r1, [r6], #1'] + test.split('\n        ') + ['%s %s' % (nt, lab), 'b.w .Ltaken%d' % (CYC[op] + 1),
                                                                lab + ':'] + NEXT(CYC[op]), '', False)
    put(0x80, lambda m, x: ['ldrsb r1, [r6], #1', 'b.w .Ltaken%d' % CYC[0x80]], '', False)                    # BRA
    put(0x82, lambda m, x: save_pc0() + ['ldrh r1, [r6], #2', 'sxth r1, r1'] + pc_after('r2') +
        ['add r1, r1, r2', 'uxth r1, r1'] + jump16(CYC[0x82]), '', False)                                      # BRL

    # --- jumps ---------------------------------------------------------------
    put(0x4c, lambda m, x: save_pc0() + ['ldrh r1, [r6], #2'] + jump16(CYC[0x4c]), '', False)                  # JMP a
    put(0x6c, lambda m, x: save_pc0() + ['ldrh r1, [r6], #2'] + RD16('b0') + ['mov r1, r0'] + jump16(CYC[0x6c]), '')
    iax = lambda: ['ldrh r1, [r6], #2', 'add r1, r1, r8', 'uxth r1, r1', 'ldrb r2, [r4, #CPU_K]',
                   'orr r1, r1, r2, lsl #16'] + RD16('k')
    put(0x7c, lambda m, x: save_pc0() + iax() + ['mov r1, r0'] + jump16(CYC[0x7c]), '')                   # JMP (a,x)
    put(0x5c, lambda m, x: save_pc0() + ['ldrh r1, [r6], #2', 'ldrb r2, [r6], #1', 'ldrb r3, [r4, #CPU_K]',
                                         'strb r2, [r4, #CPU_K]'] + jumpk(CYC[0x5c]), '', False)               # JML al
    put(0xdc, lambda m, x: save_pc0() + ['ldrh r1, [r6], #2'] + RD24_b0() +
        ['uxth r1, r0', 'lsr r2, r0, #16', 'ldrb r3, [r4, #CPU_K]', 'strb r2, [r4, #CPU_K]'] + jumpk(CYC[0xdc]), '')
    # JSR a / JSR (a,x) / JSL: push pc - 1 (and K), then jump; F_TMP holds the target
    put(0x20, lambda m, x: save_pc0() + ['ldrh r1, [r6], #2', 'str r1, [sp, #F_TMP]'] + pc_after('r0') +
        ['sub r0, r0, #1', 'uxth r0, r0'] + push16() + ['ldr r1, [sp, #F_TMP]'] + jump16(CYC[0x20]), '')
    put(0xfc, lambda m, x: save_pc0() + iax() + ['str r0, [sp, #F_TMP]'] + pc_after('r0') +
        ['sub r0, r0, #1', 'uxth r0, r0'] + push16() + ['ldr r1, [sp, #F_TMP]'] + jump16(CYC[0xfc]), '')
    put(0x22, lambda m, x: save_pc0() + ['ldrh r1, [r6], #2', 'ldrb r2, [r6], #1', 'orr r1, r1, r2, lsl #16',
                                         'str r1, [sp, #F_TMP]', 'ldrb r0, [r4, #CPU_K]'] + push8() + pc_after('r0') +
        ['sub r0, r0, #1', 'uxth r0, r0'] + push16() +
        ['ldr r1, [sp, #F_TMP]', 'lsr r2, r1, #16', 'uxth r1, r1', 'ldrb r3, [r4, #CPU_K]', 'strb r2, [r4, #CPU_K]'] +
        jumpk(CYC[0x22]), '')
    put(0x60, lambda m, x: save_pc0() + pull16() + ['add r1, r0, #1', 'uxth r1, r1'] + jump16(CYC[0x60]), '')   # RTS
    put(0x6b, lambda m, x: save_pc0() + pull16() + ['add r0, r0, #1', 'uxth r0, r0', 'str r0, [sp, #F_TMP]'] + pull8() +
        ['mov r2, r0', 'ldr r1, [sp, #F_TMP]', 'ldrb r3, [r4, #CPU_K]', 'strb r2, [r4, #CPU_K]'] + jumpk(CYC[0x6b]), '')
    return tab


TAB = gen()

HEADER = r'''@ GENERATED by tools/gen_sa1_engine.py -- edit the generator, not this file.
@
@ The SA-1's 65816 in Thumb-2 (the generator's docstring describes it). Built
@ with SNES_SA1_T2 (the device, and the QEMU harness); the host runs the C copy.
@
@ int sa1_engine_run(Cpu *cpu, Sa1Run *run)
@   runs while run->debt > 0; returns 0 debt spent, 1 C runs the next
@   instruction, 3 a wait loop is proven (sa1.c sleeps it), 4 a run-ahead's
@   watched write happened (run->stop).
        .syntax unified
        .cpu    cortex-m7
        .thumb
#include "cpu_thumb2_offsets.h"
#include "sa1_engine.h"

        .equ    F_SA1, 0
        .equ    F_RUN, 4
        .equ    F_TAB, 8
        .equ    F_BIAS, 12              @ r6 - F_BIAS = the 16-bit pc
        .equ    F_IRAM, 16
        .equ    F_LAPST, 20             @ the debt when this lap began
        .equ    F_BASE, 24              @ the code page
        .equ    F_PC0, 28
        .equ    F_OPS, 32
        .equ    F_LAPOPS, 36
        .equ    F_TMP, 40
        .equ    FRAME, 48

#if defined(SNES_PERF) && !defined(SNES_PROFLITE)
#define SA1E_COUNT 1
#endif

        .section .text.sa1_engine,"ax",%progbits
        .balign 32
        .global sa1_engine_run
        .type   sa1_engine_run, %function
        .thumb_func
sa1_engine_run:
        push    {r3, r4, r5, r6, r7, r8, r9, r10, r11, lr}
        sub     sp, sp, #FRAME
        mov     r4, r0
        str     r1, [sp, #F_RUN]
        ldr     r2, [r4, #CPU_MEM]
        str     r2, [sp, #F_SA1]
        movw    r3, #SA1E_IRAM
        add     r3, r2, r3
        str     r3, [sp, #F_IRAM]
        ldr     r11, [r1, #RUN_DEBT]
        ldr     r3, [r1, #RUN_LAPCYC]
        add     r3, r3, r11
        str     r3, [sp, #F_LAPST]
        mov.w   r3, #0
        str     r3, [sp, #F_OPS]
        ldr     r3, [r1, #RUN_LAPINS]
        rsb     r3, r3, #0
        str     r3, [sp, #F_LAPOPS]
        bl      .Lreload
        ldrh    r1, [r4, #CPU_PC]
        b       .Lsync

@ A, X, Y, N/Z from the Cpu struct (entry, after C ran an opcode)
.Lreload:
        ldrh    r7, [r4, #CPU_A]
        ldrh    r8, [r4, #CPU_X]
        ldrh    r9, [r4, #CPU_Y]
        ldrb    r2, [r4, #CPU_Z]
        ldrb    r3, [r4, #CPU_N]
        lsl     r10, r3, #31
        eor     r2, r2, #1
        orr     r10, r10, r2
        bx      lr

@ r1 = the 16-bit pc: what C must see (an interrupt due, WAI, STP, emulation
@ mode), the dispatch table for m / x, the code page; then the next opcode.
@ A run-ahead asked to stop (Sa1Run.stop: a write it watches): return 4.
.Lsync:
        ldr     r0, [sp, #F_RUN]
        ldrb    r2, [r0, #RUN_STOP]
        cmp     r2, #0
        bne     .Lsync_stop
        ldrb    r0, [r4, #CPU_STOPPED]
        ldrb    r2, [r4, #CPU_WAITING]
        orr     r0, r0, r2
        ldrb    r2, [r4, #CPU_NMIWANTED]
        orr     r0, r0, r2
        ldrb    r2, [r4, #CPU_IRQWANTED]
        ldrb    r3, [r4, #CPU_I]
        bic     r2, r2, r3
        orr     r0, r0, r2
        ldrb    r2, [r4, #CPU_E]
        orrs    r0, r0, r2
        bne     .Lsync_c
        ldrb    r2, [r4, #CPU_MF]
        ldrb    r3, [r4, #CPU_XF]
        orr     r2, r3, r2, lsl #1
        movw    r3, #:lower16:sa1e_tab
        movt    r3, #:upper16:sa1e_tab
        add     lr, r3, r2, lsl #10
        str     lr, [sp, #F_TAB]
        ldr     r2, [sp, #F_SA1]
        ldrb    r3, [r4, #CPU_K]
        lsr     r0, r1, #13
        orr     r0, r0, r3, lsl #3
        ldr     r3, [r2, r0, lsl #2]    @ the read map's page of the code
        cbz     r3, .Lsync_c
        sub     r0, r3, #0x30000000     @ AHB SRAM (BW-RAM): strongly-ordered, no
        lsrs    r0, r0, #28             @ unaligned halfword operand fetches
        beq     .Lsync_c
        sub     r0, r3, #0x24000000     @ the uncached start of AXI SRAM: the same
        cmp     r0, #0x4b000
        blo     .Lsync_c
        ubfx    r0, r1, #0, #13
        add     r2, r0, #3
        lsrs    r2, r2, #13
        bne     .Lsync_c                @ the page's last 3 bytes: C, bytewise
        str     r3, [sp, #F_BASE]
        add     r6, r3, r0
        movw    r2, #0x1ffc
        add     r5, r3, r2
        sub     r2, r6, r1
        str     r2, [sp, #F_BIAS]
''' + '\n'.join(code(dispatch())) + r'''
.Lsync_c:
        mov.w   r0, #1
        b       .Lexit_r1
.Lsync_stop:
        mov.w   r0, #4
        b       .Lexit_r1

@ the pc left the code page (or a C write asked for a resync: r5 = 0)
.Lpage:
        ldr     r1, [sp, #F_BIAS]
        sub     r1, r6, r1
        uxth    r1, r1
        b       .Lsync

.Lx0:
        ldr     r1, [sp, #F_BIAS]
        sub     r1, r6, r1
        uxth    r1, r1
.Lx0_r1:
        mov.w   r0, #0
@ r0 = the return code, r1 = the 16-bit pc: the state back to Cpu and Sa1Run
.Lexit_r1:
        strh    r1, [r4, #CPU_PC]
        strh    r7, [r4, #CPU_A]
        strh    r8, [r4, #CPU_X]
        strh    r9, [r4, #CPU_Y]
        lsls    r2, r10, #1
        ite     eq
        moveq   r2, #1
        movne   r2, #0
        strb    r2, [r4, #CPU_Z]
        lsr     r2, r10, #31
        strb    r2, [r4, #CPU_N]
        ldr     r1, [sp, #F_RUN]
        str     r11, [r1, #RUN_DEBT]
        ldr     r2, [sp, #F_LAPST]
        sub     r2, r2, r11
        str     r2, [r1, #RUN_LAPCYC]
        ldr     r2, [sp, #F_OPS]
        ldr     r3, [r1, #RUN_OPS]
        add     r3, r3, r2
        str     r3, [r1, #RUN_OPS]
        ldr     r3, [sp, #F_LAPOPS]
        sub     r2, r2, r3
        str     r2, [r1, #RUN_LAPINS]
        add     sp, sp, #FRAME
        pop     {r3, r4, r5, r6, r7, r8, r9, r10, r11, pc}

@ After a jump, r6 at the target: the next instruction, wherever it is
.Lcont:
        cmp     r11, #0
        ble.w   .Lx0
        ldr     r0, [sp, #F_BASE]
        sub     r0, r6, r0
        lsrs    r0, r0, #13
        bne.w   .Lpage
        cmp     r6, r5
        bhi.w   .Lpage
''' + '\n'.join(code(dispatch())) + r'''

@ After a jump back of 1..64 bytes in the bank (r6 at the target, clocks
@ charged): sa1_loop_head's proof on this engine's snapshot
.Lhead:
        ldr     r1, [sp, #F_BIAS]
        sub     r1, r6, r1
        uxth    r1, r1
HEADCORE_A
        b.w     .Lcont

@ An opcode C runs: r0 = it, r6 just past it
.Lfb:
        ldr     r3, [sp, #F_BIAS]
        sub     r1, r6, r3
        uxth    r1, r1
        sub     r2, r1, #1
        uxth    r2, r2
        ldrb    r3, [r4, #CPU_K]
        orr     r2, r2, r3, lsl #16
        str     r2, [sp, #F_PC0]
        strh    r1, [r4, #CPU_PC]
        strh    r7, [r4, #CPU_A]
        strh    r8, [r4, #CPU_X]
        strh    r9, [r4, #CPU_Y]
        lsls    r2, r10, #1
        ite     eq
        moveq   r2, #1
        movne   r2, #0
        strb    r2, [r4, #CPU_Z]
        lsr     r2, r10, #31
        strb    r2, [r4, #CPU_N]
        movw    r3, #:lower16:sa1_cycles_per_opcode
        movt    r3, #:upper16:sa1_cycles_per_opcode
        ldrb    r3, [r3, r0]
        strb    r3, [r4, #CPU_CYCLESUSED]
        ldr     r2, [sp, #F_RUN]
        str     r11, [r2, #RUN_WDEBT]   @ the debt it starts with (sa1_ahead)
        mov     r1, r0
        mov     r0, r4
        bl      sa1cpu_thumb2_fallback  @ -> cyclesUsed
        sub     r11, r11, r0
        bl      .Lreload
        ldrh    r1, [r4, #CPU_PC]
        ldr     r2, [sp, #F_PC0]
        ldrb    r3, [r4, #CPU_K]
        cmp     r3, r2, lsr #16
        bne     .Lfb_go                 @ a new bank: not a loop head
        uxth    r2, r2
        sub     r0, r2, r1
        sub     r0, r0, #1
        cmp     r0, #64
        bhs     .Lfb_go
HEADCORE_B
.Lfb_go:
        cmp     r11, #0
        ble.w   .Lx0_r1
        b.w     .Lsync

@ r0 = the P byte (from N/Z in r10 and the flags in Cpu); keeps r1
.Lgetp:
        lsr     r0, r10, #31
        lsl     r0, r0, #7
        lsls    r2, r10, #1
        it      eq
        orreq   r0, r0, #2
        ldrb    r2, [r4, #CPU_V]
        orr     r0, r0, r2, lsl #6
        ldrb    r2, [r4, #CPU_MF]
        orr     r0, r0, r2, lsl #5
        ldrb    r2, [r4, #CPU_XF]
        orr     r0, r0, r2, lsl #4
        ldrb    r2, [r4, #CPU_D]
        orr     r0, r0, r2, lsl #3
        ldrb    r2, [r4, #CPU_I]
        orr     r0, r0, r2, lsl #2
        ldrb    r2, [r4, #CPU_C]
        orr     r0, r0, r2
        bx      lr

@ cpu_setFlags(r0) in native mode (the engine never runs in emulation mode),
@ clocks already charged; the table for the new m / x, and a resync (r5 = 0)
@ for an IRQ the I flag may now let in
.Lsetp:
        ubfx    r2, r0, #7, #1
        lsl     r10, r2, #31
        tst     r0, #2
        it      eq
        orreq   r10, r10, #1
        ubfx    r2, r0, #6, #1
        strb    r2, [r4, #CPU_V]
        ubfx    r2, r0, #5, #1
        strb    r2, [r4, #CPU_MF]
        ubfx    r2, r0, #4, #1
        strb    r2, [r4, #CPU_XF]
        ubfx    r2, r0, #3, #1
        strb    r2, [r4, #CPU_D]
        ubfx    r2, r0, #2, #1
        strb    r2, [r4, #CPU_I]
        and     r2, r0, #1
        strb    r2, [r4, #CPU_C]
        tst     r0, #0x10
        itt     ne
        uxtbne  r8, r8
        uxtbne  r9, r9
        ubfx    r2, r0, #4, #2
        movw    r3, #:lower16:sa1e_tab
        movt    r3, #:upper16:sa1e_tab
        add     lr, r3, r2, lsl #10
        str     lr, [sp, #F_TAB]
        mov.w   r5, #0
''' + '\n'.join(code(NEXT(0)))


def headcore(tag):
    """r1 = the head's pc. Proven: return 3; else arm this head, fall through."""
    arm = L('arm')
    s = ['ldr r0, [sp, #F_RUN]', 'ldrh r2, [r0, #RUN_CANDPC]', 'cmp r2, r1', 'bne.w %s' % arm,
         'ldrb r2, [r0, #RUN_CANDK]', 'ldrb r3, [r4, #CPU_K]', 'cmp r2, r3', 'bne.w %s' % arm,
         'ldrb r2, [r0, #RUN_DIRTY]', 'ldr r3, [sp, #F_IRAM]', 'ldr r3, [r3, #SA1E_WDIRTY]', 'orrs r2, r2, r3',
         'bne.w %s' % arm, 'ldr r2, [r0, #RUN_EXTEV]', 'ldr r3, [r0, #RUN_LAPEV]', 'cmp r2, r3', 'bne.w %s' % arm]
    for i, r in enumerate(('r7', 'r8', 'r9', 'r10')):
        s += ['ldr r2, [r0, #(RUN_AS + %d)]' % (4 * i), 'cmp r2, %s' % r, 'bne.w %s' % arm]
    s += ['ldr r2, [r0, #(RUN_AS + 16)]', 'ldrh r3, [r4, #CPU_SP]', 'cmp r2, r3', 'bne.w %s' % arm]
    for i, off in enumerate((16, 20, 24)):
        s += ['ldr r2, [r0, #(RUN_AS + %d)]' % (20 + 4 * i), 'ldr r3, [r4, #%d]' % off, 'cmp r2, r3', 'bne.w %s' % arm]
    s += ['ldrb r2, [r0, #(RUN_AS + 32)]', 'ldrb r3, [r4, #CPU_E]', 'cmp r2, r3', 'bne.w %s' % arm,
          'mov.w r0, #3', 'b.w .Lexit_r1', arm + ':',
          'strh r1, [r0, #RUN_CANDPC]', 'ldrb r3, [r4, #CPU_K]', 'strb r3, [r0, #RUN_CANDK]', 'mov.w r2, #0',
          'strb r2, [r0, #RUN_DIRTY]', 'ldr r3, [sp, #F_IRAM]', 'str r2, [r3, #SA1E_WDIRTY]',
          'ldr r2, [r0, #RUN_EXTEV]', 'str r2, [r0, #RUN_LAPEV]', 'str r11, [sp, #F_LAPST]',
          'ldr r2, [sp, #F_OPS]', 'str r2, [sp, #F_LAPOPS]']
    for i, r in enumerate(('r7', 'r8', 'r9', 'r10')):
        s += ['str %s, [r0, #(RUN_AS + %d)]' % (r, 4 * i)]
    s += ['ldrh r2, [r4, #CPU_SP]', 'str r2, [r0, #(RUN_AS + 16)]']
    for i, off in enumerate((16, 20, 24)):
        s += ['ldr r2, [r4, #%d]' % off, 'str r2, [r0, #(RUN_AS + %d)]' % (20 + 4 * i)]
    s += ['ldrb r2, [r4, #CPU_E]', 'strb r2, [r0, #(RUN_AS + 32)]', 'ldr lr, [sp, #F_TAB]']
    return '\n'.join(code(s))


def taken_tails():
    """.LtakenN: r1 = the branch displacement, r6 past it: charge N, the loop
    head test (back 1..64 bytes: displacement -66..-3), on to the target"""
    s = []
    for c in sorted({CYC[op] + 1 for op in (0x10, 0x30, 0x50, 0x70, 0x90, 0xb0, 0xd0, 0xf0)} | {CYC[0x80]}):
        s += ['.Ltaken%d:' % c] + code(['add r6, r6, r1', 'subs r11, r11, #%d' % c, 'adds r0, r1, #66',
                                         'cmp r0, #63', 'bls.w .Lhead', 'b.w .Lcont'])
    return s


STUBS = r'''
@ The slow halves of the bus, called from the handlers' cold paths: r1 (the
@ address) kept, r0 the value read; lr = the dispatch table again on return.
@ C may clobber r0-r3, ip and lr only (AAPCS), the state is in r4-r11.
        .macro  STUB name, call, prep
        .thumb_func
\name:
        push    {r1, lr}
        \prep
        ldr     r0, [sp, #(8 + F_SA1)]
        bl      \call
        pop     {r1, r2}
        ldr     lr, [sp, #F_TAB]
        bx      r2
        .endm

        .balign 4
        STUB    sa1e_rd8_s, sa1e_read, "nop"
        STUB    sa1e_rd16_s_b0, sa1e_rd16, "add r2, r1, #1; uxth r2, r2"
        STUB    sa1e_rd16_s_l, sa1e_rd16, "add r2, r1, #1; bic r2, r2, #0xff000000"
        STUB    sa1e_rd16_s_k, sa1e_rd16, "add r2, r1, #1; uxth r2, r2; and r3, r1, #0xff0000; orr r2, r2, r3"
        STUB    sa1e_rd24_s, sa1e_rd24_b0, "nop"

@ writes: sa1e_wr8 / sa1e_wr16 / sa1e_wr16r return non-zero for a write to
@ the SA-1's registers, which can move the read map or raise an interrupt:
@ then r5 = 0, a resync before the next opcode. Sa1Run.wdebt gets the debt
@ the instruction started with (r11: its extra cycles come at its end), for
@ the run-ahead (sa1.c, sa1_ahead).
        .macro  WSTUB name, call, prep
        .thumb_func
\name:
        push    {r1, lr}
        ldr     ip, [sp, #(8 + F_RUN)]
        str     r11, [ip, #RUN_WDEBT]
        \prep
        ldr     r0, [sp, #(8 + F_SA1)]
        bl      \call
        cmp     r0, #0
        it      ne
        movne   r5, #0
        pop     {r1, r2}
        ldr     lr, [sp, #F_TAB]
        bx      r2
        .endm

        WSTUB   sa1e_wr8_s, sa1e_wr8, "uxtb r2, r0"
        WSTUB   sa1e_wr16_s_b0, sa1e_wr16, "uxth r3, r0; add r2, r1, #1; uxth r2, r2"
        WSTUB   sa1e_wr16_s_b0r, sa1e_wr16r, "uxth r3, r0; add r2, r1, #1; uxth r2, r2"
        WSTUB   sa1e_wr16_s_l, sa1e_wr16, "uxth r3, r0; add r2, r1, #1; bic r2, r2, #0xff000000"
        WSTUB   sa1e_wr16_s_lr, sa1e_wr16r, "uxth r3, r0; add r2, r1, #1; bic r2, r2, #0xff000000"
'''


def main():
    s = HEADER.replace('HEADCORE_A', headcore('a')).replace('HEADCORE_B', headcore('b'))
    s += '\n\n' + '\n'.join(taken_tails()) + '\n'
    s += '\n@ --- the handlers ---\n'
    for lab, lines in bodies:
        s += '\n' + lab + ':\n' + '\n'.join(code(lines)) + '\n'
    s += STUBS
    s += '\n@ the dispatch tables: m = 0/1, x = 0/1, 256 entries each\n'
    s += '        .section .rodata.sa1_engine,"a",%progbits\n        .balign 4\n        .global sa1e_tab\nsa1e_tab:\n'
    for m in (0, 1):
        for x in (0, 1):
            s += '@ m=%d x=%d\n' % (m, x)
            for op in range(256):
                s += '        .word   %s + 1\n' % TAB[(m, x)][op]
    s += '        .section .note.GNU-stack,"",%progbits\n'
    open(OUT, 'w').write(s)
    print('wrote %s: %d handlers' % (OUT, len(bodies)))


if __name__ == '__main__':
    main()
