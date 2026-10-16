#!/usr/bin/env python3
"""Generate per-game Super FX (GSU) block code: gsu_zsnes/gsu_blocks_<name>.inc.

The GSU interpreter (gsu_zsnes/fx_ops.h, the ZSNES core) dispatches one opcode at
a time through tables of handlers, with its state in globals. A block is the
straight-line GSU code from a hot address to the next control transfer, compiled
to C with the decode done here (built in src/snes/gsu_zsnes/gsu_blocks.c): registers, flags and the RAM/ROM bank pointers
live in locals, immediates and register numbers are constants, prefixes
(TO/FROM/WITH/ALTn) are resolved, and a transfer back to the block's own head
loops in C. Each instruction does what its ZSNES handler does (the templates
below are transcribed from fx_ops.h, quirks included), so the state after a
block is the interpreter's. STOP, CMODE, RPIX, JMP/LJMP, writes to R15 other than
the jumps, the b-table contexts and prefixed or multi-byte delay slots are not
compiled: a block ends before them and the interpreter runs them.

Budget: MainLoop runs instruction m+1 only while m <= NumberOfOpcodes (as it was
when the loop entered the block). A block runs whole iterations of K top-level
instructions, so it checks c + K - 1 <= N once per iteration; when an iteration
does not fit it returns 0 (nothing run, the interpreter goes on) or exits at its
head.

Selection follows tools/arcade/nec_hle_build.py in gw-arcade-core: a host
histogram of top-level GSU instructions (env GSU_PCHIST, fx_ops.h), the hottest
addresses up to --cover of them. Block heads: the selected addresses entered
from outside the selection, the jump/LOOP targets the histogram shows, and every
selected address a block exits to (iterated to a fixed point).

Usage:
  gen_gsu_blocks.py --rom yoshi.sfc --hist yoshi_all.hist --name yoshi \\
      --tables src/snes/gsu_zsnes/c_fxtable.inc --ops src/snes/gsu_zsnes/fx_ops.h \\
      --cover 0.95 -o src/snes/gsu_zsnes/gsu_blocks_yoshi.inc
"""
import argparse, itertools, re, sys

# ---------------------------------------------------------------------------
# ROM access: the GSU's view (banks $00-$3F = 32 KB LoROM halves, $40-$5F linear)

class Rom:
    def __init__(self, path):
        d = open(path, 'rb').read()
        if len(d) % 1024 == 512:
            d = d[512:]
        self.d = d

    def byte(self, bank, a):
        a &= 0xFFFF
        if bank < 0x40:
            off = (bank * 0x8000) % len(self.d) + (a & 0x7FFF)
        elif bank < 0x60:
            off = ((bank & 0x1F) * 0x10000) % len(self.d) + a
        else:
            raise ValueError('not ROM: %02x' % bank)
        return self.d[off]

# ---------------------------------------------------------------------------
# What ZSNES runs for each (context, ALT, opcode)

def parse_tables(path):
    src = open(path).read()

    def arr(name):
        m = re.search(r'static fxop\* const ' + name + r'\[256\] = \{(.*?)\};', src, re.S)
        items = re.findall(r'(c_FxOp\w+)', m.group(1))
        assert len(items) == 256, name
        return items

    def patches(name):
        m = re.search(r'static fxpatch const ' + name + r'\[\] = \{(.*?)\};', src, re.S)
        return [(int(a, 0), b) for a, b in
                re.findall(r'\{\s*(0x[0-9A-Fa-f]+|\d+)\s*,\s*(c_FxOp\w+)\s*\}', m.group(1))]

    alts = ('', 'A1', 'A2', 'A3')
    base = [arr('init_FxTable' + a) for a in alts]
    top = [arr('init_FxTabled' + a) for a in alts]
    tb = [list(t) for t in base]
    tc = [list(t) for t in base]
    for i, a in enumerate(alts):
        for o, n in patches('init_FxTableb' + a):
            tb[i][o] = n
        for o, n in patches('init_FxTablec' + a):
            tc[i][o] = n
    return {'top': top, 'base': base, 'b': tb, 'c': tc}


def parse_handlers(path):
    """handler name -> (helper, arg) for the one-line and macro-made handlers."""
    src = open(path).read()
    h = {}
    for name, fn, arg in re.findall(
            r'^void (c_FxOp\w+)\(void\) \{ (fx_\w+)\(([^)]*)\); \}', src, re.M):
        h[name] = (fn, arg.strip())
    conds = ['always', 'ge', 'lt', 'ne', 'eq', 'pl', 'mi', 'cc', 'cs', 'vc', 'vs']
    for sfx in ('', 'b', 'c'):
        for i, c in enumerate(conds):
            h['c_FxOp%s%02X' % (sfx, 5 + i)] = ('branch_' + c, sfx)
    for k in ('b', 'c'):
        for n in range(14):
            h['c_FxOp%s%X' % (k, 0x10 + n)] = ('torn_' + k, str(n))
        for n in range(15):
            h['c_FxOp%s%X' % (k, 0xB0 + n)] = ('fromrn_' + k, str(n))
    return h


def oplen(op):
    if 0x05 <= op <= 0x0F or 0xA0 <= op <= 0xAF:
        return 2
    if 0xF0 <= op <= 0xFF:
        return 3
    return 1

PREFIX = set(range(0x10, 0x30)) | set(range(0xB0, 0xC0)) | {0x3D, 0x3E, 0x3F}

def ilen(rom, bank, a):
    """Length of the top-level instruction at a: its prefixes plus the opcode."""
    n = 0
    while rom.byte(bank, a + n) in PREFIX and n < 8:
        n += 1
    return n + oplen(rom.byte(bank, a + n))

# ops that read the program counter (R15 operand forms, LINK, CACHE, SM/SMS R15):
# not compiled in a delay slot, where the PC is the jump target
PC_READERS = {'c_FxOp5F', 'c_FxOp5FA1', 'c_FxOp6F', 'c_FxOp6FA1', 'c_FxOp6FA3', 'c_FxOp7F',
              'c_FxOp7FA1', 'c_FxOp8F', 'c_FxOp8FA1', 'c_FxOpCF', 'c_FxOpCFA1', 'c_FxOpAFA2',
              'c_FxOpFFA2', 'c_FxOp02'} | {'c_FxOp%X' % o for o in range(0x91, 0x95)}

# ---------------------------------------------------------------------------
# Code generation

class Unsupported(Exception):
    pass


class Gen:
    """One block. Locals: r0..r14; sz/cy/ov (SfxSignZero/Carry/Overflow);
    rmem (SfxRAMMem), lra (SfxLastRamAdr), rbuf (SfxRomBuffer), crom/cmask
    (SfxCROM/SfxCROMMask)."""

    def __init__(self, rom, tables, handlers, bank):
        self.rom, self.t, self.h, self.bank = rom, tables, handlers, bank
        self.lines = []
        self.used = set()     # registers used (loaded at entry)
        self.wr = set()       # registers written (stored at exit)
        self.bytes = set()    # ROM addresses the block depends on
        self.glob = set()     # cached globals the block touches: 'ram', 'lra', 'rom'
        self.alt = 0

    def b(self, a):
        if not 0 <= a <= 0xFFFF:
            raise Unsupported('address wraps')
        self.bytes.add(a)
        return self.rom.byte(self.bank, a)

    def sb(self, a):
        v = self.b(a)
        return v - 256 if v >= 128 else v

    def w(self, a):
        return self.b(a) | self.b(a + 1) << 8

    def e(self, s):
        self.lines.append(s)

    def R(self, n):
        if n == 15:
            raise Unsupported('R15 operand')
        self.used.add(n)
        return 'r%d' % n

    def D(self, n):
        if n == 15:
            raise Unsupported('R15 destination')
        if n == 13:
            self.k13 = None      # R13 no longer the constant an IWT gave it
        self.used.add(n)
        self.wr.add(n)
        return 'r%d' % n

    def ram(self, addr):
        self.glob.add('ram')
        return 'BLK_RAM(%s)' % addr

    def last(self, addr):
        self.glob.add('ram'); self.glob.add('lra')
        self.e('lra = rmem + (zreg)(%s);' % addr)

    def r14(self):
        self.used.add(14)
        self.glob.add('rom')
        self.e('rbuf = crom + (r14 & cmask);')

    def flags_add(self, s, d, rhs, cin):
        self.e('{ u4 const a = %s, rhs = %s, sum = (a & 0xFFFFu) + (rhs & 0xFFFFu) + %s;' % (self.R(s), rhs, cin))
        self.e('  ov = (ov & ~0xFFu) | (u4)((~(a ^ rhs) & (a ^ sum) & 0x8000u) != 0);')
        self.e('  cy = (cy & ~0xFFu) | (u4)((sum & 0x10000u) != 0);')
        self.e('  sz = (a & 0xFFFF0000u) | (sum & 0xFFFFu); %s = sz; }' % self.D(d))

    def flags_sub(self, s, d, rhs, bin_, store):
        self.e('{ u4 const a = %s, rhs = %s, diff = (a & 0xFFFFu) - (rhs & 0xFFFFu) - %s;' % (self.R(s), rhs, bin_))
        self.e('  ov = (ov & ~0xFFu) | (u4)(((a ^ rhs) & (a ^ diff) & 0x8000u) != 0);')
        self.e('  cy = (cy & ~0xFFu) | (u4)((diff & 0x10000u) == 0);')
        self.e('  sz = (a & 0xFFFF0000u) | (diff & 0xFFFFu);%s }' % (' %s = sz;' % self.D(d) if store else ''))

    def logic(self, s, d, o, rhs):
        self.e('sz = %s %s %s; %s = sz;' % (self.R(s), o, rhs, self.D(d)))

    def mult(self, s, d, rhs, sign):
        a, b = '(%s & 0xFFu)' % self.R(s), '(%s & 0xFFu)' % rhs
        prod = '(u4)((s4)(s1)%s * (s4)(s1)%s)' % (a, b) if sign else '(%s * %s)' % (a, b)
        self.e('sz = %s & 0xFFFFu; %s = sz;' % (prod, self.D(d)))

    def plot(self, ctx):
        table = {'top': 'FxTabled', 'base': 'FxTable', 'c': 'FxTablec'}[ctx]
        self.used.update((1, 2)); self.wr.add(1)
        self.e('r1 = BLK_PLOT(%s, r1, r2);' % table)

    def op(self, name, X, s, d, ctx):
        """The non-prefix opcode `name` at X, source s, destination d."""
        pc = (X + 1) & 0xFFFF            # fx_pc_rel() inside its handler
        op = self.b(X)
        if op == 0x4C and self.alt == 0 and ctx in ('top', 'base', 'c'):
            return self.plot(ctx)        # whatever variant CMODE patched in
        fn, arg = self.h.get(name, (None, None))
        n = int(arg) if arg is not None and arg.isdigit() else None
        if fn in ('fx_addrn', 'fx_adcrn', 'fx_adirn', 'fx_adcirn'):
            rhs = self.R(n) if fn in ('fx_addrn', 'fx_adcrn') else '%du' % n
            return self.flags_add(s, d, rhs, '(cy & 1u)' if fn in ('fx_adcrn', 'fx_adcirn') else '0u')
        if fn in ('fx_subrn', 'fx_sbcrn', 'fx_subirn'):
            rhs = self.R(n) if fn != 'fx_subirn' else '%du' % n
            return self.flags_sub(s, d, rhs, '(u4)((cy & 0xFFu) == 0)' if fn == 'fx_sbcrn' else '0u', True)
        if fn == 'fx_cmprn':
            return self.flags_sub(s, d, self.R(n), '0u', False)
        if fn == 'fx_andrn':  return self.logic(s, d, '&', self.R(n))
        if fn == 'fx_bicrn':  return self.logic(s, d, '&', '(%s ^ 0xFFFFu)' % self.R(n))
        if fn == 'fx_andirn': return self.logic(s, d, '&', '%du' % n)
        if fn == 'fx_bicirn': return self.logic(s, d, '&', '0x%Xu' % (n ^ 0xFFFF))
        if fn == 'fx_orrn':   return self.logic(s, d, '|', self.R(n))
        if fn == 'fx_ori':    return self.logic(s, d, '|', '%du' % n)
        if fn == 'fx_xorrn':  return self.logic(s, d, '^', self.R(n))
        if fn == 'fx_xori':   return self.logic(s, d, '^', '%du' % n)
        if fn in ('fx_incrn', 'fx_decrn'):
            r = self.D(n)
            self.e('%s = (%s & 0xFFFF0000u) | ((%s %s 1u) & 0xFFFFu); sz = %s;'
                   % (r, r, r, '+' if fn == 'fx_incrn' else '-', r))
            return
        if fn == 'fx_multrn':   return self.mult(s, d, self.R(n), True)
        if fn == 'fx_umultrn':  return self.mult(s, d, self.R(n), False)
        if fn == 'fx_multirn':  return self.mult(s, d, '%du' % n, True)
        if fn == 'fx_umultirn': return self.mult(s, d, '%du' % n, False)
        if fn == 'fx_stwrn':
            r = self.R(n)
            self.last(r)
            self.e('{ u4 const v = %s; *%s = (u1)v; *%s = (u1)(v >> 8); }'
                   % (self.R(s), self.ram(r), self.ram('%s ^ 1u' % r)))
            return
        if fn == 'fx_stbrn':
            r = self.R(n)
            self.last(r)
            self.e('*%s = (u1)%s;' % (self.ram(r), self.R(s)))
            return
        if fn == 'fx_ldwrn':
            r = self.R(n)
            self.last(r)
            self.e('{ u4 const k = %s; %s = (u4)*%s | ((u4)*%s << 8); }'
                   % (r, self.D(d), self.ram('k'), self.ram('k ^ 1u')))
            return
        if fn == 'fx_ldbrn':
            r = self.R(n)
            self.last(r)
            self.e('{ u4 const k = %s; %s = *%s; }' % (r, self.D(d), self.ram('k')))
            return
        if fn == 'fx_ibtrn':
            r = self.D(n)
            self.e('%s = (%s & 0xFFFF0000u) | 0x%04Xu;' % (r, r, self.sb(X + 1) & 0xFFFF))
            return
        if fn == 'fx_iwtrn':
            self.e('%s = 0x%04Xu;' % (self.D(n), self.w(X + 1)))
            if n == 13:
                self.k13 = self.w(X + 1)
            return
        if fn == 'fx_lmrn':
            a = self.w(X + 1)
            self.last('0x%04Xu' % a)
            r = self.D(n)
            self.e('%s = (%s & 0xFFFF0000u) | (u4)*%s | ((u4)*%s << 8);'
                   % (r, r, self.ram('0x%04Xu' % a), self.ram('0x%04Xu' % (a ^ 1))))
            return
        if fn == 'fx_smrn':
            a = self.w(X + 1)
            self.last('0x%04Xu' % a)
            self.e('*%s = (u1)%s; *%s = (u1)(%s >> 8);'
                   % (self.ram('0x%04Xu' % a), self.R(n), self.ram('0x%04Xu' % (a ^ 1)), self.R(n)))
            return
        if fn == 'fx_lmsrn':
            a = self.b(X + 1) * 2
            self.last('0x%04Xu' % a)
            r = self.D(n)
            self.e('%s = (%s & 0xFFFF0000u) | (u4)*%s | ((u4)*%s << 8);'
                   % (r, r, self.ram('0x%04Xu' % a), self.ram('0x%04Xu' % (a + 1))))
            return
        if fn == 'fx_smsrn':
            a = self.b(X + 1) * 2
            self.last('0x%04Xu' % a)
            self.e('*%s = (u1)%s; *%s = (u1)(%s >> 8);'
                   % (self.ram('0x%04Xu' % a), self.R(n), self.ram('0x%04Xu' % (a + 1)), self.R(n)))
            return
        if fn == 'fx_linkn':
            self.links.add((pc + n) & 0xFFFF)
            r = self.D(11)
            self.e('%s = (%s & 0xFFFF0000u) | 0x%04Xu;' % (r, r, (pc + n) & 0xFFFF))
            return
        if fn == 'fx_cache':
            base = pc & 0xFFF0
            self.e('if (SfxCBR != 0x%04Xu && (SfxCacheActive & 0xFFu) != 1) { SfxCBR = 0x%04Xu; SfxCacheActive = 1; }'
                   % (base, base))
            return
        if fn == 'fx_fmult':
            self.e('{ u4 const prod = (u4)((s4)(s2)(u2)%s * (s4)(s2)(u2)%s), hi = (prod >> 16) & 0xFFFFu;'
                   % (self.R(s), self.R(6)))
            self.e('  %s = hi;' % self.D(d))
            if n == 1:
                self.e('  %s = (r4 & 0xFFFF0000u) | (prod & 0xFFFFu);' % self.D(4))
            self.e('  sz = hi; cy = (cy & ~0xFFu) | ((prod >> 15) & 1u); }')
            return
        if fn == 'torn_c':        # MOVE (TO inside WITH)
            self.e('%s = %s;' % (self.D(n), self.R(s)))
            return
        if fn == 'fromrn_c':      # MOVES (FROM inside WITH)
            self.e('sz = %s; %s = sz; ov = (ov & ~0xFFu) | ((sz & 0xFFu) >> 7);' % (self.R(n), self.D(d)))
            return
        sp = name[len('c_FxOp'):]
        if sp == '01':            # NOP
            return
        if sp in ('5F', '5FA1'):
            return self.flags_add(s, d, '0x%04Xu' % pc, '(cy & 1u)' if sp == '5FA1' else '0u')
        if sp in ('6F', '6FA1'):
            return self.flags_sub(s, d, '0x%04Xu' % pc, '(u4)((cy & 0xFFu) == 0)' if sp == '6FA1' else '0u', True)
        if sp == '6FA3':
            return self.flags_sub(s, d, '0x%04Xu' % pc, '0u', False)
        if sp == '7F':   return self.logic(s, d, '&', '0x%04Xu' % pc)
        if sp == '7FA1': return self.logic(s, d, '&', '0x%04Xu' % (pc ^ 0xFFFF))
        if sp == 'CF':   return self.logic(s, d, '|', '0x%04Xu' % pc)
        if sp == 'CFA1': return self.logic(s, d, '^', '0x%04Xu' % pc)
        if sp == '8F':   return self.mult(s, d, '0x%04Xu' % pc, True)
        if sp == '8FA1': return self.mult(s, d, '0x%04Xu' % pc, False)
        if sp == '03':            # LSR
            self.e('{ u4 const a = %s; cy = (cy & ~0xFFu) | (a & 1u); sz = (a & 0xFFFF0000u) | ((a & 0xFFFFu) >> 1); %s = sz; }'
                   % (self.R(s), self.D(d)))
            return
        if sp == '04':            # ROL
            self.e('{ u4 const a = %s, k = cy; sz = (a & 0xFFFF0000u) | ((((a & 0xFFFFu) << 1) | (k & 1u)) & 0xFFFFu);'
                   % self.R(s))
            self.e('  cy = (k & ~0xFFu) | (k & 0xFEu) | ((a >> 15) & 1u); %s = sz; }' % self.D(d))
            return
        if sp == '96':            # ASR
            self.e('{ u4 const a = %s, lo = a & 0xFFFFu; cy = (cy & ~0xFFu) | (lo & 1u);' % self.R(s))
            self.e('  sz = (a & 0xFFFF0000u) | ((lo >> 1) | (lo & 0x8000u)); %s = sz; }' % self.D(d))
            return
        if sp == '96A1':          # DIV2
            self.e('{ u4 const a = %s, lo = a & 0xFFFFu;' % self.R(s))
            self.e('  if (lo != 0xFFFFu) { cy = (cy & ~0xFFu) | (lo & 1u); sz = (a & 0xFFFF0000u) | ((lo >> 1) | (lo & 0x8000u)); }')
            self.e('  else { cy = (cy & ~0xFFu) | 1u; sz = 0; }')
            self.e('  %s = sz; }' % self.D(d))
            return
        if sp == '97':            # ROR
            self.e('{ u4 const a = %s; sz = (a & 0xFFFF0000u) | (((cy & 1u) << 15) | ((a & 0xFFFFu) >> 1));' % self.R(s))
            self.e('  cy = (cy & ~0xFFu) | (a & 1u); %s = sz; }' % self.D(d))
            return
        if sp == '4D':            # SWAP
            self.e('{ u4 const a = %s; sz = (a & 0xFFFF0000u) | ((a & 0xFFu) << 8) | ((a >> 8) & 0xFFu); %s = sz; }'
                   % (self.R(s), self.D(d)))
            return
        if sp == '4F':            # NOT
            return self.logic(s, d, '^', '0xFFFFu')
        if sp == '95':            # SEX
            self.e('sz = (u4)(s4)(s1)(u1)%s & 0xFFFFu; %s = sz;' % (self.R(s), self.D(d)))
            return
        if sp == '9E':            # LOB
            self.e('{ u4 const v = %s & 0xFFu; %s = v; sz = v << 8; }' % (self.R(s), self.D(d)))
            return
        if sp == 'C0':            # HIB
            self.e('sz = %s & 0xFF00u; %s = sz >> 8;' % (self.R(s), self.D(d)))
            return
        if sp == '70':            # MERGE
            self.e('{ u4 const v = (%s & 0xFF00u) | ((%s >> 8) & 0xFFu); %s = v;' % (self.R(7), self.R(8), self.D(d)))
            self.e('  sz = (v & 0xF0F0u) ? 0u : 1u; if (v & 0x8080u) sz |= 0x80000u;')
            self.e('  ov = (v & 0xC0C0u) ? 1u : 0u; cy = (v & 0xE0E0u) ? 1u : 0u; }')
            return
        if sp in ('DE', 'EE'):    # INC/DEC R14
            r = self.D(14)
            self.e('%s = (%s & 0xFFFF0000u) | ((%s %s 1u) & 0xFFFFu); sz = %s;' % (r, r, r, '+' if sp == 'DE' else '-', r))
            return self.r14()
        if sp == 'AE':            # IBT R14
            self.e('r14 = (%s & 0xFFFF0000u) | 0x%04Xu;' % (self.D(14), self.sb(X + 1) & 0xFFFF))
            return self.r14()
        if sp == 'FE':            # IWT R14
            self.e('%s = 0x%04Xu;' % (self.D(14), self.w(X + 1)))
            return self.r14()
        if sp == 'AEA1':          # LMS R14
            a = self.b(X + 1) * 2
            self.last('0x%04Xu' % a)
            self.e('r14 = (%s & 0xFFFF0000u) | (u4)*%s | ((u4)*%s << 8);'
                   % (self.D(14), self.ram('0x%04Xu' % a), self.ram('0x%04Xu' % (a + 1))))
            return self.r14()
        if sp == 'FEA1':          # LM R14
            a = self.w(X + 1)
            self.last('0x%04Xu' % a)
            self.e('r14 = (%s & 0xFFFF0000u) | (u4)*%s | ((u4)*%s << 8);'
                   % (self.D(14), self.ram('0x%04Xu' % a), self.ram('0x%04Xu' % (a ^ 1))))
            return self.r14()
        if sp == 'AFA2':          # SMS R15: store the PC
            a = self.b(X + 1) * 2
            self.last('0x%04Xu' % a)
            self.e('*%s = 0x%02Xu; *%s = 0x%02Xu;' % (self.ram('0x%04Xu' % a), pc & 0xFF, self.ram('0x%04Xu' % (a + 1)), pc >> 8))
            return
        if sp == 'FFA2':          # SM R15
            a = self.w(X + 1)
            self.last('0x%04Xu' % a)
            self.e('*%s = 0x%02Xu; *%s = 0x%02Xu;' % (self.ram('0x%04Xu' % a), pc & 0xFF, self.ram('0x%04Xu' % (a ^ 1)), pc >> 8))
            return
        if sp == '90':            # SBK
            self.glob.update(('ram', 'lra'))
            self.e('{ u4 const a = (u4)(lra - rmem), v = %s; *%s = (u1)v; *%s = (u1)(v >> 8); }'
                   % (self.R(s), self.ram('a'), self.ram('a ^ 1u')))
            return
        if sp in ('EF', 'EFA1', 'EFA2', 'EFA3', 'DF'):
            self.glob.add('rom')
            g = '(u4)*(u1 const*)(uintptr_t)rbuf'
            if sp == 'EF':   self.e('%s = %s;' % (self.D(d), g))
            if sp == 'EFA1': self.e('%s = (%s & 0xFFu) | (%s << 8);' % (self.D(d), self.R(s), g))
            if sp == 'EFA2': self.e('%s = (%s & 0xFF00u) | %s;' % (self.D(d), self.R(s), g))
            if sp == 'EFA3':
                r = self.D(d)
                self.e('%s = (%s & 0xFFFF0000u) | ((u4)(s4)(s1)(u1)%s & 0xFFFFu);' % (r, r, g))
            if sp == 'DF':   self.e('fx_color(%s);' % g)
            return
        if sp == '4E':            # COLOR
            self.e('fx_color(%s);' % self.R(s))
            return
        if sp == 'DFA2':          # RAMB
            self.glob.add('ram')
            self.e('{ u4 const k = %s & (SfxnRamBanks - 1u); SfxRAMBR = k; rmem = SfxRAMMem = ((zreg)k << 16) + (uintptr_t)sfxgameram; }'
                   % self.R(s))
            return
        if sp == 'DFA3':          # ROMB
            self.glob.add('rom')
            self.e('{ u4 const k = %s & 0x7Fu; SfxROMBR = k; SfxCROM = crom = SfxMemTable[k]; SfxCROMMask = cmask = SfxMemMask[k]; }'
                   % self.R(s))
            return
        if sp == 'c1E':           # MOVE R14 (inside WITH)
            self.e('%s = %s;' % (self.D(14), self.R(s)))
            return self.r14()
        raise Unsupported(name)

    def decode(self, addrs, ctx='top'):
        """Resolve the prefixes of one instruction whose bytes are read at the
        addresses `addrs` yields, dispatching from table `ctx`: (opcode address,
        opcode, handler name, src, dst, ctx, R14 tail)."""
        self.alt, s, d, tail = 0, 0, 0, False
        for a in addrs:
            op = self.b(a)
            name = self.t[ctx][self.alt][op]
            fn, arg = self.h.get(name, (None, None))
            if name in ('c_FxOp1E', 'c_FxOp2E') and ctx in ('top', 'base') and self.alt == 0:
                if name == 'c_FxOp1E':
                    d, ctx = 14, 'base'
                else:
                    s = d = 14
                    ctx = 'c'
                tail = True
            elif fn in ('fx_torn', 'fx_fromrn', 'fx_with') and ctx in ('top', 'base') and self.alt == 0:
                n = int(arg)
                if fn == 'fx_torn':
                    d, ctx = n, 'base'
                elif fn == 'fx_fromrn':
                    s, ctx = n, 'base'
                else:
                    s = d = n
                    ctx = 'c'
            elif fn == 'fx_with' and ctx == 'c' and self.alt == 0:
                s = d = int(arg)
            elif fn == 'fx_alt':
                mode, table = (x.strip() for x in arg.split(','))
                self.alt |= int(mode)
                ctx = {'FxTable': 'base', 'FxTablec': 'c'}[table]
            elif op in PREFIX and not (fn or '').startswith(('torn_c', 'fromrn_c')) and name != 'c_FxOpc1E':
                raise Unsupported('prefix %s in %s' % (name, ctx))
            else:
                return a, op, name, s, d, ctx, tail
        raise Unsupported('prefix chain')

    def delay(self, at, path, ctx, s=0, d=0):
        """The delay slot at `at` of a transfer to `path` (the program counter it
        runs with). A one-byte op runs as itself; a prefix takes its opcode from
        `path` (the GSU fetches it from there). Returns where the program goes on."""
        op = self.b(at)
        if op in PREFIX:
            if s or d:
                raise Unsupported('delay prefix after a prefix')
            a, op2, name, s, d, ctx2, tail = self.decode(itertools.chain([at], itertools.count(path)), ctx)
            if a == at:
                raise Unsupported('delay prefix')
            self.op(name, a, s, d, ctx2)    # every control op raises in op()
            if tail:
                self.r14()
            return a + oplen(op2)
        name = self.t[ctx][0][op]
        fn = self.h.get(name, (None,))[0] or ''
        if oplen(op) != 1 or op in (0x00, 0x3C) or 0x98 <= op <= 0x9D \
                or fn.startswith('branch_') or name in PC_READERS:
            raise Unsupported('delay slot %02x' % op)
        self.alt = 0
        self.op(name, at, s, d, ctx)
        return path

    def instr(self, X, starts):
        """Emit the top-level instruction at X (after its label). Returns
        (instructions counted, fall-through address or None). Transfers are
        markers resolved by emit_func: BLK_TO(a) goes to a, BLK_EXIT(a) leaves
        before the instruction at a, BLK_DYN(v) goes to the address in local v."""
        a, op, name, s, d, ctx, tail = self.decode(itertools.count(X))
        fn = self.h.get(name, (None,))[0] or ''
        if fn.startswith('branch_'):
            # after TO/FROM/WITH the branch still runs; its delay slot gets the
            # prefix's registers and runs in the table that branch dispatches in
            if a != X and (self.alt or self.h[name][1] not in ('', 'c')):
                raise Unsupported('prefixed branch')
            dctx = 'c' if self.h[name][1] == 'c' else 'base'
            pre = (s, d) if a != X else (0, 0)
            cond = {'always': '1', 'ge': '((((sz >> 15) ^ ov) & 1u) == 0)', 'lt': '((((sz >> 15) ^ ov) & 1u) != 0)',
                    'ne': '((sz & 0xFFFFu) != 0)', 'eq': '((sz & 0xFFFFu) == 0)', 'pl': '((sz & 0x88000u) == 0)',
                    'mi': '((sz & 0x88000u) != 0)', 'cc': '((cy & 1u) == 0)', 'cs': '((cy & 1u) != 0)',
                    'vc': '((ov & 1u) == 0)', 'vs': '((ov & 1u) != 0)'}[fn[len('branch_'):]]
            T = a + 2 + self.sb(a + 1)
            if not 0 <= T <= 0xFFFF:
                raise Unsupported('branch wraps')
            self.b(a + 2)
            self.e('c += 1u;')

            def path(to):
                nxt = self.delay(a + 2, to, dctx, *pre)
                if tail:
                    self.r14()
                return nxt
            if cond == '1':
                self.e('BLK_TO(0x%04X);' % path(T))
                return 1, None
            self.e('if (%s) {' % cond)
            self.e('BLK_TO(0x%04X);' % path(T))
            self.e('}')
            return 1, path(a + 3)
        if name == 'c_FxOp3C' and a == X:      # LOOP: the delay slot is its own instruction
            r = self.D(12)
            self.used.add(13)
            if self.b(X + 1) in PREFIX:
                # the slot's opcode comes from R13 when the loop is taken: compiled
                # for the R13 an IWT in this chain set, else for the loop targets
                # the profile saw (nearest four); any other R13 leaves to the interpreter
                if getattr(self, 'k13', None) is not None:
                    B = [self.k13]
                else:   # IWT R13s that ran in the 1 KB before it, then profiled targets
                    iwt = [self.w(i + 1) for i in range(X - 1, max(X - 0x400, 0) - 1, -1)
                           if (self.bank, i) in self.seen and self.rom.byte(self.bank, i) == 0xFD]
                    near = sorted((t for t in self.dyn if abs(t - X) <= 0x400), key=lambda t: abs(t - X))
                    B = list(dict.fromkeys(iwt + near))[:4]
                if not B:
                    raise Unsupported('LOOP delay prefix, unknown R13')
                self.e('if ((((%s & 0xFFFF0000u) | ((%s - 1u) & 0xFFFFu)) != 0) && !(%s)) { BLK_EXIT(0x%04X); }'
                       % (r, r, ' || '.join('r13 == 0x%04Xu' % t for t in B), X))
                self.e('c += 2u;')
                self.e('%s = (%s & 0xFFFF0000u) | ((%s - 1u) & 0xFFFFu); sz = %s;' % (r, r, r, r))
                self.e('if (r12 != 0) {')
                for t in B:
                    self.e('if (r13 == 0x%04Xu) {' % t)
                    self.e('BLK_TO(0x%04X);' % self.delay(X + 1, t, 'top'))
                    self.e('}')
                self.e('}')
                return 2, self.delay(X + 1, X + 2, 'top')
            self.e('c += 2u;')
            self.e('%s = (%s & 0xFFFF0000u) | ((%s - 1u) & 0xFFFFu); sz = %s;' % (r, r, r, r))
            self.e('{ int const taken = r12 != 0;')
            self.delay(X + 1, None, 'top')
            self.e('if (taken) { BLK_DYN(r13); } }')
            return 2, X + 2
        if self.h.get(name, (None,))[0] == 'fx_jmprn' and a == X:   # JMP Rn
            n = int(self.h[name][1])
            self.e('c += 2u;')
            self.e('{ u4 const tgt = %s;' % self.R(n))   # the PC is set before the slot runs
            self.delay(X + 1, None, 'top')
            self.e('BLK_DYN(tgt); }')
            return 2, None
        if name in ('c_FxOpFF', 'c_FxOpAF') and a == X:   # IWT / IBT R15
            if name == 'c_FxOpFF':
                T, slot = self.w(X + 1), X + 3
            else:
                T, slot = self.sb(X + 1) & 0xFFFF, X + 2
            self.e('c += 2u;')
            self.e('BLK_TO(0x%04X);' % self.delay(slot, T, 'top'))
            return 2, None
        self.e('c += 1u;')
        self.op(name, a, s, d, ctx)
        if tail:
            self.r14()
        return 1, a + oplen(op)


class Chain:
    """Straight-line hot code from `head` on: through untaken branches, up to an
    unconditional transfer, an instruction not compiled, or cold code."""

    def __init__(self, rom, tables, handlers, bank, head, sel, dyn, maxlen, seen=frozenset()):
        g = self.g = Gen(rom, tables, handlers, bank)
        g.dyn = {t for b, t in dyn if b == bank}
        g.links = set()
        g.k13 = None
        g.seen = seen
        self.bank, self.head = bank, head
        self.starts = []          # (address, instructions counted)
        a = head
        while True:
            if len(self.starts) == maxlen or (a != head and (bank, a) not in sel):
                g.e('BLK_TO(0x%04X);' % a)
                break
            mark = len(g.lines)
            saved = (set(g.used), set(g.wr), set(g.bytes), set(g.glob))
            g.e('@@L %04X' % a)
            try:
                cnt, nxt = g.instr(a, {s for s, _ in self.starts} | {a})
            except Unsupported as ex:
                del g.lines[mark:]
                g.used, g.wr, g.bytes, g.glob = saved
                self.stop = str(ex).split()[0]
                g.e('BLK_TO(0x%04X);' % a)
                break
            self.starts.append((a, cnt))
            if nxt is None:
                self.stop = 'transfer'
                break
            a = nxt
        else:
            pass
        if not hasattr(self, 'stop'):
            self.stop = 'cold/length'
        text = '\n'.join(g.lines)
        self.targets = {int(x, 16) for x in re.findall(r'BLK_TO\(0x([0-9A-F]{4})\)', text)}
        self.dyn_used = 'BLK_DYN' in text


def emit_func(chs, entries, out, cold=False):
    """One C function for the chains `chs` (they share locals; transfers between
    them are gotos). entries: the addresses the lookup table enters at, the
    first chain's head first."""
    bank = chs[0].bank
    rom = chs[0].g.rom
    gs = [ch.g for ch in chs]
    inside = {a for ch in chs for a, _ in ch.starts}
    links = set().union(*(g.links for g in gs))
    dyn = set().union(*(g.dyn for g in gs))
    labels = set(entries) | (set().union(*(ch.targets for ch in chs)) & inside)
    if any(ch.dyn_used for ch in chs):
        labels |= (dyn | links) & inside
    fname = 'gsu_fn_%02x_%04x' % (bank, chs[0].head)
    entries = entries + sorted(labels - set(entries))   # every label is an entry too
    deps = set()

    def exit_at(t):
        deps.add(t)
        return 'epc = 0x%04Xu; ecx = 0x%02Xu; goto done;' % (t + 1, rom.byte(bank, t))

    def to(m):
        t = int(m.group(1), 16)
        return 'goto L%04X;' % t if t in labels else exit_at(t)

    def dyn_code(m):
        v = m.group(1)
        return ' '.join('if (%s == 0x%04Xu) goto L%04X;' % (v, t, t) for t in sorted((dyn | links) & labels)) + \
            ' epc = %s + 1u; ecx = FxCodeBase[%s & FxCodeMask]; goto done;' % (v, v)
    body = []
    for ch in chs:
        # budget at a label: the instructions up to the next label
        kmax, run = {}, 0
        for a, cnt in reversed(ch.starts):
            run += cnt
            if a in labels:
                kmax[a] = run
                run = 0
        for line in ch.g.lines:
            m = re.match(r'@@L ([0-9A-F]{4})$', line)
            if m:
                a = int(m.group(1), 16)
                if a in labels:
                    body.append('L%04X:' % a)
                    body.append('    if ((s4)(c + %du) > N) { if (c == 0) return 0; %s }' % (kmax[a] - 1, exit_at(a)))
                continue
            line = re.sub(r'BLK_TO\(0x([0-9A-F]{4})\);', to, line)
            line = re.sub(r'BLK_EXIT\(0x([0-9A-F]{4})\);', lambda m: exit_at(int(m.group(1), 16)), line)
            line = re.sub(r'BLK_DYN\((\w+)\);', dyn_code, line)
            body.append('    ' + line)
    used = set().union(*(g.used for g in gs))
    wr = set().union(*(g.wr for g in gs))
    glob = set().union(*(g.glob for g in gs))
    if cold:   # the profile's long tail: small rather than fast
        out.append('__attribute__((optimize("Os")))')
    out.append('static u4 %s(s4 const N, u4 const entry)' % fname)
    out.append('{')
    out.append('    u4 c = 0, epc, ecx;')
    out.append('    u4 sz = SfxSignZero, cy = SfxCarry, ov = SfxOverflow;')
    if used:
        out.append('    u4 %s;' % ', '.join('r%d = SfxR0[%d]' % (r, r) for r in sorted(used)))
    if 'ram' in glob:
        out.append('    zreg rmem = SfxRAMMem; u1 *const gram = sfxgameram; u4 const gmask = SfxRamMask;')
    if 'lra' in glob:
        out.append('    zreg lra = SfxLastRamAdr;')
    if 'rom' in glob:
        out.append('    zreg rbuf = SfxRomBuffer, crom = SfxCROM; u4 cmask = SfxCROMMask;')
    if len(entries) == 1:
        out.append('    (void)entry;')
    else:
        out.append('    switch (entry) {')
        for i, e in enumerate(entries[1:], 1):
            out.append('    case %d: goto L%04X;' % (i, e))
        out.append('    default: break;')
        out.append('    }')
    out += body
    out.append('done:')
    for r in sorted(wr):
        out.append('    SfxR0[%d] = r%d;' % (r, r))
    out.append('    SfxSignZero = sz; SfxCarry = cy; SfxOverflow = ov;')
    if 'lra' in glob:
        out.append('    SfxLastRamAdr = lra;')
    if 'rom' in glob:
        out.append('    SfxRomBuffer = rbuf;')
    out.append('    FxSeamPC = epc; FxSeamCX = ecx;')
    out.append('    return c;')
    out.append('}')
    out.append('')
    return fname, deps, entries

# ---------------------------------------------------------------------------

def plot_helper(name, variants):
    out = []
    for v in variants:
        out.append('extern void %s(void);' % v)
    out.append('/* PLOT at (r1, r2) through dispatch table `tab`; returns the new R1 */')
    out.append('static inline __attribute__((always_inline)) u4 blk_plot_%s(zreg const *const tab, u4 const r1, u4 const r2)' % name)
    out.append('{')
    out.append('    zreg const fn = tab[0x4C];')
    for v in variants:
        m = re.match(r'c_FxOp4C128([248])b(z?)(d?)(l?)$', v)
        if not m:
            raise SystemExit('unknown PLOT variant ' + v)
        depth, z, d, l = m.groups()
        zmask = {'2': '0x03u', '4': '0x0Fu', '8': '0xFFu'}[depth] if not l else '0x0Fu'
        out.append('    if (fn == (zreg)(uintptr_t)&%s) {' % v)
        out.append('        fx_plot_pixel(r1, r2, FX_PLOT_%sBPP, %s, %d, %d);' % (depth, zmask, bool(z), bool(d)))
        out.append('        return (r1 & 0xFFFF0000u) | ((r1 + 1u) & 0xFFFFu);')
        out.append('    }')
    out.append('    SfxR0[1] = r1;')
    out.append('    SfxR0[2] = r2;')
    out.append('    ((void (*)(void))(uintptr_t)fn)();')
    out.append('    return SfxR0[1];')
    out.append('}')
    out.append('#define BLK_PLOT(tab, r1, r2) blk_plot_%s(tab, r1, r2)' % name)
    out.append('')
    return out


def fnv1a(data):
    h = 0x811C9DC5
    for x in data:
        h = ((h ^ x) * 0x01000193) & 0xFFFFFFFF
    return h


def read_hist(rom, path):
    """(instruction counts by (bank, address), delay-slot jump targets)."""
    rows, dyn = {}, set()
    for line in open(path):
        if line.startswith('#'):
            continue
        k, n, op, alt = line.split()[:4]
        bank, addr = (int(x, 16) for x in k.split(':'))
        if bank >= 0x60:
            continue
        if rom.byte(bank, addr) == int(op, 16) and int(alt) == 0:
            rows[(bank, addr)] = int(n)
        else:   # PC-1 after a jump / LOOP: the delay slot ran with PC = target
            dyn.add((bank, (addr + 1) & 0xFFFF))
    return rows, dyn


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--rom', required=True)
    ap.add_argument('--hist', required=True, action='append',
                    help='file[:cover] -- a histogram and the share of it to compile (default --cover)')
    ap.add_argument('--name', required=True)
    ap.add_argument('--tables', required=True)
    ap.add_argument('--ops', required=True)
    ap.add_argument('--cover', type=float, default=0.95)
    ap.add_argument('--maxlen', type=int, default=256)
    ap.add_argument('--fnmax', type=int, default=200, help='instructions per C function')
    ap.add_argument('--hot', type=float, default=1.0,
                    help='share of the profile whose functions are compiled for speed; the rest -Os')
    ap.add_argument('--plot', default='',
                    help='PLOT variants to run inline, e.g. c_FxOp4C1284b,c_FxOp4C1284bz (the rest are called)')
    ap.add_argument('-o', required=True)
    a = ap.parse_args()
    rom = Rom(a.rom)
    tables = parse_tables(a.tables)
    handlers = parse_handlers(a.ops)

    sel, dyn, hists, seen = set(), set(), [], set()
    for spec in a.hist:
        path, _, cov = spec.partition(':')
        cov = float(cov) if cov else a.cover
        rows, d = read_hist(rom, path)
        dyn |= d
        seen |= set(rows)
        total, acc = sum(rows.values()), 0
        for p, n in sorted(rows.items(), key=lambda r: -r[1]):
            if acc >= cov * total:
                break
            sel.add(p)
            acc += n
        hists.append((path, rows, total))

    # chains, lowest address first; an address another chain already runs
    # through becomes one of its entries
    chains, owner = {}, {}
    for p in sorted(sel):
        if p in owner:
            continue
        ch = Chain(rom, tables, handlers, p[0], p[1], sel, dyn, a.maxlen, seen)
        if not ch.starts:
            continue
        chains[p] = ch
        for s, _ in ch.starts:
            owner.setdefault((p[0], s), p)
    # functions: chains joined along their hottest transfers (exits into
    # another chain, and LINK return addresses), up to --fnmax instructions
    weight = {}
    for path, rows, total in hists:
        for p, n in rows.items():
            weight[p] = weight.get(p, 0) + n / total
    parent = {p: p for p in chains}
    size = {p: len(ch.starts) for p, ch in chains.items()}

    def find(p):
        while parent[p] != p:
            parent[p] = parent[parent[p]]
            p = parent[p]
        return p
    edges = []
    for (b, h), ch in chains.items():
        for t in ch.targets | ch.g.links:
            o = owner.get((b, t))
            if o and o != (b, h):
                edges.append((weight.get((b, t), 0), (b, h), o))
    for w, x, y in sorted(edges, reverse=True):
        x, y = find(x), find(y)
        if x != y and size[x] + size[y] <= a.fnmax:
            parent[y] = x
            size[x] += size[y]
    groups = {}
    for p in sorted(chains):
        groups.setdefault(find(p), []).append(p)

    # entries: chain heads, exit targets, LOOP/jump/return targets
    entries = {p for p in chains}
    for (b, h), ch in chains.items():
        entries |= {(b, t) for t in ch.targets | ch.g.dyn | ch.g.links if (b, t) in owner}

    # the functions holding the hottest --hot share of the profile stay -O3
    fw = {r: sum(weight.get((r[0], x), 0) for p in m for x, _ in chains[p].starts) for r, m in groups.items()}
    hot, acc, tot = set(), 0.0, sum(fw.values())
    for r in sorted(fw, key=lambda r: -fw[r]):
        if acc >= a.hot * tot:
            break
        hot.add(r)
        acc += fw[r]
    out, keys, dep, stops = [], [], set(), {}
    for root, members in sorted(groups.items()):
        chs = [chains[p] for p in members]
        b = root[0]
        for ch in chs:
            stops[ch.stop] = stops.get(ch.stop, 0) + 1
        inside = {(b, x) for ch in chs for x, _ in ch.starts}
        ents = [members[0][1]] + sorted({p[1] for p in entries & inside} - {members[0][1]})
        fname, deps, ents = emit_func(chs, ents, out, root not in hot)
        for i, e in enumerate(ents):
            if e < 0xFFFF:
                keys.append(((b << 24) | ((e + 1) << 8) | rom.byte(b, e), i, fname))
        dep |= {(b, x) for ch in chs for x in ch.g.bytes} | {(b, x) for x in deps}
    covered = set(owner)
    covtxt = ', '.join('%s %.1f%%' % (re.sub(r'.*/', '', path), 100.0 * sum(n for p, n in rows.items() if p in covered) / total)
                       for path, rows, total in hists)

    nslot = 1                      # load factor <= 2/3
    while 2 * nslot < 3 * len(keys):
        nslot <<= 1
    slots = [None] * nslot
    for key, i, f in keys:
        hh = ((key * 2654435761) & 0xFFFFFFFF) >> 16
        while slots[hh & (nslot - 1)] is not None:
            hh += 1
        slots[hh & (nslot - 1)] = (key, i, f)

    # runs of ROM bytes to check, gaps of up to 32 bytes bridged (checking a
    # few more bytes is harmless and keeps the list short)
    runs = []
    for b, x in sorted(dep):
        if runs and runs[-1][0] == b and x - (runs[-1][1] + runs[-1][2]) <= 32 \
                and x + 1 - runs[-1][1] <= 255:
            runs[-1][2] = x + 1 - runs[-1][1]
        elif not (runs and runs[-1][0] == b and runs[-1][1] <= x < runs[-1][1] + runs[-1][2]):
            runs.append([b, x, 1])
    hsh = fnv1a(rom.byte(b, x + i) for b, x, n in runs for i in range(n))

    N = a.name
    txt = ['/* GENERATED by tools/gen_gsu_blocks.py (%s) -- do not edit.' % N,
           ' * %d chains in %d functions (%d -Os), %d entries, from %d selected addresses.'
           % (len(chains), len(groups), len(groups) - len(hot), len(keys), len(sel)),
           ' * Profiled top-level GSU instructions inside the chains: %s.' % covtxt,
           ' * Chain ends: %s.' % ', '.join('%s %d' % kv for kv in sorted(stops.items())),
           ' * Included by gsu_blocks.c. */', '',
           '/* fx_ram() through the locals: rmem = SfxRAMMem, gram = sfxgameram, gmask = SfxRamMask */',
           '#define BLK_RAM(a) (gram + ((rmem - (uintptr_t)gram + (a)) & gmask))', '']
    # PLOT: the variants CMODE selects in this game run inline, any other is called
    txt += plot_helper(N, [v for v in a.plot.split(',') if v])
    txt += out
    txt.append('static struct gsu_blk const gsu_blk_tab_%s[%d] = {' % (N, nslot))
    for sl in slots:
        txt.append('    { 0x%08Xu, %d, %s },' % sl if sl else '    { 0u, 0, 0 },')
    txt.append('};')
    txt.append('')
    txt.append('/* the ROM bytes the chains were compiled from (and small gaps between them):')
    txt.append('   bank << 24 | addr << 8 | count */')
    txt.append('static u4 const gsu_blk_dep_%s[] = {' % N)
    for i in range(0, len(runs), 6):
        txt.append('    ' + ' '.join('0x%08Xu,' % ((b << 24) | (x << 8) | n) for b, x, n in runs[i:i + 6]))
    txt.append('};')
    txt.append('#define GSU_BLK_%s_HASH 0x%08Xu /* FNV-1a of those bytes */' % (N.upper(), hsh))
    txt.append('#undef BLK_RAM')
    txt.append('#undef BLK_PLOT')
    open(a.o, 'w').write('\n'.join(txt) + '\n')
    print('%s: %d chains in %d functions, %d entries, %d selected addresses, covered: %s; %d dependent ROM bytes; ends %s'
          % (a.o, len(chains), len(groups), len(keys), len(sel), covtxt, len(dep), stops))


if __name__ == '__main__':
    main()
