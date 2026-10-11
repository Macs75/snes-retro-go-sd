#!/usr/bin/env python3
"""Idle-skip survey analysis: one row per game from survey/gNNN/.

For each game:
  - correctness: REC hash lines of the idle build vs the unmodified build's
    recording run (same emulated frames, no input);
  - generic skip: armed site, dots it advanced per frame (fraction of a frame);
  - where the CPU waits: the pc the NMI interrupted, most often, in the window;
  - that loop decoded from the ROM, with why the generic skip refuses it;
  - its share of the CPU work still executed (the PC histogram, window
    frames 600..4200, with the generic skip on -- so a loop the generic skip
    takes shows up small or not at all).
"""
import os, re, sys, json, zlib, collections

S = os.environ.get("SURVEY_DIR") or os.path.dirname(os.path.abspath(__file__))  # dir holding the gNNN/ run folders

# 65816: mnemonic and addressing mode per opcode
M = ("BRK ORA COP ORA TSB ORA ASL ORA PHP ORA ASL PHD TSB ORA ASL ORA "
     "BPL ORA ORA ORA TRB ORA ASL ORA CLC ORA INC TCS TRB ORA ASL ORA "
     "JSR AND JSL AND BIT AND ROL AND PLP AND ROL PLD BIT AND ROL AND "
     "BMI AND AND AND BIT AND ROL AND SEC AND DEC TSC BIT AND ROL AND "
     "RTI EOR WDM EOR MVP EOR LSR EOR PHA EOR LSR PHK JMP EOR LSR EOR "
     "BVC EOR EOR EOR MVN EOR LSR EOR CLI EOR PHY TCD JML EOR LSR EOR "
     "RTS ADC PER ADC STZ ADC ROR ADC PLA ADC ROR RTL JMP ADC ROR ADC "
     "BVS ADC ADC ADC STZ ADC ROR ADC SEI ADC PLY TDC JMP ADC ROR ADC "
     "BRA STA BRL STA STY STA STX STA DEY BIT TXA PHB STY STA STX STA "
     "BCC STA STA STA STY STA STX STA TYA STA TXS TXY STZ STA STZ STA "
     "LDY LDA LDX LDA LDY LDA LDX LDA TAY LDA TAX PLB LDY LDA LDX LDA "
     "BCS LDA LDA LDA LDY LDA LDX LDA CLV LDA TSX TYX LDY LDA LDX LDA "
     "CPY CMP REP CMP CPY CMP DEC CMP INY CMP DEX WAI CPY CMP DEC CMP "
     "BNE CMP CMP CMP PEI CMP DEC CMP CLD CMP PHX STP JML CMP DEC CMP "
     "CPX SBC SEP SBC CPX SBC INC SBC INX SBC NOP XBA CPX SBC INC SBC "
     "BEQ SBC SBC SBC PEA SBC INC SBC SED SBC PLX XCE JSR SBC INC SBC").split()
# per low nibble pattern of the classic 65816 grid, column by column
def mode(op):
    hi, lo = op >> 4, op & 15
    special = {0x00: 'imm8', 0x02: 'imm8', 0x20: 'abs', 0x22: 'long', 0x40: 'imp', 0x42: 'imm8',
               0x44: 'mv', 0x54: 'mv', 0x4c: 'abs', 0x5c: 'long', 0x60: 'imp', 0x62: 'rel16',
               0x6b: 'imp', 0x6c: '(abs)', 0x7c: '(abs,x)', 0x80: 'rel', 0x82: 'rel16',
               0x89: 'immM', 0x96: 'dp,y', 0xb6: 'dp,y', 0xbe: 'abs,y', 0xa0: 'immX', 0xa2: 'immX',
               0xc0: 'immX', 0xe0: 'immX', 0xc2: 'imm8', 0xe2: 'imm8', 0xd4: 'dp', 0xf4: 'abs',
               0xdc: '[abs]', 0xfc: '(abs,x)', 0x14: 'dp', 0x1c: 'abs', 0x04: 'dp', 0x0c: 'abs',
               0x34: 'dp,x', 0x3c: 'abs,x', 0x64: 'dp', 0x74: 'dp,x', 0x9c: 'abs', 0x9e: 'abs,x',
               0x24: 'dp', 0x2c: 'abs', 0x84: 'dp', 0x86: 'dp', 0x8c: 'abs', 0x8e: 'abs',
               0x94: 'dp,x', 0xa4: 'dp', 0xa6: 'dp', 0xac: 'abs', 0xae: 'abs', 0xb4: 'dp,x',
               0xbc: 'abs,x', 0xc4: 'dp', 0xcc: 'abs', 0xe4: 'dp', 0xec: 'abs'}
    if op in special: return special[op]
    if lo == 0: return 'rel'
    if lo == 1: return '(dp,x)' if hi % 2 == 0 else '(dp),y'
    if lo == 2: return '(dp)'
    if lo == 3: return 'sr' if hi % 2 == 0 else '(sr),y'
    if lo == 5: return 'dp' if hi % 2 == 0 else 'dp,x'
    if lo == 6: return 'dp' if hi % 2 == 0 else 'dp,x'
    if lo == 7: return '[dp]' if hi % 2 == 0 else '[dp],y'
    if lo == 9: return 'immM' if hi % 2 == 0 else 'abs,y'
    if lo == 0xd: return 'abs' if hi % 2 == 0 else 'abs,x'
    if lo == 0xe: return 'abs' if hi % 2 == 0 else 'abs,x'
    if lo == 0xf: return 'long' if hi % 2 == 0 else 'long,x'
    return 'imp'   # x8, xa, xb, xc-implied etc.
LEN = {'imp': 1, 'imm8': 2, 'dp': 2, 'dp,x': 2, 'dp,y': 2, '(dp,x)': 2, '(dp),y': 2, '(dp)': 2,
       'sr': 2, '(sr),y': 2, '[dp]': 2, '[dp],y': 2, 'rel': 2, 'abs': 3, 'abs,x': 3, 'abs,y': 3,
       '(abs)': 3, '(abs,x)': 3, '[abs]': 3, 'rel16': 3, 'mv': 3, 'long': 4, 'long,x': 4}
WRITE_MN = {'STA', 'STX', 'STY', 'STZ', 'TSB', 'TRB', 'MVN', 'MVP'}
RMW_MN = {'ASL', 'LSR', 'ROL', 'ROR', 'INC', 'DEC'}
STACK_MN = {'PHA', 'PHX', 'PHY', 'PHP', 'PHB', 'PHD', 'PHK', 'PLA', 'PLX', 'PLY', 'PLP', 'PLB', 'PLD',
            'PEA', 'PEI', 'PER', 'JSR', 'JSL', 'RTS', 'RTL', 'RTI', 'BRK', 'COP'}

class Rom:
    def __init__(self, path):
        d = open(path, 'rb').read()
        if len(d) % 1024 == 512: d = d[512:]
        self.d = d
        self.crc = '%08x' % (zlib.crc32(d) & 0xffffffff)
        ok = lambda o: o + 0x30 <= len(d) and ((d[o+0x1c] | d[o+0x1d] << 8) ^ (d[o+0x1e] | d[o+0x1f] << 8)) == 0xffff
        self.hi = ok(0xffc0) and not ok(0x7fc0)
        if len(d) > 0x400000 and ok(0x40ffc0): self.hi = True
    def rd(self, a):
        b, o = a >> 16, a & 0xffff
        if b in (0x7e, 0x7f): return None
        if self.hi:
            if (b & 0x7f) < 0x40 and o < 0x8000: return None
            return self.d[(((b & 0x3f) << 16) | o) % len(self.d)]
        if o < 0x8000: return None
        return self.d[(((b & 0x7f) << 15) | (o & 0x7fff)) % len(self.d)]

def io_addr(bank, off):
    b = bank & 0x7f
    if b < 0x40 and 0x2000 <= off < 0x8000: return True
    return False

def decode_loop(rom, pcs):
    """pcs: executed instruction starts of the loop (sorted). Returns text and flags."""
    out, flags = [], set()
    for i, q in enumerate(pcs):
        op = rom.rd(q)
        if op is None:
            flags.add('code in RAM'); out.append('%04x:RAM' % (q & 0xffff)); continue
        mn, md = M[op], mode(op)
        ln = pcs[i+1] - q if i + 1 < len(pcs) and 0 < pcs[i+1] - q <= 4 else LEN.get(md, 2 if md != 'immM' and md != 'immX' else 2)
        ops = [rom.rd(q + k) for k in range(1, ln)]
        ops = [x for x in ops if x is not None]
        txt = '%s' % mn
        if md != 'imp' and ops: txt += ' ' + {'immM': '#', 'immX': '#', 'imm8': '#'}.get(md, '') + ''.join('%02x' % x for x in reversed(ops)) + ('' if md in ('immM', 'immX', 'imm8', 'dp', 'abs', 'long', 'rel', 'rel16') else ' ' + md)
        out.append('%04x:%s' % (q & 0xffff, txt))
        if mn == 'WAI': flags.add('WAI')
        if mn in WRITE_MN: flags.add('writes')
        if mn in RMW_MN and md != 'imp': flags.add('writes')
        if mn in STACK_MN: flags.add('stack/call')
        if md in ('(dp,x)', '(dp),y', '(dp)', '[dp]', '[dp],y', 'sr', '(sr),y', '(abs)', '(abs,x)', '[abs]'): flags.add('indirect')
        if md in ('dp,x', 'dp,y', 'abs,x', 'abs,y', 'long,x'): flags.add('indexed')
        if mn in ('SEP', 'REP', 'XCE', 'TCD', 'TCS', 'PLB', 'PLD', 'SEI', 'CLI'): flags.add('changes M/X/DB/DP/I')
        if md == 'abs' and len(ops) == 2:
            a = ops[0] | ops[1] << 8
            if 0x2000 <= a < 0x8000: flags.add('reads I/O $%04x' % a if mn not in WRITE_MN else 'writes I/O $%04x' % a)
        if md == 'long' and len(ops) == 3:
            a = ops[0] | ops[1] << 8
            if io_addr(ops[2], a): flags.add('I/O $%02x%04x' % (ops[2], a))
    return out, flags

def cluster(P, seed):
    n0 = P[seed][0]
    grp = sorted(q for q in P if abs(q - seed) <= 40 and q >> 16 == seed >> 16 and P[q][0] * 4 >= n0)
    # keep the contiguous-ish run around seed (gaps <= 6 bytes)
    i = grp.index(seed); lo = hi = i
    while lo > 0 and grp[lo] - grp[lo-1] <= 6: lo -= 1
    while hi + 1 < len(grp) and grp[hi+1] - grp[hi] <= 6: hi += 1
    return grp[lo:hi+1]

def one(d):
    nm = open(os.path.join(d, 'name')).read().split('\n')
    title, region, chip, path = nm[0], nm[1], nm[2], nm[3]
    row = {'dir': os.path.basename(d), 'title': title, 'region': region, 'chip': chip, 'file': os.path.basename(path)}
    try:
        rom = Rom(path); row['crc32'] = rom.crc
    except Exception as e:
        row['crc32'] = '?'; rom = None
    st = open(os.path.join(d, 'status')).read() if os.path.exists(os.path.join(d, 'status')) else ''
    base = open(os.path.join(d, 'base.txt'), errors='replace').read() if os.path.exists(os.path.join(d, 'base.txt')) else ''
    sv = open(os.path.join(d, 'sv.txt'), errors='replace').read() if os.path.exists(os.path.join(d, 'sv.txt')) else ''
    if 'load failed' in base or 'Unsupported' in base:
        row['class'] = 'unsupported'; row['why'] = re.search(r'load failed: ([^\n]*)', base).group(1) if 'load failed' in base else 'unsupported'
        return row
    pal = 'PAL' in (re.search(r'REC: recording from power-on, (\w+)', base) or [None, ''])[1]
    row['tv'] = 'PAL' if pal else 'NTSC'
    fdots = 1364 * (312 if pal else 262)
    hb = [l for l in base.splitlines() if l.startswith('REC/')]
    hs = [l for l in sv.splitlines() if l.startswith('REC/')]
    row['hash_lines'] = len(hb)
    row['hash_diff'] = sum(1 for a, b in zip(hb, hs) if a != b) + abs(len(hb) - len(hs))
    row['status'] = st.replace('\n', ' ').strip()
    # generic skip, window frames 600..4200 (the IDLE lines are every 600)
    ff = laps = 0; site = ''; nwin = 0
    for l in open(os.path.join(d, 'sv_err.txt'), errors='replace'):
        m = re.match(r'IDLE f=(\d+) site=(\S+) mem=\d+ snap=(\d) laps=(\d+) ff=(\d+) wai=(\d+)', l)
        if m and int(m.group(1)) > 600:
            nwin += 1; ff += int(m.group(5)); laps += int(m.group(4)); site = m.group(2)
            ff += int(m.group(6))
    frames = nwin * 600 or 1
    row['site'] = site if not site.endswith(':ffff+0') else ''
    row['skip_pct'] = round(100.0 * ff / frames / fdots, 1)
    # where the NMI lands
    nmi = collections.Counter()
    nf = os.path.join(d, 'nmi.txt')
    if os.path.exists(nf):
        for l in open(nf):
            f, pc, w = l.split()
            if 600 <= int(f) < 4200: nmi[int(pc, 16)] += 1
    tot_nmi = sum(nmi.values())
    row['nmi_per_frame'] = round(tot_nmi / 3600.0, 2)
    # executed CPU work (window), clustered around the top NMI landing pcs
    P = {}
    pf = os.path.join(d, 'pch.txt')
    if os.path.exists(pf):
        for l in open(pf):
            pc, n, c = l.split(); P[int(pc, 16)] = (int(n), int(c))
    T = sum(c for n, c in P.values()) or 1
    row['cpu_units_per_frame'] = int(T / 3600)
    loops = []
    used = set()
    # seeds: where the NMI lands; a game that runs without NMI (polls $4210 /
    # $4212 or paces on IRQ) gives none, so fall back to the hottest pcs
    seeds = [pc for pc, k in nmi.most_common(12)]
    row['seed'] = 'nmi'
    if tot_nmi < 0.5 * 3600:
        seeds = [pc for pc, _ in sorted(P.items(), key=lambda kv: -kv[1][1])[:12]]
        row['seed'] = 'cpu'
    for pc in seeds:
        if pc in used or pc not in P: continue
        g = cluster(P, pc)
        used.update(g)
        share_cpu = sum(P[q][1] for q in g) / T
        share_nmi = sum(nmi[q] for q in g) / tot_nmi if row['seed'] == 'nmi' else share_cpu
        if share_nmi < 0.05: continue
        dis, flags = decode_loop(rom, g) if rom else ([], set())
        loops.append({'start': '%06x' % g[0], 'pcs': len(g), 'nmi_share': round(100 * share_nmi, 1),
                      'cpu_share': round(100 * share_cpu, 1), 'code': ' | '.join(dis[:14]),
                      'flags': sorted(flags)})
        if len(loops) == 3: break
    row['loops'] = loops
    # classification
    top = loops[0] if loops else None
    if row['hash_diff']:
        row['class'] = 'MISMATCH'
    elif row['skip_pct'] >= 5:
        row['class'] = 'generic'
        if top and top['cpu_share'] >= 15: row['class'] = 'generic+specific'
    elif top and top['nmi_share'] >= 50 and top['cpu_share'] >= 15:
        row['class'] = 'specific'
    else:
        row['class'] = 'busy/none'
    return row

def main():
    rows = []
    for d in sorted(x for x in os.listdir(S) if re.match(r'g\d{3}$', x)):
        dd = os.path.join(S, d)
        if not os.path.exists(os.path.join(dd, 'done')): continue
        try: rows.append(one(dd))
        except Exception as e: rows.append({'dir': d, 'title': '?', 'class': 'analysis error', 'why': repr(e)})
    json.dump(rows, open(os.path.join(S, 'survey.json'), 'w'), indent=1)
    c = collections.Counter(r['class'] for r in rows)
    print(len(rows), 'games', dict(c))

if __name__ == '__main__':
    main()
