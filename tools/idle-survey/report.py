#!/usr/bin/env python3
"""Turn survey.json into the overnight report (markdown) and the draft list
for the external file (CSV), with one category per game:

  A  generic          the current generic skip already takes the wait loop
  B  generic, fixable the loop is idle and exact to skip, but the current
                      generic skip misses it for a reason that has a generic
                      fix (no table): code in WRAM, $4210 poll, SEP/REP in the
                      lap, two alternating sites, DP/DB changed after decode,
                      countdown (DEC/BNE)
  C  traced proof     the lap writes memory or calls a subroutine but returns
                      the machine to the same state (rewrites the same values):
                      idle, provable by tracing one real lap
  D  table            the lap changes state every time (an accumulator such as
                      a random-seed stir, a $4212 H/V poll): skipping it needs
                      arithmetic for that loop -> external file entry
  E  none             no dominant wait loop in the scene surveyed
  X  unsupported / mismatch
"""
import json, os, re, sys, collections
S = os.environ.get("SURVEY_DIR") or os.path.dirname(os.path.abspath(__file__))  # dir holding survey.json
rows = json.load(open(os.path.join(S, 'survey.json')))

def loop_kind(l):
    f = set(l['flags']); code = l['code']
    mns = re.findall(r':([A-Z]{3})', code)
    if not mns: return 'B', 'code in WRAM (bytes not in ROM)', 'B-ram'
    io = [x for x in f if 'I/O' in x and 'writes I/O' not in x]
    if any('4212' in x for x in io): return 'D', 'polls $4212 (H/V blank): value moves with the dot', 'D-hvpoll'
    if any('4210' in x for x in io) and not (f & {'writes', 'stack/call'}): return 'B', 'polls $4210 (NMI flag): changes only at vblank', 'B-4210'
    if any(re.search(r'214[0-3]', x) for x in f): return 'E', 'APU port handshake ($2140-3): the sound CPU moves it', 'E-apu'
    # an accumulator stores the sum back to an operand it loaded: LDA x / ADC y / STA x
    ins = re.findall(r':([A-Z]{3}) ?([0-9a-f]*)', code)
    lda = {o for m, o in ins if m == 'LDA' and o}
    sta = {o for m, o in ins if m == 'STA' and o}
    if ('ADC' in mns or 'SBC' in mns) and (lda & sta): return 'D', 'accumulates (LDA x / ADC / STA x every lap): needs arithmetic', 'D-accum'
    if set(mns) <= {'DEC', 'DEX', 'DEY', 'BNE', 'NOP'} and 'writes' not in f: return 'B', 'countdown (DEC/BNE): subtract the laps', 'B-count'
    if f == {'indexed'} or (f <= {'indexed', 'changes M/X/DB/DP/I'} and 'indexed' in f):
        return 'B', 'read-only poll with an index register (constant across laps)', 'B-indexed'
    if f & {'writes', 'stack/call', 'indirect', 'indexed'} or any('writes I/O' in x for x in f):
        if any('writes I/O' in x for x in f): return 'C', 'writes I/O in the lap: check side effects first', 'C-io'
        return 'C', 'writes / calls / indexed, same state each lap: traced proof', 'C-trace'
    if 'changes M/X/DB/DP/I' in f: return 'B', 'SEP/REP/TCD inside the lap: decode the width change', 'B-sep'
    if 'WAI' in f: return 'A', 'WAI', 'A'
    return 'B', 'read-only poll the current decoder misses (multi-site / DP or DB changed / long)', 'B-misc'

out = []
for r in rows:
    if r.get('class') in ('unsupported',) : r['cat'] = 'X'; r['reason'] = r.get('why', ''); out.append(r); continue
    if r.get('class') == 'MISMATCH': r['cat'] = 'X'; r['reason'] = 'HASH MISMATCH'; out.append(r); continue
    loops = r.get('loops', [])
    site = r.get('site', '')
    sitepc = int(site.split(':')[0] + site.split(':')[1].split('+')[0], 16) if site else -1
    top = loops[0] if loops else None
    skip = r.get('skip_pct', 0)
    def is_site(l):
        s = int(l['start'], 16)
        return sitepc >= 0 and abs(s - sitepc) <= 8
    # the biggest loop the generic skip does NOT take
    rest = [l for l in loops if not is_site(l)]
    big = max(rest, key=lambda l: l['nmi_share'] * l['cpu_share'], default=None)
    if top and is_site(top) and top['nmi_share'] >= 50 and top['cpu_share'] >= 15 and skip < 5:
        r['cat'] = 'B'; r['sub'] = 'B-env'; r['loop'] = top
        r['reason'] = 'armed but not taken: DP/DB/flags changed after decode (re-decode the site)'
    elif skip >= 5 and (big is None or big['nmi_share'] < 30):
        r['cat'] = 'A'; r['reason'] = 'generic skip takes %.0f%% of the frame' % skip
        if top and is_site(top) and top['cpu_share'] >= 15:
            r['reason'] += ' (%.0f%% of the CPU left in it: spans it cannot take, e.g. HDMA)' % top['cpu_share']
    elif big and big['nmi_share'] >= 30 and big['cpu_share'] >= 8:
        c, why, sub = loop_kind(big)
        if c == 'B' and sub == 'B-misc' and skip > 0:
            # same clean shape as the armed site: two alternating wait loops
            why = 'second wait loop (two sites alternate): arm several sites'
            sub = 'B-multi'
        r['cat'] = c; r['reason'] = why; r['sub'] = sub; r['loop'] = big
        if skip >= 5: r['reason'] += ' [generic already takes %.0f%%]' % skip
    elif skip >= 5:
        r['cat'] = 'A'; r['reason'] = 'generic skip takes %.0f%% of the frame' % skip
    else:
        r['cat'] = 'E'; r['reason'] = ('no dominant wait loop in the scene (NMI lands in game code)' if r.get('seed') == 'nmi'
                                        else 'no NMI in the scene and no dominant loop in the CPU profile')
    out.append(r)

cnt = collections.Counter(r['cat'] for r in out)
sub = collections.Counter(r.get('sub', r['cat']) for r in out)
json.dump(out, open(os.path.join(S, 'report.json'), 'w'), indent=1)
print(len(out), dict(cnt))
print(dict(sub))
