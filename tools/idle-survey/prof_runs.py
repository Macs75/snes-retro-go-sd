import re,sys,statistics as st
B=5890300
def runs(path):
    rs=[[]]; cur={}; lastf=0
    for l in open(path):
        if l.startswith('PROF/frame'): cur={'shown':int(re.search(r'shown=(\d+)',l).group(1)),'mode':re.search(r'half-rate (\w+)',l).group(1)}
        if l.startswith('PROF/budget'): cur['busy']=int(re.search(r'busy=(\d+)',l).group(1)); cur['heavy']=int(re.search(r'heaviest=(\d+)',l).group(1))
        if l.startswith('PROF/emu'):
            for k in ['cpu','ppu','apu','dma']: cur[k]=int(re.search(k+r'=(\d+)',l).group(1))
        if l.startswith('PROF/pace'): cur['late']=int(re.search(r'late=(\d+)',l).group(1))
        if l.startswith('PROF/audio'): cur['stepmin']=int(re.search(r'step_min=(\d+)',l).group(1))
        if l.startswith('PROF/idle: site'): cur['skip']=int(re.search(r'skipped_dots=(\d+)',l).group(1))
        if l.startswith('REC/hash'):
            f=int(re.search(r'frame=(\d+)',l).group(1))
            if f<lastf: rs.append([])
            lastf=f; cur['f']=f; cur['hash']=l.strip(); rs[-1].append(dict(cur))
    return [r for r in rs if r]
for path in sys.argv[1:]:
    for i,r in enumerate(runs(path)):
        sel=[x for x in r if x['f']>=4200]
        if not sel: continue
        m=lambda k: st.mean(x.get(k,0) for x in sel)
        print(f"{path.split('/')[-1]} run{i+1} ({sel[0]['mode']}): busy={m('busy')/1e6:.2f}M ({100*m('busy')/B:.1f}%) max {100*max(x['busy'] for x in sel)/B:.1f}%  heaviest avg {100*m('heavy')/B:.1f}% max {100*max(x['heavy'] for x in sel)/B:.1f}%  cpu={m('cpu')/1e3:.0f}K ppu={m('ppu')/1e3:.0f}K  late={sum(x['late'] for x in sel)}/{120*len(sel)} stepmin={min(x['stepmin'] for x in sel)} skip={m('skip')/1e3:.0f}K")
