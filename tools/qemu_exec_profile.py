#!/usr/bin/env python3
"""ARM instructions the device code path executes, per function, over a few
frames of a replay on gwemu (make qemu): QEMU logs every translated block
(in_asm) from the start and, from the REC/hash checkpoint of --start on, every
block it executes (exec, nochain); the run stops --frames later. Each executed
block counts its instructions, credited to the function holding its address
(the ELF's symbols). Counts, not cycles: QEMU has no timing.

  qemu_exec_profile.py <dir with ROM + .rec> <rom> --start 840 --frames 2 [--top 40] [--sa1]
  qemu_exec_profile.py --analyse <log> [--sa1]          (a log kept with --log)

--sa1: also the SA-1's engine (thumb2/sa1_engine.S): the SA-1 instructions it
dispatched (blocks starting at a handler of its table, sa1e_tab), and its ARM
instructions per SA-1 instruction by engine part (labels from an assembler
listing of the engine), plus the C bus helpers it calls.

--func NAME[,NAME]: also each function's disassembly, every instruction with
the times it ran over the profiled frames (from the blocks that cover it) --
which lines of a function are its cost, and how often its calls are made.

--start must be a frame with a REC/hash line (a multiple of 120). The log is
large (~100 MB a frame): it goes to --log (default: a temp file, deleted).
"""
import argparse, bisect, collections, json, os, re, socket, subprocess, sys, tempfile, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, 'snes_qemu.elf')
QEMU = os.environ.get('QEMU', os.path.expanduser('~/retro/gwemu/gwemu.AppImage'))


def symbols():
    out = subprocess.run(['docker', 'run', '--rm', '-v', ROOT + ':/w', '-w', '/w', 'sylverb/retro-go-sd-builder:v1.5',
                          'arm-none-eabi-nm', '-S', '-n', '--defined-only', 'snes_qemu.elf'],
                         capture_output=True, text=True, check=True).stdout
    syms = []
    for l in out.splitlines():
        f = l.split()
        if len(f) == 4 and f[2] in 'tTwW':
            syms.append((int(f[0], 16) & ~1, int(f[1], 16), f[3]))
        elif len(f) == 3 and f[1] in 'tTwW':           # no size (assembly)
            syms.append((int(f[0], 16) & ~1, 0, f[2]))
    syms.sort()
    return syms


def qmp(port, cmds):
    s = socket.create_connection(('127.0.0.1', port))
    f = s.makefile('rw')
    f.readline()
    for c in cmds:
        f.write(json.dumps(c) + '\n'); f.flush()
        while True:
            r = json.loads(f.readline())
            if 'return' in r or 'error' in r:
                break
    s.close()


def run(a, log):
    port = 4480
    p = subprocess.Popen([QEMU, '-M', 'mps2-an500', '-display', 'none', '-monitor', 'none', '-serial', 'none',
                          '-semihosting-config', 'enable=on,target=native,arg=snes,arg=%s,arg=%d' %
                          (a.rom, a.start + a.frames),
                          '-d', 'in_asm', '-D', log, '-qmp', 'tcp:127.0.0.1:%d,server,nowait' % port, '-kernel', ELF],
                         cwd=a.dir, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    for _ in range(100):
        try:
            socket.create_connection(('127.0.0.1', port)).close(); break
        except OSError:
            time.sleep(0.1)
    qmp(port, [{'execute': 'qmp_capabilities'}, {'execute': 'cont'}])
    mark = 'REC/hash frame=%d ' % a.start
    on = False
    for line in p.stdout:
        if not on and mark in line:
            qmp(port, [{'execute': 'qmp_capabilities'},
                       {'execute': 'human-monitor-command', 'arguments': {'command-line': 'log in_asm,exec,nochain'}}])
            on = True
    p.wait()
    if not on:
        sys.exit('no REC/hash line for frame %d' % a.start)


def thumb_count(hexs):
    """Thumb instructions in a block's bytes (this QEMU prints them raw,
    OBJD-T): a halfword 0b11101/0b11110/0b11111... opens a 32-bit one"""
    b = bytes.fromhex(hexs)
    n = i = 0
    while i + 1 < len(b):
        hw = b[i] | b[i + 1] << 8
        i += 4 if (hw >> 11) >= 0x1d else 2
        n += 1
    return n


def analyse(log, syms, top):
    size = {}                       # block start -> its instructions
    execs = collections.Counter()   # block start -> times run
    ex = re.compile(r'^Trace \d+: \S+ \[[0-9a-f]+/([0-9a-f]+)/')
    cur = None
    with open(log, errors='replace') as f:
        for l in f:
            m = ex.match(l)
            if m:
                execs[int(m.group(1), 16)] += 1
            elif l.startswith('0x') and cur is None:
                cur = int(l.split(':')[0], 16)
            elif l.startswith('OBJD-T:') and cur is not None:
                size[cur] = thumb_count(l.split(':', 1)[1].strip())
                cur = None
            elif l.startswith('IN:'):
                cur = None
    starts = [s[0] for s in syms]
    per = collections.Counter()
    for pc, k in execs.items():
        i = bisect.bisect_right(starts, pc) - 1
        # a symbol without a size (assembly) runs to the next one
        end = syms[i][0] + syms[i][1] if i >= 0 and syms[i][1] else (starts[i + 1] if i + 1 < len(starts) else 0)
        name = syms[i][2] if i >= 0 and pc < end else '?%08x' % pc
        per[name] += k * size.get(pc, 0)
    total = sum(per.values())
    print('# %d ARM instructions executed' % total)
    for name, k in per.most_common(top):
        print('%12d %5.1f%%  %s' % (k, 100.0 * k / total, name))


def docker(cmd):
    return subprocess.run(['docker', 'run', '--rm', '-u', '%d:%d' % (os.getuid(), os.getgid()), '-v', ROOT + ':/w',
                           '-w', '/w', 'sylverb/retro-go-sd-builder:v1.5', 'sh', '-c', cmd],
                          capture_output=True, text=True, check=True).stdout


def load(log):
    size, execs, cur = {}, collections.Counter(), None
    ex = re.compile(r'^Trace \d+: \S+ \[[0-9a-f]+/([0-9a-f]+)/')
    with open(log, errors='replace') as f:
        for l in f:
            m = ex.match(l)
            if m:
                execs[int(m.group(1), 16)] += 1
            elif l.startswith('0x') and cur is None:
                cur = int(l.split(':')[0], 16)
            elif l.startswith('OBJD-T:') and cur is not None:
                size[cur] = thumb_count(l.split(':', 1)[1].strip()); cur = None
            elif l.startswith('IN:'):
                cur = None
    return size, execs


def sa1_report(log, syms):
    addr = {n: a for a, sz, n in syms}
    base, stubs = addr['sa1_engine_run'], addr['sa1e_rd8_s']
    tab = int(docker("arm-none-eabi-nm snes_qemu.elf | grep ' sa1e_tab$' | cut -d' ' -f1"), 16)
    words = []
    for l in docker('arm-none-eabi-objdump -s --start-address=%d --stop-address=%d snes_qemu.elf' %
                    (tab, tab + 4096)).splitlines():
        m = re.match(r'^ ([0-9a-f]{5,8}) ((?:[0-9a-f]{8} ?){1,4})', l)
        if m:
            words += [int.from_bytes(bytes.fromhex(w), 'little') & ~1 for w in m.group(2).split()]
    handlers = set(words[:1024])
    fb = collections.Counter(words[:1024]).most_common(1)[0][0]
    lst = docker('arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -DSNES_SA1_T2 -Isrc/snes/thumb2 -Isrc/snes -c '
                 '-x assembler-with-cpp src/snes/thumb2/sa1_engine.S -o /dev/null -Wa,-aln=/dev/stdout')
    lab, at = '?', {}
    for l in lst.splitlines():
        lm = re.search(r'^\s*\d+\s+(?:[0-9a-f]{4}\s+)?(\.L\w+|sa1_engine_run):', l)
        if lm: lab = lm.group(1)
        mm = re.match(r'^\s*\d+\s+([0-9a-f]{4})\s+[0-9A-F]+', l)
        if mm: at.setdefault(int(mm.group(1), 16), lab)
    offs = sorted(at)
    size, execs = load(log)
    ins = sum(n for pc, n in execs.items() if pc in handlers)
    part = collections.Counter()
    for pc, n in execs.items():
        if not base <= pc < stubs + 0x400:
            continue
        if pc >= stubs:
            part['bus stubs'] += n * size.get(pc, 0); continue
        i = bisect.bisect_right(offs, pc - base) - 1
        L = at[offs[i]] if i >= 0 else '?'
        c = ('handlers' if L.startswith('.Lh') else 'handlers: bus slow paths' if L.startswith(('.Lm', '.Ld')) else
             'entry, reload' if L in ('sa1_engine_run', '.Lreload') else
             '.Lsync' if L.startswith('.Lsync') else 'exit' if L in ('.Lx0', '.Lx0_r1', '.Lexit_r1') else
             'REP/SEP/PLP (.Lgetp/.Lsetp)' if L in ('.Lgetp', '.Lsetp') else
             'branches taken' if L.startswith(('.Ltaken', '.Lnt')) else L)
        part[c] += n * size.get(pc, 0)
    starts = [a for a, sz, n in syms]
    for pc, n in execs.items():
        j = bisect.bisect_right(starts, pc) - 1
        if j >= 0 and re.match(r'(sa1e_(read|rd16|rd24_b0|wr8|wr16|wr16r)|sa1_cpu_(read|write)|sa1_reg_\w+|'
                               r'sa1cpu_\w+|sa1_arith|sa1_dma\w*)$', syms[j][2]) and pc < starts[j + 1]:
            part['C bus helpers, fallbacks'] += n * size.get(pc, 0)
    tot = sum(part.values())
    print('# SA-1: %d instructions dispatched (%d to C), %d ARM instructions: %.1f each' %
          (ins, execs[fb], tot, tot / max(1, ins)))
    for c, v in part.most_common():
        print('%10d %5.1f%%  %5.1f per SA-1 instruction  %s' % (v, 100.0 * v / tot, v / max(1, ins), c))
    sync = base + min(o for o in offs if at[o] == '.Lsync')
    print('# runs: engine entries %d, .Lsync %d' % (execs[base], execs[sync]))


def func_report(log, syms, names, frames):
    size, execs = load(log)
    addr = {n: (a, sz) for a, sz, n in syms}
    for name in names.split(','):
        if name not in addr:
            print('# %s: no such symbol' % name); continue
        a0, sz = addr[name]
        if not sz:                  # assembly: no size, it ends at the next symbol
            sz = min((a for a, _, _ in syms if a > a0), default=a0 + 0x10000) - a0
        lines, src, where = [], [], '?'
        for l in docker('arm-none-eabi-objdump -d -l --no-show-raw-insn --start-address=%d --stop-address=%d '
                        'snes_qemu.elf' % (a0, a0 + sz)).splitlines():
            m = re.match(r'^\s*([0-9a-f]+):\s+(.*)$', l)
            if m:
                lines.append((int(m.group(1), 16), m.group(2)))
                src.append(where)
            elif re.match(r'^/\S+:\d+', l):
                where = os.path.basename(l.split()[0])
        idx = {pc: i for i, (pc, _) in enumerate(lines)}
        ran = collections.Counter()
        for pc, k in execs.items():
            if pc in idx:
                for j in range(idx[pc], min(len(lines), idx[pc] + size.get(pc, 0))):
                    ran[j] += k
        tot = sum(ran.values())
        print('# %s: %d ARM instructions in %d frames (%.0f a frame); entered %d times' %
              (name, tot, frames, tot / max(1, frames), execs.get(a0, 0)))
        by = collections.Counter()
        for i in range(len(lines)):
            by[src[i]] += ran[i]
        print('#   by source line (a frame):')
        for w, k in by.most_common(25):
            if k:
                print('#   %8.0f  %s' % (k / max(1, frames), w))
        for i, (pc, txt) in enumerate(lines):
            print('%10s  %08x  %-40s %s' % (ran[i] if ran[i] else '', pc, txt, src[i]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir', nargs='?'); ap.add_argument('rom', nargs='?')
    ap.add_argument('--analyse', metavar='LOG')
    ap.add_argument('--sa1', action='store_true')
    ap.add_argument('--start', type=int)
    ap.add_argument('--frames', type=int, default=2)
    ap.add_argument('--top', type=int, default=40)
    ap.add_argument('--log')
    ap.add_argument('--func')
    a = ap.parse_args()
    syms = symbols()
    if a.analyse:
        analyse(a.analyse, syms, a.top)
        if a.sa1: sa1_report(a.analyse, syms)
        if a.func: func_report(a.analyse, syms, a.func, a.frames)
        return
    log = a.log or tempfile.mktemp(suffix='.qlog')
    try:
        run(a, log)
        analyse(log, syms, a.top)
        if a.sa1: sa1_report(log, syms)
        if a.func: func_report(log, syms, a.func, a.frames)
    finally:
        if not a.log and os.path.exists(log):
            os.remove(log)


if __name__ == '__main__':
    main()
