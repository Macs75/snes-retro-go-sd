#!/usr/bin/env python3
"""Compare device PERF logs of the same recording, window by window and by phase.

Each PERF log (make PERF=1, a replay of a .rec) has one block of PROF/ lines per
120 emulated frames, tagged by the REC/hash line that precedes it. This lines
the windows of two or more logs up by frame and prints, as % of the frame
budget: busy, the heaviest single frame, the cpu/ppu/apu/dma/gsu split, late
frames and the audio step; per phase (frame ranges given with --phase) and for
chosen windows. It also checks the REC hashes agree. A PROFLITE log (late
frames and wall time only) is lined up the same way; speed% is emulated time
over wall time.

  prof_log_compare.py old.log new.log [more.log ...] \\
      --phase intro:0:12720 --phase title:12960:13560 --phase gameplay:13680:99999 \\
      --frames 240,9360,12960,13920,16080,17520,18240
"""
import argparse, re

PATS = [
    (r'busy=(\d+) heaviest=(\d+)', ('busy', 'heavy')),
    (r'budget=(\d+)', ('budget',)),
    (r'PROF/emu: total=(\d+) \[cpu=(\d+) ppu=(\d+) apu=(\d+) dma=(\d+) (?:gsu|sa1)=(\d+)\]',
     ('tot', 'cpu', 'ppu', 'apu', 'dma', 'gsu')),
    (r'PROF/pace: late=(\d+)', ('late',)),
    (r'dropped=(\d+) ticks past the catch-up cap, fill_min=(\d+)', ('drop', 'fmin')),
    (r'floor_pulls=(\d+) ceil_pulls=(\d+) asked_avg=(\d+) base avg=(\d+) min=(\d+) max=(\d+)',
     ('flp', 'clp', 'ask', 'bavg', 'bmin', 'bmax')),
    (r'PROF/audio3: frames by backlog at the wait <64=(\d+) <256=(\d+) <512=(\d+) <640=(\d+) >=640=(\d+)',
     ('h0', 'h1', 'h2', 'h3', 'h4')),
    (r'PROF/frame: emulated=(\d+) shown=\d+ wall_ms=(\d+)', ('nfr', 'wall')),
    (r'PROFLITE: frames=(\d+) shown=\d+ late=(\d+) of \d+ wall_ms=(\d+)', ('nfr', 'late', 'wall')),
    (r'PROFLITE: audio SLOWED step_avg=(\d+) step_min=(\d+)', ('savg', 'smin')),
    (r'step_avg=(\d+) step_min=(\d+).*underruns=(\d+)', ('savg', 'smin', 'under')),
    (r'PROF/cpub: batch=(\d+) in (\d+) calls, single=(\d+) in (\d+) opcodes, events=(\d+), bake=(\d+), '
     r'idle=(\d+), rest=(\d+)', ('cb', 'cbn', 'cs', 'csn', 'ce', 'ck', 'ci', 'cr')),
]


def parse(path):
    rows, cur, hashes = {}, None, {}
    for line in open(path, errors='replace'):
        m = re.search(r'REC/hash frame=(\d+) (.*)', line)
        if m:
            cur = {'f': int(m.group(1))}
            rows[cur['f']] = cur
            hashes[cur['f']] = re.sub(r'\s+$', '', m.group(2))
            continue
        if cur is None:
            continue
        for pat, keys in PATS:
            m = re.search(pat, line)
            if m:
                cur.update(zip(keys, map(int, m.groups())))
    return {f: r for f, r in rows.items() if 'late' in r}, hashes


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--phase', action='append', default=[], help='name:first:last (window end frames)')
    ap.add_argument('--frames', default='', help='comma-separated windows to list')
    a = ap.parse_args()
    logs = [parse(p) for p in a.logs]
    names = ['L%d' % i for i in range(len(logs))]
    for n, p in zip(names, a.logs):
        print('%s = %s' % (n, p))
    common = sorted(set.intersection(*(set(r) for r, _ in logs)))
    base = logs[0][0][common[0]].get('budget', 5890300) if common else 5890300
    B = base / 100.0
    ref = logs[0][1]
    for n, (_, h) in zip(names[1:], logs[1:]):
        same = sum(1 for f in ref if f in h and h[f] == ref[f])
        print('REC hashes %s vs L0: %d of %d windows equal' % (n, same, len(ref)))

    def agg(rows, frames, key):
        return sum(rows[f].get(key, 0) for f in frames)

    if a.phase:
        print('\nphase (n windows)   ' + '   '.join('%-40s' % n for n in names))
        print(' ' * 20 + '   '.join('%-40s' % 'busy  gsu   cpu   heaviest  late speed%' for _ in names))
        for spec in a.phase:
            name, lo, hi = spec.split(':')
            fr = [f for f in common if int(lo) <= f <= int(hi)]
            if not fr:
                continue
            cells = []
            for rows, _ in logs:
                k = len(fr)
                # speed = emulated time / wall time (60.0988 Hz NTSC); PROFLITE logs have no busy split
                spd = agg(rows, fr, 'nfr') / 60.0988 * 1000 / max(1, agg(rows, fr, 'wall')) * 100
                if 'busy' in rows[fr[0]]:
                    cells.append('%5.1f %5.1f %5.1f %7.1f %6d %5.1f' % (
                        agg(rows, fr, 'busy') / k / B, agg(rows, fr, 'gsu') / k / B, agg(rows, fr, 'cpu') / k / B,
                        max(rows[f]['heavy'] for f in fr) / B, agg(rows, fr, 'late'), spd))
                else:
                    cells.append('%5s %5s %5s %7s %6d %5.1f' % ('-', '-', '-', '-', agg(rows, fr, 'late'), spd))
            print('%-10s (%3d)     ' % (name, len(fr)) + '   '.join('%-40s' % c for c in cells))
        if any('cb' in r for rows, _ in logs for r in rows.values()):
            # PERF_CPUB: the 65816 figure split, K cycles per emulated frame
            print('\nPROF/cpub, K cycles/frame  ' + '   '.join('%-36s' % n for n in names))
            print(' ' * 26 + '   '.join('%-36s' % 'batch  calls events  idle  rest  bake' for _ in names))
            for spec in a.phase:
                name, lo, hi = spec.split(':')
                fr = [f for f in common if int(lo) <= f <= int(hi)]
                if not fr:
                    continue
                k = len(fr) * 1000.0
                cells = ['%6.0f %5.0f %6.0f %5.0f %5.0f %5.0f' % tuple(
                    agg(rows, fr, x) / k * (1000 if x == 'cbn' else 1)
                    for x in ('cb', 'cbn', 'ce', 'ci', 'cr', 'ck')) if 'cb' in rows[fr[0]] else '-'
                    for rows, _ in logs]
                print('%-10s (%3d)           ' % (name, len(fr)) + '   '.join('%-36s' % c for c in cells))
        # pacing and audio: step 1.000 = normal pitch (a PROFLITE window without
        # its SLOWED line played at 1.0); dropped/fill_min only from builds that print them
        print('\npacing, audio            ' + '   '.join('%-46s' % n for n in names))
        print(' ' * 25 + '   '.join('%-46s' % 'late dropped under step_avg step_min fill_min' for _ in names))
        for spec in a.phase:
            name, lo, hi = spec.split(':')
            fr = [f for f in common if int(lo) <= f <= int(hi)]
            if not fr:
                continue
            cells = []
            for rows, _ in logs:
                r0 = rows[fr[0]]
                cells.append('%4d %7s %5s %8.3f %8.3f %8s' % (
                    agg(rows, fr, 'late'),
                    agg(rows, fr, 'drop') if 'drop' in r0 else '-',
                    agg(rows, fr, 'under') if 'under' in r0 else '-',
                    sum(rows[f].get('savg', 65536) for f in fr) / len(fr) / 65536.0,
                    min(rows[f].get('smin', 65536) for f in fr) / 65536.0,
                    min(rows[f]['fmin'] for f in fr) if 'fmin' in r0 else '-'))
            print('%-10s (%3d)           ' % (name, len(fr)) + '   '.join('%-46s' % c for c in cells))
        if any('h0' in r for rows, _ in logs for r in rows.values()):
            # PERF_AUDIO: % of frames by stretcher backlog at the frame wait (TARGET 640),
            # pulls clamped at the rate floor / ceiling, and the rate estimate's range
            print('\ncushion (PERF_AUDIO)     ' + '   '.join('%-50s' % n for n in names))
            print(' ' * 25 + '   '.join('%-50s' % '<64 <256 <512 <640 >=640%  floor ceil base_min max' for _ in names))
            for spec in a.phase:
                name, lo, hi = spec.split(':')
                fr = [f for f in common if int(lo) <= f <= int(hi)]
                if not fr:
                    continue
                cells = []
                for rows, _ in logs:
                    if 'h0' not in rows[fr[0]]:
                        cells.append('-')
                        continue
                    h = [agg(rows, fr, 'h%d' % i) for i in range(5)]
                    n = max(1, sum(h))
                    cells.append('%3.0f %4.0f %4.0f %4.0f %5.0f  %5d %4d %.3f %.3f' % (
                        tuple(100.0 * x / n for x in h) + (agg(rows, fr, 'flp'), agg(rows, fr, 'clp'),
                        min(rows[f]['bmin'] for f in fr) / 65536.0, max(rows[f]['bmax'] for f in fr) / 65536.0)))
                print('%-10s (%3d)           ' % (name, len(fr)) + '   '.join('%-50s' % c for c in cells))
    if a.frames:
        print('\nframe  ' + '   '.join('%-40s' % (n + ': busy  gsu  heaviest late step_min') for n in names))
        for f in (int(x) for x in a.frames.split(',')):
            if f not in common:
                continue
            cells = []
            for rows, _ in logs:
                r = rows[f]
                cells.append('%5.1f %5.1f %7.1f %4d %6d' % (r.get('busy', 0) / B, r.get('gsu', 0) / B,
                                                            r.get('heavy', 0) / B, r.get('late', 0), r.get('smin', 0)))
            print('%5d  ' % f + '   '.join('%-40s' % c for c in cells))


if __name__ == '__main__':
    main()
