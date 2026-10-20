#!/usr/bin/env python3
"""Two host builds over the regression recordings, in parallel: is the machine
state the same (the REC/hash checkpoints, and for an SA-1 cart SA1_STATELOG,
both CPUs every frame), and how much did each run (FRAME_STATS: 65816 / SA-1
instructions per frame, per window of --window frames). Exit status 1 if any
state differs or a run failed.

  host_regress.py --old <snes_host> --new <snes_host> [-j 4] [--window 120] [ROM ...]

Each ROM replays its recording (same name, .rec) unthrottled and windowless,
once per build: hashes, counts and state log come from the same run. Results
are cached by the binary's and the ROM's and recording's contents
(~/.cache/snes_host_regress), so a baseline runs once, not every round.
Without ROMs: the set in tools/host_regress.set (one path per line, ~ allowed).
"""
import argparse, concurrent.futures, hashlib, os, struct, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE = os.path.expanduser('~/.cache/snes_host_regress')


def digest(*paths):
    h = hashlib.sha1()
    for p in paths:
        with open(p, 'rb') as f:
            for b in iter(lambda: f.read(1 << 20), b''):
                h.update(b)
    return h.hexdigest()[:16]


def frames_of(rec):
    return struct.unpack_from('<I', open(rec, 'rb').read(), 12)[0]


def run(binary, bkey, rom):
    """the replay's results, from the cache or a run: (dir, error or None)"""
    rec = os.path.splitext(rom)[0] + '.rec'
    d = os.path.join(CACHE, bkey, digest(rom, rec))
    if os.path.exists(os.path.join(d, 'ok')):
        return d, None
    os.makedirs(d, exist_ok=True)
    n = frames_of(rec)
    env = dict(os.environ, SDL_VIDEODRIVER='dummy', SDL_AUDIODRIVER='dummy', SNES_REPLAY='1',
               SNES_FRAMES=str(n), FRAME_STATS=os.path.join(d, 'fs.txt'), SA1_STATELOG=os.path.join(d, 'sl.txt'))
    for f in ('fs.txt', 'sl.txt'):
        if os.path.exists(os.path.join(d, f)):
            os.remove(os.path.join(d, f))
    try:
        p = subprocess.run(['timeout', '1800', binary, os.path.basename(rom)], cwd=os.path.dirname(rom), env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, errors='replace')
    except OSError as e:
        return d, str(e)
    hashes = sorted({l[l.index('REC/hash'):] for l in p.stdout.splitlines() if 'REC/hash' in l},
                    key=lambda l: int(l.split('frame=')[1].split()[0]))
    open(os.path.join(d, 'hash.txt'), 'w').write('\n'.join(hashes) + '\n')
    fs = os.path.join(d, 'fs.txt')
    got = sum(1 for _ in open(fs)) if os.path.exists(fs) else 0
    if p.returncode != 0 or got < n:
        return d, 'exit %d, %d of %d frames' % (p.returncode, got, n)
    open(os.path.join(d, 'ok'), 'w').write('')
    return d, None


def lines(path):
    return open(path).read().splitlines() if os.path.exists(path) else None


def stats(d):
    return [list(map(int, l.split())) for l in lines(os.path.join(d, 'fs.txt')) or []]


def report(rom, a, b, W):
    """the comparison of one ROM's two runs; True if the state is the same"""
    name = os.path.basename(rom)
    ha, hb = lines(os.path.join(a, 'hash.txt')), lines(os.path.join(b, 'hash.txt'))
    first = next((x.split()[1] for x, y in zip(ha, hb) if x != y), None)
    same = first is None and len(ha) == len(hb) and len(ha) > 0
    out = ['== %s: %d hashes %s' % (name, len(ha), 'SAME' if same else 'DIFFERENT from %s' % (first or 'the count'))]
    sa, sb = lines(os.path.join(a, 'sl.txt')), lines(os.path.join(b, 'sl.txt'))
    if sa is not None or sb is not None:
        sa, sb = sa or [], sb or []
        f = next((i + 1 for i, (x, y) in enumerate(zip(sa, sb)) if x != y), None)
        ok = f is None and len(sa) == len(sb)
        same = same and ok
        out.append('   state every frame: %s' % ('identical (%d frames)' % len(sa) if ok else
                                                  'FIRST DIFFERENCE at frame %s' % (f or min(len(sa), len(sb)) + 1)))
    x, y = stats(a), stats(b)
    n = min(len(x), len(y))
    c0, c1 = sum(r[1] for r in x[:n]), sum(r[1] for r in y[:n])
    s0, s1 = sum(r[5] for r in x[:n]), sum(r[5] for r in y[:n])
    out.append('   65816: %d -> %d (%+.1f%%); SA-1: %d -> %d' % (c0, c1, 100.0 * (c1 - c0) / max(1, c0), s0, s1))
    wins = []
    for s in range(0, n, W):
        k = min(W, n - s)
        p, q = sum(r[1] for r in x[s:s + k]) / k, sum(r[1] for r in y[s:s + k]) / k
        wins.append((s // W + 1, p, q))
    heavy = sorted((w for w in wins if w[2] > w[1]), key=lambda w: w[1] - w[2])
    light = sorted((w for w in wins if w[2] < w[1]), key=lambda w: (w[2] - w[1]) / w[1])
    out.append('   windows of %d frames: %d, heavier %d, lighter %d' % (W, len(wins), len(heavy), len(light)))
    for tag, ws in (('heavier', heavy[:3]), ('lighter', light[:3])):
        for w, p, q in ws:
            out.append('     %s window %3d: %7.0f -> %7.0f a frame (%+.1f%%)' % (tag, w, p, q, 100.0 * (q - p) / p))
    return same, '\n'.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--old', required=True)
    ap.add_argument('--new', required=True)
    ap.add_argument('-j', type=int, default=4)
    ap.add_argument('--window', type=int, default=120)
    ap.add_argument('roms', nargs='*')
    a = ap.parse_args()
    roms = a.roms or [os.path.expanduser(l.strip()) for l in open(os.path.join(ROOT, 'tools', 'host_regress.set'))
                      if l.strip() and not l.startswith('#')]
    roms = [os.path.abspath(r) for r in roms]
    bins = {'old': os.path.abspath(a.old), 'new': os.path.abspath(a.new)}
    keys = {k: digest(v) for k, v in bins.items()}
    jobs = {}
    with concurrent.futures.ThreadPoolExecutor(a.j) as ex:
        # the longest recordings first, so they do not finish last
        for rom in sorted(roms, key=lambda r: -frames_of(os.path.splitext(r)[0] + '.rec')):
            for k in ('old', 'new'):
                jobs[rom, k] = ex.submit(run, bins[k], keys[k], rom)
    bad = False
    for rom in roms:
        (da, ea), (db, eb) = jobs[rom, 'old'].result(), jobs[rom, 'new'].result()
        if ea or eb:
            print('== %s: RUN FAILED (old: %s, new: %s)' % (os.path.basename(rom), ea or 'ok', eb or 'ok'))
            bad = True
            continue
        same, text = report(rom, da, db, a.window)
        print(text)
        bad |= not same
    print('RESULT: %s' % ('state DIFFERS or a run failed' if bad else 'state the same in every recording'))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
