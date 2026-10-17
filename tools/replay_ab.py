#!/usr/bin/env python3
"""Host A/B: do two snes_host builds replay recordings to the same machine state?

Each ROM (a .sfc with its .rec beside it) is replayed to the end of its
recording by both binaries, windowless and unthrottled, in a scratch working
directory (host_saves/ untouched). The REC/hash lines (wram vram cgram oam aram
cpu audio, every 120 frames) and REC/pic lines are compared.

  replay_ab.py base/snes_host ./snes_host ~/retro/roms/snes/"Super Mario Kart (USA).sfc" ...

Exit status 0 = identical everywhere, 1 = a difference, 2 = a run failed or
printed no hashes.
"""
import argparse, os, re, struct, subprocess, sys, tempfile

LINE = re.compile(r'^REC/(hash|pic) frame=\d+ .*$', re.M)


def frames_of(rec):
    with open(rec, 'rb') as f:
        magic, version, flags, rom_hash, frames = struct.unpack('<IHHII', f.read(16))
    return frames


def run(binary, rom, frames, workdir):
    env = dict(os.environ, SDL_VIDEODRIVER='dummy', SDL_AUDIODRIVER='dummy',
               SNES_REPLAY='1', SNES_FRAMES=str(frames))
    r = subprocess.run([binary, rom], cwd=workdir, env=env, stdout=subprocess.PIPE,
                       stderr=subprocess.DEVNULL, timeout=3600)
    out = r.stdout.decode(errors='replace')
    return r.returncode, LINE.findall(out) and [m.group(0) for m in LINE.finditer(out)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('roms', nargs='+')
    args = ap.parse_args()
    a, b = (os.path.abspath(os.path.expanduser(x)) for x in (args.a, args.b))
    status = 0
    for rom in args.roms:
        rom = os.path.abspath(os.path.expanduser(rom))
        frames = frames_of(os.path.splitext(rom)[0] + '.rec')
        res = []
        for binary in (a, b):
            with tempfile.TemporaryDirectory(prefix='replay_ab_') as wd:
                res.append(run(binary, rom, frames, wd))
        (ra, la), (rb, lb) = res
        name = os.path.basename(rom)
        if ra or rb or not la or not lb:
            print('%s: RUN FAILED (exit %d / %d, %d / %d lines)' % (name, ra, rb, len(la or []), len(lb or [])))
            status = 2
            continue
        diff = [i for i, (x, y) in enumerate(zip(la, lb)) if x != y]
        if len(la) != len(lb): diff.append(min(len(la), len(lb)))
        if diff:
            print('%s: %d frames, %d of %d lines DIFFER, first: %s' % (name, frames, len(diff), max(len(la), len(lb)),
                                                                     la[diff[0]] if diff[0] < len(la) else '(missing)'))
            status = max(status, 1)
        else:
            print('%s: %d frames, %d lines identical' % (name, frames, len(la)))
    sys.exit(status)


if __name__ == '__main__':
    main()
