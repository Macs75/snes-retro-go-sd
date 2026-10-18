#!/usr/bin/env python3
"""Device code path vs host: do gwemu and the host replay recordings to the same state?

The host build runs the C interpreters only; the Thumb-2 65816/SPC700 engines
and their bus bridges run on gwemu's mps2-an500 (make qemu, host/qemu/run.sh).
Each ROM (a .sfc with its .rec beside it) is replayed to the end of its
recording by both, and the REC/hash and REC/pic lines are compared. A change to
the engines or bridges is checked here; a host-only A/B cannot see it.

  qemu_vs_host.py ./snes_host ~/retro/roms/snes/"Castlevania - Vampire's Kiss (E).sfc" ...

The ROM's file name must have no spaces or quotes (the semihosting argument
breaks on them and gwemu spins): link the ROM and copy its .rec under a short
name. Run `make host` and `make qemu` (Docker toolchain) first. Exit status 0 =
identical everywhere, 1 = a difference, 2 = a run failed or printed no hashes.
"""
import argparse, os, re, struct, subprocess, sys, tempfile

LINE = re.compile(r'^REC/(hash|pic) frame=\d+ .*$', re.M)
RUN_SH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'host', 'qemu', 'run.sh')


def frames_of(rec):
    with open(rec, 'rb') as f:
        return struct.unpack('<IHHII', f.read(16))[4]


def lines(out):
    return [m.group(0).rstrip('\r') for m in LINE.finditer(out)]


def run_host(binary, rom, frames):
    env = dict(os.environ, SDL_VIDEODRIVER='dummy', SDL_AUDIODRIVER='dummy',
               SNES_REPLAY='1', SNES_FRAMES=str(frames))
    with tempfile.TemporaryDirectory(prefix='qvh_') as wd:
        r = subprocess.run([binary, rom], cwd=wd, env=env, stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, timeout=3600)
    return r.returncode, lines(r.stdout.decode(errors='replace'))


def run_qemu(rom, frames, port):
    r = subprocess.run(['bash', RUN_SH, os.path.dirname(rom), os.path.basename(rom), str(frames), str(port)],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=7200)
    return r.returncode, lines(r.stdout.decode(errors='replace'))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('host')
    ap.add_argument('roms', nargs='+')
    ap.add_argument('--port', type=int, default=4477)
    a = ap.parse_args()
    host = os.path.abspath(os.path.expanduser(a.host))
    status = 0
    for rom in a.roms:
        rom = os.path.abspath(os.path.expanduser(rom))
        frames = frames_of(os.path.splitext(rom)[0] + '.rec')
        (rh, lh), (rq, lq) = run_host(host, rom, frames), run_qemu(rom, frames, a.port)
        name = os.path.basename(rom)
        if rh or rq or not lh or not lq:
            print('%s: RUN FAILED (host exit %d, %d lines; qemu exit %d, %d lines)' % (name, rh, len(lh), rq, len(lq)))
            status = 2
            continue
        diff = [i for i, (x, y) in enumerate(zip(lh, lq)) if x != y]
        if len(lh) != len(lq):
            diff.append(min(len(lh), len(lq)))
        if diff:
            i = diff[0]
            print('%s: %d frames, %d of %d lines DIFFER, first: host %s / qemu %s' % (
                name, frames, len(diff), max(len(lh), len(lq)),
                lh[i] if i < len(lh) else '(missing)', lq[i] if i < len(lq) else '(missing)'))
            status = max(status, 1)
        else:
            print('%s: %d frames, %d lines identical' % (name, frames, len(lh)))
    return status


if __name__ == '__main__':
    sys.exit(main())
