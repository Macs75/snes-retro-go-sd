#!/usr/bin/env python3
"""Save-state cold-resume test on the host: does a loaded state continue exactly
like the machine that saved it?

Run A replays a recording (game.rec next to game.sfc) to frame X + N, saving
the state at frame X (SNES_STATE_SAVE_AT). Run B is a fresh process that loads
that state at boot and replays the recording's input from frame X
(SNES_STATE_RESUME). Compared for the N frames after X:
  - the REC/hash machine hashes printed every 120 frames (wram vram cgram oam
    aram cpu; the audio hash runs from the replay's start, so it is left out);
  - for a Super FX cart, the per-frame GSU state + Game Pak RAM hash
    (GSU_STATELOG), aligned on the frame the state was saved at.

  state_resume_test.py ./snes_host ~/retro/roms/snes "Super Mario Kart (USA)" 2400 600
  state_resume_test.py ./snes_host scratch/yrec yoshi 15000 600 --gsu

Exit status 0 = identical, 1 = differs, 2 = a run failed.
"""
import argparse, os, re, subprocess, sys, tempfile

HASH = re.compile(r'REC/hash frame=(\d+) (wram=\w+ vram=\w+ cgram=\w+ oam=\w+ aram=\w+ cpu=\w+)')


def run(binary, cwd, rom, env_extra, log):
    env = dict(os.environ, SDL_VIDEODRIVER='dummy', SDL_AUDIODRIVER='dummy', SNES_REPLAY='1', **env_extra)
    with open(log, 'w') as f:
        r = subprocess.run([binary, rom + '.sfc'], cwd=cwd, env=env, stdout=f, stderr=subprocess.STDOUT, timeout=1800)
    return r.returncode, open(log, errors='replace').read()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('binary')
    ap.add_argument('romdir')
    ap.add_argument('rom', help='ROM name without .sfc (the .rec beside it)')
    ap.add_argument('frame', type=int, help='frame X the state is saved at')
    ap.add_argument('frames', type=int, help='frames N compared after it')
    ap.add_argument('--gsu', action='store_true', help='also compare the GSU state log')
    ap.add_argument('--keep', help='directory to keep the logs in')
    a = ap.parse_args()
    binary = os.path.abspath(os.path.expanduser(a.binary))
    romdir = os.path.expanduser(a.romdir)
    tmp = a.keep or tempfile.mkdtemp(prefix='state_resume_')
    os.makedirs(tmp, exist_ok=True)
    X, N = a.frame, a.frames

    ea = {'SNES_FRAMES': str(X + N), 'SNES_STATE_SAVE_AT': str(X)}
    eb = {'SNES_FRAMES': str(N), 'SNES_STATE_RESUME': str(X)}
    if a.gsu:
        ea['GSU_STATELOG'] = os.path.join(tmp, 'gsu_a.txt')
        eb['GSU_STATELOG'] = os.path.join(tmp, 'gsu_b.txt')
    ra, oa = run(binary, romdir, a.rom, ea, os.path.join(tmp, 'a.log'))
    if 'state saved at frame %d: ok' % X not in oa:
        print('run A did not save the state (exit %d), see %s' % (ra, tmp)); sys.exit(2)
    rb, ob = run(binary, romdir, a.rom, eb, os.path.join(tmp, 'b.log'))
    if 'state loaded: ok' not in ob or 'resumed the replay at frame %d' % X not in ob:
        print('run B did not load / resume (exit %d), see %s' % (rb, tmp)); sys.exit(2)

    ha = {int(f): h for f, h in HASH.findall(oa) if X < int(f) <= X + N}
    hb = {int(f): h for f, h in HASH.findall(ob) if X < int(f) <= X + N}
    common = sorted(set(ha) & set(hb))
    bad = [f for f in common if ha[f] != hb[f]]
    print('REC hashes: %d windows compared after frame %d, %d differ%s'
          % (len(common), X, len(bad), (' (first at frame %d)' % bad[0]) if bad else ''))
    status = 1 if bad or not common else 0

    if a.gsu:
        la = [l.split() for l in open(ea['GSU_STATELOG']) if not l.startswith('#')]
        lb = [l.split() for l in open(eb['GSU_STATELOG']) if not l.startswith('#')]
        # B's first frame lines up with A's frame X; take the best of a frame either side
        best = None
        for off in (X - 1, X, X + 1):
            pa = {int(r[0]) - off: (r[1], r[2]) for r in la}
            pairs = [(int(r[0]), pa.get(int(r[0])), (r[1], r[2])) for r in lb]
            pairs = [(k, x, y) for k, x, y in pairs if x is not None]
            # B's first line holds only the MainLoop exits from the load to the
            # first vblank: compare its Game Pak RAM hash, not its GSU hash
            pairs = [(k, (x[1],), (y[1],)) if k == 0 else (k, x, y) for k, x, y in pairs]
            nbad = sum(1 for k, x, y in pairs if x != y)
            if best is None or nbad < best[0]:
                best = (nbad, off, pairs)
        nbad, off, pairs = best
        first = next((k for k, x, y in pairs if x != y), None)
        print('GSU state + Game Pak RAM: %d frames compared (offset %d), %d differ%s'
              % (len(pairs), off - X, nbad, (' (first %d frames after the load)' % first) if first is not None else ''))
        if nbad or not pairs:
            status = 1
    print('logs in', tmp)
    sys.exit(status)


if __name__ == '__main__':
    main()
