#!/usr/bin/env python3
"""The pause menu's Controls profiles and "L/R on Start/Select" against their
tables: compiles main_snes.c's read_snes_pad as it is (cut from the source,
with stubs for the firmware types) and presses every button and chord of the
tables in every profile, with the switch off and on. Exit status 1 on any
difference.

  pad_map_test.py
"""
import os, re, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIT = {'B': 0, 'Y': 1, 'Select': 2, 'Start': 3, 'A': 8, 'X': 9, 'L': 10, 'R': 11}
# device buttons: A, B, GAME, TIME, and the Zelda unit's START / SELECT
PRESSES = ['A', 'B', 'GAME', 'TIME', 'START', 'SELECT', 'GAME+A', 'GAME+B', 'GAME+TIME', 'TIME+A', 'TIME+B']
ZELDA = {'A': 'A', 'B': 'B', 'START': 'X', 'SELECT': 'Y', 'TIME': 'Start', 'GAME': '',
         'GAME+A': 'R', 'GAME+B': 'L', 'GAME+TIME': 'Select', 'TIME+A': 'Start A', 'TIME+B': 'Start B'}
ZELDA_B = dict(ZELDA, **{'A': 'B', 'B': 'Y', 'START': 'A', 'SELECT': 'X', 'TIME+A': 'Start B', 'TIME+B': 'Start Y'})
MARIO = {'A': 'A', 'B': 'B', 'START': '', 'SELECT': '', 'TIME': 'Select', 'GAME': 'Start',
         'GAME+A': 'R', 'GAME+B': 'L', 'GAME+TIME': 'Start Select', 'TIME+A': 'X', 'TIME+B': 'Y'}
MARIO_B = dict(MARIO, **{'A': 'B', 'B': 'Y', 'TIME+A': 'A', 'TIME+B': 'X'})
SIMPLE = {'A': 'A', 'B': 'B', 'START': 'X', 'SELECT': 'Y', 'TIME': 'Start', 'GAME': 'Select',
          'GAME+A': 'Select A', 'GAME+B': 'Select B', 'GAME+TIME': 'Select Start', 'TIME+A': 'Start A',
          'TIME+B': 'Start B'}
# profile number (snes_pad_profile_t) -> (table with the switch off, table with it on)
EXPECT = {
    1: (ZELDA, dict(ZELDA, START='R', SELECT='L', **{'GAME+A': 'X', 'GAME+B': 'Y'})),
    4: (ZELDA_B, dict(ZELDA_B, START='R', SELECT='L', **{'GAME+A': 'A', 'GAME+B': 'X'})),
    2: (MARIO, MARIO), 5: (MARIO_B, MARIO_B), 3: (SIMPLE, SIMPLE),
}
NAMES = {1: 'Zelda', 2: 'Mario', 3: 'Simple', 4: 'Zelda B', 5: 'Mario B'}

STUB = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
enum { ODROID_INPUT_UP = 0, ODROID_INPUT_RIGHT, ODROID_INPUT_DOWN, ODROID_INPUT_LEFT, ODROID_INPUT_SELECT,
       ODROID_INPUT_START, ODROID_INPUT_A, ODROID_INPUT_B, ODROID_INPUT_MENU, ODROID_INPUT_VOLUME,
       ODROID_INPUT_POWER, ODROID_INPUT_X, ODROID_INPUT_Y, ODROID_INPUT_MAX };
typedef struct { uint8_t values[ODROID_INPUT_MAX]; } odroid_gamepad_state_t;
static bool get_ofw_is_mario(void) { return false; }
%s
int main(int argc, char **argv) {
  snes_pad_profile = atoi(argv[1]);
  snes_pad_lr_swap = atoi(argv[2]) != 0;
  odroid_gamepad_state_t j = {{0}};
  for (int i = 3; i < argc; i++) j.values[atoi(argv[i])] = 1;
  printf("%%u\n", (unsigned)read_snes_pad(&j));
  return 0;
}
'''
DEV = {'A': 6, 'B': 7, 'GAME': 5, 'TIME': 4, 'START': 11, 'SELECT': 12}


def main():
    src = open(os.path.join(ROOT, 'src/main_snes.c')).read()
    i = src.index('typedef enum {\n  SNES_PAD_AUTO')
    j = src.index('static uint16_t read_snes_pad(')
    k = src.index('\n}\n', j) + 3
    body = src[i:src.index('\n', src.index('snes_pad_lr_swap;')) + 1]   # the enum and the two settings
    body += src[src.index('static snes_pad_profile_t snes_pad_profile_effective'):k]
    body = re.sub(r'#if defined\(HOST_BUILD\).*?#endif\n', '', body, flags=re.S)
    with tempfile.TemporaryDirectory() as d:
        c, exe = os.path.join(d, 't.c'), os.path.join(d, 't')
        open(c, 'w').write(STUB % body)
        subprocess.run(['cc', '-O1', '-o', exe, c], check=True)
        bad = 0
        for prof, tables in EXPECT.items():
            for swap in (0, 1):
                for p in PRESSES:
                    got = int(subprocess.run([exe, str(prof), str(swap)] + [str(DEV[x]) for x in p.split('+')],
                                             capture_output=True, text=True).stdout)
                    want = sum(1 << BIT[n] for n in tables[swap][p].split())
                    if got != want:
                        bad += 1
                        names = ' '.join(n for n, b in BIT.items() if got >> b & 1) or 'nothing'
                        print('%-8s switch %s  %-10s -> %-14s expected %s' % (
                            NAMES[prof], 'on ' if swap else 'off', p, names, tables[swap][p] or 'nothing'))
        print('%d presses checked, %d differ' % (len(EXPECT) * 2 * len(PRESSES), bad))
        sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
