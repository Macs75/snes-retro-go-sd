#!/usr/bin/env python3
"""Linker lines that place the SA-1 overlay's hot code where a reference
build had it in the I-cache: each function's section at the address its
function had in the reference ELF, modulo 8 KB (the I-cache way, see
docs/perf-lessons.md), laid out in order of that offset (a function that
would overlap the one before goes one 8 KB window on). The lines go first in
.sa1_overlay (snes_core.ld); what they do not name follows as before.

  sa1_overlay_pin.py <reference snes_core.elf> [--objs build/core] [FUNC:OBJ:SECTION ...]

Default FUNC list: the overlay's hot code in a Kirby Super Star replay on
gwemu (tools/qemu_exec_profile.py). A function's offset in its section comes
from the current object (an assembly section holds more than its entry).
"""
import argparse, os, re, subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WAY = 0x2000
HOT = ['sa1_cpu_write', 'sa1e_wr8', 'sa1_sync', 'sa1e_rd16', 'sa1_reg_read', 'sa1_reg_write', 'sa1_cpu_read',
       'sa1e_wr16', 'sa1_snes_read', 'sa1_ahead', 'sa1_interpret', 'sa1_peek']
DEFAULT = ['%s:sa1.o:.text.%s' % (f, f) for f in HOT] + \
    ['idle_sa1_awake_span:main_snes.o:.sa1ovl.text', 'sa1_engine_run:sa1_engine.o:.text.sa1_engine']


def nm(elf):
    out = subprocess.run(['arm-none-eabi-nm', elf], capture_output=True, text=True, check=True).stdout
    return {f[2]: int(f[0], 16) & ~1 for f in (l.split() for l in out.splitlines()) if len(f) == 3}


def in_section(obj, sec, name):
    """the function's offset in its section, and the section's size and alignment"""
    t = subprocess.run(['arm-none-eabi-objdump', '-t', '-h', obj], capture_output=True, text=True, check=True).stdout
    off = size = align = None
    for l in t.splitlines():
        f = l.split()
        if len(f) >= 7 and f[1] == sec and f[0].isdigit():
            size, align = int(f[2], 16), 1 << int(f[6].split('**')[1])
        m = re.match(r'^([0-9a-f]{8}) .{7} (\S+)\s+[0-9a-f]{8} (\S+)$', l)
        if m and m.group(2) == sec and m.group(3) == name:
            off = int(m.group(1), 16) & ~1
    assert off is not None and size is not None, (obj, sec, name)
    return off, size, align


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ref')
    ap.add_argument('--objs', default=os.path.join(ROOT, 'build/core'))
    ap.add_argument('funcs', nargs='*', default=DEFAULT)
    a = ap.parse_args()
    ref = nm(a.ref)
    pins = []
    for spec in a.funcs:
        name, obj, sec = spec.split(':')
        off, size, align = in_section(os.path.join(a.objs, obj), sec, name)
        start = (ref[name] - off) % WAY            # the section's start, for the function to land on ref's offset
        assert start % align == 0, (name, hex(start), align)
        pins.append((start, size, obj, sec, name))
    pins.sort()
    print('      /* pinned: the I-cache offsets of %s (tools/sa1_overlay_pin.py) */' % os.path.basename(a.ref))
    for start, size, obj, sec, name in pins:
        # in an OVERLAY member . is the offset from its start, __CORE_BSS_END__
        # (ABSOLUTE(.) there corrupts the later overlays' load addresses)
        print('      . = . + ((0x%04x - ((__CORE_BSS_END__ + .) & 0x%x)) & 0x%x);' % (start, WAY - 1, WAY - 1))
        print('      *%s(%s)   /* %s, %d B */' % (obj, sec, name, size))


if __name__ == '__main__':
    main()
