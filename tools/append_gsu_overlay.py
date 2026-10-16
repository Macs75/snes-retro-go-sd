#!/usr/bin/env python3
"""Append the Super FX (GSU) overlay to a packed snes.bin.

The overlay is the ELF's .gsu_overlay section (gsu_core.o's code and data,
linked to run at the start of the RAM_EMU pool; see snes_core.ld). It is
appended after the CORE segments, which the firmware loader never reads past:

    u32 magic 'GSUO', u32 run address, u32 code+data size, u32 bss size,
    code+data bytes

gsu_overlay_load() in main_snes.c checks all four words against its own
linker symbols before copying, so a snes.bin and an overlay from different
links can never be combined.

Usage: append_gsu_overlay.py <elf> <snes.bin> [--nm NM] [--objcopy OBJCOPY]
"""
import argparse
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

MAGIC = 0x4F555347  # "GSUO"


def symbols(nm, elf):
    out = subprocess.run([nm, str(elf)], check=True, capture_output=True, text=True).stdout
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            syms[parts[2]] = int(parts[0], 16)
    return syms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf", type=Path)
    ap.add_argument("packed", type=Path)
    ap.add_argument("--nm", default="arm-none-eabi-nm")
    ap.add_argument("--objcopy", default="arm-none-eabi-objcopy")
    a = ap.parse_args()

    s = symbols(a.nm, a.elf)
    try:
        start, code_end, end = s["__GSU_OVL_START__"], s["__GSU_OVL_CODE_END__"], s["__GSU_OVL_END__"]
    except KeyError as e:
        sys.exit(f"error: {a.elf} has no {e.args[0]} (no .gsu_overlay?)")
    code, bss = code_end - start, end - code_end

    with tempfile.TemporaryDirectory() as d:
        raw = Path(d) / "gsu_overlay.bin"
        subprocess.run([a.objcopy, "-O", "binary", "--only-section=.gsu_overlay", str(a.elf), str(raw)],
                       check=True)
        payload = raw.read_bytes()
    if len(payload) != code:
        sys.exit(f"error: .gsu_overlay is {len(payload)} bytes, symbols say {code}")

    with open(a.packed, "ab") as f:
        f.write(struct.pack("<IIII", MAGIC, start, code, bss))
        f.write(payload)
    print(f"append_gsu_overlay: {code}+{bss} B at 0x{start:08x} -> {a.packed}")


if __name__ == "__main__":
    main()
