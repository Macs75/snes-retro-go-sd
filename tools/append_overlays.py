#!/usr/bin/env python3
"""Append the coprocessor overlays to a packed snes.bin.

The overlays are the ELF's OVERLAY sections (snes_core.ld): the code of the
Super FX (.gsu_overlay, plus its .gsu_overlay_bss), Cx4, S-DD1, SPC7110 and
DSP-1, all linked to run at the start of the RAM_EMU pool. They are appended
after the CORE segments, which the firmware loader never reads past, one record
each:

    u32 tag, u32 run address, u32 code+data size, u32 bss size,
    code+data bytes

copro_overlay_load() in main_snes.c walks the records to its chip's tag and
checks the address and both sizes against its own linker symbols before
copying, so a snes.bin and overlays from different links can never be combined.

Each game's Super FX block code (.gsugame_<game>, found by its
__GSUGAME_<game>_START__/_END__ symbols) follows as a GSGM record, its payload
starting with the game's header (magic, cart id); the loader reads the one whose
cart id matches the cart together with the Super FX overlay.

Usage: append_overlays.py <elf> <snes.bin> [--nm NM] [--objcopy OBJCOPY]
"""
import argparse
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

# tag, section, start, code end, end (bss end)
OVERLAYS = [
    (b"GSUO", ".gsu_overlay", "__GSU_OVL_START__", "__GSU_OVL_CODE_END__", "__GSU_OVL_END__"),
    (b"CX4O", ".cx4_overlay", "__CX4_OVL_START__", "__CX4_OVL_END__", "__CX4_OVL_END__"),
    (b"SDD1", ".sdd1_overlay", "__SDD1_OVL_START__", "__SDD1_OVL_END__", "__SDD1_OVL_END__"),
    (b"S711", ".spc7110_overlay", "__SPC7110_OVL_START__", "__SPC7110_OVL_END__", "__SPC7110_OVL_END__"),
    (b"DSP1", ".dsp1_overlay", "__DSP1_OVL_START__", "__DSP1_OVL_END__", "__DSP1_OVL_END__"),
    (b"SA1O", ".sa1_overlay", "__SA1_OVL_START__", "__SA1_OVL_END__", "__SA1_OVL_END__"),
]


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
    records = []
    with tempfile.TemporaryDirectory() as d:
        for tag, section, sym0, sym1, sym2 in OVERLAYS:
            try:
                start, code_end, end = s[sym0], s[sym1], s[sym2]
            except KeyError as e:
                sys.exit(f"error: {a.elf} has no {e.args[0]} (no {section}?)")
            code, bss = code_end - start, end - code_end
            raw = Path(d) / (section.strip(".") + ".bin")
            subprocess.run([a.objcopy, "-O", "binary", f"--only-section={section}", str(a.elf), str(raw)],
                           check=True)
            payload = raw.read_bytes() if raw.exists() else b""
            if len(payload) != code:
                sys.exit(f"error: {section} is {len(payload)} bytes, symbols say {code}")
            records.append((tag, start, code, bss, payload))
        for game in sorted(k[len("__GSUGAME_"):-len("_START__")] for k in s
                           if k.startswith("__GSUGAME_") and k.endswith("_START__")):
            section = ".gsugame_" + game
            start, end = s["__GSUGAME_%s_START__" % game], s["__GSUGAME_%s_END__" % game]
            raw = Path(d) / (section.strip(".") + ".bin")
            subprocess.run([a.objcopy, "-O", "binary", f"--only-section={section}", str(a.elf), str(raw)],
                           check=True)
            payload = raw.read_bytes() if raw.exists() else b""
            if len(payload) != end - start or payload[:4] != b"GSGM":
                sys.exit(f"error: {section}: {len(payload)} bytes (symbols say {end - start}), "
                         f"header {payload[:4]!r}")
            records.append((b"GSGM", start, end - start, 0, payload))

    with open(a.packed, "ab") as f:
        for tag, start, code, bss, payload in records:
            f.write(tag + struct.pack("<III", start, code, bss))
            f.write(payload)
    for tag, start, code, bss, _ in records:
        print(f"append_overlays: {tag.decode()} {code}+{bss} B at 0x{start:08x} -> {a.packed}")


if __name__ == "__main__":
    main()
