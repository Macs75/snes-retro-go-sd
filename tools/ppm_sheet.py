#!/usr/bin/env python3
"""Tile a directory of host SNES_SHOT frames (f<n>.ppm) into one labelled PNG.

usage: ppm_sheet.py <shot_dir> <out.png> [columns]"""
import os
import sys

from PIL import Image, ImageDraw


def main():
    d, out = sys.argv[1], sys.argv[2]
    cols = int(sys.argv[3]) if len(sys.argv) > 3 else 5
    names = sorted(n for n in os.listdir(d) if n.endswith(".ppm"))
    if not names:
        sys.exit("no .ppm frames in " + d)
    ims = [Image.open(os.path.join(d, n)).convert("RGB") for n in names]
    w, h = ims[0].size
    rows = (len(ims) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * w, rows * (h + 14)), (40, 40, 40))
    dr = ImageDraw.Draw(sheet)
    for i, (n, im) in enumerate(zip(names, ims)):
        x, y = (i % cols) * w, (i // cols) * (h + 14)
        sheet.paste(im, (x, y + 14))
        dr.text((x + 2, y + 1), n[1:-4].lstrip("0") or "0", fill=(255, 255, 0))
    sheet.save(out)


if __name__ == "__main__":
    main()
