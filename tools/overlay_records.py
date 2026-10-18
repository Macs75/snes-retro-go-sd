#!/usr/bin/env python3
"""List the overlay records of a packed snes.bin, and which Super FX game block
code each given cart would load, by the rules of copro_overlay_load (main_snes.c).

The records follow the CORE segments (tools/append_overlays.py):
    u32 tag, u32 run address, u32 code+data size, u32 bss size, payload
A GSGM record (one game's GSU blocks) has a payload starting with its header:
    u32 magic "GSGM", u32 cart id, ...
and is loaded for the cart whose internal header ($7FC0, 32 bytes, copier header
removed) hashes (FNV-1a) to that id: gsu_blk_cart_id in gsu_blocks.h.

  overlay_records.py snes.bin [cart.sfc ...]
"""
import argparse, struct, sys


def fnv1a(data):
    h = 0x811C9DC5
    for x in data:
        h = ((h ^ x) * 0x01000193) & 0xFFFFFFFF
    return h


def records(path):
    d = open(path, 'rb').read()
    if d[:4] != b'CORE':
        sys.exit('%s: no CORE header' % path)
    off = 8 + struct.unpack_from('<H', d, 6)[0]
    nseg = struct.unpack_from('<I', d, 8 + 12)[0]
    for k in range(nseg):
        off += struct.unpack_from('<I', d, 8 + 16 + k * 12 + 4)[0]
    out = []
    while off + 16 <= len(d):
        tag, run, size, bss = struct.unpack_from('<4sIII', d, off)
        out.append((tag.decode('latin1'), run, size, bss, d[off + 16:off + 16 + size]))
        off += 16 + size
    if off != len(d):
        sys.exit('%s: %d bytes left after the last record' % (path, len(d) - off))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bin')
    ap.add_argument('carts', nargs='*')
    a = ap.parse_args()
    recs = records(a.bin)
    for tag, run, size, bss, payload in recs:
        extra = ''
        if tag == 'GSGM':
            magic, cid = struct.unpack_from('<4sI', payload, 0)
            extra = '  header %s, cart id 0x%08X' % (magic.decode('latin1'), cid)
        print('%s at 0x%08x: %7d + %6d B%s' % (tag, run, size, bss, extra))
    for cart in a.carts:
        d = open(cart, 'rb').read()
        if len(d) % 1024 == 512:
            d = d[512:]
        cid = fnv1a(d[0x7FC0:0x7FE0])
        title = d[0x7FC0:0x7FD5].decode('latin1').rstrip()
        games = [r for r in recs if r[0] == 'GSGM' and struct.unpack_from('<4sI', r[4], 0) == (b'GSGM', cid)]
        print('%s ("%s", cart id 0x%08X): %s' % (cart, title, cid,
              'loads the GSGM record at 0x%08x, %d B' % (games[0][1], games[0][2]) if games
              else 'no game block code, the interpreter alone'))


if __name__ == '__main__':
    main()
