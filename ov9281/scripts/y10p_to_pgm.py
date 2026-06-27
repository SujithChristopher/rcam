#!/usr/bin/env python3
"""Convert a raw Y10P (MIPI RAW10 packed, mono) capture to an 8-bit PGM preview.

Y10P packing: every 5 bytes hold 4 pixels; bytes 0..3 are the upper 8 bits of
pixels 0..3, byte 4 holds the low 2 bits of each. For a quick grey preview we
take the upper 8 bits (bytes 0..3) and drop the 5th byte.

Usage: y10p_to_pgm.py <raw> [width=1280] [height=800] [out.pgm]
No numpy/PIL required.
"""
import sys

raw = sys.argv[1]
W = int(sys.argv[2]) if len(sys.argv) > 2 else 1280
H = int(sys.argv[3]) if len(sys.argv) > 3 else 800
out = sys.argv[4] if len(sys.argv) > 4 else raw.rsplit('.', 1)[0] + '.pgm'

fsz = W * H * 10 // 8
frame = open(raw, 'rb').read()[:fsz]
px = bytearray(W * H)
j = 0
for i in range(0, fsz, 5):
    px[j], px[j + 1], px[j + 2], px[j + 3] = frame[i], frame[i + 1], frame[i + 2], frame[i + 3]
    j += 4

with open(out, 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (W, H))
    f.write(bytes(px))

print("wrote %s  (min=%d max=%d mean=%.1f)" %
      (out, min(px), max(px), sum(px) / len(px)))
