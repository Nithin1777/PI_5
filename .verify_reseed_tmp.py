#!/usr/bin/env python3
"""Offline repro: reseed.py ymax-shadowing bug vs fix (runs in the container,
imports the REAL reseed.load_map so the distance transform is identical)."""
import math
import numpy as np
from reseed import load_map

res, ox, oy, w, h, d, DT = load_map("room")
gt = tuple(map(float, open("/root/maps/room.seed").read().split()))
gx, gy, gyaw = gt
print(f"ground truth (room.seed): ({gx:.3f}, {gy:.3f}, {math.degrees(gyaw):.1f} deg)")

# ideal 504-beam scan raycast from ground truth (LD19 convention: angle_min=0, inc=2pi/504)
N = 504
ainc = 2 * math.pi / N
RMAX = 2.8
beams = []
for i in range(N):
    a = i * ainc
    r = 0.1
    hit = None
    while r <= RMAX:
        X = gx + r * math.cos(gyaw + a)
        Y = gy + r * math.sin(gyaw + a)
        col = int(round((X - ox) / res))
        row = int(round(((oy + h * res) - Y) / res))
        if 0 <= col < w and 0 <= row < h and d[row, col] == 0:
            hit = r
            break
        r += 0.01
    if hit is not None:
        beams.append((a, hit))

print(f"beams kept: {len(beams)} (same subsample rule as reseed)")


def search(ymax_eff):
    def cost(x, y, yaw):
        tot = 0.0
        for a, r in beams:
            th = yaw + a
            col = int(round((x + r * math.cos(th) - ox) / res))
            row = int(round((ymax_eff - (y + r * math.sin(th))) / res))
            if 0 <= col < w and 0 <= row < h:
                tot += DT[row, col]
            else:
                tot += 2.0
        return tot / len(beams)

    best = (1e9, None, None, None)
    for x in np.arange(ox + 0.4, ox + w * res - 0.4, 0.25):
        for y in np.arange(oy + 0.4, oy + h * res - 0.4, 0.25):
            for yaw in np.arange(0.0, 2 * math.pi, math.radians(15)):
                c = cost(x, y, yaw)
                if c < best[0]:
                    best = (c, x, y, yaw)
    c0, x0, y0, y0a = best
    for step, ystep, span in ((0.05, math.radians(3), 0.3), (0.02, math.radians(1), 0.1)):
        for x in np.arange(x0 - span, x0 + span + 1e-9, step):
            for y in np.arange(y0 - span, y0 + span + 1e-9, step):
                for yaw in np.arange(y0a - 3 * ystep, y0a + 3 * ystep + 1e-9, ystep):
                    c = cost(x, y, yaw)
                    if c < c0:
                        c0, x0, y0, y0a = c, x, y, yaw
    return c0, x0, y0, y0a


c_b, x_b, y_b, a_b = search(oy + h * res - 0.4)   # buggy: ymax shadowed by the search bound
c_f, x_f, y_f, a_f = search(oy + h * res)          # fixed: true top edge
print(f"buggy: ({x_b:.3f}, {y_b:.3f}, {math.degrees(a_b):.1f} deg)  cost {c_b:.4f}  dy={y_b - gy:+.3f}")
print(f"fixed: ({x_f:.3f}, {y_f:.3f}, {math.degrees(a_f):.1f} deg)  cost {c_f:.4f}  dy={y_f - gy:+.3f}")
