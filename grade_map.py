#!/usr/bin/env python3
"""grade_map.py [name]  -- grade a saved slam map for nav readiness.

Run automatically by `stack.sh` after `map save` (and runnable by hand).
Reads /root/maps/<name>.{pgm,yaml,seed} and reports:
  * coverage  : % free / unknown / occupied
  * explored footprint (the room size actually seen)
  * free-space connectivity : # components, % of free in the largest one
  * whether the AMCL seed pose and the nav goal (map origin 0,0) are
    connected through confirmed-free cells (the real pass/fail for nav)

Exit code 0 = good enough to nav, 2 = keep mapping.
"""
import sys, os, re, math, time
import numpy as np
from collections import deque

name = sys.argv[1] if len(sys.argv) > 1 else "room"
MAPS = "/root/maps"
PGM  = f"{MAPS}/{name}.pgm"
YML  = f"{MAPS}/{name}.yaml"
SEED = f"{MAPS}/{name}.seed"

for p in (PGM, YML):
    if not os.path.exists(p):
        print(f"[grade] missing {p} -- save the map first."); sys.exit(1)

print(f"[grade] map = {name}   (pgm mtime {time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(PGM)))})")

txt = open(YML).read()
res = float(re.search(r'resolution:\s*([\d.]+)', txt).group(1))
m = re.search(r'origin:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)', txt)
ox, oy = float(m.group(1)), float(m.group(2))

with open(PGM, "rb") as f:
    f.readline(); line = f.readline()
    while line.startswith(b"#"): line = f.readline()
    w, h = map(int, line.split()); f.readline()
    d = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
N = w * h
FREE, UNK, OCC = 254, 205, 0
nf = int((d == FREE).sum()); nu = int((d == UNK).sum()); no = int((d == OCC).sum())

print("  size      : %dx%d  = %.2f x %.2f m" % (w, h, w * res, h * res))
print("  coverage  : free=%d%%  unknown=%d%%  occupied=%d%%   (aim: free > ~55%%)" %
      (round(100 * nf / N), round(100 * nu / N), round(100 * no / N)))

ys, xs = np.where(d != UNK)
if len(xs):
    print("  explored  : %.2f x %.2f m of the room seen" %
          ((xs.max() - xs.min()) * res, (ys.max() - ys.min()) * res))

# ---- coverage map: WHERE the unknown is (so "keep mapping" is actionable) ----
CCOLS, CROWS = 26, 30          # coarse grid size
gmap = np.zeros((CROWS, CCOLS), np.int8)   # 0=unknown-dominant,1=known,2=mixed
for rr in range(CROWS):
    for cc in range(CCOLS):
        blk = d[rr * h // CROWS:(rr + 1) * h // CROWS, cc * w // CCOLS:(cc + 1) * w // CCOLS]
        if blk.size == 0: continue
        if (blk == UNK).mean() > 0.6:   gmap[rr, cc] = 0
        elif (blk != UNK).mean() > 0.6: gmap[rr, cc] = 1
        else:                           gmap[rr, cc] = 2
print("  coverage map (26x30; '.'=mapped, '?'=unknown, '+'=mixed, O=origin/drop-point):")
oc = int((0 - ox) / res); orr = int(((oy + h * res) - 0) / res)   # map origin (0,0) pixel
for rr in range(CROWS):
    row = ""
    for cc in range(CCOLS):
        ch = {0: '?', 1: '.', 2: '+'}[gmap[rr, cc]]
        if rr == orr * CROWS // h and cc == oc * CCOLS // w:
            ch = 'O'
        row += ch
    print("     " + row)
# 3x3 region hints: unknown% per region, list the worst
region_hints = []
for ry in range(3):
    for rx in range(3):
        blk = d[ry * h // 3:(ry + 1) * h // 3, rx * w // 3:(rx + 1) * w // 3]
        u = 100.0 * (blk == UNK).mean()
        if u > 50:
            where = ("top " if ry == 0 else "middle " if ry == 1 else "bottom ") + \
                    ("left" if rx == 0 else "center" if rx == 1 else "right")
            region_hints.append((u, where))
region_hints.sort(reverse=True)
if region_hints:
    print("  biggest unknown patches: " + " | ".join("%s (%d%% unknown)" % (w2, round(u)) for u, w2 in region_hints[:4]))

def isfree(c, r): return 0 <= r < h and 0 <= c < w and d[r, c] == FREE
ymax = oy + h * res
def w2p(x, y): return int(round((x - ox) / res)), int(round((ymax - y) / res))

# connected components of free cells (4-connectivity)
seen = np.zeros((h, w), bool); comp = np.zeros((h, w), np.int32); cid = 0; sizes = {}
for rr in range(h):
    for cc in range(w):
        if isfree(cc, rr) and not seen[rr, cc]:
            cid += 1; sz = 0; q = deque([(cc, rr)]); seen[rr, cc] = True
            while q:
                x0, y0 = q.popleft(); comp[y0, x0] = cid; sz += 1
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x0 + dx, y0 + dy
                    if isfree(nx, ny) and not seen[ny, nx]:
                        seen[ny, nx] = True; q.append((nx, ny))
            sizes[cid] = sz
largest = max(sizes.values()); largestpct = 100.0 * largest / nf
print("  free conn : %d components; largest = %d%% of all free   (aim: ~1-3 comps, largest > ~85%%)" %
      (cid, round(largestpct)))

def nearest_free_comp(c, r):
    if not (0 <= r < h and 0 <= c < w): return 0
    if isfree(c, r): return int(comp[r, c])
    for rad in range(1, 20):
        for dr in range(-rad, rad + 1):
            for dcc in range(-rad, rad + 1):
                if abs(dr) == rad or abs(dcc) == rad:
                    rr, cc = r + dr, c + dcc
                    if 0 <= rr < h and 0 <= cc < w and isfree(cc, rr): return int(comp[rr, cc])
    return 0

def nearest_occ_dist_m(c, r, maxr=40):
    """approx distance (m) from cell (c,r) to the nearest occupied cell; None if > maxr cells."""
    for rad in range(1, maxr + 1):
        for dr in range(-rad, rad + 1):
            for dcc in range(-rad, rad + 1):
                if abs(dr) == rad or abs(dcc) == rad:
                    rr, cc = r + dr, c + dcc
                    if 0 <= rr < h and 0 <= cc < w and d[rr, cc] == OCC:
                        return rad * res
    return None

# seed (robot pose at map save) + goal (map origin = where mapping started)
if os.path.exists(SEED):
    sx, sy, syaw = map(float, open(SEED).read().split()[:3])
    sc, sr = w2p(sx, sy)
    scf = nearest_free_comp(sc, sr)
    seed_clear = nearest_occ_dist_m(sc, sr)
    seed_clear_txt = "%.2f m" % seed_clear if seed_clear is not None else "> %.1f m" % (40 * res)
    print("  seed      : (%.2f, %.2f, yaw %.0f deg)  -> free comp #%d   clearance %s" % (sx, sy, math.degrees(syaw), scf, seed_clear_txt))
else:
    print("  seed      : (no .seed file)"); scf = 0; seed_clear = None

seed_ok = seed_clear is not None and seed_clear >= 0.30   # inside the inflation zone -> Nav2 can't plan from the start

gc, gr = w2p(0, 0)
gcf = nearest_free_comp(gc, gr)
goal_clear = nearest_occ_dist_m(gc, gr)
goal_clear_txt = "%.2f m" % goal_clear if goal_clear is not None else "> %.1f m" % (40 * res)
print("  goal (0,0): map origin               -> free comp #%d   clearance %s" % (gcf, goal_clear_txt))
conn = scf != 0 and gcf != 0 and scf == gcf
print("  CONNECTED : seed <-> goal through free space : %s" % ("YES" if conn else "NO"))
if seed_clear is not None and seed_clear < 0.30:
    print("  !! WARN    : seed is %.2f m from the nearest wall -- the robot was parked in/near a wall at" % seed_clear)
    print("               `map save`. With the costmap inflation that start cell is lethal and Nav2 will")
    print("               fail to plan immediately. Park the rover in OPEN space (>= 0.5 m from walls)")
    print("               and re-run `map save` (no need to re-map the room).")
if goal_clear is not None and goal_clear < 0.25:
    print("  !! WARN    : goal (0,0) is %.2f m from a wall -- the drop point was against a wall, so the" % goal_clear)
    print("               map origin sits in a wall gap. Nav2's goal tolerance may still find a spot, but")
    print("               re-mapping with the rover dropped in the OPEN (>= 0.5 m from walls) is safer.")

ok = (100.0 * nf / N >= 45) and (largestpct >= 70) and conn and seed_ok
print("  " + "=" * 50)
print("  VERDICT   : %s" % ("READY to nav  :-)" if ok else "KEEP MAPPING -- fix the issues below, then re-save"))
if not ok:
    tips = []
    if 100.0 * nf / N < 45: tips.append("drive the '?' patches in the coverage map above (esp. %s); the LD19 only sees ~3-4 m, so keep every part of the room within ~3 m of your path" % (region_hints[0][1] if region_hints else "the edges"))
    if largestpct < 70:    tips.append("free space is fragmented -- slow smooth driving + close the loop near start")
    if not conn:           tips.append("seed and goal aren't connected -- you must map a continuous path between them")
    if not seed_ok:        tips.append("seed too close to a wall -- park in open space and re-`map save` (or re-map with the rover in the open)")
    print("             " + " | ".join(tips))
print("  " + "=" * 50)
sys.exit(0 if ok else 2)
