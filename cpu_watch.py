#!/usr/bin/env python3
"""cpu_watch.py [seconds] [hz]  --  per-process CPU, sampled from /proc, no `top`.

map->odom was measured ageing to 8.7 s during nav with slam_toolbox's motion gate
already wide open (minimum_time_interval 0.2 s), which leaves only one explanation
worth testing: slam is being handed 10 Hz of scans and cannot consume them that
fast, so the stamp it publishes is the timestamp of an ever-older scan.

That is a claim about CPU, and it is only believable if slam_toolbox is actually
saturated *while the age is climbing*. `top` averages over the whole window and
hides that. This samples /proc/<pid>/stat directly, reports each process's own
utilisation over the window, and prints a per-second timeline for the processes
that matter so the CPU trace can be laid alongside tf_age.py's age trace.

Percentages are per-core: 100% means one core fully busy, 400% means all four on
this Pi 5.
"""
import os
import sys
import time

WATCH = ("slam_toolbox", "rf2o", "controller_server", "planner_server",
         "ldlidar", "scan_normalize", "foxglove", "l298n")
HZ_TICKS = os.sysconf("SC_CLK_TCK")


def procs():
    """pid -> short label, for anything whose cmdline matches WATCH."""
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        for w in WATCH:
            if w in cmd:
                out[int(pid)] = w
                break
    return out


def cpu_ticks(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            parts = f.read().rsplit(") ", 1)[1].split()
        return int(parts[11]) + int(parts[12])          # utime + stime
    except (OSError, IndexError):
        return None


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    interval = 1.0 / hz

    labels = procs()
    if not labels:
        print("cpu_watch: no matching processes")
        return 1
    print("watching: " + ", ".join(sorted(set(labels.values()))), flush=True)

    prev = {p: cpu_ticks(p) for p in labels}
    prev_t = time.time()
    series = {}
    t0 = time.time()
    while time.time() - t0 < dur:
        time.sleep(interval)
        now = time.time()
        dt = now - prev_t
        # Re-scan: nav nodes come and go across attempts, and a process that only
        # exists for part of the run is exactly the kind of thing being hunted.
        labels = procs()
        agg = {}
        for p, lab in labels.items():
            c = cpu_ticks(p)
            if c is None:
                continue
            if p in prev:
                pct = 100.0 * (c - prev[p]) / HZ_TICKS / dt
                agg[lab] = agg.get(lab, 0.0) + pct
            prev[p] = c
        prev_t = now
        for lab, pct in agg.items():
            series.setdefault(lab, []).append((now - t0, pct))

    print(f"\n=== CPU over {dur:.0f}s (percent of ONE core; {os.cpu_count()} cores) ===")
    for lab in sorted(series):
        vals = [v for _, v in series[lab]]
        vals_sorted = sorted(vals)
        p95 = vals_sorted[min(len(vals) - 1, int(0.95 * len(vals)))]
        print(f"  {lab:18s} mean {sum(vals) / len(vals):6.1f}%  "
              f"p95 {p95:6.1f}%  max {max(vals):6.1f}%")

    print("\n  per-second timeline (percent of one core):")
    for lab in sorted(series):
        vals = [v for _, v in series[lab]]
        for row in range(0, len(vals), 30):
            chunk = vals[row:row + 30]
            tag = f"{lab}@{int(series[lab][row][0]):3d}s"
            print(f"    {tag:22s} " + " ".join(f"{v:4.0f}" for v in chunk))
    return 0


if __name__ == "__main__":
    sys.exit(main())
