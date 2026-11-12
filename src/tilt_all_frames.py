#!/usr/bin/env python3
"""
Module-tilt term of the gyro3 score over ALL frames of valid cycles, by mode and by
pre-burst platform-rate bin. Prints median and mean so the statistic is explicit.

Usage (from deblur_data):
  python3 ../SPIRE/src/tilt_all_frames.py session_20261107_14* session_20261108_121*
"""
import csv, os, statistics, sys

BINS = [(5, 10), (10, 20), (20, 40), (40, 70)]
data = {}
for s in sorted(sys.argv[1:]):
    p = os.path.join(s, "manifest.csv")
    if not os.path.exists(p):
        continue
    for r in csv.DictReader(open(p, newline="")):
        if r["cycle_valid"] not in ("1", "True", "true") or r["mode"] == "STATIC":
            continue
        try:
            rot, tilt = abs(float(r["rotation_speed_dps"])), float(r["blur_tilt_px"])
        except (KeyError, ValueError):
            continue
        for lo, hi in BINS:
            if lo <= rot < hi:
                data.setdefault((r["mode"], (lo, hi)), []).append(tilt)
print(f"{'mode':12s} {'bin deg/s':10s} {'frames':>6s} {'median px':>10s} {'mean px':>8s}")
for (mode, (lo, hi)), v in sorted(data.items(), key=lambda k: (k[0][1], k[0][0])):
    print(f"{mode:12s} {lo:>3d}-{hi:<6d} {len(v):6d} {statistics.median(v):10.2f} {statistics.mean(v):8.2f}")
