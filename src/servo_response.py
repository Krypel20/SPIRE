#!/usr/bin/env python3
"""
SPIRE servo response check for one bench session.

For every cycle where the servo was commanded, compares the commanded servo
rate (slope of servo_cmd_deg) with the rate the servo actually produced,
measured by the IMUs as camera gyro_z minus platform gyro_z. A servo that
lost power, lost its signal or went into overload protection still gets
commands but produces no relative motion, so its gain drops to ~0.

Usage:
  python3 src/servo_response.py data/bench/session_20261022_093326
  python3 src/servo_response.py            # newest session (USB drive or ./data)

Output is a short table meant to be pasted back for analysis.
"""

import csv
import glob
import os
import sys
from statistics import median

WINDOW_NS = 200_000_000      # 200 ms analysis windows inside the burst
MIN_CMD_RATE = 10.0          # deg/s; slower windows are too noisy to judge
SAT_DEG = 134.0              # |command| at or above this counts as saturated


def load_imu(path):
    d = {"camera": [], "platform": [], "servo_cmd_deg": []}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["source"] in d:
                d[r["source"]].append((int(r["mono_ns"]), float(r["gyro_z_dps"])))
    return d


def mean_in(samples, a, b):
    v = [x for t, x in samples if a <= t < b]
    return sum(v) / len(v) if v else None


def slope(samples, a, b):
    pts = [(t / 1e9, x) for t, x in samples if a <= t < b]
    if len(pts) < 3:
        return None
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    mx = sum(x for _, x in pts) / n
    den = sum((t - mt) ** 2 for t, _ in pts)
    return sum((t - mt) * (x - mx) for t, x in pts) / den if den else None


def analyse_cycle(path):
    d = load_imu(path)
    sv = d["servo_cmd_deg"]
    if not sv or max(abs(x) for _, x in sv) < 1.0:
        return None                      # servo not commanded (CALM/HELD)
    t0, t1 = sv[0][0], sv[-1][0]
    gains, cmds, acts = [], [], []
    a = t0
    while a + WINDOW_NS <= t1:
        b = a + WINDOW_NS
        cmd = slope(sv, a, b)
        cam = mean_in(d["camera"], a, b)
        plat = mean_in(d["platform"], a, b)
        if cmd is not None and cam is not None and plat is not None \
                and abs(cmd) >= MIN_CMD_RATE:
            act = cam - plat             # servo rate relative to the platform
            gains.append(act / cmd)
            cmds.append(cmd)
            acts.append(act)
        a = b
    peak = max(abs(x) for _, x in sv)
    return {
        "gain": median(gains) if gains else None,
        "cmd": median(cmds) if cmds else None,
        "act": median(acts) if acts else None,
        "windows": len(gains),
        "peak_cmd": peak,
        "saturated": peak >= SAT_DEG,
        "end_cmd": sv[-1][1],
    }


def verdict(g):
    if g is None:
        return "n/a"
    if abs(g) < 0.15:
        return "NO RESPONSE"
    if g < 0.45:
        return "weak"
    return "ok"


def main():
    if len(sys.argv) > 1:
        session = sys.argv[1]
    else:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import storage
        sessions = storage.find_sessions("bench")
        if not sessions:
            sys.exit("no session found")
        session = sessions[-1]

    print(f"Session: {session}")

    # Exposure actually used (the lock can fail; then blur numbers are wrong)
    exposures = set()
    mpath = os.path.join(session, "manifest.csv")
    if os.path.exists(mpath):
        with open(mpath, newline="") as fh:
            for r in csv.DictReader(fh):
                exposures.add(r.get("exposure_time_us", ""))
    print(f"Exposure values in manifest (us): {sorted(exposures)}")

    print(f"{'cyc':>4} {'cmd':>7} {'actual':>7} {'gain':>5} {'peak':>6} "
          f"{'end':>7}  verdict")
    prev_sat = False
    counts = {}
    for path in sorted(glob.glob(os.path.join(session, "c*_imu.csv"))):
        cyc = os.path.basename(path)[:4]
        r = analyse_cycle(path)
        if r is None:
            print(f"{cyc:>4} {'-':>7} {'-':>7} {'-':>5} {'-':>6} {'-':>7}  not commanded")
            prev_sat = False
            continue
        v = verdict(r["gain"])
        counts[v] = counts.get(v, 0) + 1
        fmt = lambda x, f: "n/a" if x is None else format(x, f)
        note = " (prev cycle saturated)" if prev_sat else ""
        print(f"{cyc:>4} {fmt(r['cmd'], '+7.1f')} {fmt(r['act'], '+7.1f')} "
              f"{fmt(r['gain'], '5.2f')} {r['peak_cmd']:6.1f} {r['end_cmd']:+7.1f}  "
              f"{v}{' SAT' if r['saturated'] else ''}{note}")
        prev_sat = r["saturated"]
    print("Summary: " + ", ".join(f"{k} {n}" for k, n in sorted(counts.items())))


if __name__ == "__main__":
    main()
