#!/usr/bin/env python3
"""
Offline check of the gyro3 burst-selection score for different exposure windows.

flight_capture.exposure_motion_px() scores each frame on BOTH candidate windows
[ts-exp, ts] (A) and [ts, ts+exp] (B) and keeps the larger tilt and yaw values
(rule "maxAB"). The deblur window test showed that B is the right window
(libcamera: SensorTimestamp = start of exposure of the first row), so the
question is whether scoring on B alone (rule "B"), or on B shifted by half the
rolling-shutter readout (rule "B+ro", middle row, as in rot_deblur), picks
other / sharper frames.

For every valid burst the script recomputes the gyro3 score of all frames from
c###_imu.csv with each rule, takes the winner (lowest score) and reports:
  1. sanity: rule maxAB must reproduce the logged winner and score_px,
  2. how often B / B+ro change the winner compared with maxAB,
  3. with --sharp (sharpness_frames.csv from sharpness_check.py): the
     within-burst rank of each rule's winner by image blur (1 = sharpest of 6;
     chance: top-1 17 %, mean rank 3.5). Only CORRECTED bursts are meaningful
     here, because without the servo the scene rotates during the burst.
     p = one-sided exact binomial test of the top-1 count against chance 1/6
     (bursts are independent of each other; frames inside a burst are not
     tested individually).

Needs c###_imu.csv with gyro_x_dps / gyro_y_dps (bench_stab_test from Test B
on). Older traces with gyro_z only are skipped.

Usage (from deblur_data):
  python3 ../SPIRE/src/gyro3_window_check.py session_20261108_121925 \\
      --sharp sharpness_frames.csv
"""

import argparse
import csv
import json
import math
import os
from statistics import mean, median

import numpy as np

PIXEL_UM = 1.55
SENSOR_PX = (4056, 3040)
TRUE = ("1", "True", "true")


def load_trace(path):
    """Camera gyro (mono_ns, x, y, z in deg/s) or None without x/y columns."""
    t, x, y, z = [], [], [], []
    with open(path, newline="") as fh:
        rd = csv.DictReader(fh)
        if "gyro_x_dps" not in (rd.fieldnames or []):
            return None
        for r in rd:
            if r.get("source") != "camera":
                continue
            t.append(int(r["mono_ns"]))
            x.append(float(r["gyro_x_dps"]))
            y.append(float(r["gyro_y_dps"]))
            z.append(float(r["gyro_z_dps"]))
    if len(t) < 3:
        return None
    o = np.argsort(t)
    return tuple(np.asarray(v, dtype=np.float64)[o] for v in (t, x, y, z))


def boot_offset(session, cycle):
    try:
        with open(os.path.join(session, f"c{cycle:03d}_meta.json")) as fh:
            for f in json.load(fh).get("frames", []):
                if f.get("sensor_timestamp_ns") and f.get("exp_mono_ns"):
                    return f["sensor_timestamp_ns"] - f["exp_mono_ns"]
    except (OSError, ValueError):
        pass
    return 0


def window_blur(tr, a, b, f_px, radius_px):
    """(tilt_px, yaw_px) over [a, b] ns, same model as flight_capture.frame_blur."""
    t = tr[0]
    if a < t[0] or b > t[-1]:
        return None
    inner = t[(t > a) & (t < b)]
    if len(inner) == 0 and (b - a) > 20e6:
        return None
    pts = np.concatenate(([a], inner, [b]))
    ranges = []
    for v in tr[1:]:
        r = np.interp(pts, t, v)
        ang = np.concatenate(([0.0], np.cumsum(0.5 * (r[1:] + r[:-1]) * np.diff(pts) / 1e9)))
        ranges.append(math.radians(ang.max() - ang.min()))
    return f_px * math.hypot(ranges[0], ranges[1]), radius_px * ranges[2]


def binom_tail(k, n, p):
    """P(X >= k) for X ~ Binomial(n, p)."""
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k, n + 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--sharp", help="sharpness_frames.csv from sharpness_check.py")
    ap.add_argument("--readout-ms", type=float, default=60.0,
                    help="rolling-shutter readout for rule B+ro (shift = half of it)")
    args = ap.parse_args()

    blur = {}
    if args.sharp:
        with open(args.sharp, newline="") as fh:
            for r in csv.DictReader(fh):
                blur[(r["session"], int(r["cycle"]), int(r["frame"]))] = float(r["blur_diag"])

    rules = ("maxAB", "B", "B+ro")
    bursts = []           # dicts: session, cycle, mode, logged winner, per-rule winners
    sanity = []           # |recomputed maxAB score - logged score_px|
    skipped = 0
    for s in sorted(args.sessions):
        name = os.path.basename(os.path.normpath(s))
        try:
            with open(os.path.join(s, "session.json")) as fh:
                bp = json.load(fh).get("bench_params", {})
        except OSError:
            bp = {}
        w, h = bp.get("sensor_px", SENSOR_PX)
        radius_px = bp.get("score_radius", 0.25) * math.hypot(w / 2, h / 2)
        f_px = bp.get("lens_mm", 6.0) * 1e-3 / (PIXEL_UM * 1e-6)
        cycles = {}
        with open(os.path.join(s, "manifest.csv"), newline="") as fh:
            for r in csv.DictReader(fh):
                cycles.setdefault(int(r["cycle_id"]), []).append(r)
        for cyc, fr in sorted(cycles.items()):
            r0 = fr[0]
            if r0["cycle_valid"] not in TRUE or r0["mode"] == "STATIC":
                continue
            ip = os.path.join(s, f"c{cyc:03d}_imu.csv")
            tr = load_trace(ip) if os.path.exists(ip) else None
            if tr is None:
                skipped += 1
                continue
            off = boot_offset(s, cyc)
            scores = {k: [] for k in rules}
            ok = True
            for r in sorted(fr, key=lambda r: int(r["frame_index"])):
                ts = int(r["sensor_timestamp_ns"]) - off
                exp = float(r["exposure_time_us"]) * 1000
                ro = args.readout_ms * 1e6 / 2
                wa = window_blur(tr, ts - exp, ts, f_px, radius_px)
                wb = window_blur(tr, ts, ts + exp, f_px, radius_px)
                wr = window_blur(tr, ts + ro, ts + ro + exp, f_px, radius_px)
                if wb is None or wr is None:
                    ok = False
                    break
                both = [v for v in (wa, wb) if v is not None]
                mx = max(v[0] for v in both) + max(v[1] for v in both)
                scores["maxAB"].append(mx)
                scores["B"].append(sum(wb))
                scores["B+ro"].append(sum(wr))
                if r.get("score_px") not in (None, ""):
                    sanity.append(abs(mx - float(r["score_px"])))
            if not ok:
                skipped += 1
                continue
            logged = [int(r["frame_index"]) for r in fr if r["winner"] in TRUE]
            bursts.append({
                "session": name, "cycle": cyc, "mode": r0["mode"],
                "logged": logged[0] if logged else None,
                "win": {k: int(np.argmin(v)) for k, v in scores.items()},
                "n": len(scores["maxAB"])})

    print(f"bursts used: {len(bursts)}  skipped (no x/y trace or window outside trace): {skipped}")
    if not bursts:
        return
    if sanity:
        print(f"sanity, maxAB vs logged score_px: median |diff| {median(sanity):.3f} px, "
              f"max {max(sanity):.2f} px")
    same = sum(b["win"]["maxAB"] == b["logged"] for b in bursts if b["logged"] is not None)
    print(f"sanity, maxAB winner = logged winner: {same}/{len(bursts)}\n")

    print("winner changed compared with maxAB:")
    for mode in ("UNCORRECTED", "CORRECTED"):
        sel = [b for b in bursts if b["mode"] == mode]
        if not sel:
            continue
        txt = "  ".join(f"{k} {sum(b['win'][k] != b['win']['maxAB'] for b in sel)}/{len(sel)}"
                        for k in ("B", "B+ro"))
        print(f"  {mode:12s} {txt}")

    if not blur:
        return
    print("\nwithin-burst rank of the winner by image blur, CORRECTED "
          "(1 = sharpest; chance top-1 17 %, mean rank 3.5):")
    sel = [b for b in bursts if b["mode"] == "CORRECTED"]
    for k in rules:
        ranks = []
        for b in sel:
            vals = [blur.get((b["session"], b["cycle"], i)) for i in range(b["n"])]
            if any(v is None for v in vals):
                continue
            order = sorted(range(b["n"]), key=lambda i: vals[i])
            ranks.append(order.index(b["win"][k]) + 1)
        if ranks:
            top1 = sum(r == 1 for r in ranks)
            top2 = sum(r <= 2 for r in ranks)
            print(f"  {k:6s} n={len(ranks):3d}  top-1 {top1}/{len(ranks)} ({100 * top1 / len(ranks):.0f} %)"
                  f"  top-2 {100 * top2 / len(ranks):.0f} %  mean rank {mean(ranks):.2f}"
                  f"  p(top-1 > chance) {binom_tail(top1, len(ranks), 1 / 6):.2g}")


if __name__ == "__main__":
    main()
