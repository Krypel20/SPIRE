#!/usr/bin/env python3
"""
Check the gyro traces that flight_capture.py saves with every kept photo.

For each spire_*_meta.json in a flight output folder:
  - the trace <photo>_imu.csv exists and covers every burst exposure,
  - the gyro3 score of every burst frame, recomputed from the trace with
    window B ([ts, ts+exp], as in flight_capture.exposure_motion_px), matches
    the logged burst_score_px (rounded to 0.01 px in the log),
  - the frame with the lowest recomputed score is the logged winner.
If all photos pass, the saved data are enough to deblur the kept frames
after the flight.

Usage (PC / WSL, from the folder that holds the flight output folder):
  python3 ../SPIRE/src/flight_trace_check.py flight_test_20261118_101500
"""

import argparse
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gyro3_window_check import load_trace, window_blur  # noqa: E402

PIXEL_UM = 1.55
SENSOR_PX = (4056, 3040)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", help="flight_capture output folder (-o)")
    ap.add_argument("--tol-px", type=float, default=0.006,
                    help="allowed |recomputed - logged| per frame (log is rounded to 0.01)")
    args = ap.parse_args()

    metas = sorted(glob.glob(os.path.join(args.folder, "spire_*_meta.json")))
    if not metas:
        sys.exit(f"no spire_*_meta.json in {args.folder}")
    r_px = math.hypot(SENSOR_PX[0] / 2, SENSOR_PX[1] / 2)
    n_ok = n_burst = 0
    worst = 0.0
    for mp in metas:
        name = os.path.basename(mp).replace("_meta.json", "")
        with open(mp) as fh:
            m = json.load(fh)
        if "burst_score_px" not in m:
            print(f"{name}: single shot, no burst (skipped)")
            continue
        n_burst += 1
        problems = []
        for key in ("boot_mono_offset_ns", "burst_sensor_timestamp_ns",
                    "burst_exposure_time_us", "imu_trace_file"):
            if m.get(key) in (None, []):
                problems.append(f"{key} missing")
        tr = None
        if not problems:
            tp = os.path.join(args.folder, m["imu_trace_file"])
            tr = load_trace(tp) if os.path.exists(tp) else None
            if tr is None:
                problems.append("trace file missing or without x/y columns")
        if problems:
            print(f"{name}: FAIL ({'; '.join(problems)})")
            continue
        f_px = m.get("lens_mm", 16.0) * 1e-3 / (PIXEL_UM * 1e-6)
        radius = m.get("score_radius_frac", 0.25) * r_px
        off = m["boot_mono_offset_ns"]
        rec, diffs = [], []
        for ts, exp_us, logged in zip(m["burst_sensor_timestamp_ns"],
                                      m["burst_exposure_time_us"],
                                      m["burst_score_px"]):
            if not ts or logged >= 999:
                rec.append(math.inf)
                continue
            a = ts - off
            w = window_blur(tr, a, a + int(exp_us * 1000), f_px, radius)
            if w is None:
                problems.append("exposure outside the trace")
                rec.append(math.inf)
                continue
            rec.append(sum(w))
            diffs.append(abs(sum(w) - logged))
        win = min(range(len(rec)), key=lambda i: rec[i])
        dmax = max(diffs) if diffs else float("nan")
        worst = max(worst, dmax if diffs else 0.0)
        if win != m.get("winner_index"):
            problems.append(f"winner recomputed f{win}, logged f{m.get('winner_index')}")
        if diffs and dmax > args.tol_px:
            problems.append(f"score differs by up to {dmax:.3f} px")
        n_samples = len(tr[0])
        if problems:
            print(f"{name}: FAIL ({'; '.join(problems)})")
        else:
            n_ok += 1
            print(f"{name}: OK  trace {n_samples} samples, winner f{win}, "
                  f"max score diff {dmax:.3f} px")
    print(f"\n{n_ok}/{n_burst} burst photos pass; worst score difference {worst:.3f} px")


if __name__ == "__main__":
    main()
