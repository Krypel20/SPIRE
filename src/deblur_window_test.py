#!/usr/bin/env python3
"""
Which exposure window does the camera timestamp describe? Image test.

For every burst winner of a session the yaw angle swept during the exposure
is computed from the camera gyro trace for both readings of libcamera's
SensorTimestamp:
  window A: [ts - exposure, ts]
  window B: [ts, ts + exposure]
The frames where the two windows disagree most are deblurred with the gyro
kernel of each window (rot_deblur.py --kernel gyro --window A / B) and put
side by side (deblur_compare.py). The window whose deblur leaves no ghost
edges and no leftover smear is the right one; on frames where A and B are
similar the test cannot tell them apart, which is why only the most
different frames are used.

Usage (from deblur_data, scripts in ../SPIRE/src):
  python3 ../SPIRE/src/deblur_window_test.py --session session_20261108_121925 [-n 4]
"""

import argparse
import csv
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rot_deblur as rd  # noqa: E402

R_CORNER = math.hypot(4056 / 2, 3040 / 2)


def main():
    ap = argparse.ArgumentParser(description="Exposure-window test by deblurring")
    ap.add_argument("--session", required=True)
    ap.add_argument("-n", type=int, default=4, help="number of frames to test")
    ap.add_argument("--min-px", type=float, default=3.0,
                    help="larger of the two corner blurs must be at least this")
    ap.add_argument("--list-only", action="store_true")
    args = ap.parse_args()

    with open(os.path.join(args.session, "manifest.csv"), newline="") as fh:
        rows = [r for r in csv.DictReader(fh) if r["winner"] in ("1", "True", "true")]
    traces, cand = {}, []
    for r in rows:
        cyc = int(r["cycle_id"])
        if cyc not in traces:
            ip = os.path.join(args.session, f"c{cyc:03d}_imu.csv")
            traces[cyc] = ((rd.load_gz(ip) if os.path.exists(ip) else None),
                           rd.boot_offset(args.session, cyc))
        tr, off = traces[cyc]
        if tr is None:
            continue
        ts = int(r["sensor_timestamp_ns"]) - off
        exp = int(float(r["exposure_time_us"]) * 1000)
        ext = {}
        for w, a in (("A", ts - exp), ("B", ts)):
            th = rd.angle_trace(tr[0], tr[1], a, a + exp)
            ext[w] = None if th is None else float(th.max() - th.min())
        if None in ext.values():
            continue
        pa, pb = (math.radians(ext[k]) * R_CORNER for k in ("A", "B"))
        if max(pa, pb) < args.min_px:
            continue
        ratio = max(pa, pb) / max(min(pa, pb), 0.05)
        cand.append((ratio, r["file"], pa, pb, r["effective_status"]))
    cand.sort(reverse=True)
    print(f"{len(cand)} winners with corner blur >= {args.min_px} px; most different A vs B:")
    for ratio, f, pa, pb, st in cand[:max(args.n, 10)]:
        print(f"  {f:26s} {st:6s} corner blur A {pa:5.1f} px  B {pb:5.1f} px  ratio {ratio:4.1f}")
    if args.list_only:
        return
    for _ratio, f, _pa, _pb, _st in cand[:args.n]:
        for w in ("A", "B"):
            subprocess.run([sys.executable, os.path.join(HERE, "rot_deblur.py"),
                            "--session", args.session, "--frame", f,
                            "--kernel", "gyro", "--window", w], check=True)
        subprocess.run([sys.executable, os.path.join(HERE, "deblur_compare.py"),
                        "--session", args.session, "--frame", f], check=True)


if __name__ == "__main__":
    main()
