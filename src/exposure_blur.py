#!/usr/bin/env python3
"""
SPIRE exposure blur: rotation of the camera during each exposure, from the
camera gyro trace, and the resulting blur at the frame corner.

For long exposures (e.g. 66.7 ms on 2026-10-14) one gyro sample at the
exposure timestamp is not enough: the rate changes during the exposure, so
the blur angle is the integral of camera gyro_z over the exposure window.

The exact window is uncertain: libcamera documents SensorTimestamp as "the
time when the first row of the image sensor active array is exposed", which
can be read as the start or the end of that row's exposure. Both windows are
computed; comparing them with the blur measured in the images (Test C)
decides which is right.

  window A (ts_end):   [ts - exposure, ts]
  window B (ts_start): [ts, ts + exposure]

With --readout-ms the later rows are exposed later by up to that time
(rolling shutter); the angle is then also given for the middle and last row.

Rotation blur at radius r is theta * r (pixels); the corner of the full
4056 x 3040 frame is r = 2534 px from the centre.

Usage:
  python3 exposure_blur.py SESSION_DIR [SESSION_DIR ...]
  python3 exposure_blur.py deblur_data/session_20261014_1* --readout-ms 80
Writes <session>/exposure_blur.csv and prints a short summary with a sharp
reference frame and candidate blurred frames per blur range.
"""

import argparse
import bisect
import csv
import glob
import json
import math
import os
import sys

R_CORNER_PX = math.hypot(4056 / 2, 3040 / 2)       # 2534 px
BLUR_BINS = [(2, 5), (5, 15), (15, 40)]


def load_camera_trace(path):
    """Sorted (mono_ns, gyro_z_dps) samples of the camera IMU."""
    t, g = [], []
    with open(path, newline="") as fh:
        rd = csv.DictReader(fh)
        need = {"source", "mono_ns", "gyro_z_dps"}
        if not need <= set(rd.fieldnames or []):
            print(f"  {path}: unexpected columns {rd.fieldnames}, need {sorted(need)}")
            return [], []
        for r in rd:
            if r["source"] == "camera":
                t.append(int(r["mono_ns"]))
                g.append(float(r["gyro_z_dps"]))
    order = sorted(range(len(t)), key=t.__getitem__)
    return [t[i] for i in order], [g[i] for i in order]


def rate_at(t, g, x):
    """Linear interpolation of the gyro rate at time x (ns)."""
    i = bisect.bisect_left(t, x)
    if i <= 0:
        return g[0]
    if i >= len(t):
        return g[-1]
    t0, t1 = t[i - 1], t[i]
    return g[i - 1] + (g[i] - g[i - 1]) * (x - t0) / (t1 - t0) if t1 > t0 else g[i]


def integrate(t, g, a, b):
    """Integral of the rate over [a, b] in degrees, trapezoid on the samples
    inside the window plus interpolated end points. None if the trace does
    not cover the window (or samples are too sparse)."""
    if not t or a < t[0] or b > t[-1]:
        return None
    pts = [a] + [x for x in t if a < x < b] + [b]
    if len(pts) < 3 and (b - a) > 20e6:              # < 1 sample in > 20 ms
        return None
    vals = [rate_at(t, g, x) for x in pts]
    s = 0.0
    for i in range(1, len(pts)):
        s += 0.5 * (vals[i] + vals[i - 1]) * (pts[i] - pts[i - 1])
    return s / 1e9                                    # deg/s * s = deg


def boot_to_mono_offset(session, cycle):
    """BOOTTIME minus MONOTONIC from c###_meta.json if stored; else 0
    (the Pi never suspends, so the two clocks differ by microseconds)."""
    p = os.path.join(session, f"c{cycle:03d}_meta.json")
    try:
        with open(p) as fh:
            meta = json.load(fh)
        for f in meta.get("frames", []):
            if f.get("sensor_timestamp_ns") and f.get("exp_mono_ns"):
                return f["sensor_timestamp_ns"] - f["exp_mono_ns"]
    except (OSError, ValueError):
        pass
    return 0


def px(theta_deg):
    return None if theta_deg is None else abs(math.radians(theta_deg)) * R_CORNER_PX


def fmt(v, f=".2f"):
    return "" if v is None else format(v, f)


def analyse(session, readout_ms):
    mpath = os.path.join(session, "manifest.csv")
    if not os.path.exists(mpath):
        print(f"{session}: no manifest.csv, skipped")
        return []
    with open(mpath, newline="") as fh:
        rows = list(csv.DictReader(fh))

    traces, offsets, out = {}, {}, []
    for r in rows:
        cyc = int(r["cycle_id"])
        if cyc not in traces:
            ip = os.path.join(session, f"c{cyc:03d}_imu.csv")
            traces[cyc] = load_camera_trace(ip) if os.path.exists(ip) else ([], [])
            offsets[cyc] = boot_to_mono_offset(session, cyc)
        t, g = traces[cyc]
        try:
            ts = int(r["sensor_timestamp_ns"]) - offsets[cyc]
            exp = int(float(r["exposure_time_us"]) * 1000)
        except (KeyError, ValueError):
            continue
        th_a = integrate(t, g, ts - exp, ts)
        th_b = integrate(t, g, ts, ts + exp)
        rec = {
            "file": r["file"], "cycle": cyc, "winner": r.get("winner", ""),
            "exposure_ms": exp / 1e6, "cam_gz_at_ts": r.get("cam_gz", ""),
            "theta_a_deg": th_a, "theta_b_deg": th_b,
            "blur_a_px": px(th_a), "blur_b_px": px(th_b),
        }
        if readout_ms:
            ro = int(readout_ms * 1e6)
            for name, shift in (("mid", ro // 2), ("last", ro)):
                rec[f"theta_a_{name}_deg"] = integrate(t, g, ts - exp + shift, ts + shift)
                rec[f"theta_b_{name}_deg"] = integrate(t, g, ts + shift, ts + exp + shift)
        out.append(rec)

    if out:
        fields = list(out[0].keys())
        with open(os.path.join(session, "exposure_blur.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            for rec in out:
                w.writerow({k: (fmt(v, ".4f") if isinstance(v, float) else v)
                            for k, v in rec.items()})
    return [dict(rec, session=os.path.basename(session)) for rec in out]


def main():
    ap = argparse.ArgumentParser(description="SPIRE exposure blur from gyro")
    ap.add_argument("sessions", nargs="+", help="Session folders (globs allowed)")
    ap.add_argument("--readout-ms", type=float, default=0.0,
                    help="Rolling-shutter readout time; adds mid and last row angles")
    args = ap.parse_args()

    sessions = sorted({p for s in args.sessions for p in glob.glob(s)
                       if os.path.isdir(p)})
    frames = []
    for s in sessions:
        fr = analyse(s, args.readout_ms)
        n_ok = sum(f["theta_a_deg"] is not None for f in fr)
        print(f"{os.path.basename(s)}: {len(fr)} frames, {n_ok} with gyro coverage "
              f"-> {os.path.join(s, 'exposure_blur.csv')}")
        frames += fr

    ok = [f for f in frames if f["blur_a_px"] is not None and f["blur_b_px"] is not None]
    if not ok:
        sys.exit("No frame has gyro data covering its exposure.")

    worst = lambda f: max(f["blur_a_px"], f["blur_b_px"])
    print("\nSharpest frames (reference candidates; both windows small):")
    for f in sorted(ok, key=worst)[:5]:
        print(f"  {f['session']}/{f['file']}  A {f['blur_a_px']:.2f} px  "
              f"B {f['blur_b_px']:.2f} px")

    for lo, hi in BLUR_BINS:
        sel = [f for f in ok if lo <= f["blur_a_px"] < hi and lo <= f["blur_b_px"] < hi]
        print(f"\nBlurred {lo}-{hi} px in both windows: {len(sel)} frames")
        for f in sorted(sel, key=worst)[:5]:
            print(f"  {f['session']}/{f['file']}  A {f['blur_a_px']:.1f} px  "
                  f"B {f['blur_b_px']:.1f} px  winner={f['winner']}")

    diff = [abs(f["blur_a_px"] - f["blur_b_px"]) for f in ok]
    diff.sort()
    print(f"\nWindow A vs B: median difference {diff[len(diff) // 2]:.1f} px, "
          f"max {diff[-1]:.1f} px. Frames where they differ a lot are the ones "
          f"that tell the windows apart in Test C.")


if __name__ == "__main__":
    main()