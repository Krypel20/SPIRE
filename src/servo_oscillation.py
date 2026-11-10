#!/usr/bin/env python3
"""
Is the fast yaw-rate swing of the camera during servo-corrected bursts an
oscillation, and where does it come from?

For every cycle of the given sessions the camera and platform gyro z traces
(c###_imu.csv) are cut to the burst (first frame to last frame + exposure),
resampled to a uniform grid, detrended (straight line removed, so a steady
rotation does not count) and turned into a power spectrum (Welch).

Printed per mode (UNCORRECTED/HELD vs CORRECTED/ACTIVE):
  - RMS of the detrended camera rate (how much it wobbles around the trend)
  - dominant frequency of the camera wobble
  - the same for the platform rate
Reading:
  - CORRECTED camera wobble >> UNCORRECTED camera wobble at the same
    platform wobble: the wobble is added by the servo loop or the servo.
  - CORRECTED platform wobble at the same frequency: reaction torque of the
    servo on the light bench platform (bench artefact).
  - a sharp peak well above the control-loop bandwidth points at the servo
    mechanics (gear backlash, internal controller) rather than at the
    software loop, which a command filter would not remove.

Writes servo_oscillation.png (mean spectra) next to the first session.

Usage (from deblur_data):
  python3 ../SPIRE/src/servo_oscillation.py session_20261107_14* session_20261108_121*
"""

import argparse
import csv
import json
import os
import sys
from statistics import median

import numpy as np
from scipy.signal import welch

FS = 400.0          # uniform resampling rate [Hz]
TRUE = ("1", "True", "true")


def load_trace(path):
    cam, plat = ([], []), ([], [])
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            dst = cam if r.get("source") == "camera" else plat if r.get("source") == "platform" else None
            if dst is not None:
                dst[0].append(int(r["mono_ns"]))
                dst[1].append(float(r["gyro_z_dps"]))
    out = []
    for t, g in (cam, plat):
        o = np.argsort(t)
        out.append((np.asarray(t, dtype=np.int64)[o], np.asarray(g)[o]))
    return out


def boot_offset(session, cycle):
    try:
        with open(os.path.join(session, f"c{cycle:03d}_meta.json")) as fh:
            for f in json.load(fh).get("frames", []):
                if f.get("sensor_timestamp_ns") and f.get("exp_mono_ns"):
                    return f["sensor_timestamp_ns"] - f["exp_mono_ns"]
    except (OSError, ValueError):
        pass
    return 0


def segment(t, g, a, b):
    if len(t) < 10 or a < t[0] or b > t[-1]:
        return None
    ts = np.arange(a, b, 1e9 / FS)
    x = np.interp(ts, t, g)
    n = np.arange(len(x))
    return x - np.polyval(np.polyfit(n, x, 1), n)      # remove trend


def spectrum(x):
    f, p = welch(x, fs=FS, nperseg=min(256, len(x)), detrend=False)
    return f, p


def main():
    ap = argparse.ArgumentParser(description="Camera/platform yaw wobble during bursts")
    ap.add_argument("sessions", nargs="+")
    args = ap.parse_args()

    groups = {}          # (mode, which) -> list of (rms, peak_hz, f, p)
    for s in sorted(args.sessions):
        mpath = os.path.join(s, "manifest.csv")
        if not os.path.exists(mpath):
            continue
        with open(mpath, newline="") as fh:
            rows = list(csv.DictReader(fh))
        cycles = {}
        for r in rows:
            cycles.setdefault(int(r["cycle_id"]), []).append(r)
        for cyc, fr in cycles.items():
            r0 = fr[0]
            if r0["cycle_valid"] not in TRUE or r0["mode"] == "STATIC":
                continue
            mode = f'{r0["mode"]}/{r0["effective_status"]}'
            ip = os.path.join(s, f"c{cyc:03d}_imu.csv")
            if not os.path.exists(ip):
                continue
            off = boot_offset(s, cyc)
            ts = sorted(int(r["sensor_timestamp_ns"]) - off for r in fr)
            exp = int(float(r0["exposure_time_us"]) * 1000)
            a, b = ts[0], ts[-1] + exp
            for which, (t, g) in zip(("camera", "platform"), load_trace(ip)):
                x = segment(t, g, a, b)
                if x is None or len(x) < 64:
                    continue
                f, p = spectrum(x)
                k = 1 + int(np.argmax(p[1:]))
                groups.setdefault((mode, which), []).append(
                    (float(np.sqrt(np.mean(x ** 2))), float(f[k]), f, p))

    if not groups:
        sys.exit("no cycles with c###_imu.csv found")
    print("detrended yaw rate during bursts (valid cycles)")
    print(f"{'mode':22s} {'sensor':9s} {'n':>4s} {'RMS median':>11s} {'peak Hz median':>15s}")
    for (mode, which), v in sorted(groups.items()):
        print(f"{mode:22s} {which:9s} {len(v):4d} {median(x[0] for x in v):9.2f} °/s"
              f" {median(x[1] for x in v):13.1f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.3, 3.6))
    styles = {("CORRECTED/ACTIVE", "camera"): ("#2a78d6", "-", "Camera, servo active"),
              ("CORRECTED/ACTIVE", "platform"): ("#2a78d6", "--", "Platform, servo active"),
              ("UNCORRECTED/HELD", "camera"): ("#eb6834", "-", "Camera, servo held")}
    for key, (col, ls, lab) in styles.items():
        v = groups.get(key)
        if not v:
            continue
        n = min(len(x[3]) for x in v)
        p = np.mean([x[3][:n] for x in v], axis=0)
        ax.semilogy(v[0][2][:n], p, color=col, linestyle=ls, linewidth=1.6,
                    label=f"{lab} (n={len(v)})")
    ax.set_xlim(0, 100)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Yaw rate PSD [(deg/s)²/Hz]")
    ax.grid(True, color="#e6e5e1", linewidth=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False)
    fig.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(sorted(args.sessions)[0])),
                       "servo_oscillation.png")
    fig.savefig(out, dpi=200)
    fig.savefig(out.replace(".png", ".pdf"))
    print(f"Written {out} (+ .pdf)")


if __name__ == "__main__":
    main()
