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

CORRECTED cycles are grouped by the d_alpha stored in session.json, so
sessions with different command filters can be compared in one run.
Also printed: RMS inside a frequency band (default 5-15 Hz, around the
coupled-oscillation peak).

Writes servo_oscillation.png (mean spectra) next to the first session,
or to --out.

Usage (from deblur_data):
  python3 ../SPIRE/src/servo_oscillation.py session_20261107_14* session_20261108_121*
  python3 ../SPIRE/src/servo_oscillation.py session_20261111_21* session_20261111_22* \\
      --rate 5 30 --no-saturated -o d_alpha_compare.png
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


def session_d_alpha(session):
    try:
        with open(os.path.join(session, "session.json")) as fh:
            v = json.load(fh).get("flight_params", {}).get("d_alpha")
        return None if v is None else float(v)
    except (OSError, ValueError):
        return None


def band_rms(f, p, lo, hi):
    m = (f >= lo) & (f <= hi)
    if m.sum() < 2:
        return float("nan")
    return float(np.sqrt(np.trapezoid(p[m], f[m]) if hasattr(np, "trapezoid") else np.trapz(p[m], f[m])))


def main():
    ap = argparse.ArgumentParser(description="Camera/platform yaw wobble during bursts")
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--rate", nargs=2, type=float, metavar=("LO", "HI"), default=None,
                    help="keep only cycles with pre-burst platform rate in [LO, HI] deg/s "
                         "(rate-match groups recorded with different rotation speeds)")
    ap.add_argument("--no-saturated", action="store_true",
                    help="drop cycles where the servo hit its +-135 deg limit in any frame")
    ap.add_argument("--band", nargs=2, type=float, metavar=("LO", "HI"), default=(5.0, 15.0),
                    help="frequency band for the band-limited RMS (default 5 15 Hz)")
    ap.add_argument("-o", "--out", default=None,
                    help="figure path (.png, a .pdf is written next to it); "
                         "default servo_oscillation.png next to the first session")
    args = ap.parse_args()

    groups = {}          # (group label, sensor) -> list of (rms, peak_hz, band_rms, f, p)
    skipped = {"rate": 0, "saturated": 0}
    for s in sorted(args.sessions):
        mpath = os.path.join(s, "manifest.csv")
        if not os.path.exists(mpath):
            continue
        da = session_d_alpha(s)
        with open(mpath, newline="") as fh:
            rows = list(csv.DictReader(fh))
        cycles = {}
        for r in rows:
            cycles.setdefault(int(r["cycle_id"]), []).append(r)
        for cyc, fr in cycles.items():
            r0 = fr[0]
            if r0["cycle_valid"] not in TRUE or r0["mode"] == "STATIC":
                continue
            if args.rate:
                try:
                    rot = abs(float(r0["rotation_speed_dps"]))
                except (KeyError, ValueError):
                    rot = None
                if rot is None or not (args.rate[0] <= rot <= args.rate[1]):
                    skipped["rate"] += 1
                    continue
            if args.no_saturated and any(r.get("servo_saturated") in TRUE for r in fr):
                skipped["saturated"] += 1
                continue
            mode = f'{r0["mode"]}/{r0["effective_status"]}'
            if r0["mode"] == "CORRECTED" and da is not None:
                mode += f" d_alpha={da:g}"
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
                    (float(np.sqrt(np.mean(x ** 2))), float(f[k]),
                     band_rms(f, p, *args.band), f, p))

    if not groups:
        sys.exit("no cycles with c###_imu.csv found")
    filt = []
    if args.rate:
        filt.append(f"pre-burst rate {args.rate[0]:g}-{args.rate[1]:g} deg/s ({skipped['rate']} cycles out)")
    if args.no_saturated:
        filt.append(f"no servo saturation ({skipped['saturated']} cycles out)")
    print("detrended yaw rate during bursts (valid cycles"
          + (", " + ", ".join(filt) if filt else "") + ")")
    blo, bhi = args.band
    print(f"{'group':34s} {'sensor':9s} {'n':>4s} {'RMS median':>11s} "
          f"{'RMS ' + f'{blo:g}-{bhi:g} Hz':>14s} {'peak Hz median':>15s}")
    for (mode, which), v in sorted(groups.items()):
        print(f"{mode:34s} {which:9s} {len(v):4d} {median(x[0] for x in v):9.2f} °/s"
              f" {median(x[2] for x in v):9.2f} °/s {median(x[1] for x in v):13.1f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    palette = ["#2a78d6", "#1a9e6a", "#8a4fd0", "#c2185b"]
    fig, ax = plt.subplots(figsize=(6.3, 3.6))
    corrected = sorted({m for m, _ in groups if m.startswith("CORRECTED")})
    plan = []
    for i, m in enumerate(corrected):
        col = palette[i % len(palette)]
        tag = m.split(" ", 1)[1] if " " in m else "servo active"
        plan.append(((m, "camera"), col, "-", f"Camera, {tag}"))
        plan.append(((m, "platform"), col, "--", f"Platform, {tag}"))
    plan.append((("UNCORRECTED/HELD", "camera"), "#eb6834", "-", "Camera, servo held"))
    for key, col, ls, lab in plan:
        v = groups.get(key)
        if not v:
            continue
        n = min(len(x[4]) for x in v)
        p = np.mean([x[4][:n] for x in v], axis=0)
        ax.semilogy(v[0][3][:n], p, color=col, linestyle=ls, linewidth=1.6,
                    label=f"{lab} (n={len(v)})")
    ax.set_xlim(0, 100)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Yaw rate PSD [(deg/s)²/Hz]")
    ax.grid(True, color="#e6e5e1", linewidth=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(sorted(args.sessions)[0])), "servo_oscillation.png")
    fig.savefig(out, dpi=200)
    fig.savefig(os.path.splitext(out)[0] + ".pdf")
    print(f"Written {out} (+ .pdf)")


if __name__ == "__main__":
    main()
