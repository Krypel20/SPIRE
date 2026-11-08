#!/usr/bin/env python3
"""
SPIRE image-based check of the gyro blur model (Test B support).

Measures blur directly in the JPEGs, independently of the gyroscopes, with
the no-reference blur metric of Crete-Roffet et al. (2007): the image is
re-blurred with a 9-tap averaging filter and the loss of neighbouring-pixel
variation is compared. 0 = sharp, 1 = fully blurred. Already-blurred image
content changes little when re-blurred, so the metric depends less on the
scene than plain gradient energy.

  F. Crete-Roffet, T. Dolmiere, P. Ladret, M. Nicolas, "The blur effect:
  perception and estimation with a new no-reference perceptual blur metric",
  Proc. SPIE 6492, Human Vision and Electronic Imaging XII, 2007.

Crops are 4x4-binned first: tested on synthetic motion blur of 1-21 px with
sensor-like noise, the metric is then monotonic in blur length (without
binning, noise reverses it above ~9 px).

Measured per frame on a centre crop (yaw blur ~0 there, tilt blur only) and
on four crops on the image diagonals at 0.6 R (yaw blur large, away from the
vignetted corners).

Output: sharpness_frames.csv and a printed summary:
  - Spearman correlation gyro score_px vs image blur (does the model rank
    frames like the images do?)
  - winner image blur per rotation bin, UNCORRECTED vs CORRECTED, Mann-Whitney
  - within-burst rank of the gyro3 winner by image blur (chance: 1/6, 3.5)

Usage (PC / WSL, needs numpy and pillow):
  python3 src/sharpness_check.py DIR [DIR ...] -o sharpness_frames.csv
"""

import argparse
import csv
import math
import os
import sys
from multiprocessing import Pool
from statistics import mean, median

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_report import bin_label, bin_of, fnum, mann_whitney  # noqa: E402

CROP = 1024       # px, crop size at full resolution
BIN = 4           # average 4x4 pixels before the metric: suppresses sensor
                  # noise and keeps 1-20 px blur inside the 9-tap filter range
DIAG_FRAC = 0.6
TRUE = ("1", "True", "true")


def blur_metric(g):
    """Crete-Roffet blur (0 sharp .. 1 blurred) of a 2-D float array."""
    def smooth(a, axis):
        k = 9
        c = np.cumsum(np.pad(a, [(k // 2, k // 2) if ax == axis else (0, 0)
                                 for ax in range(2)], mode="edge"), axis=axis)
        c = np.concatenate([np.zeros_like(c.take([0], axis=axis)), c], axis=axis)
        n = a.shape[axis]
        return (c.take(range(k, k + n), axis=axis) - c.take(range(0, n), axis=axis)) / k

    out = []
    for axis in (0, 1):
        b = smooth(g, axis)
        d_f = np.abs(np.diff(g, axis=axis))
        d_b = np.abs(np.diff(b, axis=axis))
        s_f = d_f.sum()
        s_v = np.maximum(0, d_f - d_b).sum()
        out.append((s_f - s_v) / s_f if s_f > 0 else 1.0)
    return max(out)


def crops(w, h):
    cx, cy = w / 2, h / 2
    boxes = {"center": (cx, cy)}
    dx, dy = DIAG_FRAC * w / 2, DIAG_FRAC * h / 2
    for name, sx, sy in (("tl", -1, -1), ("tr", 1, -1), ("bl", -1, 1), ("br", 1, 1)):
        boxes[name] = (cx + sx * dx, cy + sy * dy)
    return {k: (int(x - CROP / 2), int(y - CROP / 2), int(x + CROP / 2), int(y + CROP / 2))
            for k, (x, y) in boxes.items()}


def measure(path):
    try:
        im = Image.open(path)
        im.load()
    except (OSError, FileNotFoundError):
        return None
    res = {}
    for k, box in crops(*im.size).items():
        g = np.asarray(im.crop(box).convert("L"), dtype=np.float32)
        h, w = (g.shape[0] // BIN) * BIN, (g.shape[1] // BIN) * BIN
        g = g[:h, :w].reshape(h // BIN, BIN, w // BIN, BIN).mean(axis=(1, 3))
        res[k] = blur_metric(g)
    res["diag"] = mean(res[k] for k in ("tl", "tr", "bl", "br"))
    return res


def spearman(x, y):
    def rank(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and v[order[j + 1]] == v[order[i]]:
                j += 1
            for k in range(i, j + 1):
                r[order[k]] = (i + j) / 2 + 1
            i = j + 1
        return r
    rx, ry = rank(x), rank(y)
    mx, my = mean(rx), mean(ry)
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    den = math.sqrt(sum((a - mx) ** 2 for a in rx) * sum((b - my) ** 2 for b in ry))
    return num / den if den else float("nan")


def main():
    ap = argparse.ArgumentParser(description="Image-based blur check for Test B")
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("-o", "--output", default="sharpness_frames.csv")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()

    frames = []
    for s in sorted(args.sessions):
        name = os.path.basename(os.path.normpath(s))
        with open(os.path.join(s, "manifest.csv"), newline="") as fh:
            for r in csv.DictReader(fh):
                r["session"] = name
                r["path"] = os.path.join(s, r["file"])
                frames.append(r)
    print(f"{len(frames)} frames, measuring with {args.jobs} processes ...")
    with Pool(args.jobs) as pool:
        res = pool.map(measure, [f["path"] for f in frames], chunksize=4)
    missing = sum(r is None for r in res)
    if missing:
        print(f"WARNING: {missing} images missing or unreadable, skipped")

    rows = []
    for f, m in zip(frames, res):
        if m is None:
            continue
        rows.append({
            "session": f["session"], "cycle": int(f["cycle_id"]),
            "frame": int(f["frame_index"]), "mode": f["mode"],
            "status": f["effective_status"], "valid": f["cycle_valid"] in TRUE or f["mode"] == "STATIC",
            "rot_dps": fnum(f["rotation_speed_dps"]) or 0.0,
            "winner": f["winner"] in TRUE, "winner_gz": f.get("winner_gz") in TRUE,
            "score_px": fnum(f["score_px"]), "tilt_px": fnum(f["blur_tilt_px"]),
            "yaw_px": fnum(f["blur_yaw_px"]), "cam_gz": fnum(f["cam_gz"]),
            "blur_center": round(m["center"], 4), "blur_diag": round(m["diag"], 4),
            "file": f["file"]})
    with open(args.output, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"Written {args.output}\n")

    # 1. model vs image, all frames of valid rotation cycles
    rot = [r for r in rows if r["valid"] and r["mode"] != "STATIC" and r["score_px"] is not None]
    print("Spearman gyro score_px vs image blur (valid rotation frames):")
    for lab, sel in (("all", rot),
                     ("UNCORRECTED", [r for r in rot if r["mode"] == "UNCORRECTED"]),
                     ("CORRECTED", [r for r in rot if r["mode"] == "CORRECTED"])):
        print(f"  {lab:12s} n={len(sel):4d}  diag rho={spearman([r['score_px'] for r in sel], [r['blur_diag'] for r in sel]):.2f}"
              f"  centre rho={spearman([r['tilt_px'] for r in sel], [r['blur_center'] for r in sel]):.2f} (vs tilt_px)")

    # 2. winners per bin
    print("\nWinner image blur (diagonal crops), median per bin:")
    st = [r["blur_diag"] for r in rows if r["mode"] == "STATIC" and r["winner"]]
    if st:
        print(f"  STATIC reference: {median(st):.3f} (n={len(st)})")
    for lo_hi in ((5, 10), (10, 20), (20, 40), (40, 70)):
        lab = bin_label(lo_hi)
        u = [r["blur_diag"] for r in rot if r["winner"] and r["mode"] == "UNCORRECTED"
             and bin_label(bin_of(r["rot_dps"])) == lab]
        c = [r["blur_diag"] for r in rot if r["winner"] and r["mode"] == "CORRECTED"
             and r["status"] == "ACTIVE" and bin_label(bin_of(r["rot_dps"])) == lab]
        if not u or not c:
            continue
        U, p = mann_whitney(c, u)
        ptxt = f"p={p:.2g}" if p is not None else "n<5"
        print(f"  {lab:6s} UNCORRECTED {median(u):.3f} (n={len(u)})  CORRECTED {median(c):.3f} (n={len(c)})  {ptxt}")

    # 3. within-burst rank of the chosen frame
    print("\nWithin-burst rank by image blur (1 = sharpest of 6; chance top-1 17%, mean rank 3.5):")
    cyc = {}
    for r in rot:
        cyc.setdefault((r["session"], r["cycle"]), []).append(r)
    for mode in ("UNCORRECTED", "CORRECTED"):
        for key, name in (("winner", "gyro3 winner"), ("winner_gz", "gz-rule frame")):
            ranks = []
            for fr in cyc.values():
                if fr[0]["mode"] != mode or len(fr) < 2:
                    continue
                order = sorted(fr, key=lambda r: r["blur_diag"])
                pick = [i for i, r in enumerate(order, 1) if r[key]]
                if pick:
                    ranks.append(pick[0])
            if ranks:
                print(f"  {mode:12s} {name:14s} n={len(ranks):3d}  top-1 {sum(x == 1 for x in ranks) / len(ranks):.0%}"
                      f"  top-2 {sum(x <= 2 for x in ranks) / len(ranks):.0%}  mean rank {mean(ranks):.2f}")


if __name__ == "__main__":
    main()
