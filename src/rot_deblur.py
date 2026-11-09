#!/usr/bin/env python3
"""
SPIRE rotational (yaw) deblur.

A nadir camera that yaws turns about its optical axis, so the blur is a
rotation of the image during the exposure: zero at the rotation centre,
growing as r * dtheta with distance r from it. That blur is spatially
varying in Cartesian coordinates but becomes a uniform 1-D blur along the
angle axis after a Cartesian -> polar transform about the rotation centre,
where it is removed by 1-D Wiener deconvolution and transformed back. The
blur angle dtheta comes from the camera gyro, not from the image.

Radial lens distortion keeps circles about the principal point circular, so
the method still applies to the 6 mm lens as long as the rotation centre is
the principal point.

Built to run on the flight computer (RPi 5) between bursts: single
precision, processed in radial / row blocks so peak memory stays well under
1 GB for a 12 MP frame, and frames whose predicted corner blur is below
--min-blur-px are copied unchanged instead of processed.

The yaw angle per frame is read from the session manifest: blur_yaw_px is
the yaw blur at score_radius * R_corner (default 0.25 * 2534 px), so
angle = blur_yaw_px / (score_radius * R_corner) rad.

Usage:
  python3 rot_deblur.py --session DIR --frame c002_UNCORRECTED_f1.jpg
  python3 rot_deblur.py --session DIR --winners
  python3 rot_deblur.py IMAGE --angle-deg 1.22 [--snr 100] [-o out.jpg]
  python3 rot_deblur.py IMAGE --angle-deg 1.22 --synthetic   # self-test
Output for --session goes to DIR/_deblur/<frame>_deblur.jpg. Each run
prints the processing time and the peak memory of the process.
"""

import argparse
import csv
import json
import math
import os
import resource
import time

import numpy as np
from PIL import Image
from scipy import fft as sfft
from scipy.ndimage import map_coordinates

SENSOR_PX = (4056, 3040)
R_BLOCK = 128      # polar rows (radii) per block
Y_BLOCK = 256      # output image rows per block


def box_otf(n_t, length):
    """rfft of a centred box of `length` samples (fractional ends allowed)."""
    k = np.zeros(n_t, dtype=np.float64)
    full = max(1, int(math.floor(length)))
    frac = max(0.0, length - full)
    idx = np.arange(full) - (full - 1) / 2.0
    for i in idx:
        k[int(round(i)) % n_t] += 1.0
    if frac > 0:
        k[int(round(idx[0] - 1)) % n_t] += frac / 2
        k[int(round(idx[-1] + 1)) % n_t] += frac / 2
    k /= k.sum()
    return np.fft.rfft(k)


class PolarFilter:
    """Polar resampling geometry for one image size and rotation centre."""

    def __init__(self, h, w, cx=None, cy=None):
        self.h, self.w = h, w
        self.cx = (w - 1) / 2 if cx is None else cx
        self.cy = (h - 1) / 2 if cy is None else cy
        r_max = math.hypot(max(self.cx, w - self.cx), max(self.cy, h - self.cy))
        self.n_r = int(math.ceil(r_max)) + 1
        self.n_t = int(2 ** math.ceil(math.log2(2 * math.pi * r_max)))
        t = np.arange(self.n_t, dtype=np.float64) * 2 * math.pi / self.n_t
        self.cos_t = np.cos(t).astype(np.float32)[None, :]
        self.sin_t = np.sin(t).astype(np.float32)[None, :]

    def samples(self, angle_deg):
        return math.radians(angle_deg) / (2 * math.pi) * self.n_t

    def apply(self, ch, F, fill_invalid=True):
        """Filter one channel along the angle axis with frequency response F."""
        h, w, n_t = self.h, self.w, self.n_t
        P = np.empty((self.n_r, n_t + 1), dtype=np.float32)
        for r0 in range(0, self.n_r, R_BLOCK):
            r = np.arange(r0, min(r0 + R_BLOCK, self.n_r), dtype=np.float32)[:, None]
            x = self.cx + r * self.cos_t
            y = self.cy + r * self.sin_t
            p = map_coordinates(ch, [y, x], order=1, mode="nearest", output=np.float32)
            if fill_invalid:
                valid = (x >= 0) & (x <= w - 1) & (y >= 0) & (y <= h - 1)
                for i in np.where(~valid.all(axis=1))[0]:
                    v = valid[i]
                    p[i, ~v] = p[i, v].mean() if v.any() else 0.0
            del x, y
            q = sfft.irfft(sfft.rfft(p, axis=1, workers=-1) * F[None, :],
                           n=n_t, axis=1, workers=-1)
            P[r0:r0 + q.shape[0], :n_t] = q
        P[:, n_t] = P[:, 0]                      # periodic angle axis
        out = np.empty((h, w), dtype=np.float32)
        xs = np.arange(w, dtype=np.float32)[None, :] - self.cx
        scale = n_t / (2 * math.pi)
        for y0 in range(0, h, Y_BLOCK):
            ys = np.arange(y0, min(y0 + Y_BLOCK, h), dtype=np.float32)[:, None] - self.cy
            rr = np.hypot(xs, ys)
            tt = np.mod(np.arctan2(ys, xs), 2 * math.pi) * scale
            out[y0:y0 + ys.shape[0]] = map_coordinates(
                P, [rr, tt], order=1, mode="nearest", output=np.float32)
        return out


def deblur(img, angle_deg, snr=100.0, cx=None, cy=None, geom=None):
    """img: HxWx3 float32 in 0..1. Returns the deblurred image."""
    h, w = img.shape[:2]
    geom = geom or PolarFilter(h, w, cx, cy)
    H = box_otf(geom.n_t, geom.samples(angle_deg))
    W = (np.conj(H) / (np.abs(H) ** 2 + 1.0 / snr)).astype(np.complex64)
    out = np.empty_like(img)
    for c in range(img.shape[2]):
        out[:, :, c] = geom.apply(img[:, :, c], W)
    return np.clip(out, 0, 1, out=out)


def blur(img, angle_deg, geom):
    H = box_otf(geom.n_t, geom.samples(angle_deg)).astype(np.complex64)
    out = np.empty_like(img)
    for c in range(img.shape[2]):
        out[:, :, c] = geom.apply(img[:, :, c], H, fill_invalid=False)
    return np.clip(out, 0, 1, out=out)


def psnr(a, b):
    m = float(np.mean((a.astype(np.float64) - b) ** 2))
    return 10 * math.log10(1 / m) if m > 0 else float("inf")


def load(path):
    return np.asarray(Image.open(path).convert("RGB"), dtype=np.float32) / 255


def save(img, path):
    Image.fromarray((img * 255 + 0.5).astype(np.uint8)).save(path, quality=95)


def frames_from_session(session, frame=None, winners=False):
    """[(image path, yaw angle deg, manifest row)] from a session folder."""
    with open(os.path.join(session, "session.json")) as fh:
        bp = json.load(fh).get("bench_params", {})
    w, h = bp.get("sensor_px", SENSOR_PX)
    radius = bp.get("score_radius", 0.25) * math.hypot(w / 2, h / 2)
    out = []
    with open(os.path.join(session, "manifest.csv"), newline="") as fh:
        for r in csv.DictReader(fh):
            if frame and r["file"] != frame:
                continue
            if winners and r["winner"] not in ("1", "True", "true"):
                continue
            if r.get("blur_yaw_px") in (None, ""):
                continue
            angle = math.degrees(float(r["blur_yaw_px"]) / radius)
            out.append((os.path.join(session, r["file"]), angle, r))
    return out


def peak_mb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024


def main():
    ap = argparse.ArgumentParser(description="Rotational deblur (polar Wiener)")
    ap.add_argument("image", nargs="?")
    ap.add_argument("--angle-deg", type=float,
                    help="yaw angle swept during the exposure (from the camera gyro)")
    ap.add_argument("--session", help="session folder (angle from manifest.csv)")
    ap.add_argument("--frame", help="with --session: one frame file name")
    ap.add_argument("--winners", action="store_true",
                    help="with --session: every burst winner")
    ap.add_argument("--snr", type=float, default=100.0)
    ap.add_argument("--min-blur-px", type=float, default=0.5,
                    help="skip frames whose yaw blur at the frame corner is below this")
    ap.add_argument("--cx", type=float)
    ap.add_argument("--cy", type=float)
    ap.add_argument("--synthetic", action="store_true",
                    help="blur the (sharp) input first, then deblur and report PSNR")
    ap.add_argument("-o", "--output", default="deblurred.jpg")
    args = ap.parse_args()

    if args.session:
        jobs = frames_from_session(args.session, args.frame, args.winners)
        if not jobs:
            ap.error("no matching frames in manifest.csv (use --frame NAME or --winners)")
        out_dir = os.path.join(args.session, "_deblur")
        os.makedirs(out_dir, exist_ok=True)
        geom = None
        for path, angle, r in jobs:
            if not os.path.exists(path):
                print(f"missing {path}, skipped")
                continue
            t0 = time.monotonic()
            img = load(path)
            h, w = img.shape[:2]
            if geom is None or (geom.h, geom.w) != (h, w):
                geom = PolarFilter(h, w, args.cx, args.cy)
            corner_px = math.radians(angle) * math.hypot(w / 2, h / 2)
            name = os.path.splitext(os.path.basename(path))[0] + "_deblur.jpg"
            if corner_px < args.min_blur_px:
                save(img, os.path.join(out_dir, name))
                print(f"{os.path.basename(path)}: yaw {angle:.3f} deg, corner blur "
                      f"{corner_px:.2f} px < {args.min_blur_px}, copied")
                continue
            save(deblur(img, angle, args.snr, geom=geom), os.path.join(out_dir, name))
            print(f"{os.path.basename(path)}: yaw {angle:.3f} deg, corner blur "
                  f"{corner_px:.1f} px -> _deblur/{name}  "
                  f"{time.monotonic() - t0:.1f} s, peak {peak_mb():.0f} MB")
        return

    if not args.image or args.angle_deg is None:
        ap.error("give IMAGE --angle-deg, or --session DIR with --frame/--winners")
    t0 = time.monotonic()
    img = load(args.image)
    geom = PolarFilter(img.shape[0], img.shape[1], args.cx, args.cy)
    print(f"blur {args.angle_deg:.3f} deg = {geom.samples(args.angle_deg):.1f} "
          f"samples on the angle axis")
    if args.synthetic:
        b = blur(img, args.angle_deg, geom)
        out = deblur(b, args.angle_deg, args.snr, geom=geom)
        h, w = img.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        inner = np.hypot(xx - geom.cx, yy - geom.cy) < min(h, w) / 2 - 2
        print(f"PSNR blurred  {psnr(img[inner], b[inner]):.2f} dB (inscribed circle)")
        print(f"PSNR restored {psnr(img[inner], out[inner]):.2f} dB")
        save(b, args.output.replace(".jpg", "_blurred.jpg"))
    else:
        out = deblur(img, args.angle_deg, args.snr, geom=geom)
    save(out, args.output)
    print(f"Written {args.output}  {time.monotonic() - t0:.1f} s, peak {peak_mb():.0f} MB")


if __name__ == "__main__":
    main()