#!/usr/bin/env python3
"""
SPIRE rotational (yaw) deblur, first prototype.

A nadir camera that yaws turns about its optical axis, so the blur is a
rotation of the image during the exposure: zero at the rotation centre,
growing as r * dtheta with distance r from it. That blur is spatially
varying in Cartesian coordinates but becomes a uniform 1-D blur along the
angle axis after a Cartesian -> polar transform about the rotation centre,
where it can be removed by ordinary 1-D deconvolution (Wiener here) and
transformed back. The blur angle dtheta comes from the camera gyro (yaw
angle swept during the exposure), not from the image.

Radial lens distortion keeps circles about the principal point circular, so
the polar method still applies to the fisheye-like 6 mm lens as long as the
rotation centre is the principal point.

Usage:
  python3 rot_deblur.py IMAGE --angle-deg 1.22 [--snr 100] [-o out.jpg]
  python3 rot_deblur.py IMAGE --angle-deg 1.22 --synthetic   # self-test
"""

import argparse
import math

import numpy as np
from PIL import Image
from scipy.ndimage import map_coordinates


def polar_grid(h, w, cx, cy, n_t=None):
    r_max = math.hypot(max(cx, w - cx), max(cy, h - cy))
    n_r = int(math.ceil(r_max)) + 1
    if n_t is None:
        n_t = int(2 ** math.ceil(math.log2(2 * math.pi * r_max)))
    r = np.arange(n_r, dtype=np.float64)
    t = np.arange(n_t, dtype=np.float64) * 2 * math.pi / n_t
    rr, tt = np.meshgrid(r, t, indexing="ij")
    return rr, tt, n_r, n_t


def to_polar(ch, cx, cy, rr, tt):
    x = cx + rr * np.cos(tt)
    y = cy + rr * np.sin(tt)
    h, w = ch.shape
    valid = (x >= 0) & (x <= w - 1) & (y >= 0) & (y <= h - 1)
    p = map_coordinates(ch, [y, x], order=1, mode="nearest")
    return p, valid


def to_cartesian(p, cx, cy, h, w, n_t):
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)
    r = np.hypot(xx - cx, yy - cy)
    t = np.mod(np.arctan2(yy - cy, xx - cx), 2 * math.pi) * n_t / (2 * math.pi)
    # angle axis is periodic: append the first column for interpolation
    pw = np.concatenate([p, p[:, :1]], axis=1)
    return map_coordinates(pw, [r, t], order=1, mode="nearest")


def box_otf(n_t, length):
    """Frequency response of a centred box of `length` samples (fractional ok)."""
    k = np.zeros(n_t)
    full = int(math.floor(length))
    frac = length - full
    idx = np.arange(full) - (full - 1) / 2.0
    for i in idx:
        k[int(round(i)) % n_t] += 1.0
    if frac > 0:  # split the fractional remainder over both ends
        k[int(round(idx[0] - 1)) % n_t] += frac / 2
        k[int(round(idx[-1] + 1)) % n_t] += frac / 2
    k /= k.sum()
    return np.fft.rfft(k)


def blur_rows(p, H):
    return np.fft.irfft(np.fft.rfft(p, axis=1) * H[None, :], n=p.shape[1], axis=1)


def wiener_rows(p, valid, H, snr):
    q = p.copy()
    # rows that leave the image: fill the missing arc with the row mean
    for i in np.where(~valid.all(axis=1))[0]:
        v = valid[i]
        q[i, ~v] = q[i, v].mean() if v.any() else 0.0
    G = np.fft.rfft(q, axis=1)
    W = np.conj(H) / (np.abs(H) ** 2 + 1.0 / snr)
    return np.fft.irfft(G * W[None, :], n=p.shape[1], axis=1)


def process(img, angle_deg, snr, cx=None, cy=None, synthetic=False):
    h, w = img.shape[:2]
    cx = (w - 1) / 2 if cx is None else cx
    cy = (h - 1) / 2 if cy is None else cy
    rr, tt, n_r, n_t = polar_grid(h, w, cx, cy)
    length = math.radians(angle_deg) / (2 * math.pi) * n_t
    H = box_otf(n_t, length)
    out = np.zeros_like(img)
    blurred = np.zeros_like(img)
    for c in range(img.shape[2]):
        p, valid = to_polar(img[:, :, c], cx, cy, rr, tt)
        if synthetic:
            p = blur_rows(p, H)
            blurred[:, :, c] = to_cartesian(p, cx, cy, h, w, n_t)
        d = wiener_rows(p, valid, H, snr)
        out[:, :, c] = to_cartesian(d, cx, cy, h, w, n_t)
    return np.clip(out, 0, 1), np.clip(blurred, 0, 1), length


def psnr(a, b):
    m = np.mean((a - b) ** 2)
    return 10 * math.log10(1 / m) if m > 0 else float("inf")


def main():
    ap = argparse.ArgumentParser(description="Rotational deblur (polar Wiener)")
    ap.add_argument("image")
    ap.add_argument("--angle-deg", type=float, required=True,
                    help="yaw angle swept during the exposure (from the camera gyro)")
    ap.add_argument("--snr", type=float, default=100.0)
    ap.add_argument("--cx", type=float)
    ap.add_argument("--cy", type=float)
    ap.add_argument("--synthetic", action="store_true",
                    help="blur the (sharp) input first, then deblur and report PSNR")
    ap.add_argument("-o", "--output", default="deblurred.jpg")
    args = ap.parse_args()

    img = np.asarray(Image.open(args.image).convert("RGB"), dtype=np.float64) / 255
    out, blurred, length = process(img, args.angle_deg, args.snr, args.cx, args.cy,
                                   args.synthetic)
    print(f"blur {args.angle_deg:.3f} deg = {length:.1f} samples on the angle axis")
    if args.synthetic:
        h, w = img.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        inner = np.hypot(xx - (w - 1) / 2, yy - (h - 1) / 2) < min(h, w) / 2 - 2
        print(f"PSNR blurred  {psnr(img[inner], blurred[inner]):.2f} dB (inscribed circle)")
        print(f"PSNR restored {psnr(img[inner], out[inner]):.2f} dB")
        Image.fromarray((blurred * 255).astype(np.uint8)).save(
            args.output.replace(".jpg", "_blurred.jpg"), quality=95)
    Image.fromarray((out * 255).astype(np.uint8)).save(args.output, quality=95)
    print(f"Written {args.output}")


if __name__ == "__main__":
    main()
