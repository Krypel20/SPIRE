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

Two details matter on real frames: the filter works in linear light (the
JPEG is gamma-encoded, the blur happened in linear light), and circles that
leave the image are bridged smoothly across the missing arc instead of being
padded with a constant, which otherwise rings as dark lines along the image
border.

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

Defaults (from the 07.11 / 08.11 bench sessions): gyro kernel, exposure
window B = [SensorTimestamp, + exposure] (won on every decidable servo-
corrected frame; window A over-deblurs with ghost edges), Wiener SNR 30
(higher values add grain and stripes on servo-corrected frames). A
rolling-shutter shift (--readout-ms 30 / 60) made no visible difference.

Synthetic test, SNR 100 (STATIC frame, blur added in linear light, inscribed circle,
cubic resampling): PSNR blurred -> restored 47.8 -> 50.3 dB at 0.1 deg,
36.5 -> 43.5 at 0.3 deg, 29.3 -> 35.5 at 0.6 deg, 23.1 -> 30.6 at 1.22 deg;
at 0.05 deg it costs ~2 dB (53.4 -> 51.0), hence --min-blur-px.

Defaults from the bench tests of 2026-11-10: gyro kernel; window B
(SensorTimestamp = start of exposure of the first row; won against A on
every decidable servo-corrected frame); readout 60 ms (window shifted by
30 ms to the middle row; removed most stripes in the lower corners);
SNR 30 (best of 30/50/100 in the synthetic test with sensor-like noise,
0.3 deg: 35.9 -> 38.0 dB); frames under 4 px corner blur are copied
unchanged (break-even of the noisy synthetic test lies between ~3 and
~6 px; c012, ~5 px, visibly improved).

Usage:
  python3 rot_deblur.py --session DIR --frame c002_UNCORRECTED_f1.jpg
  python3 rot_deblur.py --session DIR --winners
  python3 rot_deblur.py IMAGE --angle-deg 1.22 [--snr 100] [-o out.jpg]
  python3 rot_deblur.py IMAGE --angle-deg 1.22 --synthetic   # self-test
  python3 rot_deblur.py --session DIR --winners --kernel gyro [--window A]
Output for --session goes to DIR/_deblur/<frame>_deblur_gyroB_ro60_snr30.jpg (all
parameters in the name; _box_snrN for the box kernel). Each run
prints the processing time and the peak memory of the process.
"""

import argparse
import csv
import json
import math
import os
import resource
import shutil
import time

import numpy as np
from PIL import Image
from scipy import fft as sfft
from scipy.ndimage import map_coordinates, spline_filter

SENSOR_PX = (4056, 3040)
R_BLOCK = 128      # polar rows (radii) per block
Y_BLOCK = 256      # output image rows per block
GAMMA = 2.2        # sRGB JPEG -> approximately linear light
INTERP = 3         # spline order for the polar round trip (1 = faster, lossier)


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
        self.k = np.arange(self.n_t // 2 + 1, dtype=np.float32)

    def samples(self, angle_deg):
        return math.radians(angle_deg) / (2 * math.pi) * self.n_t

    def apply(self, ch, F, fill_invalid=True, band_limit=True):
        """Filter one channel along the angle axis with frequency response F."""
        h, w, n_t = self.h, self.w, self.n_t
        P = np.empty((self.n_r, n_t + 1), dtype=np.float32)
        # cubic spline interpolation: the polar round trip alone costs ~54 dB
        # PSNR with it vs ~42 dB with linear interpolation; prefilter once
        cs = (spline_filter(ch, order=INTERP, output=np.float32, mode="nearest")
              if INTERP > 1 else ch)
        for r0 in range(0, self.n_r, R_BLOCK):
            r = np.arange(r0, min(r0 + R_BLOCK, self.n_r), dtype=np.float32)[:, None]
            x = self.cx + r * self.cos_t
            y = self.cy + r * self.sin_t
            p = map_coordinates(cs, [y, x], order=INTERP, mode="nearest", output=np.float32,
                                prefilter=False)
            if fill_invalid:
                valid = (x >= 0) & (x <= w - 1) & (y >= 0) & (y <= h - 1)
                idx = np.arange(n_t)
                for i in np.where(~valid.all(axis=1))[0]:
                    v = valid[i]
                    if v.sum() < 2:
                        p[i] = p[i, v].mean() if v.any() else 0.0
                        continue
                    # bridge the missing arc linearly between its valid ends
                    # (periodic), so the row has no step for the filter to ring on
                    p[i, ~v] = np.interp(idx[~v], idx[v], p[i, v], period=n_t)
            del x, y
            if band_limit:
                # angular frequency k at radius r is k / (2 pi r) cycles/px:
                # above the image Nyquist (k > pi r) the polar row holds only
                # interpolation error, so it is passed through, not amplified
                Fb = np.where(self.k[None, :] <= np.pi * np.maximum(r, 1.0),
                              F[None, :], np.complex64(1.0))
            else:
                Fb = F[None, :]
            q = sfft.irfft(sfft.rfft(p, axis=1, workers=-1) * Fb,
                           n=n_t, axis=1, workers=-1)
            P[r0:r0 + q.shape[0], :n_t] = q
        del cs
        P[:, n_t] = P[:, 0]                      # periodic angle axis
        if INTERP > 1:
            P = spline_filter(P, order=INTERP, output=np.float32, mode="nearest")
        out = np.empty((h, w), dtype=np.float32)
        xs = np.arange(w, dtype=np.float32)[None, :] - self.cx
        scale = n_t / (2 * math.pi)
        for y0 in range(0, h, Y_BLOCK):
            ys = np.arange(y0, min(y0 + Y_BLOCK, h), dtype=np.float32)[:, None] - self.cy
            rr = np.hypot(xs, ys)
            tt = np.mod(np.arctan2(ys, xs), 2 * math.pi) * scale
            out[y0:y0 + ys.shape[0]] = map_coordinates(
                P, [rr, tt], order=INTERP, mode="nearest", output=np.float32, prefilter=False)
        return out


def deblur(img, angle_deg, snr=30.0, cx=None, cy=None, geom=None, linear=True, H=None,
           luma=False):
    """img: HxWx3 float32 in 0..1. Returns the deblurred image. H: optional
    OTF on the angle axis (e.g. from gyro_otf); default a box of angle_deg.
    luma=True: filter only the luminance (Rec. 709 weights, linear light) and add
    the recovered detail to all three channels; colour stays as captured. About
    3x faster, since the polar filter runs once instead of once per channel."""
    h, w = img.shape[:2]
    geom = geom or PolarFilter(h, w, cx, cy)
    if H is None:
        H = box_otf(geom.n_t, geom.samples(angle_deg))
    W = np.conj(H) / (np.abs(H) ** 2 + 1.0 / snr)
    W = (W / W[0].real).astype(np.complex64)   # unit DC gain: no brightness loss
    lin = np.power(img, GAMMA) if linear else img
    if luma:
        y = lin @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
        out = lin + (geom.apply(y, W) - y)[:, :, None]
    else:
        out = np.empty_like(img)
        for c in range(img.shape[2]):
            out[:, :, c] = geom.apply(lin[:, :, c], W)
    np.clip(out, 0, 1, out=out)
    return np.power(out, 1 / GAMMA, out=out) if linear else out


def blur(img, angle_deg, geom):
    """Synthetic rotational blur, applied in linear light like a real exposure."""
    H = box_otf(geom.n_t, geom.samples(angle_deg)).astype(np.complex64)
    lin = np.power(img, GAMMA)
    out = np.empty_like(img)
    for c in range(img.shape[2]):
        out[:, :, c] = geom.apply(lin[:, :, c], H, fill_invalid=False, band_limit=False)
    np.clip(out, 0, 1, out=out)
    return np.power(out, 1 / GAMMA, out=out)


def psnr(a, b):
    m = float(np.mean((a.astype(np.float64) - b) ** 2))
    return 10 * math.log10(1 / m) if m > 0 else float("inf")


def load(path):
    return np.asarray(Image.open(path).convert("RGB"), dtype=np.float32) / 255


def save(img, path):
    Image.fromarray((img * 255 + 0.5).astype(np.uint8)).save(path, quality=95)


def load_gz(path):
    """Camera gyro z trace (mono_ns, deg/s) from a c###_imu.csv."""
    t, g = [], []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("source") == "camera":
                t.append(int(r["mono_ns"]))
                g.append(float(r["gyro_z_dps"]))
    o = np.argsort(t)
    return np.asarray(t, dtype=np.int64)[o], np.asarray(g, dtype=np.float64)[o]


def boot_offset(session, cycle):
    """SensorTimestamp (BOOTTIME) minus CLOCK_MONOTONIC from c###_meta.json."""
    try:
        with open(os.path.join(session, f"c{cycle:03d}_meta.json")) as fh:
            for f in json.load(fh).get("frames", []):
                if f.get("sensor_timestamp_ns") and f.get("exp_mono_ns"):
                    return f["sensor_timestamp_ns"] - f["exp_mono_ns"]
    except (OSError, ValueError):
        pass
    return 0


def angle_trace(t, g, a, b, n=400):
    """Yaw angle (deg) over [a, b] ns, integrated from the gyro rate; None if
    the trace does not cover the window."""
    if len(t) < 2 or a < t[0] or b > t[-1]:
        return None
    ts = np.linspace(a, b, n)
    rate = np.interp(ts, t, g)
    dt = np.diff(ts) / 1e9
    return np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * dt)])


def gyro_otf(n_t, theta_deg):
    """OTF on the angle axis from the angle trajectory: the kernel is the time
    the camera spent at each angle, so a rate that changes during the
    exposure gives the matching non-uniform kernel instead of a box."""
    s = np.radians(theta_deg) / (2 * math.pi) * n_t
    s = s - 0.5 * (s.max() + s.min())
    k = np.zeros(n_t, dtype=np.float64)
    i0 = np.floor(s).astype(np.int64)
    f = s - i0
    np.add.at(k, i0 % n_t, 1 - f)
    np.add.at(k, (i0 + 1) % n_t, f)
    k /= k.sum()
    return np.fft.rfft(k)


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
    ap.add_argument("--snr", type=float, default=30.0,
                    help="Wiener signal-to-noise ratio: higher sharpens more and amplifies "
                         "noise/JPEG artefacts (30 chosen on servo-corrected frames)")
    ap.add_argument("--kernel", choices=("box", "gyro"), default="gyro",
                    help="box: uniform rate over the yaw angle from the manifest; "
                         "gyro: kernel from the camera gyro trace in c###_imu.csv")
    ap.add_argument("--window", choices=("B", "A"), default="B",
                    help="gyro kernel exposure window: B = [ts, ts+exp] (libcamera: "
                         "SensorTimestamp = first row starts exposing), A = [ts-exp, ts]")
    ap.add_argument("--readout-ms", type=float, default=60.0,
                    help="gyro kernel: rolling-shutter readout; the window is shifted "
                         "by half of it (middle row)")
    ap.add_argument("--angle-scale", type=float, default=1.0,
                    help="diagnostic: scale the blur angle (box) or trajectory (gyro)")
    ap.add_argument("--min-blur-px", type=float, default=4.0,
                    help="skip frames whose yaw blur at the frame corner is below this")
    ap.add_argument("--interp", type=int, choices=(1, 3), default=3,
                    help="polar resampling: 3 = cubic (default), 1 = linear, ~2x faster")
    ap.add_argument("--luma", action="store_true",
                    help="deblur only the luminance and keep the captured colour: "
                         "about 3x faster (onboard mode)")
    ap.add_argument("--cx", type=float)
    ap.add_argument("--cy", type=float)
    ap.add_argument("--synthetic", action="store_true",
                    help="blur the (sharp) input first, then deblur and report PSNR")
    ap.add_argument("-o", "--output", default="deblurred.jpg")
    args = ap.parse_args()
    global INTERP
    INTERP = args.interp

    if args.session:
        jobs = frames_from_session(args.session, args.frame, args.winners)
        if not jobs:
            ap.error("no matching frames in manifest.csv (use --frame NAME or --winners)")
        # every parameter in the name, so variants never overwrite each other
        tag = ("_box" if args.kernel == "box"
               else f"_gyro{args.window}_ro{args.readout_ms:g}")
        if args.angle_scale != 1.0:
            tag += f"_x{args.angle_scale:g}"
        tag += f"_snr{args.snr:g}"
        if args.luma:
            tag += "_luma"
        if args.interp != 3:
            tag += f"_i{args.interp}"
        out_dir = os.path.join(args.session, "_deblur")
        os.makedirs(out_dir, exist_ok=True)
        geom = None
        traces = {}
        for path, angle, r in jobs:
            if not os.path.exists(path):
                print(f"missing {path}, skipped")
                continue
            t0 = time.monotonic()
            img = load(path)
            h, w = img.shape[:2]
            if geom is None or (geom.h, geom.w) != (h, w):
                geom = PolarFilter(h, w, args.cx, args.cy)
            H = None
            note = "box"
            if args.kernel == "gyro":
                cyc = int(r["cycle_id"])
                if cyc not in traces:
                    ip = os.path.join(args.session, f"c{cyc:03d}_imu.csv")
                    traces[cyc] = (load_gz(ip) if os.path.exists(ip) else None,
                                   boot_offset(args.session, cyc))
                tr, off = traces[cyc]
                ts = int(r["sensor_timestamp_ns"]) - off
                exp = int(float(r["exposure_time_us"]) * 1000)
                a = ts if args.window == "B" else ts - exp
                a += int(args.readout_ms * 1e6 / 2)
                th = angle_trace(tr[0], tr[1], a, a + exp) if tr else None
                if th is None:
                    print(f"{os.path.basename(path)}: no gyro trace for the window, box used")
                else:
                    th = th * args.angle_scale
                    angle = float(th.max() - th.min())
                    H = gyro_otf(geom.n_t, th)
                    rate = np.interp(np.linspace(a, a + exp, 5), tr[0], tr[1])
                    note = ("gyro rate " + " ".join(f"{v:+.1f}" for v in rate) + " deg/s")
            else:
                angle *= args.angle_scale
            corner_px = math.radians(angle) * math.hypot(w / 2, h / 2)
            name = os.path.splitext(os.path.basename(path))[0] + f"_deblur{tag}.jpg"
            if corner_px < args.min_blur_px:
                shutil.copyfile(path, os.path.join(out_dir, name))   # bytes unchanged
                print(f"{os.path.basename(path)}: yaw {angle:.3f} deg, corner blur "
                      f"{corner_px:.2f} px < {args.min_blur_px}, copied")
                continue
            save(deblur(img, angle, args.snr, geom=geom, H=H, luma=args.luma),
                 os.path.join(out_dir, name))
            print(f"{os.path.basename(path)}: yaw {angle:.3f} deg, corner blur "
                  f"{corner_px:.1f} px ({note}) -> _deblur/{name}  "
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
        out = deblur(b, args.angle_deg, args.snr, geom=geom, luma=args.luma)
        h, w = img.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        inner = np.hypot(xx - geom.cx, yy - geom.cy) < min(h, w) / 2 - 2
        print(f"PSNR blurred  {psnr(img[inner], b[inner]):.2f} dB (inscribed circle)")
        print(f"PSNR restored {psnr(img[inner], out[inner]):.2f} dB")
        save(b, args.output.replace(".jpg", "_blurred.jpg"))
    else:
        out = deblur(img, args.angle_deg, args.snr, geom=geom, luma=args.luma)
    save(out, args.output)
    print(f"Written {args.output}  {time.monotonic() - t0:.1f} s, peak {peak_mb():.0f} MB")


if __name__ == "__main__":
    main()
