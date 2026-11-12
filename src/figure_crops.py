#!/usr/bin/env python3
"""
Thesis figure helper: full-frame thumbnail + 100 % crop for bench example frames.

Rotation (yaw) blur grows with the distance from the image centre, so a page-width
thumbnail of a 12 MP frame hides most of it. For each input frame this writes:

  <out>/<stem>_full.jpg   downscaled frame (default 1600 px wide) with the crop box drawn
  <out>/<stem>_crop.jpg   100 % pixel crop (default 600 x 600 px)

All frames given in one call use the SAME crop radius, so their blur is comparable
(yaw blur in px is proportional to the radius). The crop position on that circle is
chosen per frame where the image has the most texture (highest grey-level standard
deviation), so the crop lands on the checkerboard and not on the floor. The servo
rotates the camera, so the board is not at the same place in every frame.

Usage (from deblur_data):
  python3 ../SPIRE/src/figure_crops.py \\
      session_20261108_121426/c008_UNCORRECTED_f5.jpg \\
      session_20261108_121925/c004_CORRECTED_f5.jpg -o figures

Options: --radius 0.45 (fraction of the centre-to-corner distance), --crop 600,
--width 1600, --at ANGLE_DEG (fix the crop angle instead of searching).
"""

import argparse
import math
import os

import numpy as np
from PIL import Image, ImageDraw


def best_angle(gray, cx, cy, r, half, step_deg=5):
    h, w = gray.shape
    best = None
    for a in range(0, 360, step_deg):
        x = cx + r * math.cos(math.radians(a))
        y = cy - r * math.sin(math.radians(a))
        x0, y0 = int(round(x - half)), int(round(y - half))
        if x0 < 0 or y0 < 0 or x0 + 2 * half > w or y0 + 2 * half > h:
            continue
        s = float(gray[y0:y0 + 2 * half:4, x0:x0 + 2 * half:4].std())
        if best is None or s > best[0]:
            best = (s, a)
    return None if best is None else best[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", nargs="+")
    ap.add_argument("-o", "--out", default="figures")
    ap.add_argument("--radius", type=float, default=0.45,
                    help="crop centre radius as a fraction of the centre-to-corner distance")
    ap.add_argument("--crop", type=int, default=600, help="crop size in px (square)")
    ap.add_argument("--width", type=int, default=1600, help="thumbnail width in px")
    ap.add_argument("--at", type=float, default=None,
                    help="fixed crop angle in degrees (0 = right, 90 = up); default: search")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    half = args.crop // 2
    for path in args.frames:
        img = Image.open(path).convert("RGB")
        w, h = img.size
        cx, cy = w / 2, h / 2
        R = math.hypot(cx, cy)
        r = args.radius * R
        gray = np.asarray(img.convert("L"), dtype=np.float32)
        a = args.at if args.at is not None else best_angle(gray, cx, cy, r, half)
        if a is None:
            raise SystemExit(f"{path}: crop does not fit at radius {args.radius}; use a smaller --radius")
        x = cx + r * math.cos(math.radians(a))
        y = cy - r * math.sin(math.radians(a))
        box = (int(round(x - half)), int(round(y - half)),
               int(round(x - half)) + args.crop, int(round(y - half)) + args.crop)

        stem = os.path.splitext(os.path.basename(path))[0]
        session = os.path.basename(os.path.dirname(os.path.abspath(path)))
        tag = f"{session.replace('session_', '')}_{stem}"
        img.crop(box).save(os.path.join(args.out, f"{tag}_crop.jpg"), quality=95)

        scale = args.width / w
        thumb = img.resize((args.width, int(round(h * scale))), Image.LANCZOS)
        d = ImageDraw.Draw(thumb)
        lw = max(3, args.width // 400)
        d.rectangle([v * scale for v in box], outline=(230, 40, 40), width=lw)
        thumb.save(os.path.join(args.out, f"{tag}_full.jpg"), quality=92)
        print(f"{tag}: crop centre r = {r:.0f} px ({args.radius:.2f} R), angle {a:.0f} deg, "
              f"box {box}")


if __name__ == "__main__":
    main()
