#!/usr/bin/env python3
"""
Side-by-side 100 % crops of a frame and its deblurred variants.

Takes the original frame and every DIR/_deblur/<frame>_deblur*.jpg, cuts
the same crops (four corners and the centre) and writes one panel
DIR/_deblur/<frame>_compare.jpg. Prints, per variant, how much it differs
from the original (mean absolute difference in 8-bit levels) in the corner
crops and in the centre, so "no visible difference" can be told apart from
"nothing was done".

Usage:
  python3 deblur_compare.py --session DIR --frame c026_CORRECTED_f3.jpg [--crop 400]
"""

import argparse
import glob
import os

import numpy as np
from PIL import Image, ImageDraw


def boxes(w, h, c, inset):
    return {
        "top-left": (inset, inset, inset + c, inset + c),
        "top-right": (w - inset - c, inset, w - inset, inset + c),
        "centre": ((w - c) // 2, (h - c) // 2, (w + c) // 2, (h + c) // 2),
        "bottom-left": (inset, h - inset - c, inset + c, h - inset),
        "bottom-right": (w - inset - c, h - inset - c, w - inset, h - inset),
    }


def main():
    ap = argparse.ArgumentParser(description="Compare a frame with its deblurred variants")
    ap.add_argument("--session", required=True)
    ap.add_argument("--frame", required=True)
    ap.add_argument("--crop", type=int, default=400, help="crop size in px (100 %%)")
    ap.add_argument("--inset", type=int, default=150, help="distance of corner crops from the edge")
    args = ap.parse_args()

    stem = os.path.splitext(args.frame)[0]
    orig_path = os.path.join(args.session, args.frame)
    variants = sorted(glob.glob(os.path.join(args.session, "_deblur", f"{stem}_deblur*.jpg")))
    if not variants:
        raise SystemExit(f"no {stem}_deblur*.jpg in {args.session}/_deblur")
    images = [("original", Image.open(orig_path).convert("RGB"))]
    images += [(os.path.basename(v)[len(stem) + 1:-4], Image.open(v).convert("RGB"))
               for v in variants]
    w, h = images[0][1].size
    bx = boxes(w, h, args.crop, args.inset)

    o = np.asarray(images[0][1], dtype=np.float32)
    print(f"{args.frame}: mean |variant - original| in 8-bit levels")
    for name, im in images[1:]:
        a = np.asarray(im, dtype=np.float32)
        corner = np.mean([np.abs(a[y0:y1, x0:x1] - o[y0:y1, x0:x1]).mean()
                          for k, (x0, y0, x1, y1) in bx.items() if k != "centre"])
        x0, y0, x1, y1 = bx["centre"]
        centre = np.abs(a[y0:y1, x0:x1] - o[y0:y1, x0:x1]).mean()
        print(f"  {name:22s} corners {corner:5.2f}   centre {centre:5.2f}")

    c, pad, head = args.crop, 6, 18
    panel = Image.new("RGB", (len(images) * (c + pad), len(bx) * (c + pad) + head), "white")
    d = ImageDraw.Draw(panel)
    for j, (name, im) in enumerate(images):
        d.text((j * (c + pad) + 4, 2), name, fill="black")
        for i, (k, b) in enumerate(bx.items()):
            panel.paste(im.crop(b), (j * (c + pad), head + i * (c + pad)))
            if j == 0:
                d.text((4, head + i * (c + pad) + 4), k, fill="yellow")
    out = os.path.join(args.session, "_deblur", f"{stem}_compare.jpg")
    panel.save(out, quality=92)
    print(f"Written {out}")


if __name__ == "__main__":
    main()
