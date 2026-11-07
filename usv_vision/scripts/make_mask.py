#!/usr/bin/env python3
"""Build a view-mask image from a photo, painting ignore regions.

Convention of the produced mask (identical to yolo_infer_node):
  pixel == 0 (black)                       -> recognition region (kept)
  pixel == IGNORE_VALUE (255, what you paint) -> ignore region (masked out)

Regions to ignore can be given as pixel-space rectangles and/or normalized
polygons. Run this WITHOUT ROS (any python with cv2+numpy is enough):

  /home/ww/venv-trt/bin/python scripts/make_mask.py \
      --image in.png --out mask.png \
      --rect 100,200,300,120 \
      --polygons '[[[0.0,0.6],[1.0,0.6],[1.0,1.0],[0.0,1.0]]]'

Optional --preview out_overlay.png draws the ignore regions over the source
photo so you can eyeball what will be masked before using it.
"""

import argparse
import os
import sys

# Make `usv_vision.common.mask` importable when run from the source tree.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from usv_vision.common.mask import (  # noqa: E402
    IGNORE_VALUE,
    parse_polygons_json,
    rasterize_polygons_mask,
    rasterize_rectangles_mask,
)


def build_mask_image(width: int, height: int, rects=None, polygons=None, value: int = IGNORE_VALUE):
    """Rasterize rectangles (pixel [x, y, w, h]) and polygons (normalized) into a mask canvas.

    Background stays 0 (recognition); painted regions become `value` (ignore).
    """
    canvas = np.zeros((height, width), dtype=np.uint8)
    rects = list(rects or [])
    if rects:
        rect_canvas = rasterize_rectangles_mask(width, height, rects, value=value)
        canvas = np.maximum(canvas, rect_canvas)
    polys = [p for p in (polygons or []) if isinstance(p, list) and len(p) >= 3]
    if polys:
        poly_canvas = rasterize_polygons_mask(width, height, polys, value=value)
        canvas = np.maximum(canvas, poly_canvas)
    return canvas


def _draw_preview(image_bgr, ignore_mask_uint8):
    """Return a copy of image with ignore regions tinted red (BGR semi-transparent)."""
    preview = image_bgr.copy()
    colored = preview.copy()
    colored[ignore_mask_uint8 > 0] = (0, 0, 255)  # red in BGR
    preview = cv2.addWeighted(preview, 0.45, colored, 0.55, 0)
    return preview


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Build a view-mask image (black=recognition, white=ignore).")
    parser.add_argument("--image", required=True, help="source photo (used only for its size)")
    parser.add_argument("--out", required=True, help="output mask png (grayscale)")
    parser.add_argument("--rect", action="append", default=[], metavar="X,Y,W,H",
                        help="pixel-space rectangle to ignore; repeatable")
    parser.add_argument("--polygons", default="", metavar="JSON",
                        help="normalized (0..1) polygons JSON to ignore, e.g. '[[[0,0.6],[1,0.6],[1,1],[0,1]]]'")
    parser.add_argument("--preview", default="", help="optional overlay preview output png")
    parser.add_argument("--value", type=int, default=IGNORE_VALUE, help="ignore pixel value (default 255)")
    args = parser.parse_args(argv)

    if not args.image or not os.path.isfile(args.image):
        sys.exit(f"error: cannot read --image {args.image}")
    if not args.rect and not args.polygons:
        sys.exit("error: provide at least one --rect or --polygons")

    img = cv2.imread(args.image, cv2.IMREAD_UNCHANGED)
    if img is None:
        sys.exit(f"error: failed to decode --image {args.image}")
    height, width = img.shape[:2]

    rects = []
    for token in args.rect:
        try:
            x, y, w, h = (float(v) for v in token.split(","))
            rects.append([int(x), int(y), int(w), int(h)])
        except (ValueError, TypeError):
            sys.exit(f"error: invalid --rect '{token}', expected X,Y,W,H")

    polygons = parse_polygons_json(args.polygons) if args.polygons else []

    mask = build_mask_image(width, height, rects=rects, polygons=polygons, value=args.value)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    if not cv2.imwrite(args.out, mask):
        sys.exit(f"error: failed to write --out {args.out}")

    ignore_count = int(np.count_nonzero(mask))
    ratio = ignore_count / float(width * height)
    print(f"mask written: {args.out}  ({width}x{height})")
    print(f"  ignore px = {ignore_count}  ({ratio * 100:.2f}% of frame)")
    print("  convention: black=recognition, painted=ignore")

    if args.preview:
        preview = _draw_preview(img, mask)
        os.makedirs(os.path.dirname(os.path.abspath(args.preview)), exist_ok=True)
        cv2.imwrite(args.preview, preview)
        print(f"preview written: {args.preview}")


if __name__ == "__main__":
    main()
