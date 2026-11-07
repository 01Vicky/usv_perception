#!/usr/bin/env python3
"""Offline proof that the view-mask changes inference the way we expect.

Runs the SAME model twice on one photo:
  1. without mask            -> *_nomask.png
  2. with the ignore region filled (mask image path form) -> *_mask.png

Then reports which detections fell inside the ignore region (and disappeared
under the mask) versus which stayed in the recognition region. Proof images are
saved under the --out directory for visual inspection.

Run WITHOUT ROS, with an interpreter that has ultralytics + cv2 + numpy, e.g.:

  /home/ww/venv-trt/bin/python scripts/verify_mask_recognition.py \
      --image indoor.png --mask mask.png \
      --model /home/ww/USV_ROS/model/pt/yolo11n.pt --conf 0.25

Acceptance: objects fully covered by the mask no longer produce a box, while
objects in the recognition region still do.
"""

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from usv_vision.common.mask import apply_input_mask, ignore_from_mask_image  # noqa: E402

DEFAULT_FILL = 114  # Ultralytics letterbox neutral grey
CLASS_COLORS = [
    (0, 140, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255),
    (255, 0, 255), (255, 255, 0), (0, 165, 255), (128, 0, 255),
    (0, 128, 255), (255, 128, 0),
]


def _extract(result):
    names = getattr(result, "names", {})
    boxes = getattr(result, "boxes", None)
    dets = []
    if boxes is None or len(boxes) == 0:
        return dets
    for box in boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        x1, y1, x2, y2 = (int(round(v)) for v in map(float, box.xyxy[0]))
        name = str(names.get(cls_id, cls_id)) if isinstance(names, dict) else str(names[cls_id])
        dets.append({"cls": cls_id, "name": name, "conf": conf,
                     "x1": x1, "y1": y1, "x2": x2, "y2": y2})
    return dets


def _center_in(det, ignore):
    h, w = ignore.shape[:2]
    cx = int((det["x1"] + det["x2"]) / 2)
    cy = int((det["y1"] + det["y2"]) / 2)
    cx = min(max(cx, 0), w - 1)
    cy = min(max(cy, 0), h - 1)
    return bool(ignore[cy, cx])


def _draw_boxes(image, dets):
    out = image.copy()
    for det in dets:
        color = CLASS_COLORS[det["cls"] % len(CLASS_COLORS)]
        label = f"{det['name']} {det['conf']:.2f}"
        cv2.rectangle(out, (det["x1"], det["y1"]), (det["x2"], det["y2"]), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        bar_top = det["y1"] - th - 8
        if bar_top < 0:
            bar_top = det["y1"]
        cv2.rectangle(out, (det["x1"], bar_top), (det["x1"] + tw + 8, bar_top + th + 8), color, -1)
        cv2.putText(out, label, (det["x1"] + 4, bar_top + th + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def _tint_ignore(image, ignore):
    """Return a copy with ignore region tinted red for visual proof."""
    out = image.copy()
    overlay = out.copy()
    overlay[ignore] = (0, 0, 255)
    return cv2.addWeighted(out, 0.4, overlay, 0.6, 0)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Offline view-mask vs no-mask recognition proof.")
    parser.add_argument("--image", required=True, help="source photo")
    parser.add_argument("--mask", required=True, help="mask png (black=recognition, painted=ignore)")
    parser.add_argument("--model", required=True, help="yolo model path (.pt/.onnx/.engine loadable in this env)")
    parser.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    parser.add_argument("--fill", type=int, default=DEFAULT_FILL, help="ignore-region fill grey (0..255)")
    parser.add_argument("--tag", default="", help="extra label appended to output file names (optional)")
    parser.add_argument("--out", default="test/output/mask_recognition", help="output directory for proof pngs")
    args = parser.parse_args(argv)

    for path, tag in ((args.image, "--image"), (args.mask, "--mask"), (args.model, "--model")):
        if not os.path.isfile(path):
            sys.exit(f"error: cannot read {tag} {path}")

    try:
        from ultralytics import YOLO
    except ImportError as exc:  # pragma: no cover
        sys.exit("error: ultralytics not available in this python; use venv-trt: " + str(exc))

    frame = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if frame is None:
        sys.exit(f"error: failed to decode --image {args.image}")
    height, width = frame.shape[:2]

    ignore = ignore_from_mask_image(args.mask, height, width, 1)
    if ignore is None:
        sys.exit(f"error: could not load/apply --mask {args.mask} at {width}x{height}")
    ignore_px = int(ignore.sum())

    model = YOLO(args.model, task="detect")
    print(f"model loaded: {os.path.basename(args.model)} | image {width}x{height} | "
          f"ignore region {ignore_px}px ({ignore_px / (width * height) * 100:.2f}%) | fill={args.fill}")

    results_nomask = model.predict(frame, conf=args.conf, verbose=False)[0]
    dets_nomask = _extract(results_nomask)

    masked = apply_input_mask(frame, ignore, args.fill)
    results_mask = model.predict(masked, conf=args.conf, verbose=False)[0]
    dets_mask = _extract(results_mask)

    os.makedirs(args.out, exist_ok=True)
    img_base = os.path.splitext(os.path.basename(args.image))[0]
    mask_base = os.path.splitext(os.path.basename(args.mask))[0]
    stem = f"{img_base}__{mask_base}"
    if args.tag:
        stem = f"{stem}__{args.tag}"

    out_nomask = os.path.join(args.out, f"{stem}_nomask.png")
    cv2.imwrite(out_nomask, _draw_boxes(frame, dets_nomask))
    out_mask = os.path.join(args.out, f"{stem}_mask.png")
    cv2.imwrite(out_mask, _tint_ignore(_draw_boxes(frame, dets_mask), ignore))

    # ── console evidence ──
    line = "=" * 66
    print(line)
    print(f"[no-mask] baseline detections ({len(dets_nomask)}):")
    print(line)
    for det in dets_nomask:
        inside = _center_in(det, ignore)
        mark = "IGNORE区" if inside else "识别区 "
        print(f"    {det['name']:<12} conf={det['conf']:.2f} bbox=({det['x1']},{det['y1']},{det['x2']},{det['y2']})  center→{mark}")

    print(line)
    print(f"[mask]   detections after filling ignore region ({len(dets_mask)}):")
    print(line)
    for det in dets_mask:
        print(f"    {det['name']:<12} conf={det['conf']:.2f} bbox=({det['x1']},{det['y1']},{det['x2']},{det['y2']})")

    nomask_inside = [d for d in dets_nomask if _center_in(d, ignore)]
    nomask_outside = [d for d in dets_nomask if not _center_in(d, ignore)]

    print(line)
    print("结论: 蒙板盖住(忽略区)的基线框理应在 mask 下消失; 识别区框理当保留")
    print(f"   忽略区内基线框数: {len(nomask_inside)}"
          f"  -> 消失 {len(nomask_inside) - sum(1 for d in dets_mask if _center_in(d, ignore))}"
          f"  (期望全消失)")
    print(f"   识别区内基线框数: {len(nomask_outside)}"
          f"  -> mask 后仍在 {sum(1 for d in dets_mask if not _center_in(d, ignore))} (期望都保留)")
    print(f"\n证据图:")
    print(f"   {out_nomask}")
    print(f"   {out_mask}")


if __name__ == "__main__":
    main()
