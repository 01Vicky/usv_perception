#!/usr/bin/env python3
"""Draw a view-mask by painting over a photo (mouse) or with a guide template.

Convention (identical to yolo_infer_node / make_mask):
  pixel == 0 (black)                   -> recognition region (kept)
  pixel == 255 (white, what you paint) -> ignore region (masked out)

The photo is only shown as a dimmed underlay; strokes are drawn on a black
canvas so the produced mask never picks up non-black photo pixels.

Three modes of the same tool:

  1) Interactive GUI (default, needs a display):
       draw_mask --image X --out mask.png
       LMB drag = paint white (ignore), RMB / 'e' = erase
       '[' ']' / mouse wheel = brush size, 'r' = reset, 's' = save+quit, Esc='q'=quit

  2) Guide template (no display needed): emit a black canvas with the dimmed
     photo so you can paint white strokes in an external image editor:
       draw_mask --image X --out mask.png --guide

  3) Convert a hand-painted image into the final black/white mask:
       draw_mask --convert painted.png --out mask.png [--threshold 200]

Run as:
  ros2 run usv_vision draw_mask --image X --out mask.png
  # or standalone with cv2+numpy python:
  /home/ww/venv-trt/bin/python scripts/draw_mask.py --image X --out mask.png
"""

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

UNDERLAY_DIM = 0.42  # how much the photo underlay is darkened


# ── pure logic (unit-testable, no window) ────────────────────────────────────
def build_underlay_canvas(image_bgr):
    """Black canvas with the photo dimmed as a painting reference."""
    h, w = image_bgr.shape[:2]
    dim = (image_bgr.astype(np.float64) * UNDERLAY_DIM).astype(np.uint8)
    return dim


def guide_from_image(image_bgr):
    """Guide template: dimmed photo on black (for painting white strokes on top)."""
    return build_underlay_canvas(image_bgr)


def convert_painted(painted_bgr, threshold: int = 200):
    """Turn white strokes on a painted image into a black/white mask (0 / 255)."""
    gray = cv2.cvtColor(painted_bgr, cv2.COLOR_BGR2GRAY)
    strokes = gray >= max(0, min(255, int(threshold)))
    mask = np.zeros(gray.shape, dtype=np.uint8)
    mask[strokes] = 255
    return mask


# ── interactive GUI ──────────────────────────────────────────────────────────
BRUSH_START = 24


def fit_scale(orig_w: int, orig_h: int, win_w: int, win_h: int, max_scale: float = 4.0):
    """Display scale that fits the original image inside the window (or enlarges).

    Returns 1.0 when window size is unknown. Enlargement beyond 1x is allowed up
    to `max_scale` so the user can zoom by resizing the window larger.
    """
    if win_w <= 0 or win_h <= 0 or orig_w <= 0 or orig_h <= 0:
        return 1.0
    s = min(win_w / float(orig_w), win_h / float(orig_h))
    return max(0.02, min(float(max_scale), s))


def map_to_orig(x: int, y: int, scale: float, orig_w: int, orig_h: int):
    """Map a display-space point (x, y) back to original-image coordinates (clamped)."""
    scale = scale if scale > 0 else 1.0
    ox = min(orig_w - 1, max(0, int(round(x / scale))))
    oy = min(orig_h - 1, max(0, int(round(y / scale))))
    return ox, oy


class _Painter:
    def __init__(self, image_bgr, mask_bgr, scale: float):
        self.image = image_bgr          # full-res dimmed underlay
        self.mask = mask_bgr            # full-res working canvas (strokes drawn white)
        self.h, self.w = image_bgr.shape[:2]
        self.scale = scale              # current display scale (updated per frame)
        self.brush = BRUSH_START        # brush radius in ORIGINAL pixels
        self.drawing = False
        self.erasing = False
        self.clean = True

    def _overlay_full(self):
        """Full-resolution preview with strokes over the dimmed underlay + HUD."""
        base = self.image.copy()
        show = base.copy()
        painted = self.mask[..., 0] > 0
        show[painted] = (255, 255, 255)
        preview = cv2.addWeighted(base, 0.55, show, 0.45, 0)
        cv2.putText(preview, f"brush={self.brush}px  [s]save  [r]reset  [e]erase  Esc=quit",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        return preview

    def _fit_view(self):
        """Resize the full-res preview to the current window so the whole image shows."""
        try:
            x0, y0, win_w, win_h = cv2.getWindowImageRect("draw_mask")
        except cv2.error:  # noqa: BLE001
            x0 = y0 = win_w = win_h = 0
        self.scale = fit_scale(self.w, self.h, win_w, win_h)
        disp_w = max(1, int(round(self.w * self.scale)))
        disp_h = max(1, int(round(self.h * self.scale)))
        full = self._overlay_full()
        if disp_w >= self.w and disp_h >= self.h:
            return full
        return cv2.resize(full, (disp_w, disp_h), interpolation=cv2.INTER_AREA)

    def mouse(self, event, x, y, flags, param):
        ox, oy = map_to_orig(x, y, self.scale, self.w, self.h)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing, self.erasing = True, False
            self.clean = False
            self._paint(ox, oy, erase=False)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.erasing, self.drawing = True, False
            self.clean = False
            self._paint(ox, oy, erase=True)
        elif event == cv2.EVENT_MOUSEMOVE and (self.drawing or self.erasing):
            self._paint(ox, oy, erase=self.erasing)
        elif event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_RBUTTONUP:
            self.drawing, self.erasing = False, False
        elif event == cv2.EVENT_MOUSEWHEEL:
            step = 4 if flags > 0 else -4
            self.brush = max(2, min(120, self.brush + step))

    def _paint(self, ox: int, oy: int, erase: bool):
        color = (0, 0, 0) if erase else (255, 255, 255)
        cv2.circle(self.mask, (ox, oy), self.brush, color, -1)


def run_interactive(image_path, out_path, max_side: int = 1000):
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image is None:
        sys.exit(f"error: cannot read image {image_path}")
    h, w = image.shape[:2]
    underlay = guide_from_image(image)      # full-res dimmed photo
    mask = np.zeros((h, w, 3), dtype=np.uint8)  # strokes drawn here

    # Initial fit so the WHOLE image is visible (may be < native size).
    init_scale = min(1.0, float(max_side) / max(w, h))
    disp_w0 = max(1, int(round(w * init_scale)))
    disp_h0 = max(1, int(round(h * init_scale)))

    win = "draw_mask — paint ignore region white"
    painter = _Painter(underlay, mask, scale=init_scale)

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, disp_w0, disp_h0)
    cv2.setMouseCallback(win, painter.mouse)
    saved = False
    while True:
        cv2.imshow(win, painter._fit_view())
        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord('q')):            # Esc / q -> quit
            break
        if key == ord('s'):                  # save full-res mask
            _write_gray_mask(painter.mask, out_path)
            saved = True
            print(f"mask saved: {out_path}")
            break
        if key == ord('r'):                  # reset
            mask[:] = 0
            painter.clean = True
        if key == ord('e'):                  # toggle erase
            painter.erasing = not painter.erasing
        if key == ord('['):                  # smaller brush
            painter.brush = max(2, painter.brush - 4)
        if key == ord(']'):                  # larger brush
            painter.brush = min(120, painter.brush + 4)
    cv2.destroyAllWindows()
    if not saved and not painter.clean:
        print("not saved (no mask written)")


def _write_gray_mask(mask_bgr, out_path):
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    gray = mask_bgr[..., 0].copy()
    gray[gray > 0] = 255
    cv2.imwrite(out_path, gray)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Draw a view-mask by painting (mouse) or via guide/convert.")
    parser.add_argument("--image", default="", help="source photo")
    parser.add_argument("--out", default="mask.png", help="output mask png")
    parser.add_argument("--guide", action="store_true",
                        help="emit a black canvas with dimmed photo for painting in an external editor")
    parser.add_argument("--convert", default="", metavar="PNG",
                        help="convert a hand-painted image into the final black/white mask")
    parser.add_argument("--threshold", type=int, default=200,
                        help="pixel brightness >= this counts as painted stroke (default 200)")
    parser.add_argument("--max-side", type=int, default=1000,
                        help="initial longest displayed side in px, so the whole image fits on screen (default 1000)")
    args = parser.parse_args(argv)

    # mode 3: convert
    if args.convert:
        painted = cv2.imread(args.convert, cv2.IMREAD_COLOR)
        if painted is None:
            sys.exit(f"error: cannot read --convert {args.convert}")
        mask = convert_painted(painted, args.threshold)
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        cv2.imwrite(args.out, mask)
        px = int(np.count_nonzero(mask))
        print(f"mask written: {args.out}  ignore px={px} ({px / mask.size * 100:.2f}%)")
        return

    if not args.image:
        sys.exit("error: provide --image (or --convert painted.png)")
    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        sys.exit(f"error: cannot read --image {args.image}")

    # mode 2: guide template only
    if args.guide:
        guide = guide_from_image(image)
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        cv2.imwrite(args.out, guide)
        print(f"guide written: {args.out}  (dimmed photo on black; paint white over ignore areas, "
              f"then run with --convert that_file --out mask.png)")
        return

    # mode 1: interactive GUI
    try:
        run_interactive(args.image, args.out, max_side=args.max_side)
    except cv2.error as exc:  # noqa: BLE001
        sys.exit(f"error: no display available ({exc}).\n"
                 f"Use --guide to emit a template, paint white in an external editor, then:\n"
                 f"  {os.path.basename(sys.argv[0])} --convert painted.png --out mask.png")


if __name__ == "__main__":
    main()
