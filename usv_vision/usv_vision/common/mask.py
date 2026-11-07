"""View-mask helpers shared by the inference node, the mask-builder tool and offline tests.

Convention (identical for mask images and runtime masks):
  pixel value == 0 (black)                       -> recognition region (kept)
  pixel value >= mask_ignore_min (what you paint) -> ignore region (masked out)

The ignore region is filled with a neutral grey before the frame is sent to the
model, so objects there do not participate in inference. It is purely an input
mask: detections are NOT post-filtered against it (so real targets straddling
the boundary are not deleted).
"""

import json
import os

import cv2
import numpy as np

# Value used when rasterizing an ignore polygon onto a black mask canvas.
IGNORE_VALUE = 255


def parse_polygons_json(text: str):
    """Parse normalized polygons from a JSON string -> [[[x, y], ...], ...].

    Coordinates are normalized to [0, 1], origin at top-left (same convention as
    yolo_infer_node's mask_polygons_json parameter). Returns [] on any error.
    """
    text = (text or "").strip()
    if not text:
        return []
    try:
        data = json.loads(text)
    except Exception:  # noqa: BLE001
        return []
    polygons = []
    if not isinstance(data, list):
        return polygons
    for polygon in data:
        if not isinstance(polygon, list):
            continue
        points = []
        for point in polygon:
            try:
                if len(point) >= 2:
                    points.append([float(point[0]), float(point[1])])
            except (TypeError, ValueError):
                continue
        if len(points) >= 3:
            polygons.append(points)
    return polygons


def _normalized_to_points(polygon, width: int, height: int):
    """Convert a normalized polygon (list of [x, y] in 0..1) to int32 pixel points."""
    return np.array(
        [
            [int(round(float(x) * (width - 1))), int(round(float(y) * (height - 1)))]
            for x, y in polygon
        ],
        dtype=np.int32,
    ).reshape((-1, 1, 2))


def rasterize_polygons_mask(
    width: int,
    height: int,
    polygons,
    value: int = IGNORE_VALUE,
):
    """Rasterize normalized polygons onto a (height, width) uint8 canvas.

    Background = 0, pixels inside any polygon = `value`.
    """
    mask = np.zeros((height, width), dtype=np.uint8)
    for polygon in polygons:
        if len(polygon) < 3:
            continue
        pts = _normalized_to_points(polygon, width, height)
        cv2.fillPoly(mask, [pts], int(value))
    return mask


def ignore_from_mask_image(
    mask_path: str,
    height: int,
    width: int,
    mask_ignore_min: int,
):
    """Load a mask image file and return its ignore-region bool map.

    Returns np.ndarray(dtype=bool) of shape (height, width) with True = ignore,
    or None if the file cannot be read (missing / invalid). Image is resized to
    (width, height) with INTER_NEAREST if its size differs.
    """
    if not mask_path or not os.path.isfile(mask_path):
        return None
    raw = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if raw is None:
        return None
    if raw.shape[:2] != (height, width):
        raw = cv2.resize(raw, (width, height), interpolation=cv2.INTER_NEAREST)
    return raw >= max(1, int(mask_ignore_min))


def ignore_from_polygons(polygons, height: int, width: int):
    """Rasterize normalized polygons into a bool ignore map (True = ignore).

    Returns None when there are no usable polygons.
    """
    polys = [p for p in polygons if isinstance(p, list) and len(p) >= 3]
    if not polys:
        return None
    mask = rasterize_polygons_mask(width, height, polys)
    return mask.astype(bool)


def combine_ignore(mask_image_ignore, polygons_ignore):
    """Merge two optional ignore maps; returns None only if both are None."""
    if mask_image_ignore is None and polygons_ignore is None:
        return None
    if mask_image_ignore is None:
        return polygons_ignore
    if polygons_ignore is None:
        return mask_image_ignore
    return mask_image_ignore | polygons_ignore


def build_ignore_mask(
    height: int,
    width: int,
    mask_path: str = "",
    mask_ignore_min: int = 1,
    polygons=None,
):
    """Build the full ignore bool map from a mask image and/or normalized polygons.

    True = ignore region. Returns None when no usable source is provided.
    """
    from_image = ignore_from_mask_image(mask_path, height, width, mask_ignore_min)
    from_polys = ignore_from_polygons(polygons or [], height, width)
    return combine_ignore(from_image, from_polys)


def apply_input_mask(frame, ignore_mask, fill: int = 114):
    """Return a copy of `frame` with the ignore region filled with `fill`.

    The original frame is never modified. When ignore_mask is None this returns
    an untouched copy (kept as a copy for a consistent return type).
    """
    if ignore_mask is None:
        return frame.copy()
    result = frame.copy()
    result[ignore_mask] = int(fill)
    return result


def rasterize_rectangles_mask(
    width: int,
    height: int,
    rects,
    value: int = IGNORE_VALUE,
):
    """Rasterize pixel-space rectangles [x, y, w, h] onto a (height, width) uint8 canvas.

    Helper used by the mask-builder tool. Background = 0.
    """
    mask = np.zeros((height, width), dtype=np.uint8)
    for rect in rects:
        try:
            x, y, w, h = (int(v) for v in rect)
        except (TypeError, ValueError):
            continue
        x2 = min(width - 1, max(0, x + max(0, w)))
        y2 = min(height - 1, max(0, y + max(0, h)))
        x1 = min(width - 1, max(0, x))
        y1 = min(height - 1, max(0, y))
        if x2 > x1 and y2 > y1:
            cv2.rectangle(mask, (x1, y1), (x2 - 1, y2 - 1), int(value), -1)
    return mask
