"""View-mask regression tests; no camera, model or ROS node is started."""

from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from usv_vision.common.mask import (
    apply_input_mask,
    build_ignore_mask,
    ignore_from_mask_image,
    parse_polygons_json,
)
from usv_vision.yolo_infer_node import YoloInferNode


@pytest.mark.parametrize("text", ["", "invalid", "{}", "[[]]", "[[[0, 0], [1, 1]]]"])
def test_invalid_polygons_are_ignored(text):
    assert parse_polygons_json(text) == []


def test_image_and_polygon_masks_are_combined(tmp_path):
    path = tmp_path / "mask.png"
    image = np.zeros((4, 4), dtype=np.uint8)
    image[0, 0] = 255
    assert cv2.imwrite(str(path), image)
    polygons = parse_polygons_json("[[[0.6,0.6],[1,0.6],[1,1],[0.6,1]]]")
    mask = build_ignore_mask(4, 4, str(path), polygons=polygons)
    assert mask.dtype == np.bool_
    assert mask[0, 0] and mask[3, 3]
    assert not mask[0, 3]


def test_mask_resize_uses_nearest_neighbor(tmp_path):
    path = tmp_path / "mask.png"
    assert cv2.imwrite(str(path), np.array([[0, 255], [0, 0]], dtype=np.uint8))
    mask = ignore_from_mask_image(str(path), 4, 4, 1)
    assert mask[:2, 2:].all()
    assert mask.sum() == 4


def test_fill_keeps_original_and_recognition_region_unchanged():
    frame = np.full((3, 3, 3), 17, dtype=np.uint8)
    mask = np.zeros((3, 3), dtype=bool)
    mask[0, 0] = True
    result = apply_input_mask(frame, mask, 114)
    assert (frame == 17).all()
    assert (result[0, 0] == 114).all()
    assert (result[1:, :] == 17).all()
    assert build_ignore_mask(3, 3) is None


def test_reload_reads_updated_parameters_and_clears_failed_cache(tmp_path):
    path = tmp_path / "new_mask.png"
    image = np.zeros((4, 4), dtype=np.uint8)
    image[0, 0] = 255
    assert cv2.imwrite(str(path), image)
    values = {
        "enable_mask": True,
        "mask_path": str(path),
        "mask_ignore_min": 1,
        "mask_fill": 114,
        "mask_polygons_json": "",
    }
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        get_logger=Mock(),
        _mask_ignore=np.ones((2, 2), dtype=bool),
        _mask_shape=(2, 2),
        _mask_failed=True,
    )
    node._build_ignore_mask = lambda h, w: YoloInferNode._build_ignore_mask(node, h, w)
    response = YoloInferNode._on_reload_mask(node, None, SimpleNamespace())
    assert response.success
    assert node.enable_mask and node.mask_path == str(path)
    assert node._mask_ignore is None and node._mask_shape is None
    assert not node._mask_failed
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    assert (YoloInferNode._apply_input_mask(node, frame)[0, 0] == 114).all()

    # Overwriting the same path must invalidate the old image cache too.
    image[0, 0], image[3, 3] = 0, 255
    assert cv2.imwrite(str(path), image)
    YoloInferNode._on_reload_mask(node, None, SimpleNamespace())
    result = YoloInferNode._apply_input_mask(node, frame)
    assert (result[0, 0] == 0).all()
    assert (result[3, 3] == 114).all()
