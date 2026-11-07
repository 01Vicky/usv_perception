import json
import os
from typing import Optional

from sensor_msgs.msg import CameraInfo


class CameraInfoLoader:
    def __init__(self, node, frame_id: str, camera_info_url: str = "") -> None:
        self.node = node
        self.frame_id = frame_id
        self.camera_info_url = camera_info_url
        self._cached: Optional[CameraInfo] = None

    def get(self, width: int, height: int) -> CameraInfo:
        if self._cached is None or self._cached.width != width or self._cached.height != height:
            self._cached = self._load(width, height)
        return self._cached

    def _load(self, width: int, height: int) -> CameraInfo:
        msg = CameraInfo()
        msg.width = int(width)
        msg.height = int(height)
        msg.header.frame_id = self.frame_id

        if self.camera_info_url and os.path.exists(self.camera_info_url):
            try:
                with open(self.camera_info_url, "r", encoding="utf-8") as file:
                    data = json.load(file)

                fx = float(data["focal_length"]["fx"])
                fy = float(data["focal_length"]["fy"])
                cx = float(data["principal_point"]["cx"])
                cy = float(data["principal_point"]["cy"])

                calib_w = data.get("image_width") or (data.get("resolution", {}) or {}).get("width")
                calib_h = data.get("image_height") or (data.get("resolution", {}) or {}).get("height")
                if calib_w and calib_h:
                    calib_w = int(calib_w)
                    calib_h = int(calib_h)
                    if calib_w > 0 and calib_h > 0 and (calib_w != width or calib_h != height):
                        sx = width / float(calib_w)
                        sy = height / float(calib_h)
                        fx *= sx
                        fy *= sy
                        cx *= sx
                        cy *= sy
                        self.node.get_logger().warn(
                            f"camera info resized from {calib_w}x{calib_h} to {width}x{height}"
                        )

                distortion = data.get("distortion_coefficients")
                if distortion is None:
                    distortion = data.get("dist_coeffs", [0.0] * 5)
                if distortion and isinstance(distortion[0], list):
                    distortion = distortion[0]

                msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
                msg.d = [float(value) for value in distortion]
                msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
                msg.distortion_model = "plumb_bob"
                self.node.get_logger().info(f"loaded camera info: {self.camera_info_url}")
                return msg
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().error(f"failed to parse camera info, fallback to empty: {exc}")

        msg.k = [0.0] * 9
        msg.d = [0.0] * 5
        msg.p = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = ""
        return msg
