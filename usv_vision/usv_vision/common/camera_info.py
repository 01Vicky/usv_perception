import json
import os
from typing import Optional

from sensor_msgs.msg import CameraInfo


class CameraInfoLoader:
    def __init__(self, node, frame_id: str, camera_info_url: str = "",
                 camera_model: str = "pinhole") -> None:
        self.node = node
        self.frame_id = frame_id
        self.camera_info_url = camera_info_url
        # pinhole（默认，D 按 plumb_bob 解释）| scaramuzza（MATLAB 鱼眼全向模型）
        self.camera_model = str(camera_model or "pinhole").strip().lower()
        self._cached: Optional[CameraInfo] = None

    def get(self, width: int, height: int) -> CameraInfo:
        if self._cached is None or self._cached.width != width or self._cached.height != height:
            self._cached = self._load(width, height)
        return self._cached

    @staticmethod
    def _resolve_url(url: str) -> str:
        """相对路径按包 share 目录解析（标定文件在 usv_vision_gmsl_cpp 包里）。"""
        if not url or os.path.isabs(url):
            return url
        names = []
        try:
            from ament_index_python.packages import get_package_share_directory
            for name in ("usv_vision_gmsl_cpp", "usv_vision"):
                try:
                    names.append(os.path.join(get_package_share_directory(name), url))
                except Exception:  # noqa: BLE001
                    continue
        except ImportError:
            pass
        for candidate in names:
            if os.path.exists(candidate):
                return candidate
        return url

    def _load(self, width: int, height: int) -> CameraInfo:
        msg = CameraInfo()
        msg.width = int(width)
        msg.height = int(height)
        msg.header.frame_id = self.frame_id

        url = self._resolve_url(self.camera_info_url)
        if url and os.path.exists(url):
            try:
                with open(url, "r", encoding="utf-8") as file:
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

                if self.camera_model == "scaramuzza":
                    # MATLAB 鱼眼（Scaramuzza 全向模型）：
                    #   f(rho) = a0 + a1*rho + a2*rho^2 + a3*rho^3 + a4*rho^4
                    #   dist_coeffs 依次存 [a0, a1, a2, a3, a4]（a1=0，凑 5 位与针孔长度兼容）
                    # a0 放进 fx/fy（中心附近与等效焦距同量级），a1..a4 放进 D。
                    if len(distortion) < 5:
                        self.node.get_logger().error(
                            f"camera_model=scaramuzza 需要 5 个映射系数 [a0..a4]，"
                            f"实际 {len(distortion)} 个，按针孔处理"
                        )
                    else:
                        a = [float(value) for value in distortion[:5]]
                        if calib_w and calib_h and (calib_w != width or calib_h != height):
                            sx = width / float(calib_w)
                            sy = height / float(calib_h)
                            if abs(sx - sy) > 0.01 * max(sx, sy):
                                self.node.get_logger().error(
                                    f"scaramuzza 标定 {calib_w}x{calib_h} 到出图 {width}x{height} "
                                    f"是**非等比**缩放（sx={sx:.4f} sy={sy:.4f}），径向模型不成立；"
                                    f"请把算法图配成保比例尺寸（原图 5:4 对应 640x512）或改用针孔模型"
                                )
                            else:
                                # 像素整体缩放 s：a0 -> s*a0，a2 -> a2/s，a3 -> a3/s^2，a4 -> a4/s^3
                                a[0] *= sx
                                a[2] /= sx
                                a[3] /= (sx * sx)
                                a[4] /= (sx * sx * sx)
                                self.node.get_logger().info(
                                    f"scaramuzza 映射系数按 {sx:.4f} 缩放到 {width}x{height}"
                                )
                        a0 = a[0]
                        msg.k = [a0, 0.0, cx, 0.0, a0, cy, 0.0, 0.0, 1.0]
                        msg.d = [a[1], a[2], a[3], a[4], 0.0]
                        msg.p = [a0, 0.0, cx, 0.0, 0.0, a0, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
                        msg.distortion_model = "scaramuzza"
                        self.node.get_logger().info(
                            f"loaded scaramuzza camera info: {url} "
                            f"(a0={a0:.3f} a2={a[2]:.6g} a3={a[3]:.6g} a4={a[4]:.6g})"
                        )
                        return msg

                msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
                msg.d = [float(value) for value in distortion]
                msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
                msg.distortion_model = "plumb_bob"
                self.node.get_logger().info(f"loaded camera info: {url}")
                return msg
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().error(f"failed to parse camera info, fallback to empty: {exc}")

        msg.k = [0.0] * 9
        msg.d = [0.0] * 5
        msg.p = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = ""
        return msg
