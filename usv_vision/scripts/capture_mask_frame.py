"""Capture a single camera frame to disk, to use as a template for drawing a view mask.

Usage (source the workspace, then):
  ros2 run usv_vision capture_mask_frame \
    --ros-args -p image_topic:=/usv_1/sensors/camera/payload_gimbal/image_raw \
              -p output:=/home/ww/capture_frame.png

Convention note (for the mask you draw later):
  mask pixel == 0 (black)        -> recognition region (keep)
  mask pixel >= mask_ignore_min  -> ignore region (what you paint)
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from usv_vision.common.qos import image_qos


class CaptureMaskFrame(Node):
    def __init__(self) -> None:
        super().__init__("capture_mask_frame")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("output", "capture_frame.png")
        self.declare_parameter("timeout_sec", 10.0)

        topic = str(self.get_parameter("image_topic").value).strip()
        self.output = str(self.get_parameter("output").value).strip() or "capture_frame.png"
        timeout = max(1.0, float(self.get_parameter("timeout_sec").value))

        if not topic:
            self.get_logger().error(
                "image_topic is required, e.g. image_topic:=/usv_1/sensors/camera/payload_gimbal/image_raw"
            )
            raise SystemExit(2)

        self._bridge = CvBridge()
        self._saved = False
        self.create_subscription(Image, topic, self._on_image, image_qos("best_effort", 1))
        self.create_timer(timeout, self._timeout)
        self.get_logger().info(
            f"capturing one frame from {topic} -> {self.output} (timeout {timeout:.0f}s)"
        )

    def _on_image(self, msg: Image) -> None:
        if self._saved:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:  # noqa: BLE001
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if cv2.imwrite(self.output, frame):
            self._saved = True
            self.get_logger().info(f"saved template frame to {self.output}")
        else:
            self.get_logger().error(f"failed to write {self.output}")

    def _timeout(self) -> None:
        if not self._saved:
            self.get_logger().warn("no image received within timeout; exiting")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CaptureMaskFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
