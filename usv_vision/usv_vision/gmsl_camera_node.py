import threading
import time

import cv2
import rclpy
from rclpy.node import Node

from .camera_output import CameraOutput


class GmslCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("gmsl_camera_node")
        self.declare_parameter("camera_name", "front_gmsl")
        self.declare_parameter("device_path", "/dev/video4")
        self.declare_parameter("frame_id", "front_gmsl_link")
        self.declare_parameter("camera_info_url", "")
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1536)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("pixel_format", "UYVY")
        self.declare_parameter("source_type", "v4l2_gmsl")
        self.declare_parameter("gstreamer_pipeline", "")
        self.declare_parameter("fallback_v4l2", True)
        self.declare_parameter("put_timestamp", False)
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("camera_info_topic", "camera_info")
        self.declare_parameter("driver_status_topic", "driver_status")
        self.declare_parameter("frame_timing_topic", "frame_timing_json")
        self.declare_parameter("image_qos_reliability", "best_effort")
        self.declare_parameter("image_qos_depth", 1)

        self.camera_name = str(self.get_parameter("camera_name").value)
        self.device_path = str(self.get_parameter("device_path").value).strip()
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.camera_info_url = str(self.get_parameter("camera_info_url").value).strip()
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.pixel_format = str(self.get_parameter("pixel_format").value).strip()
        self.source_type = str(self.get_parameter("source_type").value).strip()
        self.gstreamer_pipeline = str(self.get_parameter("gstreamer_pipeline").value).strip()
        self.fallback_v4l2 = bool(self.get_parameter("fallback_v4l2").value)

        self.output = CameraOutput(
            self,
            frame_id=self.frame_id,
            camera_info_url=self.camera_info_url,
            source_type=self.source_type,
            image_topic=str(self.get_parameter("image_topic").value),
            camera_info_topic=str(self.get_parameter("camera_info_topic").value),
            timing_topic=str(self.get_parameter("frame_timing_topic").value),
            status_topic=str(self.get_parameter("driver_status_topic").value),
            qos_reliability=str(self.get_parameter("image_qos_reliability").value),
            qos_depth=int(self.get_parameter("image_qos_depth").value),
            put_timestamp=bool(self.get_parameter("put_timestamp").value),
        )

        self._running = True
        threading.Thread(target=self._run, daemon=True).start()
        self.get_logger().info(
            f"gmsl_camera_node started: camera={self.camera_name}, device={self.device_path}"
        )

    def _default_pipeline(self) -> str:
        fps_num = int(self.fps) if self.fps > 0 else 30
        return (
            f"v4l2src device={self.device_path} ! "
            f"video/x-raw,format={self.pixel_format},width={self.width},height={self.height},framerate={fps_num}/1 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

    def _open(self):
        pipeline = self.gstreamer_pipeline or self._default_pipeline()
        self.get_logger().info(f"opening GStreamer pipeline: {pipeline}")
        capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if capture.isOpened() or not self.fallback_v4l2:
            return capture, "gstreamer"
        capture.release()
        fallback_capture = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if fallback_capture.isOpened():
            fallback_capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
            fallback_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
            fallback_capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        return fallback_capture, "v4l2"

    def _run(self) -> None:
        while self._running:
            capture, backend = self._open()
            if not capture.isOpened():
                self.output.publish_status(
                    "error",
                    "open_failed",
                    {
                        "camera_name": self.camera_name,
                        "device_path": self.device_path,
                        "width": self.width,
                        "height": self.height,
                        "fps": self.fps,
                    },
                )
                time.sleep(0.5)
                continue

            self.output.publish_status(
                "playing",
                backend,
                {
                    "camera_name": self.camera_name,
                    "device_path": self.device_path,
                    "width": self.width,
                    "height": self.height,
                    "fps": self.fps,
                },
            )
            while self._running:
                ok, frame = capture.read()
                if not ok or frame is None:
                    break
                stamp = self.get_clock().now().to_msg()
                self.output.publish_frame(frame, stamp, backend, self.device_path)

            capture.release()
            self.output.publish_status("restarting", backend, {"camera_name": self.camera_name})
            time.sleep(0.5)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GmslCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
