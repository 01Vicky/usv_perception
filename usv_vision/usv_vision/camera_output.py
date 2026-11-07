import json
import time

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common.camera_info import CameraInfoLoader
from .common.qos import image_qos
from .common.time_utils import ros_delta_ms, ros_time_to_dict


class CameraOutput:
    def __init__(
        self,
        node,
        frame_id: str,
        camera_info_url: str,
        source_type: str,
        image_topic: str = "image_raw",
        camera_info_topic: str = "camera_info",
        timing_topic: str = "frame_timing_json",
        status_topic: str = "driver_status",
        qos_reliability: str = "best_effort",
        qos_depth: int = 1,
        put_timestamp: bool = False,
    ) -> None:
        self.node = node
        self.frame_id = frame_id
        self.source_type = source_type
        self.put_timestamp = put_timestamp
        qos = image_qos(qos_reliability, qos_depth)
        self.pub_img = node.create_publisher(Image, image_topic, qos)
        self.pub_info = node.create_publisher(CameraInfo, camera_info_topic, qos)
        self.pub_timing = node.create_publisher(String, timing_topic, 10)
        self.pub_status = node.create_publisher(String, status_topic, 10)
        self.bridge = CvBridge()
        self.info_loader = CameraInfoLoader(node, frame_id, camera_info_url)

    def publish_status(self, state: str, detail: str = "", extra: dict | None = None) -> None:
        payload = {
            "state": state,
            "detail": detail,
            "source_type": self.source_type,
            "stamp": ros_time_to_dict(self.node.get_clock().now().to_msg()),
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.pub_status.publish(msg)

    def publish_frame(self, frame: np.ndarray, capture_stamp, backend: str, source: str = "") -> None:
        if frame is None:
            return

        if self.put_timestamp:
            cv2.putText(
                frame,
                time.strftime("%H:%M:%S"),
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

        height, width = frame.shape[:2]
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = capture_stamp
        img_msg.header.frame_id = self.frame_id
        self.pub_img.publish(img_msg)

        info_msg = self.info_loader.get(width, height)
        info_msg.header.stamp = capture_stamp
        info_msg.header.frame_id = self.frame_id
        self.pub_info.publish(info_msg)

        publish_stamp = self.node.get_clock().now().to_msg()
        timing = {
            "frame_id": self.frame_id,
            "source_type": self.source_type,
            "backend": backend,
            "source": source,
            "image": {
                "width": int(width),
                "height": int(height),
            },
            "timestamps": {
                "capture_stamp_ros": ros_time_to_dict(capture_stamp),
                "publish_stamp_ros": ros_time_to_dict(publish_stamp),
            },
            "latency_ms": {
                "driver_latency_ms": ros_delta_ms(capture_stamp, publish_stamp),
            },
        }
        timing_msg = String()
        timing_msg.data = json.dumps(timing, ensure_ascii=True)
        self.pub_timing.publish(timing_msg)
