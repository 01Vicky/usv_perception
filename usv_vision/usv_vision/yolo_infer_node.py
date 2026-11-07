import json
import math
from dataclasses import dataclass

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from usv_interfaces.msg import AIVisionTargets
from usv_interfaces.msg import AIVisionTarget
from usv_interfaces.msg import PipelineTimestamp
from usv_interfaces.msg import VisionDetection
from usv_interfaces.msg import VisionDetectionArray

from .common.mask import apply_input_mask
from .common.mask import build_ignore_mask
from .common.mask import parse_polygons_json
from .common.topics import algo_camera_info_topic
from .common.topics import algo_image_topic
from .common.topics import annotated_image_topic
from .common.topics import camera_info_topic
from .common.topics import image_topic
from .common.topics import infer_status_topic
from .common.topics import vision_detections_topic
from .common.topics import vision_objects_topic
from .common.qos import image_qos


@dataclass
class DetectionCandidate:
    cls_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2.0


@dataclass
class SmoothedTrack:
    track_id: int
    cls_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    hits: int
    last_seen_sec: float
    updated_in_frame: bool = False

    def to_candidate(self) -> DetectionCandidate:
        return DetectionCandidate(
            cls_id=self.cls_id,
            class_name=self.class_name,
            confidence=self.confidence,
            x1=self.x1,
            y1=self.y1,
            x2=self.x2,
            y2=self.y2,
        )


class YoloInferNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_infer_node")
        self.declare_parameter("sensor_name", "front")
        self.declare_parameter("camera_id", "")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("vision_detections_topic", "")
        self.declare_parameter("vision_objects_topic", "")
        self.declare_parameter("annotated_topic", "")
        self.declare_parameter("infer_status_topic", "")
        self.declare_parameter("engine_path", "")
        self.declare_parameter("model_backend", "tensorrt")
        self.declare_parameter("infer_width", 640)
        self.declare_parameter("infer_height", 640)
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("min_box_width_px", 12)
        self.declare_parameter("min_box_height_px", 8)
        self.declare_parameter("min_box_area_ratio", 0.0003)
        self.declare_parameter("dedup_iou_threshold", 0.55)
        self.declare_parameter("dedup_min_iou", 0.10)
        self.declare_parameter("dedup_center_dist_ratio", 0.03)
        self.declare_parameter("dedup_containment_threshold", 0.85)
        self.declare_parameter("dedup_larger_box_conf_margin", 0.08)
        self.declare_parameter("bbox_expand_ratio", 0.06)
        self.declare_parameter("enable_temporal_smoothing", True)
        self.declare_parameter("track_iou_threshold", 0.30)
        self.declare_parameter("track_max_age_sec", 0.35)
        self.declare_parameter("track_smoothing_alpha", 0.65)
        self.declare_parameter("track_min_hits", 2)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("startup_report_hz", 0.5)
        self.declare_parameter("camera_horizontal_fov_deg", 150.0)
        self.declare_parameter("max_det", 50)
        self.declare_parameter("publish_legacy_targets", True)
        self.declare_parameter("class_name_allowlist", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("class_id_allowlist", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("enable_mask", False)
        self.declare_parameter("mask_path", "")
        self.declare_parameter("mask_ignore_min", 1)
        self.declare_parameter("mask_polygons_json", "")
        self.declare_parameter("mask_fill", 114)
        # 采集端"按需分发"后，识别链默认订阅 image_algo（小图），不再吃全分辨率 image_raw
        self.declare_parameter("use_algo_image", True)
        # 坐标/阈值基准分辨率：0 表示以订阅到的图像尺寸为准（改造前行为）。
        # 填 1920/1536 时，下游 bbox 坐标与 min_box_* 阈值仍按原生分辨率语义解释，
        # 内部自动换算到实际输入尺寸，下游消费者无感。
        self.declare_parameter("reference_width", 0)
        self.declare_parameter("reference_height", 0)

        self.sensor_name = str(self.get_parameter("sensor_name").value).strip() or "front"
        configured_camera_id = str(self.get_parameter("camera_id").value).strip()
        self.camera_id = configured_camera_id or self.sensor_name
        self.use_algo_image = bool(self.get_parameter("use_algo_image").value)
        # 默认话题：算法图（采集端分发的小图）优先；驱动未开 algo 分发时把 use_algo_image 置 false 回退
        default_image_topic = (
            algo_image_topic(self.sensor_name) if self.use_algo_image else image_topic(self.sensor_name)
        )
        default_camera_info_topic = (
            algo_camera_info_topic(self.sensor_name) if self.use_algo_image else camera_info_topic(self.sensor_name)
        )
        self.image_topic = str(self.get_parameter("image_topic").value).strip() or default_image_topic
        self.camera_info_topic = (
            str(self.get_parameter("camera_info_topic").value).strip() or default_camera_info_topic
        )
        self.reference_width = int(self.get_parameter("reference_width").value)
        self.reference_height = int(self.get_parameter("reference_height").value)
        self.vision_detections_topic = (
            str(self.get_parameter("vision_detections_topic").value).strip()
            or vision_detections_topic(self.sensor_name)
        )
        self.vision_objects_topic = (
            str(self.get_parameter("vision_objects_topic").value).strip() or vision_objects_topic(self.sensor_name)
        )
        self.annotated_topic = (
            str(self.get_parameter("annotated_topic").value).strip() or annotated_image_topic(self.sensor_name)
        )
        status_topic = (
            str(self.get_parameter("infer_status_topic").value).strip() or infer_status_topic(self.sensor_name)
        )
        self.infer_status_topic = status_topic
        self.engine_path = str(self.get_parameter("engine_path").value).strip()
        configured_backend = str(self.get_parameter("model_backend").value).strip()
        self.model_backend = self._detect_backend(self.engine_path) or configured_backend or "unknown"
        self.infer_width = int(self.get_parameter("infer_width").value)
        self.infer_height = int(self.get_parameter("infer_height").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.min_box_width_px = int(self.get_parameter("min_box_width_px").value)
        self.min_box_height_px = int(self.get_parameter("min_box_height_px").value)
        self.min_box_area_ratio = float(self.get_parameter("min_box_area_ratio").value)
        self.dedup_iou_threshold = float(self.get_parameter("dedup_iou_threshold").value)
        self.dedup_min_iou = float(self.get_parameter("dedup_min_iou").value)
        self.dedup_center_dist_ratio = float(self.get_parameter("dedup_center_dist_ratio").value)
        self.dedup_containment_threshold = float(self.get_parameter("dedup_containment_threshold").value)
        self.dedup_larger_box_conf_margin = float(self.get_parameter("dedup_larger_box_conf_margin").value)
        self.bbox_expand_ratio = float(self.get_parameter("bbox_expand_ratio").value)
        self.enable_temporal_smoothing = bool(self.get_parameter("enable_temporal_smoothing").value)
        self.track_iou_threshold = float(self.get_parameter("track_iou_threshold").value)
        self.track_max_age_sec = float(self.get_parameter("track_max_age_sec").value)
        self.track_smoothing_alpha = float(self.get_parameter("track_smoothing_alpha").value)
        self.track_min_hits = int(self.get_parameter("track_min_hits").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.camera_horizontal_fov_deg = float(self.get_parameter("camera_horizontal_fov_deg").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.publish_legacy_targets = bool(self.get_parameter("publish_legacy_targets").value)
        class_name_allowlist_param = self.get_parameter_or(
            "class_name_allowlist",
            Parameter("class_name_allowlist", type_=Parameter.Type.STRING_ARRAY, value=[]),
        )
        class_id_allowlist_param = self.get_parameter_or(
            "class_id_allowlist",
            Parameter("class_id_allowlist", type_=Parameter.Type.INTEGER_ARRAY, value=[]),
        )
        self.class_name_allowlist = {
            str(item).strip().lower()
            for item in list(class_name_allowlist_param.value)
            if str(item).strip()
        }
        self.class_id_allowlist = {int(item) for item in list(class_id_allowlist_param.value)}
        self.enable_mask = bool(self.get_parameter("enable_mask").value)
        self.mask_path = str(self.get_parameter("mask_path").value).strip()
        self.mask_ignore_min = max(1, int(self.get_parameter("mask_ignore_min").value))
        self.mask_fill = int(self.get_parameter("mask_fill").value)
        self.mask_polygons = parse_polygons_json(
            str(self.get_parameter("mask_polygons_json").value)
        )
        self._inference_enabled = bool(self.engine_path)
        self._model = None
        self._model_error = ""
        self._bridge = CvBridge()
        self._latest_camera_info = None
        self._tracks = []
        self._next_track_id = 1
        self._frame_count = 0
        self._timing_log_hz = 0.2  # log timing every 5 seconds
        self._last_timing_log_time = 0.0
        self._warned_no_camera_info = False
        self._debug_frame_skip = 1  # publish debug image every N frames
        self._mask_ignore = None      # np.bool (h,w) or None
        self._mask_shape = None       # (h, w) the mask was built for
        self._mask_failed = False

        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._detections_pub = self.create_publisher(VisionDetectionArray, self.vision_detections_topic, 10)
        self._targets_pub = None
        if self.publish_legacy_targets:
            self._targets_pub = self.create_publisher(AIVisionTargets, self.vision_objects_topic, 10)
        self._annotated_pub = None
        if self.publish_debug_image:
            qos = image_qos("best_effort", 1)
            self._annotated_pub = self.create_publisher(Image, self.annotated_topic, qos)
        if self._inference_enabled:
            self._try_load_model()
            qos = image_qos("best_effort", 1)
            self.create_subscription(Image, self.image_topic, self._on_image, qos)
            self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, qos)
        self.create_service(Trigger, self._mask_service_name(), self._on_reload_mask)
        hz = max(0.2, float(self.get_parameter("startup_report_hz").value))
        self.create_timer(1.0 / hz, self._report_placeholder_status)
        if not self._inference_enabled:
            self.create_timer(1.0 / hz, self._publish_placeholder_targets)
        self.get_logger().info(
            "yolo_infer_node started. "
            f"inference_enabled={self._inference_enabled}, backend={self.model_backend}"
        )

    def _report_placeholder_status(self) -> None:
        state = "running" if self._model is not None else ("error" if self._model_error else "placeholder")
        detail = (
            "TensorRT inference active"
            if self._model is not None
            else (self._model_error or "YOLO inference implementation not wired yet")
        )
        status = {
            "state": state,
            "detail": detail,
            "sensor_name": self.sensor_name,
            "camera_id": self.camera_id,
            "image_topic": self.image_topic,
            "camera_info_topic": self.camera_info_topic,
            "vision_detections_topic": self.vision_detections_topic,
            "vision_objects_topic": self.vision_objects_topic,
            "annotated_topic": self.annotated_topic,
            "publish_debug_image": self.publish_debug_image,
            "publish_legacy_targets": self.publish_legacy_targets,
            "model_backend": self.model_backend,
            "engine_path": self.engine_path,
            "infer_width": self.infer_width,
            "infer_height": self.infer_height,
            "angle_source": self._angle_source(),
            "min_box_width_px": self.min_box_width_px,
            "min_box_height_px": self.min_box_height_px,
            "min_box_area_ratio": self.min_box_area_ratio,
            "dedup_iou_threshold": self.dedup_iou_threshold,
            "dedup_min_iou": self.dedup_min_iou,
            "dedup_center_dist_ratio": self.dedup_center_dist_ratio,
            "dedup_containment_threshold": self.dedup_containment_threshold,
            "dedup_larger_box_conf_margin": self.dedup_larger_box_conf_margin,
            "bbox_expand_ratio": self.bbox_expand_ratio,
            "enable_temporal_smoothing": self.enable_temporal_smoothing,
            "track_iou_threshold": self.track_iou_threshold,
            "track_max_age_sec": self.track_max_age_sec,
            "track_smoothing_alpha": self.track_smoothing_alpha,
            "track_min_hits": self.track_min_hits,
            "class_name_allowlist": sorted(self.class_name_allowlist),
            "class_id_allowlist": sorted(self.class_id_allowlist),
            "enable_mask": self.enable_mask,
            "mask_path": self.mask_path,
            "mask_polygons_count": len(self.mask_polygons),
            "mask_active": self.enable_mask and self._mask_ignore is not None,
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=True)
        self._status_pub.publish(msg)

    @staticmethod
    def _detect_backend(model_path: str) -> str:
        """Detect the runtime backend from the model file path/extension."""
        import os as _os
        if not model_path:
            return ""
        path = model_path.lower().rstrip("/")
        # OpenVINO: directory containing .xml + .bin
        if _os.path.isdir(model_path):
            dir_path = model_path.rstrip("/")
            has_xml = any(f.endswith(".xml") for f in _os.listdir(dir_path))
            has_bin = any(f.endswith(".bin") for f in _os.listdir(dir_path))
            if has_xml and has_bin:
                return "openvino"
        # TensorRT engine
        if path.endswith(".engine") or path.endswith(".trt"):
            return "tensorrt"
        # ONNX
        if path.endswith(".onnx"):
            return "onnxruntime"
        # PyTorch
        if path.endswith(".pt") or path.endswith(".pth") or path.endswith(".torchscript"):
            return "pytorch"
        # TorchScript
        if path.endswith(".torchscript") or path.endswith(".jit"):
            return "torchscript"
        # CoreML
        if path.endswith(".mlmodel") or path.endswith(".mlpackage"):
            return "coreml"
        # TFLite
        if path.endswith(".tflite"):
            return "tflite"
        return ""

    @staticmethod
    def _backend_human_name(backend: str) -> str:
        names = {
            "openvino": "OpenVINO",
            "tensorrt": "TensorRT",
            "onnxruntime": "ONNX Runtime",
            "pytorch": "PyTorch",
            "torchscript": "TorchScript",
            "coreml": "CoreML",
            "tflite": "TFLite",
        }
        return names.get(backend, backend or "unknown")

    def _try_load_model(self) -> None:
        try:
            from ultralytics import YOLO

            self._model = YOLO(self.engine_path, task="detect")
            backend_label = self._backend_human_name(self.model_backend)
            self.get_logger().info(
                f"loaded YOLO engine: {self.engine_path} (backend={backend_label})"
            )
        except Exception as exc:  # noqa: BLE001
            self._model = None
            self._model_error = f"model_load_failed: {exc}"
            self.get_logger().error(self._model_error)

    def _mask_service_name(self) -> str:
        status = getattr(self, "infer_status_topic", "")
        if status and status.endswith("/status"):
            return status[: -len("/status")] + "/reload_mask"
        return f"perception/camera/{self.sensor_name}/reload_mask"

    def _on_reload_mask(self, request, response):
        del request
        # Parameters may have changed since construction; rebuild from the latest
        # values as well as rereading a mask file overwritten at the same path.
        self.enable_mask = bool(self.get_parameter("enable_mask").value)
        self.mask_path = str(self.get_parameter("mask_path").value).strip()
        self.mask_ignore_min = max(1, int(self.get_parameter("mask_ignore_min").value))
        self.mask_fill = max(0, min(255, int(self.get_parameter("mask_fill").value)))
        self.mask_polygons = parse_polygons_json(
            str(self.get_parameter("mask_polygons_json").value)
        )
        self._mask_ignore = None
        self._mask_shape = None
        self._mask_failed = False
        response.success = True
        response.message = "mask reload scheduled"
        self.get_logger().info("mask reload requested; will rebuild on next frame")
        return response

    def _build_ignore_mask(self, height: int, width: int):
        """Build a bool ignore mask (h, w). Convention: True = ignore region."""
        return build_ignore_mask(
            height,
            width,
            mask_path=self.mask_path,
            mask_ignore_min=self.mask_ignore_min,
            polygons=self.mask_polygons,
        )

    def _apply_input_mask(self, frame):
        """Return a copy of frame with the ignore region filled; original frame untouched."""
        if not self.enable_mask:
            return frame
        if self._mask_failed:
            return frame
        height, width = frame.shape[:2]
        if self._mask_ignore is None or self._mask_shape != (height, width):
            self._mask_ignore = self._build_ignore_mask(height, width)
            self._mask_shape = (height, width)
            if self._mask_ignore is None:
                self._mask_failed = True
                self.get_logger().warn(
                    "view mask enabled but no mask source loaded; running without mask"
                )
                return frame
            self.get_logger().info(
                f"view mask active ({width}x{height}): "
                f"{int(self._mask_ignore.sum())} ignore px"
            )
        return apply_input_mask(frame, self._mask_ignore, self.mask_fill)

    def _on_image(self, msg: Image) -> None:
        if self._model is None:
            return

        try:
            import time as _time
            _t0 = _time.perf_counter()

            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            _t1 = _time.perf_counter()

            results = self._model.predict(
                source=self._apply_input_mask(frame),
                imgsz=[self.infer_height, self.infer_width],
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                max_det=self.max_det,
                verbose=False,
            )
            _t2 = _time.perf_counter()

            detections_msg, legacy_targets_msg, annotated = self._build_outputs(results[0], msg, frame)
            self._detections_pub.publish(detections_msg)
            if self._targets_pub is not None and legacy_targets_msg is not None:
                self._targets_pub.publish(legacy_targets_msg)
            if self._annotated_pub is not None and annotated is not None:
                annotated_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                annotated_msg.header = msg.header
                self._annotated_pub.publish(annotated_msg)

            _t3 = _time.perf_counter()
            self._frame_count += 1

            decode_ms = (_t1 - _t0) * 1000.0
            infer_ms = (_t2 - _t1) * 1000.0
            post_ms = (_t3 - _t2) * 1000.0
            total_ms = (_t3 - _t0) * 1000.0

            # Throttled timing log
            if _t3 - self._last_timing_log_time >= 1.0 / self._timing_log_hz:
                self._last_timing_log_time = _t3
                self.get_logger().info(
                    f"[#{self._frame_count}] "
                    f"decode={decode_ms:.1f}ms infer={infer_ms:.1f}ms post={post_ms:.1f}ms "
                    f"total={total_ms:.1f}ms ({1000.0 / total_ms:.0f}fps) "
                    f"backend={self._backend_human_name(self.model_backend)}"
                )
        except Exception as exc:  # noqa: BLE001
            self._model_error = f"inference_failed: {exc}"
            self.get_logger().error(self._model_error)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _publish_placeholder_targets(self) -> None:
        now = self.get_clock().now().to_msg()
        detections_msg = VisionDetectionArray()
        detections_msg.header.stamp = now
        detections_msg.header.frame_id = self.camera_id
        detections_msg.output_stamp = now
        detections_msg.detections = []
        self._detections_pub.publish(detections_msg)

        if self._targets_pub is not None:
            msg = AIVisionTargets()
            msg.header.stamp = now
            msg.header.frame_id = self.camera_id
            msg.targets_count = 0

            stamp = PipelineTimestamp()
            stamp.name = "placeholder_publish"
            stamp.time_ms = int(self.get_clock().now().nanoseconds / 1_000_000)
            msg.timestamps = [stamp]
            self._targets_pub.publish(msg)

    def _angle_source(self) -> str:
        fx, cx = self._camera_intrinsics()
        if fx is not None and cx is not None:
            return "camera_info"
        return "horizontal_fov_fallback"

    def _camera_intrinsics(self):
        if self._latest_camera_info is None:
            return None, None
        k = list(self._latest_camera_info.k)
        if len(k) < 6:
            return None, None
        fx = float(k[0])
        cx = float(k[2])
        if fx <= 0.0:
            return None, None
        return fx, cx

    def _compute_rel_angle_rad(self, center_x: float, image_width: int) -> float:
        fx, cx = self._camera_intrinsics()
        if fx is not None and cx is not None:
            return math.atan((center_x - cx) / fx)

        if not self._warned_no_camera_info:
            self._warned_no_camera_info = True
            self.get_logger().warn(
                f"No camera_info received for '{self.sensor_name}' — "
                f"angle computed from horizontal_fov={self.camera_horizontal_fov_deg}° (rough estimate). "
                f"Subscribe to /sensors/camera/{self.sensor_name}/camera_info for calibrated angles."
            )
        pixel_offset = center_x - (image_width / 2.0)
        return math.radians(
            (pixel_offset / max(1.0, image_width / 2.0)) * (self.camera_horizontal_fov_deg / 2.0)
        )

    @staticmethod
    def _round2(value: float) -> float:
        return round(float(value), 2)

    @staticmethod
    def _round4(value: float) -> float:
        return round(float(value), 4)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + (float(stamp.nanosec) / 1_000_000_000.0)

    @staticmethod
    def _iou(first: DetectionCandidate, second: DetectionCandidate) -> float:
        inter_x1 = max(first.x1, second.x1)
        inter_y1 = max(first.y1, second.y1)
        inter_x2 = min(first.x2, second.x2)
        inter_y2 = min(first.y2, second.y2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        union_area = first.area + second.area - inter_area
        if union_area <= 0.0:
            return 0.0
        return inter_area / union_area

    @staticmethod
    def _containment(first: DetectionCandidate, second: DetectionCandidate) -> float:
        inter_x1 = max(first.x1, second.x1)
        inter_y1 = max(first.y1, second.y1)
        inter_x2 = min(first.x2, second.x2)
        inter_y2 = min(first.y2, second.y2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        smaller_area = max(1.0, min(first.area, second.area))
        return inter_area / smaller_area

    def _pixel_scale(self, width: int, height: int):
        """实际输入尺寸 → 基准分辨率的缩放系数；基准未配置（0）时返回 (1.0, 1.0)。

        采集端改发小尺寸算法图后，用 reference_width/height 保持下游 bbox 坐标与
        min_box_* 阈值的"原生分辨率语义"不变，内部自动换算。
        """
        sx = 1.0
        sy = 1.0
        if self.reference_width > 0 and width > 0:
            sx = float(self.reference_width) / float(width)
        if self.reference_height > 0 and height > 0:
            sy = float(self.reference_height) / float(height)
        return sx, sy

    def _extract_candidates(self, result, image_msg: Image):
        names = result.names if hasattr(result, "names") else getattr(self._model, "names", {})
        boxes = getattr(result, "boxes", None)
        width = int(image_msg.width)
        height = int(image_msg.height)
        area_floor = self.min_box_area_ratio * max(1, width * height)
        # min_box_* 的语义是"基准分辨率下的像素"，换算到当前输入图尺寸（基准取面积比时天然不变）
        scale_x, scale_y = self._pixel_scale(width, height)
        min_width_px = self.min_box_width_px / scale_x if scale_x > 0 else float(self.min_box_width_px)
        min_height_px = self.min_box_height_px / scale_y if scale_y > 0 else float(self.min_box_height_px)
        candidates = []

        if boxes is None or len(boxes) == 0:
            return candidates

        for box in boxes:
            cls_id = int(box.cls[0])
            confidence = float(box.conf[0])
            raw_x1, raw_y1, raw_x2, raw_y2 = map(float, box.xyxy[0])
            x1 = min(max(raw_x1, 0.0), float(width))
            y1 = min(max(raw_y1, 0.0), float(height))
            x2 = min(max(raw_x2, 0.0), float(width))
            y2 = min(max(raw_y2, 0.0), float(height))
            if x2 <= x1 or y2 <= y1:
                continue
            class_name = str(names.get(cls_id, cls_id)) if isinstance(names, dict) else str(names[cls_id])
            if not self._class_allowed(cls_id, class_name):
                continue
            candidate = DetectionCandidate(
                cls_id=cls_id,
                class_name=class_name,
                confidence=confidence,
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
            )
            if candidate.width < min_width_px:
                continue
            if candidate.height < min_height_px:
                continue
            if candidate.area < area_floor:
                continue
            candidates.append(candidate)
        return candidates

    def _deduplicate_candidates(self, candidates, image_width: int, image_height: int):
        if not candidates:
            return []
        deduplicated = []
        frame_diagonal = math.hypot(float(image_width), float(image_height))
        center_dist_limit = self.dedup_center_dist_ratio * frame_diagonal
        for candidate in sorted(candidates, key=lambda item: item.confidence, reverse=True):
            keep = True
            replace_index = None
            for index, accepted in enumerate(deduplicated):
                if candidate.cls_id != accepted.cls_id:
                    continue
                overlap = self._iou(candidate, accepted)
                containment = self._containment(candidate, accepted)
                center_distance = math.hypot(
                    candidate.center_x - accepted.center_x,
                    candidate.center_y - accepted.center_y,
                )
                larger_is_candidate = candidate.area > accepted.area
                confidence_close_enough = candidate.confidence >= (
                    accepted.confidence - self.dedup_larger_box_conf_margin
                )
                if overlap >= self.dedup_iou_threshold:
                    if larger_is_candidate and confidence_close_enough:
                        replace_index = index
                    else:
                        keep = False
                    break
                if containment >= self.dedup_containment_threshold:
                    if larger_is_candidate and confidence_close_enough:
                        replace_index = index
                    else:
                        keep = False
                    break
                if overlap >= self.dedup_min_iou and center_distance <= center_dist_limit:
                    if larger_is_candidate and confidence_close_enough:
                        replace_index = index
                    else:
                        keep = False
                    break
            if replace_index is not None:
                deduplicated[replace_index] = candidate
            elif keep:
                deduplicated.append(candidate)
        return deduplicated

    def _expand_candidate(self, candidate: DetectionCandidate, image_width: int, image_height: int):
        ratio = max(0.0, self.bbox_expand_ratio)
        if ratio <= 0.0:
            return candidate
        expand_x = candidate.width * ratio
        expand_y = candidate.height * ratio
        x1 = max(0.0, candidate.x1 - expand_x)
        y1 = max(0.0, candidate.y1 - expand_y)
        x2 = min(float(image_width), candidate.x2 + expand_x)
        y2 = min(float(image_height), candidate.y2 + expand_y)
        if x2 <= x1 or y2 <= y1:
            return candidate
        return DetectionCandidate(
            cls_id=candidate.cls_id,
            class_name=candidate.class_name,
            confidence=candidate.confidence,
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
        )

    def _expand_candidates(self, candidates, image_width: int, image_height: int):
        return [self._expand_candidate(candidate, image_width, image_height) for candidate in candidates]

    def _smooth_candidates(self, candidates, image_msg: Image):
        if not self.enable_temporal_smoothing:
            return candidates

        now_sec = self._stamp_to_sec(image_msg.header.stamp)
        active_tracks = []
        for track in self._tracks:
            if now_sec - track.last_seen_sec <= self.track_max_age_sec:
                track.updated_in_frame = False
                active_tracks.append(track)
        self._tracks = active_tracks

        unmatched_candidates = list(range(len(candidates)))
        for track in self._tracks:
            best_index = None
            best_iou = 0.0
            track_candidate = track.to_candidate()
            for index in unmatched_candidates:
                candidate = candidates[index]
                if candidate.cls_id != track.cls_id:
                    continue
                overlap = self._iou(track_candidate, candidate)
                if overlap >= self.track_iou_threshold and overlap > best_iou:
                    best_iou = overlap
                    best_index = index
            if best_index is None:
                continue
            candidate = candidates[best_index]
            alpha = min(max(self.track_smoothing_alpha, 0.0), 1.0)
            track.x1 = (alpha * candidate.x1) + ((1.0 - alpha) * track.x1)
            track.y1 = (alpha * candidate.y1) + ((1.0 - alpha) * track.y1)
            track.x2 = (alpha * candidate.x2) + ((1.0 - alpha) * track.x2)
            track.y2 = (alpha * candidate.y2) + ((1.0 - alpha) * track.y2)
            track.confidence = (alpha * candidate.confidence) + ((1.0 - alpha) * track.confidence)
            track.last_seen_sec = now_sec
            track.hits += 1
            track.updated_in_frame = True
            unmatched_candidates.remove(best_index)

        for index in unmatched_candidates:
            candidate = candidates[index]
            self._tracks.append(
                SmoothedTrack(
                    track_id=self._next_track_id,
                    cls_id=candidate.cls_id,
                    class_name=candidate.class_name,
                    confidence=candidate.confidence,
                    x1=candidate.x1,
                    y1=candidate.y1,
                    x2=candidate.x2,
                    y2=candidate.y2,
                    hits=1,
                    last_seen_sec=now_sec,
                    updated_in_frame=True,
                )
            )
            self._next_track_id += 1

        smoothed = []
        for track in self._tracks:
            if not track.updated_in_frame:
                continue
            if track.hits < max(1, self.track_min_hits):
                continue
            smoothed.append(track.to_candidate())
        return smoothed

    def _build_outputs(self, result, image_msg: Image, frame):
        detections = []
        legacy_targets = []
        width = int(image_msg.width)
        height = int(image_msg.height)
        # 输出坐标基准换算：输入可能是算法小图（如 640），下游仍按 reference 分辨率解释 bbox
        scale_x, scale_y = self._pixel_scale(width, height)
        annotated = None
        if self._annotated_pub is not None and self._frame_count % self._debug_frame_skip == 0:
            annotated = frame.copy()
        candidates = self._extract_candidates(result, image_msg)
        candidates = self._deduplicate_candidates(candidates, width, height)
        candidates = self._smooth_candidates(candidates, image_msg)
        candidates = self._expand_candidates(candidates, width, height)

        if candidates:
            for candidate in candidates:
                x1 = int(round(candidate.x1))
                y1 = int(round(candidate.y1))
                x2 = int(round(candidate.x2))
                y2 = int(round(candidate.y2))
                bbox_x = int(x1)
                bbox_y = int(y1)
                bbox_width = int(max(0, x2 - x1))
                bbox_height = int(max(0, y2 - y1))
                # 对外发布的坐标换算到基准分辨率（内部画图仍用输入图坐标）
                out_x = int(round(bbox_x * scale_x))
                out_y = int(round(bbox_y * scale_y))
                out_width = int(round(bbox_width * scale_x))
                out_height = int(round(bbox_height * scale_y))

                center_x = (x1 + x2) / 2.0
                azimuth = self._compute_rel_angle_rad(center_x, width)
                rel_ang_deg = math.degrees(azimuth)
                conf_rounded = self._round2(candidate.confidence)
                azimuth_rounded = self._round4(azimuth)
                rel_ang_deg_rounded = self._round2(rel_ang_deg)

                detection = VisionDetection()
                detection.camera_id = self.camera_id
                detection.class_name = candidate.class_name
                detection.class_id = candidate.cls_id
                detection.azimuth = azimuth_rounded
                detection.distance_predict = float("nan")
                detection.size_w = float("nan")
                detection.size_h = float("nan")
                detection.pixel_width = float(out_width)
                detection.confidence = conf_rounded
                detections.append(detection)

                if self._targets_pub is not None:
                    target = AIVisionTarget()
                    target.class_name = candidate.class_name
                    target.confidence = conf_rounded
                    target.bbox_x = out_x
                    target.bbox_y = out_y
                    target.bbox_width = out_width
                    target.bbox_height = out_height
                    target.rel_ang_deg = rel_ang_deg_rounded
                    legacy_targets.append(target)
                if annotated is not None:
                    self._draw_detection(
                        annotated,
                        candidate.class_name,
                        conf_rounded,
                        bbox_x,
                        bbox_y,
                        bbox_width,
                        bbox_height,
                        rel_ang_deg_rounded,
                        cls_id=candidate.cls_id,
                    )

        detections_msg = VisionDetectionArray()
        detections_msg.header.stamp = image_msg.header.stamp
        detections_msg.header.frame_id = self.camera_id
        detections_msg.output_stamp = self.get_clock().now().to_msg()
        detections_msg.detections = detections

        legacy_msg = None
        if self._targets_pub is not None:
            legacy_msg = AIVisionTargets()
            legacy_msg.header.stamp = image_msg.header.stamp
            legacy_msg.header.frame_id = self.camera_id
            legacy_msg.targets_count = len(legacy_targets)
            legacy_msg.targets = legacy_targets

            stamp = PipelineTimestamp()
            stamp.name = "inference_publish"
            stamp.time_ms = int(self.get_clock().now().nanoseconds / 1_000_000)
            legacy_msg.timestamps = [stamp]
        return detections_msg, legacy_msg, annotated

    def _class_allowed(self, cls_id: int, class_name: str) -> bool:
        if not self.class_name_allowlist and not self.class_id_allowlist:
            return True
        if cls_id in self.class_id_allowlist:
            return True
        return class_name.strip().lower() in self.class_name_allowlist

    # Per-class BGR colors (distinct, high-visibility)
    _CLASS_COLORS = [
        (0, 140, 255),   # orange
        (0, 255, 0),     # lime
        (255, 0, 0),     # blue
        (0, 255, 255),   # yellow
        (255, 0, 255),   # magenta
        (255, 255, 0),   # cyan
        (0, 165, 255),   # orange-gold
        (128, 0, 255),   # hot pink
        (0, 128, 255),   # orange-red
        (255, 128, 0),   # sky blue
    ]

    @classmethod
    def _class_color(cls, cls_id: int):
        return cls._CLASS_COLORS[cls_id % len(cls._CLASS_COLORS)]

    def _draw_detection(
        self,
        image,
        class_name: str,
        confidence: float,
        bbox_x: int,
        bbox_y: int,
        bbox_width: int,
        bbox_height: int,
        rel_ang_deg: float,
        cls_id: int = 0,
    ) -> None:
        h, w = image.shape[:2]
        scale = max(1.0, min(w, h) / 720.0)
        thick = max(2, int(2.5 * scale))
        font = cv2.FONT_HERSHEY_SIMPLEX
        fs = 0.50 * scale  # font scale
        ft = max(1, int(1.5 * scale))  # font thickness

        x1, y1 = bbox_x, bbox_y
        x2, y2 = bbox_x + bbox_width, bbox_y + bbox_height
        color = self._class_color(cls_id)
        cx, cy = int(round((x1 + x2) / 2.0)), int(round((y1 + y2) / 2.0))

        # ── Box ──
        cv2.rectangle(image, (x1, y1), (x2, y2), color, thick)

        # ── Top label: filled bar above box ──
        label = f"{class_name} {confidence:.2f}"
        (tw, th), _ = cv2.getTextSize(label, font, fs, ft)
        bar_h = th + 8
        bar_top = y1 - bar_h
        if bar_top < 0:  # push inside box if too close to top
            bar_top = y1
            y1_shift = bar_h
        else:
            y1_shift = 0
        cv2.rectangle(image, (x1, bar_top), (x1 + tw + 8, bar_top + bar_h), color, -1)
        cv2.putText(image, label, (x1 + 4, bar_top + th + 2), font, fs,
                    (255, 255, 255), ft, cv2.LINE_AA)

        # ── Bottom info: center + angle ──
        info = f"({cx},{cy}) {rel_ang_deg:+.1f}deg"
        (iw, ih), _ = cv2.getTextSize(info, font, fs * 0.85, ft)
        info_y = y2 + ih + 6
        if info_y > h - 4:
            info_y = y2 - 4  # pull inside box if too close to bottom
        cv2.putText(image, info, (x1, info_y), font, fs * 0.85, color, ft, cv2.LINE_AA)

        # ── Center crosshair ──
        cs = max(5, int(7 * scale))
        cv2.line(image, (cx - cs, cy), (cx + cs, cy), color, thick)
        cv2.line(image, (cx, cy - cs), (cx, cy + cs), color, thick)
        cv2.circle(image, (cx, cy), max(2, int(3 * scale)), color, -1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
