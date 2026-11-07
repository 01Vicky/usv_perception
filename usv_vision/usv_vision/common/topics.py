def camera_topic_base(sensor_name: str) -> str:
    """传感器层：相机数据根路径（相对路径，由 ROS namespace 前缀）。"""
    return f"sensors/camera/{sensor_name.strip('/')}"


def image_topic(sensor_name: str) -> str:
    return f"{camera_topic_base(sensor_name)}/image_raw"


def camera_info_topic(sensor_name: str) -> str:
    return f"{camera_topic_base(sensor_name)}/camera_info"


def algo_image_topic(sensor_name: str) -> str:
    """算法图：采集端按需分发的小尺寸图（默认 640），识别链的输入。"""
    return f"{camera_topic_base(sensor_name)}/image_algo"


def algo_camera_info_topic(sensor_name: str) -> str:
    """算法图对应的 camera_info（内参已按算法图分辨率缩放）。"""
    return f"{camera_topic_base(sensor_name)}/camera_info_algo"


def frame_timing_topic(sensor_name: str) -> str:
    return f"{camera_topic_base(sensor_name)}/frame_timing_json"


def driver_status_topic(sensor_name: str) -> str:
    return f"{camera_topic_base(sensor_name)}/driver_status"


def stream_status_topic(sensor_name: str) -> str:
    """推流状态属传感器/驱动层旁路，与 driver_status 并列，不放 perception 命名空间。"""
    return f"{camera_topic_base(sensor_name)}/stream_status"


def perception_topic_base(sensor_name: str) -> str:
    """感知输出层：camera 模态根路径（相对路径，由 ROS namespace 前缀）。"""
    return f"perception/camera/{sensor_name.strip('/')}"


def vision_objects_topic(sensor_name: str) -> str:
    """AIVisionTargets —— 主输出格式。"""
    return f"{perception_topic_base(sensor_name)}/objects"


def vision_detections_topic(sensor_name: str) -> str:
    """VisionDetectionArray —— 备用输出格式。"""
    return f"{perception_topic_base(sensor_name)}/detections"


def annotated_image_topic(sensor_name: str) -> str:
    """调试可视化图像。"""
    return f"{perception_topic_base(sensor_name)}/annotated"


def infer_status_topic(sensor_name: str) -> str:
    """推理节点状态。"""
    return f"{perception_topic_base(sensor_name)}/status"


# 兼容旧名（deprecated，后续逐步移除）
vision_targets_topic = vision_objects_topic
