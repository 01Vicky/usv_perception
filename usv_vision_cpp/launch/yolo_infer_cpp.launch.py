"""yolo_infer_cpp.launch.py —— 单独起 C++ 推理节点（实验用）

用法：
  # 只起这一个节点（采集/推流由别的 launch 负责）
  ros2 launch usv_vision_cpp yolo_infer_cpp.launch.py sensor:=front

  # 全链路自测：采集(python 包) + 这个 cpp 节点，但不要同时起 usv_vision 的 yolo_infer_node
  ros2 launch usv_vision jetson_gmsl_full.launch.py cameras:="['front']" inference:=false stream_mode:=off
  ros2 launch usv_vision_cpp yolo_infer_cpp.launch.py sensor:=front

  # 想和 Python 版并排对比时，把输出话题错开，避免同话题双发布
  ros2 launch usv_vision_cpp yolo_infer_cpp.launch.py sensor:=front \
      detections_topic:=/perception/camera/front/cpp_detections \
      objects_topic:=/perception/camera/front/cpp_objects
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("usv_vision_cpp")
    default_params = os.path.join(pkg_share, "config", "yolo_infer_cpp.yaml")

    args = [
        DeclareLaunchArgument("sensor", default_value="front"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("engine_path", default_value=""),
        DeclareLaunchArgument("detections_topic", default_value=""),
        DeclareLaunchArgument("objects_topic", default_value=""),
        DeclareLaunchArgument("angle_model", default_value="scaramuzza"),
        DeclareLaunchArgument("publish_annotated", default_value="false"),
    ]

    sensor = LaunchConfiguration("sensor")
    overrides = {
        "sensor_name": sensor,
        "camera_id": sensor,
        "angle_model": LaunchConfiguration("angle_model"),
    }
    # 只在显式给了值的时候才覆盖，空串表示沿用 yaml
    for name in ("engine_path", "detections_topic", "objects_topic"):
        overrides[name] = LaunchConfiguration(name)

    return LaunchDescription(
        args
        + [
            Node(
                package="usv_vision_cpp",
                executable="yolo_infer_cpp_node",
                name=["yolo_infer_cpp_", sensor],
                output="screen",
                parameters=[LaunchConfiguration("params_file"), overrides],
                arguments=["--ros-args", "--log-level", "info"],
            )
        ]
    )
