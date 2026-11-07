from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    启动C++版本的GMSL相机节点（支持硬件时间戳）

    所有相机参数（device_path、sensor_name、分辨率、enable_hardware_timestamp、
    auto_detect_resolution 等）统一在 config yaml 中配置，通过 config: 指定配置文件。

    使用方法：
    1. 单相机前置（默认 gmsl_camera_cpp_front.yaml）：
       ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py

    2. 指定配置文件（后置 / 自定义设备 / 关硬件时间戳等，均改 yaml 内对应字段）：
       ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py \
           config:=$(ros2 pkg prefix usv_vision_gmsl_cpp)/share/usv_vision_gmsl_cpp/config/gmsl_camera_cpp_rear.yaml

    说明：换设备请编辑 config 里的 device_path，而不是命令行覆盖
    （本 launch 不提供 device_path 等命令行覆盖参数，避免“看似生效实则被忽略”）。
    """
    share_dir = get_package_share_directory("usv_vision_gmsl_cpp")
    default_config = os.path.join(share_dir, "config", "gmsl_camera_cpp_front.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Parameter file for GMSL camera (所有相机参数都在此文件内配置)",
    )

    node = Node(
        package="usv_vision_gmsl_cpp",
        executable="gmsl_camera_node_cpp",
        name="gmsl_camera_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
        ],
    )

    return LaunchDescription(
        [
            config_arg,
            node,
        ]
    )
