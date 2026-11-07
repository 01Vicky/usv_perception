from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    启动多个GMSL相机节点，测试硬件同步触发

    当前配置：
    - 相机0: /dev/video0 (物理相机1)
    - 相机1: /dev/video1 (物理相机2)

    验证同步：
    python3 /media/nvidia/nvme0n1p1/USV_ROS/scripts/check_multi_sync.py
    """
    share_dir = get_package_share_directory("usv_vision_gmsl_cpp")

    # 注：节点直接发布绝对话题 /sensors/camera/{sensor_name}/image_raw，
    # 话题名由 sensor_name 决定，无需 remapping。

    # 相机0节点 (video0)
    camera0_node = Node(
        package="usv_vision_gmsl_cpp",
        executable="gmsl_camera_node_cpp",
        name="gmsl_camera_node_0",
        output="screen",
        parameters=[
            {
                "device_path": "/dev/video0",
                "sensor_name": "camera_0",
                "width": 1920,
                "height": 1536,
                "fps": 30.0,
                "pixel_format": "UYVY",
                "enable_hardware_timestamp": True,
            }
        ],
    )

    # 相机1节点 (video1)
    camera1_node = Node(
        package="usv_vision_gmsl_cpp",
        executable="gmsl_camera_node_cpp",
        name="gmsl_camera_node_1",
        output="screen",
        parameters=[
            {
                "device_path": "/dev/video1",
                "sensor_name": "camera_1",
                "width": 1920,
                "height": 1536,
                "fps": 30.0,
                "pixel_format": "UYVY",
                "enable_hardware_timestamp": True,
            }
        ],
    )

    return LaunchDescription([
        camera0_node,
        camera1_node,
    ])
