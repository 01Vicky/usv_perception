from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    启动双路GMSL相机节点（C++版本，支持硬件同步时间戳）

    特点：
    - 两路相机使用相同的GMSL SDK实例
    - 硬件同步时间戳，保证多相机时间对齐
    - 可独立配置每路相机的分辨率和设备

    使用方法：
    ros2 launch usv_vision_gmsl_cpp dual_gmsl_cpp.launch.py
    """
    share_dir = get_package_share_directory("usv_vision_gmsl_cpp")
    default_front_config = os.path.join(share_dir, "config", "gmsl_camera_cpp_front.yaml")
    default_rear_config = os.path.join(share_dir, "config", "gmsl_camera_cpp_rear.yaml")

    front_config = DeclareLaunchArgument(
        "front_config",
        default_value=default_front_config,
        description="Parameter file for the front GMSL camera",
    )
    rear_config = DeclareLaunchArgument(
        "rear_config",
        default_value=default_rear_config,
        description="Parameter file for the rear GMSL camera",
    )

    return LaunchDescription(
        [
            front_config,
            rear_config,
            Node(
                package="usv_vision_gmsl_cpp",
                executable="gmsl_camera_node_cpp",
                name="gmsl_camera_front",
                output="screen",
                parameters=[LaunchConfiguration("front_config")],
            ),
            Node(
                package="usv_vision_gmsl_cpp",
                executable="gmsl_camera_node_cpp",
                name="gmsl_camera_rear",
                output="screen",
                parameters=[LaunchConfiguration("rear_config")],
            ),
        ]
    )
