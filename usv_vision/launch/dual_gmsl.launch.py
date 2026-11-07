from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share_dir = get_package_share_directory("usv_vision")
    default_front_config = os.path.join(share_dir, "config", "gmsl_camera.yaml")
    default_rear_config = os.path.join(share_dir, "config", "gmsl_camera_rear.yaml")

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
    front_namespace = DeclareLaunchArgument(
        "front_namespace",
        default_value="front_gmsl",
        description="Namespace for the front GMSL camera",
    )
    rear_namespace = DeclareLaunchArgument(
        "rear_namespace",
        default_value="rear_gmsl",
        description="Namespace for the rear GMSL camera",
    )

    return LaunchDescription(
        [
            front_config,
            rear_config,
            front_namespace,
            rear_namespace,
            Node(
                package="usv_vision",
                executable="gmsl_camera_node",
                name="gmsl_camera_node",
                namespace=LaunchConfiguration("front_namespace"),
                output="screen",
                parameters=[LaunchConfiguration("front_config")],
            ),
            Node(
                package="usv_vision",
                executable="gmsl_camera_node",
                name="gmsl_camera_node",
                namespace=LaunchConfiguration("rear_namespace"),
                output="screen",
                parameters=[LaunchConfiguration("rear_config")],
            ),
        ]
    )
