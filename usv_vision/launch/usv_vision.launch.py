from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share_dir = get_package_share_directory("usv_vision")
    default_gmsl_config = os.path.join(share_dir, "config", "gmsl_camera.yaml")
    default_yolo_config = os.path.join(share_dir, "config", "yolo_infer.yaml")

    gmsl_config = DeclareLaunchArgument(
        "gmsl_config",
        default_value=default_gmsl_config,
        description="Parameter file for GMSL camera node",
    )
    yolo_config = DeclareLaunchArgument(
        "yolo_config",
        default_value=default_yolo_config,
        description="Parameter file for YOLO inference node",
    )
    namespace = DeclareLaunchArgument(
        "namespace",
        default_value="front_gmsl",
        description="Namespace for the vision pipeline",
    )
    enable_yolo = DeclareLaunchArgument(
        "enable_yolo",
        default_value="false",
        description="Whether to start the inference node",
    )

    return LaunchDescription(
        [
            gmsl_config,
            yolo_config,
            namespace,
            enable_yolo,
            Node(
                package="usv_vision",
                executable="gmsl_camera_node",
                name="gmsl_camera_node",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[LaunchConfiguration("gmsl_config")],
            ),
            Node(
                package="usv_vision",
                executable="yolo_infer_node",
                name="yolo_infer_node",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                # 本 launch 用的是 Python 采集节点（只发 image_raw，不做按需分发），
                # 所以显式关闭算法图订阅，避免订 image_algo 收不到数据
                parameters=[LaunchConfiguration("yolo_config"), {"use_algo_image": False}],
                condition=IfCondition(LaunchConfiguration("enable_yolo")),
            ),
        ]
    )
