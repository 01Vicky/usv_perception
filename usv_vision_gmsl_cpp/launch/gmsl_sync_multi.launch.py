from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    启动多相机硬件同步节点（单进程控制多相机）

    特点：
    - 一个进程初始化多个相机，确保硬件同步触发生效
    - 通过配置文件动态指定相机数量和配置
    - 微秒级时间戳同步

    验证同步：
    python3 /media/nvidia/nvme0n1p1/USV_ROS/scripts/check_multi_sync.py

    修改相机配置：
    编辑 config/gmsl_multi_sync.yaml
    """
    share_dir = get_package_share_directory("usv_vision_gmsl_cpp")
    config_file = os.path.join(share_dir, "config", "gmsl_multi_sync.yaml")

    multi_sync_node = Node(
        package="usv_vision_gmsl_cpp",
        executable="gmsl_multi_camera_sync_node",
        name="gmsl_multi_camera_sync_node",
        output="screen",
        parameters=[config_file]
    )

    return LaunchDescription([
        multi_sync_node,
    ])
