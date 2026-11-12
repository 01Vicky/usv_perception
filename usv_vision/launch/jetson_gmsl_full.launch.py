"""
jetson_gmsl_full.launch.py  ——  Jetson GMSL 全链路一键启动

链路 (每路相机):
  GMSL 相机 → C++ 驱动（一次成帧、按需分发）
    ├→ /sensors/camera/{name}/image_algo   (算法尺寸, 默认640, best_effort) → yolo_infer_node
    │                                                        → /perception/camera/{name}/detections
    ├→ /sensors/camera/{name}/image_raw    (全分辨率)         → rtsp_stream_node → rtsp://.../{cam_n}
    └→ 观察：rviz 订阅 image_algo（默认 QoS 即可），或订阅 annotated 看检测框叠加

用法:
  ros2 launch usv_vision jetson_gmsl_full.launch.py                           # 前后双路
  ros2 launch usv_vision jetson_gmsl_full.launch.py cameras:="['front']"       # 单路
  ros2 launch usv_vision jetson_gmsl_full.launch.py cameras:="['front','rear','side']"  # 三路
  ros2 launch usv_vision jetson_gmsl_full.launch.py inference:=false streaming:=false   # 仅采集
  ros2 launch usv_vision jetson_gmsl_full.launch.py algo_stream:=false        # 识别退回订阅 image_raw
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def camera_nodes(context, gmsl_share, vision_share):
    """动态生成每路相机的采集/推理/推流节点"""
    cameras_str = LaunchConfiguration("cameras").perform(context)
    inference_enabled = LaunchConfiguration("inference").perform(context) == "true"
    # 推流模式：internal（C++ 采集节点内置编码推流，默认）| python（旧 rtsp_stream_node）| off
    # 兼容旧参数：streaming:=true → python，streaming:=false → off
    stream_mode = LaunchConfiguration("stream_mode").perform(context).strip()
    _legacy_streaming = LaunchConfiguration("streaming").perform(context).strip()
    if _legacy_streaming == "true":
        stream_mode = "python"
    elif _legacy_streaming == "false":
        stream_mode = "off"
    if stream_mode not in ("internal", "python", "off"):
        stream_mode = "internal"
    algo_stream_enabled = LaunchConfiguration("algo_stream").perform(context) == "true"
    # 算法图传输格式：raw（默认，sensor_msgs/Image）| jpeg（压缩 CompressedImage，已实现、默认不用）
    # jpeg 三路实测帧率反而降（解码多 3.5~3.6ms/帧），重开条件=解码挪到 nvjpeg
    algo_transport = LaunchConfiguration("algo_transport").perform(context).strip().lower() or "raw"
    if algo_transport not in ("raw", "jpeg"):
        algo_transport = "raw"

    # 解析相机列表: "['front', 'rear', 'side']" → ['front', 'rear', 'side']
    cameras = [c.strip().strip("'\"") for c in cameras_str.strip("[]").split(",")]

    nodes = []
    for i, cam in enumerate(cameras):
        # ── GMSL 采集 ──
        cfg_file = os.path.join(gmsl_share, "config", f"gmsl_camera_cpp_{cam}.yaml")
        if not os.path.exists(cfg_file):
            cfg_file = os.path.join(gmsl_share, "config", "gmsl_camera_template.yaml")

        nodes.append(Node(
            package="usv_vision_gmsl_cpp",
            executable="gmsl_camera_node_cpp",
            name=f"gmsl_camera_{cam}",
            output="screen",
            parameters=[cfg_file, {
                "sensor_name": cam,
                # 算法图传输格式（raw|jpeg）；jpeg 已实现，默认 raw
                "algo_transport": algo_transport,
                # 采集端按需分发：识别链吃 image_algo（小图），推流继续吃 image_raw（全分辨率）
                "enable_algo_stream": algo_stream_enabled,
                # 内置推流：由 launch 统一决定地址（cam1/cam2/... 按相机顺序），yaml 只做覆盖
                "enable_internal_stream": (stream_mode == "internal"),
                "stream_rtsp_url": f"rtsp://127.0.0.1:8554/cam{i+1}" if stream_mode == "internal" else "",
            }],
        ))

        # ── TensorRT 推理 ──
        if inference_enabled:
            nodes.append(Node(
                package="usv_vision",
                executable="yolo_infer_node",
                name=f"yolo_infer_{cam}",
                output="screen",
                parameters=[os.path.join(vision_share, "config", "yolo_infer.yaml"),
                            {"sensor_name": cam, "camera_id": cam,
                             "use_algo_image": algo_stream_enabled,
                             # 与采集端同源：algo_transport=jpeg 时订阅 <algo_topic>/compressed
                             "use_compressed_algo": (algo_transport == "jpeg")}],
            ))

        # ── RTSP 推流（仅 legacy python 模式；internal 模式在采集节点内部完成）──
        if stream_mode == "python":
            rtsp_url = f"rtsp://127.0.0.1:8554/cam{i+1}"
            nodes.append(Node(
                package="usv_vision",
                executable="rtsp_stream_node",
                name=f"rtsp_stream_{cam}",
                output="screen",
                parameters=[os.path.join(vision_share, "config", "rtsp_stream.yaml"),
                            {"sensor_name": cam, "camera_id": cam, "rtsp_url": rtsp_url}],
            ))

    return nodes


def generate_launch_description():
    vision_share = get_package_share_directory("usv_vision")
    gmsl_share = get_package_share_directory("usv_vision_gmsl_cpp")

    cameras_arg = DeclareLaunchArgument(
        "cameras", default_value="['front', 'rear']",
        description="相机列表，例: ['front'], ['front','rear'], ['front','rear','side']",
    )
    inference_arg = DeclareLaunchArgument("inference", default_value="true")
    stream_mode_arg = DeclareLaunchArgument(
        "stream_mode", default_value="internal",
        description="推流模式: internal（C++ 内置，默认）| python（旧节点）| off")
    streaming_arg = DeclareLaunchArgument(
        "streaming", default_value="",
        description="[兼容] 非空时覆盖 stream_mode：true→python，false→off")
    algo_stream_arg = DeclareLaunchArgument(
        "algo_stream", default_value="true",
        description="采集端是否发布 image_algo（识别链吃小图）；false 时识别退回订阅 image_raw",
    )
    algo_transport_arg = DeclareLaunchArgument(
        "algo_transport", default_value="raw",
        description="算法图传输格式：raw（默认）| jpeg（已实现，三路实测帧率降，默认不用）。"
                    "一处下发到采集与识别两端，不会错配",
    )

    return LaunchDescription([
        cameras_arg,
        inference_arg,
        stream_mode_arg,
        streaming_arg,
        algo_stream_arg,
        algo_transport_arg,
        LogInfo(msg=["=== Jetson GMSL Full Pipeline ==="]),
        LogInfo(msg=["  cameras:   ", LaunchConfiguration("cameras")]),
        LogInfo(msg=["  inference: ", LaunchConfiguration("inference")]),
        LogInfo(msg=["  streaming: ", LaunchConfiguration("streaming")]),
        LogInfo(msg=["  algo_stream: ", LaunchConfiguration("algo_stream")]),
        OpaqueFunction(function=camera_nodes, args=[gmsl_share, vision_share]),
    ])
