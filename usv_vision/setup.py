from glob import glob

from setuptools import find_packages, setup

package_name = "usv_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            [
                "config/yolo_infer.yaml",
                "config/rtsp_stream.yaml",
            ],
        ),
        (
            f"share/{package_name}/launch",
            [
                "launch/jetson_gmsl_full.launch.py",
            ],
        ),
        (f"share/{package_name}/config/masks", glob("config/masks/*.png")),
        (f"share/{package_name}/docs", ["docs/mask_usage.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="USV Developer",
    maintainer_email="developer@usv.com",
    description="USV vision — Jetson deployment subset (TensorRT inference + RTSP streaming).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # === Jetson 只需要这两个节点 ===
            "yolo_infer_node = usv_vision.yolo_infer_node:main",
            "rtsp_stream_node = usv_vision.rtsp_stream_node:main",
            "capture_mask_frame = scripts.capture_mask_frame:main",
            "make_mask = scripts.make_mask:main",
            "draw_mask = scripts.draw_mask:main",
            "verify_mask_recognition = scripts.verify_mask_recognition:main",
            # 图像话题「可靠发布 + GUI 订阅」反压验证（A/B/C 三组对照）
            "check_image_qos = scripts.check_image_qos_backpressure:main",
        ],
    },
)
