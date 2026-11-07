"""
rtsp_stream_node —— 订阅统一图像话题，硬件编码后推流到 RTSP。

运行平台：NVIDIA Jetson（依赖 nvvidconv / nvv4l2h264enc 硬件编码插件）。

用途：
- GMSL 相机设备已被采集节点（C++ / Python）独占打开，推流不能再直接 v4l2src
  抢设备；本节点从 ROS 图像话题接帧，避免设备冲突。
- 用 nvvidconv + nvv4l2h264enc 硬件编码，推到本地 rtspclientsink，
  再由 frpc 内网穿透转发到公网。

管道：
  appsrc(BGR) -> videoconvert -> nvvidconv -> NV12(NVMM)
      -> nvv4l2h264enc -> h264parse -> rtspclientsink location=<rtsp_url>

设计要点：
- 面向公网：编码器参数可配（默认画质优先 4Mbps+），并在推流管道 EOS/错误时
  自动重连（公网连接不稳）。
- 不碰设备：数据源是 ROS 话题，与采集/识别链路解耦。
- 状态心跳：定时发布 VideoStreamStatus（streaming=true/false），断流时下游也能
  区分“活着但没推流” vs “节点没了”。usv_interfaces 不可用时自动降级、不影响推流。
"""
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .common.qos import image_qos
from .common.topics import image_topic as default_image_topic
from .common.topics import stream_status_topic as default_stream_status_topic

# VideoStreamStatus 为可选依赖：缺失时关闭状态发布，但推流功能照常可用
try:
    from usv_interfaces.msg import VideoStreamStatus
    _HAVE_STREAM_STATUS = True
except ImportError:
    VideoStreamStatus = None
    _HAVE_STREAM_STATUS = False

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp  # noqa: E402


class RtspStreamNode(Node):
    def __init__(self) -> None:
        super().__init__("rtsp_stream_node")

        self.declare_parameter("sensor_name", "front")
        self.declare_parameter("input_image_topic", "")  # 空则用 /sensors/camera/{sensor_name}/image_raw
        self.declare_parameter("rtsp_url", "rtsp://127.0.0.1:8554/cam1")
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1536)
        self.declare_parameter("fps", 30)
        self.declare_parameter("bitrate", 4000000)          # 画质优先默认 4Mbps
        self.declare_parameter("iframe_interval", 30)        # I 帧间隔（帧）
        self.declare_parameter("idr_interval", 30)
        self.declare_parameter("control_rate", 1)            # 1=constant bitrate
        self.declare_parameter("preset_level", 1)
        self.declare_parameter("insert_sps_pps", True)
        self.declare_parameter("reconnect_delay_sec", 3.0)   # 断流后重连间隔
        self.declare_parameter("camera_id", "")               # VideoStreamStatus.camera_id，空则用 sensor_name
        self.declare_parameter("status_topic", "")            # 空则 /perception/vision/{sensor_name}/stream_status
        self.declare_parameter("status_rate_hz", 1.0)         # 状态心跳频率；<=0 关闭发布
        self.declare_parameter("image_qos_reliability", "best_effort")
        self.declare_parameter("image_qos_depth", 1)

        self.sensor_name = str(self.get_parameter("sensor_name").value).strip() or "front"
        self.input_topic = (
            str(self.get_parameter("input_image_topic").value).strip()
            or default_image_topic(self.sensor_name)
        )
        self.rtsp_url = str(self.get_parameter("rtsp_url").value).strip()
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.bitrate = int(self.get_parameter("bitrate").value)
        self.iframe_interval = int(self.get_parameter("iframe_interval").value)
        self.idr_interval = int(self.get_parameter("idr_interval").value)
        self.control_rate = int(self.get_parameter("control_rate").value)
        self.preset_level = int(self.get_parameter("preset_level").value)
        self.insert_sps_pps = bool(self.get_parameter("insert_sps_pps").value)
        self.reconnect_delay = float(self.get_parameter("reconnect_delay_sec").value)
        self.camera_id = str(self.get_parameter("camera_id").value).strip() or self.sensor_name

        Gst.init(None)
        self._lock = threading.Lock()
        self._pipeline = None
        self._appsrc = None
        self._expected_size = self.width * self.height * 3  # BGR
        self._got_first_frame = False
        self._streaming = False  # 当前是否真的在推流（首帧成功推出后为 True）

        self._build_pipeline()

        # 状态心跳：始终周期发布（含未推流时 streaming=false），便于下游区分状态。
        # usv_interfaces 缺失时降级为不发布，推流不受影响。
        self._status_pub = None
        status_rate = float(self.get_parameter("status_rate_hz").value)
        if _HAVE_STREAM_STATUS and status_rate > 0.0:
            # 推流属传感器/驱动层旁路，状态归 /sensors/camera/ 命名空间（与 driver_status 并列），
            # 不放 /perception/vision/（那是识别/感知层，放这会被误当感知产物）。
            status_topic = (
                str(self.get_parameter("status_topic").value).strip()
                or default_stream_status_topic(self.sensor_name)
            )
            self._status_pub = self.create_publisher(VideoStreamStatus, status_topic, 10)
            self.create_timer(1.0 / status_rate, self._publish_status)
            self.get_logger().info(f"stream status heartbeat on {status_topic} @ {status_rate}Hz")
        elif not _HAVE_STREAM_STATUS:
            self.get_logger().warn(
                "usv_interfaces.VideoStreamStatus 不可用，跳过状态发布（推流功能正常）"
            )

        qos = image_qos(
            str(self.get_parameter("image_qos_reliability").value),
            int(self.get_parameter("image_qos_depth").value),
        )
        self.create_subscription(Image, self.input_topic, self._on_image, qos)

        self.get_logger().info(
            f"rtsp_stream_node started: sensor={self.sensor_name}, "
            f"input={self.input_topic}, rtsp={self.rtsp_url}, "
            f"{self.width}x{self.height}@{self.fps}, bitrate={self.bitrate}"
        )

    def _pipeline_desc(self) -> str:
        # appsrc(BGR) → queue1(限帧) → videoconvert → nvvidconv(NVMM)
        #   → queue2(限内存) → nvv4l2h264enc → h264parse → rtspclientsink(TCP)
        # queue1: 丢弃旧帧保证实时性; queue2: 限制 NVMM 内存堆积
        return (
            "appsrc name=src is-live=true block=true format=time do-timestamp=true "
            f"caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 ! "
            "queue name=q1 max-size-buffers=6 max-size-bytes=31457280 max-size-time=200000000 leaky=downstream ! "
            "videoconvert ! "
            "nvvidconv ! "
            f"video/x-raw(memory:NVMM),format=NV12,width={self.width},height={self.height} ! "
            "queue name=q2 max-size-buffers=5 max-size-bytes=26214400 max-size-time=200000000 leaky=downstream ! "
            "nvv4l2h264enc "
            f"bitrate={self.bitrate} "
            f"control-rate={self.control_rate} "
            f"iframeinterval={self.iframe_interval} "
            f"idrinterval={self.idr_interval} "
            f"insert-sps-pps={1 if self.insert_sps_pps else 0} "
            f"preset-level={self.preset_level} ! "
            "h264parse config-interval=-1 ! "
            f"rtspclientsink location={self.rtsp_url} latency=500 "
            "protocols=tcp"
        )

    def _build_pipeline(self) -> None:
        with self._lock:
            desc = self._pipeline_desc()
            self.get_logger().info(f"building pipeline: {desc}")
            self._pipeline = Gst.parse_launch(desc)
            self._appsrc = self._pipeline.get_by_name("src")

            bus = self._pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message::error", self._on_bus_error)
            bus.connect("message::eos", self._on_bus_eos)

            self._pipeline.set_state(Gst.State.PLAYING)
            self._got_first_frame = False

    def _publish_status(self) -> None:
        if self._status_pub is None:
            return
        msg = VideoStreamStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sensor_name
        with self._lock:
            msg.streaming = self._streaming
        msg.camera_id = self.camera_id
        msg.stream_url = self.rtsp_url
        self._status_pub.publish(msg)

    def _restart_pipeline(self, reason: str) -> None:
        self.get_logger().warn(f"pipeline restart ({reason}); reconnecting in {self.reconnect_delay}s")
        with self._lock:
            self._streaming = False
            if self._pipeline is not None:
                self._pipeline.set_state(Gst.State.NULL)
                self._pipeline = None
                self._appsrc = None
        # 延迟后重建（公网断线自愈）
        timer = threading.Timer(self.reconnect_delay, self._build_pipeline)
        timer.daemon = True
        timer.start()

    def _on_bus_error(self, _bus, message) -> None:
        err, debug = message.parse_error()
        self.get_logger().error(f"gstreamer error: {err} ({debug})")
        self._restart_pipeline("bus error")

    def _on_bus_eos(self, _bus, _message) -> None:
        self._restart_pipeline("EOS")

    def _on_image(self, msg: Image) -> None:
        if msg.encoding != "bgr8":
            self.get_logger().warn_once(
                f"expected bgr8, got {msg.encoding}; skipping frames"
            )
            return

        with self._lock:
            appsrc = self._appsrc
        if appsrc is None:
            return  # 重连窗口内，丢帧

        data = bytes(msg.data)
        if len(data) != self._expected_size:
            # 分辨率与配置不符：跳过，避免编码器崩（提示一次）
            self.get_logger().warn_once(
                f"frame size {len(data)} != expected {self._expected_size} "
                f"({self.width}x{self.height}x3); check width/height params"
            )
            return

        buf = Gst.Buffer.new_wrapped(data)
        ret = appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            self.get_logger().warn(f"push-buffer returned {ret}")
            if not self._got_first_frame:
                return
            self._restart_pipeline(f"push-buffer {ret}")
            return
        self._got_first_frame = True
        with self._lock:
            self._streaming = True

    def destroy_node(self):
        with self._lock:
            if self._pipeline is not None:
                self._pipeline.set_state(Gst.State.NULL)
                self._pipeline = None
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtspStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
