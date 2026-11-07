#!/usr/bin/env python3
"""验证「可靠发布 + GUI 订阅」是否反压采集线程。

背景
----
采集节点在**采集线程里同步**发布 image_raw 与 image_algo（顺序：raw → algo → 记 timing）。
image_algo 的发布端是 reliable/depth2，rviz 以默认（reliable）订阅它。理论上：rviz 若僵住
不 ack，写端 history 满时 `publish()` 可能阻塞到 max_blocking_time，从而拖慢采集线程。

本脚本用**发布端自己带出来的时序**来验证这件事，不依赖本脚本的收包节奏：

- `frame_timing_json` 每帧一条，其中 `timestamps.capture_stamp_ros` 是硬件捕获时间戳，
  相邻差值即**真实采集周期**（不受订阅端丢包影响）；
- `latency_ms.driver_latency_ms` = 捕获 → 发布完成，正好把两次 `publish()` 包在里面，
  所以 algo 被拖住会直接体现在这个数上。

为什么不能看 CPU：阻塞是"等待"，不耗 CPU，反而会让 CPU 占用下降——看 CPU 会得出相反结论。

三组对照
--------
A 基线        ：只跑采集，不开 rviz
B 正常 rviz   ：打开 rviz 订阅 image_algo（默认 QoS）
C 最坏情况    ：把 rviz 冻住（Ctrl+Z 或 kill -STOP），持续一段时间后恢复

**C 组才是关键**：正常 rviz 会及时 ack，理论上测不出问题；真正要回答的是"GUI 僵住会不会
拖慢采集"，只有冻住它才测得到。

用法
----
    # 交互式（推荐）：三组按提示依次进行
    ros2 run usv_vision check_image_qos --sensor front

    # 自动连跑（自行按提示时间点操作 rviz）
    ros2 run usv_vision check_image_qos --sensor front --auto

    # 全自动：脚本自己等 rviz 启动、自己 SIGSTOP/SIGCONT 冻结
    # （A 组要求 rviz 未启动；B 组会提示你打开 rviz 并等待）
    ros2 run usv_vision check_image_qos --sensor front --auto-rviz --record qos.md

    # 只测一组
    ros2 run usv_vision check_image_qos --phase A

判据（默认值，可用参数调整）
--------------------------
    raw/采集周期 相对基线下降 < 1.5 帧      → 无影响
    1.5~3 帧                               → 可疑
    > 3 帧                                 → 明确受影响
    driver_latency_ms p95 相对基线 +<2ms    → 无影响
    +2~5ms                                 → 可疑
    +>5ms 或出现 >50ms 尖峰                → 明确受影响

退出码：0 = 通过 / 1 = 可疑 / 2 = 明确受影响或数据不足
"""

import argparse
import json
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# 统计结果
# ---------------------------------------------------------------------------
@dataclass
class PhaseStats:
    name: str = ""
    duration: float = 0.0
    timing_msgs: int = 0
    raw_msgs: int = 0
    algo_msgs: int = 0
    capture_hz: float = 0.0
    frame_dts_ms: float = 0.0
    gaps: int = 0                 # 采集周期 > 3×中位数 的次数（卡顿次数）
    lat_mean: float = 0.0
    lat_p50: float = 0.0
    lat_p95: float = 0.0
    lat_max: float = 0.0
    algo_published_ratio: float = 0.0
    stamp_source: str = ""        # capture=硬件捕获时间戳可用；publish=退化后改用发布时刻
    lat_invalid: int = 0          # 明显异常的 driver_latency_ms 条数

    def ok(self) -> bool:
        return self.timing_msgs >= 10


def _percentile(values, ratio: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(len(ordered) * ratio))
    return ordered[index]


# ---------------------------------------------------------------------------
# rviz 进程控制（--auto-rviz 用）
# ---------------------------------------------------------------------------
def rviz_pids() -> list[int]:
    """当前 rviz2 进程 pid 列表（没有则返回空）。"""
    if shutil.which("pgrep") is None:
        return []
    try:
        result = subprocess.run(["pgrep", "-f", "rviz2"], capture_output=True,
                                text=True, timeout=3, check=False)
    except (OSError, subprocess.SubprocessError):
        return []
    pids = []
    for token in result.stdout.split():
        try:
            pids.append(int(token))
        except ValueError:
            continue
    return pids


def signal_rviz(pids: list[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except OSError as exc:  # 进程可能已退出
            print(f"  [警告] 对 rviz(pid={pid}) 发送 {sig} 失败：{exc}", file=sys.stderr)


def wait_for_rviz(timeout_sec: float) -> list[int]:
    """等待 rviz2 启动，返回其 pid 列表；超时返回空。"""
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while time.monotonic() < deadline:
        pids = rviz_pids()
        if pids:
            return pids
        time.sleep(0.5)
    return []


# ---------------------------------------------------------------------------
# 采集器
# ---------------------------------------------------------------------------
class ImageQosProbe(Node):
    def __init__(self, sensor: str) -> None:
        super().__init__("image_qos_probe")
        self.sensor = sensor
        base = f"/sensors/camera/{sensor}"
        self.raw_topic = f"{base}/image_raw"
        self.algo_topic = f"{base}/image_algo"
        self.timing_topic = f"{base}/frame_timing_json"

        # 订阅端一律 best_effort：发布端无论是 reliable 还是 best_effort 都能收到
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        timing_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.reset()
        self.create_subscription(Image, self.raw_topic, self._on_raw, image_qos)
        self.create_subscription(Image, self.algo_topic, self._on_algo, image_qos)
        self.create_subscription(String, self.timing_topic, self._on_timing, timing_qos)

    # -- 状态 ---------------------------------------------------------------
    def reset(self) -> None:
        self._raw = 0
        self._algo = 0
        self._timing = 0
        self._dts_capture = []
        self._dts_publish = []
        self._lat = []
        self._lat_invalid = 0
        self._algo_flags = []
        self._last_capture = None
        self._last_publish = None

    @staticmethod
    def _accumulate_dt(stamp, last, bucket):
        """把相邻时间戳之差（滤波后）塞进 bucket，返回本次时间戳。"""
        if not isinstance(stamp, dict):
            return last
        sec = stamp.get("sec")
        nanosec = stamp.get("nanosec")
        if sec is None or nanosec is None:
            return last
        current = float(sec) + float(nanosec) / 1e9
        if last is not None:
            dt = current - last
            if 0.0 < dt < 1.0:
                bucket.append(dt)
        return current

    @property
    def timing_count(self) -> int:
        """已收到的 frame_timing_json 条数（= 发布端出帧数）。"""
        return self._timing

    # -- 回调 ---------------------------------------------------------------
    def _on_raw(self, _msg) -> None:
        self._raw += 1

    def _on_algo(self, _msg) -> None:
        self._algo += 1

    def _on_timing(self, msg: String) -> None:
        self._timing += 1
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return

        stamps = payload.get("timestamps", {}) or {}
        self._last_capture = self._accumulate_dt(
            stamps.get("capture_stamp_ros"), self._last_capture, self._dts_capture)
        self._last_publish = self._accumulate_dt(
            stamps.get("publish_stamp_ros"), self._last_publish, self._dts_publish)

        latency = payload.get("latency_ms", {}).get("driver_latency_ms")
        if latency is not None:
            try:
                value = float(latency)
            except (TypeError, ValueError):
                value = None
            if value is not None:
                # 硬件时间戳失效时（板上 SDK 返回 1）会算出天文数字，直接判为无效
                if value > 10000.0 or value < -10000.0:
                    self._lat_invalid += 1
                else:
                    self._lat.append(value)

        algo_flag = payload.get("image_algo", {}).get("published")
        if algo_flag is not None:
            self._algo_flags.append(bool(algo_flag))

    # -- 汇总 ---------------------------------------------------------------
    def snapshot(self, name: str, duration: float) -> PhaseStats:
        stats = PhaseStats(name=name, duration=duration, timing_msgs=self._timing,
                           raw_msgs=self._raw, algo_msgs=self._algo)
        # 优先用硬件捕获时间戳；若它退化（板上 SDK 可能恒返回 1），改用发布时刻间隔
        dts = self._dts_capture
        source = "capture"
        if not dts or statistics.median(dts) <= 0.0:
            dts = self._dts_publish
            source = "publish" if dts else ""
        stats.stamp_source = source
        stats.lat_invalid = self._lat_invalid

        if dts:
            median_dt = statistics.median(dts)
            stats.frame_dts_ms = median_dt * 1000.0
            stats.capture_hz = 1.0 / median_dt if median_dt > 0 else 0.0
            stats.gaps = sum(1 for dt in dts if dt > 3.0 * median_dt)
        elif duration > 0:
            # 没有时序消息时退回按收包计数估算，仅作参考
            stats.capture_hz = self._raw / duration
        if self._lat:
            stats.lat_mean = statistics.mean(self._lat)
            stats.lat_p50 = _percentile(self._lat, 0.50)
            stats.lat_p95 = _percentile(self._lat, 0.95)
            stats.lat_max = max(self._lat)
        if self._algo_flags:
            stats.algo_published_ratio = sum(self._algo_flags) / len(self._algo_flags)
        return stats

    # -- 发布端 QoS（便于留档：记录到底在测哪种配置）------------------------
    def publisher_qos_summary(self, topic: str) -> str:
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception:  # noqa: BLE001 - 老版本 rclpy 可能没有该接口
            return "unknown"
        if not infos:
            return "no publisher"
        parts = []
        for info in infos:
            qos = info.qos_profile
            rel = getattr(qos, "reliability", None)
            rel_name = rel.name.lower() if rel is not None else "unknown"
            hist = getattr(qos, "history", None)
            hist_name = hist.name.lower() if hist is not None else ""
            if hist_name in ("", "unknown", "system_default"):
                hist_name = ""
            # depth 在部分 rclpy 版本里拿不到真实值（恒为 0），只在有效时才打印
            depth = getattr(qos, "depth", 0)
            label = rel_name + (f"/{hist_name}" if hist_name else "")
            if depth:
                label += f"/d{depth}"
            parts.append(label)
        return ", ".join(parts)


# ---------------------------------------------------------------------------
# 运行与报告
# ---------------------------------------------------------------------------
def run_phase(node: ImageQosProbe, name: str, duration: float, prompt: str | None) -> PhaseStats:
    if prompt:
        try:
            input(prompt)
        except EOFError:
            print("\n[警告] 无交互输入，按 --auto 的方式继续", file=sys.stderr)

    node.reset()
    print(f"\n[{name}] 开始采样 {duration:.0f}s ...")
    start = time.monotonic()
    last_report = start
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now - last_report >= 10.0 and now - start < duration:
            last_report = now
            print(f"  [{name}] 已采 {now - start:.0f}s / {duration:.0f}s  "
                  f"帧数={node.timing_count}")
        if now - start >= duration:
            break
    stats = node.snapshot(name, now - start)
    if not stats.ok():
        print(f"  [{name}] 警告：只收到 {stats.timing_msgs} 条 frame_timing_json，"
              f"请确认节点在跑且 --sensor 正确")
    return stats


def _display_width(text: str) -> int:
    """按终端显示宽度计算（CJK 字符占两列）。"""
    import unicodedata

    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in text)


def _pad(text: str, width: int) -> str:
    return text + " " * max(0, width - _display_width(text))


def print_table(rows: list[PhaseStats]) -> None:
    header = ("组", "采集hz", "帧间隔ms", "卡顿", "lat_mean", "lat_p50", "lat_p95", "lat_max", "algo发布率")
    widths = (12, 8, 10, 6, 10, 9, 9, 9, 12)
    line = "  ".join(_pad(h, w) for h, w in zip(header, widths))
    print("\n" + line)
    print("-" * _display_width(line))
    for row in rows:
        cells = (
            row.name,
            f"{row.capture_hz:.1f}",
            f"{row.frame_dts_ms:.1f}",
            str(row.gaps),
            f"{row.lat_mean:.2f}",
            f"{row.lat_p50:.2f}",
            f"{row.lat_p95:.2f}",
            f"{row.lat_max:.2f}",
            f"{row.algo_published_ratio * 100:.0f}%",
        )
        print("  ".join(_pad(c, w) for c, w in zip(cells, widths)))
    notes = [f"{r.name}: 时间戳来源={r.stamp_source or 'n/a'}" for r in rows]
    invalid = sum(r.lat_invalid for r in rows)
    if invalid:
        notes.append(f"已忽略 {invalid} 条荒谬的 driver_latency_ms（硬件时间戳可能失效）")
    for line in notes:
        print(f"  · {line}")
    print()


def judge(baseline: PhaseStats, others: list[PhaseStats],
          hz_tolerance: float, p95_tolerance: float, max_tolerance: float) -> tuple[int, list[str]]:
    """返回 (退出码, 结论文本)。0=通过 1=可疑 2=受影响"""
    lines = []
    worst_code = 0
    for item in others:
        hz_drop = baseline.capture_hz - item.capture_hz
        p95_delta = item.lat_p95 - baseline.lat_p95
        code = 0
        reason = "无影响"
        if hz_drop > 3.0 or p95_delta > 2 * p95_tolerance or item.lat_max > max_tolerance:
            code, reason = 2, "明确受影响"
        elif hz_drop > hz_tolerance or p95_delta > p95_tolerance:
            code, reason = 1, "可疑"
        worst_code = max(worst_code, code)
        lines.append(
            f"  {item.name}: 采集hz {hz_drop:+.1f} 帧, p95 {p95_delta:+.2f}ms, "
            f"max {item.lat_max:.2f}ms, 卡顿 {item.gaps} 次 → {reason}"
        )

    if worst_code == 0:
        lines.append("\n结论：通过。可靠发布未拖慢采集，可保持 reliable；"
                     "rviz 用默认 QoS 即可观察。")
    elif worst_code == 1:
        lines.append("\n结论：可疑。建议复测一次；若冻结 rviz 期间才出现，"
                     "写进运维说明（不要让 rviz 僵住/挂起）。")
    else:
        lines.append("\n结论：明确受影响。回退方式：把 gmsl_camera_*_*.yaml 的 "
                     "image_qos_reliability 改为 best_effort，并在 rviz 的 Image display 里"
                     "把 Reliability 选 Best effort。")
    return worst_code, lines


def write_record(path: str, rows: list[PhaseStats], conclusions: list[str]) -> None:
    try:
        with open(path, "a", encoding="utf-8") as handle:
            handle.write(f"\n## 采样 {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            handle.write("| 组 | 采集hz | 帧间隔ms | 卡顿 | lat_mean | lat_p95 | lat_max | algo发布率 |\n")
            handle.write("|---|---|---|---|---|---|---|---|\n")
            for row in rows:
                handle.write(
                    f"| {row.name} | {row.capture_hz:.1f} | {row.frame_dts_ms:.1f} | {row.gaps} | "
                    f"{row.lat_mean:.2f} | {row.lat_p95:.2f} | {row.lat_max:.2f} | "
                    f"{row.algo_published_ratio * 100:.0f}% |\n"
                )
            handle.write("\n" + "\n".join(conclusions) + "\n")
        print(f"\n记录已追加：{path}")
    except OSError as exc:
        print(f"\n[警告] 记录写入失败：{exc}", file=sys.stderr)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="验证图像话题「可靠发布 + GUI 订阅」是否反压采集线程",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--sensor", default="front", help="sensor_name（默认 front）")
    parser.add_argument("--duration", type=float, default=30.0, help="每组采样秒数（默认 30）")
    parser.add_argument("--phase", choices=["A", "B", "C", "all"], default="all",
                        help="A=仅采集基线；B=+rviz；C=+rviz 冻结；all=三组依次（默认 all）")
    parser.add_argument("--auto", action="store_true",
                        help="不等待回车，按打印的提示自行在对应时间点操作 rviz")
    parser.add_argument("--auto-rviz", action="store_true",
                        help="由脚本自己管理 rviz：A 组要求 rviz 未启动；B 组等待 rviz 启动；"
                             "C 组对 rviz 发 SIGSTOP 模拟卡死，采样后自动 SIGCONT")
    parser.add_argument("--wait-rviz", type=float, default=60.0,
                        help="--auto-rviz 时等待 rviz 启动的秒数（默认 60）")
    parser.add_argument("--hz-tolerance", type=float, default=1.5,
                        help="采集hz 下降容忍（帧，默认 1.5）")
    parser.add_argument("--p95-tolerance", type=float, default=2.0,
                        help="driver_latency p95 增量容忍（ms，默认 2.0）")
    parser.add_argument("--max-tolerance", type=float, default=50.0,
                        help="driver_latency 单帧上限（ms，默认 50）")
    parser.add_argument("--record", default="", help="把结果追加写入该 Markdown 文件")
    parsed = parser.parse_args(args=args)

    rclpy.init(args=None)
    node = ImageQosProbe(parsed.sensor)
    frozen_pids: list[int] = []
    try:
        print(f"话题：{node.raw_topic}\n      {node.algo_topic}\n      {node.timing_topic}")
        # 启动瞬间发现还没稳定，重试几秒把发布端 QoS 读出来（便于留档）
        raw_qos = node.publisher_qos_summary(node.raw_topic)
        algo_qos = node.publisher_qos_summary(node.algo_topic)
        deadline = time.monotonic() + 5.0
        while "no publisher" in (raw_qos, algo_qos) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            raw_qos = node.publisher_qos_summary(node.raw_topic)
            algo_qos = node.publisher_qos_summary(node.algo_topic)
        print(f"发布端 QoS：image_raw={raw_qos}  image_algo={algo_qos}")

        rows: list[PhaseStats] = []
        interactive = not parsed.auto

        def prompt(text: str) -> str | None:
            if parsed.auto_rviz:
                return None
            if not interactive:
                print(f"\n>>> 请在接下来 {parsed.duration:.0f}s 内：{text}")
                return None
            return f"\n>>> {text}\n    完成后按回车继续 > "

        # A 组基线要求"没有任何订阅 image_algo 的 rviz"：已启动就先请用户关掉
        if parsed.auto_rviz and parsed.phase in ("A", "all"):
            running = rviz_pids()
            if running:
                print(f"\n[中止] 基线组要求 rviz 未启动，但检测到 rviz2 正在运行（pid={running}）。")
                print("请先关闭 rviz 再重跑；否则基线里已经有一个 reliable 读者，对照不成立。")
                sys.exit(2)

        if parsed.phase in ("A", "all"):
            rows.append(run_phase(
                node, "A 仅采集", parsed.duration,
                prompt("确认没有打开 rviz（也不要开其他订阅 image_algo 的节点）")))

        if parsed.phase in ("B", "all"):
            if parsed.auto_rviz:
                print(f"\n>>> 请现在打开 rviz 并添加 Image display 订阅 {node.algo_topic}"
                      f"（保持默认 QoS）；最多等待 {parsed.wait_rviz:.0f}s ...")
                pids = wait_for_rviz(parsed.wait_rviz)
                if not pids:
                    print("[中止] 未检测到 rviz2 进程，B/C 组无法进行。", file=sys.stderr)
                    sys.exit(2)
                print(f"    检测到 rviz2 pid={pids}，继续")
            rows.append(run_phase(
                node, "B +rviz", parsed.duration,
                prompt(f"打开 rviz 并添加 Image display 订阅 {node.algo_topic}（保持默认 QoS）")))

        if parsed.phase in ("C", "all"):
            if parsed.auto_rviz:
                frozen_pids = rviz_pids()
                if not frozen_pids:
                    print("[中止] C 组需要 rviz 正在运行，但未检测到 rviz2 进程。", file=sys.stderr)
                    sys.exit(2)
                print(f"\n>>> C 组：等 3s 让 rviz 进入稳定状态，然后 SIGSTOP pid={frozen_pids}"
                      f" 模拟卡死；采样结束自动 SIGCONT")
                time.sleep(3.0)
                signal_rviz(frozen_pids, signal.SIGSTOP)
            rows.append(run_phase(
                node, "C +rviz冻结", parsed.duration,
                prompt("先让 rviz 正常跑 5 秒，然后把它冻住：终端里 Ctrl+Z，或 "
                       "kill -STOP <rviz pid>；采样结束后恢复：fg 或 kill -CONT <rviz pid>")))
            if frozen_pids:
                signal_rviz(frozen_pids, signal.SIGCONT)
                print(f"    已恢复 rviz（SIGCONT pid={frozen_pids}）")
                frozen_pids = []

        print_table(rows)

        if not rows[0].ok():
            print("数据不足：基线组没有收到足够的 frame_timing_json。")
            print("请先确认采集节点在跑（ros2 topic hz /sensors/camera/<sensor>/frame_timing_json），"
                  "并检查 --sensor 是否与 driver 的 sensor_name 一致。")
            sys.exit(2)

        if len(rows) >= 2:
            baseline = rows[0]
            code, conclusions = judge(baseline, rows[1:], parsed.hz_tolerance,
                                      parsed.p95_tolerance, parsed.max_tolerance)
            print("\n对照结论：")
            print("\n".join(conclusions))
        else:
            code, conclusions = 0, [f"只有一组数据（{rows[0].name}），未做对照判断。"]
            print("\n" + conclusions[0])

        if parsed.record:
            write_record(parsed.record, rows, conclusions)

        sys.exit(code)
    finally:
        # 任何异常路径都不能把 rviz 留在冻结状态
        if frozen_pids:
            signal_rviz(frozen_pids, signal.SIGCONT)
            print(f"[清理] 已恢复 rviz（SIGCONT pid={frozen_pids}）")
        node.destroy_node()
        shutdown = getattr(rclpy, "try_shutdown", None) or rclpy.shutdown
        shutdown()


if __name__ == "__main__":
    main()
