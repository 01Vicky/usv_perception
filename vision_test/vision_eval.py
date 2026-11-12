#!/usr/bin/env python3
"""vision_eval —— 视觉二阶段测试（4.1.1–4.1.24）离线判分

干什么
----
读一次测试采集目录（或直接读 bag），把能自动算的测试卡算出来，输出一份
Markdown 报告。报告里每张卡都是「计划表"测试记录/实测结果"栏」的填空顺序，
可以直接复制进测试记录表。

为什么离线算
-----------
现场只负责采集：板上 8 核已经被视觉占了近 3 核，再跑统计会污染 4.1.21 的资源数据；
而且正文里大量指标是"暂定"，结论随时要按新口径复算——包在手上就能重算，
不用重新上船。所以本脚本不订阅实时话题，只读 bag。

数据从哪来
---------
用配套的 vision_test_record.sh 采集，它录这些话题并生成快照：
    /sensors/camera/{cam}/frame_timing_json      真实采集周期、原生/算法图尺寸、驱动段时延
    /sensors/camera/{cam}/camera_info            原生内参（fx,cx 决定角度）
    /sensors/camera/{cam}/camera_info_algo       算法图内参（角度是在算法图上算的）
    /perception/camera/{cam}/detections          VisionDetectionArray，带 output_stamp
    /perception/camera/{cam}/objects             AIVisionTargets，唯一的检测框来源
    /perception/camera/{cam}/status              运行状态 JSON
    /perception/camera/{cam}/annotated           可选，需 publish_debug_image:=true

用法
----
    # 读采集目录（vision_test_record.sh 产出的那个目录）
    ros2 run usv_vision vision_eval ~/snap/S03

    # 直接读 bag
    ros2 run usv_vision vision_eval ~/snap/S03/bag -o report.md

    # 带上 tegrastats 日志，顺便算 4.1.21
    ros2 run usv_vision vision_eval ~/snap/S03 --tegrastats ~/snap/S03/tegrastats.log

    # 本机没 bag 时先自检判定逻辑（不需要 ROS、不需要相机）
    python3 vision_test/vision_eval.py --selftest

这张脚本算什么
-------------
自动：4.1.3 / 4.1.4 / 4.1.8 / 4.1.9 / 4.1.10(仅帧对应) / 4.1.11 / 4.1.12 /
      4.1.16 / 4.1.17 / 4.1.19 / 4.1.20 / 4.1.21(给了 tegrastats 才算)
不覆盖（报告里会列成"需人工/未执行"）：4.1.1 版本快照（看 record 目录里的 snapshot/）、
      4.1.2 / 4.1.6 启动、4.1.5 视场覆盖（要安装角）、4.1.7 话题连接（看 graph.txt）、
      4.1.13/14/15 检出率类别 IoU（要人工标注）、4.1.18 远距大船（要距离依据）、
      4.1.22/23 RTSP（要拉流探测）、4.1.24 异常注入（另用注入脚本）

退出码：0 全部自动项通过 / 1 有项不通过 / 2 数据不足（多数卡未能计算）
"""

import argparse
import hashlib
import json
import math
import os
import re
import statistics
import sys
from bisect import bisect_left

# ─────────────────────────── 阈值（正文原文，改这里即可复算） ───────────────────────────
MIN_HZ = 20.0                 # F01.01 / F02.02 每路不低于 20 Hz
HZ_TOL_CAM3 = 2.0             # F01.01 三路频率差不大于 2 Hz
HZ_TOL_IMG2DET = 2.0          # F02.02 输出频率与图像帧率之差不大于 2 Hz
HZ_WAVE_MAX = 3.0             # F05.02 波动小于 3 Hz（20 Hz 的 15%）
BURST_DEPTH_MAX = 3           # F05.02 突发堆积深度不大于 3 帧
DROP_RATE_MAX = 0.02          # F05.02 丢帧率不大于 2%
E2E_P95_MAX_MS = 100.0        # F05.03 端到端时延 P95 ≤ 100 ms
CENTER_SHIFT_MAX_RATIO = 0.01  # F03.04 框中心位移不大于画面宽 1%/帧（≈19.2 px @1920）
CONF_DELTA_MAX = 0.15         # F03.04 置信度变化幅度不大于 0.15/帧
CLASS_FLIP_MAX_PER_5S = 1     # F03.04 类别翻转不大于 1 次/5 s/目标
DUP_RATE_MAX = 0.05           # F03.06 重复输出率不大于 5%
DUP_IOU = 0.50                # 本脚本口径：同帧同类 IoU>0.5 视为重复
DUP_CENTER_RATIO = 0.03       # 或中心距小于画面宽 3%（与 dedup_center_dist_ratio 一致）
ANG_TOL_DEG = 0.02            # F03.07 两个角度字段的舍入容差
ANG_CHECK_TOL_RAD = 0.02      # 角度与检测框复算值的容差
GAP_WARN_S = 1.0              # 超过 1 s 无输出记异常时间窗（只记录，不判定）
STALL_MS = 100.0              # 单次间隔 ≥100 ms 记一次"卡顿"（观察项，不判失败）

# F03.07 正文数值（用于 4.1.12 逐项核对）
REF_ANG_DEG_PER_PX_RAW = 0.06
REF_ANG_DEG_PER_PX_640 = 0.14
REF_LATERAL_CM_100M_RAW = 10.0
REF_LATERAL_CM_100M_640 = 25.0


# ────────────────────────────────── 小工具 ──────────────────────────────────
def sec_of(stamp):
    """builtin_interfaces/Time → 秒(float)。"""
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def pct(values, q):
    """线性插值分位数，q 取 0~100。样本空返回 None。"""
    if not values:
        return None
    data = sorted(values)
    if len(data) == 1:
        return data[0]
    pos = (len(data) - 1) * q / 100.0
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(data) - 1)
    return data[lo] + (data[hi] - data[lo]) * (pos - lo)


def stat(values):
    if not values:
        return {"n": 0}
    return {
        "n": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": pct(values, 95),
        "p99": pct(values, 99),
        "min": min(values),
        "max": max(values),
    }


def iou(a, b):
    ax2, ay2 = a["x"] + a["w"], a["y"] + a["h"]
    bx2, by2 = b["x"] + b["w"], b["y"] + b["h"]
    ix = max(0.0, min(ax2, bx2) - max(a["x"], b["x"]))
    iy = max(0.0, min(ay2, by2) - max(a["y"], b["y"]))
    inter = ix * iy
    union = a["w"] * a["h"] + b["w"] * b["h"] - inter
    return inter / union if union > 0 else 0.0


def center_dist(a, b):
    return math.hypot((a["x"] + a["w"] / 2) - (b["x"] + b["w"] / 2),
                      (a["y"] + a["h"] / 2) - (b["y"] + b["h"] / 2))


def fmt(value, digits=2, unit=""):
    if value is None:
        return "—"
    return f"{value:.{digits}f}{unit}"


def verdict_tag(passed):
    return "通过" if passed else "不通过"


def status_tag(result):
    """卡片结论：显式 status 优先（未验证/阻塞），否则按 passed 判。"""
    status = result.get("status")
    return {"pass": "通过", "fail": "不通过", "unverified": "未验证",
            "blocked": "阻塞"}.get(status) or verdict_tag(result.get("passed"))


# ────────────────────────────── 数据装载 ──────────────────────────────
class CamData:
    """单路相机的全部采集数据。"""

    def __init__(self, cam):
        self.cam = cam
        self.timing = []          # [{capture, w, h, algo_w, algo_h, algo_pub, driver_ms, recv}]
        self.dets = []            # [{stamp, output, items:[{cls,cls_id,az,conf,pw,nan_*}]}]
        self.objs = []            # [{stamp, recv, items:[{cls,conf,x,y,w,h,rel}]}]
        self.cam_info = None      # {k, width, height, n}
        self.cam_info_algo = None
        self.status = []          # [dict]
        self.annotated_stamps = []
        self.n_annotated = 0

    # 原生分辨率：优先 frame_timing 的 image 尺寸，其次 camera_info
    def image_size(self):
        for t in self.timing:
            if t.get("w") and t.get("h"):
                return t["w"], t["h"]
        if self.cam_info and self.cam_info.get("width"):
            return self.cam_info["width"], self.cam_info["height"]
        return 1920, 1536


TOPIC_RE = re.compile(r"^/(?:sensors|perception)/camera/(?P<cam>[^/]+)/(?P<kind>[^/]+)$")


def load_bag(path):
    """读 bag，返回 {cam: CamData}。"""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # 没 source 环境时给一句人话
        raise SystemExit(
            f"读 bag 需要 ROS 2 环境（{exc}）：\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  source <ws>/install/setup.bash\n"
            "只想验证判定逻辑可以用：python3 vision_test/vision_eval.py --selftest"
        )

    compressed = [n for n in os.listdir(str(path)) if n.endswith((".zstd", ".zst"))]
    if compressed:
        raise SystemExit(
            f"这是压缩包（{compressed[0]}），rosbag2_py 不能直接读。先解压：\n"
            f"  ros2 bag convert -i {path} -o /tmp/unpacked --output-options <(echo '{{\"output_bags\": [{{\"uri\": \"/tmp/unpacked\", \"all\": true}}]}}')"
        )
    storage = rosbag2_py.StorageOptions(uri=str(path), storage_id="")
    converter = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cache = {}
    cams = {}
    unknown = {}

    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()
        match = TOPIC_RE.match(topic)
        if not match:
            continue
        cam, kind = match.group("cam"), match.group("kind")
        if topic not in cache:
            cache[topic] = get_message(types[topic])
        try:
            msg = deserialize_message(raw, cache[topic])
        except Exception as exc:  # noqa: BLE001
            unknown[topic] = f"{type(exc).__name__}: {exc}"
            continue
        cams.setdefault(cam, CamData(cam))
        data = cams[cam]
        recv = recv_ns / 1e9

        if kind == "frame_timing_json":
            try:
                info = json.loads(msg.data)
            except Exception:  # noqa: BLE001
                continue
            ts = info.get("timestamps", {})
            cap = ts.get("capture_stamp_ros", {})
            image = info.get("image", {})
            algo = info.get("image_algo", {})
            data.timing.append({
                "capture": cap.get("sec", 0) + cap.get("nanosec", 0) / 1e9,
                "w": image.get("width"),
                "h": image.get("height"),
                "algo_w": algo.get("width"),
                "algo_h": algo.get("height"),
                "algo_pub": bool(algo.get("published")),
                "driver_ms": (info.get("latency_ms") or {}).get("driver_latency_ms"),
                "recv": recv,
            })
        elif kind == "camera_info":
            data.cam_info = {"k": list(msg.k), "width": msg.width, "height": msg.height,
                             "d": list(msg.d), "model": str(msg.distortion_model),
                             "n": (data.cam_info or {}).get("n", 0) + 1}
        elif kind == "camera_info_algo":
            data.cam_info_algo = {"k": list(msg.k), "width": msg.width, "height": msg.height,
                                  "d": list(msg.d), "model": str(msg.distortion_model),
                                  "n": (data.cam_info_algo or {}).get("n", 0) + 1}
        elif kind == "detections":
            stamp = sec_of(msg.header.stamp)
            items = []
            for d in msg.detections:
                items.append({
                    "cls": d.class_name, "cls_id": int(d.class_id),
                    "az": float(d.azimuth), "conf": float(d.confidence),
                    "pw": float(d.pixel_width),
                    "nan_distance": math.isnan(d.distance_predict),
                    "nan_size": math.isnan(d.size_w) and math.isnan(d.size_h),
                })
            data.dets.append({"stamp": stamp, "output": sec_of(msg.output_stamp), "items": items})
        elif kind == "objects":
            stamp = sec_of(msg.header.stamp)
            items = []
            for t in msg.targets:
                items.append({
                    "cls": t.class_name, "conf": float(t.confidence),
                    "x": float(t.bbox_x), "y": float(t.bbox_y),
                    "w": float(t.bbox_width), "h": float(t.bbox_height),
                    "rel": float(t.rel_ang_deg),
                })
            data.objs.append({"stamp": stamp, "recv": recv, "items": items})
        elif kind == "status":
            try:
                data.status.append(json.loads(msg.data))
            except Exception:  # noqa: BLE001
                data.status.append({"state": "?", "detail": msg.data[:120]})
        elif kind == "annotated":
            data.n_annotated += 1
            data.annotated_stamps.append(sec_of(msg.header.stamp))

    for data in cams.values():
        data.timing.sort(key=lambda x: x["capture"])
        data.dets.sort(key=lambda x: x["stamp"])
        data.objs.sort(key=lambda x: x["stamp"])
    if unknown:
        print(f"[警告] 有消息反序列化失败：{list(unknown.items())[:2]}", file=sys.stderr)
    return cams


# ────────────────────────────── 各卡计算 ──────────────────────────────
def intervals(stamps):
    return [b - a for a, b in zip(stamps, stamps[1:])]


def calc_hz(stamps):
    """用真实时间戳算频率：1/中位周期，同时给 count/时长。"""
    if len(stamps) < 2:
        return {"n": len(stamps), "hz": None, "hz_median": None, "duration": 0.0,
                "max_gap": None, "gap_list": []}
    span = stamps[-1] - stamps[0]
    gaps = intervals(stamps)
    med = statistics.median(gaps)
    return {
        "n": len(stamps),
        "hz": (len(stamps) - 1) / span if span > 0 else None,
        "hz_median": 1.0 / med if med > 0 else None,
        "duration": span,
        "max_gap": max(gaps),
        "gap_list": gaps,
    }


def card_413(cams):
    """4.1.3 出图频率 —— 用 frame_timing 的采集时间戳，不用订阅端计数。"""
    rows = {}
    for cam, d in cams.items():
        rows[cam] = calc_hz([t["capture"] for t in d.timing if t["capture"] > 0])
    hzs = [r["hz"] for r in rows.values() if r["hz"]]
    spread = (max(hzs) - min(hzs)) if len(hzs) >= 2 else None
    passed = bool(hzs) and all(h >= MIN_HZ for h in hzs) and (spread is None or spread <= HZ_TOL_CAM3)
    return {"rows": rows, "spread": spread, "passed": passed}


def intrinsics_entry(info):
    if not info:
        return None
    k = info["k"]
    if len(k) < 4:
        return {"valid": False, "reason": "K 字段不足"}
    fx, cx, fy, cy = k[0], k[2], k[4] if len(k) > 4 else k[0], k[5] if len(k) > 5 else k[2]
    valid = fx > 0 and fy > 0
    return {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "valid": valid,
            "width": info.get("width"), "height": info.get("height"), "n": info.get("n"),
            "all_zero": all(v == 0 for v in k)}


def card_414(cams):
    """4.1.4 内参非零有效。"""
    rows = {}
    for cam, d in cams.items():
        raw = intrinsics_entry(d.cam_info)
        algo = intrinsics_entry(d.cam_info_algo)
        ok = bool(raw and raw["valid"])
        hfov = None
        if raw and raw["valid"] and raw.get("width"):
            hfov = math.degrees(2 * math.atan(raw["width"] / 2.0 / raw["fx"]))
        rows[cam] = {"raw": raw, "algo": algo, "hfov": hfov, "passed": ok}
    passed = all(r["passed"] for r in rows.values()) if rows else False
    return {"rows": rows, "passed": passed}


def card_408(cams):
    """4.1.8 字段与时间戳。"""
    problems = []
    nan_optional = 0
    total = 0
    for cam, d in cams.items():
        ref_w, ref_h = d.image_size()
        for f in d.dets:
            if not (0 <= f["stamp"]):  # 无效时间戳
                problems.append(f"{cam}: detections header.stamp 无效")
            if f["output"] < f["stamp"]:
                problems.append(f"{cam}: output_stamp < header.stamp（时钟域不一致）")
            for it in f["items"]:
                total += 1
                if not it["cls"]:
                    problems.append(f"{cam}: class_name 为空")
                if it["cls_id"] < 0:
                    problems.append(f"{cam}: class_id 为负")
                if not (0.0 <= it["conf"] <= 1.0):
                    problems.append(f"{cam}: confidence={it['conf']} 越界")
                if not (-math.pi / 2 - 1e-6 <= it["az"] <= math.pi / 2 + 1e-6):
                    problems.append(f"{cam}: azimuth={it['az']} 越界")
                if not (it["pw"] > 0):
                    problems.append(f"{cam}: pixel_width={it['pw']} 非正")
                if it["nan_distance"] or it["nan_size"]:
                    nan_optional += 1
        for f in d.objs:
            for it in f["items"]:
                total += 1
                if not it["cls"]:
                    problems.append(f"{cam}: objects class_name 为空")
                if not (it["w"] > 0 and it["h"] > 0):
                    problems.append(f"{cam}: bbox 宽高非正")
                if it["x"] < 0 or it["y"] < 0:
                    problems.append(f"{cam}: bbox 左上角为负")
                if it["x"] + it["w"] > ref_w + 1 or it["y"] + it["h"] > ref_h + 1:
                    problems.append(f"{cam}: bbox 超出 {ref_w}x{ref_h}")
                if not (-90.0 - 1e-6 <= it["rel"] <= 90.0 + 1e-6):
                    problems.append(f"{cam}: rel_ang_deg={it['rel']} 越界")
        stamps = [f["stamp"] for f in d.dets]
        dup_stamp = len(stamps) - len(set(stamps))
        if dup_stamp:
            problems.append(f"{cam}: detections 有 {dup_stamp} 条重复时间戳")
    frames = sum(len(d.dets) for d in cams.values())
    result = {"problems": problems, "samples": total, "nan_optional": nan_optional,
              "frames": frames, "passed": not problems and frames > 0}
    if total == 0:
        result["status"] = "unverified"
        result["note"] = "包内 60s 没有任何目标，字段规则无从校验"
    return result


def card_409(cams):
    """4.1.9 输出频率与状态。"""
    rows = {}
    for cam, d in cams.items():
        img = calc_hz([t["capture"] for t in d.timing if t["capture"] > 0])
        det = calc_hz([f["stamp"] for f in d.dets])
        obj = calc_hz([f["stamp"] for f in d.objs])
        diff = None
        if img["hz"] and det["hz"]:
            diff = abs(img["hz"] - det["hz"])
        states = {}
        for s in d.status:
            states[s.get("state", "?")] = states.get(s.get("state", "?"), 0) + 1
        rows[cam] = {"img": img, "det": det, "obj": obj, "diff": diff, "states": states}
    # 执行口径（2026-11-10 拍板）：F02.02 以"输出不低于设定频率"判定；
    # "与图像帧率差 ≤2 Hz" 那一项保留实测值供复核，不单独判失败。
    passed = True
    over_tol = False
    for r in rows.values():
        if not r["det"]["hz"] or r["det"]["hz"] < MIN_HZ:
            passed = False
        if r["diff"] is not None and r["diff"] > HZ_TOL_IMG2DET:
            over_tol = True
    return {"rows": rows, "passed": passed and bool(rows), "over_tol": over_tol}


def card_410(cams):
    """4.1.10 标注图与原图对应：只验 stamp 是否一一对应，框位置要人看截图。"""
    rows = {}
    for cam, d in cams.items():
        if not d.n_annotated:
            rows[cam] = {"n": 0, "ratio": None, "missing": None}
            continue
        det_stamps = set(f["stamp"] for f in d.dets)
        ann = set(d.annotated_stamps)
        hit = len(ann & det_stamps)
        rows[cam] = {"n": d.n_annotated,
                     "ratio": hit / len(ann) if ann else None,
                     "missing": len(ann - det_stamps)}
    any_data = any(r["n"] for r in rows.values())
    passed = any_data and all((r["ratio"] or 0) >= 0.99 for r in rows.values() if r["n"])
    return {"rows": rows, "passed": passed, "executed": any_data}


def angle_from_pixel(u, v, info):
    """按 camera_info 的模型把像素偏移换算成水平方位角（rad）。"""
    if not info:
        return None
    k = info.get("k") or []
    if len(k) < 6 or k[0] <= 0:
        return None
    cx, cy = float(k[2]), float(k[5])
    a0 = float(k[0])
    du, dv = u - cx, v - cy
    if info.get("model") == "scaramuzza":
        d = list(info.get("d") or [])
        a1 = float(d[0]) if len(d) > 0 else 0.0
        a2 = float(d[1]) if len(d) > 1 else 0.0
        a3 = float(d[2]) if len(d) > 2 else 0.0
        a4 = float(d[3]) if len(d) > 3 else 0.0
        rho = math.hypot(du, dv)
        f = a0 + a1 * rho + a2 * rho ** 2 + a3 * rho ** 3 + a4 * rho ** 4
        return math.atan2(du, f) if f > 1e-6 else math.atan(du / a0)
    return math.atan(du / a0)


def sorted_pairs(det_items, obj_items):
    a = sorted(det_items, key=lambda x: x["az"])
    b = sorted(obj_items, key=lambda x: x["rel"])
    if len(a) != len(b):
        return list(zip(a, b)), False
    return list(zip(a, b)), True


def card_411(cams):
    """4.1.11 方位方向与单位：符号一致 + 度/弧度自洽 + 与检测框复算值一致。"""
    rows = {}
    for cam, d in cams.items():
        det_by_stamp = {f["stamp"]: f for f in d.dets}
        sign_bad = 0
        tol_bad = 0
        pairs = 0
        count_mismatch = 0
        bbox_err = []
        fx = cx = None
        algo = d.cam_info_algo and intrinsics_entry(d.cam_info_algo)
        if algo and algo["valid"]:
            fx, cx = algo["fx"], algo["cx"]
        img_w, _ = d.image_size()
        algo_w = None
        for t in d.timing:
            if t.get("algo_w"):
                algo_w = t["algo_w"]
                break
        for f in d.objs:
            df = det_by_stamp.get(f["stamp"])
            if not df or not f["items"] or not df["items"]:
                continue
            zipped, ok = sorted_pairs(df["items"], f["items"])
            if not ok:
                count_mismatch += 1
            for det, obj in zipped:
                pairs += 1
                if det["az"] == 0 and obj["rel"] == 0:
                    continue
                if (det["az"] > 0) != (obj["rel"] > 0):
                    sign_bad += 1
                if abs(math.degrees(det["az"]) - obj["rel"]) > ANG_TOL_DEG:
                    tol_bad += 1
                img_h = d.image_size()[1]
                algo_h_cur = None
                for t in d.timing:
                    if t.get("algo_h"):
                        algo_h_cur = t["algo_h"]
                        break
                u_box = obj["x"] + obj["w"] / 2.0
                v_box = obj["y"] + obj["h"] / 2.0
                # 优先用原图内参（bbox 本身就是原图坐标）；没有就退回算法图内参并换算坐标
                raw_info = d.cam_info if (d.cam_info and (list(d.cam_info.get("k") or [0]) or [0])[0] > 0) else None
                if raw_info:
                    ang_from_box = angle_from_pixel(u_box, v_box, raw_info)
                elif algo_w and algo_h_cur and img_w and img_h:
                    ang_from_box = angle_from_pixel(
                        u_box * algo_w / img_w, v_box * algo_h_cur / img_h, d.cam_info_algo)
                else:
                    ang_from_box = None
                if ang_from_box is not None:
                    bbox_err.append(abs(det["az"] - ang_from_box))
        err_p95 = pct(bbox_err, 95)
        passed = (pairs > 0 and sign_bad == 0 and tol_bad == 0
                  and (err_p95 is None or err_p95 <= ANG_CHECK_TOL_RAD) and count_mismatch == 0)
        rows[cam] = {"pairs": pairs, "sign_bad": sign_bad, "tol_bad": tol_bad,
                     "count_mismatch": count_mismatch,
                     "bbox_err_p95": err_p95, "bbox_n": len(bbox_err), "passed": passed,
                     "fx": fx, "cx": cx, "algo_w": algo_w, "img_w": img_w}
    passed = all(r["passed"] for r in rows.values()) if rows else False
    result = {"rows": rows, "passed": passed}
    total_pairs = sum(r["pairs"] for r in rows.values())
    if not any(r["fx"] for r in rows.values()):
        result["status"] = "blocked"
        result["note"] = "camera_info_algo 内参缺失/全零，角度无法复算（依赖内参的项按阻塞处理）"
    elif total_pairs == 0:
        result["status"] = "unverified"
        result["note"] = "包内没有目标，无配对样本"
    return result


def card_412(cams):
    """4.1.12 角度与横向分辨率：按实际内参算 °/px 与 100 m 横向间距，再对正文逐项核对。"""
    rows = {}
    for cam, d in cams.items():
        raw = intrinsics_entry(d.cam_info)
        algo = intrinsics_entry(d.cam_info_algo)
        img_w, img_h = d.image_size()
        algo_w = algo_h = None
        for t in d.timing:
            if t.get("algo_w"):
                algo_w, algo_h = t["algo_w"], t["algo_h"]
                break
        if not algo_w and algo:
            algo_w, algo_h = algo.get("width"), algo.get("height")
        entry = {"img_w": img_w, "img_h": img_h, "algo_w": algo_w, "algo_h": algo_h,
                 "raw_fx": raw and raw.get("fx"), "algo_fx": algo and algo.get("fx"),
                 "raw": None, "algo": None}
        if raw and raw.get("fx"):
            entry["raw"] = angle_metrics(raw["fx"])
            entry["raw"]["scale_x"] = img_w / algo_w if algo_w else None
        if algo and algo.get("fx"):
            entry["algo"] = angle_metrics(algo["fx"])
        if raw and algo and algo.get("fx") and img_w and algo_w:
            entry["fx_ratio_expected"] = img_w / algo_w
            entry["fx_ratio_actual"] = raw["fx"] / algo["fx"]
        rows[cam] = entry
    result = {"rows": rows}
    if not any(r.get("raw") or r.get("algo") for r in rows.values()):
        result["status"] = "blocked"
        result["note"] = "内参缺失（K 全零），无法核验角度分辨率"
    return result


def angle_metrics(fx):
    center = math.degrees(2 * math.atan(0.5 / fx))
    lateral_cm = 100.0 * math.tan(math.radians(center)) * 100.0
    return {"deg_per_px": center, "lateral_cm_100m": lateral_cm}


def card_416(cams):
    """4.1.16 同一目标重复输出：同帧同类 IoU>0.5 或中心距<3%画宽。"""
    rows = {}
    for cam, d in cams.items():
        ref_w, _ = d.image_size()
        pairs = 0
        total = 0
        frames_with_dup = 0
        for f in d.objs:
            items = f["items"]
            total += len(items)
            hit = False
            for i in range(len(items)):
                for j in range(i + 1, len(items)):
                    a, b = items[i], items[j]
                    if a["cls"] != b["cls"]:
                        continue
                    if iou(a, b) > DUP_IOU or center_dist(a, b) < DUP_CENTER_RATIO * ref_w:
                        pairs += 1
                        hit = True
            if hit:
                frames_with_dup += 1
        rate = pairs / total if total else None
        passed = rate is not None and rate <= DUP_RATE_MAX
        rows[cam] = {"pairs": pairs, "total": total, "rate": rate,
                     "frames_with_dup": frames_with_dup,
                     "frames": len(d.objs), "passed": passed}
    passed = all(r["passed"] for r in rows.values()) if rows else False
    result = {"rows": rows, "passed": passed}
    if sum(r["total"] for r in rows.values()) == 0:
        result["status"] = "unverified"
        result["note"] = "包内没有目标，重复率无分母"
    return result


def track_targets(frames, ref_w):
    """贪心跨帧关联：按几何位置匹配（不按类别，否则类别翻转会被当成新目标漏计）。

    匹配条件：同类或异类都可，只要 IoU>0.3 或中心距<5%画面宽。类别变化本身记为翻转事件。
    """
    tracks = []
    active = []
    for f in frames:
        new_active = []
        for item in f["items"]:
            best = None
            best_score = 0.0
            for tr in active:
                score = iou(tr["last"], item)
                if score < 0.3:
                    if center_dist(tr["last"], item) < 0.05 * ref_w:
                        score = 0.3
                    else:
                        continue
                if score > best_score:
                    best, best_score = tr, score
            if best is None:
                tr = {"cls": item["cls"], "first_stamp": f["stamp"], "last": item,
                      "points": [(f["stamp"], item)]}
                tracks.append(tr)
                new_active.append(tr)
            else:
                best["points"].append((f["stamp"], item))
                if best["cls"] != item["cls"]:
                    best["cls_changes"] = best.get("cls_changes", []) + [f["stamp"]]
                    best["cls"] = item["cls"]
                best["last"] = item
                new_active.append(best)
        # 未匹配的旧轨迹保留一期（容忍短暂漏检）
        active_ids = {id(tr) for tr in new_active}
        for tr in active:
            if id(tr) not in active_ids:
                tr["miss"] = tr.get("miss", 0) + 1
                if tr["miss"] <= 2:
                    new_active.append(tr)
        active = new_active
    return tracks


def card_417(cams):
    """4.1.17 连续识别结果稳定性：类别翻转 / 框中心位移 / 置信度变化。"""
    rows = {}
    for cam, d in cams.items():
        ref_w, _ = d.image_size()
        frames = [f for f in d.objs]
        source = "objects"
        tracks = track_targets(frames, ref_w)
        if not any(len(tr["points"]) >= 2 for tr in tracks) and d.dets:
            # 没有 objects 时退化为用 detections 的角度反推中心
            source = "detections"
            frames = []
            fx = cx = None
            algo = d.cam_info_algo and intrinsics_entry(d.cam_info_algo)
            algo_w = None
            for t in d.timing:
                if t.get("algo_w"):
                    algo_w = t["algo_w"]
                    break
            if algo and algo["valid"] and algo_w:
                fx, cx = algo["fx"], algo["cx"]
                scale = ref_w / algo_w
                for f in d.dets:
                    items = []
                    for it in f["items"]:
                        u = (cx + fx * math.tan(it["az"])) * scale
                        items.append({"cls": it["cls"], "conf": it["conf"],
                                      "x": u - it["pw"] / 2.0, "y": 0.0,
                                      "w": max(1.0, it["pw"]), "h": 1.0})
                    frames.append({"stamp": f["stamp"], "items": items})
            tracks = track_targets(frames, ref_w)

        shifts = []
        conf_deltas = []
        flips = 0
        tracks_with_flip = 0
        long_tracks = 0
        flip_rates = []
        for tr in tracks:
            pts = tr["points"]
            if len(pts) < 2:
                continue
            shifts.extend(abs((b[1]["x"] + b[1]["w"] / 2) - (a[1]["x"] + a[1]["w"] / 2))
                          for a, b in zip(pts, pts[1:]))
            conf_deltas.extend(abs(b[1]["conf"] - a[1]["conf"]) for a, b in zip(pts, pts[1:]))
            changes = tr.get("cls_changes", [])
            flips += len(changes)
            if changes:
                tracks_with_flip += 1
            track_dur = pts[-1][0] - pts[0][0]
            if track_dur >= 10.0:
                long_tracks += 1
            # 口径：翻转次数 / 5 s / 目标（单目标观测不足 5 s 时按 5 s 计，避免小样本放大）
            flip_rates.append(len(changes) / max(track_dur, 5.0) * 5.0)
        duration = (frames[-1]["stamp"] - frames[0]["stamp"]) if len(frames) > 1 else 0.0
        shift_stat = stat(shifts)
        conf_stat = stat(conf_deltas)
        shift_ratio_p95 = (shift_stat["p95"] / ref_w) if shift_stat.get("n") else None
        flip_rate = max(flip_rates) if flip_rates else None
        criteria = {
            "flip": flip_rate is None or flip_rate <= CLASS_FLIP_MAX_PER_5S,
            "shift": shift_ratio_p95 is None or shift_ratio_p95 <= CENTER_SHIFT_MAX_RATIO,
            "conf": (conf_stat.get("p95") is None or conf_stat["p95"] <= CONF_DELTA_MAX),
        }
        rows[cam] = {"source": source, "tracks": len(tracks), "long_tracks": long_tracks,
                     "duration": duration, "flips": flips, "tracks_with_flip": tracks_with_flip,
                     "shift_p95_px": shift_stat.get("p95"),
                     "shift_p95_ratio": shift_ratio_p95,
                     "shift_max_px": shift_stat.get("max"),
                     "conf_p95": conf_stat.get("p95"),
                     "criteria": criteria,
                     "passed": all(criteria.values()) and bool(shifts)}
    passed = all(r["passed"] for r in rows.values()) if rows else False
    result = {"rows": rows, "passed": passed}
    if sum(r["tracks"] for r in rows.values()) == 0:
        result["status"] = "unverified"
        result["note"] = "包内没有目标，无法评估稳定性"
    return result


def card_419(cams, target_hz):
    """4.1.19 连续性与积压掉帧。"""
    rows = {}
    for cam, d in cams.items():
        src = sorted(set(t["capture"] for t in d.timing if t["capture"] > 0))
        out = sorted(set(f["stamp"] for f in d.dets))
        drop_capture = None      # 输出帧数 / 采集帧数，反映"丢了多少帧"
        if src and out:
            drop_capture = max(0.0, 1.0 - len(out) / len(src))
        hz = calc_hz(out)
        # F05.02 的判据口径是"相对设定频率"：采集比设定快时，主动少收不算丢帧
        drop_target = None
        if target_hz and hz["hz"] is not None:
            drop_target = max(0.0, 1.0 - hz["hz"] / target_hz)
        gaps = hz["gap_list"]
        burst = 0
        if gaps:
            med = statistics.median(gaps)
            run = 0
            for g in gaps:
                if med > 0 and g < 0.5 * med:
                    run += 1
                    burst = max(burst, run)
                else:
                    run = 0
        wave = None
        if out:
            # 首尾那一秒大概率不完整，必须剔掉，否则 p95-p5 会被算大
            lo = int(out[0]) + 1
            hi = int(out[-1]) - 1
            buckets = {}
            for stamp in out:
                bucket = int(stamp)
                if lo <= bucket <= hi:
                    buckets[bucket] = buckets.get(bucket, 0) + 1
            counts = list(buckets.values())
            if len(counts) >= 3:
                wave = pct(counts, 95) - pct(counts, 5)
        long_gaps = []
        if gaps:
            med_gap = statistics.median(gaps)
            for idx, g in enumerate(gaps):
                if med_gap > 0 and g > 2.0 * med_gap:
                    long_gaps.append((out[idx], g))
        stalls = [g for g in gaps if g >= STALL_MS / 1000.0]
        # 执行口径（2026-11-10 拍板）：以"设定频率"为基准判达标；
        # 正文的"波动 <3 Hz"与"最大间隔"改成观察项，数值照记不参与判定。
        criteria = {
            "hz": (hz["hz"] or 0) >= MIN_HZ,
            "burst": burst <= BURST_DEPTH_MAX,
            "drop": drop_target is None or drop_target <= DROP_RATE_MAX,
        }
        rows[cam] = {"n_src": len(src), "n_out": len(out),
                     "drop": drop_capture, "drop_vs_target": drop_target,
                     "hz": hz["hz"], "max_gap": hz["max_gap"], "burst": burst,
                     "wave": wave, "long_gaps": long_gaps, "stalls": stalls,
                     "criteria": criteria,
                     "passed": all(criteria.values()) and len(out) > 0}
    passed = all(r["passed"] for r in rows.values()) if rows else False
    return {"rows": rows, "target_hz": target_hz, "passed": passed}


def card_420(cams):
    """4.1.20 端到端时延（output_stamp − header.stamp）+ 驱动段。"""
    rows = {}
    for cam, d in cams.items():
        e2e = []
        negative = 0
        for f in d.dets:
            delta = (f["output"] - f["stamp"]) * 1000.0
            if delta < 0:
                negative += 1
            else:
                e2e.append(delta)
        driver = [t["driver_ms"] for t in d.timing if t.get("driver_ms") is not None]
        st = stat(e2e)
        drv = stat(driver)
        passed = st.get("n", 0) > 0 and negative == 0 and st["p95"] <= E2E_P95_MAX_MS
        rows[cam] = {"e2e": st, "negative": negative, "driver": drv, "passed": passed}
    passed = all(r["passed"] for r in rows.values()) if rows else False
    return {"rows": rows, "passed": passed}


def card_421(tegrastats_path):
    """4.1.21 资源与温度：解析 tegrastats 日志。"""
    if not tegrastats_path or not os.path.exists(tegrastats_path):
        return {"executed": False}
    cpu, gpu, ram, tj = [], [], [], []
    with open(tegrastats_path, "r", errors="ignore") as handle:
        for line in handle:
            m = re.search(r"CPU \[([^\]]+)\]", line)
            if m:
                vals = [float(x.split("%")[0]) for x in m.group(1).split(",") if "%" in x]
                if vals:
                    cpu.append(sum(vals) / len(vals))
            m = re.search(r"GR3D_FREQ (\d+)%", line)
            if m:
                gpu.append(float(m.group(1)))
            m = re.search(r"RAM (\d+)/(\d+)MB", line)
            if m:
                ram.append(100.0 * float(m.group(1)) / float(m.group(2)))
            m = re.search(r"tj@([\d.]+)C", line)
            if m:
                tj.append(float(m.group(1)))
    return {"executed": True, "cpu": stat(cpu), "gpu": stat(gpu),
            "ram_pct": stat(ram), "tj": stat(tj), "lines": len(cpu)}


# ────────────────────────────── 报告 ──────────────────────────────
def _sha256_head(path, limit=1 << 20):
    """算文件 sha256 的前 16 位（大文件分块读）。"""
    h = hashlib.sha256()
    try:
        with open(path, "rb") as handle:
            while True:
                chunk = handle.read(limit)
                if not chunk:
                    break
                h.update(chunk)
    except OSError:
        return ""
    return h.hexdigest()[:16]


def collect_evidence(dir_path, bag_path):
    """收集原始数据留存清单：报告里的每个数字都能追回到这些文件。"""
    rows = []
    seen = set()

    def add(name, path):
        if not path or path in seen or not os.path.exists(path):
            return
        seen.add(path)
        if os.path.isdir(path):
            files = [os.path.join(base, f) for base, _, fs in os.walk(path) for f in fs]
            total = sum(os.path.getsize(f) for f in files if os.path.exists(f))
            rows.append({"name": name, "path": path, "n": len(files),
                         "size": total, "sha": "—"})
        else:
            rows.append({"name": name, "path": path, "n": 1,
                         "size": os.path.getsize(path), "sha": _sha256_head(path)})

    # bag：按目录整体 + 关键文件单独留指纹
    add("bag 目录", bag_path)
    if os.path.isdir(bag_path):
        for name in sorted(os.listdir(bag_path)):
            if name.endswith((".db3", ".mcap", "metadata.yaml")):
                add(f"bag/{name}", os.path.join(bag_path, name))
    add("tegrastats", os.path.join(dir_path, "tegrastats.log"))
    add("scene.json", os.path.join(dir_path, "scene.json"))
    add("参数快照", os.path.join(dir_path, "snapshot"))
    add("节点/话题快照", os.path.join(dir_path, "graph"))
    return rows


def build_report(ctx):
    cams = ctx["cams"]
    scene = ctx["scene"]
    lines = []
    add = lines.append

    add("# 视觉二阶段测试结果（4.1.x 自动项）")
    add("")
    add("> 本报告是判分脚本按采集数据算出的结果，**不自动等于验收结论**：")
    add("> 需在固定版本、静置条件、真实目标场景下采集，并经负责人按测试计划表签字确认。")
    add("")
    add(f"- 数据：`{ctx['source']}`")
    add(f"- 采集时间：{scene.get('recorded_at', '—')}")
    add(f"- 相机：{', '.join(sorted(cams)) or '—'}")
    if scene.get("commit"):
        add(f"- 版本：commit {scene['commit']}{'（有未提交改动）' if scene.get('dirty') else ''}")
    if scene.get("engine"):
        add(f"- 模型：`{scene.get('engine')}`")
    add("")

    summary = []

    # 4.1.3
    r = ctx["c413"]
    add("## 4.1.3 相机出图与图像有效性（TC-F01.01-01）")
    add("")
    add("判据：F01.01 每路发布频率 ≥20 Hz，三路之间频率差不大于 2 Hz")
    add("")
    add("| 相机 | 消息数 | 统计时长(s) | 频率(Hz) | 中位周期频率(Hz) | 最大间隔(s) |")
    add("|---|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        add(f"| {cam} | {row['n']} | {fmt(row['duration'])} | {fmt(row['hz'])} "
            f"| {fmt(row['hz_median'])} | {fmt(row['max_gap'], 4)} |")
    add("")
    add(f"三路最高与最低频率之差：{fmt(r['spread'])} Hz")
    add(f"结论：{verdict_tag(r['passed'])}")
    add("")
    add("> 频率取自 `frame_timing_json` 的 `capture_stamp_ros`（真实采集周期），"
        "不是订阅端计数——订阅端 best_effort/depth1 会丢帧，计数会偏低。")
    add("")
    summary.append(("4.1.3", "出图频率", f"{fmt(r['spread'])} Hz 极差", "≥20 Hz / ≤2 Hz", status_tag(r)))

    # 4.1.4
    r = ctx["c414"]
    add("## 4.1.4 相机内参与标定文件（TC-F01.02-01）")
    add("")
    add("| 相机 | fx | fy | cx | cy | 消息数 | 标称水平视场 | 结论 |")
    add("|---|---|---|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        raw = row["raw"] or {}
        add(f"| {cam} | {fmt(raw.get('fx'), 1)} | {fmt(raw.get('fy'), 1)} | {fmt(raw.get('cx'), 1)} "
            f"| {fmt(raw.get('cy'), 1)} | {raw.get('n', '—')} | {fmt(row['hfov'], 1, '°')} "
            f"| {'有效' if row['passed'] else '无效/缺失'} |")
    add("")
    add(f"结论：{status_tag(r)}（K 需非零且 fx>0；全零 K 按阻塞处理，不得当有效值统计）")
    if r.get("note"):
        add("")
        add(f"> {r['note']}")
    add("")
    summary.append(("4.1.4", "内参有效", "见上表", "K 非零、fx>0", status_tag(r)))

    # 4.1.8
    r = ctx["c408"]
    add("## 4.1.8 目标消息字段与时间戳（TC-F02.03-01）")
    add("")
    add(f"- 检查样本数：{r['samples']}（detections 条目 + objects 条目）")
    add(f"- 可选字段缺失（distance_predict / size_w / size_h 为 NaN）：{r['nan_optional']} 条"
        "（按接口现状属正常，记录不当失败）")
    add(f"- 必填违规：{len(r['problems'])} 条")
    for item in r["problems"][:10]:
        add(f"  - {item}")
    if len(r["problems"]) > 10:
        add(f"  - …… 其余 {len(r['problems']) - 10} 条见 --json")
    add("")
    add(f"结论：{verdict_tag(r['passed'])}")
    add("")
    add("> 注意：检测框只在 `objects`（AIVisionTargets）里；`detections`（VisionDetection）只有 "
        "camera_id/class_name/class_id/azimuth/confidence/pixel_width。F02.03 的\"检测框\"应按 "
        "objects 话题核对。")
    add("")
    summary.append(("4.1.8", "字段与时间戳", f"违规 {len(r['problems'])} 条", "必填无违规", status_tag(r)))

    # 4.1.9
    r = ctx["c409"]
    add("## 4.1.9 目标输出频率与状态（TC-F02.02-01）")
    add("")
    add("| 相机 | 图像(Hz) | detections(Hz) | objects(Hz) | 图像-识别差(Hz) | status 状态 |")
    add("|---|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        states = ", ".join(f"{k}×{v}" for k, v in sorted(row["states"].items()))
        add(f"| {cam} | {fmt(row['img']['hz'])} | {fmt(row['det']['hz'])} "
            f"| {fmt(row['obj']['hz'])} | {fmt(row['diff'])} | {states or '未收到'} |")
    add("")
    add(f"结论：{status_tag(r)}（执行口径：识别频率 ≥ 设定 20 Hz 且 F02.04 状态正常）")
    if r.get("over_tol"):
        add("")
        add("> 正文 F02.02 另有「输出频率与图像帧率差 ≤2 Hz」一条，本轮实测已超 2 Hz（见上表「图像-识别差」列），"
            "按 2026-11-10 拍板的执行口径不参与判定，数值留档供计划负责人复核。")
    add("")
    summary.append(("4.1.9", "输出频率与状态", "见上表", "≥设定 20 Hz（差 ≤2 Hz 仅记录）", status_tag(r)))

    # 4.1.10
    r = ctx["c410"]
    add("## 4.1.10 标注图与原图对应（TC-F04.01-01）")
    add("")
    if not r["executed"]:
        add("未执行：包内没录到 `annotated`。两个前提缺一不可：①节点 `publish_debug_image:=true`；"
            "②录制时加 `--with-annotated`。注意开调试图状态下不得采 4.1.9/19/20/21 的性能数据。")
    else:
        add("| 相机 | annotated 帧数 | 与 detections 帧戳匹配率 | 未匹配 |")
        add("|---|---|---|---|")
        for cam, row in sorted(r["rows"].items()):
            add(f"| {cam} | {row['n']} | {fmt((row['ratio'] or 0) * 100, 1, '%')} | {row['missing']} |")
        add("")
        add(f"结论（仅帧对应）：{verdict_tag(r['passed'])}；框位置是否落在目标上需现场看截图。")
    add("")
    summary.append(("4.1.10", "标注图对应", "帧戳匹配" if r["executed"] else "未执行",
                    "stamp 一一对应",
                    status_tag(r) if r["executed"] else "未执行"))

    # 4.1.11
    r = ctx["c411"]
    add("## 4.1.11 方位方向与单位（TC-F03.07-01）")
    add("")
    add("| 相机 | 配对样本 | 符号不一致 | 度/弧度不自洽 | 目标数不匹配帧 | 与框复算差 P95(rad) |")
    add("|---|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        add(f"| {cam} | {row['pairs']} | {row['sign_bad']} | {row['tol_bad']} "
            f"| {row['count_mismatch']} | {fmt(row['bbox_err_p95'], 4)} |")
    add("")
    add(f"结论：{status_tag(r)}")
    if r.get("note"):
        add("")
        add(f"> {r['note']}")
    add("")
    add("> 复算差 = |消息 azimuth − 由 objects 检测框中心与 camera_info_algo 反算的角度|，"
        "容差 0.02 rad。这是不依赖人工标注的自我一致性校验。")
    add("")
    summary.append(("4.1.11", "方位符号与单位", "见上表", "符号一致、容差内", status_tag(r)))

    # 4.1.12
    r = ctx["c412"]
    add("## 4.1.12 角度与横向分辨率核验（TC-F03.07-02）")
    add("")
    add("| 相机 | 原图 | 算法图 | 原图 fx | 原图 °/px | 算法图 fx | 算法图 °/px | 原图 100 m 横向 | 算法图 100 m 横向 |")
    add("|---|---|---|---|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        raw, algo = row.get("raw") or {}, row.get("algo") or {}
        add(f"| {cam} | {row['img_w']}×{row['img_h']} | {row['algo_w']}×{row['algo_h']} "
            f"| {fmt(row['raw_fx'], 1)} | {fmt(raw.get('deg_per_px'), 4)} "
            f"| {fmt(row['algo_fx'], 1)} | {fmt(algo.get('deg_per_px'), 4)} "
            f"| {fmt(raw.get('lateral_cm_100m'), 1, ' cm')} | {fmt(algo.get('lateral_cm_100m'), 1, ' cm')} |")
    add("")
    add(f"正文对照：原图 {REF_ANG_DEG_PER_PX_RAW} °/px（100 m 约 {REF_LATERAL_CM_100M_RAW:.0f} cm）、"
        f"640 网格 {REF_ANG_DEG_PER_PX_640} °/px（100 m 约 {REF_LATERAL_CM_100M_640:.0f} cm）")
    add("")
    add("差异说明（按实际值填）：")
    for cam, row in sorted(r["rows"].items()):
        raw, algo = row.get("raw") or {}, row.get("algo") or {}
        if not raw or not algo:
            add(f"- {cam}：内参缺失，无法核验")
            continue
        add(f"- {cam}：原图实测 {fmt(raw.get('deg_per_px'), 4)} °/px vs 正文 {REF_ANG_DEG_PER_PX_RAW}"
            f"（差 {fmt(raw.get('deg_per_px', 0) - REF_ANG_DEG_PER_PX_RAW, 4)}）；"
            f"算法图实测 {fmt(algo.get('deg_per_px'), 4)} °/px vs 正文 {REF_ANG_DEG_PER_PX_640}"
            f"（差 {fmt(algo.get('deg_per_px', 0) - REF_ANG_DEG_PER_PX_640, 4)}）；"
            f"fx 缩放比 实测 {fmt(row.get('fx_ratio_actual'), 3)} / 期望 {fmt(row.get('fx_ratio_expected'), 3)}")
    add("")
    add("结论：记录（本项为分辨率核验，正文数值内部分歧需负责人确认，不直接判通过/不通过）")
    add("")
    summary.append(("4.1.12", "角分辨率核验", "见上表", "记录差异",
                    "阻塞" if r.get("status") == "blocked" else "记录"))

    # 4.1.16
    r = ctx["c416"]
    add("## 4.1.16 同一目标重复输出（TC-F03.06-01）")
    add("")
    add(f"口径：同帧同类目标对，IoU>{DUP_IOU} 或中心距<{DUP_CENTER_RATIO * 100:.0f}%画面宽 判为重复；"
        "重复率 = 重复对数 / 总目标数")
    add("")
    add("| 相机 | 总目标数 | 重复对数 | 重复率 | 有重复的帧/总帧 |")
    add("|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        add(f"| {cam} | {row['total']} | {row['pairs']} | {fmt((row['rate'] or 0) * 100, 2, '%')} "
            f"| {row['frames_with_dup']}/{row['frames']} |")
    add("")
    add(f"结论：{status_tag(r)}（F03.06 ≤5%）")
    if r.get("note"):
        add("")
        add(f"> {r['note']}")
    add("")
    summary.append(("4.1.16", "重复输出率", "见上表", "≤5%", status_tag(r)))

    # 4.1.17
    r = ctx["c417"]
    add("## 4.1.17 连续识别结果稳定性（TC-F03.04-01）")
    add("")
    add("| 相机 | 数据源 | 轨迹数 | ≥10s 轨迹 | 类别翻转 | 中心位移 P95(px) | 占画面宽 | 置信度变化 P95 |")
    add("|---|---|---|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        add(f"| {cam} | {row['source']} | {row['tracks']} | {row['long_tracks']} | {row['flips']} "
            f"| {fmt(row['shift_p95_px'], 1)} | {fmt((row['shift_p95_ratio'] or 0) * 100, 3, '%')} "
            f"| {fmt(row['conf_p95'], 3)} |")
    add("")
    add("判据：类别翻转 ≤1 次/5 s/目标；框中心位移 ≤1%/帧（≈19.2 px @1920）；置信度变化 ≤0.15/帧")
    add("")
    for cam, row in sorted(r["rows"].items()):
        c = row["criteria"]
        add(f"- {cam}：翻转 {'满足' if c['flip'] else '超限'}、中心 {'满足' if c['shift'] else '超限'}、"
            f"置信度 {'满足' if c['conf'] else '超限'}")
    add("")
    add(f"结论：{status_tag(r)}")
    if r.get("note"):
        add("")
        add(f"> {r['note']}")
    add("")
    add("> 位移包含本船转向与目标真实运动，是上界；结论需结合现场判断（正文也要求区分真实运动与算法抖动）。")
    add("")
    summary.append(("4.1.17", "稳定性", "见上表", "三项同时满足", status_tag(r)))

    # 4.1.19
    r = ctx["c419"]
    add("## 4.1.19 数据连续性与积压掉帧（TC-F05.02-01）")
    add("")
    add("| 相机 | 源帧数 | 输出帧数 | 丢帧率(对设定) | 丢帧率(对采集) | 输出频率(Hz) | 名义设定(Hz) | 积压深度(帧) | 最大间隔(s) | 卡顿次数(≥100ms) | 波动(Hz) |")
    add("|---|---|---|---|---|---|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        add(f"| {cam} | {row['n_src']} | {row['n_out']} | {fmt((row['drop_vs_target'] or 0) * 100, 2, '%')} "
            f"| {fmt((row['drop'] or 0) * 100, 2, '%')} | {fmt(row['hz'])} | {fmt(r.get('target_hz'))} "
            f"| {row['burst']} | {fmt(row['max_gap'], 4)} | {len(row.get('stalls') or [])} | {fmt(row['wave'])} |")
    add("")
    add("执行口径（2026-11-10 拍板）：①输出频率 ≥ 设定 20 Hz；②突发堆积 ≤3 帧；③丢帧率(对设定) ≤2%。"
        "正文另有「波动 <3 Hz」与「最大间隔」两条，本轮作为观察项照记、不参与判定；"
        "「丢帧率(对采集)」同理只作参考——采集 30 Hz 高于设定 20 Hz，主动少收不算丢帧。")
    add("")
    for cam, row in sorted(r["rows"].items()):
        c = row["criteria"]
        add(f"- {cam}：频率 {'满足' if c['hz'] else '不足'}、积压 {'满足' if c['burst'] else '超限'}、"
            f"丢帧(对设定) {'满足' if c['drop'] else '超限'}"
            f"（观察项：波动 {fmt(row['wave'])} Hz、最大间隔 {fmt(row['max_gap'], 3)} s、"
            f"卡顿 {len(row.get('stalls') or [])} 次）")
    add("")
    add("")
    for cam, row in sorted(r["rows"].items()):
        gaps_list = row.get("long_gaps") or []
        if gaps_list:
            top = sorted(gaps_list, key=lambda x: -x[1])[:5]
            detail = "、".join(f"{fmt(g * 1000, 0)} ms（t={fmt(t, 2)}）" for t, g in top)
            add(f"- {cam}：>2×中位周期的间隔共 {len(gaps_list)} 次，最大几次：{detail}")
    add("")
    add(f"结论：{status_tag(r)}")
    if r.get("note"):
        add("")
        add(f"> {r['note']}")
    add("")
    summary.append(("4.1.19", "连续性与积压", "见上表", "≥20Hz/积压≤3/丢帧(对设定)≤2%", status_tag(r)))

    # 4.1.20
    r = ctx["c420"]
    add("## 4.1.20 处理与端到端时效（TC-F05.03-01）")
    add("")
    add("| 相机 | 样本 | 端到端 P50(ms) | P95(ms) | P99(ms) | 最大(ms) | 负值条数 | 驱动段 P95(ms) |")
    add("|---|---|---|---|---|---|---|---|")
    for cam, row in sorted(r["rows"].items()):
        e, d = row["e2e"], row["driver"]
        add(f"| {cam} | {e.get('n', 0)} | {fmt(e.get('median'), 1)} | {fmt(e.get('p95'), 1)} "
            f"| {fmt(pct_placeholder(e, 99), 1)} | {fmt(e.get('max'), 1)} | {row['negative']} "
            f"| {fmt(d.get('p95'), 1)} |")
    add("")
    add("端到端 = `detections.output_stamp − detections.header.stamp`（header 继承了采集时间戳）；"
        "驱动段取自 `frame_timing_json.latency_ms.driver_latency_ms`")
    add("")
    add(f"结论：{verdict_tag(r['passed'])}（F05.03 P95 ≤100 ms）")
    add("")
    add("> 单帧 decode/infer/post/total 来自节点节流日志（默认每 5 s 一条），不在本表；"
        "需要时另解析启动日志。")
    add("")
    summary.append(("4.1.20", "端到端时延", "见上表", "P95 ≤100 ms", status_tag(r)))

    # 4.1.21
    r = ctx["c421"]
    add("## 4.1.21 资源与温度（TC-SUP1-01）")
    add("")
    if not r.get("executed"):
        add("未执行：未提供 tegrastats 日志（采集时用 `--tegrastats` 传入）。")
    else:
        add(f"- 采样行数：{r['lines']}")
        add(f"- CPU 均值 {fmt(r['cpu'].get('mean'), 1, '%')}，峰值 {fmt(r['cpu'].get('max'), 1, '%')}")
        add(f"- GPU 均值 {fmt(r['gpu'].get('mean'), 1, '%')}，峰值 {fmt(r['gpu'].get('max'), 1, '%')}")
        add(f"- 内存占用率 均值 {fmt(r['ram_pct'].get('mean'), 1, '%')}，峰值 {fmt(r['ram_pct'].get('max'), 1, '%')}")
        add(f"- tj 均值 {fmt(r['tj'].get('mean'), 1, '℃')}，峰值 {fmt(r['tj'].get('max'), 1, '℃')}")
        add("")
        add("结论：记录（无正文数值门槛，只记录异常与降频）")
    add("")
    summary.append(("4.1.21", "资源与温度", "已解析" if r.get("executed") else "未执行",
                    "记录", "记录" if r.get("executed") else "未执行"))

    # 未覆盖清单
    add("## 本工具未覆盖的卡（需人工/另做）")
    add("")
    add("| 卡 | 项 | 缺什么 |")
    add("|---|---|---|")
    for card, name, need in [
        ("4.1.1", "版本与配置快照", "看 record 目录下 snapshot/（param dump、commit、engine 哈希）"),
        ("4.1.2", "相机采集编译与启动", "启动日志与首次出图记录"),
        ("4.1.5", "三路前向视场覆盖", "三路安装角/方位参照（工具只给标称视场）"),
        ("4.1.6", "识别程序编译与启动", "启动日志"),
        ("4.1.7", "识别节点与话题", "看 record 目录下 graph/（节点、话题、连接）"),
        ("4.1.13", "目标检出与漏检", "人工标注（本轮不做，数值记未执行）"),
        ("4.1.14", "类别识别正确性", "人工标注"),
        ("4.1.15", "检测框与位置一致性", "人工标注（IoU 需参考框）"),
        ("4.1.18", "远距离大船识别", "船长与距离依据；另见 min_box_width_px 阈值冲突"),
        ("4.1.22", "导航相机 RTSP 推流", "拉流探测（ffprobe）与延迟样本"),
        ("4.1.23", "云台相机推流", "同上"),
        ("4.1.24", "软件异常与恢复", "受控注入（另用 fault 注入脚本）"),
    ]:
        add(f"| {card} | {name} | {need} |")
    add("")

    # 汇总
    add("## 原始数据留存（证据）")
    add("")
    evidence = ctx.get("evidence") or []
    if evidence:
        add("| 项 | 路径 | 文件数 | 大小 | sha256(前16) |")
        add("|---|---|---|---|---|")
        for row in evidence:
            add(f"| {row['name']} | `{row['path']}` | {row['n']} | {human_size(row['size'])} "
                f"| `{row['sha']}` |")
        add("")
        add("> 报告里每个数字都来自上表文件；整包校验用采集目录下的 `manifest.sha256`。")
    else:
        add("原始数据缺失：本次判分未定位到消息包/资源日志，结论不可追溯，不能作为验收依据。")
    add("")

    add("## 汇总")
    add("")
    add("| 卡 | 项 | 实测 | 判据 | 结论 |")
    add("|---|---|---|---|---|")
    for card, name, value, criteria, tag in summary:
        add(f"| {card} | {name} | {value} | {criteria} | {tag} |")
    add("")

    add("---")
    add(f"口径与阈值见脚本顶部常量；生成时间 {ctx['generated_at']}")
    return "\n".join(lines)


def human_size(num):
    for unit in ("B", "KB", "MB", "GB"):
        if num < 1024.0:
            return f"{num:.1f} {unit}" if unit != "B" else f"{int(num)} B"
        num /= 1024.0
    return f"{num:.1f} TB"


def pct_placeholder(stat_dict, q):
    return stat_dict.get(f"p{q}")


# ────────────────────────────── 自检 ──────────────────────────────
def selftest():
    """不依赖 ROS/相机，造已知数据验证判定逻辑。"""

    def make_cam(n=60, hz=30.0, shift=0.0, conf_step=0.0, flip_every=0,
                 dup=False, drop_every=0):
        """合成一路数据。u 是算法图(640)像素坐标，原生坐标按 ×3 换算。"""
        cam = CamData("front")
        stamps = [1000.0 + i / hz for i in range(n)]
        for s in stamps:
            cam.timing.append({"capture": s, "w": 1920, "h": 1536, "algo_w": 640,
                               "algo_h": 640, "algo_pub": True, "driver_ms": 3.0, "recv": s})
        cam.cam_info = {"k": [955.0, 0.0, 960.0, 0.0, 955.0, 768.0, 0.0, 0.0, 1.0],
                        "width": 1920, "height": 1536, "n": 1}
        cam.cam_info_algo = {"k": [318.33, 0.0, 320.0, 0.0, 397.9, 320.0, 0.0, 0.0, 1.0],
                             "width": 640, "height": 640, "n": 1}
        cam.status = [{"state": "running"}]
        cls = "boat"
        for k, s in enumerate(stamps):
            if drop_every and k and k % drop_every == 0:
                continue
            if flip_every and k and k % flip_every == 0:
                cls = "buoy" if cls == "boat" else "boat"
            u = 400.0 + shift * k          # 算法图像素
            w_algo = 100.0
            az = math.atan((u - 320.0) / 318.33)
            conf = min(0.99, 0.80 + conf_step * k)
            cam.dets.append({"stamp": s, "output": s + 0.035,
                             "items": [{"cls": cls, "cls_id": 0, "az": az, "conf": conf,
                                        "pw": w_algo * 3.0,
                                        "nan_distance": True, "nan_size": True}]})
            cx_native = u * 3.0
            w_native = w_algo * 3.0
            items = [{"cls": cls, "conf": conf,
                      "x": cx_native - w_native / 2.0, "y": 600.0,
                      "w": w_native, "h": 90.0, "rel": math.degrees(az)}]
            if dup:
                items.append(dict(items[0], x=items[0]["x"] + 10.0))
            cam.objs.append({"stamp": s, "recv": s, "items": items})
        return {"front": cam}

    ok = True

    def check(name, got, want):
        nonlocal ok
        good = got == want
        ok = ok and good
        print(f"  [{'OK ' if good else 'FAIL'}] {name}: got={got} want={want}")

    print("selftest 1/6 基线（30 Hz、单目标匀速、角度自洽）")
    cams = make_cam()
    check("4.1.3", card_413(cams)["passed"], True)
    check("4.1.8", card_408(cams)["passed"], True)
    check("4.1.9", card_409(cams)["passed"], True)
    check("4.1.11", card_411(cams)["passed"], True)
    check("4.1.16 无重复", card_416(cams)["passed"], True)
    check("4.1.19", card_419(cams, 20.0)["passed"], True)
    check("4.1.20", card_420(cams)["passed"], True)
    check("4.1.17 静止目标", card_417(cams)["passed"], True)

    print("selftest 2/6 频率不足（18 Hz）")
    check("4.1.3 低频率", card_413(make_cam(hz=18.0))["passed"], False)

    print("selftest 3/6 只跑出 15Hz（每 2 帧丢 1），低于设定 20 Hz")
    check("4.1.19 频率不足", card_419(make_cam(n=100, drop_every=2), 20.0)["passed"], False)
    print("selftest 3b/6 采集 30Hz、输出 28.8Hz，按设定口径应通过（旧口径判不过）")
    check("4.1.19 差两三帧", card_419(make_cam(n=100, drop_every=20), 20.0)["passed"], True)

    print("selftest 4/6 抖动（每帧偏 7 算法图像素 = 21 原生像素 > 19.2 上限）")
    check("4.1.17 抖动", card_417(make_cam(shift=7.0))["passed"], False)

    print("selftest 5/6 重复输出（同帧复制一个框，IoU≈0.94）")
    check("4.1.16 重复", card_416(make_cam(dup=True))["passed"], False)

    print("selftest 6/6 时延超限（35 ms→120 ms）与类别翻转")
    cams = make_cam()
    for f in cams["front"].dets:
        f["output"] = f["stamp"] + 0.12
    check("4.1.20 超限", card_420(cams)["passed"], False)
    check("4.1.17 类别翻转", card_417(make_cam(flip_every=2))["passed"], False)

    # 7/7：报告自动生成 + 原始数据留存段
    print("selftest 7/7 报告自动生成（含原始数据留存段）")
    import shutil
    import tempfile
    tmp = tempfile.mkdtemp(prefix="ve_selftest_")
    try:
        bag_dir = os.path.join(tmp, "bag")
        os.makedirs(os.path.join(bag_dir, "sub"))
        with open(os.path.join(bag_dir, "metadata.yaml"), "w") as handle:
            handle.write("rosbag2_bagfile_information: {}\n")
        with open(os.path.join(bag_dir, "bag_0.db3"), "wb") as handle:
            handle.write(b"\x00" * 4096)
        os.makedirs(os.path.join(tmp, "snapshot"))
        os.makedirs(os.path.join(tmp, "graph"))
        with open(os.path.join(tmp, "tegrastats.log"), "w") as handle:
            handle.write("RAM 1000/30000MB\n" * 10)
        with open(os.path.join(tmp, "scene.json"), "w") as handle:
            json.dump({"recorded_at": "2026-01-01 00:00:00"}, handle)

        ctx = build_ctx(make_cam(), bag_dir, tmp, os.path.join(tmp, "tegrastats.log"), 20.0)
        report = build_report(ctx)
        check("报告含判定段", report.count("\n## 4.1.") >= 10, True)
        check("报告含汇总段", "## 汇总" in report, True)
        check("报告含证据段", "## 原始数据留存（证据）" in report, True)
        check("证据列出 bag 指纹", "bag_0.db3" in report and "`" in report, True)
        check("证据条数", len(ctx["evidence"]) >= 3, True)
        print(f"  报告长度 {len(report)} 字符，证据 {len(ctx['evidence'])} 项")

        # 证据缺失时必须明确告警，而不是静默出结论
        ctx_missing = build_ctx(make_cam(), os.path.join(tmp, "nope"), tmp + "_nope")
        check("证据缺失时告警", "原始数据缺失" in build_report(ctx_missing), True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print()
    print("自检结果：" + ("全部符合预期" if ok else "存在不符合预期的判定"))
    return 0 if ok else 1


# ────────────────────────────── 入口 ──────────────────────────────
def resolve_bag(path):
    """接受 bag 目录、db3 文件，或包含 bag/ 的采集目录。"""
    path = os.path.abspath(os.path.expanduser(path))
    if os.path.isdir(os.path.join(path, "bag")):
        return os.path.join(path, "bag"), path
    if os.path.isfile(path) and path.endswith(".db3"):
        return path, os.path.dirname(path)
    if os.path.isdir(os.path.join(path, "metadata.yaml")):
        return path, os.path.dirname(path)
    return path, path


def load_scene(dir_path):
    for name in ("scene.json", "snapshot/scene.json"):
        candidate = os.path.join(dir_path, name)
        if os.path.exists(candidate):
            try:
                with open(candidate, "r") as handle:
                    return json.load(handle)
            except Exception:  # noqa: BLE001
                pass
    return {}


def build_ctx(cams, bag_path, dir_path, tegrastats=None, target_hz=20.0):
    """把判分结果打包成报告上下文（main 与 selftest 共用）。"""
    import datetime
    return {
        "cams": cams,
        "source": bag_path,
        "evidence": collect_evidence(dir_path, bag_path),
        "scene": load_scene(dir_path),
        "generated_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "c413": card_413(cams),
        "c414": card_414(cams),
        "c408": card_408(cams),
        "c409": card_409(cams),
        "c410": card_410(cams),
        "c411": card_411(cams),
        "c412": card_412(cams),
        "c416": card_416(cams),
        "c417": card_417(cams),
        "c419": card_419(cams, target_hz),
        "c420": card_420(cams),
        "c421": card_421(tegrastats),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="视觉二阶段测试离线判分")
    parser.add_argument("data", nargs="?", help="采集目录或 bag 路径")
    parser.add_argument("-o", "--out", help="报告输出路径（默认打印到终端）")
    parser.add_argument("--json", help="指标 JSON 输出路径")
    parser.add_argument("--tegrastats", help="tegrastats 日志路径")
    parser.add_argument("--target-hz", type=float, default=20.0, help="设定频率（默认 20）")
    parser.add_argument("--selftest", action="store_true", help="用合成数据自检判定逻辑")
    args = parser.parse_args(argv)

    if args.selftest:
        return selftest()

    if not args.data:
        parser.error("需要采集目录或 bag 路径（或用 --selftest）")

    bag_path, dir_path = resolve_bag(args.data)
    print(f"读取 bag：{bag_path}")
    cams = load_bag(bag_path)
    if not cams:
        print("包里没有 \"/sensors/camera/*\" 或 \"/perception/camera/*\" 话题，检查录制话题清单", file=sys.stderr)
        return 2

    for cam, data in cams.items():
        print(f"  {cam}: 图像帧 {len(data.timing)}、detections {len(data.dets)}、"
              f"objects {len(data.objs)}、annotated {data.n_annotated}")

    ctx = build_ctx(cams, bag_path, dir_path, args.tegrastats, args.target_hz)
    report = build_report(ctx)
    if args.out:
        with open(args.out, "w") as handle:
            handle.write(report + "\n")
        print(f"报告已写入：{args.out}")
    else:
        print()
        print(report)

    if args.json:
        keep = {k: v for k, v in ctx.items() if k.startswith("c")}
        with open(args.json, "w") as handle:
            json.dump(keep, handle, ensure_ascii=False, indent=2, default=str)
        print(f"指标已写入：{args.json}")

    auto = [ctx[k]["passed"] for k in ("c413", "c414", "c408", "c409", "c411",
                                       "c416", "c417", "c419", "c420")]
    return 0 if all(auto) else 1


if __name__ == "__main__":
    sys.exit(main())
