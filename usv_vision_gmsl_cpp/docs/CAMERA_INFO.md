# 相机内参（camera_info）说明

本节点通过 `camera_info_url` 参数加载相机标定文件，发布到
`/sensors/camera/{sensor_name}/camera_info`（消息类型 `sensor_msgs/CameraInfo`）。
下游做去畸变、单目测距、像素→方位角换算时会用到这些内参。

内参加载逻辑实现在 `include/usv_vision_gmsl_cpp/camera_info_loader.hpp`，
与 Python 侧 `usv_vision/common/camera_info.py` 的 `CameraInfoLoader` 行为对齐。

---

## 一、三级兜底优先级

节点按以下顺序决定发布什么内参，保证任何情况下都能出图、不会崩：

1. **标定文件**（最准）：`camera_info_url` 指向的 JSON 存在且可解析 → 用实测内参
2. **规格书 FOV 反算**（近似）：无标定文件，但配了 `fov_x_deg`/`fov_y_deg` → 用视场角反算近似内参（无畸变）
3. **空标定**（兜底）：以上都没有 → K/D 全零、`distortion_model` 为空，节点照常出图

运行时日志会打印当前用的是哪一级：
- `loaded camera info: <path>` → 用了标定文件
- `no calibration file, using spec-sheet FOV fallback: ...` → 用了 FOV 反算
- 无相关日志 → 空标定

---

## 二、标定文件格式与字段含义

JSON 格式，参考 `config/camera_info_sample.json`：

```json
{
  "resolution":     { "width": 1920, "height": 1080 },
  "focal_length":   { "fx": 391.008, "fy": 385.367 },
  "principal_point":{ "cx": 1007.179, "cy": 562.595 },
  "dist_coeffs":    [[ 0.0588, -0.0186, 0.00954, -0.00867, 0.00303 ]]
}
```

| 字段 | 必需 | 含义 | 单位 | 映射到 CameraInfo |
|---|---|---|---|---|
| `focal_length.fx` | ✅ | x 方向焦距 | **像素** | `K[0]` |
| `focal_length.fy` | ✅ | y 方向焦距 | **像素** | `K[4]` |
| `principal_point.cx` | ✅ | 主点（光心）x 坐标 | **像素** | `K[2]` |
| `principal_point.cy` | ✅ | 主点（光心）y 坐标 | **像素** | `K[5]` |
| `resolution.width/height` | 建议 | 标定时的图像分辨率 | 像素 | 用于自动缩放（见第四节） |
| `dist_coeffs` | 可选 | 畸变系数 | 无量纲 | `D`，`distortion_model="plumb_bob"` |

**字段别名**（兼容 OpenCV 常见命名，任选其一）：
- `dist_coeffs` ⇔ `distortion_coefficients`
- `resolution.width/height` ⇔ 顶层 `image_width`/`image_height`

畸变系数支持嵌套数组 `[[...]]`（取第一行）或扁平数组 `[...]`；顺序为 OpenCV 的
`plumb_bob` 模型：`[k1, k2, p1, p2, k3]`（径向 k1/k2/k3、切向 p1/p2）。

其它字段（如 `fov`、`rms_error`、`timestamp`、`selected_stream` 等）会被**忽略**，
留在文件里不影响加载——所以 OpenCV 标定脚本直接产出的完整 JSON 可原样使用。

### 焦距单位说明（像素 vs 毫米）

标定得到的 fx/fy 单位是**像素**，不是物理焦距（mm）。两者关系：

```
fx = F_mm / pixel_size_mm_x      （F_mm 为镜头物理焦距，pixel_size 为单像素物理尺寸）
```

规格书通常只给物理焦距（mm）和视场角（度），不直接给像素焦距，
所以规格书数据走的是下面的 FOV 反算路径，而非直接填 fx/fy。

---

## 三、规格书 FOV 兜底（无标定时的近似方案）

实船相机还没来得及标定时，可在 config 里填规格书标称视场角作保底：

```yaml
/**:
  ros__parameters:
    camera_info_url: ""      # 留空，触发兜底
    fov_x_deg: 135.68        # 规格书水平视场角(度)
    fov_y_deg: 108.97        # 规格书垂直视场角(度)
```

反算公式（针孔模型，光心取图像中心）：

```
fx = (width  / 2) / tan(fov_x_deg / 2 · π/180)
fy = (height / 2) / tan(fov_y_deg / 2 · π/180)
cx = width  / 2
cy = height / 2
畸变按 0 处理（distortion_model = "plumb_bob"，D 全零）
```

- 只填一个方向（另一个填 0）也可用，缺的方向用已算出的焦距补齐，避免 0 焦距
- 两者都为 0（或都不填）→ 退回空标定
- **这是近似值**：无畸变校正、假设主点在正中，精度不如实测标定，仅供粗略估算过渡用

单位提醒：`fov_x_deg`/`fov_y_deg` 单位是**度（degree）**，不是弧度。

---

## 四、分辨率自动缩放

当标定分辨率与节点实际出图分辨率不一致时，内参按比例自动缩放，
无需为每个分辨率重新标定：

```
sx = 实际width  / 标定width
sy = 实际height / 标定height
fx *= sx;  cx *= sx
fy *= sy;  cy *= sy
```

触发时日志打印 `camera info resized from <标定WxH> to <实际WxH>`。
例：标定 1920×1080、出图 1920×1536 → fy/cy 乘以 1536/1080≈1.422，fx/cx 不变（宽度相同）。

---

## 五、如何使用（不改代码）

内参路径通过配置传入，换相机/换标定只改 yaml，代码无需改动：

```yaml
/**:
  ros__parameters:
    camera_info_url: "/home/ww/音乐/miwen/usv_vision_gmsl_cpp/config/camera_info_sample.json"
```

- 路径为**普通文件绝对路径**，不是 ROS 的 `file://` URL
- 留空 `""` → 进入 FOV / 空标定兜底
- 文件不存在或解析失败 → 打 ERROR 日志并降级兜底，节点不崩
