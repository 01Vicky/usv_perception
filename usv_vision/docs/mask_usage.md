# 视图蒙板（View Mask）使用说明

推理蒙板用于**在送入模型前屏蔽视野中不想被识别的区域**（例如船头、甲板、
玻璃反光等）。被屏蔽的区域填成中性灰后不参与识别；非屏蔽区照常识别。

> 本说明针对 `usv_vision` 内的 yolo_infer_node / capture_mask_frame / make_mask /
> draw_mask / verify_mask_recognition。本机脱 ROS 生成/验证工具用 venv-trt 解释器
> （`/home/ww/venv-trt/bin/python`，含 cv2/numpy/ultralytics）。

---

## 一、蒙板约定（重要）

| 图像像素 | 含义 |
|---|---|
| `0`（黑） | **识别区**（保留，正常识别） |
| `>= mask_ignore_min`（你涂的，默认白/非黑） | **忽略区**（填入 `mask_fill` 灰，不识别） |

- 蒙板是**纯输入蒙板**：只在推理前填充，检出后不做输出过滤，
  避免误删“跨边界”的真实目标。
- 默认 `mask_ignore_min=1`、`mask_fill=114`（Ultralytics letterbox 中性灰），通常不用改。

对应 yolo_infer_node 参数：

| 参数 | 说明 |
|---|---|
| `enable_mask` | 总开关（true/false） |
| `mask_path` | 形式一：蒙板图像路径（空=不用） |
| `mask_polygons_json` | 形式二：归一化多边形 JSON（空=不用） |
| `mask_ignore_min` | 像素值 >= 此值视为忽略区（默认 1） |
| `mask_fill` | 忽略区填充灰 0~255（默认 114） |

两种形式可叠加，内部取**并集**。

### 蒙板文件存放 / 命名
- 蒙板统一放在源码目录 `usv_vision/config/masks/`，按相机命名（如 `front.png`）。
  该目录已含样板 `front.png`（可用作起点）。
- 样板也安装到 `share/usv_vision/config/masks/`。建议复制到可写的标定目录后修改，
  再通过 `mask_path` 指定绝对路径；不要直接改安装目录中的样板。

### 路径解析（重要）
- `mask_path` 填**相对路径**时以节点运行目录为基准。一般会在工作区根
  工作区根目录下 `ros2 run/launch`，此时用工作区根相对路径
  `src/usv_perception/usv_vision/config/masks/front.png`（当前 yaml 默认值）可命中。
- 若运行目录不同或部署机路径不同，**请填绝对路径**，以免找不到蒙板。
- 工具里的**输入原图**建议用绝对路径（截图/底图存哪就用哪），**输出蒙板**落到
  `config/masks/front.png`。

---

## 二、截图做底图：capture_mask_frame

相机已运行并发布图像、且话题与节点 `sensor_name` 对应时，先截一帧作底图：

```
ros2 run usv_vision capture_mask_frame --ros-args \
  -p image_topic:=/sensors/camera/front/image_raw \
  -p output:=/home/ww/usv_ws_test/src/usv_perception/usv_vision/config/masks/front_base.png
```

> `image_topic` 为空会报错退出；需给实际话题。默认话题约定：
> `sensors/camera/{sensor_name}/image_raw`。

---

## 三、生成蒙板：make_mask

蒙板生成工具 `make_mask`（scripts/make_mask.py）。载入一张底图，把要去掉的
区域按矩形或多边形涂白，输出“黑=识别区 / 白=忽略区”的蒙板 png。

### 形式一：像素矩形（--rect，可多次）
```
# ros2 方式
ros2 run usv_vision make_mask \
  --image /home/ww/image.jpg --out config/masks/front.png \
  --rect 0,0,640,80 \
  --preview /tmp/bow_mask_preview.png

# 脱 ROS 方式（venv-trt）
/home/ww/venv-trt/bin/python scripts/make_mask.py \
  --image /home/ww/image.jpg --out config/masks/front.png --rect 0,0,640,80
```
`--rect X,Y,W,H`：忽略以 (X,Y) 为左上角、宽 W 高 H 的像素矩形。

### 形式二：归一化多边形（--polygons）
```
ros2 run usv_vision make_mask \
  --image /home/ww/image.jpg --out config/masks/front.png \
  --polygons '[[[0.0,0.6],[1.0,0.6],[1.0,1.0],[0.0,1.0]]]' \
  --preview /tmp/bottom_mask_preview.png
```
坐标归一化到 `[0,1]`、左上角为原点，多边形**内部=忽略区**。
多个多边形写在同一 JSON 里：`[[[p1...]],[[p2...]],...]`。

### 检查
`--preview out.png` 会输出叠加预览，肉眼确认盖的区域正确后再用于节点。

输出蒙板图本身是灰度 png，可用任意图像软件查看：
黑色=识别区，白色=忽略区。

### 画板/涂刷方式（draw_mask）
想直接在图上用鼠标/涂画忽略区，用 `draw_mask`（scripts/draw_mask.py）。底层同样是“黑画布+白笔迹”，原图只作半透明参考，不会污染蒙板。

**交互窗口模式**（需本机有显示）：
```
ros2 run usv_vision draw_mask --image /home/ww/image.jpg --out config/masks/front.png
# 或脱 ROS:
/home/ww/venv-trt/bin/python scripts/draw_mask.py --image /home/ww/image.jpg --out config/masks/front.png
```
- 左键拖动画白(忽略区)，右键或 `e` 擦除，`[` `]`/滚轮调笔刷，`r` 清空，`s` 保存退出，`Esc`/`q` 放弃。
- 窗口会**自动缩小到完整显示整幅图**（长边默认限制 1000px，可用 `--max-side` 调）。
  拖动窗口边角放大窗口即可放大看清细节，笔刷落点始终映射回原图；`s` 保存的是**全分辨率**蒙板，不受显示缩放影响。

**引导图模式**（无显示 / 想用外部画图软件涂）：
```
ros2 run usv_vision draw_mask --image /home/ww/image.jpg --out /tmp/guide.png --guide
```
生成一张“黑底 + 暗化原图”的同尺寸模板；在任意画图软件里把它上面的忽略区**涂成纯白**，另存为 `/tmp/painted.png`，再：
```
ros2 run usv_vision draw_mask --convert /tmp/painted.png --out config/masks/front.png
```
白色笔迹(默认≥200) → 忽略区 255，其余 → 0(黑/识别区)。这就是可直接用的蒙板。

---

## 四、在 yolo_infer_node 中启用

改 `config/yolo_infer.yaml`（或运行时 launch 覆盖参数）。
> 当前默认 `enable_mask: false`（默认全图识别 + 类别过滤正常）；要用蒙板时才改 true。

### 用 mask 图像
```yaml
enable_mask: true
mask_path: "src/usv_perception/usv_vision/config/masks/front.png"
            # 工作区根相对路径(在 /home/ww/usv_ws_test 运行); 部署机/目录不同请用绝对路径
mask_polygons_json: ""
mask_ignore_min: 1
mask_fill: 114
```

### 用多边形 JSON（无需图片文件）
```yaml
enable_mask: true
mask_path: ""
mask_polygons_json: "[[[0.0,0.6],[1.0,0.6],[1.0,1.0],[0.0,1.0]]]"
mask_ignore_min: 1
mask_fill: 114
```

launch 里用参数文件直接带上即可，例如：
```
ros2 run usv_vision yolo_infer_node --ros-args \
  --params-file <your_workspace>/install/usv_vision/share/usv_vision/config/yolo_infer.yaml
```

---

## 五、运行时热更新（不重启换蒙板）

节点提供 reload 服务。同路径覆盖蒙板文件后可直接调用；更换参数时，先通过
`ros2 param set` 设置 `mask_path`、`mask_polygons_json` 或 `enable_mask`，再调用。
参数会在 reload 时重新读取，下一帧重建蒙板。

```
ros2 service call /perception/camera/front/reload_mask std_srvs/srv/Trigger
```

服务名由状态话题推导：`{infer_status_topic 去掉 /status}/reload_mask`。
响应成功即会在下一帧重建蒙板。

---

## 六、确认生效

订阅状态话题（默认 `perception/camera/{sensor_name}/status`）：

```
ros2 topic echo /perception/camera/front/status
```

正常时可见：
```json
"enable_mask": true,
"mask_path": "src/usv_perception/usv_vision/config/masks/front.png",
"mask_polygons_count": 0,
"mask_active": true
```
并在节点首次进帧看到日志：
```
view mask active (WxH): N ignore px
```

> `mask_active` 为 false 的常见原因：`enable_mask` 未开、`mask_path` 文件不存在/
> 加载失败、且无 `mask_polygons_json`（无任何蒙板来源）。开启后若无来源，
> 节点会告警并“不带蒙板继续跑”。

---

## 七、离线验收（可选，便于出证据）

`verify_mask_recognition`（scripts/verify_mask_recognition.py）用同一张图跑
“无蒙板 vs 有蒙板”两次推理，输出对比图证明效果：

```
/home/ww/venv-trt/bin/python scripts/verify_mask_recognition.py \
  --image /home/ww/image.jpg --mask config/masks/front.png \
  --model /path/to/model.pt --conf 0.25 \
  --out test/output/mask_recognition
```

- 产出 `{image}__{mask}_nomask.png` 与 `{image}__{mask}_mask.png`
  （忽略区会红色高亮）。
- 验收标准：**蒙板盖住的目标在 mask 下消失；识别区目标仍保留**。

> 模型需能在所用解释器内加载（本机脱 ROS 用 venv-trt；`.engine` 可能与本机
> TensorRT 不匹配导致无法反序列化，可换 `.pt`/`.onnx`）。
