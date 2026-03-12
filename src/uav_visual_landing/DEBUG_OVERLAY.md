# `uav_visual_landing` Debug Overlay 说明

> 注意：本文描述的是旧版 debug overlay 与旧 `LandingError` / `LandingStatus` 契约。
> 当前实现已切换到 `TargetObservation` / `LandingControllerState`，本文内容仅可作为历史参考。

本文说明 `aruco_detector_node` 发布的 debug 图像里“现在都显示了什么”、每一项的含义、以及它们各自来自哪里。

## 1. Debug 图像是怎么生成的

当前 debug 图像由 `ws/src/uav_visual_landing/src/aruco_detector_node.cpp` 生成并发布到 `debug_image_topic`，默认话题是 `/uav/visual_landing/debug_image`。

生成链路如下：

1. 订阅相机图像 `image_topic`，默认 `/uav/camera/image_raw`
2. 转成灰度图 `MONO8`
3. 如果原图太大，按 `detect_max_dimension` 先缩小做检测
4. 在检测图上画出 AprilTag/调试标记
5. 在图像左上角叠加反色文字说明
6. 按 `debug_render_scale` 放大后发布

### 什么时候会发布 debug 图

只有同时满足下面条件时才会发布：

- `always_publish_debug == true`，或者当前帧 `detected == true`
- `debug_image_topic` 上至少有一个订阅者
- 若设置了 `debug_publish_rate_hz`，还要满足发布频率限制

### 为什么 debug 图尺寸可能和原图不一样

debug 图不一定等于原始相机分辨率，原因有两个：

- 如果原图最大边超过 `detect_max_dimension`，检测阶段会先缩小图像
- 最终显示前会再乘上 `debug_render_scale`

所以你看到的 debug 图，通常是“检测分辨率图像的可视化放大版”，而不是原始全分辨率图直接叠字。

## 2. 画面上有哪些视觉元素

除了文字，当前 debug 图像本身还会画出这些图形元素：

### 2.1 AprilTag 检测框

如果当前帧检测到标签，会调用 OpenCV 的 `cv::aruco::drawDetectedMarkers(...)` 在图上画出标签边框和 ID。

含义：

- 表示当前检测器在这张图上实际识别到了哪些 marker
- 这是检测阶段的直接结果，不经过控制节点滤波

### 2.2 图像中心十字

图像中心会画一个固定十字：

- 横线和竖线交点就是当前 debug 图像中心
- 它代表“理想对准中心”
- 后面的像素误差 `ERR` 也是相对于这个中心计算的

### 2.3 目标中心点

当目标标签被识别到时，会额外画一个实心圆点：

- 位置是目标 tag 四个角点的平均中心
- 这是当前帧估计得到的 tag 中心

### 2.4 中心连线

当目标标签被识别到时，还会从图像中心画一条线连到目标中心：

- 线越长，说明偏差越大
- 直观对应 `ERR: px=(...)`

### 2.5 左上角反色文字

当前版本不再把信息画在右侧单独面板，而是直接在图像左上角叠加“反色”文字：

- 文字区域会对原图局部像素取反
- 这样在亮背景和暗背景上都比较容易看清
- 标题和正文大小会随 `debug_render_scale` 一起变大

## 3. 文本每一行都表示什么

当前文字内容由 `composeDebugDisplay(...)` 拼接，顺序基本如下：

1. `VISUAL LANDING DEBUG`
2. `TAG: ...`
3. `ERR: ...`
4. `VL : ...` 或 `VL : status unavailable`
5. `CTRL: ...`
6. `HGT: ...`
7. `RNG: ...`
8. `Z  : ...`
9. `CMD: ...` 或 `CMD: unavailable`

下面逐项解释。

---

### 3.1 标题：`VISUAL LANDING DEBUG`

这只是固定标题，没有数值含义，用来表明这是一张视觉降落调试图。

---

### 3.2 `TAG: DETECTED/MISSING id=... conf=... rep=... span=...px`

这一行来自 `aruco_detector_node` 当前帧内部计算出的 `DetectionEstimate`，表示“检测器本帧怎么看这个标签”。

字段含义：

- `DETECTED / MISSING`
  - 当前帧有没有找到目标 marker
  - `DETECTED` 表示 `findTargetIndex(ids) >= 0`
  - `MISSING` 表示当前帧没找到目标 ID

- `id`
  - 当前匹配到的 marker ID
  - 没检测到时通常是 `-1`

- `conf`
  - 当前帧检测置信度，范围约为 `[0, 1]`
  - 不是神经网络概率，而是代码里的启发式评分
  - 由两个量组合而来：
    - 姿态解的重投影误差 `reproj_err_px`
    - 标签在图中大小 `marker_span_px`

- `rep`
  - 主姿态解的重投影误差，单位像素
  - 越小通常说明 PnP 姿态解越自洽
  - 来自 `solvePnPGeneric` / `solvePnP` 求出的候选姿态中最优解

- `span`
  - 标签在图像上的尺度，单位像素
  - 当前实现是四条边长度平均值，不是严格的外接框宽度
  - 越大说明目标在图里越“显眼”

#### `conf` 具体怎么算

当前代码逻辑是：

- 先计算尺寸置信度 `size_conf`
  - 当 `marker_span_px` 小于 `confidence_min_marker_span_px` 时更低
  - 当接近/超过 `confidence_full_marker_span_px` 时趋近于 1

- 如果姿态解有效：
  - 再计算 `reproj_conf = exp(-reproj_err_px / confidence_reproj_scale_px)`
  - 最终 `conf = clamp01(0.70 * reproj_conf + 0.30 * size_conf)`

- 如果姿态解无效：
  - 退化为 `conf = 0.35 * size_conf`

#### 这一行的数据来源

全部来自 `aruco_detector_node` 当前帧内部结果，不依赖其他节点：

- `detected` / `id`：AprilTag 检测结果
- `rep`：PnP 姿态求解 + 重投影误差
- `span`：角点几何尺寸
- `conf`：当前节点内部评分函数

#### 额外说明

如果目标被检测到但姿态求解失败，`conf` 会走“仅按目标尺寸估分”的分支。当前实现里 `rep` 这一列不会显式显示“invalid”，因此调试时要结合 `conf` 一起看，不要只盯着 `rep`。

---

### 3.3 `ERR: px=(u, v) n=(u_n, v_n) yaw=...`

这一行也是 `aruco_detector_node` 当前帧内部结果，描述目标相对图像中心的偏差。

字段含义：

- `px=(u, v)`
  - 原始像素偏差
  - 计算方式：`目标中心 - 图像中心`
  - `u > 0`：目标在图像中心右侧
  - `u < 0`：目标在图像中心左侧
  - `v > 0`：目标在图像中心下方
  - `v < 0`：目标在图像中心上方

- `n=(u_n, v_n)`
  - 归一化偏差
  - 计算方式：
    - `u_n = (center_x - img_center_x) / fx`
    - `v_n = (center_y - img_center_y) / fy`
  - 其中 `fx / fy` 来自相机内参，优先取 `CameraInfo`，否则用 fallback 参数

- `yaw`
  - 当前帧估计出的标签偏航误差量，单位弧度
  - 优先使用 PnP 姿态解出来的 yaw
  - 如果姿态解不可用，则退化为图像中边 `corner[0] -> corner[1]` 的角度
  - 最终都经过 `normalizeAngle(...)` 归一化到 `[-pi, pi]`

#### 这一行的数据来源

全部来自当前图像帧：

- `px`：目标中心和图像中心的差
- `n`：在 `px` 基础上除以 `fx/fy`
- `yaw`：来自 PnP 解或角点边方向

#### 和控制器的关系

控制节点 `visual_landing_node` 不直接用 `px=(...)`，而是订阅 `LandingError` 消息中的归一化误差和 yaw 误差，再做滤波和控制。

---

### 3.4 `VL : active=Y/N stage=... tag=Y/N age=...s`

这一行来自 `visual_landing_node` 发布的 `LandingStatus`。

字段含义：

- `active`
  - 视觉降落控制是否处于激活状态
  - `Y` 表示控制节点正在工作
  - `N` 表示控制未激活或已结束

- `stage`
  - 当前状态机阶段
  - 可能值来自 `stateName(...)`：
    - `READY`
    - `IDLE`
    - `ALIGN`
    - `SEARCH_RISE`
    - `DESCEND`
    - `LAND`
    - `DONE`

- `tag`
  - 控制节点视角下，最近是否认为目标可见
  - 注意这不是“当前 detector 本帧一定检测到”的同义词，而是 `visual_landing_node` 最近一次接收/接受到有效 `LandingError` 后维护的状态

- `age`
  - 当前显示的这条 `LandingStatus` 距今多久
  - 优先用 `LandingStatus.header.stamp`
  - 如果 header 时间戳为零，则退化为 `aruco_detector_node` 收到该消息的本地时间

#### 这一行的数据来源

话题来源：`status_topic`，默认 `/uav/visual_landing/status`

字段生产者：`visual_landing_node`

---

### 3.5 `CTRL: ef=(u_f, v_f) yaw_f=... lost=...s`

这一行来自 `LandingStatus`，表示控制器内部真正用于控制的“滤波后误差”和“丢失持续时间”。

字段含义：

- `ef=(u_f, v_f)`
  - filtered error，滤波后的归一化图像误差
  - 不是原始像素误差，也不是未滤波的 `err_u_norm_raw / err_v_norm_raw`
  - 当前控制器对 `u/v` 使用一阶低通：
    - `err_u_f = alpha_xy * err_u + (1 - alpha_xy) * err_u_f`
    - `err_v_f = alpha_xy * err_v + (1 - alpha_xy) * err_v_f`

- `yaw_f`
  - 滤波后的 yaw 误差，单位弧度
  - 不是简单平均，而是带预测项的更新：
    - 先根据 `yaw_error_rate_f_` 做一步预测
    - 再用当前观测创新量 `nu` 做修正
    - 如果创新过大，还会拒绝这次更新，保留预测值

- `lost`
  - 距离上一次“控制节点认为 tag 有效可用”的时间已经过去多久
  - 数值越大通常说明目标持续丢失越久

#### 这一行的数据来源

- 原始观测：来自 `visual_landing_node` 订阅 `LandingError`
- 滤波结果：由 `visual_landing_node::updateFilteredErrors(...)` 生成
- 最终通过 `LandingStatus` 发给 `aruco_detector_node` 叠加显示

#### 为什么这一行很重要

如果 `ERR` 看起来有跳动，而 `CTRL` 比较平滑，说明控制节点滤波在工作；真正拿去算速度指令的更接近 `CTRL` 而不是 `ERR`。

---

### 3.6 `HGT: est=...m use=range/none raw=...m q=...`

这一行来自 `LandingStatus`，表示控制器当前用于 z 控制的高度摘要。

字段含义：

- `est`
  - 当前控制器认定的有效高度，单位米
  - 对应 `LandingStatus.current_height_m`

- `use`
  - 当前 z 控制实际使用的高度源
  - `range` 表示测距有效，控制器正在用它闭环
  - `none` 表示当前没有有效测距，控制器不会退回到 odom 高度

- `raw`
  - 最近一次测距原始高度值，单位米
  - 对应 `LandingStatus.range_height_m`

- `q`
  - 测距信号质量 `range_signal_quality`

#### 怎么看这一行

- `use=range` 且 `est≈raw`，表示当前 z 控制链路工作正常
- `raw` 有值但 `use=none`，通常说明数据过旧或超出量程，而不是控制器切到了 odom
- `q` 很低时，即使 `raw` 看起来像有值，也要结合 `RNG` 一起看是否真的被控制器接受

---

### 3.7 `RNG: ok=Y/N fresh=Y/N in=Y/N age=...s`

这一行同样来自 `LandingStatus`，表示控制器对测距高度“能不能拿来控 z”的判断。

字段含义：

- `ok`
  - 当前测距是否已经被控制器接受并用于 z 闭环
  - 这里等价于 overlay 里 `HGT.use=range`

- `fresh`
  - 数据是否足够新鲜
  - 由 `height_source_timeout_s` 判断

- `in`
  - 数据值是否在允许范围内
  - 由 `height_range_min_m <= range <= height_range_max_m` 判断

- `age`
  - 控制器视角下，这份测距数据已经多老
  - 对应 `LandingStatus.range_age_s`

#### 怎么看这一行

- `ok=N fresh=N`：通常是测距消息断流或延迟过大
- `ok=N in=N`：通常是测距值超出当前允许量程
- `fresh=Y in=Y ok=Y`：说明 z 控制此刻确实建立在测距上

---

### 3.8 `Z  : mode=... vz=... sem=...`

这一行是 detector 节点做的“z 轴控制语义总结”，用于快速判断当前 `vz` 到底是在干什么。

字段含义：

- `mode`
  - 当前 z 轴动作模式，由 `LandingStatus.state` 和测距是否有效共同推断
  - 主要取值：
    - `OFF`：控制器未激活
    - `WAIT`：当前没有有效测距，z 控制在等待高度源恢复
    - `HOVER`：当前阶段不做特殊 z 动作
    - `HOLD`：`ALIGN` 阶段，保持进入对齐时的高度
    - `RISE`：`SEARCH_RISE` 阶段，丢标后上升尝试重找目标
    - `DOWN`：`DESCEND` 阶段，持续下降
    - `LAND`：已切到降落服务阶段
    - `DONE`：降落链已结束

- `vz`
  - 最近一次收到的 `CMD.vz`
  - 如果当前还没收到过速度命令，则显示 `n/a`

- `sem`
  - 一句短语义标签，帮助你快速把 `mode` 和状态机职责对上
  - 例如 `keep_height` / `search_up` / `descend` / `need_range`

#### 这一行为什么重要

`CMD.vz` 只告诉你“发了多少”，但 `Z` 这一行告诉你“为什么发这个值”。

例如：

- `mode=HOLD`：表示当前处于 `ALIGN`，z 在锁高度，不是在主动下沉
- `mode=RISE`：表示当前在丢标补高，不是 z 控制失控乱飞
- `mode=WAIT`：表示当前没有有效测距，先查 `RNG`，再查传感器输入

---

### 3.9 `CMD: vx=... vy=... vz=... wz=... age=...s`

这一行是 `aruco_detector_node` 直接订阅速度指令话题后显示出来的，不经过 `LandingStatus`。

字段含义：

- `vx`
  - 世界系或上层约定下的 X 方向速度指令，单位 m/s
- `vy`
  - Y 方向速度指令，单位 m/s
- `vz`
  - Z 方向速度指令，单位 m/s
- `wz`
  - 偏航角速度指令，单位 rad/s
- `age`
  - 从 `aruco_detector_node` 最近一次收到这条速度指令到现在过去了多久

#### 这一行的数据来源

话题来源：`velocity_topic`，默认 `/uav/control/velocity`

注意：

- 这里显示的是“节点收到的最新指令”，不是“飞控实际执行出来的真实速度”
- `age` 使用的是 `aruco_detector_node` 收到消息时的本地时间，不是消息头时间

如果没有收到过速度命令，就显示 `CMD: unavailable`。

## 4. 哪些行来自哪个节点/话题

可以按下面方式理解整个 overlay 的来源：

### 4.1 来自当前帧图像即时计算

这些是 `aruco_detector_node` 自己算的：

- `TAG`
- `ERR`
- 检测框
- 图像中心十字
- 目标中心点
- 中心到目标的连线

对应输入：

- `image_topic`
- `camera_info_topic`

### 4.2 来自 `visual_landing_node` 的状态汇总

这些是控制节点发布 `LandingStatus` 后，再被 `aruco_detector_node` 订阅回来显示的：

- `VL`
- `CTRL`
- `HGT`
- `RNG`

对应默认话题：

- `/uav/visual_landing/status`

### 4.3 detector 节点本地拼接的执行摘要

这些是 `aruco_detector_node` 基于已接收消息在本地拼出来的操作员视角摘要：

- `Z`
- `CMD`

其中：

- `Z` = `LandingStatus.state` + 测距可用性 + 最新 `CMD.vz`
- `CMD` = 直接订阅 `/uav/control/velocity`

## 5. `age`、`Y/N` 和单位怎么理解

### 5.1 `Y/N`

当前实现里布尔值统一显示成：

- `Y` = true
- `N` = false

### 5.2 `age`

overlay 里有多种 `age`，它们不是同一个来源：

- `VL age`
  - `LandingStatus` 自己的时间戳年龄
- `RNG age`
  - `visual_landing_node` 评估 range 数据时算出来的年龄
- `CMD age`
  - `aruco_detector_node` 收到速度命令后的本地时间差

所以两个 `age` 看起来接近，不代表它们语义完全一样。

### 5.3 单位

- `px`：像素
- `n=(...)`：按焦距归一化后的无量纲误差
- `yaw` / `yaw_f` / `wz`：弧度或弧度每秒
- `vx vy vz`：米每秒
- `range` / `est`：米

## 6. 相关参数

下面这些参数会直接影响当前 debug overlay 的观感或更新行为：

- `debug_render_scale`
  - 决定整张 debug 图的最终放大倍数
  - 也会影响当前叠加文字的字号和间距

- `always_publish_debug`
  - 为 `true` 时，即使没有检测到目标也持续发布 debug 图

- `debug_publish_rate_hz`
  - debug 图发布频率上限
  - `0` 表示不限速

- `detect_max_dimension`
  - 检测阶段用图的最大边长度
  - 值越小，检测图越小，debug 图的几何精度也越偏向缩小后的图

- `image_topic`
- `camera_info_topic`
- `status_topic`
- `velocity_topic`

### 关于 `debug_panel_width_px`

当前 overlay 已经不再使用右侧独立 panel，而是直接叠字在图像上，因此 `debug_panel_width_px` 对当前显示基本没有实际作用，可视为遗留参数。

## 7. 实际调试时怎么解读这张图

可以按下面顺序看：

1. **先看图形**
   - 检测框有没有
   - 目标中心点和中心连线偏多少

2. **再看 `TAG`**
   - `DETECTED` 还是 `MISSING`
   - `conf` 高不高
   - `span` 是否太小

3. **再看 `ERR` 和 `CTRL` 是否一致**
   - `ERR` 是当前帧原始观测
   - `CTRL` 是控制器真正更信任的滤波结果

4. **再看 `VL stage`**
   - 控制器当前到底卡在哪个阶段

5. **z 轴问题优先看 `HGT + RNG + Z`**
   - `HGT` 看控制器此刻拿到的有效高度和原始测距值
   - `RNG` 看控制器有没有接受这份测距
   - `Z` 看当前 `vz` 的语义到底是锁高、补高、下降，还是在等测距恢复

6. **机体不动/响应怪时看 `CMD`**
   - 确认控制器是否真的发出了速度指令
   - 再对照 `Z` 判断这条 `vz` 是否符合当前阶段职责

## 8. 当前实现里值得注意的两个细节

### 8.1 `TAG conf` 是“检测器当前帧信心”，不是控制器状态机信心

虽然 `LandingStatus` 里也带有 `detection_confidence`，但 overlay 目前显示的是 detector 节点本帧的 `estimate.confidence`，也就是当前帧即时计算值。

### 8.2 `HGT.use`、`RNG.ok` 和 `Z.mode` 现在是看 z 控制最关键的三项

当前 `visual_landing_node` 的垂直控制只使用测距高度，因此：

- `HGT.use=range` 表示控制器当前正在用有效测距高度闭环
- `RNG.ok=Y` 表示这份测距已经通过了“新鲜 + 量程内”的控制判定
- `Z.mode=HOLD/RISE/DOWN` 用来解释当前 `vz` 的职责，而不是只看数值正负猜含义
- `raw=...` 有值但 `HGT.use=none` 时，优先检查 `RNG.fresh` 和 `RNG.in`，不要再怀疑控制器退回了 odom
