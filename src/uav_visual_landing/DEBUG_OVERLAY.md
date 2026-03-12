# `uav_visual_landing` Debug Overlay 说明

当前 debug 图像由 `ws/src/uav_visual_landing/src/aruco_detector_node.cpp` 生成，默认发布到 `/uav/visual_landing/debug_image`。

## 当前叠字行的含义

1. `VISUAL LANDING DEBUG`
2. `PHASE: ...`
3. `TAG: DETECTED/MISSING`
4. `TAG_Z=... src=... conf=...`
5. `H_CTRL=... H_MEAS=... src=...`
6. `ERR n=(..., ...)`
7. `ERR_F=(..., ...)`
8. `DERR=(..., ...)`
9. `LERR=... xy=(..., ...)`
10. `TH=0.08/0.05 VXY_MAX=...`
11. `ZTG=... ZE=...`
12. `YAW=... rep=... span=...`

## 字段解释

- `TAG_Z`
  - 当前视觉侧估计的 tag 轴向深度，而不是相机到 tag 中心的斜距
  - `src=PNP_Z` 表示使用 `tvec.z`
  - `src=SPAN_Z` 表示使用 `tag_size * focal_mean / marker_span_px`

- `H_CTRL`
  - 控制器当前真正用于 `z` 闭环的高度
  - 该值优先来自光流测距，测距失效且仍高于 `terminal_entry_height_m` 时才回退到 odom

- `H_MEAS`
  - 最近一次收到的高度测量原始值
  - 是否真的用于控制要结合 `src` 和控制相位一起看

- `src`
  - `FLOW_RANGE` 表示当前 `z` 控制使用光流测距高度
  - `ODOM` 表示当前 `z` 控制退回到 odom 高度

- `ERR n`
  - 当前帧原始归一化像面误差

- `ERR_F`
  - 控制器滤波后的归一化像面误差

- `DERR`
  - 控制器估计的归一化误差变化率

- `LERR`
  - 基于当前 `H_CTRL` 投影得到的真实横向偏差，单位米
  - `xy=(x, y)` 分别对应图像 `u / v` 误差换算后的米制偏差

- `TH=0.08/0.05`
  - 对齐窗迟滞阈值，分别表示“进入对齐窗 8 cm”和“保持对齐窗 5 cm”

- `ZTG`
  - 当前 `z` 目标高度

- `ZE`
  - `z` 目标高度与 `H_CTRL` 之间的误差

- `YAW`
  - 当前 yaw 误差

- `rep`
  - PnP 重投影误差，单位像素

- `span`
  - 标签平均边长对应的像素尺度，单位像素

## 如何解读当前链路

- `TAG_Z` 只用于视觉诊断，不直接作为主控高输入
- `H_CTRL` 才是控制器实际拿去做 `z` 闭环的高度
- `LERR` 才是当前对齐判据对应的真实米制偏差
- 当高度进入 `terminal_entry_height_m` 以下时，若 `src` 不能保持 `FLOW_RANGE`，控制器会停止继续下降并退回等待
