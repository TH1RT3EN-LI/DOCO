# 相对位置融合模块：面向 Codex agent 的执行指南

本文档面向后续真正实现 `relative_position_fusion` 的 Codex agent。
目标是把实现步骤拆成一系列明确、可验证、低耦合的小阶段，避免一次性大改导致失控。

本文件只讲**怎么动手实现**，不重新解释高层设计。算法与接口请分别查阅：

- `ws/README.relative_position_fusion.architecture.md`
- `ws/README.relative_position_fusion.interfaces.md`
- `ws/README.relative_position_fusion.implementation.md`
- `ws/README.relative_position_fusion.verification.md`

---

## 1. 总体执行原则

后续实现必须遵守以下原则：

1. **先建新包，再写核心，再接场景**；
2. **先做最小可验证闭环，再做场景适配**；
3. **先复用标准消息，不新增自定义接口**；
4. **先写单测，再扩外围节点**；
5. **不要在前几个 patch 改动现有 UAV / UGV 关键节点逻辑**。

---

## 2. 明确禁止事项

后续 Codex agent 在实现阶段禁止：

- 把新逻辑塞进 `uav_bridge` 或 `ugv_*` 现有包内部；
- 为了省事直接在 `uav_bridge/tf_bridge_node.cpp` 里插入融合逻辑；
- 在 v1 创建新的 `msg/`、`srv/`、`action/` 包；
- 在 v1 直接实现重定位执行器；
- 修改 `enable_dynamic_global_alignment` 使其产生新行为；
- 跳过单测直接接 launch。

---

## 3. 推荐命令基线

后续实现时，工作目录固定从：

```bash
cd /home/th1rt3en/DOCO/ws
```

常用命令：

```bash
colcon build --packages-select relative_position_fusion --symlink-install
colcon test --packages-select relative_position_fusion
colcon test-result --verbose
source install/setup.bash
ros2 launch relative_position_fusion relative_position_fusion.launch.py
```

若接入联合仿真后再验证，可使用：

```bash
ros2 launch duojin01_bringup sim.launch.py
ros2 topic list | grep relative_position
ros2 topic echo /relative_position/diagnostics
```

---

## 4. 分阶段实施任务单

下面的阶段是后续实现的固定顺序。

## 阶段 1：创建新包骨架

### 目标

创建独立 ROS 2 包 `relative_position_fusion`，但暂不接入任何现有 launch。

### 需要创建的文件

- `ws/src/relative_position_fusion/package.xml`
- `ws/src/relative_position_fusion/CMakeLists.txt`
- `ws/src/relative_position_fusion/include/relative_position_fusion/*.hpp`
- `ws/src/relative_position_fusion/src/*.cpp`
- `ws/src/relative_position_fusion/launch/relative_position_fusion.launch.py`
- `ws/src/relative_position_fusion/config/*.yaml`
- `ws/src/relative_position_fusion/test/*.cpp`

### 阶段完成判据

- 能单独 `colcon build --packages-select relative_position_fusion`
- 安装后可找到可执行体与 launch 文件

### 本阶段不要做的事

- 不接入 `duojin01_bringup`
- 不修改 `uav_bridge`
- 不修改 `ugv_bringup`

---

## 阶段 2：实现数学核心

### 目标

先把滤波核心做成纯 C++ 可测试逻辑。

### 必做文件

- `relative_position_filter.hpp/.cpp`
- `relocalization_monitor.hpp/.cpp`
- `covariance_utils.hpp`
- `test_relative_position_filter.cpp`
- `test_relocalization_monitor.cpp`

### 固定要求

- `RelativePositionFilter` 不依赖 ROS topic
- 所有矩阵计算使用 `Eigen`
- 初始化、预测、更新、门控、`rho` 计算全部可直接单测

### 阶段完成判据

- 数学核心单测全部通过
- `rho` 量纲测试通过
- 门控拒绝测试通过

---

## 阶段 3：实现时间缓冲与适配器

### 目标

把 UAV / UGV 的现有输入归一化为统一内部状态。

### 必做文件

- `agent_planar_state.hpp`
- `time_alignment_buffer.hpp/.cpp`
- `uav_state_adapter.hpp/.cpp`
- `ugv_state_adapter.hpp/.cpp`
- `test_time_alignment_buffer.cpp`
- `test_uav_state_adapter.cpp`
- `test_ugv_state_adapter.cpp`

### 固定要求

- `UavStateAdapter` 只消费 `/uav/odom`
- `UgvStateAdapter` 消费 `/amcl_pose`、`/ugv/odometry/filtered`、`/ugv/odom`
- 所有输出必须统一到 `global_frame`
- 所有速度回退逻辑必须在适配器内部完成

### 阶段完成判据

- 适配器单测通过
- `/amcl_pose` 优先级行为正确
- UGV 车体系速度旋转测试通过

---

## 阶段 4：实现主节点与 diagnostics

### 目标

把输入适配、数学核心和标准输出串起来。

### 必做文件

- `relative_position_fusion_node.cpp`
- `diagnostics_publisher.hpp/.cpp`

### 固定要求

- 节点定时驱动，不要做“消息即改状态”
- 发布 5 类标准输出 topic
- diagnostics 必须包含文档规定字段

### 阶段完成判据

- 可独立启动节点
- 向节点喂测试输入时能看到标准输出

---

## 阶段 5：参数文件与独立 launch

### 目标

让同一节点在不同场景下通过参数工作，而不是通过代码分叉工作。

### 必做文件

- `config/relative_position_fusion.common.yaml`
- `config/relative_position_fusion.duojin_sim.yaml`
- `config/relative_position_fusion.sim_navigation.yaml`
- `config/relative_position_fusion.hw.yaml`
- `launch/relative_position_fusion.launch.py`

### 固定要求

- launch 只启动新节点本身
- common + preset 叠加加载
- 不在此阶段改现有 bringup

### 阶段完成判据

- 能独立 `ros2 launch relative_position_fusion relative_position_fusion.launch.py`
- 能切换不同 preset yaml

---

## 阶段 6：联合仿真接入

### 目标

先把新节点接入 `duojin01_bringup` 的联合仿真。

### 允许修改的现有文件

- `ws/src/duojin01_bringup/launch/sim.launch.py`

### 接入方式

- 仅新增 `relative_position_fusion` 节点或 include
- 使用 `relative_position_fusion.duojin_sim.yaml`
- 默认 `assume_ugv_odom_is_global=true`

### 阶段完成判据

- 联合仿真起得来
- `/relative_position/*` 相关输出可见
- diagnostics 正常刷新

---

## 阶段 7：真机 / 半实物接入

### 目标

在不破坏当前仿真链路的前提下，补齐 UAV 真机标准 `/uav/odom` 接口。

### 允许修改的现有文件

- UAV 真机相关 bringup / launch
- 必要时补一个新的 launch，而不是重写既有仿真 launch

### 固定要求

- 复用 `px4_planar_state_reader_node`
- 统一输出 `output_odom_topic=/uav/odom`
- 不直接让 `relative_position_fusion` 去读 PX4 原始 topic

### 阶段完成判据

- 真机 / 半实物场景下同样只靠 `/uav/odom` 接入
- 不需要为真机分叉新的滤波逻辑

---

## 5. 每个阶段建议运行的命令

## 阶段 1 后

```bash
cd /home/th1rt3en/DOCO/ws
colcon build --packages-select relative_position_fusion --symlink-install
```

## 阶段 2 后

```bash
cd /home/th1rt3en/DOCO/ws
colcon test --packages-select relative_position_fusion
colcon test-result --verbose
```

## 阶段 4 后

```bash
cd /home/th1rt3en/DOCO/ws
source install/setup.bash
ros2 run relative_position_fusion relative_position_fusion_node
```

## 阶段 5 后

```bash
cd /home/th1rt3en/DOCO/ws
source install/setup.bash
ros2 launch relative_position_fusion relative_position_fusion.launch.py
```

## 阶段 6 后

```bash
cd /home/th1rt3en/DOCO/ws
source install/setup.bash
ros2 launch duojin01_bringup sim.launch.py
ros2 topic list | grep relative_position
ros2 topic echo /relative_position/diagnostics
```

---

## 6. 预期产物检查单

后续 Codex agent 每完成一个阶段，都应检查以下产物。

## 阶段 1

- 新包存在
- 能 build

## 阶段 2

- 数学核心头源分离
- 单测存在且通过

## 阶段 3

- UAV / UGV 适配器具备明确优先级与 fallback
- TF 归一化逻辑可测

## 阶段 4

- 5 个标准输出 topic 齐全
- diagnostics 字段齐全

## 阶段 5

- 有 common + preset yaml
- 有独立 launch

## 阶段 6

- 联合仿真能直接看到 `/relative_position/estimate/global`
- `relocalize_requested` 可用

## 阶段 7

- 真机与仿真都只对融合模块暴露 `/uav/odom`
- 不存在为真机单独复制一份滤波代码的情况

---

## 7. 若实现中遇到冲突时的决策顺序

后续实现阶段若出现冲突，按以下优先级处理：

1. 当前仓库真实输入事实
2. `interfaces` 文档
3. `implementation` 文档
4. `verification` 文档
5. 本 Codex 执行指南

原因：

- 接口事实最优先；
- 本文档是执行清单，不应推翻接口定义。

---

## 8. 推荐给 Codex 的提交粒度

虽然当前工作流不要求真正 git commit，但后续 patch 粒度应接近以下拆法：

1. 包骨架 patch
2. 数学核心 patch
3. 适配器 patch
4. 主节点 patch
5. 配置与 launch patch
6. 联合仿真接入 patch
7. 真机接入 patch

这样每一步都可回退、可验证、可 review。

---

## 9. Codex 实施摘要

如果后续实现者只看这一页，那就按下面这条最简路线走：

1. 新建 `relative_position_fusion` 包；
2. 先写 `RelativePositionFilter` 与 `RelocalizationMonitor`；
3. 再写 `UavStateAdapter` 与 `UgvStateAdapter`；
4. 再写单节点定时驱动主循环；
5. 最后用 yaml 区分联合仿真、导航仿真、真机预设；
6. 接入联合仿真后，再考虑真机接入。

这条顺序就是后续 Codex agent 的推荐主线。
