# 相对位置融合模块：实现设计与目录规范

本文档描述后续真正实现 `relative_position_fusion` 时，包结构、类划分、算法主循环、默认参数与 patch 顺序应该如何组织。

目标是让后续实现者**不再需要做高层设计决策**，只需按本文档逐步落代码。

---

## 1. 新包位置与总体结构

### 1.1 新包位置

后续新包固定放在：

`ws/src/relative_position_fusion`

不放到 `uav_bridge` 或任何 `ugv_*` 包内部。

### 1.2 为什么必须独立成包

- 该模块同时依赖 UAV 与 UGV 的接口，不应从包结构上偏向任一侧；
- 它会同时服务仿真与真机，不应被嵌入某个场景专用包；
- 它需要独立单元测试、独立参数文件、独立 launch；
- 后续如果要复用到其他平台，独立包迁移成本最低。

---

## 2. 目录结构固定方案

后续实现时，目录结构固定如下：

```text
ws/src/relative_position_fusion/
├── CMakeLists.txt
├── package.xml
├── include/relative_position_fusion/
│   ├── agent_planar_state.hpp
│   ├── covariance_utils.hpp
│   ├── diagnostics_publisher.hpp
│   ├── relocalization_monitor.hpp
│   ├── relative_position_filter.hpp
│   ├── time_alignment_buffer.hpp
│   ├── uav_state_adapter.hpp
│   └── ugv_state_adapter.hpp
├── src/
│   ├── diagnostics_publisher.cpp
│   ├── relative_position_filter.cpp
│   ├── relative_position_fusion_node.cpp
│   ├── relocalization_monitor.cpp
│   ├── time_alignment_buffer.cpp
│   ├── uav_state_adapter.cpp
│   └── ugv_state_adapter.cpp
├── config/
│   ├── relative_position_fusion.common.yaml
│   ├── relative_position_fusion.duojin_sim.yaml
│   └── relative_tracking.common.yaml
├── launch/
│   └── relative_position_fusion.launch.py
└── test/
    ├── test_relative_position_filter.cpp
    ├── test_relocalization_monitor.cpp
    ├── test_time_alignment_buffer.cpp
    ├── test_uav_state_adapter.cpp
    └── test_ugv_state_adapter.cpp
```

v1 不新增 `msg/`、`srv/`、`action/` 目录。

---

## 3. 构建依赖固定方案

### 3.1 `package.xml` 依赖

后续 `package.xml` 固定包含：

- `ament_cmake`
- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `diagnostic_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `Eigen3`

### 3.2 `CMakeLists.txt` 依赖

固定采用：

- 一个核心库 target：`relative_position_fusion_core`
- 一个节点 executable：`relative_position_fusion_node`
- 若启用测试，使用 `ament_cmake_gtest`

不需要引入自定义接口生成。

---

## 4. 类与子模块职责固定

这一节是后续代码结构的核心。

## 4.1 `AgentPlanarState`

用途：统一表达“单个 agent 在全局平面上的状态”。

建议结构：

```cpp
struct AgentPlanarState {
  rclcpp::Time stamp;
  Eigen::Vector2d p_global;
  Eigen::Vector2d v_global;
  Eigen::Matrix2d Sigma_p;

  bool pose_valid;
  bool velocity_valid;
  bool covariance_valid;

  bool pose_from_fallback;
  bool velocity_from_fallback;

  double pose_age_sec;
  double velocity_age_sec;

  std::string pose_source;
  std::string velocity_source;
};
```

固定要求：

- `p_global`、`v_global` 一律已经在 `global_frame` 中；
- `Sigma_p` 一律为 `2×2` 平面协方差；
- 任何无效量都不能悄悄塞 NaN 后仍标 valid；
- 适配器必须显式写明该状态来自哪条 topic。

## 4.2 `TimeAlignmentBuffer`

用途：缓存异步输入，并为适配器 / 节点提供时间对齐能力。

固定职责：

- 保留每个输入流最近 `buffer_length_sec` 内的数据；
- 支持取“最新新鲜样本”；
- 支持取“最接近某时刻的一对样本”；
- 支持按两帧 pose 差分估计速度。

固定不负责：

- 不做滤波；
- 不做 TF 变换；
- 不做测量门控。

## 4.3 `UavStateAdapter`

用途：把 UAV 输入统一成 `AgentPlanarState`。

固定输入：

- `/uav/odom`

固定职责：

- 提取 pose / twist / covariance；
- 把 pose 变换到 `global_frame`；
- 根据参数决定 twist 在 child frame 还是 pose frame；
- 协方差缺失时做保护回退；
- 速度缺失时用 pose 历史差分回退。

## 4.4 `UgvStateAdapter`

用途：把 UGV 的多来源输入统一成 `AgentPlanarState`。

固定输入：

- `/amcl_pose`
- `/ugv/odometry/filtered`
- `/ugv/odom`

固定职责：

- 根据优先级选择全局 pose 来源；
- 根据优先级选择 velocity 来源；
- 将 body / child frame 速度旋转到 `global_frame`；
- 协方差提取、回退、老化膨胀；
- 明确记录当前用了哪条来源、是否进入 fallback。

## 4.5 `RelativePositionFilter`

用途：实现纯数学核心，不感知 ROS topic。

固定职责：

- 初始化状态
- `predict(dt, u, q_rate)`
- `canUpdate(z, R)`
- `gate(z, R, threshold)`
- `update(z, R)`
- 提供当前 `r̂`、`P`、`rho`

固定不负责：

- 不做 topic 订阅；
- 不做 TF；
- 不做日志；
- 不做状态源选择。

## 4.6 `RelocalizationMonitor`

用途：根据不确定性与数据质量输出“是否需要请求外部重定位”。

固定职责：

- 根据 `rho` 做迟滞判断；
- 跟踪连续门控拒绝次数；
- 处理未初始化、长期失效等硬故障；
- 输出布尔触发态与文本模式。

## 4.7 `DiagnosticsPublisher`

用途：集中生成 `diagnostic_msgs::msg::DiagnosticArray`。

固定职责：

- 把滤波状态、门控状态、输入 freshness 状态整理成标准 diagnostics；
- 保持 diagnostics 字段稳定，避免后续消费者反复适配。

---

## 5. 过滤器核心行为固定

这一节把实现细节写死，避免后续开发时再重新决策。

## 5.1 初始化规则

滤波器只有在首次获得**有效测量对**时才初始化。

初始化条件：

- UAV pose 有效；
- UGV pose 有效；
- 时间差满足 `measurement_sync_tolerance_sec`；
- `R_k` 正定或至少经保护后可逆。

初始化行为：

- `r̂_0 = z_0`
- `P_0 = R_0`

初始化前：

- 不输出有效估计；
- `relocalize_requested=true`；
- diagnostics 标记 `initialized=false`。

## 5.2 预测规则

每个 tick，先构造 `u_k`。

若 UAV 与 UGV 两侧速度都有效：

- `u_k = v_g - v_a`

否则：

- `u_k = [0, 0]^T`
- `mode = source_degraded`

然后执行：

- `r̂ = r̂ + dt * u_k`
- `P = P + q_rate(mode) * dt * I`

## 5.3 更新规则

若可形成有效测量对，则构造：

- `z_k = p_g - p_a`
- `R_k = Σ_g + Σ_a`

其中 `Σ_g`、`Σ_a` 在进入该式之前必须先完成：

- NaN 修正
- floor / ceiling
- 年龄膨胀

## 5.4 门控规则

先算：

- `y_k = z_k - r̂`
- `S_k = P + R_k`
- `d_k² = y_k^T * S_k^{-1} * y_k`

若 `use_measurement_gating=true` 且 `d_k² > gate_chi2_threshold`：

- 拒绝本次更新；
- `gate_rejected=true`；
- `consecutive_gate_rejects += 1`；
- `mode = gate_rejected`。

否则：

- 执行正常更新；
- `consecutive_gate_rejects = 0`。

## 5.5 相对误差上界计算

每次更新或仅预测后，都要计算：

- `lambda_max = max_eigenvalue(P)`
- `rho = rho_gamma * sqrt(lambda_max)`

注意：

- `lambda_max` 单位是 `m²`
- `rho` 单位是 `m`

---

## 6. UGV / UAV 适配器的具体规则

## 6.1 UAV 适配器具体规则

### 输入选择

固定只消费 `/uav/odom`。

### pose 处理

1. 若 `msg.header.frame_id == global_frame`，直接提取 `x/y`；
2. 否则通过 tf2 把 pose 变换到 `global_frame`；
3. 若 tf 查找失败，则该帧 pose 无效。

### velocity 处理

若 `uav_twist_in_child_frame=true`：

- 使用全局 yaw 把 `(vx, vy)` 从 child frame 旋转到 `global_frame`。

若 `uav_twist_in_child_frame=false`：

- 把 twist 所在 pose frame 方向旋转到 `global_frame`；
- 在当前仓库默认场景下，这等价于视为世界系平面速度。

若 twist 不可用：

- 使用最近两帧有效 pose 做差分估计；
- 若差分窗口不满足设定 dt 范围，则 velocity 无效。

### covariance 处理

从 odom pose covariance 提取 `x/y` 子块。

若提取失败：

- 使用 `covariance_nan_fallback_m2 * I`

之后统一执行：

- 对角项下限
- 对角项上限
- 年龄膨胀

## 6.2 UGV 适配器具体规则

### 全局 pose 选择

优先级固定：

1. `/amcl_pose`
2. `/ugv/odom`（仅当 `assume_ugv_odom_is_global=true`）

### velocity 选择

优先级固定：

1. `/ugv/odometry/filtered`
2. `/ugv/odom`
3. 全局 pose 差分

### 速度旋转

当所选 velocity 来源 `*_twist_in_child_frame=true` 时：

- 使用与该来源时刻最接近的全局 pose yaw；
- 将 `(vx, vy)` 从 child frame 旋转到 `global_frame`。

若缺少对应全局 yaw：

- 该 velocity 来源视为无效，继续尝试下一级回退。

---

## 7. 主节点的执行顺序固定

主节点每一拍严格按下面顺序执行：

1. 读取当前 ROS 时间 `t_k`
2. 从 UAV 适配器取 `AgentPlanarState`
3. 从 UGV 适配器取 `AgentPlanarState`
4. 若滤波器未初始化：尝试用有效 `z_k` 初始化
5. 若已初始化：
   - 构造 `u_k`
   - 选择 `q_mode`
   - 执行预测
6. 若本拍存在有效 `z_k`：
   - 构造 `R_k`
   - 执行门控
   - 若通过，则执行更新
7. 计算 `rho`
8. 更新 `RelocalizationMonitor`
9. 发布：
   - 全局相对位置
   - UAV 机体系相对位置
   - 相对速度调试量
   - 重定位请求
   - diagnostics

节点**不采用“消息一来就立刻改滤波状态”**的事件式更新；
所有滤波更新都由定时器驱动。

---

## 8. 伪代码固定版本

后续实现可直接照下述伪代码展开：

```text
on_timer_tick(now):
  uav = uav_adapter.build_state(now)
  ugv = ugv_adapter.build_state(now)

  measurement = try_build_measurement(uav, ugv)
  control = try_build_relative_velocity(uav, ugv)

  if not filter.initialized():
    if measurement.valid:
      filter.initialize(measurement.z, measurement.R)
    else:
      publish_uninitialized_outputs(now, uav, ugv)
      return

  mode = select_q_mode(uav, ugv, measurement)
  u = control.valid ? control.u : [0, 0]
  filter.predict(dt, u, q_rate(mode))

  if measurement.valid:
    gate = filter.compute_gate(measurement.z, measurement.R)
    if gate.accepted:
      filter.update(measurement.z, measurement.R)
      consecutive_gate_rejects = 0
    else:
      consecutive_gate_rejects += 1

  rho = compute_rho(filter.P, rho_gamma)
  relocalize = monitor.update(filter.initialized, rho, measurement, consecutive_gate_rejects)

  publish_global_estimate(filter.state, filter.P)
  publish_uav_body_estimate(filter.state, filter.P, uav_yaw)
  publish_relative_velocity(u)
  publish_relocalize_requested(relocalize)
  publish_diagnostics(...)
```

---

## 9. 默认参数的物理意义与由来

### 9.1 `q_rate_nominal_m2_per_s = 0.004`

这个默认值对应一个很具体的工程含义：

- 若没有可靠重定位更新；
- 且相对误差上界希望从约 `0.3 m` 增长到 `1.0 m` 大致用 `60 s`；

则在 `γ = 2` 条件下：

`q_rate ≈ (((1.0 / 2)^2 - (0.3 / 2)^2) / 60) ≈ 0.00379 m²/s`

因此默认值取 `0.004`。

### 9.2 其他 `q_rate`

- `measurement_missing`: 正常值的约 3 倍
- `gate_rejected`: 正常值的约 5 倍
- `source_degraded`: 正常值的约 12 倍

这样配置的目的不是追求绝对精确，而是保证：

- 数据越差，不确定性越快膨胀；
- 触发器越早意识到“当前估计已经不能安全使用”。

---

## 10. 配置文件固定方案

当前实现里，参数分层固定为：代码默认值 + 共享差异 + 场景差异 + 入口 overlay。

### `relative_position_fusion.common.yaml`

只放所有场景都共享、且确实偏离代码默认值的 fusion 参数。

当前仅保留：

- `uav_twist_in_child_frame=true`

### `relative_position_fusion.duojin_sim.yaml`

仅覆盖联合仿真的差异：

- `assume_ugv_odom_is_global=true`

### `relative_tracking.common.yaml`

只放所有场景都共享、且确实偏离代码默认值的 tracking 参数。

- `return_height_m=2.0`
- `enable_heading_aligned_tracking=true`

### `duojin01_bringup/config/relative_position_fusion.sim_navigation.yaml`

只放 `sim_navigation` 入口独有的 fusion overlay：

- `pose_timeout_sec=2.0`
- `bootstrap_measurement_sync_tolerance_sec=0.15`
- `startup_relocalize_grace_sec=2.0`

---

## 11. 未来 launch 接入顺序

这一部分只是定义后续 patch 顺序，本轮不改 launch。

### 第一步：独立 launch

先创建：

`ws/src/relative_position_fusion/launch/relative_position_fusion.launch.py`

只启动新节点本身，参数来自 common + preset yaml。

### 第二步：联合仿真接入

后续将其接入：

`ws/src/duojin01_bringup/launch/sim.launch.py`

要求：

- 默认使用 `relative_position_fusion.duojin_sim.yaml`
- 不改现有 UAV / UGV 节点行为

### 第三步：真机接入

后续再把其接到 UAV / UGV 真机场景，并补：

- `px4_planar_state_reader_node -> /uav/odom`

---

## 12. patch 级实施顺序

后续实现应严格拆成以下 patch：

### Patch 1：包骨架

创建：

- `package.xml`
- `CMakeLists.txt`
- 头文件与源文件空壳
- 独立 launch 与 yaml 空壳

完成标准：

- `colcon build --packages-select relative_position_fusion` 通过

### Patch 2：数学核心

实现：

- `RelativePositionFilter`
- `RelocalizationMonitor`
- 相关单测

完成标准：

- 核心单测通过

### Patch 3：时间缓冲与适配器

实现：

- `TimeAlignmentBuffer`
- `UavStateAdapter`
- `UgvStateAdapter`
- 适配器单测

### Patch 4：主节点与 diagnostics

实现：

- `relative_position_fusion_node`
- `DiagnosticsPublisher`
- 5 个标准输出 topic

### Patch 5：配置与独立 launch

实现：

- `common + preset yaml`
- `relative_position_fusion.launch.py`

### Patch 6：场景接入

先接联合仿真，再接真机场景。

---

## 13. 实现阶段明确禁止的事

为了控制风险，后续实现阶段禁止在前几个 patch 中做以下事情：

- 不要在 Patch 1–3 修改 `uav_bridge` 既有节点逻辑；
- 不要在 Patch 1–3 修改 `ugv_base_driver` 既有接口；
- 不要在 v1 新增自定义 msg；
- 不要在 v1 直接接管 UAV 重定位行为；
- 不要把 `enable_dynamic_global_alignment` 直接变成新模块控制开关；
- 不要为了省事把所有逻辑塞进单个 node 文件。

---

## 14. 实现设计小结

如果后续实现者只能记住一件事，那就是：

> 先把“输入归一化”和“数学核心”分开，
> 再通过一个定时驱动的单节点把它们串起来，
> 最后用配置而不是代码分叉去适配仿真与真机。

这就是本模块能否稳定落地的关键。
