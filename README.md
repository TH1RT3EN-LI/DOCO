# DOCO 工作区：相对位置融合设计文档集

本文档集面向后续实现 `relative_position_fusion` 模块的开发者与 Codex agent。
目标不是立即改动现有代码，而是把“要做什么、为什么这样做、接口怎么定、先后顺序如何安排”一次性讲清楚，避免后续实现时再反复做高层决策。

## 1. 为什么现在需要这组文档

当前工作区已经具备 UAV / UGV 分别运行与联合仿真的基本骨架，但还缺少一个**同时服务仿真与真机的相对位置融合模块**。这个模块要把：

- UAV 侧的全局平面位置、平面速度、位置方差；
- UGV 侧的全局平面位置、平面速度、位置方差；
- 两者的时间戳与坐标系差异；

统一整理成一个可递推估计的二维相对位置状态，并在不确定性变大时触发“需要外部重定位”的事件信号。

这个需求在当前仓库里已经具备了实现基础，但仍有几个关键空白：

- **联合仿真已有 `global / ugv_map / uav_map` 三层约定**，但 `enable_dynamic_global_alignment` 目前只是兼容参数，还没有运行时消费者；
- **UAV 仿真已经有 `/uav/odom`**，但 UAV 真机侧还没有统一对外的 `/uav/odom` 归一化出口；
- **UGV 侧有 `/ugv/odom`、`/ugv/odometry/filtered`、`/amcl_pose` 等候选来源**，但不同场景下谁是“全局位置”、谁是“速度来源”尚未被一个模块统一消费；
- 现有控制与视觉降落栈都还没有消费“带协方差的 UGV 相对 UAV 平面位置估计”。

因此，本轮只做一件事：**把未来实现所需的全部关键决策写成一组结构明确、可直接执行的 README 文档**。

## 2. 本文档集解决什么，不解决什么

### 2.1 本文档集要解决的事

- 明确 `relative_position_fusion` 的职责边界；
- 用当前仓库的真实节点、话题、坐标系重写算法；
- 固定 v1 的 ROS 2 接口、参数、降级策略、默认行为；
- 固定后续实现的目录结构、类划分、测试顺序与 patch 顺序；
- 给出仿真与真机共用的验证与调参方法。

### 2.2 本文档集暂时不解决的事

- 不实现新的 ROS 2 包；
- 不接入现有 launch；
- 不修改任何 UAV / UGV 现有节点逻辑；
- 不直接实现“UAV 自动重定位执行器”；
- 不新增自定义 message / service / action 包。

换句话说：**本轮只落文档，不落代码**。

## 3. 当前项目条件速览

下面这些事实已经在仓库中成立，后续文档全部以它们为前提。

### 3.1 联合仿真与坐标骨架

- 联合仿真入口：`ws/src/duojin01_bringup/launch/sim.launch.py`
- 顶层统一声明：
  - `global_frame`，默认 `global`
  - `ugv_map_frame`，默认 `ugv_map`
  - `uav_map_frame`，默认 `uav_map`
- 当前 `global -> ugv_map`、`global -> uav_map` 通过静态 TF 建立；
- `enable_dynamic_global_alignment` 在当前栈里只是 no-op 提示，没有运行时消费者。

### 3.2 UAV 侧现状

- 仿真 UAV 入口：`ws/src/uav_bringup/launch/sitl_uav.launch.py`
- 仿真中已有 `uav_bridge/tf_bridge_node` 发布 `/uav/odom`；
- `uav_bridge/px4_planar_state_reader_node` 已能把 PX4 的 `vehicle_local_position` + `vehicle_odometry` 归一化为平面 `nav_msgs/Odometry`；
- 该节点当前默认输出 `/uav/px4/planar_odom`，尚未被纳入统一 launch，也尚未作为真机标准 `/uav/odom` 出口使用。

### 3.3 UGV 侧现状

- 真机底盘驱动：`ws/src/ugv_base_driver/src/ugv_base_driver.cpp`
- 底盘统一发布 `/ugv/odom` 与 `/ugv/imu`；
- 导航/定位侧在 `ws/src/ugv_bringup/config/ekf.yaml` 中定义了 `/ugv/odometry/filtered`；
- 使用 Nav2 / AMCL 的场景下，`/amcl_pose` 是最明确的 `ugv_map` 下全局位姿来源；
- 纯联合仿真默认并不启动 Nav2，因此当前联合仿真场景需要允许 `/ugv/odom` 作为“仿真回退全局位姿来源”。

## 4. v1 模块边界

后续新模块 `relative_position_fusion` 的 v1 边界固定如下：

### 4.1 v1 必须完成

- 估计二维相对位置 `r = p_g - p_a`；
- 融合速度差预测与位置差测量；
- 传播二维协方差；
- 做马氏距离门控，拒绝跳变测量；
- 计算保守误差上界 `ρ_k = γ * sqrt(λ_max(P_k))`；
- 发布“需要外部重定位”的触发信号；
- 输出足够的诊断信息供仿真和真机调参。

### 4.2 v1 明确不做

- 不直接调用 UAV 的重定位 service / action；
- 不估计相对 yaw；
- 不做视觉检测与 AprilTag 级别的末端对接；
- 不在核心滤波内部做复杂 3D 姿态建模；
- 不把现有 `enable_dynamic_global_alignment` 参数硬接成新的行为。

## 5. 文档导航

### `ws/README.relative_position_fusion.architecture.md`

回答“这个模块在当前仓库里到底是什么、为什么这么设计、算法如何落地”。

重点内容：

- 统一符号与坐标系；
- 最小二维 KF 的完整解释；
- `Q / R / K` 的工程语义；
- 为什么 v1 只做 `xy`、为什么只发布触发信号；
- 模块内部的职责拆分。

### `ws/README.relative_position_fusion.interfaces.md`

回答“模块对外怎么接、各场景订阅/发布什么、参数默认值是什么”。

重点内容：

- 订阅与发布 topic 规范；
- 消息类型、QoS、时间戳规则；
- UAV / UGV 在仿真与真机中的输入矩阵；
- 回退与降级策略；
- 触发与诊断接口。

### `ws/README.relative_position_fusion.implementation.md`

回答“未来这个模块应该如何在仓库里被实现”。

重点内容：

- 新包的目录结构；
- 类与子模块边界；
- 核心伪代码；
- 默认参数与初始化规则；
- 按 patch 切分的实现顺序。

### `ws/README.relative_position_fusion.verification.md`

回答“怎么验证它是对的、怎么调参数、什么算验收通过”。

重点内容：

- 单元测试矩阵；
- 联合集成测试；
- 仿真与真机验收；
- `Q` 与 `R` 的调参方法；
- 故障注入场景。

### `ws/README.relative_position_fusion.codex.md`

回答“后续让 Codex agent 真正开工时，第一步到最后一步怎么执行”。

重点内容：

- 推荐改动顺序；
- 每一步的文件清单；
- 每一步建议运行的命令；
- 阶段完成判据；
- 禁止越界项。

## 6. 建议实现顺序

推荐后续实现严格按照下面顺序推进：

1. **先建新包骨架**：只创建 `relative_position_fusion` 的最小构建结构；
2. **先写数学核心与单测**：把 KF、门控、触发逻辑独立测试跑通；
3. **再写 UAV / UGV 适配器**：把现有仓库的实际输入整理成统一内部状态；
4. **再写 ROS 2 节点**：把核心算法与适配器串起来；
5. **最后接 launch 与场景参数**：先独立 launch，再接联合仿真，再接真机侧。

这个顺序的目的很明确：

- 把最容易被场景差异污染的部分（输入适配、launch）放到后面；
- 先把最稳定、最通用的部分（数学核心、测试）固定下来；
- 减少一上来就同时碰 UAV / UGV / launch 导致的耦合爆炸。

## 7. 关键决策摘要

为了避免后续实现时再重新争论，以下决策在本文档集中视为已固定：

- 新包名固定为 `relative_position_fusion`；
- v1 只估计二维相对位置，不估计相对 yaw；
- v1 只发布“需要重定位”的触发信号，不负责执行重定位；
- 模块内部统一使用逻辑 `global` 坐标；
- UAV 统一消费接口固定为 `/uav/odom`；
- UGV 全局位置优先 `/amcl_pose`，仿真回退允许 `/ugv/odom`；
- `Q_k` 固定为 `q_rate_mode * dt * I2` 的连续时间扩散率模型；
- 门控固定为二维马氏距离 + `χ²(2, 0.95) = 5.99`；
- 误差上界固定为 `ρ_k = γ * sqrt(λ_max(P_k))`，不能写错量纲；
- v1 只使用标准 ROS 2 消息，不新增自定义接口包。

## 8. 阅读顺序建议

如果你是第一次接手这个模块，建议按下面顺序阅读：

1. 先看本文件，了解边界与现状；
2. 再看 `architecture`，确认算法与工程假设；
3. 再看 `interfaces`，确认所有 topic / 参数 / QoS；
4. 再看 `implementation`，按目录与 patch 顺序开始写代码；
5. 最后看 `verification` 与 `codex`，保证实现与验收方式一致。

---

如果后续实现阶段与本文档冲突，优先处理顺序建议如下：

1. 当前仓库真实接口事实；
2. `interfaces` 文档；
3. `implementation` 文档；
4. 其余说明性文字。

这样做的原因是：接口一旦落地，最怕的是“说明文档说一套、实现又猜另一套”。
