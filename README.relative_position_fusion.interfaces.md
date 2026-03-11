# 相对位置融合模块：ROS 2 接口与参数规范

本文档固定 `relative_position_fusion` v1 的 ROS 2 外部接口。
实现阶段应直接以本文档为准，不再临时设计新的 topic 语义。

---

## 1. 模块对外形态

### 1.1 包名与可执行体

- ROS 2 包名：`relative_position_fusion`
- 主节点可执行体：`relative_position_fusion_node`
- 推荐节点名：`relative_position_fuser`

### 1.2 设计目标

该节点在每个 tick 完成以下步骤：

1. 读取最新 UAV / UGV 状态；
2. 统一转换到 `global_frame`；
3. 形成 `u_k` 与 `z_k`；
4. 运行二维相对位置 KF；
5. 发布估计、调试量、诊断与重定位请求。

---

## 2. 统一命名与帧约定

### 2.1 全局帧

- 参数名：`global_frame`
- 默认值：`global`
- 含义：模块内部唯一使用的逻辑共享全局平面坐标系

### 2.2 UAV 机体系输出帧

- 参数名：`uav_body_frame`
- 默认值：`uav_base_link`
- 含义：发布“UGV 在 UAV 机体系下的相对向量”时使用的 frame id

### 2.3 时间戳规则

- 输入优先使用消息自身 `header.stamp`
- 若某输入消息时间戳为零，则退化为接收时间
- 输出统一使用当前滤波 tick 时间 `t_k`

---

## 3. 订阅接口

下面是 v1 固定订阅的话题集合。

### 3.1 UAV 输入

#### `/uav/odom`

- 消息类型：`nav_msgs/msg/Odometry`
- 必需性：**必需**
- 默认 QoS：`SensorDataQoS`
- 用途：
  - UAV 平面位置来源
  - UAV 平面速度来源
  - UAV 平面位置协方差来源

#### 字段解释

- 位置：`msg.pose.pose.position.{x,y}`
- 位置协方差子块：
  - `cov_xx = pose.covariance[0]`
  - `cov_xy = pose.covariance[1]`
  - `cov_yx = pose.covariance[6]`
  - `cov_yy = pose.covariance[7]`
- 速度：`msg.twist.twist.linear.{x,y}`

#### 默认语义

v1 默认认为 UAV 速度是**全局平面速度**，而不是机体系速度。

- 参数：`uav_twist_in_child_frame`
- 默认值：`false`

如果未来某个 UAV 来源明确发布的是 child frame 速度，可将该参数改为 `true`。

---

### 3.2 UGV 全局位姿输入

#### `/amcl_pose`

- 消息类型：`geometry_msgs/msg/PoseWithCovarianceStamped`
- 必需性：**可选但优先级最高**
- 默认 QoS：可靠、depth 10
- 用途：UGV 全局位置主来源

#### 字段解释

- 位置：`msg.pose.pose.position.{x,y}`
- yaw：由 `msg.pose.pose.orientation` 解算
- 协方差子块：
  - `cov_xx = pose.covariance[0]`
  - `cov_xy = pose.covariance[1]`
  - `cov_yx = pose.covariance[6]`
  - `cov_yy = pose.covariance[7]`

#### 优先级说明

当 `/amcl_pose` 新鲜有效时，UGV 的全局位置固定取自它。

---

### 3.3 UGV 速度输入

#### `/ugv/odometry/filtered`

- 消息类型：`nav_msgs/msg/Odometry`
- 必需性：**可选但速度优先级最高**
- 默认 QoS：`SensorDataQoS`
- 用途：UGV 平面速度主来源；必要时也可作为辅助全局姿态来源

#### 默认语义

v1 默认认为 UGV 线速度是 **child frame / 车体系速度**，需要用全局 yaw 旋转到 `global_frame`。

- 参数：`ugv_filtered_twist_in_child_frame`
- 默认值：`true`

---

### 3.4 UGV 回退位姿与速度输入

#### `/ugv/odom`

- 消息类型：`nav_msgs/msg/Odometry`
- 必需性：**必需**
- 默认 QoS：`SensorDataQoS`
- 用途：
  - UGV 速度回退来源
  - UGV 全局位置仿真回退来源

#### 默认语义

- 默认认为其 `twist.linear.{x,y}` 是 child frame / 车体系速度；
- 默认**不**把其 pose 视为全局位姿；
- 只有在 `assume_ugv_odom_is_global=true` 时，才允许把它作为 UGV 全局位置回退来源。

---

### 3.5 TF 依赖

节点必须订阅并查询：

- `/tf`
- `/tf_static`

用来完成以下变换：

- UAV pose 所在 frame -> `global_frame`
- UGV pose 所在 frame -> `global_frame`
- 若需要，将“非 child frame 速度”从源 frame 旋转到 `global_frame`

v1 要求：

- 节点内部必须使用 `tf2_ros::Buffer` + `tf2_ros::TransformListener`
- 所有 pose 在进入滤波前都必须先变换为 `global_frame`

---

## 4. 发布接口

v1 固定发布以下 5 类输出。

### 4.1 全局相对位置估计

#### `/relative_position/estimate/global`

- 消息类型：`geometry_msgs/msg/PoseWithCovarianceStamped`
- QoS：可靠、depth 10
- `header.frame_id`：`global_frame`

#### 编码方式

- `pose.pose.position.x = r̂_x`
- `pose.pose.position.y = r̂_y`
- `pose.pose.position.z = 0`
- `pose.pose.orientation = identity`
- 只使用 `pose.covariance` 中的 `x/y` 子块：
  - `[0] = P_xx`
  - `[1] = P_xy`
  - `[6] = P_yx`
  - `[7] = P_yy`
- 其余位姿协方差元素固定填 0 或大数均可；v1 建议：
  - 非 `x/y` 位置元素填 0
  - yaw 相关元素填 `1e6`，明确表示“不是本模块估计对象”

### 4.2 UAV 机体系下的相对位置估计

#### `/relative_position/estimate/uav_body`

- 消息类型：`geometry_msgs/msg/PoseWithCovarianceStamped`
- QoS：可靠、depth 10
- `header.frame_id`：`uav_body_frame`

#### 编码方式

- 表示 `r̂_body = R(ψ_a)^T * r̂_global`
- 协方差按相似变换：`P_body = R(ψ_a)^T * P_global * R(ψ_a)`

### 4.3 相对速度调试输出

#### `/relative_position/debug/relative_velocity`

- 消息类型：`geometry_msgs/msg/Vector3Stamped`
- QoS：可靠、depth 10
- `header.frame_id`：`global_frame`

#### 编码方式

- `vector.x = u_x`
- `vector.y = u_y`
- `vector.z = 0`

### 4.4 重定位请求信号

#### `/relative_position/relocalize_requested`

- 消息类型：`std_msgs/msg/Bool`
- QoS：可靠、depth 10
- 含义：
  - `true`：当前相对估计不再满足上层安全使用条件，需要执行外部重定位 / 锚定恢复
  - `false`：当前相对估计仍在可接受不确定性内

### 4.5 诊断输出

#### `/relative_position/diagnostics`

- 消息类型：`diagnostic_msgs/msg/DiagnosticArray`
- QoS：可靠、depth 10

#### 诊断字段固定包含

至少输出以下 key-value：

- `initialized`
- `mode`
- `rho_m`
- `lambda_max_m2`
- `mahalanobis_d2`
- `measurement_used`
- `gate_rejected`
- `consecutive_gate_rejects`
- `uav_pose_age_s`
- `uav_velocity_age_s`
- `ugv_pose_age_s`
- `ugv_velocity_age_s`
- `uav_pose_source`
- `ugv_pose_source`
- `ugv_velocity_source`

---

## 5. 参数规范

下面给出 v1 固定参数表。实现时不要随意改名。

## 5.1 基础参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `publish_rate_hz` | `double` | `20.0` | 主循环频率 |
| `global_frame` | `string` | `global` | 模块内部统一逻辑全局系 |
| `uav_body_frame` | `string` | `uav_base_link` | 机体系相对向量输出 frame |
| `tf_lookup_timeout_sec` | `double` | `0.03` | TF 查询超时 |

## 5.2 输入 topic 参数

| 参数名 | 类型 | 默认值 |
|---|---|---|
| `uav_odom_topic` | `string` | `/uav/odom` |
| `ugv_amcl_pose_topic` | `string` | `/amcl_pose` |
| `ugv_filtered_odom_topic` | `string` | `/ugv/odometry/filtered` |
| `ugv_odom_topic` | `string` | `/ugv/odom` |

## 5.3 输出 topic 参数

| 参数名 | 类型 | 默认值 |
|---|---|---|
| `relative_pose_global_topic` | `string` | `/relative_position/estimate/global` |
| `relative_pose_body_topic` | `string` | `/relative_position/estimate/uav_body` |
| `relative_velocity_topic` | `string` | `/relative_position/debug/relative_velocity` |
| `relocalize_requested_topic` | `string` | `/relative_position/relocalize_requested` |
| `diagnostics_topic` | `string` | `/relative_position/diagnostics` |

## 5.4 输入语义参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `uav_twist_in_child_frame` | `bool` | `false` | UAV 速度是否在 child frame |
| `ugv_filtered_twist_in_child_frame` | `bool` | `true` | `/ugv/odometry/filtered` 速度是否在 child frame |
| `ugv_odom_twist_in_child_frame` | `bool` | `true` | `/ugv/odom` 速度是否在 child frame |
| `assume_ugv_odom_is_global` | `bool` | `false` | 是否允许把 `/ugv/odom` pose 作为全局位姿回退来源 |

## 5.5 新鲜度与同步参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `pose_timeout_sec` | `double` | `0.25` | pose 超过该年龄视为 stale |
| `velocity_timeout_sec` | `double` | `0.15` | velocity 超过该年龄视为 stale |
| `measurement_sync_tolerance_sec` | `double` | `0.05` | 允许构造 `z_k` 的最大时间差 |
| `min_pose_diff_velocity_dt_sec` | `double` | `0.03` | 位姿差分估计速度的最小 dt |
| `max_pose_diff_velocity_dt_sec` | `double` | `0.25` | 位姿差分估计速度的最大 dt |
| `buffer_length_sec` | `double` | `2.0` | 各输入缓冲保留时长 |

## 5.6 协方差保护参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `covariance_floor_m2` | `double` | `1e-4` | 协方差对角下限 |
| `covariance_ceiling_m2` | `double` | `25.0` | 协方差对角上限 |
| `covariance_nan_fallback_m2` | `double` | `1.0` | 协方差缺失时的回退方差 |
| `covariance_age_inflation_m2_per_s` | `double` | `0.25` | 协方差按年龄膨胀速率 |

## 5.7 过程噪声参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `q_rate_nominal_m2_per_s` | `double` | `0.004` | 正常模式扩散率 |
| `q_rate_measurement_missing_m2_per_s` | `double` | `0.012` | 无测量时扩散率 |
| `q_rate_gate_rejected_m2_per_s` | `double` | `0.020` | 测量被拒绝时扩散率 |
| `q_rate_source_degraded_m2_per_s` | `double` | `0.050` | 输入降级时扩散率 |

## 5.8 门控参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `use_measurement_gating` | `bool` | `true` | 是否开启门控 |
| `gate_chi2_threshold` | `double` | `5.99` | 二维观测 95% 阈值 |

## 5.9 触发参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `rho_gamma` | `double` | `2.0` | 误差半径系数 |
| `relocalize_enter_threshold_m` | `double` | `1.0` | 请求进入阈值 |
| `relocalize_exit_threshold_m` | `double` | `0.7` | 请求退出阈值 |
| `relocalize_enter_hold_cycles` | `int` | `5` | 连续满足进入阈值多少拍才触发 |
| `relocalize_exit_hold_cycles` | `int` | `10` | 连续满足退出阈值多少拍才解除 |
| `max_consecutive_gate_rejects_before_force_request` | `int` | `10` | 连续拒绝超限时强制触发 |

---

## 6. 各场景输入矩阵

这一节固定不同场景下该模块应该怎么接当前仓库。

## 6.1 当前联合仿真：`duojin01_bringup/launch/sim.launch.py`

### 目标

尽量零侵入复用当前联合仿真，先把模块跑起来。

### 输入矩阵

| 项目 | 话题 | 说明 |
|---|---|---|
| UAV pose/vel/cov | `/uav/odom` | 由 `tf_bridge_node` 提供 |
| UGV pose | `/ugv/odom` | 作为仿真回退全局位姿 |
| UGV velocity | `/ugv/odom` | 车体系速度，需旋转 |

### 关键参数

- `assume_ugv_odom_is_global=true`
- `uav_twist_in_child_frame=false`
- `ugv_odom_twist_in_child_frame=true`

### 说明

该场景默认没有 `/amcl_pose` 与 `/ugv/odometry/filtered`，因此属于“仿真最低可运行集”。

---

## 6.2 UGV 仿真导航场景：`ugv_bringup/launch/sim_navigation.launch.py`

### 输入矩阵

| 项目 | 话题 | 说明 |
|---|---|---|
| UAV pose/vel/cov | `/uav/odom` | 仿真 UAV |
| UGV pose | `/amcl_pose` | AMCL 全局位姿 |
| UGV velocity | `/ugv/odometry/filtered` | robot_localization 输出 |
| UGV velocity fallback | `/ugv/odom` | 回退 |

### 关键参数

- `assume_ugv_odom_is_global=false`
- `ugv_filtered_twist_in_child_frame=true`

---

## 6.3 真机 / 半实物：UAV 真机 + UGV 导航

### 输入矩阵

| 项目 | 话题 | 说明 |
|---|---|---|
| UAV pose/vel/cov | `/uav/odom` | 由 `px4_planar_state_reader_node` 归一化输出 |
| UGV pose | `/amcl_pose` | 全局位姿 |
| UGV velocity | `/ugv/odometry/filtered` | 全局定位链配套速度来源 |
| UGV velocity fallback | `/ugv/odom` | 回退 |

### 关键要求

- UAV 真机 launch 中必须实例化 `px4_planar_state_reader_node`
- 并明确配置：
  - `output_odom_topic=/uav/odom`
  - `world_frame_id=uav_map`
  - `base_frame_id=uav_base_link`

---

## 6.4 不推荐场景

下面的输入组合不作为 v1 的标准支持目标：

- 没有 `/uav/odom`，直接让模块读 PX4 原始 topic；
- UGV 没有任何全局位姿来源，却仍希望拿到“可信全局相对位置”；
- 把 `/ugv/odom` 直接在真机场景当作长期全局位姿使用。

这些场景不是完全不能跑，而是会显著削弱接口一致性和误差解释的一致性。

---

## 7. 输入降级与回退规则

这一节固定 v1 的“谁缺失了怎么办”。

## 7.1 UAV 侧

### UAV 位置

- 仅来自 `/uav/odom` pose
- 若 pose 无效、TF 失败或 stale，则 `uav_pose_valid=false`

### UAV 速度

优先级：

1. `/uav/odom` 中的 `twist.linear.{x,y}`
2. UAV pose 历史差分
3. 否则速度无效

### UAV 协方差

优先级：

1. `/uav/odom.pose.covariance` 的 `x/y` 子块
2. 若提取值为 NaN / 非法，则回退 `covariance_nan_fallback_m2 * I`
3. 然后统一施加 floor / ceiling / age inflation

## 7.2 UGV 侧

### UGV 全局位置

优先级：

1. `/amcl_pose`
2. `/ugv/odom`（仅当 `assume_ugv_odom_is_global=true`）
3. 否则无全局位置

### UGV 速度

优先级：

1. `/ugv/odometry/filtered`
2. `/ugv/odom`
3. UGV 全局位姿差分
4. 否则无速度

### UGV 协方差

全局位置协方差优先使用与全局位置相同来源携带的协方差：

- 若来源是 `/amcl_pose`，取其 pose covariance 子块；
- 若来源是 `/ugv/odom`，取其 pose covariance 子块；
- 若提取失败，则回退 `covariance_nan_fallback_m2 * I`。

## 7.3 预测输入 `u_k`

只有当 UAV 与 UGV 两边速度都有效时，才构造：

`u_k = v_g - v_a`

否则：

- `u_k = 0`
- `mode_k = source_degraded`

这样做的原因是：

- 相对速度只要一边无效，就不能假定差值仍然可靠；
- 此时继续做零输入预测但放大 `Q_k`，是最保守的工程处理。

## 7.4 测量 `z_k`

只有当下列条件同时满足时，才允许构造位置差测量：

- UAV pose 有效且新鲜；
- UGV pose 有效且新鲜；
- 两者时间戳差 `<= measurement_sync_tolerance_sec`。

否则本拍不做更新，只做预测。

---

## 8. QoS 约定

### 8.1 输入 QoS

- 所有 `nav_msgs/Odometry` 输入：`rclcpp::SensorDataQoS()`
- `/amcl_pose`：可靠、depth 10

### 8.2 输出 QoS

v1 所有输出统一使用：

- reliable
- keep last 10

原因：

- 输出给上层控制与诊断消费，不属于高频原始传感器流；
- 更关心稳定送达与可回放，而不是纯最低延迟。

---

## 9. 机体系相对位置的定义

发布 `/relative_position/estimate/uav_body` 时，固定使用 UAV 当前全局 yaw：

`r_body = R(ψ_a)^T * r_global`

其中：

- `ψ_a` 来自 UAV 全局 pose 的 yaw
- `R(ψ_a)` 是二维平面旋转矩阵

对应协方差：

`P_body = R(ψ_a)^T * P_global * R(ψ_a)`

这个输出是为了让后续 UAV 跟随 / 会合控制器可以直接在 UAV 机体系下消费相对向量。

---

## 10. 未初始化与失效时的对外行为

接口层必须把“无估计”与“有估计但不可靠”区分开。

### 10.1 未初始化

在首次成功构造有效 `z_k` 并完成初始化前：

- 不发布有效相对位置估计；
- `relocalize_requested=true`；
- diagnostics 中 `initialized=false`。

### 10.2 输入全面失效

若长时间无法得到有效 pose / velocity / tf：

- 继续允许 diagnostics 发布；
- 相对位置估计可停止更新或保留最后值，但必须在 diagnostics 中声明 stale；
- `relocalize_requested=true`。

---

## 11. 本接口文档的落地边界

本文档固定的是：

- 未来新包的外部 ROS 2 接口；
- 对当前仓库已有话题的使用方式；
- 仿真与真机场景下的参数预设方向。

本文档**不**要求当前仓库立刻提供全部输入。
它只要求未来实现 `relative_position_fusion` 时，不再重新发明接口。
