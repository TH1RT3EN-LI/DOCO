# `uav_bridge` 与 PX4 连接说明

> 注意：当前主执行链已切换到 `uav_state_bridge_node` 与 `uav_control_node`。本文其余内容若提到 `offboard_bridge_node` 或旧 planner 链路，均视为历史说明。

> 注意：本文大部分章节描述的是旧 `offboard_bridge_node` / `/uav/odom` 链路。
> 当前工作区主状态与控制链已经切换到 `uav_state_bridge_node`、`uav_control_node`、`/uav/state/odometry`。

本文档说明当前工程里 `uav_bridge` 包，尤其是 `offboard_bridge_node`，在整套 UAV 仿真/控制链路中的作用。

重点回答 4 个问题：

1. `offboard_bridge_node` 是干什么的
2. 当前工程是怎么和 PX4 连起来的
3. 它在运行时具体做了什么转换和控制
4. 仿真和真机共用时，哪些接口必须保持一致

## 1. 总体架构

当前工程不是通过 `MAVROS` 或 `cmd_vel` 直接控制 PX4，而是走 PX4 的 ROS 2 `uXRCE-DDS` 话题接口。

运行链路如下：

```text
Gazebo / 仿真世界
  -> tf_bridge_node
  -> /uav/sim/ground_truth/odom

PX4 / uXRCE-DDS
  -> px4_odom_adapter_node
  -> /uav/odom

PX4 / uXRCE-DDS
  -> px4_planar_state_reader_node
  -> /uav/px4/planar_odom

上层规划器 / 手动控制
  -> /uav/planning/position_cmd
  或 /uav/control/pose
  或 /uav/control/nudge
  或 /uav/control/* services

offboard_bridge_node
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
  -> /fmu/in/vehicle_command

MicroXRCEAgent
  <-> PX4 uXRCE-DDS client

PX4
  -> /fmu/out/vehicle_local_position
  -> /fmu/out/vehicle_status
```

也就是说，`offboard_bridge_node` 的角色是：

- 接收上层的 ROS 2 控制命令
- 转成 PX4 能理解的 `px4_msgs`
- 帮你持续维持 `OFFBOARD` 控制所需的心跳和 setpoint
- 在需要时帮你自动切 `OFFBOARD`、解锁、降落

## 2. 启动时，PX4 是怎么接进来的

### 2.1 仿真里 PX4 是怎么启动的

`sitl_uav.launch.py` 会调用：

- `run_px4_gz_uav.sh`

这个脚本会设置：

- `PX4_SYS_AUTOSTART`
- `PX4_SIM_MODEL`
- `PX4_GZ_WORLD`
- 自定义机体参数 profile，例如 `sim_uav` / `hw_uav`

然后执行：

```bash
make -C "$PX4_DIR" px4_sitl none
```

也就是直接启动 PX4 SITL。

### 2.2 ROS 2 和 PX4 是怎么打通的

在启用 `use_offboard_bridge:=true` 时，`sitl_uav.launch.py` 里还会启动：

```bash
MicroXRCEAgent udp4 -p 8888
```

它的作用是：

- 作为 PX4 `uXRCE-DDS client` 的代理
- 把 PX4 内部的 `uORB` 主题桥接成 ROS 2 侧的 `/fmu/in/*` 和 `/fmu/out/*`

因此当前工程里，`offboard_bridge_node` 并不是直接和 PX4 进程通信，而是：

- 对 ROS 2 发布 `px4_msgs`
- 这些消息经 `MicroXRCEAgent` 进入 PX4

### 2.3 仿真中谁提供 `/uav/odom`

现在的 SITL 里，`/uav/odom` 由 `px4_odom_adapter_node` 根据 PX4 的 `vehicle_local_position` + `vehicle_odometry` 发布。

Gazebo 真值仍然保留，但只通过独立的旁路接口输出到：

- `/uav/sim/ground_truth/odom`

因此当前仿真已经不再把 Gazebo 真值直接当作主控制状态。

这点很重要，因为：

- 上层规划器和手动控制都用 `/uav/odom`
- `offboard_bridge_node` 也用 `/uav/odom` 和 PX4 的本地位置做坐标对齐

所以当前主链路是：

```text
PX4 estimated state -> px4_odom_adapter_node -> /uav/odom
PX4 本地状态 -> /fmu/out/vehicle_local_position
offboard_bridge_node 同时订阅两者并做对齐
```

同时保留一条调试/评估旁路：

```text
Gazebo 位姿 -> tf_bridge_node -> /uav/sim/ground_truth/odom
```

## 3. `offboard_bridge_node` 的职责

这个节点做 5 件事。

### 3.1 接收上层控制源

它当前支持三种控制源：

- `Planner`
  - 话题：`/uav/planning/position_cmd`
  - 消息：`quadrotor_msgs/msg/PositionCommand`
- `Manual`
  - 话题：`/uav/control/pose`
  - 话题：`/uav/control/nudge`
  - 服务：`/uav/control/takeoff`
  - 服务：`/uav/control/hover`
- `Landing`
  - 服务：`/uav/control/land`

并且支持：

- `/uav/control/resume_auto`

用于从手动控制切回规划器控制。

### 3.2 持续发布 PX4 Offboard 心跳

PX4 进入 `OFFBOARD` 模式的前提之一，是外部控制端必须持续发送 `OffboardControlMode` 和 setpoint。

这个节点会按参数 `publish_rate_hz` 周期发送：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`

默认频率是：

- `50 Hz`

### 3.3 自动切换 `OFFBOARD + ARM`

节点内部有一个 `warmup_cycles` 参数，默认：

- `20`

也就是在开始连续发送 setpoint 一段时间后，节点会自动发送：

- `VEHICLE_CMD_DO_SET_MODE`
- `VEHICLE_CMD_COMPONENT_ARM_DISARM`

这样做的目的是满足 PX4 对 Offboard 进入条件的要求，避免一上来直接切模式失败。

### 3.4 做坐标系转换

上层控制和规划器使用的是：

- ENU
- `map/world` 风格坐标

PX4 `TrajectorySetpoint` 使用的是：

- NED

所以节点会做这些转换：

- 位置：`ENU -> NED`
- 速度：`ENU -> NED`
- 加速度：`ENU -> NED`
- yaw：`ENU yaw -> NED yaw`

转换逻辑大致是：

```text
ENU:  x=East, y=North, z=Up
NED:  x=North, y=East,  z=Down
```

代码里对应的转换是：

```cpp
position_ned = {y_enu, x_enu, -z_enu}
```

### 3.5 做规划系和 PX4 本地系的对齐

这一步是这个节点最关键的工程逻辑之一。

节点同时订阅：

- `/uav/odom`，规划/仿真世界里的 ENU 位姿
- `/fmu/out/vehicle_local_position`，PX4 的本地 NED 位姿

它会计算一个 `frame_offset_ned_`：

```text
offset = px4_local_ned - planner_odom_converted_to_ned
```

之后发布给 PX4 的 `TrajectorySetpoint.position` 都会叠加这个 offset。

这样做的目的：

- 让上层继续用自己的 `map` / `odom` 坐标系
- 同时让 PX4 在自己的 local NED 坐标系里收到正确目标
- 避免仿真初始点、PX4 local origin、规划世界原点不一致时直接飞偏

如果 PX4 的 local position 发生 reset，节点还会重新锁定对齐偏移。

## 4. 节点对 PX4 发布了什么

### 4.1 `/fmu/in/offboard_control_mode`

告诉 PX4：

- 本节点要控制位置
- 也提供速度、加速度前馈

当前字段设置是：

- `position = true`
- `velocity = true`
- `acceleration = true`
- 其它控制位为 `false`

### 4.2 `/fmu/in/trajectory_setpoint`

这是实际给 PX4 的目标轨迹点，包含：

- `position`
- `velocity`
- `acceleration`
- `yaw`
- `yawspeed`

如果当前源是规划器，就来自 `PositionCommand`。

如果当前源是手动控制，就来自节点内部构造的“目标点保持命令”。

### 4.3 `/fmu/in/vehicle_command`

这个节点会发三类典型命令：

- `VEHICLE_CMD_DO_SET_MODE`
  - 切到 `OFFBOARD`
- `VEHICLE_CMD_COMPONENT_ARM_DISARM`
  - 解锁
- `VEHICLE_CMD_NAV_LAND`
  - 进入 PX4 自动降落

## 5. 手动控制功能是怎么实现的

这是本节点现在新加的能力。

### 5.1 `takeoff`

调用：

```bash
ros2 service call /uav/control/takeoff std_srvs/srv/Trigger "{}"
```

逻辑：

- 读取当前 `/uav/odom`
- 保持当前 `x/y/yaw`
- 目标 `z += takeoff_height_m`

注意：

- 这是“相对当前高度抬升”
- 不是固定飞到绝对高度

### 5.2 `hover`

调用：

```bash
ros2 service call /uav/control/hover std_srvs/srv/Trigger "{}"
```

逻辑：

- 锁存当前 `/uav/odom`
- 后续持续向 PX4 发送同一个位置 setpoint

### 5.3 `pose`

调用示例：

```bash
ros2 topic pub --once /uav/control/pose geometry_msgs/msg/PoseStamped \
'{pose: {position: {x: 1.0, y: 0.0, z: 1.2}, orientation: {w: 1.0}}}'
```

逻辑：

- 直接切到手动目标控制
- 将这个绝对 ENU 位姿当作保持目标

### 5.4 `nudge`

调用示例：

```bash
ros2 topic pub --once /uav/control/nudge geometry_msgs/msg/Twist \
'{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
```

默认行为：

- `linear.x`：机体前方增量
- `linear.y`：机体左方增量
- `linear.z`：向上增量
- `angular.z`：偏航增量

如果参数 `nudge_body_frame=false`，则会按世界系直接加到目标点。

### 5.5 `land`

调用：

```bash
ros2 service call /uav/control/land std_srvs/srv/Trigger "{}"
```

逻辑：

- 发送 `VEHICLE_CMD_NAV_LAND`
- 切换到 `Landing` 控制源
- 在 PX4 还没真正进入 `AUTO_LAND` 之前，继续发送当前位置保持 setpoint
- 同时停止那套“不断重新拉起 OFFBOARD + ARM”的逻辑，避免和降落状态机打架

### 5.6 `resume_auto`

调用：

```bash
ros2 service call /uav/control/resume_auto std_srvs/srv/Trigger "{}"
```

逻辑：

- 放弃内部手动目标
- 切回 `/uav/planning/position_cmd`

## 6. 当前工程里，这个节点和上层规划器的关系

当前工程已经移除了原先的规划器包；`offboard_bridge_node` 只保留一个稳定的上层输入接口：

- `/uav/planning/position_cmd`
- 消息类型：`quadrotor_msgs/msg/PositionCommand`

也就是说，任何上层规划器只要发布这条命令话题，就可以接入当前 PX4 Offboard 控制链路。

现在的关系是：

```text
上层规划器
  -> /uav/planning/position_cmd
offboard_bridge_node
  -> /fmu/in/trajectory_setpoint
PX4
  -> 位置控制器
```

所以这个节点不是规划器本身，它是上层规划器/手动指令到 PX4 Offboard 控制接口之间的适配层。

## 7. 仿真和真机共用时，什么必须保持一致

如果以后要真机共用，建议保持下面这层接口不变：

- `/uav/odom`
- `/uav/planning/position_cmd`
- `/uav/control/pose`
- `/uav/control/nudge`
- `/uav/control/*`
- `/fmu/in/*`
- `/fmu/out/*`

其中最关键的是：

### 7.1 `/uav/odom` 的语义必须一致

这个节点假设 `/uav/odom` 是：

- ENU
- 3D 位姿和速度
- 与你上层规划和手动控制使用的世界系一致

当前 SITL 里这个话题由 `px4_odom_adapter_node` 提供。

真机上如果你要复用这套控制逻辑，就需要继续保证这个接口语义成立。

当前仓库里单独保留了一个二维派生接口：

- `/uav/px4/planar_odom`

它只用于平面对比或协同消费，不应替代 `/uav/odom`。

如果你要接其他状态源，也应该用：

- VIO
- SLAM
- mocap
- GPS/融合定位

中的一种，发布同语义的话题给 `/uav/odom`。

### 7.2 PX4 侧仍然要能提供 `/fmu/out/*`

本节点依赖：

- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status`

只要真机上的 PX4 也通过 `uXRCE-DDS` 接入 ROS 2，这部分逻辑就可以直接复用。

## 8. 和 PX4 的连接，不是“直接 socket 通信”，而是“话题桥”

这里再强调一次当前工程的边界：

`offboard_bridge_node` 不直接：

- 调用 PX4 API
- 读写 PX4 内存
- 发 MAVLink socket

它做的是：

1. 订阅 ROS 2 上层命令
2. 发布 ROS 2 `px4_msgs`
3. 由 `MicroXRCEAgent` 把这些消息送进 PX4

所以你可以把它理解成：

- “PX4 Offboard 控制的 ROS 2 适配器”

而不是：

- “飞控本体”

## 9. 典型排查方法

### 9.1 看 PX4 是否已经接上 ROS 2

检查这些话题是否存在：

```bash
ros2 topic list | grep /fmu
```

如果 `/fmu/in/*` 和 `/fmu/out/*` 都没有，优先检查：

- `MicroXRCEAgent` 是否启动
- PX4 是否启动成功
- PX4 的 DDS client 是否正常

### 9.2 看 `offboard_bridge_node` 是否已经在送心跳

```bash
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
```

正常情况下应该接近参数设置的频率，默认约 `50Hz`。

### 9.3 看是否已经切入 `OFFBOARD`

```bash
ros2 topic echo /fmu/out/vehicle_status
```

关注：

- `arming_state`
- `nav_state`

### 9.4 看上层控制是否已经到桥接节点

自动控制：

```bash
ros2 topic echo /uav/planning/position_cmd
```

手动控制：

```bash
ros2 topic echo /uav/control/pose
ros2 topic echo /uav/control/nudge
```

### 9.5 看 `/uav/odom` 是否正常

```bash
ros2 topic echo /uav/odom
```

如果这个话题没有值，手动 `takeoff/hover` 和坐标对齐都会失败。

## 10. 一句话总结

`offboard_bridge_node` 是这套工程里“ROS 2 上层控制”和“PX4 Offboard 控制接口”之间的桥。

它的工作可以概括成：

- 接收规划器或人工控制命令
- 把 ENU 世界系目标转换成 PX4 所需的 NED setpoint
- 持续发布 Offboard 心跳
- 自动处理 `OFFBOARD/ARM`
- 在降落时切换到 PX4 的 `AUTO_LAND`
- 用 `/uav/odom` 和 PX4 local position 做坐标对齐

如果以后要扩展真机，只要继续保持：

- 上层控制接口不变
- `/uav/odom` 语义不变
- PX4 仍通过 `uXRCE-DDS` 暴露 `/fmu/in/*` 和 `/fmu/out/*`

这套节点就可以直接复用。
