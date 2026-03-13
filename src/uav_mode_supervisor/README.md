# uav_mode_supervisor

`uav_mode_supervisor` 对外统一暴露 5 个简洁模式服务，屏蔽 `relative_tracking`、`visual_landing` 和底层 PX4 模式切换的实现细节。

## Public services

### `/uav/mode/track`
- 类型：`std_srvs/srv/Trigger`
- 语义：进入 `relative tracking`
- 说明：使用 supervisor 默认的 `default_tracking_height_m`；如果需要自定义高度，继续调用 `/uav/mode_supervisor/command`

```bash
ros2 service call /uav/mode/track std_srvs/srv/Trigger "{}"
```

### `/uav/mode/land`
- 类型：`std_srvs/srv/Trigger`
- 语义：进入 `visual landing`
- 说明：会按 supervisor 仲裁规则从当前 owner handoff 到视觉降落

```bash
ros2 service call /uav/mode/land std_srvs/srv/Trigger "{}"
```

### `/uav/mode/position`
- 类型：`std_srvs/srv/Trigger`
- 语义：退出当前高层 owner，并切到 PX4 `position mode`
- 说明：该命令会先停止 `relative tracking` / `visual landing`，再调用 `/uav/control/command/position_mode`

```bash
ros2 service call /uav/mode/position std_srvs/srv/Trigger "{}"
```

### `/uav/mode/hold`
- 类型：`std_srvs/srv/Trigger`
- 语义：请求安全 `hold`
- 说明：安全类命令，优先级高于普通模式切换

```bash
ros2 service call /uav/mode/hold std_srvs/srv/Trigger "{}"
```

### `/uav/mode/stop`
- 类型：`std_srvs/srv/Trigger`
- 语义：停止当前高层 owner，并发送 `hold`
- 说明：用于结束当前 supervisor 控制流程，但不切到 PX4 `position mode`

```bash
ros2 service call /uav/mode/stop std_srvs/srv/Trigger "{}"
```

## Advanced command

如果需要保留通用入口，例如为 tracking 指定目标高度，使用：

- 服务：`/uav/mode_supervisor/command`
- 类型：`uav_mode_supervisor/srv/CommandSupervisor`

示例：

```bash
ros2 service call /uav/mode_supervisor/command \
  uav_mode_supervisor/srv/CommandSupervisor \
  "{command: start_tracking, tracking_target_height_m: 2.0}"
```
