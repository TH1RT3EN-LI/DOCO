# DOCO 仓库拆分说明

这个工作区已经完成阶段 1 的边界梳理：

- `uav_gz_bridge`：承接 UAV Gazebo sensor / ground-truth bridge
- `uav_sim_bringup`：承接 UAV SITL / Gazebo 启动入口与 PX4 仿真资产
- `ugv_sim_bringup`：承接 UGV 联合仿真入口与 Gazebo-facing 配置
- `uav_bringup` / `ugv_bringup`：保留真机导向入口，并通过 wrapper 兼容旧 sim launch
- `ugv_bringup` 导航参数已切到 `nav2.common.yaml + nav2.hw/sim.overlay.yaml`，避免再维护平行的整份 Nav2 参数
- `duojin01_bringup`：联合仿真编排只经由 `uav_sim_bringup` / `ugv_sim_bringup` 进入 UGV/UAV 仿真栈，不再直接触达 `ugv_sim_tools` 或 `ugv_bringup` 的导航细节
- `manifests/*.repos`：作为阶段 1 的工作区组装入口

## 阶段 2 目标仓库

- `doco_common`：`src/quadrotor_msgs`、`src/relative_position_fusion`
- `doco_uav`：`src/uav_bridge`、`src/uav_bringup`、`src/uav_visual_landing`、`src/uav_mode_supervisor`、`src/uav_description`
- `doco_ugv`：`src/ugv_base_driver`、`src/ugv_bringup`、`src/ugv_description`、`src/ugv_safety_watchdog`、`src/ugv_slam_tools`、`src/ugv_teleop`、`src/lslidar_driver`
- `doco_sim`：`src/uav_gz_bridge`、`src/uav_sim_bringup`、`src/ugv_sim_bringup`、`src/ugv_controller_emulator`、`src/ugv_sim_tools`、`src/sim_worlds`、`src/duojin01_bringup`
- `doco_manifests`：`manifests/`

## 执行方式

使用 `scripts/phase2_split_repos.sh`：

```bash
cd /path/to/DOCO/ws
./scripts/phase2_split_repos.sh
./scripts/phase2_split_repos.sh --output-dir ~/tmp/doco-split --execute
```

默认是 dry-run，只打印命令；加 `--execute` 才真的 clone + `git filter-repo`。

## 当前阶段的已知约束

- 阶段 2 的仿真工作区仍需要同时拉取 `doco_uav` 与 `doco_sim`，原因是仿真链仍依赖 `uav_bridge`、`uav_description`、`uav_visual_landing` 与 `uav_mode_supervisor`，而不是默认值模块共享。
- `manifests/*.repos` 当前仍指向单仓库 `DOCO.git`；完成阶段 2 后，只需要把 self repo 条目替换成新仓库 URL / tag 即可。
- 旧 `uav_bringup/*.sim*` 与 `ugv_bringup/*.sim*` launch 现在是兼容 wrapper；下一轮稳定后可以删除。
