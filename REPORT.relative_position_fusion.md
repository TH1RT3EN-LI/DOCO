# relative_position_fusion 实现报告

生成时间：2026-03-12

## 1. 目标

本次实现完成了 `relative_position_fusion` 模块的 v1 落地，目标是为当前 DOCO 工作区提供一个：

- 同时兼容仿真与真机输入的二维相对位置融合节点；
- 基于速度差预测 + 位置差更新的最小 Kalman 融合器；
- 带马氏距离门控、协方差传播与重定位请求触发逻辑；
- 可复用的独立 ROS 2 包、配置文件、launch 和单元测试集。

## 2. 本次实际完成内容

### 2.1 新增包

新增 ROS 2 包：`ws/src/relative_position_fusion`

包含：

- 核心库：
  - `RelativePositionFilter`
  - `RelocalizationMonitor`
  - `TimeAlignmentBuffer`
  - `UavStateAdapter`
  - `UgvStateAdapter`
  - `DiagnosticsPublisher`
- 主节点：`relative_position_fusion_node`
- 配置：
  - `relative_position_fusion.common.yaml`
  - `relative_position_fusion.duojin_sim.yaml`
  - `relative_position_fusion.sim_navigation.yaml`
  - `relative_position_fusion.hw.yaml`
- 独立 launch：`relative_position_fusion.launch.py`
- 单元测试：5 个 gtest 目标

### 2.2 算法行为

已按设计实现：

- 状态：二维相对位置 `r ∈ R²`
- 预测：`r_{k+1} = r_k + dt * u_k + w_k`
- 输入：`u_k = v_g - v_a`
- 测量：`z_k = p_g - p_a`
- 测量协方差：`R_k = Σ_g + Σ_a`
- 过程噪声：`Q_k = q_rate(mode) * dt * I`
- 门控：二维马氏距离 + `5.99` 阈值
- 误差上界：`rho = gamma * sqrt(lambda_max(P))`
- 触发：带进入/退出迟滞和连续门控拒绝保护

### 2.3 ROS 2 输出

主节点当前发布：

- `/relative_position/estimate/global`
- `/relative_position/estimate/uav_body`
- `/relative_position/debug/relative_velocity`
- `/relative_position/relocalize_requested`
- `/relative_position/diagnostics`

### 2.4 仓库接入

已完成以下接入：

- 联合仿真入口接入新模块：
  - `ws/src/duojin01_bringup/launch/sim.launch.py`
- `duojin01_bringup` 新增运行依赖：
  - `ws/src/duojin01_bringup/package.xml`
- 补充 UAV 真机 / 半实物用的 PX4 平面状态归一化 launch：
  - `ws/src/uav_bringup/launch/px4_planar_odom.launch.py`
- 为 `duojin01_bringup` 补齐与当前工作区一致的 setuptools argv 清洗，以便在 `--symlink-install` 下可构建：
  - `ws/src/duojin01_bringup/setup.py`

## 3. 设计文档同步状态

此前规划的设计文档集保留并可直接指导后续迭代：

- `ws/README.md`
- `ws/README.relative_position_fusion.architecture.md`
- `ws/README.relative_position_fusion.interfaces.md`
- `ws/README.relative_position_fusion.implementation.md`
- `ws/README.relative_position_fusion.verification.md`
- `ws/README.relative_position_fusion.codex.md`

## 4. Docker 内构建与测试记录

### 4.1 构建环境

- Docker 镜像：`duojin01-dev:humble-harmonic`
- 工作目录：`/repo/ws`
- 构建产物位置：`ws/build`、`ws/install`、`ws/log`

### 4.2 实际执行命令

#### 构建 `relative_position_fusion + duojin01_bringup`

```bash
docker run --rm -u $(id -u):$(id -g) \
  -v /home/th1rt3en/DOCO:/repo \
  -w /repo/ws \
  duojin01-dev:humble-harmonic \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    colcon build --packages-select relative_position_fusion duojin01_bringup --symlink-install --event-handlers console_direct+'
```

#### 构建并测试 `relative_position_fusion`

```bash
docker run --rm -u $(id -u):$(id -g) \
  -v /home/th1rt3en/DOCO:/repo \
  -w /repo/ws \
  duojin01-dev:humble-harmonic \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    colcon build --packages-select relative_position_fusion --symlink-install --event-handlers console_direct+ && \
    colcon test --packages-select relative_position_fusion --event-handlers console_direct+ && \
    colcon test-result --verbose --test-result-base build/relative_position_fusion'
```

#### launch 语法检查

```bash
docker run --rm -u $(id -u):$(id -g) \
  -v /home/th1rt3en/DOCO:/repo \
  -w /repo/ws \
  duojin01-dev:humble-harmonic \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /repo/ws/install/setup.bash && \
    ros2 launch relative_position_fusion relative_position_fusion.launch.py --show-args >/tmp/rpf_args.txt && \
    ros2 launch uav_bringup px4_planar_odom.launch.py --show-args >/tmp/uav_args.txt && \
    ros2 launch duojin01_bringup sim.launch.py --show-args >/tmp/duojin_args.txt && \
    echo OK'
```

### 4.3 测试结果

`relative_position_fusion` 单元测试结果：

- 总计：20 tests
- 结果：0 errors, 0 failures, 0 skipped

覆盖范围：

- `RelativePositionFilter`
- `RelocalizationMonitor`
- `TimeAlignmentBuffer`
- `UavStateAdapter`
- `UgvStateAdapter`

### 4.4 构建过程中额外修复

为使 `duojin01_bringup` 在当前 Docker / colcon / setuptools 组合下可构建，补充了与 `ugv_bringup` 同风格的 argv sanitizer。该修复是构建链兼容修复，不改变联合仿真业务逻辑。

## 5. 当前已知边界

本次实现仍保持 v1 设计边界，不做以下内容：

- 不直接执行 UAV 重定位动作，只发布请求信号；
- 不估计相对 yaw；
- 不引入自定义消息；
- 不接管 `enable_dynamic_global_alignment` 的运行时行为；
- 不把视觉降落或 AprilTag 末端控制耦合到融合节点内部。

## 6. 推荐后续工作

建议按以下顺序继续推进：

1. 在联合仿真中增加一个真值对比工具，直接评估相对位置误差；
2. 为 `duojin01_bringup` 增加预设参数开关，方便切换 `duojin_sim` / `sim_navigation`；
3. 在真机或袋回放上标定 `q_rate_*` 参数；
4. 若上层确实需要，再新增一个 orchestrator 消费 `/relative_position/relocalize_requested`。

## 7. 变更文件概览

### 新增

- `ws/src/relative_position_fusion/package.xml`
- `ws/src/relative_position_fusion/CMakeLists.txt`
- `ws/src/relative_position_fusion/include/relative_position_fusion/*`
- `ws/src/relative_position_fusion/src/*`
- `ws/src/relative_position_fusion/config/*`
- `ws/src/relative_position_fusion/launch/relative_position_fusion.launch.py`
- `ws/src/relative_position_fusion/test/*`
- `ws/src/uav_bringup/launch/px4_planar_odom.launch.py`
- `ws/REPORT.relative_position_fusion.md`

### 修改

- `ws/README.md`
- `ws/src/duojin01_bringup/package.xml`
- `ws/src/duojin01_bringup/launch/sim.launch.py`
- `ws/src/duojin01_bringup/setup.py`

## 8. 结论

本次已经把“文档计划”推进为“可编译、可测试、可被 launch 解析”的实现版本：

- 新包已存在并可在 Docker 内构建；
- 单元测试已全部通过；
- 联合仿真入口已能挂载该模块；
- UAV 真机侧 `/uav/odom` 归一化入口已补齐；
- 所有产物均写入 `ws/build`、`ws/install`、`ws/log`。
