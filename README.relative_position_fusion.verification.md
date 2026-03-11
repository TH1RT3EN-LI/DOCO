# 相对位置融合模块：验证、验收与调参说明

本文档固定 `relative_position_fusion` 后续实现阶段的验证方法。

目标不是“写完能编译就算完成”，而是确保：

- 数学核心行为正确；
- 输入适配在当前仓库场景中语义正确；
- 联合仿真与真机都能解释同一组参数；
- 重定位请求不会因调参不当而误触发或漏触发。

---

## 1. 验证分层原则

后续验证必须分四层推进：

1. **数学层单测**：不涉及 ROS topic，只验证 KF / 门控 / 触发逻辑
2. **适配器单测**：验证字段提取、TF 归一化、速度旋转、回退策略
3. **节点集成测试**：验证标准输入到标准输出的整链路
4. **场景验收**：在联合仿真与真机日志上验证误差与触发行为

只有上一层稳定后，才进入下一层。

---

## 2. 数学核心单元测试

这一层只测：

- `RelativePositionFilter`
- `RelocalizationMonitor`

不引入任何 ROS topic 与 TF。

## 2.1 `RelativePositionFilter` 必测场景

### 用例 A：零输入、零噪声保持稳定

条件：

- 初值 `r̂=[0,0]`
- `u=[0,0]`
- `Q=0`
- `z=[0,0]`
- `R=0`

期望：

- 状态保持不变
- 协方差不增长

### 用例 B：常相对速度输入与解析解一致

条件：

- `u=[1.0, -0.5] m/s`
- 无更新
- 连续预测 `N` 步

期望：

- `r̂_N = r̂_0 + T * u`
- `P` 仅由 `Q` 线性增长

### 用例 C：仅预测时 `P` 线性增长

条件：

- `Q_k = q_rate * dt * I`
- 不做测量更新

期望：

- `P_k = P_0 + Σ(Q_i)`
- 最大特征值按理论线性增长

### 用例 D：正常更新时协方差收缩

条件：

- 有限 `P`、有限 `R`
- innovation 有效

期望：

- 更新后 `trace(P)` 下降
- `K` 在合理范围内

### 用例 E：跳变测量被门控拒绝

条件：

- 先让状态收敛
- 再输入一个偏移明显超过正常范围的 `z`

期望：

- `d² > 5.99`
- 更新被拒绝
- 状态保持预测值

### 用例 F：`rho` 量纲正确

条件：

- 构造已知二维协方差

期望：

- `rho = gamma * sqrt(lambda_max(P))`
- 不允许误写成 `gamma * lambda_max(P)`

## 2.2 `RelocalizationMonitor` 必测场景

### 用例 A：未初始化即请求重定位

期望：

- `initialized=false` 时，`relocalize_requested=true`

### 用例 B：`rho` 超阈值后有迟滞进入

期望：

- 单次越阈不立刻触发
- 连续满足 `enter_hold_cycles` 后才触发

### 用例 C：`rho` 回落后有迟滞退出

期望：

- 单次回落不立刻清除
- 连续满足 `exit_hold_cycles` 后才清除

### 用例 D：连续门控拒绝强制触发

期望：

- 当连续拒绝次数超过阈值时，强制请求重定位

---

## 3. 适配器单元测试

这一层重点验证“当前仓库已有输入到底被解释对了没有”。

## 3.1 `UavStateAdapter` 必测场景

### 用例 A：从 `/uav/odom` 正确提取 `x/y`

验证：

- pose 在 `global_frame` 时直接提取
- pose 在 `uav_odom` / `uav_map` 时，通过 tf2 正确变换到 `global_frame`

### 用例 B：协方差子块提取正确

验证：

- `cov[0], cov[1], cov[6], cov[7]` 映射到 `2×2` 矩阵正确
- 对角协方差源与当前 PX4 平面读取器兼容

### 用例 C：UAV 速度语义正确

验证：

- `uav_twist_in_child_frame=false` 时，按 pose frame / world frame 语义处理
- `uav_twist_in_child_frame=true` 时，按 global yaw 旋转

### 用例 D：速度缺失时位姿差分回退正确

验证：

- 使用两帧 pose 求差分
- dt 太小或太大时正确拒绝回退

## 3.2 `UgvStateAdapter` 必测场景

### 用例 A：全局位姿优先级正确

验证：

- `/amcl_pose` 新鲜时优先使用 `/amcl_pose`
- `/amcl_pose` 缺失时，仅在 `assume_ugv_odom_is_global=true` 下才允许回退到 `/ugv/odom`

### 用例 B：速度优先级正确

验证：

- 优先 `/ugv/odometry/filtered`
- 再回退 `/ugv/odom`
- 最后回退 pose 差分

### 用例 C：UGV 车体系速度旋转正确

验证：

- 已知 yaw 下，将 `(vx_body, vy_body)` 正确旋转到 `global`
- yaw 不可用时，该速度源失效

### 用例 D：stale / NaN / 协方差缺失时行为正确

验证：

- freshness 标记准确
- fallback 标记准确
- 协方差回退不会产生 NaN

---

## 4. 节点级集成测试

这一层开始把适配器、滤波器和输出话题串起来。

## 4.1 集成测试目标

- 证明标准输入能得到标准输出；
- 证明输出时间戳、frame id、topic 语义稳定；
- 证明 diagnostics 能完整描述当前内部状态。

## 4.2 必测集成场景

### 用例 A：最小健康流

输入：

- 正常 `/uav/odom`
- 正常 `/amcl_pose`
- 正常 `/ugv/odometry/filtered`

期望：

- 发布 5 类标准输出
- `initialized=true`
- `measurement_used=true`
- `relocalize_requested=false`

### 用例 B：只有仿真回退流

输入：

- `/uav/odom`
- `/ugv/odom`
- 参数 `assume_ugv_odom_is_global=true`

期望：

- 节点可运行
- diagnostics 中明确声明 pose / velocity 来源来自 fallback

### 用例 C：丢失测量仅预测

输入：

- 正常速度
- 位置测量中断

期望：

- 估计继续连续输出
- `P` 增大
- `rho` 增大

### 用例 D：跳变测量

输入：

- 某一时刻 `/uav/odom` 或 `/amcl_pose` 突然跳 2–3 m

期望：

- `gate_rejected=true`
- 状态不被拉飞
- diagnostics 正确记录 `mahalanobis_d2`

### 用例 E：未初始化状态

输入：

- 只有速度、无有效位置测量

期望：

- `initialized=false`
- `relocalize_requested=true`

---

## 5. 联合仿真验收

这一层验证该模块是否真的适用于当前仓库。

## 5.1 当前联合仿真最小验收场景

场景入口：

- `ws/src/duojin01_bringup/launch/sim.launch.py`

前提：

- `global_to_ugv_map = 0`
- `global_to_uav_map = 0`
- `assume_ugv_odom_is_global=true`

### 验收项 A：静止场景

期望：

- `r̂` 稳定
- `rho < 0.3 m`
- 不误触发 `relocalize_requested`

### 验收项 B：UGV 匀速移动，UAV 悬停

期望：

- `r̂` 随相对运动连续变化
- 相对速度调试输出方向正确

### 验收项 C：短时测量丢失

做法：

- 暂时停止一侧 pose 输入，持续 1–3 s

期望：

- 节点仍持续发布估计
- `rho` 按预期增大

### 验收项 D：跳变注入

做法：

- 对 UAV 或 UGV pose 注入位置跃迁

期望：

- 本拍更新被拒绝
- diagnostics 能看到 reject 计数增长

## 5.2 与真值的比较策略

联合仿真验收应优先与真值比，而不是只看是否“看起来平滑”。

推荐两种方式：

1. **短期实现最简版本时**：
   使用当前可用的 `/uav/odom` 与 `/ugv/odom` 做一致性检查

2. **中期完善时**：
   新增专用对比工具，直接从 Gazebo `pose/info` 获取 UAV / UGV 世界位姿作为真值

第二种方式更严格，建议作为正式验收标准。

---

## 6. 真机 / 半实物验收

真机验证必须和仿真分层推进，不能一开始就上动态复杂工况。

## 6.1 基础静止验证

条件：

- UAV 静止悬停
- UGV 静止停放

期望：

- `r̂` 抖动小于设定阈值
- `rho` 稳定且不过快增长

## 6.2 单边运动验证

### 用例 A：UGV 慢速直线

期望：

- `r̂` 能连续跟踪
- 方向不反、不交换坐标轴

### 用例 B：UAV 慢速平移

期望：

- `r̂` 变化符合相对运动直觉

## 6.3 弱定位验证

人为制造：

- UGV 全局位姿间歇更新
- UAV 全局定位质量变弱

期望：

- `rho` 明显增大
- `relocalize_requested` 在阈值前后触发符合预期

---

## 7. 验收标准

v1 建议固定采用以下验收标准。

## 7.1 功能正确性

- 模块能在联合仿真中持续运行
- 标准 5 类输出全部可见
- 未初始化、降级、拒绝、恢复等状态都能在 diagnostics 中解释清楚

## 7.2 误差目标

### 联合仿真

- 正常稳态相对位置误差 `< 0.3 m`
- 保守上界触发在 `1.0 m` 阈值前后行为一致

### 真机 / 半实物

- 静止 5 分钟不误触发
- 弱定位时 `rho` 会合理增长而不是长期假稳定

## 7.3 工程可观测性

- 所有核心输出时间戳不为 0
- `P` 不出现 NaN
- `rho` 不出现 NaN
- diagnostics 字段齐全

---

## 8. `Q` 的调参公式

这一节是后续现场调参最需要用到的公式。

在仅预测、各向同性扩散条件下：

- `P(T) = P(0) + q_rate * T * I`
- `rho(T) = gamma * sqrt(lambda(T))`

若希望 `rho` 从 `rho_0` 增长到 `rho_1`，耗时 `T`，则：

`q_rate = (((rho_1 / gamma)^2 - (rho_0 / gamma)^2) / T)`

### 推荐示例

取：

- `rho_0 = 0.3 m`
- `rho_1 = 1.0 m`
- `gamma = 2.0`

则：

`q_rate = 0.2275 / T`

对应不同时间：

| 目标增长时间 T | 推荐 `q_rate` |
|---:|---:|
| 30 s | `0.00758 m²/s` |
| 60 s | `0.00379 m²/s` |
| 120 s | `0.00190 m²/s` |

因此本文档默认把 nominal 取在 `0.004 m²/s` 左右。

---

## 9. `R` 的来源与保护策略

`R_k` 固定由：

`R_k = Σ_{g,k} + Σ_{a,k}`

为了保证工程鲁棒性，每个来源协方差在相加前必须执行：

1. NaN / inf 检查
2. 对角项下限保护
3. 对角项上限保护
4. 年龄膨胀

推荐规则：

- 缺失时回退到 `covariance_nan_fallback_m2 * I`
- 每老化 `1 s`，对角增加 `covariance_age_inflation_m2_per_s`

这样做的目的：

- 新鲜数据方差小，测量影响大；
- 老旧数据方差自动变大，避免过度相信旧测量。

---

## 10. 日志与录包建议

后续仿真 / 真机测试时，建议至少录制：

- `/uav/odom`
- `/amcl_pose`
- `/ugv/odometry/filtered`
- `/ugv/odom`
- `/relative_position/estimate/global`
- `/relative_position/estimate/uav_body`
- `/relative_position/debug/relative_velocity`
- `/relative_position/relocalize_requested`
- `/relative_position/diagnostics`
- `/tf`
- `/tf_static`

这样后续才能离线复现：

- 估计误差
- 触发时机
- 门控行为
- 来源切换行为

---

## 11. 故障注入清单

后续集成验证时，建议固定做以下故障注入：

### 故障 A：AMCL 中断

目的：验证 UGV 全局位姿丢失后的回退与触发。

### 故障 B：UAV 位置跳变

目的：验证门控是否能挡住重定位突变。

### 故障 C：TF 暂时不可用

目的：验证适配器对 TF 失败的降级逻辑。

### 故障 D：协方差异常为零

目的：验证 floor 保护是否生效，避免滤波器过度自信。

### 故障 E：速度缺失

目的：验证 pose 差分回退与 `q_rate` 切换是否正确。

---

## 12. 验证小结

如果后续实现后只做一件验证工作，那必须先把下面三件事跑通：

1. 门控能挡住跳变；
2. `rho` 会在无测量时持续增长；
3. `relocalize_requested` 会在进入 / 退出阈值附近稳定切换而不抖振。

这三件事是本模块从“一个平滑器”变成“一个可用于系统决策的融合器”的关键。
