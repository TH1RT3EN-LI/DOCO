# sim_navigation 中 relative_position_fusion 改造说明

## 1. 这次改了什么

这次改造的核心不是“把所有门限都放宽”，而是把 `relative_position_fusion` 的**启动阶段**和**稳态阶段**分开处理：

- 启动阶段更容易形成第一组可用测量；
- 一旦滤波器初始化成功，立即恢复现有稳态策略；
- `relative_tracking` 保持原有控制语义，重点先解决 fusion “起不来”的问题；
- `duojin01_bringup` 的 `sim_navigation` 已经集成这套配置，不需要再手工单独拼 launch。

这套方案可以概括为：

> **对齐驱动的宽启动、严稳态**

即：**通过缓冲区时间对齐解决“有数据却起不来”的根因，在启动期宽容配对，在稳态期继续严格更新。**

---

## 2. 为什么要改

在原来的链路里，融合测量更接近“最新帧硬同步”模式：

- UAV 取一帧最新 pose；
- UGV 取一帧最新 pose；
- 直接用这两帧做相对位置测量；
- 如果两帧时间差超过 `measurement_sync_tolerance_sec`，这一拍就不能用。

这在仿真和异步数据流里会出现一个典型问题：

- 两侧都在正常发数据；
- 两侧 buffer 里其实存在“时间上很接近”的历史样本；
- 但**恰好最新两帧彼此错开**；
- 于是 fusion 一直拿不到第一组可用测量，滤波器就无法初始化。

从现象上看，会表现为：

- 明明 topic 都在跳，fusion 却迟迟不输出有效结果；
- 静止场景下更明显，因为大家容易误以为“是不是必须运动了才能算”；
- `relocalize_requested` 在旧语义下也可能过早拉高，给下游脚本造成“系统已失败”的假象。

### 关键判断

这里的根因通常不是“滤波器太保守”，而是：

1. **测量配对方式过于刚性**；
2. **启动期和稳态期使用同一套严阈值**；
3. **未初始化时的 relocalize 语义过于激进**。

所以，最不推荐的做法是**直接全局放宽稳态门限**。那样虽然可能更容易起算，但会把稳态质量一起带坏，尤其会影响：

- 测量质量；
- gate 拒绝行为；
- 协方差收敛与 `rho` 语义；
- 异常场景下的误更新风险。

---

## 3. 这次方案为什么更合适

这次采用的是“**缓冲区最近时刻配对 + 启动期宽容、稳态期严格**”方案，原因如下：

### 3.1 优先修根因，而不是只调门限

新的测量构造不再执着于“最新帧必须刚好同步”，而是：

1. 先拿 UAV / UGV 两边各自当前最新 fresh pose；
2. 选择时间更新的一侧作为 anchor；
3. 去另一侧 buffer 里找距离 anchor 最近的 pose；
4. 只要这对样本的时间差在当前阶段允许范围内，就构造测量。

这样做的好处是：

- 有历史样本就能被利用起来；
- 更符合异步传感器系统的现实；
- 不需要上来就做插值，复杂度和调试成本都比较低；
- 能明显减少“其实有数据，但就是配不成对”的情况。

### 3.2 启动期单独放宽，稳态立即收紧

新的逻辑只在**滤波器未初始化**时使用更宽的配对容忍度：

- 启动期：`bootstrap_measurement_sync_tolerance_sec`
- 稳态期：`measurement_sync_tolerance_sec`

因此它不是“永久放宽”，而是：

- 先尽快拿到第一组合理测量，完成初始化；
- 初始化成功后立刻回到原来的严格标准。

这使得方案同时兼顾：

- **启动成功率**；
- **稳态更新质量**；
- **实现复杂度可控**。

### 3.3 启动期 relocalize 不再过早误伤

新增了启动 grace 窗口：

- 在 grace 时间内，如果滤波器还没初始化：
  - diagnostics 继续报 `WARN`；
  - `relocalization_reason` 标成 `startup_grace`；
  - `relocalize_requested` 暂时不拉高。
- 超过 grace 还拿不到可用测量，才恢复原有请求语义。

这让系统启动过程更符合实际：

- 刚启动时允许缓冲区积累历史；
- 不会因为“还在攒第一组测量”就被脚本误判为 ready 或 failure；
- 下游自动化逻辑更稳定。

### 3.4 不把 tracking 一起搅进去

`relative_tracking` 在这里被视为 fusion 的下游使用者，而不是本次问题的根因。

因此本次没有去改：

- tracking 服务语义；
- 跟踪控制律；
- 控制 topic；
- 相关单元测试行为预期。

这样可以把改动范围收敛在“**先保证 fusion 更容易起算**”这一件事上，真实收益更高，也更容易回归验证。

---

## 4. 改动后的实际行为

## 4.1 融合何时可以开始计算

现在 fusion 开始计算的条件可以概括成：

1. UAV 和 UGV 至少都要有 **fresh pose**；
2. 两边 pose 协方差要有效；
3. 在 buffer 中能找到一对满足当前阶段时间容忍度的 pose；
4. 一旦形成第一组有效测量，就可以 `Initialize()`。

也就是说：

- **不要求车辆必须运动了才能开始融合**；
- **静止场景也可以初始化**，前提是上游仍然在持续发布新鲜 pose；
- 如果静止时上游完全不再发 pose，这个方案也不会“凭空造数据”。

### 4.2 启动后和稳态后的区别

- **启动期（bootstrap）**：更宽松，只为拿到第一组可用测量。
- **稳态期（nominal）**：恢复原本严格阈值，gate / rho / relocalize 行为保持原设计。

因此，这次改造的目标不是“让所有数据都更容易进滤波器”，而是：

- **让系统更容易开始**；
- **开始以后，继续保持稳态质量**。

### 4.3 诊断语义更清晰

新增 diagnostics 字段：

- `measurement_phase`：`bootstrap` 或 `nominal`
- `measurement_pairing_mode`：固定为 `nearest_buffer_pair`
- `measurement_pair_dt_s`：这次使用的测量配对时间差

这几个字段可以直接帮助判断：

- 现在处于启动期还是稳态期；
- 当前是否用了最近样本配对；
- 配对时间差到底有多大。

---

## 5. 代码与配置落点

本次改动主要分成三块。

### 5.1 `relative_position_fusion` 核心行为

- `src/relative_position_fusion/src/time_alignment_buffer.cpp`
  - 新增最近 pose 查询能力 `NearestPose(...)`。
- `src/relative_position_fusion/src/uav_state_adapter.cpp`
  - 新增 `BuildPoseStateNear(...)`。
- `src/relative_position_fusion/src/ugv_state_adapter.cpp`
  - 新增 `BuildPoseStateNear(...)`，并保持 UGV 侧“优先 AMCL，必要时才回退 odom”的优先级。
- `src/relative_position_fusion/src/relative_position_fusion_node.cpp`
  - `BuildMeasurement(...)` 改为最近时刻配对；
  - 增加 bootstrap 同步容忍度；
  - 增加 startup relocalize grace。
- `src/relative_position_fusion/src/diagnostics_publisher.cpp`
  - 发布新的 diagnostics 字段。

### 5.2 默认融合参数

公共默认参数放在：

- `src/relative_position_fusion/config/relative_position_fusion.common.yaml`

新增了两个关键参数：

- `bootstrap_measurement_sync_tolerance_sec: 0.12`
- `startup_relocalize_grace_sec: 1.0`

稳态阈值仍然保留：

- `measurement_sync_tolerance_sec: 0.05`

### 5.3 `duojin01_bringup` 的 `sim_navigation` 集成

为了让 `sim_navigation` 直接使用这套能力，这次把融合配置以 **bringup overlay** 的形式集成进去了。

相关文件：

- `src/duojin01_bringup/launch/sim_navigation.launch.py`
- `src/relative_position_fusion/launch/relative_position_fusion.launch.py`
- `src/duojin01_bringup/config/relative_position_fusion.sim_navigation.yaml`

其中 `sim_navigation` 默认会把 overlay 传给 fusion launch，当前 overlay 内容为：

- `assume_ugv_odom_is_global: false`
- `pose_timeout_sec: 2.0`
- `bootstrap_measurement_sync_tolerance_sec: 0.15`
- `startup_relocalize_grace_sec: 2.0`

这表示在 `sim_navigation` 场景下：

- 启动等待历史样本的时间更宽松；
- pose 过期判断更宽一些；
- relocalize 请求会更晚进入激进语义；
- 但稳态 `measurement_sync_tolerance_sec` 并没有被永久放宽。

---

## 6. 脚本行为为什么也要改

`ws/scripts/uav_relative_tracking.sh` 的 ready 判定也做了同步更新。

原因是：

- 旧逻辑如果只看 `/relative_position/relocalize_requested=false`，在 startup grace 阶段会误以为“fusion 已就绪”；
- 但实际上这时系统可能只是“暂时不请求 relocalize”，并不等于“已经初始化成功”。

现在脚本会优先看 diagnostics：

- `initialized == true`
- `relocalize_requested == false`

只有这两个同时满足，才认为 fusion 真正 ready。

这样能避免：

- 启动过早触发 tracking；
- tracking 在 fusion 尚未稳定时就开始工作；
- 启动脚本和节点内部语义不一致。

---

## 7. 结果与收益

这次改造的预期收益是：

### 7.1 能更容易开始融合

尤其是在下面这些场景里收益明显：

- UAV / UGV pose 到达不同步；
- topic 都是活的，但最新帧恰好错位；
- 系统刚启动，buffer 还在积累历史；
- 静止场景下仍持续发布 pose。

### 7.2 静止场景不再天然吃亏

只要上游仍持续发新鲜 pose，系统就可以初始化；不再隐含要求“必须通过运动制造可用信息”。

### 7.3 稳态质量基本不变

这次没有去动：

- `use_measurement_gating`
- `gate_chi2_threshold`
- `q_rate_*`
- `rho_*`
- 预测模型
- yaw 发布逻辑

因此稳态表现仍然以原有设计为主，不会因为“为了更容易起算”而把整条链路永久放松。

---

## 8. 使用方法

## 8.1 构建

如果工作区还没重新构建，先执行：

```bash
cd /home/th1rt3en/DOCO/ws
colcon build --packages-select relative_position_fusion duojin01_bringup
source install/setup.bash
```

如果你当前工作区里存在旧的、失效的 prefix 路径缓存，可能需要先清理相关 `build/` 和 `install/` 产物后再重新构建。

## 8.2 直接启动 `sim_navigation`

现在推荐的启动入口就是：

```bash
cd /home/th1rt3en/DOCO/ws
source install/setup.bash
ros2 launch duojin01_bringup sim_navigation.launch.py
```

默认情况下，这个 launch 已经会：

- 启动 `relative_position_fusion`；
- 默认使用 `duojin_sim` preset；
- 自动叠加 `duojin01_bringup` 自己的 `sim_navigation` overlay；
- 按 `enable_relative_tracking` 决定是否一起启动 tracking。

## 8.3 覆盖 fusion 配置

如果你想临时替换 `sim_navigation` 使用的 overlay，可以在 launch 时传：

```bash
ros2 launch duojin01_bringup sim_navigation.launch.py \
  relative_position_fusion_config:=/absolute/path/to/your_overlay.yaml
```

如果你只想跑 fusion、不带 tracking，可以传：

```bash
ros2 launch duojin01_bringup sim_navigation.launch.py \
  enable_relative_tracking:=false
```

## 8.4 查看 fusion 是否真的 ready

推荐优先看 diagnostics：

```bash
ros2 topic echo /relative_position/diagnostics --once
```

重点关注这些字段：

- `initialized`
- `relocalize_requested`
- `measurement_phase`
- `measurement_pair_dt_s`
- `measurement_available`
- `measurement_used`
- `relocalization_reason`

通常可以这样理解：

- `initialized=true` 且 `relocalize_requested=false`：fusion 已 ready
- `measurement_phase=bootstrap`：还在启动期
- `measurement_phase=nominal`：已经进入稳态期
- `relocalization_reason=startup_grace`：还在启动 grace 窗口内

## 8.5 启动相对跟踪脚本

如果 `sim_navigation` 已启动，并且 fusion ready，可以使用：

```bash
cd /home/th1rt3en/DOCO
./ws/scripts/uav_relative_tracking.sh start
```

或者指定起飞高度：

```bash
./ws/scripts/uav_relative_tracking.sh start 1.2
```

该脚本现在会优先依据 diagnostics 判断 fusion 是否真正 ready，而不是只看 `/relative_position/relocalize_requested`。

---

## 9. 如何判断是不是“启动问题”而不是“稳态问题”

可以参考下面这个经验判断：

### 9.1 启动问题

常见表现：

- `initialized=false`
- `measurement_available=false`
- `measurement_phase=bootstrap`
- `relocalization_reason=startup_grace` 或 `uninitialized`

这通常说明：

- 两边还没形成可用 pose 配对；
- 或者 pose 太旧；
- 或者静止时上游根本没持续发布数据。

### 9.2 稳态问题

常见表现：

- `initialized=true`
- `measurement_available=true` 但 `measurement_used=false`
- `gate_rejected=true`
- `measurement_phase=nominal`

这说明 fusion 已经起来了，但某些测量在稳态下被 gate 拒绝，这属于另外一类问题，不是本次启动改造的对象。

---

## 10. 已知边界与不做的事情

这次方案刻意保持克制，以下内容**没有改**：

- 不做 pose 插值；
- 不做 yaw 插值；
- 不改预测模型，预测仍用当前 `BuildState(now)` 的速度；
- 不改 tracking 控制律；
- 不通过永久放宽稳态门限来换启动成功率。

因此它解决的是：

- **有历史、有新鲜数据，但因为最新帧错位导致起不来**。

它**不解决**的是：

- 上游静止时完全不发 pose；
- 上游时间戳本身错误；
- 稳态 gate 频繁拒绝所代表的模型或观测质量问题。

---

## 11. 一句话总结

这次改造的本质是：

> **让 fusion 更容易“开始”，但不让稳态质量因此变松。**

如果你现在的目标是“在 `duojin01_bringup` 的 `sim_navigation` 中更稳定地拉起相对位置融合，并让 tracking 在真正 ready 之后再启动”，那么这套方案比“简单放宽所有门限”更稳、更可控，也更接近真实可落地的工程结果。
