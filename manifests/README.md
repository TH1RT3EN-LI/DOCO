# DOCO manifests

这些 manifest 是当前 `阶段 1` 的工作区草案：

- 仍然基于现有单仓库 `DOCO.git`
- 通过不同 `.repos` 文件约束外部依赖闭包
- 统一把主仓库克隆到 `workspace/`，便于在 `workspace/src` 与 `workspace/third_party` 下补齐外部仓库
- 所有上游依赖都固定到精确 commit SHA；发布时只更新 `.repos` 里的 SHA

## 用法

```bash
mkdir -p checkout
cd checkout
vcs import . < /path/to/DOCO/ws/manifests/dev_full.repos
cd workspace
```

随后按场景构建：

- `uav_hw.repos`：面向 UAV 真机 / 半实物调试
- `ugv_hw.repos`：面向 UGV 真机 / 导航部署
- `sim_full.repos`：面向联合仿真
- `dev_full.repos`：面向开发机的全量环境

## 说明

- 当前 self repo 仍然是单仓库，因此各 manifest 都会拉取同一个 `workspace` 仓库。
- 真正按 `doco_common / doco_uav / doco_ugv / doco_sim / doco_manifests` 拆仓的动作放在 `阶段 2`，届时只需要把这些 manifest 中的 self repo 条目替换成新仓库 URL + commit/tag。
- 这一步的价值是先把部署和更新入口稳定下来，再做历史拆仓。
