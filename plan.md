# 开发计划

## 任务列表

- [ ] 接 `diesel_vision` 的目标，实现多源目标融合与注入
- [ ] 实现友军**过滤**（进入候选集之前）—— 核心函数与 7 条测试已落地，待接入链路
- [ ] 实现弹道解算与云台伺服（硬实时环，独立线程、禁动态分配）—— 弹道暂不做 直瞄已落地 伺服环待做
- [x] **整车状态估计 EKF**（11 维，`predict` / `update` / 观测雅可比 `h_jacobian`）—— 2026-10-03 落地
- [x] **数据关联与野值剔除**（最近邻认板 + 距离门限 + NIS 卡方门限）—— 2026-10-05 落地
- [ ] 定义开火门限的目标准入契约（对接 X1 安全门）

## 当前 Agent State

TARGET_ASSOCIATION_READY

## Before Snapshot

commit hash:   19eb996
branch:        feat/associate
modified files: include/diesel_auto_aim/target.hpp · src/target.cpp · test/test_target.cpp · tree.md · decision.md · plan.md
risk level:    L1

## 模糊点与待确认项

- 本仓接口尚未冻结，任务清单为**方向性**的，落地顺序以 `foray_docs/algorithm_structure.md` §9 演进阶段为准
- 依赖的自研仓尚未建齐，跨仓任务需等对方 `README.md` 明确接口后再启动
- `package.xml` 的 `license` 暂为 `TODO`：组织文档未规定私有仓许可证，待确认后补
- 敌我标识的最终来源待定：接口冻结前本模块用自己的 `TargetCandidate`；冻结后需补一层「上游消息 → TargetCandidate」转换

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md`
Initialization: 2026-02-19
Commit: （首次提交）

## Acceptance Criteria

- [x] 本仓能独立 `colcon build`（本地：1 package finished）
- [ ] CI 绿灯（待 PR 触发）
- [x] 每个新增模块带独立可执行测试（友军过滤 7 条 · 云台角 5 条 · 整车 EKF 5 条，共 **17 条全绿**）
