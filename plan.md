# 开发计划

## 任务列表

- [ ] 接 `diesel_vision_frontend` 的逐帧 `Observation`，实现多源目标融合与注入
- [ ] 实现友军**过滤**（进入候选集之前）—— 核心函数与 7 条测试已落地，待接入链路
- [ ] 实现弹道解算与云台伺服（硬实时环，独立线程、禁动态分配）—— 弹道暂不做 直瞄已落地 伺服环待做
- [x] **整车状态估计 EKF**（11 维，`predict` / `update` / 观测雅可比 `h_jacobian` + 仿真测试）—— 2026-10-03 落地
- [x] 单目标跟踪核心：四板位置关联、连续确认、短暂丢帧和时间跳变重置（离线测试）；尚未接入真实 Vision 数据流
- [x] 将 Tracker 的输入目标类别与 Vision 对齐为 -1/0/1/2；重装仅预留，不代表现有模型已识别
- [x] 旋转靶离线回归：板位置 + 朝向联合更新、角度 wrap 与角过程噪声；原地旋转的中心漂移测试由失败转为通过
- [x] 快转初值与关联门控离线回归：模拟器适配层放宽角速度初值方差，Tracker 的关联同时检查位置与朝向；仍待模拟器完整旋转日志和实测门限验收
- [ ] 用连续真实模型观测验证上述旋转修复；统计中心漂移、错板次数与检测缺失帧，不能把单元测试当闭环通过
- [ ] SP 双模型同帧后端对照：分开记录主关联残差、主板与次板更新导致的车心修正，再做单变量门限 A/B；当前仅增加离线诊断，尚未证明门限根因
- [x] 将 SP `Target::update` 的近三板、朝向差＋方位差选板规则移成独立纯函数并测试；已作为 `SpNearestThree` 可选策略接入 Tracker，默认仍为 Foray 距离门；尚非完整 SP 跟踪基线
- [x] 模拟器 100 帧同板离线诊断：记录角点、PnP 与关联拒绝原因；确认双模型四角系统性内缩、PnP 测距偏远，59 帧被 0.25 m 距离门拒绝。尚未修复模型输入或接实机。
- [x] 离线 v1：由整车估计按局部板号预测未来位置并计算云台相对角，接入模拟器适配层；默认提前量为 0。旧 ONNX 模型实跑 120 帧，曾进入 Tracking、仿真云台反馈角度变化且 fire=false；途中仍有 Lost，尚未做 A/B 统计
- [ ] 在同一模拟场景对比原始 PnP 直瞄、整车预测 0 ms 和不同提前量的跟随误差；确认高速换板时预测板是否仍可见，再决定切板策略
- [ ] 将同帧 PnP 姿态和 IMU 转为稳定坐标系后接入 Tracker，并显示预测四板；再验证锁定目标选择
- [ ] 定义开火门限的目标准入契约（对接 X1 安全门）
- [x] 离线适配云台系 `GimbalCommand` + 有效 IMU 四元数到 v0.2 世界系目标角控制帧，验证旋转、单位和禁用条件
- [x] 将适配帧接入平台主机链路排队入口；握手与状态回读前拒绝控制
- [ ] 完成图像与 IMU 姿态的时间对齐及真实标定外参验证，再接入装配层真实 USB 控制
- [x] （已迁出）独立实机禁能联调窗口：海康相机→双模型→可选 PnP/静止相机 Tracker；RoboLink 只握手、保活和回读 —— 调试入口归 `diesel_vision_frontend`
- [ ] 现场测量装甲板实际外轮廓尺寸与板心—车心半径，验证相机系 PnP 和静止相机 Tracker；记录真实检测/漏检/换板日志
- [ ] 实机禁能联调核验 v0.2 世界系 rad 目标角、状态回读与控制权裁决
- [ ] 连靶车后另起独立 Foray 视觉调试软件，参考现有 MIT 模拟器的接口/可视化思路；先完成当前跟踪与实机验证，不在本轮启动新项目

## 当前 Agent State

- [x] 统一C++接口AimPipeline与独立foray_auto_aim_runtime已实现：现有Vision→候选手眼→主机时刻姿态插值→世界观测→Tracker/EKF→选板/预测→直瞄→C++ USB线程；调试台只启动C++进程并显示，不再调用Python运行桥/worker。
- [ ] 新C++入口实机运动复测：当前MVS枚举不到海康相机（USB列表亦无该相机），不能完成当前取图闭环验收；MCU串口仍存在。不自动发送运动。
- [ ] 整车底盘平移场景需接入云台世界原点/里程计，目前固定原点假设仅适合当前台架；精确对时、手眼精度、半径漂移与换板稳定性仍未验收。

- [x] 2026-11-11：独立world_tracker_worker已串联世界观测→Tracker/EKF→选板/预测→直瞄；外部工具独占串口，20秒实机短测已完成。几何漂移和精确对时未验收，不能宣称完整比赛后端通过。
- [x] 世界系WorldAimSolver替换边界与直瞄实现，弹道由另一队员实现；当前不接弹道或发射。
- [ ] 后续诊断实测半径漂移、候选外参和板朝向输入；完成同场景A/B与运动对时验收。加速度观测器与电控前馈没有在本轮实现。
- [x] 2026-11-14：视觉前后端分家第一刀（I2）：本仓收敛为后端，只消费前端逐帧 `Observation`（相机系位置/法向/板号）；
  删除 `tools/`（相机调试归前端）、`camera_diagnostic_adapter` 与 `vision_adapter`，去掉 OpenCV 依赖；
  `AimPipeline::process_observations` 不再接收图像或检测器；`Target` 只保留 EKF，NIS 野值门限由 Tracker 关联层承担。
  本地 11 个测试程序 82 条全绿；CI 待 PR 触发。
- [x] 2026-11-13：时间门限三档统一 + 姿态时间偏移 + 图案模型边长可配；实机闭环 `control_seq/rx` 只差 1，稳态误差 1.4 mrad，`rejected_backward=0`。103 条测试全绿（diesel_vision 20 + 本仓 83）。后端收敛后本仓为 82 条，前端 20 条待 I3 收敛后合并统计。
- [ ] 曝光绝对时刻标定：屏幕闪烁(PRBS)+固定曝光+自动开始采集+互相关分析链路已就位；两次采集均因相机未正对屏幕作废（最亮帧为暗墙面）。参数 `--exposure-offset-ms` 已接入取帧处，默认 0 待填。

- [x] 2026-11-11：只读入口增加 `--target-feed-json`，复用双模型/PnP 导出逐帧观测（含空帧），供人工静止点动验证；未接入运动 Tracker 或自动发射。

TRACKED_AIM_SIM_V1_RUNNABLE：整车预测已进入仿真控制输出，旧模型 120 帧短跑出现 Tracking 与 Lost；尚无 A/B 指标、图像/IMU 时间对齐或真实电机控制。

## Before Snapshot

commit hash:   3e6f90f
branch:        feat/detector
modified files: 保留既有 detector/vision_adapter 等未提交改动；本轮新增 control_frame_adapter 与测试，并更新构建依赖及文档
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
- [x] 每个新增模块带独立可执行测试（友军过滤 7 条 · 云台角 5 条 · 整车 EKF 1 条，共 13 条全绿）
