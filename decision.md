# 关键工程决策日志

> 记录 fork patch、换版本、**破坏性接口变更**、架构取舍。
> 格式见 `foray_docs/repository_structure.md` §7.2。

---

## Decision

Date: 2026-02-19

Context: 本仓从 0 初始化。需要确定它在整体架构中的位置与依赖边界。

Decision: 本仓定位为 **L4** 层，owner 角色为「视觉」，
依赖层级 X2、L1、L2、L3、L4。

Reason:
- 切分判据 C1–C6 见 `foray_docs/repository_structure.md` §1
- 粒度结论：**一个仓 = 一个团队角色 + 一个变更率档位**
- 消费融合后的目标，**不关心来源**。跟踪环是硬实时

Alternatives:
- 并入相邻仓（减少仓库数量，但违反团队边界或变更率差异）

Rejected:
- 按 ROS 2 package 粒度切仓 —— 会导致版本矩阵地狱
- 按分层机械切仓 —— 会切断团队边界（Conway）

---

## Decision

Date: 2026-09-28

Context: 需要让 `colcon` 与组织 CI 识别本仓，并确定源码、参数与测试的目录约定。

Decision:
- 构建：**ament_cmake**，C++17
- 目录约定：`include/diesel_auto_aim/`（对外头文件）· `src/`（实现）· `config/`（参数 YAML）· `test/`（每模块一个测试）
- C/C++ 风格沿用 `diesel_interfaces` 的 `.clang-format`（4 空格 · LF · UTF-8）

Reason:
- 组织 `build-ros2` 作业以 `package.xml` 是否存在为开关；缺该文件时构建与测试会被整段跳过
- 兵种差异落在 `config/`，不进算法代码（`foray_docs/algorithm_structure.md` §1.2）
- 每模块带独立可执行测试是 `repository_structure.md` §9 验收标准 1 的落地方式

Alternatives:
- 沿用 `diesel_interfaces` 的独立可执行测试 + 自定义 CI 作业（不建 ROS 2 包）
- 单一 `src/` 平铺，不区分 `include/` 与 `src/`

Rejected:
- 不建 `package.xml`、仅靠自定义脚本编译 —— 组织 CI 与 rosdep 均无法介入
- 自建一套代码风格 —— 与组织 §6.6 及其余仓分叉
---

## Decision

Date: 2026-09-29

Context: 实现「友军过滤」——位于候选目标集之前的硬约束（友军不得进入候选集）。

Decision:
- 敌我用**三态枚举** `Side{Enemy, Friendly, Unknown}` 表达，不用 `bool`
- 输入输出共用 `TargetCandidate`；过滤器是**纯函数**，返回新的子集（不原地删除）
- **不承担去重职责**：重复目标交由多源融合处理

Reason:
- IFF 取保守取向（宁可漏报，不可误报）：`Unknown` 必须可表达，否则会被默认当作敌人
- 纯函数输入输出明确，可用"造输入 → 断言输出"独立测试（`repository_structure.md` §9 验收标准 1）
- 过滤与去重职责分离，避免单函数承担两件事而难以测试与演进

Alternatives:
- 复用上游 `Armor` 结构：其字段（像素角点、灯条）对共享来源的目标无意义，会长期为空
- 原地 `remove_if`：与参考实现一致，但不利于对输出做断言

Rejected:
- 用 `bool is_ally`：无法表达"未知"，违背 IFF 保守取向
- 在本模块内顺带去重：职责重叠，多源融合会重复实现

---

## Decision

Date: 2026-09-30

Context: 需要由目标位置解算云台指向角（闭环链路中「目标 → 云台角」这一环）。

Decision:
- 输入取 `Eigen::Vector3d`（云台系：x 前 / y 左 / z 上，单位米），输出 `GimbalCommand{yaw, pitch}`
- 只做**直瞄**：`yaw = atan2(y, x)`、`pitch = -atan2(z, sqrt(x²+y²))`，**不含弹道补偿**
- **不在本模块输出 `fire`**：开火决策归属 X1 安全门

Reason:
- 角度符号与坐标系约定绑定：战队约定 yaw 向左为正、pitch 向下为正，而输入 z 轴朝上 → pitch 取负
- 弹道补偿（重力、飞行时间）需要弹速与弹道模型，属后续增量；先让闭环能转起来
- 单一职责：算角与决定开火分开，4 条用例钉住符号

Alternatives:
- 复用参考实现的世界系输出 → 还需一次旋转，且引入上游依赖
- 输入用三个裸 double → 顺序易错、语义不清

Rejected:
- 在本模块内做弹道补偿 → 现缺弹速与弹道模型，会拖住整个闭环
- 输出里带 `fire` → 与安全门职责重叠


---

## Decision

Date: 2026-10-01

Context: 弹道解算什么时候做

Decision: 暂不做 只留接口 aim_at 的 bullet_speed 参数 与 GimbalCommand.valid 当前实现为直瞄

Reason: 目前用激光头测自瞄算法 不需要弹道 后续做成单独模块

Rejected: 保留补偿实现但默认关闭 未经验证的代码路径

---

## Decision

Date: 2026-10-03

Context: 目标状态估计器的建模选择。

Decision: 采用 **11 维整车 EKF**（`x vx y vy z vz a w r l h`），对齐参考实现 sp。

Reason: 靶车有 4 块装甲板，只估单块板会在换板时位置跳变，必须估**车体中心 + 朝向**；
11 维可直接复用 sp 的建模与观测函数。靶车不转，`w/l/h` 保持 0。

Rejected: 1 维卡尔曼（同济 LockTarget 的 `Kalman1D`）—— 简单，但对换板与几何无建模；
6 维（位置+速度）—— 无法处理 4 块板轮流被看到。

---

## Decision

Date: 2026-10-03

Context: 观测量取 `xyz` 还是 `ypda`。

Decision: 先用装甲板 **`xyz`（3 维）**，`H` 为 3×11。

Reason: 简单，不需要 `xyz2ypd` 与它的雅可比。
Cost: 观测噪声无法按方向分别建模（横向误差与测距误差特性不同），实测后再升级为 `ypda`。

---

## Decision

Date: 2026-10-03

Context: 初值协方差与过程噪声的取值。

Decision: 初值协方差**逐维给**；`r/l/h` 视为已知（给 `1e-6`）；过程噪声 `v1` 从 sp 默认 100 调到 **1**。

Reason: 对照实验 —— 全给 `1.0` 时"朝向"与"半径"也在被估，观测偏差被归因于"车转了"，
估计 RMSE **0.096 m**（观测 0.017 m）；逐维给 + `v1 = 1` 后估计 RMSE **0.006 m**，优于观测。

Source: 静止靶车不机动，故 `v1` 远小于 sp 的步兵默认值。


---

## Decision

Date: 2026-10-05

Context: 板号（armor_id）从哪来。

Decision: 用**最近邻数据关联**自己认，不依赖外部传入。由状态预测 4 块板位置，与观测比距离取最近；
距离超过门限 `0.5 m` 判为野值（返回 -1）。

Reason: 观测模型 `h_armor_xyz` 与雅可比 `h_jacobian` 里的 `angle = a + id*90°` 依赖板号，
必须先知道是哪块板才能算残差与 `H`；外部传 id 会把关联逻辑推给调用方。

Source: 门限 0.5 m —— 相邻板间距 `r*sqrt(2) ≈ 0.28 m`，理论门限应小于半间距（0.14 m），
但滤波未收敛时预测误差可达数十厘米，故放宽到 0.5 m 只挡明显野值。**待实测调整**。

Rejected: 外部传 `armor_id`（调用方需自己维护关联）；纯角度误差选板（sp 在 target 层的做法，本质也是最近邻变体）。

---

## Decision

Date: 2026-10-05

Context: 观测野值（误检 / 别的车）怎么挡。

Decision: `update()` 里加 **NIS 卡方门限**：`S = H P Hᵀ + R`，`nis = residualᵀ S⁻¹ residual`，
`nis > 门限` 则丢弃该帧（不修正状态）。门限取 **χ²(dof=3, 95%) = 7.815**。

Reason: NIS 是残差的**马氏距离平方** —— 按新息协方差把残差按各方向不确定度归一化；
相比欧氏距离，它不会因某方向本来就不准而误判，也不会漏掉本该很准的方向的偏差。
收敛前 `P` 大 → 门限自动放宽；收敛后自动收紧，不用手调固定米数。

Source: 卡方分布临界值表（dof = 观测维数 3，95% 分位 = 7.815）。
⚠️ 参考实现 sp 用 `0.711`，对 dof=3/4 明显偏小，**不采用**。

实测（2026-10-05）：正常观测 nis `1e-6 ~ 1.4`；偏 5 m 的野值 nis `2.2e5` → 门限余量约 2.8 万倍。
