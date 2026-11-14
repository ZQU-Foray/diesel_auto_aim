# 关键工程决策日志

## 2026-11-11：完整直瞄运行链路收归C++

新增AimPipeline公共接口：复用VisionFrameResult或直接调用同帧双模型/PnP，再加载候选手眼、主机接收时刻SLERP、转换世界观测、调用既有Tracker/EKF/选板与DirectWorldAimSolver。NativeControlLoop独立1ms轮询USB、约100Hz发送，不再经Python坐标适配或世界系worker。候选JSON方向、旋转正交性和内参路径强校验；候选警告保留，不硬编码个人路径。

foray_auto_aim_runtime与原只读工具共享海康取图实现，但为不同可执行入口：旧live_debug仍只读，新runtime默认只观察，必须人工开始。调试台两组仅切换同一C++流水线的完整模式/单板旁路，Python只处理按钮和显示。弹道接口仍独立，不添加发射或底盘控制。控制保留原准入：新鲜READY姿态、MCU link=3、fault=0、新鲜状态、新鲜图像；过期撤销，人工停止不自动恢复，断链解除使能。

本次不调整既有实验EKF参数；0.17m半径先验及其漂移问题不因迁移而视为解决。世界系原点暂固定于云台原点，移动底盘需另接平移定位。30ms主机时刻插值不是曝光/MCU时钟精确同步。普通线程含锁与动态分配，不宣称硬实时。当前海康相机不在USB/MVS枚举中，实机运动验收保留待办。

## 2026-11-11：隔离第二阶段直瞄与弹道协作接口

独立无硬件world_tracker_worker复用原生Tracker、EKF、选板滞回和未来板预测；位置与朝向必须由外部装配转到相同世界系。WorldAimSolver接收整车状态、局部板号、观测时刻、预测时长、世界系云台原点和预留弹速，输出明确标记为世界系绝对角，避免复用GimbalCommand的相对角语义造成二次旋转。当前只实例化DirectWorldAimSolver，弹道由队友另行实现与测试，不输出发射请求。

20秒实机短测中用户实际移动了靶车，非零速度不能直接判假；估计半径0.039–0.245米仍未验收。简化理想四板、2/4/5/10rad/s各300帧回归稳定不代表实机位姿、时间或关联已正确。外部调试台修复临时状态失效导致永久解除持续跟踪的问题；人工停止仍解除。发送角限速默认30度/秒，可调仅为调试参数，不是加速度观测器、电控前馈或机械能力证明。

## 2026-11-11：复用实机观测供人工静止点动

同日联调补充：相机窗口响应 SIGTERM/SIGINT，退出主循环并按正常析构停止取流/关闭设备，避免重开时因直接终止进程遗留相机独占连接。视觉观测导出本身仍不发送控制。

只读实机入口可用 `--target-feed-json` 导出逐帧已有双模型/PnP 观测及主机单调取帧时间（含空目标）。外部工具组合候选手眼与静止姿态，用于人工确认的单步功能验证；相机入口本身不发送控制。禁止复用静止相机诊断 Tracker 作为运动世界系估计，也不把主机取帧/接收时间近邻配对称为曝光—IMU 对时。当前候选外参和实验性点动不代表精确自瞄验收。

> 记录 fork patch、换版本、**破坏性接口变更**、架构取舍。
> 格式见 `foray_docs/repository_structure.md` §7.2。

---

## Decision

Date: 2026-11-07

Context: 实车已能取海康相机画面和读 RoboLink 姿态/状态，但云台控制实际执行记录与手眼外参、曝光—IMU 对时仍未验收。

Decision: 新增可选 MVS 联调可执行程序，组合 Vision 双模型、相机系 PnP、静止相机诊断坐标下的单目标 Tracker 与 RoboLink 只读反馈。串口线程仅调用 HostLink::step 以握手/保活/接收，不排控制帧；无实测尺寸不启用 PnP，无实测车半径/关联门不启用 Tracker；姿态无效或相机旋转后暂停诊断跟踪。

Reason: 先用同一窗口定位前端、位姿、关联和通讯各环节，避免把静止相机诊断轨迹误当成已完成世界系对时和手眼标定的自动控制目标。真正控制仍需独立验收。

---

## Decision

Date: 2026-11-05

Context: 同帧双模型、同一 SP PnP 观测的原地旋转测试中，Foray 300 帧无关联拒绝，却有 13 次车心单帧位移超过 0.20 m；需要区分门限过宽、主板更新和同帧第二块板更新。

Decision: 仅给 `TrackerResult.diagnostics` 增加主板位置残差、主板更新造成的车心修正及次板更新造成的车心总修正。暂不改变任何关联门限、EKF 方差或控制输出。

Reason: 先把跳动定位到确切更新步骤，再对单个参数做 A/B；SP `Aimer` 的前馈不参与本轮跟踪层对照。

---

## Decision

Date: 2026-11-05

Context: SP `Target::update` 先从四块预测板中取距离观察原点最近的三块，再以板朝向差和视线方位角差之和选局部板号；Foray 当前则先保上一板并通过位置/朝向门。

Decision: 移植为独立 `associate_sp_plate` 纯函数，显式接收观察原点，并用合成四板验证；再通过 `TrackerConfig::association_strategy` 可选接入运行 Tracker，默认保留 Foray 距离门。此选板规则不等于已复现 SP 的 EKF 更新、状态机或 Aimer。

Reason: 保留可对照的两套关联规则；后续将同帧观测分别送入两种策略，先比较板号，再比较状态更新，不一次改多个变量。

---

## Decision

Date: 2026-11-03

Context: Tracker 的 `robot_id` 原本承接具体机器人号，与新的模型类别值冲突。

Decision: 当前 Tracker 输入暂沿用字段名 `robot_id`，但语义统一为目标类别 `0=哨兵、1=步兵、2=重装`；`-1` 与其他值不得初始化或更新轨迹。具体车辆身份和板号由时序关联与 `associated_armor_id` 分别表达。2026-11-04 根据最新约定对调哨兵/步兵的 0/1 顺序。

Reason: 同类多车不能靠类别 ID 区分；本次先修正输入契约，不声称已完成多车轨迹管理。

---

## Decision

Date: 2026-11-03

Context: 4 rad/s 模拟器中模型 PnP 有效但 Tracker 经常丢失；需区分无观测、类别不符、朝向门、距离门和时间跳变，避免直接放宽门限。

Decision: 给 TrackerResult 增加只读的离线关联诊断字段，拒绝时额外统计候选未通过哪道门，不修改原有关联决策、EKF 或控制输出。模拟器适配层把诊断与同帧理想/模型四角一起写入 JSONL。

Reason: 100 帧复核中 99 帧有模型 PnP，但 59 帧被距离门拒绝；逐板配对见 `/home/user/Foray_project/output/paired_pnp_ekf_spin4_500_20261103/diagnostic_100_复核.md`。诊断字段不应进入实时控制安全判断；模型四角契约仍需单独修正。

---

## Decision

Date: 2026-11-02

Context: 模拟器适配层已有整车 Tracker，但锁定后仍直接把当帧原始 PnP 板位置送入直瞄；估计状态和预测时长没有进入控制目标，无法比较预测对动态滞后的影响。

Decision: 增加独立的 `tracked_aim` 离线纯计算接口，按 11 维整车状态外推指定局部板位置，再以显式云台姿态转换为相对直瞄角。模拟器在 Tracking 且本帧有关联板时改用此结果；`--lead-ms` 默认 0，仅允许 0–1000 ms 的仿真实验，始终不发送发射请求。

Reason: 先让已有整车估计真正进入仿真控制输出，并保留单元测试与 A/B 比较入口。此实现不自动选择未来可见板、不测量真实总延迟、不处理相机/IMU 对时；不准直接用于真实电机。

---

## Decision

Date: 2026-10-27

Context: 理想角点的模拟器原地 4 rad/s 自转日志中，单板反推车心接近真值，但跟踪估计车心仍偏移；适配器初始角速度为 0，且 `P0[w]=0.01`，关联只看位置。

Decision: 保持初始角速度均值为 0，在模拟器适配器里试用 `P0[w]=25 (rad/s)^2`（约 0±5 rad/s 的离线先验）；Tracker 的四板关联增加可配置的朝向差门限，位置与朝向必须同时匹配，角度差用 ±π 折返。朝向门限初始取 1.2 rad，作为离线诊断值而非实车常量。误判朝向和快转起步的合成测试已通过。

Reason: 增大方差表示“不确定是否在快转”，并不强迫车旋转；朝向门控防止空间上接近但板号不符的观测拉偏整车 EKF。此改动未加入 NIS，也未证明完整模拟器旋转、更不证明真实模型/相机/云台闭环通过；后续需对比原始日志的车心误差、关联拒绝率和换板时机，再由实测噪声设置门限。

---

## Decision

Date: 2026-10-26

Context: 原地旋转靶的白色四板预测会随可见装甲板平移。旧 `Target::update()` 仅使用板中心 xyz，角过程噪声近乎为零；回归测试中车体真值静止，估计中心最大偏离约 0.184 m。

Decision: `Tracker` 在已关联板上同时使用位置与朝向观测；朝向新息 wrap 到 ±π，朝向方差由配置给出。角过程噪声的离线起始值从近乎冻结的 `1e-6` 调为 `1.0`，允许角速度随旋转变化。保留原有纯位置 `Target::update()` 接口供既有测试与调用者使用。

Reason: 单板位置随旋转而变，仅靠 xyz 难以区分车体平移与板绕中心运动。离线回归现在通过；这不是实测调参，更不证明现有 ONNX 检测或 PnP 朝向可靠。实际模型角点、同帧重复候选、半径和高度差仍可能导致白框失位，后续要逐帧对照，不以单项修复宣称闭环完成。

---

## Decision

Date: 2026-10-25

Context: 现有 `Target` 只实现整车 EKF，模拟器按单帧置信度选板，缺少跨帧关联与丢帧状态；直接把云台系观测送入 EKF 会把云台转动误认为目标运动。

Decision: 新增独立的单目标 `Tracker`，借鉴 SP 的 Lost/Detecting/Tracking/TempLost 状态转换，但用显式配置代替 YAML、只做四板最近邻距离门控。输入必须为稳定坐标系的板位置/朝向及采样时间。此模块只输出状态与几何预测，不触发控制；坐标转换、敌我过滤、时间对齐由上游/装配层负责。

Reason: 先用离线数据验证跨帧逻辑，避免没有真实标定和同步数据时假装完成闭环。距离门控是第一道保护，尚不等于 NIS 统计检验或实机准入。

Update (2026-10-25): 在两块装甲板同帧可见时，先尝试上一帧关联的板；只要该板仍在距离门限内，就不因另一块板的瞬时分数或几何残差更优而切换。只有旧板不再匹配时才搜索四板。新增输入观测下标，供调试适配器将关联结果映射回原检测框。离线双板切换测试通过，真实模型输出尚待验证。

---

## Decision

Date: 2026-10-21

Context: 需要将自瞄的弧度制 `GimbalCommand` 与平台仓候选 USB 控制协议连接，但电控尚未完成协议确认和真实联调。

Decision: 在自瞄仓增加组帧与排队适配函数，依赖下层 `foray_platform`；仅在指令可信且有限时生成控制帧，只声明云台位，不声明底盘或发射使能。角度转换按组长确认的线上 deg；候选协议文档仍写 rad，实机联调时需核对电控实际单位。平台层负责连接状态，装配层负责调用时序与实际设备接入。

Protocol audit: 2026-10-22 将实际排队入口的帧序号、消息序号和控制序号交由平台主机链路统一分配；离线组帧函数仍可用于测试，不应由运行层独立生成序号。

Angle audit: 2026-10-22 复核 `vision_learn/笔记/01_RM战队/069_RoboLink协议要点_v0.1.md` 的组长答复：“发下角度就行，不需要处理”，累计角和跨圈由下位机负责，线上角度 deg。撤回此前擅自增加的上位机角反馈前置条件；状态回读仍用于联调观察是否接受和应用。候选协议文字与此口径存在不一致，实机测试应检验电控实现。

Update (2026-10-24): 最新对接口径把云台目标角线上单位改为 rad，覆盖上述 deg 历史结论。适配层不再乘 `180/π`，只检查可信度、有限性和 f32 范围后原样传递 rad；下位机四元数姿态反馈的线上布局尚待电控确认。

Update (2026-10-24): `foray_interfaces` 已合入 RoboLink v0.2 候选契约，规定线上目标角是世界系而非云台系。保留 `aim_at()` 的云台系相对角职责；控制适配层要求显式传入有效且 READY 的云台系到世界系四元数，将相对指向向量旋转后重算世界系 yaw/pitch。旧的无姿态组帧入口改为编译期不匹配，防止相对角被直接发送。图像—IMU 时间对齐尚未实现，此处只做离线坐标系验证，不能启用真实云台控制。

Reason: 先用无硬件测试固定数据边界，同时避免未完成的握手与电控裁决被误当作安全闭环。

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
- 目录约定：`include/diesel_vision_backend/`（对外头文件）· `src/`（实现）· `config/`（参数 YAML）· `test/`（每模块一个测试）
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
Date: 2026-11-13

Context: 图像/姿态/指令三条时间链的时效门限，以及姿态时间偏移的取值。

Decision: 拆成三档分离配置，集中在 `freshness_policy.hpp`：
图像年龄 **60 ms**、指令年龄 **55 ms**、姿态插值窗口 **30 ms**。姿态时间偏移默认 **0**，
由运行入口注入（离线三轮回放实测最优 **+8 ms**）。

Reason:
- 图像年龄按实测 frame_age 定（30fps 链路 P50 36.9 / P90 40.6 ms），收紧只会丢帧。
- ⭐ 指令年龄必须**从命令生成时刻**算，不是从取帧时刻：取帧到提交之间还有约 36 ms 推理耗时，
  用取帧时刻会把推理时间算进指令年龄，导致帧间出现撤销-重发抖动（`active` 闪断、一帧都发不出去）。
  且必须大于视觉帧间隔（33 ms），否则每帧之间必有空洞。
- 姿态偏移是**每次会话实测的量**，不是硬件常数（相机 PTP 未启用，实测时钟速率差 +39 ppm、抖动 σ=239 µs），
  所以库默认 0、由入口显式给、面板打印实际值。

Source: 实机 60 s 闭环：指令门限 20 ms→55 ms 后 `command_interruptions` 113→24；
姿态偏移一致性 73/73 条满足 `attitude_stamp = 报告时刻 − 图像年龄 − 8 ms`。

Rejected: 单一门限（三处各写死 0.25 s，改一处会飘）；把偏移写进库默认（连坐既有测试）。

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
Date: 2026-11-13

Context: 把"曝光时刻"接入时间链的方式。

Decision: 新增运行参数 `--exposure-offset-ms`（默认 **0**，范围 0~100 ms），
在**取帧处**把 `frame_stamp` 减去该偏移，使下游的 `frame_stamp` **定义即为曝光时刻**。
下游新鲜度门限与姿态插值不再各自扣减。

Reason: 若让「图像年龄」「姿态插值」「指令年龄」三处各自减去曝光延迟，会出现三份重复且易漂移的偏移量；
在唯一入口扣一次，语义最清晰，也不会漏改。

⚠️ 当前值 **未标定**：屏幕闪烁法能拍到亮度方波，但相关系数只有 0.6，且测得的滞后（+43.75 ms，正数=未来）
物理上只能解释为**屏幕自身的显示延迟**（Tk 配置到实际像素变化，X11 合成下约十余至数十毫秒），
无法与"曝光→到达"延迟分离。故保持默认 0，并在面板输出实际值以便核对。

Source: 两次采集（周期方波 / PRBS 随机方波）互相关均未得到物理上合理的照明延迟；
最亮帧检查显示相机未正对屏幕，数据不可用于标定。

Rejected: 直接把 +43.75 ms 写进默认值（会把屏幕延迟误当曝光延迟）。
---

## Decision

Date: 2026-11-14

Context: I2 —— 把 `feat/sp-tracker-auto-aim` 收敛到 dev 的 diesel 命名之上，并与「前端检测 / 后端融合跟踪」的分家边界对齐。

Decision:

- 后端只消费前端逐帧 `Observation`（`xyz_camera` / `normal_camera` / `armor_id` / `normal_valid` /
  `reprojection_error`）；`AimPipeline::process_observations` 不再接收图像或检测器。I6 换成
  `diesel_interfaces` 消息后字段语义不变。
- 删除 `vision_adapter`（被 `observation` 取代）、`camera_diagnostic_adapter`（静止相机诊断属前端）与
  `tools/`（相机调试、只读联调窗口归前端）；原 `world_tracker_worker` 的 JSONL 入口待 I7 按新接口重建。
- 去掉 OpenCV 依赖：`load_handeye` 的 `cv::FileStorage` 读取移出库层（标定文件由装配入口读，库只收
  `HandEyeCalibration`）；板法向由前端给出，后端不再做 `cv::Rodrigues`。
- `Target` 只保留 11 维 EKF 与 `update(xyz_measured, armor_id)` / `update(xyz, armor_id, yaw, var, noise_scale)`；
  原 `Target::associate` 与 NIS 野值门限由 Tracker 关联层承担（`test/test_tracker.cpp` 已覆盖野值拒绝）。

Reason: 检测/PnP 与跟踪在变更率、依赖重量、实时要求上都不同（OpenCV + ONNX Runtime 对比 Eigen-only 硬实时环）；
同库会让后端被迫链接推理运行时，也让 CI 变重、构建环境要求变高。

Source: rebase 后本仓 11 个测试程序共 82 条全绿（本地 cmake + 直接运行测试二进制），
`control_frame_adapter` 改指 `diesel_platform_core` 的 `diesel::platform::link`。

Rejected: 用 `#ifdef` 把视觉入口留在后端（依赖与构建仍然连坐）；把 `load_handeye` 留在库内（库层读文件、依赖 OpenCV）。

---

## Decision

Date: 2026-11-14

Context: 原 `diesel_vision` 已实现装甲板双模型检测 + PnP，而本仓 `detector.hpp` 只有空接口；
两者职责重叠，且需要装载更多类别的目标识别。

Decision: 前后端分家。检测识别（相机取流、多类别 detector、PnP 几何）归 L3 前端
`diesel_vision_frontend`（原 `diesel_vision`，持有 OpenCV / ONNXRuntime）；本仓改名为
`diesel_vision_backend`，承接多源融合（OwnCamera / AllyShared）+ 友军过滤 + 关联 / EKF 跟踪 +
瞄准解算 + `WorldSnapshot` 生成，只链 Eigen。`Observation` / `WorldSnapshot` / `GimbalCommand`
消息契约放 `diesel_interfaces`，消费方只依赖消息。

Reason:
- C4 依赖重量：后端只链 Eigen，跟踪环保持 1–10 ms 硬实时，不被模型推理抖动污染
- C3 团队边界：前端（高频、重依赖）与后端（硬实时、轻依赖）可独立演进
- 避免 L5/L4 反向依赖 L3 实现：契约放 X2，上层只依赖消息

Rejected:
- 让 backend 继续实现 detector（与前端重复，且 OpenCV/ONNX 进实时环）
- 融合留在 L3 vision（决策/导航将反向依赖 L3 实现）

Source: `foray_docs/repository_structure.md` §2、§6.6；`algorithm_structure.md` §1.4/§1.5；
迁移计划 P0–P6（dev_task `vision-front-backend-split`）。

---

## Decision

Date: 2026-11-14

Context: I4 —— 仓名与包名从 `diesel_auto_aim` 改为 `diesel_vision_backend`，与「前端检测 / 后端融合跟踪」的分家边界对齐。

Decision:
- GitHub 仓库原地改名 `diesel_auto_aim` → `diesel_vision_backend`（平台自动重定向旧 URL），本地目录与 `origin` 同步改名。
- ROS 包名、CMake `project()`、`ament_export_dependencies`、`include/` 目录与 `#include` 路径、`namespace`、头文件 guard（`DIESEL_VISION_BACKEND_*`）一并改为新名。
- 目录约定改为 `include/diesel_vision_backend/`；`README.md` 定位改为「视觉后端：多源融合 · 跟踪 · 瞄准解算」，依赖表上游写 `diesel_vision_frontend`。
- PR #8 先合入 `dev`，再执行改名；改名后本仓 `dev` 上的 2026-11-14 边界决策记录随改名提交一起推送。

Reason: 仓名要和职责一致——本仓已不含任何检测代码，继续叫 auto_aim 会让上游文档、`.repos` 与新人理解都指向错误边界；原地改名保留提交历史、PR 与权限，和上次 `foray_*` → `diesel_*` 的做法一致。

Source: 迁移计划 I4（dev_task `vision-front-backend-split`）；PR #8（merge commit `96d794e`）。

Rejected: 新建 `diesel_vision_backend` 仓并搬代码（丢历史、丢 PR、要重配 CI 与权限）；只改 README 不改包名（`find_package` 与 `include` 路径仍指旧名）。

## Decision / Date: 2026-11-14 融合放在几何变换之前，快照不暴露未确认轨迹

**Context**: 前端与队友共享会给出同一块板的重复观测，且来源精度不同（自车高精度、队友低精度有延迟）。
后端此前把所有观测一律送入 Tracker，重复观测会被当成两次独立观测累计信息，等于虚增置信度。
同时决策层需要一个「世界模型快照」，但后端只有单目标 Tracker。

**Decision**:
1. 融合在**几何变换之前**、以相机系坐标做：同板号 + 距离 <0.15 m 合并，自车优先，**不平均位置**；
2. 过期门限 100 ms（三个前端帧周期），未来戳直接丢弃；
3. `build_world_snapshot()` 只输出已确认轨迹：`Lost` → `valid=false`，`Detecting` 置信度上限 0.5，
   `Tracking` 随丢帧衰减，`TempLost` 固定 0.2；
4. 未打戳观测按本帧戳处理，兼容旧调用方；显式打戳的观测仍按自身时间判过期。

**Reason**: 在相机系合并可以避免两车不同的手眼标定误差被混进同一位置；不平均位置是为了让「自车优先」
真正生效，而不是把队友的偏差摊薄到自车观测上。快照不给未确认轨迹，是为了让决策层不可能把
Detecting 当成可开火目标——宁可少一条轨迹，也不给一个假锁定。

**Rejected**: 在世界系合并（需先把队友观测转到本车世界系，引入标定误差叠加）；
按置信度加权平均位置（观测不是独立同分布，平均会产生「看起来更准」的假象）；
让快照输出 `Detecting` 轨迹并附状态位（消费方很难不误用）。

**Source**: `AimPipelineResult`、`TrackerResult`、`diesel_interfaces/protocol/vision.yaml` 的
`TrackedTarget` / `WorldSnapshot` 字段语义。
