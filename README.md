# diesel_auto_aim

> **L4** · owner 角色：视觉

自瞄（哨兵/步兵/英雄共用）· 多源目标注入

消费融合后的目标，**不关心来源**。跟踪环是硬实时。

---

## 快速开始

> 本仓处于**初始化状态**，尚无源码。以下流程随 P0/P1 落地逐步可用。

```bash
# 1. 拉取（仓齐备后改为从元仓 diesel_ws 一键拉取）
git clone git@github.com:ZQU-Foray/diesel_auto_aim.git
cd diesel_auto_aim

# 2. 依赖安装
rosdep install -y -r --from-paths . --ignore-src --rosdistro humble

# 3. 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. 运行
# （待装配根就绪后补充）
```

## 依赖

| 类型 | 依赖 |
|---|---|
| **自研仓** | X2、L1、L2、L3、L4 |
| **第三方** | OpenCV · Eigen3 · 弹道解算库（自研） |

## 接口

### 统一C++直瞄运行接口

`AimPipeline`是完整前后端的公共调用边界：同帧Vision双模型/PnP、候选手眼外参、主机时刻姿态插值、世界系观测、Tracker/EKF、锁定选板和直瞄在C++中串联。`foray_auto_aim_runtime`直接取海康相机并独占USB，独立C++通讯线程负责接收、心跳、时效检查、角度限速和控制；不再通过Python转换坐标或调用后端worker。启动默认只观察，键盘或调试台显式开始才发送。第一阶段PnP旁路仅为排错对照，完整Tracker模式为主链路。

通过`--handeye FILE`加载用户候选JSON，校验旋转、平移、变换方向及内参路径；不硬编码个人路径。候选文件的测试范围和精度警告原样保留，不能据此宣称生产标定验收。当前仍按主机取帧/姿态接收时刻配对，不伪称曝光—IMU硬件同步。此运行入口不实现弹道与发射，不保证硬实时，尚需现场性能验收。

### 第二阶段直瞄联调与弹道协作边界（2026-11-11）

独立工具 `foray_world_tracker_worker` 接收已转换到稳定世界系的装甲板位置/朝向，串联原生 Tracker、整车 EKF、选板滞回、指定提前量预测和直瞄输出。它不打开硬件、不发送控制；外部调试装配工具负责相机与串口。该实验路径已做短测，但半径估计漂移、候选外参与精确对时未验收，不等于比赛生产链路完成。下文“只读”仍指相机工具本身，旧的静止点动限制不描述外部第二阶段工具。

弹道由另一位队员负责：`world_aim_solver.hpp` 规定测试输入（整车状态、局部板号、观测时刻、预测时长、世界系云台原点及弹速）与输出（世界系绝对yaw/pitch，rad、有效位）。`WorldAimSolver` 为替换接口，当前 worker 仅实例化 `DirectWorldAimSolver`。不实现重力补偿、飞行时间或开火决策；弹速默认0仅表示直瞄未使用，不可当作真实弹速。队友实现可离线调用同一接口测试，再单独评审接入。

| 产出 | 内容 |
|---|---|
| 云台指令 | pitch/yaw（硬实时环，100–1000 Hz） |
| 目标状态 | 供决策层使用的目标可信度 |

当前 `aim_at()` 输出的是云台系相对指向角。RoboLink v0.2 的控制快照要求世界系目标角，因此协议适配函数必须收到有效的云台系到世界系四元数，先旋转指向向量再解出世界系 yaw/pitch（rad）；无有效姿态时拒绝组帧。时间对齐尚未实现，真实控制仍禁用。握手或状态回读前也拒绝排队；排队不证明电控接受或执行。

### 海康相机＋后端＋云台通讯联调（只读）

`--target-feed-json` 在标准输出逐帧导出已有双模型/PnP 的相机系位置、颜色、类别、重投影误差及歧义标志，供外部人工静止点动工具使用。包含空目标帧，禁止延用丢失目标；时间是 Linux 主机单调取帧时间，尚非曝光—IMU 对时。本入口仍不发送控制，外部点动程序独占串口并负责候选外参、静止限制和姿态配对，不能据此宣称移动闭环完成。

安装海康 MVS SDK 的机器会构建 `foray_auto_aim_live_debug`。它在独立窗口显示同帧双模型检测、可选相机系 PnP、可选静止相机 Tracker，以及 RoboLink 握手/保活和下位机上报的 IMU/状态。**此工具没有调用 `queue_control`、`queue_gimbal_command` 或开火接口，绝不发送目标角。** 先关闭 MVS 预览释放相机。

```bash
source /opt/ros/humble/setup.bash
cd /home/user/work/foray_ws
colcon build --packages-up-to foray_auto_aim --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
./install/foray_auto_aim/lib/foray_auto_aim/foray_auto_aim_live_debug \
  --pose-model /path/to/pose.onnx --pattern-model /path/to/pattern.onnx \
  --serial /dev/serial/by-id/你的设备
```

仅当有**当前分辨率对应内参**和**装甲板实际外轮廓实测尺寸**时，再加 `--calibration FILE --armor-width-mm MM --armor-height-mm MM` 打开相机系 PnP。另需实测靶车板中心至车心半径，才能加 `--radius-m M --gate-m M` 打开诊断 Tracker；门限是实验参数，不能沿用模拟器值充当实车验收。Tracker 使用静止相机诊断坐标，串口姿态无效或偏离启动锚点超过 0.5°便暂停，按 `r` 仅在云台已停稳时重新锚定；`q`/Esc 退出。窗口中的 `rx/accepted/applied` 是电控回报的控制序号，不等于本工具发送了控制帧。

该诊断入口尚未解决相机—云台手眼外参、机械两轴运动学、曝光—IMU 对时和目标敌我判定，故不能将 Tracker 结果接入真实云台控制或发射。真实控制须另行完成标定、时间对齐、限幅与状态回读验收。

当前离线 `Tracker` 负责单目标的连续确认、四板位置与朝向联合关联、短暂丢帧与超时重置。其输入位置和装甲板朝向必须已转到同一个稳定坐标系，并使用图像采样时间；它不负责相机/IMU 时间对齐、敌我识别或真实云台控制。仅 `Tracking` 状态可作为后续瞄准的候选，`Detecting`/`TempLost` 不表示已安全锁定。距离/朝向门限、半径和逐维初值方差由调用者注入，实机使用前必须标定和调参。

旋转目标的离线回归测试已覆盖“板绕中心运动不能被当作车体平移”：Tracker 将板位置和朝向联合送入整车 EKF，角度新息按 ±π 折返，朝向观测方差可配置。测试通过只说明合成观测下的估计行为；真实模型角点误差、同帧重复候选、误认板、半径/高度差及实机时序仍需分别验证。

`TrackerResult.diagnostics` 仅供离线诊断，区分空输入、ID、朝向、距离和时间间隔等拒绝原因；不会放宽门限，也不代表目标可安全控制。模拟器同板对照已发现模型四角相对物理外角内缩，需先校正输入契约再标定实车门限。

离线 v1 另提供 `aim_at_tracked_armor()`：用 Tracker 的整车状态、局部板号与显式预测时长计算未来板位置，再按下发时刻云台姿态转成直瞄角。它不做图像/IMU 对时、目标可见性判定、弹道或发射授权；当前仅供模拟器适配层使用，不能直接接真实电机。

**目标源可插拔**（本仓只消费融合后的目标）：

| 来源 | 特性 |
|---|---|
| 自身相机（装甲板识别） | 高精度、低延迟、视野受限 |
| 哨兵共享态势 | 覆盖广、精度低、有延迟 |
| 队友共享 | 同上，来源可能重复 |
| 裁判系统 | 无位置，仅血量/状态 |

## 两条硬约束



### 1. 必须**过滤**友军，不只是标记友军



步兵开火由操作手授权（见 `foray_docs/algorithm_structure.md` §5），因此

**禁止把友军作为瞄准目标**。这是**过滤**——友军不得进入候选目标集，

而不是标记为「友军」后交给下游判断。



### 2. 跟踪环仍是硬实时



人在环允许端到端数百毫秒延迟，**但这不放松跟踪环**。

云台伺服仍在硬实时域（1–10 ms）：去中间件、直连 HAL、独立实时线程、禁止动态分配与锁。



## 注意



- 开火门限的作用点是本仓的**目标准入**——归属 X1 安全门，不归本仓自决

- 哨兵/步兵/英雄共用本仓代码，差异只在装配根与参数

## 上下游

| 方向 | 对象 |
|---|---|
| **上游**（本仓依赖谁） | `diesel_interfaces` · `diesel_platform` · `diesel_localization` · `diesel_vision` |
| **下游**（谁依赖本仓） | `diesel_robots` · `diesel_ws` |

## 参考

- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md)
- [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
