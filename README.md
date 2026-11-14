# diesel_auto_aim

> **L4** · owner 角色：视觉

自瞄（哨兵/步兵/英雄共用）· 多源目标注入

消费融合后的目标，**不关心来源**。跟踪环是硬实时。

---

## 快速开始

> 后端库 + 无硬件测试已可构建；控制链路只在台架验证过，未接整机。

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
| **自研仓** | X2、L1、L2、L3（`diesel_vision_frontend` 提供 `Observation`）、L4 |
| **第三方** | Eigen3 · 弹道解算库（自研，队友实现） |

本仓不链接 OpenCV / ONNX Runtime：检测与 PnP 在前端完成。

## 接口

### 统一C++后端接口

`AimPipeline` 是后端唯一入口：接收前端逐帧 `Observation`（相机系位置、板法向、局部板号、重投影误差），
在本仓完成手眼与姿态变换、时间门限、Tracker/EKF、锁定选板、预测和直瞄。
它不打开相机、不运行检测、不链接 OpenCV 或推理运行时，也不实现弹道与发射；弹道接口仍是 `world_aim_solver.hpp`。

`NativeControlLoop` 独占 USB，负责握手、心跳、时效检查、角度限速和控制撤销；启动默认只观察，
显式 arm 后才发送。当前仍按主机取帧/姿态接收时刻配对，不伪称曝光—IMU 硬件同步，也不保证硬实时验收。

| 产出 | 内容 |
|---|---|
| 云台指令 | pitch/yaw（硬实时环，100–1000 Hz） |
| 目标状态 | 供决策层使用的目标可信度 |

当前 `aim_at()` 输出的是云台系相对指向角。RoboLink v0.2 的控制快照要求世界系目标角，因此协议适配函数必须收到有效的云台系到世界系四元数，先旋转指向向量再解出世界系 yaw/pitch（rad）；无有效姿态时拒绝组帧。时间对齐尚未实现，真实控制仍禁用。握手或状态回读前也拒绝排队；排队不证明电控接受或执行。

### 相机调试入口已迁出

海康相机取流、双模型检测、PnP 与静止相机诊断窗口归 `diesel_vision_frontend`；
本仓不再提供 `foray_auto_aim_live_debug` / `foray_world_tracker_worker` 等工具。
后端离线调试请用单元测试或由装配层喂入 `Observation` 回放。

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
