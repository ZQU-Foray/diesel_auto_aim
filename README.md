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

| 产出 | 内容 |
|---|---|
| 云台指令 | pitch/yaw（硬实时环，100–1000 Hz） |
| 目标状态 | 供决策层使用的目标可信度 |

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
