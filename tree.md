# 目录结构说明

## 目录结构

```
.
├── README.md                本仓定位、接口、上下游
├── AGENTS.md                AI 协作规范
├── plan.md                  开发计划与进度
├── decision.md              关键工程决策日志
├── tree.md                  本文件
├── CODEOWNERS               Review 自动分派
├── package.xml              ROS 2 包定义（ament_cmake）
├── CMakeLists.txt           构建与测试目标
├── .clang-format            组织 C/C++ 风格（CI 的 lint-c 依据）
├── .diesel-layer             所属层与依赖边界（CI 校验）
├── .gitignore
├── include/
│   └── diesel_auto_aim/      对外头文件
│       ├── gimbal_command.hpp     云台角指令结构
│       ├── aimer.hpp              云台角解算接口（直瞄）
│       ├── target_candidate.hpp   候选目标结构 + 友军过滤接口
│       ├── detector.hpp           装甲板检测接口（只留接口 实现待模型）
│       └── target.hpp             整车状态估计 EKF（11 维 含认板与野值门限）
├── src/
│   ├── aimer.cpp            云台角解算实现（直瞄）
│   ├── filter_friendly.cpp  友军过滤实现
│   └── target.cpp           整车状态估计实现（predict/update/认板/NIS）
├── test/
│   ├── test_aimer.cpp             云台角解算用例（5 条）
│   ├── test_filter_friendly.cpp   友军过滤用例（7 条）
│   └── test_target.cpp            整车 EKF 用例（5 条：收敛/速度/换板/认板野值/NIS）
└── .github/
    └── workflows/
        └── ci.yml           复制自组织 CI 模板
```

> 已落地三个模块：① **候选目标结构 + 友军过滤**（7 条测试）② **云台角解算（直瞄）**（5 条测试）③ **整车状态估计 EKF**（5 条测试：静止收敛 · 速度估计 · 换板不跳变 · 认板与野值 · NIS 门限）。
> 弹道解算暂不做；`detector` 只留接口，实现待模型就绪。
> `config/`（参数 YAML）随多源融合与云台参数落地后补记。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
