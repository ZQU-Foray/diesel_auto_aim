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
├── .diesel-layer            所属层与依赖边界（CI 校验）
├── .gitignore
├── include/
│   └── diesel_auto_aim/      对外头文件
│       ├── gimbal_command.hpp     云台角指令结构
│       ├── control_frame_adapter.hpp  云台相对角与 IMU 姿态到世界系控制帧的离线适配接口
│       ├── aimer.hpp              云台角解算接口（直瞄）
│       ├── target_candidate.hpp   候选目标结构 + 友军过滤接口
│       ├── tracker.hpp            单目标状态机、四板关联门限及离线拒绝/主次板修正诊断接口
│       ├── sp_association.hpp     SP 近三板朝向＋方位评分的独立关联接口
│       ├── tracked_aim.hpp        整车估计到未来板位置与直瞄角的离线接口
│       ├── vision_adapter.hpp     前端观测（过渡 POD）到直瞄指令的适配接口
│       └── target.hpp             整车状态估计 EKF（11 维）
├── src/
│   ├── aimer.cpp            云台角解算实现（直瞄）
│   ├── control_frame_adapter.cpp  四元数坐标旋转、单位检查与候选协议组帧
│   ├── filter_friendly.cpp  友军过滤实现
│   ├── tracker.cpp          单目标跨帧确认、位置/朝向联合关联与丢帧处理
│   ├── sp_association.cpp   SP 局部板号评分，由 Tracker 可选策略调用
│   ├── tracked_aim.cpp      整车状态外推、云台坐标转换和直瞄角计算
│   ├── vision_adapter.cpp   前端观测到直瞄指令的转换实现
│   └── target.cpp           整车状态估计实现
├── test/
│   ├── test_aimer.cpp             云台角解算用例（5 条）
│   ├── test_control_frame_adapter.cpp 云台角到控制帧映射及伪终端全链路用例（5 条）
│   ├── test_filter_friendly.cpp   友军过滤用例（7 条）
│   ├── test_sp_association.cpp    SP 局部板号评分用例
│   ├── test_tracker.cpp           四板关联、朝向异常、丢帧、时间戳与快转初期车心稳定性测试
│   ├── test_tracked_aim.cpp       平移/旋转外推、交替板几何、坐标转换与非法输入测试
│   └── test_target.cpp            整车 EKF 用例（仿真 静止目标收敛）
└── .github/
    └── workflows/
        └── ci.yml           复制自组织 CI 模板
```

> 已落地：① **候选目标结构 + 友军过滤**（7 条测试）② **云台角解算（直瞄）**（5 条测试）③ **整车状态估计 EKF**（1 条仿真测试）
> ④ **跟踪状态机 + 四板关联** ⑤ **SP 局部板号评分** ⑥ **整车外推到板位与直瞄角** ⑦ **控制帧适配（含伪终端全链路）**。
> 弹道解算暂不做；装甲板检测与 PnP 归 `diesel_vision_frontend`（原 `diesel_vision`），
> 本仓只消费逐帧 `Observation`，不链接 OpenCV / 推理运行时。
> `config/`（参数 YAML）随多源融合参数落地后补记。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
