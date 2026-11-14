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
│   └── diesel_vision_backend/     对外头文件
│       ├── gimbal_command.hpp     云台角指令结构
│       ├── control_frame_adapter.hpp  云台相对角与 IMU 姿态到世界系控制帧的离线适配接口
│       ├── aimer.hpp              云台角解算接口（直瞄）
│       ├── target_candidate.hpp   候选目标结构 + 友军过滤接口
│       ├── tracker.hpp            单目标状态机、四板关联门限及离线拒绝/主次板修正诊断接口
│       ├── sp_association.hpp     SP 近三板朝向＋方位评分的独立关联接口
│       ├── tracked_aim.hpp        整车估计到未来板位置与直瞄角的离线接口
│       ├── world_aim_solver.hpp   世界系直瞄与队友弹道的替换测试边界 不负责发射
│       ├── freshness_policy.hpp   时间门限三档(图像/指令/姿态) + 姿态时间偏移 + 拒绝原因
│       │                          (曝光偏移不在此处 由运行入口在取帧处扣减)
│       ├── aim_pipeline.hpp       统一C++后端接口 Observation→手眼/姿态→世界观测→Tracker→选板→直瞄
│       ├── observation.hpp        前端逐帧观测 POD：相机系位置/法向/板号/来源/类别（I8 换 interfaces 消息）
│       ├── fusion.hpp             多源融合：来源优先级、过期门限、同板去重
│       ├── world_snapshot.hpp     WorldSnapshot 构建与轨迹置信度模型
│       ├── native_control_loop.hpp 独立C++ USB线程 握手/反馈/限速/指令时效撤销 默认只观察
│       └── target.hpp             整车状态估计 EKF（11 维）
├── src/
│   ├── aimer.cpp            云台角解算实现（直瞄）
│   ├── control_frame_adapter.cpp  四元数坐标旋转、单位检查与候选协议组帧
│   ├── filter_friendly.cpp  友军过滤实现
│   ├── tracker.cpp          单目标跨帧确认、位置/朝向联合关联与丢帧处理
│   ├── sp_association.cpp   SP 局部板号评分，由 Tracker 可选策略调用
│   ├── tracked_aim.cpp      整车状态外推、云台坐标转换和直瞄角计算
│   ├── world_aim_solver.cpp 世界系直瞄实现 弹道队友另行实现
│   ├── aim_pipeline.cpp     观测校验/手眼与姿态变换/时间门限/Tracker→选板→直瞄 不触相机与检测
│   ├── native_control_loop.cpp 单USB所有者 C++收发和控制时效 不依赖Python
│   └── target.cpp           整车状态估计实现
├── test/
│   ├── test_aimer.cpp             云台角解算用例（6 条）
│   ├── test_control_frame_adapter.cpp 云台角到控制帧映射及伪终端全链路用例（7 条）
│   ├── test_filter_friendly.cpp   友军过滤用例（7 条）
│   ├── test_sp_association.cpp    SP 局部板号评分用例（2 条）
│   ├── test_tracker.cpp           四板关联、朝向异常、丢帧、时间戳与快转初期车心稳定性测试（18 条）
│   ├── test_tracked_aim.cpp       平移/旋转外推、交替板几何、坐标转换与非法输入测试（7 条）
│   ├── test_fusion.cpp            来源优先级、过期丢弃、同板合并（8 条）
│   ├── test_world_snapshot.cpp    快照轨迹与置信度模型（4 条）
│   ├── test_world_aim_solver.cpp  世界角、原点、无效输入与队友替换接口测试（2 条）
│   ├── test_freshness_policy.cpp  时间门限边界(非有限/回退/超龄/插值窗口/姿态偏移)（21 条）
│   ├── test_aim_pipeline.cpp      Observation 插值/完整跟踪直瞄/手眼只转一次/歧义/非法板号与几何（6 条）
│   ├── test_native_control_loop.cpp 无硬件USB端到端 建链/完整后端/超时/恢复/故障/停止/禁发射（1 条）
│   └── test_target.cpp            整车 EKF 用例（5 条）
└── .github/
    └── workflows/
        └── ci.yml           复制自组织 CI 模板
```

> 已落地：候选目标结构+友军过滤、云台角直瞄解算、11 维整车 EKF、跟踪状态机+四板关联、
> SP 局部板号评分、整车外推到板位与直瞄角、控制帧适配（含伪终端全链路）、
> Observation 统一入口与时间门限三档（含姿态/曝光偏移）。
> 装甲板检测、PnP 与相机调试归 `diesel_vision_frontend`（原 `diesel_vision`）；
> 本仓只消费逐帧 `Observation`，不链接 OpenCV / 推理运行时，也不打开相机。
> `config/`（参数 YAML）随多源融合参数落地后补记。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
