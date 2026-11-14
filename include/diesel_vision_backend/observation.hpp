#ifndef DIESEL_VISION_BACKEND_OBSERVATION_HPP
#define DIESEL_VISION_BACKEND_OBSERVATION_HPP

#include <cstdint>

#include <Eigen/Core>

namespace diesel_vision_backend {

/// 观测来源。与 diesel_interfaces 的 ObservationSource 数值一致（0 自车、1 友军共享），
/// I8 接入 ROS 包后本枚举删除，直接消费 interfaces 生成的消息。
enum class ObservationSource : std::uint8_t { OwnCamera = 0, AllyShared = 1 };

/// 目标类别。与 diesel_interfaces 的 TargetClass 数值一致。
enum class TargetClass : std::uint8_t {
    Armor = 0,
    EnergyMechanism = 1,
    Outpost = 2,
    Base = 3,
    Unknown = 4
};

/// 前端检测前端输出的一帧观测（后端内部 POD）。
/// 检测类别、图像、PnP 都归 diesel_vision_frontend；后端只消费几何结果。
/// 字段语义与 diesel_interfaces/protocol/vision.yaml 的 Observation 一一对应；
/// 时间在后端内部用单调秒（stamp_s），跨进程时由 interfaces 的纳秒 stamp 适配。
struct Observation {
    double stamp_s = 0.0; ///< 采集时刻（与送入的 frame_stamp 同源，单调秒）
    TargetClass target_class = TargetClass::Armor; ///< 目标类别；非装甲板由融合层按策略过滤
    ObservationSource source = ObservationSource::OwnCamera; ///< 自车相机或友军共享
    Eigen::Vector3d xyz_camera = Eigen::Vector3d::Zero();    ///< 相机系平移，米
    Eigen::Vector3d normal_camera = Eigen::Vector3d::Zero(); ///< 相机系板面法向
    int armor_id = -1;               ///< 局部板号 0..2，超出范围视为非法
    bool normal_valid = false;       ///< PnP 多解或退化时法向不可用，只保留位置
    double reprojection_error = 0.0; ///< 前端重投影误差，像素
    double confidence = 0.0;         ///< 检测置信度 0..1
};

} // namespace diesel_vision_backend

#endif
