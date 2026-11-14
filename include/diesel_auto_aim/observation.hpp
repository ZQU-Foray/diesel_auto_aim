#ifndef DIESEL_AUTO_AIM_OBSERVATION_HPP
#define DIESEL_AUTO_AIM_OBSERVATION_HPP

#include <Eigen/Core>

namespace diesel_auto_aim {

// 前端检测前端输出的一帧观测（后端内部 POD）。
// 检测类别、图像、PnP 都归 diesel_vision_frontend；后端只消费几何结果。
// I6 把本结构替换为 diesel_interfaces 的 Observation 消息，字段语义不变。
struct Observation {
    Eigen::Vector3d xyz_camera = Eigen::Vector3d::Zero();    ///< 相机系平移，米
    Eigen::Vector3d normal_camera = Eigen::Vector3d::Zero(); ///< 相机系板面法向
    int armor_id = -1;               ///< 局部板号 0..2，超出范围视为非法
    bool normal_valid = false;       ///< PnP 多解或退化时法向不可用，只保留位置
    double reprojection_error = 0.0; ///< 前端重投影误差，像素
};

} // namespace diesel_auto_aim

#endif
