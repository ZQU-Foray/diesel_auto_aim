#include "diesel_vision_backend/world_aim_solver.hpp"

#include <cmath>

#include "diesel_vision_backend/aimer.hpp"
#include "diesel_vision_backend/tracked_aim.hpp"

namespace diesel_vision_backend {

WorldAimResult DirectWorldAimSolver::solve(const WorldAimRequest& request) const {
    if (!std::isfinite(request.observation_stamp_s) || request.observation_stamp_s < 0.0 ||
        !request.gimbal_origin_world.allFinite()) {
        return {};
    }
    const Eigen::Vector3d position =
        predict_tracked_armor(request.target, request.armor_id, request.prediction_s);
    if (!position.allFinite()) {
        return {};
    }
    // 这里方向仍在世界系 复用直瞄公式 得到的就是世界系角度
    // 传0明确不使用弹速 不把队友的弹道逻辑塞进跟踪器
    const auto command = aim_at(position - request.gimbal_origin_world, 0.0);
    return {command.yaw, command.pitch, command.valid};
}

} // namespace diesel_vision_backend
