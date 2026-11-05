#include "diesel_auto_aim/vision_adapter.hpp"

#include <Eigen/Dense>

namespace diesel_auto_aim {

GimbalCommand adapter(const Observation& observation, const double bullet_speed) {
    if (!observation.pose_success) {
        return {0.0, 0.0, false};
    }
    const Eigen::Vector3d t_armor2gimbal(observation.t_armor2gimbal[0],
                                         observation.t_armor2gimbal[1],
                                         observation.t_armor2gimbal[2]);
    return aim_at(t_armor2gimbal, bullet_speed);
}

} // namespace diesel_auto_aim
