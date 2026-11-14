#ifndef FORAY_AUTO_AIM_SP_ASSOCIATION_HPP
#define FORAY_AUTO_AIM_SP_ASSOCIATION_HPP

#include <optional>

#include "diesel_auto_aim/tracker.hpp"

namespace diesel_auto_aim {

/// 按 SP Target::update 的近三板、板朝向差＋观测方位差规则选择局部板号。
/// 仅负责关联评分；不执行 EKF 更新、置信度判断或目标切换。
struct SpAssociation {
    int armor_id;
    double score_rad;
};

std::optional<SpAssociation> associate_sp_plate(const Target& target,
                                                const ArmorTrackObservation& observation,
                                                const Eigen::Vector3d& origin_world);

} // namespace diesel_auto_aim

#endif
