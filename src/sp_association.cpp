#include "diesel_vision_backend/sp_association.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace diesel_vision_backend {

std::optional<SpAssociation> associate_sp_plate(const Target& target,
                                                const ArmorTrackObservation& observation,
                                                const Eigen::Vector3d& origin_world) {
    if (!observation.xyz.allFinite() || !origin_world.allFinite() ||
        !std::isfinite(observation.yaw))
        return std::nullopt;

    struct Candidate {
        int id;
        Eigen::Vector3d xyz;
        double range;
    };
    std::array<Candidate, 4> candidates{};
    for (int id = 0; id < 4; ++id) {
        const Eigen::Vector3d xyz = target.predicted_armor_xyz(id);
        if (!xyz.allFinite())
            return std::nullopt;
        candidates[static_cast<std::size_t>(id)] = {id, xyz, (xyz - origin_world).norm()};
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.range < b.range; });

    const Eigen::Vector3d observed_ray = observation.xyz - origin_world;
    if (observed_ray.head<2>().norm() < 1e-9)
        return std::nullopt;
    const double observed_bearing = std::atan2(observed_ray.y(), observed_ray.x());
    const double base_yaw = target.state().a;
    SpAssociation best{-1, std::numeric_limits<double>::infinity()};
    // SP 只在距观测原点最近的三块预测板中，比较板朝向差和方位角差。
    for (int i = 0; i < 3; ++i) {
        const Candidate& candidate = candidates[static_cast<std::size_t>(i)];
        const Eigen::Vector3d predicted_ray = candidate.xyz - origin_world;
        if (predicted_ray.head<2>().norm() < 1e-9)
            continue;
        const double predicted_bearing = std::atan2(predicted_ray.y(), predicted_ray.x());
        const double predicted_yaw = base_yaw + candidate.id * M_PI / 2.0;
        const double score =
            std::abs(std::remainder(observation.yaw - predicted_yaw, 2.0 * M_PI)) +
            std::abs(std::remainder(observed_bearing - predicted_bearing, 2.0 * M_PI));
        if (score < best.score_rad)
            best = {candidate.id, score};
    }
    if (best.armor_id < 0)
        return std::nullopt;
    return best;
}

} // namespace diesel_vision_backend
