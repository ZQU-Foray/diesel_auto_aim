#include "diesel_vision_backend/fusion.hpp"

#include <algorithm>
#include <cmath>

namespace diesel_vision_backend {
namespace {

bool finite3(const Eigen::Vector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

/// 自车相机优先，其次置信度高者；完全相同则保留先到者（输入顺序即前端顺序）。
bool better(const Observation& candidate, const Observation& current) {
    const bool candidate_own = candidate.source == ObservationSource::OwnCamera;
    const bool current_own = current.source == ObservationSource::OwnCamera;
    if (candidate_own != current_own) {
        return candidate_own;
    }
    return candidate.confidence > current.confidence;
}

} // namespace

FusionResult fuse_observations(const std::vector<Observation>& observations, double now_s,
                               const FusionConfig& config) {
    FusionResult result;
    if (!std::isfinite(now_s) || !(config.max_age_s > 0.0) || !(config.same_armor_merge_m > 0.0)) {
        // 参数不合法时不做任何"尽量处理"：全部丢弃并让计数说明原因，调用方可据此报错
        result.dropped_invalid = static_cast<int>(observations.size());
        return result;
    }

    std::vector<Observation> kept;
    kept.reserve(observations.size());
    for (const Observation& observation : observations) {
        if (!std::isfinite(observation.stamp_s) || observation.stamp_s > now_s ||
            now_s - observation.stamp_s > config.max_age_s) {
            ++result.dropped_stale;
            continue;
        }
        if (config.armor_only && observation.target_class != TargetClass::Armor) {
            ++result.dropped_class;
            continue;
        }
        if (observation.armor_id < 0 || observation.armor_id > 2 ||
            !finite3(observation.xyz_camera) || observation.xyz_camera.z() <= 0.0 ||
            (observation.normal_valid && !finite3(observation.normal_camera))) {
            ++result.dropped_invalid;
            continue;
        }
        kept.push_back(observation);
    }

    std::sort(kept.begin(), kept.end(), [](const Observation& lhs, const Observation& rhs) {
        return lhs.armor_id < rhs.armor_id;
    });

    for (const Observation& observation : kept) {
        bool merged = false;
        for (Observation& existing : result.observations) {
            if (existing.armor_id != observation.armor_id) {
                continue;
            }
            if ((existing.xyz_camera - observation.xyz_camera).norm() > config.same_armor_merge_m) {
                continue;
            }
            if (better(observation, existing)) {
                existing = observation;
            }
            ++result.merged;
            merged = true;
            break;
        }
        if (!merged) {
            result.observations.push_back(observation);
        }
    }
    return result;
}

} // namespace diesel_vision_backend
