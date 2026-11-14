#include "diesel_vision_backend/target_candidate.hpp"

namespace diesel_vision_backend {

std::vector<TargetCandidate> filter_friendly(const std::vector<TargetCandidate>& candidates) {
    // 只保留 side == Side::Enemy 的项，且保持原顺序
    std::vector<TargetCandidate> enemycandidates;
    for (const auto& c : candidates) {
        if (c.side == Side::Enemy) {
            enemycandidates.push_back(c);
        }
    }
    return enemycandidates;
}

} // namespace diesel_vision_backend
