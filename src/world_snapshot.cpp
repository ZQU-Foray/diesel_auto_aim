#include "diesel_vision_backend/world_snapshot.hpp"

#include <algorithm>

namespace diesel_vision_backend {
namespace {

int missed_frames_of(const TrackerResult& result) {
    // Tracker 没有直接暴露丢帧计数，用状态与诊断推断：TempLost 视为已丢帧。
    // I7 不改 Tracker 内部状态机，只按契约把「是否纯预测」反映到置信度上。
    if (result.state == TrackerState::TempLost || result.predicted_only) {
        return 1;
    }
    return 0;
}

} // namespace

double track_confidence(TrackerState state, int missed_frames, int max_missed_frames) {
    if (state == TrackerState::Lost) {
        return 0.0;
    }
    if (state == TrackerState::Detecting) {
        return 0.5;
    }
    if (state == TrackerState::TempLost) {
        // 暂时丢失：保留但强烈衰减，避免上层把预测当成实测
        return 0.2;
    }
    const int limit = std::max(max_missed_frames, 1);
    const double decay = 1.0 - static_cast<double>(std::max(missed_frames, 0)) / limit;
    return std::clamp(decay, 0.0, 1.0);
}

WorldSnapshotData build_world_snapshot(const TrackerResult& result, double stamp_s,
                                       int max_missed_frames) {
    WorldSnapshotData snapshot;
    snapshot.stamp_s = stamp_s;
    if (!result.target.has_value() || result.state == TrackerState::Lost) {
        return snapshot;
    }
    const TargetState& target = result.target.value();
    SnapshotTrack track;
    track.armor_id = result.associated_armor_id;
    track.xyz_world = target.xyz;
    track.velocity_world = target.vxyz;
    track.confidence = track_confidence(result.state, missed_frames_of(result), max_missed_frames);
    track.state = static_cast<int>(result.state);
    snapshot.tracks.push_back(track);
    snapshot.valid = true;
    return snapshot;
}

} // namespace diesel_vision_backend
