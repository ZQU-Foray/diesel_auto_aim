#ifndef DIESEL_VISION_BACKEND_WORLD_SNAPSHOT_HPP
#define DIESEL_VISION_BACKEND_WORLD_SNAPSHOT_HPP

#include <vector>

#include "diesel_vision_backend/tracker.hpp"

namespace diesel_vision_backend {

/// 快照里的单条轨迹。字段与 diesel_interfaces/protocol/vision.yaml 的 TrackedTarget 对应；
/// 世界系位置/速度是后端已经做过手眼与姿态变换后的结果。
struct SnapshotTrack {
    int armor_id = -1; ///< 当前瞄准的装甲板；-1 表示未选板
    Eigen::Vector3d xyz_world = Eigen::Vector3d::Zero();      ///< 世界系位置，米
    Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero(); ///< 世界系速度，米每秒
    double confidence = 0.0;                                  ///< 轨迹置信度 0..1
    int state = 0; ///< 0 Lost / 1 Detecting / 2 Tracking / 3 TempLost，与 TrackState 数值一致
};

/// 世界模型快照。决策层与自瞄共用同一份，避免两处各猜一套目标状态。
struct WorldSnapshotData {
    double stamp_s = 0.0; ///< 快照生成时刻（单调秒）
    bool valid = false;   ///< false 时 tracks 为空，消费方必须按「无目标」处理
    std::vector<SnapshotTrack> tracks;
};

/// 置信度模型：状态越确定、越少丢帧、越新，置信度越高。
///
/// - Lost / TempLost 直接返回 0：预测帧不能当成可开火目标；
/// - Detecting 上限 0.5：还没有足够帧数确认，不允许上层据此开火；
/// - Tracking 上限 1.0，由丢帧数线性衰减（missed_frames 达到 max_missed_frames 时归零）。
double track_confidence(TrackerState state, int missed_frames, int max_missed_frames);

/// 由跟踪结果构建快照。当前 Tracker 是单目标跟踪器，因此 tracks 至多 1 条；
/// 多目标快照要等每目标一条轨迹后才有意义，届时这里扩展，消费方契约不变。
WorldSnapshotData build_world_snapshot(const TrackerResult& result, double stamp_s,
                                       int max_missed_frames);

} // namespace diesel_vision_backend

#endif
