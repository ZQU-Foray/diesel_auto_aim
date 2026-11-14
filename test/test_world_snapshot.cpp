#include <gtest/gtest.h>

#include "diesel_vision_backend/world_snapshot.hpp"

namespace diesel_vision_backend {
namespace {

TrackerResult tracked_result(TrackerState state, int armor_id, bool predicted_only = false) {
    TrackerResult result;
    result.state = state;
    result.associated_armor_id = armor_id;
    result.predicted_only = predicted_only;
    TargetState target;
    target.xyz = Eigen::Vector3d(1.0, 2.0, 0.2);
    target.vxyz = Eigen::Vector3d(0.5, 0.0, 0.0);
    result.target = target;
    return result;
}

} // namespace

TEST(WorldSnapshot, LostProducesInvalidEmptySnapshot) {
    TrackerResult result;
    result.state = TrackerState::Lost;
    const WorldSnapshotData snapshot = build_world_snapshot(result, 5.0, 8);
    EXPECT_FALSE(snapshot.valid);
    EXPECT_TRUE(snapshot.tracks.empty());
    EXPECT_DOUBLE_EQ(snapshot.stamp_s, 5.0);
}

TEST(WorldSnapshot, TrackingExposesPositionVelocityAndState) {
    const WorldSnapshotData snapshot =
        build_world_snapshot(tracked_result(TrackerState::Tracking, 2), 5.0, 8);
    ASSERT_TRUE(snapshot.valid);
    ASSERT_EQ(snapshot.tracks.size(), 1U);
    const SnapshotTrack& track = snapshot.tracks.front();
    EXPECT_EQ(track.armor_id, 2);
    EXPECT_DOUBLE_EQ(track.xyz_world.x(), 1.0);
    EXPECT_DOUBLE_EQ(track.velocity_world.x(), 0.5);
    EXPECT_EQ(track.state, static_cast<int>(TrackerState::Tracking));
    EXPECT_DOUBLE_EQ(track.confidence, 1.0);
}

TEST(WorldSnapshot, PredictionOnlyNeverLooksLikeAConfirmedLock) {
    const WorldSnapshotData snapshot =
        build_world_snapshot(tracked_result(TrackerState::Tracking, 0, true), 5.0, 8);
    ASSERT_TRUE(snapshot.valid);
    EXPECT_LT(snapshot.tracks.front().confidence, 1.0);
}

TEST(WorldSnapshot, ConfidenceCapsDetectingAndZeroesLost) {
    EXPECT_DOUBLE_EQ(track_confidence(TrackerState::Lost, 0, 8), 0.0);
    EXPECT_DOUBLE_EQ(track_confidence(TrackerState::Detecting, 0, 8), 0.5);
    EXPECT_DOUBLE_EQ(track_confidence(TrackerState::Tracking, 0, 8), 1.0);
    EXPECT_DOUBLE_EQ(track_confidence(TrackerState::Tracking, 8, 8), 0.0);
}

} // namespace diesel_vision_backend
