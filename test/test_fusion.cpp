#include <gtest/gtest.h>

#include <cmath>

#include "diesel_vision_backend/fusion.hpp"

namespace diesel_vision_backend {
namespace {

Observation make(int armor_id, double x, double z, double stamp, ObservationSource source,
                 TargetClass target_class = TargetClass::Armor, double confidence = 0.8) {
    Observation observation;
    observation.armor_id = armor_id;
    observation.xyz_camera = Eigen::Vector3d(x, 0.0, z);
    observation.stamp_s = stamp;
    observation.source = source;
    observation.target_class = target_class;
    observation.confidence = confidence;
    return observation;
}

} // namespace

TEST(Fusion, KeepsFreshObservationsAndSortsByArmorId) {
    const FusionConfig config;
    const std::vector<Observation> input{make(2, 0.2, 3.0, 10.0, ObservationSource::OwnCamera),
                                         make(0, -0.2, 3.0, 10.0, ObservationSource::OwnCamera)};
    const FusionResult result = fuse_observations(input, 10.0, config);
    ASSERT_EQ(result.observations.size(), 2U);
    EXPECT_EQ(result.observations[0].armor_id, 0);
    EXPECT_EQ(result.observations[1].armor_id, 2);
    EXPECT_EQ(result.dropped_stale + result.dropped_class + result.dropped_invalid, 0);
}

TEST(Fusion, DropsStaleAndFutureStampsInsteadOfUsingThem) {
    FusionConfig config;
    config.max_age_s = 0.1;
    const std::vector<Observation> input{make(0, 0.0, 3.0, 9.5, ObservationSource::OwnCamera),
                                         make(1, 0.1, 3.0, 10.2, ObservationSource::OwnCamera)};
    const FusionResult result = fuse_observations(input, 10.0, config);
    EXPECT_TRUE(result.observations.empty());
    EXPECT_EQ(result.dropped_stale, 2);
}

TEST(Fusion, NonArmorTargetsAreDroppedWhileOnlyArmorIsImplemented) {
    const FusionConfig config;
    const std::vector<Observation> input{
        make(0, 0.0, 3.0, 10.0, ObservationSource::OwnCamera, TargetClass::EnergyMechanism),
        make(1, 0.1, 3.0, 10.0, ObservationSource::OwnCamera, TargetClass::Armor)};
    const FusionResult result = fuse_observations(input, 10.0, config);
    ASSERT_EQ(result.observations.size(), 1U);
    EXPECT_EQ(result.observations.front().armor_id, 1);
    EXPECT_EQ(result.dropped_class, 1);
}

TEST(Fusion, SameArmorFromBothSourcesKeepsOwnCameraObservation) {
    const FusionConfig config;
    const std::vector<Observation> input{
        make(1, 0.30, 3.0, 10.0, ObservationSource::AllyShared, TargetClass::Armor, 0.95),
        make(1, 0.32, 3.0, 10.0, ObservationSource::OwnCamera, TargetClass::Armor, 0.60)};
    const FusionResult result = fuse_observations(input, 10.0, config);
    ASSERT_EQ(result.observations.size(), 1U);
    EXPECT_EQ(result.observations.front().source, ObservationSource::OwnCamera);
    EXPECT_DOUBLE_EQ(result.observations.front().confidence, 0.60);
    EXPECT_EQ(result.merged, 1);
}

TEST(Fusion, SameArmorIdFarApartIsNotMerged) {
    // 0.3 m 差已超过同板门限：更可能是相邻板编号错了，合并会把误差混进跟踪
    const FusionConfig config;
    const std::vector<Observation> input{make(1, 0.0, 3.0, 10.0, ObservationSource::AllyShared),
                                         make(1, 0.3, 3.0, 10.0, ObservationSource::OwnCamera)};
    const FusionResult result = fuse_observations(input, 10.0, config);
    EXPECT_EQ(result.observations.size(), 2U);
    EXPECT_EQ(result.merged, 0);
}

TEST(Fusion, HigherConfidenceWinsWithinSameSource) {
    const FusionConfig config;
    const std::vector<Observation> input{
        make(0, 0.0, 3.0, 10.0, ObservationSource::OwnCamera, TargetClass::Armor, 0.30),
        make(0, 0.01, 3.0, 10.0, ObservationSource::OwnCamera, TargetClass::Armor, 0.90)};
    const FusionResult result = fuse_observations(input, 10.0, config);
    ASSERT_EQ(result.observations.size(), 1U);
    EXPECT_DOUBLE_EQ(result.observations.front().confidence, 0.90);
}

TEST(Fusion, IllegalIdsGeometryAndNaNsAreRejected) {
    const FusionConfig config;
    std::vector<Observation> input{make(3, 0.0, 3.0, 10.0, ObservationSource::OwnCamera),
                                   make(-1, 0.0, 3.0, 10.0, ObservationSource::OwnCamera),
                                   make(0, 0.0, 0.0, 10.0, ObservationSource::OwnCamera)};
    Observation nan = make(1, 0.0, 3.0, 10.0, ObservationSource::OwnCamera);
    nan.xyz_camera.x() = std::nan("");
    input.push_back(nan);
    const FusionResult result = fuse_observations(input, 10.0, config);
    EXPECT_TRUE(result.observations.empty());
    EXPECT_EQ(result.dropped_invalid, 4);
}

TEST(Fusion, InvalidConfigDropsEverythingInsteadOfGuessing) {
    FusionConfig config;
    config.max_age_s = 0.0; // 非法：没有可接受的年龄窗口
    const std::vector<Observation> input{make(0, 0.0, 3.0, 10.0, ObservationSource::OwnCamera)};
    const FusionResult result = fuse_observations(input, 10.0, config);
    EXPECT_TRUE(result.observations.empty());
    EXPECT_EQ(result.dropped_invalid, 1);
}

} // namespace diesel_vision_backend
