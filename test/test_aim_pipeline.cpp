#include "diesel_vision_backend/aim_pipeline.hpp"
#include <gtest/gtest.h>

using namespace diesel_vision_backend;
namespace {
HandEyeCalibration handeye() {
    HandEyeCalibration h;
    h.rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    h.translation_m.setZero();
    return h;
}
std::vector<Observation> frame() {
    Observation o;
    o.armor_id = 1;
    o.reprojection_error = .1;
    o.xyz_camera = Eigen::Vector3d(0, 0, 3.83);
    // 前端在相机系给出的板面法向，已做过多解判定
    o.normal_camera = Eigen::Vector3d(0, 0, 1);
    o.normal_valid = true;
    return {o};
}
std::vector<AttitudeSample> history(double t,
                                    Eigen::Quaterniond q = Eigen::Quaterniond::Identity()) {
    return {{t - .005, q}, {t + .005, q}};
}
} // namespace

TEST(AimPipeline, InterpolationNoExtrapolationOrLargeGap) {
    auto samples = history(10.);
    EXPECT_TRUE(interpolate_attitude(samples, 10.));
    EXPECT_FALSE(interpolate_attitude(samples, 11.));
    samples[1].host_received_s = 10.1;
    EXPECT_FALSE(interpolate_attitude(samples, 10.));
}
TEST(AimPipeline, FullTrackerEkfSelectionAndWorldDirectAim) {
    AimPipelineConfig c;
    c.tracker = runtime_tracker_config(.17, .25);
    AimPipeline p(handeye(), c);
    AimPipelineResult r;
    for (int i = 0; i < 20; ++i) {
        const double t = 10. + i * .01;
        r = p.process_observations(frame(), t, t + .02, history(t));
    }
    ASSERT_TRUE(r.valid);
    ASSERT_TRUE(r.tracked);
    EXPECT_EQ(r.tracked->state, TrackerState::Tracking);
    EXPECT_NEAR(r.tracked->target->xyz.x(), 4., .01);
    EXPECT_NEAR(r.angles.yaw_rad, 0., 1e-6);
    EXPECT_EQ(r.selected_plate, 0);
    EXPECT_FALSE(p.process_observations(frame(), 10., 11., history(10.)).valid);
}
TEST(AimPipeline, GeometryTransformAppliedExactlyOnce) {
    auto h = handeye();
    h.translation_m = Eigen::Vector3d(.17, 0, 0);
    AimPipelineConfig c;
    c.tracker = runtime_tracker_config(.17, .25);
    c.backend = AimBackend::DirectPnP;
    AimPipeline p(h, c);
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ()));
    auto r = p.process_observations(frame(), 10., 10.02, history(10., q));
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(r.angles.yaw_rad, M_PI / 2., 1e-6);
}
TEST(AimPipeline, AmbiguousOrientationRejectedForTrackerButPositionUsableInDirectMode) {
    auto v = frame();
    v[0].normal_valid = false;
    AimPipelineConfig c;
    c.tracker = runtime_tracker_config(.17, .25);
    AimPipeline p(handeye(), c);
    auto r = p.process_observations(v, 10., 10.02, history(10.));
    EXPECT_FALSE(r.valid);
    EXPECT_EQ(r.world_observation_count, 0);
    c.backend = AimBackend::DirectPnP;
    p.configure(c);
    EXPECT_TRUE(p.process_observations(v, 11., 11.02, history(11.)).valid);
}
TEST(AimPipeline, RejectsIllegalPlateAndGeometry) {
    auto v = frame();
    const double t = 10.;
    v[0].armor_id = 3;
    AimPipelineConfig c;
    c.backend = AimBackend::DirectPnP;
    AimPipeline p(handeye(), c);
    EXPECT_FALSE(p.process_observations(v, t, t + .02, history(t)).valid);
    v = frame();
    v[0].xyz_camera.z() = -1.;
    EXPECT_FALSE(p.process_observations(v, t + 1., t + 1.02, history(t + 1.)).valid);
    v = frame();
    v[0].reprojection_error = 3.5;
    EXPECT_FALSE(p.process_observations(v, t + 2., t + 2.02, history(t + 2.)).valid);
}
TEST(AimPipeline, RateLimitsAndRejectsInvalidValues) {
    WorldAimResult goal{.5, .3, true}, previous{0, 0, true};
    EXPECT_NEAR(advance_world_aim(goal, previous, .01, 30.).yaw_rad, .3 * M_PI / 180., 1e-9);
    EXPECT_FALSE(advance_world_aim(goal, previous, .01, 181.).valid);
}
