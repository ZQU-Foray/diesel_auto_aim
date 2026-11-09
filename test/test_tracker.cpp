#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "diesel_auto_aim/tracker.hpp"

using namespace diesel_auto_aim;

namespace {

TrackerConfig config() {
    TrackerConfig c;
    c.radius_m = 0.2; // 仅为离线靶车测试参数，不是实车标定值
    c.armor_count = 4;
    c.association_gate_m = 0.1;
    c.confirmation_frames = 2;
    c.max_missed_frames = 2;
    c.max_frame_gap_s = 0.1;
    c.initial_covariance_diag << 0.25, 1.0, 0.25, 1.0, 0.25, 1.0, 0.04, 0.01, 1e-6, 1e-6, 1e-6;
    return c;
}

ArmorTrackObservation plate(double x, double y, int robot_id = 1, double yaw = 0.0) {
    return {{x, y, 0.5}, yaw, robot_id};
}

} // namespace

TEST(Tracker, 类别契约拒绝无目标和其他编号) {
    for (int invalid_id : {-1, 3, 7}) {
        Tracker tracker(config());
        const auto result = tracker.step({plate(4.8, 0.0, invalid_id)}, 1.0);
        EXPECT_EQ(result.state, TrackerState::Lost);
        EXPECT_FALSE(result.target.has_value());
    }
    for (int valid_id : {0, 1, 2}) {
        Tracker tracker(config());
        EXPECT_EQ(tracker.step({plate(4.8, 0.0, valid_id)}, 1.0).state, TrackerState::Detecting);
    }
}

TEST(Tracker, 按类别建轨并跳过未配置的重装) {
    auto c = config();
    c.init_profiles[0] = TargetInitProfile{0.17, 4, c.initial_covariance_diag};
    c.init_profiles[1] = TargetInitProfile{0.17, 4, c.initial_covariance_diag};
    Tracker sentinel(c);
    const auto s = sentinel.step({plate(3.83, 0.0, 0)}, 1.0);
    ASSERT_TRUE(s.target.has_value());
    EXPECT_NEAR(s.target->xyz.x(), 4.0, 1e-9);
    EXPECT_NEAR(s.target->r, 0.17, 1e-9);
    Tracker infantry(c);
    const auto i = infantry.step({plate(3.83, 0.0, 1)}, 1.0);
    ASSERT_TRUE(i.target.has_value());
    EXPECT_NEAR(i.target->xyz.x(), 4.0, 1e-9);
    Tracker heavy(c);
    const auto h = heavy.step({plate(3.83, 0.0, 2)}, 1.0);
    EXPECT_EQ(h.state, TrackerState::Lost);
    EXPECT_FALSE(h.target.has_value());
}

TEST(Tracker, 四板关联与短暂丢帧) {
    Tracker tracker(config());
    auto first = tracker.step({plate(4.8, 0.0)}, 1.00);
    ASSERT_EQ(first.state, TrackerState::Detecting);
    ASSERT_TRUE(first.target.has_value());
    EXPECT_EQ(first.associated_armor_id, 0);
    EXPECT_NEAR(first.target->xyz.x(), 5.0, 1e-9);

    auto second = tracker.step({plate(5.0, -0.2, 1, M_PI / 2.0)}, 1.01);
    EXPECT_EQ(second.state, TrackerState::Tracking);
    EXPECT_EQ(second.associated_armor_id, 1);
    const auto predicted = tracker.predicted_armors();
    ASSERT_TRUE(predicted.has_value());
    EXPECT_NEAR((*predicted)[2].x(), 5.2, 1e-3);

    EXPECT_EQ(tracker.step({}, 1.02).state, TrackerState::TempLost);
    auto recovered = tracker.step({plate(5.2, 0.0, 1, M_PI)}, 1.03);
    EXPECT_EQ(recovered.state, TrackerState::Tracking);
    EXPECT_EQ(recovered.associated_armor_id, 2);
}

TEST(Tracker, 分方向关联不再暗中套用三维距离上限) {
    auto c = config();
    c.association_gate_m = 0.25; // 旧 3D 门；此用例仅用于对照
    c.sensor_origin_world = Eigen::Vector3d(0.0, 0.0, 0.5);
    c.association_bearing_gate_rad = 0.06;
    c.association_depth_gate_m = 1.0;
    Tracker tracker(c);
    EXPECT_EQ(tracker.step({plate(4.8, 0.0)}, 1.00).state, TrackerState::Detecting);
    // 深度差 0.4 m 超过旧 0.25 m 三维门，但方向一致、深度在显式 1 m 门内。
    const auto accepted = tracker.step({plate(5.2, 0.0)}, 1.01);
    EXPECT_EQ(accepted.state, TrackerState::Tracking);
    EXPECT_EQ(accepted.associated_observation_index, 0);
    // 横向偏离明显时仍应被方位角门拒绝，不能误以为修复等于取消关联保护。
    const auto rejected = tracker.step({plate(5.2, 1.0)}, 1.02);
    EXPECT_EQ(rejected.state, TrackerState::TempLost);
    EXPECT_EQ(rejected.associated_observation_index, -1);
}

TEST(Tracker, SP关联策略与Foray距离门独立) {
    auto c = config();
    c.association_strategy = AssociationStrategy::SpNearestThree;
    c.association_gate_m = 0.01; // SP 评分策略不套用 Foray 距离门
    c.sensor_origin_world = Eigen::Vector3d::Zero();
    Tracker tracker(c);
    EXPECT_EQ(tracker.step({plate(4.8, 0.0)}, 1.00).state, TrackerState::Detecting);
    const auto result = tracker.step({plate(5.0, -0.2, 1, M_PI / 2.0)}, 1.01);
    EXPECT_EQ(result.state, TrackerState::Tracking);
    EXPECT_EQ(result.associated_armor_id, 1);
}

TEST(Tracker, 错误目标和野值不更新状态) {
    Tracker tracker(config());
    tracker.step({plate(4.8, 0.0)}, 1.00);
    auto result = tracker.step({plate(4.8, 0.0, 0), plate(10.0, 0.0)}, 1.01);
    EXPECT_EQ(result.state, TrackerState::Lost); // 未确认阶段丢失，重新初始化
    EXPECT_EQ(result.associated_armor_id, -1);
    EXPECT_FALSE(tracker.predicted_armors().has_value());
}

TEST(Tracker, 连续丢帧与时间跳变重置) {
    Tracker tracker(config());
    tracker.step({plate(4.8, 0.0)}, 1.00);
    tracker.step({plate(4.8, 0.0)}, 1.01);
    EXPECT_EQ(tracker.step({}, 1.02).state, TrackerState::TempLost);
    EXPECT_EQ(tracker.step({}, 1.03).state, TrackerState::TempLost);
    EXPECT_EQ(tracker.step({}, 1.04).state, TrackerState::Lost);
    tracker.step({plate(4.8, 0.0)}, 2.00);
    EXPECT_EQ(tracker.step({plate(4.8, 0.0)}, 2.20).state, TrackerState::Detecting);
    EXPECT_EQ(tracker.step({}, std::numeric_limits<double>::quiet_NaN()).state, TrackerState::Lost);
}

TEST(Tracker, 两块板同帧可见时优先保持上一块) {
    Tracker tracker(config());
    tracker.step({plate(4.8, 0.0)}, 1.00);
    tracker.step({plate(4.8, 0.0)}, 1.01);
    // 即使另一块板更贴近预测点，原板仍在关联门限内就不应跳板。
    const auto held = tracker.step({plate(4.84, 0.0), plate(5.0, -0.2, 1, M_PI / 2.0)}, 1.02);
    EXPECT_EQ(held.state, TrackerState::Tracking);
    EXPECT_EQ(held.associated_armor_id, 0);
    EXPECT_EQ(held.associated_observation_index, 0);

    const auto switched = tracker.step({plate(5.0, -0.2, 1, M_PI / 2.0)}, 1.03);
    EXPECT_EQ(switched.associated_armor_id, 1);
    EXPECT_EQ(switched.associated_observation_index, 0);
}

TEST(Tracker, 换板距离近似打平时优先相邻板) {
    auto c = config();
    c.adjacent_switch_margin_m = 0.02; // 离线测试阈值；实机须由观测误差重新确定
    Tracker tracker(c);
    tracker.step({plate(4.8, 0.0)}, 1.00);
    tracker.step({plate(4.8, 0.0)}, 1.01);
    // 旧板 0 消失。板 2 略近，但板 1 与其只差约 1 cm，选连续的邻板。
    const auto near_tie =
        tracker.step({plate(5.01, -0.2, 1, M_PI / 2.0), plate(5.2, 0.0, 1, M_PI)}, 1.02);
    EXPECT_EQ(near_tie.associated_armor_id, 1);
    EXPECT_EQ(near_tie.associated_observation_index, 0);
}

TEST(Tracker, 相邻板明显更远时不覆盖几何证据) {
    auto c = config();
    c.adjacent_switch_margin_m = 0.02; // 离线测试阈值；实机须由观测误差重新确定
    Tracker tracker(c);
    tracker.step({plate(4.8, 0.0)}, 1.00);
    tracker.step({plate(4.8, 0.0)}, 1.01);
    const auto clear_winner =
        tracker.step({plate(5.07, -0.2, 1, M_PI / 2.0), plate(5.2, 0.0, 1, M_PI)}, 1.02);
    EXPECT_EQ(clear_winner.associated_armor_id, 2);
    EXPECT_EQ(clear_winner.associated_observation_index, 1);
}

TEST(Tracker, 原地转车不能把装甲板绕中心运动当成车体平移) {
    Tracker tracker(config());
    constexpr double kCenterX = 5.0;
    constexpr double kRadius = 0.2;
    for (int frame = 0; frame < 40; ++frame) {
        const double yaw = 0.025 * frame;
        const ArmorTrackObservation observation{
            {kCenterX - kRadius * std::cos(yaw), -kRadius * std::sin(yaw), 0.5}, yaw, 1};
        const auto result = tracker.step({observation}, 1.0 + 0.01 * frame);
        ASSERT_TRUE(result.target.has_value());
        if (frame > 10)
            EXPECT_LT((result.target->xyz - Eigen::Vector3d(kCenterX, 0, 0.5)).norm(), 0.08);
    }
}

TEST(Tracker, 单帧朝向误判不能立即扭转整车预测) {
    Tracker tracker(config());
    for (int frame = 0; frame < 20; ++frame) {
        const auto result = tracker.step({plate(4.8, 0.0)}, 1.0 + 0.01 * frame);
        ASSERT_TRUE(result.target.has_value());
    }
    const auto outlier = tracker.step({plate(4.8, 0.0, 1, M_PI)}, 1.20);
    ASSERT_TRUE(outlier.target.has_value());
    EXPECT_EQ(outlier.associated_armor_id, -1);
    EXPECT_EQ(outlier.state, TrackerState::TempLost);
    EXPECT_EQ(outlier.diagnostics.reason, AssociationDiagnostics::Reason::DistanceGate);
    EXPECT_EQ(outlier.diagnostics.distance_gate_count, 1);
    EXPECT_LT(std::abs(outlier.target->a), 0.2);
    EXPECT_LT((outlier.target->xyz - Eigen::Vector3d(5.0, 0.0, 0.5)).norm(), 0.05);
}

TEST(Tracker, 快转初期允许角速度从零估计而不是拉动车心) {
    auto c = config();
    c.radius_m = 0.248; // 当前模拟器靶车的离线几何值，不用于实车
    c.association_gate_m = 0.25;
    c.initial_covariance_diag[7] = 25.0; // 0±5 rad/s 的初始角速度假设
    Tracker tracker(c);
    auto narrow_c = c;
    narrow_c.initial_covariance_diag[7] = 0.01; // 旧先验：误以为角速度几乎为零
    Tracker narrow_tracker(narrow_c);
    constexpr double kCenterX = 4.0;
    constexpr double kRadius = 0.248;
    constexpr double kAngularVelocity = 4.0;
    double max_center_error = 0.0;
    double max_narrow_error = 0.0;
    for (int frame = 0; frame < 16; ++frame) {
        const double yaw = kAngularVelocity * frame * 0.03;
        const ArmorTrackObservation observation{
            {kCenterX - kRadius * std::cos(yaw), -kRadius * std::sin(yaw), 0.5}, yaw, 1};
        const auto result = tracker.step({observation}, 1.0 + frame * 0.03);
        const auto narrow_result = narrow_tracker.step({observation}, 1.0 + frame * 0.03);
        ASSERT_TRUE(result.target.has_value());
        ASSERT_EQ(result.associated_armor_id, 0) << "frame=" << frame;
        if (frame >= 5) {
            const Eigen::Vector3d center(kCenterX, 0.0, 0.5);
            const double center_error = (result.target->xyz - center).norm();
            max_center_error = std::max(max_center_error, center_error);
            EXPECT_LT(center_error, 0.05) << "frame=" << frame;
            if (narrow_result.target)
                max_narrow_error =
                    std::max(max_narrow_error, (narrow_result.target->xyz - center).norm());
        }
    }
    EXPECT_GT(max_narrow_error, max_center_error * 2.0);
}

namespace {

// 四板全可见：几何真值 radius/高度差可指定；h 只作用在 1、3 号板
std::vector<ArmorTrackObservation> four_plates(double center_x, double radius, double h,
                                               double noise = 0.0) {
    std::vector<ArmorTrackObservation> obs;
    for (int id = 0; id < 4; ++id) {
        const double yaw = id * 2.0 * M_PI / 4.0;
        const bool alternate = (id == 1 || id == 3);
        const double r = alternate ? radius : radius;
        Eigen::Vector3d p(center_x - r * std::cos(yaw), -r * std::sin(yaw),
                          0.5 + (alternate ? h : 0.0));
        // 用固定的小扰动代替随机数，测试必须可复现
        p.x() += (id == 0 ? noise : -noise);
        obs.push_back({p, yaw, 1});
    }
    return obs;
}

} // namespace

TEST(Tracker, 同帧多板弱更新让半径从错误初值收敛) {
    auto c = config();
    c.radius_m = 0.20;                   // 配置错 5 cm
    c.secondary_plate_noise_scale = 3.0; // 其余板做弱更新（噪声放大 3 倍）
    // r/l/h 的初始方差必须反映真实标定不确定度（5 cm → 2.5e-3）。
    // 若仍用 1e-6，卡尔曼增益近零，弱更新再多也推不动半径——这是本测试要固定的事实。
    c.initial_covariance_diag[8] = 2.5e-3;
    c.initial_covariance_diag[9] = 2.5e-3;
    c.initial_covariance_diag[10] = 1e-2;
    Tracker tracker(c);
    tracker.step(four_plates(5.0, 0.25, 0.0), 1.00);
    for (int frame = 1; frame < 20; ++frame) {
        const auto result = tracker.step(four_plates(5.0, 0.25, 0.0), 1.00 + 0.01 * frame);
        ASSERT_TRUE(result.target.has_value());
        if (frame == 19) {
            EXPECT_NEAR(result.target->r, 0.25, 0.01); // 收敛到真值
            EXPECT_LT((result.target->xyz - Eigen::Vector3d(5.0, 0.0, 0.5)).norm(), 0.01);
        }
    }
}

TEST(Tracker, 同帧多板弱更新让高低差收敛) {
    auto c = config();
    c.secondary_plate_noise_scale = 3.0;
    c.initial_covariance_diag[8] = 2.5e-3;
    c.initial_covariance_diag[9] = 2.5e-3;
    c.initial_covariance_diag[10] = 1e-2; // 高低差先验 10 cm 量级
    Tracker tracker(c);
    for (int frame = 0; frame < 20; ++frame) {
        const auto result = tracker.step(four_plates(5.0, 0.2, 0.10), 1.00 + 0.01 * frame);
        ASSERT_TRUE(result.target.has_value());
        if (frame >= 10)
            EXPECT_NEAR(result.target->h, 0.10, 0.02);
    }
}

TEST(Tracker, 关闭弱更新时半径不动但高低差可由同帧异奇偶板直接解出) {
    auto c = config();
    c.radius_m = 0.20;
    c.secondary_plate_noise_scale = 0.0; // 只更新关联到的第一块板
    c.initial_covariance_diag[8] = 2.5e-3;
    Tracker tracker(c);
    for (int frame = 0; frame < 20; ++frame) {
        const auto result = tracker.step(four_plates(5.0, 0.25, 0.10), 1.00 + 0.01 * frame);
        ASSERT_TRUE(result.target.has_value());
        if (frame == 19) {
            EXPECT_NEAR(result.target->r, 0.20, 0.005); // 半径仍靠 P0 与观测，弱更新关掉就不动
            // 初始化时同帧有 0/1 号板，异奇偶两次观测可直接解出高低差，不必等滤波器收敛
            EXPECT_NEAR(result.target->h, 0.10, 0.02);
        }
    }
}
