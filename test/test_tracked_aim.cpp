#include <cmath>

#include <gtest/gtest.h>

#include "diesel_auto_aim/tracked_aim.hpp"

namespace diesel_auto_aim {
namespace {

TargetState fixture() {
    return {
        Eigen::Vector3d(4.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), 0.0, 0.0, 0.2, 0.0, 0.0};
}

TEST(TrackedAim, PredictsCenterTranslationAndBoardRotation) {
    auto state = fixture();
    state.w = M_PI / 2.0;
    const auto point = predict_tracked_armor(state, 0, 1.0);
    EXPECT_NEAR(point.x(), 5.0, 1e-9);
    EXPECT_NEAR(point.y(), -0.2, 1e-9);
}

TEST(TrackedAim, UsesAlternateRadiusAndHeight) {
    auto state = fixture();
    state.l = 0.05;
    state.h = -0.1;
    const auto point = predict_tracked_armor(state, 1, 0.0);
    EXPECT_NEAR(point.x(), 4.0, 1e-9);
    EXPECT_NEAR(point.y(), -0.25, 1e-9);
    EXPECT_NEAR(point.z(), -0.1, 1e-9);
}

TEST(TrackedAim, ConvertsWorldPointToCurrentGimbalAngle) {
    const auto state = fixture();
    const auto q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
    const auto command = aim_at_tracked_armor(state, 0, 0.0, q, Eigen::Vector3d::Zero(), 0.0);
    ASSERT_TRUE(command.valid);
    EXPECT_NEAR(command.yaw, -M_PI / 2.0, 1e-9);
    EXPECT_NEAR(command.pitch, 0.0, 1e-9);
}

TEST(TrackedAim, RejectsBadIdTimeAndAttitude) {
    const auto state = fixture();
    const auto q = Eigen::Quaterniond::Identity();
    EXPECT_FALSE(aim_at_tracked_armor(state, 4, 0.0, q, Eigen::Vector3d::Zero(), 0.0).valid);
    EXPECT_FALSE(aim_at_tracked_armor(state, 0, -0.1, q, Eigen::Vector3d::Zero(), 0.0).valid);
    EXPECT_FALSE(aim_at_tracked_armor(state, 0, 0.0, Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0),
                                      Eigen::Vector3d::Zero(), 0.0)
                     .valid);
}

TEST(TrackedAim, SelectsTheBoardFacingTheGimbalNotTheFirstOne) {
    auto state = fixture(); // 车心 (4,0)，朝向 a=0，板 0 在 (3.8, 0)
    // 云台在 +y 方向：此时朝向云台的是板 1（板 1 在 (4,-0.2)，法向 +y？按约定法向从板指向车心）
    // 用 facing 的几何定义直接核对：选择结果必须是四块板里 facing 最大的那块。
    // 云台在 +y 方向：面向云台的是板 1（位于 y=-0.2 一侧，外法向 +y）
    const Eigen::Vector3d gimbal(4.0, 3.0, 0.0);
    const auto selection = select_aim_armor(state, gimbal, 0.0, -1);
    ASSERT_TRUE(selection.valid());
    double best = -2.0;
    int best_id = -1;
    for (int id = 0; id < 4; ++id) {
        const Eigen::Vector3d point = predict_tracked_armor(state, id, 0.0);
        const Eigen::Vector3d to_gimbal = gimbal - point;
        const double angle = id * M_PI / 2.0;
        // 外法向 = -(cos, sin)：Target 约定 (cos,sin) 指向板→车心
        const Eigen::Vector3d outward(-std::cos(angle), -std::sin(angle), 0.0);
        const double facing = outward.dot(to_gimbal.normalized());
        if (facing > best) {
            best = facing;
            best_id = id;
        }
    }
    EXPECT_EQ(selection.armor_id, best_id);
    EXPECT_NEAR(selection.facing, best, 1e-9);
    // 物理含义：选中的板必须是"外法向朝向云台"的那块，facing 应为正
    EXPECT_GT(selection.facing, 0.9);
}

TEST(TrackedAim, ReturnsInvalidWhenAllBoardsAreEdgeOn) {
    auto state = fixture();
    // 云台放在车心正上方：四块板的法向都在水平面内，与"板→云台"接近垂直
    const Eigen::Vector3d gimbal(4.0, 0.0, 50.0);
    const auto selection = select_aim_armor(state, gimbal, 0.0, -1, 30.0 * M_PI / 180.0);
    EXPECT_FALSE(selection.valid());
}

TEST(TrackedAim, KeepsPreviousBoardWhenFacingsAreClose) {
    auto state = fixture();
    // 云台放在 45° 方向，板 0 与板 1 的 facing 接近打平，滞回应保留上一块
    const Eigen::Vector3d gimbal(4.0 - 3.0, 3.0, 0.0);
    const auto first = select_aim_armor(state, gimbal, 0.0, -1);
    ASSERT_TRUE(first.valid());
    const auto again = select_aim_armor(state, gimbal, 0.0, first.armor_id);
    EXPECT_EQ(again.armor_id, first.armor_id);

    // 另一块板明显更好时必须切换，不能死锁在上一块
    const Eigen::Vector3d better_gimbal(4.0, 5.0, 0.0);
    const auto switched = select_aim_armor(state, better_gimbal, 0.0, first.armor_id);
    ASSERT_TRUE(switched.valid());
    EXPECT_NE(switched.armor_id, first.armor_id);
}

} // namespace
} // namespace diesel_auto_aim
