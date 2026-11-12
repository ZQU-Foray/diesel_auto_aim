#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>

#include "diesel_auto_aim/aimer.hpp"

using namespace diesel_auto_aim;

namespace {
constexpr double kBulletSpeed = 25.0; // 步兵哨兵弹速上限 直瞄不用 按接口传入
constexpr double kTolerance = 1e-9;
} // namespace

TEST(AimAt, 左前方四十五度) {
    const auto command = aim_at({1.0, 1.0, 0.0}, kBulletSpeed);
    EXPECT_NEAR(command.yaw, M_PI / 4.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
    EXPECT_TRUE(command.valid);
}

TEST(AimAt, 正前方) {
    const auto command = aim_at({1.0, 0.0, 0.0}, kBulletSpeed);
    EXPECT_NEAR(command.yaw, 0.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
    EXPECT_TRUE(command.valid);
}

TEST(AimAt, 前上方四十五度) {
    const auto command = aim_at({1.0, 0.0, 1.0}, kBulletSpeed);
    EXPECT_NEAR(command.yaw, 0.0, kTolerance);
    EXPECT_NEAR(command.pitch, -M_PI / 4.0, kTolerance);
    EXPECT_TRUE(command.valid);
}

TEST(AimAt, 右前方四十五度) {
    const auto command = aim_at({1.0, -1.0, 0.0}, kBulletSpeed);
    EXPECT_NEAR(command.yaw, -M_PI / 4.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
    EXPECT_TRUE(command.valid);
}

// 目标在正上方 水平距离退化 判不可信
TEST(AimAt, 正上方不可信) {
    const auto command = aim_at({0.0, 0.0, 5.0}, kBulletSpeed);
    EXPECT_FALSE(command.valid);
}
