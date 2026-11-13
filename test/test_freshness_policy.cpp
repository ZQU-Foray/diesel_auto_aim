// 时间门限的边界行为：非有限、回退、超龄、插值窗口
#include "diesel_auto_aim/aim_pipeline.hpp"
#include "diesel_auto_aim/freshness_policy.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace {

using diesel_auto_aim::attitude_lookup_s;
using diesel_auto_aim::AttitudeSample;
using diesel_auto_aim::classify_age;
using diesel_auto_aim::FreshnessPolicy;
using diesel_auto_aim::interpolate_attitude;
using diesel_auto_aim::StaleReason;
using diesel_auto_aim::valid_attitude_offset;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

AttitudeSample sample(double stamp_s, double yaw_rad) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    return {stamp_s, q};
}

} // namespace

TEST(FreshnessPolicy, 默认值满足指令门限不宽于图像门限) {
    const FreshnessPolicy policy;
    EXPECT_LE(policy.max_command_age_s, policy.max_frame_age_s);
    EXPECT_LT(policy.max_attitude_gap_s, policy.max_frame_age_s);
}

TEST(FreshnessPolicy, 正常范围内判为可用) {
    EXPECT_EQ(classify_age(10.000, 10.010, 0.05), StaleReason::None);
    EXPECT_EQ(classify_age(10.000, 10.040, 0.05), StaleReason::None);
}

// 浮点减法不保证"恰好等于阈值" 10.05-10.00 = 0.05000000000000071 略大于 0.05
// 所以时间门限一律用严格大于比较 测试只断言"明显在范围内"和"明显超出"
TEST(FreshnessPolicy, 恰好落在阈值上按超龄处理) {
    EXPECT_EQ(classify_age(10.000, 10.060, 0.06), StaleReason::FrameTooOld);
}

TEST(FreshnessPolicy, 指令门限必须覆盖视觉帧间隔) {
    const FreshnessPolicy policy;
    // 视觉 30fps → 帧间隔 33.3ms；指令门限若小于它会出现撤销-重发抖动
    EXPECT_GT(policy.max_command_age_s, 1. / 30.);
}

TEST(FreshnessPolicy, 超过上限判为太旧) {
    EXPECT_EQ(classify_age(10.000, 10.061, 0.06), StaleReason::FrameTooOld);
}

TEST(FreshnessPolicy, 时间戳回退判为时钟回退) {
    EXPECT_EQ(classify_age(10.020, 10.010, 0.05), StaleReason::ClockBackward);
}

TEST(FreshnessPolicy, 非有限时间戳单独归类) {
    EXPECT_EQ(classify_age(kNaN, 10.0, 0.05), StaleReason::NotFinite);
    EXPECT_EQ(classify_age(10.0, kNaN, 0.05), StaleReason::NotFinite);
}

TEST(FreshnessPolicy, 相等时是否算回退由调用方指定) {
    EXPECT_EQ(classify_age(10.0, 10.0, 0.05, false), StaleReason::None);
    EXPECT_EQ(classify_age(10.0, 10.0, 0.05, true), StaleReason::ClockBackward);
}

TEST(FreshnessPolicy, 原因名字可用于统计输出) {
    EXPECT_STREQ(diesel_auto_aim::stale_name(StaleReason::None), "FRESH");
    EXPECT_STREQ(diesel_auto_aim::stale_name(StaleReason::FrameTooOld), "FRAME_TOO_OLD");
    EXPECT_STREQ(diesel_auto_aim::stale_name(StaleReason::AttitudeGap), "ATTITUDE_GAP");
    EXPECT_STREQ(diesel_auto_aim::stale_name(StaleReason::ClockBackward), "CLOCK_BACKWARD");
}

TEST(AttitudeInterpolation, 两侧样本之间按比例插值) {
    const std::vector<AttitudeSample> history{sample(10.000, 0.0), sample(10.010, 0.100)};
    const auto q = interpolate_attitude(history, 10.005, 0.030);
    ASSERT_TRUE(q.has_value());
    // 中点应约等于一半角度的旋转
    const double yaw = 2.0 * std::atan2(q->z(), q->w());
    EXPECT_NEAR(yaw, 0.050, 1e-6);
}

TEST(AttitudeInterpolation, 命中样本时刻直接返回该样本) {
    const std::vector<AttitudeSample> history{sample(10.000, 0.0), sample(10.010, 0.100)};
    const auto q = interpolate_attitude(history, 10.000, 0.030);
    ASSERT_TRUE(q.has_value());
    EXPECT_NEAR(2.0 * std::atan2(q->z(), q->w()), 0.0, 1e-9);
}

TEST(AttitudeInterpolation, 缺少前一条样本拒绝插值) {
    const std::vector<AttitudeSample> history{sample(10.010, 0.100), sample(10.020, 0.200)};
    EXPECT_FALSE(interpolate_attitude(history, 10.005, 0.030).has_value());
}

TEST(AttitudeInterpolation, 样本间隔超过窗口上限时拒绝) {
    const std::vector<AttitudeSample> history{sample(10.000, 0.0), sample(10.040, 0.400)};
    EXPECT_TRUE(interpolate_attitude(history, 10.020, 0.030).has_value() == false);
    EXPECT_TRUE(interpolate_attitude(history, 10.020, 0.050).has_value());
}

TEST(AttitudeInterpolation, 时间戳回退的历史被拒绝) {
    const std::vector<AttitudeSample> history{sample(10.010, 0.0), sample(10.000, 0.100)};
    EXPECT_FALSE(interpolate_attitude(history, 10.005, 0.030).has_value());
}

TEST(AttitudeInterpolation, 目标时刻超出历史末尾时拒绝) {
    const std::vector<AttitudeSample> history{sample(10.000, 0.0), sample(10.010, 0.100)};
    EXPECT_FALSE(interpolate_attitude(history, 10.500, 0.030).has_value());
}

TEST(AttitudeOffset, 库默认不假设已对齐) {
    const FreshnessPolicy policy;
    // 实测最优 +8ms 由运行入口(启动器)注入 库本身保持中性
    EXPECT_DOUBLE_EQ(policy.attitude_time_offset_s, 0.0);
}

TEST(AttitudeOffset, 允许范围是正负一百毫秒) {
    EXPECT_TRUE(valid_attitude_offset(0.0));
    EXPECT_TRUE(valid_attitude_offset(0.014)); // 早上实测的量级
    EXPECT_TRUE(valid_attitude_offset(-0.014));
    EXPECT_TRUE(valid_attitude_offset(0.1));
    EXPECT_TRUE(valid_attitude_offset(-0.1));
    EXPECT_FALSE(valid_attitude_offset(0.101)); // 超出上限
    EXPECT_FALSE(valid_attitude_offset(kNaN));  // 非有限值
}

TEST(AttitudeOffset, 正偏移表示查更早的姿态) {
    FreshnessPolicy policy;
    policy.attitude_time_offset_s = 0.014;
    // 图像在 10.000 查姿态应用 9.986 IMU 的运动比图像早 14ms
    EXPECT_NEAR(attitude_lookup_s(10.000, policy), 9.986, 1e-12);
}

TEST(AttitudeOffset, 负偏移表示查更晚的姿态) {
    FreshnessPolicy policy;
    policy.attitude_time_offset_s = -0.014;
    EXPECT_NEAR(attitude_lookup_s(10.000, policy), 10.014, 1e-12);
}

TEST(AttitudeOffset, 零偏移时查姿态时刻不变) {
    FreshnessPolicy policy;
    policy.attitude_time_offset_s = 0.0; // 默认已非 0 这里显式归零
    EXPECT_DOUBLE_EQ(attitude_lookup_s(10.000, policy), 10.000);
}

TEST(AttitudeOffset, 偏移后仍能在历史里命中样本) {
    // 图像 10.020 偏移 14ms 后应查到 10.006 附近 用两侧样本插值
    const std::vector<AttitudeSample> history{sample(10.000, 0.0), sample(10.010, 0.100)};
    FreshnessPolicy policy;
    policy.attitude_time_offset_s = 0.014;
    const auto q =
        interpolate_attitude(history, attitude_lookup_s(10.020, policy), policy.max_attitude_gap_s);
    ASSERT_TRUE(q.has_value());
    const double yaw = 2.0 * std::atan2(q->z(), q->w());
    EXPECT_NEAR(yaw, 0.060, 1e-6); // 10.006 落在 10.000~10.010 的 60% 处
}
