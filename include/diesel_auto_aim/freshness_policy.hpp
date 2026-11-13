#ifndef FORAY_AUTO_AIM_FRESHNESS_POLICY_HPP
#define FORAY_AUTO_AIM_FRESHNESS_POLICY_HPP

#include <cmath>

namespace diesel_auto_aim {

/// 时间新鲜度策略 流水线与控制环共用 禁止两处各写一个魔法数
/// 三种年龄的物理含义不同 不能共用一个阈值
struct FreshnessPolicy {
    /// A 图像年龄 now-frame_stamp 观测到决策之间允许的耗时
    /// 0.05s 约等于 100Hz 控制的 5 拍 1m/s 目标横移 5cm
    double max_frame_age_s = 0.05;
    /// B 姿态插值窗口 前后两条姿态的最大间隔
    /// IMU 约 1ms 采样 0.03s 已含 30 个样本 超过说明丢包
    double max_attitude_gap_s = 0.03;
    /// C 指令年龄 发包时刻-frame_stamp 超限必须撤销控制
    /// 必须满足 max_command_age_s <= max_frame_age_s
    double max_command_age_s = 0.02;
};

/// 拒绝原因 只用于统计和日志 不代表精度评价
enum class StaleReason {
    None,
    NotFinite,     ///< 时间戳非有限值
    FrameTooOld,   ///< 图像年龄超过 max_frame_age_s
    CommandTooOld, ///< 指令年龄超过 max_command_age_s
    AttitudeGap,   ///< 缺少前后姿态 或插值区间超过 max_attitude_gap_s
    ClockBackward, ///< 时间戳回退 会话切换或回绕 不得续用旧目标
};

inline const char* stale_name(StaleReason reason) {
    switch (reason) {
    case StaleReason::None:
        return "FRESH";
    case StaleReason::NotFinite:
        return "STAMP_NOT_FINITE";
    case StaleReason::FrameTooOld:
        return "FRAME_TOO_OLD";
    case StaleReason::CommandTooOld:
        return "COMMAND_TOO_OLD";
    case StaleReason::AttitudeGap:
        return "ATTITUDE_GAP";
    case StaleReason::ClockBackward:
        return "CLOCK_BACKWARD";
    }
    return "UNKNOWN";
}

/// 检查同一单调时间轴上 sample_s 到 now_s 的年龄
/// @param backward_if_equal true 时 sample==now 也视为回退 用于流水线判重
inline StaleReason classify_age(double sample_s, double now_s, double max_age_s,
                                bool backward_if_equal = false) {
    if (!std::isfinite(sample_s) || !std::isfinite(now_s))
        return StaleReason::NotFinite;
    if (now_s < sample_s)
        return StaleReason::ClockBackward;
    if (now_s == sample_s && backward_if_equal)
        return StaleReason::ClockBackward;
    if (now_s - sample_s > max_age_s)
        return StaleReason::FrameTooOld;
    return StaleReason::None;
}

} // namespace diesel_auto_aim

#endif // FORAY_AUTO_AIM_FRESHNESS_POLICY_HPP
