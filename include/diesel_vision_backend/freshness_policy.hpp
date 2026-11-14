#ifndef FORAY_AUTO_AIM_FRESHNESS_POLICY_HPP
#define FORAY_AUTO_AIM_FRESHNESS_POLICY_HPP

#include <cmath>

namespace diesel_vision_backend {

/// 时间新鲜度策略 流水线与控制环共用 禁止两处各写一个魔法数
/// 三种年龄的物理含义不同 不能共用一个阈值
struct FreshnessPolicy {
    /// A 图像年龄 now-frame_stamp 观测到决策之间允许的耗时
    /// 实测 30fps 链路 frame_age P50 36.9ms P90 40.6ms → 60ms 留余量
    /// 注意 瓶颈是推理耗时 收紧门限不会变快 只会丢帧
    double max_frame_age_s = 0.06;
    /// B 姿态插值窗口 前后两条姿态的最大间隔
    /// IMU 约 1ms 采样 0.03s 已含 30 个样本 超过说明丢包
    double max_attitude_gap_s = 0.03;
    /// C 指令年龄 发包时刻-frame_stamp 超限必须撤销控制
    /// ⭐ 从【命令生成时刻】算 不是从取帧时刻算
    /// 必须大于视觉帧间隔(30fps → 33ms) 否则帧间会出现撤销-重发抖动
    /// 这里取 55ms：实测帧间隔 max 48ms（推理 33ms P50 / 48ms max）+ 余量
    /// 必须满足 max_command_age_s <= max_frame_age_s
    double max_command_age_s = 0.055;
    /// D 姿态时间偏移 查姿态用 frame_stamp - 此值 正值表示往【早】取
    /// 库默认 0 不替调用方假设已对齐；实测最优 +8ms 由运行入口注入（见 098）
    /// 这不是硬件常数 换会话/换设备必须重新确认 允许范围 ±0.1s
    double attitude_time_offset_s = 0.0;
};

/// 姿态偏移的允许范围 超出视为配置错误
inline bool valid_attitude_offset(double offset_s) {
    return std::isfinite(offset_s) && std::abs(offset_s) <= 0.1;
}

/// 由图像时刻和姿态偏移算出【查姿态】用的时刻
/// 图像运动早于或晚于 IMU 时 用这个时刻去 history 里插值
inline double attitude_lookup_s(double frame_stamp_s, const FreshnessPolicy& policy) {
    return frame_stamp_s - policy.attitude_time_offset_s;
}

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

} // namespace diesel_vision_backend

#endif // FORAY_AUTO_AIM_FRESHNESS_POLICY_HPP
