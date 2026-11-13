#ifndef FORAY_AUTO_AIM_NATIVE_CONTROL_LOOP_HPP
#define FORAY_AUTO_AIM_NATIVE_CONTROL_LOOP_HPP

#include "diesel_auto_aim/aim_pipeline.hpp"
#include "diesel_auto_aim/freshness_policy.hpp"
#include "diesel/platform/link/robolink_host_link.hpp"
#include "diesel/platform/link/usb_cdc_port.hpp"
#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

namespace diesel_auto_aim {

struct NativeControlFeedback {
    bool ready = false;
    bool armed = false;
    bool active = false;
    std::uint64_t queued_controls = 0; // 本机排队数量 不冒充电机执行次数
    diesel::platform::link::HostLinkState link_state = diesel::platform::link::HostLinkState::Disconnected;
    std::optional<diesel::platform::link::SystemStatus> status;
    WorldAimResult measured;
    WorldAimResult sent;
    std::vector<AttitudeSample> history;
    std::string error;
    // 时间门限统计 供回放分析 不代表精度评价
    std::uint64_t submitted = 0;               ///< submit 调用次数
    std::uint64_t rejected_clock_backward = 0; ///< 时间戳回退次数 真异常
    std::uint64_t command_interruptions = 0;   ///< 控制 active→inactive 次数 真中断
    std::uint64_t stale_events = 0; ///< 目标过期事件数 逐帧自然发生 仅诊断
    StaleReason last_stale = StaleReason::None; ///< 最近一次被撤销的原因
};

/// C++通讯装配 独占USB 1ms轮询/约100Hz控制 不是硬实时保证
/// 仅云台有效位 不发送底盘/弹道/发射命令 默认只观察
class NativeControlLoop {
  public:
    /// @param freshness 指令新鲜度门限 与 AimPipeline 使用同一份配置
    explicit NativeControlLoop(const std::string& device,
                               FreshnessPolicy freshness = FreshnessPolicy{});
    ~NativeControlLoop();
    NativeControlLoop(const NativeControlLoop&) = delete;
    NativeControlLoop& operator=(const NativeControlLoop&) = delete;
    /// 人工开启/停止 停止清除旧目标 开启后仍需新鲜有效目标
    void arm(bool enabled);
    /// 输入世界系绝对角和原始图像时刻 失效不得继续续用旧目标
    void submit(WorldAimResult goal, double frame_stamp_s, double rate_deg_s);
    NativeControlFeedback feedback() const;
    /// @brief 只取 from_s 之前一条及其后的姿态 减少持锁拷贝
    /// @param from_s 一般传 frame_stamp-窗口 必须含 stamp 之前的一条 否则插值缺 before
    std::vector<AttitudeSample> history_since(double from_s) const;

  private:
    void run();
    diesel::platform::link::UsbCdcPort port_;
    diesel::platform::link::RoboLinkHostLink link_;
    mutable std::mutex mutex_;
    NativeControlFeedback feedback_;
    WorldAimResult goal_;
    double frame_stamp_s_ = -1.;
    double rate_deg_s_ = 30.;
    double last_submitted_stamp_s_ = -1.; ///< 判时间戳非单调
    double command_stamp_s_ = -1.;        ///< 指令生成时刻 指令年龄从这里算
    FreshnessPolicy freshness_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

} // namespace diesel_auto_aim
#endif
