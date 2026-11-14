#include "diesel_vision_backend/native_control_loop.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace diesel_vision_backend {
namespace {
double monotonic_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
} // namespace
NativeControlLoop::NativeControlLoop(const std::string& device, FreshnessPolicy freshness)
    : port_(device), link_(port_), freshness_(freshness) {
    if (!std::isfinite(freshness_.max_frame_age_s) || freshness_.max_frame_age_s <= 0 ||
        !std::isfinite(freshness_.max_attitude_gap_s) || freshness_.max_attitude_gap_s <= 0 ||
        !std::isfinite(freshness_.max_command_age_s) || freshness_.max_command_age_s <= 0)
        throw std::invalid_argument("invalid freshness policy");
    // 指令门限必须不宽于图像门限 否则流水线放行而控制已过期
    if (freshness_.max_command_age_s > freshness_.max_frame_age_s)
        throw std::invalid_argument("command age must not exceed frame age");
    thread_ = std::thread(&NativeControlLoop::run, this);
}
NativeControlLoop::~NativeControlLoop() {
    stop_ = true;
    if (thread_.joinable())
        thread_.join();
}
void NativeControlLoop::arm(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    feedback_.armed = enabled;
    goal_.valid = false; // 重新开启也必须等待本次的新目标
}
void NativeControlLoop::submit(WorldAimResult goal, double stamp, double rate) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++feedback_.submitted;
    // 时间戳非有限或比上次还早 → 时钟回退 一律作废 不续用旧目标
    const bool backward = !std::isfinite(stamp) || (std::isfinite(last_submitted_stamp_s_) &&
                                                    stamp < last_submitted_stamp_s_);
    if (backward) {
        ++feedback_.rejected_clock_backward;
        feedback_.last_stale = StaleReason::ClockBackward;
        goal_ = WorldAimResult{};
        frame_stamp_s_ = -1.;
        command_stamp_s_ = -1.;
        rate_deg_s_ = rate;
        return;
    }
    last_submitted_stamp_s_ = stamp;
    goal_ = goal;
    frame_stamp_s_ = stamp;
    // ⭐ 指令年龄从"命令生成时刻"算 不是从取帧时刻
    // 取帧到提交之间还有推理耗时(实测~36ms) 若用取帧时刻会把推理时间也算进指令年龄
    command_stamp_s_ = monotonic_s();
    rate_deg_s_ = rate;
}
NativeControlFeedback NativeControlLoop::feedback() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_;
}

std::vector<AttitudeSample> NativeControlLoop::history_since(double from_s) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<AttitudeSample> out;
    if (!std::isfinite(from_s) || feedback_.history.empty())
        return out;
    std::size_t begin = 0;
    while (begin < feedback_.history.size() && feedback_.history[begin].host_received_s < from_s)
        ++begin;
    // 必须保留 from_s 之前那一条 否则插值没有 before 会直接失败
    if (begin > 0)
        --begin;
    out.assign(feedback_.history.begin() + static_cast<std::ptrdiff_t>(begin),
               feedback_.history.end());
    return out;
}
void NativeControlLoop::run() {
    bool was_active = false;
    double next_tx = 0., last_advance = monotonic_s(), last_status = -1.;
    std::optional<std::uint32_t> last_sample, last_status_uptime;
    WorldAimResult previous;
    while (!stop_) {
        const double now = monotonic_s();
        const auto ms = static_cast<std::uint64_t>(now * 1000.);
        try {
            const auto state = link_.step(ms);
            std::lock_guard<std::mutex> lock(mutex_);
            feedback_.error.clear();
            feedback_.link_state = state;
            feedback_.status = link_.latest_status();
            if (feedback_.status &&
                (!last_status_uptime || *last_status_uptime != feedback_.status->uptime_ms)) {
                last_status_uptime = feedback_.status->uptime_ms;
                last_status = now;
            }
            const auto attitude = link_.latest_attitude();
            bool pose_ready = false;
            if (attitude) {
                const auto& a = attitude->attitude;
                Eigen::Quaterniond q(a.quaternion_wxyz[0], a.quaternion_wxyz[1],
                                     a.quaternion_wxyz[2], a.quaternion_wxyz[3]);
                pose_ready = a.attitude_valid && a.imu_state == 1 && q.coeffs().allFinite() &&
                             std::abs(q.norm() - 1.) < .002 && ms >= attitude->received_at_ms &&
                             ms - attitude->received_at_ms <= 100;
                if (pose_ready) {
                    q.normalize();
                    if (!last_sample || *last_sample != a.sample_time_us) {
                        last_sample = a.sample_time_us;
                        // 保留主机收到此姿态的时刻 不将MCU采样时钟当成主机时钟
                        feedback_.history.push_back({attitude->received_at_ms / 1000., q});
                        if (feedback_.history.size() > 2500)
                            feedback_.history.erase(feedback_.history.begin());
                    }
                    const auto r = q.toRotationMatrix();
                    feedback_.measured = {std::atan2(r(1, 0), r(0, 0)),
                                          std::asin(std::clamp(-r(2, 0), -1., 1.)), true};
                } else {
                    feedback_.history.clear();
                    last_sample.reset();
                    feedback_.measured.valid = false;
                }
            }
            // 保留旧实机工具的准入条件 不能因改成C++就绕过故障和电控链路状态
            feedback_.ready = state == diesel::platform::link::HostLinkState::Active &&
                              pose_ready && feedback_.status && feedback_.status->link_state == 3 &&
                              feedback_.status->fault_flags == 0 && last_status >= 0 &&
                              now - last_status <= .1;
            if (state == diesel::platform::link::HostLinkState::Disconnected) {
                feedback_.armed = false;
                feedback_.history.clear();
                last_sample.reset();
                last_status_uptime.reset();
                last_status = -1.;
                goal_.valid = false;
            }
            // 指令新鲜度：超龄一次即作废 不靠 validity_hint_ms 兜底
            if (goal_.valid) {
                // 锁内重新取时刻：submit() 也在锁内写 command_stamp_s_
                // 若用锁外读到的 now 会比它早几微秒 被误判成时钟回退
                const double check_now = monotonic_s();
                const auto stale =
                    classify_age(command_stamp_s_, check_now, freshness_.max_command_age_s);
                if (stale != StaleReason::None) {
                    goal_.valid = false;
                    feedback_.last_stale = stale;
                    // 只在已接管时计入"指令过期"；观察模式每帧提交会逐帧过期
                    // 若也计数会随帧率线性膨胀 让日志看起来像大量错误
                    if (feedback_.armed) {
                        // 时钟回退是真异常；其余只是目标逐帧自然过期 仅作诊断
                        if (stale == StaleReason::ClockBackward)
                            ++feedback_.rejected_clock_backward;
                        else
                            ++feedback_.stale_events;
                    }
                }
            }
            const bool valid_goal = goal_.valid;
            const bool active = feedback_.armed && feedback_.ready && valid_goal;
            if (active) {
                if (!was_active)
                    previous = feedback_.measured;
                previous = advance_world_aim(goal_, previous, was_active ? now - last_advance : .01,
                                             rate_deg_s_);
            }
            last_advance = now;
            feedback_.active = active && previous.valid;
            if (feedback_.active && now >= next_tx) {
                diesel::platform::link::ControlSnapshot snapshot;
                snapshot.source = 2;
                snapshot.mode = 4;
                snapshot.flags = 0x0002;
                snapshot.gimbal_yaw_rad = static_cast<float>(previous.yaw_rad);
                snapshot.gimbal_pitch_rad = static_cast<float>(previous.pitch_rad);
                snapshot.validity_hint_ms = 100; // 与既有台架控制契约一致 不是推理超时
                if (link_.queue_control(snapshot, ms)) {
                    ++feedback_.queued_controls;
                    feedback_.sent = previous;
                    next_tx = now + .01;
                }
            } else if (!feedback_.active && was_active) {
                ++feedback_.command_interruptions; // 真正的控制中断 供可用率统计
                diesel::platform::link::ControlSnapshot withdraw;
                withdraw.source = 2;
                withdraw.mode = 7;
                withdraw.flags = 0;
                withdraw.validity_hint_ms = 100;
                if (!link_.queue_control(withdraw, ms)) {
                    // 本次队列忙就保留待撤销状态 下轮重试 固件100ms有效期仍独立保护
                    feedback_.error = "waiting to queue withdraw";
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                ++feedback_.queued_controls;
            }
            was_active = feedback_.active;
        } catch (const std::exception& error) {
            std::lock_guard<std::mutex> lock(mutex_);
            feedback_.armed = feedback_.active = feedback_.ready = false;
            feedback_.error = error.what();
            goal_.valid = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // 正常退出主动撤销 若USB已断仍由下位机目标超时保护
    if (was_active) {
        diesel::platform::link::ControlSnapshot withdraw;
        withdraw.source = 2;
        withdraw.mode = 7;
        withdraw.flags = 0;
        withdraw.validity_hint_ms = 100;
        for (int i = 0; i < 10; ++i) {
            const auto ms = static_cast<std::uint64_t>(monotonic_s() * 1000.);
            link_.step(ms);
            if (link_.queue_control(withdraw, ms)) {
                link_.step(ms + 1);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}
} // namespace diesel_vision_backend
