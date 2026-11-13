#include "diesel_auto_aim/aim_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "diesel_auto_aim/aimer.hpp"
#include "diesel_auto_aim/tracked_aim.hpp"

namespace diesel_auto_aim {
namespace {
bool valid_quaternion(const Eigen::Quaterniond& q) {
    return q.coeffs().allFinite() && std::abs(q.norm() - 1.0) <= 0.002;
}
} // namespace

std::optional<Eigen::Quaterniond> interpolate_attitude(const std::vector<AttitudeSample>& history,
                                                       double stamp) {
    // 旧签名保留 默认窗口与 FreshnessPolicy 初值一致
    return interpolate_attitude(history, stamp, FreshnessPolicy{}.max_attitude_gap_s);
}

std::optional<Eigen::Quaterniond> interpolate_attitude(const std::vector<AttitudeSample>& history,
                                                       double stamp, double max_gap_s) {
    if (!std::isfinite(stamp))
        return std::nullopt;
    const AttitudeSample* before = nullptr;
    for (const auto& item : history) {
        if (!std::isfinite(item.host_received_s) || !valid_quaternion(item.gimbal_to_world))
            return std::nullopt;
        if (before && item.host_received_s < before->host_received_s)
            return std::nullopt;
        if (item.host_received_s >= stamp) {
            if (item.host_received_s == stamp)
                return item.gimbal_to_world.normalized();
            if (!before || item.host_received_s - before->host_received_s > max_gap_s)
                return std::nullopt;
            const double ratio = (stamp - before->host_received_s) /
                                 (item.host_received_s - before->host_received_s);
            return before->gimbal_to_world.normalized()
                .slerp(ratio, item.gimbal_to_world.normalized())
                .normalized();
        }
        before = &item;
    }
    return std::nullopt;
}

TrackerConfig runtime_tracker_config(double radius, double gate) {
    if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(gate) || gate <= 0)
        throw std::invalid_argument("invalid tracker geometry");
    TrackerConfig c{};
    c.radius_m = radius;
    c.armor_count = 4;
    c.association_gate_m = gate;
    c.confirmation_frames = 3;
    c.max_missed_frames = 8;
    c.max_frame_gap_s = .25;
    c.association_strategy = AssociationStrategy::AngularScored;
    c.sensor_origin_world = Eigen::Vector3d::Zero();
    // 沿用前一轮台架实验配置 本次只统一入口 不同时调整滤波器
    c.association_bearing_gate_rad = .12;
    c.association_depth_gate_m = 1.0;
    c.association_yaw_gate_rad = .7;
    c.initial_covariance_diag << .25, 1., .25, 1., .25, 1., 1., 25., .01, .01, .01;
    c.plate_random_walk_m2_per_s = 1e-5;
    c.secondary_plate_noise_scale = 5.;
    return c;
}

AimPipeline::AimPipeline(HandEyeCalibration handeye, AimPipelineConfig config)
    : handeye_(std::move(handeye)), config_(config), tracker_(config.tracker) {
    configure(config);
}

void AimPipeline::configure(AimPipelineConfig config) {
    if (!std::isfinite(config.prediction_s) || config.prediction_s < 0 || config.prediction_s > .2)
        throw std::invalid_argument("prediction must be 0..200ms");
    config_ = config;
    tracker_ = Tracker(config.tracker);
    previous_selected_ = -1;
    last_frame_stamp_s_ = last_associated_s_ = -1.;
}

AimPipelineResult AimPipeline::process_observations(std::vector<Observation> observations,
                                                    double stamp, double now,
                                                    const std::vector<AttitudeSample>& history) {
    AimPipelineResult result;
    result.frame_stamp_s = stamp;
    // 时间门限统一走 FreshnessPolicy 不再写死 0.25
    // 先判非单调再判年龄 这样"回退"和"太旧"在统计里能分开
    if (!std::isfinite(stamp) || !std::isfinite(now)) {
        result.stale = StaleReason::NotFinite;
        result.reason = "vision frame stamp not finite";
        return result;
    }
    if (stamp <= last_frame_stamp_s_) {
        result.stale = StaleReason::ClockBackward;
        result.reason = "vision frame stamp not monotonic";
        return result;
    }
    if (now < stamp) {
        result.stale = StaleReason::ClockBackward;
        result.reason = "vision frame stamp ahead of now";
        return result;
    }
    if (now - stamp > config_.freshness.max_frame_age_s) {
        result.stale = StaleReason::FrameTooOld;
        result.reason = "vision frame too old for decision";
        return result;
    }
    const auto q = interpolate_attitude(history, stamp, config_.freshness.max_attitude_gap_s);
    if (!q) {
        result.stale = StaleReason::AttitudeGap;
        result.reason = "waiting for host-time attitude brackets";
        return result;
    }
    last_frame_stamp_s_ = stamp;
    std::vector<ArmorTrackObservation> tracked_observations;
    std::vector<Eigen::Vector3d> direct;
    for (const auto& observation : observations) {
        if (observation.armor_id < 0 || observation.armor_id > 2)
            continue;
        if (!observation.xyz_camera.allFinite() || observation.xyz_camera.z() <= 0)
            continue;
        if (!std::isfinite(observation.reprojection_error) ||
            observation.reprojection_error < 0 || observation.reprojection_error > 3.)
            continue;
        // 手眼与姿态只在后端转一次：世界系 = q * (R_camera_to_gimbal * p_camera + t)
        const Eigen::Vector3d world = (*q) * (handeye_.rotation * observation.xyz_camera +
                                              handeye_.translation_m);
        direct.push_back(world);
        if (!observation.normal_valid || !observation.normal_camera.allFinite())
            continue;
        const Eigen::Vector3d normal = (*q) * (handeye_.rotation * observation.normal_camera);
        if (!normal.allFinite() || std::hypot(normal.x(), normal.y()) < 1e-6)
            continue;
        tracked_observations.push_back(
            {world, std::atan2(normal.y(), normal.x()), observation.armor_id});
    }
    result.world_observation_count = static_cast<int>(tracked_observations.size());
    if (config_.backend == AimBackend::DirectPnP) {
        if (direct.size() != 1) {
            result.reason = "direct PnP needs exactly one target";
            return result;
        }
        const auto angle = aim_at(direct.front(), 0.);
        result.angles = {angle.yaw, angle.pitch, angle.valid};
        result.valid = angle.valid;
    } else {
        result.tracked = tracker_.step(tracked_observations, stamp);
        const auto& tracked = *result.tracked;
        if (!tracked.predicted_only && tracked.associated_observation_index >= 0)
            last_associated_s_ = stamp;
        const bool short_lost = tracked.state == TrackerState::TempLost &&
                                last_associated_s_ >= 0 && stamp - last_associated_s_ <= .08;
        if (!tracked.target || (tracked.state != TrackerState::Tracking && !short_lost)) {
            if (tracked.state == TrackerState::Lost)
                previous_selected_ = -1;
            result.reason = "tracker not confirmed or lost";
            return result;
        }
        const auto selection = select_aim_armor(*tracked.target, Eigen::Vector3d::Zero(),
                                                config_.prediction_s, previous_selected_);
        if (!selection.valid()) {
            result.reason = "no facing armor selected";
            return result;
        }
        WorldAimRequest request;
        request.target = *tracked.target;
        request.armor_id = selection.armor_id;
        request.observation_stamp_s = stamp;
        request.prediction_s = config_.prediction_s;
        result.angles = DirectWorldAimSolver().solve(request);
        result.valid = result.angles.valid;
        result.selected_plate = selection.armor_id;
        if (result.valid)
            previous_selected_ = selection.armor_id;
    }
    if (!result.valid || std::abs(result.angles.pitch_rad) >= 70. * M_PI / 180.) {
        result.valid = false;
        result.reason = "invalid aim direction";
    } else
        result.reason = "valid";
    return result;
}

WorldAimResult advance_world_aim(const WorldAimResult& goal, const WorldAimResult& previous,
                                 double dt, double rate) {
    if (!goal.valid || !previous.valid || !std::isfinite(dt) || !std::isfinite(rate) || rate <= 0 ||
        rate > 180 || !std::isfinite(goal.yaw_rad) || !std::isfinite(goal.pitch_rad) ||
        !std::isfinite(previous.yaw_rad) || !std::isfinite(previous.pitch_rad) ||
        std::abs(goal.pitch_rad) >= 70. * M_PI / 180.)
        return {};
    const double step = rate * M_PI / 180. * std::clamp(dt, 0., .05);
    const double dy = std::remainder(goal.yaw_rad - previous.yaw_rad, 2. * M_PI);
    return {std::remainder(previous.yaw_rad + std::clamp(dy, -step, step), 2. * M_PI),
            previous.pitch_rad + std::clamp(goal.pitch_rad - previous.pitch_rad, -step, step),
            true};
}

} // namespace diesel_auto_aim
