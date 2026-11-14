#include "diesel_auto_aim/tracker.hpp"
#include "diesel_auto_aim/sp_association.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace diesel_auto_aim {

Tracker::Tracker(TrackerConfig config) : config_(std::move(config)) {
    if (!std::isfinite(config_.radius_m) || config_.radius_m <= 0 || config_.armor_count != 4 ||
        !std::isfinite(config_.association_gate_m) || config_.association_gate_m <= 0 ||
        !std::isfinite(config_.association_yaw_gate_rad) || config_.association_yaw_gate_rad <= 0 ||
        config_.association_yaw_gate_rad >= M_PI ||
        !std::isfinite(config_.adjacent_switch_margin_m) || config_.adjacent_switch_margin_m < 0 ||
        config_.adjacent_switch_margin_m >= config_.association_gate_m ||
        config_.confirmation_frames <= 0 || !std::isfinite(config_.yaw_variance_rad2) ||
        config_.yaw_variance_rad2 <= 0 || config_.max_missed_frames < 0 ||
        !std::isfinite(config_.max_frame_gap_s) || config_.max_frame_gap_s <= 0 ||
        !std::isfinite(config_.sigma_lateral_m) || config_.sigma_lateral_m < 0 ||
        !std::isfinite(config_.sigma_range_coeff) || config_.sigma_range_coeff < 0 ||
        !std::isfinite(config_.plate_random_walk_m2_per_s) ||
        config_.plate_random_walk_m2_per_s < 0 ||
        !std::isfinite(config_.association_yaw_gate_warmup_rad) ||
        config_.association_yaw_gate_warmup_rad < 0 || config_.yaw_gate_warmup_updates < 0 ||
        !std::isfinite(config_.secondary_plate_noise_scale) ||
        config_.secondary_plate_noise_scale < 0 || !std::isfinite(config_.radius_lower_m) ||
        !std::isfinite(config_.radius_upper_m) || config_.radius_lower_m < 0 ||
        config_.radius_upper_m < 0 ||
        (config_.radius_upper_m > 0 && config_.radius_upper_m <= config_.radius_lower_m) ||
        (config_.sensor_origin_world && !config_.sensor_origin_world->allFinite()) ||
        !config_.initial_covariance_diag.allFinite() ||
        (config_.initial_covariance_diag.array() < 0).any() ||
        !std::isfinite(config_.acceleration_variance) || config_.acceleration_variance <= 0.0 ||
        !std::isfinite(config_.angular_acceleration_variance) ||
        config_.angular_acceleration_variance <= 0.0 ||
        !std::isfinite(config_.scored_previous_bias_rad) ||
        config_.scored_previous_bias_rad < 0.0 ||
        (config_.association_strategy == AssociationStrategy::AngularScored &&
         (!config_.sensor_origin_world || !std::isfinite(config_.association_bearing_gate_rad) ||
          config_.association_bearing_gate_rad <= 0.0 ||
          !std::isfinite(config_.association_depth_gate_m) ||
          config_.association_depth_gate_m <= 0.0))) {
        throw std::invalid_argument("Invalid tracker configuration");
    }
    for (const auto& profile : config_.init_profiles) {
        if (profile &&
            (!std::isfinite(profile->radius_m) || profile->radius_m <= 0.0 ||
             profile->armor_count != 4 || !profile->initial_covariance_diag.allFinite() ||
             (profile->initial_covariance_diag.array() < 0).any()))
            throw std::invalid_argument("Invalid target initialization profile");
    }
}

void Tracker::reset() {
    target_.reset();
    state_ = TrackerState::Lost;
    robot_id_ = -1;
    detected_frames_ = 0;
    missed_frames_ = 0;
    last_armor_id_ = -1;
    updates_since_init_ = 0;
}

bool Tracker::valid(const ArmorTrackObservation& observation) const {
    return observation.robot_id >= 0 && observation.robot_id <= 2 && observation.xyz.allFinite() &&
           std::isfinite(observation.yaw);
}

void Tracker::set_position_noise_model(const Eigen::Vector3d& sensor_origin_world,
                                       double sigma_lateral_m, double sigma_range_coeff) {
    config_.sensor_origin_world = sensor_origin_world;
    config_.sigma_lateral_m = sigma_lateral_m;
    config_.sigma_range_coeff = sigma_range_coeff;
    if (target_)
        apply_noise_model(*target_);
}

void Tracker::apply_noise_model(Target& target) const {
    target.set_process_noise(config_.acceleration_variance, config_.angular_acceleration_variance);
    if (config_.sensor_origin_world)
        target.set_position_noise_model(*config_.sensor_origin_world, config_.sigma_lateral_m,
                                        config_.sigma_range_coeff);
    else
        target.clear_position_noise_model();
    target.set_plate_random_walk(config_.plate_random_walk_m2_per_s);
}

TrackerResult Tracker::result(int associated_armor_id, int associated_observation_index) const {
    TrackerResult output{
        state_, std::nullopt, associated_armor_id, associated_observation_index, false, {}};
    // 有估计但本帧没有实测关联：目标只是预测，调用方必须能区分
    output.predicted_only = target_.has_value() && associated_observation_index < 0;
    if (target_)
        output.target = target_->state();
    output.diagnostics = diagnostics_;
    return output;
}

TrackerResult Tracker::step(const std::vector<ArmorTrackObservation>& observations,
                            double stamp_s) {
    diagnostics_ = {};
    if (!std::isfinite(stamp_s)) {
        diagnostics_.reason = AssociationDiagnostics::Reason::TimeGap;
        reset();
        last_stamp_s_.reset();
        return result();
    }
    if (last_stamp_s_) {
        const double dt = stamp_s - *last_stamp_s_;
        diagnostics_.dt_s = dt;
        if (dt <= 0 || dt > config_.max_frame_gap_s) {
            diagnostics_.reason = AssociationDiagnostics::Reason::TimeGap;
            reset();
        } else if (target_)
            target_->predict(dt);
    }
    last_stamp_s_ = stamp_s;

    if (!target_) {
        const bool use_profiles =
            std::any_of(config_.init_profiles.begin(), config_.init_profiles.end(),
                        [](const auto& profile) { return profile.has_value(); });
        for (std::size_t i = 0; i < observations.size(); ++i) {
            const auto& observation = observations[i];
            if (!valid(observation))
                continue;
            if (use_profiles && !config_.init_profiles[observation.robot_id])
                continue;
            const TargetInitProfile fallback{config_.radius_m, config_.armor_count,
                                             config_.initial_covariance_diag};
            const TargetInitProfile& profile =
                use_profiles ? *config_.init_profiles[observation.robot_id] : fallback;
            // 第一块板暂定为局部 id=0；由它的朝向和半径反推车体中心。
            const Eigen::Vector3d offset(profile.radius_m * std::cos(observation.yaw),
                                         profile.radius_m * std::sin(observation.yaw), 0.0);
            target_.emplace(observation.xyz + offset, observation.yaw, profile.radius_m,
                            profile.initial_covariance_diag, profile.armor_count);
            apply_noise_model(*target_);
            // 同帧若还能看到另一块"异奇偶"的板，直接解析算出高低差：
            // h = z(奇号板) - z(偶号板)。只靠滤波器等 1/3 号板被选中才更新，
            // 在"每帧只看到一块板"的场景里可能永远等不到（模拟器实测 h 恒为 0）。
            if (profile.armor_count == 4) {
                for (std::size_t j = 0; j < observations.size(); ++j) {
                    if (j == i)
                        continue;
                    const auto& other = observations[j];
                    if (!valid(other) || other.robot_id != observation.robot_id)
                        continue;
                    // 用朝向差 90° 判定"异奇偶"：按初始板为局部 0 号，异奇偶即 ±90°
                    const double delta =
                        std::abs(std::remainder(other.yaw - observation.yaw, 2.0 * M_PI));
                    if (std::abs(delta - M_PI / 2.0) > config_.association_yaw_gate_rad)
                        continue;
                    const double height_delta = other.xyz.z() - observation.xyz.z();
                    target_->seed_height_delta(height_delta);
                    diagnostics_.height_seeded = true;
                    break;
                }
            }
            robot_id_ = observation.robot_id;
            last_armor_id_ = 0;
            detected_frames_ = 1;
            updates_since_init_ = 0;
            state_ = detected_frames_ >= config_.confirmation_frames ? TrackerState::Tracking
                                                                     : TrackerState::Detecting;
            if (diagnostics_.reason == AssociationDiagnostics::Reason::None)
                diagnostics_.reason = AssociationDiagnostics::Reason::Initialized;
            return result(0, static_cast<int>(i));
        }
        if (diagnostics_.reason == AssociationDiagnostics::Reason::None)
            diagnostics_.reason = observations.empty() ? AssociationDiagnostics::Reason::Empty
                                                       : AssociationDiagnostics::Reason::Invalid;
        return result();
    }

    const ArmorTrackObservation* best = nullptr;
    int best_id = -1;
    int best_index = -1;
    // 分方向门已由 position_matches 检查角度/深度；候选排序不能再偷偷套用
    // 旧 3D 距离上限，否则“方向和深度都合格”仍会因 0.25 m 被拒绝。
    const bool split_gate =
        config_.sensor_origin_world.has_value() && config_.association_bearing_gate_rad > 0.0;
    double best_distance =
        split_gate ? std::numeric_limits<double>::infinity() : config_.association_gate_m;
    const double base_yaw = target_->state().a;
    // 建轨初期放宽朝向门限：此时角速度还停在初值 0，未建模的转角会吃掉稳态门限的余量
    double yaw_gate = config_.association_yaw_gate_rad;
    if (config_.yaw_gate_warmup_updates > 0 && config_.association_yaw_gate_warmup_rad > yaw_gate &&
        updates_since_init_ < config_.yaw_gate_warmup_updates)
        yaw_gate = config_.association_yaw_gate_warmup_rad;
    const auto yaw_matches = [&](const ArmorTrackObservation& observation, int id) {
        const double predicted_yaw = base_yaw + id * 2.0 * M_PI / config_.armor_count;
        return std::abs(std::remainder(observation.yaw - predicted_yaw, 2.0 * M_PI)) < yaw_gate;
    };
    // 位置门限。给了传感器原点且方位角门限为正时，按"方位角 + 深度"两个物理量分别判：
    // 单目深度不可信时仍能用准确的方位角完成关联，也便于诊断到底哪个量失败。
    const auto position_matches = [&](const ArmorTrackObservation& observation, int id) {
        const Eigen::Vector3d predicted = target_->predicted_armor_xyz(id);
        if (!split_gate)
            return (observation.xyz - predicted).norm() < config_.association_gate_m;
        const Eigen::Vector3d observed_ray = observation.xyz - *config_.sensor_origin_world;
        const Eigen::Vector3d predicted_ray = predicted - *config_.sensor_origin_world;
        const double observed_norm = observed_ray.norm();
        const double predicted_norm = predicted_ray.norm();
        if (observed_norm < 1e-6 || predicted_norm < 1e-6)
            return false;
        const double cosine = std::clamp(
            observed_ray.dot(predicted_ray) / (observed_norm * predicted_norm), -1.0, 1.0);
        if (std::acos(cosine) > config_.association_bearing_gate_rad)
            return false;
        if (config_.association_depth_gate_m > 0.0 &&
            std::abs(observed_norm - predicted_norm) > config_.association_depth_gate_m)
            return false;
        return true;
    };
    if (config_.association_strategy == AssociationStrategy::SpNearestThree) {
        const Eigen::Vector3d origin =
            config_.sensor_origin_world.value_or(Eigen::Vector3d::Zero());
        for (std::size_t i = 0; i < observations.size(); ++i) {
            const auto& observation = observations[i];
            if (!valid(observation) || observation.robot_id != robot_id_)
                continue;
            const auto candidate = associate_sp_plate(*target_, observation, origin);
            if (candidate) {
                best = &observation;
                best_id = candidate->armor_id;
                best_index = static_cast<int>(i);
                break;
            }
        }
    } else if (config_.association_strategy == AssociationStrategy::AngularScored) {
        // 同帧全局比较，不按输入顺序挑第一个，也不无条件保留上一块板。
        // 朝向和方位角负责认板；深度只作粗差保护，不参与精细排名。
        double best_score = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < observations.size(); ++i) {
            const auto& observation = observations[i];
            if (!valid(observation) || observation.robot_id != robot_id_)
                continue;
            for (int id = 0; id < config_.armor_count; ++id) {
                const double predicted_yaw = base_yaw + id * 2.0 * M_PI / config_.armor_count;
                const double yaw_error =
                    std::abs(std::remainder(observation.yaw - predicted_yaw, 2.0 * M_PI));
                if (yaw_error >= yaw_gate)
                    continue;
                const Eigen::Vector3d observed_ray = observation.xyz - *config_.sensor_origin_world;
                const Eigen::Vector3d predicted_ray =
                    target_->predicted_armor_xyz(id) - *config_.sensor_origin_world;
                const double observed_range = observed_ray.norm();
                const double predicted_range = predicted_ray.norm();
                if (observed_range < 1e-6 || predicted_range < 1e-6)
                    continue;
                const double cosine =
                    std::clamp(observed_ray.dot(predicted_ray) / (observed_range * predicted_range),
                               -1.0, 1.0);
                const double bearing_error = std::acos(cosine);
                if (bearing_error > config_.association_bearing_gate_rad ||
                    std::abs(observed_range - predicted_range) > config_.association_depth_gate_m)
                    continue;
                const double score =
                    yaw_error + bearing_error -
                    (id == last_armor_id_ ? config_.scored_previous_bias_rad : 0.0);
                if (score < best_score) {
                    best_score = score;
                    best = &observation;
                    best_id = id;
                    best_index = static_cast<int>(i);
                }
            }
        }
    } else {
        // 先检查上一帧锁定的板。只要它仍在门限内，就不因另一块板偶然更靠近而跳板。
        for (std::size_t i = 0; i < observations.size(); ++i) {
            const auto& observation = observations[i];
            if (!valid(observation) || observation.robot_id != robot_id_)
                continue;
            if (last_armor_id_ >= 0 && yaw_matches(observation, last_armor_id_) &&
                position_matches(observation, last_armor_id_)) {
                const double distance =
                    (observation.xyz - target_->predicted_armor_xyz(last_armor_id_)).norm();
                if (distance < best_distance) {
                    best = &observation;
                    best_id = last_armor_id_;
                    best_index = static_cast<int>(i);
                    best_distance = distance;
                }
            }
        }
        if (!best) {
            const ArmorTrackObservation* adjacent_best = nullptr;
            int adjacent_id = -1;
            int adjacent_index = -1;
            double adjacent_distance =
                split_gate ? std::numeric_limits<double>::infinity() : config_.association_gate_m;
            for (std::size_t i = 0; i < observations.size(); ++i) {
                const auto& observation = observations[i];
                if (!valid(observation) || observation.robot_id != robot_id_)
                    continue;
                for (int id = 0; id < config_.armor_count; ++id) {
                    if (!yaw_matches(observation, id) || !position_matches(observation, id))
                        continue;
                    const double distance =
                        (observation.xyz - target_->predicted_armor_xyz(id)).norm();
                    if (distance < best_distance) {
                        best = &observation;
                        best_id = id;
                        best_index = static_cast<int>(i);
                        best_distance = distance;
                    }
                    const bool adjacent =
                        last_armor_id_ >= 0 &&
                        (id == (last_armor_id_ + 1) % config_.armor_count ||
                         id == (last_armor_id_ + config_.armor_count - 1) % config_.armor_count);
                    if (adjacent && distance < adjacent_distance) {
                        adjacent_best = &observation;
                        adjacent_id = id;
                        adjacent_index = static_cast<int>(i);
                        adjacent_distance = distance;
                    }
                }
            }
            // 只在位置残差几乎打平时保留旋转方向上的邻板连续性；明显更差时不覆盖几何证据。
            if (config_.adjacent_switch_margin_m > 0 && best && adjacent_best &&
                adjacent_distance <= best_distance + config_.adjacent_switch_margin_m) {
                best = adjacent_best;
                best_id = adjacent_id;
                best_index = adjacent_index;
            }
        }
    }

    if (best) {
        diagnostics_.reason = AssociationDiagnostics::Reason::Matched;
        diagnostics_.primary_position_residual_m =
            (best->xyz - target_->predicted_armor_xyz(best_id)).norm();
        const Eigen::Vector3d center_before_primary = target_->state().xyz;
        target_->update(best->xyz, best_id, best->yaw, config_.yaw_variance_rad2);
        diagnostics_.primary_center_correction_m =
            (target_->state().xyz - center_before_primary).norm();
        const Eigen::Vector3d center_before_secondary = target_->state().xyz;
        // 同帧其余板做"弱更新"：只有不同板参与，r/l/h 才可观测。
        // 只用一块板时会退化成"单车心与半径的简并"，半径永远学不动。
        // 角度评分的同帧多板分配留到第 5 步决定；此处不混用旧的位置匹配做弱更新。
        if (config_.secondary_plate_noise_scale > 0.0 &&
            config_.association_strategy != AssociationStrategy::AngularScored) {
            std::array<bool, 4> used{};
            used[static_cast<std::size_t>(best_id)] = true;
            for (std::size_t i = 0; i < observations.size(); ++i) {
                if (static_cast<int>(i) == best_index)
                    continue;
                const auto& observation = observations[i];
                if (!valid(observation) || observation.robot_id != robot_id_)
                    continue;
                if (config_.association_strategy == AssociationStrategy::SpNearestThree) {
                    const Eigen::Vector3d origin =
                        config_.sensor_origin_world.value_or(Eigen::Vector3d::Zero());
                    const auto candidate = associate_sp_plate(*target_, observation, origin);
                    if (!candidate)
                        continue;
                    ++diagnostics_.secondary_update_count;
                    target_->update(observation.xyz, candidate->armor_id, observation.yaw,
                                    config_.yaw_variance_rad2, config_.secondary_plate_noise_scale);
                    continue;
                }
                int matched_id = -1;
                double matched_distance = split_gate ? std::numeric_limits<double>::infinity()
                                                     : config_.association_gate_m;
                for (int id = 0; id < config_.armor_count; ++id) {
                    if (used[static_cast<std::size_t>(id)])
                        continue;
                    if (!yaw_matches(observation, id) || !position_matches(observation, id))
                        continue;
                    const double distance =
                        (observation.xyz - target_->predicted_armor_xyz(id)).norm();
                    if (distance < matched_distance) {
                        matched_distance = distance;
                        matched_id = id;
                    }
                }
                if (matched_id < 0)
                    continue;
                used[static_cast<std::size_t>(matched_id)] = true;
                ++diagnostics_.secondary_update_count;
                target_->update(observation.xyz, matched_id, observation.yaw,
                                config_.yaw_variance_rad2, config_.secondary_plate_noise_scale);
            }
        }
        diagnostics_.secondary_center_correction_m =
            (target_->state().xyz - center_before_secondary).norm();
        last_armor_id_ = best_id;
        ++updates_since_init_;
        missed_frames_ = 0;
        // 健康判据：半径被错误观测拖出合理区间时，轨迹已经不可用。
        // 门限只保证"这一帧看起来合理"，管不了"更新之后状态是否还合理"。
        if (config_.radius_lower_m > 0.0 && config_.radius_upper_m > 0.0 &&
            !target_->radius_in_range(config_.radius_lower_m, config_.radius_upper_m)) {
            diagnostics_.reason = AssociationDiagnostics::Reason::RadiusOutOfRange;
            reset();
            last_stamp_s_ = stamp_s;
            return result();
        }
        if (state_ == TrackerState::Detecting) {
            ++detected_frames_;
            if (detected_frames_ >= config_.confirmation_frames)
                state_ = TrackerState::Tracking;
        } else {
            state_ = TrackerState::Tracking;
        }
        return result(best_id, best_index);
    }

    // 只在关联失败时复算门控原因；不参与上方的目标选择。
    for (const auto& observation : observations) {
        if (!valid(observation)) {
            ++diagnostics_.invalid_count;
            continue;
        }
        if (observation.robot_id != robot_id_) {
            ++diagnostics_.robot_id_mismatch_count;
            continue;
        }
        double nearest_yaw = M_PI;
        double nearest_distance = std::numeric_limits<double>::infinity();
        bool any_yaw_match = false;
        for (int id = 0; id < config_.armor_count; ++id) {
            const double predicted_yaw = base_yaw + id * 2.0 * M_PI / config_.armor_count;
            const double yaw_error =
                std::abs(std::remainder(observation.yaw - predicted_yaw, 2.0 * M_PI));
            nearest_yaw = std::min(nearest_yaw, yaw_error);
            if (yaw_error < yaw_gate) {
                any_yaw_match = true;
                nearest_distance = std::min(
                    nearest_distance, (observation.xyz - target_->predicted_armor_xyz(id)).norm());
            }
        }
        if (diagnostics_.nearest_yaw_error_rad < 0 ||
            nearest_yaw < diagnostics_.nearest_yaw_error_rad)
            diagnostics_.nearest_yaw_error_rad = nearest_yaw;
        if (!any_yaw_match) {
            ++diagnostics_.yaw_gate_count;
        } else if (split_gate) {
            // 分方向门限：把"方向不对"和"远近不对"分开统计，便于区分是前端深度偏还是朝向偏
            bool bearing_ok = false;
            double nearest_bearing = M_PI;
            double nearest_depth = std::numeric_limits<double>::infinity();
            double nearest_range = -1.0;
            for (int id = 0; id < config_.armor_count; ++id) {
                const double predicted_yaw = base_yaw + id * 2.0 * M_PI / config_.armor_count;
                if (std::abs(std::remainder(observation.yaw - predicted_yaw, 2.0 * M_PI)) >=
                    yaw_gate)
                    continue;
                const Eigen::Vector3d predicted = target_->predicted_armor_xyz(id);
                const Eigen::Vector3d observed_ray = observation.xyz - *config_.sensor_origin_world;
                const Eigen::Vector3d predicted_ray = predicted - *config_.sensor_origin_world;
                const double observed_norm = observed_ray.norm();
                const double predicted_norm = predicted_ray.norm();
                if (observed_norm < 1e-6 || predicted_norm < 1e-6)
                    continue;
                const double cosine = std::clamp(
                    observed_ray.dot(predicted_ray) / (observed_norm * predicted_norm), -1.0, 1.0);
                const double bearing = std::acos(cosine);
                nearest_bearing = std::min(nearest_bearing, bearing);
                nearest_depth = std::min(nearest_depth, std::abs(observed_norm - predicted_norm));
                if (bearing <= config_.association_bearing_gate_rad) {
                    bearing_ok = true;
                    nearest_range = observed_norm;
                }
            }
            if (nearest_bearing < M_PI)
                diagnostics_.nearest_bearing_error_rad = nearest_bearing;
            if (!bearing_ok) {
                ++diagnostics_.bearing_gate_count;
            } else if (config_.association_depth_gate_m > 0.0 &&
                       nearest_depth > config_.association_depth_gate_m) {
                ++diagnostics_.depth_gate_count;
                if (nearest_range > 0)
                    diagnostics_.nearest_distance_m = nearest_range;
            } else {
                ++diagnostics_.distance_gate_count;
            }
        } else {
            ++diagnostics_.distance_gate_count;
            if (diagnostics_.nearest_distance_m < 0 ||
                nearest_distance < diagnostics_.nearest_distance_m)
                diagnostics_.nearest_distance_m = nearest_distance;
        }
    }
    if (diagnostics_.reason == AssociationDiagnostics::Reason::None) {
        if (observations.empty())
            diagnostics_.reason = AssociationDiagnostics::Reason::Empty;
        else if (diagnostics_.bearing_gate_count)
            diagnostics_.reason = AssociationDiagnostics::Reason::BearingGate;
        else if (diagnostics_.depth_gate_count)
            diagnostics_.reason = AssociationDiagnostics::Reason::DepthGate;
        else if (diagnostics_.distance_gate_count)
            diagnostics_.reason = AssociationDiagnostics::Reason::DistanceGate;
        else if (diagnostics_.yaw_gate_count)
            diagnostics_.reason = AssociationDiagnostics::Reason::YawGate;
        else if (diagnostics_.robot_id_mismatch_count)
            diagnostics_.reason = AssociationDiagnostics::Reason::RobotId;
        else
            diagnostics_.reason = AssociationDiagnostics::Reason::Invalid;
    }

    if (state_ == TrackerState::Detecting)
        reset();
    else if (++missed_frames_ > config_.max_missed_frames)
        reset();
    else
        state_ = TrackerState::TempLost;
    return result();
}

std::optional<std::array<Eigen::Vector3d, 4>> Tracker::predicted_armors() const {
    if (!target_)
        return std::nullopt;
    std::array<Eigen::Vector3d, 4> positions;
    for (int id = 0; id < config_.armor_count; ++id)
        positions[static_cast<std::size_t>(id)] = target_->predicted_armor_xyz(id);
    return positions;
}

} // namespace diesel_auto_aim
