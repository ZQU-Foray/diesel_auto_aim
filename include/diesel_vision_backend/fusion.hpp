#ifndef DIESEL_VISION_BACKEND_FUSION_HPP
#define DIESEL_VISION_BACKEND_FUSION_HPP

#include <vector>

#include "diesel_vision_backend/observation.hpp"

namespace diesel_vision_backend {

/// 融合层参数。默认值对应 30–60 fps 前端与「同一块板多源上报」的现场情况。
struct FusionConfig {
    /// 观测年龄上限（秒）。晚于它的观测直接丢弃，不参与本帧。
    /// 取 100 ms：三个前端帧周期，再晚就说明链路或时钟有问题，宁可少一个观测。
    double max_age_s = 0.1;
    /// 同一块板的判定距离（米）。两个来源报告的同一块板位置差通常 < 0.1 m；
    /// 超过 0.15 m 更可能是相邻板，不能合并。
    double same_armor_merge_m = 0.15;
    /// 只保留装甲板类别（3v3 裁剪只实装装甲板检测器）。
    bool armor_only = true;
};

/// 单帧融合结果与丢弃计数，便于离线诊断「为什么这帧没有观测」。
struct FusionResult {
    std::vector<Observation> observations; ///< 按 armor_id 升序，稳定可复现
    int dropped_stale = 0;                 ///< 过期/未来戳丢弃数
    int dropped_class = 0;                 ///< 非装甲板类别丢弃数
    int dropped_invalid = 0;               ///< 非法板号/非有限几何丢弃数
    int merged = 0;                        ///< 被合并进同一块板的多余观测数
};

/// 融合一帧来自多个来源的观测。
///
/// 规则（顺序即优先级）：
///   1. 时间：stamp 非有限、晚于 now、或早于 now-max_age_s → 丢弃；
///   2. 类别：armor_only 时只保留 TargetClass::Armor；
///   3. 几何：位置/法向必须有限，armor_id 必须在 0..2；
///   4. 去重：同板号且距离小于 same_armor_merge_m 的观测合并为一条，
///      自车相机优先，其次置信度高者；不平均位置——平均会把两车的标定误差混进来。
FusionResult fuse_observations(const std::vector<Observation>& observations, double now_s,
                               const FusionConfig& config);

} // namespace diesel_vision_backend

#endif
