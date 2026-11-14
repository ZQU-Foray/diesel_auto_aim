#ifndef DIESEL_VISION_BACKEND_TARGET_CANDIDATE_HPP
#define DIESEL_VISION_BACKEND_TARGET_CANDIDATE_HPP

#include <cstdint>
#include <vector>

namespace diesel_vision_backend {

/// 敌我判定结果。
/// Unknown 必须保留：IFF 取保守取向（宁可漏报，不可误报），不确定的目标由过滤器丢弃。
enum class Side {
    Enemy,    ///< 敌方
    Friendly, ///< 我方
    Unknown   ///< 无法判定
};

/// 目标信息来源。
enum class Source {
    OwnCamera, // 本机相机
    AllyShared // 队友共享（英雄 / 步兵之间）
    // TODO: SentryShared —— 3V3 规则中哨兵不允许多机通信，待与组长确认后启用
};

/// 候选目标：尚未锁定为整车的单个观测目标。
struct TargetCandidate {
    uint8_t id;        ///< 目标编号（机器人编号）
    Side side;         ///< 敌我判定
    Source source;     ///< 信息来源
    double confidence; ///< 置信度，取值 [0, 1]
};

/// @brief 过滤掉友军与敌我不明的候选目标（友军不得进入候选集）。
/// @param candidates 候选目标列表
/// @return 仅含确认敌方目标的列表，保持输入的相对顺序
std::vector<TargetCandidate> filter_friendly(const std::vector<TargetCandidate>& candidates);

} // namespace diesel_vision_backend

#endif // DIESEL_VISION_BACKEND_TARGET_CANDIDATE_HPP
