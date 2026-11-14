#ifndef DIESEL_VISION_BACKEND_AIM_PIPELINE_HPP
#define DIESEL_VISION_BACKEND_AIM_PIPELINE_HPP

#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "diesel_vision_backend/freshness_policy.hpp"
#include "diesel_vision_backend/fusion.hpp"
#include "diesel_vision_backend/observation.hpp"
#include "diesel_vision_backend/tracker.hpp"
#include "diesel_vision_backend/world_aim_solver.hpp"
#include "diesel_vision_backend/world_snapshot.hpp"

namespace diesel_vision_backend {

// 候选手眼外参 相机点乘R再加t得到云台点 只在这里转换一次
// 标定文件由装配入口读取（读文件属前端/装配职责），本仓只接收结果。
struct HandEyeCalibration {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation_m = Eigen::Vector3d::Zero();
    std::string status;
    std::string warning;
    std::string control_scope;
};

struct AttitudeSample {
    double host_received_s = 0.0;
    Eigen::Quaterniond gimbal_to_world = Eigen::Quaterniond::Identity();
};

/// 主机接收时刻两侧SLERP 间隔最多30ms 不外推 不等于MCU采样/曝光精确同步
std::optional<Eigen::Quaterniond> interpolate_attitude(const std::vector<AttitudeSample>& history,
                                                       double frame_stamp_s);

/// 同上 但插值区间超过 max_gap_s 时返回空 便于调用方区分"缺样本"和"间隔过大"
std::optional<Eigen::Quaterniond> interpolate_attitude(const std::vector<AttitudeSample>& history,
                                                       double frame_stamp_s, double max_gap_s);

enum class AimBackend { DirectPnP, TrackerEkf };
struct AimPipelineConfig {
    TrackerConfig tracker;
    AimBackend backend = AimBackend::TrackerEkf;
    double prediction_s = 0.0; // 显式实验提前量 不暗中添加未知延迟或飞行时间
    FreshnessPolicy freshness; // 时间新鲜度门限 与 NativeControlLoop 共用同一份
    FusionConfig fusion; // 多源观测融合参数（来源优先级、过期门限、同板合并）
};

/// 当前台架的实验Tracker配置 由装配入口注入实测半径与关联门
TrackerConfig runtime_tracker_config(double radius_m, double gate_m);

struct AimPipelineResult {
    bool valid = false;
    std::string reason;
    StaleReason stale = StaleReason::None; ///< 时间门限的拒绝原因 供回放统计
    double attitude_stamp_s = 0.0; ///< 实际查姿态用的时刻 便于核对偏移是否生效
    double frame_stamp_s = 0.0;
    WorldAimResult angles;
    std::optional<TrackerResult> tracked;
    WorldSnapshotData snapshot; ///< 本帧世界模型快照；无轨迹时 valid=false
    int fused_count = 0;        ///< 融合后留下的观测数
    int fused_dropped = 0;      ///< 融合阶段丢弃数（过期/类别/非法）
    int fused_merged = 0;       ///< 被判为同一块板而合并的观测数
    int selected_plate = -1;
    int world_observation_count = 0;
};

/// 直瞄调用层：只消费前端逐帧观测，不打开相机、不跑检测、不做弹道或发射。
/// 颜色/队别过滤在前端完成；本层只按板号与几何有效性筛观测。
class AimPipeline {
  public:
    AimPipeline(HandEyeCalibration handeye, AimPipelineConfig config);
    /// 改组别/颜色等参数会清除旧轨迹 调用方必须先撤销旧控制
    void configure(AimPipelineConfig config);
    /// 统一观测入口：前端检测结果按同一主机单调秒打戳后送入
    AimPipelineResult process_observations(std::vector<Observation> observations,
                                           double frame_stamp_s, double now_s,
                                           const std::vector<AttitudeSample>& history);

  private:
    HandEyeCalibration handeye_;
    AimPipelineConfig config_;
    Tracker tracker_;
    int previous_selected_ = -1;
    double last_frame_stamp_s_ = -1.0;
    double last_associated_s_ = -1.0;
};

/// rad域限速 不实现电机加速度观测器/前馈 实验范围(0,180]度每秒
WorldAimResult advance_world_aim(const WorldAimResult& goal, const WorldAimResult& previous,
                                 double dt_s, double rate_deg_s);

} // namespace diesel_vision_backend
#endif
