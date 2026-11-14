#ifndef FORAY_AUTO_AIM_TRACKER_HPP
#define FORAY_AUTO_AIM_TRACKER_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "diesel_auto_aim/target.hpp"

namespace diesel_auto_aim {

/// 每块板的三维观测。xyz 与 yaw 必须在同一个稳定坐标系，不能直接传随云台旋转的坐标。
struct ArmorTrackObservation {
    Eigen::Vector3d xyz;
    double yaw;
    int robot_id; ///< 目标类别：-1 无目标，0 哨兵，1 步兵，2 重装；不是具体机器人编号
};

enum class AssociationStrategy { ForayGated, SpNearestThree, AngularScored };

/// 按目标类别选择建轨先验；板尺寸属于 PnP 配置，不在这里重复定义。
struct TargetInitProfile {
    double radius_m;
    int armor_count;
    Eigen::Matrix<double, 11, 1> initial_covariance_diag;
};

struct TrackerConfig {
    // 默认值必须让 TrackerConfig{} 通过 Tracker 的构造校验：默认构造曾留下未初始化的
    // 标量与协方差，任何拿默认配置直接建 AimPipeline 的调用方都会在构造期抛
    // "Invalid tracker configuration"（离线复现：test_aim_pipeline 的非法板号用例）。
    // 这里的数值与 runtime_tracker_config(0.2, 1.0) 保持一致，调用方可整体覆盖。
    AssociationStrategy association_strategy = AssociationStrategy::ForayGated;
    double radius_m = 0.2; ///< 装甲板外接半径（m），标定值与 PnP 配置一致
    int armor_count = 4;
    double association_gate_m = 1.0;
    // 4 板周向间隔 90°；门限必须小于半间距 45°(0.785 rad) 才能唯一确定板号。
    // 1.2 rad(69°) 比半间距还宽，实测会出现"把邻板观测塞给当前板"的静默串板
    // （离线：1.2 rad 时 6/44 次探测串板，0.7 rad 时 0 次）。
    // 模拟器实测（4 rad/s、每帧 0.16 s、双模型角点）：固定 0.7 rad 会让 tracking 帧
    // 从 35/200 掉到 10–17/200，因为新轨迹角速度初值为 0、单帧未建模转角就有 0.65 rad。
    // 因此在"前端还偏"的阶段先用 1.2 rad 保跟踪连续性；等角点修好、单帧转角变小后，
    // 再收到 0.7（此时才真正换来"不串板"的收益）。
    double association_yaw_gate_rad = 1.2; ///< 约 69°；收到 0.7 前请先看模拟器 A/B 数据
    double adjacent_switch_margin_m = 0.0; ///< 两候选距离近似打平时优先相邻板；0 表示关闭
    double yaw_variance_rad2 = 0.04; ///< PnP 朝向噪声方差，默认 0.2 rad 标准差；实测后调整
    int confirmation_frames = 3;
    int max_missed_frames = 8;
    double max_frame_gap_s = 0.25; ///< 前端 30–60 fps，超过 250 ms 视为断流
    /// 初值协方差：位置 0.25、速度 1、角速度 25、几何量 0.01；沿用 rb2025 台架取值。
    Eigen::Matrix<double, 11, 1> initial_covariance_diag =
        (Eigen::Matrix<double, 11, 1>() << .25, 1., .25, 1., .25, 1., 1., 25., .01, .01, .01)
            .finished();
    /// 比赛普通目标采用 SP 的高机动先验；其他场景由调用方按运动日志覆盖。
    double acceleration_variance = 100.0;
    double angular_acceleration_variance = 400.0;
    /// 0 哨兵、1 步兵、2 重装。任一项被设置后，未配置的类别不建轨。
    /// 全部为空时沿用上面的通用参数，供原有仿真和离线测试使用。
    std::array<std::optional<TargetInitProfile>, 3> init_profiles{};

    /// 位置观测噪声的分方向模型（单目 PnP 深度比横向糊得多）。
    /// sensor_origin_world 为空、或两个 sigma 非正时，退回各向同性 R，即旧行为。
    std::optional<Eigen::Vector3d> sensor_origin_world; ///< 相机/云台原点，稳定坐标系
    double sigma_lateral_m = 0.0;   ///< 横向标准差；约 距离×像素噪声/焦距
    double sigma_range_coeff = 0.0; ///< 深度标准差系数：sigma = coeff × 距离²

    /// r/l/h 的随机游走强度 (m²/s)。0 表示三者完全冻结（旧行为）；
    /// 非零后半径/高低差在标定不准时仍有机会收敛。
    double plate_random_walk_m2_per_s = 0.0;

    /// 同帧第二块及以后的板是否也参与更新（0 = 关闭，只更新关联到的第一块，旧行为）。
    /// 值为这些"弱更新"的观测噪声放大倍数，建议 3~10：既让不同板参与更新（r/h 才可观测），
    /// 又不把同一帧的信息重复计数。实测：只更新同一块板时 r 永远不动；
    /// 让不同板参与后 r 能收敛到真值。
    double secondary_plate_noise_scale = 0.0;

    /// 半径合理区间 (m)，用于健康判据；<=0 表示不检查。
    /// 越界立即判 Lost：半径被错误观测拖到 0.14 m 这类情况，宁可从零重建。
    double radius_lower_m = 0.0;
    double radius_upper_m = 0.0;

    /// 建轨初期的朝向门限放宽量。新轨迹的角速度初值是 0，而目标可能每帧转几十度，
    /// 这段"未建模转角"会正好顶到稳态门限上，把 correct 观测挡掉（模拟器实测：
    /// 4 rad/s 且每帧 0.16 s 时，固定 0.7 rad 门限使 tracking 帧从 35 掉到 10–17）。
    /// 前 yaw_gate_warmup_updates 次成功更新用 max(稳态门限, warmup 门限)，
    /// warmup 门限设为 0 或 updates 为 0 表示关闭该机制（退回固定门限）。
    double association_yaw_gate_warmup_rad = 1.2;
    int yaw_gate_warmup_updates = 3;

    /// 方位角门限 (rad)。>0 且提供 sensor_origin_world 时，关联改用"方位角 + 深度"
    /// 两个物理量分别判，而不是一个 3D 距离。0 = 只用 association_gate_m（旧行为）。
    double association_bearing_gate_rad = 0.0;
    /// 深度门限 (m)，<=0 表示只查方位角。前端测距不可信时保持 0：
    /// 拿一个已知有偏的量做门限，只会把正确观测挡掉。
    double association_depth_gate_m = 0.0;
    /// AngularScored：上一板只获得轻微分数优势，不再一旦过门就直接胜出。
    double scored_previous_bias_rad = 0.05;
};

enum class TrackerState { Lost, Detecting, Tracking, TempLost };

/// 离线关联诊断；只报告本帧为什么没有匹配，不改变筛选或更新行为。
struct AssociationDiagnostics {
    enum class Reason {
        None,
        Initialized,
        Matched,
        Empty,
        Invalid,
        RobotId,
        YawGate,
        DistanceGate,
        TimeGap,
        BearingGate,
        DepthGate,
        RadiusOutOfRange
    };
    Reason reason = Reason::None;
    int invalid_count = 0;
    int robot_id_mismatch_count = 0;
    int yaw_gate_count = 0;
    int distance_gate_count = 0;
    int bearing_gate_count = 0; ///< 方位角门限拒绝数（仅分方向门限启用时统计）
    int depth_gate_count = 0;   ///< 深度门限拒绝数
    int secondary_update_count = 0; ///< 本帧额外参与更新的板数（不含主关联板）
    bool height_seeded = false; ///< 初始化时是否用同帧异奇偶板直接解出高低差
    double nearest_distance_m = -1.0;
    double nearest_yaw_error_rad = -1.0;
    double nearest_bearing_error_rad = -1.0; ///< 分方向门限启用时才有意义
    double dt_s = 0.0;
    double primary_position_residual_m = -1.0; ///< 主板更新前，观测与预测板中心的距离
    double primary_center_correction_m = 0.0;   ///< 主板 EKF 更新造成的车心位移
    double secondary_center_correction_m = 0.0; ///< 同帧其余板更新造成的车心总位移
};

struct TrackerResult {
    TrackerState state;
    std::optional<TargetState> target;
    int associated_armor_id = -1;          ///< 本帧未关联时为 -1
    int associated_observation_index = -1; ///< 本帧输入列表中的下标，未关联时为 -1
    /// true 表示本帧没有实测关联，target 是纯预测。调用方可以据此显示"预测"而不是"锁定"。
    bool predicted_only = false;
    AssociationDiagnostics diagnostics; ///< 仅供离线排障，不可直接作为控制可信度
};

/// 单目标跟踪核心；调用者负责敌我过滤、坐标转换和图像/姿态时间对齐。
class Tracker {
  public:
    explicit Tracker(TrackerConfig config);

    /// @param stamp_s 单调递增、与所有观测同源的采样时间（秒），不是处理结束时间
    TrackerResult step(const std::vector<ArmorTrackObservation>& observations, double stamp_s);

    /// 仅供调试显示；在 Lost 时返回空，在其余状态返回四块板的预测位置。
    std::optional<std::array<Eigen::Vector3d, 4>> predicted_armors() const;

    /// 运行期设置位置观测噪声模型（相机/云台原点与标定量）。
    /// 模拟器适配层每帧可用当前帧的 K 与外参位置调用；参数无效时退回各向同性 R。
    void set_position_noise_model(const Eigen::Vector3d& sensor_origin_world,
                                  double sigma_lateral_m, double sigma_range_coeff);

  private:
    TrackerResult result(int associated_armor_id = -1, int associated_observation_index = -1) const;
    void reset();
    bool valid(const ArmorTrackObservation& observation) const;
    /// 把配置里的噪声模型下发到 Target（每新建一条轨迹调用一次）
    void apply_noise_model(Target& target) const;

    TrackerConfig config_;
    TrackerState state_ = TrackerState::Lost;
    std::optional<Target> target_;
    int robot_id_ = -1;
    std::optional<double> last_stamp_s_;
    int detected_frames_ = 0;
    int missed_frames_ = 0;
    int last_armor_id_ = -1;
    int updates_since_init_ = 0; ///< 建轨/重建后成功更新次数，用于门限放宽
    AssociationDiagnostics diagnostics_;
};

} // namespace diesel_auto_aim

#endif // FORAY_AUTO_AIM_TRACKER_HPP
