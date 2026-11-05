#ifndef FORAY_AUTO_AIM_TARGET_HPP
#define FORAY_AUTO_AIM_TARGET_HPP

#include <Eigen/Dense>

#include <optional>

namespace diesel_auto_aim {

/// 整车状态11维度 参考sp
/// v vx y vy z vz a w r l h
/// a 车体的朝向角度 w 角速度 板到中心半径 l 板间半径差 h 板间高度差

struct TargetState {
    Eigen::Vector3d xyz;  ///< 车体中心 米
    Eigen::Vector3d vxyz; ///< 中心速度 米每秒
    double a;             ///< 朝向 弧度
    double w;             ///< 角速度 弧度每秒
    double r;             ///< 板到中心半径 米
    double l;             ///< 板间半径差 米 靶车等距时为 0
    double h;             ///< 板间高度差 米 靶车等高时为 0
};

/// 整车状态估计器 EKF 匀速模型 装甲板位置观测
class Target {
  public:
    /// @param xyz0 初始中心位置 米
    /// @param a0 初始朝向 弧度 由第一次观测折算
    /// @param r0 板到中心半径 米
    /// @param p0_diag 初始协方差对角 11 维
    /// @param armor_num 装甲板数量 靶车 4
    Target(const Eigen::Vector3d& xyz0, double a0, double r0,
           const Eigen::Matrix<double, 11, 1>& p0_diag, int armor_num);

    /// @brief 预测到当前时刻
    /// @param dt 距上次的时间 秒
    void predict(double dt);

    /// @brief 用一次装甲板观测修正
    /// @param xyz_measured 与跟踪状态相同的稳定坐标系下该装甲板的位置 米
    /// @param armor_id 装甲板编号 0-3
    void update(const Eigen::Vector3d& xyz_measured, int armor_id);

    /// 用板中心与板朝向联合修正；yaw_variance 为朝向观测方差 (rad²)。
    /// 输入 yaw 指向板到车体中心，与构造时 a0 的约定一致。
    /// @param noise_scale 观测噪声放大倍数。同帧第二块及以后的板用 >1 的倍数做弱更新，
    ///        避免把同一帧的信息重复计数（否则会人为压低协方差、产生过收敛）。
    void update(const Eigen::Vector3d& xyz_measured, int armor_id, double yaw_measured,
                double yaw_variance, double noise_scale = 1.0);

    /// 把位置观测噪声从"各方向都是 1 cm"改成"沿视线方向糊、垂直于视线方向准"。
    /// 单目 PnP 的深度误差随距离平方增长，各向同性 R 会让滤波器把测距偏差当成车体平移。
    /// @param sensor_origin_world 相机/云台原点在**同一稳定坐标系**下的位置（不是每帧姿态角）
    /// @param sigma_lateral_m 垂直于视线方向的观测标准差，约等于 距离×像素噪声/焦距
    /// @param sigma_range_coeff 深度标准差系数：sigma_depth = coeff × 距离²，
    ///        由 sigma_px/(fx×板宽) 得到，见 range_sigma_coeff()
    void set_position_noise_model(const Eigen::Vector3d& sensor_origin_world,
                                  double sigma_lateral_m, double sigma_range_coeff);

    /// 清空分方向模型，退回各向同性 kMeasVar（保持旧行为，供对照与单元测试）
    void clear_position_noise_model();

    /// @brief 由标定量算深度噪声系数：sigma_px/(fx*board_width_m)
    static double range_sigma_coeff(double sigma_px, double focal_length_px, double board_width_m);

    /// @brief r、l、h 的随机游走强度 (m²/s)；0 表示沿用旧行为（三者完全冻结）
    void set_plate_random_walk(double var_m2_per_s);

    /// 直接写入高低差 h（m）并同时收紧其方差。
    /// 用途：初始化时若同帧看到两块异奇偶板，可直接算出 h，不必等滤波器慢慢收敛。
    void seed_height_delta(double height_delta_m, double variance_m2 = 1e-4);

    /// @brief r 与 r+l 是否落在合理区间；用于健康判据，不参与滤波
    bool radius_in_range(double lower_m, double upper_m) const;

    /// @brief 当前半径与半径差，便于调用方做诊断而不必解包 TargetState
    double radius() const {
        return x_[8];
    }
    double alternate_radius_delta() const {
        return x_[9];
    }

    /// @brief 当前估计
    TargetState state() const;

    /// @brief 根据当前整车状态预测一块装甲板的位置；与输入观测使用同一稳定坐标系
    Eigen::Vector3d predicted_armor_xyz(int armor_id) const;

  private:
    int armor_num_;                   ///< 装甲板数量
    Eigen::Matrix<double, 11, 1> x_;  ///< 状态 固定尺寸 无堆分配
    Eigen::Matrix<double, 11, 11> P_; ///< 状态协方差 固定尺寸 无堆分配

    /// 分方向位置噪声模型；sensor_origin_world_ 为空时退回各向同性
    std::optional<Eigen::Vector3d> sensor_origin_world_;
    double sigma_lateral_m_ = 0.0;
    double sigma_range_coeff_ = 0.0;
    double plate_random_walk_ = 0.0; ///< r/l/h 随机游走强度 m²/s

    /// @brief 3x3 位置观测协方差：沿视线(r)与垂直视线(t)两个方向分开给
    Eigen::Matrix3d position_measurement_covariance(const Eigen::Vector3d& xyz_measured) const;

    /// @brief 由状态计算第id块板子的位置
    Eigen::Vector3d h_armor_xyz(const Eigen::Matrix<double, 11, 1>& x, int armor_id) const;
    /// @brief 观测雅可比 3x11 观测为装甲板 xyz
    Eigen::Matrix<double, 3, 11> h_jacobian(const Eigen::Matrix<double, 11, 1>& x,
                                            int armor_id) const;
};

} // namespace diesel_auto_aim

#endif
