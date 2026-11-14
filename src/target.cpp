#include "diesel_vision_backend/target.hpp"

#include <cmath>

namespace diesel_vision_backend {

namespace {

// 观测噪声 位置误差约 1cm → (0.01)² = 1e-4
// 现在测试的相机不同 待新相机标定后替换为实测值
constexpr double kMeasVar = 1e-4;

double limit_rad(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

} // namespace

Target::Target(const Eigen::Vector3d& xyz0, double a0, double r0,
               const Eigen::Matrix<double, 11, 1>& p0_diag, int armor_num)
    : armor_num_(armor_num) {
    x_.setZero();
    // 顺序 x vx y vy z vz a w r l h
    // 为了滤波效果 按照靶车 来放参数 后面可以调整 测试文件有写
    P_ = p0_diag.asDiagonal();
    x_[0] = xyz0.x();
    x_[2] = xyz0.y();
    x_[4] = xyz0.z();
    x_[6] = a0;
    x_[8] = r0;
    // 因为现在是靶车测试 1 3 5 7 为速度和角速度 暂时不考虑
    // 靶车的装甲板相对 h l 为 0
}

void Target::set_process_noise(double acceleration_variance, double angular_acceleration_variance) {
    if (std::isfinite(acceleration_variance) && acceleration_variance > 0.0 &&
        std::isfinite(angular_acceleration_variance) && angular_acceleration_variance > 0.0) {
        acceleration_variance_ = acceleration_variance;
        angular_acceleration_variance_ = angular_acceleration_variance;
    }
}

void Target::predict(double dt) {
    const double v1 = acceleration_variance_;
    const double v2 = angular_acceleration_variance_;
    // 1 F矩阵
    const double a = dt * dt * dt * dt / 4;
    const double b = dt * dt * dt / 2;
    const double c = dt * dt;
    // 参考sp的写法但是跑了clang之后 没那么好看
    // clang-format off
    Eigen::Matrix<double, 11, 11> F{
        {1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};

    // Q
    Eigen::Matrix<double, 11, 11> Q{{a * v1, b * v1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {b * v1, c * v1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, a * v1, b * v1, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, b * v1, c * v1, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, a * v1, b * v1, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, b * v1, c * v1, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, a * v2, b * v2, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, b * v2, c * v2, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    // clang-format on
    // r / l / h 的随机游走：默认 0 时保持旧行为（三者完全冻结，配置错了就永远改不回）。
    // 实车几何不会突变，但标定有误差、板面安装有公差，留一点"我还可能猜错"的余地。
    if (plate_random_walk_ > 0.0) {
        const double q_plate = dt * plate_random_walk_;
        Q(8, 8) = q_plate;
        Q(9, 9) = q_plate;
        Q(10, 10) = q_plate;
    }
    // 预测
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

TargetState Target::state() const {
    TargetState s;
    s.xyz = Eigen::Vector3d(x_[0], x_[2], x_[4]);
    s.vxyz = Eigen::Vector3d(x_[1], x_[3], x_[5]);
    s.a = x_[6];
    s.w = x_[7];
    s.r = x_[8];
    s.l = x_[9];
    s.h = x_[10];
    return s;
}

Eigen::Vector3d Target::predicted_armor_xyz(int armor_id) const {
    return h_armor_xyz(x_, armor_id);
}

// 由整车状态算第 id 块装甲板的位置
Eigen::Vector3d Target::h_armor_xyz(const Eigen::Matrix<double, 11, 1>& x, int armor_id) const {
    // angle 是板朝向 4 块板分别偏 0 90 180 270
    const double angle = limit_rad(x[6] + armor_id * 2 * M_PI / armor_num_);
    // 只有 1 3 号板是长边 才用 l 与 h
    const bool use_l_h = (armor_num_ == 4) && (armor_id == 1 || armor_id == 3);

    const double r = (use_l_h) ? x[8] + x[9] : x[8];
    // 由车体中心推装甲板中心
    const double armor_x = x[0] - r * std::cos(angle);
    const double armor_y = x[2] - r * std::sin(angle);
    const double armor_z = (use_l_h) ? x[4] + x[10] : x[4];

    return {armor_x, armor_y, armor_z};
}

Eigen::Matrix<double, 3, 11> Target::h_jacobian(const Eigen::Matrix<double, 11, 1>& x,
                                                int armor_id) const {
    // angle 是板朝向 4 块板分别偏 0 90 180 270
    const double angle = limit_rad(x[6] + armor_id * 2 * M_PI / armor_num_);
    // 只有 1 3 号板是长边 才用 l 与 h 靶车 l h 为 0 暂不影响
    const bool use_l_h = (armor_num_ == 4) && (armor_id == 1 || armor_id == 3);

    const double r = (use_l_h) ? x[8] + x[9] : x[8];
    // 板位置对朝向a(不是加速度) 板子绕着中心转动
    const double dx_da = r * std::sin(angle);
    const double dy_da = -r * std::cos(angle);
    // 板位置对半径r求偏导
    const double dx_dr = -std::cos(angle);
    const double dy_dr = -std::sin(angle);
    const double dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
    const double dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;
    // 板位置对半径差 l / 高度差 h 的偏导 只有长边短边(1 3 号板)才用
    const double dz_dh = (use_l_h) ? 1.0 : 0.0;
    // clang-format off
  Eigen::Matrix<double, 3, 11> H {
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh}
  };
    // clang-format on
    return H;
}
// xyz_measured 来自solver观察的实际装甲板位置 armor_id 第几个板子
void Target::update(const Eigen::Vector3d& xyz_measured, int armor_id) {
    const Eigen::Vector3d xyz_predicted =
        h_armor_xyz(x_, armor_id); // 用当前状态计算出第armor_id块板子在哪里
    const Eigen::Matrix<double, 3, 11> H =
        h_jacobian(x_, armor_id); // 敏感度矩阵 状态改变一点 预测位置会变化多少
    const Eigen::Vector3d residual = xyz_measured - xyz_predicted; // 观测减去预测
    const Eigen::Matrix<double, 3, 3> R =
        position_measurement_covariance(xyz_measured); // 观测噪声 重投影误差
    const Eigen::Matrix<double, 11, 3> K =
        P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse(); // 跟一维的一样 计算k
    x_ += K * residual;
    x_[6] = limit_rad(x_[6]); // 上一步 角度有影响重新限制
    const Eigen::Matrix<double, 11, 11> I = Eigen::Matrix<double, 11, 11>::Identity();
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
}

void Target::update(const Eigen::Vector3d& xyz_measured, int armor_id, double yaw_measured,
                    double yaw_variance, double noise_scale) {
    // 四维观测：板中心 xyz + 对应板的朝向。仅用 xyz 时，原地转车可能被解释成车体平移。
    Eigen::Matrix<double, 4, 11> H = Eigen::Matrix<double, 4, 11>::Zero();
    H.topRows<3>() = h_jacobian(x_, armor_id);
    H(3, 6) = 1.0;
    Eigen::Matrix<double, 4, 1> residual;
    residual.head<3>() = xyz_measured - h_armor_xyz(x_, armor_id);
    residual[3] = limit_rad(yaw_measured - (x_[6] + armor_id * 2.0 * M_PI / armor_num_));
    const double scale = (std::isfinite(noise_scale) && noise_scale > 0.0) ? noise_scale : 1.0;
    Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();
    R.topLeftCorner<3, 3>() = scale * scale * position_measurement_covariance(xyz_measured);
    R(3, 3) = scale * scale * yaw_variance;
    const Eigen::Matrix<double, 11, 4> K =
        P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    x_ += K * residual;
    x_[6] = limit_rad(x_[6]);
    const Eigen::Matrix<double, 11, 11> I = Eigen::Matrix<double, 11, 11>::Identity();
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
}

// 位置观测协方差：沿视线方向（深度）与垂直视线方向（横向）分开给。
// 单目 PnP 的深度误差 ≈ sigma_px * z² / (fx * 板宽)，横向误差 ≈ sigma_px * z / fx，
// 两者差一个 z/板宽 的因子，距离越远差得越多；各向同性 R 会让滤波器把测距偏差
// 误当成车体平移，这是"车心被拖着走"的主要来源。
Eigen::Matrix3d Target::position_measurement_covariance(const Eigen::Vector3d& xyz_measured) const {
    if (!sensor_origin_world_ || sigma_lateral_m_ <= 0.0 || sigma_range_coeff_ <= 0.0)
        return Eigen::Matrix3d::Identity() * kMeasVar; // 未配置：保持旧行为

    const Eigen::Vector3d offset = xyz_measured - *sensor_origin_world_;
    const double range = offset.norm();
    if (!std::isfinite(range) || range < 1e-3)
        return Eigen::Matrix3d::Identity() * kMeasVar;

    const Eigen::Vector3d u = offset / range; // 视线方向
    const Eigen::Matrix3d tangent =
        Eigen::Matrix3d::Identity() - u * u.transpose(); // 垂直于视线的平面
    const double sigma_depth = sigma_range_coeff_ * range * range;
    return sigma_lateral_m_ * sigma_lateral_m_ * tangent +
           sigma_depth * sigma_depth * (u * u.transpose());
}

void Target::set_position_noise_model(const Eigen::Vector3d& sensor_origin_world,
                                      double sigma_lateral_m, double sigma_range_coeff) {
    if (!sensor_origin_world.allFinite() || !std::isfinite(sigma_lateral_m) ||
        !std::isfinite(sigma_range_coeff) || sigma_lateral_m <= 0.0 || sigma_range_coeff <= 0.0) {
        clear_position_noise_model();
        return;
    }
    sensor_origin_world_ = sensor_origin_world;
    sigma_lateral_m_ = sigma_lateral_m;
    sigma_range_coeff_ = sigma_range_coeff;
}

void Target::clear_position_noise_model() {
    sensor_origin_world_.reset();
    sigma_lateral_m_ = 0.0;
    sigma_range_coeff_ = 0.0;
}

double Target::range_sigma_coeff(double sigma_px, double focal_length_px, double board_width_m) {
    if (!std::isfinite(sigma_px) || !std::isfinite(focal_length_px) ||
        !std::isfinite(board_width_m) || sigma_px <= 0.0 || focal_length_px <= 0.0 ||
        board_width_m <= 0.0)
        return 0.0;
    return sigma_px / (focal_length_px * board_width_m);
}

void Target::set_plate_random_walk(double var_m2_per_s) {
    plate_random_walk_ = (std::isfinite(var_m2_per_s) && var_m2_per_s > 0.0) ? var_m2_per_s : 0.0;
}

void Target::seed_height_delta(double height_delta_m, double variance_m2) {
    if (!std::isfinite(height_delta_m))
        return;
    x_[10] = height_delta_m;
    if (std::isfinite(variance_m2) && variance_m2 > 0.0)
        P_(10, 10) = variance_m2;
}

bool Target::radius_in_range(double lower_m, double upper_m) const {
    const double r = x_[8];
    const double alternate = x_[8] + x_[9];
    return std::isfinite(r) && std::isfinite(alternate) && r > lower_m && r < upper_m &&
           alternate > lower_m && alternate < upper_m;
}

} // namespace diesel_vision_backend
