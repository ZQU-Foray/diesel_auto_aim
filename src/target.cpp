#include "diesel_auto_aim/target.hpp"

#include <cmath>

namespace diesel_auto_aim {

namespace {

// 过程噪声 加速度方差和角加速度方差
// 测试的靶车不转动 角度速度极小 防止偏移(靶车测试阶段)
constexpr double v1 = 1.0;  // 加速度方差 靶车测试没那么灵活 方差小 sp默认100.0
constexpr double v2 = 1e-6; // 角速度方差

// 观测噪声 位置误差约 1cm → (0.01)² = 1e-4
// 现在测试的相机不同 待新相机标定后替换为实测值
constexpr double kMeasVar = 1e-4;
// 车体的装甲板认板子的距离们限 滤波器未收敛的时候有误差
// 而且r需要根据实际情况调整再算(后面实际测会改)
constexpr double kAssociateMaxDist = 0.5;

// NIS 卡方门限 dof维度为3(xyz) 取地95% = 7.815
// 来自sp的nis 数据来自概率论的卡方分布临界值表
constexpr double kNisThreshold = 7.815;

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

void Target::predict(double dt) {
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
// xyz_measured 来自solver观察的实际装甲板位置
// 在内部完成数据关联，避免调用方绕过认板逻辑
void Target::update(const Eigen::Vector3d& xyz_measured) {
    const int armor_id = associate(xyz_measured).id;
    if (armor_id == -1) {
        return;
    }
    // 用当前状态计算出第armor_id块板子在哪里
    const Eigen::Vector3d xyz_predicted = h_armor_xyz(x_, armor_id);
    // 敏感度矩阵：状态改变一点，预测位置会变化多少
    const Eigen::Matrix<double, 3, 11> H = h_jacobian(x_, armor_id);
    // 观测减去预测
    const Eigen::Vector3d residual = xyz_measured - xyz_predicted;
    // 观测噪声矩阵（重投影误差），用于 NIS
    const Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity() * kMeasVar;
    // 新息协方差
    const Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R;
    // 马氏距离
    const double nis = residual.transpose() * S.inverse() * residual;
    if (nis > kNisThreshold)
        return; // 野值 丢掉这一帧

    const Eigen::Matrix<double, 11, 3> K =
        P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse(); // 跟一维的一样 计算k
    x_ += K * residual;
    x_[6] = limit_rad(x_[6]); // 上一步 角度有影响重新限制
    const Eigen::Matrix<double, 11, 11> I = Eigen::Matrix<double, 11, 11>::Identity();
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
}
// 观测的装甲板 对比四个由状态中心解算出来的装甲板 得出是哪一个装甲板
AssociationResult Target::associate(const Eigen::Vector3d& xyz_measured) const {
    AssociationResult result;
    // 野值默认-1 空值也是-1
    result.id = -1;
    double best_dist = kAssociateMaxDist; // 以野值为阈值
    for (int id = 0; id < armor_num_; ++id) {
        auto predicted_armor_xyz =
            h_armor_xyz(x_, id); // x_输入的是目标整车状态 根据目标整车状态来计算的
        auto dist = (xyz_measured - predicted_armor_xyz).norm();
        if (dist < best_dist) {
            best_dist = dist;
            result.id = id;
        }
    }
    result.dist = best_dist;
    return result;
}

} // namespace diesel_auto_aim
