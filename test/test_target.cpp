#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "diesel_auto_aim/target.hpp"

using namespace diesel_auto_aim;

namespace {

constexpr double kR = 0.2; // 板到中心半径 米
constexpr int kArmorNum = 4;
constexpr double kSigma = 0.01; // 观测噪声标准差 米（1 cm）不是实际相机的 只是用于测试
constexpr int kFrames = 200;
constexpr double kDt = 0.01; // 100 Hz 预测的hz 后面主程序需要调整

// NIS 卡方门限
// 重新计算一遍与c文件的计算实现隔离再验证
Eigen::Vector3d true_armor_xyz(const Eigen::Vector3d& center, double yaw, int id) {
    const double angle = yaw + id * 2.0 * M_PI / kArmorNum;
    return {center.x() - kR * std::cos(angle), center.y() - kR * std::sin(angle), center.z()};
}

} // namespace

TEST(Target, 静止的目标收敛并且误差小于观测误差) {
    // 车静止朝向 0
    const Eigen::Vector3d center{5.0, 0.0, 0.5};
    const double yaw = 0.0;
    const int id = 0;
    // 故意给偏离才能测出滤波效果
    const Eigen::Vector3d init_guess = center + Eigen::Vector3d(0.2, -0.1, 0.0);
    // 按照靶车的来 但可能不准确 以后续实际测试再调试数值 先保证ekf的逻辑和处理没问题
    // 顺序 x vx y vy z vz a w r l h
    // 位置 σ=0.5m 速度 σ=1m/s 朝向 σ=0.2rad 角速度 σ=0.1
    // r l h 按靶车标称已知 给极小方差
    Eigen::Matrix<double, 11, 1> p0_diag{{0.25}, {1.0},  {0.25}, {1.0},  {0.25}, {1.0},
                                         {0.04}, {0.01}, {1e-6}, {1e-6}, {1e-6}};
    Target target(init_guess, yaw, kR, p0_diag, kArmorNum); // 构造

    // 每一次跑 噪声都是这个固定的随机值
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, kSigma);

    double sum_meas_err = 0.0; // 观测误差平方和
    double sum_est_err = 0.0;  // 估计误差平方和

    for (int i = 0; i < kFrames; ++i) {
        // 观测  =  真实位置加上噪声
        Eigen::Vector3d z = true_armor_xyz(center, yaw, id);
        z.x() += noise(gen);
        z.y() += noise(gen);
        z.z() += noise(gen);

        target.predict(kDt); // 更新预测
        target.update(z);    // 更新观测

        sum_meas_err += (z - true_armor_xyz(center, yaw, id))
                            .squaredNorm(); // z是全部加上观测噪声的读数
                                            // 另外一个是没有加噪声直接推出的装甲板数字
        sum_est_err +=
            (target.state().xyz - center).squaredNorm(); // 滤波器输出预测的中心减去真实中心
    }

    EXPECT_LT(std::sqrt(sum_est_err / kFrames),
              std::sqrt(sum_meas_err / kFrames)); // 估计的误差小于观测的误差
}

TEST(Target, 匀速的目标能估计出速度) {
    const Eigen::Vector3d center0{5.0, 0.0, 0.5};
    const Eigen::Vector3d v_true{0.0, 1.0, 0.0}; // y方向 1m/s移动
    const double yaw = 0.0;
    const int id = 0;

    const Eigen::Vector3d init_guess = center0 + Eigen::Vector3d(0.2, -0.1, 0.0);
    // 顺序 x vx y vy z vz a w r l h
    Eigen::Matrix<double, 11, 1> p0_diag{{0.25}, {1.0},  {0.25}, {1.0},  {0.25}, {1.0},
                                         {0.04}, {0.01}, {1e-6}, {1e-6}, {1e-6}};

    Target target(init_guess, yaw, kR, p0_diag, kArmorNum);

    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, kSigma);

    double sum_meas_err = 0.0;
    double sum_est_err = 0.0;
    double sum_v_err = 0.0;
    int n_tail = 0;

    for (int i = 0; i < kFrames; ++i) {
        // 真值 不加噪声
        Eigen::Vector3d center_now = {5.0, center0[1] + v_true[1] * (i * kDt),
                                      0.5}; // 模拟y方向上匀速 中心随着帧移动
        // 观测 装甲板加上噪声
        Eigen::Vector3d z = true_armor_xyz(center_now, yaw, id);
        z.x() += noise(gen);
        z.y() += noise(gen);
        z.z() += noise(gen);

        target.predict(kDt); // 当前时刻的预测
        target.update(z);    // 更新当前时刻的滤波后的值

        // 为了观测实验的结果只看后半段落
        if (i < kFrames / 2) {
            continue;
        }

        sum_meas_err += (z - true_armor_xyz(center_now, yaw, id)).squaredNorm();
        sum_est_err += (target.state().xyz - center_now).squaredNorm();
        sum_v_err += (target.state().vxyz - v_true).squaredNorm();

        ++n_tail;
    }

    const double v_rmse = std::sqrt(sum_v_err / n_tail);
    std::cout << "速度 RMSE = " << v_rmse << " m/s\n";
    EXPECT_LT(v_rmse, 0.05);
    EXPECT_LT(std::sqrt(sum_est_err / n_tail), std::sqrt(sum_meas_err / n_tail));
}
// 这个同目标微微动 靶车的情况
TEST(Target, 四块板轮流出现时中心不跳变) {
    const Eigen::Vector3d center{5.0, 0.0, 0.5}; // 真实值中心
    const double yaw = 0.0;
    const Eigen::Vector3d init_guess = center + Eigen::Vector3d(0.2, -0.1, 0.0);
    Eigen::Matrix<double, 11, 1> p0_diag{{0.25}, {1.0},  {0.25}, {1.0},  {0.25}, {1.0},
                                         {0.04}, {0.01}, {1e-6}, {1e-6}, {1e-6}};

    Target target(init_guess, yaw, kR, p0_diag, kArmorNum);

    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, kSigma);

    double sum_est_err = 0.0;
    int n = 0;
    // 前面的kFrame 只有200 太少了这里单独换成400
    for (int i = 0; i < 400; ++i) {
        // 每50帧率换一块板子 测试换板子(0-1-2-3-0...)
        const int id = (i / 50) % kArmorNum;
        // 观察(加上噪声)
        Eigen::Vector3d z = true_armor_xyz(center, yaw, id);
        z.x() += noise(gen);
        z.y() += noise(gen);
        z.z() += noise(gen);

        target.predict(kDt); // 预测当前的
        target.update(z);    // 更新滤波数值
        // 前面在收敛 统计稍微后一点的 240 而不是 250是为了多验证一个装甲板
        if (i < 240) {
            continue;
        }
        ++n;
        sum_est_err += (target.state().xyz - center).squaredNorm();
    }
    const double rmse = std::sqrt(sum_est_err / n);
    std::cout << "换板时中心 RMSE = " << rmse << " m\n";
    EXPECT_LT(rmse, 0.02);
}

TEST(Target, 认板子可以识别出哪一个装甲板加上野值判断) {
    const Eigen::Vector3d center{5.0, 0.0, 0.5};
    const double yaw = 0.0;
    Eigen::Matrix<double, 11, 1> p0_diag{{0.25}, {1.0},  {0.25}, {1.0},  {0.25}, {1.0},
                                         {0.04}, {0.01}, {1e-6}, {1e-6}, {1e-6}};
    Target target(center, yaw, kR, p0_diag, kArmorNum);
    for (int id = 0; id < kArmorNum; ++id) {
        const auto res = target.associate(true_armor_xyz(center, yaw, id));
        EXPECT_EQ(res.id, id);
    }
    const auto outlier = target.associate(center + Eigen::Vector3d(3.0, 0.0, 0.0));
    EXPECT_EQ(outlier.id, -1);
}

TEST(Target, NIS能排除野值影响观测) {
    const Eigen::Vector3d center{5.0, 0.0, 0.5};
    const Eigen::Vector3d v_true{0.0, 1.0, 0.0}; // y方向 1m/s移动
    const double yaw = 0.0;
    const int id = 0;

    const Eigen::Vector3d init_guess = center + Eigen::Vector3d(0.2, -0.3, 0.0);
    // 顺序 x vx y vy z vz a w r l h
    Eigen::Matrix<double, 11, 1> p0_diag{{0.25}, {1.0},  {0.25}, {1.0},  {0.25}, {1.0},
                                         {0.04}, {0.01}, {1e-6}, {1e-6}, {1e-6}};

    Target target(init_guess, yaw, kR, p0_diag, kArmorNum);

    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, kSigma);

    const Eigen::Vector3d z_normal = true_armor_xyz(center, yaw, 0);
    const Eigen::Vector3d z_outlier = z_normal + Eigen::Vector3d(0.1, 0.0, 0.0); // 野值偏离 0.1 米

    // 先运行一会 让数值收敛
    // NIS按照协方差来缩放 没有收敛的时候的P很大 s很大 这样马氏距离的对应影响小了
    for (int i = 0; i < 100; ++i) {
        target.predict(kDt);
        target.update(z_normal);
    }

    // 预测减去观测 这里是野值
    const Eigen::Vector3d before = target.state().xyz;
    target.update(z_outlier); // 只调 update（不需要 predict）
    const Eigen::Vector3d after = target.state().xyz;
    EXPECT_LT((after - before).norm(), 1e-9) << "野值没被挡住";

    // 正常的观测  误差1cm略有变化
    const Eigen::Vector3d z_normal2 =
        true_armor_xyz(center, yaw, 0) + Eigen::Vector3d(0.01, 0.0, 0.0);
    const Eigen::Vector3d before2 = target.state().xyz;
    target.update(z_normal2);
    const Eigen::Vector3d after2 = target.state().xyz;
    EXPECT_GT((after2 - before2).norm(), 1e-6) << "正常观测被误挡了";
}

TEST(Target, 高机动配置不改变匀速预测位置但会更快接纳角速度变化) {
    Eigen::Matrix<double, 11, 1> p0 = Eigen::Matrix<double, 11, 1>::Constant(1e-4);
    Target slow({5.0, 0.0, 0.5}, 0.0, kR, p0, kArmorNum);
    Target fast({5.0, 0.0, 0.5}, 0.0, kR, p0, kArmorNum);
    slow.set_process_noise(1.0, 1.0);
    fast.set_process_noise(100.0, 400.0);
    slow.predict(0.1);
    fast.predict(0.1);
    EXPECT_NEAR(slow.state().a, fast.state().a, 1e-12);
    EXPECT_NEAR(slow.state().xyz.x(), fast.state().xyz.x(), 1e-12);

    // 突然开始旋转；两者看到同一块板的同一份位置和朝向观测。
    const auto observed = true_armor_xyz({5.0, 0.0, 0.5}, 0.4, 0);
    slow.update(observed, 0, 0.4, 0.04);
    fast.update(observed, 0, 0.4, 0.04);
    EXPECT_GT(fast.state().w, slow.state().w);
}
