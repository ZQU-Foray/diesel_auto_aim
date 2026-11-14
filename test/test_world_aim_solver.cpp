#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "diesel_auto_aim/world_aim_solver.hpp"

using namespace diesel_auto_aim;

TEST(WorldAimSolverTest, DirectWorldAngleAndOrigin) {
    WorldAimRequest request;
    request.target.xyz = Eigen::Vector3d(4.17, 1.0, 0.5);
    request.target.vxyz.setZero();
    request.target.a = request.target.w = request.target.l = request.target.h = 0.0;
    request.target.r = 0.17;
    request.armor_id = 0;
    request.gimbal_origin_world = Eigen::Vector3d(1.0, 0.0, 0.0);
    const DirectWorldAimSolver solver;
    const auto result = solver.solve(request);
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.yaw_rad, std::atan2(1.0, 3.0), 1e-9);
    EXPECT_NEAR(result.pitch_rad, -std::atan2(0.5, std::sqrt(10.0)), 1e-9);
    request.bullet_speed_m_s = 20.0;
    EXPECT_DOUBLE_EQ(result.pitch_rad, solver.solve(request).pitch_rad); // 直瞄不偷做弹道
    request.armor_id = -1;
    EXPECT_FALSE(solver.solve(request).valid);
    request.armor_id = 0;
    request.observation_stamp_s = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(solver.solve(request).valid);
}

TEST(WorldAimSolverTest, TeammateImplementationCanBeTestedWithoutHardware) {
    class TestSolver final : public WorldAimSolver {
      public:
        WorldAimResult solve(const WorldAimRequest&) const override {
            return {0.1, -0.2, true}; // 只验证替换边界 不是弹道实现
        }
    } solver;
    const WorldAimSolver& interface = solver;
    EXPECT_DOUBLE_EQ(interface.solve(WorldAimRequest{}).yaw_rad, 0.1);
}
