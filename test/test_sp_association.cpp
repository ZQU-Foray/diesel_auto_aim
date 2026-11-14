#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "diesel_auto_aim/sp_association.hpp"

namespace {

TEST(SpAssociationTest, SelectsVisibleAdjacentPlate) {
    Eigen::Matrix<double, 11, 1> p0;
    p0.setOnes();
    const diesel_auto_aim::Target target({5.0, 0.0, 0.5}, 0.0, 0.2, p0, 4);
    const diesel_auto_aim::ArmorTrackObservation observation{{5.0, -0.2, 0.5}, M_PI / 2.0, 1};
    const auto match =
        diesel_auto_aim::associate_sp_plate(target, observation, Eigen::Vector3d::Zero());
    ASSERT_TRUE(match.has_value());
    EXPECT_EQ(match->armor_id, 1);
    EXPECT_NEAR(match->score_rad, 0.0, 1e-9);
}

TEST(SpAssociationTest, RejectsNonFiniteObservation) {
    Eigen::Matrix<double, 11, 1> p0;
    p0.setOnes();
    const diesel_auto_aim::Target target({5.0, 0.0, 0.5}, 0.0, 0.2, p0, 4);
    const diesel_auto_aim::ArmorTrackObservation observation{
        {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.5}, 0.0, 1};
    EXPECT_FALSE(diesel_auto_aim::associate_sp_plate(target, observation, Eigen::Vector3d::Zero()));
}

} // namespace
