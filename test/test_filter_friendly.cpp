#include <gtest/gtest.h>

#include "diesel_vision_backend/target_candidate.hpp"

using namespace diesel_vision_backend;

// 1 只保留敌人
TEST(FilterFriendly, 只保留敌人) {
    std::vector<TargetCandidate> input = {{3, Side::Friendly, Source::OwnCamera, 0.9},
                                          {7, Side::Enemy, Source::OwnCamera, 0.8}};

    auto out = filter_friendly(input);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].id, 7);
}
// 2
TEST(FilterFriendly, 全友军返回) {
    std::vector<TargetCandidate> input = {
        {3, Side::Friendly, Source::OwnCamera, 0.9},
        {4, Side::Friendly, Source::AllyShared, 0.7},
    };

    auto out = filter_friendly(input);

    ASSERT_TRUE(out.empty());
}
// 3
TEST(FilterFriendly, 返回敌人未知排除) {
    std::vector<TargetCandidate> input = {
        {3, Side::Enemy, Source::OwnCamera, 0.9},
        {5, Side::Unknown, Source::OwnCamera, 0.8},
    };

    auto out = filter_friendly(input);

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].id, 3);
}
// 4
TEST(FilterFriendly, 全未知返回空) {
    std::vector<TargetCandidate> input = {
        {5, Side::Unknown, Source::OwnCamera, 0.5},
        {6, Side::Unknown, Source::AllyShared, 0.4},
    };
    auto out = filter_friendly(input);
    ASSERT_TRUE(out.empty());
}
// 5
TEST(FilterFriendly, 输入空无输出) {
    std::vector<TargetCandidate> input = {

    };
    auto out = filter_friendly(input);
    ASSERT_TRUE(out.empty());
}
// 6
TEST(FilterFriendly, 敌人在前的顺序输入敌人友军输出敌人) {
    std::vector<TargetCandidate> input = {
        {7, Side::Enemy, Source::OwnCamera, 0.6},
        {1, Side::Enemy, Source::AllyShared, 0.8},
        {3, Side::Friendly, Source::OwnCamera, 0.9},
    };
    auto out = filter_friendly(input);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_EQ(out[0].id, 7);
    EXPECT_EQ(out[1].id, 1);
}
// 7
TEST(FilterFriendly, 敌人与友军穿插顺序输入敌人友军输出敌人) {
    std::vector<TargetCandidate> input = {
        {7, Side::Enemy, Source::OwnCamera, 0.8},
        {3, Side::Friendly, Source::OwnCamera, 0.8},
        {1, Side::Enemy, Source::AllyShared, 0.6},
    };
    auto out = filter_friendly(input);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_EQ(out[0].id, 7);
    EXPECT_EQ(out[1].id, 1);
}
