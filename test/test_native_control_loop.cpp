#include "diesel/platform/link/robolink_frame_parser.hpp"
#include "diesel_vision_backend/native_control_loop.hpp"
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <pty.h>
#include <thread>
#include <unistd.h>

using namespace diesel_vision_backend;
using namespace diesel::platform::link;
namespace {
double now_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
void u32(std::vector<std::uint8_t>& data, std::size_t off, std::uint32_t value) {
    for (int i = 0; i < 4; ++i)
        data[off + i] = static_cast<std::uint8_t>(value >> (8 * i));
}
} // namespace
TEST(NativeControlLoop, CppUsbHandshakeObserveStartExpireRecoverStopWithoutFire) {
    int master = -1, slave = -1;
    char name[128]{};
    ASSERT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0);
    fcntl(master, F_SETFL, O_NONBLOCK);
    {
        NativeControlLoop control(name);
        RoboLinkFrameParser parser;
        std::vector<RoboLinkSlot> controls;
        std::uint16_t seq = 0;
        std::uint32_t stamp = 100;
        bool fault = false;
        HandEyeCalibration h;
        h.rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        AimPipelineConfig pipeline_config;
        pipeline_config.tracker = runtime_tracker_config(.17, .25);
        AimPipeline pipeline(h, pipeline_config);
        auto send = [&](RoboLinkSlot slot) {
            const auto bytes = encode_single_slot_frame(seq++, stamp, slot);
            EXPECT_EQ(::write(master, bytes.data(), bytes.size()),
                      static_cast<ssize_t>(bytes.size()));
        };
        auto pump = [&](double seconds, bool fresh_goal) {
            const double end = now_s() + seconds;
            while (now_s() < end) {
                std::uint8_t buffer[4096];
                const auto n = ::read(master, buffer, sizeof(buffer));
                if (n > 0)
                    for (const auto& frame : parser.feed(buffer, static_cast<std::size_t>(n))) {
                        const auto slots = decode_frame(frame);
                        if (!slots)
                            continue;
                        for (const auto& slot : *slots) {
                            if (slot.type == 1) {
                                HelloAck ack;
                                ack.acked_msg_seq = slot.sequence;
                                send({2, 0x0C, 0, encode_hello_ack_payload(ack)});
                            } else if (slot.type == 0x20)
                                controls.push_back(slot);
                        }
                    }
                std::vector<std::uint8_t> status(28, 0);
                u32(status, 0, stamp);
                status[22] = 3;
                u32(status, 24, fault ? 1 : 0);
                send({0x10, 0, 0, status});
                std::vector<std::uint8_t> pose(36, 0);
                u32(pose, 0, stamp++);
                const float one = 1.;
                std::memcpy(pose.data() + 4, &one, 4);
                pose[32] = 1;
                pose[33] = 1;
                send({0x30, 0, 0, pose});
                if (fresh_goal) {
                    // 同一条C++链路：模拟前端Observation→候选变换→Tracker/EKF→选板→直瞄→真实协议收发
                    Observation observation;
                    observation.armor_id = 1;
                    observation.reprojection_error = .1;
                    observation.xyz_camera = Eigen::Vector3d(-.3, -.1, 3.83);
                    observation.normal_camera = Eigen::Vector3d(0, 0, 1);
                    observation.normal_valid = true;
                    const double frame_stamp = now_s() - .01;
                    const auto result = pipeline.process_observations(
                        {observation}, frame_stamp, now_s(), control.feedback().history);
                    control.submit(result.valid ? result.angles : WorldAimResult{}, frame_stamp,
                                   30.);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        };
        pump(.1, false);
        ASSERT_TRUE(control.feedback().ready);
        EXPECT_TRUE(controls.empty()); // 启动只观察
        control.arm(true);
        pump(.15, true);
        ASSERT_TRUE(control.feedback().active);
        ASSERT_FALSE(controls.empty());
        pump(.35, false); // 不再续图像 原始时刻过期 必须撤销
        EXPECT_FALSE(control.feedback().active);
        EXPECT_TRUE(control.feedback().armed);
        EXPECT_EQ(controls.back().payload[6], 0);
        pump(.1, true);
        EXPECT_TRUE(control.feedback().active); // 新图像可恢复
        fault = true;
        pump(.05, true);
        EXPECT_FALSE(control.feedback().ready);
        EXPECT_FALSE(control.feedback().active);
        fault = false;
        pump(.1, true);
        EXPECT_TRUE(control.feedback().active);
        control.arm(false);
        pump(.05, true);
        EXPECT_FALSE(control.feedback().active);
        EXPECT_FALSE(control.feedback().armed);
        for (const auto& c : controls) {
            EXPECT_EQ(c.payload[6] & ~0x02, 0); // 从未使能发射或底盘
            for (int i = 8; i < 20; ++i)
                EXPECT_EQ(c.payload[i], 0);
            for (int i = 28; i < 44; ++i)
                EXPECT_EQ(c.payload[i], 0);
        }
    }
    close(master);
    close(slave);
}
