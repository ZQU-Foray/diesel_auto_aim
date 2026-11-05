#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <pty.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "diesel_auto_aim/aimer.hpp"
#include "diesel_auto_aim/control_frame_adapter.hpp"
#include "diesel/platform/link/robolink_codec.hpp"
#include "diesel/platform/link/robolink_host_link.hpp"

namespace {

float read_float_le(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    const std::uint32_t bits = static_cast<std::uint32_t>(bytes[offset]) |
                               (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
                               (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
                               (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
    float value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

diesel::platform::link::GimbalAttitude ready_attitude() {
    diesel::platform::link::GimbalAttitude attitude;
    attitude.quaternion_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    attitude.imu_state = 1;
    attitude.attitude_valid = true;
    return attitude;
}

} // namespace

TEST(ControlFrameAdapter, EncodesValidGimbalCommandWithoutFire) {
    const diesel_auto_aim::GimbalCommand command{3.14159265358979323846 / 2.0,
                                                -3.14159265358979323846 / 4.0, true};
    const auto frame =
        diesel_auto_aim::make_gimbal_control_frame(command, ready_attitude(), 21, 3, 4, 1000);
    ASSERT_TRUE(frame.has_value());
    const auto slot = diesel::platform::link::decode_single_slot_frame(*frame);
    ASSERT_TRUE(slot.has_value());
    ASSERT_EQ(slot->payload.size(), 48u);
    EXPECT_EQ(slot->type, 0x20u);
    EXPECT_EQ(slot->sequence, 4u);
    EXPECT_EQ(slot->payload[0], 21u);
    EXPECT_EQ(slot->payload[4], 2u);    // 来源 PC。
    EXPECT_EQ(slot->payload[5], 2u);    // 期望视觉跟踪模式。
    EXPECT_EQ(slot->payload[6], 0x02u); // 只声明云台，不声明开火。
    EXPECT_EQ(slot->payload[7], 0u);
    EXPECT_FLOAT_EQ(read_float_le(slot->payload, 20),
                    static_cast<float>(3.14159265358979323846 / 2.0));
    EXPECT_FLOAT_EQ(read_float_le(slot->payload, 24),
                    static_cast<float>(-3.14159265358979323846 / 4.0));
}

TEST(ControlFrameAdapter, RejectsUntrustedAndNonFiniteCommand) {
    const auto invalid =
        diesel_auto_aim::make_gimbal_control_frame({1.0, 2.0, false}, ready_attitude(), 1, 1, 1, 1);
    EXPECT_FALSE(invalid.has_value());
    const auto nonfinite = diesel_auto_aim::make_gimbal_control_frame(
        {std::numeric_limits<double>::quiet_NaN(), 0.0, true}, ready_attitude(), 1, 1, 1, 1);
    EXPECT_FALSE(nonfinite.has_value());

    auto unavailable = ready_attitude();
    unavailable.attitude_valid = false;
    EXPECT_FALSE(
        diesel_auto_aim::make_gimbal_control_frame({0.0, 0.0, true}, unavailable, 1, 1, 1, 1)
            .has_value());
}

TEST(ControlFrameAdapter, RotatesGimbalDirectionIntoWorldFrame) {
    auto attitude = ready_attitude();
    // 云台相对世界绕 +z 转 90°；云台正前方在世界系中指向 +y。
    attitude.quaternion_wxyz = {0.70710677f, 0.0f, 0.0f, 0.70710677f};
    const auto frame =
        diesel_auto_aim::make_gimbal_control_frame({0.0, 0.0, true}, attitude, 1, 1, 1, 1);
    ASSERT_TRUE(frame.has_value());
    const auto slot = diesel::platform::link::decode_single_slot_frame(*frame);
    ASSERT_TRUE(slot.has_value());
    EXPECT_NEAR(read_float_le(slot->payload, 20), static_cast<float>(3.14159265358979323846 / 2.0),
                1e-5f);
    EXPECT_NEAR(read_float_le(slot->payload, 24), 0.0f, 1e-5f);

    // 同一静止世界方向 +x：云台左转 90° 后，相对目标变成右侧 90°。
    const auto stationary = diesel_auto_aim::make_gimbal_control_frame(
        {-3.14159265358979323846 / 2.0, 0.0, true}, attitude, 2, 2, 2, 2);
    ASSERT_TRUE(stationary.has_value());
    const auto stationary_slot = diesel::platform::link::decode_single_slot_frame(*stationary);
    ASSERT_TRUE(stationary_slot.has_value());
    EXPECT_NEAR(read_float_le(stationary_slot->payload, 20), 0.0f, 1e-5f);

    attitude.imu_state = 0; // IMU 仍在标定，世界系目标不能用。
    EXPECT_FALSE(diesel_auto_aim::make_gimbal_control_frame({0.0, 0.0, true}, attitude, 3, 3, 3, 3)
                     .has_value());
}

TEST(ControlFrameAdapter, WorldPitchUsesDownPositiveConvention) {
    auto attitude = ready_attitude();
    // 绕世界 +y 旋转 30°，云台正前方转向世界 -z，即目标 pitch 下正。
    attitude.quaternion_wxyz = {0.9659258f, 0.0f, 0.2588190f, 0.0f};
    const auto frame =
        diesel_auto_aim::make_gimbal_control_frame({0.0, 0.0, true}, attitude, 1, 1, 1, 1);
    ASSERT_TRUE(frame.has_value());
    const auto slot = diesel::platform::link::decode_single_slot_frame(*frame);
    ASSERT_TRUE(slot.has_value());
    EXPECT_NEAR(read_float_le(slot->payload, 20), 0.0f, 1e-5f);
    EXPECT_NEAR(read_float_le(slot->payload, 24), static_cast<float>(3.14159265358979323846 / 6.0),
                1e-5f);
}

TEST(ControlFrameAdapter, AimResultCanEnterProtocolFrame) {
    // 用真实直瞄函数产生指令，验证自瞄输出确实能进入协议组帧入口。
    const auto command = diesel_auto_aim::aim_at({1.0, 1.0, 0.0}, 0.0);
    const auto frame =
        diesel_auto_aim::make_gimbal_control_frame(command, ready_attitude(), 5, 6, 7, 8);
    ASSERT_TRUE(frame.has_value());
    const auto slot = diesel::platform::link::decode_single_slot_frame(*frame);
    ASSERT_TRUE(slot.has_value());
    EXPECT_NEAR(read_float_le(slot->payload, 20), static_cast<float>(3.14159265358979323846 / 4.0),
                1e-4f);
}

TEST(ControlFrameAdapter, RefusesToQueueBeforeHandshake) {
    diesel::platform::link::UsbCdcPort port("/dev/foray_not_connected");
    diesel::platform::link::RoboLinkHostLink link(port);
    EXPECT_FALSE(diesel_auto_aim::queue_gimbal_command(link, {0.0, 0.0, true}, ready_attitude(), 1));
}

TEST(ControlFrameAdapter, AimToUsbBytesAfterHandshake) {
    int master = -1;
    int slave = -1;
    ASSERT_EQ(::openpty(&master, &slave, nullptr, nullptr, nullptr), 0);
    char path[128]{};
    ASSERT_EQ(::ttyname_r(slave, path, sizeof(path)), 0);
    diesel::platform::link::UsbCdcPort port(path);
    diesel::platform::link::RoboLinkHostLink link(port);

    ASSERT_EQ(link.step(0), diesel::platform::link::HostLinkState::HelloPending);
    std::array<std::uint8_t, 128> bytes{};
    const ssize_t hello_size = ::read(master, bytes.data(), bytes.size());
    ASSERT_GT(hello_size, 0);
    const auto hello =
        diesel::platform::link::decode_single_slot_frame({bytes.begin(), bytes.begin() + hello_size});
    ASSERT_TRUE(hello.has_value());

    diesel::platform::link::HelloAck ack;
    ack.acked_msg_seq = hello->sequence;
    diesel::platform::link::RoboLinkSlot ack_slot;
    ack_slot.type = 0x02;
    ack_slot.flags = 0x0C;
    ack_slot.payload = diesel::platform::link::encode_hello_ack_payload(ack);
    const auto ack_frame = diesel::platform::link::encode_single_slot_frame(1, 1000, ack_slot);
    ASSERT_EQ(::write(master, ack_frame.data(), ack_frame.size()),
              static_cast<ssize_t>(ack_frame.size()));
    ASSERT_EQ(link.step(1), diesel::platform::link::HostLinkState::Ready);

    diesel::platform::link::RoboLinkSlot status_slot;
    status_slot.type = 0x10;
    status_slot.payload.resize(28);
    status_slot.payload[22] = 3;
    const auto status_frame = diesel::platform::link::encode_single_slot_frame(2, 2000, status_slot);
    ASSERT_EQ(::write(master, status_frame.data(), status_frame.size()),
              static_cast<ssize_t>(status_frame.size()));
    ASSERT_EQ(link.step(2), diesel::platform::link::HostLinkState::Active);

    const auto command = diesel_auto_aim::aim_at({1.0, 1.0, 0.0}, 0.0);
    ASSERT_TRUE(diesel_auto_aim::queue_gimbal_command(link, command, ready_attitude(), 3));
    ASSERT_EQ(link.step(3), diesel::platform::link::HostLinkState::Active);
    const ssize_t sent_size = ::read(master, bytes.data(), bytes.size());
    ASSERT_EQ(sent_size, 72);
    const auto control =
        diesel::platform::link::decode_single_slot_frame({bytes.begin(), bytes.begin() + sent_size});
    ASSERT_TRUE(control.has_value());
    EXPECT_EQ(control->type, 0x20u);
    EXPECT_NEAR(read_float_le(control->payload, 20),
                static_cast<float>(3.14159265358979323846 / 4.0), 1e-4f);

    port.close();
    ::close(master);
    ::close(slave);
}
