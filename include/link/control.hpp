#pragma once
#include <cstring>
#include "link/common.hpp"

namespace robotlink {

// Downlink application packet: joystick + emergency stop + robot mode.
// Sent by the controller at 100 Hz on the control stream (reliable).
constexpr std::size_t kControlSize = 20;
constexpr int kJoystickAxes = 4;      // x, y, z (altitude), yaw

#pragma pack(push, 1)
struct ControlPacket {
    f32 joystick[kJoystickAxes];   // normalized stick axes, [-1, 1]
    u8  estop;                     // 1 = emergency stop (latched by robot)
    u8  mode;                      // robot mode (0=idle 1=manual 2=auto ...)
    u8  buttons;                   // button bitfield
    u8  reserved;                  // alignment / future use
};
#pragma pack(pop)

static_assert(sizeof(ControlPacket) == kControlSize, "ControlPacket must be 20 bytes");

std::size_t encodeControl(const ControlPacket& p, u8* out, std::size_t cap);
bool decodeControl(const u8* in, std::size_t len, ControlPacket& p);

} // namespace robotlink
