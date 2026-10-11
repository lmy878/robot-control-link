#pragma once
#include <cstring>
#include "link/common.hpp"

namespace robotlink {

// Downlink application packet: exactly 168 bytes on the wire (little-endian,
// packed). Sent by the controller at 100 Hz on the control stream.
constexpr std::size_t kControlSize = 168;
constexpr u16 kControlMagic = 0x4354; // "CT"

#pragma pack(push, 1)
struct ControlPacket {
    u16 magic;              // 0   kControlMagic
    u16 size;               // 2   = kControlSize
    u32 seq;                // 4
    u64 timestamp_us;       // 8
    f32 vel[3];             // 16  vx, vy, wz body velocity (m/s, m/s, rad/s)
    u8  arm_count;          // 28
    u8  arm_dof;            // 29
    u16 flags;              // 30  bitfield (arm enable, etc.)
    f32 arm_joints[4][6];   // 32  target joint angles (rad)
    u16 command;            // 128 command id (idle/move/stop/...)
    u8  mode;               // 130 control mode
    u8  estop;              // 131 emergency stop (latched by robot)
    u8  reserved[36];       // 132 -> 168
};
#pragma pack(pop)

static_assert(sizeof(ControlPacket) == kControlSize, "ControlPacket must be 168 bytes");

std::size_t encodeControl(const ControlPacket& p, u8* out, std::size_t cap);
bool decodeControl(const u8* in, std::size_t len, ControlPacket& p);

} // namespace robotlink
