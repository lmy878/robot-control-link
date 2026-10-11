#pragma once
#include <cstring>
#include "link/common.hpp"

namespace robotlink {

// Uplink application packet: exactly 340 bytes on the wire (little-endian,
// packed). Sent by the robot node at 200 Hz on the telemetry stream.
constexpr std::size_t kTelemetrySize = 340;
constexpr u16 kTelemetryMagic = 0x544D; // "TM"

#pragma pack(push, 1)
struct TelemetryPacket {
    u16 magic;              // 0   kTelemetryMagic
    u16 size;               // 2   = kTelemetrySize
    u32 seq;                // 4
    u64 timestamp_us;       // 8   monotonic microseconds
    f32 pos[3];             // 16  x, y, z          (m)
    f32 vel[3];             // 28  vx, vy, vz       (m/s)
    f32 rpy[3];             // 40  roll, pitch, yaw (rad)
    f32 ang_vel[3];         // 52  wx, wy, wz       (rad/s)
    f32 accel[3];           // 64  ax, ay, az       (m/s^2)
    f32 battery_v;          // 76  (V)
    f32 battery_i;          // 80  (A)
    f32 temps[8];           // 84  (degC) motor/board temperatures
    u8  arm_count;          // 116 number of arms (<=4)
    u8  arm_dof;            // 117 joints per arm (<=6)
    u16 status_flags;       // 118 bitfield (armed, moving, ...)
    f32 arm_joints[4][6];   // 120 joint angles (rad)
    f32 motor_current[6];   // 216 (A)
    f32 motor_rpm[6];       // 240 (rpm)
    u32 faults;             // 264 fault bitmap
    f32 cpu_load;           // 268 (%)
    f32 mem_used_mb;        // 272 (MB)
    f32 loop_rate_hz;       // 276 measured telemetry rate
    u32 uptime_ms;          // 280
    u8  mode;               // 284
    u8  estop;              // 285
    u8  reserved[54];       // 286 -> 340
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == kTelemetrySize, "TelemetryPacket must be 340 bytes");

// Serialize / parse. Both return false on size mismatch.
std::size_t encodeTelemetry(const TelemetryPacket& p, u8* out, std::size_t cap);
bool decodeTelemetry(const u8* in, std::size_t len, TelemetryPacket& p);

} // namespace robotlink
