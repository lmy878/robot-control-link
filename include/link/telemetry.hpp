#pragma once
#include <cstring>
#include "link/common.hpp"

namespace robotlink {

// Uplink application packet: exactly 340 bytes (little-endian, packed).
//  5 quaternions   = 5 * 4 * 4 =  80 B
//  15 laser ranges = 15 * 4    =  60 B
//  50 robot states = 50 * 4    = 200 B
//                                -----
//                                 340 B
// Sent by the robot node at 200 Hz on the telemetry stream.
constexpr std::size_t kTelemetrySize = 340;
constexpr int kQuatCount  = 5;
constexpr int kLaserCount = 15;
constexpr int kStateCount = 50;

#pragma pack(push, 1)
struct TelemetryPacket {
    f32 quaternions[kQuatCount][4];   // [w, x, y, z] per body: base + 4 arms
    f32 laser[kLaserCount];           // range measurements (m)
    f32 states[kStateCount];          // robot state vector (joints, vel, ...)
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == kTelemetrySize, "TelemetryPacket must be 340 bytes");

std::size_t encodeTelemetry(const TelemetryPacket& p, u8* out, std::size_t cap);
bool decodeTelemetry(const u8* in, std::size_t len, TelemetryPacket& p);

} // namespace robotlink
