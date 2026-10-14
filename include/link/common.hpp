#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace robotlink {

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using i32 = std::int32_t;
using f32 = float;

constexpr u8  kMagic0  = 0x52; // 'R'
constexpr u8  kMagic1  = 0x4C; // 'L'
constexpr u8  kVersion = 1;

constexpr std::size_t kHeaderSize = 14;
constexpr std::size_t kMaxPayload = 1024;
constexpr std::size_t kMaxFrame   = kHeaderSize + kMaxPayload + 2; // +2 payload CRC

// ---------------------------------------------------------------- little endian
inline void putU16(u8* p, u16 v) { p[0] = u8(v & 0xFF); p[1] = u8((v >> 8) & 0xFF); }
inline void putU32(u8* p, u32 v) {
    p[0] = u8(v & 0xFF); p[1] = u8((v >> 8) & 0xFF);
    p[2] = u8((v >> 16) & 0xFF); p[3] = u8((v >> 24) & 0xFF);
}
inline u16 getU16(const u8* p) { return u16(p[0]) | u16(u16(p[1]) << 8); }
inline u32 getU32(const u8* p) {
    return u32(p[0]) | (u32(p[1]) << 8) | (u32(p[2]) << 16) | (u32(p[3]) << 24);
}
inline void putF32(u8* p, f32 v) { u32 t; std::memcpy(&t, &v, 4); putU32(p, t); }
inline f32  getF32(const u8* p) { u32 t = getU32(p); f32 v; std::memcpy(&v, &t, 4); return v; }

// sequential readers/writers
inline u8  rdU8 (const u8* p, std::size_t& o) { return p[o++]; }
inline u16 rdU16(const u8* p, std::size_t& o) { u16 v = getU16(p + o); o += 2; return v; }
inline u32 rdU32(const u8* p, std::size_t& o) { u32 v = getU32(p + o); o += 4; return v; }
inline f32 rdF32(const u8* p, std::size_t& o) { f32 v = getF32(p + o); o += 4; return v; }

// ---------------------------------------------------------------- message types
enum class MsgType : u8 {
    // ---- downlink (controller -> robot), 0x00..0x7F ----
    ConnectReq    = 0x01,
    Disconnect    = 0x02,
    Heartbeat     = 0x03,
    SetVelocity   = 0x10,
    MoveOrder     = 0x11,
    Stop          = 0x12,
    EmergencyStop = 0x13,
    SetArmJoints  = 0x14,
    SetArmPose    = 0x15,
    SetArmMode    = 0x16,
    SetParam      = 0x17,
    ControlCmd    = 0x18,   // fixed-size ControlPacket
    // ---- uplink (robot -> controller), 0x80..0xFF ----
    ConnectAck    = 0x81,
    HeartbeatUp   = 0x82,
    TelemetryBase = 0x83,
    TelemetryArm  = 0x84,
    Status        = 0x85,
    EventLog      = 0x88,
    Telemetry     = 0x89,   // fixed-size TelemetryPacket
};

inline bool isDownlink(MsgType t) { return u8(t) <= 0x7F; }

enum Flags : u8 {
    FlagFragStart  = 1u << 0,
    FlagFragMore   = 1u << 1,
    FlagPayloadCrc = 1u << 2,
    FlagEmergency  = 1u << 3,
};

enum StreamId : u8 {
    StreamControl   = 0,   // highest priority
    StreamTelemetry = 1,
    StreamDebug     = 2,   // lowest priority
    StreamSession   = 3,
    StreamCount     = 4,
};

// Lower number = higher priority. Control is highest, debug lowest.
inline int streamPriority(StreamId s) {
    switch (s) {
        case StreamControl:   return 0;
        case StreamTelemetry: return 1;
        case StreamSession:   return 2;
        case StreamDebug:     return 3;
        default:              return 3;
    }
}

enum class MoveDir : u8 {
    Forward = 0, Backward = 1, StrafeLeft = 2,
    StrafeRight = 3, RotateLeft = 4, RotateRight = 5, Stop = 6,
};

} // namespace robotlink
