#pragma once
#include "link/common.hpp"
#include "link/tlv.hpp"

namespace robotlink {

constexpr std::size_t kMaxJoints = 16;

struct VelocityCmd { f32 vx = 0, vy = 0, wz = 0; };

struct MoveOrder {
    MoveDir dir = MoveDir::Stop;
    u8  speed = 0;        // 0..255
    u16 duration_ms = 0;  // 0 = until next order
};

struct ArmJoints {
    u8  arm_id = 0;
    u8  count = 0;
    f32 joints[kMaxJoints]{};   // target radians
};

struct ArmPose {
    u8  arm_id = 0;
    f32 x = 0, y = 0, z = 0;
    f32 roll = 0, pitch = 0, yaw = 0;
};

struct BaseTelemetry {
    f32 x = 0, y = 0, yaw = 0;
    f32 vx = 0, vy = 0, wz = 0;
    u8  battery = 0;
};

struct ArmTelemetry {
    u8  arm_id = 0;
    u8  count = 0;
    f32 joints[kMaxJoints]{};
};

struct StatusTelem { u8 mode = 0; u16 faults = 0; u8 estop = 0; };

struct AckMsg { u16 ack_seq = 0; u8 status = 0; };

struct ArmMode { u8 arm_id = 0; u8 mode = 0; };

struct ParamSet { u16 param_id = 0; f32 value = 0; };

struct EventLog {
    u8  level = 0;                 // 0=debug 1=info 2=warn 3=error
    u8  text_len = 0;
    u8  text[64]{};
};

struct ConnectReqMsg { u16 version = kVersion; u32 device_id = 0; u32 nonce = 0; };

struct ConnectAckMsg { u32 session_id = 0; u8 status = 0; u16 capabilities = 0; };

// TLV tags (unique names per message kind, values chosen per message)
enum : u8 {
    // SetVelocity
    VEL_VX = 1, VEL_VY = 2, VEL_WZ = 3,
    // MoveOrder
    MOV_DIR = 1, MOV_SPEED = 2, MOV_DUR = 3,
    // ArmJoints / ArmTelemetry
    ARM_ID = 1, ARM_COUNT = 2, ARM_JOINTS = 3,
    // ArmPose
    POSE_ARM = 1, POSE_X = 2, POSE_Y = 3, POSE_Z = 4,
    POSE_ROLL = 5, POSE_PITCH = 6, POSE_YAW = 7,
    // BaseTelemetry
    BT_X = 1, BT_Y = 2, BT_YAW = 3, BT_VX = 4, BT_VY = 5, BT_WZ = 6, BT_BATT = 7,
    // Status
    ST_MODE = 1, ST_FAULTS = 2, ST_ESTOP = 3,
    // Ack / Nack
    ACK_SEQ = 1, ACK_STAT = 2,
    // ArmMode
    MODE_ARM = 1, MODE_VALUE = 2,
    // ParamSet
    PARAM_ID = 1, PARAM_VAL = 2,
    // EventLog
    LOG_LEVEL = 1, LOG_TEXT = 2,
    // ConnectReq
    CREQ_VERSION = 1, CREQ_DEVICE = 2, CREQ_NONCE = 3,
    // ConnectAck
    CACK_SESSION = 1, CACK_STATUS = 2, CACK_CAPS = 3,
};

// ---- encode ----
std::size_t encodeVelocity(const VelocityCmd& c, u8* out, std::size_t cap);
std::size_t encodeMoveOrder(const MoveOrder& c, u8* out, std::size_t cap);
std::size_t encodeArmJoints(const ArmJoints& c, u8* out, std::size_t cap);
std::size_t encodeArmPose(const ArmPose& c, u8* out, std::size_t cap);
std::size_t encodeBaseTelemetry(const BaseTelemetry& t, u8* out, std::size_t cap);
std::size_t encodeArmTelemetry(const ArmTelemetry& t, u8* out, std::size_t cap);
std::size_t encodeStatus(const StatusTelem& t, u8* out, std::size_t cap);
std::size_t encodeAck(const AckMsg& a, u8* out, std::size_t cap);
std::size_t encodeArmMode(const ArmMode& m, u8* out, std::size_t cap);
std::size_t encodeParamSet(const ParamSet& p, u8* out, std::size_t cap);
std::size_t encodeEventLog(const EventLog& e, u8* out, std::size_t cap);
std::size_t encodeConnectReq(const ConnectReqMsg& c, u8* out, std::size_t cap);
std::size_t encodeConnectAck(const ConnectAckMsg& c, u8* out, std::size_t cap);
// ---- decode ----
bool decodeVelocity(const u8* in, std::size_t len, VelocityCmd& c);
bool decodeMoveOrder(const u8* in, std::size_t len, MoveOrder& c);
bool decodeArmJoints(const u8* in, std::size_t len, ArmJoints& c);
bool decodeArmPose(const u8* in, std::size_t len, ArmPose& c);
bool decodeBaseTelemetry(const u8* in, std::size_t len, BaseTelemetry& t);
bool decodeArmTelemetry(const u8* in, std::size_t len, ArmTelemetry& t);
bool decodeStatus(const u8* in, std::size_t len, StatusTelem& t);
bool decodeAck(const u8* in, std::size_t len, AckMsg& a);
bool decodeArmMode(const u8* in, std::size_t len, ArmMode& m);
bool decodeParamSet(const u8* in, std::size_t len, ParamSet& p);
bool decodeEventLog(const u8* in, std::size_t len, EventLog& e);
bool decodeConnectReq(const u8* in, std::size_t len, ConnectReqMsg& c);
bool decodeConnectAck(const u8* in, std::size_t len, ConnectAckMsg& c);

} // namespace robotlink
