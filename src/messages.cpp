#include "link/messages.hpp"

namespace robotlink {
namespace {

bool readJointArray(const u8* v, std::size_t n, u8 count, f32* out) {
    if (n < std::size_t(count) * 4) return false;
    for (u8 i = 0; i < count; ++i) out[i] = getF32(v + 4 * i);
    return true;
}

} // namespace

std::size_t encodeVelocity(const VelocityCmd& c, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.f32v(VEL_VX, c.vx); w.f32v(VEL_VY, c.vy); w.f32v(VEL_WZ, c.wz);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeMoveOrder(const MoveOrder& c, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u8v(MOV_DIR, u8(c.dir)); w.u8v(MOV_SPEED, c.speed); w.u16v(MOV_DUR, c.duration_ms);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeArmJoints(const ArmJoints& c, u8* out, std::size_t cap) {
    if (c.count > kMaxJoints) return 0;
    TlvWriter w(out, cap);
    w.u8v(ARM_ID, c.arm_id);
    w.u8v(ARM_COUNT, c.count);
    w.f32Array(ARM_JOINTS, c.joints, c.count);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeArmPose(const ArmPose& c, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u8v(POSE_ARM, c.arm_id);
    w.f32v(POSE_X, c.x); w.f32v(POSE_Y, c.y); w.f32v(POSE_Z, c.z);
    w.f32v(POSE_ROLL, c.roll); w.f32v(POSE_PITCH, c.pitch); w.f32v(POSE_YAW, c.yaw);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeBaseTelemetry(const BaseTelemetry& t, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.f32v(BT_X, t.x); w.f32v(BT_Y, t.y); w.f32v(BT_YAW, t.yaw);
    w.f32v(BT_VX, t.vx); w.f32v(BT_VY, t.vy); w.f32v(BT_WZ, t.wz);
    w.u8v(BT_BATT, t.battery);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeArmTelemetry(const ArmTelemetry& t, u8* out, std::size_t cap) {
    if (t.count > kMaxJoints) return 0;
    TlvWriter w(out, cap);
    w.u8v(ARM_ID, t.arm_id);
    w.u8v(ARM_COUNT, t.count);
    w.f32Array(ARM_JOINTS, t.joints, t.count);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeStatus(const StatusTelem& t, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u8v(ST_MODE, t.mode); w.u16v(ST_FAULTS, t.faults); w.u8v(ST_ESTOP, t.estop);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeArmMode(const ArmMode& m, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u8v(MODE_ARM, m.arm_id); w.u8v(MODE_VALUE, m.mode);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeParamSet(const ParamSet& p, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u16v(PARAM_ID, p.param_id); w.f32v(PARAM_VAL, p.value);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeEventLog(const EventLog& e, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u8v(LOG_LEVEL, e.level);
    w.bytes(LOG_TEXT, e.text, e.text_len);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeConnectReq(const ConnectReqMsg& c, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u16v(CREQ_VERSION, c.version);
    w.u32v(CREQ_DEVICE, c.device_id);
    w.u32v(CREQ_NONCE, c.nonce);
    return w.ok() ? w.size() : 0;
}

std::size_t encodeConnectAck(const ConnectAckMsg& c, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u32v(CACK_SESSION, c.session_id);
    w.u8v(CACK_STATUS, c.status);
    w.u16v(CACK_CAPS, c.capabilities);
    return w.ok() ? w.size() : 0;
}

// ------------------------------------------------------------------------ decode
bool decodeVelocity(const u8* in, std::size_t len, VelocityCmd& c) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case VEL_VX: any |= TlvReader::asF32(v, n, c.vx); break;
            case VEL_VY: any |= TlvReader::asF32(v, n, c.vy); break;
            case VEL_WZ: any |= TlvReader::asF32(v, n, c.wz); break;
            default: break;
        }
    }
    return any;
}

bool decodeMoveOrder(const u8* in, std::size_t len, MoveOrder& c) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; u8 d = 0; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case MOV_DIR:   any |= TlvReader::asU8(v, n, d); c.dir = MoveDir(d); break;
            case MOV_SPEED: any |= TlvReader::asU8(v, n, c.speed); break;
            case MOV_DUR:   any |= TlvReader::asU16(v, n, c.duration_ms); break;
            default: break;
        }
    }
    return any;
}

bool decodeArmJoints(const u8* in, std::size_t len, ArmJoints& c) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case ARM_ID:    any |= TlvReader::asU8(v, n, c.arm_id); break;
            case ARM_COUNT: any |= TlvReader::asU8(v, n, c.count); break;
            case ARM_JOINTS:
                if (readJointArray(v, n, c.count, c.joints)) any = true;
                break;
            default: break;
        }
    }
    return any && c.count <= kMaxJoints;
}

bool decodeArmPose(const u8* in, std::size_t len, ArmPose& c) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case POSE_ARM:   any |= TlvReader::asU8(v, n, c.arm_id); break;
            case POSE_X:     any |= TlvReader::asF32(v, n, c.x); break;
            case POSE_Y:     any |= TlvReader::asF32(v, n, c.y); break;
            case POSE_Z:     any |= TlvReader::asF32(v, n, c.z); break;
            case POSE_ROLL:  any |= TlvReader::asF32(v, n, c.roll); break;
            case POSE_PITCH: any |= TlvReader::asF32(v, n, c.pitch); break;
            case POSE_YAW:   any |= TlvReader::asF32(v, n, c.yaw); break;
            default: break;
        }
    }
    return any;
}

bool decodeBaseTelemetry(const u8* in, std::size_t len, BaseTelemetry& t) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case BT_X:    any |= TlvReader::asF32(v, n, t.x); break;
            case BT_Y:    any |= TlvReader::asF32(v, n, t.y); break;
            case BT_YAW:  any |= TlvReader::asF32(v, n, t.yaw); break;
            case BT_VX:   any |= TlvReader::asF32(v, n, t.vx); break;
            case BT_VY:   any |= TlvReader::asF32(v, n, t.vy); break;
            case BT_WZ:   any |= TlvReader::asF32(v, n, t.wz); break;
            case BT_BATT: any |= TlvReader::asU8(v, n, t.battery); break;
            default: break;
        }
    }
    return any;
}

bool decodeArmTelemetry(const u8* in, std::size_t len, ArmTelemetry& t) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case ARM_ID:    any |= TlvReader::asU8(v, n, t.arm_id); break;
            case ARM_COUNT: any |= TlvReader::asU8(v, n, t.count); break;
            case ARM_JOINTS:
                if (readJointArray(v, n, t.count, t.joints)) any = true;
                break;
            default: break;
        }
    }
    return any && t.count <= kMaxJoints;
}

bool decodeStatus(const u8* in, std::size_t len, StatusTelem& t) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case ST_MODE:   any |= TlvReader::asU8(v, n, t.mode); break;
            case ST_FAULTS: any |= TlvReader::asU16(v, n, t.faults); break;
            case ST_ESTOP:  any |= TlvReader::asU8(v, n, t.estop); break;
            default: break;
        }
    }
    return any;
}

bool decodeArmMode(const u8* in, std::size_t len, ArmMode& m) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case MODE_ARM:   any |= TlvReader::asU8(v, n, m.arm_id); break;
            case MODE_VALUE: any |= TlvReader::asU8(v, n, m.mode); break;
            default: break;
        }
    }
    return any;
}

bool decodeParamSet(const u8* in, std::size_t len, ParamSet& p) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case PARAM_ID:  any |= TlvReader::asU16(v, n, p.param_id); break;
            case PARAM_VAL: any |= TlvReader::asF32(v, n, p.value); break;
            default: break;
        }
    }
    return any;
}

bool decodeEventLog(const u8* in, std::size_t len, EventLog& e) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case LOG_LEVEL: any |= TlvReader::asU8(v, n, e.level); break;
            case LOG_TEXT: {
                std::size_t c = n < sizeof(e.text) ? n : sizeof(e.text);
                if (c) std::memcpy(e.text, v, c);
                e.text_len = u8(c);
                any = true;
                break;
            }
            default: break;
        }
    }
    return any;
}

bool decodeConnectReq(const u8* in, std::size_t len, ConnectReqMsg& c) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case CREQ_VERSION: any |= TlvReader::asU16(v, n, c.version); break;
            case CREQ_DEVICE:  any |= TlvReader::asU32(v, n, c.device_id); break;
            case CREQ_NONCE:   any |= TlvReader::asU32(v, n, c.nonce); break;
            default: break;
        }
    }
    return any;
}

bool decodeConnectAck(const u8* in, std::size_t len, ConnectAckMsg& c) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case CACK_SESSION: any |= TlvReader::asU32(v, n, c.session_id); break;
            case CACK_STATUS:  any |= TlvReader::asU8(v, n, c.status); break;
            case CACK_CAPS:    any |= TlvReader::asU16(v, n, c.capabilities); break;
            default: break;
        }
    }
    return any;
}

} // namespace robotlink
