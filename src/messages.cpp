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

std::size_t encodeAck(const AckMsg& a, u8* out, std::size_t cap) {
    TlvWriter w(out, cap);
    w.u16v(ACK_SEQ, a.ack_seq); w.u8v(ACK_STAT, a.status);
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

bool decodeAck(const u8* in, std::size_t len, AckMsg& a) {
    TlvReader r(in, len); u8 tag; const u8* v; std::size_t n; bool any = false;
    while (r.next(tag, v, n)) {
        switch (tag) {
            case ACK_SEQ:  any |= TlvReader::asU16(v, n, a.ack_seq); break;
            case ACK_STAT: any |= TlvReader::asU8(v, n, a.status); break;
            default: break;
        }
    }
    return any;
}

} // namespace robotlink
