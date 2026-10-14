// tests/unit/test_codec.cpp
//
// T1 — protocol conformance & robustness. Pure unit tests, no network.
// Exercises CRC, frame header encode/decode, corruption detection, TLV codec,
// fixed-size application packets and stream classification.
//
// Prints a JSON summary as its final line so the Python orchestrator can parse
// pass/fail counts and the header corruption-detection rate.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "link/common.hpp"
#include "link/control.hpp"
#include "link/crc.hpp"
#include "link/frame.hpp"
#include "link/messages.hpp"
#include "link/telemetry.hpp"
#include "link/tlv.hpp"

using namespace robotlink;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* name) {
    if (ok) { ++g_pass; }
    else { ++g_fail; std::fprintf(stderr, "  FAIL: %s\n", name); }
}

static bool approx(f32 a, f32 b) { return std::fabs(a - b) < 1e-5f; }

int main() {
    // ---- CRC known vectors ----
    {
        const u8 v[] = "123456789";
        check(crc16(v, 9) == 0x29B1, "crc16 CCITT-FALSE check value");
        check(crc32(v, 9) == 0xCBF43926u, "crc32 ISO-HDLC check value");
    }

    // ---- header round trip ----
    {
        FrameHeader h;
        h.version = kVersion; h.msg_type = u8(MsgType::ControlCmd);
        h.flags = FlagPayloadCrc; h.stream_id = u8(StreamControl);
        h.seq = 0x1234; h.payload_len = 0; h.reserved = 0;
        u8 buf[kHeaderSize];
        encodeHeader(h, buf);
        FrameHeader out;
        check(decodeHeader(buf, kHeaderSize, out), "header decode valid");
        check(out.msg_type == h.msg_type && out.flags == h.flags &&
              out.stream_id == h.stream_id && out.seq == h.seq,
              "header field round trip");
    }

    // ---- header corruption detection (single-bit flips) ----
    {
        FrameHeader h; h.msg_type = 0x18; h.flags = 0; h.stream_id = 0;
        h.seq = 100; h.payload_len = 0;
        u8 buf[kHeaderSize];
        encodeHeader(h, buf);
        int detected = 0, total = 0;
        for (int i = 0; i < int(kHeaderSize); ++i) {  // every header byte
            for (int b = 0; b < 8; ++b) {
                u8 t[kHeaderSize];
                std::memcpy(t, buf, kHeaderSize);
                t[i] ^= u8(1u << b);
                FrameHeader out;
                ++total;
                if (!decodeHeader(t, kHeaderSize, out)) ++detected;
            }
        }
        check(detected == total, "all single-bit header corruptions detected");
        std::printf("header_corruption_detection_rate=%.4f\n",
                    double(detected) / double(total));
    }

    // ---- malformed headers rejected ----
    {
        FrameHeader h; h.payload_len = 0;
        u8 buf[kHeaderSize];
        encodeHeader(h, buf);
        FrameHeader out;
        check(!decodeHeader(buf, 4, out), "short buffer rejected");
        u8 t[kHeaderSize]; std::memcpy(t, buf, kHeaderSize); t[0] = 0x00;
        check(!decodeHeader(t, kHeaderSize, out), "bad magic rejected");
        std::memcpy(t, buf, kHeaderSize); t[2] = 0x7F;
        check(!decodeHeader(t, kHeaderSize, out), "bad version rejected");
        FrameHeader big; big.payload_len = u16(kMaxPayload + 3);
        encodeHeader(big, buf);
        check(!decodeHeader(buf, kHeaderSize + kMaxPayload + 8, out), "oversized payload rejected");
    }

    // ---- payload CRC ----
    {
        FrameHeader h; h.msg_type = 0x18; h.flags = FlagPayloadCrc;
        const u8 payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
        u8 buf[kMaxFrame];
        std::size_t n = buildFrame(h, payload, 8, buf, sizeof(buf));
        check(n == kHeaderSize + 8 + 2, "buildFrame length with payload CRC");
        check(getU16(buf + kHeaderSize + 8) == crc16(payload, 8), "payload CRC value");
        FrameHeader out;
        check(decodeHeader(buf, n, out), "frame header valid");
        check(out.payload_len == 10, "payload_len includes trailing CRC");
        u8 t[kMaxFrame]; std::memcpy(t, buf, n); t[kHeaderSize + 2] ^= 0xFF;
        check(getU16(t + kHeaderSize + 8) != crc16(t + kHeaderSize, 8), "payload corruption detected");
    }

    // ---- TLV round trips ----
    {
        u8 buf[256];
        VelocityCmd v{1.5f, -2.5f, 0.25f}, v2;
        std::size_t n = encodeVelocity(v, buf, sizeof(buf));
        check(n > 0 && decodeVelocity(buf, n, v2) && approx(v2.vx, 1.5f) &&
              approx(v2.vy, -2.5f) && approx(v2.wz, 0.25f), "Velocity TLV round trip");

        MoveOrder m{MoveDir::RotateLeft, 200, 1500}, m2;
        n = encodeMoveOrder(m, buf, sizeof(buf));
        check(n > 0 && decodeMoveOrder(buf, n, m2) && m2.dir == MoveDir::RotateLeft &&
              m2.speed == 200 && m2.duration_ms == 1500, "MoveOrder TLV round trip");

        ArmJoints a; a.arm_id = 2; a.count = 6;
        for (int i = 0; i < 6; ++i) a.joints[i] = f32(i) * 0.1f;
        ArmJoints a2;
        n = encodeArmJoints(a, buf, sizeof(buf));
        check(n > 0 && decodeArmJoints(buf, n, a2) && a2.arm_id == 2 && a2.count == 6 &&
              approx(a2.joints[5], 0.5f), "ArmJoints TLV round trip");

        ArmPose p; p.arm_id = 3; p.x = 1; p.y = 2; p.z = 3; p.roll = 0.1f;
        p.pitch = 0.2f; p.yaw = 0.3f;
        ArmPose p2;
        n = encodeArmPose(p, buf, sizeof(buf));
        check(n > 0 && decodeArmPose(buf, n, p2) && approx(p2.yaw, 0.3f) &&
              approx(p2.z, 3.0f), "ArmPose TLV round trip");

        EventLog e; e.level = 2; std::strcpy(reinterpret_cast<char*>(e.text), "hello");
        e.text_len = 5; EventLog e2;
        n = encodeEventLog(e, buf, sizeof(buf));
        check(n > 0 && decodeEventLog(buf, n, e2) && e2.level == 2 && e2.text_len == 5 &&
              std::memcmp(e2.text, "hello", 5) == 0, "EventLog TLV round trip");

        ConnectReqMsg cr{1, 0xDEADBEEF, 0x11223344}, cr2;
        n = encodeConnectReq(cr, buf, sizeof(buf));
        check(n > 0 && decodeConnectReq(buf, n, cr2) && cr2.device_id == 0xDEADBEEF &&
              cr2.nonce == 0x11223344, "ConnectReq TLV round trip");

        ConnectAckMsg ca{0xCAFEBABE, 0, 0x0001}, ca2;
        n = encodeConnectAck(ca, buf, sizeof(buf));
        check(n > 0 && decodeConnectAck(buf, n, ca2) && ca2.session_id == 0xCAFEBABE &&
              ca2.capabilities == 1, "ConnectAck TLV round trip");
    }

    // ---- unknown tags are skipped ----
    {
        u8 buf[64];
        TlvWriter w(buf, sizeof(buf));
        w.u16v(200, 0xABCD);                 // unknown tag first
        w.f32v(VEL_VX, 9.0f);
        w.f32v(VEL_VY, 8.0f);
        w.f32v(VEL_WZ, 7.0f);
        VelocityCmd c;
        check(decodeVelocity(buf, w.size(), c) && approx(c.vx, 9.0f) &&
              approx(c.wz, 7.0f), "unknown TLV tag skipped");
    }

    // ---- fixed-size application packets ----
    {
        check(sizeof(ControlPacket) == 20, "ControlPacket is 20 bytes");
        check(sizeof(TelemetryPacket) == 340, "TelemetryPacket is 340 bytes");

        ControlPacket cp{};
        for (int i = 0; i < kJoystickAxes; ++i) cp.joystick[i] = f32(i) * 0.25f;
        cp.estop = 1; cp.mode = 2; cp.buttons = 0x0A; cp.reserved = 0;
        u8 buf[kControlSize];
        std::size_t n = encodeControl(cp, buf, sizeof(buf));
        ControlPacket cp2;
        check(n == 20 && decodeControl(buf, n, cp2) && cp2.estop == 1 &&
              cp2.mode == 2 && approx(cp2.joystick[3], 0.75f), "ControlPacket round trip");

        TelemetryPacket tp{};
        for (int i = 0; i < kQuatCount; ++i) tp.quaternions[i][i % 4] = f32(i) + 1.0f;
        for (int i = 0; i < kLaserCount; ++i) tp.laser[i] = f32(i) * 0.5f;
        for (int i = 0; i < kStateCount; ++i) tp.states[i] = f32(i) * 0.01f;
        u8 tbuf[kTelemetrySize];
        n = encodeTelemetry(tp, tbuf, sizeof(tbuf));
        TelemetryPacket tp2;
        check(n == 340 && decodeTelemetry(tbuf, n, tp2) &&
              approx(tp2.laser[14], 7.0f) && approx(tp2.states[49], 0.49f),
              "TelemetryPacket round trip");
        check(!decodeTelemetry(tbuf, 100, tp2), "Telemetry wrong length rejected");
    }

    // ---- stream classification & priority ----
    {
        check(isDownlink(MsgType::ControlCmd), "ControlCmd is downlink");
        check(!isDownlink(MsgType::Telemetry), "Telemetry is uplink");
        check(streamPriority(StreamControl) < streamPriority(StreamTelemetry), "control > telemetry");
        check(streamPriority(StreamTelemetry) < streamPriority(StreamSession), "telemetry > session");
        check(streamPriority(StreamSession) < streamPriority(StreamDebug), "session > debug");
    }

    std::printf("{\"suite\":\"codec\",\"pass\":%d,\"fail\":%d,\"total\":%d}\n",
                g_pass, g_fail, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
