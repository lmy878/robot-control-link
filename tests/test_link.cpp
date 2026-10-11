#include <cstdio>
#include <cstring>
#include <deque>
#include <thread>
#include <vector>

#include "link/crc.hpp"
#include "link/endpoint.hpp"
#include "link/frame.hpp"
#include "link/kinematics.hpp"
#include "link/messages.hpp"
#include "link/tlv.hpp"

using namespace robotlink;

static int g_fail = 0;
#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
            ++g_fail;                                                    \
        }                                                                \
    } while (0)

struct FakeTransport : Transport {
    std::deque<std::vector<u8>> inbox, outbox;
    int send(const u8* d, std::size_t n) override {
        outbox.emplace_back(d, d + n);
        return static_cast<int>(n);
    }
    int recv(u8* d, std::size_t cap, int) override {
        if (inbox.empty()) return 0;
        auto f = std::move(inbox.front());
        inbox.pop_front();
        std::size_t n = f.size() < cap ? f.size() : cap;
        std::memcpy(d, f.data(), n);
        return static_cast<int>(n);
    }
};

static void pump(FakeTransport& from, FakeTransport& to) {
    while (!from.outbox.empty()) {
        to.inbox.push_back(std::move(from.outbox.front()));
        from.outbox.pop_front();
    }
}

static void testCrc() {
    const char* s = "123456789";
    CHECK(crc16(reinterpret_cast<const u8*>(s), 9) == 0x29B1);
    CHECK(crc32(reinterpret_cast<const u8*>(s), 9) == 0xCBF43926u);
}

static void testFrame() {
    FrameHeader h;
    h.msg_type = u8(MsgType::SetVelocity);
    h.stream_id = StreamControl;
    h.flags = FlagAckReq | FlagPayloadCrc;
    h.seq = 42;
    const u8 payload[] = {1, 2, 3, 4};
    u8 out[kMaxFrame];
    std::size_t n = buildFrame(h, payload, sizeof(payload), out, sizeof(out));
    CHECK(n == kHeaderSize + sizeof(payload) + 2);

    FrameHeader p;
    CHECK(decodeHeader(out, n, p));
    CHECK(p.seq == 42);
    CHECK(p.msg_type == u8(MsgType::SetVelocity));

    out[kHeaderSize] ^= 0xFF;                        // corrupt payload
    FrameHeader q;
    CHECK(decodeHeader(out, n, q));                  // header still ok
    CHECK(getU16(out + kHeaderSize + sizeof(payload)) != crc16(out + kHeaderSize, sizeof(payload)));

    out[3] ^= 0xFF;                                  // corrupt header field
    FrameHeader r;
    CHECK(!decodeHeader(out, n, r));                 // header CRC catches it
}

static void testTlv() {
    u8 buf[64];
    TlvWriter w(buf, sizeof(buf));
    w.u8v(1, 7); w.u16v(2, 0x1234); w.f32v(3, 1.5f);
    CHECK(w.ok());

    TlvReader r(buf, w.size());
    u8 tag; const u8* v; std::size_t len;
    CHECK(r.next(tag, v, len) && tag == 1 && len == 1 && v[0] == 7);
    u16 u; CHECK(r.next(tag, v, len) && TlvReader::asU16(v, len, u) && u == 0x1234);
    f32 f; CHECK(r.next(tag, v, len) && TlvReader::asF32(v, len, f) && f == 1.5f);
    CHECK(!r.next(tag, v, len));
}

static void testMessages() {
    u8 buf[128];

    VelocityCmd vc{0.5f, -0.25f, 1.0f};
    std::size_t n = encodeVelocity(vc, buf, sizeof(buf));
    VelocityCmd vo;
    CHECK(n && decodeVelocity(buf, n, vo));
    CHECK(vo.vx == 0.5f && vo.vy == -0.25f && vo.wz == 1.0f);

    MoveOrder mo{MoveDir::StrafeRight, 200, 1500};
    n = encodeMoveOrder(mo, buf, sizeof(buf));
    MoveOrder moo;
    CHECK(n && decodeMoveOrder(buf, n, moo));
    CHECK(moo.dir == MoveDir::StrafeRight && moo.speed == 200 && moo.duration_ms == 1500);

    ArmJoints aj; aj.arm_id = 2; aj.count = 3; aj.joints[0] = 0.1f; aj.joints[1] = 0.2f; aj.joints[2] = -0.3f;
    n = encodeArmJoints(aj, buf, sizeof(buf));
    ArmJoints ajo;
    CHECK(n && decodeArmJoints(buf, n, ajo));
    CHECK(ajo.arm_id == 2 && ajo.count == 3 && ajo.joints[2] == -0.3f);

    BaseTelemetry bt{1, 2, 3, 4, 5, 6, 77};
    n = encodeBaseTelemetry(bt, buf, sizeof(buf));
    BaseTelemetry bto;
    CHECK(n && decodeBaseTelemetry(buf, n, bto));
    CHECK(bto.wz == 6 && bto.battery == 77);
}

static void testArq() {
    FakeTransport ta, tb;
    Endpoint a(ta), b(tb);
    a.setRto(std::chrono::milliseconds(20));

    unsigned got = 0;
    b.onMessage([&](const FrameHeader& h, const u8*, std::size_t) {
        if (h.msg_type == u8(MsgType::SetVelocity)) ++got;
    });

    u8 pl[16];
    std::size_t n = encodeVelocity(VelocityCmd{1, 0, 0}, pl, sizeof(pl));
    a.sendReliable(MsgType::SetVelocity, StreamControl, pl, n);
    CHECK(a.pendingCount() == 1);

    pump(ta, tb);           // command reaches robot
    b.poll(0);              // robot processes + ACKs
    CHECK(got == 1);

    pump(tb, ta);           // ACK reaches controller
    a.poll(0);
    CHECK(a.pendingCount() == 0);

    // Duplicate suppression: the same raw frame delivered twice executes once.
    FakeTransport tc, td;
    Endpoint c(tc), d(td);
    unsigned got2 = 0;
    d.onMessage([&](const FrameHeader& h, const u8*, std::size_t) {
        if (h.msg_type == u8(MsgType::SetVelocity)) ++got2;
    });
    u8 q[16];
    std::size_t qn = encodeVelocity(VelocityCmd{1, 0, 0}, q, sizeof(q));
    c.sendReliable(MsgType::SetVelocity, StreamControl, q, qn);
    std::vector<u8> frame = std::move(tc.outbox.front());
    tc.outbox.pop_front();
    td.inbox.push_back(frame);
    td.inbox.push_back(frame);       // duplicate
    d.poll(0);
    CHECK(got2 == 1);
}

static void testRetransmit() {
    FakeTransport ta, tb;
    Endpoint a(ta);
    a.setRto(std::chrono::milliseconds(1));
    a.setMaxRetries(3);

    u8 pl[8];
    std::size_t n = encodeAck(AckMsg{5, 0}, pl, sizeof(pl));
    a.sendReliable(MsgType::SetVelocity, StreamControl, pl, n);
    std::size_t before = ta.outbox.size();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    a.tick();
    CHECK(ta.outbox.size() > before);      // retransmitted
}

static void testKinematics() {
    WheelSpeeds w = mecanumInverse(1.0f, 0.0f, 0.0f, 0.05f, 0.2f, 0.2f);
    CHECK(w.fl == w.fr && w.fr == w.rl && w.rl == w.rr);   // pure forward
    WheelSpeeds r = mecanumInverse(0.0f, 0.0f, 1.0f, 0.05f, 0.2f, 0.2f);
    CHECK(r.fl < 0 && r.fr > 0);                            // pure yaw -> opposite sides
}

int main() {
    testCrc();
    testFrame();
    testTlv();
    testMessages();
    testArq();
    testRetransmit();
    testKinematics();

    if (g_fail) { std::printf("%d CHECK(s) failed\n", g_fail); return 1; }
    std::printf("all tests passed\n");
    return 0;
}
