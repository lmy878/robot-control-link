// tests/unit/test_monitor.cpp
//
// T6 — passive monitor. Pure unit tests, no network. Verifies that the Monitor
// class observes both directions, infers packet loss from sequence gaps, drops
// duplicates, tracks reordering, flips link state on timeout, decodes the
// application packets for live analysis and writes a match log — all without
// ever transmitting.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "link/control.hpp"
#include "link/frame.hpp"
#include "link/monitor.hpp"
#include "link/messages.hpp"
#include "link/telemetry.hpp"

using namespace robotlink;
using clk = std::chrono::steady_clock;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* name) {
    if (ok) ++g_pass;
    else { ++g_fail; std::fprintf(stderr, "  FAIL: %s\n", name); }
}

static std::vector<u8> makeFrame(u8 type, u8 stream, u16 seq,
                                 const u8* pl, std::size_t plen) {
    FrameHeader h;
    h.msg_type = type;
    h.stream_id = stream;
    h.seq = seq;
    std::vector<u8> out(kMaxFrame);
    std::size_t n = buildFrame(h, pl, plen, out.data(), out.size());
    out.resize(n);
    return out;
}

static bool feed(Monitor& m, u8 type, u8 stream, u16 seq,
                 const u8* pl, std::size_t plen, clk::time_point now) {
    std::vector<u8> f = makeFrame(type, stream, seq, pl, plen);
    FrameHeader h;
    if (!decodeHeader(f.data(), f.size(), h)) return false;
    return m.observe(h, f.data() + kHeaderSize, h.payload_len, f.size(), now);
}

int main() {
    const auto t0 = clk::now();

    // ---- direction classification + per-stream loss ----
    {
        Monitor m;
        // C->R control: seq 1,2,5 -> seq 3,4 lost (2 gaps).
        for (u16 s : {1, 2, 5})
            check(feed(m, u8(MsgType::ControlCmd), u8(StreamControl), s, nullptr, 0, t0),
                  "control frame accepted");
        // R->C telemetry: seq 10,11 -> no loss.
        for (u16 s : {10, 11})
            feed(m, u8(MsgType::Telemetry), u8(StreamTelemetry), s, nullptr, 0, t0);

        const auto& cr = m.direction(LinkDirection::ControllerToRobot);
        const auto& rc = m.direction(LinkDirection::RobotToController);
        check(cr.frames == 3, "C->R frames counted");
        check(rc.frames == 2, "R->C frames counted");
        check(cr.streams[StreamControl].lost == 2, "sequence gap => 2 lost");
        check(rc.streams[StreamTelemetry].lost == 0, "no gap => no loss");
        check(m.lossRate(LinkDirection::ControllerToRobot) > 0.39 &&
              m.lossRate(LinkDirection::ControllerToRobot) < 0.41,
              "C->R loss rate = 2/5");
    }

    // ---- duplicate suppression + reordering ----
    {
        Monitor m;
        feed(m, u8(MsgType::ControlCmd), u8(StreamControl), 5, nullptr, 0, t0);
        bool dup = feed(m, u8(MsgType::ControlCmd), u8(StreamControl), 5, nullptr, 0, t0);
        bool older = feed(m, u8(MsgType::ControlCmd), u8(StreamControl), 4, nullptr, 0, t0);
        const auto& ss = m.direction(LinkDirection::ControllerToRobot).streams[StreamControl];
        check(!dup, "duplicate returns false");
        check(ss.duplicates == 1, "duplicate counted once");
        check(older && ss.reordered == 1, "older seq counted as reordered");
        check(ss.received == 2, "unique frames counted (5 and 4)");
    }

    // ---- link state timeout ----
    {
        Monitor m(std::chrono::milliseconds(100));
        feed(m, u8(MsgType::Telemetry), u8(StreamTelemetry), 1, nullptr, 0, t0);
        m.tick(t0);
        check(m.linkUp(LinkDirection::RobotToController), "link up after first frame");
        m.tick(t0 + std::chrono::milliseconds(150));
        check(!m.linkUp(LinkDirection::RobotToController), "link down after timeout");
        feed(m, u8(MsgType::Telemetry), u8(StreamTelemetry), 2, nullptr, 0,
             t0 + std::chrono::milliseconds(200));
        check(m.linkUp(LinkDirection::RobotToController), "link recovers on next frame");
    }

    // ---- application-level analysis ----
    {
        Monitor m;
        ControlPacket cp{};
        cp.mode = 2;
        cp.estop = 1;
        u8 cpl[kControlSize];
        std::size_t cn = encodeControl(cp, cpl, sizeof(cpl));
        feed(m, u8(MsgType::ControlCmd), u8(StreamControl), 1, cpl, cn, t0);

        TelemetryPacket tp{};
        for (int i = 0; i < kQuatCount; ++i) tp.quaternions[i][0] = 1.0f;  // unit
        for (int i = 0; i < kLaserCount; ++i) tp.laser[i] = 5.0f;
        tp.quaternions[0][0] = 2.0f;   // non-unit -> flagged
        tp.laser[0] = -1.0f;           // implausible -> flagged
        u8 tpl[kTelemetrySize];
        std::size_t tn = encodeTelemetry(tp, tpl, sizeof(tpl));
        feed(m, u8(MsgType::Telemetry), u8(StreamTelemetry), 10, tpl, tn, t0);

        const RobotAnalysis& ra = m.analysis();
        check(ra.control_frames == 1 && ra.mode == 2 && ra.estop,
              "control packet analysed");
        check(ra.telemetry_frames == 1, "telemetry packet analysed");
        check(ra.bad_quaternions == 1, "non-unit quaternion flagged");
        check(ra.bad_laser == 1, "implausible laser range flagged");
    }

    // ---- match log ----
    {
        const std::string path = "/tmp/rclp_monitor_test.csv";
        Monitor m;
        check(m.openLog(path), "open log");
        feed(m, u8(MsgType::ControlCmd), u8(StreamControl), 1, nullptr, 0, t0);
        m.logFrame(LinkDirection::ControllerToRobot,
                   FrameHeader{}, 20, t0 + std::chrono::milliseconds(3));
        m.closeLog();
        check(!m.logging(), "log closed");

        std::FILE* f = std::fopen(path.c_str(), "r");
        check(f != nullptr, "log file exists");
        if (f) {
            char buf[256];
            bool header = std::fgets(buf, sizeof(buf), f) != nullptr &&
                          std::strstr(buf, "RCLP monitor log") != nullptr;
            char* col = std::fgets(buf, sizeof(buf), f);   // column header
            (void)col;
            bool row = std::fgets(buf, sizeof(buf), f) != nullptr &&
                       std::strstr(buf, "C->R") != nullptr;
            std::fclose(f);
            std::remove(path.c_str());
            check(header, "log has header");
            check(row, "log has frame row");
        }
    }

    std::printf("{\"suite\":\"monitor\",\"pass\":%d,\"fail\":%d,\"total\":%d}\n",
                g_pass, g_fail, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
