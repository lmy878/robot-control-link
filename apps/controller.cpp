// Controller (RPi): 100 Hz downlink control packets (reliable, control stream)
// + uplink telemetry receive/parse (340-byte packet). Session disabled.
//
// Usage: controller [peer_ip] [peer_port] [bind_port]
//   defaults: 127.0.0.1  5600  5601
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "link/control.hpp"
#include "link/endpoint.hpp"
#include "link/messages.hpp"
#include "link/telemetry.hpp"
#include "link/transport.hpp"

using namespace robotlink;
using clk = std::chrono::steady_clock;

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run = false; }

int main(int argc, char** argv) {
    std::string peer_ip = (argc > 1) ? argv[1] : "127.0.0.1";
    u16 peer_port = (argc > 2) ? u16(std::atoi(argv[2])) : 5600;
    u16 bind_port = (argc > 3) ? u16(std::atoi(argv[3])) : 5601;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    UdpTransport t;
    if (!t.open("0.0.0.0", bind_port, peer_ip, peer_port)) {
        std::fprintf(stderr, "controller: cannot bind udp %u\n", bind_port);
        return 1;
    }
    std::printf("controller: bind=%u peer=%s:%u (session disabled)\n",
                bind_port, peer_ip.c_str(), peer_port);

    Endpoint ep(t);               // session intentionally paused
    ep.setRto(std::chrono::milliseconds(30));

    std::atomic<unsigned> acked{0}, telem_rx{0};
    TelemetryPacket last{};
    ep.onAck([&](u16, u8) { acked++; });

    ep.onMessage([&](const FrameHeader& h, const u8* pl, std::size_t n) {
        switch (MsgType(h.msg_type)) {
            case MsgType::Telemetry: {
                TelemetryPacket tp;
                if (decodeTelemetry(pl, n, tp)) {
                    last = tp;
                    telem_rx++;
                }
                break;
            }
            case MsgType::EventLog: {   // debug: lowest priority
                EventLog el;
                if (decodeEventLog(pl, n, el))
                    std::printf("robot log[%u]: %.*s\n", el.level, int(el.text_len),
                                reinterpret_cast<const char*>(el.text));
                break;
            }
            default: break;
        }
    });

    const auto start = clk::now();
    auto next = start;
    auto next_print = start;
    u32 cmd_seq = 0;
    const unsigned in_flight_limit = 8;

    while (g_run) {
        next += std::chrono::milliseconds(10);   // 100 Hz downlink

        // ---- build + send control packet (highest priority stream) ----
        double phase = std::chrono::duration<double>(clk::now() - start).count();
        ControlPacket cp{};
        cp.magic = kControlMagic;
        cp.size = u16(kControlSize);
        cp.seq = cmd_seq++;
        cp.timestamp_us = u64(std::chrono::duration_cast<std::chrono::microseconds>(clk::now() - start).count());
        cp.vel[0] = 0.5f * f32(std::sin(phase));
        cp.vel[1] = 0.5f * f32(std::cos(phase));
        cp.vel[2] = 0.3f * f32(std::sin(0.5 * phase));
        cp.arm_count = 4;
        cp.arm_dof = 6;
        cp.command = 1;              // move
        cp.mode = 1;
        cp.estop = 0;
        for (u8 a = 0; a < 4; ++a)
            for (u8 j = 0; j < 6; ++j)
                cp.arm_joints[a][j] = 0.2f * f32(std::sin(phase + a + j));

        if (ep.pendingCount() < in_flight_limit) {
            u8 pl[kControlSize];
            std::size_t n = encodeControl(cp, pl, sizeof(pl));
            if (n) ep.sendReliable(MsgType::ControlCmd, StreamControl, pl, n);
        }

        ep.poll(0);      // receive telemetry (control acks handled first)
        ep.tick();

        // ---- status print, ~1 Hz ----
        if (clk::now() >= next_print) {
            next_print += std::chrono::milliseconds(1000);
            std::printf("ctl: sent=%u acked=%u pending=%zu | telem=%.1fHz seq=%u vel=(%.2f,%.2f,%.2f) batt=%.1fV\n",
                        cmd_seq, acked.load(), ep.pendingCount(), telem_rx.load() / 1.0,
                        last.seq, last.vel[0], last.vel[1], last.vel[2], last.battery_v);
            telem_rx = 0;
        }

        std::this_thread::sleep_until(next);
    }
    std::printf("controller: done\n");
    return 0;
}
