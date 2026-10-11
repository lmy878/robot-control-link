// Controller (RPi): 100 Hz downlink control (joystick + estop + mode, reliable
// on the control stream) and uplink telemetry receive/parse (5 quaternions +
// 15 laser + 50 states = 340-byte packet). Session disabled.
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
    const unsigned in_flight_limit = 8;
    u8 mode = 1;
    u8 estop = 0;

    while (g_run) {
        next += std::chrono::milliseconds(10);   // 100 Hz downlink

        // ---- build + send control packet (highest priority stream) ----
        double phase = std::chrono::duration<double>(clk::now() - start).count();
        ControlPacket cp{};
        cp.joystick[0] = f32(std::sin(phase));        // x
        cp.joystick[1] = f32(std::cos(phase));        // y
        cp.joystick[2] = 0.0f;                        // z
        cp.joystick[3] = f32(0.3 * std::sin(0.5 * phase)); // yaw
        cp.estop = estop;
        cp.mode = mode;
        cp.buttons = 0;

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
            std::printf("ctl: acked=%u pending=%zu | telem=%.0fHz | quat0=(%.2f,%.2f,%.2f,%.2f) laser0=%.2f state0=%.3f\n",
                        acked.load(), ep.pendingCount(), double(telem_rx.load()),
                        last.quaternions[0][0], last.quaternions[0][1],
                        last.quaternions[0][2], last.quaternions[0][3],
                        last.laser[0], last.states[0]);
            telem_rx = 0;
        }

        std::this_thread::sleep_until(next);
    }
    std::printf("controller: done\n");
    return 0;
}
