// Robot node (Jetson): 200 Hz uplink telemetry (5 quaternions + 15 laser +
// 50 states = 340-byte packet) and downlink control parsing (joystick + estop +
// mode). Control stream is highest priority, debug lowest. Session disabled.
//
// Telemetry is filled with VIRTUAL data for now.
//
// Usage: robot_node [peer_ip] [peer_port] [bind_port]
//   defaults: 127.0.0.1  5601  5600
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include <pthread.h>
#include <sched.h>

#include "link/control.hpp"
#include "link/endpoint.hpp"
#include "link/messages.hpp"
#include "link/telemetry.hpp"
#include "link/transport.hpp"

using namespace robotlink;
using clk = std::chrono::steady_clock;

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run = false; }

static void tryRealtime() {
    sched_param sp{};
    sp.sched_priority = 50;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        std::fprintf(stderr, "robot_node: SCHED_FIFO unavailable (run as root?)\n");
}

struct RobotState {
    f32 joystick[kJoystickAxes]{};   // last commanded axes
    u8  mode = 0;
    bool estop = false;
    clk::time_point last_ctl;
};

// ---- virtual telemetry generation (replace with real sensor reads) ----
static void fillVirtualTelemetry(TelemetryPacket& tp, const RobotState& rs, double t) {
    // base + 4 arms orientations as quaternions (w, x, y, z)
    tp.quaternions[0][0] = f32(std::cos(0.5 * rs.joystick[3]));
    tp.quaternions[0][1] = 0.0f;
    tp.quaternions[0][2] = 0.0f;
    tp.quaternions[0][3] = f32(std::sin(0.5 * rs.joystick[3]));
    for (int a = 1; a < kQuatCount; ++a) {
        f32 ang = f32(0.3 * std::sin(t + a));
        tp.quaternions[a][0] = f32(std::cos(0.5 * ang));
        tp.quaternions[a][1] = f32(std::sin(0.5 * ang));
        tp.quaternions[a][2] = 0.0f;
        tp.quaternions[a][3] = 0.0f;
    }
    // 15 laser range measurements
    for (int i = 0; i < kLaserCount; ++i)
        tp.laser[i] = f32(5.0 + 2.0 * std::sin(t + 0.4 * i));
    // 50 robot states
    for (int i = 0; i < kStateCount; ++i)
        tp.states[i] = f32(std::sin(0.3 * t + 0.1 * i));
}

int main(int argc, char** argv) {
    std::string peer_ip = (argc > 1) ? argv[1] : "127.0.0.1";
    u16 peer_port = (argc > 2) ? u16(std::atoi(argv[2])) : 5601;
    u16 bind_port = (argc > 3) ? u16(std::atoi(argv[3])) : 5600;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    UdpTransport t;
    if (!t.open("0.0.0.0", bind_port, peer_ip, peer_port)) {
        std::fprintf(stderr, "robot_node: cannot bind udp %u\n", bind_port);
        return 1;
    }
    std::printf("robot_node: bind=%u peer=%s:%u (session disabled)\n",
                bind_port, peer_ip.c_str(), peer_port);

    Endpoint ep(t);               // no setDeviceId/beginConnect -> session paused
    RobotState rs;
    rs.last_ctl = clk::now();
    const auto failsafe = std::chrono::milliseconds(500);

    ep.onMessage([&](const FrameHeader& h, const u8* pl, std::size_t n) {
        switch (MsgType(h.msg_type)) {
            case MsgType::ControlCmd: {
                ControlPacket cp;
                if (decodeControl(pl, n, cp)) {
                    rs.last_ctl = clk::now();
                    rs.estop = cp.estop != 0;
                    rs.mode = cp.mode;
                    for (int i = 0; i < kJoystickAxes; ++i) rs.joystick[i] = cp.joystick[i];
                }
                break;
            }
            case MsgType::EmergencyStop:
                rs.estop = true;
                std::printf("robot_node: EMERGENCY STOP\n");
                break;
            default:
                break;
        }
    });

    tryRealtime();

    const auto start = clk::now();
    auto next = start;
    auto next_dbg = start;

    while (g_run) {
        next += std::chrono::milliseconds(5);   // 200 Hz uplink

        ep.poll(0);      // control stream dispatched before debug
        ep.tick();

        // Failsafe: zero the sticks if control goes silent.
        if (clk::now() - rs.last_ctl > failsafe) {
            for (int i = 0; i < kJoystickAxes; ++i) rs.joystick[i] = 0;
        }

        auto now = clk::now();
        double tel = std::chrono::duration<double>(now - start).count();

        TelemetryPacket tp{};
        fillVirtualTelemetry(tp, rs, tel);
        u8 pl[kTelemetrySize];
        std::size_t n = encodeTelemetry(tp, pl, sizeof(pl));
        if (n) ep.send(MsgType::Telemetry, StreamTelemetry, pl, n);

        // ---- debug stream: lowest priority, 1 Hz ----
        if (now >= next_dbg) {
            next_dbg += std::chrono::milliseconds(1000);
            EventLog el;
            el.level = 1;
            std::snprintf(reinterpret_cast<char*>(el.text), sizeof(el.text),
                          "mode=%u estop=%u", rs.mode, rs.estop ? 1 : 0);
            el.text_len = u8(std::strlen(reinterpret_cast<char*>(el.text)));
            u8 dbg[80];
            std::size_t dn = encodeEventLog(el, dbg, sizeof(dbg));
            if (dn) ep.send(MsgType::EventLog, StreamDebug, dbg, dn);
        }

        std::this_thread::sleep_until(next);
    }
    std::printf("robot_node: done\n");
    return 0;
}
