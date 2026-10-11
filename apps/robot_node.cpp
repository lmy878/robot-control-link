// Robot node (Jetson): 200 Hz uplink telemetry (340-byte packet) + downlink
// control parsing. Control stream is highest priority, debug lowest.
// Session features are intentionally disabled.
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
#include "link/kinematics.hpp"
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
    f32 vel[3]{};                 // commanded vx, vy, wz
    f32 joints[4][6]{};           // commanded joint targets
    u8  arm_count = 4;
    u8  arm_dof = 6;
    bool estop = false;
    clk::time_point last_ctl;
};

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
                    for (int i = 0; i < 3; ++i) rs.vel[i] = cp.vel[i];
                    if (cp.arm_count <= 4 && cp.arm_dof <= 6) {
                        rs.arm_count = cp.arm_count;
                        rs.arm_dof = cp.arm_dof;
                        for (u8 a = 0; a < cp.arm_count; ++a)
                            for (u8 j = 0; j < cp.arm_dof; ++j)
                                rs.joints[a][j] = cp.arm_joints[a][j];
                    }
                }
                break;
            }
            case MsgType::Stop:
                rs.estop = false;
                rs.vel[0] = rs.vel[1] = rs.vel[2] = 0;
                break;
            case MsgType::EmergencyStop:
                rs.estop = true;
                rs.vel[0] = rs.vel[1] = rs.vel[2] = 0;
                std::printf("robot_node: EMERGENCY STOP\n");
                break;
            default:
                break;   // debug/other handled at lowest priority
        }
    });

    tryRealtime();

    const auto start = clk::now();
    auto next = start;
    auto next_dbg = start;
    u32 seq = 0;

    while (g_run) {
        next += std::chrono::milliseconds(5);   // 200 Hz uplink

        ep.poll(0);      // <-- control stream dispatched before debug
        ep.tick();       // ARQ timers

        // Failsafe: stop if control is silent.
        if (!rs.estop && clk::now() - rs.last_ctl > failsafe) {
            rs.vel[0] = rs.vel[1] = rs.vel[2] = 0;
        }

        // ---- build + transmit 340-byte telemetry packet ----
        auto now = clk::now();
        TelemetryPacket tp{};
        tp.magic = kTelemetryMagic;
        tp.size = u16(kTelemetrySize);
        tp.seq = seq++;
        tp.timestamp_us = u64(std::chrono::duration_cast<std::chrono::microseconds>(now - start).count());
        tp.pos[0] = tp.pos[1] = tp.pos[2] = 0;
        tp.vel[0] = rs.vel[0]; tp.vel[1] = rs.vel[1]; tp.vel[2] = 0;
        tp.rpy[2] = rs.vel[2];                 // yaw command as stand-in
        tp.battery_v = 24.0f; tp.battery_i = 3.5f;
        tp.arm_count = rs.arm_count;
        tp.arm_dof = rs.arm_dof;
        tp.status_flags = rs.estop ? 0u : 1u;
        for (u8 a = 0; a < 4; ++a)
            for (u8 j = 0; j < 6; ++j) tp.arm_joints[a][j] = rs.joints[a][j];
        tp.mode = rs.estop ? 2 : 1;
        tp.estop = rs.estop ? 1 : 0;
        tp.uptime_ms = u32(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
        tp.loop_rate_hz = 200.0f;

        u8 pl[kTelemetrySize];
        std::size_t n = encodeTelemetry(tp, pl, sizeof(pl));
        if (n) ep.send(MsgType::Telemetry, StreamTelemetry, pl, n);

        // ---- debug stream: lowest priority, 1 Hz ----
        if (now >= next_dbg) {
            next_dbg += std::chrono::milliseconds(1000);
            EventLog el;
            el.level = 1;
            const char* msg = rs.estop ? "estop" : (rs.vel[0] || rs.vel[1] || rs.vel[2] ? "moving" : "idle");
            std::snprintf(reinterpret_cast<char*>(el.text), sizeof(el.text), "robot %s", msg);
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
