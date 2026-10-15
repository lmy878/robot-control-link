// Robot side: receives + decodes commands (downlink, best-effort / FEC-only),
// drives the omni base and 4 arms, and streams telemetry (uplink, FEC-only).
//
// Usage: receiver [peer_ip] [peer_port] [bind_port] [src_ip]
//   defaults: 127.0.0.1  5601  5600  0.0.0.0
//   src_ip selects the local NIC used for sending (and for RX); 0.0.0.0 = any.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include <pthread.h>
#include <sched.h>

#include "link/endpoint.hpp"
#include "link/kinematics.hpp"
#include "link/messages.hpp"
#include "link/transport.hpp"

using namespace robotlink;
using clk = std::chrono::steady_clock;

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run = false; }

static void tryRealtime() {
    sched_param sp{};
    sp.sched_priority = 50;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        std::fprintf(stderr, "receiver: SCHED_FIFO unavailable (run as root?)\n");
}

struct RobotState {
    f32 vx = 0, vy = 0, wz = 0;
    WheelSpeeds wheels;
    f32 arm[4][kMaxJoints]{};
    u8  arm_count[4] = {6, 6, 6, 6};
    bool estop = false;
    clk::time_point last_cmd;
};

int main(int argc, char** argv) {
    std::string peer_ip = (argc > 1) ? argv[1] : "127.0.0.1";
    u16 peer_port = (argc > 2) ? u16(std::atoi(argv[2])) : 5601;
    u16 bind_port = (argc > 3) ? u16(std::atoi(argv[3])) : 5600;
    std::string src_ip = (argc > 4) ? argv[4] : "0.0.0.0";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::setvbuf(stdout, nullptr, _IOLBF, 0);   // live output when redirected

    UdpTransport t;
    if (!t.open(src_ip, bind_port, peer_ip, peer_port)) {
        std::fprintf(stderr, "receiver: cannot bind %s:%u\n", src_ip.c_str(), bind_port);
        return 1;
    }
    std::printf("receiver: bind=%s:%u peer=%s:%u\n",
                src_ip.c_str(), bind_port, peer_ip.c_str(), peer_port);

    Endpoint ep(t);
    ep.setDeviceId(0x524F424Fu);       // "ROBO"
    ep.setHeartbeatInterval(std::chrono::milliseconds(1000));
    ep.setLinkTimeout(std::chrono::milliseconds(3000));
    ep.onState([](Endpoint::ConnState s) {
        static const char* names[] = {"Idle", "Connecting", "Active", "Lost"};
        std::printf("receiver: link state -> %s\n", names[int(s)]);
    });
    RobotState rs;
    rs.last_cmd = clk::now();

    const f32 wheel_radius = 0.05f, half_length = 0.20f, half_width = 0.20f;
    const auto failsafe_ms = std::chrono::milliseconds(500);

    ep.onMessage([&](const FrameHeader& h, const u8* pl, std::size_t n) {
        rs.last_cmd = clk::now();
        switch (MsgType(h.msg_type)) {
            case MsgType::SetVelocity: {
                VelocityCmd c;
                if (decodeVelocity(pl, n, c)) { rs.vx = c.vx; rs.vy = c.vy; rs.wz = c.wz; }
                break;
            }
            case MsgType::MoveOrder: {
                MoveOrder m;
                if (decodeMoveOrder(pl, n, m)) {
                    const f32 s = m.speed / 255.0f;
                    rs.vx = rs.vy = rs.wz = 0;
                    switch (m.dir) {
                        case MoveDir::Forward:     rs.vx =  1.0f * s; break;
                        case MoveDir::Backward:    rs.vx = -1.0f * s; break;
                        case MoveDir::StrafeLeft:  rs.vy =  1.0f * s; break;
                        case MoveDir::StrafeRight: rs.vy = -1.0f * s; break;
                        case MoveDir::RotateLeft:  rs.wz =  2.0f * s; break;
                        case MoveDir::RotateRight: rs.wz = -2.0f * s; break;
                        case MoveDir::Stop:        default: break;
                    }
                    std::printf("CMD move dir=%u speed=%u dur=%ums\n",
                                unsigned(m.dir), m.speed, m.duration_ms);
                }
                break;
            }
            case MsgType::SetArmJoints: {
                ArmJoints a;
                if (decodeArmJoints(pl, n, a) && a.arm_id < 4) {
                    JointLimit lim[kMaxJoints];
                    for (std::size_t i = 0; i < kMaxJoints; ++i) lim[i] = {-2.9f, 2.9f, 3.0f};
                    if (!clampJoints(lim, a.joints, a.count))
                        std::printf("CMD arm %u: joints clamped\n", a.arm_id);
                    for (u8 i = 0; i < a.count; ++i) rs.arm[a.arm_id][i] = a.joints[i];
                    rs.arm_count[a.arm_id] = a.count;
                }
                break;
            }
            case MsgType::Stop:
                rs.vx = rs.vy = rs.wz = 0;
                break;
            case MsgType::EmergencyStop:
                rs.estop = true;
                rs.vx = rs.vy = rs.wz = 0;
                std::printf("CMD EMERGENCY STOP\n");
                break;
            default: break;
        }
    });

    tryRealtime();

    const auto start = clk::now();
    auto next_telem = start;
    auto next_arm = start;
    unsigned status_hz = 0;

    while (g_run) {
        ep.poll(1);           // decode RX + dispatch
        ep.tick();            // session housekeeping

        // Failsafe: no valid command within timeout -> stop.
        if (!rs.estop && clk::now() - rs.last_cmd > failsafe_ms) {
            rs.vx = rs.vy = rs.wz = 0;
        }

        rs.wheels = mecanumInverse(rs.vx, rs.vy, rs.wz, wheel_radius, half_length, half_width);

        auto now = clk::now();

        // Uplink telemetry: base @200 Hz (best effort, FEC-only).
        if (now >= next_telem) {
            next_telem += std::chrono::milliseconds(5);
            BaseTelemetry bt;
            double tt = std::chrono::duration<double>(now - start).count();
            bt.x = f32(tt * 0.1); bt.y = 0; bt.yaw = f32(tt * 0.05);
            bt.vx = rs.vx; bt.vy = rs.vy; bt.wz = rs.wz; bt.battery = 87;
            u8 pl[64];
            std::size_t n = encodeBaseTelemetry(bt, pl, sizeof(pl));
            if (n) ep.send(MsgType::TelemetryBase, StreamTelemetry, pl, n);
        }

        // Arm telemetry @50 Hz, round robin over the 4 arms.
        if (now >= next_arm) {
            next_arm += std::chrono::milliseconds(20);
            static u8 arm_idx = 0;
            ArmTelemetry at;
            at.arm_id = arm_idx;
            at.count = rs.arm_count[arm_idx];
            for (u8 i = 0; i < at.count; ++i) at.joints[i] = rs.arm[arm_idx][i];
            arm_idx = u8((arm_idx + 1) & 3);
            u8 pl[4 + 4 * kMaxJoints];
            std::size_t n = encodeArmTelemetry(at, pl, sizeof(pl));
            if (n) ep.send(MsgType::TelemetryArm, StreamTelemetry, pl, n);
        }

        // Status @1 Hz.
        if (++status_hz >= 200) {
            status_hz = 0;
            StatusTelem st;
            st.mode = rs.estop ? 2 : 1;
            st.faults = 0;
            st.estop = rs.estop ? 1 : 0;
            u8 pl[16];
            std::size_t n = encodeStatus(st, pl, sizeof(pl));
            if (n) ep.send(MsgType::Status, StreamSession, pl, n);
            std::printf("wheels fl=%.1f fr=%.1f rl=%.1f rr=%.1f\n",
                        rs.wheels.fl, rs.wheels.fr, rs.wheels.rl, rs.wheels.rr);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::printf("receiver: done\n");
    return 0;
}
