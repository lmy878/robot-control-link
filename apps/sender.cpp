// Controller side: sends movement orders + velocity setpoints (downlink) and
// receives telemetry (uplink). Downlink commands are best-effort / FEC-only.
//
// Usage: sender [peer_ip] [peer_port] [bind_port]
//   defaults: 127.0.0.1  5600  5601
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "link/endpoint.hpp"
#include "link/messages.hpp"
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
    std::setvbuf(stdout, nullptr, _IOLBF, 0);   // live output when redirected

    UdpTransport t;
    if (!t.open("0.0.0.0", bind_port, peer_ip, peer_port)) {
        std::fprintf(stderr, "sender: cannot bind udp %u\n", bind_port);
        return 1;
    }
    std::printf("sender: bind=%u peer=%s:%u\n", bind_port, peer_ip.c_str(), peer_port);

    Endpoint ep(t);
    ep.setHeartbeatInterval(std::chrono::milliseconds(1000));
    ep.setLinkTimeout(std::chrono::milliseconds(3000));
    ep.onState([](Endpoint::ConnState s) {
        static const char* names[] = {"Idle", "Connecting", "Active", "Lost"};
        std::printf("sender: link state -> %s\n", names[int(s)]);
    });
    ep.beginConnect(0xABCD1234u);      // start the handshake

    ep.onMessage([&](const FrameHeader& h, const u8* pl, std::size_t n) {
        switch (MsgType(h.msg_type)) {
            case MsgType::TelemetryBase: {
                static unsigned telem_n = 0;
                if (++telem_n % 200 != 0) break;   // print ~1 Hz
                BaseTelemetry bt;
                if (decodeBaseTelemetry(pl, n, bt))
                    std::printf("TELEM x=%.2f y=%.2f yaw=%.2f v=(%.2f,%.2f,%.2f) batt=%u%%\n",
                                bt.x, bt.y, bt.yaw, bt.vx, bt.vy, bt.wz, bt.battery);
                break;
            }
            case MsgType::Status: {
                StatusTelem st;
                if (decodeStatus(pl, n, st))
                    std::printf("STATUS mode=%u faults=0x%04x estop=%u\n",
                                st.mode, st.faults, st.estop);
                break;
            }
            default: break;
        }
    });

    const auto start = clk::now();
    auto next = start;
    MoveDir last_dir = MoveDir::Stop;
    unsigned ticks = 0;

    while (g_run) {
        next += std::chrono::milliseconds(10); // 100 Hz downlink

        double phase = std::chrono::duration<double>(clk::now() - start).count();
        double slot = std::fmod(phase, 7.0);

        VelocityCmd v{};
        MoveDir dir;
        if (slot < 2.0)       { v.vx = 0.5f; dir = MoveDir::Forward; }
        else if (slot < 4.0)  { v.vy = 0.5f; dir = MoveDir::StrafeLeft; }
        else if (slot < 6.0)  { v.wz = 1.0f; dir = MoveDir::RotateLeft; }
        else                  { dir = MoveDir::Stop; }

        // Discrete order on state change: best-effort (FEC-only).
        const bool active = ep.connState() == Endpoint::ConnState::Active;
        if (active && dir != last_dir) {
            MoveOrder mo{dir, u8(dir == MoveDir::Stop ? 0 : 180), 0};
            u8 pl[16];
            std::size_t n = encodeMoveOrder(mo, pl, sizeof(pl));
            if (n) ep.send(MsgType::MoveOrder, StreamControl, pl, n, FlagPayloadCrc);
            last_dir = dir;
        }

        // Continuous velocity setpoint: best-effort; the next setpoint simply
        // supersedes a lost one (latching on the receiver).
        if (active) {
            u8 pl[24];
            std::size_t n = encodeVelocity(v, pl, sizeof(pl));
            if (n) ep.send(MsgType::SetVelocity, StreamControl, pl, n, FlagPayloadCrc);
        }

        ep.poll(0);
        ep.tick();
        if (++ticks % 100 == 0)
            std::printf("sender: t=%.1fs\n", phase);

        std::this_thread::sleep_until(next);
    }
    std::printf("sender: done\n");
    return 0;
}
