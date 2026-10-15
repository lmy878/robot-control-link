// Passive link monitor. It is a pure observer: it never sends control data,
// never sends ACKs and does not take part in session/scheduling. If the monitor
// is switched off, crashes or loses its connection, the Controller <-> Robot
// link is completely unaffected.
//
// It receives a copy of the radio traffic (both directions) — in a WFB-ng
// deployment the monitor's own `wfb_rx` forwards every overheard frame to this
// socket — and records Controller -> Robot and Robot -> Controller frames,
// infers packet loss from sequence gaps, tracks link state, writes a match log
// and decodes the application packets for live state analysis.
//
// Usage: monitor [bind_ip] [bind_port] [log_path]
//   defaults: 0.0.0.0  5700  monitor_log.csv
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "link/frame.hpp"
#include "link/monitor.hpp"
#include "link/transport.hpp"

using namespace robotlink;
using clk = std::chrono::steady_clock;

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run = false; }

static void printStatus(const Monitor& mon, const std::array<u64, 2>& rate) {
    const auto& ra = mon.analysis();
    const DirectionStats& cr = mon.direction(LinkDirection::ControllerToRobot);
    const DirectionStats& rc = mon.direction(LinkDirection::RobotToController);
    std::printf(
        "[mnt] C->R %s rx=%llu/s(%llu) loss=%.2f%% | "
        "R->C %s rx=%llu/s(%llu) loss=%.2f%% | "
        "mode=%u estop=%u qbad=%llu laser=%s\n",
        mon.linkUp(LinkDirection::ControllerToRobot) ? "UP  " : "DOWN",
        (unsigned long long)rate[0], (unsigned long long)cr.frames,
        mon.lossRate(LinkDirection::ControllerToRobot) * 100.0,
        mon.linkUp(LinkDirection::RobotToController) ? "UP  " : "DOWN",
        (unsigned long long)rate[1], (unsigned long long)rc.frames,
        mon.lossRate(LinkDirection::RobotToController) * 100.0,
        unsigned(ra.mode), ra.estop ? 1u : 0u,
        (unsigned long long)ra.bad_quaternions,
        ra.have_laser ? "ok" : "n/a");
}

int main(int argc, char** argv) {
    std::string bind_ip = (argc > 1) ? argv[1] : "0.0.0.0";
    u16 bind_port = (argc > 2) ? u16(std::atoi(argv[2])) : 5700;
    std::string log_path = (argc > 3) ? argv[3] : "monitor_log.csv";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    UdpTransport t;
    if (!t.openMonitor(bind_ip, bind_port)) {
        std::fprintf(stderr, "monitor: cannot bind udp %u\n", bind_port);
        return 1;
    }

    Monitor mon;
    if (!log_path.empty() && !mon.openLog(log_path))
        std::fprintf(stderr, "monitor: cannot open log %s (continuing without)\n",
                     log_path.c_str());
    std::printf("monitor: bind=%s:%u log=%s (passive, never transmits)\n",
                bind_ip.c_str(), bind_port, log_path.c_str());

    u8 buf[kMaxFrame];
    auto next_print = clk::now() + std::chrono::seconds(1);
    std::array<u64, 2> prev{{0, 0}};

    while (g_run) {
        // Drain everything currently queued (bounded), then do housekeeping.
        for (int drained = 0; drained < 256; ++drained) {
            int n = t.recv(buf, sizeof(buf), drained == 0 ? 100 : 0);
            if (n <= 0) break;

            FrameHeader h;
            if (!decodeHeader(buf, static_cast<std::size_t>(n), h)) continue;

            const u8* pl = buf + kHeaderSize;
            std::size_t plen = h.payload_len;
            if (plen > kMaxPayload) continue;

            const auto now = clk::now();
            const LinkDirection d = directionOf(MsgType(h.msg_type));
            const bool fresh = mon.observe(h, pl, plen, std::size_t(n), now);
            if (fresh) mon.logFrame(d, h, std::size_t(n), now);
        }

        const auto now = clk::now();
        mon.tick(now);

        if (now >= next_print) {
            next_print += std::chrono::seconds(1);
            std::array<u64, 2> rate{{
                mon.direction(LinkDirection::ControllerToRobot).frames - prev[0],
                mon.direction(LinkDirection::RobotToController).frames - prev[1],
            }};
            prev[0] = mon.direction(LinkDirection::ControllerToRobot).frames;
            prev[1] = mon.direction(LinkDirection::RobotToController).frames;
            printStatus(mon, rate);
        }
    }

    mon.closeLog();
    const auto& ra = mon.analysis();
    std::printf(
        "{\"role\":\"monitor\",\"c2r\":{\"frames\":%llu,\"loss\":%.4f,\"up\":%s},"
        "\"r2c\":{\"frames\":%llu,\"loss\":%.4f,\"up\":%s},"
        "\"control_frames\":%llu,\"telemetry_frames\":%llu,\"estop\":%u,\"mode\":%u}\n",
        (unsigned long long)mon.direction(LinkDirection::ControllerToRobot).frames,
        mon.lossRate(LinkDirection::ControllerToRobot),
        mon.linkUp(LinkDirection::ControllerToRobot) ? "true" : "false",
        (unsigned long long)mon.direction(LinkDirection::RobotToController).frames,
        mon.lossRate(LinkDirection::RobotToController),
        mon.linkUp(LinkDirection::RobotToController) ? "true" : "false",
        (unsigned long long)ra.control_frames,
        (unsigned long long)ra.telemetry_frames,
        ra.estop ? 1u : 0u, unsigned(ra.mode));
    return 0;
}
