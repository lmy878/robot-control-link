// tests/probe/control_probe.cpp
//
// Downlink performance probe (test tool). The tx side pushes ControlPacket
// frames on the control stream best-effort (FEC-only, no ACK/retransmit); the
// rx side receives them, de-duplicates by message counter and measures one-way
// latency / jitter.
//
// The probe repurposes ControlPacket.joystick[0] as a command counter and
// joystick[1..2] as a little-endian double holding a CLOCK_MONOTONIC timestamp,
// so the rx can compute per-command one-way latency.
//
// Usage:
//   control_probe tx <bind_port> <peer_ip> <peer_port> <duration_s> <rate_hz>
//   control_probe rx <bind_port> <peer_ip> <peer_port> <duration_s> <rate_hz>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "link/control.hpp"
#include "link/endpoint.hpp"
#include "link/transport.hpp"

using namespace robotlink;
using clk = std::chrono::steady_clock;

static std::atomic<bool> g_run{true};
static void onSig(int) { g_run = false; }

static double nowS() {
    return std::chrono::duration<double>(clk::now().time_since_epoch()).count();
}
static void putDouble(f32* dst, double v) { std::memcpy(dst, &v, 8); }
static double getDouble(const f32* src) { double v; std::memcpy(&v, src, 8); return v; }

static double percentile(std::vector<double>& v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    double idx = (p / 100.0) * double(v.size() - 1);
    std::size_t lo = std::size_t(idx);
    std::size_t hi = std::min(lo + 1, v.size() - 1);
    double f = idx - double(lo);
    return v[lo] * (1.0 - f) + v[hi] * f;
}
static double mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double s = 0;
    for (double x : v) s += x;
    return s / double(v.size());
}
static double stddev(const std::vector<double>& v) {
    if (v.size() < 2) return 0.0;
    double m = mean(v), s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / double(v.size() - 1));
}

int main(int argc, char** argv) {
    if (argc < 7) {
        std::fprintf(stderr,
            "usage: %s <tx|rx> <bind_port> <peer_ip> <peer_port> <duration_s> <rate_hz>\n",
            argv[0]);
        return 2;
    }
    std::string role = argv[1];
    u16 bind_port = u16(std::atoi(argv[2]));
    std::string peer_ip = argv[3];
    u16 peer_port = u16(std::atoi(argv[4]));
    double duration = std::atof(argv[5]);
    double rate = std::atof(argv[6]);

    std::signal(SIGINT, onSig);
    std::signal(SIGTERM, onSig);

    UdpTransport t;
    if (!t.open("0.0.0.0", bind_port, peer_ip, peer_port)) {
        std::fprintf(stderr, "control_probe: cannot bind %u\n", bind_port);
        return 1;
    }
    Endpoint ep(t);

    if (role == "tx") {
        const auto period = std::chrono::microseconds((long)(1e6 / (rate > 0 ? rate : 100.0)));
        auto next = clk::now();
        const double start = nowS();
        u64 sent = 0;
        u32 counter = 0;
        while (g_run && (nowS() - start) < duration) {
            next += period;
            ControlPacket cp{};
            cp.joystick[0] = f32(double(counter));
            putDouble(&cp.joystick[1], nowS());
            cp.mode = 1;
            u8 pl[kControlSize];
            std::size_t n = encodeControl(cp, pl, sizeof(pl));
            if (n && ep.send(MsgType::ControlCmd, StreamControl, pl, n, FlagPayloadCrc)) {
                ++sent;
                ++counter;
            }
            std::this_thread::sleep_until(next);
        }
        std::printf("{\"role\":\"tx\",\"sent\":%llu,\"duration_s\":%.3f,\"rate_hz\":%.1f}\n",
                    (unsigned long long)sent, duration, rate);
        return 0;
    }

    // ---- rx ----
    std::unordered_set<u32> seen;
    std::vector<double> lat_ms;
    double jitter_s = 0.0, last_s = 0.0, last_r = 0.0;
    bool first = true;
    u64 received = 0, duplicates = 0;

    ep.onMessage([&](const FrameHeader&, const u8* pl, std::size_t n) {
        ControlPacket cp;
        if (!decodeControl(pl, n, cp)) return;
        u32 c = u32(cp.joystick[0]);
        if (!seen.insert(c).second) { ++duplicates; return; }
        ++received;
        double s = getDouble(&cp.joystick[1]);
        double r = nowS();
        lat_ms.push_back((r - s) * 1000.0);
        if (!first) {
            double d = (r - last_r) - (s - last_s);     // RFC 3550 D(i-1,i)
            jitter_s += (std::fabs(d) - jitter_s) / 16.0;
        }
        last_s = s; last_r = r; first = false;
    });

    const double start = nowS();
    while (g_run && (nowS() - start) < duration) {
        ep.poll(1);
    }

    double js = stddev(lat_ms);
    std::printf(
        "{\"role\":\"rx\",\"received\":%llu,\"unique\":%zu,\"duplicates\":%llu,"
        "\"latency_ms\":{\"mean\":%.3f,\"p50\":%.3f,\"p95\":%.3f,\"p99\":%.3f,"
        "\"max\":%.3f,\"stddev\":%.3f},"
        "\"jitter_ms\":{\"rfc3550\":%.3f,\"stddev\":%.3f}}\n",
        (unsigned long long)received, seen.size(), (unsigned long long)duplicates,
        mean(lat_ms), percentile(lat_ms, 50), percentile(lat_ms, 95),
        percentile(lat_ms, 99),
        lat_ms.empty() ? 0.0 : *std::max_element(lat_ms.begin(), lat_ms.end()), js,
        jitter_s * 1000.0, js);
    return 0;
}
