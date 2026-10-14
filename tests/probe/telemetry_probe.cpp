// tests/probe/telemetry_probe.cpp
//
// Uplink performance probe (test tool). Sends 340-byte TelemetryPacket frames
// on the telemetry stream at a fixed rate and, on the receive side, measures
// one-way latency, RFC 3550 interarrival jitter and packet error rate.
//
// The probe reuses the production TelemetryPacket but repurposes the first two
// f32 slots of `states[]` to carry a little-endian IEEE-754 double holding a
// CLOCK_MONOTONIC timestamp (seconds). That lets two processes on the same host
// compute true one-way latency without touching the on-wire protocol.
//
// Usage:
//   telemetry_probe tx <bind_port> <peer_ip> <peer_port> <duration_s> <rate_hz>
//   telemetry_probe rx <bind_port> <peer_ip> <peer_port> <duration_s> <rate_hz>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "link/endpoint.hpp"
#include "link/telemetry.hpp"
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
        std::fprintf(stderr, "telemetry_probe: cannot bind %u\n", bind_port);
        return 1;
    }
    Endpoint ep(t);

    if (role == "tx") {
        auto next = clk::now();
        const auto period = std::chrono::microseconds((long)(1e6 / (rate > 0 ? rate : 200.0)));
        const double start = nowS();
        u64 sent = 0;
        while (g_run && (nowS() - start) < duration) {
            next += period;
            TelemetryPacket tp{};
            putDouble(tp.states, nowS());          // [0,1] -> send timestamp
            tp.states[2] = f32(double(sent));      // counter (cross-check)
            u8 pl[kTelemetrySize];
            std::size_t n = encodeTelemetry(tp, pl, sizeof(pl));
            if (n) { ep.send(MsgType::Telemetry, StreamTelemetry, pl, n); ++sent; }
            std::this_thread::sleep_until(next);
        }
        std::printf("{\"role\":\"tx\",\"sent\":%llu,\"duration_s\":%.3f,\"rate_hz\":%.1f}\n",
                    (unsigned long long)sent, duration, rate);
        return 0;
    }

    // ---- rx ----
    std::vector<double> lat_ms;
    std::set<u16> seqs;
    double jitter_s = 0.0, last_s = 0.0, last_r = 0.0;
    bool first = true;
    u64 received = 0, bad = 0;

    ep.onMessage([&](const FrameHeader& h, const u8* pl, std::size_t n) {
        if (h.msg_type != u8(MsgType::Telemetry)) return;
        TelemetryPacket tp;
        if (!decodeTelemetry(pl, n, tp)) { ++bad; return; }
        double s = getDouble(tp.states);
        double r = nowS();
        lat_ms.push_back((r - s) * 1000.0);
        seqs.insert(h.seq);
        if (!first) {
            double d = (r - last_r) - (s - last_s);     // RFC 3550 D(i-1,i)
            jitter_s += (std::fabs(d) - jitter_s) / 16.0;
        }
        last_s = s; last_r = r; first = false;
        ++received;
    });

    const double start = nowS();
    while (g_run && (nowS() - start) < duration) {
        ep.poll(1);
    }

    double jm = mean(lat_ms), js = stddev(lat_ms);
    std::printf(
        "{\"role\":\"rx\",\"received\":%llu,\"unique_seq\":%zu,\"bad\":%llu,"
        "\"latency_ms\":{\"mean\":%.3f,\"p50\":%.3f,\"p95\":%.3f,\"p99\":%.3f,"
        "\"min\":%.3f,\"max\":%.3f,\"stddev\":%.3f},"
        "\"jitter_ms\":{\"rfc3550\":%.3f,\"stddev\":%.3f}}\n",
        (unsigned long long)received, seqs.size(), (unsigned long long)bad,
        jm, percentile(lat_ms, 50), percentile(lat_ms, 95), percentile(lat_ms, 99),
        lat_ms.empty() ? 0.0 : *std::min_element(lat_ms.begin(), lat_ms.end()),
        lat_ms.empty() ? 0.0 : *std::max_element(lat_ms.begin(), lat_ms.end()), js,
        jitter_s * 1000.0, js);
    return 0;
}
