// tests/probe/control_probe.cpp
//
// Reliable-downlink probe (test tool). The tx side pushes ControlPacket frames
// on the control stream with ACK + retransmit (Endpoint::sendReliable); the rx
// side receives them, de-duplicates by message counter and ACKs automatically.
//
// The probe repurposes ControlPacket.joystick[0] as a command counter and
// joystick[1..2] as a little-endian double holding a CLOCK_MONOTONIC timestamp,
// so the tx can report per-command ACK latency.
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
#include <unordered_map>
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

static double percentile(std::vector<double>& v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    double idx = (p / 100.0) * double(v.size() - 1);
    std::size_t lo = std::size_t(idx);
    std::size_t hi = std::min(lo + 1, v.size() - 1);
    double f = idx - double(lo);
    return v[lo] * (1.0 - f) + v[hi] * f;
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
    ep.setRto(std::chrono::milliseconds(30));
    ep.setMaxRetries(5);

    if (role == "tx") {
        std::atomic<u64> acked{0};
        std::unordered_map<u16, double> sent_at;
        std::vector<double> ack_lat_ms;

        ep.onAck([&](u16 seq, u8) {
            ++acked;
            auto it = sent_at.find(seq);
            if (it != sent_at.end()) {
                ack_lat_ms.push_back((nowS() - it->second) * 1000.0);
                sent_at.erase(it);
            }
        });

        const auto period = std::chrono::microseconds((long)(1e6 / (rate > 0 ? rate : 100.0)));
        auto next = clk::now();
        const double start = nowS();
        u64 sent = 0;
        u32 counter = 0;
        while (g_run && (nowS() - start) < duration) {
            next += period;
            if (ep.pendingCount() < 8) {
                ControlPacket cp{};
                cp.joystick[0] = f32(double(counter));
                putDouble(&cp.joystick[1], nowS());
                cp.mode = 1;
                u8 pl[kControlSize];
                std::size_t n = encodeControl(cp, pl, sizeof(pl));
                if (n) {
                    u16 seq = ep.sendReliable(MsgType::ControlCmd, StreamControl, pl, n);
                    if (seq) { ++sent; sent_at[seq] = nowS(); ++counter; }
                }
            }
            ep.poll(0);
            ep.tick();
            std::this_thread::sleep_until(next);
        }
        // Drain: let the ARQ finish confirming the commands still in flight so
        // the final ACK count is not truncated by shutdown.
        const double drain_until = nowS() + 0.5;
        while (g_run && nowS() < drain_until && ep.pendingCount() > 0) {
            ep.poll(1);
            ep.tick();
        }
        std::printf(
            "{\"role\":\"tx\",\"sent\":%llu,\"acked\":%llu,\"unacked\":%llu,"
            "\"ack_latency_ms\":{\"mean\":%.3f,\"p50\":%.3f,\"p95\":%.3f,\"p99\":%.3f,"
            "\"max\":%.3f}}\n",
            (unsigned long long)sent, (unsigned long long)acked.load(),
            (unsigned long long)(sent - acked.load()),
            ack_lat_ms.empty() ? 0.0 : [&]{ double s = 0; for (double x : ack_lat_ms) s += x; return s / ack_lat_ms.size(); }(),
            percentile(ack_lat_ms, 50), percentile(ack_lat_ms, 95), percentile(ack_lat_ms, 99),
            ack_lat_ms.empty() ? 0.0 : *std::max_element(ack_lat_ms.begin(), ack_lat_ms.end()));
        return 0;
    }

    // ---- rx ----
    std::unordered_set<u32> seen;
    u64 recv = 0, duplicates = 0;
    u32 min_c = 0, max_c = 0;
    bool first = true;
    ep.onMessage([&](const FrameHeader&, const u8* pl, std::size_t n) {
        ControlPacket cp;
        if (!decodeControl(pl, n, cp)) return;
        u32 c = u32(cp.joystick[0]);
        if (seen.insert(c).second) {
            if (first) { min_c = max_c = c; first = false; }
            else { if (c < min_c) min_c = c; if (c > max_c) max_c = c; }
            ++recv;
        } else {
            ++duplicates;
        }
    });

    const double start = nowS();
    while (g_run && (nowS() - start) < duration) {
        ep.poll(1);
    }
    u64 expected = first ? 0 : (u64(max_c) - u64(min_c) + 1);
    std::printf(
        "{\"role\":\"rx\",\"unique\":%llu,\"duplicates\":%llu,\"gaps\":%llu,"
        "\"min_counter\":%u,\"max_counter\":%u}\n",
        (unsigned long long)recv, (unsigned long long)duplicates,
        (unsigned long long)(expected > recv ? expected - recv : 0), min_c, max_c);
    return 0;
}
