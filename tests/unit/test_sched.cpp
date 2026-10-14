// tests/unit/test_sched.cpp
//
// T5 — stream priority & duplicate suppression. Uses an in-memory mock
// transport so the Endpoint dispatch order can be observed deterministically.
//
// Verifies:
//   * incoming frames are dispatched highest-priority-first (control > tele > debug)
//   * arrival order is preserved within a stream
//   * duplicate sequence numbers are dropped (dispatched only once)
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

#include "link/endpoint.hpp"
#include "link/frame.hpp"
#include "link/transport.hpp"

using namespace robotlink;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* name) {
    if (ok) ++g_pass;
    else { ++g_fail; std::fprintf(stderr, "  FAIL: %s\n", name); }
}

struct MockTransport : Transport {
    std::deque<std::vector<u8>> rx;
    std::vector<std::vector<u8>> tx;

    int send(const u8* data, std::size_t len) override {
        tx.emplace_back(data, data + len);
        return int(len);
    }
    int recv(u8* data, std::size_t cap, int) override {
        if (rx.empty()) return 0;
        std::vector<u8> f = rx.front();
        rx.pop_front();
        std::size_t n = f.size() < cap ? f.size() : cap;
        std::memcpy(data, f.data(), n);
        return int(n);
    }
};

static std::vector<u8> makeFrame(u8 type, u8 stream, u16 seq, u8 flags = 0) {
    FrameHeader h;
    h.msg_type = type;
    h.stream_id = stream;
    h.seq = seq;
    h.flags = flags;
    std::vector<u8> out(kMaxFrame);
    std::size_t n = buildFrame(h, nullptr, 0, out.data(), out.size());
    out.resize(n);
    return out;
}

int main() {
    // ---- priority ordering ----
    {
        MockTransport m;
        // Enqueue deliberately in reverse priority order.
        m.rx.push_back(makeFrame(u8(MsgType::EventLog), u8(StreamDebug), 1));
        m.rx.push_back(makeFrame(u8(MsgType::Telemetry), u8(StreamTelemetry), 1));
        m.rx.push_back(makeFrame(u8(MsgType::ControlCmd), u8(StreamControl), 1));

        Endpoint ep(m);
        std::vector<u8> order;
        ep.onMessage([&](const FrameHeader& h, const u8*, std::size_t) {
            order.push_back(h.stream_id);
        });
        ep.poll(0);

        bool sorted = order.size() == 3 &&
                      order[0] == u8(StreamControl) &&
                      order[1] == u8(StreamTelemetry) &&
                      order[2] == u8(StreamDebug);
        check(sorted, "dispatch order control -> telemetry -> debug");
    }

    // ---- arrival order preserved within a stream ----
    {
        MockTransport m;
        m.rx.push_back(makeFrame(u8(MsgType::Telemetry), u8(StreamTelemetry), 10));
        m.rx.push_back(makeFrame(u8(MsgType::Telemetry), u8(StreamTelemetry), 11));
        m.rx.push_back(makeFrame(u8(MsgType::Telemetry), u8(StreamTelemetry), 12));

        Endpoint ep(m);
        std::vector<u16> seqs;
        ep.onMessage([&](const FrameHeader& h, const u8*, std::size_t) { seqs.push_back(h.seq); });
        ep.poll(0);
        check(seqs.size() == 3 && seqs[0] == 10 && seqs[1] == 11 && seqs[2] == 12,
              "intra-stream arrival order preserved");
    }

    // ---- duplicate suppression ----
    {
        MockTransport m;
        m.rx.push_back(makeFrame(u8(MsgType::ControlCmd), u8(StreamControl), 42));
        m.rx.push_back(makeFrame(u8(MsgType::ControlCmd), u8(StreamControl), 42));

        Endpoint ep(m);
        int dispatched = 0;
        ep.onMessage([&](const FrameHeader&, const u8*, std::size_t) { ++dispatched; });
        ep.poll(0);

        check(dispatched == 1, "duplicate dispatched exactly once");
        check(m.tx.empty(), "duplicate produces no reply");
    }

    std::printf("{\"suite\":\"sched\",\"pass\":%d,\"fail\":%d,\"total\":%d}\n",
                g_pass, g_fail, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
