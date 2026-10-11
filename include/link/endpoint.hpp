#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>

#include "link/common.hpp"
#include "link/frame.hpp"
#include "link/transport.hpp"

namespace robotlink {

// Bidirectional endpoint: frame I/O, per-stream sequence spaces, duplicate
// suppression, explicit ACK/NACK, and a small stop-and-wait style ARQ for
// downlink commands (ACK + retransmit).
class Endpoint {
public:
    using Handler    = std::function<void(const FrameHeader&, const u8*, std::size_t)>;
    using AckHandler = std::function<void(u16 ack_seq, u8 status)>;

    explicit Endpoint(Transport& t);

    void onMessage(Handler h) { on_msg_ = std::move(h); }
    void onAck(AckHandler h)  { on_ack_ = std::move(h); }

    // Best-effort send (telemetry / acks / session). Returns true on success.
    bool send(MsgType type, StreamId stream, const u8* payload, std::size_t plen,
              u16 ack_seq = 0, u8 flags = 0);

    // Reliable send: sets ACK_REQ + payload CRC, tracks the frame until acked.
    // Returns the assigned sequence number (0 on failure).
    u16 sendReliable(MsgType type, StreamId stream, const u8* payload, std::size_t plen);

    // Read and dispatch frames, waiting up to timeout_ms for the first one.
    void poll(int timeout_ms);

    // Advance ARQ retransmission timers. Call periodically (e.g. every 5 ms).
    void tick();

    void sendAckFor(u16 seq, u8 status = 0);
    void sendNackFor(u16 seq, u8 reason = 0);

    // Piggyback an ACK for an uplink seq on the next outbound frame.
    void setPiggybackAck(u16 seq) { piggyback_ack_ = seq; }

    void setRto(std::chrono::milliseconds v) { rto_ = v; }
    void setMaxRetries(int v) { max_retries_ = v; }
    std::size_t pendingCount();

private:
    struct Pending {
        u8 frame[kMaxFrame];
        std::size_t len = 0;
        std::chrono::steady_clock::time_point sent;
        int retries = 0;
    };

    u16 nextSeq(StreamId s);
    bool isDuplicate(StreamId s, u16 seq);

    Transport& t_;
    Handler on_msg_;
    AckHandler on_ack_;

    u16 seq_ctr_[StreamCount] = {0, 0, 0};
    u16 last_rx_[StreamCount] = {0, 0, 0};
    u64 win_[StreamCount]     = {0, 0, 0};
    bool have_rx_[StreamCount] = {false, false, false};

    std::map<u16, Pending> pending_;   // keyed by command seq (control stream)
    std::mutex mtx_;
    std::chrono::milliseconds rto_{30};
    int max_retries_ = 5;
    u16 piggyback_ack_ = 0;
};

} // namespace robotlink
