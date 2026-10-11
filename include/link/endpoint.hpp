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
    enum class ConnState : u8 { Idle, Connecting, Active, Lost };

    using Handler      = std::function<void(const FrameHeader&, const u8*, std::size_t)>;
    using AckHandler   = std::function<void(u16 ack_seq, u8 status)>;
    using StateHandler = std::function<void(ConnState)>;

    explicit Endpoint(Transport& t);

    void onMessage(Handler h) { on_msg_ = std::move(h); }
    void onAck(AckHandler h)  { on_ack_ = std::move(h); }
    void onState(StateHandler h) { on_state_ = std::move(h); }

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

    // ---- session / link monitoring ----
    void beginConnect(u32 nonce = 0);              // controller initiates handshake
    void setDeviceId(u32 id) { device_id_ = id; session_enabled_ = true; } // robot identity
    bool sessionEnabled() const { return session_enabled_; }
    void setSessionEnabled(bool on) { session_enabled_ = on; }
    ConnState connState() const { return conn_state_; }
    u32 sessionId() const { return session_id_; }
    void setHeartbeatInterval(std::chrono::milliseconds v) { hb_interval_ = v; }
    void setLinkTimeout(std::chrono::milliseconds v) { link_timeout_ = v; }

private:
    struct Pending {
        u8 frame[kMaxFrame];
        std::size_t len = 0;
        std::chrono::steady_clock::time_point sent;
        int retries = 0;
    };

    u16 nextSeq(StreamId s);
    bool isDuplicate(StreamId s, u16 seq);
    void setState(ConnState s);
    void handleSession(const FrameHeader& h, const u8* pl, std::size_t n);
    void sendHeartbeat();

    Transport& t_;
    Handler on_msg_;
    AckHandler on_ack_;

    u16 seq_ctr_[StreamCount] = {0, 0, 0, 0};
    u16 last_rx_[StreamCount] = {0, 0, 0, 0};
    u64 win_[StreamCount]     = {0, 0, 0, 0};
    bool have_rx_[StreamCount] = {false, false, false, false};

    std::map<u16, Pending> pending_;   // keyed by command seq (control stream)
    std::mutex mtx_;
    std::chrono::milliseconds rto_{30};
    int max_retries_ = 5;
    u16 piggyback_ack_ = 0;

    // session state
    ConnState conn_state_ = ConnState::Idle;
    StateHandler on_state_;
    bool session_enabled_ = false;   // paused by default
    bool is_controller_ = false;
    u32 device_id_ = 0;
    u32 session_id_ = 0;
    u32 nonce_ = 0;
    std::chrono::steady_clock::time_point last_rx_time_{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point next_hb_time_{std::chrono::steady_clock::now()};
    std::chrono::milliseconds hb_interval_{1000};
    std::chrono::milliseconds link_timeout_{3000};
};

} // namespace robotlink
