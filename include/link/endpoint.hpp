#pragma once
#include <chrono>
#include <cstdint>
#include <functional>

#include "link/common.hpp"
#include "link/frame.hpp"
#include "link/transport.hpp"

namespace robotlink {

// Bidirectional endpoint: frame I/O, per-stream sequence spaces, duplicate
// suppression and stream-priority dispatch. Delivery is best-effort: the radio
// link's FEC (below us) protects against loss, so there is no ARQ/retransmit
// and no ACK/NACK.
class Endpoint {
public:
    enum class ConnState : u8 { Idle, Connecting, Active, Lost };

    using Handler      = std::function<void(const FrameHeader&, const u8*, std::size_t)>;
    using StateHandler = std::function<void(ConnState)>;

    explicit Endpoint(Transport& t);

    void onMessage(Handler h) { on_msg_ = std::move(h); }
    void onState(StateHandler h) { on_state_ = std::move(h); }

    // Best-effort send (control / telemetry / session). Returns true on success.
    bool send(MsgType type, StreamId stream, const u8* payload, std::size_t plen,
              u8 flags = 0);

    // Read and dispatch frames, waiting up to timeout_ms for the first one.
    void poll(int timeout_ms);

    // Session housekeeping (heartbeat + link timeout). Call periodically.
    void tick();

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
    u16 nextSeq(StreamId s);
    bool isDuplicate(StreamId s, u16 seq);
    void setState(ConnState s);
    void handleSession(const FrameHeader& h, const u8* pl, std::size_t n);
    void sendHeartbeat();

    Transport& t_;
    Handler on_msg_;

    u16 seq_ctr_[StreamCount] = {0, 0, 0, 0};
    u16 last_rx_[StreamCount] = {0, 0, 0, 0};
    u64 win_[StreamCount]     = {0, 0, 0, 0};
    bool have_rx_[StreamCount] = {false, false, false, false};

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
