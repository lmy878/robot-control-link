#include "link/endpoint.hpp"
#include "link/crc.hpp"
#include "link/messages.hpp"

namespace robotlink {

Endpoint::Endpoint(Transport& t) : t_(t) {}

u16 Endpoint::nextSeq(StreamId s) {
    auto i = static_cast<std::size_t>(s);
    return ++seq_ctr_[i];
}

bool Endpoint::isDuplicate(StreamId s, u16 seq) {
    auto i = static_cast<std::size_t>(s);
    if (!have_rx_[i]) {
        have_rx_[i] = true;
        last_rx_[i] = seq;
        win_[i] = 1;
        return false;
    }
    u16 last = last_rx_[i];
    u16 diff = u16(seq - last);          // modular distance
    if (diff == 0) return true;          // exact duplicate
    if (diff < 0x8000) {                 // newer
        win_[i] = (diff >= 64) ? 1u : ((win_[i] << diff) | 1u);
        last_rx_[i] = seq;
        return false;
    }
    // older
    u16 back = u16(last - seq);
    if (back >= 64) return true;         // far too old -> drop
    u64 bit = 1ull << back;
    if (win_[i] & bit) return true;
    win_[i] |= bit;
    return false;
}

bool Endpoint::send(MsgType type, StreamId stream, const u8* payload, std::size_t plen,
                    u8 flags) {
    FrameHeader h;
    h.msg_type  = u8(type);
    h.flags     = flags;
    h.stream_id = u8(stream);
    h.seq       = nextSeq(stream);

    u8 out[kMaxFrame];
    std::size_t n = buildFrame(h, payload, plen, out, sizeof(out));
    if (n == 0) return false;
    return t_.send(out, n) == static_cast<int>(n);
}

void Endpoint::poll(int timeout_ms) {
    // Receive everything available, then dispatch in stream-priority order:
    // control (highest) first, debug (lowest) last.
    struct Item {
        int prio;
        FrameHeader h;
        std::size_t len;
        u8 payload[kMaxPayload];
    };
    static constexpr std::size_t kMaxBatch = 32;
    Item items[kMaxBatch];
    std::size_t count = 0;

    u8 buf[kMaxFrame];
    for (;;) {
        int n = t_.recv(buf, sizeof(buf), timeout_ms);
        if (n <= 0) break;

        FrameHeader h;
        if (!decodeHeader(buf, static_cast<std::size_t>(n), h)) continue;
        timeout_ms = 0; // got one frame: drain the rest without blocking

        const u8* pl = buf + kHeaderSize;
        std::size_t plen = h.payload_len;
        if (h.flags & FlagPayloadCrc) {
            if (plen < 2) continue;
            std::size_t real = plen - 2;
            if (getU16(pl + real) != crc16(pl, real)) continue;
            plen = real;
        }

        // Any valid frame proves the link is alive.
        last_rx_time_ = std::chrono::steady_clock::now();
        if (conn_state_ == ConnState::Lost) setState(ConnState::Active);

        // Duplicate suppression (best-effort: duplicates are simply dropped).
        if (isDuplicate(static_cast<StreamId>(h.stream_id), h.seq))
            continue;

        if (count < kMaxBatch && plen <= kMaxPayload) {
            Item& it = items[count++];
            it.prio = streamPriority(static_cast<StreamId>(h.stream_id));
            it.h = h;
            it.len = plen;
            if (plen) std::memcpy(it.payload, pl, plen);
        } else {
            // batch full: deliver immediately (still correct, just unordered)
            handleSession(h, pl, plen);
            if (on_msg_) on_msg_(h, pl, plen);
        }
    }

    // Stable insertion sort by priority (lowest number = highest priority).
    for (std::size_t i = 1; i < count; ++i) {
        Item key = items[i];
        std::size_t j = i;
        while (j > 0 && items[j - 1].prio > key.prio) {
            items[j] = items[j - 1];
            --j;
        }
        items[j] = key;
    }

    for (std::size_t i = 0; i < count; ++i) {
        handleSession(items[i].h, items[i].payload, items[i].len);
        if (on_msg_) on_msg_(items[i].h, items[i].payload, items[i].len);
    }
}

void Endpoint::tick() {
    auto now = std::chrono::steady_clock::now();

    // Session housekeeping: heartbeat + link timeout (paused unless enabled).
    if (session_enabled_ && conn_state_ != ConnState::Idle) {
        if (now >= next_hb_time_) {
            if (conn_state_ == ConnState::Connecting) beginConnect(nonce_);
            else sendHeartbeat();
            next_hb_time_ = now + hb_interval_;
        }
        if (conn_state_ == ConnState::Active && now - last_rx_time_ > link_timeout_)
            setState(ConnState::Lost);
    }
}

void Endpoint::setState(ConnState s) {
    if (s == conn_state_) return;
    conn_state_ = s;
    if (on_state_) on_state_(s);
}

void Endpoint::beginConnect(u32 nonce) {
    session_enabled_ = true;
    is_controller_ = true;
    nonce_ = nonce;
    u8 pl[16];
    ConnectReqMsg req;
    req.version = kVersion;
    req.device_id = device_id_;
    req.nonce = nonce;
    std::size_t n = encodeConnectReq(req, pl, sizeof(pl));
    if (n) send(MsgType::ConnectReq, StreamSession, pl, n);
    if (conn_state_ == ConnState::Idle) setState(ConnState::Connecting);
    next_hb_time_ = std::chrono::steady_clock::now() + hb_interval_;
}

void Endpoint::sendHeartbeat() {
    MsgType t = is_controller_ ? MsgType::Heartbeat : MsgType::HeartbeatUp;
    send(t, StreamSession, nullptr, 0);
}

void Endpoint::handleSession(const FrameHeader& h, const u8* pl, std::size_t n) {
    if (!session_enabled_) return;   // session features paused
    switch (MsgType(h.msg_type)) {
        case MsgType::ConnectReq: {
            if (is_controller_) break;                // only the robot answers
            ConnectReqMsg req;
            if (!decodeConnectReq(pl, n, req)) break;
            session_id_ = req.nonce ? req.nonce : 0x00000001u;
            u8 buf[16];
            ConnectAckMsg ack;
            ack.session_id = session_id_;
            ack.status = 0;
            ack.capabilities = 0x0001;
            std::size_t m = encodeConnectAck(ack, buf, sizeof(buf));
            if (m) send(MsgType::ConnectAck, StreamSession, buf, m);
            next_hb_time_ = std::chrono::steady_clock::now() + hb_interval_;
            setState(ConnState::Active);
            break;
        }
        case MsgType::ConnectAck: {
            if (!is_controller_) break;               // only the controller awaits it
            ConnectAckMsg ack;
            if (decodeConnectAck(pl, n, ack) && ack.status == 0) {
                session_id_ = ack.session_id;
                setState(ConnState::Active);
            }
            break;
        }
        default: break;
    }
}

} // namespace robotlink
