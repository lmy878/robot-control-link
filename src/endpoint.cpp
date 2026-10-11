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
                    u16 ack_seq, u8 flags) {
    FrameHeader h;
    h.msg_type  = u8(type);
    h.flags     = flags;
    h.stream_id = u8(stream);
    h.seq       = nextSeq(stream);
    h.ack_seq   = ack_seq ? ack_seq : piggyback_ack_;
    piggyback_ack_ = 0;

    u8 out[kMaxFrame];
    std::size_t n = buildFrame(h, payload, plen, out, sizeof(out));
    if (n == 0) return false;
    return t_.send(out, n) == static_cast<int>(n);
}

u16 Endpoint::sendReliable(MsgType type, StreamId stream, const u8* payload, std::size_t plen) {
    FrameHeader h;
    h.msg_type  = u8(type);
    h.flags     = FlagAckReq | FlagPayloadCrc;
    h.stream_id = u8(stream);
    h.seq       = nextSeq(stream);
    h.ack_seq   = piggyback_ack_;
    piggyback_ack_ = 0;

    Pending p;
    p.len = buildFrame(h, payload, plen, p.frame, sizeof(p.frame));
    if (p.len == 0) return 0;
    p.sent = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lk(mtx_);
        pending_[h.seq] = p;
    }
    t_.send(p.frame, p.len);
    return h.seq;
}

void Endpoint::poll(int timeout_ms) {
    u8 buf[kMaxFrame];
    for (;;) {
        int n = t_.recv(buf, sizeof(buf), timeout_ms);
        if (n <= 0) return;

        FrameHeader h;
        if (!decodeHeader(buf, static_cast<std::size_t>(n), h)) continue;

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

        // ACK / NACK handling
        if (h.flags & FlagIsAck) {
            {
                std::lock_guard<std::mutex> lk(mtx_);
                pending_.erase(h.ack_seq);
            }
            if (on_ack_) on_ack_(h.ack_seq, 0);
        }
        if (h.flags & FlagIsNack) {
            if (on_ack_) on_ack_(h.ack_seq, 1);
        }

        // Duplicate suppression + re-ack
        bool dup = isDuplicate(static_cast<StreamId>(h.stream_id), h.seq);
        if (dup) {
            if (h.flags & FlagAckReq) sendAckFor(h.seq, 0);
            timeout_ms = 0;
            continue;
        }

        if (h.flags & FlagAckReq) sendAckFor(h.seq, 0);
        handleSession(h, pl, plen);
        if (on_msg_) on_msg_(h, pl, plen);

        timeout_ms = 0; // drain any queued frames
    }
}

void Endpoint::tick() {
    auto now = std::chrono::steady_clock::now();

    // Session housekeeping: heartbeat + link timeout.
    if (conn_state_ != ConnState::Idle) {
        if (now >= next_hb_time_) {
            if (conn_state_ == ConnState::Connecting) beginConnect(nonce_);
            else sendHeartbeat();
            next_hb_time_ = now + hb_interval_;
        }
        if (conn_state_ == ConnState::Active && now - last_rx_time_ > link_timeout_)
            setState(ConnState::Lost);
    }

    std::lock_guard<std::mutex> lk(mtx_);
    for (auto it = pending_.begin(); it != pending_.end();) {
        Pending& p = it->second;
        if (now - p.sent < rto_) { ++it; continue; }

        if (p.retries >= max_retries_) {
            it = pending_.erase(it);
            continue;
        }
        FrameHeader h;
        if (decodeHeader(p.frame, p.len, h)) {
            h.flags |= FlagRetransmit;
            encodeHeader(h, p.frame);
        }
        t_.send(p.frame, p.len);
        p.sent = now;
        ++p.retries;
        ++it;
    }
}

void Endpoint::sendAckFor(u16 seq, u8 status) {
    u8 pl[8];
    AckMsg a; a.ack_seq = seq; a.status = status;
    std::size_t n = encodeAck(a, pl, sizeof(pl));
    if (n == 0) return;
    send(MsgType::Ack, StreamControl, pl, n, seq, FlagIsAck);
}

void Endpoint::sendNackFor(u16 seq, u8 reason) {
    u8 pl[8];
    AckMsg a; a.ack_seq = seq; a.status = reason;
    std::size_t n = encodeAck(a, pl, sizeof(pl));
    if (n == 0) return;
    send(MsgType::Nack, StreamControl, pl, n, seq, FlagIsNack);
}

std::size_t Endpoint::pendingCount() {
    std::lock_guard<std::mutex> lk(mtx_);
    return pending_.size();
}

void Endpoint::setState(ConnState s) {
    if (s == conn_state_) return;
    conn_state_ = s;
    if (on_state_) on_state_(s);
}

void Endpoint::beginConnect(u32 nonce) {
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
