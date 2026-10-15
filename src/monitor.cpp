#include "link/monitor.hpp"
#include "link/messages.hpp"

#include <cmath>
#include <cstring>
#include <ctime>

namespace robotlink {

Monitor::Monitor(std::chrono::milliseconds link_timeout)
    : link_timeout_(link_timeout) {}

bool Monitor::observe(const FrameHeader& h, const u8* payload, std::size_t payload_len,
                      std::size_t frame_len, std::chrono::steady_clock::time_point now) {
    const LinkDirection d = directionOf(MsgType(h.msg_type));
    const auto di = std::size_t(d);
    DirectionStats& ds = dirs_[di];

    // Unknown streams are still counted at the direction level.
    if (std::size_t(h.stream_id) >= StreamCount) {
        ds.frames++;
        ds.bytes += frame_len;
        ds.seen = true;
        ds.last_rx = now;
        ever_seen_[di] = true;
        up_[di] = true;
        return true;
    }

    StreamStats& ss = ds.streams[std::size_t(h.stream_id)];
    bool duplicate = false;
    if (ss.have_last) {
        const u16 diff = u16(h.seq - ss.last_seq);   // modular distance
        if (diff == 0) {
            duplicate = true;
        } else if (diff < 0x8000) {                  // newer
            if (diff > 1) ss.lost += diff - 1;       // gap(s) skipped
            ss.last_seq = h.seq;
        } else {                                     // older -> reordered
            ss.reordered++;
        }
    } else {
        ss.have_last = true;
        ss.last_seq = h.seq;
    }
    if (duplicate) {
        ss.duplicates++;
        return false;
    }

    ds.frames++;
    ds.bytes += frame_len;
    ds.seen = true;
    ds.last_rx = now;
    ss.received++;
    ss.bytes += frame_len;
    ever_seen_[di] = true;
    up_[di] = true;

    // ---- live application-level analysis ----
    switch (MsgType(h.msg_type)) {
        case MsgType::ControlCmd: {
            ControlPacket cp;
            if (decodeControl(payload, payload_len, cp)) {
                ana_.control_frames++;
                ana_.have_mode = true;
                ana_.mode = cp.mode;
                ana_.estop = cp.estop != 0;
                ana_.last_control = now;
            }
            break;
        }
        case MsgType::Telemetry: {
            TelemetryPacket tp;
            if (decodeTelemetry(payload, payload_len, tp)) {
                ana_.telemetry_frames++;
                for (int i = 0; i < kQuatCount; ++i) {
                    const f32* q = tp.quaternions[i];
                    const f32 n = std::sqrt(q[0] * q[0] + q[1] * q[1] +
                                            q[2] * q[2] + q[3] * q[3]);
                    if (n > 1e-3f && (n < 0.9f || n > 1.1f)) ana_.bad_quaternions++;
                }
                for (int i = 0; i < kLaserCount; ++i) {
                    const f32 v = tp.laser[i];
                    if (!std::isfinite(v) || v < 0.0f || v > 200.0f) {
                        ana_.bad_laser++;
                        continue;
                    }
                    if (!ana_.have_laser) {
                        ana_.min_laser = ana_.max_laser = v;
                        ana_.have_laser = true;
                    } else {
                        if (v < ana_.min_laser) ana_.min_laser = v;
                        if (v > ana_.max_laser) ana_.max_laser = v;
                    }
                }
            }
            break;
        }
        case MsgType::Status: {
            StatusTelem st;
            if (decodeStatus(payload, payload_len, st)) {
                ana_.status_frames++;
                ana_.have_mode = true;
                ana_.mode = st.mode;
                ana_.estop = st.estop != 0;
            }
            break;
        }
        default:
            break;
    }
    return true;
}

void Monitor::tick(std::chrono::steady_clock::time_point now) {
    for (std::size_t i = 0; i < dirs_.size(); ++i) {
        if (ever_seen_[i] && up_[i] && now - dirs_[i].last_rx > link_timeout_)
            up_[i] = false;
    }
}

double Monitor::lossRate(LinkDirection d, StreamId s) const {
    if (std::size_t(s) >= StreamCount) return 0.0;
    const StreamStats& ss = dirs_[std::size_t(d)].streams[std::size_t(s)];
    const u64 total = ss.received + ss.lost;
    return total ? double(ss.lost) / double(total) : 0.0;
}

double Monitor::lossRate(LinkDirection d) const {
    u64 received = 0, lost = 0;
    for (const StreamStats& ss : dirs_[std::size_t(d)].streams) {
        received += ss.received;
        lost += ss.lost;
    }
    const u64 total = received + lost;
    return total ? double(lost) / double(total) : 0.0;
}

bool Monitor::openLog(const std::string& path) {
    closeLog();
    log_ = std::fopen(path.c_str(), "w");
    if (!log_) return false;
    log_t0_ = std::chrono::steady_clock::now();

    std::time_t wall = std::time(nullptr);
    char stamp[32] = "?";
    std::tm tmv{};
    if (localtime_r(&wall, &tmv))
        std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", &tmv);
    std::fprintf(log_, "# RCLP monitor log v1 start=%s\n", stamp);
    std::fprintf(log_, "t_ms,dir,type,stream,seq,flags,bytes\n");
    return true;
}

void Monitor::closeLog() {
    if (log_) {
        std::fclose(log_);
        log_ = nullptr;
    }
}

void Monitor::logFrame(LinkDirection d, const FrameHeader& h, std::size_t frame_len,
                       std::chrono::steady_clock::time_point now) {
    if (!log_) return;
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - log_t0_).count();
    std::fprintf(log_, "%lld,%s,0x%02X,%u,%u,%u,%zu\n",
                 static_cast<long long>(ms), directionName(d), unsigned(h.msg_type),
                 unsigned(h.stream_id), unsigned(h.seq), unsigned(h.flags), frame_len);
}

} // namespace robotlink
