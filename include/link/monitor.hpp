#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

#include "link/common.hpp"
#include "link/control.hpp"
#include "link/frame.hpp"
#include "link/telemetry.hpp"

namespace robotlink {

// Direction of an observed frame. The message type range decides it: downlink
// types (0x00..0x7F) travel Controller -> Robot, uplink types (0x80..0xFF) travel
// Robot -> Controller.
enum class LinkDirection : u8 {
    ControllerToRobot = 0,
    RobotToController = 1,
    Count             = 2,
};

inline const char* directionName(LinkDirection d) {
    return d == LinkDirection::ControllerToRobot ? "C->R" : "R->C";
}

inline LinkDirection directionOf(MsgType t) {
    return isDownlink(t) ? LinkDirection::ControllerToRobot
                         : LinkDirection::RobotToController;
}

struct StreamStats {
    u64 received   = 0;   // unique, in-order (and gap-closing) frames
    u64 lost       = 0;   // inferred from forward sequence gaps
    u64 duplicates = 0;   // seq == last_seq
    u64 reordered  = 0;   // seq older than last_seq
    u64 bytes      = 0;
    u16 last_seq   = 0;
    bool have_last = false;
};

struct DirectionStats {
    u64 frames = 0;
    u64 bytes  = 0;
    bool seen  = false;
    std::array<StreamStats, StreamCount> streams{};
    std::chrono::steady_clock::time_point last_rx{};
};

// Live decode of the two fixed-size application packets, for on-the-fly robot
// state analysis (no control decisions are taken from it).
struct RobotAnalysis {
    // Controller -> Robot
    u64 control_frames = 0;
    bool have_mode = false;
    u8  mode = 0;
    bool estop = false;
    std::chrono::steady_clock::time_point last_control{};

    // Robot -> Controller
    u64 telemetry_frames = 0;
    u64 status_frames    = 0;
    u64 bad_quaternions  = 0;   // non-unit quaternion (|q| outside 0.9..1.1)
    u64 bad_laser        = 0;   // negative / implausible range
    f32 min_laser = 0.0f;
    f32 max_laser = 0.0f;
    bool have_laser = false;
};

// Passive link observer. It only ever *reads* what it hears: `Monitor` has no
// send path and no session state, so stopping, crashing or disconnecting it can
// never disturb the Controller <-> Robot link. It records both directions,
// infers per-stream packet loss from sequence gaps, tracks link up/down, writes
// a match log and decodes the application packets for live state analysis.
class Monitor {
public:
    explicit Monitor(std::chrono::milliseconds link_timeout = std::chrono::milliseconds(3000));

    void setLinkTimeout(std::chrono::milliseconds v) { link_timeout_ = v; }

    // Record one decoded frame. Returns false when the frame is a duplicate
    // (already counted, not dispatched again).
    bool observe(const FrameHeader& h, const u8* payload, std::size_t payload_len,
                 std::size_t frame_len, std::chrono::steady_clock::time_point now);

    // Refresh link-up/down state; call periodically.
    void tick(std::chrono::steady_clock::time_point now);

    const DirectionStats& direction(LinkDirection d) const {
        return dirs_[std::size_t(d)];
    }
    bool linkUp(LinkDirection d) const { return up_[std::size_t(d)]; }
    bool seen(LinkDirection d) const { return ever_seen_[std::size_t(d)]; }

    // Aggregate loss over all streams of a direction (or a single stream).
    double lossRate(LinkDirection d) const;
    double lossRate(LinkDirection d, StreamId s) const;

    const RobotAnalysis& analysis() const { return ana_; }

    // ---- match log (CSV) ----
    bool openLog(const std::string& path);
    void closeLog();
    bool logging() const { return log_ != nullptr; }
    void logFrame(LinkDirection d, const FrameHeader& h, std::size_t frame_len,
                  std::chrono::steady_clock::time_point now);

private:
    std::array<DirectionStats, 2> dirs_{};
    std::array<bool, 2> up_{{false, false}};
    std::array<bool, 2> ever_seen_{{false, false}};
    std::chrono::milliseconds link_timeout_{3000};
    RobotAnalysis ana_;

    std::FILE* log_ = nullptr;
    std::chrono::steady_clock::time_point log_t0_{};
};

} // namespace robotlink
