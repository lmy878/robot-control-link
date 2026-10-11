#pragma once
#include "link/common.hpp"

namespace robotlink {

struct FrameHeader {
    u8  version    = kVersion;
    u8  msg_type   = 0;
    u8  flags      = 0;
    u8  stream_id  = 0;
    u16 seq        = 0;
    u16 ack_seq    = 0;
    u16 payload_len = 0;
    u16 header_crc  = 0;
    u16 reserved    = 0;
};

// Serialize a 16-byte header (recomputes header_crc).
std::size_t encodeHeader(const FrameHeader& h, u8* out);

// Validate + parse a buffer. Returns false on bad magic / CRC / version / length.
bool decodeHeader(const u8* in, std::size_t len, FrameHeader& out);

// Build a full frame = header + payload (+ trailing payload CRC16 when
// FlagPayloadCrc is set). Returns the frame size, or 0 on error.
std::size_t buildFrame(const FrameHeader& h, const u8* payload, std::size_t plen,
                       u8* out, std::size_t cap);

} // namespace robotlink
