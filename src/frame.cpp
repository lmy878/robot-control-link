#include "link/frame.hpp"
#include "link/crc.hpp"

namespace robotlink {

std::size_t encodeHeader(const FrameHeader& h, u8* out) {
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = h.version;
    out[3] = h.msg_type;
    out[4] = h.flags;
    out[5] = h.stream_id;
    putU16(out + 6,  h.seq);
    putU16(out + 8,  h.payload_len);
    putU16(out + 10, h.reserved);
    putU16(out + 12, crc16(out, 12));
    return kHeaderSize;
}

bool decodeHeader(const u8* in, std::size_t len, FrameHeader& out) {
    if (len < kHeaderSize) return false;
    if (in[0] != kMagic0 || in[1] != kMagic1) return false;
    out.version     = in[2];
    out.msg_type    = in[3];
    out.flags       = in[4];
    out.stream_id   = in[5];
    out.seq         = getU16(in + 6);
    out.payload_len = getU16(in + 8);
    out.reserved    = getU16(in + 10);
    out.header_crc  = getU16(in + 12);
    if (crc16(in, 12) != out.header_crc) return false;
    if (out.version != kVersion) return false;
    if (out.payload_len > kMaxPayload + 2) return false;
    if (len < kHeaderSize + out.payload_len) return false;
    return true;
}

std::size_t buildFrame(const FrameHeader& h, const u8* payload, std::size_t plen,
                       u8* out, std::size_t cap) {
    if (plen > kMaxPayload) return 0;
    const bool with_crc = (h.flags & FlagPayloadCrc) != 0;
    std::size_t total = kHeaderSize + plen + (with_crc ? 2 : 0);
    if (total > cap) return 0;

    FrameHeader hh = h;
    u8* p = out + kHeaderSize;
    if (plen && payload) std::memcpy(p, payload, plen);
    hh.payload_len = u16(plen);
    if (with_crc) {
        putU16(p + plen, crc16(p, plen));
        hh.payload_len = u16(plen + 2);
    }
    encodeHeader(hh, out);
    return total;
}

} // namespace robotlink
