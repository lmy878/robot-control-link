#include "link/control.hpp"

namespace robotlink {

std::size_t encodeControl(const ControlPacket& p, u8* out, std::size_t cap) {
    if (cap < kControlSize) return 0;
    std::memcpy(out, &p, kControlSize);
    return kControlSize;
}

bool decodeControl(const u8* in, std::size_t len, ControlPacket& p) {
    if (len != kControlSize) return false;
    std::memcpy(&p, in, kControlSize);
    return p.magic == kControlMagic && p.size == kControlSize;
}

} // namespace robotlink
