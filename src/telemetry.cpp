#include "link/telemetry.hpp"

namespace robotlink {

std::size_t encodeTelemetry(const TelemetryPacket& p, u8* out, std::size_t cap) {
    if (cap < kTelemetrySize) return 0;
    std::memcpy(out, &p, kTelemetrySize);
    return kTelemetrySize;
}

bool decodeTelemetry(const u8* in, std::size_t len, TelemetryPacket& p) {
    if (len != kTelemetrySize) return false;
    std::memcpy(&p, in, kTelemetrySize);
    return true;
}

} // namespace robotlink
