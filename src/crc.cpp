#include "link/crc.hpp"

namespace robotlink {

u16 crc16(const u8* data, std::size_t len) {
    u16 crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= u16(u16(data[i]) << 8);
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) crc = u16((crc << 1) ^ 0x1021);
            else              crc = u16(crc << 1);
        }
    }
    return crc;
}

u32 crc32(const u8* data, std::size_t len) {
    u32 crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 1u) crc = (crc >> 1) ^ 0xEDB88320u;
            else          crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

} // namespace robotlink
