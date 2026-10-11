#pragma once
#include "link/common.hpp"

namespace robotlink {

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, no xorout.
u16 crc16(const u8* data, std::size_t len);

// CRC-32/ISO-HDLC (zlib): poly 0xEDB88320 reflected, init/xorout 0xFFFFFFFF.
u32 crc32(const u8* data, std::size_t len);

} // namespace robotlink
