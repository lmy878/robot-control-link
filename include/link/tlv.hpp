#pragma once
#include "link/common.hpp"

namespace robotlink {

// Tiny hand-rolled TLV codec: [tag u8][len u8][value[len]], fully self-describing.
// Unknown tags can be skipped, so payloads stay forward compatible.

class TlvWriter {
public:
    TlvWriter(u8* buf, std::size_t cap) : buf_(buf), cap_(cap) {}

    bool u8v (u8 tag, u8 v)  { return raw(tag, &v, 1); }
    bool u16v(u8 tag, u16 v) { u8 t[2]; putU16(t, v); return raw(tag, t, 2); }
    bool f32v(u8 tag, f32 v) { u8 t[4]; putF32(t, v); return raw(tag, t, 4); }

    bool bytes(u8 tag, const u8* d, std::size_t n) { return raw(tag, d, n); }

    template <std::size_t N>
    bool f32Array(u8 tag, const f32 (&v)[N], std::size_t n) {
        if (n > N) return false;
        u8 tmp[4 * N];
        for (std::size_t i = 0; i < n; ++i) putF32(tmp + 4 * i, v[i]);
        return raw(tag, tmp, 4 * n);
    }

    std::size_t size() const { return off_; }
    bool ok() const { return ok_; }

private:
    bool raw(u8 tag, const u8* d, std::size_t n) {
        if (!ok_) return false;
        if (n > 255 || off_ + 2 + n > cap_) { ok_ = false; return false; }
        buf_[off_++] = tag;
        buf_[off_++] = u8(n);
        if (n && d) std::memcpy(buf_ + off_, d, n);
        off_ += n;
        return true;
    }
    u8* buf_;
    std::size_t cap_;
    std::size_t off_ = 0;
    bool ok_ = true;
};

class TlvReader {
public:
    TlvReader(const u8* b, std::size_t l) : buf_(b), len_(l) {}

    bool next(u8& tag, const u8*& val, std::size_t& vlen) {
        if (off_ + 2 > len_) return false;
        tag = buf_[off_];
        std::size_t n = buf_[off_ + 1];
        if (off_ + 2 + n > len_) return false;
        val = buf_ + off_ + 2;
        vlen = n;
        off_ += 2 + n;
        return true;
    }

    static bool asU8 (const u8* v, std::size_t n, u8& o)  { if (n < 1) return false; o = v[0]; return true; }
    static bool asU16(const u8* v, std::size_t n, u16& o) { if (n < 2) return false; o = getU16(v); return true; }
    static bool asF32(const u8* v, std::size_t n, f32& o) { if (n < 4) return false; o = getF32(v); return true; }

private:
    const u8* buf_;
    std::size_t len_;
    std::size_t off_ = 0;
};

} // namespace robotlink
