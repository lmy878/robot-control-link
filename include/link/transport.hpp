#pragma once
#include <string>
#include <netinet/in.h>
#include "link/common.hpp"

namespace robotlink {

// Abstract byte-oriented datagram transport.
//
// WFB-ng integration: WFB-ng's userspace `wfb_tx` reads raw frames from a UDP
// socket and `wfb_rx` forwards received frames to a UDP socket. This adapter is
// therefore just a UDP endpoint pointed at those service ports; the radio link,
// FEC and link stats live entirely inside WFB-ng, below us.
class Transport {
public:
    virtual ~Transport() = default;
    virtual int send(const u8* data, std::size_t len) = 0;
    virtual int recv(u8* data, std::size_t cap, int timeout_ms) = 0;
};

class UdpTransport : public Transport {
public:
    UdpTransport() = default;
    ~UdpTransport() override;

    bool open(const std::string& bind_ip, u16 bind_port,
              const std::string& peer_ip, u16 peer_port);

    int send(const u8* data, std::size_t len) override;
    int recv(u8* data, std::size_t cap, int timeout_ms) override;

    int fd() const { return fd_; }

private:
    int fd_ = -1;
    sockaddr_in peer_{};
};

} // namespace robotlink
