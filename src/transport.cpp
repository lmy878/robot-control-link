#include "link/transport.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace robotlink {

UdpTransport::~UdpTransport() {
    if (fd_ >= 0) ::close(fd_);
}

bool UdpTransport::bindSocket(const std::string& bind_ip, u16 bind_port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;

    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(bind_port);
    if (::inet_pton(AF_INET, bind_ip.c_str(), &addr.sin_addr) != 1) return false;
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return false;

    int fl = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);
    return true;
}

bool UdpTransport::open(const std::string& bind_ip, u16 bind_port,
                        const std::string& peer_ip, u16 peer_port) {
    if (!bindSocket(bind_ip, bind_port)) return false;

    std::memset(&peer_, 0, sizeof(peer_));
    peer_.sin_family = AF_INET;
    peer_.sin_port = htons(peer_port);
    if (::inet_pton(AF_INET, peer_ip.c_str(), &peer_.sin_addr) != 1) return false;
    has_peer_ = true;
    return true;
}

bool UdpTransport::openMonitor(const std::string& bind_ip, u16 bind_port) {
    has_peer_ = false;
    return bindSocket(bind_ip, bind_port);
}

int UdpTransport::send(const u8* data, std::size_t len) {
    if (fd_ < 0 || !has_peer_) return -1;   // passive transport never transmits
    ssize_t n = ::sendto(fd_, data, len, 0,
                         reinterpret_cast<sockaddr*>(&peer_), sizeof(peer_));
    return static_cast<int>(n);
}

int UdpTransport::recv(u8* data, std::size_t cap, int timeout_ms) {
    if (fd_ < 0) return -1;
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr <= 0) return 0; // timeout / error
    ssize_t n = ::recv(fd_, data, cap, 0);
    return static_cast<int>(n);
}

} // namespace robotlink
