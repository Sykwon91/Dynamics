#include "udp.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace
{
sockaddr_in toSockAddr(const UdpSocket::Endpoint& endpoint)
{
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(endpoint.Port);

    if (inet_pton(AF_INET, endpoint.Address.c_str(), &addr.sin_addr) != 1)
        addr.sin_addr.s_addr = INADDR_NONE;

    return addr;
}

UdpSocket::Endpoint fromSockAddr(const sockaddr_in& addr)
{
    char ip[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
    return UdpSocket::Endpoint{ip, ntohs(addr.sin_port)};
}
}

UdpSocket::UdpSocket() = default;

UdpSocket::~UdpSocket()
{
    close();
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept
    : SocketFd(other.SocketFd), LastError(std::move(other.LastError))
{
    other.SocketFd = -1;
}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept
{
    if (this == &other) return *this;

    close();
    SocketFd = other.SocketFd;
    LastError = std::move(other.LastError);
    other.SocketFd = -1;
    return *this;
}

bool UdpSocket::open()
{
    if (isOpen()) return true;

    SocketFd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (SocketFd < 0)
    {
        setError("socket");
        return false;
    }

    return true;
}

bool UdpSocket::bind(std::uint16_t port, const std::string& address)
{
    if (!open()) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (address == "0.0.0.0")
    {
        addr.sin_addr.s_addr = INADDR_ANY;
    }
    else if (inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1)
    {
        LastError = "bind: invalid IPv4 address: " + address;
        return false;
    }

    int reuse = 1;
    setsockopt(SocketFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    if (::bind(SocketFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        setError("bind");
        return false;
    }

    return true;
}

void UdpSocket::close()
{
    if (SocketFd >= 0)
    {
        ::close(SocketFd);
        SocketFd = -1;
    }
}

bool UdpSocket::isOpen() const
{
    return SocketFd >= 0;
}

bool UdpSocket::setNonBlocking(bool enabled)
{
    if (!isOpen() && !open()) return false;

    const int flags = fcntl(SocketFd, F_GETFL, 0);
    if (flags < 0)
    {
        setError("fcntl(F_GETFL)");
        return false;
    }

    const int nextFlags = enabled ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    if (fcntl(SocketFd, F_SETFL, nextFlags) < 0)
    {
        setError("fcntl(F_SETFL)");
        return false;
    }

    return true;
}

bool UdpSocket::setReceiveTimeoutMs(int timeoutMs)
{
    if (!isOpen() && !open()) return false;

    timeval tv{};
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;

    if (setsockopt(SocketFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        setError("setsockopt(SO_RCVTIMEO)");
        return false;
    }

    return true;
}

int UdpSocket::sendTo(const void* data, std::size_t size, const Endpoint& endpoint) const
{
    if (!isOpen())
    {
        LastError = "sendTo: socket is not open";
        return -1;
    }

    const sockaddr_in addr = toSockAddr(endpoint);
    if (addr.sin_addr.s_addr == INADDR_NONE)
    {
        LastError = "sendTo: invalid IPv4 address: " + endpoint.Address;
        return -1;
    }

    const ssize_t sent = ::sendto(SocketFd,
                                  data,
                                  size,
                                  0,
                                  reinterpret_cast<const sockaddr*>(&addr),
                                  sizeof(addr));
    if (sent < 0)
    {
        setError("sendto");
        return -1;
    }

    return static_cast<int>(sent);
}

int UdpSocket::sendTo(const std::string& message, const Endpoint& endpoint) const
{
    return sendTo(message.data(), message.size(), endpoint);
}

int UdpSocket::receiveFrom(void* buffer, std::size_t size, Endpoint* sender) const
{
    if (!isOpen())
    {
        LastError = "receiveFrom: socket is not open";
        return -1;
    }

    sockaddr_in addr{};
    socklen_t addrLen = sizeof(addr);
    const ssize_t received = ::recvfrom(SocketFd,
                                        buffer,
                                        size,
                                        0,
                                        reinterpret_cast<sockaddr*>(&addr),
                                        &addrLen);
    if (received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            LastError.clear();
            return 0;
        }

        setError("recvfrom");
        return -1;
    }

    if (sender)
        *sender = fromSockAddr(addr);

    return static_cast<int>(received);
}

std::vector<std::uint8_t> UdpSocket::receiveFrom(Endpoint* sender, std::size_t maxSize) const
{
    std::vector<std::uint8_t> buffer(maxSize);
    const int received = receiveFrom(buffer.data(), buffer.size(), sender);
    if (received <= 0)
    {
        buffer.clear();
        return buffer;
    }

    buffer.resize(static_cast<std::size_t>(received));
    return buffer;
}

int UdpSocket::descriptor() const
{
    return SocketFd;
}

std::string UdpSocket::lastError() const
{
    return LastError;
}

void UdpSocket::setError(const std::string& prefix) const
{
    LastError = prefix + ": " + std::strerror(errno);
}
