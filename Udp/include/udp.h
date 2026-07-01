#ifndef UDP_H
#define UDP_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class UdpSocket
{
public:
    struct Endpoint
    {
        std::string Address = "127.0.0.1";
        std::uint16_t Port = 0;
    };

    UdpSocket();
    ~UdpSocket();

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;
    UdpSocket(UdpSocket&& other) noexcept;
    UdpSocket& operator=(UdpSocket&& other) noexcept;

    bool open();
    bool bind(std::uint16_t port, const std::string& address = "0.0.0.0");
    void close();
    bool isOpen() const;

    bool setNonBlocking(bool enabled);
    bool setReceiveTimeoutMs(int timeoutMs);

    int sendTo(const void* data, std::size_t size, const Endpoint& endpoint) const;
    int sendTo(const std::string& message, const Endpoint& endpoint) const;
    int receiveFrom(void* buffer, std::size_t size, Endpoint* sender = nullptr) const;
    std::vector<std::uint8_t> receiveFrom(Endpoint* sender = nullptr, std::size_t maxSize = 65535) const;

    int descriptor() const;
    std::string lastError() const;

private:
    int SocketFd = -1;
    mutable std::string LastError;

    void setError(const std::string& prefix) const;
};

#endif
