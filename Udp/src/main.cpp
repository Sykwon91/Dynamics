#include "udp.h"

#include <iostream>
#include <string>

int main()
{
    UdpSocket receiver;
    UdpSocket sender;

    if (!receiver.bind(45000, "127.0.0.1"))
    {
        std::cerr << receiver.lastError() << std::endl;
        return 1;
    }

    if (!receiver.setReceiveTimeoutMs(1000))
    {
        std::cerr << receiver.lastError() << std::endl;
        return 1;
    }

    if (!sender.open())
    {
        std::cerr << sender.lastError() << std::endl;
        return 1;
    }

    const UdpSocket::Endpoint endpoint{"127.0.0.1", 45000};
    const std::string message = "udp loopback test";

    if (sender.sendTo(message, endpoint) < 0)
    {
        std::cerr << sender.lastError() << std::endl;
        return 1;
    }

    char buffer[1024] = {};
    UdpSocket::Endpoint peer;
    const int received = receiver.receiveFrom(buffer, sizeof(buffer) - 1, &peer);
    if (received <= 0)
    {
        std::cerr << "receive failed: " << receiver.lastError() << std::endl;
        return 1;
    }

    std::cout << "received " << received << " bytes from "
              << peer.Address << ":" << peer.Port << std::endl;
    std::cout << buffer << std::endl;
    return 0;
}
