#pragma once
#pragma comment(lib, "ws2_32.lib")
#include <array>
#include <mutex>
#include <queue>
#include <string>
#define WIN32_LEAN_AND_MEAN
#include "protocol/udp.hpp"
#include <vector>
#include <winsock2.h>

#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)

class UDPServer {
public:
    UDPServer(unsigned short port);
    ~UDPServer();

    UDPPacket recv();
    bool send(UDPPacket packet);

    bool isHealthy();

    void loopRecv(std::vector<UDPPacket>&received);
    void loopSend(std::queue<UDPPacket>& toSend, std::mutex& sendMutex, std::condition_variable& sendCV);
    void kill();

private:
    bool healthy = false;
    struct addrinfo* addr = NULL, hints{};
    SOCKET sock = {};
};
