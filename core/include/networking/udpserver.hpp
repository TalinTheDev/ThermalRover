#pragma once
#pragma comment(lib, "ws2_32.lib")
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <queue>
#include <string>
#define WIN32_LEAN_AND_MEAN
#include "protocol/udp.hpp"
#include <winsock2.h>

#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)

class UDPServer {
public:
    UDPServer(unsigned short port);
    ~UDPServer();

    UDPPacket recv();
    bool send(UDPPacket packet);

    bool isHealthy() const;
    unsigned short getLocalPort() const { return localPort; }
    std::uint64_t getDroppedReceivePacketCount() const;
    std::uint64_t getRejectedDatagramCount() const;
    std::uint64_t getProtocolVersionMismatchCount() const;

    void loopRecv(std::deque<UDPPacket>& received, std::mutex& receivedMutex);
    void loopSend(std::queue<UDPPacket>& toSend, std::mutex& sendMutex, std::condition_variable& sendCV);
    void kill();

private:
    std::atomic_bool healthy = false;
    std::atomic<std::uint64_t> droppedReceivePacketCount = 0;
    std::atomic<std::uint64_t> rejectedDatagramCount = 0;
    std::atomic<std::uint64_t> protocolVersionMismatchCount = 0;
    unsigned short localPort = 0;
    bool winsockStarted = false;
    struct addrinfo* addr = nullptr;
    struct addrinfo hints{};
    SOCKET sock = INVALID_SOCKET;
};
