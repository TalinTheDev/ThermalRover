#pragma once
#pragma comment(lib, "ws2_32.lib")
#include <array>
#include <mutex>
#include <queue>
#include <string>
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>

#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)

constexpr unsigned int BUFFER_LEN = 512;

struct PeerAddress {
    std::string ip;
    unsigned short port;
};

struct UDPPacket {
    std::array<char, BUFFER_LEN> data;
    int size;
    PeerAddress peer;
};

class UDPServer {
public:
    UDPServer(unsigned short port);
    ~UDPServer();

    UDPPacket recv();
    bool send(UDPPacket packet);

    bool isHealthy();

    void loopRecv();
    void loopSend(std::queue<UDPPacket>& toSend, std::mutex& sendMutex, std::condition_variable& sendCV);

private:
    bool healthy = false;
    struct addrinfo* addr = NULL, hints{};
    SOCKET sock = {};
};
