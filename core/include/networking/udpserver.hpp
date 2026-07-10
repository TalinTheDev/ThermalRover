#pragma once
#pragma comment(lib, "ws2_32.lib")
#include <array>
#include <mutex>
#include <queue>
#include <string>
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <vector>

#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)

constexpr unsigned int BUFFER_LEN = 512;

enum PacketType : uint8_t {
    MESSAGE = 0x00,
    COMMAND = 0x01,
    STATUS = 0x02,
    DATA = 0x03,
};

enum PacketID : uint8_t {
    NONE = 0x00,
    LED = 0x01,
    MOVE = 0x02
};

struct LED_CMD {
    uint8_t led;
    uint8_t on;
};

struct STR_MSG {
    char str[128];
};

struct PacketHeader {
    uint16_t length;
    PacketType type;
    PacketID id;
};

class Packet {
public:
    template<typename T>
    Packet(PacketType type, PacketID id, T payload) {
        uint16_t payloadSize = sizeof(T);
        PacketHeader header{
            payloadSize,
            type,
            id
        };

        memcpy(buffer, &header, sizeof(header));
        memcpy(buffer + sizeof(header), &payload, payloadSize);
        size = sizeof(header) + payloadSize;
    }

    Packet(uint8_t* buffer_, uint16_t size): size(size) {
        memcpy(buffer, buffer_, size);
    }
    Packet()  {}

    ~Packet() {}

    uint8_t buffer[BUFFER_LEN]{};
    uint16_t size = 0;
};

struct PeerAddress {
    std::string ip;
    unsigned short port;
};

struct UDPPacket {
    Packet packet;
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
