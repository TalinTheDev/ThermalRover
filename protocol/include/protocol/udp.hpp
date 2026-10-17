#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <cassert>
#include <type_traits>

#define PRINT_PACKET_BUF(p) for(int i = 0; i < (p).size; i++){printf("%x ", (p).buffer[i]);} printf("\n");

constexpr unsigned int BUFFER_LEN = 512;
constexpr unsigned short IP_LEN = 16;
static uint8_t packetsReceived = 1;
static uint8_t packetsSent = 1;

enum PacketType : uint8_t {
    MESSAGE = 0x00,
    COMMAND = 0x01,
    STATUS = 0x02,
    DATA = 0x03,
};

enum PacketID : uint8_t {
    NONE = 0x00,
    STR = 0x01,
    LED = 0x02,
    MOVE = 0x03,
    ESP = 0x04
};

#pragma pack(push, 1)
struct LED_CMD {
    uint8_t led;
    uint8_t on;
};

struct STR_MSG {
    char str[128];
};

struct ESP_STATUS {
    float batteryP = -1.0f;
    float batteryV = -1.0f;
    unsigned short port = 0;
    char ip[16] = {};
};

struct PacketHeader {
    uint16_t length;
    PacketType type;
    PacketID id;
    uint8_t seq;
};
#pragma pack(pop)  

class Packet {
public:
    template<typename T>
    Packet(PacketType type, PacketID id, const T& payload) {
        static_assert(std::is_trivially_copyable_v<T>,
                  "Packet payload must be trivially copyable");
        uint16_t payloadSize = sizeof(T);
        assert(sizeof(PacketHeader) + payloadSize <= BUFFER_LEN && "Packet too large");

        PacketHeader header{
        payloadSize,
        type,
        id,
        packetsSent++
        };
        memcpy(buffer, &header, sizeof(header));
        memcpy(buffer + sizeof(header), &payload, payloadSize);
        size = sizeof(header) + payloadSize;
    }

    Packet(uint8_t* buffer_, uint16_t size) : size(size) {
        memcpy(buffer, buffer_, size);
    }

    Packet() {}

    ~Packet() {}

    uint8_t buffer[BUFFER_LEN]{};
    uint16_t size = 0;
};

struct PeerAddress {
    char ip[16] = {};
    unsigned short port = 0;
};

struct UDPPacket {
    Packet packet;
    PeerAddress peer;
};