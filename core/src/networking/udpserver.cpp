#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <Windows.h>
#include <ws2def.h>
#include <ws2ipdef.h>
#include <ws2tcpip.h>

#include "app.hpp"
#include "networking/udpserver.hpp"
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <format>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <string.h>
#include "protocol/udp.hpp"
#include <utility>
UDPServer::UDPServer(unsigned short port) {
    WSAData wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if(result != 0) {
        App::log.error(std::format("Network stack startup failed: {}", result));
        healthy = false;
        return;
    }
    winsockStarted = true;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_PASSIVE;

    result = getaddrinfo(NULL, std::to_string(port).c_str(), &hints, &addr);
    if(result != 0) {
        App::log.error(std::format("Failed to get address (self): {}", result));
        return;
    }

    sock = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if(sock == INVALID_SOCKET) {
        App::log.error(
            std::format("Failed to create socket: {}", WSAGetLastError()));
        return;
    }
    DWORD receiveTimeoutMs = 250;
    if(setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                  reinterpret_cast<const char*>(&receiveTimeoutMs),
                  sizeof(receiveTimeoutMs)) == SOCKET_ERROR) {
        App::log.error(std::format("Could not set UDP receive timeout: {}",
                                   WSAGetLastError()));
        return;
    }
    BOOL disableUdpConnectionReset = FALSE;
    DWORD bytesReturned = 0;
    if(WSAIoctl(sock, SIO_UDP_CONNRESET, &disableUdpConnectionReset,
                sizeof(disableUdpConnectionReset), nullptr, 0,
                &bytesReturned, nullptr, nullptr) == SOCKET_ERROR) {
        App::log.error(std::format("Could not disable UDP connection-reset reports: {}",
                                   WSAGetLastError()));
    }

    BOOL broadcastEnabled = TRUE;
    if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
                  reinterpret_cast<const char*>(&broadcastEnabled),
                  sizeof(broadcastEnabled)) == SOCKET_ERROR) {
        App::log.error(std::format("Could not enable UDP discovery broadcast: {}",
                                   WSAGetLastError()));
    }

    result = bind(sock, addr->ai_addr, (int)addr->ai_addrlen);
    if(result == SOCKET_ERROR) {
        App::log.error(std::format("Failed to bind socket: {}", WSAGetLastError()));
        return;
    }
    sockaddr_in localAddress{};
    int localAddressBytes = sizeof(localAddress);
    if(getsockname(sock, reinterpret_cast<sockaddr*>(&localAddress),
                   &localAddressBytes) == SOCKET_ERROR) {
        App::log.error(std::format("Could not read bound UDP address: {}", WSAGetLastError()));
        return;
    }
    localPort = ntohs(localAddress.sin_port);
    App::log.info(std::format("UDP Server listening on port {}", localPort));
    healthy = true;
}

UDPServer::~UDPServer() {
    if(sock != INVALID_SOCKET) closesocket(sock);
    if(addr != nullptr) freeaddrinfo(addr);
    if(winsockStarted) WSACleanup();
}

bool UDPServer::isHealthy() const {
    return healthy;
}

std::uint64_t UDPServer::getDroppedReceivePacketCount() const {
    return droppedReceivePacketCount.load(std::memory_order_relaxed);
}

std::uint64_t UDPServer::getRejectedDatagramCount() const {
    return rejectedDatagramCount.load(std::memory_order_relaxed);
}

std::uint64_t UDPServer::getProtocolVersionMismatchCount() const {
    return protocolVersionMismatchCount.load(std::memory_order_relaxed);
}

UDPPacket UDPServer::recv() {
    uint8_t buffer[BUFFER_LEN + 1];
    sockaddr_in clientAddr{};
    int clientAddrSize = sizeof(clientAddr);
    int bytesReceived = recvfrom(sock, (char*)buffer, sizeof(buffer), 0,
                                 (struct sockaddr*)&clientAddr, &clientAddrSize);

    if(bytesReceived == SOCKET_ERROR) {
        const int error = WSAGetLastError();
        if(error == WSAEMSGSIZE) {
            rejectedDatagramCount.fetch_add(1, std::memory_order_relaxed);
        }
        if(healthy && error != WSAETIMEDOUT && error != WSAEWOULDBLOCK &&
           error != WSAEMSGSIZE) {
            App::log.error(std::format("Receiving request failed: {}", error));
        }
        return UDPPacket{Packet(), {}};
    } else {
        if(bytesReceived < static_cast<int>(sizeof(PacketHeader)) ||
           bytesReceived > static_cast<int>(BUFFER_LEN)) {
            rejectedDatagramCount.fetch_add(1, std::memory_order_relaxed);
            return UDPPacket{Packet(), {}};
        }
        PacketHeader header;
        memcpy(&header, buffer, sizeof(header));
        const auto payloadBytes = static_cast<size_t>(bytesReceived) - sizeof(PacketHeader);
        if(header.version != PROTOCOL_VERSION) {
            rejectedDatagramCount.fetch_add(1, std::memory_order_relaxed);
            protocolVersionMismatchCount.fetch_add(1, std::memory_order_relaxed);
            return UDPPacket{Packet(), {}};
        }
        if(header.length != payloadBytes ||
           sizeof(PacketHeader) + header.length > BUFFER_LEN) {
            rejectedDatagramCount.fetch_add(1, std::memory_order_relaxed);
            return UDPPacket{Packet(), {}};
        }

        char ip[INET_ADDRSTRLEN]{};
        if(inet_ntop(AF_INET, &clientAddr.sin_addr, ip, sizeof(ip)) == nullptr) {
            return UDPPacket{Packet(), {}};
        }
        PeerAddress client{.ip = "",
                           .port = ntohs(clientAddr.sin_port)};
        memcpy(client.ip, ip, IP_LEN);
        // Sensor streams can deliver hundreds of small DATA packets per second;
        // logging every datagram would add avoidable disk/console overhead.
        if(header.type != DATA) {
            App::log.info(std::format("Received data from {}:{}", client.ip, client.port));
        }
        UDPPacket p{Packet(buffer, static_cast<uint16_t>(bytesReceived)), client
        };
        return p;
    }
}

bool UDPServer::send(UDPPacket packet) {
    sockaddr_in sendAddr{};
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(packet.peer.port);

    if(inet_pton(AF_INET, packet.peer.ip, &sendAddr.sin_addr) != 1) {
        App::log.error("Invalid destination IP address");
        return false;
    }
    PacketHeader header{};
    bool movementCommand = false;
    if(packet.packet.size >= sizeof(PacketHeader)) {
        memcpy(&header, packet.packet.buffer, sizeof(header));
        movementCommand = header.type == COMMAND && header.id == MOVE;
    }
    int bytesSent = sendto(sock, (const char*)packet.packet.buffer, packet.packet.size, 0, reinterpret_cast<sockaddr*>(&sendAddr), sizeof(sendAddr));
    if(bytesSent == SOCKET_ERROR) {
        App::log.error(
        std::format("Sending data failed: {}", WSAGetLastError()));
        return false;
    } else {
        if(!movementCommand && header.type != DATA) {
            App::log.info(std::format("Sent data (length {}) to {}:{}", packet.packet.size, packet.peer.ip, packet.peer.port));
        }
        return true;
    }
}

void UDPServer::loopRecv(std::deque<UDPPacket>& received, std::mutex& receivedMutex) {
    constexpr std::size_t maximumQueuedPackets = 2048;
    while(this->healthy) {
        UDPPacket packet = recv();
        if(packet.packet.size != 0) {
            std::scoped_lock lock(receivedMutex);
            if(received.size() >= maximumQueuedPackets) {
                received.pop_front();
                droppedReceivePacketCount.fetch_add(1, std::memory_order_relaxed);
            }
            received.push_back(std::move(packet));
        }
    }
}

void UDPServer::loopSend(std::queue<UDPPacket>& toSend, std::mutex& sendMutex, std::condition_variable& sendCV) {
    while(this->healthy) {
        UDPPacket p;
        {
            std::unique_lock lock(sendMutex);
            sendCV.wait(lock, [&] {return !healthy || !toSend.empty(); });
            if(!healthy && toSend.empty()) return;
            p = std::move(toSend.front());
            toSend.pop();
        }
        send(p);
    }
}

void UDPServer::kill() {
    healthy = false;
}
