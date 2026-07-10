#include "app.hpp"
#include "networking/udpserver.hpp"
#include <array>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <format>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <string.h>
#include <utility>
#include <vector>
#include <Windows.h>
#include <WinSock2.h>
#include <ws2def.h>
#include <ws2ipdef.h>
#include <ws2tcpip.h>


UDPServer::UDPServer(unsigned short port) {
    WSAData wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if(result != 0) {
        App::log.error(std::format("Network stack startup failed: {}", result));
        healthy = false;
        return;
    }
    healthy = true;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_PASSIVE;

    result = getaddrinfo(NULL, std::to_string(port).c_str(), &hints, &addr);
    if(result != 0) {
        App::log.error(std::format("Failed to get address (self): {}", result));
        healthy = false;
        return;
    }

    sock = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if(sock == INVALID_SOCKET) {
        App::log.error(
            std::format("Failed to create socket: {}", WSAGetLastError()));
        healthy = false;
        return;
    }
    bool enable = true;
    DWORD bytesReturned = 0;
    WSAIoctl(sock, SIO_UDP_CONNRESET, &enable, sizeof(enable), nullptr, 0,
             &bytesReturned, nullptr, nullptr);

    result = bind(sock, addr->ai_addr, (int)addr->ai_addrlen);
    if(result == SOCKET_ERROR) {
        App::log.error(std::format("Failed to bind socket: {}", WSAGetLastError()));
        healthy = false;
        return;
    } else {
        App::log.info(std::format("UDP Server listening on port {}", port));
    }
}

UDPServer::~UDPServer() {
    closesocket(sock);
    freeaddrinfo(addr);
    WSACleanup();
}

bool UDPServer::isHealthy() {
    return healthy;
}

UDPPacket UDPServer::recv() {
    uint8_t buffer[BUFFER_LEN];
    sockaddr_in clientAddr{};
    int clientAddrSize = sizeof(clientAddr);
    uint16_t bytesReceived = recvfrom(sock, (char*)buffer, BUFFER_LEN, 0,
                                 (struct sockaddr*)&clientAddr, &clientAddrSize);

    if(bytesReceived == SOCKET_ERROR) {
        App::log.error(
            std::format("Receiving request failed: {}", WSAGetLastError()));
        return UDPPacket{Packet(), {}};
    } else {
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &clientAddr.sin_addr, ip, sizeof(ip));
        PeerAddress client{.ip = std::string(ip),
                           .port = ntohs(clientAddr.sin_port)};

        App::log.info(std::format("Received data from {}:{}", client.ip, client.port));
        return UDPPacket{Packet((uint8_t*)buffer, bytesReceived), client
    };
    }
}

bool UDPServer::send(UDPPacket packet) {
    sockaddr_in sendAddr{};
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(packet.peer.port);

    inet_pton(AF_INET, packet.peer.ip.c_str(), &sendAddr.sin_addr);
    int bytesSent = sendto(sock, (const char*)packet.packet.buffer, packet.packet.size, 0, reinterpret_cast<sockaddr*>(&sendAddr), sizeof(sendAddr));
    if(bytesSent == SOCKET_ERROR) {
        App::log.error(
        std::format("Sending data failed: {}", WSAGetLastError()));
        return false;
    } else {
        App::log.info(std::format("Sent data (length {}) to {}:{}", packet.packet.size, packet.peer.ip, packet.peer.port));
        return true;
    }
}

void UDPServer::loopRecv() {
    while(true) {
        UDPPacket p = recv();
    }
}

void UDPServer::loopSend(std::queue<UDPPacket>& toSend, std::mutex& sendMutex, std::condition_variable& sendCV) {
    while(true) {
        UDPPacket p;
        {
            std::unique_lock lock(sendMutex);
            sendCV.wait(lock, [&] {return !toSend.empty(); });
            p = std::move(toSend.front());
            toSend.pop();
        }
        send(p);
    }
}