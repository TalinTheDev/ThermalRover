#include "networking/socket.hpp"
#include <app.hpp>
#include <format>
#include <Windows.h>
#include <WinSock2.h>
#include <ws2def.h>
#include <ws2tcpip.h>
#include <mstcpip.h>

#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)

int initNetworkingStack() {
    WSAData wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if(result != 0) {
        App::log.error(std::format("WSAStartup failed: {}", result));
        return 1;
    }
    return 0;
}
void destroyNetworkingStack() {
    WSACleanup();
}

AddrInfo::AddrInfo(const char* port, int family, int protocol) {
    hints.ai_family = family;
    hints.ai_socktype = (protocol == IPPROTO_TCP) ? SOCK_STREAM : SOCK_DGRAM;
    hints.ai_protocol = protocol;
    hints.ai_flags = AI_PASSIVE;


    int result = getaddrinfo(NULL, port, &hints, &addrResult);
    if(result != 0) {
        App::log.error(std::format("getaddrinfo failed: {}", result));
        WSACleanup();
    } else {
        success = true;
    }
}

AddrInfo::~AddrInfo() {
    freeaddrinfo(addrResult);
}

bool AddrInfo::valid() {
    return success;
}

addrinfo* AddrInfo::get() {
    return addrResult;
}

Socket::Socket(addrinfo* info, const char* port_) {
    sock = socket(info->ai_family, info->ai_socktype, info->ai_protocol);

    if(sock == INVALID_SOCKET) {
        App::log.error(std::format("Error creating socket: %ld", WSAGetLastError()));
        WSACleanup();
    }

    if(info->ai_protocol == IPPROTO_UDP) {
        bool enable = false;
        DWORD bytesReturned = 0;

        WSAIoctl(sock, SIO_UDP_CONNRESET, &enable, sizeof(enable), nullptr, 0, &bytesReturned, nullptr, nullptr);
    }

    port = port_;
    addrInfo = info;
}

Socket::~Socket() {
    closesocket(sock);
}

bool Socket::valid() {
    return sock != INVALID_SOCKET;
}

void Socket::bindToSystem() {
    int result = bind(sock, addrInfo->ai_addr, (int)addrInfo->ai_addrlen);
    if(result == SOCKET_ERROR) {
        App::log.error(std::format("Socket binding failed with error {}", WSAGetLastError()));
        sock = INVALID_SOCKET;
    } else {
        App::log.info(std::format("Listening on port {}", port));
    }
}

int Socket::receive(char* buffer, unsigned int size) {
    int bytesReceived = recvfrom(sock, buffer, size, 0, (struct sockaddr*)&clientAddress, &clientAddressLen);

    if(bytesReceived == SOCKET_ERROR) {
        int error = WSAGetLastError();
        switch(error) {
            case 10054:
                App::log.info("Receiving request failed due to peer closing the connection");
                break;
            default:
                App::log.error(std::format("Receiving request failed with error {}", error));
        }

        return 0;
    } else {
        char ip[INET_ADDRSTRLEN];

        inet_ntop(AF_INET, &clientAddress.sin_addr, ip, sizeof(ip));
        App::log.info(std::format("Received data from {}:{}", ip, ntohs(clientAddress.sin_port)));
        return  bytesReceived;
    }
}

void Socket::send(char* buffer, unsigned int size) {
    int bytesSent = sendto(sock, (const char*)buffer, size, 0, (struct sockaddr*)&clientAddress, clientAddressLen);

    if(bytesSent == SOCKET_ERROR) {
        App::log.error(std::format("Sending data failed with error {}", WSAGetLastError()));
    } else {
        char ip[INET_ADDRSTRLEN];

        inet_ntop(AF_INET, &clientAddress.sin_addr, ip, sizeof(ip));
        App::log.info(std::format("Sent data to {}:{}", ip, ntohs(clientAddress.sin_port)));
    }
}