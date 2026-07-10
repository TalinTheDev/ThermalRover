#pragma once
#pragma comment(lib, "ws2_32.lib")
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>

class NetworkStack {
public:
    NetworkStack();
    ~NetworkStack();
    bool valid = false;
};

class AddrInfo {
public:
    AddrInfo(const char* port, int family, int protocol);
    ~AddrInfo();

    addrinfo* get();
    bool valid();
private:
    struct addrinfo* addrResult = NULL, hints{};
    bool success = false;
};

class Socket {
public:
    Socket(addrinfo* info, const char* port);
    ~Socket();

    void bindToSystem();
    int receive(char* buffer, unsigned int size);
    void send(char* buffer, unsigned int size);

    bool valid();
private:
    const char* port = NULL;
    struct addrinfo* addrInfo{};
    sockaddr_in clientAddress{};
    int clientAddressLen = (int)sizeof(clientAddress);
    SOCKET sock{};
};