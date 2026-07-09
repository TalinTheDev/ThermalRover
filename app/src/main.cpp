#pragma comment(lib, "ws2_32.lib")
#define WIN32_LEAN_AND_MEAN
#include <cstdio>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#define DEFAULT_PORT "27015"
#define DEFAULT_BUF_LEN 512

int main() {
    printf("Hello World: Networking Version\n");
    WSADATA wsaData;

    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if(iResult != 0) {
        printf("WSAStartup failed: %d\n", iResult);
        return 1;
    }

    struct addrinfo* result = NULL, * ptr = NULL, hints;

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_PASSIVE;

    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if(iResult != 0) {
        printf("getaddrinfo failed: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    SOCKET ListenSocket = INVALID_SOCKET;
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if(ListenSocket == INVALID_SOCKET) {
        printf("Error at socket(): %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if(iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }
    freeaddrinfo(result);
    while(true) {
        char buffer[DEFAULT_BUF_LEN] = "";
        sockaddr_in clientAddress{};
        int clientAddressLen = (int)sizeof(clientAddress);
        int bytesReceived = recvfrom(ListenSocket, buffer, DEFAULT_BUF_LEN - 1, 0, (struct sockaddr*)&clientAddress, &clientAddressLen);
        if(bytesReceived == SOCKET_ERROR) {
            printf("Couldn't receive data\n");
            continue;
        } else {
            buffer[bytesReceived] = '\0';
            printf("Received %d bytes: %s\n", bytesReceived, buffer);
        }

        int byteSent = sendto(ListenSocket, (const char*)buffer, bytesReceived, 0, (struct sockaddr*)&clientAddress, clientAddressLen);
        if(byteSent == SOCKET_ERROR) {
            printf("Error sending data\n");
            continue;
        } else {
            printf("Data sent\n");
        }
    }



    closesocket(ListenSocket);
    WSACleanup();

    return 0;
}