#include "networking/socket.hpp"
#include <mutex>
#include <condition_variable>
#include <string.h>
#include <thread>
#include <ws2def.h>
#include <app.hpp>
#include <format>

const char* PORT = "3000";
#define BUFFER_SIZE 1028

std::mutex dataMutex;
std::condition_variable newDataCV;
char data[BUFFER_SIZE];
int dataSize = 0;
bool newData = false;


void receiveData(Socket* sock) {
    while(true) {
        char tmpBuf[BUFFER_SIZE];
        int size = sock->receive(tmpBuf, BUFFER_SIZE);
        if(size > 0) {
            {
                std::scoped_lock lock(dataMutex);
                memcpy(data, tmpBuf, size);
                dataSize = size;
                newData = true;
            }
            newDataCV.notify_one();
        }
    }
}

void sendData(Socket* sock) {
    while(true) {
        std::unique_lock lock(dataMutex);
        newDataCV.wait(lock, [] {return newData; });

        sock->send(data, dataSize);
        memset(data, 0, dataSize);
        dataSize = 0;
        newData = false;
    }
}


int main() {
    if(initNetworkingStack()) {
        return 1;
    }

    AddrInfo addrInfo(PORT, AF_INET, IPPROTO_UDP);
    if(!addrInfo.valid()) {
        return 1;
    }

    Socket sock(addrInfo.get(), PORT);
    if(!sock.valid()) {
        return 1;
    }

    sock.bindToSystem();
    if(!sock.valid()) {
        return 1;
    }

    std::thread receiver(receiveData, &sock);
    std::thread sender(sendData, &sock);

    receiver.join();
    sender.join();

    destroyNetworkingStack();
    return 0;
}
