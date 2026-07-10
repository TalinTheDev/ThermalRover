#include "network.hpp"
#include <memory>
#include <mutex>
#include <networking/socket.hpp>
#include <string.h>
#include <thread>
#include <ws2def.h>

Server::Server() {
    network = std::make_unique<NetworkStack>();
    if(network->valid) {
        alive = true;
    }

    AddrInfo addrInfo(PORT, AF_INET, IPPROTO_UDP);
    if(!addrInfo.valid()) {
        alive = false;
        return;
    }

    sock = std::make_shared<Socket>(addrInfo.get(), PORT);
    if(!sock->valid()) {
        alive = false;
        return;
    }

    sock->bindToSystem();
    if(!sock->valid()) {
        alive = false;
        return;
    }

    receiver = std::thread(&Server::receiveData, this, sock);
    sender = std::thread(&Server::sendData, this, sock);
}

Server::~Server() {
    receiver.join();
    sender.join();
}

void Server::receiveData(std::shared_ptr<Socket> sock) {
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

void Server::sendData(std::shared_ptr<Socket> sock) {
    while(true) {
        std::unique_lock lock(dataMutex);
        newDataCV.wait(lock, [this] {return newData; });

        sock->send(data, dataSize);
        memset(data, 0, dataSize);
        dataSize = 0;
        newData = false;
    }
}