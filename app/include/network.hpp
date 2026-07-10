#pragma once
#include "networking/socket.hpp"
#include <condition_variable>
#include <memory>
#include <mutex>

constexpr int BUFFER_SIZE = 512;

class Server {
public:
    Server();
    ~Server();

    void receiveData(std::shared_ptr<Socket> sock);
    void sendData(std::shared_ptr<Socket> sock);

    bool alive = false;

private:
    std::unique_ptr<NetworkStack> network;
    std::shared_ptr<Socket> sock;
    std::thread receiver;
    std::thread sender;

    const char* PORT = "3000";
    std::mutex dataMutex;
    std::condition_variable newDataCV;
    char data[BUFFER_SIZE] = "";
    int dataSize = 0;
    bool newData = false;
};  