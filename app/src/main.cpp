#include "app.hpp"
#include "imgui.h"
#include "layers/debug.hpp"
#include "misc/cpp/imgui_stdlib.h"
#include "networking/udpserver.hpp"
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <string.h>
#include <string.h>
#include <thread>
#include "protocol/udp.hpp"
#include <vector>

UDPServer server(3000);
std::string espIP = "192.168.1.23";
unsigned short espPort = 27015;
uint8_t buffer[BUFFER_LEN];

std::queue<UDPPacket> toSend;
std::mutex sendMutex;
std::condition_variable sendCV;

void sendButton() {
    ImGui::Begin("Network");
    ImGui::InputText("IP", &espIP);
    ImGui::InputScalar("Port", ImGuiDataType_U16, &espPort);
    if(ImGui::Button("Say Hello", {100, 50})) {
        {
            std::scoped_lock lock(sendMutex);
            Packet packet(MESSAGE, NONE, STR_MSG{"Hello!"});
            toSend.push({packet, {espIP, espPort}});
        }
        sendCV.notify_one();
    }
    ImGui::End();
}

void runApp() {
    App app(1500, 1000, "Networking stack");
    app.newLayer<DebugLayer>(app.getWindow()).push(sendButton);
    app.run();
}

int main() {
    if(!server.isHealthy()) {
        return 1;
    }
    std::thread recvThread(&UDPServer::loopRecv, &server);
    std::thread sendThread(&UDPServer::loopSend, &server, std::ref(toSend),
                           std::ref(sendMutex), std::ref(sendCV));
    std::thread appThread(runApp);

    {
        std::scoped_lock lock(sendMutex);
        Packet packet(MESSAGE, NONE, STR_MSG{"Hello!"});
        toSend.push({packet, {espIP, espPort}});
    }
    sendCV.notify_one();

    recvThread.join();
    sendThread.join();
    appThread.join();
    return 0;
}
