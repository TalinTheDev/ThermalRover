#include "app.hpp"
#include <condition_variable>
#include <functional>
#include "imgui.h"
#include "misc/cpp/imgui_stdlib.h"
#include "layers/debug.hpp"
#include <mutex>
#include "networking/udpserver.hpp"
#include <queue>
#include <string>
#include <thread>

UDPServer server(3000);
std::string espIP = "192.168.1.23";
unsigned short espPort = 27015;

std::queue<UDPPacket> toSend;
std::mutex sendMutex;
std::condition_variable sendCV;

void sendButton() {
    ImGui::Begin("Network");
    ImGui::InputText("IP", &espIP);
    ImGui::InputScalar("Port", ImGuiDataType_U16, &espPort);
    if(ImGui::Button("Send", {100, 50})) {
        {
            std::scoped_lock lock(sendMutex);
            toSend.push({{'h', 'e', 'l', 'l', 'o'}, 5, {espIP, espPort}});
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
    std::thread sendThread(&UDPServer::loopSend, &server, std::ref(toSend), std::ref(sendMutex), std::ref(sendCV));
    std::thread appThread(runApp);

    recvThread.join();
    sendThread.join();
    appThread.join();
    return 0;
}
