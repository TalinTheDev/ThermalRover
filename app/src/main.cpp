#include "app.hpp"
#include "imgui.h"
#include "layers/debug.hpp"
#include "misc/cpp/imgui_stdlib.h"
#include "networking/udpserver.hpp"
#include "protocol/udp.hpp"
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
#include <types.hpp>
#include <vector>
#include <format>

UDPServer server(3000);
std::string espIP = "192.168.1.23";
std::string msg = "Hello World!";
unsigned short espPort = 27015;
uint8_t buffer[BUFFER_LEN];

std::queue<UDPPacket> toSend;
std::vector<UDPPacket> messagesReceived;
std::mutex sendMutex;
std::condition_variable sendCV;

int width = 1500;
int height = 1000;

App app(width, height, "Networking stack");
static ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;

ESP_STATUS status{};

void network() {
    Vector2 renderSize = app.getRenderSize();
    width = renderSize.x;
    height = renderSize.y;
    ImGui::SetNextWindowPos({0,0});
    ImGui::SetNextWindowSize({width * 0.2f, height * 1.0f});
    ImGui::Begin("Left", 0, flags);
    ImGui::Text("ESP Status");
    ImGui::Text("Battery: %.2f%% @ %.3f", status.batteryP, status.batteryV);
    ImGui::Text("IP: %.*s", IP_LEN, status.ip);
    ImGui::Text("Port: %d", status.port);
    ImGui::Text("Networking");
    ImGui::InputText("Send IP", &espIP);
    ImGui::InputScalar("Send Port", ImGuiDataType_U16, &espPort);
    ImGui::InputText("Message", &msg);
    if(ImGui::Button("Send Message", {250, 25})) {
        {
            std::scoped_lock lock(sendMutex);
            STR_MSG str{};
            memcpy(&str.str, msg.c_str(), msg.length());
            Packet packet(MESSAGE, STR, str);
            UDPPacket p = {packet, {"", espPort}};
            memcpy(p.peer.ip, espIP.c_str(), sizeof(p.peer.ip));
            toSend.push(p);
        }
        sendCV.notify_one();
    }
    ImGui::Text("Received Messages");
    for(UDPPacket p : messagesReceived) {
        PacketHeader header;
        memcpy(&header, p.packet.buffer, sizeof(header));
        if(header.type == MESSAGE) {
            STR_MSG msg;
            memcpy(&msg, p.packet.buffer + sizeof(header), sizeof(STR_MSG));
            ImGui::BulletText(std::format("{}: {}", header.seq, msg.str).c_str());
        } else if(header.type == STATUS) {
            char ip[IP_LEN]{};
            memcpy(&status, p.packet.buffer + sizeof(header), sizeof(status));
        }
    }
    ImGui::End();
}

void runApp() {
    app.newLayer<DebugLayer>(app.getWindow()).push(network);
    app.run();
}

int main() {
    if(!server.isHealthy()) {
        return 1;
    }

    std::thread recvThread(&UDPServer::loopRecv, &server, std::ref(messagesReceived));
    std::thread sendThread(&UDPServer::loopSend, &server, std::ref(toSend),
                           std::ref(sendMutex), std::ref(sendCV));
    runApp();
    server.kill();

    if(recvThread.joinable()) {
        recvThread.join();
    }
    if(sendThread.joinable()) {
        sendThread.join();
    }
    return 0;
}
