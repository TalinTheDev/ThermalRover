#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include <cstddef>
#include <machine/endian.h>
#include <string.h>

#include <sys/param.h>

#include "lwip/sockets.h"
#include "portmacro.h"
#include "protocol/udp.hpp"
#include <lwip/netdb.h>

#include "network.hpp"

void UDPServer::initWIFI() {
  ESP_LOGI(WIFI_TAG, "Connecting to WiFi in station mode");
  wifiEventGroup = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandler, NULL, &instance_got_ip));

  wifi_config_t wifi_config{};

  strcpy((char *)wifi_config.sta.ssid, "master oogway");
  strcpy(reinterpret_cast<char *>(wifi_config.sta.password), "hilkudilku02");
  // strcpy(reinterpret_cast<char*>(wifi_config.sta.password), "hilkudkilu02");

  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(WIFI_TAG, "WiFi initialization finished.");

  EventBits_t bits =
      xEventGroupWaitBits(wifiEventGroup, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                          pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(WIFI_TAG, "Connected to ap");
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(WIFI_TAG, "Failed to connect to AP");
  } else {
    ESP_LOGE(WIFI_TAG, "Unexpected event");
  }
}

int UDPServer::createSocket() {
  struct sockaddr_in dest_addr = {};
  int ip_protocol = IPPROTO_IP;

  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(PORT);

  int sock = socket(AF_INET, SOCK_DGRAM, ip_protocol);
  if (sock < 0) {
    ESP_LOGE(WIFI_TAG, "Unable to create socket...error %d", errno);
    return -1;
  }
  ESP_LOGI(WIFI_TAG, "Socket created");

  int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err < 0) {
    ESP_LOGE(WIFI_TAG, "Unable to bind socket...error %d)", errno);
    close(sock);
    return -1;
  }
  ESP_LOGI(WIFI_TAG, "Socket bound on port %d", PORT);

  return sock;
}

UDPPacket UDPServer::parsePacket(char *buf, uint8_t bytesReceived,
                                 sockaddr_in &clientAddr) {
  PacketHeader header;
  memcpy(&header, buf, sizeof(header));

  switch (header.type) {
    using enum PacketType;
    using enum PacketID;
  case MESSAGE:
    switch (header.id) {
    case STR:
      STR_MSG msg;
      memcpy(&msg, buf + sizeof(header), header.length);
      ESP_LOGI(WIFI_TAG, "Received message %s", msg.str);
      break;
    default:
      ESP_LOGI(WIFI_TAG, "Received message but do not know how to read it");
    }
    break;
  default:
    ESP_LOGI(WIFI_TAG, "Received something else (type: %02hhX)", buf[3]);
  };

  char ip[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &clientAddr.sin_addr, ip, sizeof(ip));
  PeerAddress client{.ip = std::string(ip), .port = ntohs(clientAddr.sin_port)};
  UDPPacket p{Packet((uint8_t *)buf, bytesReceived), client};
  return p;
}

void UDPServer::listen() {
  char recvBuffer[BUFFER_LEN];
  sockaddr_in clientAddr{};
  socklen_t clientAddrSize = sizeof(clientAddr);

  while (true) {
    int bytesReceived =
        recvfrom(sock, (char *)recvBuffer, BUFFER_LEN, 0,
                 (struct sockaddr *)&clientAddr, &clientAddrSize);

    if (bytesReceived < 0) {
      if (errno == 11) {
        continue;
      }

      ESP_LOGE(WIFI_TAG, "recvfrom failed...error %d", errno);
    } else {
      UDPPacket *p =
          new UDPPacket(parsePacket(recvBuffer, bytesReceived, clientAddr));
      ESP_LOGI(WIFI_TAG, "Received %d bytes from %s:%d", bytesReceived,
               p->peer.ip.c_str(), p->peer.port);

      xQueueSend(toSend, &p, portMAX_DELAY);
    }
  }
}

void UDPServer::send() {
  UDPPacket *packet;
  while (true) {
    {
      xQueueReceive(toSend, &packet, portMAX_DELAY);

      sockaddr_in sendAddr{};
      sendAddr.sin_family = AF_INET;
      sendAddr.sin_port = htons(packet->peer.port);

      inet_pton(AF_INET, packet->peer.ip.c_str(), &sendAddr.sin_addr);

      int bytesSent =
          sendto(sock, (const char *)packet->packet.buffer, packet->packet.size,
                 0, reinterpret_cast<sockaddr *>(&sendAddr), sizeof(sendAddr));

      if (bytesSent < 0) {
        ESP_LOGE(WIFI_TAG, "Error occurred during sending...error %d", errno);
      } else {
        ESP_LOGI(WIFI_TAG, "Sent %d bytes to %s:%d", bytesSent,
                 packet->peer.ip.c_str(), packet->peer.port);
      }
      delete packet;
    }
  }
}

UDPServer::UDPServer() {
  initWIFI();
  sock = createSocket();
  toSend = xQueueCreate(10, sizeof(UDPPacket *));
  if (toSend == NULL) {
    sock = -1;
  }
}

bool UDPServer::healthy() { return sock > 0; }