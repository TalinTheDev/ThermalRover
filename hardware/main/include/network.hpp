#pragma once
#include "esp_event_base.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "protocol/udp.hpp"

#include <concepts>
#include <sys/param.h>

#include <lwip/netdb.h>

#define MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define PORT 27015

static EventGroupHandle_t wifiEventGroup;
static unsigned int connectRetries = 0;
static const char *WIFI_TAG = "WIFI";

class UDPServer {
public:
  UDPServer();

  bool healthy();
  void loopListen();
  void loopSend();
  void send(Packet *packet, const char* ip, unsigned short port);

  static void taskListen(void *arg) {
    static_cast<UDPServer *>(arg)->loopListen();
  }
  static void taskSend(void *arg) { static_cast<UDPServer *>(arg)->loopSend(); }
  static void eventHandler(void *arg, esp_event_base_t eventBase,
                           int32_t eventID, void *eventData) {
    if (eventBase == WIFI_EVENT && eventID == WIFI_EVENT_STA_START) {
      esp_wifi_connect();
    } else if (eventBase == WIFI_EVENT &&
               eventID == WIFI_EVENT_STA_DISCONNECTED) {
      if (connectRetries < MAXIMUM_RETRY) {
        esp_wifi_connect();
        connectRetries++;
        ESP_LOGI(WIFI_TAG, "Retrying to connect to the AP");
      } else {
        xEventGroupSetBits(wifiEventGroup, WIFI_FAIL_BIT);
      }
      ESP_LOGI(WIFI_TAG, "Connecting to the AP fail");
    } else if (eventBase == IP_EVENT && eventID == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)eventData;
      ESP_LOGI(WIFI_TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
      connectRetries = 0;
      xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
    }
  }

  char ip[IP_LEN];
  unsigned short port;

private:
  int sock = -1;
  QueueHandle_t toSend{};

  int createSocket();
  void initWIFI();
  UDPPacket parsePacket(char *buf, uint8_t bytesReceived,
                        sockaddr_in &clientAddr);
};