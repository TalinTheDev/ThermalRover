#pragma once
#include "esp_event_base.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "protocol/udp.hpp"

#include <concepts>
#include <sys/param.h>

#include <lwip/netdb.h>

#define MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define PORT 27015

// Implemented by hardware.cpp. Returns false if the command is invalid or
// cannot be queued for the motor control task.
bool handleMoveCommand(const MOVE_CMD &command);
bool handlePidMotionCommand(const PID_MOTION_CMD &command);
bool handleMotionStopCommand();
bool handleMotionKeepalive(const MOTION_KEEPALIVE_CMD &command);
bool handleRecordingCommand(const RECORDING_CMD &command);
bool handleArmCommand(const ARM_CMD &command);
void disarmDrive();
bool driveArmLeaseActive();
std::uint8_t getRecordingState();

// The initializer and inline event handler must share one Wi-Fi state even
// when this header is included by multiple translation units.
inline EventGroupHandle_t wifiEventGroup = nullptr;
inline unsigned int connectRetries = 0;
inline bool wifiEverConnected = false;
inline constexpr const char *WIFI_TAG = "WIFI";

class UDPServer {
public:
  UDPServer();

  bool healthy();
  void loopListen();
  void loopSend();
  void send(const Packet &packet, const char* ip, unsigned short port);
  bool getLocalAddress(char *localIp, std::size_t ipCapacity,
                       unsigned short &localPort);
  bool getStatusDestination(char *ip, std::size_t ipCapacity,
                           unsigned short &port);
  bool connectPeer(std::uint32_t token, const char *ip, unsigned short port);
  bool isPairedEndpoint(const char *ip, unsigned short port);
  bool isAuthorizedPeer(const char *ip, unsigned short port,
                       std::uint32_t authToken);
  std::uint32_t getDroppedPacketCount();

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
      xEventGroupClearBits(wifiEventGroup, WIFI_CONNECTED_BIT);
      disarmDrive();
      if (wifiEverConnected) {
        // Keep the rover recoverable after a router or signal interruption.
        // Motor watchdogs remain local and do not depend on reconnect timing.
        const esp_err_t result = esp_wifi_connect();
        if (result != ESP_OK && result != ESP_ERR_WIFI_STATE) {
          ESP_LOGW(WIFI_TAG, "Wi-Fi reconnect request failed: %s",
                   esp_err_to_name(result));
        }
        ESP_LOGI(WIFI_TAG, "Reconnecting to the configured AP");
      } else if (connectRetries < MAXIMUM_RETRY) {
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
      wifiEverConnected = true;
      connectRetries = 0;
      xEventGroupClearBits(wifiEventGroup, WIFI_FAIL_BIT);
      xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
    }
  }

  unsigned short port = 0;

private:
  int sock = -1;
  QueueHandle_t toSend{};
  std::uint32_t droppedPacketCount = 0;
  std::uint32_t sessionToken = 0;
  portMUX_TYPE droppedPacketMux = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE statusDestinationMux = portMUX_INITIALIZER_UNLOCKED;
  char statusDestinationIp[IP_LEN]{};
  unsigned short statusDestinationPort = 0;

  int createSocket();
  std::uint32_t sessionTokenForPeer(const char *ip, unsigned short port);
  bool initWIFI();
};
