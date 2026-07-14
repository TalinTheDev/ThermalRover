#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <sys/param.h>

#include <lwip/netdb.h>

#include "mutex"
#include "network.hpp"
#include "protocol/udp.hpp"
#include "queue"

std::queue<UDPPacket> toSend{};
std::mutex toSendMutex;

void nvsFlashInit() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);
}

extern "C" void app_main(void) {
  nvsFlashInit();

  UDPServer server;
  if (!server.healthy()) {
    return;
  }

  xTaskCreatePinnedToCore(UDPServer::taskListen, "NETWORK_LISTEN_LOOP", 4096,
                          &server, 5, nullptr, 1);
  xTaskCreatePinnedToCore(UDPServer::taskSend, "NETWORK_SEND_LOOP", 4096,
                          &server, 5, nullptr, 1);
  while (true) {
    vTaskDelay(1);
  }
}