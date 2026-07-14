#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>

#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>

#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "protocol/udp.hpp"

#define MAXIMUM_RETRY  5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define PORT 27015

EventGroupHandle_t s_wifi_event_group;
static unsigned int s_retry_num = 0;
static const char* TAG = "wifi_station";

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if(s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config{};

    strcpy(reinterpret_cast<char*>(wifi_config.sta.ssid), "master oogway");
    strcpy(reinterpret_cast<char*>(wifi_config.sta.password), "hilkudilku02");

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if(bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap");
    } else if(bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect");
    } else {
        ESP_LOGE(TAG, "Unexpected event");
    }
}

void parsePacket(char* buf) {
    PacketHeader header;
    memcpy(&header, buf, sizeof(header));

    switch(header.type) {
        using enum PacketType;
        using enum PacketID;
        case MESSAGE:
            switch(header.id) {
                case STR:
                    STR_MSG msg;
                    memcpy(&msg, buf + sizeof(header), header.length);
                    ESP_LOGI(TAG, "Received message %s", msg.str);
                    break;
                default:
                    ESP_LOGI(TAG, "Received message but do not know how to read it");
            }
            break;
        default:
            ESP_LOGI(TAG, "Received something else (type: %02hhX)", buf[3]);
    };
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    int ip_protocol = 0;
    char rx_buffer[BUFFER_LEN];
    char addr_str[BUFFER_LEN];

    while(true) {
        struct sockaddr_in* dest_addr_ip4 = &dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;

        int sock = socket(AF_INET, SOCK_DGRAM, ip_protocol);
        if(sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        int err = bind(sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if(err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            close(sock);
            continue;
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);

        while(true) {
            ESP_LOGI(TAG, "Waiting for data");

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr*)&source_addr, &socklen);

            if(len < 0) {
                if(errno == 11) {
                    continue;
                }

                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            } else {
                if(source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                }
                rx_buffer[len] = 0;
                ESP_LOGI(TAG, "Received %d bytes from %s", len, addr_str);
                for(int i = 0; i < len; i++) {
                    printf("%x", rx_buffer[i]);
                }
                parsePacket(rx_buffer);

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr*)&source_addr, sizeof(source_addr));
                if(err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }
        if(sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
}