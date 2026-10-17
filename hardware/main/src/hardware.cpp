#include "driver/i2c.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/event_groups.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <sys/param.h>

#include <lwip/netdb.h>

#include "network.hpp"
#include "protocol/udp.hpp"

#define MAX17048_ADDR 0x36

#include "driver/gpio.h"
#include "driver/ledc.h"

#define STBY_PIN GPIO_NUM_5
#define AIN1_PIN GPIO_NUM_4
#define AIN2_PIN GPIO_NUM_3
#define PWMA_PIN GPIO_NUM_2
#define BIN1_PIN GPIO_NUM_6
#define BIN2_PIN GPIO_NUM_7
#define PWMB_PIN GPIO_NUM_8

i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                     .sda_io_num = GPIO_NUM_47,
                     .scl_io_num = GPIO_NUM_48,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE,
                     .master =
                         {
                             .clk_speed = 400000,
                         },
                     .clk_flags = {}};

void nvsFlashInit() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);
}

float max17048_get_percent() {
  uint8_t reg = 0x04;
  uint8_t data[2];

  ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_NUM_0, MAX17048_ADDR, &reg,
                                               1, data, 2, pdMS_TO_TICKS(100)));

  uint16_t raw = (data[0] << 8) | data[1];

  // Upper byte = integer %
  // Lower byte = fractional part (1/256 %)
  return (raw >> 8) + ((raw & 0xFF) / 256.0f);
}

float max17048_get_voltage() {
  uint8_t reg = 0x02;
  uint8_t data[2];

  ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_NUM_0, MAX17048_ADDR, &reg,
                                               1, data, 2, pdMS_TO_TICKS(100)));

  uint16_t raw = (data[0] << 8) | data[1];

  // Each LSB = 78.125 µV
  return raw * 78.125e-6f;
}

void motor_init(void) {
  // Configure GPIOs
  gpio_config_t io_conf{};
  io_conf.pin_bit_mask = (1ULL << STBY_PIN) | (1ULL << AIN1_PIN) |
                         (1ULL << AIN2_PIN) | (1ULL << BIN1_PIN) |
                         (1ULL << BIN2_PIN);
  io_conf.mode = GPIO_MODE_OUTPUT;

  gpio_config(&io_conf);

  // Configure PWM timer
  ledc_timer_config_t timer{};

  timer.speed_mode = LEDC_LOW_SPEED_MODE;

  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = 20000; // 20 kHz
  timer.clk_cfg = LEDC_AUTO_CLK;

  ledc_timer_config(&timer);

  // Configure PWM channel
  ledc_channel_config_t channel{};
  channel.gpio_num = PWMA_PIN;
  channel.speed_mode = LEDC_LOW_SPEED_MODE;
  channel.channel = LEDC_CHANNEL_0;
  channel.timer_sel = LEDC_TIMER_0;
  channel.duty = 0;
  channel.hpoint = 0;
  ledc_channel_config(&channel);

  channel.gpio_num = PWMB_PIN;
  channel.channel = LEDC_CHANNEL_1;
  ledc_channel_config(&channel);

  // Enable the driver
  gpio_set_level(STBY_PIN, 1);
}

void motor_forward(uint8_t speed) {
  gpio_set_level(AIN1_PIN, 1);
  gpio_set_level(AIN2_PIN, 0);

  gpio_set_level(BIN1_PIN, 1);
  gpio_set_level(BIN2_PIN, 0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motor_reverse(uint8_t speed) {
  gpio_set_level(AIN1_PIN, 0);
  gpio_set_level(AIN2_PIN, 1);

  gpio_set_level(BIN1_PIN, 0);
  gpio_set_level(BIN2_PIN, 1);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motor_stop(void) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void broadcastStatus(void* param) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  ESP_STATUS *status;
  Packet *p;
  UDPServer* server = (UDPServer*)param;
  while (true) {
    status = new ESP_STATUS{.batteryP = max17048_get_percent(),
                            .batteryV = max17048_get_voltage(),
                            .port = server->port,
                            .ip = ""};

    memcpy(status->ip, server->ip, IP_LEN);
    p = new Packet(STATUS, ESP, *status);

    server->send(p, "192.168.1.11", 3000);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5000));
  }
}

extern "C" void app_main(void) {
  nvsFlashInit();
  motor_init();

  UDPServer server;
  if (!server.healthy()) {
    return;
  }

  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

  xTaskCreatePinnedToCore(UDPServer::taskListen, "NETWORK_LISTEN_LOOP", 4096,
                          &server, 5, nullptr, 1);
  xTaskCreatePinnedToCore(UDPServer::taskSend, "NETWORK_SEND_LOOP", 4096,
                          &server, 5, nullptr, 1);
  xTaskCreatePinnedToCore(broadcastStatus, "BROADCAST_STATUS_LOOP", 4096,
                          &server, 5, nullptr, 0);

  while (true) {
    motor_forward(255); // Full speed forward
    vTaskDelay(pdMS_TO_TICKS(2000));

    motor_forward(128); // Half speed
    vTaskDelay(pdMS_TO_TICKS(2000));

    motor_reverse(255); // Full speed reverse
    vTaskDelay(pdMS_TO_TICKS(2000));

    motor_stop();
  }
}