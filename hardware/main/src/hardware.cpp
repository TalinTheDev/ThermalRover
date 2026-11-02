#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/event_groups.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include <sys/param.h>

#include <lwip/netdb.h>

#include "network.hpp"
#include "protocol/udp.hpp"
#include "protocol/robot_config.hpp"
#include "protocol/lidar_imu_odometry.hpp"
#include "protocol/motion_control.hpp"

#define MAX17048_ADDR 0x36

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "mlx90640_i2c.hpp"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <limits>

extern "C" {
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
}

#define STBY_PIN GPIO_NUM_5
#define AIN1_PIN GPIO_NUM_4
#define AIN2_PIN GPIO_NUM_3
#define PWMA_PIN GPIO_NUM_2
#define BIN1_PIN GPIO_NUM_6
#define BIN2_PIN GPIO_NUM_7
#define PWMB_PIN GPIO_NUM_8

namespace {
constexpr float pi = 3.14159265358979323846f;
constexpr float IMU_GYRO_LSB_PER_DEGREE_PER_SECOND = 65.5f; // +/-500 deg/s
constexpr float IMU_ACCEL_LSB_PER_G = 16384.0f; // +/-2 g
constexpr float IMU_CALIBRATION_MAX_GYRO_DEGREES_PER_SECOND = 5.0f;
constexpr float IMU_CALIBRATION_MAX_ACCEL_DEVIATION_G = 0.20f;
constexpr std::int64_t IMU_SAMPLE_STALE_TIMEOUT_MICROSECONDS = 100000;
constexpr float IMU_YAW_DIRECTION = rover::config::imuYawDirection;
constexpr std::size_t IMU_BIAS_SAMPLE_COUNT = 200;
constexpr std::uint8_t MPU6050_ADDRESS_PRIMARY = 0x68;
constexpr std::uint8_t MPU6050_ADDRESS_ALTERNATE = 0x69;
constexpr std::uint8_t MLX90640_ADDRESS = 0x33;
constexpr gpio_num_t SHARED_I2C_SDA_PIN = GPIO_NUM_47;
constexpr gpio_num_t SHARED_I2C_SCL_PIN = GPIO_NUM_48;
constexpr int I2C_PROBE_TIMEOUT_MS = 20;
constexpr std::uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
constexpr std::uint8_t MPU6050_WHO_AM_I = 0x75;
constexpr std::uint8_t MPU6050_IDENTITY = 0x68;
constexpr std::uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
constexpr std::uint8_t MPU6050_CONFIG = 0x1A;
constexpr std::uint8_t MPU6050_SMPLRT_DIV = 0x19;
constexpr std::uint8_t MPU6050_GYRO_CONFIG = 0x1B;
constexpr std::uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
constexpr uart_port_t LIDAR_UART_PORT = UART_NUM_1;
constexpr std::size_t LD19_PACKET_SIZE = 47;
constexpr std::uint8_t LD19_HEADER = 0x54;
constexpr std::uint8_t LD19_VERSION_LENGTH = 0x2C;
constexpr std::size_t LD19_CRC_OFFSET = 46;

rover::odometry::LidarImuOdometry lidarOdometry;
rover::odometry::Scan accumulatingLidarScan{};
rover::odometry::Estimate latestOdometry{};
float odometryImuYaw = 0;
i2c_master_dev_handle_t max17048Device = nullptr;
i2c_master_bus_handle_t sharedI2CBus = nullptr;
i2c_master_dev_handle_t mpu6050Device = nullptr;
i2c_master_dev_handle_t mlx90640Device = nullptr;
bool lidarUartReady = false;
bool mlx90640Ready = false;
std::atomic_bool motorDriverReady{false};
portMUX_TYPE poseMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE sensorHealthMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE imuYawMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE recordingMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE driveArmMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE thermalDiagnosticsMux = portMUX_INITIALIZER_UNLOCKED;
THERMAL_CAPTURE_DIAGNOSTICS thermalDiagnostics{};
float imuYawRadiansClockwise = 0.0f;
float imuGyroZDegreesPerSecondClockwise = 0.0f;
float imuGyroBiasDegreesPerSecond = 0.0f;
std::int64_t imuLastSampleTimeMicroseconds = 0;
bool imuYawValid = false;
std::uint8_t recordingState = 0;
TickType_t recordingLastCommandTick = 0;
bool driveArmed = false;
TickType_t driveArmLastCommandTick = 0;
UDPServer *sensorUdpServer = nullptr;
uint16_t sensorHealthFlags = 0;
uint16_t sensorInitErrorFlags = 0;
uint16_t reportedMotionCommandId = 0;
bool reportedMotionActive = false;
MotionResult reportedMotionResult = MOTION_RESULT_IDLE;
paramsMLX90640 mlx90640Parameters{};
uint16_t mlx90640EeData[MLX90640_EEPROM_DUMP_NUM]{};
uint16_t mlx90640FrameData[MLX90640_PIXEL_NUM + MLX90640_AUX_NUM + 2]{};
float mlx90640SubpageTemperatures[MLX90640_PIXEL_NUM]{};
float mlx90640Temperatures[MLX90640_PIXEL_NUM]{};
int16_t mlx90640CentiTemperatures[MLX90640_PIXEL_NUM]{};
} // namespace

static QueueHandle_t movementCommands = nullptr;
struct MotionEvent {
  uint8_t kind;
  PID_MOTION_CMD motion{};
  MOTION_KEEPALIVE_CMD keepalive{};
};
static QueueHandle_t motionEvents = nullptr;
constexpr TickType_t DRIVE_ARM_LEASE_TIMEOUT_TICKS = pdMS_TO_TICKS(750);
static int motorADirection = 0;
static int motorBDirection = 0;


static float wrapHeadingRadians(float heading) {
  if (!std::isfinite(heading)) return 0.0f;
  return std::remainder(heading, 2.0f * pi);
}

static void setSensorHealthFlag(SensorHealthFlag flag, bool enabled) {
  portENTER_CRITICAL(&sensorHealthMux);
  if (enabled) {
    sensorHealthFlags |= static_cast<uint16_t>(flag);
  } else {
    sensorHealthFlags &= static_cast<uint16_t>(~static_cast<uint16_t>(flag));
  }
  portEXIT_CRITICAL(&sensorHealthMux);
}

std::uint8_t getRecordingState() {
  const TickType_t now = xTaskGetTickCount();
  portENTER_CRITICAL(&recordingMux);
  if (recordingState != 0 &&
      now - recordingLastCommandTick >= pdMS_TO_TICKS(2500)) {
    recordingState = 0;
  }
  const std::uint8_t state = recordingState;
  portEXIT_CRITICAL(&recordingMux);
  return state;
}

template<typename Payload>
static void sendSensorPacket(PacketID id, const Payload &payload) {
  if (getRecordingState() != 1 || sensorUdpServer == nullptr) return;
  char destinationIp[IP_LEN]{};
  unsigned short destinationPort = 0;
  if (sensorUdpServer->getStatusDestination(destinationIp,
                                             sizeof(destinationIp),
                                             destinationPort)) {
    sensorUdpServer->send(Packet(DATA, id, payload), destinationIp,
                          destinationPort);
  }
}

static bool getImuYaw(float &yawRadiansClockwise);
static rover::control::MotionPose getMotionPose();

static uint16_t getSensorHealthFlags() {
  setSensorHealthFlag(SENSOR_LIDAR_IMU_ODOMETRY_READY,
                      getMotionPose().translationValid);
  portENTER_CRITICAL(&sensorHealthMux);
  const uint16_t flags = sensorHealthFlags;
  portEXIT_CRITICAL(&sensorHealthMux);
  return flags;
}

static void setSensorInitError(SensorInitErrorFlag flag) {
  portENTER_CRITICAL(&sensorHealthMux);
  sensorInitErrorFlags |= static_cast<uint16_t>(flag);
  portEXIT_CRITICAL(&sensorHealthMux);
}

static uint16_t getSensorInitErrors() {
  portENTER_CRITICAL(&sensorHealthMux);
  const uint16_t flags = sensorInitErrorFlags;
  portEXIT_CRITICAL(&sensorHealthMux);
  return flags;
}

static void latchMotorRuntimeFault(const char *operation, esp_err_t result) {
  const bool wasReady = motorDriverReady.exchange(false);
  setSensorHealthFlag(SENSOR_MOTORS_READY, false);
  setSensorInitError(SENSOR_ERROR_MOTOR_RUNTIME);

  // The TB6612 standby pin is the fastest software-controlled way to disable
  // both H bridges after a GPIO or LEDC operation fails. This is best effort:
  // the original operation failure may indicate a wider GPIO/peripheral fault.
  const esp_err_t standbyResult = gpio_set_level(STBY_PIN, 0);
  if (wasReady) {
    ESP_LOGE(WIFI_TAG, "Motor runtime operation %s failed: %s; disabling driver",
             operation, esp_err_to_name(result));
    if (standbyResult != ESP_OK) {
      ESP_LOGE(WIFI_TAG, "Could not assert TB6612 standby after motor fault: %s",
               esp_err_to_name(standbyResult));
    }
  }
}


void nvsFlashInit() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);
}

// Called only during startup, before any sensor/status sampling tasks exist.
// Read the actual input levels without changing pin modes or driving the bus.
static void logIdleI2CLineLevels(const char *stage) {
  constexpr int samples = 8;
  int sdaHigh = 0;
  int sclHigh = 0;
  for (int sample = 0; sample < samples; ++sample) {
    sdaHigh += gpio_get_level(SHARED_I2C_SDA_PIN) != 0;
    sclHigh += gpio_get_level(SHARED_I2C_SCL_PIN) != 0;
    if (sample + 1 < samples) vTaskDelay(1);
  }
  const auto levelName = [](int high) {
    return high == 0 ? "LOW" : high == samples ? "HIGH" : "CHANGING";
  };
  ESP_LOGI(WIFI_TAG,
           "I2C_DIAG %s: SDA GPIO%d=%s (%d/%d high), SCL GPIO%d=%s (%d/%d high)",
           stage, static_cast<int>(SHARED_I2C_SDA_PIN), levelName(sdaHigh),
           sdaHigh, samples, static_cast<int>(SHARED_I2C_SCL_PIN),
           levelName(sclHigh), sclHigh, samples);
  if (sdaHigh == 0 || sclHigh == 0) {
    ESP_LOGW(WIFI_TAG,
             "I2C_DIAG line remained LOW while no transaction was active; "
             "inspect shared signal wiring, ground and connected devices");
  }
}

static esp_err_t probeI2CDevice(std::uint8_t address) {
  if (sharedI2CBus == nullptr) return ESP_ERR_INVALID_STATE;
  const esp_err_t result =
      i2c_master_probe(sharedI2CBus, address, I2C_PROBE_TIMEOUT_MS);
  if (result == ESP_OK) {
    ESP_LOGI(WIFI_TAG, "I2C_DIAG address 0x%02X: ACK",
             static_cast<unsigned>(address));
  } else if (result == ESP_ERR_NOT_FOUND) {
    ESP_LOGW(WIFI_TAG, "I2C_DIAG address 0x%02X: NACK (no device acknowledged)",
             static_cast<unsigned>(address));
  } else {
    // A timeout is a shared bus failure, not evidence that this device is absent.
    setSensorInitError(SENSOR_ERROR_I2C_BUS);
    ESP_LOGE(WIFI_TAG, "I2C_DIAG address 0x%02X: %s (probe limit %d ms)",
             static_cast<unsigned>(address), esp_err_to_name(result),
             I2C_PROBE_TIMEOUT_MS);
    logIdleI2CLineLevels("after failed probe");
  }
  return result;
}

// Call only after this address has acknowledged its startup probe.
static bool addI2CDevice(std::uint8_t address,
                        i2c_master_dev_handle_t *deviceHandle) {
  if (sharedI2CBus == nullptr || deviceHandle == nullptr) return false;

  i2c_device_config_t deviceConfig{};
  deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  deviceConfig.device_address = address;
  deviceConfig.scl_speed_hz = 400000;
  const esp_err_t result =
      i2c_master_bus_add_device(sharedI2CBus, &deviceConfig, deviceHandle);
  if (result != ESP_OK) {
    setSensorInitError(SENSOR_ERROR_I2C_BUS);
    ESP_LOGW(WIFI_TAG, "Could not add I2C device 0x%02X: %s",
             static_cast<unsigned>(address), esp_err_to_name(result));
    return false;
  }
  return true;
}

static bool initializeMLX90640() {
  if (!addI2CDevice(MLX90640_ADDRESS, &mlx90640Device)) return false;
  setMLX90640I2CDevice(mlx90640Device);
  MLX90640_I2CInit();

  int result = MLX90640_DumpEE(MLX90640_ADDRESS, mlx90640EeData);
  if (result != MLX90640_NO_ERROR) {
    ESP_LOGW(WIFI_TAG, "MLX90640 EEPROM read failed (%d)", result);
    setSensorInitError(SENSOR_ERROR_MLX90640_CONFIG);
    return false;
  }
  result = MLX90640_ExtractParameters(mlx90640EeData,
                                     &mlx90640Parameters);
  if (result != MLX90640_NO_ERROR) {
    ESP_LOGW(WIFI_TAG, "MLX90640 calibration data invalid (%d)", result);
    setSensorInitError(SENSOR_ERROR_MLX90640_CONFIG);
    return false;
  }

  // Four subpage updates per second provide roughly two complete chess-mode
  // frames per second while keeping shared I2C traffic modest.
  if (MLX90640_SetResolution(MLX90640_ADDRESS, 3) != MLX90640_NO_ERROR ||
      MLX90640_SetRefreshRate(MLX90640_ADDRESS, 3) != MLX90640_NO_ERROR ||
      MLX90640_SetChessMode(MLX90640_ADDRESS) != MLX90640_NO_ERROR) {
    ESP_LOGW(WIFI_TAG, "MLX90640 configuration failed");
    setSensorInitError(SENSOR_ERROR_MLX90640_CONFIG);
    return false;
  }

  ESP_LOGI(WIFI_TAG, "MLX90640 calibrated and configured for chess mode");
  return true;
}

static bool writeMPU6050Register(std::uint8_t registerAddress,
                                 std::uint8_t value) {
  if (mpu6050Device == nullptr) return false;
  const std::uint8_t command[]{registerAddress, value};
  return i2c_master_transmit(mpu6050Device, command, sizeof(command), 50) ==
         ESP_OK;
}

static bool readMPU6050Register(std::uint8_t registerAddress,
                                std::uint8_t &value) {
  if (mpu6050Device == nullptr) return false;
  return i2c_master_transmit_receive(mpu6050Device, &registerAddress, 1,
                                     &value, 1, 50) == ESP_OK;
}

static bool initializeMPU6050() {
  if (sharedI2CBus == nullptr) return false;
  std::uint8_t address = MPU6050_ADDRESS_PRIMARY;
  esp_err_t probeResult = probeI2CDevice(address);
  if (probeResult == ESP_ERR_NOT_FOUND) {
    address = MPU6050_ADDRESS_ALTERNATE;
    probeResult = probeI2CDevice(address);
  }
  if (probeResult != ESP_OK) {
    if (probeResult == ESP_ERR_NOT_FOUND) {
      ESP_LOGW(WIFI_TAG, "MPU-6050 not detected at 0x68 or 0x69");
      setSensorInitError(SENSOR_ERROR_MPU6050_MISSING);
    } else {
      ESP_LOGW(WIFI_TAG, "MPU-6050 unavailable because its I2C probe failed");
    }
    return false;
  }
  if (!addI2CDevice(address, &mpu6050Device)) {
    setSensorInitError(SENSOR_ERROR_MPU6050_CONFIG);
    return false;
  }

  std::uint8_t identity = 0;
  if (!readMPU6050Register(MPU6050_WHO_AM_I, identity) ||
      identity != MPU6050_IDENTITY) {
    ESP_LOGW(WIFI_TAG, "Unexpected MPU-6050 identity at 0x%02X (WHO_AM_I 0x%02X)",
             static_cast<unsigned>(address), static_cast<unsigned>(identity));
    setSensorInitError(SENSOR_ERROR_MPU6050_CONFIG);
    i2c_master_bus_rm_device(mpu6050Device);
    mpu6050Device = nullptr;
    return false;
  }

  // Wake the sensor, select a 44 Hz gyro/accel low-pass filter, sample at
  // 100 Hz, and use +/-500 degrees/second gyro range with +/-2g accelerometer.
  const bool configured =
      writeMPU6050Register(MPU6050_PWR_MGMT_1, 0x00) &&
      writeMPU6050Register(MPU6050_CONFIG, 0x03) &&
      writeMPU6050Register(MPU6050_SMPLRT_DIV, 0x09) &&
      writeMPU6050Register(MPU6050_GYRO_CONFIG, 0x08) &&
      writeMPU6050Register(MPU6050_ACCEL_CONFIG, 0x00);
  if (!configured) {
    ESP_LOGW(WIFI_TAG, "MPU-6050 configuration failed");
    setSensorInitError(SENSOR_ERROR_MPU6050_CONFIG);
    i2c_master_bus_rm_device(mpu6050Device);
    mpu6050Device = nullptr;
    return false;
  }
  ESP_LOGI(WIFI_TAG, "MPU-6050 initialized at I2C address 0x%02X",
           static_cast<unsigned>(address));
  return true;
}

static void initializeSharedI2CBus() {
  i2c_master_bus_config_t busConfig{};
  busConfig.i2c_port = I2C_NUM_0;
  busConfig.sda_io_num = SHARED_I2C_SDA_PIN;
  busConfig.scl_io_num = SHARED_I2C_SCL_PIN;
  busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
  busConfig.glitch_ignore_cnt = 7;
  busConfig.flags.enable_internal_pullup = true;
  const esp_err_t busResult = i2c_new_master_bus(&busConfig, &sharedI2CBus);
  if (busResult != ESP_OK) {
    ESP_LOGE(WIFI_TAG, "I2C bus initialization failed: %s",
             esp_err_to_name(busResult));
    setSensorInitError(SENSOR_ERROR_I2C_BUS);
    return;
  }

  logIdleI2CLineLevels("before probes");
  const esp_err_t batteryProbe = probeI2CDevice(MAX17048_ADDR);
  if (batteryProbe == ESP_OK && addI2CDevice(MAX17048_ADDR, &max17048Device)) {
    ESP_LOGI(WIFI_TAG, "MAX17048 battery monitor detected");
    setSensorHealthFlag(SENSOR_BATTERY_READY, true);
  } else {
    if (batteryProbe == ESP_ERR_NOT_FOUND) {
      ESP_LOGW(WIFI_TAG, "MAX17048 battery monitor not detected");
      setSensorInitError(SENSOR_ERROR_BATTERY_MISSING);
    } else {
      ESP_LOGW(WIFI_TAG, "MAX17048 unavailable because its I2C probe/setup failed");
    }
  }

  const esp_err_t thermalProbe = probeI2CDevice(MLX90640_ADDRESS);
  if (thermalProbe == ESP_OK) {
    ESP_LOGI(WIFI_TAG, "MLX90640 detected at I2C address 0x33");
    setSensorHealthFlag(SENSOR_MLX90640_DETECTED, true);
    mlx90640Ready = initializeMLX90640();
    if (!mlx90640Ready) {
      setSensorHealthFlag(SENSOR_MLX90640_STREAMING, false);
      setSensorInitError(SENSOR_ERROR_MLX90640_CONFIG);
    }
  } else {
    if (thermalProbe == ESP_ERR_NOT_FOUND) {
      ESP_LOGW(WIFI_TAG, "MLX90640 not detected at I2C address 0x33");
      setSensorInitError(SENSOR_ERROR_MLX90640_MISSING);
    } else {
      ESP_LOGW(WIFI_TAG, "MLX90640 unavailable because its I2C probe failed");
    }
  }

  if (!initializeMPU6050()) {
    setSensorHealthFlag(SENSOR_MPU6050_STREAMING, false);
    setSensorHealthFlag(SENSOR_MPU6050_CALIBRATED, false);
  }
}

static std::uint16_t readLittleEndianUnsigned(const std::uint8_t *bytes) {
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(
             static_cast<std::uint16_t>(bytes[1]) << 8U);
}

static std::uint8_t ld19Crc8(const std::uint8_t *bytes, std::size_t count) {
  std::uint8_t crc = 0;
  for (std::size_t i = 0; i < count; ++i) {
    crc = static_cast<std::uint8_t>(crc ^ bytes[i]);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0
                ? static_cast<std::uint8_t>((crc << 1U) ^ 0x4DU)
                : static_cast<std::uint8_t>(crc << 1U);
    }
  }
  return crc;
}

static bool isValidLd19Packet(const std::uint8_t *packet) {
  if (packet[0] != LD19_HEADER || packet[1] != LD19_VERSION_LENGTH ||
      ld19Crc8(packet, LD19_CRC_OFFSET) != packet[LD19_CRC_OFFSET]) {
    return false;
  }
  return readLittleEndianUnsigned(packet + 4) < 36000U &&
         readLittleEndianUnsigned(packet + 42) < 36000U;
}

static bool initializeLidarUart() {
  gpio_config_t pwmConfig{};
  pwmConfig.pin_bit_mask = 1ULL << rover::config::lidarPwmGpio;
  pwmConfig.mode = GPIO_MODE_OUTPUT;
  pwmConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  pwmConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  pwmConfig.intr_type = GPIO_INTR_DISABLE;
  esp_err_t result = gpio_config(&pwmConfig);
  if (result != ESP_OK) {
    ESP_LOGW(WIFI_TAG, "Could not configure LiDAR PWM pin: %s",
             esp_err_to_name(result));
    setSensorInitError(SENSOR_ERROR_LD19_UART);
    return false;
  }
  result = gpio_set_level(static_cast<gpio_num_t>(rover::config::lidarPwmGpio),
                          0);
  if (result != ESP_OK) {
    ESP_LOGW(WIFI_TAG, "Could not hold LiDAR PWM low: %s",
             esp_err_to_name(result));
    setSensorInitError(SENSOR_ERROR_LD19_UART);
    return false;
  }

  uart_config_t uartConfig{};
  uartConfig.baud_rate = rover::config::lidarBaudRate;
  uartConfig.data_bits = UART_DATA_8_BITS;
  uartConfig.parity = UART_PARITY_DISABLE;
  uartConfig.stop_bits = UART_STOP_BITS_1;
  uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uartConfig.source_clk = UART_SCLK_DEFAULT;
  result = uart_param_config(LIDAR_UART_PORT, &uartConfig);
  if (result == ESP_OK) {
    result = uart_set_pin(
        LIDAR_UART_PORT, UART_PIN_NO_CHANGE,
        static_cast<gpio_num_t>(rover::config::lidarRxGpio),
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  }
  if (result == ESP_OK) {
    result = uart_driver_install(LIDAR_UART_PORT, 4096, 0, 0, nullptr, 0);
  }
  if (result != ESP_OK) {
    ESP_LOGW(WIFI_TAG, "Could not initialize LD19 UART: %s",
             esp_err_to_name(result));
    setSensorInitError(SENSOR_ERROR_LD19_UART);
    return false;
  }

  ESP_LOGI(WIFI_TAG, "LD19 UART ready at %d baud on RX GPIO%d; PWM GPIO%d held low",
           rover::config::lidarBaudRate, rover::config::lidarRxGpio,
           rover::config::lidarPwmGpio);
  return true;
}

static void lidarReaderTask(void *param) {
  (void)param;
  std::uint8_t packet[LD19_PACKET_SIZE]{};
  std::size_t packetBytes = 0;
  std::uint8_t input[256]{};
  std::int64_t lastValidPacketTimeMicroseconds = 0;
  std::uint16_t previousStartAngle = 0;
  bool hasPreviousAngle = false;
  std::uint32_t scanTime = 0;
  float scanImuYaw = 0;
  bool scanImuValid = false;

  while (true) {
    const int bytesRead = uart_read_bytes(LIDAR_UART_PORT, input, sizeof(input),
                                          pdMS_TO_TICKS(20));
    for (int i = 0; i < bytesRead; ++i) {
      const std::uint8_t value = input[i];
      if (packetBytes == 0) {
        if (value == LD19_HEADER) packet[packetBytes++] = value;
        continue;
      }

      packet[packetBytes++] = value;
      if (packetBytes == 2 && packet[1] != LD19_VERSION_LENGTH) {
        packetBytes = value == LD19_HEADER ? 1 : 0;
        if (packetBytes == 1) packet[0] = LD19_HEADER;
        continue;
      }
      if (packetBytes < LD19_PACKET_SIZE) continue;

      const std::int64_t nowMicroseconds = esp_timer_get_time();
      if (isValidLd19Packet(packet)) {
        const auto startAngle = readLittleEndianUnsigned(packet + 4);
        if (hasPreviousAngle && previousStartAngle > 27000U && startAngle < 9000U) {
          const auto estimate = lidarOdometry.update(accumulatingLidarScan, scanTime,
                                                      scanImuYaw, scanImuValid);
          portENTER_CRITICAL(&poseMux);
          latestOdometry = estimate;
          odometryImuYaw = scanImuYaw;
          portEXIT_CRITICAL(&poseMux);
          FUSED_ODOMETRY_DATA fused{};
          fused.timestampMilliseconds = estimate.timestampMilliseconds;
          fused.xCentimeters = estimate.pose.xCm;
          fused.yCentimeters = estimate.pose.yCm;
          fused.headingRadiansClockwise = estimate.pose.headingRadians;
          fused.residualCentimeters = estimate.residualCm;
          fused.matchedPoints = estimate.matchedPoints;
          fused.resetCount = estimate.resets;
          fused.quality = static_cast<std::uint8_t>(estimate.quality);
          sendSensorPacket(FUSED_ODOMETRY, fused);
          accumulatingLidarScan = {};
        }
        const auto endAngle = readLittleEndianUnsigned(packet + 42);
        const std::uint32_t span = (static_cast<std::uint32_t>(endAngle) + 36000U - startAngle) % 36000U;
        for(std::size_t point = 0; point < 12; ++point) {
          const float angle = (static_cast<float>(startAngle) +
              static_cast<float>(span) * static_cast<float>(point) / 11.0f) * pi / 18000.0f;
          const float distance = static_cast<float>(readLittleEndianUnsigned(packet + 6 + point * 3)) * 0.1f;
          rover::odometry::addPolarPoint(accumulatingLidarScan, angle, distance);
        }
        previousStartAngle = startAngle;
        hasPreviousAngle = true;
        scanTime = static_cast<std::uint32_t>(nowMicroseconds / 1000);
        scanImuValid = getImuYaw(scanImuYaw);
        lastValidPacketTimeMicroseconds = nowMicroseconds;
        setSensorHealthFlag(SENSOR_LD19_STREAMING, true);
        LIDAR_PACKET_DATA payload{};
        payload.timestampMilliseconds =
            static_cast<std::uint32_t>(nowMicroseconds / 1000);
        std::memcpy(payload.packet, packet, sizeof(payload.packet));
        sendSensorPacket(LIDAR_PACKET, payload);
        packetBytes = 0;
        continue;
      }

      std::size_t nextHeader = 1;
      while (nextHeader < LD19_PACKET_SIZE &&
             packet[nextHeader] != LD19_HEADER) {
        ++nextHeader;
      }
      if (nextHeader < LD19_PACKET_SIZE) {
        packetBytes = LD19_PACKET_SIZE - nextHeader;
        std::memmove(packet, packet + nextHeader, packetBytes);
      } else {
        packetBytes = 0;
      }
    }

    if (lastValidPacketTimeMicroseconds == 0 ||
        esp_timer_get_time() - lastValidPacketTimeMicroseconds > 1000000) {
      setSensorHealthFlag(SENSOR_LD19_STREAMING, false);
    }
  }
}

static std::int16_t readBigEndianSigned(const std::uint8_t *bytes) {
  const std::uint16_t value =
      (static_cast<std::uint16_t>(bytes[0]) << 8U) |
      static_cast<std::uint16_t>(bytes[1]);
  return static_cast<std::int16_t>(value);
}

static void waitForNextImuSample(TickType_t &lastWakeTime) {
  constexpr TickType_t period = pdMS_TO_TICKS(10) > 0 ? pdMS_TO_TICKS(10) : 1;
  const TickType_t now = xTaskGetTickCount();
  // A failed/blocking transfer can miss many periods. Discard those deadlines
  // rather than issuing a burst of reads to catch up and compete with the
  // other devices on the shared bus. Normal sampling retains its fixed cadence.
  if (static_cast<TickType_t>(now - lastWakeTime) > period) {
    lastWakeTime = now;
  }
  vTaskDelayUntil(&lastWakeTime, period);
}

static void mpu6050SamplingTask(void *param) {
  (void)param;
  if (mpu6050Device == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  TickType_t lastWakeTime = xTaskGetTickCount();
  std::size_t calibrationSamples = 0;
  double gyroBiasDegreesPerSecond = 0.0;
  std::int64_t previousSampleTimeMicroseconds = 0;
  int consecutiveErrors = 0;
  TickType_t lastReadErrorLogTick = 0;
  bool hasReadErrorLog = false;

  while (true) {
    std::uint8_t registerAddress = MPU6050_ACCEL_XOUT_H;
    std::uint8_t sample[14]{};
    const esp_err_t result = i2c_master_transmit_receive(
        mpu6050Device, &registerAddress, 1, sample, sizeof(sample), 25);
    const std::int64_t sampleTimeMicroseconds = esp_timer_get_time();
    if (result != ESP_OK) {
      ++consecutiveErrors;
      const TickType_t now = xTaskGetTickCount();
      if (!hasReadErrorLog ||
          static_cast<TickType_t>(now - lastReadErrorLogTick) >= pdMS_TO_TICKS(5000)) {
        ESP_LOGW(WIFI_TAG, "I2C_DIAG MPU-6050 runtime read: %s (%d consecutive failures)",
                 esp_err_to_name(result), consecutiveErrors);
        lastReadErrorLogTick = now;
        hasReadErrorLog = true;
      }
      previousSampleTimeMicroseconds = 0;
      if (calibrationSamples < IMU_BIAS_SAMPLE_COUNT) {
        calibrationSamples = 0;
        gyroBiasDegreesPerSecond = 0.0;
      }
      if (consecutiveErrors >= 3) {
        setSensorHealthFlag(SENSOR_MPU6050_STREAMING, false);
        portENTER_CRITICAL(&imuYawMux);
        imuYawValid = false;
        portEXIT_CRITICAL(&imuYawMux);
      }
      waitForNextImuSample(lastWakeTime);
      continue;
    }

    consecutiveErrors = 0;
    portENTER_CRITICAL(&imuYawMux);
    imuLastSampleTimeMicroseconds = sampleTimeMicroseconds;
    portEXIT_CRITICAL(&imuYawMux);
    setSensorHealthFlag(SENSOR_MPU6050_STREAMING, true);
    const std::int16_t gyroZRaw = readBigEndianSigned(sample + 12);
    const double gyroZDegreesPerSecond =
        static_cast<double>(gyroZRaw) /
        static_cast<double>(IMU_GYRO_LSB_PER_DEGREE_PER_SECOND);

    if (calibrationSamples < IMU_BIAS_SAMPLE_COUNT) {
      const std::int16_t accelXRaw = readBigEndianSigned(sample);
      const std::int16_t accelYRaw = readBigEndianSigned(sample + 2);
      const std::int16_t accelZRaw = readBigEndianSigned(sample + 4);
      const double accelMagnitudeG = std::sqrt(
          static_cast<double>(accelXRaw) * accelXRaw +
          static_cast<double>(accelYRaw) * accelYRaw +
          static_cast<double>(accelZRaw) * accelZRaw) /
          static_cast<double>(IMU_ACCEL_LSB_PER_G);
      const bool stationary =
          std::fabs(gyroZDegreesPerSecond) <=
              IMU_CALIBRATION_MAX_GYRO_DEGREES_PER_SECOND &&
          std::fabs(accelMagnitudeG - 1.0) <=
              IMU_CALIBRATION_MAX_ACCEL_DEVIATION_G;
      if (!stationary) {
        calibrationSamples = 0;
        gyroBiasDegreesPerSecond = 0.0;
        previousSampleTimeMicroseconds = sampleTimeMicroseconds;
        waitForNextImuSample(lastWakeTime);
        continue;
      }

      gyroBiasDegreesPerSecond += gyroZDegreesPerSecond;
      ++calibrationSamples;
      if (calibrationSamples == IMU_BIAS_SAMPLE_COUNT) {
        gyroBiasDegreesPerSecond /= static_cast<double>(IMU_BIAS_SAMPLE_COUNT);
        portENTER_CRITICAL(&imuYawMux);
        imuGyroBiasDegreesPerSecond =
            static_cast<float>(gyroBiasDegreesPerSecond);
        imuYawValid = false;
        portEXIT_CRITICAL(&imuYawMux);
        setSensorHealthFlag(SENSOR_MPU6050_CALIBRATED, true);
        ESP_LOGI(WIFI_TAG, "MPU-6050 stationary gyro calibration complete: %.3f deg/s",
                 gyroBiasDegreesPerSecond);
      }
      previousSampleTimeMicroseconds = sampleTimeMicroseconds;
      waitForNextImuSample(lastWakeTime);
      continue;
    }

    if (previousSampleTimeMicroseconds > 0) {
      const std::int64_t elapsedMicroseconds =
          sampleTimeMicroseconds - previousSampleTimeMicroseconds;
      if (elapsedMicroseconds > 0 && elapsedMicroseconds <= 100000) {
        const double elapsedSeconds =
            static_cast<double>(elapsedMicroseconds) / 1000000.0;
        const double clockwiseDegreesPerSecond =
            (gyroZDegreesPerSecond - gyroBiasDegreesPerSecond) *
            static_cast<double>(IMU_YAW_DIRECTION);
        portENTER_CRITICAL(&imuYawMux);
        imuGyroZDegreesPerSecondClockwise =
            static_cast<float>(clockwiseDegreesPerSecond);
        portEXIT_CRITICAL(&imuYawMux);
        const float yawDeltaRadians = static_cast<float>(
            clockwiseDegreesPerSecond * elapsedSeconds *
            static_cast<double>(pi) / 180.0);
        portENTER_CRITICAL(&imuYawMux);
        imuYawRadiansClockwise = wrapHeadingRadians(
            imuYawRadiansClockwise + yawDeltaRadians);
        imuYawValid = true;
        portEXIT_CRITICAL(&imuYawMux);
      }
    }

    float yawRadiansClockwise = 0.0f;
    if (getImuYaw(yawRadiansClockwise) &&
        getRecordingState() == 1) {
      IMU_SAMPLE_DATA imuSample{};
      imuSample.timestampMilliseconds = static_cast<std::uint32_t>(
          sampleTimeMicroseconds / 1000);
      portENTER_CRITICAL(&imuYawMux);
      imuSample.gyroZDegreesPerSecondClockwise =
          imuGyroZDegreesPerSecondClockwise;
      portEXIT_CRITICAL(&imuYawMux);
      imuSample.yawRadiansClockwise = yawRadiansClockwise;
      sendSensorPacket(IMU_SAMPLE, imuSample);
    }
    previousSampleTimeMicroseconds = sampleTimeMicroseconds;
    waitForNextImuSample(lastWakeTime);
  }
}

static void mlx90640AcquisitionTask(void *param) {
  (void)param;
  if (!mlx90640Ready) {
    vTaskDelete(nullptr);
    return;
  }

  std::uint8_t receivedSubpages = 0;
  std::int64_t subpageCaptureTimes[2]{};
  std::int64_t lastSubpageTimeMicroseconds = 0;
  std::uint16_t frameId = 0;
  int consecutiveErrors = 0;
  THERMAL_CAPTURE_DIAGNOSTICS diagnostics{};
  const auto publishStage = [&](ThermalCaptureStage stage) {
    diagnostics.stage = stage;
    portENTER_CRITICAL(&thermalDiagnosticsMux);
    thermalDiagnostics = diagnostics;
    portEXIT_CRITICAL(&thermalDiagnosticsMux);
  };

  while (true) {
    if (getRecordingState() != 1) {
      receivedSubpages = 0;
      publishStage(ThermalCaptureStage::Idle);
      setSensorHealthFlag(SENSOR_MLX90640_STREAMING, false);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    std::uint16_t statusRegister = 0;
    const int statusResult = MLX90640_I2CRead(
        MLX90640_ADDRESS, MLX90640_STATUS_REG, 1, &statusRegister);
    if (statusResult != MLX90640_NO_ERROR) {
      ++consecutiveErrors;
      ++diagnostics.readErrors;
      diagnostics.lastResult = static_cast<std::int16_t>(statusResult);
      publishStage(ThermalCaptureStage::ReadError);
      if (consecutiveErrors >= 3) {
        setSensorHealthFlag(SENSOR_MLX90640_STREAMING, false);
      }
      receivedSubpages = 0;
      vTaskDelay(pdMS_TO_TICKS(25));
      continue;
    }
    diagnostics.statusRegister = statusRegister;
    if (MLX90640_GET_DATA_READY(statusRegister) == 0) {
      publishStage(ThermalCaptureStage::WaitingForData);
      if (lastSubpageTimeMicroseconds > 0 &&
          esp_timer_get_time() - lastSubpageTimeMicroseconds > 1000000) {
        setSensorHealthFlag(SENSOR_MLX90640_STREAMING, false);
        receivedSubpages = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // The Melexis helper polls internally without a timeout. The ready-bit
    // check above guarantees its first poll can complete immediately.
    publishStage(ThermalCaptureStage::Reading);
    const int subpage = MLX90640_GetFrameData(
        MLX90640_ADDRESS, mlx90640FrameData);
    if (subpage < 0 || subpage > 1) {
      ++consecutiveErrors;
      ++diagnostics.readErrors;
      diagnostics.lastResult = static_cast<std::int16_t>(subpage);
      publishStage(ThermalCaptureStage::ReadError);
      receivedSubpages = 0;
      if (consecutiveErrors >= 3) {
        setSensorHealthFlag(SENSOR_MLX90640_STREAMING, false);
      }
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    diagnostics.lastSubpage = static_cast<std::uint8_t>(subpage);
    subpageCaptureTimes[subpage] = esp_timer_get_time();
    diagnostics.lastSubpageMilliseconds = static_cast<std::uint32_t>(subpageCaptureTimes[subpage] / 1000);
    if(subpage == 0) ++diagnostics.subpage0Count;
    else ++diagnostics.subpage1Count;
    publishStage(ThermalCaptureStage::Converting);
    const float ambientTemperature = MLX90640_GetTa(
        mlx90640FrameData, &mlx90640Parameters);
    MLX90640_CalculateTo(mlx90640FrameData, &mlx90640Parameters, 0.95f,
                         ambientTemperature - 8.0f,
                         mlx90640SubpageTemperatures);
    for (int pixel = 0; pixel < MLX90640_PIXEL_NUM; ++pixel) {
      const int linePattern = pixel / 32 - (pixel / 64) * 2;
      const int chessPattern = linePattern ^ (pixel & 1);
      if (chessPattern == subpage) {
        mlx90640Temperatures[pixel] = mlx90640SubpageTemperatures[pixel];
      }
    }

    const std::uint8_t subpageBit = static_cast<std::uint8_t>(1U << subpage);
    if ((receivedSubpages & subpageBit) != 0) {
      // A repeated page means the complementary page was lost. Start a fresh
      // pair instead of publishing a partially stale thermal frame.
      receivedSubpages = 0;
    }
    receivedSubpages = static_cast<std::uint8_t>(receivedSubpages | subpageBit);
    lastSubpageTimeMicroseconds = esp_timer_get_time();

    if (receivedSubpages != 0x03U) {
      publishStage(ThermalCaptureStage::WaitingForPartner);
      continue;
    }
    receivedSubpages = 0;
    MLX90640_BadPixelsCorrection(mlx90640Parameters.brokenPixels,
                                 mlx90640Temperatures, 1,
                                 &mlx90640Parameters);
    MLX90640_BadPixelsCorrection(mlx90640Parameters.outlierPixels,
                                 mlx90640Temperatures, 1,
                                 &mlx90640Parameters);

    bool validFrame = true;
    for (int pixel = 0; pixel < MLX90640_PIXEL_NUM; ++pixel) {
      const float temperature = mlx90640Temperatures[pixel];
      if (!std::isfinite(temperature) || temperature < -40.0f ||
          temperature > 300.0f) {
        validFrame = false;
        diagnostics.invalidPixel = static_cast<std::uint16_t>(pixel);
        break;
      }
      const long centiDegrees = std::lround(temperature * 100.0f);
      mlx90640CentiTemperatures[pixel] = static_cast<std::int16_t>(
          std::clamp(centiDegrees, -4000L, 30000L));
    }
    if (!validFrame) {
      ++consecutiveErrors;
      ++diagnostics.invalidFrames;
      publishStage(ThermalCaptureStage::InvalidTemperature);
      if (consecutiveErrors >= 3) {
        setSensorHealthFlag(SENSOR_MLX90640_STREAMING, false);
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Timestamp the observation between its two acquired subpages, rather
    // than after conversion. Mapping interpolates the camera pose at this time.
    const auto captureMidpoint = subpageCaptureTimes[0] +
        (subpageCaptureTimes[1] - subpageCaptureTimes[0]) / 2;
    const std::uint32_t timestampMilliseconds = static_cast<std::uint32_t>(captureMidpoint / 1000);
    for (std::size_t offset = 0; offset < MLX90640_PIXEL_NUM;
         offset += THERMAL_CHUNK_PIXELS) {
      THERMAL_FRAME_CHUNK chunk{};
      chunk.timestampMilliseconds = timestampMilliseconds;
      chunk.frameId = frameId;
      chunk.pixelOffset = static_cast<std::uint16_t>(offset);
      const std::size_t sampleCount = std::min<std::size_t>(
          THERMAL_CHUNK_PIXELS, MLX90640_PIXEL_NUM - offset);
      chunk.sampleCount = static_cast<std::uint16_t>(sampleCount);
      std::memcpy(chunk.temperaturesCentiC,
                  mlx90640CentiTemperatures + offset,
                  sampleCount * sizeof(chunk.temperaturesCentiC[0]));
      sendSensorPacket(THERMAL_CHUNK, chunk);
    }
    ++frameId;
    ++diagnostics.completedFrames;
    diagnostics.lastResult = 0;
    diagnostics.invalidPixel = 0xffff;
    publishStage(ThermalCaptureStage::Streaming);
    consecutiveErrors = 0;
    setSensorHealthFlag(SENSOR_MLX90640_STREAMING, true);
  }
}

static bool getImuYaw(float &yawRadiansClockwise) {
  const std::int64_t nowMicroseconds = esp_timer_get_time();
  portENTER_CRITICAL(&imuYawMux);
  const bool sampleFresh = imuLastSampleTimeMicroseconds > 0 &&
      nowMicroseconds >= imuLastSampleTimeMicroseconds &&
      nowMicroseconds - imuLastSampleTimeMicroseconds <=
          IMU_SAMPLE_STALE_TIMEOUT_MICROSECONDS;
  if (!sampleFresh) imuYawValid = false;
  const bool valid = imuYawValid && sampleFresh;
  yawRadiansClockwise = imuYawRadiansClockwise;
  portEXIT_CRITICAL(&imuYawMux);
  if (!sampleFresh) setSensorHealthFlag(SENSOR_MPU6050_STREAMING, false);
  return valid;
}

static rover::control::MotionPose getMotionPose() {
  rover::odometry::Estimate estimate;
  float referenceImuYaw = 0;
  portENTER_CRITICAL(&poseMux);
  estimate = latestOdometry;
  referenceImuYaw = odometryImuYaw;
  portEXIT_CRITICAL(&poseMux);
  float currentYaw = 0;
  const bool imuReady = getImuYaw(currentYaw);
  rover::control::MotionPose result;
  result.pose = estimate.pose;
  result.headingValid = imuReady;
  if(imuReady) {
    result.pose.headingRadians = estimate.headingValid
        ? wrapHeadingRadians(estimate.pose.headingRadians + wrapHeadingRadians(currentYaw - referenceImuYaw))
        : currentYaw;
  }
  const auto now = static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
  result.translationValid = imuReady && estimate.translationValid &&
      now - estimate.timestampMilliseconds <= 350U;
  return result;
}

float max17048_get_percent() {
  if (max17048Device == nullptr) return -1.0f;
  uint8_t reg = 0x04;
  uint8_t data[2];

  if (i2c_master_transmit_receive(max17048Device, &reg, 1, data, 2, 100) !=
      ESP_OK) {
    setSensorHealthFlag(SENSOR_BATTERY_READY, false);
    return -1.0f;
  }
  setSensorHealthFlag(SENSOR_BATTERY_READY, true);

  uint16_t raw = (data[0] << 8) | data[1];

  // Upper byte = integer %
  // Lower byte = fractional part (1/256 %)
  const float percent = static_cast<float>(raw >> 8) +
                        static_cast<float>(raw & 0xFFU) / 256.0f;
  return std::clamp(percent, 0.0f, 100.0f);
}

float max17048_get_voltage() {
  if (max17048Device == nullptr) return -1.0f;
  uint8_t reg = 0x02;
  uint8_t data[2];

  if (i2c_master_transmit_receive(max17048Device, &reg, 1, data, 2, 100) !=
      ESP_OK) {
    setSensorHealthFlag(SENSOR_BATTERY_READY, false);
    return -1.0f;
  }
  setSensorHealthFlag(SENSOR_BATTERY_READY, true);

  uint16_t raw = (data[0] << 8) | data[1];

  // Each LSB = 78.125 µV
  return raw * 78.125e-6f;
}

bool motor_init(void) {
  motorDriverReady.store(false);

  // Keep the TB6612 disabled while configuring the direction and PWM pins.
  gpio_config_t standbyConfig{};
  standbyConfig.pin_bit_mask = 1ULL << STBY_PIN;
  standbyConfig.mode = GPIO_MODE_OUTPUT;
  standbyConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  standbyConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
  standbyConfig.intr_type = GPIO_INTR_DISABLE;
  esp_err_t result = gpio_config(&standbyConfig);
  if (result != ESP_OK) {
    ESP_LOGE(WIFI_TAG, "Motor standby GPIO setup failed: %s",
             esp_err_to_name(result));
    return false;
  }
  result = gpio_set_level(STBY_PIN, 0);
  if (result != ESP_OK) {
    ESP_LOGE(WIFI_TAG, "Could not hold TB6612 in standby: %s",
             esp_err_to_name(result));
    return false;
  }

  gpio_config_t directionConfig{};
  directionConfig.pin_bit_mask = (1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) |
                                 (1ULL << BIN1_PIN) | (1ULL << BIN2_PIN);
  directionConfig.mode = GPIO_MODE_OUTPUT;
  directionConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  directionConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
  directionConfig.intr_type = GPIO_INTR_DISABLE;
  result = gpio_config(&directionConfig);
  if (result != ESP_OK) {
    ESP_LOGE(WIFI_TAG, "Motor direction GPIO setup failed: %s",
             esp_err_to_name(result));
    return false;
  }
  result = gpio_set_level(AIN1_PIN, 0);
  if (result == ESP_OK) result = gpio_set_level(AIN2_PIN, 0);
  if (result == ESP_OK) result = gpio_set_level(BIN1_PIN, 0);
  if (result == ESP_OK) result = gpio_set_level(BIN2_PIN, 0);
  if (result != ESP_OK) {
    ESP_LOGE(WIFI_TAG, "Could not set motor direction pins low: %s",
             esp_err_to_name(result));
    return false;
  }

  ledc_timer_config_t timer{};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = 20000;
  timer.clk_cfg = LEDC_AUTO_CLK;
  result = ledc_timer_config(&timer);
  if (result != ESP_OK) {
    ESP_LOGE(WIFI_TAG, "Motor PWM timer setup failed: %s",
             esp_err_to_name(result));
    return false;
  }

  ledc_channel_config_t channel{};
  channel.gpio_num = PWMA_PIN;
  channel.speed_mode = LEDC_LOW_SPEED_MODE;
  channel.channel = LEDC_CHANNEL_0;
  channel.timer_sel = LEDC_TIMER_0;
  channel.duty = 0;
  channel.hpoint = 0;
  result = ledc_channel_config(&channel);
  if (result == ESP_OK) {
    channel.gpio_num = PWMB_PIN;
    channel.channel = LEDC_CHANNEL_1;
    result = ledc_channel_config(&channel);
  }
  if (result != ESP_OK) {
    ESP_LOGE(WIFI_TAG, "Motor PWM channel setup failed: %s",
             esp_err_to_name(result));
    return false;
  }

  result = gpio_set_level(STBY_PIN, 1);
  if (result != ESP_OK) {
    gpio_set_level(STBY_PIN, 0);
    ESP_LOGE(WIFI_TAG, "Could not enable TB6612 after setup: %s",
             esp_err_to_name(result));
    return false;
  }
  motorDriverReady.store(true);
  return true;
}

bool motor_stop(void) {
  if (!motorDriverReady.load()) {
    (void)gpio_set_level(STBY_PIN, 0);
    return false;
  }

  bool success = true;
  auto check = [&](const char *operation, esp_err_t result) {
    if (result != ESP_OK) {
      latchMotorRuntimeFault(operation, result);
      success = false;
    }
  };
  check("zero left PWM", ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
  check("apply zero left PWM",
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
  check("zero right PWM", ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
  check("apply zero right PWM",
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
  check("clear left direction A", gpio_set_level(AIN1_PIN, 0));
  check("clear left direction B", gpio_set_level(AIN2_PIN, 0));
  check("clear right direction A", gpio_set_level(BIN1_PIN, 0));
  check("clear right direction B", gpio_set_level(BIN2_PIN, 0));
  motorADirection = 0;
  motorBDirection = 0;
  return success;
}

static void disableMotorForControlFailure(const char *reason) {
  motorDriverReady.store(false);
  setSensorHealthFlag(SENSOR_MOTORS_READY, false);
  setSensorInitError(SENSOR_ERROR_MOTOR_TASK);
  const esp_err_t standbyResult = gpio_set_level(STBY_PIN, 0);
  if (standbyResult != ESP_OK) {
    ESP_LOGE(WIFI_TAG, "Could not assert TB6612 standby after %s failure: %s",
             reason, esp_err_to_name(standbyResult));
  }
  ESP_LOGE(WIFI_TAG, "%s failed; motor driver disabled", reason);
}

static bool setMotorSpeed(int speed, gpio_num_t in1, gpio_num_t in2,
                          ledc_channel_t channel, bool invertDirection) {
  const int magnitude = speed < 0 ? -speed : speed;
  const int direction = speed > 0 ? 1 : (speed < 0 ? -1 : 0);
  int &currentDirection = channel == LEDC_CHANNEL_0 ? motorADirection
                                                    : motorBDirection;
  auto check = [&](const char *operation, esp_err_t result) {
    if (result == ESP_OK) return true;
    latchMotorRuntimeFault(operation, result);
    return false;
  };

  if (direction != currentDirection) {
    // Briefly remove PWM only when changing direction. Reapplying a heartbeat
    // at the same direction/speed no longer interrupts the motor.
    if (!check("zero PWM before direction change",
               ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0)) ||
        !check("apply zero PWM before direction change",
               ledc_update_duty(LEDC_LOW_SPEED_MODE, channel))) {
      return false;
    }

    esp_err_t directionResult = ESP_OK;
    if (direction == 0) {
      directionResult = gpio_set_level(in1, 0);
      if (directionResult == ESP_OK) directionResult = gpio_set_level(in2, 0);
    } else if ((direction > 0) != invertDirection) {
      directionResult = gpio_set_level(in1, 1);
      if (directionResult == ESP_OK) directionResult = gpio_set_level(in2, 0);
    } else {
      directionResult = gpio_set_level(in1, 0);
      if (directionResult == ESP_OK) directionResult = gpio_set_level(in2, 1);
    }
    if (!check("set motor direction pins", directionResult)) return false;
    currentDirection = direction;
  }

  if (!check("set PWM duty", ledc_set_duty(
          LEDC_LOW_SPEED_MODE, channel, static_cast<uint32_t>(magnitude)))) {
    return false;
  }
  return check("apply PWM duty",
               ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void mixMoveCommand(const MOVE_CMD &command, int &left, int &right) {
  if (command.speed == 0 ||
      (command.throttle == 0 && command.steering == 0)) {
    left = 0;
    right = 0;
    return;
  }

  // Motor A is the left wheel and motor B is the right wheel. Opposing wheel
  // speeds pivot; weighted mixing keeps both wheels moving during a forward
  // or reverse turn while preserving the requested maximum PWM speed.
  left = static_cast<int>(command.throttle) * 2 +
         static_cast<int>(command.steering);
  right = static_cast<int>(command.throttle) * 2 -
          static_cast<int>(command.steering);
  const int leftMagnitude = left < 0 ? -left : left;
  const int rightMagnitude = right < 0 ? -right : right;
  const int maximumMagnitude = leftMagnitude > rightMagnitude
                                   ? leftMagnitude
                                   : rightMagnitude;
  if (maximumMagnitude > 1) {
    left = left * static_cast<int>(command.speed) / maximumMagnitude;
    right = right * static_cast<int>(command.speed) / maximumMagnitude;
  } else {
    left *= static_cast<int>(command.speed);
    right *= static_cast<int>(command.speed);
  }
}


bool handleMoveCommand(const MOVE_CMD &command) {
  if (command.throttle < -1 || command.throttle > 1 ||
      command.steering < -1 || command.steering > 1 ||
      movementCommands == nullptr) {
    return false;
  }
  const bool moving = command.speed != 0 &&
      (command.throttle != 0 || command.steering != 0);
  if (moving &&
      (!motorDriverReady.load() || !driveArmLeaseActive())) {
    return false;
  }
  // Manual-drive input takes precedence over queued PID starts/keepalives;
  // the motor task cancels its active PID state when it consumes this state.
  if (motionEvents != nullptr) xQueueReset(motionEvents);
  return xQueueOverwrite(movementCommands, &command) == pdPASS;
}

bool handlePidMotionCommand(const PID_MOTION_CMD &command) {
  if (motionEvents == nullptr || !motorDriverReady.load() ||
      !driveArmLeaseActive() ||
      !rover::control::validMotionRequest(command, getMotionPose())) return false;
  MotionEvent event{};
  event.kind = MOTION;
  event.motion = command;
  return xQueueSend(motionEvents, &event, 0) == pdPASS;
}


bool handleMotionStopCommand() {
  if (motionEvents == nullptr) return false;
  MotionEvent event{};
  event.kind = MOTION_STOP;
  if (xQueueSendToFront(motionEvents, &event, 0) == pdPASS) return true;
  xQueueReset(motionEvents);
  return xQueueSendToFront(motionEvents, &event, 0) == pdPASS;
}

bool handleMotionKeepalive(const MOTION_KEEPALIVE_CMD &command) {
  if (motionEvents == nullptr || command.commandId == 0) return false;
  MotionEvent event{};
  event.kind = MOTION_KEEPALIVE;
  event.keepalive = command;
  return xQueueSend(motionEvents, &event, 0) == pdPASS;
}

bool handleRecordingCommand(const RECORDING_CMD &command) {
  if (command.commandId == 0) return false;
  bool accepted = false;
  portENTER_CRITICAL(&recordingMux);
  switch (command.action) {
  case RECORDING_START:
    if (recordingState == 0 || recordingState == 1) {
      recordingState = 1;
      accepted = true;
    }
    break;
  case RECORDING_PAUSE:
    if (recordingState == 1) {
      recordingState = 2;
      accepted = true;
    }
    break;
  case RECORDING_RESUME:
    if (recordingState == 2) {
      recordingState = 1;
      accepted = true;
    }
    break;
  case RECORDING_STOP:
    recordingState = 0;
    accepted = true;
    break;
  case RECORDING_KEEPALIVE:
    if (recordingState == 1 || recordingState == 2) accepted = true;
    break;
  default:
    break;
  }
  if (accepted) recordingLastCommandTick = xTaskGetTickCount();
  portEXIT_CRITICAL(&recordingMux);
  return accepted;
}

void disarmDrive() {
  portENTER_CRITICAL(&driveArmMux);
  driveArmed = false;
  portEXIT_CRITICAL(&driveArmMux);
}

bool driveArmLeaseActive() {
  const TickType_t now = xTaskGetTickCount();
  const bool networkTaskFailed =
      (getSensorInitErrors() & SENSOR_ERROR_NETWORK_TASK) != 0;
  portENTER_CRITICAL(&driveArmMux);
  if (networkTaskFailed ||
      (driveArmed && now - driveArmLastCommandTick >=
                         DRIVE_ARM_LEASE_TIMEOUT_TICKS)) {
    driveArmed = false;
  }
  const bool active = driveArmed;
  portEXIT_CRITICAL(&driveArmMux);
  return active;
}

bool handleArmCommand(const ARM_CMD &command) {
  if (command.commandId == 0) return false;
  if (command.action == ARM_DISARM) {
    disarmDrive();
    return true;
  }
  if (command.action == ARM_REQUEST) {
    if (!motorDriverReady.load() || movementCommands == nullptr ||
        motionEvents == nullptr ||
        (getSensorInitErrors() & SENSOR_ERROR_NETWORK_TASK) != 0) {
      return false;
    }
    const TickType_t now = xTaskGetTickCount();
    portENTER_CRITICAL(&driveArmMux);
    driveArmed = true;
    driveArmLastCommandTick = now;
    portEXIT_CRITICAL(&driveArmMux);
    return true;
  }
  if (command.action == ARM_KEEPALIVE) {
    const TickType_t now = xTaskGetTickCount();
    const bool networkTaskFailed =
        (getSensorInitErrors() & SENSOR_ERROR_NETWORK_TASK) != 0;
    portENTER_CRITICAL(&driveArmMux);
    const bool accepted = driveArmed &&
        now - driveArmLastCommandTick < DRIVE_ARM_LEASE_TIMEOUT_TICKS &&
        motorDriverReady.load() && !networkTaskFailed;
    driveArmed = accepted;
    if (accepted) driveArmLastCommandTick = now;
    portEXIT_CRITICAL(&driveArmMux);
    return accepted;
  }
  return false;
}

static void motorControlTask(void *param) {
  (void)param;
  rover::control::MotionController controller;
  MOVE_CMD manual{};
  std::uint32_t manualUpdated = 0;
  TickType_t lastWakeTime = xTaskGetTickCount();
  bool headingHold = false;
  int headingHoldDirection = 0;
  float headingTarget = 0;
  while (true) {
    const auto now = static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
    auto pose = getMotionPose();
    MOVE_CMD next{};
    if (xQueueReceive(movementCommands, &next, 0) == pdTRUE) {
      controller.stop();
      if (next.throttle != manual.throttle || next.steering != manual.steering ||
          next.speed != manual.speed) headingHold = false;
      manual = next;
      manualUpdated = now;
    }
    MotionEvent event{};
    while (xQueueReceive(motionEvents, &event, 0) == pdTRUE) {
      if (event.kind == MOTION) {
        manual = {};
        headingHold = false;
        controller.start(event.motion, pose, now);
      } else if (event.kind == MOTION_STOP) {
        controller.stop();
        manual = {};
        headingHold = false;
      } else if (event.kind == MOTION_KEEPALIVE) {
        controller.keepalive(event.keepalive.commandId, now);
      }
    }
    if (!motorDriverReady.load()) {
      disarmDrive();
      controller.stop(MOTION_RESULT_MOTOR_FAULT);
      manual = {};
    }
    if (!driveArmLeaseActive()) {
      controller.stop(MOTION_RESULT_TIMEOUT);
      manual = {};
      headingHold = false;
    }
    if (now - manualUpdated >= 750U) {
      manual = {};
      headingHold = false;
    }
    rover::control::MotorOutput output{};
    if (controller.active()) {
      output = controller.update(pose, now);
    } else if (manual.speed != 0 && (manual.throttle != 0 || manual.steering != 0)) {
      mixMoveCommand(manual, output.left, output.right);
      if (manual.steering == 0 && manual.throttle != 0 && pose.headingValid) {
        if (!headingHold || headingHoldDirection != manual.throttle) {
          headingTarget = pose.pose.headingRadians;
          headingHold = true;
          headingHoldDirection = manual.throttle;
        }
        const float maximum = static_cast<float>(manual.speed);
        const float correction = std::clamp(
            wrapHeadingRadians(headingTarget - pose.pose.headingRadians) * 180.0f / pi * 2.0f,
            -maximum * 0.35f, maximum * 0.35f);
        output.left = static_cast<int>(std::lround(std::clamp(
            static_cast<float>(output.left) + correction, -maximum, maximum)));
        output.right = static_cast<int>(std::lround(std::clamp(
            static_cast<float>(output.right) - correction, -maximum, maximum)));
      } else {
        headingHold = false;
      }
    }
    if (motorDriverReady.load() && driveArmLeaseActive() &&
        (output.left != 0 || output.right != 0)) {
      if (setMotorSpeed(output.left, AIN1_PIN, AIN2_PIN, LEDC_CHANNEL_0, true))
        (void)setMotorSpeed(output.right, BIN1_PIN, BIN2_PIN, LEDC_CHANNEL_1, false);
    } else {
      (void)motor_stop();
    }
    portENTER_CRITICAL(&poseMux);
    reportedMotionCommandId = controller.commandId();
    reportedMotionActive = controller.active();
    reportedMotionResult = controller.result();
    portEXIT_CRITICAL(&poseMux);
    constexpr TickType_t period = pdMS_TO_TICKS(20) > 0 ? pdMS_TO_TICKS(20) : 1;
    if (static_cast<TickType_t>(xTaskGetTickCount() - lastWakeTime) > period)
      lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, period);
  }
}


void broadcastStatus(void* param) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  UDPServer* server = (UDPServer*)param;
  while (true) {
    ESP_STATUS status{};
    status.batteryP = max17048_get_percent();
    status.batteryV = max17048_get_voltage();
    setSensorHealthFlag(SENSOR_BATTERY_READY,
                        status.batteryP >= 0.0f && status.batteryV >= 0.0f);
    const auto pose = getMotionPose();
    status.xCentimeters = pose.pose.xCm;
    status.yCentimeters = pose.pose.yCm;
    float headingDegrees = pose.pose.headingRadians * 180.0f / pi;
    if (headingDegrees < 0.0f) headingDegrees += 360.0f;
    status.headingDegreesClockwise = headingDegrees;
    portENTER_CRITICAL(&poseMux);
    status.motionCommandId = reportedMotionCommandId;
    status.motionActive = reportedMotionActive ? 1 : 0;
    status.motionResult = reportedMotionResult;
    portEXIT_CRITICAL(&poseMux);
    status.uptimeMilliseconds = static_cast<std::uint32_t>(
        esp_timer_get_time() / 1000);
    status.imuYawDirection = IMU_YAW_DIRECTION;
    portENTER_CRITICAL(&imuYawMux);
    status.imuGyroBiasDegreesPerSecond = imuGyroBiasDegreesPerSecond;
    portEXIT_CRITICAL(&imuYawMux);
    status.sensorHealthFlags = getSensorHealthFlags();
    status.sensorInitErrorFlags = getSensorInitErrors();
    status.recordingState = getRecordingState();
    status.streamDroppedPackets = server->getDroppedPacketCount();
    if (!server->getLocalAddress(status.ip, sizeof(status.ip), status.port)) {
      status.port = server->port;
      snprintf(status.ip, sizeof(status.ip), "N/A");
    }
    char destinationIp[IP_LEN]{};
    unsigned short destinationPort = 0;
    if (server->getStatusDestination(destinationIp, sizeof(destinationIp),
                                    destinationPort)) {
      server->send(Packet(STATUS, ESP, status), destinationIp,
                   destinationPort);
      THERMAL_CAPTURE_DIAGNOSTICS diagnostics;
      portENTER_CRITICAL(&thermalDiagnosticsMux);
      diagnostics = thermalDiagnostics;
      portEXIT_CRITICAL(&thermalDiagnosticsMux);
      diagnostics.timestampMilliseconds = status.uptimeMilliseconds;
      server->send(Packet(STATUS, THERMAL_DIAGNOSTICS, diagnostics),
                   destinationIp, destinationPort);
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
  }
}

extern "C" void app_main(void) {
  nvsFlashInit();
  if (motor_init()) {
    setSensorHealthFlag(SENSOR_MOTORS_READY, true);
  } else {
    setSensorHealthFlag(SENSOR_MOTORS_READY, false);
    setSensorInitError(SENSOR_ERROR_MOTOR_INIT);
  }
  motor_stop();
  movementCommands = xQueueCreate(1, sizeof(MOVE_CMD));
  motionEvents = xQueueCreate(8, sizeof(MotionEvent));
  const bool motorQueuesReady =
      movementCommands != nullptr && motionEvents != nullptr;
  if (!motorQueuesReady) {
    disableMotorForControlFailure("motor command queue creation");
  }

  initializeSharedI2CBus();

  lidarUartReady = initializeLidarUart();

  UDPServer server;
  if (!server.healthy()) {
    disarmDrive();
    motorDriverReady.store(false);
    setSensorHealthFlag(SENSOR_MOTORS_READY, false);
    (void)gpio_set_level(STBY_PIN, 0);
    return;
  }
  sensorUdpServer = &server;

  if (mpu6050Device != nullptr &&
      xTaskCreatePinnedToCore(mpu6050SamplingTask, "MPU6050_SAMPLE_LOOP", 8192,
                              nullptr, 4, nullptr, 1) != pdPASS) {
    ESP_LOGE(WIFI_TAG, "Could not start MPU-6050 sampling task");
    setSensorHealthFlag(SENSOR_MPU6050_STREAMING, false);
    setSensorHealthFlag(SENSOR_MPU6050_CALIBRATED, false);
    setSensorInitError(SENSOR_ERROR_MPU6050_TASK);
  }

  // Scan matching runs in the higher-priority LD19 reader on core 1. Keep
  // thermal acquisition/conversion on core 0 so that backlog cannot starve
  // it or repeatedly skip the complementary thermal subpage. Wi-Fi and status
  // retain their higher priorities; the I2C master serializes bus transfers.
  if (mlx90640Ready &&
      xTaskCreatePinnedToCore(mlx90640AcquisitionTask, "MLX90640_CAPTURE",
                              12288, nullptr, 2, nullptr, 0) != pdPASS) {
    ESP_LOGE(WIFI_TAG, "Could not start MLX90640 capture task");
    setSensorHealthFlag(SENSOR_MLX90640_STREAMING, false);
    setSensorInitError(SENSOR_ERROR_MLX90640_TASK);
  }

  if (lidarUartReady &&
      xTaskCreatePinnedToCore(lidarReaderTask, "LD19_UART_READER", 12288,
                              nullptr, 4, nullptr, 1) != pdPASS) {
    ESP_LOGE(WIFI_TAG, "Could not start LD19 UART reader task");
    lidarUartReady = false;
    setSensorHealthFlag(SENSOR_LD19_STREAMING, false);
    setSensorInitError(SENSOR_ERROR_LD19_TASK);
  }

  if (motorQueuesReady &&
      xTaskCreatePinnedToCore(motorControlTask, "MOTOR_CONTROL_LOOP", 8192,
                              nullptr, 6, nullptr, 1) != pdPASS) {
    disableMotorForControlFailure("motor control task creation");
  }

  if (xTaskCreatePinnedToCore(UDPServer::taskListen, "NETWORK_LISTEN_LOOP", 8192,
                              &server, 5, nullptr, 1) != pdPASS) {
    setSensorInitError(SENSOR_ERROR_NETWORK_TASK);
    ESP_LOGE(WIFI_TAG, "Could not start UDP receive task");
  }
  if (xTaskCreatePinnedToCore(UDPServer::taskSend, "NETWORK_SEND_LOOP", 8192,
                              &server, 5, nullptr, 1) != pdPASS) {
    setSensorInitError(SENSOR_ERROR_NETWORK_TASK);
    ESP_LOGE(WIFI_TAG, "Could not start UDP send task");
  }
  if (xTaskCreatePinnedToCore(broadcastStatus, "BROADCAST_STATUS_LOOP", 8192,
                              &server, 5, nullptr, 0) != pdPASS) {
    setSensorInitError(SENSOR_ERROR_NETWORK_TASK);
    ESP_LOGE(WIFI_TAG, "Could not start rover status task");
  }

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
