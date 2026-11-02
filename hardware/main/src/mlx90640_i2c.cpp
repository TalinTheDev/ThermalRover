#include "mlx90640_i2c.hpp"

extern "C" {
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
}

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace {
constexpr std::uint8_t mlx90640Address = 0x33;
constexpr std::size_t maxReadWords = 64;
i2c_master_dev_handle_t mlx90640I2CDevice = nullptr;

int mapI2CError(esp_err_t error) {
  return error == ESP_OK ? MLX90640_NO_ERROR : -MLX90640_I2C_NACK_ERROR;
}
} // namespace

void setMLX90640I2CDevice(i2c_master_dev_handle_t device) {
  mlx90640I2CDevice = device;
}

extern "C" void MLX90640_I2CInit(void) {}

extern "C" int MLX90640_I2CGeneralReset(void) {
  // The normal frame acquisition path does not use a general call reset.
  return MLX90640_NO_ERROR;
}

extern "C" int MLX90640_I2CRead(std::uint8_t slaveAddress,
                                 std::uint16_t startAddress,
                                 std::uint16_t wordCount,
                                 std::uint16_t *data) {
  if (slaveAddress != mlx90640Address || mlx90640I2CDevice == nullptr ||
      data == nullptr || wordCount == 0) {
    return -MLX90640_I2C_NACK_ERROR;
  }

  std::array<std::uint8_t, maxReadWords * 2> bytes{};
  std::uint16_t wordsRead = 0;
  while (wordsRead < wordCount) {
    const std::size_t batchWords = std::min<std::size_t>(
        maxReadWords, static_cast<std::size_t>(wordCount - wordsRead));
    const std::uint16_t address = static_cast<std::uint16_t>(
        startAddress + wordsRead);
    const std::uint8_t registerAddress[]{
        static_cast<std::uint8_t>(address >> 8U),
        static_cast<std::uint8_t>(address & 0xFFU)};
    const esp_err_t error = i2c_master_transmit_receive(
        mlx90640I2CDevice, registerAddress, sizeof(registerAddress),
        bytes.data(), batchWords * 2, 100);
    if (error != ESP_OK) return mapI2CError(error);

    for (std::size_t index = 0; index < batchWords; ++index) {
      data[wordsRead + index] = static_cast<std::uint16_t>(
          (static_cast<std::uint16_t>(bytes[index * 2]) << 8U) |
          static_cast<std::uint16_t>(bytes[index * 2 + 1]));
    }
    wordsRead = static_cast<std::uint16_t>(wordsRead + batchWords);
  }
  return MLX90640_NO_ERROR;
}

extern "C" int MLX90640_I2CWrite(std::uint8_t slaveAddress,
                                  std::uint16_t writeAddress,
                                  std::uint16_t value) {
  if (slaveAddress != mlx90640Address || mlx90640I2CDevice == nullptr) {
    return -MLX90640_I2C_NACK_ERROR;
  }
  const std::uint8_t bytes[]{
      static_cast<std::uint8_t>(writeAddress >> 8U),
      static_cast<std::uint8_t>(writeAddress & 0xFFU),
      static_cast<std::uint8_t>(value >> 8U),
      static_cast<std::uint8_t>(value & 0xFFU)};
  return mapI2CError(i2c_master_transmit(mlx90640I2CDevice, bytes,
                                         sizeof(bytes), 100));
}

extern "C" void MLX90640_I2CFreqSet(int frequency) {
  (void)frequency;
}
