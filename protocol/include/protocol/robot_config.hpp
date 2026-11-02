#pragma once

#include <cstdint>

// Wheel values below are retained only for historical session metadata.
// Current firmware and mapping use LiDAR/IMU fusion and never read encoders.
namespace rover::config {

inline constexpr float wheelDiameterCentimeters = 3.2f;
inline constexpr float wheelTrackCentimeters = 13.3f;

// SparkFun N20 datasheet: 7 pulses per motor revolution and a 1:30 gearbox.
// Historical firmware counted rising A edges with B direction. This legacy
// metadata value is never used by the current firmware or mapper.
inline constexpr float encoderCountsPerWheelRevolution = 210.0f;
// Reject implausible lifetime totals before subtracting successive encoder
// samples, keeping corrupted telemetry from overflowing signed deltas.
inline constexpr std::int64_t maxEncoderTotalTicks = 1000000000000LL;

// Historical raw-count polarity, retained for older metadata readers.
inline constexpr int leftEncoderForwardSign = 1;
inline constexpr int rightEncoderForwardSign = -1;
inline constexpr float imuYawDirection = -1.0f; // Provisional: board Z points up.
inline constexpr float lidarImuGyroPriorSigmaRadians = 0.015f;
// User confirmed LD19 zero points toward the rover's right, not forward.
inline constexpr float lidarYawRadiansClockwise = 1.5707963267948966f;

// User-confirmed wiring: LD19 TX connects to Metro RX/D0, GPIO41.
// UART1 is routed to that pin through the GPIO matrix. GPIO14/A0 holds
// the sensor PWM/control input low for internal-speed mode.
inline constexpr int lidarRxGpio = 41;
inline constexpr int lidarPwmGpio = 14;
inline constexpr int lidarBaudRate = 230400;

} // namespace rover::config
