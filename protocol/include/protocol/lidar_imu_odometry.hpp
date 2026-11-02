#pragma once
#include "protocol/robot_config.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace rover::odometry {
// Centimeters, +X right, +Y forward, clockwise radians. No wheel inputs.
struct Pose { float xCm = 0, yCm = 0, headingRadians = 0; };
struct Point { float x = 0, y = 0; };
inline constexpr std::size_t scanBins = 360;
using Scan = std::array<Point, scanBins>;
enum class Quality : std::uint8_t {
    Initializing, Tracking, WeakGeometry, RejectedMatch, ImuUnavailable
};
struct Config {
    float lidarXcm = 0, lidarYcm = 0, lidarYawRadians = rover::config::lidarYawRadiansClockwise;
    float minimumRangeCm = 8, maximumRangeCm = 800;
    float gyroPriorSigmaRadians = rover::config::lidarImuGyroPriorSigmaRadians;
    bool requireImu = true; // Onboard measured motion keeps its IMU safety requirement.
};
struct Estimate {
    Pose pose{};
    std::uint32_t timestampMilliseconds = 0;
    float residualCm = 0;
    std::uint16_t matchedPoints = 0;
    std::uint32_t resets = 0;
    Quality quality = Quality::Initializing;
    bool translationValid = false, headingValid = false;
};
float wrap(float radians);
void addPolarPoint(Scan& scan, float angleRadians, float distanceCm, const Config& config = {});
// Bounded-memory point-to-line ICP with a gyro yaw prior. Weak geometric
// constraints reject translation rather than inventing motion in a corridor.
// The same implementation runs on the MCU and in desktop mapping/replay.
class LidarImuOdometry {
public:
    explicit LidarImuOdometry(Config config = {});
    void reset(Pose origin = {});
    const Estimate& update(const Scan& scan, std::uint32_t timestampMilliseconds,
                           float imuYawRadians, bool imuValid);
    const Estimate& estimate() const { return estimate_; }
private:
    Config config_;
    Estimate estimate_{};
    Scan reference_{};
    Pose referencePose_{};
    std::uint32_t referenceTime_ = 0;
    float previousImuYaw_ = 0;
    bool referenceSet_ = false, imuBaselineSet_ = false;
};
} // namespace rover::odometry
