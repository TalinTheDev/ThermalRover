#pragma once

#include "protocol/robot_config.hpp"
#include "protocol/lidar_imu_odometry.hpp"
#include "mapping/thermal_scale.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

namespace mapping {

inline constexpr std::size_t thermalWidth = 32;
inline constexpr std::size_t thermalHeight = 24;
inline constexpr std::size_t thermalPixelCount = thermalWidth * thermalHeight;

struct Pose2D {
    float xCm = 0.0f;
    float yCm = 0.0f;
    float headingRadiansClockwise = 0.0f;
};

struct LidarPoint {
    float angleRadiansClockwise = 0.0f; // Clockwise from robot forward.
    float distanceCm = 0.0f;
    std::uint8_t intensity = 0;
    std::uint32_t timestampMilliseconds = 0;
    bool hasTimestamp = false;
    float headingOffsetRadians = 0;
    bool hasHeadingOffset = false;
};

struct LidarScan {
    std::vector<LidarPoint> points;
    std::uint64_t timestampMilliseconds = 0;
};

struct ThermalFrame {
    std::array<float, thermalPixelCount> temperaturesC{};
    std::uint64_t timestampMilliseconds = 0;
    bool valid = false;
};

struct SensorFrame {
    LidarScan lidar;
    ThermalFrame thermal;
    std::int64_t leftEncoderTicks = 0;
    std::int64_t rightEncoderTicks = 0;
    bool hasEncoderCounts = false;
    float imuHeadingRadiansClockwise = 0.0f;
    bool hasImuHeading = false;
    Pose2D fusedPose{};
    bool hasFusedPose = false;
    bool fusedOdometrySource = false;
};

struct DifferentialDriveConfig {
    // Legacy session metadata only. Live odometry never uses wheel inputs.
    float wheelDiameterCm = rover::config::wheelDiameterCentimeters;
    float wheelTrackCm = rover::config::wheelTrackCentimeters;
    // Historical wheel parameters for old metadata readers. Not an input to
    // either current odometry implementation.
    float ticksPerWheelRevolution =
        rover::config::encoderCountsPerWheelRevolution;
    int leftEncoderDirection = rover::config::leftEncoderForwardSign;
    int rightEncoderDirection = rover::config::rightEncoderForwardSign;
};

struct SensorMount {
    // Robot frame: +X right, +Y forward; heading is positive clockwise.
    float xCm = 0.0f;
    float yCm = 0.0f;
    float zCm = 0.0f; // Stored for calibration; the current map is planar.
    float headingRadiansClockwise = 0.0f;
};

struct SensorMounts {
    SensorMount lidar{0.0f, 0.0f, 12.0f, rover::config::lidarYawRadiansClockwise};
    SensorMount thermal{0.0f, 0.0f, 8.0f, -1.57079632679f}; // Faces left, below LiDAR.
    float thermalHorizontalFovDegrees = 110.0f; // MLX90640 BAA; set 55 for BAB.
    float thermalVerticalFovDegrees = 75.0f; // BAA/BAB metadata; current map uses center rows.
    bool mirrorThermalColumns = false;
    std::uint8_t thermalRotationQuarterTurns = 0;
};

struct SlamConfig {
    std::size_t mapWidthCells = 512;
    std::size_t mapHeightCells = 512;
    float centimetersPerCell = 4.0f;
    float displayMapFraction = 0.68f; // Fraction of the mapped world shown at once.
    float lidarMinRangeCm = 8.0f;
    float lidarMaxRangeCm = 800.0f;
    DifferentialDriveConfig drive{};
    SensorMounts mounts{};
    Pose2D initialPose{};
    float gyroYawPriorSigmaRadians = rover::config::lidarImuGyroPriorSigmaRadians;
    bool enableScanMatching = true;
};

class SlamMapper {
public:
    explicit SlamMapper(SlamConfig config = {});

    void reset(Pose2D initialPose = {}, bool keepThermalStatistics = false);
    bool integrate(const SensorFrame& frame);

    const SlamConfig& config() const { return config_; }
    Pose2D pose() const { return pose_; }
    std::size_t scanCount() const { return scanCount_; }
    std::uint64_t revision() const { return revision_; }
    bool hasMap() const { return scanCount_ > 0; }
    const rover::odometry::Estimate& odometry() const { return lidarImuOdometry_.estimate(); }

    std::span<const std::int8_t> occupancy() const { return occupancy_; }
    std::span<const std::uint8_t> observed() const { return observed_; }
    std::span<const float> thermalAverageC() const { return thermalAverageC_; }
    std::span<const std::uint16_t> thermalSampleCount() const { return thermalSampleCount_; }
    ThermalScale thermalScale() const { return thermalStatistics_.scale(); }
    void observeThermalFrame(const ThermalFrame& frame);
    float localizationScore() const { return lastMatchScore_; }
    bool localizationTracking() const { return scanCount_ != 0 && lastMatchAccepted_; }
    std::uint64_t rejectedScanCount() const { return rejectedScanCount_; }

private:
    SlamConfig config_;
    Pose2D pose_{};
    std::vector<std::int8_t> occupancy_;
    std::vector<std::uint8_t> observed_;
    std::vector<float> thermalAverageC_;
    std::vector<std::uint16_t> thermalSampleCount_;
    ThermalStatistics thermalStatistics_;
    std::deque<std::uint32_t> recentThermalFrames_;
    std::deque<ThermalFrame> pendingThermalFrames_;
    struct LocalizedScan { LidarScan scan; Pose2D pose; };
    std::deque<LocalizedScan> scanHistory_;
    rover::odometry::LidarImuOdometry lidarImuOdometry_;
    Pose2D previousFusedPose_{};
    bool fusedPoseBaselineSet_ = false;
    std::size_t scanCount_ = 0;
    std::uint64_t revision_ = 0;
    bool containsOccupiedCells_ = false;
    std::vector<float> matchingField_;
    Pose2D previousLocalPose_{}, mapVelocity_{};
    std::uint32_t lastLocalizedTime_ = 0;
    bool hasLocalizedTime_ = false, previousLocalTracking_ = false, previousFusedTracking_ = false;
    std::uint64_t rejectedScanCount_ = 0;
    unsigned consecutiveRejectedScans_ = 0;
    float lastMatchScore_ = 0;
    bool lastMatchAccepted_ = false;

    Pose2D estimateOdometry(const SensorFrame& frame);
    struct ScanMatch { Pose2D pose; float score; bool accepted; };
    ScanMatch matchScan(const LidarScan& scan, Pose2D prior, float elapsedSeconds);
    LidarScan deskewScan(const LidarScan& scan) const;
    bool worldToCell(float xCm, float yCm, int& cellX, int& cellY) const;
    std::size_t cellIndex(int cellX, int cellY) const;
    void integrateRay(const LidarPoint& point, Pose2D pose);
    void projectPendingThermalFrames();
    void projectThermalFrame(const ThermalFrame& frame, const LocalizedScan& geometry,
                             Pose2D capturePose);
};

float wrapAngle(float radiansClockwise);
std::size_t thermalImageWidth(const SensorMounts& mounts);
std::size_t thermalImageHeight(const SensorMounts& mounts);
float thermalHorizontalFov(const SensorMounts& mounts);
std::size_t thermalImageRawIndex(const SensorMounts& mounts, std::size_t column, std::size_t row);
float thermalImagePixel(const ThermalFrame& frame, const SensorMounts& mounts,
                        std::size_t column, std::size_t row);

} // namespace mapping
