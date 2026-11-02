#include "mapping/slam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace mapping {
namespace {
constexpr float pi = std::numbers::pi_v<float>;
constexpr float radiansPerDegree = pi / 180.0f;
constexpr int occupiedThreshold = 3;
constexpr int freeUpdate = -1;
constexpr int occupiedUpdate = 5;
constexpr int minimumLogOdds = -25;
constexpr int maximumLogOdds = 60;

rover::odometry::Config odometryConfig(const SlamConfig& config) {
    return {config.mounts.lidar.xCm, config.mounts.lidar.yCm,
            config.mounts.lidar.headingRadiansClockwise,
            config.lidarMinRangeCm, config.lidarMaxRangeCm, config.gyroYawPriorSigmaRadians, false};
}

Pose2D sensorOrigin(Pose2D pose, const SensorMount& mount) {
    const float c = std::cos(pose.headingRadiansClockwise);
    const float s = std::sin(pose.headingRadiansClockwise);
    return {pose.xCm + c * mount.xCm + s * mount.yCm,
            pose.yCm - s * mount.xCm + c * mount.yCm,
            wrapAngle(pose.headingRadiansClockwise + mount.headingRadiansClockwise)};
}

float medianCenterTemperature(const ThermalFrame& frame, std::size_t column,
                              const SensorMounts& mounts) {
    const std::size_t firstRow = thermalImageHeight(mounts) / 2 - 3;
    constexpr std::size_t rowCount = 6;
    std::array<float, rowCount> values{};
    for(std::size_t row = 0; row < rowCount; ++row) {
        values[row] = thermalImagePixel(frame, mounts, column, firstRow + row);
    }
    std::sort(values.begin(), values.end());
    return (values[rowCount / 2 - 1] + values[rowCount / 2]) * 0.5f;
}
} // namespace

std::size_t thermalImageWidth(const SensorMounts& mounts) {
    return mounts.thermalRotationQuarterTurns % 2 == 0 ? thermalWidth : thermalHeight;
}
std::size_t thermalImageHeight(const SensorMounts& mounts) {
    return mounts.thermalRotationQuarterTurns % 2 == 0 ? thermalHeight : thermalWidth;
}
float thermalHorizontalFov(const SensorMounts& mounts) {
    return mounts.thermalRotationQuarterTurns % 2 == 0
        ? mounts.thermalHorizontalFovDegrees : mounts.thermalVerticalFovDegrees;
}
std::size_t thermalImageRawIndex(const SensorMounts& mounts, std::size_t column, std::size_t row) {
    const auto width = thermalImageWidth(mounts), height = thermalImageHeight(mounts);
    if(column >= width || row >= height) return thermalPixelCount;
    if(mounts.mirrorThermalColumns) column = width - 1 - column;
    std::size_t x = column, y = row;
    switch(mounts.thermalRotationQuarterTurns % 4) {
    case 1: x = row; y = thermalHeight - 1 - column; break;
    case 2: x = thermalWidth - 1 - column; y = thermalHeight - 1 - row; break;
    case 3: x = thermalWidth - 1 - row; y = column; break;
    default: break;
    }
    return y * thermalWidth + x;
}
float thermalImagePixel(const ThermalFrame& frame, const SensorMounts& mounts,
                        std::size_t column, std::size_t row) {
    const auto index = thermalImageRawIndex(mounts, column, row);
    return index < thermalPixelCount ? frame.temperaturesC[index] : std::numeric_limits<float>::quiet_NaN();
}

float wrapAngle(float radiansClockwise) {
    if(!std::isfinite(radiansClockwise)) return 0.0f;
    return std::remainder(radiansClockwise, 2.0f * pi);
}

SlamMapper::SlamMapper(SlamConfig config) : config_(config) {
    config_.mapWidthCells = std::max<std::size_t>(config_.mapWidthCells, 16);
    config_.mapHeightCells = std::max<std::size_t>(config_.mapHeightCells, 16);
    if(!std::isfinite(config_.centimetersPerCell)) config_.centimetersPerCell = 4.0f;
    config_.centimetersPerCell = std::max(config_.centimetersPerCell, 0.5f);
    if(!std::isfinite(config_.displayMapFraction)) config_.displayMapFraction = 0.68f;
    config_.displayMapFraction = std::clamp(config_.displayMapFraction, 0.05f, 1.0f);
    if(!std::isfinite(config_.gyroYawPriorSigmaRadians)) {
        config_.gyroYawPriorSigmaRadians = rover::config::lidarImuGyroPriorSigmaRadians;
    }
    config_.gyroYawPriorSigmaRadians = std::clamp(config_.gyroYawPriorSigmaRadians, 0.005f, 0.05f);
    const std::size_t cellCount = config_.mapWidthCells * config_.mapHeightCells;
    occupancy_.resize(cellCount);
    observed_.resize(cellCount);
    thermalAverageC_.resize(cellCount);
    thermalSampleCount_.resize(cellCount);
    lidarImuOdometry_ = rover::odometry::LidarImuOdometry(odometryConfig(config_));
    reset(config_.initialPose);
}

void SlamMapper::reset(Pose2D initialPose, bool keepThermalStatistics) {
    std::fill(occupancy_.begin(), occupancy_.end(), std::int8_t{0});
    std::fill(observed_.begin(), observed_.end(), std::uint8_t{0});
    std::fill(thermalAverageC_.begin(), thermalAverageC_.end(), 0.0f);
    std::fill(thermalSampleCount_.begin(), thermalSampleCount_.end(), std::uint16_t{0});
    if(!keepThermalStatistics) {
        thermalStatistics_.reset();
        recentThermalFrames_.clear();
    }
    pendingThermalFrames_.clear();
    scanHistory_.clear();
    previousLocalPose_ = {};
    mapVelocity_ = {};
    hasLocalizedTime_ = previousLocalTracking_ = previousFusedTracking_ = false;
    rejectedScanCount_ = consecutiveRejectedScans_ = 0;
    lastMatchScore_ = 0;
    lastMatchAccepted_ = false;
    pose_ = initialPose;
    lidarImuOdometry_.reset({initialPose.xCm, initialPose.yCm,
                             initialPose.headingRadiansClockwise});
    fusedPoseBaselineSet_ = false;
    scanCount_ = 0;
    ++revision_;
    containsOccupiedCells_ = false;
}

bool SlamMapper::integrate(const SensorFrame& input) {
    observeThermalFrame(input.thermal);
    if(input.lidar.points.empty()) return false;
    if(!config_.enableScanMatching && input.fusedOdometrySource && !input.hasFusedPose) return false;
    SensorFrame frame = input;
    frame.lidar = deskewScan(input.lidar);
    const auto timestamp = static_cast<std::uint32_t>(frame.lidar.timestampMilliseconds);
    const float elapsed = hasLocalizedTime_ ? static_cast<float>(timestamp - lastLocalizedTime_) * 0.001f : 0.1f;
    const Pose2D odometryPose = estimateOdometry(frame);
    Pose2D nextPose = odometryPose;
    if(config_.enableScanMatching && hasMap()) {
        const auto match = matchScan(frame.lidar, odometryPose, elapsed);
        lastMatchScore_ = match.score;
        lastMatchAccepted_ = match.accepted;
        if(!match.accepted) {
            ++rejectedScanCount_;
            ++consecutiveRejectedScans_;
            return false;
        }
        nextPose = match.pose;
    } else if(!config_.enableScanMatching && !frame.hasFusedPose && scanCount_ != 0 &&
              !lidarImuOdometry_.estimate().translationValid) return false;
    else lastMatchAccepted_ = true;
    if(hasLocalizedTime_ && elapsed > 0.01f && elapsed <= 0.3f) {
        const float vx = (nextPose.xCm - pose_.xCm) / elapsed;
        const float vy = (nextPose.yCm - pose_.yCm) / elapsed;
        const float angular = wrapAngle(nextPose.headingRadiansClockwise - pose_.headingRadiansClockwise) / elapsed;
        mapVelocity_ = {std::clamp(vx, -200.0f, 200.0f), std::clamp(vy, -200.0f, 200.0f),
                        std::clamp(angular, -6.0f, 6.0f)};
    } else mapVelocity_ = {};
    pose_ = nextPose;
    lastLocalizedTime_ = timestamp;
    hasLocalizedTime_ = true;
    consecutiveRejectedScans_ = 0;

    for(const LidarPoint& point : frame.lidar.points) {
        if(!std::isfinite(point.distanceCm) ||
           !std::isfinite(point.angleRadiansClockwise) ||
           point.distanceCm < config_.lidarMinRangeCm ||
           point.distanceCm > config_.lidarMaxRangeCm) {
            continue;
        }
        integrateRay(point, pose_);
    }

    scanHistory_.push_back({frame.lidar, pose_});
    while(scanHistory_.size() > 16) scanHistory_.pop_front();
    projectPendingThermalFrames();

    ++scanCount_;
    ++revision_;
    return true;
}

bool SlamMapper::worldToCell(float xCm, float yCm, int& cellX, int& cellY) const {
    const float halfWidth = static_cast<float>(config_.mapWidthCells) *
                            config_.centimetersPerCell * 0.5f;
    const float halfHeight = static_cast<float>(config_.mapHeightCells) *
                             config_.centimetersPerCell * 0.5f;
    const float cellXFloat = xCm / config_.centimetersPerCell +
                             static_cast<float>(config_.mapWidthCells) * 0.5f;
    const float cellYFloat = yCm / config_.centimetersPerCell +
                             static_cast<float>(config_.mapHeightCells) * 0.5f;
    if(xCm < -halfWidth || xCm >= halfWidth ||
       yCm < -halfHeight || yCm >= halfHeight) {
        return false;
    }
    cellX = static_cast<int>(cellXFloat);
    cellY = static_cast<int>(cellYFloat);
    return cellX >= 0 && cellY >= 0 &&
           cellX < static_cast<int>(config_.mapWidthCells) &&
           cellY < static_cast<int>(config_.mapHeightCells);
}

std::size_t SlamMapper::cellIndex(int cellX, int cellY) const {
    return static_cast<std::size_t>(cellY) * config_.mapWidthCells +
           static_cast<std::size_t>(cellX);
}

void SlamMapper::integrateRay(const LidarPoint& point, Pose2D pose) {
    const Pose2D lidarPose = sensorOrigin(pose, config_.mounts.lidar);
    const float angle = wrapAngle(lidarPose.headingRadiansClockwise +
                                  point.angleRadiansClockwise);
    const float endX = lidarPose.xCm + std::sin(angle) * point.distanceCm;
    const float endY = lidarPose.yCm + std::cos(angle) * point.distanceCm;
    int startCellX = 0, startCellY = 0, endCellX = 0, endCellY = 0;
    if(!worldToCell(lidarPose.xCm, lidarPose.yCm, startCellX, startCellY) ||
       !worldToCell(endX, endY, endCellX, endCellY)) return;

    int x = startCellX;
    int y = startCellY;
    const int dx = std::abs(endCellX - startCellX);
    const int sx = startCellX < endCellX ? 1 : -1;
    const int dy = -std::abs(endCellY - startCellY);
    const int sy = startCellY < endCellY ? 1 : -1;
    int error = dx + dy;
    while(x != endCellX || y != endCellY) {
        const std::size_t index = cellIndex(x, y);
        observed_[index] = 1;
        occupancy_[index] = static_cast<std::int8_t>(std::max(
            static_cast<int>(occupancy_[index]) + freeUpdate, minimumLogOdds));
        const int twiceError = 2 * error;
        if(twiceError >= dy) {
            error += dy;
            x += sx;
        }
        if(twiceError <= dx) {
            error += dx;
            y += sy;
        }
    }

    const std::size_t endIndex = cellIndex(endCellX, endCellY);
    observed_[endIndex] = 1;
    occupancy_[endIndex] = static_cast<std::int8_t>(std::min(
        static_cast<int>(occupancy_[endIndex]) + occupiedUpdate, maximumLogOdds));
    containsOccupiedCells_ = true;

}

void SlamMapper::observeThermalFrame(const ThermalFrame& frame) {
    const auto timestamp = static_cast<std::uint32_t>(frame.timestampMilliseconds);
    if(!frame.valid || std::find(recentThermalFrames_.begin(), recentThermalFrames_.end(), timestamp) !=
                      recentThermalFrames_.end()) return;
    recentThermalFrames_.push_back(timestamp);
    while(recentThermalFrames_.size() > 256) recentThermalFrames_.pop_front();
    thermalStatistics_.observe(frame.temperaturesC);
    pendingThermalFrames_.push_back(frame);
    while(pendingThermalFrames_.size() > 8) pendingThermalFrames_.pop_front();
    projectPendingThermalFrames();
    ++revision_;
}

void SlamMapper::projectPendingThermalFrames() {
    if(scanHistory_.empty()) return;
    for(auto pending = pendingThermalFrames_.begin(); pending != pendingThermalFrames_.end();) {
        const auto timestamp = static_cast<std::uint32_t>(pending->timestampMilliseconds);
        const LocalizedScan *before = nullptr, *after = nullptr;
        std::uint32_t beforeAge = 0xffffffffU, afterAge = 0xffffffffU;
        for(const auto& scan : scanHistory_) {
            const auto scanTime = static_cast<std::uint32_t>(scan.scan.timestampMilliseconds);
            const auto past = timestamp - scanTime, future = scanTime - timestamp;
            if(past <= 0x7fffffffU && past < beforeAge) { before = &scan; beforeAge = past; }
            if(future <= 0x7fffffffU && future < afterAge) { after = &scan; afterAge = future; }
        }
        if(after == nullptr) { ++pending; continue; }
        if(before != nullptr && beforeAge + afterAge <= 250U && std::min(beforeAge, afterAge) <= 120U) {
            const float fraction = beforeAge + afterAge == 0 ? 0.0f
                : static_cast<float>(beforeAge) / static_cast<float>(beforeAge + afterAge);
            Pose2D capturePose{
                before->pose.xCm + (after->pose.xCm - before->pose.xCm) * fraction,
                before->pose.yCm + (after->pose.yCm - before->pose.yCm) * fraction,
                wrapAngle(before->pose.headingRadiansClockwise + fraction * wrapAngle(
                    after->pose.headingRadiansClockwise - before->pose.headingRadiansClockwise))};
            projectThermalFrame(*pending, beforeAge <= afterAge ? *before : *after, capturePose);
        }
        pending = pendingThermalFrames_.erase(pending);
    }
}

void SlamMapper::projectThermalFrame(const ThermalFrame& frame, const LocalizedScan& geometry,
                                    Pose2D capturePose) {
    struct Hit { std::size_t cell, column; float depth; };
    std::vector<Hit> hits;
    const auto width = thermalImageWidth(config_.mounts);
    std::vector<float> nearestDepth(width, std::numeric_limits<float>::infinity());
    const Pose2D lidarPose = sensorOrigin(geometry.pose, config_.mounts.lidar);
    const Pose2D camera = sensorOrigin(capturePose, config_.mounts.thermal);
    const float halfFov = thermalHorizontalFov(config_.mounts) * radiansPerDegree * 0.5f;
    const float tangent = std::tan(halfFov);
    if(halfFov <= 0 || halfFov >= pi * 0.5f || !std::isfinite(tangent)) return;
    for(const auto& point : geometry.scan.points) {
        if(!std::isfinite(point.distanceCm) || !std::isfinite(point.angleRadiansClockwise) ||
           point.distanceCm < config_.lidarMinRangeCm || point.distanceCm > config_.lidarMaxRangeCm) continue;
        const float angle = lidarPose.headingRadiansClockwise + point.angleRadiansClockwise;
        const float x = lidarPose.xCm + std::sin(angle) * point.distanceCm;
        const float y = lidarPose.yCm + std::cos(angle) * point.distanceCm;
        const float dx = x - camera.xCm, dy = y - camera.yCm;
        const float bearing = wrapAngle(std::atan2(dx, dy) - camera.headingRadiansClockwise);
        if(std::fabs(bearing) > halfFov) continue;
        const float normalized = std::clamp(0.5f + 0.5f * std::tan(bearing) / tangent, 0.0f, 1.0f);
        const auto column = std::min(static_cast<std::size_t>(normalized * static_cast<float>(width)), width - 1);
        int cellX = 0, cellY = 0;
        if(!worldToCell(x, y, cellX, cellY)) continue;
        const float depth = std::hypot(dx, dy);
        nearestDepth[column] = std::min(nearestDepth[column], depth);
        hits.push_back({cellIndex(cellX, cellY), column, depth});
    }
    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) {
        return a.cell != b.cell ? a.cell < b.cell : a.depth < b.depth;
    });
    std::size_t previousCell = std::numeric_limits<std::size_t>::max();
    for(const auto& hit : hits) {
        // A camera pixel sees the nearest surface, not every LiDAR endpoint
        // falling in the same angular pixel behind a foreground object.
        if(hit.cell == previousCell || hit.depth > nearestDepth[hit.column] +
           std::max(8.0f, nearestDepth[hit.column] * 0.03f)) continue;
        const float temperature = medianCenterTemperature(frame, hit.column, config_.mounts);
        if(!std::isfinite(temperature) || temperature < -40 || temperature > 300) continue;
        previousCell = hit.cell;
        auto& samples = thermalSampleCount_[hit.cell];
        if(samples == std::numeric_limits<std::uint16_t>::max()) {
            thermalAverageC_[hit.cell] += (temperature - thermalAverageC_[hit.cell]) * 0.01f;
        } else {
            thermalAverageC_[hit.cell] += (temperature - thermalAverageC_[hit.cell]) /
                                         (static_cast<float>(samples) + 1);
            ++samples;
        }
    }
}

} // namespace mapping
