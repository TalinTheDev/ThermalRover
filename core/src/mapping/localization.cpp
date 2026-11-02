#include "mapping/slam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mapping {
namespace {
constexpr float pi = 3.14159265358979323846f;
struct Point { float x, y; };
Pose2D applyDelta(Pose2D mapPose, Pose2D previous, Pose2D current) {
    const float angle = mapPose.headingRadiansClockwise - previous.headingRadiansClockwise;
    const float c = std::cos(angle), s = std::sin(angle);
    const float dx = current.xCm - previous.xCm, dy = current.yCm - previous.yCm;
    return {mapPose.xCm + c * dx + s * dy, mapPose.yCm - s * dx + c * dy,
            wrapAngle(mapPose.headingRadiansClockwise + wrapAngle(
                current.headingRadiansClockwise - previous.headingRadiansClockwise))};
}
} // namespace

Pose2D SlamMapper::estimateOdometry(const SensorFrame& frame) {
    // With scan matching explicitly disabled, retain deterministic rover-pose
    // playback and pause/resume behavior. Normal mapping treats it as a prior.
    if(!config_.enableScanMatching && frame.hasFusedPose) {
        if(!fusedPoseBaselineSet_) {
            previousFusedPose_ = frame.fusedPose;
            fusedPoseBaselineSet_ = true;
            return pose_;
        }
        const auto result = applyDelta(pose_, previousFusedPose_, frame.fusedPose);
        previousFusedPose_ = frame.fusedPose;
        return result;
    }
    rover::odometry::Config localConfig{
        config_.mounts.lidar.xCm, config_.mounts.lidar.yCm,
        config_.mounts.lidar.headingRadiansClockwise, config_.lidarMinRangeCm,
        config_.lidarMaxRangeCm, config_.gyroYawPriorSigmaRadians, false};
    rover::odometry::Scan scan{};
    for(const auto& point : frame.lidar.points)
        rover::odometry::addPolarPoint(scan, point.angleRadiansClockwise, point.distanceCm, localConfig);
    const auto estimate = lidarImuOdometry_.update(scan,
        static_cast<std::uint32_t>(frame.lidar.timestampMilliseconds),
        frame.imuHeadingRadiansClockwise, frame.hasImuHeading);
    const Pose2D local{estimate.pose.xCm, estimate.pose.yCm, estimate.pose.headingRadians};
    if(!config_.enableScanMatching) return local;
    const float dt = hasLocalizedTime_ ? std::clamp(static_cast<float>(
        static_cast<std::uint32_t>(frame.lidar.timestampMilliseconds) - lastLocalizedTime_) * 0.001f,
        0.0f, 0.25f) : 0;
    Pose2D prediction{pose_.xCm + mapVelocity_.xCm * dt,
        pose_.yCm + mapVelocity_.yCm * dt,
        wrapAngle(pose_.headingRadiansClockwise + mapVelocity_.headingRadiansClockwise * dt)};
    if(estimate.translationValid && previousLocalTracking_)
        prediction = applyDelta(pose_, previousLocalPose_, local);
    else if(frame.hasFusedPose && previousFusedTracking_)
        prediction = applyDelta(pose_, previousFusedPose_, frame.fusedPose);
    previousLocalPose_ = local;
    previousLocalTracking_ = estimate.translationValid;
    if(frame.hasFusedPose) previousFusedPose_ = frame.fusedPose;
    previousFusedTracking_ = frame.hasFusedPose;
    // An initialization/reset must not drag the current map to a new origin.
    const float dx = prediction.xCm - pose_.xCm, dy = prediction.yCm - pose_.yCm;
    const float distance = std::hypot(dx, dy);
    if(distance > 40) {
        prediction.xCm = pose_.xCm + dx * 40 / distance;
        prediction.yCm = pose_.yCm + dy * 40 / distance;
    }
    return prediction;
}

LidarScan SlamMapper::deskewScan(const LidarScan& input) const {
    LidarScan result = input;
    const auto time = static_cast<std::uint32_t>(input.timestampMilliseconds);
    const float dt = hasLocalizedTime_ ? std::min(0.25f, static_cast<float>(time - lastLocalizedTime_) * 0.001f) : 0;
    const float endHeading = pose_.headingRadiansClockwise + mapVelocity_.headingRadiansClockwise * dt;
    const float c = std::cos(endHeading), s = std::sin(endHeading);
    const auto& mount = config_.mounts.lidar;
    const float cm = std::cos(mount.headingRadiansClockwise), sm = std::sin(mount.headingRadiansClockwise);
    for(auto& point : result.points) {
        if(!point.hasTimestamp) continue;
        const auto ageMs = time - point.timestampMilliseconds;
        if(ageMs > 250) continue;
        const float age = static_cast<float>(ageMs) * 0.001f;
        const float yaw = point.hasHeadingOffset ? point.headingOffsetRadians
                                                : -mapVelocity_.headingRadiansClockwise * age;
        const float cx = std::sin(point.angleRadiansClockwise) * point.distanceCm;
        const float cy = std::cos(point.angleRadiansClockwise) * point.distanceCm;
        const float x = mount.xCm + cm * cx + sm * cy;
        const float y = mount.yCm - sm * cx + cm * cy;
        const float translationX = -mapVelocity_.xCm * age, translationY = -mapVelocity_.yCm * age;
        const float endX = std::cos(yaw) * x + std::sin(yaw) * y +
                           c * translationX - s * translationY - mount.xCm;
        const float endY = -std::sin(yaw) * x + std::cos(yaw) * y +
                           s * translationX + c * translationY - mount.yCm;
        const float lidarX = cm * endX - sm * endY, lidarY = sm * endX + cm * endY;
        point.distanceCm = std::hypot(lidarX, lidarY);
        point.angleRadiansClockwise = std::atan2(lidarX, lidarY);
    }
    return result;
}

SlamMapper::ScanMatch SlamMapper::matchScan(const LidarScan& scan, Pose2D prior,
                                          float elapsedSeconds) {
    if(!containsOccupiedCells_) return {prior, 0, false};
    const int width = static_cast<int>(config_.mapWidthCells), height = static_cast<int>(config_.mapHeightCells);
    matchingField_.resize(occupancy_.size());
    for(std::size_t i = 0; i < occupancy_.size(); ++i)
        matchingField_[i] = occupancy_[i] >= 3 ? 0.0f : 10000.0f;
    // Two-pass chamfer distance field. Unlike scoring occupied cells directly,
    // this has a smooth neighborhood for continuous, sub-degree refinement.
    for(int y = 0; y < height; ++y) for(int x = 0; x < width; ++x) {
        auto& value = matchingField_[static_cast<std::size_t>(y) * width + x];
        if(x > 0) value = std::min(value, matchingField_[static_cast<std::size_t>(y) * width + x - 1] + 1);
        if(y > 0) {
            value = std::min(value, matchingField_[static_cast<std::size_t>(y - 1) * width + x] + 1);
            if(x > 0) value = std::min(value, matchingField_[static_cast<std::size_t>(y - 1) * width + x - 1] + 1.414214f);
            if(x + 1 < width) value = std::min(value, matchingField_[static_cast<std::size_t>(y - 1) * width + x + 1] + 1.414214f);
        }
    }
    for(int y = height - 1; y >= 0; --y) for(int x = width - 1; x >= 0; --x) {
        auto& value = matchingField_[static_cast<std::size_t>(y) * width + x];
        if(x + 1 < width) value = std::min(value, matchingField_[static_cast<std::size_t>(y) * width + x + 1] + 1);
        if(y + 1 < height) {
            value = std::min(value, matchingField_[static_cast<std::size_t>(y + 1) * width + x] + 1);
            if(x > 0) value = std::min(value, matchingField_[static_cast<std::size_t>(y + 1) * width + x - 1] + 1.414214f);
            if(x + 1 < width) value = std::min(value, matchingField_[static_cast<std::size_t>(y + 1) * width + x + 1] + 1.414214f);
        }
    }
    for(auto& value : matchingField_) value = value > 5 ? 0 : std::exp(-value * value / 4.5f);
    std::vector<Point> points;
    const auto& mount = config_.mounts.lidar;
    const std::size_t step = std::max<std::size_t>(1, scan.points.size() / 160);
    for(std::size_t i = 0; i < scan.points.size(); i += step) {
        const auto& point = scan.points[i];
        if(!std::isfinite(point.distanceCm) || !std::isfinite(point.angleRadiansClockwise) ||
           point.distanceCm < config_.lidarMinRangeCm || point.distanceCm > config_.lidarMaxRangeCm) continue;
        const float angle = point.angleRadiansClockwise + mount.headingRadiansClockwise;
        points.push_back({mount.xCm + std::sin(angle) * point.distanceCm,
                          mount.yCm + std::cos(angle) * point.distanceCm});
    }
    if(points.size() < 30) return {pose_, 0, false};
    const float inverseCell = 1.0f / config_.centimetersPerCell;
    const auto score = [&](Pose2D candidate) {
        const float c = std::cos(candidate.headingRadiansClockwise), s = std::sin(candidate.headingRadiansClockwise);
        float sum = 0;
        for(const auto point : points) {
            const float gx = (candidate.xCm + c * point.x + s * point.y) * inverseCell + static_cast<float>(width) * 0.5f - 0.5f;
            const float gy = (candidate.yCm - s * point.x + c * point.y) * inverseCell + static_cast<float>(height) * 0.5f - 0.5f;
            if(gx < 0 || gy < 0 || gx >= static_cast<float>(width - 1) || gy >= static_cast<float>(height - 1)) continue;
            const int ix = static_cast<int>(gx), iy = static_cast<int>(gy);
            const float fx = gx - static_cast<float>(ix), fy = gy - static_cast<float>(iy);
            const auto index = static_cast<std::size_t>(iy) * width + ix;
            const float lower = matchingField_[index] * (1 - fx) + matchingField_[index + 1] * fx;
            const float upper = matchingField_[index + width] * (1 - fx) + matchingField_[index + width + 1] * fx;
            sum += lower * (1 - fy) + upper * fy;
        }
        return sum / static_cast<float>(points.size());
    };
    const auto ranked = [&](Pose2D candidate) {
        return score(candidate) - 0.0003f * std::hypot(candidate.xCm - prior.xCm, candidate.yCm - prior.yCm) -
               0.01f * std::fabs(wrapAngle(candidate.headingRadiansClockwise - prior.headingRadiansClockwise));
    };
    Pose2D best = prior;
    float bestRank = ranked(best);
    const bool recovery = consecutiveRejectedScans_ != 0 || elapsedSeconds > 0.3f;
    const float radius = recovery ? std::min(120.0f, 48.0f + 8.0f * static_cast<float>(consecutiveRejectedScans_)) : 24.0f;
    const float translationStep = recovery ? 8.0f : 4.0f;
    const int translations = static_cast<int>(std::ceil(radius / translationStep));
    const int yawRadius = elapsedSeconds > 0.6f ? 180 : recovery ? 90 : 45;
    const int yawStep = recovery ? 5 : 3;
    for(int yaw = -yawRadius; yaw <= yawRadius; yaw += yawStep)
        for(int y = -translations; y <= translations; ++y)
            for(int x = -translations; x <= translations; ++x) {
                const Pose2D candidate{prior.xCm + static_cast<float>(x) * translationStep,
                    prior.yCm + static_cast<float>(y) * translationStep,
                    wrapAngle(pose_.headingRadiansClockwise + static_cast<float>(yaw) * pi / 180)};
                const float rank = ranked(candidate);
                if(rank > bestRank) { bestRank = rank; best = candidate; }
            }
    for(int level = 0; level < 3; ++level) {
        const float positionStep = level == 0 ? 2.0f : level == 1 ? 0.5f : 0.125f;
        const float rotationStep = (level == 0 ? 1.0f : level == 1 ? 0.25f : 0.0625f) * pi / 180;
        for(int iteration = 0; iteration < 4; ++iteration) {
            const Pose2D center = best;
            for(int yaw = -1; yaw <= 1; ++yaw) for(int y = -1; y <= 1; ++y) for(int x = -1; x <= 1; ++x) {
                const Pose2D candidate{center.xCm + static_cast<float>(x) * positionStep,
                    center.yCm + static_cast<float>(y) * positionStep,
                    wrapAngle(center.headingRadiansClockwise + static_cast<float>(yaw) * rotationStep)};
                const float rank = ranked(candidate);
                if(rank > bestRank) { bestRank = rank; best = candidate; }
            }
            if(center.xCm == best.xCm && center.yCm == best.yCm &&
               center.headingRadiansClockwise == best.headingRadiansClockwise) break;
        }
    }
    const float fit = score(best);
    // A flat likelihood direction cannot establish translation (e.g. an
    // unbounded corridor). Keep the last map instead of painting guessed poses.
    constexpr float curvatureStep = 8;
    const auto offsetScore = [&](float x, float y) { return score({best.xCm + x, best.yCm + y, best.headingRadiansClockwise}); };
    const float hxx = 2 * fit - offsetScore(curvatureStep, 0) - offsetScore(-curvatureStep, 0);
    const float hyy = 2 * fit - offsetScore(0, curvatureStep) - offsetScore(0, -curvatureStep);
    const float hxy = -(offsetScore(curvatureStep, curvatureStep) - offsetScore(curvatureStep, -curvatureStep) -
                       offsetScore(-curvatureStep, curvatureStep) + offsetScore(-curvatureStep, -curvatureStep)) * 0.25f;
    const float minimumCurvature = 0.5f * (hxx + hyy - std::hypot(hxx - hyy, 2 * hxy));
    return {best, fit, fit >= 0.45f && minimumCurvature > 0.001f};
}
} // namespace mapping
