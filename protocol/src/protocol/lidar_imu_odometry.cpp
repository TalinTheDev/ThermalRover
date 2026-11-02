#include "protocol/lidar_imu_odometry.hpp"
#include <algorithm>
#include <cmath>

namespace rover::odometry {
namespace {
constexpr float pi = 3.14159265358979323846f;
constexpr float twoPi = 2 * pi;
bool valid(Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && (p.x != 0 || p.y != 0);
}
int binFor(Point p) {
    const int bin = static_cast<int>(std::lround(std::atan2(p.x, p.y) * 180.0f / pi));
    return (bin % 360 + 360) % 360;
}
float squaredDistance(Point a, Point b) {
    const float x = a.x - b.x, y = a.y - b.y;
    return x * x + y * y;
}
bool solve(float h[3][3], const float gradient[3], float step[3]) {
    float a[3][4]{};
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) a[row][col] = h[row][col];
        a[row][3] = -gradient[row];
    }
    for(int col = 0; col < 3; ++col) {
        int pivot = col;
        for(int row = col + 1; row < 3; ++row)
            if(std::fabs(a[row][col]) > std::fabs(a[pivot][col])) pivot = row;
        if(std::fabs(a[pivot][col]) < 1e-5f) return false;
        for(int k = col; k < 4; ++k) std::swap(a[col][k], a[pivot][k]);
        const float scale = a[col][col];
        for(int k = col; k < 4; ++k) a[col][k] /= scale;
        for(int row = 0; row < 3; ++row) {
            if(row == col) continue;
            const float factor = a[row][col];
            for(int k = col; k < 4; ++k) a[row][k] -= factor * a[col][k];
        }
    }
    for(int row = 0; row < 3; ++row) {
        step[row] = a[row][3];
        if(!std::isfinite(step[row])) return false;
    }
    return true;
}
} // namespace
float wrap(float radians) {
    return std::isfinite(radians) ? std::remainder(radians, twoPi) : 0.0f;
}
void addPolarPoint(Scan& scan, float angle, float distance, const Config& config) {
    if(!std::isfinite(angle) || !std::isfinite(distance) ||
       distance < config.minimumRangeCm || distance > config.maximumRangeCm) return;
    const float bearing = angle + config.lidarYawRadians;
    const Point point{config.lidarXcm + std::sin(bearing) * distance,
                      config.lidarYcm + std::cos(bearing) * distance};
    if(!valid(point)) return;
    auto& bin = scan[static_cast<std::size_t>(binFor(point))];
    if(!valid(bin) || squaredDistance(point, {}) < squaredDistance(bin, {})) bin = point;
}
LidarImuOdometry::LidarImuOdometry(Config config) : config_(config) { reset(); }
void LidarImuOdometry::reset(Pose origin) {
    estimate_ = {};
    estimate_.pose = origin;
    reference_ = {};
    referencePose_ = origin;
    referenceTime_ = 0;
    referenceSet_ = imuBaselineSet_ = false;
}
const Estimate& LidarImuOdometry::update(const Scan& scan, std::uint32_t time,
                                        float imuYaw, bool imuValid) {
    estimate_.timestampMilliseconds = time;
    estimate_.translationValid = false;
    estimate_.matchedPoints = 0;
    estimate_.residualCm = 0;
    const bool gyroReady = imuValid && std::isfinite(imuYaw);
    estimate_.headingValid = gyroReady;
    if(!gyroReady && config_.requireImu) {
        estimate_.quality = Quality::ImuUnavailable;
        imuBaselineSet_ = false;
        referenceSet_ = false;
        return estimate_;
    }
    if(gyroReady && imuBaselineSet_)
        estimate_.pose.headingRadians = wrap(estimate_.pose.headingRadians + wrap(imuYaw - previousImuYaw_));
    if(gyroReady) previousImuYaw_ = imuYaw;
    imuBaselineSet_ = gyroReady;
    const auto pointCount = std::count_if(scan.begin(), scan.end(), valid);
    if(pointCount < 60) {
        estimate_.quality = Quality::WeakGeometry;
        return estimate_;
    }
    if(!referenceSet_ || time - referenceTime_ > 500U) {
        if(referenceSet_) ++estimate_.resets;
        reference_ = scan;
        referencePose_ = estimate_.pose;
        referenceTime_ = time;
        referenceSet_ = true;
        estimate_.quality = Quality::Initializing;
        return estimate_;
    }
    const float yawPrior = wrap(estimate_.pose.headingRadians - referencePose_.headingRadians);
    float x = 0, y = 0, yaw = yawPrior;
    bool observable = false;
    float residualSquared = 0;
    std::uint16_t matches = 0;
    for(int iteration = 0; iteration < 10; ++iteration) {
        float h[3][3]{}, gradient[3]{};
        residualSquared = 0;
        matches = 0;
        const float c = std::cos(yaw), s = std::sin(yaw);
        for(const Point point : scan) {
            if(!valid(point)) continue;
            const Point transformed{c * point.x + s * point.y + x,
                                    -s * point.x + c * point.y + y};
            const int center = binFor(transformed);
            int nearest = -1;
            float best = 35.0f * 35.0f;
            for(int offset = -12; offset <= 12; ++offset) {
                const int index = (center + offset + 360) % 360;
                if(!valid(reference_[index])) continue;
                const float distance = squaredDistance(transformed, reference_[index]);
                if(distance < best) { best = distance; nearest = index; }
            }
            if(nearest < 0) continue;
            int before = -1, after = -1;
            for(int offset = 1; offset <= 3; ++offset) {
                const int left = (nearest - offset + 360) % 360;
                const int right = (nearest + offset) % 360;
                if(before < 0 && valid(reference_[left])) before = left;
                if(after < 0 && valid(reference_[right])) after = right;
            }
            if(before < 0 || after < 0) continue;
            const Point tangent{reference_[after].x - reference_[before].x,
                                reference_[after].y - reference_[before].y};
            const float length = std::hypot(tangent.x, tangent.y);
            if(length < 0.5f || length > 35.0f) continue;
            const float nx = tangent.y / length, ny = -tangent.x / length;
            const float residual = nx * (transformed.x - reference_[nearest].x) +
                                   ny * (transformed.y - reference_[nearest].y);
            if(std::fabs(residual) > 12.0f) continue;
            const float weight = std::fabs(residual) > 2.0f ? 2.0f / std::fabs(residual) : 1.0f;
            const float jacobian[3]{nx, ny,
                nx * (-s * point.x + c * point.y) + ny * (-c * point.x - s * point.y)};
            for(int row = 0; row < 3; ++row) {
                gradient[row] += weight * jacobian[row] * residual;
                for(int col = 0; col < 3; ++col) h[row][col] += weight * jacobian[row] * jacobian[col];
            }
            residualSquared += residual * residual;
            ++matches;
        }
        const float trace = h[0][0] + h[1][1];
        const float eigenMinimum = 0.5f * (trace - std::hypot(h[0][0] - h[1][1], 2 * h[0][1]));
        observable = matches >= 40 && eigenMinimum > 0.04f * static_cast<float>(matches);
        if(!observable) break;
        const float sigma = std::max(0.005f, config_.gyroPriorSigmaRadians);
        const float gyroWeight = gyroReady ? 1.0f / (sigma * sigma) : 0.0f;
        h[2][2] += gyroWeight;
        gradient[2] += gyroWeight * wrap(yaw - yawPrior);
        float step[3]{};
        if(!solve(h, gradient, step)) { observable = false; break; }
        x += std::clamp(step[0], -10.0f, 10.0f);
        y += std::clamp(step[1], -10.0f, 10.0f);
        yaw += std::clamp(step[2], -0.035f, 0.035f);
        if(std::hypot(step[0], step[1]) < 0.01f && std::fabs(step[2]) < 0.0001f) break;
    }
    estimate_.matchedPoints = matches;
    estimate_.residualCm = matches ? std::sqrt(residualSquared / static_cast<float>(matches)) : 0;
    if(!observable) {
        estimate_.quality = Quality::WeakGeometry;
        return estimate_;
    }
    const float elapsed = static_cast<float>(time - referenceTime_) * 0.001f;
    if(estimate_.residualCm > 4.0f || std::hypot(x, y) > std::max(5.0f, elapsed * 100.0f) ||
       (gyroReady && std::fabs(wrap(yaw - yawPrior)) > 0.07f)) {
        estimate_.quality = Quality::RejectedMatch;
        return estimate_;
    }
    const float c = std::cos(referencePose_.headingRadians), s = std::sin(referencePose_.headingRadians);
    estimate_.pose.xCm = referencePose_.xCm + c * x + s * y;
    estimate_.pose.yCm = referencePose_.yCm - s * x + c * y;
    estimate_.pose.headingRadians = wrap(referencePose_.headingRadians + yaw);
    estimate_.quality = Quality::Tracking;
    estimate_.translationValid = true;
    estimate_.headingValid = true;
    reference_ = scan;
    referencePose_ = estimate_.pose;
    referenceTime_ = time;
    return estimate_;
}
} // namespace rover::odometry
