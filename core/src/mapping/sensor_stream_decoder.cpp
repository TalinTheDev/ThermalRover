#include "mapping/sensor_stream_decoder.hpp"

#include "protocol/robot_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <utility>

namespace mapping {
namespace {
constexpr std::uint32_t maximumOdometryAgeMilliseconds = 100;
constexpr std::uint32_t maximumImuAgeMilliseconds = 100;
constexpr std::uint32_t maximumThermalAgeMilliseconds = 500;
constexpr std::uint32_t maximumThermalAssemblyAgeMilliseconds = 500;
constexpr std::size_t maximumOdometrySamples = 96;
constexpr std::size_t maximumImuSamples = 128;
constexpr std::size_t maximumThermalFrames = 8;
constexpr std::size_t maximumThermalAssemblies = 4;
constexpr float maximumImuRateDegreesPerSecond = 510.0f;
constexpr float maximumTemperatureCelsius = 300.0f;

std::uint32_t timestampDistance(std::uint32_t left, std::uint32_t right) {
    const std::uint32_t forward = left - right;
    return forward <= 0x7fffffffU ? forward : 0U - forward;
}
} // namespace

std::vector<SensorFrame> SensorStreamDecoder::consume(const Packet& packet) {
    std::vector<SensorFrame> frames;
    if(packet.size < sizeof(PacketHeader) || packet.size > BUFFER_LEN) return frames;

    PacketHeader header{};
    std::memcpy(&header, packet.buffer, sizeof(header));
    const std::size_t payloadSize = packet.size - sizeof(PacketHeader);
    if((header.version != PROTOCOL_VERSION &&
        header.version != PREVIOUS_REPLAY_PROTOCOL_VERSION &&
        header.version != LEGACY_REPLAY_PROTOCOL_VERSION) || header.type != DATA ||
       header.length != payloadSize) return frames;
    const std::span<const std::uint8_t> payload(packet.buffer + sizeof(PacketHeader),
                                                payloadSize);

    if(header.id == FUSED_ODOMETRY) {
        acceptOdometry(payload);
    } else if(header.id == IMU_SAMPLE) {
        acceptImu(payload);
    } else if(header.id == THERMAL_CHUNK) {
        acceptThermalChunk(payload);
    } else if(header.id == LIDAR_PACKET && payload.size() == sizeof(LIDAR_PACKET_DATA)) {
        LIDAR_PACKET_DATA lidar{};
        std::memcpy(&lidar, payload.data(), sizeof(lidar));
        const auto scans = lidarParser_.feed(lidar.packet, lidar.timestampMilliseconds);
        frames.reserve(scans.size());
        for(const LidarScan& scan : scans) {
            SensorFrame frame;
            frame.lidar = scan;
            attachSynchronizedSensors(frame);
            frames.push_back(std::move(frame));
        }
    }
    return frames;
}

void SensorStreamDecoder::acceptOdometry(std::span<const std::uint8_t> payload) {
    if(payload.size() != sizeof(FUSED_ODOMETRY_DATA)) return;
    FUSED_ODOMETRY_DATA data{};
    std::memcpy(&data, payload.data(), sizeof(data));
    if(!std::isfinite(data.xCentimeters) || !std::isfinite(data.yCentimeters) ||
       !std::isfinite(data.headingRadiansClockwise) ||
       !std::isfinite(data.residualCentimeters) || data.residualCentimeters < 0 ||
       std::fabs(data.headingRadiansClockwise) > std::numbers::pi_v<float> ||
       data.quality > 4 || data.matchedPoints > rover::odometry::scanBins) return;
    receivedFusedOdometry_ = true;
    odometry_.push_back({data.timestampMilliseconds,
        {data.xCentimeters, data.yCentimeters, data.headingRadiansClockwise}, data.quality == 1});
    while(odometry_.size() > maximumOdometrySamples) odometry_.pop_front();
}

void SensorStreamDecoder::acceptImu(std::span<const std::uint8_t> payload) {
    if(payload.size() != sizeof(IMU_SAMPLE_DATA)) return;
    IMU_SAMPLE_DATA data{};
    std::memcpy(&data, payload.data(), sizeof(data));
    if(!std::isfinite(data.yawRadiansClockwise) ||
       !std::isfinite(data.gyroZDegreesPerSecondClockwise) ||
       std::fabs(data.yawRadiansClockwise) > std::numbers::pi_v<float> ||
       std::fabs(data.gyroZDegreesPerSecondClockwise) >
           maximumImuRateDegreesPerSecond) return;
    imuSamples_.push_back({data.timestampMilliseconds,
                           data.yawRadiansClockwise, data.gyroZDegreesPerSecondClockwise});
    while(imuSamples_.size() > maximumImuSamples) imuSamples_.pop_front();
}

void SensorStreamDecoder::acceptThermalChunk(std::span<const std::uint8_t> payload) {
    if(payload.size() != sizeof(THERMAL_FRAME_CHUNK)) return;
    THERMAL_FRAME_CHUNK chunk{};
    std::memcpy(&chunk, payload.data(), sizeof(chunk));
    const std::size_t offset = chunk.pixelOffset;
    const std::size_t count = chunk.sampleCount;
    if(count == 0 || count > THERMAL_CHUNK_PIXELS ||
       offset > thermalPixelCount || count > thermalPixelCount - offset) return;

    // Reject invalid temperatures before expiring or evicting other frames.
    for(std::size_t i = 0; i < count; ++i) {
        if(chunk.temperaturesCentiC[i] < -4000 ||
           chunk.temperaturesCentiC[i] >
               static_cast<std::int16_t>(maximumTemperatureCelsius * 100.0f)) {
            ++incompleteThermalFrameCount_;
            thermalAssemblies_.erase(chunk.frameId);
            return;
        }
    }

    for(auto assembly = thermalAssemblies_.begin();
        assembly != thermalAssemblies_.end();) {
        const std::uint32_t age = chunk.timestampMilliseconds -
                                  assembly->second.timestampMilliseconds;
        if(age > maximumThermalAssemblyAgeMilliseconds && age <= 0x7fffffffU) {
            ++incompleteThermalFrameCount_;
            assembly = thermalAssemblies_.erase(assembly);
        } else {
            ++assembly;
        }
    }

    if(!thermalAssemblies_.contains(chunk.frameId) &&
       thermalAssemblies_.size() >= maximumThermalAssemblies) {
        auto evicted = thermalAssemblies_.begin();
        for(auto candidate = thermalAssemblies_.begin();
            candidate != thermalAssemblies_.end(); ++candidate) {
            const std::uint32_t olderBy = evicted->second.timestampMilliseconds -
                                          candidate->second.timestampMilliseconds;
            if(olderBy != 0 && olderBy <= 0x7fffffffU) evicted = candidate;
        }
        // UDP delivery order and frame IDs do not establish capture order.
        // Preserve recent partial frames, including across the rover clock wrap.
        ++incompleteThermalFrameCount_;
        const std::uint32_t incomingOlderBy = evicted->second.timestampMilliseconds -
                                              chunk.timestampMilliseconds;
        if(incomingOlderBy != 0 && incomingOlderBy <= 0x7fffffffU) return;
        thermalAssemblies_.erase(evicted);
    }
    ThermalAssembly& assembly = thermalAssemblies_[chunk.frameId];
    if(assembly.receivedCount != 0 &&
       assembly.timestampMilliseconds != chunk.timestampMilliseconds) {
        ++incompleteThermalFrameCount_;
        assembly = ThermalAssembly{};
    }
    assembly.timestampMilliseconds = chunk.timestampMilliseconds;
    for(std::size_t i = 0; i < count; ++i) {
        const std::size_t pixel = offset + i;
        if(assembly.received[pixel] == 0) {
            assembly.received[pixel] = 1;
            ++assembly.receivedCount;
        }
        assembly.centiDegrees[pixel] = chunk.temperaturesCentiC[i];
    }
    if(assembly.receivedCount != thermalPixelCount) return;

    TimedThermal completed;
    completed.timestampMilliseconds = assembly.timestampMilliseconds;
    completed.frame.timestampMilliseconds = assembly.timestampMilliseconds;
    completed.frame.valid = true;
    for(std::size_t i = 0; i < thermalPixelCount; ++i) {
        completed.frame.temperaturesC[i] =
            static_cast<float>(assembly.centiDegrees[i]) * 0.01f;
    }
    thermalFrames_.push_back(std::move(completed));
    ++completedThermalFrameCount_;
    while(thermalFrames_.size() > maximumThermalFrames) thermalFrames_.pop_front();
    thermalAssemblies_.erase(chunk.frameId);
}

void SensorStreamDecoder::attachSynchronizedSensors(SensorFrame& frame) const {
    const std::uint32_t scanTimestamp =
        static_cast<std::uint32_t>(frame.lidar.timestampMilliseconds);
    const TimedOdometry* nearestOdometry = nullptr;
    std::uint32_t odometryAge = maximumOdometryAgeMilliseconds + 1;
    for(const TimedOdometry& sample : odometry_) {
        const std::uint32_t age = timestampDistance(scanTimestamp,
                                                    sample.timestampMilliseconds);
        if(age < odometryAge) {
            odometryAge = age;
            nearestOdometry = &sample;
        }
    }
    frame.fusedOdometrySource = receivedFusedOdometry_;
    if(nearestOdometry != nullptr && odometryAge <= maximumOdometryAgeMilliseconds &&
       nearestOdometry->valid) {
        frame.fusedPose = nearestOdometry->pose;
        frame.hasFusedPose = true;
    }

    const TimedImu* nearestImu = nullptr;
    std::uint32_t imuAge = maximumImuAgeMilliseconds + 1;
    for(const TimedImu& sample : imuSamples_) {
        const std::uint32_t age = timestampDistance(
            scanTimestamp, sample.timestampMilliseconds);
        if(age < imuAge) {
            imuAge = age;
            nearestImu = &sample;
        }
    }
    if(nearestImu != nullptr && imuAge <= maximumImuAgeMilliseconds) {
        frame.imuHeadingRadiansClockwise = nearestImu->yawRadiansClockwise;
        frame.hasImuHeading = true;
    }
    float endYaw = 0;
    if(imuYawAt(scanTimestamp, endYaw)) {
        frame.imuHeadingRadiansClockwise = endYaw;
        frame.hasImuHeading = true;
        for(auto& point : frame.lidar.points) {
            float pointYaw = 0;
            if(point.hasTimestamp && imuYawAt(point.timestampMilliseconds, pointYaw)) {
                point.headingOffsetRadians = wrapAngle(pointYaw - endYaw);
                point.hasHeadingOffset = true;
            }
        }
    }

    const TimedThermal* nearestThermal = nullptr;
    std::uint32_t thermalAge = maximumThermalAgeMilliseconds + 1;
    for(const TimedThermal& sample : thermalFrames_) {
        const std::uint32_t age = timestampDistance(scanTimestamp,
                                                   sample.timestampMilliseconds);
        if(age < thermalAge) {
            thermalAge = age;
            nearestThermal = &sample;
        }
    }
    if(nearestThermal != nullptr && thermalAge <= maximumThermalAgeMilliseconds) {
        frame.thermal = nearestThermal->frame;
    }
}

bool SensorStreamDecoder::imuYawAt(std::uint32_t timestamp, float& yaw) const {
    const TimedImu *before = nullptr, *after = nullptr;
    std::uint32_t beforeAge = 0xffffffffU, afterAge = 0xffffffffU;
    for(const auto& sample : imuSamples_) {
        const auto past = timestamp - sample.timestampMilliseconds;
        const auto future = sample.timestampMilliseconds - timestamp;
        if(past <= 0x7fffffffU && past < beforeAge) { before = &sample; beforeAge = past; }
        if(future <= 0x7fffffffU && future < afterAge) { after = &sample; afterAge = future; }
    }
    if(before != nullptr && after != nullptr && beforeAge <= 100 && afterAge <= 100 &&
       beforeAge + afterAge <= 150) {
        const float fraction = beforeAge + afterAge == 0 ? 0.0f
            : static_cast<float>(beforeAge) / static_cast<float>(beforeAge + afterAge);
        yaw = wrapAngle(before->yawRadiansClockwise + fraction * wrapAngle(
            after->yawRadiansClockwise - before->yawRadiansClockwise));
        return true;
    }
    const auto* nearest = beforeAge <= afterAge ? before : after;
    if(nearest != nullptr && std::min(beforeAge, afterAge) <= 30) {
        const float seconds = beforeAge <= afterAge ? static_cast<float>(beforeAge) * 0.001f
                                                   : -static_cast<float>(afterAge) * 0.001f;
        yaw = wrapAngle(nearest->yawRadiansClockwise + seconds *
                       nearest->rateDegreesPerSecond * std::numbers::pi_v<float> / 180);
        return true;
    }
    return false;
}

void SensorStreamDecoder::reset() {
    lidarParser_.reset();
    odometry_.clear();
    receivedFusedOdometry_ = false;
    imuSamples_.clear();
    thermalAssemblies_.clear();
    thermalFrames_.clear();
    completedThermalFrameCount_ = 0;
    incompleteThermalFrameCount_ = 0;
}

} // namespace mapping
