#pragma once

#include "mapping/lidar_packet_parser.hpp"
#include "protocol/udp.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <span>
#include <unordered_map>
#include <vector>

namespace mapping {

// Reassembles the ESP32 DATA packets into synchronized frames for the mapper.
// Nearest fused pose, IMU, and thermal samples use the shared rover clock.
// Historical wheel packets are ignored.
class SensorStreamDecoder {
public:
    std::vector<SensorFrame> consume(const Packet& packet);
    void reset();
    std::uint64_t rejectedLidarAnglePacketCount() const {
        return lidarParser_.rejectedAnglePacketCount();
    }
    std::uint64_t rejectedLidarTimestampPacketCount() const {
        return lidarParser_.rejectedTimestampPacketCount();
    }
    std::uint64_t completedThermalFrameCount() const {
        return completedThermalFrameCount_;
    }
    ThermalFrame latestThermalFrame() const {
        return thermalFrames_.empty() ? ThermalFrame{} : thermalFrames_.back().frame;
    }
    std::uint64_t incompleteThermalFrameCount() const {
        return incompleteThermalFrameCount_;
    }

private:
    struct TimedOdometry {
        std::uint32_t timestampMilliseconds = 0;
        Pose2D pose{};
        bool valid = false;
    };

    struct ThermalAssembly {
        std::uint32_t timestampMilliseconds = 0;
        std::array<std::int16_t, thermalPixelCount> centiDegrees{};
        std::array<std::uint8_t, thermalPixelCount> received{};
        std::size_t receivedCount = 0;
    };

    struct TimedThermal {
        std::uint32_t timestampMilliseconds = 0;
        ThermalFrame frame{};
    };

    struct TimedImu {
        std::uint32_t timestampMilliseconds = 0;
        float yawRadiansClockwise = 0.0f;
        float rateDegreesPerSecond = 0;
    };

    LidarPacketParser lidarParser_;
    std::deque<TimedOdometry> odometry_;
    bool receivedFusedOdometry_ = false;
    std::deque<TimedImu> imuSamples_;
    std::unordered_map<std::uint16_t, ThermalAssembly> thermalAssemblies_;
    std::deque<TimedThermal> thermalFrames_;
    std::uint64_t completedThermalFrameCount_ = 0;
    std::uint64_t incompleteThermalFrameCount_ = 0;

    void acceptOdometry(std::span<const std::uint8_t> payload);
    void acceptImu(std::span<const std::uint8_t> payload);
    void acceptThermalChunk(std::span<const std::uint8_t> payload);
    void attachSynchronizedSensors(SensorFrame& frame) const;
    bool imuYawAt(std::uint32_t timestamp, float& yaw) const;
};

} // namespace mapping
