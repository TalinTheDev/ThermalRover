#include "mapping/lidar_packet_parser.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace mapping {
namespace {
constexpr std::uint8_t packetHeader = 0x54;
constexpr std::uint8_t packetVersionAndLength = 0x2c;
constexpr std::size_t pointsPerPacket = 12;
constexpr std::size_t pointsOffset = 6;
constexpr std::size_t endAngleOffset = 42;
constexpr std::size_t timestampOffset = 44;
constexpr std::size_t crcOffset = 46;
constexpr float centidegreesToRadians = std::numbers::pi_v<float> / 18000.0f;

std::uint16_t readLittleEndian(const std::uint8_t* bytes) {
    return static_cast<std::uint16_t>(bytes[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U);
}
} // namespace

std::uint8_t LidarPacketParser::crc8(std::span<const std::uint8_t> bytes) {
    // LDROBOT's table is equivalent to an MSB-first CRC-8 with polynomial 0x4D.
    std::uint8_t crc = 0;
    for(const std::uint8_t byte : bytes) {
        crc = static_cast<std::uint8_t>(crc ^ byte);
        for(int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U) != 0
                ? static_cast<std::uint8_t>((crc << 1U) ^ 0x4DU)
                : static_cast<std::uint8_t>(crc << 1U);
        }
    }
    return crc;
}

bool LidarPacketParser::decodePacket(std::span<const std::uint8_t> packet,
                                    std::vector<LidarPoint>& points,
                                    std::uint16_t* startAngleCentidegrees) {
    points.clear();
    if(packet.size() != packetSize || packet[0] != packetHeader ||
       packet[1] != packetVersionAndLength ||
       crc8(packet.first(crcOffset)) != packet[crcOffset]) {
        return false;
    }

    const std::uint16_t startAngle = readLittleEndian(packet.data() + 4);
    const std::uint16_t endAngle = readLittleEndian(packet.data() + endAngleOffset);
    if(startAngle >= 36000 || endAngle >= 36000) return false;
    if(startAngleCentidegrees != nullptr) *startAngleCentidegrees = startAngle;

    std::uint32_t angularSpan = (static_cast<std::uint32_t>(endAngle) + 36000U -
                                 static_cast<std::uint32_t>(startAngle)) % 36000U;
    // Identical endpoints are a zero span for a single packet, rather than a
    // complete revolution.
    if(angularSpan > 18000U) angularSpan = 0;

    points.reserve(pointsPerPacket);
    for(std::size_t i = 0; i < pointsPerPacket; ++i) {
        const std::uint8_t* pointBytes = packet.data() + pointsOffset + i * 3;
        const std::uint16_t distanceMillimeters = readLittleEndian(pointBytes);
        if(distanceMillimeters > 12000U) {
            points.clear();
            return false;
        }
        const float fraction = static_cast<float>(i) /
                               static_cast<float>(pointsPerPacket - 1);
        const float clockwiseCentidegrees = static_cast<float>(startAngle) +
                                             static_cast<float>(angularSpan) * fraction;
        float angleRadians = clockwiseCentidegrees * centidegreesToRadians;
        angleRadians = std::remainder(angleRadians, 2.0f * std::numbers::pi_v<float>);
        points.push_back({angleRadians,
                          static_cast<float>(distanceMillimeters) * 0.1f,
                          pointBytes[2]});
    }
    return true;
}

std::vector<LidarScan> LidarPacketParser::feed(std::span<const std::uint8_t> bytes) {
    return feedImpl(bytes, false, 0);
}

std::vector<LidarScan> LidarPacketParser::feed(
    std::span<const std::uint8_t> bytes, std::uint32_t captureTimestampMilliseconds) {
    return feedImpl(bytes, true, captureTimestampMilliseconds);
}

std::vector<LidarScan> LidarPacketParser::feedImpl(
    std::span<const std::uint8_t> bytes, bool hasCaptureTimestamp,
    std::uint32_t captureTimestampMilliseconds) {
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
    std::vector<LidarScan> scans;

    while(buffer_.size() >= packetSize) {
        const auto header = std::find(buffer_.begin(), buffer_.end(), packetHeader);
        if(header != buffer_.begin()) {
            if(header == buffer_.end()) {
                buffer_.clear();
                break;
            }
            buffer_.erase(buffer_.begin(), header);
        }
        if(buffer_.size() < packetSize) break;
        if(buffer_[1] != packetVersionAndLength ||
           crc8(std::span<const std::uint8_t>(buffer_.data(), crcOffset)) !=
               buffer_[crcOffset]) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        std::vector<LidarPoint> packetPoints;
        std::uint16_t startAngle = 0;
        const bool decoded = decodePacket(
            std::span<const std::uint8_t>(buffer_.data(), packetSize),
            packetPoints, &startAngle);
        const std::uint64_t packetTimestamp = hasCaptureTimestamp
            ? captureTimestampMilliseconds
            : readLittleEndian(buffer_.data() + timestampOffset);
        if(decoded && hasCaptureTimestamp) {
            const auto endAngle = readLittleEndian(buffer_.data() + endAngleOffset);
            const float spanDegrees = static_cast<float>((static_cast<unsigned>(endAngle) + 36000U - startAngle) % 36000U) * 0.01f;
            const float speed = static_cast<float>(readLittleEndian(buffer_.data() + 2));
            const float durationMs = speed >= 100 && spanDegrees <= 180 ? spanDegrees * 1000 / speed : 0;
            for(std::size_t i = 0; i < packetPoints.size(); ++i) {
                const float fraction = static_cast<float>(i) / static_cast<float>(packetPoints.size() - 1);
                auto& point = packetPoints[i];
                point.timestampMilliseconds = static_cast<std::uint32_t>(packetTimestamp) -
                    static_cast<std::uint32_t>(std::lround(durationMs * (1 - fraction)));
                point.hasTimestamp = true;
            }
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(packetSize));
        if(!decoded) continue;

        if(hasCaptureTimestamp && hasPreviousCaptureTimestamp_) {
            const std::uint32_t elapsed = captureTimestampMilliseconds -
                previousCaptureTimestampMilliseconds_;
            if(elapsed >= 0x80000000U) {
                // A late UDP datagram may have a plausible angle but belong
                // to an older scan. Allow normal 32-bit clock wraparound.
                ++rejectedTimestampPacketCount_;
                continue;
            }
        }

        bool crossedRevolution = false;
        if(hasPreviousAngle_ && startAngle < previousStartAngle_) {
            crossedRevolution = previousStartAngle_ > 27000U &&
                                startAngle < 9000U;
            if(!crossedRevolution) {
                // UDP may reorder adjacent sensor datagrams. Do not splice an
                // older packet into the current revolution or move the parser
                // angle baseline backwards.
                ++rejectedAnglePacketCount_;
                continue;
            }
        } else if(hasPreviousAngle_ && previousStartAngle_ < 9000U &&
                  startAngle > 27000U) {
            // A high-angle packet arriving just after wrap belongs to the
            // preceding revolution and must not contaminate this scan.
            ++rejectedAnglePacketCount_;
            continue;
        }

        if(crossedRevolution && !currentScan_.empty()) {
            scans.push_back({std::move(currentScan_), scanTimestampMilliseconds_});
            currentScan_.clear();
        }
        currentScan_.insert(currentScan_.end(), packetPoints.begin(), packetPoints.end());
        // Timestamp a revolution at its final received packet, which is closest
        // to the gyro, pose, and thermal samples used to interpret that scan.
        scanTimestampMilliseconds_ = packetTimestamp;
        previousStartAngle_ = startAngle;
        hasPreviousAngle_ = true;
        if(hasCaptureTimestamp) {
            previousCaptureTimestampMilliseconds_ = captureTimestampMilliseconds;
            hasPreviousCaptureTimestamp_ = true;
        } else {
            hasPreviousCaptureTimestamp_ = false;
        }
    }
    return scans;
}

void LidarPacketParser::reset() {
    buffer_.clear();
    currentScan_.clear();
    previousStartAngle_ = 0;
    hasPreviousAngle_ = false;
    scanTimestampMilliseconds_ = 0;
    rejectedAnglePacketCount_ = 0;
    previousCaptureTimestampMilliseconds_ = 0;
    hasPreviousCaptureTimestamp_ = false;
    rejectedTimestampPacketCount_ = 0;
}

} // namespace mapping
