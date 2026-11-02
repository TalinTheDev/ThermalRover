#pragma once

#include "mapping/slam.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace mapping {

// Streaming decoder for the 47-byte LD19 data packet. feed() accepts arbitrary
// byte chunks and emits a scan after the packet stream crosses 360 degrees.
class LidarPacketParser {
public:
    std::vector<LidarScan> feed(std::span<const std::uint8_t> bytes);
    std::vector<LidarScan> feed(std::span<const std::uint8_t> bytes,
                                std::uint32_t captureTimestampMilliseconds);
    void reset();
    std::uint64_t rejectedAnglePacketCount() const {
        return rejectedAnglePacketCount_;
    }
    std::uint64_t rejectedTimestampPacketCount() const {
        return rejectedTimestampPacketCount_;
    }

    static bool decodePacket(std::span<const std::uint8_t> packet,
                             std::vector<LidarPoint>& points,
                             std::uint16_t* startAngleCentidegrees = nullptr);
    static std::uint8_t crc8(std::span<const std::uint8_t> bytes);

private:
    static constexpr std::size_t packetSize = 47;
    std::vector<std::uint8_t> buffer_;
    std::vector<LidarPoint> currentScan_;
    std::uint16_t previousStartAngle_ = 0;
    bool hasPreviousAngle_ = false;
    std::uint64_t scanTimestampMilliseconds_ = 0;
    std::uint64_t rejectedAnglePacketCount_ = 0;
    std::uint32_t previousCaptureTimestampMilliseconds_ = 0;
    bool hasPreviousCaptureTimestamp_ = false;
    std::uint64_t rejectedTimestampPacketCount_ = 0;

    std::vector<LidarScan> feedImpl(std::span<const std::uint8_t> bytes,
                                    bool hasCaptureTimestamp,
                                    std::uint32_t captureTimestampMilliseconds);
};

} // namespace mapping
