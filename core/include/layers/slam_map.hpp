#pragma once

#include "app.hpp"
#include "mapping/lidar_packet_parser.hpp"
#include "mapping/sensor_stream_decoder.hpp"
#include "mapping/slam.hpp"
#include "renderer/thermal_map_renderer.hpp"

#include <atomic>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

class SlamMapLayer : public AppLayer {
public:
    explicit SlamMapLayer(mapping::SlamConfig config = {});

    void update() override;
    void render() override;
    void renderUI() override;

    void submitFrame(mapping::SensorFrame frame);
    void ingestPacket(const Packet& packet);
    // Replay runs on the UI thread and must retain every recorded scan,
    // regardless of how many scans a seek processes in one application frame.
    void ingestReplayPacket(const Packet& packet);
    void flushPendingFrames();
    void applySessionEvent(std::string_view kind);
    bool drawMap(ImDrawList* drawList, ImVec2 minimum, ImVec2 maximum);
    bool saveMapPng(const std::filesystem::path& path, std::string& error);
    void clearMap();
    // Complete recorded scans are integrated before partial assemblies reset.
    // Failed storage can discard its uncommitted queued scans explicitly.
    void resetSensorStream(bool preserveCompletedFrames = true);
    void resetLiveMap();
    void setConfig(mapping::SlamConfig config);

    bool hasMap() const { return mapper_.hasMap(); }
    std::size_t scanCount() const { return mapper_.scanCount(); }
    std::uint64_t droppedIncomingFrameCount() const {
        return droppedIncomingFrameCount_.load(std::memory_order_relaxed);
    }
    std::uint64_t rejectedLidarAnglePacketCount() const;
    std::uint64_t rejectedLidarTimestampPacketCount() const;
    std::uint64_t completedThermalFrameCount() const;
    std::uint64_t incompleteThermalFrameCount() const;
    mapping::ThermalFrame latestThermalFrame() const;
    mapping::ThermalScale thermalScale() const { return mapper_.thermalScale(); }
    float localizationScore() const { return mapper_.localizationScore(); }
    bool localizationTracking() const { return mapper_.localizationTracking(); }
    std::uint64_t rejectedScanCount() const { return mapper_.rejectedScanCount(); }
    mapping::Pose2D pose() const { return mapper_.pose(); }
    const mapping::SlamConfig& config() const { return mapper_.config(); }

private:
    mapping::SlamConfig config_;
    mapping::SlamMapper mapper_;
    mapping::SensorStreamDecoder streamDecoder_;
    mapping::ThermalFrame latestThermalFrame_{};
    ThermalMapRenderer renderer_;
    mutable std::mutex decoderMutex_;
    std::mutex incomingMutex_;
    std::deque<mapping::SensorFrame> incomingFrames_;
    std::atomic<std::uint64_t> droppedIncomingFrameCount_{0};

    void clearIncomingFrames();
    std::vector<mapping::SensorFrame> decodePacket(const Packet& packet);
};
