#include "layers/slam_map.hpp"

#include <utility>

SlamMapLayer::SlamMapLayer(mapping::SlamConfig config)
    : config_(config), mapper_(config_) {
    mapper_.reset(config_.initialPose);
    renderer_.resetView(config_.initialPose.xCm, config_.initialPose.yCm);
}

void SlamMapLayer::update() {
    flushPendingFrames();
}

void SlamMapLayer::flushPendingFrames() {
    std::deque<mapping::SensorFrame> frames;
    {
        std::scoped_lock lock(incomingMutex_);
        frames.swap(incomingFrames_);
    }
    for(const mapping::SensorFrame& frame : frames) mapper_.integrate(frame);
}

void SlamMapLayer::render() {}
void SlamMapLayer::renderUI() {}

void SlamMapLayer::submitFrame(mapping::SensorFrame frame) {
    std::scoped_lock lock(incomingMutex_);
    constexpr std::size_t maximumQueuedFrames = 8;
    while(incomingFrames_.size() >= maximumQueuedFrames) {
        incomingFrames_.pop_front();
        droppedIncomingFrameCount_.fetch_add(1, std::memory_order_relaxed);
    }
    incomingFrames_.push_back(std::move(frame));
}

void SlamMapLayer::ingestPacket(const Packet& packet) {
    auto frames = decodePacket(packet);
    for(mapping::SensorFrame& frame : frames) submitFrame(std::move(frame));
}

std::vector<mapping::SensorFrame> SlamMapLayer::decodePacket(const Packet& packet) {
    std::scoped_lock lock(decoderMutex_);
    const auto completed = streamDecoder_.completedThermalFrameCount();
    auto frames = streamDecoder_.consume(packet);
    if(streamDecoder_.completedThermalFrameCount() != completed) {
        latestThermalFrame_ = streamDecoder_.latestThermalFrame();
        mapper_.observeThermalFrame(latestThermalFrame_);
    }
    return frames;
}

mapping::ThermalFrame SlamMapLayer::latestThermalFrame() const {
    std::scoped_lock lock(decoderMutex_);
    return latestThermalFrame_;
}

void SlamMapLayer::ingestReplayPacket(const Packet& packet) {
    for(const mapping::SensorFrame& frame : decodePacket(packet)) {
        mapper_.integrate(frame);
    }
}

void SlamMapLayer::applySessionEvent(std::string_view kind) {
    if(kind == "map_clear") {
        clearMap();
    } else if(kind == "recording_started") {
        resetSensorStream(false);
    } else if(kind == "recording_paused" || kind == "recording_resumed" ||
              kind == "recording_stopped") {
        resetSensorStream();
    }
}

std::uint64_t SlamMapLayer::rejectedLidarAnglePacketCount() const {
    std::scoped_lock lock(decoderMutex_);
    return streamDecoder_.rejectedLidarAnglePacketCount();
}

std::uint64_t SlamMapLayer::rejectedLidarTimestampPacketCount() const {
    std::scoped_lock lock(decoderMutex_);
    return streamDecoder_.rejectedLidarTimestampPacketCount();
}

std::uint64_t SlamMapLayer::completedThermalFrameCount() const {
    std::scoped_lock lock(decoderMutex_);
    return streamDecoder_.completedThermalFrameCount();
}

std::uint64_t SlamMapLayer::incompleteThermalFrameCount() const {
    std::scoped_lock lock(decoderMutex_);
    return streamDecoder_.incompleteThermalFrameCount();
}

bool SlamMapLayer::drawMap(ImDrawList* drawList, ImVec2 minimum, ImVec2 maximum) {
    return renderer_.draw(drawList, minimum, maximum, mapper_);
}

bool SlamMapLayer::saveMapPng(const std::filesystem::path& path,
                              std::string& error) {
    return renderer_.savePng(path, mapper_, error);
}

void SlamMapLayer::resetLiveMap() {
    clearIncomingFrames();
    {
        std::scoped_lock lock(decoderMutex_);
        streamDecoder_.reset();
        latestThermalFrame_ = {};
    }
    mapper_.reset(config_.initialPose);
    renderer_.resetView(config_.initialPose.xCm, config_.initialPose.yCm);
}

void SlamMapLayer::setConfig(mapping::SlamConfig config) {
    config_ = std::move(config);
    mapper_ = mapping::SlamMapper(config_);
    latestThermalFrame_ = {};
    clearIncomingFrames();
    {
        std::scoped_lock lock(decoderMutex_);
        streamDecoder_.reset();
    }
    renderer_.resetView(config_.initialPose.xCm, config_.initialPose.yCm);
}

void SlamMapLayer::clearMap() {
    clearIncomingFrames();
    {
        std::scoped_lock lock(decoderMutex_);
        streamDecoder_.reset();
    }
    mapper_.reset(config_.initialPose, true);
    renderer_.resetView(config_.initialPose.xCm, config_.initialPose.yCm);
}

void SlamMapLayer::resetSensorStream(bool preserveCompletedFrames) {
    if(preserveCompletedFrames) flushPendingFrames();
    else clearIncomingFrames();
    {
        std::scoped_lock lock(decoderMutex_);
        streamDecoder_.reset();
    }
}

void SlamMapLayer::clearIncomingFrames() {
    std::scoped_lock lock(incomingMutex_);
    incomingFrames_.clear();
}
