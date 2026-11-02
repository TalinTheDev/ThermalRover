#pragma once

#include "protocol/udp.hpp"

#include <chrono>
#include <optional>
#include <string_view>

namespace rover::desktop {
bool validCommandAcknowledgement(const COMMAND_ACK& acknowledgement);

class MotionCommandTracker {
public:
    using Clock = std::chrono::steady_clock;
    static constexpr auto acknowledgementTimeout = std::chrono::milliseconds(1500);
    enum class AckResult { Ignored, Accepted, Rejected };
    enum class StatusResult { Ignored, Running, Finished };

    bool begin(std::uint16_t commandId, Clock::time_point now);
    AckResult observeAcknowledgement(const COMMAND_ACK& acknowledgement);
    StatusResult observeStatus(const ESP_STATUS& status);
    bool acknowledgementTimedOut(Clock::time_point now) const;
    void reset();

    bool pending() const { return phase_ != Phase::Idle; }
    bool awaitingAcknowledgement() const { return phase_ == Phase::AwaitingAcknowledgement; }
    std::uint16_t commandId() const { return commandId_; }

private:
    enum class Phase { Idle, AwaitingAcknowledgement, Running };
    Phase phase_ = Phase::Idle;
    std::uint16_t commandId_ = 0;
    Clock::time_point requestedAt_{};
};

// STR_MSG has 128 bytes including its terminator. Reject rather than silently
// truncate, so the pending ping and echoed payload always contain the same text.
std::optional<STR_MSG> makeDebugMessagePayload(std::string_view text);
} // namespace rover::desktop
