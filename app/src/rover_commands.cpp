#include "rover_commands.hpp"

#include <cstring>

namespace rover::desktop {
bool validCommandAcknowledgement(const COMMAND_ACK& acknowledgement) {
    if(acknowledgement.commandId == 0 || acknowledgement.accepted > 1) return false;
    switch(acknowledgement.command) {
    case PAIR:
    case MOTION:
        return acknowledgement.state <= 1 &&
               acknowledgement.accepted == acknowledgement.state;
    case ARM:
        // ARM_DISARM can succeed while reporting disarmed. A rejected request
        // can also report an arm lease that was already active.
        return acknowledgement.state <= 1;
    case RECORDING:
        return acknowledgement.state <= 2;
    default:
        return false;
    }
}

bool MotionCommandTracker::begin(std::uint16_t commandId, Clock::time_point now) {
    if(commandId == 0 || pending()) return false;
    commandId_ = commandId;
    requestedAt_ = now;
    phase_ = Phase::AwaitingAcknowledgement;
    return true;
}

MotionCommandTracker::AckResult MotionCommandTracker::observeAcknowledgement(
    const COMMAND_ACK& acknowledgement) {
    if(!pending() || acknowledgement.command != MOTION ||
       acknowledgement.commandId != commandId_ ||
       !validCommandAcknowledgement(acknowledgement)) {
        return AckResult::Ignored;
    }
    if(acknowledgement.accepted == 0 && acknowledgement.state == 0) {
        reset();
        return AckResult::Rejected;
    }
    if(acknowledgement.accepted == 1 && acknowledgement.state == 1) {
        phase_ = Phase::Running;
        return AckResult::Accepted;
    }
    return AckResult::Ignored;
}

MotionCommandTracker::StatusResult MotionCommandTracker::observeStatus(const ESP_STATUS& status) {
    if(!pending() || status.motionCommandId != commandId_) return StatusResult::Ignored;
    if(status.motionActive == 1 && status.motionResult == MOTION_RESULT_RUNNING) {
        phase_ = Phase::Running;
        return StatusResult::Running;
    }
    if(status.motionActive == 0 && status.motionResult >= MOTION_RESULT_COMPLETE &&
       status.motionResult <= MOTION_RESULT_MOTOR_FAULT) {
        reset();
        return StatusResult::Finished;
    }
    return StatusResult::Ignored;
}

bool MotionCommandTracker::acknowledgementTimedOut(Clock::time_point now) const {
    return awaitingAcknowledgement() && now - requestedAt_ >= acknowledgementTimeout;
}

void MotionCommandTracker::reset() {
    phase_ = Phase::Idle;
    commandId_ = 0;
    requestedAt_ = {};
}

std::optional<STR_MSG> makeDebugMessagePayload(std::string_view text) {
    STR_MSG payload{};
    if(text.empty() || text.size() >= sizeof(payload.str) ||
       text.find('\0') != std::string_view::npos) return std::nullopt;
    std::memcpy(payload.str, text.data(), text.size());
    return payload;
}
} // namespace rover::desktop
