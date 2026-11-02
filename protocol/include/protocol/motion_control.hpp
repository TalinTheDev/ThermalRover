#pragma once
#include "protocol/lidar_imu_odometry.hpp"
#include "protocol/udp.hpp"

namespace rover::control {
struct MotionPose {
    odometry::Pose pose{};
    bool translationValid = false, headingValid = false;
};
struct MotorOutput { int left = 0, right = 0; };
bool validMotionRequest(const PID_MOTION_CMD& command, const MotionPose& pose);
class MotionController {
public:
    bool start(const PID_MOTION_CMD& command, const MotionPose& pose, std::uint32_t now);
    void keepalive(std::uint16_t id, std::uint32_t now);
    void stop(MotionResult result = MOTION_RESULT_CANCELLED);
    MotorOutput update(const MotionPose& pose, std::uint32_t now);
    bool active() const { return active_; }
    MotionResult result() const { return result_; }
    std::uint16_t commandId() const { return command_.commandId; }
private:
    PID_MOTION_CMD command_{};
    odometry::Pose origin_{};
    float previousHeading_ = 0, measured_ = 0, integral_ = 0, bestError_ = 0;
    std::uint32_t started_ = 0, keepalive_ = 0, progressed_ = 0, updated_ = 0, duration_ = 0;
    bool active_ = false;
    MotionResult result_ = MOTION_RESULT_IDLE;
};
} // namespace rover::control
