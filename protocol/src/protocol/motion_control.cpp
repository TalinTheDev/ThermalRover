#include "protocol/motion_control.hpp"
#include <algorithm>
#include <cmath>

namespace rover::control {
namespace {
constexpr float radiansToDegrees = 57.29577951308232f;
constexpr float distanceToleranceCm = 0.7f, angleToleranceDegrees = 2.0f;
}
bool validMotionRequest(const PID_MOTION_CMD& command, const MotionPose& pose) {
    if(command.commandId == 0 || !pose.headingValid ||
       (command.kind != DISTANCE_CM && command.kind != ANGLE_DEGREES) ||
       (command.kind == DISTANCE_CM && !pose.translationValid) ||
       !std::isfinite(command.target) || !std::isfinite(command.kp) ||
       !std::isfinite(command.ki) || !std::isfinite(command.kd) ||
       command.maxSpeed < 20 || command.kp < 0 || command.kp > 100 ||
       command.ki < 0 || command.ki > 50 || command.kd < 0 || command.kd > 100 ||
       (command.kp == 0 && command.ki == 0 && command.kd == 0)) return false;
    const float target = std::fabs(command.target);
    return command.kind == DISTANCE_CM
        ? target >= distanceToleranceCm && target <= 500
        : target >= angleToleranceDegrees && target <= 720;
}
bool MotionController::start(const PID_MOTION_CMD& command, const MotionPose& pose, std::uint32_t now) {
    command_ = command;
    active_ = validMotionRequest(command, pose);
    result_ = active_ ? MOTION_RESULT_RUNNING : MOTION_RESULT_SENSOR_FAULT;
    if(!active_) return false;
    origin_ = pose.pose;
    previousHeading_ = origin_.headingRadians;
    measured_ = integral_ = 0;
    bestError_ = std::fabs(command.target);
    started_ = keepalive_ = progressed_ = updated_ = now;
    duration_ = static_cast<std::uint32_t>(std::clamp(
        10.0f + std::fabs(command.target) / (command.kind == DISTANCE_CM ? 1.0f : 10.0f),
        10.0f, 120.0f) * 1000);
    return true;
}
void MotionController::keepalive(std::uint16_t id, std::uint32_t now) {
    if(active_ && id == command_.commandId) keepalive_ = now;
}
void MotionController::stop(MotionResult result) {
    if(active_) result_ = result;
    active_ = false;
}
MotorOutput MotionController::update(const MotionPose& pose, std::uint32_t now) {
    if(!active_) return {};
    if(!pose.headingValid || (command_.kind == DISTANCE_CM && !pose.translationValid)) {
        stop(MOTION_RESULT_SENSOR_FAULT); return {};
    }
    if(now - keepalive_ >= 750U || now - started_ >= duration_) {
        stop(MOTION_RESULT_TIMEOUT); return {};
    }
    const float previous = measured_;
    const float dt = std::clamp(static_cast<float>(now - updated_) * 0.001f, 0.001f, 0.2f);
    updated_ = now;
    if(command_.kind == DISTANCE_CM) {
        measured_ = (pose.pose.xCm - origin_.xCm) * std::sin(origin_.headingRadians) +
                    (pose.pose.yCm - origin_.yCm) * std::cos(origin_.headingRadians);
    } else {
        measured_ += odometry::wrap(pose.pose.headingRadians - previousHeading_) * radiansToDegrees;
    }
    previousHeading_ = pose.pose.headingRadians;
    const float error = command_.target - measured_;
    const float tolerance = command_.kind == DISTANCE_CM ? distanceToleranceCm : angleToleranceDegrees;
    if(std::fabs(error) <= tolerance) { stop(MOTION_RESULT_COMPLETE); return {}; }
    const float progressThreshold = command_.kind == DISTANCE_CM ? 0.1f : 0.5f;
    if(bestError_ - std::fabs(error) >= progressThreshold) {
        bestError_ = std::fabs(error); progressed_ = now;
    }
    if(now - progressed_ >= 1500U) { stop(MOTION_RESULT_STALLED); return {}; }
    const float maximum = static_cast<float>(command_.maxSpeed);
    const float candidateIntegral = std::clamp(integral_ + error * dt, -200.0f, 200.0f);
    float output = command_.kp * error + command_.ki * candidateIntegral -
                   command_.kd * (measured_ - previous) / dt;
    if(std::fabs(output) < maximum || output * error < 0) integral_ = candidateIntegral;
    output = std::clamp(output, -maximum, maximum);
    const float minimum = std::min(25.0f, maximum);
    if(std::fabs(output) < minimum) output = std::copysign(minimum, error);
    if(command_.kind == ANGLE_DEGREES) {
        const int pwm = static_cast<int>(std::lround(output));
        return {pwm, -pwm};
    }
    const float headingError = odometry::wrap(origin_.headingRadians - pose.pose.headingRadians);
    const float trim = std::clamp(headingError * radiansToDegrees * 2.0f, -maximum * 0.35f, maximum * 0.35f);
    return {static_cast<int>(std::lround(std::clamp(output + trim, -maximum, maximum))),
            static_cast<int>(std::lround(std::clamp(output - trim, -maximum, maximum)))};
}
} // namespace rover::control
