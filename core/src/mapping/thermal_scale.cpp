#include "mapping/thermal_scale.hpp"

#include <algorithm>
#include <cmath>

namespace mapping {
void ThermalStatistics::reset() { count_ = 0; mean_ = squaredDeviation_ = 0; }
void ThermalStatistics::observe(std::span<const float> temperatures) {
    for(const float temperature : temperatures) {
        if(!std::isfinite(temperature) || temperature < -40 || temperature > 300) continue;
        ++count_;
        const double difference = temperature - mean_;
        mean_ += difference / static_cast<double>(count_);
        squaredDeviation_ += difference * (temperature - mean_);
    }
}
ThermalScale ThermalStatistics::scale() const {
    if(count_ == 0) return {};
    const float mean = static_cast<float>(mean_);
    const float deviation = static_cast<float>(std::sqrt(std::max(0.0, squaredDeviation_) /
                                                       static_cast<double>(count_)));
    // A quiet scene stays neutral rather than magnifying sensor noise.
    const float span = std::max(0.5f, 2.0f * deviation);
    return {mean, mean - span, mean + span, count_};
}
ThermalRgb thermalColor(float temperature, const ThermalScale& scale) {
    constexpr ThermalRgb cold{25, 115, 255}, neutral{224, 232, 240}, warm{255, 124, 42};
    const bool below = temperature < scale.meanC;
    const float span = std::max(0.01f, below ? scale.meanC - scale.lowC : scale.highC - scale.meanC);
    const float amount = std::clamp(std::fabs(temperature - scale.meanC) / span, 0.0f, 1.0f);
    const auto end = below ? cold : warm;
    return {neutral.r + (end.r - neutral.r) * amount,
            neutral.g + (end.g - neutral.g) * amount,
            neutral.b + (end.b - neutral.b) * amount};
}
} // namespace mapping
