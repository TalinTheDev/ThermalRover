#pragma once

#include <cstdint>
#include <span>

namespace mapping {
struct ThermalScale {
    float meanC = 0;
    float lowC = -0.5f;
    float highC = 0.5f;
    std::uint64_t samples = 0;
};
struct ThermalRgb { float r, g, b; };

// Every valid recorded pixel contributes once, independent of localization,
// LiDAR density, or how often a thermal frame is reused for display.
class ThermalStatistics {
public:
    void reset();
    void observe(std::span<const float> temperatures);
    ThermalScale scale() const;
private:
    std::uint64_t count_ = 0;
    double mean_ = 0, squaredDeviation_ = 0;
};
ThermalRgb thermalColor(float temperatureC, const ThermalScale& scale);
} // namespace mapping
