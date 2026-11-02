#pragma once

#include "mapping/slam.hpp"
#include "imgui.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

class ThermalMapRenderer {
public:
    static constexpr ImU32 canvasBackground = IM_COL32(9, 12, 15, 255);
    ThermalMapRenderer() = default;
    ~ThermalMapRenderer();

    ThermalMapRenderer(const ThermalMapRenderer&) = delete;
    ThermalMapRenderer& operator=(const ThermalMapRenderer&) = delete;

    bool draw(ImDrawList* drawList, ImVec2 minimum, ImVec2 maximum,
              const mapping::SlamMapper& mapper);
    bool savePng(const std::filesystem::path& path,
                 const mapping::SlamMapper& mapper, std::string& error);
    void resetView(float centerX = 0.0f, float centerY = 0.0f);

private:
    unsigned int texture_ = 0;
    bool textureAllocated_ = false;
    std::uint64_t uploadedRevision_ = static_cast<std::uint64_t>(-1);
    std::vector<std::uint8_t> rgba_;
    float cameraCenterX_ = 0.0f;
    float cameraCenterY_ = 0.0f;
    float zoom_ = 1.8f;
    bool panning_ = false;

    void ensureTexture();
    void updateTexture(const mapping::SlamMapper& mapper);
    void drawRobotMarker(ImDrawList* drawList, ImVec2 minimum, ImVec2 maximum,
                         const mapping::SlamMapper& mapper, float viewLeft,
                         float viewTop, float viewWidth, float viewHeight) const;
};
