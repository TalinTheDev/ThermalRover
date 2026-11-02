#define GLFW_INCLUDE_NONE
#include <glad/gl.h>

#include "renderer/thermal_map_renderer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <format>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace {
using Rgb = mapping::ThermalRgb;

std::uint8_t channel(float value) {
    return static_cast<std::uint8_t>(std::clamp(value, 0.0f, 255.0f));
}


void putPixel(std::vector<std::uint8_t>& pixels, std::size_t width,
              std::size_t height, int x, int y,
              std::array<std::uint8_t, 4> color) {
    if(x < 0 || y < 0 || x >= static_cast<int>(width) ||
       y >= static_cast<int>(height)) return;
    const std::size_t offset =
        (static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)) * 4;
    std::copy(color.begin(), color.end(), pixels.begin() +
              static_cast<std::ptrdiff_t>(offset));
}

void fillRect(std::vector<std::uint8_t>& pixels, std::size_t width,
              std::size_t height, int x, int y, int rectWidth, int rectHeight,
              std::array<std::uint8_t, 4> color) {
    const int right = std::min(x + rectWidth, static_cast<int>(width));
    const int bottom = std::min(y + rectHeight, static_cast<int>(height));
    for(int pixelY = std::max(y, 0); pixelY < bottom; ++pixelY) {
        for(int pixelX = std::max(x, 0); pixelX < right; ++pixelX) {
            putPixel(pixels, width, height, pixelX, pixelY, color);
        }
    }
}

std::array<std::uint8_t, 7> glyph(char character) {
    switch(character) {
    case '.': return {0,0,0,0,0,0,4};
    case '-': return {0,0,0,31,0,0,0};
    case '0': return {14,17,19,21,25,17,14};
    case '1': return {4,12,4,4,4,4,14};
    case '2': return {14,17,1,2,4,8,31};
    case '3': return {30,1,1,14,1,1,30};
    case '4': return {2,6,10,18,31,2,2};
    case '5': return {31,16,30,1,1,17,14};
    case '6': return {14,16,16,30,17,17,14};
    case '7': return {31,1,2,4,8,8,8};
    case '8': return {14,17,17,14,17,17,14};
    case '9': return {14,17,17,15,1,2,12};
    case 'A': return {14,17,17,31,17,17,17};
    case 'C': return {14,17,16,16,16,17,14};
    case 'D': return {30,17,17,17,17,17,30};
    case 'E': return {31,16,16,30,16,16,31};
    case 'F': return {31,16,16,30,16,16,16};
    case 'H': return {17,17,17,31,17,17,17};
    case 'I': return {14,4,4,4,4,4,14};
    case 'K': return {17,18,20,24,20,18,17};
    case 'L': return {16,16,16,16,16,16,31};
    case 'M': return {17,27,21,21,17,17,17};
    case 'N': return {17,25,25,21,19,19,17};
    case 'O': return {14,17,17,17,17,17,14};
    case 'P': return {30,17,17,30,16,16,16};
    case 'R': return {30,17,17,30,20,18,17};
    case 'T': return {31,4,4,4,4,4,4};
    case 'U': return {17,17,17,17,17,17,14};
    case 'V': return {17,17,17,17,17,10,4};
    case 'W': return {17,17,17,21,21,21,10};
    default: return {};
    }
}

void drawText(std::vector<std::uint8_t>& pixels, std::size_t width,
              std::size_t height, int x, int y, std::string_view text,
              std::array<std::uint8_t, 4> color, int scale = 1) {
    int cursorX = x;
    for(const char character : text) {
        const auto pattern = glyph(character);
        if(character != ' ') {
            for(int row = 0; row < 7; ++row) {
                for(int column = 0; column < 5; ++column) {
                    if((pattern[row] & (1U << (4 - column))) == 0) continue;
                    fillRect(pixels, width, height,
                             cursorX + column * scale, y + row * scale,
                             scale, scale, color);
                }
            }
        }
        cursorX += 6 * scale;
    }
}

void appendPngText(std::vector<std::uint8_t>& data, std::string_view keyword,
                   std::string_view value) {
    data.insert(data.end(), keyword.begin(), keyword.end());
    data.push_back(0);
    data.insert(data.end(), value.begin(), value.end());
}

std::uint32_t crc32(std::span<const std::uint8_t> bytes) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for(const std::uint8_t byte : bytes) {
        crc ^= byte;
        for(int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

void appendBigEndian32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value >> 24U));
    bytes.push_back(static_cast<std::uint8_t>(value >> 16U));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8U));
    bytes.push_back(static_cast<std::uint8_t>(value));
}

bool writePngChunk(std::ofstream& output, const std::array<char, 4>& type,
                   std::span<const std::uint8_t> data) {
    if(data.size() > std::numeric_limits<std::uint32_t>::max()) return false;
    std::vector<std::uint8_t> checksumInput;
    checksumInput.reserve(type.size() + data.size());
    for(const char value : type) checksumInput.push_back(static_cast<std::uint8_t>(value));
    checksumInput.insert(checksumInput.end(), data.begin(), data.end());
    std::vector<std::uint8_t> header;
    appendBigEndian32(header, static_cast<std::uint32_t>(data.size()));
    const std::uint32_t checksum = crc32(checksumInput);
    std::vector<std::uint8_t> footer;
    appendBigEndian32(footer, checksum);
    output.write(reinterpret_cast<const char*>(header.data()),
                 static_cast<std::streamsize>(header.size()));
    output.write(type.data(), static_cast<std::streamsize>(type.size()));
    if(!data.empty()) {
        output.write(reinterpret_cast<const char*>(data.data()),
                     static_cast<std::streamsize>(data.size()));
    }
    output.write(reinterpret_cast<const char*>(footer.data()),
                 static_cast<std::streamsize>(footer.size()));
    return static_cast<bool>(output);
}

struct MapView {
    float width = 0.0f;
    float height = 0.0f;
};

MapView calculateView(const mapping::SlamConfig& config, float pixelWidth,
                      float pixelHeight, float zoom) {
    const float mapWidth = static_cast<float>(config.mapWidthCells) * config.centimetersPerCell;
    const float mapHeight = static_cast<float>(config.mapHeightCells) * config.centimetersPerCell;
    const float aspect = pixelWidth / pixelHeight;
    const float baseSpan = std::min(mapWidth, mapHeight) *
                           std::clamp(config.displayMapFraction, 0.05f, 1.0f) /
                           std::max(zoom, 0.01f);
    float viewWidth = baseSpan * std::max(1.0f, aspect);
    float viewHeight = baseSpan * std::max(1.0f, 1.0f / aspect);
    const float fitScale = std::min({1.0f, mapWidth / viewWidth, mapHeight / viewHeight});
    viewWidth *= fitScale;
    viewHeight *= fitScale;
    return {viewWidth, viewHeight};
}

void clampCamera(float& centerX, float& centerY, float mapWidth, float mapHeight,
                 float viewWidth, float viewHeight) {
    if(viewWidth >= mapWidth) centerX = 0.0f;
    else centerX = std::clamp(centerX, -mapWidth * 0.5f + viewWidth * 0.5f,
                              mapWidth * 0.5f - viewWidth * 0.5f);
    if(viewHeight >= mapHeight) centerY = 0.0f;
    else centerY = std::clamp(centerY, -mapHeight * 0.5f + viewHeight * 0.5f,
                              mapHeight * 0.5f - viewHeight * 0.5f);
}

std::vector<std::uint8_t> rasterizeMap(const mapping::SlamMapper& mapper) {
    const auto& config = mapper.config();
    const std::size_t cellCount = config.mapWidthCells * config.mapHeightCells;
    std::vector<std::uint8_t> pixels(cellCount * 4);
    const auto occupancy = mapper.occupancy();
    const auto observed = mapper.observed();
    const auto thermal = mapper.thermalAverageC();
    const auto thermalCount = mapper.thermalSampleCount();
    const auto scale = mapper.thermalScale();
    for(std::size_t i = 0; i < cellCount; ++i) {
        const std::size_t pixel = i * 4;
        if(thermalCount[i] > 0) {
            const Rgb color = mapping::thermalColor(thermal[i], scale);
            pixels[pixel] = channel(color.r);
            pixels[pixel + 1] = channel(color.g);
            pixels[pixel + 2] = channel(color.b);
            pixels[pixel + 3] = 242;
        } else if(occupancy[i] >= 3) {
            const float shade = 104.0f +
                std::min(static_cast<float>(occupancy[i]), 40.0f) * 0.8f;
            pixels[pixel] = channel(shade);
            pixels[pixel + 1] = channel(shade + 5.0f);
            pixels[pixel + 2] = channel(shade + 11.0f);
            pixels[pixel + 3] = 255;
        } else if(observed[i] != 0) {
            pixels[pixel] = 98;
            pixels[pixel + 1] = 108;
            pixels[pixel + 2] = 120;
            pixels[pixel + 3] = 28;
        }
    }
    return pixels;
}
} // namespace

ThermalMapRenderer::~ThermalMapRenderer() {
    if(texture_ != 0) glDeleteTextures(1, &texture_);
}

void ThermalMapRenderer::ensureTexture() {
    if(texture_ != 0) return;
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void ThermalMapRenderer::resetView(float centerX, float centerY) {
    cameraCenterX_ = centerX;
    cameraCenterY_ = centerY;
    zoom_ = 1.8f;
    panning_ = false;
    // A new mapper may restart its revision counter or change map dimensions.
    // Reallocate the existing GL texture on the next draw or PNG export.
    textureAllocated_ = false;
    uploadedRevision_ = static_cast<std::uint64_t>(-1);
}

void ThermalMapRenderer::updateTexture(const mapping::SlamMapper& mapper) {
    if(uploadedRevision_ == mapper.revision()) return;
    const mapping::SlamConfig& config = mapper.config();
    rgba_ = rasterizeMap(mapper);

    ensureTexture();
    glBindTexture(GL_TEXTURE_2D, texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    if(!textureAllocated_) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
                     static_cast<GLsizei>(config.mapWidthCells),
                     static_cast<GLsizei>(config.mapHeightCells), 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, rgba_.data());
        textureAllocated_ = true;
    } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                        static_cast<GLsizei>(config.mapWidthCells),
                        static_cast<GLsizei>(config.mapHeightCells),
                        GL_RGBA, GL_UNSIGNED_BYTE, rgba_.data());
    }
    uploadedRevision_ = mapper.revision();
}

bool ThermalMapRenderer::savePng(const std::filesystem::path& path,
                                 const mapping::SlamMapper& mapper,
                                 std::string& error) {
    const mapping::SlamConfig& config = mapper.config();
    constexpr std::size_t footerHeight = 48;
    if(config.mapWidthCells == 0 || config.mapHeightCells == 0 ||
       config.mapWidthCells > std::numeric_limits<int>::max() ||
       config.mapHeightCells > std::numeric_limits<int>::max() ||
       config.mapWidthCells > std::numeric_limits<std::uint32_t>::max() ||
       config.mapHeightCells > std::numeric_limits<std::uint32_t>::max() - footerHeight ||
       !std::isfinite(config.centimetersPerCell) ||
       config.centimetersPerCell <= 0.0f) {
        error = "Map dimensions cannot be exported as PNG.";
        return false;
    }

    const std::size_t width = config.mapWidthCells;
    const std::size_t height = config.mapHeightCells;
    const std::size_t outputHeight = height + footerHeight;
    if(width > std::numeric_limits<std::size_t>::max() / 4 ||
       outputHeight > std::numeric_limits<std::size_t>::max() / (width * 4)) {
        error = "Map image is too large to export.";
        return false;
    }
    // Export uses the same pixels as the GL texture but needs no GL context.
    const auto rgba = rasterizeMap(mapper);
    const std::size_t rowBytes = width * 4;
    std::vector<std::uint8_t> image(width * outputHeight * 4);
    constexpr std::array<std::uint8_t, 4> background{14, 20, 27, 255};
    for(std::size_t pixel = 0; pixel < width * outputHeight; ++pixel) {
        std::copy(background.begin(), background.end(), image.begin() +
                  static_cast<std::ptrdiff_t>(pixel * 4));
    }
    for(std::size_t outputY = 0; outputY < height; ++outputY) {
        const std::size_t sourceY = height - 1 - outputY;
        const std::size_t sourceOffset = sourceY * rowBytes;
        const std::size_t targetOffset = outputY * rowBytes;
        for(std::size_t x = 0; x < width; ++x) {
            const std::size_t source = sourceOffset + x * 4;
            const std::size_t target = targetOffset + x * 4;
            const unsigned int alpha = rgba[source + 3];
            for(std::size_t color = 0; color < 3; ++color) {
                image[target + color] = static_cast<std::uint8_t>(
                    (static_cast<unsigned int>(rgba[source + color]) * alpha +
                     static_cast<unsigned int>(background[color]) * (255U - alpha)) /
                    255U);
            }
            image[target + 3] = 255;
        }
    }

    const int imageWidth = static_cast<int>(width);
    const int footerY = static_cast<int>(height);
    fillRect(image, width, outputHeight, 0, footerY, imageWidth,
             static_cast<int>(footerHeight), {25, 33, 42, 255});
    constexpr std::array<std::uint8_t, 4> legendText{224, 232, 240, 255};
    drawText(image, width, outputHeight, 8, footerY + 6, "THERMAL C", legendText);

    const int gradientX = (std::min)(74, imageWidth / 4);
    const int gradientWidth = (std::max)(1, (std::min)(170, imageWidth - gradientX - 8));
    const auto thermalScale = mapper.thermalScale();
    for(int x = 0; x < gradientWidth; ++x) {
        const float fraction = gradientWidth > 1
            ? static_cast<float>(x) / static_cast<float>(gradientWidth - 1) : 0.0f;
        const Rgb color = mapping::thermalColor(thermalScale.lowC + fraction *
            (thermalScale.highC - thermalScale.lowC), thermalScale);
        fillRect(image, width, outputHeight, gradientX + x, footerY + 5, 1, 9,
                 {channel(color.r), channel(color.g), channel(color.b), 255});
    }
    drawText(image, width, outputHeight, gradientX, footerY + 18,
             std::format("{:.1f}", thermalScale.lowC), legendText);
    drawText(image, width, outputHeight, gradientX + gradientWidth / 2 - 12, footerY + 18,
             std::format("{:.1f}", thermalScale.meanC), legendText);
    drawText(image, width, outputHeight,
             (std::max)(gradientX, gradientX + gradientWidth - 24),
             footerY + 18, std::format("{:.1f}", thermalScale.highC), legendText);

    fillRect(image, width, outputHeight, 8, footerY + 34, 10, 8,
             {14, 20, 27, 255});
    fillRect(image, width, outputHeight, 8, footerY + 34, 10, 1,
             {105, 118, 132, 255});
    drawText(image, width, outputHeight, 23, footerY + 34, "UNKNOWN", legendText);
    fillRect(image, width, outputHeight, 83, footerY + 34, 10, 8,
             {105, 118, 132, 255});
    drawText(image, width, outputHeight, 98, footerY + 34, "FREE", legendText);
    fillRect(image, width, outputHeight, 140, footerY + 34, 10, 8,
             {139, 148, 160, 255});
    drawText(image, width, outputHeight, 155, footerY + 34, "OCCUPIED", legendText);

    const mapping::Pose2D pose = mapper.pose();
    const int robotCellX = static_cast<int>(std::floor(
        pose.xCm / config.centimetersPerCell + static_cast<float>(width) * 0.5f));
    const int robotCellY = static_cast<int>(std::floor(
        pose.yCm / config.centimetersPerCell + static_cast<float>(height) * 0.5f));
    if(robotCellX >= 0 && robotCellY >= 0 &&
       robotCellX < imageWidth && robotCellY < static_cast<int>(height)) {
        const int robotImageY = static_cast<int>(height) - 1 - robotCellY;
        fillRect(image, width, outputHeight, robotCellX - 3, robotImageY - 3,
                 7, 7, {52, 191, 255, 255});
        const int forwardX = robotCellX + static_cast<int>(std::lround(
            std::sin(pose.headingRadiansClockwise) * 9.0f));
        const int forwardY = robotImageY - static_cast<int>(std::lround(
            std::cos(pose.headingRadiansClockwise) * 9.0f));
        const int steps = (std::max)(std::abs(forwardX - robotCellX),
                                     std::abs(forwardY - robotImageY));
        for(int step = 0; step <= steps; ++step) {
            const float fraction = steps > 0
                ? static_cast<float>(step) / static_cast<float>(steps) : 0.0f;
            putPixel(image, width, outputHeight,
                static_cast<int>(std::lround(robotCellX + (forwardX - robotCellX) * fraction)),
                static_cast<int>(std::lround(robotImageY + (forwardY - robotImageY) * fraction)),
                {52, 191, 255, 255});
        }
    }
    fillRect(image, width, outputHeight, 214, footerY + 34, 8, 8,
             {52, 191, 255, 255});
    drawText(image, width, outputHeight, 227, footerY + 34, "ROVER", legendText);

    const int scaleMeters = static_cast<int>(std::floor(
        static_cast<double>(width) * config.centimetersPerCell * 0.25 / 100.0));
    const int scalePixels = static_cast<int>(std::lround(
        static_cast<double>(scaleMeters) * 100.0 / config.centimetersPerCell));
    const int scaleX = imageWidth - scalePixels - 10;
    if(scaleMeters > 0 && scalePixels > 0 && scalePixels < imageWidth / 2 &&
       scaleX > 250) {
        drawText(image, width, outputHeight, scaleX, footerY + 20,
                 std::format("{} M", scaleMeters), legendText);
        fillRect(image, width, outputHeight, scaleX, footerY + 34,
                 scalePixels, 3, {224, 232, 240, 255});
        fillRect(image, width, outputHeight, scaleX, footerY + 32,
                 2, 7, {224, 232, 240, 255});
        fillRect(image, width, outputHeight, scaleX + scalePixels - 2,
                 footerY + 32, 2, 7, {224, 232, 240, 255});
    }

    std::vector<std::uint8_t> raw((rowBytes + 1) * outputHeight);
    for(std::size_t outputY = 0; outputY < outputHeight; ++outputY) {
        const std::size_t rowStart = outputY * (rowBytes + 1);
        raw[rowStart] = 0; // PNG filter: None.
        std::copy_n(image.begin() + static_cast<std::ptrdiff_t>(outputY * rowBytes),
                    rowBytes, raw.begin() + static_cast<std::ptrdiff_t>(rowStart + 1));
    }

    std::vector<std::uint8_t> compressed{0x78, 0x01}; // zlib / no compression.
    std::size_t offset = 0;
    while(offset < raw.size()) {
        const std::size_t blockBytes = std::min<std::size_t>(65535, raw.size() - offset);
        const bool finalBlock = offset + blockBytes == raw.size();
        compressed.push_back(finalBlock ? 0x01 : 0x00); // Stored DEFLATE block.
        const auto length = static_cast<std::uint16_t>(blockBytes);
        const auto inverseLength = static_cast<std::uint16_t>(~length);
        compressed.push_back(static_cast<std::uint8_t>(length));
        compressed.push_back(static_cast<std::uint8_t>(length >> 8U));
        compressed.push_back(static_cast<std::uint8_t>(inverseLength));
        compressed.push_back(static_cast<std::uint8_t>(inverseLength >> 8U));
        compressed.insert(compressed.end(),
                          raw.begin() + static_cast<std::ptrdiff_t>(offset),
                          raw.begin() + static_cast<std::ptrdiff_t>(offset + blockBytes));
        offset += blockBytes;
    }

    std::uint32_t adlerA = 1;
    std::uint32_t adlerB = 0;
    for(const std::uint8_t byte : raw) {
        adlerA = (adlerA + byte) % 65521U;
        adlerB = (adlerB + adlerA) % 65521U;
    }
    appendBigEndian32(compressed, (adlerB << 16U) | adlerA);

    std::error_code directoryError;
    if(!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), directoryError);
        if(directoryError) {
            error = "Could not create the PNG directory: " + directoryError.message();
            return false;
        }
    }
    std::ofstream output(path, std::ios::binary);
    if(!output) {
        error = "Could not open the PNG output file.";
        return false;
    }
    constexpr std::array<std::uint8_t, 8> signature{
        137, 80, 78, 71, 13, 10, 26, 10};
    output.write(reinterpret_cast<const char*>(signature.data()),
                 static_cast<std::streamsize>(signature.size()));

    std::vector<std::uint8_t> header;
    appendBigEndian32(header, static_cast<std::uint32_t>(width));
    appendBigEndian32(header, static_cast<std::uint32_t>(outputHeight));
    header.insert(header.end(), {8, 6, 0, 0, 0}); // 8-bit RGBA.
    std::vector<std::uint8_t> titleText;
    appendPngText(titleText, "Title", "Thermal Rover SLAM map");
    std::vector<std::uint8_t> scaleText;
    appendPngText(scaleText, "CellSizeCm",
                  std::format("{:.4f}", config.centimetersPerCell));
    std::vector<std::uint8_t> poseText;
    appendPngText(poseText, "RobotPoseCmClockwise",
        std::format("{:.3f},{:.3f},{:.3f}", pose.xCm, pose.yCm,
                    pose.headingRadiansClockwise * 180.0f /
                        3.14159265358979323846f));
    std::vector<std::uint8_t> thermalScaleText;
    appendPngText(thermalScaleText, "ThermalScaleC",
                 std::format("{:.6f},{:.6f},{:.6f}", thermalScale.lowC,
                             thermalScale.meanC, thermalScale.highC));
    const bool chunksWritten =
        writePngChunk(output, {'I', 'H', 'D', 'R'}, header) &&
        writePngChunk(output, {'t', 'E', 'X', 't'}, titleText) &&
        writePngChunk(output, {'t', 'E', 'X', 't'}, scaleText) &&
        writePngChunk(output, {'t', 'E', 'X', 't'}, poseText) &&
        writePngChunk(output, {'t', 'E', 'X', 't'}, thermalScaleText) &&
        writePngChunk(output, {'I', 'D', 'A', 'T'}, compressed) &&
        writePngChunk(output, {'I', 'E', 'N', 'D'}, {});
    output.flush();
    if(!chunksWritten || !output) {
        error = "Writing the PNG map failed.";
        return false;
    }
    error.clear();
    return true;
}

bool ThermalMapRenderer::draw(ImDrawList* drawList, ImVec2 minimum, ImVec2 maximum,
                              const mapping::SlamMapper& mapper) {
    if(drawList == nullptr) return false;
    updateTexture(mapper);
    if(texture_ == 0) return false;

    const mapping::SlamConfig& config = mapper.config();
    const float mapWidth = static_cast<float>(config.mapWidthCells) * config.centimetersPerCell;
    const float mapHeight = static_cast<float>(config.mapHeightCells) * config.centimetersPerCell;
    const float pixelWidth = maximum.x - minimum.x;
    const float pixelHeight = maximum.y - minimum.y;
    if(mapWidth <= 0.0f || mapHeight <= 0.0f || pixelWidth <= 0.0f || pixelHeight <= 0.0f) {
        return false;
    }

    const bool hovered = ImGui::IsMouseHoveringRect(minimum, maximum, true);
    MapView view = calculateView(config, pixelWidth, pixelHeight, zoom_);
    if(hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) panning_ = true;
    if(!ImGui::IsMouseDown(ImGuiMouseButton_Left)) panning_ = false;
    if(panning_) {
        const ImVec2 mouseDelta = ImGui::GetIO().MouseDelta;
        cameraCenterX_ -= mouseDelta.x * view.width / pixelWidth;
        cameraCenterY_ += mouseDelta.y * view.height / pixelHeight;
        clampCamera(cameraCenterX_, cameraCenterY_, mapWidth, mapHeight,
                    view.width, view.height);
    }

    if(hovered && ImGui::GetIO().MouseWheel != 0.0f) {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const float cursorX = std::clamp((mouse.x - minimum.x) / pixelWidth, 0.0f, 1.0f);
        const float cursorY = std::clamp((mouse.y - minimum.y) / pixelHeight, 0.0f, 1.0f);
        const float oldLeft = cameraCenterX_ - view.width * 0.5f;
        const float oldTop = cameraCenterY_ + view.height * 0.5f;
        const float worldX = oldLeft + cursorX * view.width;
        const float worldY = oldTop - cursorY * view.height;
        zoom_ = std::clamp(zoom_ * std::pow(1.2f, ImGui::GetIO().MouseWheel),
                           1.0f, 8.0f);
        view = calculateView(config, pixelWidth, pixelHeight, zoom_);
        cameraCenterX_ = worldX - (cursorX - 0.5f) * view.width;
        cameraCenterY_ = worldY + (cursorY - 0.5f) * view.height;
        clampCamera(cameraCenterX_, cameraCenterY_, mapWidth, mapHeight,
                    view.width, view.height);
    }

    const float viewWidth = view.width;
    const float viewHeight = view.height;
    const float left = cameraCenterX_ - viewWidth * 0.5f;
    const float bottom = cameraCenterY_ - viewHeight * 0.5f;
    const float right = left + viewWidth;
    const float top = bottom + viewHeight;
    const float uLeft = left / mapWidth + 0.5f;
    const float uRight = right / mapWidth + 0.5f;
    const float vTop = top / mapHeight + 0.5f;
    const float vBottom = bottom / mapHeight + 0.5f;
    drawList->AddRectFilled(minimum, maximum, canvasBackground);
    drawList->AddImage(static_cast<ImTextureID>(texture_), minimum, maximum,
                       {uLeft, vTop}, {uRight, vBottom});

    constexpr float gridCentimeters = 100.0f;
    const ImU32 gridColor = IM_COL32(218, 226, 235, 21);
    for(float x = std::ceil(left / gridCentimeters) * gridCentimeters;
        x <= right; x += gridCentimeters) {
        const float screenX = minimum.x + (x - left) / viewWidth * pixelWidth;
        drawList->AddLine({screenX, minimum.y}, {screenX, maximum.y}, gridColor);
    }
    for(float y = std::ceil(bottom / gridCentimeters) * gridCentimeters;
        y <= top; y += gridCentimeters) {
        const float screenY = minimum.y + (top - y) / viewHeight * pixelHeight;
        drawList->AddLine({minimum.x, screenY}, {maximum.x, screenY}, gridColor);
    }
    if(left <= 0.0f && right >= 0.0f) {
        const float axisX = minimum.x + (0.0f - left) / viewWidth * pixelWidth;
        drawList->AddLine({axisX, minimum.y}, {axisX, maximum.y}, IM_COL32(225, 232, 240, 36));
    }
    if(bottom <= 0.0f && top >= 0.0f) {
        const float axisY = minimum.y + (top - 0.0f) / viewHeight * pixelHeight;
        drawList->AddLine({minimum.x, axisY}, {maximum.x, axisY}, IM_COL32(225, 232, 240, 36));
    }
    drawRobotMarker(drawList, minimum, maximum, mapper, left, top,
                    viewWidth, viewHeight);
    return true;
}

void ThermalMapRenderer::drawRobotMarker(ImDrawList* drawList, ImVec2 minimum,
                                         ImVec2 maximum,
                                         const mapping::SlamMapper& mapper,
                                         float viewLeft, float viewTop,
                                         float viewWidth, float viewHeight) const {
    const float pixelWidth = maximum.x - minimum.x;
    const float pixelHeight = maximum.y - minimum.y;
    if(viewWidth <= 0.0f || viewHeight <= 0.0f ||
       pixelWidth <= 0.0f || pixelHeight <= 0.0f) return;
    const mapping::Pose2D pose = mapper.pose();
    const ImVec2 center{
        minimum.x + (pose.xCm - viewLeft) / viewWidth * pixelWidth,
        minimum.y + (viewTop - pose.yCm) / viewHeight * pixelHeight
    };
    if(center.x < minimum.x || center.x > maximum.x ||
       center.y < minimum.y || center.y > maximum.y) return;

    const auto& mounts = mapper.config().mounts;
    const float c = std::cos(pose.headingRadiansClockwise), s = std::sin(pose.headingRadiansClockwise);
    const float cameraX = pose.xCm + c * mounts.thermal.xCm + s * mounts.thermal.yCm;
    const float cameraY = pose.yCm - s * mounts.thermal.xCm + c * mounts.thermal.yCm;
    const float cameraHeading = pose.headingRadiansClockwise + mounts.thermal.headingRadiansClockwise;
    const float halfFov = mapping::thermalHorizontalFov(mounts) * 3.14159265358979323846f / 360;
    const float radius = std::clamp(viewWidth * 0.15f, 50.0f, 200.0f);
    const auto screen = [&](float x, float y) {
        return ImVec2{minimum.x + (x - viewLeft) / viewWidth * pixelWidth,
                      minimum.y + (viewTop - y) / viewHeight * pixelHeight};
    };
    std::array<ImVec2, 18> fan{};
    fan[0] = screen(cameraX, cameraY);
    for(int i = 0; i <= 16; ++i) {
        const float angle = cameraHeading - halfFov + 2 * halfFov * static_cast<float>(i) / 16;
        fan[static_cast<std::size_t>(i) + 1] = screen(cameraX + std::sin(angle) * radius,
                                                     cameraY + std::cos(angle) * radius);
    }
    drawList->PushClipRect(minimum, maximum, true);
    drawList->AddConvexPolyFilled(fan.data(), static_cast<int>(fan.size()), IM_COL32(225, 232, 240, 7));
    drawList->AddPolyline(fan.data() + 1, 17, IM_COL32(225, 232, 240, 55), 1.0f, ImDrawFlags_None);
    drawList->AddLine(fan[0], fan[1], IM_COL32(225, 232, 240, 75));
    drawList->AddLine(fan[0], fan.back(), IM_COL32(225, 232, 240, 75));
    const auto label = screen(cameraX + std::sin(cameraHeading) * radius * 0.65f,
                              cameraY + std::cos(cameraHeading) * radius * 0.65f);
    drawList->AddText(ImGui::GetFont(), 12.0f, label, IM_COL32(225, 232, 240, 145), "IR");
    drawList->PopClipRect();

    const float markerRadius = std::clamp(pixelWidth / viewWidth * 0.035f, 4.0f, 6.0f);
    const ImVec2 forward{center.x + std::sin(pose.headingRadiansClockwise) *
                                        (markerRadius + 7.0f),
                         center.y - std::cos(pose.headingRadiansClockwise) *
                                        (markerRadius + 7.0f)};
    drawList->AddLine(center, forward, IM_COL32(245, 249, 255, 225), 1.5f);
    drawList->AddCircleFilled(center, markerRadius + 2.5f, IM_COL32(10, 16, 23, 225));
    drawList->AddCircle(center, markerRadius + 2.5f, IM_COL32(70, 205, 255, 235), 0, 1.2f);
    drawList->AddCircleFilled(center, markerRadius, IM_COL32(246, 249, 252, 255));
}
