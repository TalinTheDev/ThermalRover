#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>

#include "app.hpp"
#include "imgui.h"
#include "layers/debug.hpp"
#include "layers/slam_map.hpp"
#include "misc/cpp/imgui_stdlib.h"
#include "networking/udpserver.hpp"
#include "protocol/robot_config.hpp"
#include "protocol/udp.hpp"
#include "session_store.hpp"
#include "ui_theme.hpp"
#include "rover_commands.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <charconv>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <format>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <queue>
#include <random>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <thread>

#ifdef THERMAL_ROVER_WORKFLOW_CHECKS
#include "imgui_internal.h"
#endif

namespace {
#ifdef THERMAL_ROVER_WORKFLOW_CHECKS
constexpr unsigned short desktopPort = 0;
constexpr bool applicationWindowVisible = false;
std::filesystem::path workflowArtifactDirectory() {
    static const auto directory = std::filesystem::path(THERMAL_ROVER_WORKFLOW_ARTIFACT_ROOT) /
        std::format("workflow-{}", std::chrono::steady_clock::now().time_since_epoch().count());
    return directory;
}
struct WorkflowDriveInput {
    bool focused = true;
    bool forward = false;
    bool reverse = false;
    bool left = false;
    bool right = false;
};
WorkflowDriveInput workflowDriveInput;
void installWorkflowLayers(DebugLayer& debugLayer);
#else
constexpr unsigned short desktopPort = 3000;
constexpr bool applicationWindowVisible = true;
#endif
constexpr size_t maxVisibleMessages = 40;
constexpr auto moveHeartbeatInterval = std::chrono::milliseconds(40);
constexpr auto statusProbeInterval = std::chrono::milliseconds(500);
constexpr auto motionKeepaliveInterval = std::chrono::milliseconds(200);
constexpr auto driveArmKeepaliveInterval = std::chrono::milliseconds(200);
constexpr auto driveArmAckTimeout = std::chrono::milliseconds(1000);
constexpr auto driveContactTimeout = std::chrono::milliseconds(1000);
constexpr std::uint32_t sequenceHalfRange = 0x80000000U;
constexpr unsigned int sequenceReplayWindowBits = 64;
constexpr std::size_t replayTimelineItemsPerFrameLimit = 4096;
int selectedTheme = 0;
constexpr float applicationFontSize = 20.0f;
mapping::SlamConfig persistentMappingConfig{};
mapping::SlamConfig mappingCalibrationDraft{};
bool invalidSavedMappingCalibration = false;

std::string pathToUtf8(const std::filesystem::path& path) {
    const std::u8string encoded = path.u8string();
    std::string result(encoded.size(), '\0');
    if(!encoded.empty()) {
        std::memcpy(result.data(), encoded.data(), encoded.size());
    }
    return result;
}

std::filesystem::path pathFromUtf8(std::string_view value) {
    std::u8string encoded;
    encoded.reserve(value.size());
    for(const unsigned char byte : value) {
        encoded.push_back(static_cast<char8_t>(byte));
    }
    return std::filesystem::path(encoded);
}

std::filesystem::path bundledFontPath() {
    std::array<wchar_t, 32768> executablePath{};
    const DWORD length = GetModuleFileNameW(nullptr, executablePath.data(),
                                            static_cast<DWORD>(executablePath.size()));
    if(length == 0 || length >= executablePath.size()) return {};
    return std::filesystem::path(std::wstring_view(executablePath.data(), length))
        .parent_path() / "fonts" / "Karla-Regular.ttf";
}

void loadApplicationFont() {
    auto& io = ImGui::GetIO();
    ImGui::GetStyle().FontSizeBase = applicationFontSize;
    ImFontConfig config;
    config.Flags |= ImFontFlags_NoLoadError;
    const auto load = [&](const std::filesystem::path& path) {
        if(path.empty()) return static_cast<ImFont*>(nullptr);
        return io.Fonts->AddFontFromFileTTF(pathToUtf8(path).c_str(), applicationFontSize, &config);
    };
    io.FontDefault = load(bundledFontPath());
#ifdef THERMAL_ROVER_FONT_PATH
    if(io.FontDefault == nullptr) io.FontDefault = load(pathFromUtf8(THERMAL_ROVER_FONT_PATH));
#endif
}

template<typename Value>
bool parsePreferenceValue(const std::string& line, std::string_view key,
                         Value& value) {
    if(!line.starts_with(key)) return false;
    const char* begin = line.data() + key.size();
    const char* end = line.data() + line.size();
    const auto parsed = std::from_chars(begin, end, value);
    return parsed.ec == std::errc{} && parsed.ptr == end;
}

std::filesystem::path applicationDataDirectory() {
#ifdef THERMAL_ROVER_WORKFLOW_CHECKS
    return workflowArtifactDirectory() / "settings";
#else
    wchar_t* localAppData = nullptr;
    std::size_t localAppDataSize = 0;
    std::filesystem::path baseDirectory;
    if(_wdupenv_s(&localAppData, &localAppDataSize, L"LOCALAPPDATA") == 0 &&
       localAppData != nullptr && localAppData[0] != L'\0') {
        baseDirectory = localAppData;
    }
    std::free(localAppData);
    if(baseDirectory.empty()) {
        std::error_code currentPathError;
        baseDirectory = std::filesystem::current_path(currentPathError);
        if(currentPathError) return {};
    }
    return baseDirectory / "ThermalRover";
#endif
}

std::filesystem::path defaultSessionDirectory() {
    const std::filesystem::path appData = applicationDataDirectory();
    return appData.empty() ? std::filesystem::path{} : appData / "sessions";
}

std::string makeSessionFilePath() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
    localtime_s(&localTime, &time);
    char timestamp[32]{};
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d-%H%M%S", &localTime);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    const std::string filename = std::format(
        "thermal-rover-{}-{:03}.sqlite", timestamp, milliseconds);
    const std::filesystem::path directory = defaultSessionDirectory();
    return directory.empty() ? filename : pathToUtf8(directory / filename);
}

std::string formatElapsedTime(std::uint64_t milliseconds) {
    const std::uint64_t totalSeconds = milliseconds / 1000U;
    const std::uint64_t hours = totalSeconds / 3600U;
    const std::uint64_t minutes = (totalSeconds / 60U) % 60U;
    const std::uint64_t seconds = totalSeconds % 60U;
    return std::format("{:02}:{:02}:{:02}", hours, minutes, seconds);
}

std::filesystem::path preferencesFilePath() {
    const std::filesystem::path directory = applicationDataDirectory();
    return directory.empty() ? std::filesystem::path{}
                             : directory / "preferences.ini";
}

void loadPreferences() {
    persistentMappingConfig = mapping::SlamConfig{};
    mappingCalibrationDraft = persistentMappingConfig;
    invalidSavedMappingCalibration = false;
    const std::filesystem::path path = preferencesFilePath();
    if(path.empty()) return;
    std::ifstream input(path);
    if(!input) return;

    float lidarYawDegrees = persistentMappingConfig.mounts.lidar.headingRadiansClockwise *
                            180.0f / std::numbers::pi_v<float>;
    float thermalYawDegrees = persistentMappingConfig.mounts.thermal.headingRadiansClockwise *
                              180.0f / std::numbers::pi_v<float>;
    int mirrorThermalColumns = persistentMappingConfig.mounts.mirrorThermalColumns
        ? 1 : 0;
    int thermalRotation = 0;
    int sensorReferenceVersion = 0;
    std::string line;
    while(std::getline(input, line)) {
        const auto loadMappingValue = [&](std::string_view key, auto& value) {
            if(line.starts_with(key) && !parsePreferenceValue(line, key, value)) {
                invalidSavedMappingCalibration = true;
            }
        };
        constexpr std::string_view themeKey = "theme=";
        if(line.starts_with(themeKey)) {
            int theme = 0;
            const char* begin = line.data() + themeKey.size();
            const char* end = line.data() + line.size();
            const auto parsed = std::from_chars(begin, end, theme);
            if(parsed.ec == std::errc{} && parsed.ptr == end &&
               theme >= 0 && theme <= 2) {
                selectedTheme = theme;
            }
        }
        loadMappingValue("slam_lidar_x_cm=",
                         persistentMappingConfig.mounts.lidar.xCm);
        loadMappingValue("slam_lidar_y_cm=",
                         persistentMappingConfig.mounts.lidar.yCm);
        loadMappingValue("slam_lidar_z_cm=",
                         persistentMappingConfig.mounts.lidar.zCm);
        loadMappingValue("slam_lidar_yaw_degrees=", lidarYawDegrees);
        loadMappingValue("slam_sensor_reference_version=", sensorReferenceVersion);
        loadMappingValue("slam_thermal_x_cm=",
                         persistentMappingConfig.mounts.thermal.xCm);
        loadMappingValue("slam_thermal_y_cm=",
                         persistentMappingConfig.mounts.thermal.yCm);
        loadMappingValue("slam_thermal_z_cm=",
                         persistentMappingConfig.mounts.thermal.zCm);
        loadMappingValue("slam_thermal_yaw_degrees=", thermalYawDegrees);
        loadMappingValue("slam_thermal_horizontal_fov_degrees=",
            persistentMappingConfig.mounts.thermalHorizontalFovDegrees);
        loadMappingValue("slam_thermal_vertical_fov_degrees=",
            persistentMappingConfig.mounts.thermalVerticalFovDegrees);
        loadMappingValue("slam_thermal_mirror_columns=", mirrorThermalColumns);
        loadMappingValue("slam_thermal_rotation_quarter_turns=", thermalRotation);
    }
    // Migrate the old assumed-forward default to the user's confirmed right-facing zero.
    if(sensorReferenceVersion == 0 && std::fabs(lidarYawDegrees) < 0.0001f) lidarYawDegrees = 90.0f;
    persistentMappingConfig.mounts.lidar.headingRadiansClockwise =
        lidarYawDegrees * std::numbers::pi_v<float> / 180.0f;
    persistentMappingConfig.mounts.thermal.headingRadiansClockwise =
        thermalYawDegrees * std::numbers::pi_v<float> / 180.0f;
    persistentMappingConfig.mounts.mirrorThermalColumns =
        mirrorThermalColumns != 0;
    persistentMappingConfig.mounts.thermalRotationQuarterTurns =
        static_cast<std::uint8_t>(std::clamp(thermalRotation, 0, 3));
    const bool invalidPreferenceFormat = invalidSavedMappingCalibration;
    invalidSavedMappingCalibration = invalidPreferenceFormat ||
        !std::isfinite(persistentMappingConfig.mounts.lidar.xCm) ||
        !std::isfinite(persistentMappingConfig.mounts.lidar.yCm) ||
        !std::isfinite(persistentMappingConfig.mounts.lidar.zCm) ||
        !std::isfinite(persistentMappingConfig.mounts.lidar.headingRadiansClockwise) ||
        !std::isfinite(persistentMappingConfig.mounts.thermal.xCm) ||
        !std::isfinite(persistentMappingConfig.mounts.thermal.yCm) ||
        !std::isfinite(persistentMappingConfig.mounts.thermal.zCm) ||
        !std::isfinite(persistentMappingConfig.mounts.thermal.headingRadiansClockwise) ||
        !std::isfinite(persistentMappingConfig.mounts.thermalHorizontalFovDegrees) ||
        !std::isfinite(persistentMappingConfig.mounts.thermalVerticalFovDegrees) ||
        std::fabs(persistentMappingConfig.mounts.lidar.xCm) > 100.0f ||
        std::fabs(persistentMappingConfig.mounts.lidar.yCm) > 100.0f ||
        persistentMappingConfig.mounts.lidar.zCm < 0.0f ||
        persistentMappingConfig.mounts.lidar.zCm > 200.0f ||
        std::fabs(persistentMappingConfig.mounts.lidar.headingRadiansClockwise) >
            std::numbers::pi_v<float> ||
        std::fabs(persistentMappingConfig.mounts.thermal.xCm) > 100.0f ||
        std::fabs(persistentMappingConfig.mounts.thermal.yCm) > 100.0f ||
        persistentMappingConfig.mounts.thermal.zCm < 0.0f ||
        persistentMappingConfig.mounts.thermal.zCm > 200.0f ||
        std::fabs(persistentMappingConfig.mounts.thermal.headingRadiansClockwise) >
            std::numbers::pi_v<float> ||
        persistentMappingConfig.mounts.thermalHorizontalFovDegrees < 1.0f ||
        persistentMappingConfig.mounts.thermalHorizontalFovDegrees > 180.0f ||
        persistentMappingConfig.mounts.thermalVerticalFovDegrees < 1.0f ||
        persistentMappingConfig.mounts.thermalVerticalFovDegrees > 180.0f ||
        (mirrorThermalColumns != 0 && mirrorThermalColumns != 1) ||
        thermalRotation < 0 || thermalRotation > 3 || sensorReferenceVersion < 0 || sensorReferenceVersion > 1;
    if(invalidSavedMappingCalibration) {
        persistentMappingConfig = mapping::SlamConfig{};
    }
    mappingCalibrationDraft = persistentMappingConfig;
}

bool savePreferences() {
    const std::filesystem::path path = preferencesFilePath();
    if(path.empty()) return false;
    std::error_code error;
    std::filesystem::create_directories(path.parent_path(), error);
    if(error) return false;
    std::ofstream output(path, std::ios::trunc);
    if(!output) return false;
    output << "theme=" << selectedTheme << '\n'
           << "slam_sensor_reference_version=1\n"
           << std::format("slam_lidar_x_cm={:.4f}\n",
                          persistentMappingConfig.mounts.lidar.xCm)
           << std::format("slam_lidar_y_cm={:.4f}\n",
                          persistentMappingConfig.mounts.lidar.yCm)
           << std::format("slam_lidar_z_cm={:.4f}\n",
                          persistentMappingConfig.mounts.lidar.zCm)
           << std::format("slam_lidar_yaw_degrees={:.4f}\n",
                persistentMappingConfig.mounts.lidar.headingRadiansClockwise *
                180.0f / std::numbers::pi_v<float>)
           << std::format("slam_thermal_x_cm={:.4f}\n",
                          persistentMappingConfig.mounts.thermal.xCm)
           << std::format("slam_thermal_y_cm={:.4f}\n",
                          persistentMappingConfig.mounts.thermal.yCm)
           << std::format("slam_thermal_z_cm={:.4f}\n",
                          persistentMappingConfig.mounts.thermal.zCm)
           << std::format("slam_thermal_yaw_degrees={:.4f}\n",
                persistentMappingConfig.mounts.thermal.headingRadiansClockwise *
                180.0f / std::numbers::pi_v<float>)
           << std::format("slam_thermal_horizontal_fov_degrees={:.4f}\n",
                persistentMappingConfig.mounts.thermalHorizontalFovDegrees)
           << std::format("slam_thermal_vertical_fov_degrees={:.4f}\n",
                persistentMappingConfig.mounts.thermalVerticalFovDegrees)
           << std::format("slam_thermal_mirror_columns={}\n",
                persistentMappingConfig.mounts.mirrorThermalColumns ? 1 : 0)
           << std::format("slam_thermal_rotation_quarter_turns={}\n",
                static_cast<unsigned>(persistentMappingConfig.mounts.thermalRotationQuarterTurns));
    output.flush();
    return output.good();
}

bool metadataFieldOffset(const std::string& metadata, std::string_view key,
                         std::size_t& valueOffset) {
    const std::string field = std::format("\"{}\"", key);
    const std::size_t fieldOffset = metadata.find(field);
    if(fieldOffset == std::string::npos) return false;
    std::size_t cursor = fieldOffset + field.size();
    while(cursor < metadata.size() &&
          (metadata[cursor] == ' ' || metadata[cursor] == '\t' ||
           metadata[cursor] == '\r' || metadata[cursor] == '\n')) ++cursor;
    if(cursor >= metadata.size() || metadata[cursor++] != ':') return false;
    while(cursor < metadata.size() &&
          (metadata[cursor] == ' ' || metadata[cursor] == '\t' ||
           metadata[cursor] == '\r' || metadata[cursor] == '\n')) ++cursor;
    valueOffset = cursor;
    return cursor < metadata.size();
}

bool parseMetadataFloatAt(const std::string& metadata, std::size_t& cursor,
                          float& value) {
    while(cursor < metadata.size() &&
          (metadata[cursor] == ' ' || metadata[cursor] == '\t' ||
           metadata[cursor] == '\r' || metadata[cursor] == '\n')) ++cursor;
    if(cursor >= metadata.size()) return false;
    char* end = nullptr;
    value = std::strtof(metadata.c_str() + cursor, &end);
    if(end == metadata.c_str() + cursor || !std::isfinite(value)) return false;
    cursor = static_cast<std::size_t>(end - metadata.c_str());
    return true;
}

bool parseMetadataFloat(const std::string& metadata, std::string_view key,
                        float& value) {
    std::size_t cursor = 0;
    if(!metadataFieldOffset(metadata, key, cursor) ||
       !parseMetadataFloatAt(metadata, cursor, value)) return false;
    while(cursor < metadata.size() &&
          (metadata[cursor] == ' ' || metadata[cursor] == '\t' ||
           metadata[cursor] == '\r' || metadata[cursor] == '\n')) ++cursor;
    return cursor < metadata.size() &&
           (metadata[cursor] == ',' || metadata[cursor] == '}');
}

bool parseMetadataInteger(const std::string& metadata, std::string_view key,
                          int& value) {
    float parsed = 0.0f;
    if(!parseMetadataFloat(metadata, key, parsed) ||
       parsed < static_cast<float>((std::numeric_limits<int>::min)()) ||
       parsed > static_cast<float>((std::numeric_limits<int>::max)())) return false;
    const float rounded = std::round(parsed);
    if(std::fabs(parsed - rounded) > 0.001f) return false;
    value = static_cast<int>(rounded);
    return true;
}

bool parseMetadataBoolean(const std::string& metadata, std::string_view key,
                          bool& value) {
    std::size_t cursor = 0;
    if(!metadataFieldOffset(metadata, key, cursor)) return false;
    if(metadata.compare(cursor, 4, "true") == 0) {
        value = true;
        cursor += 4;
    } else if(metadata.compare(cursor, 5, "false") == 0) {
        value = false;
        cursor += 5;
    } else {
        return false;
    }
    while(cursor < metadata.size() &&
          (metadata[cursor] == ' ' || metadata[cursor] == '\t' ||
           metadata[cursor] == '\r' || metadata[cursor] == '\n')) ++cursor;
    return cursor < metadata.size() &&
           (metadata[cursor] == ',' || metadata[cursor] == '}');
}

template<std::size_t Count>
bool parseMetadataArray(const std::string& metadata, std::string_view key,
                        std::array<float, Count>& values) {
    std::size_t cursor = 0;
    if(!metadataFieldOffset(metadata, key, cursor) ||
       metadata[cursor++] != '[') return false;
    for(std::size_t index = 0; index < Count; ++index) {
        if(!parseMetadataFloatAt(metadata, cursor, values[index])) return false;
        while(cursor < metadata.size() &&
              (metadata[cursor] == ' ' || metadata[cursor] == '\t' ||
               metadata[cursor] == '\r' || metadata[cursor] == '\n')) ++cursor;
        const char expected = index + 1 == Count ? ']' : ',';
        if(cursor >= metadata.size() || metadata[cursor++] != expected) return false;
    }
    while(cursor < metadata.size() &&
          (metadata[cursor] == ' ' || metadata[cursor] == '\t' ||
           metadata[cursor] == '\r' || metadata[cursor] == '\n')) ++cursor;
    return cursor < metadata.size() &&
           (metadata[cursor] == ',' || metadata[cursor] == '}');
}

bool parseSessionMappingConfig(const std::string& metadata,
                               mapping::SlamConfig& config) {
    int format = 0;
    int mapWidth = 0;
    int mapHeight = 0;
    std::array<float, 4> lidarMount{};
    std::array<float, 4> thermalMount{};
    std::array<float, 3> initialPose{};
    bool mirrorThermalColumns = false;
    bool scanMatching = false;
    float recordedGyroSigma = config.gyroYawPriorSigmaRadians;
    int thermalRotation = 0;
    int sensorReferenceVersion = 0;

    if(!parseMetadataInteger(metadata, "format", format) || format != 1 ||
       !parseMetadataArray(metadata, "lidar_mount_cm", lidarMount) ||
       !parseMetadataArray(metadata, "thermal_mount_cm", thermalMount) ||
       !parseMetadataFloat(metadata, "thermal_horizontal_fov_degrees",
                           config.mounts.thermalHorizontalFovDegrees) ||
       !parseMetadataFloat(metadata, "thermal_vertical_fov_degrees",
                           config.mounts.thermalVerticalFovDegrees) ||
       !parseMetadataBoolean(metadata, "thermal_mirror_columns",
                             mirrorThermalColumns) ||
       !parseMetadataFloat(metadata, "lidar_min_range_cm",
                           config.lidarMinRangeCm) ||
       !parseMetadataFloat(metadata, "lidar_max_range_cm",
                           config.lidarMaxRangeCm) ||
       !parseMetadataInteger(metadata, "map_width_cells", mapWidth) ||
       !parseMetadataInteger(metadata, "map_height_cells", mapHeight) ||
       !parseMetadataFloat(metadata, "map_cell_cm", config.centimetersPerCell) ||
       !parseMetadataFloat(metadata, "display_map_fraction",
                           config.displayMapFraction) ||
       !parseMetadataBoolean(metadata, "scan_matching_enabled", scanMatching) ||
       !parseMetadataArray(metadata, "initial_pose_cm", initialPose)) {
        return false;
    }

    // Older session metadata may predate this field; use the compiled
    // starting weight for those captures and validate it when present.
    if(metadata.find("\"gyro_yaw_prior_sigma_radians\"") != std::string::npos &&
       !parseMetadataFloat(metadata, "gyro_yaw_prior_sigma_radians",
                           recordedGyroSigma)) {
        return false;
    }

    if(metadata.find("\"thermal_rotation_quarter_turns\"") != std::string::npos &&
       !parseMetadataInteger(metadata, "thermal_rotation_quarter_turns", thermalRotation)) return false;
    if(metadata.find("\"sensor_reference_version\"") != std::string::npos &&
       !parseMetadataInteger(metadata, "sensor_reference_version", sensorReferenceVersion)) return false;
    std::size_t sourceOffset = 0;
    if(sensorReferenceVersion == 0 && std::fabs(lidarMount[3]) < 0.0001f &&
       metadataFieldOffset(metadata, "capture_source", sourceOffset) &&
       metadata.compare(sourceOffset, 7, "\"rover\"") == 0)
        lidarMount[3] = rover::config::lidarYawRadiansClockwise;

    if(thermalRotation < 0 || thermalRotation > 3 || sensorReferenceVersion < 0 || sensorReferenceVersion > 1 ||
       std::fabs(lidarMount[0]) > 1000.0f || std::fabs(lidarMount[1]) > 1000.0f ||
       std::fabs(lidarMount[2]) > 1000.0f ||
       std::fabs(lidarMount[3]) > 6.28319f ||
       std::fabs(thermalMount[0]) > 1000.0f ||
       std::fabs(thermalMount[1]) > 1000.0f ||
       std::fabs(thermalMount[2]) > 1000.0f ||
       std::fabs(thermalMount[3]) > 6.28319f ||
       config.mounts.thermalHorizontalFovDegrees <= 0.0f ||
       config.mounts.thermalHorizontalFovDegrees > 180.0f ||
       config.mounts.thermalVerticalFovDegrees <= 0.0f ||
       config.mounts.thermalVerticalFovDegrees > 180.0f ||
       config.lidarMinRangeCm < 0.0f || config.lidarMaxRangeCm <= config.lidarMinRangeCm ||
       config.lidarMaxRangeCm > 2000.0f || mapWidth < 64 || mapWidth > 1024 ||
       mapHeight < 64 || mapHeight > 1024 || config.centimetersPerCell < 0.5f ||
       config.centimetersPerCell > 100.0f || config.displayMapFraction < 0.05f ||
       config.displayMapFraction > 1.0f || std::fabs(initialPose[0]) > 10000.0f ||
       std::fabs(initialPose[1]) > 10000.0f ||
       std::fabs(initialPose[2]) > 6.28319f ||
       recordedGyroSigma < 0.005f || recordedGyroSigma > 0.05f) {
        return false;
    }

    config.mounts.lidar = {lidarMount[0], lidarMount[1], lidarMount[2],
                           lidarMount[3]};
    config.mounts.thermal = {thermalMount[0], thermalMount[1], thermalMount[2],
                             thermalMount[3]};
    config.mounts.mirrorThermalColumns = mirrorThermalColumns;
    config.mounts.thermalRotationQuarterTurns = static_cast<std::uint8_t>(thermalRotation);
    config.mapWidthCells = static_cast<std::size_t>(mapWidth);
    config.mapHeightCells = static_cast<std::size_t>(mapHeight);
    config.enableScanMatching = scanMatching;
    config.gyroYawPriorSigmaRadians = recordedGyroSigma;
    config.initialPose = {initialPose[0], initialPose[1], initialPose[2]};
    return true;
}

std::string findLatestSessionFile() {
    std::error_code error;
    const std::filesystem::path directory = defaultSessionDirectory();
    if(directory.empty() || !std::filesystem::exists(directory, error) || error) {
        return {};
    }
    std::filesystem::file_time_type newestTime{};
    std::string newestPath;
    for(const std::filesystem::directory_entry& entry :
        std::filesystem::directory_iterator(directory, error)) {
        if(error) break;
        if(!entry.is_regular_file(error) || error ||
           entry.path().extension() != ".sqlite") continue;
        const auto modified = entry.last_write_time(error);
        if(error) {
            error.clear();
            continue;
        }
        if(newestPath.empty() || modified > newestTime) {
            newestTime = modified;
            newestPath = pathToUtf8(entry.path());
        }
    }
    return newestPath;
}

UDPServer server(desktopPort);
App app(1500, 1000, "Thermal Rover", applicationWindowVisible);
SlamMapLayer* slamLayer = nullptr;
std::optional<mapping::SlamConfig> liveConfigBeforeReplay;
std::string roverIP;
std::uint32_t pairingToken = 0;
std::uint32_t lastConnectionToken = 0;
bool roverPaired = false;
bool pairingPending = false;
std::chrono::steady_clock::time_point pairingSentAt{};
std::uint32_t discoveryNonce = 0;
bool discoveryPending = false;
std::chrono::steady_clock::time_point discoverySentAt{};
std::string outgoingMessage;
unsigned short roverPort = 27015;

std::queue<UDPPacket> toSend;
std::deque<UDPPacket> receivedPackets;
std::mutex sendMutex;
std::condition_variable sendCV;
std::mutex receivedMutex;
std::deque<std::string> visibleMessages;
struct PendingDebugPing {
    std::string text;
    std::chrono::steady_clock::time_point sentAt;
};
std::deque<PendingDebugPing> pendingDebugPings;
ESP_STATUS roverStatus{};
ESP_STATUS replayStatus{};
bool hasStatus = false;
bool hasReplayStatus = false;
std::uint64_t replayStatusMilliseconds = 0;
std::uint64_t replayMalformedStatusPackets = 0;
std::uint32_t replayLastStatusSequence = 0;
bool hasReplayStatusSequence = false;
std::uint64_t replayStaleStatusPackets = 0;
bool warnedStatusSizeMismatch = false;
std::chrono::steady_clock::time_point lastStatusAt{};
std::uint32_t newestRoverSequence = 0;
std::uint64_t roverSequenceWindow = 0;
bool hasRoverSequence = false;
std::uint32_t lastStatusSequence = 0;
bool hasStatusSequence = false;
std::uint64_t duplicateRoverPackets = 0;
std::uint64_t staleRoverPackets = 0;
std::uint64_t reorderedRoverPackets = 0;
std::uint64_t confirmedMissingRoverPackets = 0;
unsigned int roverSequenceHistorySize = 0;
std::uint64_t staleStatusPackets = 0;
std::uint64_t malformedStatusPackets = 0;
std::uint64_t malformedAcknowledgements = 0;
std::optional<THERMAL_CAPTURE_DIAGNOSTICS> roverThermalDiagnostics;
std::chrono::steady_clock::time_point lastThermalDiagnosticsAt{};
struct StreamMetric {
    std::chrono::steady_clock::time_point windowStart{};
    std::chrono::steady_clock::time_point lastReceive{};
    std::uint64_t packetsInWindow = 0;
    double packetsPerSecond = 0.0;
};
struct StreamMetrics {
    StreamMetric lidarPackets;
    StreamMetric thermalFrames;
    StreamMetric fusedOdometry;
    StreamMetric imuSamples;
    std::uint64_t malformedSensorPackets = 0;
    std::array<std::uint64_t, 256> malformedById{};
};
StreamMetrics streamMetrics;
enum class AppWorkspace : std::uint8_t { MapCapture, Drive, Diagnostics };
AppWorkspace activeWorkspace = AppWorkspace::MapCapture;
bool driveModeActive = false;
bool armPending = false;
uint16_t nextArmCommandId = 1;
uint16_t pendingArmCommandId = 0;
uint16_t activeArmCommandId = 0;
std::chrono::steady_clock::time_point armRequestSentAt{};
std::chrono::steady_clock::time_point lastArmKeepaliveAt{};
std::chrono::steady_clock::time_point lastArmAcknowledgedAt{};
// Start conservatively while assembled-rover scale and heading are uncalibrated.
int driveSpeedPercent = 30;
MOVE_CMD lastMoveCommand{};
bool hasSentMoveCommand = false;
std::chrono::steady_clock::time_point lastMoveSentAt{};
std::chrono::steady_clock::time_point lastStatusProbeAt{};
rover::desktop::MotionCommandTracker motionTracker;
uint16_t nextMotionCommandId = 1;
uint16_t nextRecordingCommandId = 1;
uint16_t lastRecordingCommandId = 0;
uint8_t roverRecordingState = 0;
std::chrono::steady_clock::time_point lastRecordingKeepaliveAt{};
std::chrono::steady_clock::time_point lastMotionKeepaliveAt{};
SessionStore sessions;
std::string sessionPath = makeSessionFilePath();
std::string replaySessionPath = findLatestSessionFile();
std::string pngExportPath = pathToUtf8(defaultSessionDirectory() /
                                       "thermal-rover-map.png");
bool replayRunning = false;
std::uint64_t replayPositionMilliseconds = 0;
std::uint64_t replayBaseMilliseconds = 0;
double replaySpeed = 1.0;
std::chrono::steady_clock::time_point replayStartedAt{};
bool hasReplayPendingPacket = false;
bool hasReplayPendingEvent = false;
bool replayReadErrorReported = false;
Packet replayPendingPacket;
PeerAddress replayPendingPeer{};
std::uint64_t replayPendingMilliseconds = 0;
std::uint64_t replayPendingOrder = 0;
std::string replayPendingEventKind;
std::string replayPendingEventDetail;
std::uint64_t replayPendingEventMilliseconds = 0;
std::uint64_t replayPendingEventOrder = 0;
int appliedTheme = -1;
float distanceTargetCm = 5.0f;
float distanceKp = 8.0f;
float distanceKi = 0.0f;
float distanceKd = 0.0f;
float angleTargetDegrees = 90.0f;
float angleKp = 2.0f;
float angleKi = 0.0f;
float angleKd = 0.0f;

void resetStreamMetrics();

void restoreLiveMappingConfigAfterReplay() {
    if(slamLayer != nullptr && liveConfigBeforeReplay.has_value()) {
        slamLayer->setConfig(*liveConfigBeforeReplay);
        slamLayer->resetLiveMap();
    }
    liveConfigBeforeReplay.reset();
}

void closeReplaySession() {
    sessions.closeReplay();
    replayRunning = false;
    hasReplayPendingPacket = false;
    hasReplayPendingEvent = false;
    replayReadErrorReported = false;
    resetStreamMetrics();
    restoreLiveMappingConfigAfterReplay();
}

void updatePackets();
void drawSessionControls();

enum class SequenceDisposition {
    Forward,
    Reordered,
    Duplicate,
    TooOld
};

void resetRoverSequenceTracking() {
    newestRoverSequence = 0;
    roverSequenceWindow = 0;
    hasRoverSequence = false;
    lastStatusSequence = 0;
    hasStatusSequence = false;
    duplicateRoverPackets = 0;
    staleRoverPackets = 0;
    reorderedRoverPackets = 0;
    confirmedMissingRoverPackets = 0;
    roverSequenceHistorySize = 0;
    staleStatusPackets = 0;
    malformedStatusPackets = 0;
    roverThermalDiagnostics.reset();
}

void seedRoverSequence(std::uint32_t sequence) {
    newestRoverSequence = sequence;
    roverSequenceWindow = 1;
    roverSequenceHistorySize = 1;
    hasRoverSequence = true;
}

SequenceDisposition acceptRoverSequence(std::uint32_t sequence) {
    if(!hasRoverSequence) {
        seedRoverSequence(sequence);
        return SequenceDisposition::Forward;
    }

    const std::uint32_t forwardDistance = sequence - newestRoverSequence;
    if(forwardDistance == 0) return SequenceDisposition::Duplicate;
    if(forwardDistance < sequenceHalfRange) {
        const unsigned int previousHistorySize = roverSequenceHistorySize;
        const std::uint64_t previousWindow = roverSequenceWindow;
        if(forwardDistance >= sequenceReplayWindowBits) {
            const std::uint64_t knownMask = previousHistorySize == 64
                ? ~std::uint64_t{0}
                : ((1ULL << previousHistorySize) - 1ULL);
            confirmedMissingRoverPackets += previousHistorySize -
                static_cast<unsigned int>(std::popcount(previousWindow & knownMask));
            confirmedMissingRoverPackets +=
                forwardDistance - sequenceReplayWindowBits;
            roverSequenceHistorySize = sequenceReplayWindowBits;
        } else {
            const unsigned int distance = forwardDistance;
            const unsigned int expiredCount = previousHistorySize + distance >
                    sequenceReplayWindowBits
                ? previousHistorySize + distance - sequenceReplayWindowBits
                : 0;
            if(expiredCount != 0) {
                const unsigned int firstExpiredOffset =
                    previousHistorySize - expiredCount;
                const std::uint64_t expiredMask =
                    ((1ULL << expiredCount) - 1ULL) << firstExpiredOffset;
                confirmedMissingRoverPackets += expiredCount -
                    static_cast<unsigned int>(
                        std::popcount(previousWindow & expiredMask));
            }
            roverSequenceHistorySize = std::min<unsigned int>(
                sequenceReplayWindowBits, previousHistorySize + distance);
        }
        roverSequenceWindow = forwardDistance >= sequenceReplayWindowBits
            ? 1ULL : (roverSequenceWindow << forwardDistance) | 1ULL;
        newestRoverSequence = sequence;
        return SequenceDisposition::Forward;
    }

    const std::uint32_t backwardDistance = newestRoverSequence - sequence;
    if(backwardDistance >= sequenceReplayWindowBits) {
        return SequenceDisposition::TooOld;
    }
    const std::uint64_t bit = 1ULL << backwardDistance;
    if((roverSequenceWindow & bit) != 0) return SequenceDisposition::Duplicate;
    roverSequenceWindow |= bit;
    return SequenceDisposition::Reordered;
}

void resetStreamMetrics() {
    streamMetrics = {};
}

void recordStreamMetric(StreamMetric& metric) {
    const auto now = std::chrono::steady_clock::now();
    if(metric.windowStart.time_since_epoch().count() == 0) {
        metric.windowStart = now;
    }
    metric.lastReceive = now;
    ++metric.packetsInWindow;
}


void updateStreamMetricRates() {
    const auto now = std::chrono::steady_clock::now();
    const auto update = [now](StreamMetric& metric) {
        if(metric.windowStart.time_since_epoch().count() == 0) return;
        const double elapsed = std::chrono::duration<double>(
            now - metric.windowStart).count();
        if(elapsed < 1.0) return;
        metric.packetsPerSecond =
            static_cast<double>(metric.packetsInWindow) / elapsed;
        metric.packetsInWindow = 0;
        metric.windowStart = now;
    };
    update(streamMetrics.lidarPackets);
    update(streamMetrics.thermalFrames);
    update(streamMetrics.fusedOdometry);
    update(streamMetrics.imuSamples);
}

bool validateThermalDiagnostics(const THERMAL_CAPTURE_DIAGNOSTICS& diagnostics) {
    return diagnostics.stage <= ThermalCaptureStage::InvalidTemperature &&
           (diagnostics.lastSubpage <= 1 || diagnostics.lastSubpage == 0xff) &&
           (diagnostics.invalidPixel < mapping::thermalPixelCount ||
            diagnostics.invalidPixel == 0xffff);
}

bool validateRoverStatus(const ESP_STATUS& status) {
    const bool batteryPercentValid = status.batteryP == -1.0f ||
        (status.batteryP >= 0.0f && status.batteryP <= 100.0f);
    const bool batteryVoltageValid = status.batteryV == -1.0f ||
        (status.batteryV >= 0.0f && status.batteryV <= 10.0f);
    const bool yawDirectionValid =
        std::fabs(std::fabs(status.imuYawDirection) - 1.0f) <= 0.001f;
    const bool batteryReady =
        (status.sensorHealthFlags & SENSOR_BATTERY_READY) != 0;
    const bool batteryValuesAvailable =
        status.batteryP >= 0.0f && status.batteryV >= 0.0f;
    return std::isfinite(status.batteryP) &&
           std::isfinite(status.batteryV) &&
           std::isfinite(status.xCentimeters) &&
           std::isfinite(status.yCentimeters) &&
           std::isfinite(status.headingDegreesClockwise) &&
           std::isfinite(status.imuGyroBiasDegreesPerSecond) &&
           std::isfinite(status.imuYawDirection) &&
           batteryPercentValid && batteryVoltageValid &&
           batteryReady == batteryValuesAvailable && yawDirectionValid &&
           std::fabs(status.xCentimeters) <= 100000000.0f &&
           std::fabs(status.yCentimeters) <= 100000000.0f &&
           status.headingDegreesClockwise >= 0.0f &&
           status.headingDegreesClockwise < 360.0f &&
           status.motionActive <= 1 &&
           status.motionResult <= MOTION_RESULT_MOTOR_FAULT &&
           status.port != 0 && status.recordingState <= 2 &&
           std::fabs(status.imuGyroBiasDegreesPerSecond) <= 100.0f;
}

bool validateSensorPayload(const Packet& packet, PacketID packetId,
                           std::uint16_t payloadLength) {
    if(packetId == LIDAR_PACKET) {
        if(payloadLength != sizeof(LIDAR_PACKET_DATA)) {
            return false;
        }
        LIDAR_PACKET_DATA lidar{};
        std::memcpy(&lidar, packet.buffer + sizeof(PacketHeader), sizeof(lidar));
        std::vector<mapping::LidarPoint> points;
        return mapping::LidarPacketParser::decodePacket(lidar.packet, points);
    } else if(packetId == WHEEL_ODOMETRY) {
        // Historical recordings may contain these rows; they never affect pose.
        return payloadLength == sizeof(WHEEL_ODOMETRY_DATA);
    } else if(packetId == FUSED_ODOMETRY) {
        if(payloadLength != sizeof(FUSED_ODOMETRY_DATA)) return false;
        FUSED_ODOMETRY_DATA sample{};
        std::memcpy(&sample, packet.buffer + sizeof(PacketHeader), sizeof(sample));
        return std::isfinite(sample.xCentimeters) && std::isfinite(sample.yCentimeters) &&
               std::isfinite(sample.headingRadiansClockwise) &&
               std::fabs(sample.headingRadiansClockwise) <= std::numbers::pi_v<float> &&
               std::isfinite(sample.residualCentimeters) && sample.residualCentimeters >= 0 &&
               sample.matchedPoints <= rover::odometry::scanBins && sample.quality <= 4;
    } else if(packetId == IMU_SAMPLE) {
        if(payloadLength != sizeof(IMU_SAMPLE_DATA)) {
            return false;
        }
        IMU_SAMPLE_DATA sample{};
        std::memcpy(&sample, packet.buffer + sizeof(PacketHeader), sizeof(sample));
        return std::isfinite(sample.yawRadiansClockwise) &&
               std::isfinite(sample.gyroZDegreesPerSecondClockwise) &&
               std::fabs(sample.yawRadiansClockwise) <=
                   std::numbers::pi_v<float> &&
               std::fabs(sample.gyroZDegreesPerSecondClockwise) <= 510.0f;
    } else if(packetId == THERMAL_CHUNK) {
        if(payloadLength != sizeof(THERMAL_FRAME_CHUNK)) {
            return false;
        }
        THERMAL_FRAME_CHUNK chunk{};
        std::memcpy(&chunk, packet.buffer + sizeof(PacketHeader), sizeof(chunk));
        const std::size_t offset = chunk.pixelOffset;
        const std::size_t count = chunk.sampleCount;
        if(count == 0 || count > THERMAL_CHUNK_PIXELS ||
           offset > mapping::thermalPixelCount ||
           count > mapping::thermalPixelCount - offset) {
            return false;
        }
        for(std::size_t i = 0; i < count; ++i) {
            if(chunk.temperaturesCentiC[i] < -4000 ||
               chunk.temperaturesCentiC[i] > 30000) return false;
        }
        return true;
    }
    return false;
}

bool recordLiveSensorMetric(const Packet& packet, PacketID packetId,
                            std::uint16_t payloadLength) {
    if(!validateSensorPayload(packet, packetId, payloadLength)) {
        ++streamMetrics.malformedSensorPackets;
        ++streamMetrics.malformedById[packetId];
        return false;
    }
    if(packetId == LIDAR_PACKET) {
        recordStreamMetric(streamMetrics.lidarPackets);
    } else if(packetId == FUSED_ODOMETRY) {
        recordStreamMetric(streamMetrics.fusedOdometry);
    } else if(packetId == IMU_SAMPLE) {
        recordStreamMetric(streamMetrics.imuSamples);
    } else if(packetId == THERMAL_CHUNK) {
        THERMAL_FRAME_CHUNK chunk{};
        std::memcpy(&chunk, packet.buffer + sizeof(PacketHeader), sizeof(chunk));
        if(chunk.pixelOffset == 0) recordStreamMetric(streamMetrics.thermalFrames);
    }
    return true;
}

void drawStreamMetric(const char* label, const StreamMetric& metric,
                      std::chrono::steady_clock::time_point now,
                      std::chrono::milliseconds maximumAge) {
    const float labelX = ImGui::GetCursorPosX();
    ImGui::Text("%s", label);
    ImGui::SameLine(labelX + std::max(160.0f, ImGui::CalcTextSize(label).x + 12.0f));
    if(metric.lastReceive.time_since_epoch().count() == 0) {
        ImGui::TextDisabled("NO DATA");
        return;
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - metric.lastReceive);
    const ImVec4 color = age <= maximumAge
        ? ui::themeColors().healthy
        : ui::themeColors().error;
    ImGui::TextColored(color, "%.1f/s  /  %lld ms",
        metric.packetsPerSecond,
        static_cast<long long>(age.count()));
}


bool acceptStatusSequence(std::uint32_t sequence) {
    if(!hasStatusSequence) {
        lastStatusSequence = sequence;
        hasStatusSequence = true;
        return true;
    }
    const std::uint32_t forwardDistance = sequence - lastStatusSequence;
    if(forwardDistance == 0 || forwardDistance >= sequenceHalfRange) return false;
    lastStatusSequence = sequence;
    return true;
}

bool statusContactFresh() {
    return hasStatus && lastStatusAt.time_since_epoch().count() != 0 &&
           std::chrono::steady_clock::now() - lastStatusAt <= driveContactTimeout;
}

void applyTheme(int theme) {
    ui::applyTheme(theme);
    appliedTheme = theme;
}

void sectionLabel(const char* title) {
    ImGui::PushFont(nullptr, 22.0f);
    ImGui::TextUnformatted(title);
    ImGui::PopFont();
    ImGui::Spacing();
}

void fieldLabel(const char* label) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s", label);
    ImGui::PopStyleColor();
}

void propertyRow(const char* label, const std::string& value) {
    if(ImGui::BeginTable(label, 2, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed,
                                std::min(155.0f, ImGui::GetContentRegionAvail().x * 0.45f));
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("%s", label);
        ImGui::TableSetColumnIndex(1);
        ImGui::TextWrapped("%s", value.c_str());
        ImGui::EndTable();
    }
}

void pathField(const char* label, const char* id, std::string& value) {
    fieldLabel(label);
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputText(id, &value);
    if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort) && !value.empty()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(520.0f);
        ImGui::TextUnformatted(value.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

bool isMoving(const MOVE_CMD& command) {
    return command.speed != 0 &&
           (command.throttle != 0 || command.steering != 0);
}

PacketID packetId(const UDPPacket& packet) {
    if(packet.packet.size < sizeof(PacketHeader)) return NONE;
    PacketHeader header{};
    std::memcpy(&header, packet.packet.buffer, sizeof(header));
    return header.type == COMMAND ? header.id : NONE;
}

void stampPacketAuthentication(Packet& packet, std::uint32_t authToken) {
    if(packet.size < sizeof(PacketHeader)) return;
    PacketHeader header{};
    std::memcpy(&header, packet.buffer, sizeof(header));
    header.authToken = authToken;
    header.version = PROTOCOL_VERSION;
    std::memcpy(packet.buffer, &header, sizeof(header));
}

bool sameMoveCommand(const MOVE_CMD& lhs, const MOVE_CMD& rhs) {
    return lhs.throttle == rhs.throttle && lhs.steering == rhs.steering &&
           lhs.speed == rhs.speed;
}

bool enqueueMoveCommand(const MOVE_CMD& command) {
    if(roverIP.empty() || roverIP.size() >= IP_LEN) return false;

    Packet packet(COMMAND, MOVE, command);
    stampPacketAuthentication(packet, roverPaired ? pairingToken : 0);
    UDPPacket outgoing{packet, {"", roverPort}};
    std::memcpy(outgoing.peer.ip, roverIP.c_str(), roverIP.size() + 1);
    sessions.recordPacket(outgoing.packet, outgoing.peer, true);
    {
        std::scoped_lock lock(sendMutex);
        // Coalesce movement heartbeats without moving the newest sequence
        // number ahead of older queued commands.
        std::queue<UDPPacket> pending;
        while(!toSend.empty()) {
            UDPPacket queued = std::move(toSend.front());
            toSend.pop();
            if(packetId(queued) != MOVE) pending.push(std::move(queued));
        }
        pending.push(std::move(outgoing));
        toSend.swap(pending);
    }
    sendCV.notify_one();
    return true;
}

void sendMoveCommand(const MOVE_CMD& command, bool force = false) {
    const auto now = std::chrono::steady_clock::now();
    const bool changed = !hasSentMoveCommand ||
                         !sameMoveCommand(command, lastMoveCommand);
    const bool heartbeatDue = isMoving(command) &&
                              now - lastMoveSentAt >= moveHeartbeatInterval;
    if(!force && !changed && !heartbeatDue) return;

    enqueueMoveCommand(command);
    lastMoveCommand = command;
    lastMoveSentAt = now;
    hasSentMoveCommand = true;
}

bool enqueueMotionPacket(const Packet& packet) {
    if(roverIP.empty() || roverIP.size() >= IP_LEN) return false;
    Packet authenticatedPacket = packet;
    PacketHeader outgoingHeader{};
    std::memcpy(&outgoingHeader, authenticatedPacket.buffer,
                sizeof(outgoingHeader));
    const std::uint32_t token = outgoingHeader.id == PAIR
        ? 0 : (roverPaired ? pairingToken : 0);
    stampPacketAuthentication(authenticatedPacket, token);
    UDPPacket outgoing{authenticatedPacket, {"", roverPort}};
    std::memcpy(outgoing.peer.ip, roverIP.c_str(), roverIP.size() + 1);
    sessions.recordPacket(outgoing.packet, outgoing.peer, true);
    {
        std::scoped_lock lock(sendMutex);
        std::queue<UDPPacket> pending;
        PacketHeader newHeader{};
        std::memcpy(&newHeader, authenticatedPacket.buffer, sizeof(newHeader));
        RECORDING_CMD newRecordingCommand{};
        ARM_CMD newArmCommand{};
        const bool isRecordingCommand =
            newHeader.type == COMMAND && newHeader.id == RECORDING &&
            newHeader.length == sizeof(RECORDING_CMD);
        const bool isArmCommand =
            newHeader.type == COMMAND && newHeader.id == ARM &&
            newHeader.length == sizeof(ARM_CMD);
        if(isRecordingCommand) {
            std::memcpy(&newRecordingCommand,
                        authenticatedPacket.buffer + sizeof(PacketHeader),
                        sizeof(newRecordingCommand));
        }
        if(isArmCommand) {
            std::memcpy(&newArmCommand,
                        authenticatedPacket.buffer + sizeof(PacketHeader),
                        sizeof(newArmCommand));
        }
        while(!toSend.empty()) {
            UDPPacket queued = std::move(toSend.front());
            toSend.pop();
            const PacketID queuedId = packetId(queued);
            RECORDING_CMD queuedRecordingCommand{};
            if(queuedId == RECORDING &&
               queued.packet.size == sizeof(PacketHeader) +
                                     sizeof(RECORDING_CMD)) {
                std::memcpy(&queuedRecordingCommand,
                            queued.packet.buffer + sizeof(PacketHeader),
                            sizeof(queuedRecordingCommand));
            }
            ARM_CMD queuedArmCommand{};
            if(queuedId == ARM &&
               queued.packet.size == sizeof(PacketHeader) + sizeof(ARM_CMD)) {
                std::memcpy(&queuedArmCommand,
                            queued.packet.buffer + sizeof(PacketHeader),
                            sizeof(queuedArmCommand));
            }
            if(newHeader.id == MOTION_STOP) continue;
            const bool removeQueued =
                (isRecordingCommand &&
                 newRecordingCommand.action == RECORDING_KEEPALIVE &&
                 queuedId == RECORDING &&
                 queuedRecordingCommand.action == RECORDING_KEEPALIVE) ||
                (newHeader.id == MOTION &&
                 (queuedId == MOVE || queuedId == MOTION_KEEPALIVE)) ||
                (newHeader.id == MOTION_KEEPALIVE &&
                 queuedId == MOTION_KEEPALIVE) ||
                (isArmCommand && queuedId == ARM &&
                 (newArmCommand.action == ARM_DISARM ||
                  (queuedArmCommand.action == ARM_KEEPALIVE &&
                   (newArmCommand.action == ARM_KEEPALIVE ||
                    newArmCommand.action == ARM_REQUEST))));
            if(!removeQueued) pending.push(std::move(queued));
        }
        if(newHeader.id == MOTION_STOP) {
            std::queue<UDPPacket> urgent;
            urgent.push(std::move(outgoing));
            pending.swap(urgent);
        } else {
            pending.push(std::move(outgoing));
        }
        toSend.swap(pending);
    }
    sendCV.notify_one();
    return true;
}

bool sendRecordingControl(RecordingAction action) {
    if(!roverPaired) return false;
    uint16_t commandId = nextRecordingCommandId++;
    if(commandId == 0) commandId = nextRecordingCommandId++;
    const RECORDING_CMD command{commandId, action};
    Packet packet(COMMAND, RECORDING, command);
    if(!enqueueMotionPacket(packet)) return false;
    lastRecordingCommandId = commandId;
    lastRecordingKeepaliveAt = std::chrono::steady_clock::now();
    return true;
}

bool sendArmControl(ArmAction action, uint16_t commandId) {
    if(!roverPaired || commandId == 0) return false;
    return enqueueMotionPacket(Packet(COMMAND, ARM, ARM_CMD{commandId, action}));
}

bool sendMotionStop() {
    // MOTION_STOP has an empty payload.
    Packet emptyPacket;
    PacketHeader header{0, COMMAND, MOTION_STOP, packetsSent++,
                        roverPaired ? pairingToken : 0, PROTOCOL_VERSION};
    std::memcpy(emptyPacket.buffer, &header, sizeof(header));
    emptyPacket.size = sizeof(header);
    const bool queued = enqueueMotionPacket(emptyPacket);
    motionTracker.reset();
    return queued;
}

bool startPidMotion(MotionKind kind, float target, float kp, float ki,
                    float kd) {
    if(!driveModeActive || !hasStatus || motionTracker.pending() ||
       !std::isfinite(target) || !std::isfinite(kp) || !std::isfinite(ki) ||
       !std::isfinite(kd)) return false;
    const bool headingReady =
        (roverStatus.sensorHealthFlags & (SENSOR_MPU6050_STREAMING | SENSOR_MPU6050_CALIBRATED)) ==
        (SENSOR_MPU6050_STREAMING | SENSOR_MPU6050_CALIBRATED);
    if(!headingReady || (kind == DISTANCE_CM &&
       (roverStatus.sensorHealthFlags & SENSOR_LIDAR_IMU_ODOMETRY_READY) == 0)) {
        visibleMessages.emplace_back("Measured motion requires fresh LiDAR/IMU feedback.");
        return false;
    }
    if(isMoving(lastMoveCommand)) {
        visibleMessages.emplace_back(
            "Release WASD before starting a PID move.");
        while(visibleMessages.size() > maxVisibleMessages) {
            visibleMessages.pop_front();
        }
        return false;
    }
    const float minimum = kind == DISTANCE_CM ? 0.7f : 2.0f;
    const float maximum = kind == DISTANCE_CM ? 500.0f : 720.0f;
    if(std::fabs(target) < minimum || std::fabs(target) > maximum ||
       kp < 0.0f || kp > 100.0f || ki < 0.0f || ki > 50.0f ||
       kd < 0.0f || kd > 100.0f || (kp == 0.0f && ki == 0.0f && kd == 0.0f)) {
        visibleMessages.emplace_back("Invalid target or PID gains.");
        return false;
    }
    uint16_t commandId = nextMotionCommandId++;
    if(commandId == 0) commandId = nextMotionCommandId++;
    PID_MOTION_CMD command{commandId, kind, target, kp, ki, kd,
                           static_cast<uint8_t>((driveSpeedPercent * 255 + 50) / 100)};
    Packet packet(COMMAND, MOTION, command);
    if(!enqueueMotionPacket(packet)) return false;
    lastMotionKeepaliveAt = std::chrono::steady_clock::now();
    motionTracker.begin(commandId, lastMotionKeepaliveAt);
    return true;
}

void setDriveMode(bool active) {
    if(active ? (driveModeActive || armPending)
              : (!driveModeActive && !armPending)) return;
    const bool motorsReady = hasStatus &&
        (roverStatus.sensorHealthFlags & SENSOR_MOTORS_READY) != 0;
    const bool networkTaskFailed = hasStatus &&
        (roverStatus.sensorInitErrorFlags & SENSOR_ERROR_NETWORK_TASK) != 0;
    if(active && (!roverPaired || !statusContactFresh() ||
                  !motorsReady || networkTaskFailed ||
                  sessions.isReplaying())) {
        const bool motorInitFailed = hasStatus &&
            (roverStatus.sensorInitErrorFlags & SENSOR_ERROR_MOTOR_INIT) != 0;
        const bool motorRuntimeFailed = hasStatus &&
            (roverStatus.sensorInitErrorFlags & SENSOR_ERROR_MOTOR_RUNTIME) != 0;
        const bool motorControlFailed = hasStatus &&
            (roverStatus.sensorInitErrorFlags & SENSOR_ERROR_MOTOR_TASK) != 0;
        const char* reason = sessions.isReplaying()
            ? "Drive unavailable during session replay."
            : !roverPaired
                ? "Pair with the rover before arming drive."
                : !statusContactFresh()
                    ? "Drive remains disarmed until fresh rover status is received."
                    : networkTaskFailed
                        ? "Drive remains disarmed: rover network task failed to start."
                        : motorInitFailed
                            ? "Drive remains disarmed: motor driver initialization failed."
                            : motorRuntimeFailed
                                ? "Drive remains disarmed: motor driver runtime fault."
                                : motorControlFailed
                                    ? "Drive remains disarmed: motor control initialization failed."
                                : "Drive remains disarmed: firmware did not report motor readiness.";
        visibleMessages.emplace_back(reason);
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }
    if(active) {
        uint16_t commandId = nextArmCommandId++;
        if(commandId == 0) commandId = nextArmCommandId++;
        if(!sendArmControl(ARM_REQUEST, commandId)) {
            visibleMessages.emplace_back("Could not send the rover arm request.");
            return;
        }
        armPending = true;
        pendingArmCommandId = commandId;
        armRequestSentAt = std::chrono::steady_clock::now();
        visibleMessages.emplace_back("Waiting for the rover to confirm drive arm.");
    } else {
        const uint16_t commandId = driveModeActive
            ? activeArmCommandId : pendingArmCommandId;
        driveModeActive = false;
        armPending = false;
        activeArmCommandId = 0;
        pendingArmCommandId = 0;
        sendMotionStop();
        if(roverPaired && commandId != 0) {
            sendArmControl(ARM_DISARM, commandId);
        }
        sendMoveCommand({}, true);
    }
    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
}

void emergencyStop() {
    uint16_t commandId = driveModeActive
        ? activeArmCommandId : pendingArmCommandId;
    if(commandId == 0) {
        commandId = nextArmCommandId++;
        if(commandId == 0) commandId = nextArmCommandId++;
    }
    driveModeActive = false;
    armPending = false;
    activeArmCommandId = 0;
    pendingArmCommandId = 0;
    const bool stopQueued = sendMotionStop();
    if(roverPaired) sendArmControl(ARM_DISARM, commandId);
    sendMoveCommand({}, true);
    visibleMessages.emplace_back(stopQueued
        ? "Emergency stop sent; drive disarmed."
        : "Emergency stop could not be queued; the rover watchdog remains active.");
    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
}

void enqueueTextMessage() {
    if(!roverPaired) {
        visibleMessages.emplace_back("Pair with the rover before sending debug messages.");
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }
    if(roverIP.empty() || roverIP.size() >= IP_LEN) {
        visibleMessages.emplace_back("Send failed: enter an IPv4 address shorter than 16 characters.");
        if(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }

    const auto payload = rover::desktop::makeDebugMessagePayload(outgoingMessage);
    if(!payload.has_value()) {
        visibleMessages.emplace_back("Debug message must contain 1 to 127 UTF-8 bytes.");
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }
    Packet packet(MESSAGE, STR, *payload, pairingToken);
    UDPPacket outgoing{packet, {"", roverPort}};
    std::memcpy(outgoing.peer.ip, roverIP.data(), roverIP.size());
    sessions.recordPacket(outgoing.packet, outgoing.peer, true);
    {
        std::scoped_lock lock(sendMutex);
        toSend.push(std::move(outgoing));
    }
    pendingDebugPings.push_back({outgoingMessage, std::chrono::steady_clock::now()});
    while(pendingDebugPings.size() > 16) pendingDebugPings.pop_front();
    sendCV.notify_one();
}

void sendPairingRequest() {
    if(roverPaired || pairingPending) return;
    if(sessions.isRecording() || sessions.isPaused()) {
        visibleMessages.emplace_back("Stop the active session before connecting to a rover.");
        return;
    }
    if(sessions.isReplaying()) {
        visibleMessages.emplace_back("Close session replay before connecting to a rover.");
        return;
    }
    if(roverIP.empty() || roverIP.size() >= IP_LEN || roverPort == 0) {
        visibleMessages.emplace_back("Enter the rover IPv4 address and port before connecting.");
        return;
    }
    std::uint32_t token = 0;
    try {
        std::random_device random;
        while(token == 0 || token == lastConnectionToken) token = random();
    } catch(const std::exception&) {
        visibleMessages.emplace_back("Could not create a rover connection token. Retry Connect.");
        return;
    }

    resetRoverSequenceTracking();
    resetStreamMetrics();
    pairingToken = token;
    lastConnectionToken = token;
    roverPaired = false;
    hasStatus = false;
    lastStatusAt = {};
    pairingPending = true;
    pairingSentAt = std::chrono::steady_clock::now();
    const PAIR_REQUEST request{token};
    if(!enqueueMotionPacket(Packet(COMMAND, PAIR, request))) {
        pairingPending = false;
        visibleMessages.emplace_back("Could not queue the connection request.");
        return;
    }
    visibleMessages.emplace_back("Connecting to rover.");
}

void sendDiscoveryRequest() {
    if(sessions.isRecording() || sessions.isPaused()) {
        visibleMessages.emplace_back("Stop the active session before rediscovering the rover.");
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }
    if(sessions.isReplaying()) {
        visibleMessages.emplace_back("Close session replay before changing the rover link.");
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }
    if(roverPaired) {
        if(driveModeActive || armPending) {
            setDriveMode(false);
        } else {
            sendMotionStop();
            uint16_t commandId = nextArmCommandId++;
            if(commandId == 0) commandId = nextArmCommandId++;
            sendArmControl(ARM_DISARM, commandId);
            sendMoveCommand({}, true);
        }
    }
    roverPaired = false;
    pairingToken = 0;
    pairingPending = false;
    hasStatus = false;
    lastStatusAt = {};
    lastStatusProbeAt = {};
    roverRecordingState = 0;
    motionTracker.reset();
    pendingDebugPings.clear();
    resetRoverSequenceTracking();
    resetStreamMetrics();

    discoveryNonce = static_cast<std::uint32_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    if(discoveryNonce == 0) discoveryNonce = 1;
    const DISCOVERY_REQUEST request{discoveryNonce};
    Packet packet(COMMAND, DISCOVERY, request);
    UDPPacket outgoing{packet, {"", roverPort}};
#ifdef THERMAL_ROVER_WORKFLOW_CHECKS
    constexpr char broadcastIp[] = "127.0.0.1";
#else
    constexpr char broadcastIp[] = "255.255.255.255";
#endif
    std::memcpy(outgoing.peer.ip, broadcastIp, sizeof(broadcastIp));
    sessions.recordPacket(outgoing.packet, outgoing.peer, true);
    {
        std::scoped_lock lock(sendMutex);
        toSend.push(std::move(outgoing));
    }
    discoveryPending = true;
    discoverySentAt = std::chrono::steady_clock::now();
    sendCV.notify_one();
    visibleMessages.emplace_back("Searching for the rover on the local network.");
}

void drawPidTuning(const char* id, float& kp, float& ki, float& kd) {
    ImGui::PushID(id);
    if(ImGui::BeginTable("##pid-gains", 3, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableSetupColumn("Kp");
        ImGui::TableSetupColumn("Ki");
        ImGui::TableSetupColumn("Kd");
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputFloat("##kp", &kp, 0.0f, 0.0f, "%.3f");
        ImGui::TableSetColumnIndex(1);
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputFloat("##ki", &ki, 0.0f, 0.0f, "%.3f");
        ImGui::TableSetColumnIndex(2);
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputFloat("##kd", &kd, 0.0f, 0.0f, "%.3f");
        ImGui::EndTable();
    }
    ImGui::PopID();
}

void drawDrivePanel() {
    sectionLabel("Drive control");
    ImGui::BeginDisabled(!roverPaired);
    if(ui::dangerButton("Emergency stop", {-1.0f, 38.0f})) emergencyStop();
    ImGui::EndDisabled();
    if(ui::primaryButton(driveModeActive ? "Disarm" :
                         armPending ? "Cancel arming" : "Arm drive", {-1.0f, 34.0f})) {
        setDriveMode(!driveModeActive && !armPending);
    }
    ImGui::TextColored(driveModeActive ? ui::themeColors().healthy :
                       armPending ? ui::themeColors().warning : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled),
                       "%s", driveModeActive ? "Armed - keyboard control enabled" :
                       armPending ? "Waiting for rover acknowledgement" : "Disarmed");
    ImGui::Spacing();
    fieldLabel("Motor speed limit");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::SliderInt("##drive-speed", &driveSpeedPercent, 0, 100, "%d%%");

    const auto key = [](const char* label, bool down) {
        const auto origin = ImGui::GetCursorScreenPos();
        ImGui::Dummy({34.0f, 28.0f});
        auto* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(origin, {origin.x + 34, origin.y + 28},
            ImGui::GetColorU32(down ? ImGuiCol_HeaderActive : ImGuiCol_FrameBg), 3.0f);
        draw->AddRect(origin, {origin.x + 34, origin.y + 28}, ImGui::GetColorU32(ImGuiCol_Border), 3.0f);
        const auto size = ImGui::CalcTextSize(label);
        draw->AddText({origin.x + (34 - size.x) * 0.5f, origin.y + (28 - size.y) * 0.5f},
                      ImGui::GetColorU32(ImGuiCol_Text), label);
    };
    ImGui::Spacing();
    key("W", driveModeActive && lastMoveCommand.throttle > 0); ImGui::SameLine();
    key("A", driveModeActive && lastMoveCommand.steering < 0); ImGui::SameLine();
    key("S", driveModeActive && lastMoveCommand.throttle < 0); ImGui::SameLine();
    key("D", driveModeActive && lastMoveCommand.steering > 0);
    fieldLabel("Hold keys to drive. Release to stop.");
    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
    sectionLabel("Measured movement");
    const bool manualDriveActive = isMoving(lastMoveCommand);
    const bool canStartMotion = driveModeActive && hasStatus && !motionTracker.pending() &&
                                !manualDriveActive && driveSpeedPercent >= 8;
    fieldLabel("Distance (cm, positive = forward)");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputFloat("##distance-target", &distanceTargetCm, 1.0f, 5.0f, "%.1f");
    ImGui::BeginDisabled(!canStartMotion ||
        (roverStatus.sensorHealthFlags & SENSOR_LIDAR_IMU_ODOMETRY_READY) == 0);
    if(ImGui::Button("Move distance", {-1.0f, 0.0f})) {
        startPidMotion(DISTANCE_CM, distanceTargetCm, distanceKp, distanceKi, distanceKd);
    }
    ImGui::EndDisabled();
    ImGui::Spacing();
    fieldLabel("Angle (degrees, positive = clockwise)");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputFloat("##angle-target", &angleTargetDegrees, 1.0f, 5.0f, "%.1f");
    ImGui::BeginDisabled(!canStartMotion ||
        (roverStatus.sensorHealthFlags & (SENSOR_MPU6050_STREAMING | SENSOR_MPU6050_CALIBRATED)) !=
        (SENSOR_MPU6050_STREAMING | SENSOR_MPU6050_CALIBRATED));
    if(ImGui::Button("Turn angle", {-1.0f, 0.0f})) {
        startPidMotion(ANGLE_DEGREES, angleTargetDegrees, angleKp, angleKi, angleKd);
    }
    ImGui::EndDisabled();
    if(motionTracker.pending()) {
        ImGui::TextColored(ui::themeColors().accent, "%s", motionTracker.awaitingAcknowledgement()
            ? "Waiting for motion acknowledgement" : "Movement in progress");
        if(ImGui::Button("Stop motion", {-1.0f, 0.0f})) {
            sendMotionStop();
            sendMoveCommand({}, true);
        }
    } else if(!hasStatus) {
        ImGui::TextDisabled("Waiting for rover status.");
    } else if(manualDriveActive) {
        ImGui::TextDisabled("Release WASD before starting a measured move.");
    }
    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
    if(ImGui::CollapsingHeader("PID tuning")) {
        fieldLabel("Distance controller");
        drawPidTuning("distance", distanceKp, distanceKi, distanceKd);
        fieldLabel("Angle controller");
        drawPidTuning("angle", angleKp, angleKi, angleKd);
    }
}

void drawConnectionPanel() {
    sectionLabel("Rover connection");
    const bool linkChangeLocked = sessions.isRecording() || sessions.isPaused() ||
                                  sessions.isReplaying();
    ImGui::BeginDisabled(roverPaired || pairingPending || discoveryPending || linkChangeLocked);
    fieldLabel("IPv4 address");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputTextWithHint("##rover-ip", "Automatic when blank", &roverIP);
    fieldLabel("UDP port");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputScalar("##rover-port", ImGuiDataType_U16, &roverPort);
    ImGui::EndDisabled();
    if(linkChangeLocked) {
        ImGui::TextDisabled(sessions.isReplaying()
            ? "Close replay before changing the rover link."
            : "Stop capture before changing the rover link.");
    }
    ImGui::Spacing();
    ImGui::Text("Link: %s", roverPaired ? "Connected" :
                 discoveryPending ? "Searching" :
                 pairingPending ? "Connecting" : "Disconnected");
    const std::uint64_t protocolMismatches =
        server.getProtocolVersionMismatchCount();
    if(protocolMismatches != 0) {
        ImGui::TextColored(ui::themeColors().error,
                           "INCOMPATIBLE UDP VERSION / %llu",
                           static_cast<unsigned long long>(protocolMismatches));
    }
    ImGui::BeginDisabled(discoveryPending || pairingPending || linkChangeLocked);
    if(ui::primaryButton(discoveryPending ? "Searching..." :
                     pairingPending ? "Linking..." :
                     roverPaired ? "Relink rover" : "Link rover",
                     {-1.0f, 0.0f})) {
        if(roverPaired || roverIP.empty()) sendDiscoveryRequest();
        else sendPairingRequest();
    }
    ImGui::EndDisabled();
    if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("Leave the IPv4 address blank to find the rover automatically, or enter it to link directly. Relink searches again after an address change or reboot.");
    }
    ImGui::Spacing();
    ImGui::Text("Laptop UDP port: %u", static_cast<unsigned>(server.getLocalPort()));
}

void drawDebugMessagePanel() {
    ImGui::Spacing();
    sectionLabel("Debug message");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputText("##outgoing-message", &outgoingMessage);
    ImGui::TextDisabled("%zu/127 bytes", outgoingMessage.size());
    if(outgoingMessage.size() > 127) {
        ImGui::TextColored(ui::themeColors().error, "Message exceeds the 127-byte limit.");
    }
    const char* sendLabel = ImGui::GetContentRegionAvail().x < 145.0f
        ? "Send" : "Send message";
    ImGui::BeginDisabled(outgoingMessage.empty() || outgoingMessage.size() > 127);
    if(ImGui::Button(sendLabel, {-1.0f, 0.0f})) {
        enqueueTextMessage();
    }
    ImGui::EndDisabled();
}

bool driveWindowFocused() {
#ifdef THERMAL_ROVER_WORKFLOW_CHECKS
    return workflowDriveInput.focused;
#else
    return glfwGetWindowAttrib(app.getWindow().get(), GLFW_FOCUSED) == GLFW_TRUE;
#endif
}

bool driveKeyDown(GLFWwindow* window, int key) {
#ifdef THERMAL_ROVER_WORKFLOW_CHECKS
    (void)window;
    switch(key) {
    case GLFW_KEY_W: return workflowDriveInput.forward;
    case GLFW_KEY_S: return workflowDriveInput.reverse;
    case GLFW_KEY_A: return workflowDriveInput.left;
    case GLFW_KEY_D: return workflowDriveInput.right;
    default: return false;
    }
#else
    return glfwGetKey(window, key) == GLFW_PRESS;
#endif
}

void updateDriveArmLease() {
    const auto now = std::chrono::steady_clock::now();
    if((armPending || driveModeActive) &&
       (activeWorkspace != AppWorkspace::Drive ||
        !driveWindowFocused())) {
        setDriveMode(false);
        return;
    }
    if(armPending && now - armRequestSentAt >= driveArmAckTimeout) {
        setDriveMode(false);
        visibleMessages.emplace_back("Drive arm timed out; rover remains disarmed.");
    }
    if(driveModeActive && now - lastArmAcknowledgedAt >= driveArmAckTimeout) {
        setDriveMode(false);
        visibleMessages.emplace_back(
            "Drive disarmed: rover stopped acknowledging the arm lease.");
    }
    if(driveModeActive && now - lastArmKeepaliveAt >=
                             driveArmKeepaliveInterval) {
        if(sendArmControl(ARM_KEEPALIVE, activeArmCommandId)) {
            lastArmKeepaliveAt = now;
        } else {
            setDriveMode(false);
            visibleMessages.emplace_back("Drive disarmed: arm lease could not be sent.");
        }
    }
    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
}

void updateDriveInput() {
    if(!driveModeActive) return;

    const bool motorsReady = hasStatus &&
        (roverStatus.sensorHealthFlags & SENSOR_MOTORS_READY) != 0;
    const bool networkTaskFailed = hasStatus &&
        (roverStatus.sensorInitErrorFlags & SENSOR_ERROR_NETWORK_TASK) != 0;
    if(!statusContactFresh() || !motorsReady ||
       networkTaskFailed || sessions.isReplaying()) {
        setDriveMode(false);
        const bool motorInitFailed = hasStatus &&
            (roverStatus.sensorInitErrorFlags & SENSOR_ERROR_MOTOR_INIT) != 0;
        const bool motorRuntimeFailed = hasStatus &&
            (roverStatus.sensorInitErrorFlags & SENSOR_ERROR_MOTOR_RUNTIME) != 0;
        const bool motorControlFailed = hasStatus &&
            (roverStatus.sensorInitErrorFlags & SENSOR_ERROR_MOTOR_TASK) != 0;
        const char* reason = sessions.isReplaying()
            ? "Drive disarmed during session replay."
            : !statusContactFresh()
                ? "Drive disarmed: rover contact is stale."
                : networkTaskFailed
                    ? "Drive disarmed: rover network task failed to start."
                    : motorInitFailed
                        ? "Drive disarmed: motor driver initialization failed."
                        : motorRuntimeFailed
                            ? "Drive disarmed: motor driver runtime fault."
                            : motorControlFailed
                                ? "Drive disarmed: motor control initialization failed."
                            : "Drive disarmed: firmware did not report motor readiness.";
        visibleMessages.emplace_back(reason);
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }

    GLFWwindow* window = app.getWindow().get();
    if(!driveWindowFocused()) {
        setDriveMode(false);
        return;
    }

    int throttle = 0;
    int steering = 0;
    const ImGuiIO& io = ImGui::GetIO();
    // WantCaptureKeyboard is also set by the app's ImGui keyboard-navigation
    // mode. Suppress rover controls only while text input or an active widget
    // is consuming direct interaction.
    if(!io.WantTextInput && !ImGui::IsAnyItemActive()) {
        throttle = (driveKeyDown(window, GLFW_KEY_W) ? 1 : 0) -
                   (driveKeyDown(window, GLFW_KEY_S) ? 1 : 0);
        steering = (driveKeyDown(window, GLFW_KEY_D) ? 1 : 0) -
                   (driveKeyDown(window, GLFW_KEY_A) ? 1 : 0);
    }

    MOVE_CMD command{};
    if(throttle != 0 || steering != 0) {
        command.throttle = static_cast<int8_t>(throttle);
        command.steering = static_cast<int8_t>(steering);
        command.speed = static_cast<uint8_t>(
            (driveSpeedPercent * 255 + 50) / 100);
    }
    if((throttle != 0 || steering != 0) && motionTracker.pending()) {
        sendMotionStop();
    }
    sendMoveCommand(command);
}

void requestStatusIfNeeded() {
    if(!roverPaired || statusContactFresh()) return;
    const auto now = std::chrono::steady_clock::now();
    if(lastStatusProbeAt.time_since_epoch().count() != 0 &&
       now - lastStatusProbeAt < statusProbeInterval) return;

    // A zero-speed MOVE is a safe stop and lets the ESP32 learn this laptop's
    // current IP/UDP port for its periodic status replies.
    sendMoveCommand({}, true);
    lastStatusProbeAt = now;
}

std::string makeSessionMetadata() {
    const mapping::SlamConfig config = slamLayer != nullptr
        ? slamLayer->config() : mapping::SlamConfig{};
    const bool imuCalibrated =
        (roverStatus.sensorHealthFlags & SENSOR_MPU6050_CALIBRATED) != 0;
    return std::format(
        "{{\"format\":1,\"protocol_version\":{},\"capture_source\":\"{}\","
        "\"odometry_source\":\"lidar_imu\","
        "\"sensor_reference_version\":1,\"mapping_localization\":\"scan_to_map\","
        "\"thermal_scale\":\"run_mean\","
        "\"imu_calibrated\":{},\"imu_gyro_bias_dps\":{:.4f},"
        "\"imu_yaw_direction\":{:.1f},\"gyro_yaw_prior_sigma_radians\":{:.9g},"
        "\"rover_uptime_start_ms\":{},\"sensor_health_flags\":{},"
        "\"sensor_init_error_flags\":{},"
        "\"lidar_mount_cm\":[{:.9g},{:.9g},{:.9g},{:.9g}],"
        "\"thermal_mount_cm\":[{:.9g},{:.9g},{:.9g},{:.9g}],"
        "\"thermal_horizontal_fov_degrees\":{:.9g},"
        "\"thermal_vertical_fov_degrees\":{:.9g},"
        "\"thermal_mirror_columns\":{},"
        "\"thermal_rotation_quarter_turns\":{},"
        "\"lidar_min_range_cm\":{:.9g},\"lidar_max_range_cm\":{:.9g},"
        "\"map_width_cells\":{},\"map_height_cells\":{},"
        "\"map_cell_cm\":{:.9g},\"display_map_fraction\":{:.9g},"
        "\"scan_matching_enabled\":{},"
        "\"initial_pose_cm\":[{:.9g},{:.9g},{:.9g}],"
        "\"lidar_uart_baud\":{},\"lidar_rx_gpio\":{},"
        "\"lidar_pwm_gpio\":{}}}",
        static_cast<unsigned>(PROTOCOL_VERSION),
        "rover",
        imuCalibrated ? "true" : "false",
        std::isfinite(roverStatus.imuGyroBiasDegreesPerSecond)
            ? roverStatus.imuGyroBiasDegreesPerSecond : 0.0f,
        std::isfinite(roverStatus.imuYawDirection)
            ? roverStatus.imuYawDirection : rover::config::imuYawDirection,
        config.gyroYawPriorSigmaRadians, roverStatus.uptimeMilliseconds,
        roverStatus.sensorHealthFlags, roverStatus.sensorInitErrorFlags,
        config.mounts.lidar.xCm, config.mounts.lidar.yCm,
        config.mounts.lidar.zCm, config.mounts.lidar.headingRadiansClockwise,
        config.mounts.thermal.xCm, config.mounts.thermal.yCm,
        config.mounts.thermal.zCm,
        config.mounts.thermal.headingRadiansClockwise,
        config.mounts.thermalHorizontalFovDegrees,
        config.mounts.thermalVerticalFovDegrees,
        config.mounts.mirrorThermalColumns ? "true" : "false",
        static_cast<unsigned>(config.mounts.thermalRotationQuarterTurns),
        config.lidarMinRangeCm, config.lidarMaxRangeCm,
        config.mapWidthCells, config.mapHeightCells, config.centimetersPerCell,
        config.displayMapFraction,
        config.enableScanMatching ? "true" : "false",
        config.initialPose.xCm, config.initialPose.yCm,
        config.initialPose.headingRadiansClockwise,
        rover::config::lidarBaudRate, rover::config::lidarRxGpio,
        rover::config::lidarPwmGpio);
}

void startSessionRecording() {
    if(!roverPaired) {
        visibleMessages.emplace_back("Pair with the rover before starting a live recording.");
        return;
    }
    if(roverPaired && !statusContactFresh()) {
        visibleMessages.emplace_back("Recording start requires fresh rover status.");
        return;
    }
    if(replayRunning || sessions.isReplaying()) {
        closeReplaySession();
    }
    resetStreamMetrics();
    if(sessions.startRecording(pathFromUtf8(sessionPath),
                               makeSessionMetadata())) {
        if(slamLayer != nullptr) slamLayer->resetLiveMap();
        sessions.recordEvent("rover", std::format("{}:{}",
            roverStatus.ip, static_cast<unsigned>(roverStatus.port)));
        if(!sendRecordingControl(RECORDING_START)) {
            visibleMessages.emplace_back("Local capture started, but rover command could not be queued.");
        }
        visibleMessages.emplace_back("Session recording started.");
    } else {
        visibleMessages.push_back("Session start failed: " + sessions.lastError());
    }
    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
}

void stopSessionRecording() {
    const std::filesystem::path savedPath = sessions.path();
    sessions.recordEvent("command", "Recording stop requested.");
    if(roverPaired) sendRecordingControl(RECORDING_STOP);
    if(!sessions.stopRecording()) {
        visibleMessages.push_back("Session close failed: " + sessions.lastError());
        replaySessionPath = pathToUtf8(savedPath);
        sessionPath = makeSessionFilePath();
    } else {
        visibleMessages.push_back("Session saved: " + pathToUtf8(savedPath));
        replaySessionPath = pathToUtf8(savedPath);
        sessionPath = makeSessionFilePath();
    }
    if(slamLayer != nullptr) slamLayer->resetSensorStream();
    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
}

void stopCaptureAfterStorageFailure() {
    if(!sessions.hasStorageFailure() ||
       (!sessions.isRecording() && !sessions.isPaused())) return;

    const std::string failedPath = pathToUtf8(sessions.path());
    const std::string error = sessions.lastError();
    if(roverPaired) sendRecordingControl(RECORDING_STOP);
    sessions.stopRecording(); // Leaves complete=0 and preserves committed batches.
    if(slamLayer != nullptr) slamLayer->resetSensorStream(false);
    replaySessionPath = failedPath;
    sessionPath = makeSessionFilePath();
    visibleMessages.push_back(
        "Capture stopped after a session storage failure; the file is marked incomplete: " +
        failedPath + " / " + error);
    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
}

void updateReplay() {
    if(!sessions.isReplaying()) return;

    if(replayRunning) {
        const auto wallElapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - replayStartedAt).count();
        const double target = static_cast<double>(replayBaseMilliseconds) +
                              wallElapsed * 1000.0 * replaySpeed;
        replayPositionMilliseconds = static_cast<std::uint64_t>(std::clamp(
            target, 0.0,
            static_cast<double>(sessions.replayDurationMilliseconds())));
    }

    const auto reportReplayReadError = [] {
        if(sessions.lastError().empty() || replayReadErrorReported) return;
        replayRunning = false;
        replayReadErrorReported = true;
        visibleMessages.push_back("Replay stopped: " + sessions.lastError());
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
    };

    if(!hasReplayPendingPacket) {
        hasReplayPendingPacket = sessions.readReplayPacket(
            replayPendingPacket, replayPendingPeer, replayPendingMilliseconds,
            &replayPendingOrder);
        if(!hasReplayPendingPacket) reportReplayReadError();
    }
    if(!hasReplayPendingEvent) {
        hasReplayPendingEvent = sessions.readReplayEvent(
            replayPendingEventKind, replayPendingEventDetail,
            replayPendingEventMilliseconds, &replayPendingEventOrder);
        if(!hasReplayPendingEvent) reportReplayReadError();
    }

    std::size_t processedTimelineItems = 0;
    const auto replayWorkStartedAt = std::chrono::steady_clock::now();
    while(!replayReadErrorReported &&
          processedTimelineItems < replayTimelineItemsPerFrameLimit &&
          (processedTimelineItems == 0 ||
           std::chrono::steady_clock::now() - replayWorkStartedAt <
               std::chrono::milliseconds(8))) {
        const bool packetDue = hasReplayPendingPacket &&
            replayPendingMilliseconds <= replayPositionMilliseconds;
        const bool eventDue = hasReplayPendingEvent &&
            replayPendingEventMilliseconds <= replayPositionMilliseconds;
        if(!packetDue && !eventDue) break;

        // New sessions retain the insertion order across packets and events.
        // Older files use packet order 0 and event order 1 for tied timestamps.
        const bool applyEvent = eventDue &&
            (!packetDue || replayPendingEventMilliseconds <
                               replayPendingMilliseconds ||
             (replayPendingEventMilliseconds == replayPendingMilliseconds &&
              replayPendingEventOrder < replayPendingOrder));
        if(applyEvent) {
            if(slamLayer != nullptr) {
                slamLayer->applySessionEvent(replayPendingEventKind);
            }
            hasReplayPendingEvent = sessions.readReplayEvent(
                replayPendingEventKind, replayPendingEventDetail,
                replayPendingEventMilliseconds, &replayPendingEventOrder);
            if(!hasReplayPendingEvent) reportReplayReadError();
        } else {
            PacketHeader header{};
            std::memcpy(&header, replayPendingPacket.buffer, sizeof(header));
            if(header.type == DATA && slamLayer != nullptr) {
                if(validateSensorPayload(replayPendingPacket, header.id,
                                         header.length)) {
                    slamLayer->ingestReplayPacket(replayPendingPacket);
                } else {
                    ++streamMetrics.malformedSensorPackets;
                    ++streamMetrics.malformedById[header.id];
                }
            } else if(header.type == STATUS && header.id == ESP &&
                      (header.version == PROTOCOL_VERSION || header.version == PREVIOUS_REPLAY_PROTOCOL_VERSION) &&
                      header.length == sizeof(ESP_STATUS)) {
                const std::uint32_t statusSequenceDelta =
                    header.seq - replayLastStatusSequence;
                const bool staleStatus = hasReplayStatusSequence &&
                    (statusSequenceDelta == 0 ||
                     statusSequenceDelta >= sequenceHalfRange);
                if(staleStatus) {
                    ++replayStaleStatusPackets;
                } else {
                    replayLastStatusSequence = header.seq;
                    hasReplayStatusSequence = true;
                    ESP_STATUS status{};
                    std::memcpy(&status,
                                replayPendingPacket.buffer + sizeof(PacketHeader),
                                sizeof(status));
                    status.ip[sizeof(status.ip) - 1] = '\0';
                    if(validateRoverStatus(status)) {
                        replayStatus = status;
                        replayStatusMilliseconds = replayPendingMilliseconds;
                        hasReplayStatus = true;
                    } else {
                        ++replayMalformedStatusPackets;
                    }
                }
            } else if(header.type == STATUS && header.id == ESP &&
                      header.version == PROTOCOL_VERSION) {
                ++replayMalformedStatusPackets;
            }
            hasReplayPendingPacket = sessions.readReplayPacket(
                replayPendingPacket, replayPendingPeer,
                replayPendingMilliseconds, &replayPendingOrder);
            if(!hasReplayPendingPacket) reportReplayReadError();
        }
        ++processedTimelineItems;
    }

    if(replayRunning &&
       replayPositionMilliseconds >= sessions.replayDurationMilliseconds() &&
       !hasReplayPendingPacket && !hasReplayPendingEvent &&
       sessions.lastError().empty()) {
        replayRunning = false;
        visibleMessages.emplace_back("Session replay reached the end.");
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
    }
}

void updateRecordingKeepalive() {
    if((!sessions.isRecording() && !sessions.isPaused()) ||
       !statusContactFresh()) return;
    const auto now = std::chrono::steady_clock::now();
    if(lastRecordingKeepaliveAt.time_since_epoch().count() != 0 &&
       now - lastRecordingKeepaliveAt < std::chrono::milliseconds(500)) return;
    if(sessions.isRecording()) {
        if(roverRecordingState == 0) {
            sendRecordingControl(RECORDING_START);
        } else if(roverRecordingState == 2) {
            sendRecordingControl(RECORDING_RESUME);
        } else {
            sendRecordingControl(RECORDING_KEEPALIVE);
        }
    } else if(roverRecordingState == 0) {
        sendRecordingControl(RECORDING_START);
        sendRecordingControl(RECORDING_PAUSE);
    } else if(roverRecordingState == 1) {
        sendRecordingControl(RECORDING_PAUSE);
    } else {
        sendRecordingControl(RECORDING_KEEPALIVE);
    }
}

void expireDebugPings() {
    const auto now = std::chrono::steady_clock::now();
    if(pairingPending && now - pairingSentAt > std::chrono::seconds(3)) {
        pairingPending = false;
        pairingToken = 0;
        visibleMessages.emplace_back(
            "Connection timed out. Check the rover address and firmware, then retry.");
    }
    if(discoveryPending && now - discoverySentAt > std::chrono::seconds(3)) {
        discoveryPending = false;
        visibleMessages.emplace_back("No rover discovery reply. Check Wi-Fi and enter its IP manually.");
    }
    while(!pendingDebugPings.empty() &&
          now - pendingDebugPings.front().sentAt > std::chrono::seconds(2)) {
        visibleMessages.push_back("DEBUG PING / timeout / " +
                                  pendingDebugPings.front().text);
        pendingDebugPings.pop_front();
    }
    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
}

void beginReplay() {
    setDriveMode(false);
    if(replaySessionPath.empty() ||
       !sessions.openReplay(pathFromUtf8(replaySessionPath))) {
        visibleMessages.push_back("Replay load failed: " + sessions.lastError());
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }
    if(slamLayer != nullptr) {
        if(!liveConfigBeforeReplay.has_value()) {
            liveConfigBeforeReplay = slamLayer->config();
        }
        mapping::SlamConfig replayConfig{};
        if(parseSessionMappingConfig(sessions.replayMetadata(), replayConfig)) {
            slamLayer->setConfig(replayConfig);
        } else {
            slamLayer->setConfig(mapping::SlamConfig{});
            visibleMessages.emplace_back(
                "Session calibration metadata is missing or invalid; using compiled mapping defaults.");
        }
    }
    replayPositionMilliseconds = 0;
    replayBaseMilliseconds = 0;
    replayStartedAt = std::chrono::steady_clock::now();
    replayRunning = true;
    hasReplayPendingPacket = false;
    hasReplayPendingEvent = false;
    replayReadErrorReported = false;
    replayStatus = {};
    hasReplayStatus = false;
    replayStatusMilliseconds = 0;
    replayMalformedStatusPackets = 0;
    replayLastStatusSequence = 0;
    hasReplayStatusSequence = false;
    replayStaleStatusPackets = 0;
    resetStreamMetrics();
    if(slamLayer != nullptr) slamLayer->resetLiveMap();
    visibleMessages.emplace_back("Session replay started.");
    if(!sessions.replayIsComplete()) {
        visibleMessages.emplace_back(
            "This session was interrupted; replaying the recovered packets.");
    }
    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
}

void drawSessionControls() {
    sectionLabel(sessions.isReplaying() ? "Session replay" : "Capture");
    if(!sessions.isReplaying()) pathField("Session file", "##session-path", sessionPath);
    if(sessions.isRecording() || sessions.isPaused()) {
        ImGui::PushFont(nullptr, 28.0f);
        ImGui::TextUnformatted(formatElapsedTime(sessions.recordingDurationMilliseconds()).c_str());
        ImGui::PopFont();
    }

    if(sessions.isRecording()) {
        ImGui::TextColored(ui::themeColors().error, "Recording");
        if(ImGui::Button("Pause capture", {-1.0f, 0.0f})) {
            sessions.recordEvent("command", "Recording pause requested.");
            if(roverPaired) sendRecordingControl(RECORDING_PAUSE);
            if(!sessions.pauseRecording()) {
                visibleMessages.push_back("Pause failed: " + sessions.lastError());
            } else if(slamLayer != nullptr) {
                slamLayer->resetSensorStream();
            }
        }
        if(ImGui::Button("Stop and save", {-1.0f, 0.0f})) stopSessionRecording();
    } else if(sessions.isPaused()) {
        ImGui::TextColored(ui::themeColors().warning, "Capture paused");
        if(ui::primaryButton("Resume capture", {-1.0f, 0.0f})) {
            if(!sessions.resumeRecording()) {
                visibleMessages.push_back("Resume failed: " + sessions.lastError());
            } else {
                if(slamLayer != nullptr) slamLayer->resetSensorStream();
                resetStreamMetrics();
                sessions.recordEvent("command", "Recording resume requested.");
                if(roverPaired) sendRecordingControl(RECORDING_RESUME);
            }
        }
        if(ImGui::Button("Stop and save", {-1.0f, 0.0f})) stopSessionRecording();
    } else if(sessions.isReplaying()) {
        const bool rebuilding =
            (hasReplayPendingPacket &&
             replayPendingMilliseconds <= replayPositionMilliseconds) ||
            (hasReplayPendingEvent &&
             replayPendingEventMilliseconds <= replayPositionMilliseconds);
        if(!sessions.replayIsComplete()) {
            ImGui::TextColored(ui::themeColors().warning,
                               "Recovered session (incomplete)");
        } else {
            ImGui::TextColored(ui::themeColors().accent,
                               replayRunning ? "Playing" : "Replay paused");
        }
        if(rebuilding) {
            ImGui::TextDisabled("Rebuilding map to playhead...");
        }
        if(ImGui::Button(replayRunning ? "Pause replay" : "Play replay",
                         {-1.0f, 0.0f})) {
            if(replayRunning) {
                updateReplay();
                replayRunning = false;
            } else {
                replayBaseMilliseconds = replayPositionMilliseconds;
                replayStartedAt = std::chrono::steady_clock::now();
                replayRunning = true;
            }
        }
        if(ImGui::Button("Close replay", {-1.0f, 0.0f})) {
            closeReplaySession();
        }
        const std::uint64_t duration = sessions.replayDurationMilliseconds();
        const std::uint64_t minimumPosition = 0;
        if(duration > 0) {
            ImGui::SetNextItemWidth(-1.0f);
            if(ImGui::SliderScalar("##replay-position", ImGuiDataType_U64,
                                   &replayPositionMilliseconds, &minimumPosition,
                                   &duration, "%llu ms")) {
                if(replayPositionMilliseconds > duration) {
                    replayPositionMilliseconds = duration;
                }
                replayBaseMilliseconds = replayPositionMilliseconds;
                replayStartedAt = std::chrono::steady_clock::now();
                hasReplayPendingPacket = false;
                hasReplayPendingEvent = false;
                replayReadErrorReported = false;
                resetStreamMetrics();
                if(!sessions.rewindReplayPackets() ||
                   !sessions.rewindReplayEvents()) {
                    visibleMessages.push_back("Replay seek failed: " + sessions.lastError());
                }
                replayStatus = {};
                hasReplayStatus = false;
                replayStatusMilliseconds = 0;
                replayMalformedStatusPackets = 0;
                replayLastStatusSequence = 0;
                hasReplayStatusSequence = false;
                replayStaleStatusPackets = 0;
                if(slamLayer != nullptr) slamLayer->resetLiveMap();
            }
        }
        static const double minimumReplaySpeed = 0.25;
        static const double maximumReplaySpeed = 4.0;
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::SliderScalar("Playback speed", ImGuiDataType_Double,
                            &replaySpeed, &minimumReplaySpeed,
                            &maximumReplaySpeed, "%.2fx");
    } else {
        ImGui::BeginDisabled(!statusContactFresh());
        if(ui::primaryButton("Start recording", {-1.0f, 0.0f})) startSessionRecording();
        ImGui::EndDisabled();
        if(!statusContactFresh()) ImGui::TextDisabled("Connect a rover to start capture.");
        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
        sectionLabel("Replay");
        pathField("Saved session", "##replay-session-path", replaySessionPath);
        ImGui::BeginDisabled(replaySessionPath.empty());
        if(ImGui::Button("Open replay", {-1.0f, 0.0f})) beginReplay();
        ImGui::EndDisabled();
    }
    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
    sectionLabel("Export map");
    pathField("PNG file", "##png-export-path", pngExportPath);
    const bool mapCanBeExported = slamLayer != nullptr && slamLayer->hasMap();
    ImGui::BeginDisabled(!mapCanBeExported);
    if(ImGui::Button("Export PNG", {-1.0f, 0.0f})) {
        std::string error;
        if(slamLayer->saveMapPng(pathFromUtf8(pngExportPath), error)) {
            visibleMessages.push_back("Map exported: " + pngExportPath);
        } else {
            visibleMessages.push_back("Map export failed: " + error);
        }
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
    }
    ImGui::EndDisabled();
    if(!sessions.lastError().empty()) {
        ImGui::TextWrapped("Session error: %s", sessions.lastError().c_str());
    }
}

void updateMotionKeepalive() {
    if(!motionTracker.pending() || !driveModeActive) return;
    const auto now = std::chrono::steady_clock::now();
    if(motionTracker.acknowledgementTimedOut(now)) {
        sendMotionStop();
        sendMoveCommand({}, true);
        visibleMessages.emplace_back(
            "Motion acknowledgement timed out; stop sent. Check the rover link before retrying.");
        while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        return;
    }
    if(now - lastMotionKeepaliveAt < motionKeepaliveInterval) return;
    MOTION_KEEPALIVE_CMD keepalive{motionTracker.commandId()};
    Packet packet(COMMAND, MOTION_KEEPALIVE, keepalive);
    if(enqueueMotionPacket(packet)) lastMotionKeepaliveAt = now;
}

void drawSensorHealth(const char* label, bool ready, const char* readyText,
                      const char* missingText) {
    ImGui::Text("%s", label);
    ImGui::SameLine(150.0f);
    const ImVec4 color = ready ? ui::themeColors().healthy
                               : ui::themeColors().error;
    ImGui::TextColored(color, "%s", ready ? readyText : missingText);
}

void drawOdometrySource(std::uint16_t healthFlags, std::uint16_t initErrorFlags) {
    (void)initErrorFlags;
    ImGui::TextUnformatted("Odometry");
    ImGui::SameLine(0.0f, 16.0f);
    if((healthFlags & SENSOR_LIDAR_IMU_ODOMETRY_READY) != 0) {
        ImGui::TextColored(ui::themeColors().healthy, "LIDAR + IMU");
    } else if((healthFlags & (SENSOR_MPU6050_STREAMING | SENSOR_MPU6050_CALIBRATED)) ==
              (SENSOR_MPU6050_STREAMING | SENSOR_MPU6050_CALIBRATED)) {
        ImGui::TextColored(ui::themeColors().warning, "IMU HEADING / LIDAR POSITION UNAVAILABLE");
    } else {
        ImGui::TextColored(ui::themeColors().error, "LIDAR / IMU NOT READY");
    }
}


void drawMotorDriverHealth(std::uint16_t healthFlags,
                           std::uint16_t initErrorFlags) {
    ImGui::Text("TB6612FNG");
    ImGui::SameLine(150.0f);
    if((initErrorFlags & SENSOR_ERROR_MOTOR_RUNTIME) != 0) {
        ImGui::TextColored(ui::themeColors().error, "Runtime fault");
    } else if((initErrorFlags & SENSOR_ERROR_MOTOR_TASK) != 0) {
        ImGui::TextColored(ui::themeColors().error, "Control initialization failed");
    } else if((healthFlags & SENSOR_MOTORS_READY) != 0) {
        ImGui::TextColored(ui::themeColors().healthy, "Ready");
    } else if((initErrorFlags & SENSOR_ERROR_MOTOR_INIT) != 0) {
        ImGui::TextColored(ui::themeColors().error, "Initialization error");
    } else {
        ImGui::TextColored(ui::themeColors().warning,
                           "NOT REPORTED / UPDATE FIRMWARE");
    }
}

void drawInfraredPreview() {
    if(slamLayer == nullptr) return;
    const auto frame = slamLayer->latestThermalFrame();
    if(!frame.valid || !ImGui::CollapsingHeader("Infrared image", ImGuiTreeNodeFlags_DefaultOpen)) return;
    const auto& mounts = slamLayer->config().mounts;
    const auto width = mapping::thermalImageWidth(mounts), height = mapping::thermalImageHeight(mounts);
    auto scale = slamLayer->thermalScale();
    if(scale.samples == 0) {
        mapping::ThermalStatistics fallback;
        fallback.observe(frame.temperaturesC);
        scale = fallback.scale();
    }
    const float pixelSize = std::max(1.0f, std::min({7.0f,
        ImGui::GetContentRegionAvail().x / static_cast<float>(width), 180.0f / static_cast<float>(height)}));
    const auto origin = ImGui::GetCursorScreenPos();
    ImGui::Dummy({static_cast<float>(width) * pixelSize, static_cast<float>(height) * pixelSize});
    const bool hovered = ImGui::IsItemHovered();
    auto* drawList = ImGui::GetWindowDrawList();
    for(std::size_t y = 0; y < height; ++y) for(std::size_t x = 0; x < width; ++x) {
        const float value = mapping::thermalImagePixel(frame, mounts, x, y);
        const auto color = mapping::thermalColor(value, scale);
        const ImVec2 start{origin.x + static_cast<float>(x) * pixelSize,
                           origin.y + static_cast<float>(y) * pixelSize};
        drawList->AddRectFilled(start, {start.x + pixelSize, start.y + pixelSize},
            IM_COL32(static_cast<int>(color.r), static_cast<int>(color.g), static_cast<int>(color.b), 255));
    }
    const float stripTop = static_cast<float>(height / 2 - 3) * pixelSize;
    drawList->AddRect({origin.x, origin.y + stripTop},
        {origin.x + static_cast<float>(width) * pixelSize, origin.y + stripTop + 6 * pixelSize},
        IM_COL32(9, 12, 15, 180), 0.0f, 1.5f, ImDrawFlags_None);
    if(hovered) {
        const auto mouse = ImGui::GetIO().MousePos;
        const auto x = std::min(static_cast<std::size_t>(std::max(0.0f, (mouse.x - origin.x) / pixelSize)), width - 1);
        const auto y = std::min(static_cast<std::size_t>(std::max(0.0f, (mouse.y - origin.y) / pixelSize)), height - 1);
        ImGui::SetTooltip("Pixel %zu, %zu: %.2f C", x, y, mapping::thermalImagePixel(frame, mounts, x, y));
    }
    const auto [minimum, maximum] = std::minmax_element(frame.temperaturesC.begin(), frame.temperaturesC.end());
    ImGui::Text("Range: %.1f to %.1f C", *minimum, *maximum);
    ImGui::TextDisabled("Run mean: %.2f C", scale.meanC);
}

void drawTelemetryPanel() {
    if(slamLayer != nullptr && slamLayer->hasMap()) {
        ImGui::TextColored(slamLayer->localizationTracking() ? ui::themeColors().healthy : ui::themeColors().warning,
            "Map alignment: %s", slamLayer->localizationTracking() ? "Tracking" : "Not integrating");
        ImGui::TextDisabled("Match: %.2f    Rejected scans: %llu", slamLayer->localizationScore(),
            static_cast<unsigned long long>(slamLayer->rejectedScanCount()));
    }
    sectionLabel("Telemetry");
    if(sessions.isReplaying()) {
        if(!hasReplayStatus) {
            ImGui::TextWrapped("No valid rover status snapshot has been replayed at this point.");
            if(replayMalformedStatusPackets != 0) {
                ImGui::TextColored(ui::themeColors().error,
                    "INVALID REPLAY STATUS / %llu",
                    static_cast<unsigned long long>(replayMalformedStatusPackets));
            }
            return;
        }
        const std::uint64_t statusAge = replayPositionMilliseconds >= replayStatusMilliseconds
            ? replayPositionMilliseconds - replayStatusMilliseconds : 0;
        ImGui::Text("REPLAY STATUS / %s", formatElapsedTime(
            replayStatusMilliseconds).c_str());
        ImGui::TextDisabled("Snapshot age at playhead / %llu ms",
                            static_cast<unsigned long long>(statusAge));
        ImGui::Spacing();
        if(replayStatus.batteryP >= 0.0f)
            ImGui::Text("BATTERY / %.1f%%", replayStatus.batteryP);
        else
            ImGui::TextUnformatted("BATTERY / UNAVAILABLE");
        if(replayStatus.batteryV >= 0.0f)
            ImGui::Text("PACK VOLTAGE / %.2f V", replayStatus.batteryV);
        else
            ImGui::TextUnformatted("PACK VOLTAGE / UNAVAILABLE");
        ImGui::Text("ROVER ADDRESS / %s:%u", replayStatus.ip,
                    static_cast<unsigned>(replayStatus.port));
        ImGui::Text("POSE / X %.1f cm  Y %.1f cm",
                    replayStatus.xCentimeters, replayStatus.yCentimeters);
        ImGui::Text("HEADING / %.1f deg clockwise",
                    replayStatus.headingDegreesClockwise);
        ImGui::Text("Uptime: %u s",
                    static_cast<unsigned>(replayStatus.uptimeMilliseconds / 1000U));
        ImGui::Spacing();
        const std::uint16_t health = replayStatus.sensorHealthFlags;
        const std::uint16_t initErrors = replayStatus.sensorInitErrorFlags;
        drawOdometrySource(health, initErrors);
        ImGui::TextUnformatted("RECORDED SENSOR HEALTH");
        ImGui::Separator();
        drawMotorDriverHealth(health, initErrors);
        const bool i2cBusError = (initErrors & SENSOR_ERROR_I2C_BUS) != 0;
        const char* batteryUnavailableText = i2cBusError ? "I2C BUS ERROR"
            : (initErrors & SENSOR_ERROR_BATTERY_MISSING) != 0
                ? "Not detected" : "UNAVAILABLE";
        drawSensorHealth("MAX17048", (health & SENSOR_BATTERY_READY) != 0,
                         "Online", batteryUnavailableText);
        const bool imuInitError =
            (initErrors & (SENSOR_ERROR_I2C_BUS |
                           SENSOR_ERROR_MPU6050_MISSING |
                           SENSOR_ERROR_MPU6050_CONFIG |
                           SENSOR_ERROR_MPU6050_TASK)) != 0;
        drawSensorHealth("MPU-6050",
                         !imuInitError &&
                             (health & SENSOR_MPU6050_STREAMING) != 0,
                         imuInitError ? "Ready" :
                             (health & SENSOR_MPU6050_CALIBRATED) != 0
                                 ? "Ready" : "Calibrating",
                         imuInitError ? "Initialization error" : "Missing / no data");
        const bool thermalDetected =
            (health & SENSOR_MLX90640_DETECTED) != 0;
        const bool thermalStreaming =
            (health & SENSOR_MLX90640_STREAMING) != 0;
        const bool replayCaptureActive = replayStatus.recordingState == 1;
        const bool replayCapturePaused = replayStatus.recordingState == 2;
        if((initErrors & (SENSOR_ERROR_I2C_BUS |
                          SENSOR_ERROR_MLX90640_CONFIG |
                          SENSOR_ERROR_MLX90640_TASK)) != 0) {
            drawSensorHealth("MLX90640", false, "Ready", "Initialization error");
        } else if(thermalStreaming) {
            drawSensorHealth("MLX90640", true, "Streaming", "Not detected");
        } else if(thermalDetected && replayCapturePaused) {
            drawSensorHealth("MLX90640", true, "Paused", "Not detected");
        } else if(thermalDetected && !replayCaptureActive) {
            drawSensorHealth("MLX90640", true, "Ready / idle", "Not detected");
        } else {
            drawSensorHealth("MLX90640", false, "Ready / idle",
                             thermalDetected ? "NO DATA DURING CAPTURE"
                                             : "Not detected");
        }
        if(thermalDetected && replayCaptureActive && !thermalStreaming) {
            ImGui::TextWrapped("MLX90640 is detected but no complete frames are arriving during capture.");
        }
        drawInfraredPreview();
        const bool lidarInitError =
            (initErrors & (SENSOR_ERROR_LD19_UART | SENSOR_ERROR_LD19_TASK)) != 0;
        const bool lidarStreaming = (health & SENSOR_LD19_STREAMING) != 0;
        drawSensorHealth("FHL-LD19",
                         !lidarInitError && lidarStreaming,
                         lidarInitError ? "Ready" : "Streaming",
                         lidarInitError ? "Initialization error" : "No valid data");
        if(!lidarInitError && !lidarStreaming) {
            ImGui::TextWrapped("LD19 has not sent a valid packet; check sensor power and TX/PWM wiring.");
        }
        ImGui::Text("INIT ERROR FLAGS / 0x%04X", initErrors);
        ImGui::Text("STALE STATUS PACKETS / %llu",
                    static_cast<unsigned long long>(replayStaleStatusPackets));
        if(initErrors != 0) {
            ImGui::Spacing();
            ImGui::TextColored(ui::themeColors().error,
                               "RECORDED ROVER DIAGNOSTICS");
            if((initErrors & SENSOR_ERROR_I2C_BUS) != 0)
                ImGui::TextWrapped("I2C bus setup or device probe failed.");
            if((initErrors & SENSOR_ERROR_BATTERY_MISSING) != 0)
                ImGui::TextWrapped("MAX17048 battery monitor was not found.");
            if((initErrors & SENSOR_ERROR_MPU6050_MISSING) != 0)
                ImGui::TextWrapped("MPU-6050 was not found at 0x68 or 0x69.");
            if((initErrors & SENSOR_ERROR_MPU6050_CONFIG) != 0)
                ImGui::TextWrapped("MPU-6050 identity check or register setup failed.");
            if((initErrors & SENSOR_ERROR_MPU6050_TASK) != 0)
                ImGui::TextWrapped("MPU-6050 sampling task could not start.");
            if((initErrors & SENSOR_ERROR_MLX90640_MISSING) != 0)
                ImGui::TextWrapped("MLX90640 was not found at 0x33.");
            if((initErrors & SENSOR_ERROR_MLX90640_CONFIG) != 0)
                ImGui::TextWrapped("MLX90640 calibration or setup failed.");
            if((initErrors & SENSOR_ERROR_MLX90640_TASK) != 0)
                ImGui::TextWrapped("MLX90640 acquisition task could not start.");
            if((initErrors & SENSOR_ERROR_MOTOR_INIT) != 0)
                ImGui::TextWrapped("TB6612FNG GPIO or PWM initialization failed; drive is disabled.");
            if((initErrors & SENSOR_ERROR_MOTOR_RUNTIME) != 0)
                ImGui::TextWrapped("TB6612FNG GPIO or PWM operation failed at runtime; standby was asserted and drive is disabled.");
            if((initErrors & SENSOR_ERROR_MOTOR_TASK) != 0)
                ImGui::TextWrapped("Motor control queue or task could not start; TB6612 standby was asserted and drive is disabled.");
            if((initErrors & SENSOR_ERROR_LD19_UART) != 0)
                ImGui::TextWrapped("LD19 PWM GPIO or UART initialization failed.");
            if((initErrors & SENSOR_ERROR_LD19_TASK) != 0)
                ImGui::TextWrapped("LD19 reader task could not start.");
            if((initErrors & SENSOR_ERROR_NETWORK_TASK) != 0)
                ImGui::TextWrapped("A UDP receive, send, or status task could not start; network control or telemetry may be unavailable.");
        }
        ImGui::Spacing();
    ImGui::Separator();
    sectionLabel("Transport counters");
    ImGui::Text("Rover TX drops: %u",
                    static_cast<unsigned>(replayStatus.streamDroppedPackets));
        if(replayMalformedStatusPackets != 0) {
            ImGui::TextColored(ui::themeColors().error,
                "INVALID REPLAY STATUS / %llu",
                static_cast<unsigned long long>(replayMalformedStatusPackets));
        }
        ImGui::Spacing();
        return;
    }
    if(!hasStatus) {
        ImGui::TextWrapped("Awaiting status packets from the rover.");
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto statusAge = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastStatusAt);
    const bool statusFresh = lastStatusAt.time_since_epoch().count() != 0 &&
                             statusContactFresh();

    const bool columns = ImGui::GetContentRegionAvail().x >= 730.0f &&
        ImGui::BeginTable("Live diagnostics", 2, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_BordersInnerV);
    if(columns) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
    }
    propertyRow("Last status", statusFresh ? std::format("{} ms ago", statusAge.count()) : "Stale / offline");
    propertyRow("Battery", roverStatus.batteryP >= 0.0f ? std::format("{:.1f}%", roverStatus.batteryP) : "Unavailable");
    propertyRow("Voltage", roverStatus.batteryV >= 0.0f ? std::format("{:.2f} V", roverStatus.batteryV) : "Unavailable");
    propertyRow("Address", std::format("{}:{}", roverStatus.ip, roverStatus.port));
    propertyRow("Position", std::format("X {:.1f}   Y {:.1f} cm", roverStatus.xCentimeters, roverStatus.yCentimeters));
    propertyRow("Heading", std::format("{:.1f} deg CW", roverStatus.headingDegreesClockwise));
    ImGui::Spacing();
    drawOdometrySource(roverStatus.sensorHealthFlags,
                       roverStatus.sensorInitErrorFlags);
    ImGui::Text("Uptime: %u s",
                static_cast<unsigned>(roverStatus.uptimeMilliseconds / 1000U));
    if((roverStatus.sensorHealthFlags & SENSOR_MPU6050_CALIBRATED) != 0) {
        ImGui::Text("Gyro bias: %.3f deg/s    Axis sign: %.0f",
                    roverStatus.imuGyroBiasDegreesPerSecond,
                    roverStatus.imuYawDirection);
    } else {
        ImGui::TextDisabled("Gyro bias: calibrating or unavailable");
    }
    const char* motionLabel = "Idle";
    if(roverStatus.motionActive != 0) motionLabel = "RUNNING";
    else if(roverStatus.motionResult == MOTION_RESULT_COMPLETE) motionLabel = "COMPLETE";
    else if(roverStatus.motionResult == MOTION_RESULT_STALLED) motionLabel = "STALLED";
    else if(roverStatus.motionResult == MOTION_RESULT_TIMEOUT) motionLabel = "TIMED OUT";
    else if(roverStatus.motionResult == MOTION_RESULT_CANCELLED) motionLabel = "CANCELLED";
    else if(roverStatus.motionResult == MOTION_RESULT_SENSOR_FAULT) motionLabel = "SENSOR FAULT";
    else if(roverStatus.motionResult == MOTION_RESULT_MOTOR_FAULT) motionLabel = "MOTOR FAULT";
    ImGui::Text("Motion: %s", motionLabel);
    if(roverStatus.motionCommandId != 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("#%u", static_cast<unsigned>(roverStatus.motionCommandId));
    }
    ImGui::Spacing();
    sectionLabel("Sensors");
    ImGui::Separator();
    const uint16_t health = roverStatus.sensorHealthFlags;
    const uint16_t initErrors = roverStatus.sensorInitErrorFlags;
    drawMotorDriverHealth(health, initErrors);
    const bool i2cBusError = (initErrors & SENSOR_ERROR_I2C_BUS) != 0;
    const char* batteryUnavailableText = i2cBusError ? "I2C BUS ERROR"
        : (initErrors & SENSOR_ERROR_BATTERY_MISSING) != 0
            ? "Not detected" : "UNAVAILABLE";
    drawSensorHealth("MAX17048", (health & SENSOR_BATTERY_READY) != 0,
                     "Online", batteryUnavailableText);
    const bool imuStreaming = (health & SENSOR_MPU6050_STREAMING) != 0;
    const bool imuCalibrated = (health & SENSOR_MPU6050_CALIBRATED) != 0;
    const bool imuInitError =
        (initErrors & (SENSOR_ERROR_I2C_BUS |
                       SENSOR_ERROR_MPU6050_MISSING |
                       SENSOR_ERROR_MPU6050_CONFIG |
                       SENSOR_ERROR_MPU6050_TASK)) != 0;
    ImGui::Text("MPU-6050");
    ImGui::SameLine(150.0f);
    if(imuInitError) {
        ImGui::TextColored(ui::themeColors().error, "Initialization error");
    } else if(imuStreaming && imuCalibrated) {
        ImGui::TextColored(ui::themeColors().healthy, "Ready");
    } else if(imuStreaming) {
        ImGui::TextColored(ui::themeColors().warning, "Calibrating");
    } else {
        ImGui::TextColored(ui::themeColors().error, "Missing / no data");
    }
    const bool thermalDetected = (health & SENSOR_MLX90640_DETECTED) != 0;
    const bool thermalStreaming = (health & SENSOR_MLX90640_STREAMING) != 0;
    const bool liveCaptureActive = roverStatus.recordingState == 1;
    const bool liveCapturePaused = roverStatus.recordingState == 2;
    ImGui::Text("MLX90640");
    ImGui::SameLine(150.0f);
    if((initErrors & (SENSOR_ERROR_I2C_BUS |
                      SENSOR_ERROR_MLX90640_CONFIG |
                      SENSOR_ERROR_MLX90640_TASK)) != 0) {
        ImGui::TextColored(ui::themeColors().error, "Initialization error");
    } else if(thermalStreaming) {
        ImGui::TextColored(ui::themeColors().healthy, "Streaming");
    } else if(thermalDetected && liveCapturePaused) {
        ImGui::TextColored(ui::themeColors().warning, "Paused");
    } else if(thermalDetected && !liveCaptureActive) {
        ImGui::TextColored(ui::themeColors().warning, "Ready / idle");
    } else if(thermalDetected) {
        ImGui::TextColored(ui::themeColors().error, "NO DATA DURING CAPTURE");
    } else {
        ImGui::TextColored(ui::themeColors().error, "Not detected");
    }
    if(thermalDetected && liveCaptureActive && !thermalStreaming) {
        ImGui::TextWrapped("MLX90640 is detected but no complete frames are arriving during capture.");
    }
    drawInfraredPreview();
    if(roverThermalDiagnostics.has_value()) {
        const auto& capture = *roverThermalDiagnostics;
        static constexpr const char* stages[]{"Idle", "WAITING FOR SENSOR", "READING",
            "CONVERTING", "WAITING FOR PARTNER PAGE", "Streaming", "READ ERROR",
            "INVALID TEMPERATURE"};
        const bool fresh = std::chrono::steady_clock::now() - lastThermalDiagnosticsAt <
                           std::chrono::seconds(2);
        ImGui::Text("Thermal capture: %s%s", stages[static_cast<unsigned>(capture.stage)],
                    fresh ? "" : " / STALE");
        ImGui::Text("Subpages: %u / %u    Frames: %u",
                    capture.subpage0Count, capture.subpage1Count, capture.completedFrames);
        if(capture.subpage0Count != 0 || capture.subpage1Count != 0) {
            ImGui::TextDisabled("Last page: %u    Age: %u ms    Status: 0x%04X",
                static_cast<unsigned>(capture.lastSubpage),
                capture.timestampMilliseconds - capture.lastSubpageMilliseconds,
                static_cast<unsigned>(capture.statusRegister));
        }
        if(capture.readErrors != 0 || capture.invalidFrames != 0) {
            ImGui::TextColored(ui::themeColors().warning,
                "READ ERRORS / %u    INVALID FRAMES / %u    LAST RESULT / %d",
                capture.readErrors, capture.invalidFrames, static_cast<int>(capture.lastResult));
            if(capture.invalidPixel != 0xffff)
                ImGui::TextDisabled("INVALID THERMAL PIXEL / %u", static_cast<unsigned>(capture.invalidPixel));
        }
    }
    const bool lidarInitError =
        (initErrors & (SENSOR_ERROR_LD19_UART | SENSOR_ERROR_LD19_TASK)) != 0;
    const bool lidarStreaming = (health & SENSOR_LD19_STREAMING) != 0;
    drawSensorHealth("FHL-LD19",
                     !lidarInitError && lidarStreaming,
                     lidarInitError ? "Ready" : "Streaming",
                     lidarInitError ? "Initialization error" : "No valid data");
    if(!lidarInitError && !lidarStreaming) {
        ImGui::TextWrapped("LD19 has not sent a valid packet; check sensor power and TX/PWM wiring.");
    }
    ImGui::Spacing();
    ImGui::Separator();
    if(columns) ImGui::TableSetColumnIndex(1);
    sectionLabel("Data stream");
    if(roverRecordingState == 1) {
        ImGui::TextColored(ui::themeColors().error, "Active");
        const auto streamNow = std::chrono::steady_clock::now();
        drawStreamMetric("LD19 / S", streamMetrics.lidarPackets,
                         streamNow, std::chrono::milliseconds(750));
        drawStreamMetric("THERMAL STARTS / S", streamMetrics.thermalFrames,
                         streamNow, std::chrono::milliseconds(1500));
        if(slamLayer != nullptr) {
            ImGui::Text("Complete thermal frames: %llu",
                static_cast<unsigned long long>(
                    slamLayer->completedThermalFrameCount()));
            const std::uint64_t incompleteThermalFrames =
                slamLayer->incompleteThermalFrameCount();
            if(incompleteThermalFrames != 0) {
                ImGui::TextColored(ui::themeColors().warning,
                    "INCOMPLETE THERMAL FRAMES / %llu",
                    static_cast<unsigned long long>(incompleteThermalFrames));
            }
        }
        drawStreamMetric("FUSED POSES / S", streamMetrics.fusedOdometry,
                         streamNow, std::chrono::milliseconds(500));
        drawStreamMetric("IMU / S", streamMetrics.imuSamples,
                         streamNow, std::chrono::milliseconds(500));
    } else if(roverRecordingState == 2) {
        ImGui::TextColored(ui::themeColors().warning, "Paused");
    } else {
        ImGui::TextDisabled("Stopped");
    }
    ImGui::Spacing();
    ImGui::Separator();
    sectionLabel("Transport counters");
    ImGui::Text("Rover TX drops: %u",
                static_cast<unsigned>(roverStatus.streamDroppedPackets));
    if(slamLayer != nullptr) {
        ImGui::Text("Map queue drops: %llu",
                    static_cast<unsigned long long>(
                        slamLayer->droppedIncomingFrameCount()));
        const std::uint64_t lidarAngleDrops =
            slamLayer->rejectedLidarAnglePacketCount();
        if(lidarAngleDrops != 0) {
            ImGui::TextColored(ui::themeColors().warning,
                "LIDAR ANGLE-ORDER PACKETS REJECTED / %llu",
                static_cast<unsigned long long>(lidarAngleDrops));
        }
        const std::uint64_t lidarTimestampDrops =
            slamLayer->rejectedLidarTimestampPacketCount();
        if(lidarTimestampDrops != 0) {
            ImGui::TextColored(ui::themeColors().warning,
                "LIDAR OLD-TIMESTAMP PACKETS REJECTED / %llu",
                static_cast<unsigned long long>(lidarTimestampDrops));
        }
    }
    ImGui::Text("Receive queue drops: %llu",
                static_cast<unsigned long long>(
                    server.getDroppedReceivePacketCount()));
    ImGui::Text("Invalid datagrams: %llu",
                static_cast<unsigned long long>(
                    server.getRejectedDatagramCount()));
    ImGui::Text("Invalid acknowledgements: %llu",
                static_cast<unsigned long long>(malformedAcknowledgements));
    ImGui::Text("Duplicate packets: %llu",
                static_cast<unsigned long long>(duplicateRoverPackets));
    ImGui::Text("Old packets rejected: %llu",
                static_cast<unsigned long long>(staleRoverPackets));
    ImGui::Text("Reordered packets: %llu",
                static_cast<unsigned long long>(reorderedRoverPackets));
    const std::uint64_t knownSequenceMask = roverSequenceHistorySize == 64
        ? ~std::uint64_t{0}
        : ((1ULL << roverSequenceHistorySize) - 1ULL);
    const std::uint64_t unresolvedSequenceGaps = roverSequenceHistorySize -
        static_cast<unsigned int>(std::popcount(
            roverSequenceWindow & knownSequenceMask));
    if(unresolvedSequenceGaps != 0 || confirmedMissingRoverPackets != 0) {
        ImGui::TextColored(ui::themeColors().warning,
            "LAPTOP RX MISSING SEQUENCES / %llu REORDERABLE, %llu AGED OUT",
            static_cast<unsigned long long>(unresolvedSequenceGaps),
            static_cast<unsigned long long>(confirmedMissingRoverPackets));
    }
    ImGui::Text("Stale status ignored: %llu",
                static_cast<unsigned long long>(staleStatusPackets));
    if(malformedStatusPackets != 0) {
        ImGui::TextColored(ui::themeColors().error,
            "MALFORMED STATUS PACKETS / %llu",
            static_cast<unsigned long long>(malformedStatusPackets));
    }
    if(streamMetrics.malformedSensorPackets != 0) {
        ImGui::TextColored(ui::themeColors().error,
            "MALFORMED SENSOR PACKETS / %llu",
            static_cast<unsigned long long>(streamMetrics.malformedSensorPackets));
        const std::pair<PacketID, const char*> sensorKinds[]{
            {LIDAR_PACKET, "LIDAR"}, {THERMAL_CHUNK, "THERMAL"},
            {IMU_SAMPLE, "IMU"}, {FUSED_ODOMETRY, "FUSED POSE"}};
        for(const auto& [id, name] : sensorKinds) {
            if(streamMetrics.malformedById[id] != 0)
                ImGui::TextDisabled("REJECTED %s PAYLOADS / %llu", name,
                    static_cast<unsigned long long>(streamMetrics.malformedById[id]));
        }
    }
    if(initErrors != 0) {
        ImGui::Spacing();
        ImGui::TextColored(ui::themeColors().error, "Reported faults");
        if((initErrors & SENSOR_ERROR_I2C_BUS) != 0)
            ImGui::TextWrapped("I2C bus setup or device probe failed.");
        if((initErrors & SENSOR_ERROR_BATTERY_MISSING) != 0)
            ImGui::TextWrapped("MAX17048 battery monitor was not found.");
        if((initErrors & SENSOR_ERROR_MPU6050_MISSING) != 0)
            ImGui::TextWrapped("MPU-6050 was not found at 0x68 or 0x69.");
        if((initErrors & SENSOR_ERROR_MPU6050_CONFIG) != 0)
            ImGui::TextWrapped("MPU-6050 identity check or register setup failed.");
        if((initErrors & SENSOR_ERROR_MPU6050_TASK) != 0)
            ImGui::TextWrapped("MPU-6050 sampling task could not start.");
        if((initErrors & SENSOR_ERROR_MLX90640_MISSING) != 0)
            ImGui::TextWrapped("MLX90640 was not found at 0x33.");
        if((initErrors & SENSOR_ERROR_MLX90640_CONFIG) != 0)
            ImGui::TextWrapped("MLX90640 calibration, setup, or capture task initialization failed.");
        if((initErrors & SENSOR_ERROR_MLX90640_TASK) != 0)
            ImGui::TextWrapped("MLX90640 acquisition task could not start.");
        if((initErrors & SENSOR_ERROR_MOTOR_INIT) != 0)
            ImGui::TextWrapped("TB6612FNG GPIO or PWM initialization failed; drive is disabled.");
        if((initErrors & SENSOR_ERROR_MOTOR_RUNTIME) != 0)
            ImGui::TextWrapped("TB6612FNG GPIO or PWM operation failed at runtime; standby was asserted and drive is disabled.");
        if((initErrors & SENSOR_ERROR_MOTOR_TASK) != 0)
            ImGui::TextWrapped("Motor control queue or task could not start; TB6612 standby was asserted and drive is disabled.");
        if((initErrors & SENSOR_ERROR_LD19_UART) != 0)
            ImGui::TextWrapped("LD19 PWM GPIO or UART initialization failed.");
        if((initErrors & SENSOR_ERROR_LD19_TASK) != 0)
            ImGui::TextWrapped("LD19 reader task could not start.");
        if((initErrors & SENSOR_ERROR_NETWORK_TASK) != 0)
            ImGui::TextWrapped("A UDP receive, send, or status task could not start; network control or telemetry may be unavailable.");
    }
    if(columns) ImGui::EndTable();
}

void drawActivityPanel() {
    sectionLabel("Activity log");
    if(visibleMessages.empty()) {
        ImGui::TextWrapped("No messages received yet.");
        return;
    }

    for(auto message = visibleMessages.rbegin(); message != visibleMessages.rend(); ++message) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextWrapped("%s", message->c_str());
        ImGui::PopTextWrapPos();
        ImGui::Spacing();
    }
}

void drawMapCanvas(ImVec2 size) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, {0.0f, 0.0f, 0.0f, 0.0f});
    ImGui::BeginChild("SLAM map canvas", size, ImGuiChildFlags_Borders);
    ImGui::PopStyleColor();

    const ImVec2 origin = ImGui::GetWindowPos();
    const ImVec2 canvasSize = ImGui::GetWindowSize();
    const ImVec2 bottomRight{origin.x + canvasSize.x, origin.y + canvasSize.y};
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->PushClipRect(origin, bottomRight, true);
    drawList->AddRectFilled(origin, bottomRight, ThermalMapRenderer::canvasBackground);

    const bool hasRenderedMap = slamLayer != nullptr &&
        slamLayer->drawMap(drawList, origin, bottomRight);
    const bool hasMapData = slamLayer != nullptr && slamLayer->hasMap();

    const ImU32 gridColor = IM_COL32(220, 226, 232, 17);
    const ImU32 frameColor = IM_COL32(220, 226, 232, 42);
    if(!hasRenderedMap) {
        const float gridSpacing = (std::max)(22.0f, canvasSize.x / 16.0f);
        for(float x = origin.x; x < bottomRight.x; x += gridSpacing) {
            drawList->AddLine({x, origin.y}, {x, bottomRight.y}, gridColor);
        }
        for(float y = origin.y; y < bottomRight.y; y += gridSpacing) {
            drawList->AddLine({origin.x, y}, {bottomRight.x, y}, gridColor);
        }
    }
    drawList->AddRect(origin, bottomRight, frameColor);

    const ImVec2 center{origin.x + canvasSize.x * 0.5f,
                        origin.y + canvasSize.y * 0.5f};
    if(!hasRenderedMap) {
        const ImU32 markerColor = IM_COL32(235, 239, 244, 95);
        drawList->AddLine({center.x - 12.0f, center.y},
                          {center.x + 12.0f, center.y}, markerColor);
        drawList->AddLine({center.x, center.y - 12.0f},
                          {center.x, center.y + 12.0f}, markerColor);
        drawList->AddRect({center.x - 4.0f, center.y - 4.0f},
                          {center.x + 4.0f, center.y + 4.0f}, markerColor);
    }

    if(!hasRenderedMap) {
        const char* waiting = "Waiting for scan data";
        const ImVec2 waitingSize = ImGui::CalcTextSize(waiting);
        if(canvasSize.y >= 175.0f && canvasSize.x >= waitingSize.x + 24.0f) {
            drawList->AddText({center.x - waitingSize.x * 0.5f,
                               center.y + 16.0f},
                              IM_COL32(200, 207, 215, 190), waiting);
        } else {
            drawList->AddText({center.x - waitingSize.x * 0.5f,
                               center.y - waitingSize.y * 0.5f},
                              IM_COL32(200, 207, 215, 190), waiting);
        }
    }
    if(!hasMapData && canvasSize.x > 420.0f && canvasSize.y > 80.0f) {
        drawList->AddText({origin.x + 14.0f, origin.y + 14.0f},
                          IM_COL32(200, 209, 220, 255), "Waiting for sensor data");
        const char* hint = sessions.isReplaying() ? "No complete scans at this playhead."
            : sessions.isRecording() ? "Waiting for the first complete scan."
            : "Start capture to build the map.";
        drawList->AddText({origin.x + 14.0f, origin.y + 36.0f},
                          IM_COL32(151, 166, 183, 255), hint);
    }
    if(slamLayer != nullptr && canvasSize.x > 190.0f && canvasSize.y > 32.0f) {
        const float cellSize = slamLayer->config().centimetersPerCell;
        const std::string scale = std::format("Grid: {:.1f} cm", cellSize);
        const ImVec2 scaleSize = ImGui::CalcTextSize(scale.c_str());
        if(scaleSize.x + 24.0f <= canvasSize.x) {
            drawList->AddText({origin.x + 12.0f,
                               bottomRight.y - scaleSize.y - 10.0f},
                              IM_COL32(150, 158, 168, 185), scale.c_str());
        }
    }

    const float textHeight = ImGui::GetTextLineHeight();
    if(canvasSize.x >= 265.0f && canvasSize.y >= textHeight * 3.0f + 36.0f) {
        const float legendX = bottomRight.x - 206.0f;
        const float legendY = origin.y + 12.0f;
        drawList->AddText({legendX, legendY}, IM_COL32(190, 198, 207, 205), "Temperature / C");
        constexpr float swatchWidth = 42.0f;
        constexpr float swatchSpacing = 70.0f;
        constexpr float swatchHeight = 5.0f;
        const float swatchY = legendY + textHeight + 4.0f;
        const float labelY = swatchY + swatchHeight + 6.0f;
        constexpr ImU32 cool = IM_COL32(25, 115, 255, 245);
        constexpr ImU32 neutral = IM_COL32(224, 232, 240, 245);
        constexpr ImU32 warm = IM_COL32(255, 124, 42, 245);
        drawList->AddRectFilled({legendX, swatchY},
                                {legendX + swatchWidth, swatchY + swatchHeight}, cool);
        drawList->AddRectFilled({legendX + swatchSpacing, swatchY},
                                {legendX + swatchSpacing + swatchWidth,
                                 swatchY + swatchHeight}, neutral);
        drawList->AddRectFilled({legendX + swatchSpacing * 2.0f, swatchY},
                                {legendX + swatchSpacing * 2.0f + swatchWidth,
                                 swatchY + swatchHeight}, warm);
        drawList->AddText({legendX, labelY}, IM_COL32(168, 178, 187, 185), "Cool");
        drawList->AddText({legendX + swatchSpacing, labelY},
                          IM_COL32(168, 178, 187, 185), "Mean");
        drawList->AddText({legendX + swatchSpacing * 2.0f, labelY},
                          IM_COL32(168, 178, 187, 185), "Warm");
        const auto thermalScale = slamLayer != nullptr ? slamLayer->thermalScale() : mapping::ThermalScale{};
        if(thermalScale.samples != 0) {
            const float values[]{thermalScale.lowC, thermalScale.meanC, thermalScale.highC};
            for(int i = 0; i < 3; ++i) {
                const auto value = std::format("{:.1f} C", values[i]);
                drawList->AddText({legendX + swatchSpacing * static_cast<float>(i), labelY + textHeight + 3.0f},
                                  IM_COL32(198, 208, 218, 220), value.c_str());
            }
        }
    }
    if(canvasSize.x >= 400.0f && canvasSize.y >= 48.0f) {
        const ImVec2 unknownTextSize = ImGui::CalcTextSize("Unknown");
        const ImVec2 freeTextSize = ImGui::CalcTextSize("Free");
        const ImVec2 occupiedTextSize = ImGui::CalcTextSize("Occupied");
        const float keyX = origin.x + 132.0f;
        const float keyY = bottomRight.y - textHeight - 10.0f;
        const float swatchY = keyY + (textHeight - 10.0f) * 0.5f;
        const float keyWidth = 14.0f + unknownTextSize.x + 12.0f +
                               14.0f + freeTextSize.x + 12.0f +
                               14.0f + occupiedTextSize.x + 8.0f;
        drawList->AddRectFilled({keyX - 5.0f, keyY - 3.0f},
                                {keyX + keyWidth, keyY + textHeight + 3.0f},
                                IM_COL32(18, 24, 31, 220), 3.0f);
        float cursorX = keyX;
        drawList->AddRectFilled({cursorX, swatchY},
                                {cursorX + 10.0f, swatchY + 10.0f},
                                IM_COL32(20, 26, 34, 255));
        drawList->AddRect({cursorX, swatchY},
                          {cursorX + 10.0f, swatchY + 10.0f},
                          IM_COL32(110, 120, 132, 220));
        cursorX += 14.0f;
        drawList->AddText({cursorX, keyY}, IM_COL32(204, 211, 218, 225), "Unknown");
        cursorX += unknownTextSize.x + 12.0f;
        drawList->AddRectFilled({cursorX, swatchY},
                                {cursorX + 10.0f, swatchY + 10.0f},
                                IM_COL32(98, 108, 120, 80));
        cursorX += 14.0f;
        drawList->AddText({cursorX, keyY}, IM_COL32(204, 211, 218, 225), "Free");
        cursorX += freeTextSize.x + 12.0f;
        drawList->AddRectFilled({cursorX, swatchY},
                                {cursorX + 10.0f, swatchY + 10.0f},
                                IM_COL32(139, 148, 160, 255));
        cursorX += 14.0f;
        drawList->AddText({cursorX, keyY}, IM_COL32(204, 211, 218, 225), "Occupied");
    }
    drawList->PopClipRect();
    ImGui::EndChild();
}

bool hasMappingCalibrationChanges(const mapping::SlamConfig& left,
                                  const mapping::SlamConfig& right) {
    const auto mountChanged = [](const mapping::SensorMount& first,
                                 const mapping::SensorMount& second) {
        return first.xCm != second.xCm || first.yCm != second.yCm ||
               first.zCm != second.zCm ||
               first.headingRadiansClockwise != second.headingRadiansClockwise;
    };
    return mountChanged(left.mounts.lidar, right.mounts.lidar) ||
           mountChanged(left.mounts.thermal, right.mounts.thermal) ||
           left.mounts.thermalHorizontalFovDegrees !=
               right.mounts.thermalHorizontalFovDegrees ||
           left.mounts.thermalVerticalFovDegrees !=
               right.mounts.thermalVerticalFovDegrees ||
           left.mounts.mirrorThermalColumns !=
               right.mounts.mirrorThermalColumns ||
           left.mounts.thermalRotationQuarterTurns != right.mounts.thermalRotationQuarterTurns;
}

bool validMappingCalibration(const mapping::SlamConfig& config) {
    const auto mountValid = [](const mapping::SensorMount& mount) {
        return std::isfinite(mount.xCm) && std::fabs(mount.xCm) <= 100.0f &&
               std::isfinite(mount.yCm) && std::fabs(mount.yCm) <= 100.0f &&
               std::isfinite(mount.zCm) && mount.zCm >= 0.0f &&
               mount.zCm <= 200.0f &&
               std::isfinite(mount.headingRadiansClockwise) &&
               std::fabs(mount.headingRadiansClockwise) <=
                   std::numbers::pi_v<float>;
    };
    return config.mounts.thermalRotationQuarterTurns <= 3 && mountValid(config.mounts.lidar) &&
           mountValid(config.mounts.thermal) &&
           std::isfinite(config.mounts.thermalHorizontalFovDegrees) &&
           config.mounts.thermalHorizontalFovDegrees >= 1.0f &&
           config.mounts.thermalHorizontalFovDegrees <= 180.0f &&
           std::isfinite(config.mounts.thermalVerticalFovDegrees) &&
           config.mounts.thermalVerticalFovDegrees >= 1.0f &&
           config.mounts.thermalVerticalFovDegrees <= 180.0f;
}

void drawMappingCalibration() {
    if(!ImGui::CollapsingHeader("Sensor calibration")) return;
    const bool captureLocked = sessions.isRecording() || sessions.isPaused() ||
                               sessions.isReplaying();
    ImGui::BeginDisabled(captureLocked);
    const auto beginFields = [](const char* id) {
        if(!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp |
                                   ImGuiTableFlags_NoSavedSettings)) return false;
        ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed, 128.0f);
        return true;
    };
    const auto fieldLabel = [](const char* label) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::AlignTextToFramePadding();
        ImGui::TextWrapped("%s", label);
        ImGui::TableSetColumnIndex(1);
        ImGui::SetNextItemWidth(-1.0f);
        return std::string("##") + label;
    };
    const auto numberField = [&](const char* label, float* value, const char* format) {
        const auto id = fieldLabel(label);
        return ImGui::InputFloat(id.c_str(), value, 0.0f, 0.0f, format);
    };


    ImGui::Separator();
    ImGui::TextWrapped("LIDAR MOUNT / X-Y CM AND CLOCKWISE DEGREES");
    float lidarYawDegrees = mappingCalibrationDraft.mounts.lidar.headingRadiansClockwise *
                            180.0f / std::numbers::pi_v<float>;
    if(beginFields("lidar-calibration")) {
        numberField("LiDAR X", &mappingCalibrationDraft.mounts.lidar.xCm, "%.2f");
        numberField("LiDAR Y", &mappingCalibrationDraft.mounts.lidar.yCm, "%.2f");
        numberField("LiDAR height / metadata", &mappingCalibrationDraft.mounts.lidar.zCm, "%.2f");
        if(numberField("LiDAR yaw", &lidarYawDegrees, "%.2f")) {
            mappingCalibrationDraft.mounts.lidar.headingRadiansClockwise =
                lidarYawDegrees * std::numbers::pi_v<float> / 180.0f;
        }
        ImGui::EndTable();
    }

    ImGui::Separator();
    ImGui::TextWrapped("THERMAL MOUNT / X-Y CM AND CLOCKWISE DEGREES");
    float thermalYawDegrees = mappingCalibrationDraft.mounts.thermal.headingRadiansClockwise *
                              180.0f / std::numbers::pi_v<float>;
    if(beginFields("thermal-calibration")) {
        numberField("Thermal X", &mappingCalibrationDraft.mounts.thermal.xCm, "%.2f");
        numberField("Thermal Y", &mappingCalibrationDraft.mounts.thermal.yCm, "%.2f");
        numberField("Thermal height / metadata", &mappingCalibrationDraft.mounts.thermal.zCm, "%.2f");
        if(numberField("Thermal yaw", &thermalYawDegrees, "%.2f")) {
            mappingCalibrationDraft.mounts.thermal.headingRadiansClockwise =
                thermalYawDegrees * std::numbers::pi_v<float> / 180.0f;
        }
        numberField("Horizontal FOV / degrees",
                    &mappingCalibrationDraft.mounts.thermalHorizontalFovDegrees, "%.1f");
        numberField("Vertical FOV / degrees",
                    &mappingCalibrationDraft.mounts.thermalVerticalFovDegrees, "%.1f");
        ImGui::EndTable();
    }
    ImGui::Checkbox("Mirror thermal image columns",
                    &mappingCalibrationDraft.mounts.mirrorThermalColumns);
    int imageRotation = mappingCalibrationDraft.mounts.thermalRotationQuarterTurns;
    ImGui::SetNextItemWidth(128.0f);
    if(ImGui::Combo("Thermal image rotation", &imageRotation,
                    "0 deg\0" "90 deg CW\0" "180 deg\0" "270 deg CW\0"))
        mappingCalibrationDraft.mounts.thermalRotationQuarterTurns = static_cast<std::uint8_t>(imageRotation);
    ImGui::EndDisabled();

    const bool calibrationValid = validMappingCalibration(mappingCalibrationDraft);
    if(!calibrationValid) {
        ImGui::TextColored(ui::themeColors().error,
            "Check ranges: mount X/Y +/-100 cm, height 0-200 cm, yaw +/-180 deg, FOV 1-180 deg.");
    }
    if(invalidSavedMappingCalibration) {
        ImGui::TextColored(ui::themeColors().error,
            "Saved calibration was invalid; compiled defaults are loaded.");
    }
    if(captureLocked) {
        ImGui::TextDisabled("Stop capture or close replay before changing calibration.");
    }

    ImGui::BeginDisabled(captureLocked);
    if(ImGui::Button("Restore defaults")) {
        mappingCalibrationDraft = mapping::SlamConfig{};
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(captureLocked || !calibrationValid);
    if(ImGui::Button("Apply and save")) {
        const bool changed = hasMappingCalibrationChanges(
            persistentMappingConfig, mappingCalibrationDraft);
        persistentMappingConfig = mappingCalibrationDraft;
        if(changed && slamLayer != nullptr) {
            slamLayer->setConfig(persistentMappingConfig);
        }
        invalidSavedMappingCalibration = false;
        const bool saved = savePreferences();
        if(!saved) {
            visibleMessages.emplace_back(changed
                ? "Calibration applied and map cleared, but saving preferences failed."
                : "Calibration is active, but saving preferences failed.");
        } else {
            visibleMessages.emplace_back(changed
                ? "SLAM calibration applied, saved, and map cleared."
                : "SLAM calibration saved.");
        }
        while(visibleMessages.size() > maxVisibleMessages)
            visibleMessages.pop_front();
    }
    ImGui::EndDisabled();
}

void drawMapPanel() {
    ImGui::AlignTextToFramePadding();
    ImGui::PushFont(nullptr, 22.0f);
    ImGui::TextUnformatted("Thermal map");
    ImGui::PopFont();
    if(slamLayer != nullptr) {
        ImGui::SameLine(0.0f, 16.0f);
        ImGui::TextDisabled("%s  /  %zu scans", sessions.isReplaying() ? "Replay" : "Live", slamLayer->scanCount());
        const float clearWidth = 96.0f;
        if(ImGui::GetContentRegionAvail().x > clearWidth + 16.0f) {
            ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - clearWidth);
        }
        if(ImGui::Button("Clear map", {clearWidth, 0.0f})) {
            sessions.recordEvent("map_clear", "Operator cleared the map.");
            slamLayer->clearMap();
            visibleMessages.emplace_back("Map cleared. Live capture and session storage continue.");
            while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        }
        if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
            ImGui::SetTooltip("Clear the map without stopping capture or deleting the session.");
        drawMappingCalibration();
    }
    drawMapCanvas({-1.0f, std::max(120.0f, ImGui::GetContentRegionAvail().y)});
}

void selectWorkspace(AppWorkspace workspace) {
    if(activeWorkspace == workspace) return;
    activeWorkspace = workspace;
    if(workspace != AppWorkspace::Drive && driveModeActive) {
        setDriveMode(false);
    }
}

void drawWorkspaceButton(const char* label, AppWorkspace workspace, float width) {
    const bool selected = activeWorkspace == workspace;
    ImGui::PushStyleColor(ImGuiCol_Button, {0, 0, 0, 0});
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(selected ? ImGuiCol_Text : ImGuiCol_TextDisabled));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);
    if(ImGui::Button(label, {width, 36.0f})) selectWorkspace(workspace);
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
    if(selected) {
        const auto min = ImGui::GetItemRectMin(), max = ImGui::GetItemRectMax();
        ImGui::GetWindowDrawList()->AddRectFilled({min.x + 8, max.y - 2}, {max.x - 8, max.y},
                                                  ImGui::GetColorU32(ui::themeColors().accent));
    }
}

void drawPersistentRoverSummary() {
    const bool fresh = roverPaired && statusContactFresh();
    const auto color = sessions.isReplaying() ? ui::themeColors().accent : fresh ? ui::themeColors().healthy
        : roverPaired ? ui::themeColors().warning : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
    ImGui::AlignTextToFramePadding();
    const auto position = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddCircleFilled({position.x + 4, position.y + ImGui::GetFrameHeight() * 0.5f},
                                               3.5f, ImGui::GetColorU32(color));
    ImGui::Dummy({12.0f, ImGui::GetFrameHeight()});
    ImGui::SameLine(0.0f, 4.0f);
    ImGui::TextColored(color, "%s", sessions.isReplaying() ? "Offline replay" : fresh ? "Connected" :
        discoveryPending ? "Searching" : pairingPending ? "Linking" : roverPaired ? "Status stale" : "Disconnected");
    if(fresh && !sessions.isReplaying() && ImGui::GetContentRegionAvail().x > 330.0f) {
        ImGui::SameLine(0.0f, 18.0f);
        ImGui::TextDisabled("%s:%u", roverStatus.ip, static_cast<unsigned>(roverStatus.port));
        ImGui::SameLine(0.0f, 18.0f);
        if(roverStatus.batteryP >= 0.0f) ImGui::TextDisabled("Battery %.0f%%", roverStatus.batteryP);
        else ImGui::TextDisabled("Battery unavailable");
    }
}

void drawDriveReadout() {
    const bool fresh = roverPaired && statusContactFresh();
    const ESP_STATUS* status = fresh ? &roverStatus : nullptr;
    const float width = ImGui::GetContentRegionAvail().x;
    if(ImGui::BeginTable("Drive readings", width >= 450.0f ? 4 : 2, ImGuiTableFlags_SizingStretchSame)) {
        const auto reading = [&](const char* label, const std::string& value) {
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", label);
            ImGui::PushFont(nullptr, 24.0f);
            ImGui::TextUnformatted(value.c_str());
            ImGui::PopFont();
        };
        reading("X position", status ? std::format("{:.1f} cm", status->xCentimeters) : "--");
        reading("Y position", status ? std::format("{:.1f} cm", status->yCentimeters) : "--");
        reading("Heading", status ? std::format("{:.1f} deg", status->headingDegreesClockwise) : "--");
        reading("Battery", status && status->batteryP >= 0.0f ? std::format("{:.0f}%", status->batteryP) : "--");
        ImGui::EndTable();
    }
    ImGui::Spacing();
    if(status != nullptr) drawOdometrySource(status->sensorHealthFlags, status->sensorInitErrorFlags);
    else ImGui::TextDisabled("Waiting for fresh rover telemetry.");
    ImGui::Separator();
}

void drawDashboard() {
    // Read keys before mapping and dashboard rendering so heavier frames do
    // not add latency before a changed control state reaches the UDP sender.
    if(activeWorkspace == AppWorkspace::Drive) updateDriveInput();
    else if(driveModeActive || armPending) setDriveMode(false);
    updatePackets();
    updateDriveArmLease();
    stopCaptureAfterStorageFailure();
    updateStreamMetricRates();
    updateReplay();
    expireDebugPings();
    updateRecordingKeepalive();
    requestStatusIfNeeded();
    updateMotionKeepalive();
    if(appliedTheme != selectedTheme) applyTheme(selectedTheme);
    const Vector2 renderSize = app.getRenderSize();
    ImGui::SetNextWindowPos({0.0f, 0.0f});
    ImGui::SetNextWindowSize({renderSize.x, renderSize.y});
    constexpr ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoSavedSettings;
    ImGui::Begin("Thermal Rover", nullptr, windowFlags);


    const float contentWidth = ImGui::GetContentRegionAvail().x;
    const bool wideHeader = contentWidth >= 880.0f;
    if(ImGui::BeginTable("Application header", wideHeader ? 3 : 2, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Title", wideHeader ? ImGuiTableColumnFlags_WidthFixed : ImGuiTableColumnFlags_WidthStretch, 220.0f);
        if(wideHeader) ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Appearance", ImGuiTableColumnFlags_WidthFixed, 124.0f);
        ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0);
        ImGui::AlignTextToFramePadding();
        ImGui::PushFont(nullptr, 28.0f);
        ImGui::TextUnformatted("Thermal Rover");
        ImGui::PopFont();
        if(wideHeader) { ImGui::TableNextColumn(); drawPersistentRoverSummary(); }
        ImGui::TableNextColumn();
        static constexpr const char* themeNames[]{"Polar", "Paper", "Graphite"};
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::Combo("##theme", &selectedTheme, themeNames, 3);
        ImGui::EndTable();
    }
    if(!wideHeader) drawPersistentRoverSummary();
    if(contentWidth < 370.0f) {
        static constexpr const char* workspaces[]{"Map & capture", "Drive", "Diagnostics"};
        int index = static_cast<int>(activeWorkspace);
        ImGui::SetNextItemWidth(-1.0f);
        if(ImGui::Combo("##workspace", &index, workspaces, 3)) selectWorkspace(static_cast<AppWorkspace>(index));
    } else {
        drawWorkspaceButton("Map & capture", AppWorkspace::MapCapture, 145.0f);
        ImGui::SameLine(0.0f, 4.0f);
        drawWorkspaceButton("Drive", AppWorkspace::Drive, 85.0f);
        ImGui::SameLine(0.0f, 4.0f);
        drawWorkspaceButton("Diagnostics", AppWorkspace::Diagnostics, 125.0f);
    }
    ImGui::Separator();
    ImGui::Spacing();
    const auto available = ImGui::GetContentRegionAvail();
    const float gap = 12.0f;
    if(activeWorkspace == AppWorkspace::MapCapture) {
        if(available.x >= 800.0f) {
            const float controlsWidth = 340.0f;
            ImGui::BeginChild("Map viewport", {available.x - controlsWidth - gap, available.y}, ImGuiChildFlags_Borders);
            drawMapPanel();
            ImGui::EndChild();
            ImGui::SameLine(0.0f, gap);
            ImGui::BeginChild("Capture controls", {0.0f, available.y}, ImGuiChildFlags_Borders);
            drawSessionControls();
            ImGui::EndChild();
        } else {
            ImGui::BeginChild("Map and capture workspace", {0.0f, 0.0f});
            ImGui::BeginChild("Map viewport", {0.0f, std::max(340.0f, available.y * 0.62f)}, ImGuiChildFlags_Borders);
            drawMapPanel();
            ImGui::EndChild();
            ImGui::Spacing();
            drawSessionControls();
            ImGui::EndChild();
        }
    } else if(activeWorkspace == AppWorkspace::Drive) {
        if(available.x >= 800.0f) {
            ImGui::BeginChild("Drive controls", {340.0f, available.y}, ImGuiChildFlags_Borders);
            drawDrivePanel();
            ImGui::EndChild();
            ImGui::SameLine(0.0f, gap);
            ImGui::BeginChild("Drive map", {0.0f, available.y}, ImGuiChildFlags_Borders);
            drawDriveReadout();
            ImGui::Spacing();
            drawMapCanvas({-1.0f, std::max(120.0f, ImGui::GetContentRegionAvail().y)});
            ImGui::EndChild();
        } else {
            ImGui::BeginChild("Drive controls", {0.0f, 0.0f}, ImGuiChildFlags_Borders);
            drawDriveReadout();
            drawDrivePanel();
            ImGui::EndChild();
        }
    } else if(activeWorkspace == AppWorkspace::Diagnostics) {
        if(available.x >= 800.0f) {
            const float activityHeight = std::clamp(available.y * 0.19f, 125.0f, 165.0f);
            const float detailHeight = available.y - activityHeight - gap;
            ImGui::BeginChild("Link and debug", {320.0f, detailHeight}, ImGuiChildFlags_Borders);
            drawConnectionPanel();
            ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
            drawDebugMessagePanel();
            ImGui::EndChild();
            ImGui::SameLine(0.0f, gap);
            ImGui::BeginChild("Sensor diagnostics", {0.0f, detailHeight}, ImGuiChildFlags_Borders);
            drawTelemetryPanel();
            ImGui::EndChild();
            ImGui::BeginChild("Recent activity", {0.0f, activityHeight}, ImGuiChildFlags_Borders);
            drawActivityPanel();
            ImGui::EndChild();
        } else {
            ImGui::BeginChild("Diagnostics workspace", {0.0f, 0.0f}, ImGuiChildFlags_Borders);
            drawConnectionPanel();
            ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
            drawDebugMessagePanel();
            ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
            drawTelemetryPanel();
            ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
            drawActivityPanel();
            ImGui::EndChild();
        }
    }

    ImGui::End();
}

void updatePackets() {
    std::deque<UDPPacket> packets;
    {
        std::scoped_lock lock(receivedMutex);
        packets.swap(receivedPackets);
    }

    for(const UDPPacket& packet : packets) {
        if(packet.packet.size < sizeof(PacketHeader)) continue;
        PacketHeader header{};
        std::memcpy(&header, packet.packet.buffer, sizeof(header));
        const size_t availablePayload = packet.packet.size - sizeof(PacketHeader);
        if(header.version != PROTOCOL_VERSION ||
           header.length != availablePayload) continue;

        COMMAND_ACK acknowledgement{};
        if(header.type == STATUS && header.id == ACK) {
            if(header.length != sizeof(COMMAND_ACK)) {
                ++malformedAcknowledgements;
                continue;
            }
            std::memcpy(&acknowledgement,
                        packet.packet.buffer + sizeof(header), sizeof(acknowledgement));
            if(!rover::desktop::validCommandAcknowledgement(acknowledgement)) {
                ++malformedAcknowledgements;
                continue;
            }
        }

        if(header.type == STATUS && header.id == DISCOVERY &&
           header.length == sizeof(DISCOVERY_RESPONSE) && discoveryPending) {
            DISCOVERY_RESPONSE response{};
            std::memcpy(&response,
                        packet.packet.buffer + sizeof(header),
                        sizeof(response));
            if(response.nonce == discoveryNonce && response.port != 0 &&
               response.port == packet.peer.port) {
                roverIP = packet.peer.ip;
                roverPort = response.port;
                discoveryPending = false;
                visibleMessages.push_back(std::format(
                    "Rover found at {}:{}", roverIP,
                    static_cast<unsigned>(roverPort)));
                sendPairingRequest();
            }
            continue;
        }

        if(header.type == STATUS && header.id == ACK &&
           header.length == sizeof(COMMAND_ACK) && packet.peer.ip == roverIP &&
           packet.peer.port == roverPort && pairingPending) {
            if(acknowledgement.command == PAIR &&
               (header.authToken == 0 || header.authToken == pairingToken)) {
                sessions.recordPacket(packet.packet, packet.peer, false);
                pairingPending = false;
                if(acknowledgement.accepted != 0 &&
                   header.authToken == pairingToken) {
                    roverPaired = true;
                    seedRoverSequence(header.seq);
                    lastStatusSequence = header.seq;
                    hasStatusSequence = true;
                    visibleMessages.emplace_back("Rover connected. Waiting for status.");
                    lastStatusProbeAt = {};
                    sendMoveCommand({}, true);
                } else {
                    roverPaired = false;
                    pairingToken = 0;
                    visibleMessages.emplace_back("Connection rejected. Update the rover firmware and retry.");
                }
                while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
            }
            continue;
        }

        if(!roverPaired || header.authToken != pairingToken ||
           packet.peer.ip != roverIP || packet.peer.port != roverPort) continue;

        const SequenceDisposition sequenceDisposition =
            acceptRoverSequence(header.seq);
        if(sequenceDisposition == SequenceDisposition::Duplicate) {
            ++duplicateRoverPackets;
            continue;
        }
        if(sequenceDisposition == SequenceDisposition::TooOld) {
            ++staleRoverPackets;
            continue;
        }
        if(sequenceDisposition == SequenceDisposition::Reordered) {
            ++reorderedRoverPackets;
        }

        sessions.recordPacket(packet.packet, packet.peer, false);

        if(header.type == STATUS && header.id == THERMAL_DIAGNOSTICS) {
            THERMAL_CAPTURE_DIAGNOSTICS diagnostics{};
            if(header.length != sizeof(diagnostics)) {
                ++malformedStatusPackets;
                continue;
            }
            std::memcpy(&diagnostics, packet.packet.buffer + sizeof(header), sizeof(diagnostics));
            if(!validateThermalDiagnostics(diagnostics)) {
                ++malformedStatusPackets;
                continue;
            }
            roverThermalDiagnostics = diagnostics;
            lastThermalDiagnosticsAt = std::chrono::steady_clock::now();
        } else if(header.type == MESSAGE && header.id == STR && header.length == sizeof(STR_MSG)) {
            STR_MSG payload{};
            std::memcpy(&payload, packet.packet.buffer + sizeof(header), sizeof(payload));
            const size_t textLength = std::find(payload.str, payload.str + sizeof(payload.str), '\0') - payload.str;
            const std::string message(payload.str, textLength);
            const auto pending = std::find_if(
                pendingDebugPings.begin(), pendingDebugPings.end(),
                [&](const PendingDebugPing& item) { return item.text == message; });
            if(pending != pendingDebugPings.end()) {
                const auto roundTripMilliseconds =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - pending->sentAt).count();
                visibleMessages.push_back(std::format(
                    "DEBUG PING / {} ms / {}", roundTripMilliseconds, message));
                pendingDebugPings.erase(pending);
            } else {
                visibleMessages.push_back(std::format("{}:{}  {}", packet.peer.ip,
                                                       header.seq, message));
            }
            while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
        } else if(header.type == STATUS && header.id == ESP) {
            if(header.length != sizeof(ESP_STATUS)) {
                ++malformedStatusPackets;
                if(!warnedStatusSizeMismatch) {
                    visibleMessages.push_back(std::format(
                        "Status size mismatch: received {} bytes, expected {}. Update ESP32 firmware.",
                        header.length, sizeof(ESP_STATUS)));
                    warnedStatusSizeMismatch = true;
                }
                continue;
            }
            ESP_STATUS status{};
            std::memcpy(&status,
                        packet.packet.buffer + sizeof(header), sizeof(status));
            status.ip[sizeof(status.ip) - 1] = '\0';
            if(!validateRoverStatus(status)) {
                ++malformedStatusPackets;
                continue;
            }
            if(!acceptStatusSequence(header.seq)) {
                ++staleStatusPackets;
                continue;
            }
            roverStatus = status;
            hasStatus = true;
            lastStatusAt = std::chrono::steady_clock::now();
            roverRecordingState = roverStatus.recordingState;
            if(motionTracker.observeStatus(roverStatus) ==
               rover::desktop::MotionCommandTracker::StatusResult::Finished) {
                if(roverStatus.motionResult == MOTION_RESULT_COMPLETE) {
                    visibleMessages.emplace_back("Motion completed.");
                } else if(roverStatus.motionResult == MOTION_RESULT_STALLED) {
                    visibleMessages.emplace_back("Motion stopped: wheel or target progress stalled.");
                } else if(roverStatus.motionResult == MOTION_RESULT_TIMEOUT) {
                    visibleMessages.emplace_back("Motion stopped: keepalive or move time limit expired.");
                } else if(roverStatus.motionResult == MOTION_RESULT_SENSOR_FAULT) {
                    visibleMessages.emplace_back("Motion stopped: LiDAR/IMU feedback is unavailable.");
                } else if(roverStatus.motionResult == MOTION_RESULT_MOTOR_FAULT) {
                    visibleMessages.emplace_back("Motion stopped: motor driver fault; standby asserted.");
                } else if(roverStatus.motionResult == MOTION_RESULT_CANCELLED) {
                    visibleMessages.emplace_back("Motion cancelled.");
                }
                while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
            }
        } else if(header.type == STATUS && header.id == ACK &&
                  header.length == sizeof(COMMAND_ACK)) {
            if(acknowledgement.command == RECORDING) {
                roverRecordingState = acknowledgement.state;
                visibleMessages.push_back(std::format(
                    "Recording command {} / {} / rover state {}",
                    acknowledgement.commandId,
                    acknowledgement.accepted ? "accepted" : "rejected",
                    static_cast<unsigned>(acknowledgement.state)));
                while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
            } else if(acknowledgement.command == ARM) {
                if(armPending &&
                   acknowledgement.commandId == pendingArmCommandId) {
                    if(acknowledgement.accepted != 0 &&
                       acknowledgement.state == 1 &&
                       statusContactFresh()) {
                        armPending = false;
                        driveModeActive = true;
                        activeArmCommandId = pendingArmCommandId;
                        pendingArmCommandId = 0;
                        lastArmKeepaliveAt = std::chrono::steady_clock::now();
                        lastArmAcknowledgedAt = lastArmKeepaliveAt;
                        visibleMessages.emplace_back("Rover drive armed.");
                    } else {
                        setDriveMode(false);
                        visibleMessages.emplace_back(
                            "Rover rejected drive arm or contact became stale.");
                    }
                } else if(driveModeActive &&
                          acknowledgement.commandId == activeArmCommandId &&
                          (acknowledgement.accepted == 0 ||
                           acknowledgement.state == 0)) {
                    setDriveMode(false);
                    visibleMessages.emplace_back(
                        "Rover arm lease expired; drive disarmed.");
                } else if(driveModeActive &&
                          acknowledgement.commandId == activeArmCommandId &&
                          acknowledgement.accepted != 0 &&
                          acknowledgement.state == 1) {
                    lastArmAcknowledgedAt = std::chrono::steady_clock::now();
                }
                while(visibleMessages.size() > maxVisibleMessages) {
                    visibleMessages.pop_front();
                }
            } else if(acknowledgement.command == MOTION) {
                if(motionTracker.observeAcknowledgement(acknowledgement) ==
                   rover::desktop::MotionCommandTracker::AckResult::Rejected) {
                    sendMotionStop();
                    sendMoveCommand({}, true);
                    visibleMessages.emplace_back("The rover rejected the motion command; stop sent.");
                    while(visibleMessages.size() > maxVisibleMessages) visibleMessages.pop_front();
                }
            }
        } else if(header.type == DATA && !sessions.isReplaying()) {
            if(!recordLiveSensorMetric(packet.packet, header.id,
                                       header.length)) continue;
            if(sessions.isRecording() && slamLayer != nullptr) {
                slamLayer->ingestPacket(packet.packet);
            }
        }
    }
}

void runApp() {
    if(!app.isInitialized()) return;
    slamLayer = &app.newLayer<SlamMapLayer>(persistentMappingConfig);
    auto& debugLayer = app.newLayer<DebugLayer>(app.getWindow());
    loadApplicationFont();
    debugLayer.push(drawDashboard);
#ifdef THERMAL_ROVER_WORKFLOW_CHECKS
    installWorkflowLayers(debugLayer);
#endif
    app.run();
}
} // namespace

#ifdef THERMAL_ROVER_WORKFLOW_CHECKS
#include "application_workflow_checks.inc"
#else
int main() {
    if(!server.isHealthy()) return 1;
    loadPreferences();

    std::thread receiveThread(&UDPServer::loopRecv, &server,
                              std::ref(receivedPackets), std::ref(receivedMutex));
    std::thread sendThread(&UDPServer::loopSend, &server, std::ref(toSend),
                           std::ref(sendMutex), std::ref(sendCV));

    runApp();
    savePreferences();

    emergencyStop();
    if(sessions.isRecording() || sessions.isPaused()) {
        if(roverPaired) sendRecordingControl(RECORDING_STOP);
        sessions.stopRecording();
    }
    const auto drainDeadline = std::chrono::steady_clock::now() +
                               std::chrono::milliseconds(500);
    while(std::chrono::steady_clock::now() < drainDeadline) {
        bool queueEmpty = false;
        {
            std::scoped_lock lock(sendMutex);
            queueEmpty = toSend.empty();
        }
        if(queueEmpty) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    server.kill();
    sendCV.notify_all();
    if(receiveThread.joinable()) receiveThread.join();
    if(sendThread.joinable()) sendThread.join();
    app.shutdown();
    return 0;
}
#endif
