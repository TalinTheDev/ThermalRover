#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <cassert>
#include <type_traits>

#define PRINT_PACKET_BUF(p) for(int i = 0; i < (p).size; i++){printf("%x ", (p).buffer[i]);} printf("\n");

constexpr unsigned int BUFFER_LEN = 512;
constexpr unsigned short IP_LEN = 16;
constexpr std::size_t LD19_PACKET_BYTES = 47;
constexpr std::size_t THERMAL_CHUNK_PIXELS = 244;
constexpr std::uint8_t PROTOCOL_VERSION = 3;
constexpr std::uint8_t PREVIOUS_REPLAY_PROTOCOL_VERSION = 2;
constexpr std::uint8_t LEGACY_REPLAY_PROTOCOL_VERSION = 1;
inline std::atomic<std::uint32_t> packetsSent{1};

enum PacketType : uint8_t {
    MESSAGE = 0x00,
    COMMAND = 0x01,
    STATUS = 0x02,
    DATA = 0x03,
};

enum PacketID : uint8_t {
    NONE = 0x00,
    STR = 0x01,
    LED = 0x02,
    MOVE = 0x03,
    ESP = 0x04,
    LIDAR_PACKET = 0x05,
    THERMAL_CHUNK = 0x06,
    WHEEL_ODOMETRY = 0x07,
    MOTION = 0x08,
    MOTION_STOP = 0x09,
    MOTION_KEEPALIVE = 0x0A,
    RECORDING = 0x0B,
    ACK = 0x0C,
    IMU_SAMPLE = 0x0D,
    PAIR = 0x0E,
    DISCOVERY = 0x0F,
    ARM = 0x10,
    FUSED_ODOMETRY = 0x11,
    THERMAL_DIAGNOSTICS = 0x12
};

enum MotionKind : uint8_t {
    DISTANCE_CM = 0x01,
    ANGLE_DEGREES = 0x02,
};

enum MotionResult : uint8_t {
    MOTION_RESULT_IDLE = 0x00,
    MOTION_RESULT_RUNNING = 0x01,
    MOTION_RESULT_COMPLETE = 0x02,
    MOTION_RESULT_STALLED = 0x03,
    MOTION_RESULT_TIMEOUT = 0x04,
    MOTION_RESULT_CANCELLED = 0x05,
    MOTION_RESULT_SENSOR_FAULT = 0x06,
    MOTION_RESULT_MOTOR_FAULT = 0x07,
};

enum RecordingAction : uint8_t {
    RECORDING_START = 0x01,
    RECORDING_PAUSE = 0x02,
    RECORDING_RESUME = 0x03,
    RECORDING_STOP = 0x04,
    RECORDING_KEEPALIVE = 0x05,
};

enum ArmAction : uint8_t {
    ARM_REQUEST = 0x01,
    ARM_DISARM = 0x02,
    ARM_KEEPALIVE = 0x03,
};

enum SensorHealthFlag : uint16_t {
    SENSOR_ENCODERS_READY = 1U << 0,
    SENSOR_BATTERY_READY = 1U << 1,
    SENSOR_MPU6050_STREAMING = 1U << 2,
    SENSOR_MPU6050_CALIBRATED = 1U << 3,
    SENSOR_MLX90640_DETECTED = 1U << 4,
    SENSOR_LD19_STREAMING = 1U << 5,
    SENSOR_MLX90640_STREAMING = 1U << 6,
    SENSOR_MOTORS_READY = 1U << 7,
    SENSOR_LIDAR_IMU_ODOMETRY_READY = 1U << 8,
};

enum SensorInitErrorFlag : uint16_t {
    SENSOR_ERROR_I2C_BUS = 1U << 0,
    SENSOR_ERROR_BATTERY_MISSING = 1U << 1,
    SENSOR_ERROR_MPU6050_MISSING = 1U << 2,
    SENSOR_ERROR_MPU6050_CONFIG = 1U << 3,
    SENSOR_ERROR_MLX90640_MISSING = 1U << 4,
    SENSOR_ERROR_LD19_UART = 1U << 5,
    SENSOR_ERROR_MLX90640_CONFIG = 1U << 6,
    SENSOR_ERROR_ENCODERS_INIT = 1U << 7,
    SENSOR_ERROR_ENCODERS_RUNTIME = 1U << 8,
    SENSOR_ERROR_MOTOR_INIT = 1U << 9,
    SENSOR_ERROR_MOTOR_RUNTIME = 1U << 10,
    SENSOR_ERROR_MPU6050_TASK = 1U << 11,
    SENSOR_ERROR_MLX90640_TASK = 1U << 12,
    SENSOR_ERROR_LD19_TASK = 1U << 13,
    SENSOR_ERROR_NETWORK_TASK = 1U << 14,
    SENSOR_ERROR_MOTOR_TASK = 1U << 15,
};

#pragma pack(push, 1)
struct LED_CMD {
    uint8_t led;
    uint8_t on;
};

// COMMAND/MOVE: signed normalized forward and turn axes (-1, 0, 1), plus
// an 8-bit motor PWM speed (0-255). Differential mixing is performed by the
// ESP32 so the desktop can send the same intent for straight, pivot, and
// combined forward/turn motion.
struct MOVE_CMD {
    int8_t throttle;
    int8_t steering;
    uint8_t speed;
};

// One-shot LiDAR/IMU-based move. Distance is signed centimeters; angle is
// signed clockwise degrees. PID gains and maximum PWM are sent with each move.
struct PID_MOTION_CMD {
    uint16_t commandId;
    MotionKind kind;
    float target;
    float kp;
    float ki;
    float kd;
    uint8_t maxSpeed;
};

struct MOTION_KEEPALIVE_CMD {
    uint16_t commandId;
};

struct RECORDING_CMD {
    uint16_t commandId;
    RecordingAction action;
};

// The rover requires a renewed arm lease before accepting any nonzero MOVE or
// PID motion. DISARM and emergency stop remain valid while the lease is absent.
struct ARM_CMD {
    uint16_t commandId;
    ArmAction action;
};

struct PAIR_REQUEST {
    // Automatic connection handshake: a fresh, nonzero token chosen by the
    // laptop, echoed in the acknowledgement header. No operator code is used.
    std::uint32_t sessionToken;
};

struct DISCOVERY_REQUEST {
    std::uint32_t nonce;
};

struct DISCOVERY_RESPONSE {
    std::uint32_t nonce;
    std::uint16_t port;
    char ip[IP_LEN];
};

struct COMMAND_ACK {
    uint16_t commandId;
    PacketID command;
    uint8_t accepted;
    uint8_t state;
};

struct STR_MSG {
    char str[128];
};

struct ESP_STATUS {
    float batteryP = -1.0f;
    float batteryV = -1.0f;
    // Rover pose in the project-wide frame: +X right, +Y forward, and
    // Clockwise heading in degrees [0, 360) (0 forward, 90 right); distances in cm.
    float xCentimeters = 0.0f;
    float yCentimeters = 0.0f;
    float headingDegreesClockwise = 0.0f;
    uint16_t motionCommandId = 0;
    uint8_t motionActive = 0;
    MotionResult motionResult = MOTION_RESULT_IDLE;
    unsigned short port = 0;
    char ip[16] = {};
    uint16_t sensorHealthFlags = 0;
    uint8_t recordingState = 0; // 0 stopped, 1 streaming, 2 paused.
    uint32_t streamDroppedPackets = 0;
    uint16_t sensorInitErrorFlags = 0;
    uint32_t uptimeMilliseconds = 0;
    float imuGyroBiasDegreesPerSecond = 0.0f;
    float imuYawDirection = 0.0f;
};
static_assert(sizeof(ESP_STATUS) == 63,
              "Desktop and ESP32 status payload layouts must match");

// DATA/LIDAR_PACKET: one raw LD19 0x54/0x2C serial packet with a shared ESP32
// millisecond timestamp for synchronizing it with encoder and thermal data.
struct LIDAR_PACKET_DATA {
    uint32_t timestampMilliseconds;
    uint8_t packet[LD19_PACKET_BYTES];
};

// DATA/WHEEL_ODOMETRY: absolute raw signed PCNT totals at the capture timestamp.
// Apply configured left/right forward signs in the consumer before odometry.
struct WHEEL_ODOMETRY_DATA {
    uint32_t timestampMilliseconds;
    int64_t leftEncoderTicks;
    int64_t rightEncoderTicks;
};

// DATA/IMU_SAMPLE: timestamp and the bias-corrected Z gyro rate/yaw used by
// fused local odometry. Yaw remains relative; MPU-6050 has no compass.
struct IMU_SAMPLE_DATA {
    uint32_t timestampMilliseconds;
    float gyroZDegreesPerSecondClockwise;
    float yawRadiansClockwise;
};

// Scan-time rover pose from LiDAR/IMU fusion. Quality 1 is tracking; other
// values describe initialization or degraded tracking and cannot drive distance PID.
struct FUSED_ODOMETRY_DATA {
    std::uint32_t timestampMilliseconds;
    float xCentimeters, yCentimeters, headingRadiansClockwise;
    float residualCentimeters;
    std::uint16_t matchedPoints;
    std::uint32_t resetCount;
    std::uint8_t quality;
};

// DATA/THERMAL_CHUNK: up to 244 signed temperatures in hundredths of a Celsius
// degree, row-major in the combined 32 by 24 MLX90640 frame.
struct THERMAL_FRAME_CHUNK {
    uint32_t timestampMilliseconds;
    uint16_t frameId;
    uint16_t pixelOffset;
    uint16_t sampleCount;
    int16_t temperaturesCentiC[THERMAL_CHUNK_PIXELS];
};

enum class ThermalCaptureStage : std::uint8_t {
    Idle, WaitingForData, Reading, Converting, WaitingForPartner,
    Streaming, ReadError, InvalidTemperature
};

// STATUS/THERMAL_DIAGNOSTICS: additive protocol-3 telemetry, also sent when
// capture is stalled. Counters describe acquisition, not UDP delivery.
struct THERMAL_CAPTURE_DIAGNOSTICS {
    std::uint32_t timestampMilliseconds = 0;
    std::uint32_t lastSubpageMilliseconds = 0;
    std::uint32_t completedFrames = 0;
    std::uint32_t subpage0Count = 0;
    std::uint32_t subpage1Count = 0;
    std::uint32_t readErrors = 0;
    std::uint32_t invalidFrames = 0;
    std::int16_t lastResult = 0;
    std::uint16_t statusRegister = 0;
    std::uint16_t invalidPixel = 0xffff;
    ThermalCaptureStage stage = ThermalCaptureStage::Idle;
    std::uint8_t lastSubpage = 0xff;
};

struct PacketHeader {
    uint16_t length;
    PacketType type;
    PacketID id;
    std::uint32_t seq;
    // Connection token established automatically by the PAIR handshake.
    std::uint32_t authToken = 0;
    std::uint8_t version = PROTOCOL_VERSION;
};
#pragma pack(pop)  

static_assert(sizeof(PacketHeader) == 13);
static_assert(sizeof(MOVE_CMD) == 3);
static_assert(sizeof(PID_MOTION_CMD) == 20);
static_assert(sizeof(MOTION_KEEPALIVE_CMD) == 2);
static_assert(sizeof(RECORDING_CMD) == 3);
static_assert(sizeof(ARM_CMD) == 3);
static_assert(sizeof(PAIR_REQUEST) == 4);
static_assert(sizeof(DISCOVERY_REQUEST) == 4);
static_assert(sizeof(DISCOVERY_RESPONSE) == 22);
static_assert(sizeof(COMMAND_ACK) == 5);
static_assert(sizeof(FUSED_ODOMETRY_DATA) == 27);
static_assert(sizeof(THERMAL_CAPTURE_DIAGNOSTICS) == 36);
static_assert(sizeof(THERMAL_CAPTURE_DIAGNOSTICS) + sizeof(PacketHeader) <= BUFFER_LEN);
static_assert(sizeof(FUSED_ODOMETRY_DATA) + sizeof(PacketHeader) <= BUFFER_LEN);
static_assert(sizeof(THERMAL_FRAME_CHUNK) + sizeof(PacketHeader) <= BUFFER_LEN,
              "A thermal chunk must fit one UDP protocol packet");
static_assert(sizeof(LIDAR_PACKET_DATA) + sizeof(PacketHeader) <= BUFFER_LEN);
static_assert(sizeof(WHEEL_ODOMETRY_DATA) + sizeof(PacketHeader) <= BUFFER_LEN);

class Packet {
public:
    template<typename T>
    Packet(PacketType type, PacketID id, const T& payload,
           std::uint32_t authToken = 0) {
        static_assert(std::is_trivially_copyable_v<T>,
                  "Packet payload must be trivially copyable");
        uint16_t payloadSize = sizeof(T);
        assert(sizeof(PacketHeader) + payloadSize <= BUFFER_LEN && "Packet too large");

        PacketHeader header{
        payloadSize,
        type,
        id,
        packetsSent.fetch_add(1, std::memory_order_relaxed),
        authToken,
        PROTOCOL_VERSION
        };
        memcpy(buffer, &header, sizeof(header));
        memcpy(buffer + sizeof(header), &payload, payloadSize);
        size = sizeof(header) + payloadSize;
    }

    Packet(uint8_t* buffer_, uint16_t size) : size(size) {
        memcpy(buffer, buffer_, size);
    }

    Packet() {}

    ~Packet() {}

    uint8_t buffer[BUFFER_LEN]{};
    uint16_t size = 0;
};

struct PeerAddress {
    char ip[16] = {};
    unsigned short port = 0;
};

struct UDPPacket {
    Packet packet;
    PeerAddress peer;
};
