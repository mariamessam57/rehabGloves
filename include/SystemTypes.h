#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// ================================================================
// System State Machine
// ================================================================
enum class SystemMode : uint8_t {
    SAFE_LOCK = 0,
    CALIBRATING,
    MANUAL_CALIB,
    PASSIVE,
    ASSISTIVE,
    RESISTANCE,
    ESTOP
};

// ================================================================
// Calibration Phases
// ================================================================
enum class CalibPhase : uint8_t {
    IDLE = 0,
    OPEN_HAND,
    CLOSE_HAND,
    DONE,
    FAILED
};

// ================================================================
// Manual Calibration Steps
// ================================================================
enum class ManualCalibStep : uint8_t {
    IDLE = 0,
    WARN_OPEN,
    MOVING_OPEN,
    WAIT_OPEN_CONFIRM,
    MOVING_CLOSE,
    WAIT_CLOSE_CONFIRM,
    SAVING,
    DONE
};

// ================================================================
// Motion Intent
// ================================================================
enum class MotionIntent : uint8_t {
    NONE = 0,
    OPENING,
    CLOSING
};

// ================================================================
// Motor Direction
// ================================================================
enum class MotorDir : uint8_t {
    STOP = 0,
    FORWARD,
    REVERSE
};

// ================================================================
// FLEX SENSOR DATA
// ================================================================
struct FlexData {
    float raw        = 0.0f;
    float filtered   = 0.0f;
    float normalized = 0.0f;
    float angle      = 0.0f;
    float velocity   = 0.0f;
    uint32_t last_ms = 0;
};

// ================================================================
// FLEX CALIBRATION
// ================================================================
struct FlexCalib {
    float min_raw = 0.0f;
    float max_raw = 4095.0f;
    bool  valid   = false;
};

// ================================================================
// IMU DATA (FIXED COMPATIBILITY)
// ================================================================
struct IMUData {
    float gyro[3]  = {0};
    float accel[3] = {0};

    float pitch = 0.0f;
    float roll  = 0.0f;

    float gyro_mag = 0.0f;

    uint32_t last_ms = 0;

    // NEW (clean)
    bool motion_detected = false;
    bool spike_detected  = false;

    // OLD compatibility (FIX for SafetyMonitor.cpp)
    bool stuck = false;
    bool spike = false;
};

// ================================================================
// FSR DATA
// ================================================================
struct FSRData {
    float raw        = 0.0f;
    float filtered   = 0.0f;
    float normalized = 0.0f;
};

// ================================================================
// MOTOR STATE (FIXED COMPATIBILITY)
// ================================================================
struct MotorState {
    MotorDir dir = MotorDir::STOP;

    // NEW clean naming
    uint8_t target_pwm  = 0;
    uint8_t current_pwm = 0;

    bool enabled = false;
    uint32_t last_update_ms = 0;

    // OLD compatibility (FIX MotorDriver.cpp + ControlTask)
    uint8_t target = 0;
    uint8_t current = 0;
};

// ================================================================
// SYSTEM SNAPSHOT
// ================================================================
struct SensorSnapshot {

    FlexData  flex[3];
    FlexCalib calib[3];

    IMUData imu;
    FSRData fsr;

    bool calib_complete = false;
    CalibPhase calib_phase = CalibPhase::IDLE;

    SystemMode mode = SystemMode::SAFE_LOCK;
};

#endif // SYSTEM_TYPES_H