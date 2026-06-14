#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// ================================================================
//  SystemMode
//  MANUAL_CALIB is new — it is the only mode where control_task
//  drives motors during calibration.
// ================================================================
enum class SystemMode : uint8_t {
    SAFE_LOCK    = 0,
    CALIBRATING,        // auto calibration (sensor_task blocks)
    MANUAL_CALIB,       // manual calibration (control_task drives motors)
    PASSIVE,
    ASSISTIVE,
    RESISTANCE,
    ESTOP
};

// ================================================================
//  CalibPhase  — used by both auto and manual flows
// ================================================================
enum class CalibPhase : uint8_t {
    IDLE = 0,
    OPEN_HAND,
    CLOSE_HAND,
    DONE,
    FAILED
};

// ================================================================
//  ManualCalibStep  — sub-state machine for manual calibration.
//  Lives inside SharedState, advanced by sensor_task and input_task.
// ================================================================
enum class ManualCalibStep : uint8_t {
    IDLE            = 0,
    WARN_OPEN,          // display warning "fully open hand, starting in 3 2 1"
    MOVING_OPEN,        // control_task drives motors toward open
    WAIT_OPEN_CONFIRM,  // motors hold, wait for 1:Yes or 2:More
    MOVING_CLOSE,       // control_task drives motors toward close
    WAIT_CLOSE_CONFIRM, // motors hold, wait for 1:Yes or 2:More
    SAVING,             // sensor_task commits values to NVS
    DONE                // 2-second display, then SAFE_LOCK
};

// ================================================================
//  MotionIntent
// ================================================================
enum class MotionIntent : uint8_t {
    NONE = 0,
    CLOSING,
    OPENING
};

// ================================================================
//  MotorDir
// ================================================================
enum class MotorDir : uint8_t {
    STOP    = 0,
    FORWARD,    // open direction
    REVERSE     // close direction
};

// ─── Per-finger flex data ────────────────────────────────────────
struct FlexData {
    float    raw        = 0.0f;
    float    filtered   = 0.0f;
    float    normalized = 0.0f;   // [0.0 – 1.0]
    float    velocity   = 0.0f;   // normalized / sec
    uint32_t last_ms    = 0;
};

// ─── Per-finger calibration ─────────────────────────────────────
struct FlexCalib {
    float min_raw = 0.0f;
    float max_raw = 4095.0f;
    bool  valid   = false;
};

// ─── IMU packet ─────────────────────────────────────────────────
struct IMUData {
    float    gyro[3]  = {};
    float    accel[3] = {};
    float    gyro_mag = 0.0f;
    uint32_t last_ms  = 0;
    bool     stuck    = false;
    bool     spike    = false;
};

// ─── FSR data ───────────────────────────────────────────────────
struct FSRData {
    float raw        = 0.0f;
    float filtered   = 0.0f;
    float normalized = 0.0f;
};

// ─── Per-finger motor state ──────────────────────────────────────
struct MotorState {
    MotorDir dir            = MotorDir::STOP;
    uint8_t  target         = 0;
    uint8_t  current        = 0;
    bool     enabled        = false;
    uint32_t stall_start_ms = 0;
};

// ─── Complete sensor snapshot ────────────────────────────────────
struct SensorSnapshot {
    FlexData   flex[3];
    FlexCalib  calib[3];
    IMUData    imu;
    FSRData    fsr;
    bool       calib_complete = false;
    CalibPhase calib_phase    = CalibPhase::IDLE;
};

#endif // SYSTEM_TYPES_H