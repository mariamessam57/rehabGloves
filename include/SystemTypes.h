#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>
#include <stdbool.h>


enum class SystemMode : uint8_t {
    SAFE_LOCK   = 0,
    CALIBRATING,
    PASSIVE,
    ASSISTIVE,
    RESISTANCE,
    ESTOP
};

enum class CalibPhase : uint8_t {
    IDLE = 0,
    OPEN_HAND,
    CLOSE_HAND,
    DONE,
    FAILED
};

enum class MotionIntent : uint8_t {
    NONE = 0,
    CLOSING,
    OPENING
};

enum class MotorDir : uint8_t {
    STOP = 0,
    FORWARD,    // open
    REVERSE     // close
};

// ─── Per-finger flex data ────────────────────────────────────────
struct FlexData {
    float raw         = 0.0f;
    float filtered    = 0.0f;   // EMA output
    float normalized  = 0.0f;   // [0.0 – 1.0]
    float velocity    = 0.0f;   // normalized / sec
    uint32_t last_ms  = 0;
};

// ─── Per-finger calibration ─────────────────────────────────────
struct FlexCalib {
    float min_raw = 0.0f;
    float max_raw = 4095.0f;
    bool  valid   = false;
};

// ─── IMU packet ─────────────────────────────────────────────────
struct IMUData {
    float gyro[3]     = {};     // deg/s  Kalman-filtered
    float accel[3]    = {};     // g
    float gyro_mag    = 0.0f;
    uint32_t last_ms  = 0;
    bool stuck        = false;
    bool spike        = false;
};

// ─── FSR data ───────────────────────────────────────────────────
struct FSRData {
    float raw        = 0.0f;
    float filtered   = 0.0f;
    float normalized = 0.0f;    // [0.0 – 1.0]
};

// ─── Per-finger motor state ──────────────────────────────────────
struct MotorState {
    MotorDir dir       = MotorDir::STOP;
    uint8_t  target    = 0;     // desired duty
    uint8_t  current   = 0;     // ramped duty
    bool     enabled   = false;
    uint32_t stall_start_ms = 0;
};

// ─── Complete snapshot (read by multiple tasks) ──────────────────
struct SensorSnapshot {
    FlexData  flex[3];
    FlexCalib calib[3];
    IMUData   imu;
    FSRData   fsr;
    bool      calib_complete = false;
    CalibPhase calib_phase   = CalibPhase::IDLE;
};
#endif