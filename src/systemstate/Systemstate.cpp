// ================================================================
//  System_State.cpp
//
//  Implements SharedState.  All fixes annotated inline.
// ================================================================

#include "systemstate/System_State.h"
#include <string.h>
#include <Arduino.h>

// ── FIX C5: Global I2C mutex definition ──────────────────────────
SemaphoreHandle_t g_i2c_mutex = nullptr;

// ================================================================
void SharedState::init() {
    _mtx_mode    = xSemaphoreCreateMutex();
    _mtx_sensors = xSemaphoreCreateMutex();
    _mtx_motors  = xSemaphoreCreateMutex();
    _mtx_flags   = xSemaphoreCreateMutex();   // FIX H3
    _mtx_display = xSemaphoreCreateMutex();   // FIX C4
    events       = xEventGroupCreate();

    // FIX C5: create global I2C mutex here so it's ready before
    // any task starts and any Wire call happens.
    i2c_mutex_init();

    configASSERT(_mtx_mode);
    configASSERT(_mtx_sensors);
    configASSERT(_mtx_motors);
    configASSERT(_mtx_flags);
    configASSERT(_mtx_display);
    configASSERT(events);
    configASSERT(g_i2c_mutex);
}

// ── Mode ─────────────────────────────────────────────────────────
static const char* modeToString(SystemMode m) {
    switch (m) {
        case SystemMode::SAFE_LOCK:    return "SAFE_LOCK";
        case SystemMode::CALIBRATING:  return "CALIBRATING";
        case SystemMode::MANUAL_CALIB:  return "MANUAL_CALIB";
        case SystemMode::PASSIVE:      return "PASSIVE";
        case SystemMode::ASSISTIVE:    return "ASSISTIVE";
        case SystemMode::RESISTANCE:   return "RESISTANCE";
        case SystemMode::ESTOP:        return "ESTOP";
        default:                       return "UNKNOWN";
    }
}

SystemMode SharedState::getMode() {
    SystemMode m = SystemMode::SAFE_LOCK;
    if (_take(_mtx_mode)) {
        m = _mode;
        xSemaphoreGive(_mtx_mode);
    }
    return m;
}

void SharedState::setMode(SystemMode m) {
    if (_take(_mtx_mode)) {
        if (_mode != m) {
            Serial.printf("[MODE] %s -> %s\n", modeToString(_mode), modeToString(m));
        }
        _mode = m;
        xSemaphoreGive(_mtx_mode);
    }
}
bool SharedState::_take(SemaphoreHandle_t m, TickType_t ticks)
{
    return m && xSemaphoreTake(m, ticks) == pdTRUE;
}

// ── Sensors ──────────────────────────────────────────────────────
void SharedState::writeSensors(const SensorSnapshot& snap) {
    if (_take(_mtx_sensors)) {
        _sensors = snap;
        xSemaphoreGive(_mtx_sensors);
    }
}

void SharedState::readSensors(SensorSnapshot& out) {
    if (_take(_mtx_sensors)) {
        out = _sensors;
        xSemaphoreGive(_mtx_sensors);
    }
}

// ── Motors ───────────────────────────────────────────────────────
void SharedState::writeMotors(const MotorState motors[NUM_FINGERS]) {
    if (_take(_mtx_motors)) {
        for (int i = 0; i < NUM_FINGERS; i++) _motors[i] = motors[i];
        xSemaphoreGive(_mtx_motors);
    }
}

void SharedState::readMotors(MotorState out[NUM_FINGERS]) {
    if (_take(_mtx_motors)) {
        for (int i = 0; i < NUM_FINGERS; i++) out[i] = _motors[i];
        xSemaphoreGive(_mtx_motors);
    }
}

// ── ESTOP ────────────────────────────────────────────────────────
// FIX H3: _estop now protected by _mtx_flags.
void SharedState::triggerEStop(const char* reason) {
    if (_take(_mtx_flags)) {
        _estop = true;
        xSemaphoreGive(_mtx_flags);
    }
    xEventGroupSetBits(events, EVT_ESTOP);
    Serial.printf("[ESTOP] %s\n", reason ? reason : "unknown");
}

void SharedState::clearEStop() {
    if (_take(_mtx_flags)) {
        _estop = false;
        xSemaphoreGive(_mtx_flags);
    }
    xEventGroupClearBits(events, EVT_ESTOP);
}

bool SharedState::isEStop() {
    bool v = false;
    if (_take(_mtx_flags)) {
        v = _estop;
        xSemaphoreGive(_mtx_flags);
    }
    return v;
}

// ── Calibration flags ────────────────────────────────────────────
bool SharedState::isCalibComplete() {
    bool v = false;
    if (_take(_mtx_flags)) {
        v = _calib_complete;
        xSemaphoreGive(_mtx_flags);
    }
    return v;
}

void SharedState::setCalibComplete(bool v) {
    if (_take(_mtx_flags)) {
        _calib_complete = v;
        xSemaphoreGive(_mtx_flags);
    }
}

void SharedState::setCalibPhase(CalibPhase p) {
    if (_take(_mtx_flags)) {
        _calib_phase = p;
        xSemaphoreGive(_mtx_flags);
    }
}

CalibPhase SharedState::getCalibPhase() {
    CalibPhase p = CalibPhase::IDLE;
    if (_take(_mtx_flags)) {
        p = _calib_phase;
        xSemaphoreGive(_mtx_flags);
    }
    return p;
}

void SharedState::requestRecalibration() {
    if (_take(_mtx_flags)) {
        _request_recalib = true;
        xSemaphoreGive(_mtx_flags);
    }
}

bool SharedState::shouldRecalibrate() {
    bool v = false;
    if (_take(_mtx_flags)) {
        v = _request_recalib;
        xSemaphoreGive(_mtx_flags);
    }
    return v;
}

void SharedState::clearRecalibrationRequest() {
    if (_take(_mtx_flags)) {
        _request_recalib = false;
        xSemaphoreGive(_mtx_flags);
    }
}

// ── FIX C4: Warning ──────────────────────────────────────────────
void SharedState::setWarning(const char* msg) {
    if (_take(_mtx_display)) {
        if (msg) strncpy(_warning, msg, sizeof(_warning) - 1);
        _warning[sizeof(_warning) - 1] = '\0';
        xSemaphoreGive(_mtx_display);
    }
}

void SharedState::clearWarning() {
    if (_take(_mtx_display)) {
        _warning[0] = '\0';
        xSemaphoreGive(_mtx_display);
    }
}

// ── FIX C6: Manual calib accessors ───────────────────────────────
void SharedState::setManualCalibStep(ManualCalibStep s) {
    if (_take(_mtx_flags)) {
        _manual_step = s;
        xSemaphoreGive(_mtx_flags);
    }
}

ManualCalibStep SharedState::getManualCalibStep() {
    ManualCalibStep s = ManualCalibStep::IDLE;
    if (_take(_mtx_flags)) {
        s = _manual_step;
        xSemaphoreGive(_mtx_flags);
    }
    return s;
}

void SharedState::setManualCalibCountdown(int cd) {
    if (_take(_mtx_flags)) {
        _manual_cd = cd;
        xSemaphoreGive(_mtx_flags);
    }
}

void SharedState::setCalibDoneTimestamp(TickType_t ts) {
    if (_take(_mtx_flags)) {
        _calib_done_ts = ts;
        xSemaphoreGive(_mtx_flags);
    }
}

void SharedState::clearCalibDoneTimestamp() {
    if (_take(_mtx_flags)) {
        _calib_done_ts = 0;
        xSemaphoreGive(_mtx_flags);
    }
}

// ── FIX C3: readSystemSnapshot ────────────────────────────────────
// Collects every piece of display-relevant state in the minimum
// number of mutex sections.  The display_task calls this once per
// frame and then renders from the returned copies — no further
// SharedState access needed during rendering, eliminating all
// data races between the render loop and writer tasks.
//
// FIX H4: calib_done_ts auto-clears after CALIB_DONE_SHOW_MS so
// the CalibDone overlay doesn't stay on screen forever.
// CALIB_DONE_SHOW_MS should be defined in Config.h (e.g. 2000).
// If it is not defined, we default to 2000 ms here.
#ifndef CALIB_DONE_SHOW_MS
#  define CALIB_DONE_SHOW_MS  2000U
#endif

bool SharedState::readSystemSnapshot(
    SensorSnapshot&  snap_out,
    SystemMode&      mode_out,
    bool&            estop_out,
    char             warning_out[32],
    CalibPhase&      cp_out,
    bool&            calib_complete_out,
    bool&            calib_manual_out,
    int&             countdown_out,
    ManualCalibStep& mstep_out,
    int&             manual_cd_out,
    TickType_t&      calib_done_ts_out)
{
    // --- sensors ---
    bool ok = _take(_mtx_sensors);
    if (!ok) return false;
    snap_out = _sensors;
    xSemaphoreGive(_mtx_sensors);

    // --- mode ---
    ok = _take(_mtx_mode);
    if (!ok) return false;
    mode_out = _mode;
    xSemaphoreGive(_mtx_mode);

    // --- flags (estop, calib, manual step, done timestamp) ---
    ok = _take(_mtx_flags);
    if (!ok) return false;
    estop_out          = _estop;
    cp_out             = _calib_phase;
    calib_complete_out = _calib_complete;
    calib_manual_out   = (_mode == SystemMode::MANUAL_CALIB);
    countdown_out      = 0;                // auto-calib countdown not implemented yet
    mstep_out          = _manual_step;
    manual_cd_out      = _manual_cd;
    calib_done_ts_out  = _calib_done_ts;

    // FIX H4: auto-clear calib_done_ts after the display window expires
    if (_calib_done_ts != 0) {
        TickType_t elapsed_ms = (xTaskGetTickCount() - _calib_done_ts)
                                * portTICK_PERIOD_MS;
        if (elapsed_ms >= CALIB_DONE_SHOW_MS) {
            _calib_done_ts    = 0;
            calib_done_ts_out = 0;
        }
    }
    xSemaphoreGive(_mtx_flags);

    // --- warning (separate mutex, copied into caller buffer) ---
    ok = _take(_mtx_display);
    if (!ok) {
        warning_out[0] = '\0';
    } else {
        strncpy(warning_out, _warning, 32);
        warning_out[31] = '\0';
        xSemaphoreGive(_mtx_display);
    }

    return true;
}