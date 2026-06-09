#include "systemstate/System_State.h"
#include <Arduino.h>

void SharedState::init() {
    _mtx_mode    = xSemaphoreCreateMutex();
    _mtx_sensors = xSemaphoreCreateMutex();
    _mtx_motors  = xSemaphoreCreateMutex();
    events       = xEventGroupCreate();

    configASSERT(_mtx_mode);
    configASSERT(_mtx_sensors);
    configASSERT(_mtx_motors);
    configASSERT(events);
}

// ── Mode ─────────────────────────────────────────────────────────
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
        _mode = m;
        xSemaphoreGive(_mtx_mode);
    }
}

// ── Sensor snapshot ──────────────────────────────────────────────
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

void SharedState::readSystemSnapshot(SensorSnapshot& out, SystemMode& mode,
                                     bool& estop, const char*& warning,
                                     CalibPhase& calib_phase, bool& calib_complete)
{
    if (_take(_mtx_mode)) {
        mode           = _mode;
        estop          = _estop;
        warning        = _warning;
        calib_phase    = _calib_phase;
        calib_complete = _calib_complete;

        if (_take(_mtx_sensors)) {
            out = _sensors;
            xSemaphoreGive(_mtx_sensors);
        }
        xSemaphoreGive(_mtx_mode);
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

// ── E-Stop ───────────────────────────────────────────────────────
void SharedState::triggerEStop(const char* reason) {
    if (_take(_mtx_mode)) {
        _estop = true;
        _mode  = SystemMode::ESTOP;
        xSemaphoreGive(_mtx_mode);
    }
    xEventGroupSetBits(events, EVT_ESTOP);
    Serial.printf("[SAFETY] ESTOP: %s\n", reason ? reason : "unknown");
}

void SharedState::clearEStop() {
    if (_take(_mtx_mode)) {
        _estop         = false;
        _calib_complete = false;
        _calib_phase   = CalibPhase::IDLE;
        _mode          = SystemMode::SAFE_LOCK;
        _warning       = nullptr;
        xSemaphoreGive(_mtx_mode);
    }
    xEventGroupClearBits(events, EVT_ESTOP | EVT_CALIB_DONE);
}

void SharedState::setWarning(const char* warning) {
    if (_take(_mtx_mode)) {
        _warning = warning;
        xSemaphoreGive(_mtx_mode);
    }
}

void SharedState::clearWarning() {
    if (_take(_mtx_mode)) {
        _warning = nullptr;
        xSemaphoreGive(_mtx_mode);
    }
}

const char* SharedState::getWarning() {
    const char* w = nullptr;
    if (_take(_mtx_mode)) { w = _warning; xSemaphoreGive(_mtx_mode); }
    return w;
}

bool SharedState::isEStop() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _estop; xSemaphoreGive(_mtx_mode); }
    return v;
}

// ── Calibration ──────────────────────────────────────────────────
bool SharedState::isCalibComplete() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _calib_complete; xSemaphoreGive(_mtx_mode); }
    return v;
}

void SharedState::setCalibComplete(bool v) {
    if (_take(_mtx_mode)) { _calib_complete = v; xSemaphoreGive(_mtx_mode); }
}

void SharedState::setCalibPhase(CalibPhase p) {
    if (_take(_mtx_mode)) { _calib_phase = p; xSemaphoreGive(_mtx_mode); }
}

CalibPhase SharedState::getCalibPhase() {
    CalibPhase p = CalibPhase::IDLE;
    if (_take(_mtx_mode)) { p = _calib_phase; xSemaphoreGive(_mtx_mode); }
    return p;
}

void SharedState::requestRecalibration() {
    if (_take(_mtx_mode)) {
        _request_recalib = true;
        _calib_complete = false;
        xSemaphoreGive(_mtx_mode);
    }
}

bool SharedState::shouldRecalibrate() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _request_recalib; xSemaphoreGive(_mtx_mode); }
    return v;
}

void SharedState::clearRecalibrationRequest() {
    if (_take(_mtx_mode)) {
        _request_recalib = false;
        xSemaphoreGive(_mtx_mode);
    }
}