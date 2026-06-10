#include "systemstate/System_State.h"
#include <Arduino.h>

void SharedState::init() {
    _mtx_mode    = xSemaphoreCreateMutex();
    _mtx_sensors = xSemaphoreCreateMutex();
    _mtx_motors  = xSemaphoreCreateMutex();
    _events      = xEventGroupCreate();

    configASSERT(_mtx_mode);
    configASSERT(_mtx_sensors);
    configASSERT(_mtx_motors);
    configASSERT(_events);
    _initialized = true;
}

bool SharedState::isInitialized() {
    return _initialized;
}

EventBits_t SharedState::waitEventBits(EventBits_t bits, bool clearOnExit,
                                       bool waitForAll, TickType_t timeout) {
    if (!_events) {
        Serial.println("[WARN] waitEventBits called before SharedState init");
        return 0;
    }
    return xEventGroupWaitBits(_events, bits, clearOnExit ? pdTRUE : pdFALSE,
                               waitForAll ? pdTRUE : pdFALSE, timeout);
}

void SharedState::setEventBits(EventBits_t bits) {
    if (!_events) {
        Serial.println("[WARN] setEventBits called before SharedState init");
        return;
    }
    xEventGroupSetBits(_events, bits);
}

void SharedState::clearEventBits(EventBits_t bits) {
    if (!_events) {
        Serial.println("[WARN] clearEventBits called before SharedState init");
        return;
    }
    xEventGroupClearBits(_events, bits);
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

bool SharedState::readSystemSnapshot(SensorSnapshot& out, SystemMode& mode,
                                        bool& estop, const char*& warning,
                                        CalibPhase& calib_phase, bool& calib_complete,
                                        bool& calib_manual)
{
    if (!_initialized) {
        return false;
    }

    if (!_take(_mtx_mode)) {
        return false;
    }

    mode           = _mode;
    estop          = _estop;
    warning        = (_warning[0] != '\0') ? _warning : nullptr;
    calib_phase    = _calib_phase;
    calib_complete = _calib_complete;
    calib_manual   = _calib_manual_mode;

    if (!_take(_mtx_sensors)) {
        xSemaphoreGive(_mtx_mode);
        return false;
    }

    out = _sensors;
    xSemaphoreGive(_mtx_sensors);
    xSemaphoreGive(_mtx_mode);
    return true;
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
    setEventBits(EVT_ESTOP);
    Serial.printf("[SAFETY] ESTOP: %s\n", reason ? reason : "unknown");
}

void SharedState::clearEStop() {
    if (_take(_mtx_mode)) {
        _estop             = false;
        _calib_complete    = false;
        _calib_in_progress = false;
        _calib_manual_mode = false;
        _calib_phase       = CalibPhase::IDLE;
        _mode              = SystemMode::SAFE_LOCK;
        _warning[0]        = '\0';
        xSemaphoreGive(_mtx_mode);
    }
    clearEventBits(EVT_ESTOP | EVT_CALIB_DONE);
}

void SharedState::setWarning(const char* warning) {
    if (_take(_mtx_mode)) {
        if (warning) {
            strncpy(_warning, warning, sizeof(_warning) - 1);
            _warning[sizeof(_warning) - 1] = '\0';
        } else {
            _warning[0] = '\0';
        }
        xSemaphoreGive(_mtx_mode);
    }
}

void SharedState::clearWarning() {
    if (_take(_mtx_mode)) {
        _warning[0] = '\0';
        xSemaphoreGive(_mtx_mode);
    }
}

const char* SharedState::getWarning() {
    const char* w = nullptr;
    if (_take(_mtx_mode)) {
        w = (_warning[0] != '\0') ? _warning : nullptr;
        xSemaphoreGive(_mtx_mode);
    }
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

bool SharedState::isCalibInProgress() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _calib_in_progress; xSemaphoreGive(_mtx_mode); }
    return v;
}

void SharedState::setCalibInProgress(bool v) {
    if (_take(_mtx_mode)) { _calib_in_progress = v; xSemaphoreGive(_mtx_mode); }
}

bool SharedState::isCalibManualMode() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _calib_manual_mode; xSemaphoreGive(_mtx_mode); }
    return v;
}

void SharedState::setCalibManualMode(bool v) {
    if (_take(_mtx_mode)) { _calib_manual_mode = v; xSemaphoreGive(_mtx_mode); }
}

void SharedState::requestRecalibration() {
    if (_take(_mtx_mode)) {
        _request_recalib     = true;
        _calib_complete      = false;
        _calib_in_progress   = false;
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