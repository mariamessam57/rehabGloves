#include "systemstate/System_State.h"
#include "FSM/fsm_events.h"
#include <Arduino.h>

// ─────────────────────────────────────────────── SINGLETON ──────
SharedState& SharedState::get() {
    static SharedState instance;
    return instance;
}

// ─────────────────────────────────────────────── INIT ───────────
void SharedState::init() {
    _mtx_mode    = xSemaphoreCreateMutex();
    _mtx_sensors = xSemaphoreCreateMutex();
    _mtx_motors  = xSemaphoreCreateMutex();
    _events      = xEventGroupCreate();

    configASSERT(_mtx_mode);
    configASSERT(_mtx_sensors);
    configASSERT(_mtx_motors);
    configASSERT(_events);

    _initialized   = true;
    _countdown_sec = 0;
}

// ─────────────────────────────────────────────── LOCK HELPER ────
inline bool SharedState::_take(SemaphoreHandle_t m) {
    if (!m) return false;
    return xSemaphoreTake(m, pdMS_TO_TICKS(10)) == pdTRUE;
}

// ─────────────────────────────────────────────── MODE ───────────
SystemMode SharedState::getMode() {
    SystemMode m = SystemMode::SAFE_LOCK;
    if (_take(_mtx_mode)) { m = _mode; xSemaphoreGive(_mtx_mode); }
    return m;
}

void SharedState::setMode(SystemMode m) {
    if (_take(_mtx_mode)) { _mode = m; xSemaphoreGive(_mtx_mode); }
}

// ─────────────────────────────────────────────── SENSORS ────────
void SharedState::writeSensors(const SensorSnapshot& snap) {
    if (_take(_mtx_sensors)) { _sensors = snap; xSemaphoreGive(_mtx_sensors); }
}

void SharedState::readSensors(SensorSnapshot& out) {
    if (_take(_mtx_sensors)) { out = _sensors; xSemaphoreGive(_mtx_sensors); }
}

// ─────────────────────────────────────────────── MOTORS ─────────
void SharedState::writeMotors(MotorState motors[NUM_FINGERS]) {
    if (_take(_mtx_motors)) {
        for (int i = 0; i < NUM_FINGERS; i++) _motors[i] = motors[i];
        xSemaphoreGive(_mtx_motors);
    }
}

void SharedState::readMotors(MotorState motors[NUM_FINGERS]) {
    if (_take(_mtx_motors)) {
        for (int i = 0; i < NUM_FINGERS; i++) motors[i] = _motors[i];
        xSemaphoreGive(_mtx_motors);
    }
}

// ─────────────────────────────────────────────── ESTOP ──────────
void SharedState::triggerEStop(const char* reason) {
    if (_take(_mtx_mode)) {
        _estop = true;
        _mode  = SystemMode::ESTOP;
        xSemaphoreGive(_mtx_mode);
    }
    setEventBits(EVT_ESTOP);
    Serial.printf("[ESTOP] %s\n", reason ? reason : "unknown");
}

void SharedState::clearEStop() {
    if (_take(_mtx_mode)) {
        _estop                  = false;
        _calib_in_progress      = false;
        _calib_manual_mode      = false;
        _calib_phase            = CalibPhase::IDLE;
        _request_recalib        = false;
        _countdown_sec          = 0;
        _manual_calib_step      = ManualCalibStep::IDLE;
        _manual_calib_confirmed = false;
        _manual_calib_more      = false;
        _manual_countdown       = 0;
        _calib_done_ts          = 0;
        _manual_save_done       = false;
        _mode                   = SystemMode::SAFE_LOCK;
        xSemaphoreGive(_mtx_mode);
    }
    clearEventBits(EVT_ESTOP | EVT_MANUAL_SAVE_DONE);
}

bool SharedState::isEStop() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _estop; xSemaphoreGive(_mtx_mode); }
    return v;
}

// ─────────────────────────────────────────────── WARNING ────────
void SharedState::setWarning(const char* w) {
    if (_take(_mtx_mode)) {
        if (w) strncpy(_warning, w, sizeof(_warning) - 1);
        else   _warning[0] = '\0';
        _warning[sizeof(_warning) - 1] = '\0';
        xSemaphoreGive(_mtx_mode);
    }
}

void SharedState::clearWarning() {
    if (_take(_mtx_mode)) { _warning[0] = '\0'; xSemaphoreGive(_mtx_mode); }
}

const char* SharedState::getWarning() {
    const char* w = nullptr;
    if (_take(_mtx_mode)) {
        w = (_warning[0]) ? _warning : nullptr;
        xSemaphoreGive(_mtx_mode);
    }
    return w;
}

// ─────────────────────────────────────────────── AUTO CALIB ─────
bool SharedState::isCalibComplete() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _calib_complete; xSemaphoreGive(_mtx_mode); }
    return v;
}
void SharedState::setCalibComplete(bool v) {
    if (_take(_mtx_mode)) { _calib_complete = v; xSemaphoreGive(_mtx_mode); }
}

CalibPhase SharedState::getCalibPhase() {
    CalibPhase p = CalibPhase::IDLE;
    if (_take(_mtx_mode)) { p = _calib_phase; xSemaphoreGive(_mtx_mode); }
    return p;
}
void SharedState::setCalibPhase(CalibPhase p) {
    if (_take(_mtx_mode)) { _calib_phase = p; xSemaphoreGive(_mtx_mode); }
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
    if (_take(_mtx_mode)) { _request_recalib = true; xSemaphoreGive(_mtx_mode); }
}
bool SharedState::shouldRecalibrate() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _request_recalib; xSemaphoreGive(_mtx_mode); }
    return v;
}
void SharedState::clearRecalibrationRequest() {
    if (_take(_mtx_mode)) { _request_recalib = false; xSemaphoreGive(_mtx_mode); }
}

void SharedState::setCountdown(int sec) {
    if (_take(_mtx_mode)) { _countdown_sec = sec; xSemaphoreGive(_mtx_mode); }
}
int SharedState::getCountdown() {
    int v = 0;
    if (_take(_mtx_mode)) { v = _countdown_sec; xSemaphoreGive(_mtx_mode); }
    return v;
}

// ─────────────────────────────────────────────── MANUAL CALIB ───
ManualCalibStep SharedState::getManualCalibStep() {
    ManualCalibStep s = ManualCalibStep::IDLE;
    if (_take(_mtx_mode)) { s = _manual_calib_step; xSemaphoreGive(_mtx_mode); }
    return s;
}
void SharedState::setManualCalibStep(ManualCalibStep s) {
    if (_take(_mtx_mode)) { _manual_calib_step = s; xSemaphoreGive(_mtx_mode); }
}

bool SharedState::getManualCalibConfirmed() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _manual_calib_confirmed; xSemaphoreGive(_mtx_mode); }
    return v;
}
void SharedState::setManualCalibConfirmed(bool v) {
    if (_take(_mtx_mode)) { _manual_calib_confirmed = v; xSemaphoreGive(_mtx_mode); }
}

bool SharedState::getManualCalibMore() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _manual_calib_more; xSemaphoreGive(_mtx_mode); }
    return v;
}
void SharedState::setManualCalibMore(bool v) {
    if (_take(_mtx_mode)) { _manual_calib_more = v; xSemaphoreGive(_mtx_mode); }
}

void SharedState::setManualCountdown(int sec) {
    if (_take(_mtx_mode)) { _manual_countdown = sec; xSemaphoreGive(_mtx_mode); }
}
int SharedState::getManualCountdown() {
    int v = 0;
    if (_take(_mtx_mode)) { v = _manual_countdown; xSemaphoreGive(_mtx_mode); }
    return v;
}

void SharedState::setCalibDoneTs(uint32_t ts) {
    if (_take(_mtx_mode)) { _calib_done_ts = ts; xSemaphoreGive(_mtx_mode); }
}
uint32_t SharedState::getCalibDoneTs() {
    uint32_t v = 0;
    if (_take(_mtx_mode)) { v = _calib_done_ts; xSemaphoreGive(_mtx_mode); }
    return v;
}

// ─────────────────────────────────── MANUAL CALIB ADC SNAPSHOTS ─
void SharedState::setManualOpenRaw(const float raw[NUM_FINGERS]) {
    if (_take(_mtx_mode)) {
        for (int i = 0; i < NUM_FINGERS; i++) _manual_open_raw[i] = raw[i];
        xSemaphoreGive(_mtx_mode);
    }
}
void SharedState::getManualOpenRaw(float out[NUM_FINGERS]) {
    if (_take(_mtx_mode)) {
        for (int i = 0; i < NUM_FINGERS; i++) out[i] = _manual_open_raw[i];
        xSemaphoreGive(_mtx_mode);
    }
}
void SharedState::setManualCloseRaw(const float raw[NUM_FINGERS]) {
    if (_take(_mtx_mode)) {
        for (int i = 0; i < NUM_FINGERS; i++) _manual_close_raw[i] = raw[i];
        xSemaphoreGive(_mtx_mode);
    }
}
void SharedState::getManualCloseRaw(float out[NUM_FINGERS]) {
    if (_take(_mtx_mode)) {
        for (int i = 0; i < NUM_FINGERS; i++) out[i] = _manual_close_raw[i];
        xSemaphoreGive(_mtx_mode);
    }
}

// ──────────────────────────────────── MANUAL CALIB SAVE HANDSHAKE
bool SharedState::isManualSaveDone() {
    bool v = false;
    if (_take(_mtx_mode)) { v = _manual_save_done; xSemaphoreGive(_mtx_mode); }
    return v;
}
void SharedState::setManualSaveDone(bool v) {
    if (_take(_mtx_mode)) { _manual_save_done = v; xSemaphoreGive(_mtx_mode); }
}

// ─────────────────────────────────────── ATOMIC DISPLAY SNAPSHOT ─
bool SharedState::readSystemSnapshot(
    SensorSnapshot&  out,
    SystemMode&      mode,
    bool&            estop,
    const char*&     warning,
    CalibPhase&      calib_phase,
    bool&            calib_complete,
    bool&            calib_manual,
    int&             countdown_sec,
    ManualCalibStep& manual_step,
    int&             manual_countdown,
    uint32_t&        calib_done_ts)
{
    if (!_initialized) return false;

    if (!_take(_mtx_mode)) return false;
    mode             = _mode;
    estop            = _estop;
    warning          = (_warning[0]) ? _warning : nullptr;
    calib_phase      = _calib_phase;
    calib_complete   = _calib_complete;
    calib_manual     = _calib_manual_mode;
    countdown_sec    = _countdown_sec;
    manual_step      = _manual_calib_step;
    manual_countdown = _manual_countdown;
    calib_done_ts    = _calib_done_ts;
    xSemaphoreGive(_mtx_mode);

    if (!_take(_mtx_sensors)) return false;
    out = _sensors;
    xSemaphoreGive(_mtx_sensors);

    return true;
}

// ─────────────────────────────────────────────── EVENTS ─────────
EventBits_t SharedState::waitEventBits(EventBits_t bits, bool clearOnExit,
                                       bool waitForAll, TickType_t timeout) {
    return xEventGroupWaitBits(_events, bits,
        clearOnExit ? pdTRUE : pdFALSE,
        waitForAll  ? pdTRUE : pdFALSE,
        timeout);
}

void SharedState::setEventBits(EventBits_t bits)   { xEventGroupSetBits  (_events, bits); }
void SharedState::clearEventBits(EventBits_t bits) { xEventGroupClearBits(_events, bits); }

bool SharedState::isInitialized() { return _initialized; }