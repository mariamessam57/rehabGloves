#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "SystemTypes.h"
#include "Config.h"
#include <Arduino.h>

#define EVT_ESTOP       (1 << 0)
#define EVT_CALIB_DONE  (1 << 1)

class SharedState {
public:
    static SharedState& get() {
        static SharedState instance;
        return instance;
    }

    // ── init (call once from setup()) ────────────────────────────
    void init();

    // ── Mode ─────────────────────────────────────────────────────
    SystemMode getMode();
    void       setMode(SystemMode m);

    // ── Sensor snapshot (written by SensorTask) ──────────────────
    void writeSensors(const SensorSnapshot& snap);
    void readSensors(SensorSnapshot& out);
    
    // تم تعديل الدالة لإضافة متغير countdown_sec لتمرير قيمة التايمر للشاشة بالتزامن
    bool readSystemSnapshot(SensorSnapshot& out, SystemMode& mode,
                            bool& estop, const char*& warning,
                            CalibPhase& calib_phase, bool& calib_complete,
                            bool& calib_manual, int& countdown_sec);

    // ── Motor commands (written by ControlTask) ───────────────────
    void  writeMotors(const MotorState motors[NUM_FINGERS]);
    void  readMotors(MotorState out[NUM_FINGERS]);

    // ── Emergency stop ───────────────────────────────────────────
    void  triggerEStop(const char* reason);
    void  clearEStop();
    bool  isEStop();

    // ── Safety warnings ──────────────────────────────────────────
    void  setWarning(const char* warning);
    void  clearWarning();
    const char* getWarning();

    // ── Calibration ──────────────────────────────────────────────
    bool  isCalibComplete();
    void  setCalibComplete(bool v);
    void  setCalibPhase(CalibPhase p);
    CalibPhase getCalibPhase();

    bool  isCalibInProgress();
    void  setCalibInProgress(bool v);
    bool  isCalibManualMode();
    void  setCalibManualMode(bool v);

    void  requestRecalibration();
    bool  shouldRecalibrate();
    void  clearRecalibrationRequest();

    // ── دوال التايمر الجديدة للعد التنازلي (Thread-Safe) ──────────────────
    void setCountdown(int sec);
    int  getCountdown();

    // ── Event group helpers (safe access) ─────────────────────────────
    EventBits_t waitEventBits(EventBits_t bits, bool clearOnExit,
                              bool waitForAll, TickType_t timeout);
    void setEventBits(EventBits_t bits);
    void clearEventBits(EventBits_t bits);

    // ── Initialization sentinel ────────────────────────────────────────
    bool isInitialized();

private:
    SharedState() = default;
    SharedState(const SharedState&) = delete;

    SemaphoreHandle_t _mtx_mode    = nullptr;
    SemaphoreHandle_t _mtx_sensors = nullptr;
    SemaphoreHandle_t _mtx_motors  = nullptr;
    EventGroupHandle_t _events     = nullptr;
    bool               _initialized = false;

    SystemMode     _mode           = SystemMode::SAFE_LOCK;
    SensorSnapshot _sensors       = {};
    MotorState     _motors[NUM_FINGERS] = {};
    bool           _estop          = false;
    bool           _calib_complete  = false;
    CalibPhase     _calib_phase     = CalibPhase::IDLE;
    bool           _calib_in_progress = false;
    bool           _calib_manual_mode = false;
    bool           _request_recalib = false;
    char           _warning[64]    = {};
    
    // المتغير الخاص بالتايمر
    int            _countdown_sec   = 0; 

    inline bool _take(SemaphoreHandle_t m) {
        if (!m) {
            Serial.println("[WARN] SharedState mutex not initialized");
            return false;
        }
        bool ok = xSemaphoreTake(m, pdMS_TO_TICKS(10)) == pdTRUE;
        if (!ok) {
            Serial.println("[WARN] SharedState mutex timeout");
        }
        return ok;
    }
};

#endif