#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H


#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "SystemTypes.h"
#include "Config.h"
#include<Arduino.h>



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
    void readSensors(SensorSnapshot& out);    void  readSystemSnapshot(SensorSnapshot& out, SystemMode& mode,
                             bool& estop, const char*& warning,
                             CalibPhase& calib_phase, bool& calib_complete);
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

    void  requestRecalibration();
    bool  shouldRecalibrate();
    void  clearRecalibrationRequest();

    // ── Event group (raw access for tasks) ───────────────────────
    EventGroupHandle_t events = nullptr;

private:
    SharedState() = default;
    SharedState(const SharedState&) = delete;

    SemaphoreHandle_t _mtx_mode    = nullptr;
    SemaphoreHandle_t _mtx_sensors = nullptr;
    SemaphoreHandle_t _mtx_motors  = nullptr;

    SystemMode    _mode           = SystemMode::SAFE_LOCK;
    SensorSnapshot _sensors       = {};
    MotorState    _motors[NUM_FINGERS] = {};
    bool          _estop          = false;
    bool          _calib_complete  = false;
    CalibPhase    _calib_phase     = CalibPhase::IDLE;
    bool          _calib_in_progress = false;
    bool          _request_recalib = false;
    const char*   _warning         = nullptr;

    inline bool _take(SemaphoreHandle_t m) {
        bool ok = xSemaphoreTake(m, pdMS_TO_TICKS(10)) == pdTRUE;
        if (!ok) {
            Serial.println("[WARN] SharedState mutex timeout");
        }
        return ok;
    }
};

#endif