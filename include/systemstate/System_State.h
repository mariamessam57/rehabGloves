#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "SystemTypes.h"
#include "config.h"
#include <string.h>

// ── Event-group bit definitions ──────────────────────────────────
#define EVT_CALIB_DONE   (1u << 0)
#define EVT_ESTOP        (1u << 1)
#define EVT_BTN1         (1u << 2)
#define EVT_BTN2         (1u << 3)
#define EVT_BTN4         (1u << 4)

// ================================================================
//  SharedState  — singleton, all inter-task communication
// ================================================================
class SharedState {
public:
    static SharedState& get();

    void init();
    bool isInitialized();

    // ── Mode ──────────────────────────────────────────────────────
    SystemMode  getMode();
    void        setMode(SystemMode m);

    // ── Sensors ───────────────────────────────────────────────────
    void writeSensors(const SensorSnapshot& snap);
    void readSensors(SensorSnapshot& out);

    // ── Motors ────────────────────────────────────────────────────
    void writeMotors(MotorState motors[NUM_FINGERS]);
    void readMotors(MotorState motors[NUM_FINGERS]);

    // ── ESTOP ─────────────────────────────────────────────────────
    void triggerEStop(const char* reason);
    void clearEStop();
    bool isEStop();

    // ── Warning ───────────────────────────────────────────────────
    void        setWarning(const char* w);
    void        clearWarning();
    const char* getWarning();

    // ── Auto calibration ──────────────────────────────────────────
    bool        isCalibComplete();
    void        setCalibComplete(bool v);
    CalibPhase  getCalibPhase();
    void        setCalibPhase(CalibPhase p);
    bool        isCalibInProgress();
    void        setCalibInProgress(bool v);
    bool        isCalibManualMode();
    void        setCalibManualMode(bool v);
    void        requestRecalibration();
    bool        shouldRecalibrate();
    void        clearRecalibrationRequest();

    // ── Countdown timer (auto calib display) ─────────────────────
    void setCountdown(int sec);
    int  getCountdown();

    // ── Manual calibration step machine ──────────────────────────
    ManualCalibStep getManualCalibStep();
    void            setManualCalibStep(ManualCalibStep s);

    bool getManualCalibConfirmed();
    void setManualCalibConfirmed(bool v);
    bool getManualCalibMore();
    void setManualCalibMore(bool v);

    void setManualCountdown(int sec);
    int  getManualCountdown();

    void     setCalibDoneTs(uint32_t ts);
    uint32_t getCalibDoneTs();

    // ── Manual calib ADC snapshot storage ────────────────────────
    void setManualOpenRaw (const float raw[NUM_FINGERS]);
    void getManualOpenRaw (float out[NUM_FINGERS]);
    void setManualCloseRaw(const float raw[NUM_FINGERS]);
    void getManualCloseRaw(float out[NUM_FINGERS]);

    // ── Manual calib NVS save handshake ──────────────────────────
    bool isManualSaveDone();
    void setManualSaveDone(bool v);

    // ── Atomic snapshot for display_task ─────────────────────────
    bool readSystemSnapshot(
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
        uint32_t&        calib_done_ts
    );

    // ── Event group ───────────────────────────────────────────────
    EventBits_t waitEventBits(EventBits_t bits, bool clearOnExit,
                              bool waitForAll, TickType_t timeout);
    void        setEventBits(EventBits_t bits);
    void        clearEventBits(EventBits_t bits);

private:
    SharedState() = default;

    bool _take(SemaphoreHandle_t m);

    // ── Kernel objects ────────────────────────────────────────────
    SemaphoreHandle_t  _mtx_mode    = nullptr;
    SemaphoreHandle_t  _mtx_sensors = nullptr;
    SemaphoreHandle_t  _mtx_motors  = nullptr;
    EventGroupHandle_t _events      = nullptr;
    bool               _initialized = false;

    // ── Mode-domain fields (guarded by _mtx_mode) ─────────────────
    SystemMode  _mode              = SystemMode::SAFE_LOCK;
    bool        _estop             = false;
    char        _warning[32]       = {};

    // Auto calibration
    CalibPhase  _calib_phase       = CalibPhase::IDLE;
    bool        _calib_complete    = false;
    bool        _calib_in_progress = false;
    bool        _calib_manual_mode = false;
    bool        _request_recalib   = false;
    int         _countdown_sec     = 0;

    // Manual calibration step machine
    ManualCalibStep _manual_calib_step      = ManualCalibStep::IDLE;
    bool            _manual_calib_confirmed = false;
    bool            _manual_calib_more      = false;
    int             _manual_countdown       = 0;
    uint32_t        _calib_done_ts          = 0;

    // Manual calibration ADC snapshot storage
    float _manual_open_raw [NUM_FINGERS] = {};
    float _manual_close_raw[NUM_FINGERS] = {};
    bool  _manual_save_done              = false;

    // ── Sensor domain (guarded by _mtx_sensors) ───────────────────
    SensorSnapshot _sensors = {};

    // ── Motor domain (guarded by _mtx_motors) ─────────────────────
    MotorState _motors[NUM_FINGERS] = {};
};