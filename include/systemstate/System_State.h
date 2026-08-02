#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

// ================================================================
//  System_State.h — FIXED (round 2)
//
//  FIX-R2-A: Removed Serial.println() from the inline _take()
//  method.  Serial requires Arduino.h.  When System_State.h is
//  included by a TU that pulls in FreeRTOS headers before
//  Arduino.h, the compiler cannot resolve 'Serial' at the point
//  of the inline definition, producing "Serial was not declared
//  in this scope" across every TU that includes this header.
//
//  Fix: _take() is now a non-inline declaration only.  The
//  implementation lives in System_State.cpp where Arduino.h is
//  unconditionally included first.
// ================================================================

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "SystemTypes.h"
#include "Config.h"

#define EVT_ESTOP       (1 << 0)
#define EVT_CALIB_DONE  (1 << 1)

// ── System-wide I2C mutex (FIX C5) ───────────────────────────────
extern SemaphoreHandle_t g_i2c_mutex;

inline void i2c_mutex_init() {
    if (g_i2c_mutex == nullptr)
        g_i2c_mutex = xSemaphoreCreateMutex();
}

// RAII I2C bus guard — works without Arduino.h
struct I2CLockGuard {
    bool _held;
    explicit I2CLockGuard(TickType_t timeout_ticks = pdMS_TO_TICKS(20))
        : _held(g_i2c_mutex &&
                xSemaphoreTake(g_i2c_mutex, timeout_ticks) == pdTRUE) {}
    ~I2CLockGuard() { if (_held) xSemaphoreGive(g_i2c_mutex); }
    bool ok() const { return _held; }
};

// ================================================================

class SharedState {
public:
    static SharedState& get() {
        static SharedState instance;
        return instance;
    }

    void init();

    SystemMode getMode();
    void       setMode(SystemMode m);

    void writeSensors(const SensorSnapshot& snap);
    void readSensors(SensorSnapshot& out);

    void writeMotors(const MotorState motors[NUM_FINGERS]);
    void readMotors(MotorState out[NUM_FINGERS]);

    void triggerEStop(const char* reason);
    void clearEStop();
    bool isEStop();

    bool       isCalibComplete();
    void       setCalibComplete(bool v);
    void       setCalibPhase(CalibPhase p);
    CalibPhase getCalibPhase();

    void requestRecalibration();
    bool shouldRecalibrate();
    void clearRecalibrationRequest();

    void setWarning(const char* msg);
    void clearWarning();

    void            setManualCalibStep(ManualCalibStep s);
    ManualCalibStep getManualCalibStep();
    void            setManualCalibCountdown(int cd);
    void            setCalibDoneTimestamp(TickType_t ts);
    void            clearCalibDoneTimestamp();

    bool readSystemSnapshot(
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
        TickType_t&      calib_done_ts_out
    );

    EventGroupHandle_t events = nullptr;

private:
    SharedState() = default;
    SharedState(const SharedState&) = delete;

    SemaphoreHandle_t _mtx_mode    = nullptr;
    SemaphoreHandle_t _mtx_sensors = nullptr;
    SemaphoreHandle_t _mtx_motors  = nullptr;
    SemaphoreHandle_t _mtx_flags   = nullptr;
    SemaphoreHandle_t _mtx_display = nullptr;

    SystemMode      _mode            = SystemMode::SAFE_LOCK;
    SensorSnapshot  _sensors         = {};
    MotorState      _motors[NUM_FINGERS] = {};

    bool            _estop           = false;
    bool            _calib_complete  = false;
    CalibPhase      _calib_phase     = CalibPhase::IDLE;
    bool            _request_recalib = false;

    char            _warning[32]     = {};

    ManualCalibStep _manual_step     = ManualCalibStep::IDLE;
    int             _manual_cd       = 0;
    TickType_t      _calib_done_ts   = 0;

    // FIX-R2-A: non-inline — defined in .cpp so Serial is in scope
    bool _take(SemaphoreHandle_t m,
               TickType_t ticks = pdMS_TO_TICKS(10));
};

#endif // SYSTEM_STATE_H