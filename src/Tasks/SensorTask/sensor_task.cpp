// ================================================================
//  sensor_task.cpp  — INTEGRATION WITH NON-BLOCKING CalibrationSystem
//
//  sensor_task responsibilities regarding calibration:
//
//  1. On every tick during OPEN_HAND or CLOSE_HAND phase:
//       Read current raw ADC values from flex sensor objects
//       Call calib_sys.addSample(raw[]) — O(1), non-blocking
//
//  2. When mode == MANUAL_CALIB / SAVING:
//       Read raw values stored by control_task (via SharedState)
//       Call calib_sys.save() to write NVS
//       Set EVT_MANUAL_SAVE_DONE
//
//  3. On boot:
//       Call calib_sys.load() to check for saved calibration
//
//  What sensor_task does NOT do:
//    ✗ Run blocking calibration loops
//    ✗ Call computeOpenHand/CloseHand (control_task does that)
//    ✗ Call validate() (control_task does that)
//    ✗ Write countdown to SharedState
//    ✗ Make any FSM decisions
//
//  Auto-calib phase flow (driven by control_task):
//
//    control_task: setCalibPhase(OPEN_HAND) + setCalibInProgress(true)
//    sensor_task: detects phase == OPEN_HAND → calls addSample() every tick
//    control_task: OPEN_HAND timer expires → calls computeOpenHand()
//                  → resets buffer → setCalibPhase(CLOSE_HAND)
//    sensor_task: detects phase == CLOSE_HAND → calls addSample() every tick
//    control_task: CLOSE_HAND timer expires → calls computeCloseHand()
//                  → calls validate() → if OK: calls save() via sensor_task
//                                              signals EVT_CALIB_DONE
//
//  Wait — save() involves flash write which can take ~5ms.
//  Calling it from control_task (priority 3) is acceptable.
//  Alternatively keep save() in sensor_task via SAVING state.
//  For auto calib, control_task calls calib_sys.save() directly
//  since sensor_task is already sampling (no conflict — save() uses
//  Preferences/NVS which is independent of the sample buffer).
//  This is simpler than the manual calib SAVING handshake.
// ================================================================

#include "Tasks/SensorTask/sensor_task.h"
#include "systemstate/System_State.h"
#include "FSM/fsm_events.h"
#include "config.h"
#include "Flex_sensor/FlexSensor.h"
#include "IMU_Sensor/imu_sensor.h"
#include "FSR_sensor/fsrsensor.h"
#include "Calibration.h"
#include <Arduino.h>

// ── Sensor objects ────────────────────────────────────────────────
static FlexSensor flex[NUM_FINGERS] = {
    FlexSensor(PIN_FLEX_0, EMA_ALPHA_FLEX),
    FlexSensor(PIN_FLEX_1, EMA_ALPHA_FLEX),
    FlexSensor(PIN_FLEX_2, EMA_ALPHA_FLEX)
};
static IMUWrapper        imu;
static FSRSensor         fsr(PIN_FSR, EMA_ALPHA_FSR);
static CalibrationSystem calib_sys;

// ── sample_and_publish — sample all sensors and write snapshot ────
static void sample_and_publish(SharedState& ss) {
    for (int f = 0; f < NUM_FINGERS; f++) flex[f].sample();
    imu.sample();
    fsr.sample();

    SensorSnapshot snap;
    for (int f = 0; f < NUM_FINGERS; f++) {
        snap.flex[f]  = flex[f].getData();
        snap.calib[f] = flex[f].getCalib();
    }
    snap.imu            = imu.getData();
    snap.fsr            = fsr.getData();
    snap.calib_complete = ss.isCalibComplete();
    snap.calib_phase    = ss.getCalibPhase();
    ss.writeSensors(snap);
}

// ================================================================
void sensor_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    for (int f = 0; f < NUM_FINGERS; f++) flex[f].begin();
    fsr.begin();

    if (!imu.begin()) {
        Serial.println("[SENSOR] MPU6050 not found!");
        ss.triggerEStop("IMU_NOT_FOUND");
        vTaskDelete(nullptr);
    }

    // ── Boot: try loading saved calibration ──────────────────────
    FlexCalib loaded[NUM_FINGERS];
    if (calib_sys.load(loaded)) {
        for (int f = 0; f < NUM_FINGERS; f++) flex[f].setCalib(loaded[f]);
        ss.setCalibComplete(true);
        ss.setCalibPhase(CalibPhase::DONE);
        ss.setEventBits(EVT_CALIB_DONE);
        Serial.println("[SENSOR] Calibration loaded from NVS.");
    } else {
        // No saved calib: control_task will handle entry into
        // CALIBRATING mode when it sees !isCalibComplete().
        ss.setCalibComplete(false);
        ss.setCalibPhase(CalibPhase::IDLE);
        Serial.println("[SENSOR] No calibration found. Awaiting calib.");
    }

    // Initial publish so safety_task sees a live IMU timestamp
    sample_and_publish(ss);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // Always sample sensors first (provides live data for
        // safety_task and display_task regardless of mode)
        sample_and_publish(ss);

        const SystemMode  mode  = ss.getMode();
        const CalibPhase  phase = ss.getCalibPhase();

        // ── AUTO CALIBRATION: feed samples into calib_sys ────────
        // sensor_task feeds addSample() on every tick while the
        // control_task's phase timer is running.
        // control_task sets CalibInProgress=true for the duration.
        // No blocking. No waiting. Pure data feed.
        if (mode == SystemMode::CALIBRATING && ss.isCalibInProgress()) {

            if (phase == CalibPhase::OPEN_HAND ||
                phase == CalibPhase::CLOSE_HAND)
            {
                // Build raw array from flex sensor objects
                // (sensor_task owns the flex objects — no analogRead here)
                float raw[NUM_FINGERS];
                for (int f = 0; f < NUM_FINGERS; f++) {
                    raw[f] = flex[f].getData().raw;
                }

                bool accepted = calib_sys.addSample(raw);
                if (!accepted) {
                    // Buffer full — sample silently dropped.
                    // control_task's timer will end the phase
                    // before this becomes an issue at 20ms×400=8s.
                }
            }
        }

        // ── MANUAL CALIBRATION: SAVING handshake ─────────────────
        // control_task stored raw ADC values via setManualOpenRaw/
        // setManualCloseRaw. sensor_task reads them, builds result,
        // saves to NVS, signals EVT_MANUAL_SAVE_DONE.
        // Runs only once (isManualSaveDone() gates re-entry).
        if (mode == SystemMode::MANUAL_CALIB &&
            ss.getManualCalibStep() == ManualCalibStep::SAVING &&
            !ss.isManualSaveDone())
        {
            float open_raw [NUM_FINGERS];
            float close_raw[NUM_FINGERS];
            ss.getManualOpenRaw (open_raw);
            ss.getManualCloseRaw(close_raw);

            // Build CalibrationResult from control_task's snapshots
            CalibrationResult manual_result;
            manual_result.open_computed  = true;
            manual_result.close_computed = true;
            bool range_ok = true;

            for (int f = 0; f < NUM_FINGERS; f++) {
                manual_result.open_raw [f] = open_raw [f];
                manual_result.close_raw[f] = close_raw[f];

                float range = close_raw[f] - open_raw[f];
                Serial.printf("[SENSOR] Manual F%d: open=%.0f close=%.0f range=%.0f\n",
                              f, open_raw[f], close_raw[f], range);

                if (range < (float)CALIB_MIN_RANGE) {
                    Serial.printf("[SENSOR] FAIL: range too small F%d\n", f);
                    range_ok = false;
                    break;
                }

                manual_result.calib[f].min_raw = open_raw [f];
                manual_result.calib[f].max_raw = close_raw[f];
                manual_result.calib[f].valid   = true;
            }

            if (!range_ok) {
                // Validation failed — trigger ESTOP.
                // control_task will handle mode transition.
                ss.triggerEStop("MANUAL_CALIB_RANGE");
            } else {
                manual_result.success = true;
                for (int f = 0; f < NUM_FINGERS; f++) {
                    flex[f].setCalib(manual_result.calib[f]);
                }
                calib_sys.save(manual_result);
                ss.setManualSaveDone(true);   // signal control_task
                Serial.println("[SENSOR] Manual calib saved.");
                // EVT_MANUAL_SAVE_DONE is set by sensor_task here
                ss.setEventBits(EVT_MANUAL_SAVE_DONE);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}