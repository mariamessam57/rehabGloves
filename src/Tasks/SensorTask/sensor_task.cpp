// ================================================================
//  sensor_task.cpp  — REDESIGNED
//
//  Key changes vs original:
//  [S-1]  Manual calibration (MANUAL_CALIB mode) does NOT block
//         sensor_task. sensor_task samples normally and, when the
//         user confirms a position, snapshots current ADC raw
//         values and stores them via CalibrationSystem.
//
//  [S-2]  Auto calibration still blocks (unavoidable — it needs
//         continuous ADC sampling for 5 s). CalibInProgress is set
//         BEFORE blocking so safety_task suppresses SENSOR_STALE.
//
//  [S-3]  last_wake is reset after any blocking calibration call
//         so vTaskDelayUntil does not over-deadline.
//
//  [S-4]  sample_and_publish() helper used everywhere to keep a
//         live IMU timestamp in SharedState at all times.
// ================================================================

#include "Tasks/SensorTask/sensor_task.h"
#include "systemstate/System_State.h"
#include "config.h"
#include "Flex_sensor/FlexSensor.h"
#include "IMU_Sensor/imu_sensor.h"
#include "FSR_sensor/fsrsensor.h"
#include "Calibration/Calibration.h"
#include <Arduino.h>

// ── Static sensor objects ────────────────────────────────────────
static FlexSensor flex[NUM_FINGERS] = {
    FlexSensor(PIN_FLEX_0, EMA_ALPHA_FLEX),
    FlexSensor(PIN_FLEX_1, EMA_ALPHA_FLEX),
    FlexSensor(PIN_FLEX_2, EMA_ALPHA_FLEX)
};
static IMUWrapper        imu;
static FSRSensor         fsr(PIN_FSR, EMA_ALPHA_FSR);
static CalibrationSystem calib_sys;

static const uint8_t FLEX_PINS[NUM_FINGERS] = {
    PIN_FLEX_0, PIN_FLEX_1, PIN_FLEX_2
};

// Stored raw values for manual calibration
static float manual_open_raw [NUM_FINGERS] = {};
static float manual_close_raw[NUM_FINGERS] = {};

static void phase_cb(CalibPhase p) {
    SharedState::get().setCalibPhase(p);
}

// ── [S-4] sample_and_publish ────────────────────────────────────
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

// ── Snapshot raw ADC values into a float array ───────────────────
static void snapshot_raw(float out[NUM_FINGERS]) {
    for (int f = 0; f < NUM_FINGERS; f++) {
        out[f] = (float)analogRead(FLEX_PINS[f]);
    }
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

    // ── Try loading saved calibration ────────────────────────────
    FlexCalib loaded[NUM_FINGERS];
    bool have_calib = calib_sys.load(loaded);

    if (have_calib) {
        for (int f = 0; f < NUM_FINGERS; f++) flex[f].setCalib(loaded[f]);
        ss.setCalibPhase(CalibPhase::DONE);
        ss.setCalibComplete(true);
        ss.setCalibInProgress(false);
        ss.setEventBits(EVT_CALIB_DONE);
        Serial.println("[SENSOR] Calibration loaded from NVS.");
    } else {
        ss.requestRecalibration();
        ss.setMode(SystemMode::CALIBRATING);
        ss.setCalibPhase(CalibPhase::IDLE);
        ss.setCalibComplete(false);
        ss.setCalibInProgress(false);
        Serial.println("[SENSOR] Waiting for calibration choice.");
    }

    // Initial publish so safety_task never sees a zero timestamp
    sample_and_publish(ss);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        SystemMode mode = ss.getMode();

        // ── AUTO CALIBRATION ─────────────────────────────────────
        if (mode == SystemMode::CALIBRATING
            && !ss.isEStop()
            && ss.shouldRecalibrate()
            && !ss.isCalibInProgress()
            && ss.getCalibPhase() != CalibPhase::IDLE)
        {
            // [S-2] Set flag BEFORE blocking
            ss.setCalibInProgress(true);

            // [S-4] Fresh timestamp before long block
            sample_and_publish(ss);

            Serial.println("[SENSOR] Auto calibration starting.");

            FlexCalib new_calib[NUM_FINGERS];
            bool ok = calib_sys.runCalibration(FLEX_PINS, new_calib, phase_cb);

            if (!ok) {
                ss.setCalibInProgress(false);
                ss.setCalibPhase(CalibPhase::FAILED);
                ss.triggerEStop("CALIB_FAILED");
                vTaskDelete(nullptr);
            }

            for (int f = 0; f < NUM_FINGERS; f++) flex[f].setCalib(new_calib[f]);
            calib_sys.save(new_calib);

            ss.setCalibComplete(true);
            ss.clearRecalibrationRequest();
            ss.setCalibInProgress(false);
            ss.setCalibPhase(CalibPhase::DONE);

            // 2-second "Calibration Completed" display
            ss.setCalibDoneTs(millis());
            ss.setMode(SystemMode::CALIBRATING); // display_task transitions to SAFE_LOCK

            ss.setEventBits(EVT_CALIB_DONE);
            Serial.println("[SENSOR] Auto calibration complete.");

            // [S-3] Reset timing after blocking
            last_wake = xTaskGetTickCount();
        }

        // ── MANUAL CALIBRATION — snapshot on confirm ──────────────
        // [S-1] sensor_task is NOT blocked during manual calib.
        // It just watches for user confirmations and records the
        // current ADC values at that moment.
        if (mode == SystemMode::MANUAL_CALIB && !ss.isEStop()) {
            ManualCalibStep step = ss.getManualCalibStep();

            if (step == ManualCalibStep::WAIT_OPEN_CONFIRM
                && ss.getManualCalibConfirmed())
            {
                // User pressed 1:Yes for OPEN position
                snapshot_raw(manual_open_raw);
                for (int f = 0; f < NUM_FINGERS; f++) {
                    Serial.printf("[SENSOR] Open raw F%d = %.1f\n",
                                  f, manual_open_raw[f]);
                }
                ss.setManualCalibConfirmed(false);
                ss.setManualCalibStep(ManualCalibStep::MOVING_CLOSE);
                Serial.println("[SENSOR] Open confirmed, moving to close phase.");
            }
            else if (step == ManualCalibStep::WAIT_CLOSE_CONFIRM
                     && ss.getManualCalibConfirmed())
            {
                // User pressed 1:Yes for CLOSE position
                snapshot_raw(manual_close_raw);
                for (int f = 0; f < NUM_FINGERS; f++) {
                    Serial.printf("[SENSOR] Close raw F%d = %.1f\n",
                                  f, manual_close_raw[f]);
                }
                ss.setManualCalibConfirmed(false);
                ss.setManualCalibStep(ManualCalibStep::SAVING);

                // Build and save calibration
                FlexCalib new_calib[NUM_FINGERS];
                bool valid = true;
                for (int f = 0; f < NUM_FINGERS; f++) {
                    new_calib[f].min_raw = manual_open_raw[f];
                    new_calib[f].max_raw = manual_close_raw[f];
                    float range = new_calib[f].max_raw - new_calib[f].min_raw;
                    if (range < CALIB_MIN_RANGE) {
                        Serial.printf("[SENSOR] Manual calib range too small F%d: %.1f\n",
                                      f, range);
                        valid = false;
                    }
                    new_calib[f].valid = valid;
                }

                if (!valid) {
                    ss.setCalibPhase(CalibPhase::FAILED);
                    ss.triggerEStop("MANUAL_CALIB_RANGE");
                } else {
                    for (int f = 0; f < NUM_FINGERS; f++) flex[f].setCalib(new_calib[f]);
                    calib_sys.save(new_calib);
                    ss.setCalibComplete(true);
                    ss.setCalibPhase(CalibPhase::DONE);
                    ss.setManualCalibStep(ManualCalibStep::DONE);
                    ss.setCalibDoneTs(millis());
                    ss.setEventBits(EVT_CALIB_DONE);
                    Serial.println("[SENSOR] Manual calibration saved.");
                    // display_task will auto-return to SAFE_LOCK after 2s
                }
            }
        }

        // ── Skip normal publish only while auto-calib is blocking ─
        if (ss.isEStop() || ss.isCalibInProgress()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        sample_and_publish(ss);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}