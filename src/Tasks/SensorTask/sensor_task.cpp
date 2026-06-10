#include "Tasks/SensorTask/sensor_task.h"
#include "systemstate/System_State.h"
#include "config.h"
#include "Flex_sensor/FlexSensor.h"
#include "IMU_Sensor/imu_sensor.h"
#include "FSR_sensor/fsrsensor.h"
#include "Calibration/Calibration.h"
#include <Arduino.h>

// ── Static sensor objects ────────────────────────────────────────
static FlexSensor   flex[NUM_FINGERS] = {
    FlexSensor(PIN_FLEX_0, EMA_ALPHA_FLEX),
    FlexSensor(PIN_FLEX_1, EMA_ALPHA_FLEX),
    FlexSensor(PIN_FLEX_2, EMA_ALPHA_FLEX)
};
static IMUWrapper   imu;
static FSRSensor    fsr(PIN_FSR, EMA_ALPHA_FSR);
static CalibrationSystem calib_sys;

// ── Pin array for calibration ────────────────────────────────────
static const uint8_t FLEX_PINS[NUM_FINGERS] = {
    PIN_FLEX_0, PIN_FLEX_1, PIN_FLEX_2
};

// ── Phase callback (runs inside FreeRTOS task) ───────────────────
static void phase_cb(CalibPhase p) {
    SharedState::get().setCalibPhase(p);
}

// ================================================================
void sensor_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    // ── Hardware init ────────────────────────────────────────────
    for (int f = 0; f < NUM_FINGERS; f++) flex[f].begin();
    fsr.begin();

    if (!imu.begin()) {
        Serial.println("[SENSOR] MPU6050 not found!");
        ss.triggerEStop("IMU_NOT_FOUND");
        vTaskDelete(nullptr);
    }

    // ── Try loading saved calibration ────────────────────────────
    FlexCalib loaded[NUM_FINGERS];
    bool      have_calib = calib_sys.load(loaded);

    if (have_calib) {
        for (int f = 0; f < NUM_FINGERS; f++) flex[f].setCalib(loaded[f]);
        ss.setCalibPhase(CalibPhase::DONE);
        ss.setCalibComplete(true);
        ss.setCalibInProgress(false);
        xEventGroupSetBits(ss.events, EVT_CALIB_DONE);
        Serial.println("[SENSOR] Calibration loaded from NVS.");
    } else {
        // Request calibration and wait for user confirmation first
        ss.requestRecalibration();
        ss.setMode(SystemMode::CALIBRATING);
        ss.setCalibPhase(CalibPhase::IDLE);
        ss.setCalibComplete(false);
        ss.setCalibInProgress(false);
        Serial.println("[SENSOR] Waiting to start calibration.");
    }

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (!ss.isEStop() && ss.shouldRecalibrate() && !ss.isCalibInProgress()) {
            if (ss.getCalibPhase() != CalibPhase::IDLE) {
                ss.setCalibInProgress(true);
                ss.setMode(SystemMode::CALIBRATING);

                FlexCalib new_calib[NUM_FINGERS];
                bool ok = calib_sys.runCalibration(FLEX_PINS, new_calib, phase_cb);
                if (!ok) {
                    ss.setCalibInProgress(false);
                    ss.triggerEStop("CALIB_FAILED");
                    vTaskDelete(nullptr);
                }
                for (int f = 0; f < NUM_FINGERS; f++) flex[f].setCalib(new_calib[f]);
                ss.setCalibComplete(true);
                ss.clearRecalibrationRequest();
                ss.setCalibInProgress(false);
                ss.setMode(SystemMode::SAFE_LOCK);
                xEventGroupSetBits(ss.events, EVT_CALIB_DONE);
                Serial.println("[SENSOR] Calibration completed.");
            }
        }

        // Skip if ESTOP or if calibration is actively running
        if (ss.isEStop() || ss.isCalibInProgress()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ── Sample all sensors ───────────────────────────────────
        for (int f = 0; f < NUM_FINGERS; f++) flex[f].sample();
        imu.sample();
        fsr.sample();

        // ── Build snapshot and publish ───────────────────────────
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

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}