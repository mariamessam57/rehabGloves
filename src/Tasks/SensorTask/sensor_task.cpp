#include "Tasks/SensorTask/sensor_task.h"
#include "systemstate/System_State.h"
#include "config.h"
#include "Flex_sensor/FlexSensor.h"
#include "IMU_Sensor/imu_sensor.h"
#include "FSR_sensor/fsrsensor.h"
#include "Calibration/Calibration.h"
#include <Arduino.h>

extern volatile bool calib_done;

// ── Static sensor objects ─────────────────────────────────────────
static FlexSensor flex[NUM_FINGERS] = {
    FlexSensor(PIN_FLEX_0, EMA_ALPHA_FLEX),
    FlexSensor(PIN_FLEX_1, EMA_ALPHA_FLEX),
    FlexSensor(PIN_FLEX_2, EMA_ALPHA_FLEX)};

static IMUWrapper imu;
static FSRSensor fsr(PIN_FSR, EMA_ALPHA_FSR);
static CalibrationSystem calib_sys;

static const uint8_t FLEX_PINS[NUM_FINGERS] = {
    PIN_FLEX_0, PIN_FLEX_1, PIN_FLEX_2};

// ── phase callback ────────────────────────────────────────────────
static void phase_cb(CalibPhase p)
{
    SharedState::get().setCalibPhase(p);
}

// ── apply calibration ─────────────────────────────────────────────
static void applyCalib(FlexCalib calib[NUM_FINGERS])
{
    SharedState &ss = SharedState::get();

    for (int f = 0; f < NUM_FINGERS; f++)
        flex[f].setCalib(calib[f]);

    ss.setCalibComplete(true);
    ss.clearRecalibrationRequest();

    xEventGroupSetBits(ss.events, EVT_CALIB_DONE);

    ss.setCalibDoneTimestamp(xTaskGetTickCount());
}

// ── sample sensors ───────────────────────────────────────────────
static void sample_and_publish(SharedState &ss)
{

    for (int f = 0; f < NUM_FINGERS; f++)
        flex[f].sample();

    imu.sample();
    fsr.sample();

    SensorSnapshot snap;

    for (int f = 0; f < NUM_FINGERS; f++)
    {
        snap.flex[f] = flex[f].getData();
        snap.calib[f] = flex[f].getCalib();
    }

    // compute per-flex velocity (normalized/sec)
    static float prev_norm[NUM_FINGERS] = {0};
    static uint32_t prev_ms[NUM_FINGERS] = {0};
    uint32_t now = millis();
    for (int f = 0; f < NUM_FINGERS; f++)
    {
        float n = snap.flex[f].normalized;
        uint32_t last = prev_ms[f];
        if (last == 0 || now <= last)
        {
            snap.flex[f].velocity = 0.0f;
        }
        else
        {
            float dt = (now - last) / 1000.0f;
            snap.flex[f].velocity = (n - prev_norm[f]) / (dt > 0 ? dt : 1.0f);
        }
        snap.flex[f].last_ms = now;
        prev_norm[f] = n;
        prev_ms[f] = now;
    }

    snap.imu = imu.getData();
    snap.fsr = fsr.getData();

    snap.calib_complete = ss.isCalibComplete();
    snap.calib_phase = ss.getCalibPhase();

    ss.writeSensors(snap);
}

// ================================================================
void sensor_task(void *pvParam)
{

    while (!calib_done)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    SharedState &ss = SharedState::get();

    // ── init sensors ─────────────────────────────────────────────
    for (int f = 0; f < NUM_FINGERS; f++)
        flex[f].begin();

    fsr.begin();

    // IMU init
    if (!imu.begin())
    {
        Serial.println("[SENSOR] IMU not found!");
        ss.triggerEStop("IMU_NOT_FOUND");
        for (;;)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ── load calibration ─────────────────────────────────────────
    FlexCalib loaded[NUM_FINGERS];
    bool have_calib = calib_sys.load(loaded);

    if (have_calib)
    {

        for (int f = 0; f < NUM_FINGERS; f++)
            flex[f].setCalib(loaded[f]);

        ss.setCalibPhase(CalibPhase::DONE);
        ss.setCalibComplete(true);

        xEventGroupSetBits(ss.events, EVT_CALIB_DONE);

        Serial.println("[SENSOR] Calibration loaded from NVS.");
    }
    else
    {

        ss.setMode(SystemMode::CALIBRATING);

        FlexCalib new_calib[NUM_FINGERS];

        bool ok = calib_sys.runCalibration(FLEX_PINS, new_calib, phase_cb);

        if (!ok)
        {
            ss.triggerEStop("CALIB_FAILED");
            for (;;)
                vTaskDelay(pdMS_TO_TICKS(1000));
        }

        applyCalib(new_calib);
        Serial.println("[SENSOR] Initial calibration complete.");
    }

    // initial publish
    sample_and_publish(ss);

    TickType_t last_wake = xTaskGetTickCount();

    // ── main loop ────────────────────────────────────────────────
    for (;;)
    {

        if (ss.shouldRecalibrate() || !ss.isCalibComplete())
        {

            ss.setMode(SystemMode::CALIBRATING);

            FlexCalib new_calib[NUM_FINGERS];

            bool ok = calib_sys.runCalibration(FLEX_PINS, new_calib, phase_cb);

            if (!ok)
            {
                ss.triggerEStop("CALIB_FAILED");
                for (;;)
                    vTaskDelay(pdMS_TO_TICKS(1000));
            }

            applyCalib(new_calib);
            Serial.println("[SENSOR] Recalibration complete.");
        }

        if (ss.isEStop())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        sample_and_publish(ss);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}