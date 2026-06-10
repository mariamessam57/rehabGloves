#include "Calibration.h"
#include <Arduino.h>

CalibrationSystem::CalibrationSystem() {
    // Constructor intentionally left empty; Preferences initializes on first use.
}

bool CalibrationSystem::startPhase(
    CalibPhase phase,
    const uint8_t flex_pins[NUM_FINGERS],
    FlexCalib out_calib[NUM_FINGERS])
{
    static float bufs[NUM_FINGERS][CALIB_SAMPLES_MAX];
    int sample_count = 0;

    if (phase == CalibPhase::OPEN_HAND)
    {
        Serial.printf("[CALIB] OPEN_HAND stage: collecting samples for %u ms\n", CALIB_DURATION_MS);
        vTaskDelay(pdMS_TO_TICKS(1000)); // give user time
    }
    else
    {
        Serial.printf("[CALIB] CLOSE_HAND stage: collecting samples for %u ms\n", CALIB_DURATION_MS);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    _collectSamples(flex_pins, bufs, sample_count, CALIB_DURATION_MS);

    Serial.printf("[CALIB] %s samples collected: %d\n",
        phase == CalibPhase::OPEN_HAND ? "OPEN_HAND" : "CLOSE_HAND",
        sample_count);

    for (int f = 0; f < NUM_FINGERS; f++)
    {
        float value = SigmaFilter::compute(bufs[f], sample_count, CALIB_SIGMA);

        if (phase == CalibPhase::OPEN_HAND)
            out_calib[f].min_raw = value;
        else
            out_calib[f].max_raw = value;
    }

    // validation only after CLOSE
    if (phase == CalibPhase::CLOSE_HAND)
    {
        for (int f = 0; f < NUM_FINGERS; f++)
        {
            float range = out_calib[f].max_raw - out_calib[f].min_raw;

            if (range < CALIB_MIN_RANGE)
            {
                Serial.printf("[CALIB] FAIL F%d range=%.1f\n", f, range);
                return false;
            }

            out_calib[f].valid = true;
        }
    }

    return true;
}

bool CalibrationSystem::runCalibration(
    const uint8_t flex_pins[NUM_FINGERS],
    FlexCalib out_calib[NUM_FINGERS],
    void (*phase_cb)(CalibPhase)
)
{
    Serial.println("[CALIB] runCalibration: starting OPEN_HAND stage.");
    if (phase_cb) phase_cb(CalibPhase::OPEN_HAND);
    if (!startPhase(CalibPhase::OPEN_HAND, flex_pins, out_calib)) {
        Serial.println("[CALIB] OPEN_HAND stage failed.");
        return false;
    }
    Serial.println("[CALIB] OPEN_HAND stage complete.");

    Serial.println("[CALIB] runCalibration: starting CLOSE_HAND stage.");
    if (phase_cb) phase_cb(CalibPhase::CLOSE_HAND);
    if (!startPhase(CalibPhase::CLOSE_HAND, flex_pins, out_calib)) {
        Serial.println("[CALIB] CLOSE_HAND stage failed.");
        return false;
    }
    Serial.println("[CALIB] CLOSE_HAND stage complete.");

    if (phase_cb) phase_cb(CalibPhase::DONE);
    Serial.println("[CALIB] Calibration run complete.");
    return true;
}

void CalibrationSystem::_collectSamples(
    const uint8_t pins[NUM_FINGERS],
    float bufs[NUM_FINGERS][CALIB_SAMPLES_MAX],
    int& sample_count,
    uint32_t duration_ms)
{
    sample_count = 0;
    uint32_t start = millis();

    while ((millis() - start) < duration_ms) {
        if (sample_count < CALIB_SAMPLES_MAX) {
            for (int f = 0; f < NUM_FINGERS; f++) {
                bufs[f][sample_count] = (float)analogRead(pins[f]);
            }
            sample_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}

bool CalibrationSystem::save(const FlexCalib calib[NUM_FINGERS]) {
    _prefs.begin(PREFS_NAMESPACE, false);
    for (int f = 0; f < NUM_FINGERS; f++) {
        char k1[12], k2[12];
        snprintf(k1, sizeof(k1), "f%d_min", f);
        snprintf(k2, sizeof(k2), "f%d_max", f);
        _prefs.putFloat(k1, calib[f].min_raw);
        _prefs.putFloat(k2, calib[f].max_raw);
    }
    _prefs.putBool("valid", true);
    _prefs.end();
    return true;
}

bool CalibrationSystem::load(FlexCalib calib[NUM_FINGERS]) {
    _prefs.begin(PREFS_NAMESPACE, true);
    bool valid = _prefs.getBool("valid", false);
    if (valid) {
        for (int f = 0; f < NUM_FINGERS; f++) {
            char k1[12], k2[12];
            snprintf(k1, sizeof(k1), "f%d_min", f);
            snprintf(k2, sizeof(k2), "f%d_max", f);
            calib[f].min_raw = _prefs.getFloat(k1, 0.0f);
            calib[f].max_raw = _prefs.getFloat(k2, 4095.0f);
            calib[f].valid   = true;
        }
    }
    _prefs.end();
    return valid;
}

void CalibrationSystem::clear() {
    _prefs.begin(PREFS_NAMESPACE, false);
    _prefs.clear();
    _prefs.end();
}