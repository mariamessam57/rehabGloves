#include "Calibration.h"
#include <Arduino.h>

CalibrationSystem::CalibrationSystem() {}

bool CalibrationSystem::runCalibration(
    const uint8_t flex_pins[NUM_FINGERS],
    FlexCalib out_calib[NUM_FINGERS],
    void (*phase_cb)(CalibPhase))
{
    static float bufs[NUM_FINGERS][CALIB_SAMPLES_MAX];
    int sample_count = 0;

    // ── PHASE 1: OPEN HAND ──────────────────────────────────────
    if (phase_cb) phase_cb(CalibPhase::OPEN_HAND);
    Serial.println("[CALIB] Phase 1: Open hand...");
    _collectSamples(flex_pins, bufs, sample_count, CALIB_DURATION_MS);

    for (int f = 0; f < NUM_FINGERS; f++) {
        out_calib[f].min_raw = SigmaFilter::compute(
            bufs[f], sample_count, CALIB_SIGMA);
    }

    // ── PHASE 2: CLOSE HAND ─────────────────────────────────────
    if (phase_cb) phase_cb(CalibPhase::CLOSE_HAND);
    Serial.println("[CALIB] Phase 2: Close hand...");
    _collectSamples(flex_pins, bufs, sample_count, CALIB_DURATION_MS);

    for (int f = 0; f < NUM_FINGERS; f++) {
        out_calib[f].max_raw = SigmaFilter::compute(
            bufs[f], sample_count, CALIB_SIGMA);

        // Validate range
        float range = out_calib[f].max_raw - out_calib[f].min_raw;
        if (range < (float)CALIB_MIN_RANGE) {
            Serial.printf("[CALIB] Finger %d range too small: %.1f\n", f, range);
            if (phase_cb) phase_cb(CalibPhase::FAILED);
            return false;
        }
        out_calib[f].valid = true;
        Serial.printf("[CALIB] F%d min=%.0f max=%.0f range=%.0f\n",
            f, out_calib[f].min_raw, out_calib[f].max_raw, range);
    }

    if (phase_cb) phase_cb(CalibPhase::DONE);
    save(out_calib);
    Serial.println("[CALIB] Done — saved to NVS.");
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