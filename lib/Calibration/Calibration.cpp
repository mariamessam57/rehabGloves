// ================================================================
//  Calibration.cpp  — NON-BLOCKING FSM-COMPLIANT IMPLEMENTATION
//
//  Violation elimination summary:
//
//  [V-1]  FIXED: while(prepare_seconds) loop → REMOVED entirely.
//                Timing is now control_task's WARN_OPEN countdown.
//
//  [V-2]  FIXED: while(remaining_seconds) loop → REMOVED entirely.
//                Replaced by addSample() called from sensor_task
//                on every 20ms tick. 250 calls × 20ms = 5s of
//                samples; control_task's tick timer ends the phase.
//
//  [V-3]  FIXED: ss.setCountdown() → REMOVED. control_task writes
//                countdown to SharedState from its own tick timer.
//                CalibrationSystem has zero SharedState access.
//
//  [V-4]  FIXED: analogRead() → REMOVED. addSample() accepts raw[]
//                passed from sensor_task's own flex getData().raw.
//
//  [V-5]  FIXED: loop_counter / loops_per_second hidden FSM →
//                REMOVED. No internal timing state whatsoever.
//
//  [V-6]  FIXED: vTaskDelay() → REMOVED. Every function returns
//                immediately. No blocking anywhere.
//
//  [V-7]  FIXED: startPhase() → REMOVED. Replaced by explicit
//                computeOpenHand() / computeCloseHand() called by
//                control_task at the moment it chooses.
//
//  [V-8]  FIXED: Range validation inside startPhase → moved to
//                validate(), called by control_task explicitly.
//                CalibrationSystem returns a code; control_task
//                decides what to do with it.
//
//  [V-9]  FIXED: static float bufs[][] inside function → moved to
//                instance variable _buf[][]. No stack allocation.
//                No reentrance hazard.
//
//  [V-10] FIXED: runCalibration() sequence → REMOVED. control_task
//                owns the OPEN→CLOSE transition.
//
//  [V-11] FIXED: phase_cb() calls → REMOVED. control_task calls
//                setCalibPhase() directly at the right moment.
//
//  [V-12] FIXED: vTaskDelay(50) between phases → REMOVED.
//
//  [V-13] FIXED: bool return driving upstream FSM → replaced by
//                CalibResultCode enum with explicit error cases.
// ================================================================

#include "Calibration.h"
#include <Arduino.h>
#include <string.h>

// ================================================================
//  resetBuffers — full reset at calibration start
//  Called by: control_task (once, at CALIBRATING entry)
// ================================================================
void CalibrationSystem::resetBuffers() {
    memset(_buf, 0, sizeof(_buf));
    _sample_count   = 0;
    _result         = CalibrationResult{};
    Serial.println("[CALIB] Buffers reset.");
}

// ================================================================
//  resetSampleBuffer — partial reset between phases
//  Called by: control_task (after computeOpenHand, before CLOSE phase)
// ================================================================
void CalibrationSystem::resetSampleBuffer() {
    memset(_buf, 0, sizeof(_buf));
    _sample_count = 0;
    Serial.println("[CALIB] Sample buffer cleared for next phase.");
}

// ================================================================
//  addSample — O(1), non-blocking sample ingestion
//  Called by: sensor_task on every 20ms tick during a calib phase
//
//  raw[] contains NUM_FINGERS raw ADC values read by sensor_task's
//  own flex sensor objects. No analogRead() here.
//
//  Returns true  → sample accepted.
//  Returns false → buffer full; sensor_task should stop calling
//                  (control_task's timer will fire anyway).
// ================================================================
bool CalibrationSystem::addSample(const float raw[NUM_FINGERS]) {
    if (_sample_count >= CALIB_SAMPLES_MAX) {
        // Buffer full — not an error; just silently cap.
        return false;
    }

    for (int f = 0; f < NUM_FINGERS; f++) {
        _buf[f][_sample_count] = raw[f];
    }
    _sample_count++;
    return true;
}

// ================================================================
//  computeOpenHand — pure math, O(n), non-blocking
//  Called by: control_task when OPEN_HAND phase timer expires
//
//  Applies SigmaFilter to remove outliers, then records the
//  filtered mean as the open-hand reference for each finger.
//
//  Returns false if no samples were collected (guard only;
//  normal operation always produces samples).
// ================================================================
bool CalibrationSystem::computeOpenHand() {
    if (_sample_count == 0) {
        Serial.println("[CALIB] computeOpenHand: no samples.");
        return false;
    }

    Serial.printf("[CALIB] computeOpenHand: %d samples\n", _sample_count);

    for (int f = 0; f < NUM_FINGERS; f++) {
        _result.open_raw[f] = SigmaFilter::compute(
            _buf[f], _sample_count, CALIB_SIGMA);

        Serial.printf("[CALIB]   F%d open_raw = %.1f\n",
                      f, _result.open_raw[f]);
    }

    _result.open_computed = true;
    return true;
}

// ================================================================
//  computeCloseHand — pure math, O(n), non-blocking
//  Called by: control_task when CLOSE_HAND phase timer expires
//
//  Same operation as computeOpenHand but stores close reference.
// ================================================================
bool CalibrationSystem::computeCloseHand() {
    if (_sample_count == 0) {
        Serial.println("[CALIB] computeCloseHand: no samples.");
        return false;
    }

    Serial.printf("[CALIB] computeCloseHand: %d samples\n", _sample_count);

    for (int f = 0; f < NUM_FINGERS; f++) {
        _result.close_raw[f] = SigmaFilter::compute(
            _buf[f], _sample_count, CALIB_SIGMA);

        Serial.printf("[CALIB]   F%d close_raw = %.1f\n",
                      f, _result.close_raw[f]);
    }

    _result.close_computed = true;
    return true;
}

// ================================================================
//  validate — pure computation, O(n), non-blocking
//  Called by: control_task after computeCloseHand()
//
//  Checks that both phases were computed and that the range
//  (close - open) meets the minimum threshold per finger.
//
//  On success: populates _result.calib[] and sets success=true.
//  Returns CALIB_OK on success, error code otherwise.
//
//  NOTE: control_task decides what to do with the error code.
//        CalibrationSystem has no authority over FSM transitions.
// ================================================================
CalibResultCode CalibrationSystem::validate() {
    if (!_result.open_computed || !_result.close_computed) {
        Serial.println("[CALIB] validate: phases not complete.");
        return CalibResultCode::CALIB_PHASES_MISSING;
    }

    for (int f = 0; f < NUM_FINGERS; f++) {
        float range = _result.close_raw[f] - _result.open_raw[f];

        Serial.printf("[CALIB]   F%d range = %.1f (min=%d)\n",
                      f, range, CALIB_MIN_RANGE);

        if (range < (float)CALIB_MIN_RANGE) {
            Serial.printf("[CALIB] FAIL F%d range too small: %.1f\n",
                          f, range);
            _result.success = false;
            return CalibResultCode::CALIB_RANGE_TOO_SMALL;
        }

        // Populate FlexCalib for this finger
        _result.calib[f].min_raw = _result.open_raw[f];
        _result.calib[f].max_raw = _result.close_raw[f];
        _result.calib[f].valid   = true;
    }

    _result.success = true;
    Serial.println("[CALIB] Validation passed.");
    return CalibResultCode::CALIB_OK;
}

// ================================================================
//  save — NVS write, blocking but brief (flash write)
//  Called by: sensor_task after control_task confirms CALIB_OK
//
//  Accepts CalibrationResult directly so sensor_task never needs
//  to query CalibrationSystem after the fact.
// ================================================================
bool CalibrationSystem::save(const CalibrationResult& result) {
    if (!result.success) {
        Serial.println("[CALIB] save: result not valid, skipping.");
        return false;
    }

    _prefs.begin(PREFS_NAMESPACE, false);
    for (int f = 0; f < NUM_FINGERS; f++) {
        char k_min[12], k_max[12];
        snprintf(k_min, sizeof(k_min), "f%d_min", f);
        snprintf(k_max, sizeof(k_max), "f%d_max", f);
        _prefs.putFloat(k_min, result.calib[f].min_raw);
        _prefs.putFloat(k_max, result.calib[f].max_raw);
    }
    _prefs.putBool("valid", true);
    _prefs.end();

    Serial.println("[CALIB] Saved to NVS.");
    return true;
}

// ================================================================
//  load — NVS read, blocking but brief
//  Called by: sensor_task on boot to check for saved calibration
// ================================================================
bool CalibrationSystem::load(FlexCalib out_calib[NUM_FINGERS]) {
    _prefs.begin(PREFS_NAMESPACE, true);
    bool valid = _prefs.getBool("valid", false);

    if (valid) {
        for (int f = 0; f < NUM_FINGERS; f++) {
            char k_min[12], k_max[12];
            snprintf(k_min, sizeof(k_min), "f%d_min", f);
            snprintf(k_max, sizeof(k_max), "f%d_max", f);
            out_calib[f].min_raw = _prefs.getFloat(k_min, 0.0f);
            out_calib[f].max_raw = _prefs.getFloat(k_max, 4095.0f);
            out_calib[f].valid   = true;
        }
        Serial.println("[CALIB] Loaded from NVS.");
    } else {
        Serial.println("[CALIB] No valid calibration in NVS.");
    }

    _prefs.end();
    return valid;
}

// ================================================================
//  clear — erase NVS calibration data
//  Called by: sensor_task or control_task when recalibrating
// ================================================================
void CalibrationSystem::clear() {
    _prefs.begin(PREFS_NAMESPACE, false);
    _prefs.clear();
    _prefs.end();
    Serial.println("[CALIB] NVS calibration cleared.");
}