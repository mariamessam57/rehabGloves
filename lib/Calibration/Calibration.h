#pragma once

// ================================================================
//  Calibration.h  — NON-BLOCKING, FSM-COMPLIANT REDESIGN
//
//  What CalibrationSystem is NOW:
//    A pure computation + NVS worker. No control flow. No timing.
//    No SharedState access. No blocking. No FSM authority.
//
//  What it was (REMOVED):
//    ✗ runCalibration()      — owned OPEN→CLOSE sequence (V-10)
//    ✗ startPhase()          — called _collectSamples (V-7)
//    ✗ _collectSamples()     — 7-second blocking loop (V-1,V-2)
//    ✗ ss.setCountdown()     — wrote to SharedState (V-3)
//    ✗ analogRead() direct   — bypassed sensor_task (V-4)
//    ✗ vTaskDelay() inside   — blocked calling task (V-6)
//    ✗ phase_cb()            — drove FSM transitions (V-11)
//    ✗ OPEN→CLOSE sequencing — owned phase ordering (V-10)
//
//  NEW API CONTRACT:
//
//  Call sequence (all calls from control_task or sensor_task):
//
//    1. control_task:   calib.resetBuffers()
//       (at start of calibration entry)
//
//    2. sensor_task:    calib.addSample(raw[])      ← every 20ms tick
//       (called continuously during OPEN_HAND and CLOSE_HAND phases)
//       (sensor_task passes raw ADC values from its own flex.getData())
//
//    3. control_task:   calib.computeOpenHand()
//       (called exactly ONCE when OPEN_HAND timer expires)
//       (returns false if no samples collected)
//
//    4. control_task:   calib.resetSampleBuffer()
//       (clear buffer before CLOSE_HAND phase begins)
//
//    5. sensor_task:    calib.addSample(raw[])      ← every 20ms tick
//       (same call, now filling CLOSE_HAND buffer)
//
//    6. control_task:   calib.computeCloseHand()
//       (called exactly ONCE when CLOSE_HAND timer expires)
//
//    7. control_task:   CalibResultCode rc = calib.validate()
//       (validates range; rc == CALIB_OK means success)
//
//    8. control_task:   CalibrationResult r = calib.getResult()
//       (retrieve computed FlexCalib data)
//
//    9. sensor_task:    calib.save(r) / calib.load(out[])
//       (NVS persistence — sensor_task owns calibration data)
//
//  Countdown display:
//    control_task computes remaining seconds from its own tick timer
//    and calls ss.setCountdown(). CalibrationSystem never touches
//    SharedState at all.
// ================================================================

#include <Preferences.h>
#include "SystemTypes.h"
#include "Filters.h"
#include "config.h"

// ── Result codes ─────────────────────────────────────────────────
enum class CalibResultCode : uint8_t {
    CALIB_OK             = 0,
    CALIB_NO_SAMPLES     = 1,   // addSample never called
    CALIB_RANGE_TOO_SMALL = 2,  // |max - min| < CALIB_MIN_RANGE
    CALIB_PHASES_MISSING = 3,   // computeOpenHand or computeCloseHand not done
};

// ── Computed calibration output ───────────────────────────────────
struct CalibrationResult {
    bool      success                    = false;
    bool      open_computed              = false;
    bool      close_computed             = false;
    float     open_raw [NUM_FINGERS]     = {};  // sigma-filtered open mean
    float     close_raw[NUM_FINGERS]     = {};  // sigma-filtered close mean
    FlexCalib calib    [NUM_FINGERS]     = {};  // final min/max per finger
};

// ================================================================
//  CalibrationSystem
// ================================================================
class CalibrationSystem {
public:
    CalibrationSystem() = default;

    // ── Buffer management ─────────────────────────────────────────

    // Reset BOTH open and close sample buffers and result state.
    // Call from control_task at the very start of a calibration run.
    void resetBuffers();

    // Reset ONLY the sample buffer (not the result).
    // Call from control_task between OPEN_HAND and CLOSE_HAND phases.
    void resetSampleBuffer();

    // ── Sample ingestion (called by sensor_task, non-blocking) ────

    // Accept one set of raw ADC readings into the current buffer.
    // raw[] must have NUM_FINGERS elements.
    // Returns false when the buffer is full (caller should stop adding).
    // This function executes in O(1) — no loops, no delays.
    bool addSample(const float raw[NUM_FINGERS]);

    // How many samples are currently in the buffer.
    int sampleCount() const { return _sample_count; }

    // ── Computation (called by control_task after each phase) ─────

    // Apply sigma filter to current buffer, store result as open values.
    // Returns false if no samples were collected.
    // Call once from control_task when OPEN_HAND timer expires.
    bool computeOpenHand();

    // Apply sigma filter to current buffer, store result as close values.
    // Returns false if no samples were collected.
    // Call once from control_task when CLOSE_HAND timer expires.
    bool computeCloseHand();

    // ── Validation (called by control_task after both phases) ─────

    // Validate that (close - open) >= CALIB_MIN_RANGE for every finger.
    // Returns CALIB_OK on success, error code on failure.
    // Call once from control_task after computeCloseHand().
    CalibResultCode validate();

    // ── Result access ─────────────────────────────────────────────

    // Retrieve the final CalibrationResult.
    // Only valid after validate() returns CALIB_OK.
    const CalibrationResult& getResult() const { return _result; }

    // ── NVS persistence (called by sensor_task) ───────────────────

    bool save(const CalibrationResult& result);
    bool load(FlexCalib out_calib[NUM_FINGERS]);
    void clear();

private:
    // ── Sample buffer (instance variable — no stack allocation) ───
    // Sized for one phase at a time. resetSampleBuffer() clears between phases.
    float _buf[NUM_FINGERS][CALIB_SAMPLES_MAX] = {};
    int   _sample_count                         = 0;

    // ── Accumulated result ────────────────────────────────────────
    CalibrationResult _result = {};

    Preferences _prefs;
};