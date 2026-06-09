#pragma once
#include <Preferences.h>
#include "SystemTypes.h"
#include "../Filters/Filters.h"
#include "Config.h"

// ================================================================
//  CALIBRATION SYSTEM
//  - Phase 1: open hand  → record min per finger
//  - Phase 2: close hand → record max per finger
//  - Sigma filtering for outlier removal
//  - Persistent storage via Preferences (NVS)
// ================================================================
class CalibrationSystem {
public:
    CalibrationSystem();

    /**
     * Run full calibration sequence.
     * Blocks the calling task (runs inside SensorTask).
     * @param flex_pins ADC pins for 3 fingers
     * @param out_calib output calibration data
     * @param phase_cb  callback to update CalibPhase in SharedState
     * @return true if calibration succeeded
     */
    bool runCalibration(
        const uint8_t flex_pins[NUM_FINGERS],
        FlexCalib out_calib[NUM_FINGERS],
        void (*phase_cb)(CalibPhase)
    );

    // Persist / load from NVS
    bool save(const FlexCalib calib[NUM_FINGERS]);
    bool load(FlexCalib calib[NUM_FINGERS]);
    void clear();

private:
    Preferences _prefs;

    void _collectSamples(
        const uint8_t pins[NUM_FINGERS],
        float bufs[NUM_FINGERS][CALIB_SAMPLES_MAX],
        int& sample_count,
        uint32_t duration_ms
    );
};