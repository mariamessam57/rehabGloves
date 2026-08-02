#pragma once

#include <Arduino.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "SystemTypes.h"
#include "../Filters/Filters.h"
#include "Config.h"

class CalibrationSystem {
public:
    CalibrationSystem();

    bool runCalibration(
        const uint8_t flex_pins[NUM_FINGERS],
        FlexCalib out_calib[NUM_FINGERS],
        void (*phase_cb)(CalibPhase)
    );

    bool save(const FlexCalib calib[NUM_FINGERS]);
    bool load(FlexCalib calib[NUM_FINGERS]);
    void clear();

private:
    static constexpr const char* _namespace = PREFS_NAMESPACE;

    void _collectSamples(
        const uint8_t pins[NUM_FINGERS],
        float bufs[NUM_FINGERS][CALIB_SAMPLES_MAX],
        int& sample_count,
        uint32_t duration_ms
    );

    bool _openPrefs(nvs_handle_t& handle, nvs_open_mode_t mode) const;
};