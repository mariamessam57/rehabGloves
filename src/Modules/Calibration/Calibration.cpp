#include "Calibration.h"
#include "systemstate/System_State.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

// ================================================================
// Constructor
// ================================================================
CalibrationSystem::CalibrationSystem() {}

// ================================================================
// DEBUG helper
// ================================================================
static void printFlex(const char *tag,
                      float bufs[NUM_FINGERS][CALIB_SAMPLES_MAX],
                      int sample_count)
{
    if (sample_count <= 0)
    {
        Serial.printf("\n[%s] NO SAMPLES COLLECTED!\n", tag);
        return;
    }

    Serial.printf("\n[%s] RAW FLEX VALUES (last sample)\n", tag);

    int master = MASTER_FLEX_IDX;
    float v = bufs[master][sample_count - 1];
    Serial.printf("F%d = %.2f\n", master, v);
    Serial.println("(using master flex sensor only)\n");
}

// ================================================================
// MAIN CALIBRATION FLOW
// ================================================================
bool CalibrationSystem::runCalibration(
    const uint8_t flex_pins[NUM_FINGERS],
    FlexCalib out_calib[NUM_FINGERS],
    void (*phase_cb)(CalibPhase))
{
    Serial.println("🔥 ENTER RUN CALIBRATION");

    static float bufs[NUM_FINGERS][CALIB_SAMPLES_MAX];
    int sample_count = 0;

    // ================= OPEN HAND =================
    if (phase_cb)
        phase_cb(CalibPhase::OPEN_HAND);
    Serial.println("[CALIB] Phase 1: Open hand...");

    _collectSamples(flex_pins, bufs, sample_count, CALIB_DURATION_MS);
    printFlex("OPEN_HAND", bufs, sample_count);

    int master = MASTER_FLEX_IDX;
    out_calib[master].min_raw =
        SigmaFilter::compute(bufs[master], sample_count, CALIB_SIGMA);

    for (int f = 0; f < NUM_FINGERS; f++)
    {
        out_calib[f].min_raw = out_calib[master].min_raw;
    }

    // ================= CLOSE HAND =================
    if (phase_cb)
        phase_cb(CalibPhase::CLOSE_HAND);
    Serial.println("[CALIB] Phase 2: Close hand...");

    _collectSamples(flex_pins, bufs, sample_count, CALIB_DURATION_MS);
    printFlex("CLOSE_HAND", bufs, sample_count);

    out_calib[master].max_raw =
        SigmaFilter::compute(bufs[master], sample_count, CALIB_SIGMA);

    if (out_calib[master].max_raw < out_calib[master].min_raw)
    {
        float tmp = out_calib[master].max_raw;
        out_calib[master].max_raw = out_calib[master].min_raw;
        out_calib[master].min_raw = tmp;
    }

    float range = out_calib[master].max_raw - out_calib[master].min_raw;

    Serial.printf("[CALIB] F%d range=%.2f\n", master, range);

    if (range < (float)CALIB_MIN_RANGE)
    {
        Serial.printf("[CALIB] FAILED F%d range=%.2f\n", master, range);
        if (phase_cb)
            phase_cb(CalibPhase::FAILED);
        SharedState::get().setCalibComplete(false);
        return false;
    }

    for (int f = 0; f < NUM_FINGERS; f++)
    {
        out_calib[f].max_raw = out_calib[master].max_raw;
        out_calib[f].min_raw = out_calib[master].min_raw;
        out_calib[f].valid = true;
        Serial.printf("[CALIB] F%d min=%.0f max=%.0f\n",
                      f,
                      out_calib[f].min_raw,
                      out_calib[f].max_raw);
    }

    if (phase_cb)
        phase_cb(CalibPhase::DONE);

    save(out_calib);

    // ============================================================
    // FIX 1: Unlock system after successful calibration
    // ============================================================
    SharedState::get().setCalibComplete(true);

    Serial.println("[CALIB] DONE — saved to NVS + SYSTEM UNLOCKED");

    return true;
}

// ================================================================
// SAMPLE COLLECTION
// ================================================================
void CalibrationSystem::_collectSamples(
    const uint8_t pins[NUM_FINGERS],
    float bufs[NUM_FINGERS][CALIB_SAMPLES_MAX],
    int &sample_count,
    uint32_t duration_ms)
{
    Serial.println("[CALIB] CollectSamples START");

    sample_count = 0;
    uint32_t start = millis();

    memset(bufs, 0, sizeof(float) * NUM_FINGERS * CALIB_SAMPLES_MAX);

    float smooth[NUM_FINGERS] = {0};

    while ((millis() - start) < duration_ms)
    {

        if (sample_count >= CALIB_SAMPLES_MAX)
            break;

        for (int f = 0; f < NUM_FINGERS; f++)
        {

            analogRead(pins[f]);
            delayMicroseconds(50);

            int sum = 0;

            for (int i = 0; i < 5; i++)
            {
                sum += analogRead(pins[f]);
                delayMicroseconds(300);
            }

            float raw = sum / 5.0f;
            raw = constrain(raw, 0, 4095);

            smooth[f] = 0.75f * smooth[f] + 0.25f * raw;

            bufs[f][sample_count] = smooth[f];
        }

        sample_count++;

        vTaskDelay(pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }

    Serial.printf("[CALIB] CollectSamples END (%d samples)\n", sample_count);
}

// ================================================================
// SAVE TO NVS
// ================================================================
bool CalibrationSystem::save(const FlexCalib calib[NUM_FINGERS])
{
    nvs_handle_t handle;

    if (!_openPrefs(handle, NVS_READWRITE))
        return false;

    for (int f = 0; f < NUM_FINGERS; f++)
    {

        char k1[16], k2[16];
        snprintf(k1, sizeof(k1), "f%d_min", f);
        snprintf(k2, sizeof(k2), "f%d_max", f);

        nvs_set_blob(handle, k1, &calib[f].min_raw, sizeof(float));
        nvs_set_blob(handle, k2, &calib[f].max_raw, sizeof(float));
    }

    nvs_set_u8(handle, "valid", 1);

    esp_err_t err = nvs_commit(handle);
    nvs_close(handle);

    return (err == ESP_OK);
}

// ================================================================
// LOAD FROM NVS
// ================================================================
bool CalibrationSystem::load(FlexCalib calib[NUM_FINGERS])
{
    nvs_handle_t handle;

    if (!_openPrefs(handle, NVS_READONLY))
        return false;

    uint8_t valid_flag = 0;

    if (nvs_get_u8(handle, "valid", &valid_flag) != ESP_OK || valid_flag == 0)
    {
        nvs_close(handle);
        return false;
    }

    for (int f = 0; f < NUM_FINGERS; f++)
    {

        char k1[16], k2[16];
        snprintf(k1, sizeof(k1), "f%d_min", f);
        snprintf(k2, sizeof(k2), "f%d_max", f);

        size_t sz = sizeof(float);

        calib[f].min_raw = 0.0f;
        calib[f].max_raw = 4095.0f;

        nvs_get_blob(handle, k1, &calib[f].min_raw, &sz);
        nvs_get_blob(handle, k2, &calib[f].max_raw, &sz);

        calib[f].valid = true;
    }

    nvs_close(handle);

    Serial.println("[NVS] Calibration loaded OK");
    return true;
}

// ================================================================
// CLEAR NVS
// ================================================================
void CalibrationSystem::clear()
{
    nvs_handle_t handle;

    if (!_openPrefs(handle, NVS_READWRITE))
        return;

    nvs_erase_all(handle);
    nvs_commit(handle);
    nvs_close(handle);

    Serial.println("[NVS] Calibration CLEARED");

    // ============================================================
    // FIX 2: Lock system until recalibration
    // ============================================================
    SharedState::get().setCalibComplete(false);
}

// ================================================================
// OPEN NVS SAFE
// ================================================================
bool CalibrationSystem::_openPrefs(
    nvs_handle_t &handle,
    nvs_open_mode_t mode) const
{
    esp_err_t init_err = nvs_flash_init();

    if (init_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        init_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        init_err = nvs_flash_init();
    }

    if (init_err != ESP_OK)
        return false;

    return (nvs_open(_namespace, mode, &handle) == ESP_OK);
}