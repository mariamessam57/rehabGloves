#include "Calibration.h"
#include "systemstate/System_State.h" // تضمين ملف الحالة المشتركة لتحديث التايمر
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

    // الدخول الفوري والمباشر لدالة التجميع لمنع أي تعليق خارجي
    _collectSamples(flex_pins, bufs, sample_count, CALIB_DURATION_MS, phase);

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

    // التحقق من صحة المدى الحركي (الفارق بين الفتح والغلق) بعد انتهاء مرحلة الغلق
    if (phase == CalibPhase::CLOSE_HAND)
    {
        for (int f = 0; f < NUM_FINGERS; f++)
        {
            float range = out_calib[f].max_raw - out_calib[f].min_raw;

            // إذا كان الفارق أقل من الحد المسموح تعتبر المعايرة خاطئة
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
    
    vTaskDelay(pdMS_TO_TICKS(50)); 

    if (!startPhase(CalibPhase::OPEN_HAND, flex_pins, out_calib)) {
        Serial.println("[CALIB] OPEN_HAND stage failed.");
        if (phase_cb) phase_cb(CalibPhase::FAILED);
        return false;
    }
    Serial.println("[CALIB] OPEN_HAND stage complete.");

    // ── الانتقال لمرحلة غلق اليد ──
    Serial.println("[CALIB] runCalibration: switching to CLOSE_HAND stage.");
    if (phase_cb) phase_cb(CalibPhase::CLOSE_HAND);
    
    vTaskDelay(pdMS_TO_TICKS(50)); 

    if (!startPhase(CalibPhase::CLOSE_HAND, flex_pins, out_calib)) {
        Serial.println("[CALIB] CLOSE_HAND stage failed.");
        if (phase_cb) phase_cb(CalibPhase::FAILED);
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
    uint32_t duration_ms,
    CalibPhase phase)
{
    SharedState& ss = SharedState::get();
    sample_count = 0;

    uint32_t period_sensor_ms = PERIOD_SENSOR_MS;
    if (period_sensor_ms == 0) period_sensor_ms = 20;
    const uint32_t loops_per_second = 1000 / period_sensor_ms;
    uint32_t loop_counter = 0;

    // 1️⃣ مرحلة الاستعداد الآمنة (2 ثانية) مدمجة داخل الـ Counter لمنع تعليق النواة
    int prepare_seconds = 2;
    Serial.printf("[CALIB] Phase: %s -> Prepare Stage Started.\n", 
                  phase == CalibPhase::OPEN_HAND ? "OPEN_HAND" : "CLOSE_HAND");
    
    ss.setCountdown(prepare_seconds);
    Serial.printf("[CALIB] Prepare Counter: %d s...\n", prepare_seconds);

    while (prepare_seconds > 0) {
        loop_counter++;
        if (loop_counter >= loops_per_second) {
            prepare_seconds--;
            loop_counter = 0;
            if (prepare_seconds > 0) {
                ss.setCountdown(prepare_seconds);
                Serial.printf("[CALIB] Prepare Counter: %d s...\n", prepare_seconds);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(period_sensor_ms));
    }

    // 2️⃣ مرحلة التسجيل الفعلي وجمع العينات (5 ثواني)
    int remaining_seconds = duration_ms / 1000;
    loop_counter = 0;

    Serial.println("[CALIB] Recording started...");
    ss.setCountdown(remaining_seconds);
    Serial.printf("[CALIB] Remaining: %d s...\n", remaining_seconds);

    while (remaining_seconds > 0) {

        // حماية الـ Buffer
        if (sample_count >= CALIB_SAMPLES_MAX) {
            Serial.println("[CALIB] Buffer full! Breaking early.");
            break; 
        }

        // قراءة الـ ADC للمستشعرات
        for (int f = 0; f < NUM_FINGERS; f++) {
            bufs[f][sample_count] = (float)analogRead(pins[f]);
        }
        sample_count++;
        loop_counter++;

        // معالجة تغير الثواني بدقة عينات ثابتة
        if (loop_counter >= loops_per_second) {
            remaining_seconds--;
            loop_counter = 0;
            
            ss.setCountdown(remaining_seconds);
            Serial.printf("[CALIB] Remaining: %d s...\n", remaining_seconds);
        }
        
        vTaskDelay(pdMS_TO_TICKS(period_sensor_ms));
    }
    
    ss.setCountdown(0);
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