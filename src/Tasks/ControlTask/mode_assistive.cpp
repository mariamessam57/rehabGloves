#include "Tasks/ControlTask/control_modes.h"
#include "systemstate/System_State.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

// ================================================================
// CONFIG
// ================================================================
static constexpr uint32_t DRIVE_MS      = 4000U;
static constexpr uint32_t PAUSE_MS      = 3000U;
static constexpr uint32_t FSR_WINDOW_MS = 10000U;
static constexpr uint32_t HOLD_STABLE_MS = 3000U;

static constexpr float FLEX_LOW      = 0.10f;
static constexpr float FLEX_HIGH     = 0.90f;
// FIX 1: أكبر threshold للـ HOLD حتى ما يعتبرش حركة عادية motion
static constexpr float FLEX_MOVE_TH  = 0.20f;   // للـ PAUSE
static constexpr float HOLD_STILL_TH = 0.05f;   // للـ HOLD — أصغر بكتير

// ================================================================
// INTERNAL STATE
// ================================================================
enum class AssistPhase : uint8_t {
    FSR_WINDOW,
    DRIVE,
    PAUSE,
    HOLD
};

struct AssistState {
    AssistPhase phase            = AssistPhase::FSR_WINDOW;
    uint32_t    t_start          = 0;
    float       assist_duty      = 0.0f;
    float       fsr_window_start = 0.0f;
    bool        fsr_locked       = false;
    float       flex_start       = 0.0f;   // snapshot عند بداية DRIVE
    float       hold_flex_ref    = 0.0f;   // FIX 1: snapshot عند دخول HOLD
    MotorDir    dir              = MotorDir::FORWARD;
    bool        motion_detected  = false;
    uint32_t    motion_stop_time = 0;
};

static AssistState ctrl;

// ================================================================
// HELPERS
// ================================================================
static inline bool inFSRWindow() {
    return (millis() - (uint32_t)ctrl.fsr_window_start) < FSR_WINDOW_MS;
}

// FIX 2: flexDirection بتشوف الـ norm الحالي مش بس اللي في البداية
// لو الإيد في المنتصف → تكمل في نفس الاتجاه اللي كانت فيه
static inline MotorDir flexDirection(float flex_norm) {
    if (flex_norm <= FLEX_LOW)  return MotorDir::FORWARD;   // إيد مفتوحة → إغلاق
    if (flex_norm >= FLEX_HIGH) return MotorDir::REVERSE;   // إيد مغلقة → فتح
    // في المنتصف: لو FORWARD وصلت تقريباً للنهاية → اعكس
    // لو لسه في الأول → كمّل FORWARD
    return ctrl.dir;
}

// detectMotion للـ PAUSE: مقارنة بـ flex_start
static inline bool detectMotionPause(const SensorSnapshot& snap) {
    if (fabsf(snap.flex[MASTER_FLEX_IDX].normalized - ctrl.flex_start) > FLEX_MOVE_TH)
        return true;
    return snap.imu.gyro_mag > INTENT_GYRO_THRESH;
}

// FIX 1: detectMotion للـ HOLD: مقارنة بـ hold_flex_ref (snapshot لحظة دخول HOLD)
static inline bool detectMotionHold(const SensorSnapshot& snap) {
    if (fabsf(snap.flex[MASTER_FLEX_IDX].normalized - ctrl.hold_flex_ref) > HOLD_STILL_TH)
        return true;
    return snap.imu.gyro_mag > INTENT_GYRO_THRESH;
}

static void applyToAllFingers(MotorState out[NUM_FINGERS], MotorDir dir, float duty) {
    uint8_t pwm = dutyFromFraction(duty);
    for (int f = 0; f < NUM_FINGERS; f++) {
        out[f].enabled    = true;
        out[f].dir        = dir;
        out[f].target     = pwm;
        out[f].target_pwm = pwm;
    }
}

static void stopAllFingers(MotorState out[NUM_FINGERS]) {
    for (int f = 0; f < NUM_FINGERS; f++) {
        out[f].enabled    = false;
        out[f].dir        = MotorDir::STOP;
        out[f].target     = 0;
        out[f].target_pwm = 0;
    }
}

// ================================================================
// SERIAL DEBUG  (throttled 200 ms)
// ================================================================
static void printAssistDebug(const SensorSnapshot& snap,
                              const MotorState out[NUM_FINGERS],
                              bool motion)
{
    static uint32_t last_print = 0;
    uint32_t now = millis();
    if (now - last_print < 200) return;
    last_print = now;

    const char* phaseStr = "UNKNOWN";
    switch (ctrl.phase) {
        case AssistPhase::FSR_WINDOW: phaseStr = "FSR_WINDOW"; break;
        case AssistPhase::DRIVE:      phaseStr = "DRIVE";      break;
        case AssistPhase::PAUSE:      phaseStr = "PAUSE";      break;
        case AssistPhase::HOLD:       phaseStr = "HOLD";       break;
    }
    const char* dirStr[] = {"STOP", "FWD", "REV"};

    uint32_t elapsed = now - ctrl.t_start;

    Serial.println(F("=== ASSISTIVE ==="));

    // phase / timing
    Serial.print(F("  phase="));       Serial.print(phaseStr);
    Serial.print(F("  elapsed_ms="));  Serial.print(elapsed);
    Serial.print(F("  fsr_locked="));  Serial.print(ctrl.fsr_locked ? "Y" : "N");
    Serial.print(F("  motion="));      Serial.println(motion ? "YES" : "NO");

    // duty / direction
    Serial.print(F("  assist_duty=")); Serial.print(ctrl.assist_duty, 3);
    Serial.print(F("  ctrl_dir="));    Serial.println(dirStr[(uint8_t)ctrl.dir]);

    // flex reference values
    Serial.print(F("  flex_start="));    Serial.print(ctrl.flex_start, 3);
    Serial.print(F("  hold_flex_ref=")); Serial.println(ctrl.hold_flex_ref, 3);

    // IMU
    Serial.print(F("  IMU  gyro=["));
    Serial.print(snap.imu.gyro[0], 2); Serial.print(F(", "));
    Serial.print(snap.imu.gyro[1], 2); Serial.print(F(", "));
    Serial.print(snap.imu.gyro[2], 2); Serial.print(F("]"));
    Serial.print(F("  mag="));   Serial.print(snap.imu.gyro_mag, 2);
    Serial.print(F("  pitch=")); Serial.print(snap.imu.pitch, 2);
    Serial.print(F("  roll="));  Serial.println(snap.imu.roll, 2);

    // FSR
    Serial.print(F("  FSR  raw="));  Serial.print(snap.fsr.raw, 1);
    Serial.print(F("  filt="));      Serial.print(snap.fsr.filtered, 3);
    Serial.print(F("  norm="));      Serial.println(snap.fsr.normalized, 3);

    // Flex (كل الـ 3)
    for (int f = 0; f < NUM_FINGERS; f++) {
        Serial.print(F("  F")); Serial.print(f);
        if (f == MASTER_FLEX_IDX) Serial.print(F("*"));
        Serial.print(F("  raw="));  Serial.print(snap.flex[f].raw, 1);
        Serial.print(F("  filt=")); Serial.print(snap.flex[f].filtered, 3);
        Serial.print(F("  norm=")); Serial.print(snap.flex[f].normalized, 3);
        Serial.print(F("  ang="));  Serial.print(snap.flex[f].angle, 1);
        Serial.print(F("  vel="));  Serial.print(snap.flex[f].velocity, 4);
        Serial.print(F("  en="));   Serial.print(out[f].enabled ? "Y" : "N");
        Serial.print(F("  dir="));  Serial.print(dirStr[(uint8_t)out[f].dir]);
        Serial.print(F("  pwm="));  Serial.println(out[f].target);
    }

    // phase-specific
    if (ctrl.phase == AssistPhase::FSR_WINDOW) {
        uint32_t rem = FSR_WINDOW_MS > elapsed ? FSR_WINDOW_MS - elapsed : 0;
        Serial.print(F("  [FSR_WINDOW] remaining_ms=")); Serial.println(rem);
    }
    if (ctrl.phase == AssistPhase::DRIVE) {
        uint32_t rem = DRIVE_MS > elapsed ? DRIVE_MS - elapsed : 0;
        Serial.print(F("  [DRIVE] remaining_ms=")); Serial.println(rem);
        // FIX 2 debug: وضّح ليه الاتجاه كده
        float cur_norm = snap.flex[MASTER_FLEX_IDX].normalized;
        Serial.print(F("  [DRIVE] cur_norm="));   Serial.print(cur_norm, 3);
        Serial.print(F("  flex_start="));          Serial.println(ctrl.flex_start, 3);
    }
    if (ctrl.phase == AssistPhase::PAUSE) {
        uint32_t rem = PAUSE_MS > elapsed ? PAUSE_MS - elapsed : 0;
        Serial.print(F("  [PAUSE] remaining_ms=")); Serial.println(rem);
    }
    if (ctrl.phase == AssistPhase::HOLD) {
        float delta = fabsf(snap.flex[MASTER_FLEX_IDX].normalized - ctrl.hold_flex_ref);
        Serial.print(F("  [HOLD] flex_delta="));  Serial.print(delta, 4);
        Serial.print(F("  still_th="));            Serial.print(HOLD_STILL_TH, 3);
        if (ctrl.motion_stop_time != 0) {
            uint32_t stable = now - ctrl.motion_stop_time;
            Serial.print(F("  stable_ms="));  Serial.print(stable);
            Serial.print(F("  need_ms="));    Serial.print(HOLD_STABLE_MS);
        }
        Serial.println();
    }
}

// ================================================================
// PUBLIC API
// ================================================================

void resetAssistState() {
    ctrl = AssistState{};
}

void modeAssistive(const SensorSnapshot& snap, MotorState out[NUM_FINGERS], SharedState& ss) {
    uint32_t now = millis();

    // ── 1) FSR WINDOW ──────────────────────────────────────────
    if (ctrl.phase == AssistPhase::FSR_WINDOW) {
        if (ctrl.t_start == 0) {
            ctrl.t_start          = now;
            ctrl.fsr_window_start = (float)now;
        }

        ctrl.assist_duty = inFSRWindow() ? snap.fsr.normalized : ctrl.assist_duty;
        if (!inFSRWindow()) ctrl.fsr_locked = true;

        stopAllFingers(out);

        if (now - ctrl.t_start > FSR_WINDOW_MS) {
            float cur = snap.flex[MASTER_FLEX_IDX].normalized;
            ctrl.phase      = AssistPhase::DRIVE;
            ctrl.t_start    = now;
            ctrl.flex_start = cur;
            // FIX 2: لو مفيش duty من FSR استخدم الـ default
            if (ctrl.assist_duty < 0.05f) ctrl.assist_duty = ASSISTIVE_DUTY_PCT;
            // اتجاه بناءً على وضع الإيد الحالي
            ctrl.dir = flexDirection(cur);
            // لو في المنتصف تماماً → ابدأ بـ FORWARD (إغلاق)
            if (ctrl.dir == ctrl.dir && cur > FLEX_LOW && cur < FLEX_HIGH)
                ctrl.dir = MotorDir::FORWARD;
        }

        printAssistDebug(snap, out, false);
        return;
    }

    // ── 2) DRIVE ───────────────────────────────────────────────
    if (ctrl.phase == AssistPhase::DRIVE) {
        float cur = snap.flex[MASTER_FLEX_IDX].normalized;

        // FIX 2: لو وصلنا للنهاية أثناء الـ DRIVE → اعكس الاتجاه فوراً
        if (ctrl.dir == MotorDir::FORWARD && cur >= FLEX_HIGH) {
            ctrl.dir = MotorDir::REVERSE;
            Serial.println(F("[ASSIST] Hit FLEX_HIGH → switching to REVERSE"));
        } else if (ctrl.dir == MotorDir::REVERSE && cur <= FLEX_LOW) {
            ctrl.dir = MotorDir::FORWARD;
            Serial.println(F("[ASSIST] Hit FLEX_LOW → switching to FORWARD"));
        }

        applyToAllFingers(out, ctrl.dir, ctrl.assist_duty);

        if (now - ctrl.t_start >= DRIVE_MS) {
            stopAllFingers(out);
            ctrl.phase           = AssistPhase::PAUSE;
            ctrl.t_start         = now;
            ctrl.flex_start      = cur;   // snapshot للـ PAUSE motion detection
            ctrl.motion_detected = false;
        }

        printAssistDebug(snap, out, false);
        return;
    }

    // ── 3) PAUSE ───────────────────────────────────────────────
    if (ctrl.phase == AssistPhase::PAUSE) {
        stopAllFingers(out);
        bool motion = detectMotionPause(snap);

        if (motion) {
            ctrl.motion_detected  = true;
            ctrl.phase            = AssistPhase::HOLD;
            ctrl.hold_flex_ref    = snap.flex[MASTER_FLEX_IDX].normalized; // FIX 1
            ctrl.motion_stop_time = 0;
            printAssistDebug(snap, out, motion);
            return;
        }

        if (now - ctrl.t_start >= PAUSE_MS) {
            float cur = snap.flex[MASTER_FLEX_IDX].normalized;
            ctrl.phase      = AssistPhase::DRIVE;
            ctrl.t_start    = now;
            ctrl.flex_start = cur;
            // FIX 2: عكس الاتجاه بعد كل pause
            ctrl.dir = (ctrl.dir == MotorDir::FORWARD) ? MotorDir::REVERSE : MotorDir::FORWARD;
            Serial.printf("[ASSIST] PAUSE done → DRIVE dir=%s\n",
                          ctrl.dir == MotorDir::FORWARD ? "FWD" : "REV");
        }

        printAssistDebug(snap, out, motion);
        return;
    }

    // ── 4) HOLD ────────────────────────────────────────────────
    if (ctrl.phase == AssistPhase::HOLD) {
        stopAllFingers(out);

        // FIX 1: استخدم hold_flex_ref مش flex_start
        bool motion = detectMotionHold(snap);

        if (motion) {
            // لسه بيتحرك → رجّع الـ stable timer
            ctrl.motion_stop_time = 0;
            // حدّث الـ ref عشان نتابع الحركة
            ctrl.hold_flex_ref = snap.flex[MASTER_FLEX_IDX].normalized;
            printAssistDebug(snap, out, motion);
            return;
        }

        // مش بيتحرك → ابدأ العد
        if (ctrl.motion_stop_time == 0)
            ctrl.motion_stop_time = now;

        if (now - ctrl.motion_stop_time >= HOLD_STABLE_MS) {
            float cur = snap.flex[MASTER_FLEX_IDX].normalized;
            ctrl.phase            = AssistPhase::DRIVE;
            ctrl.t_start          = now;
            ctrl.flex_start       = cur;
            ctrl.motion_stop_time = 0;
            // FIX 2: بعد HOLD كمّل في الاتجاه العكسي
            ctrl.dir = (ctrl.dir == MotorDir::FORWARD) ? MotorDir::REVERSE : MotorDir::FORWARD;
            Serial.printf("[ASSIST] HOLD done → DRIVE dir=%s\n",
                          ctrl.dir == MotorDir::FORWARD ? "FWD" : "REV");
        }

        printAssistDebug(snap, out, motion);
        return;
    }
}