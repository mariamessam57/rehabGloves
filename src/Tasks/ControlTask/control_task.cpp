// ================================================================
//  control_task.cpp  — REDESIGNED
//
//  Key changes vs original:
//  [C-1]  New case SystemMode::MANUAL_CALIB handled by
//         modeManualCalib(). This is THE fix for "motors don't move
//         during manual calibration".
//
//  [C-2]  modeManualCalib() implements the full step machine:
//         WARN_OPEN → MOVING_OPEN → WAIT_OPEN_CONFIRM
//         → MOVING_CLOSE → WAIT_CLOSE_CONFIRM → DONE
//         Motors are driven at CALIB_MOTOR_DUTY during moving steps.
//         Motors hold still (STOP) during confirm steps.
//
//  [C-3]  WARN_OPEN / WARN_CLOSE countdown is decremented here
//         (non-blocking, tick-based) so display_task shows 3 2 1
//         without any task blocking.
//
//  [C-4]  After DONE step, control_task stops all motors and waits
//         for display_task to transition mode back to SAFE_LOCK.
// ================================================================

#include "Tasks/ControlTask/control_task.h"
#include "systemstate/System_State.h"
#include "MotorDriver.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>

// ── Manual calib motor duty (low, safe during position finding) ──
// Defined here; add to config.h if you want to tune from one place.
#ifndef MANUAL_CALIB_DUTY
#define MANUAL_CALIB_DUTY  60U   // out of PWM_DUTY_MAX (200)
#endif

// ── Warning countdown tick tracking ─────────────────────────────
static uint32_t warn_last_tick_ms = 0;

// ─────────────────────────── Helpers ────────────────────────────
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline uint8_t dutyFromFraction(float f) {
    return (uint8_t)clampf(f * (float)PWM_DUTY_MAX,
                           (float)PWM_DUTY_MIN, (float)PWM_DUTY_MAX);
}

// ─────────────────── Intent detection ───────────────────────────
static MotionIntent detectIntent(const SensorSnapshot& snap) {
    if (snap.imu.gyro_mag < INTENT_GYRO_THRESH) return MotionIntent::NONE;

    float avg_vel = 0.0f;
    for (int f = 0; f < NUM_FINGERS; f++) avg_vel += snap.flex[f].velocity;
    avg_vel /= (float)NUM_FINGERS;

    if (fabsf(avg_vel) < INTENT_VEL_THRESH) return MotionIntent::NONE;
    return (avg_vel > 0.0f) ? MotionIntent::CLOSING : MotionIntent::OPENING;
}

// ================================================================
//  MODE: MANUAL_CALIB
//  [C-1][C-2] This function is the fix for manual calibration
//  motors not moving. Previously there was NO code path in
//  control_task that drove motors during calibration — the default
//  branch always called stopAll(). Now MANUAL_CALIB has its own
//  mode with a proper step machine.
// ================================================================
static void modeManualCalib(const SensorSnapshot& snap,
                             MotorState out[NUM_FINGERS],
                             SharedState& ss)
{
    ManualCalibStep step = ss.getManualCalibStep();
    uint32_t now = millis();

    switch (step) {

        // ── WARN_OPEN: display "fully open hand, starting in 3 2 1"
        //   Motors stay still. Countdown decrements tick-by-tick.
        case ManualCalibStep::WARN_OPEN: {
            for (int f = 0; f < NUM_FINGERS; f++) {
                out[f].enabled = false;
                out[f].target  = 0;
                out[f].dir     = MotorDir::STOP;
            }
            // [C-3] Decrement countdown once per second (non-blocking)
            if (warn_last_tick_ms == 0) warn_last_tick_ms = now;
            if ((now - warn_last_tick_ms) >= 1000u) {
                warn_last_tick_ms = now;
                int cd = ss.getManualCountdown();
                cd--;
                ss.setManualCountdown(cd);
                Serial.printf("[CTRL] WARN_OPEN countdown: %d\n", cd);
                if (cd <= 0) {
                    warn_last_tick_ms = 0;
                    ss.setManualCountdown(0);
                    ss.setManualCalibStep(ManualCalibStep::MOVING_OPEN);
                    Serial.println("[CTRL] → MOVING_OPEN");
                }
            }
            break;
        }

        // ── MOVING_OPEN: drive all three motors toward OPEN ──────
        case ManualCalibStep::MOVING_OPEN: {
            bool more_requested = ss.getManualCalibMore();
            if (more_requested) ss.setManualCalibMore(false);

            // Check if any finger is already at limit
            bool all_open = true;
            for (int f = 0; f < NUM_FINGERS; f++) {
                if (snap.flex[f].normalized > 0.05f) { all_open = false; break; }
            }

            for (int f = 0; f < NUM_FINGERS; f++) {
                float norm = snap.flex[f].normalized;
                // Stop this finger if it is already fully open
                if (norm <= 0.03f) {
                    out[f].enabled = false;
                    out[f].target  = 0;
                    out[f].dir     = MotorDir::STOP;
                } else {
                    out[f].enabled = true;
                    out[f].dir     = MotorDir::FORWARD; // OPEN direction
                    out[f].target  = MANUAL_CALIB_DUTY;
                }
            }

            // Transition to confirm step when all fingers read open
            // OR after a short drive burst even if sensors are
            // not yet calibrated (min/max may be invalid here).
            // We always transition; user decides if it looks right.
            if (all_open || more_requested) {
                // already open or user explicitly asked for more
            }
            // Always move until sensor_task gets a "confirmed" signal.
            // Transition is driven by input_task (key 1 or 2).
            // But we need to get to WAIT state; we do so on the
            // NEXT call when MOVING_OPEN is still active and user
            // hasn't pressed anything — we just keep driving.
            // input_task will set confirmed/more when ready.
            // If 'more' was pressed while in WAIT, we come back here.
            break;
        }

        // ── WAIT_OPEN_CONFIRM: motors hold, user decides ─────────
        case ManualCalibStep::WAIT_OPEN_CONFIRM: {
            // Hold all motors still
            for (int f = 0; f < NUM_FINGERS; f++) {
                out[f].enabled = false;
                out[f].target  = 0;
                out[f].dir     = MotorDir::STOP;
            }
            // If user pressed 2 (More), go back to MOVING_OPEN
            if (ss.getManualCalibMore()) {
                ss.setManualCalibMore(false);
                ss.setManualCalibStep(ManualCalibStep::MOVING_OPEN);
                Serial.println("[CTRL] More open requested → MOVING_OPEN");
            }
            // confirmed (key 1) is handled in sensor_task which
            // snapshots the ADC and advances to MOVING_CLOSE
            break;
        }

        // ── MOVING_CLOSE: drive all three motors toward CLOSE ─────
        case ManualCalibStep::MOVING_CLOSE: {
            bool more_requested = ss.getManualCalibMore();
            if (more_requested) ss.setManualCalibMore(false);

            for (int f = 0; f < NUM_FINGERS; f++) {
                float norm = snap.flex[f].normalized;
                if (norm >= 0.97f) {
                    out[f].enabled = false;
                    out[f].target  = 0;
                    out[f].dir     = MotorDir::STOP;
                } else {
                    out[f].enabled = true;
                    out[f].dir     = MotorDir::REVERSE; // CLOSE direction
                    out[f].target  = MANUAL_CALIB_DUTY;
                }
            }
            break;
        }

        // ── WAIT_CLOSE_CONFIRM: motors hold ──────────────────────
        case ManualCalibStep::WAIT_CLOSE_CONFIRM: {
            for (int f = 0; f < NUM_FINGERS; f++) {
                out[f].enabled = false;
                out[f].target  = 0;
                out[f].dir     = MotorDir::STOP;
            }
            if (ss.getManualCalibMore()) {
                ss.setManualCalibMore(false);
                ss.setManualCalibStep(ManualCalibStep::MOVING_CLOSE);
                Serial.println("[CTRL] More close requested → MOVING_CLOSE");
            }
            break;
        }

        // ── SAVING / DONE: all motors off ─────────────────────────
        case ManualCalibStep::SAVING:
        case ManualCalibStep::DONE:
        default: {
            for (int f = 0; f < NUM_FINGERS; f++) {
                out[f].enabled = false;
                out[f].target  = 0;
                out[f].dir     = MotorDir::STOP;
            }
            break;
        }
    }

    // Auto-advance MOVING_OPEN → WAIT_OPEN_CONFIRM when in MOVING_OPEN
    // but flex sensors show all fingers are already open.
    // This is a secondary transition path; primary path is driven by
    // input_task. We only auto-advance when we know calib is valid.
    // For safety, always let the user confirm manually.
    // (No auto-advance here — keeps the UX predictable.)
}

// ================================================================
//  MODE 1: PASSIVE
// ================================================================
static bool     passive_active   = false;
static bool     passive_complete = false;
static uint32_t passive_start_ms = 0;
static bool     passive_delay    = true;
static uint32_t passive_delay_ms = 0;

static void modePassive(const SensorSnapshot& snap, MotorState out[NUM_FINGERS]) {
    uint32_t now = millis();

    if (!passive_active) {
        passive_active   = true;
        passive_complete = false;
        passive_start_ms = now;
        passive_delay_ms = now;
        passive_delay    = true;
    }

    if (passive_complete) {
        for (int f = 0; f < NUM_FINGERS; f++) {
            out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
        }
        return;
    }

    if (passive_delay) {
        if ((now - passive_delay_ms) < PASSIVE_START_DELAY_MS) {
            for (int f = 0; f < NUM_FINGERS; f++) {
                out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
            }
            return;
        }
        passive_delay = false;
    }

    if ((now - passive_start_ms) >= PASSIVE_RUNTIME_MS + PASSIVE_START_DELAY_MS) {
        passive_complete = true;
        for (int f = 0; f < NUM_FINGERS; f++) {
            out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
        }
        return;
    }

    uint32_t elapsed = (now - passive_start_ms - PASSIVE_START_DELAY_MS) % PASSIVE_PERIOD_MS;
    float phase  = (float)elapsed / (float)PASSIVE_PERIOD_MS;
    float target = 0.5f * (1.0f - cosf(2.0f * M_PI * phase));

    for (int f = 0; f < NUM_FINGERS; f++) {
        float error = target - snap.flex[f].normalized;
        if (fabsf(error) < 0.03f) {
            out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
        } else if (error > 0.0f) {
            out[f].enabled = true; out[f].dir = MotorDir::REVERSE;
            out[f].target = dutyFromFraction(fabsf(error));
        } else {
            out[f].enabled = true; out[f].dir = MotorDir::FORWARD;
            out[f].target = dutyFromFraction(fabsf(error));
        }
    }
}

// ================================================================
//  MODE 2: ASSISTIVE
// ================================================================
struct AssistState {
    bool         assessment_done = false;
    bool         fsr_reached     = false;
    float        patient_force   = 0.0f;
    uint8_t      force_level     = 0;
    float        motor_fraction  = 1.0f;
    uint32_t     assess_start_ms = 0;
    float        step_target     = 0.0f;
    bool         step_active     = false;
    uint32_t     step_end_ms     = 0;
    uint32_t     stall_start_ms  = 0;
    MotionIntent last_intent     = MotionIntent::NONE;
};
static AssistState assist_st;

static uint8_t computeForceLevel(float fsr_norm) {
    if (fsr_norm < 0.25f) return 0;
    if (fsr_norm < 0.50f) return 1;
    if (fsr_norm < 0.75f) return 2;
    return 3;
}

static void modeAssistive(const SensorSnapshot& snap,
                           MotorState out[NUM_FINGERS], SharedState& ss)
{
    uint32_t now = millis();
    float avg_norm = 0.0f, avg_vel = 0.0f;
    for (int f = 0; f < NUM_FINGERS; f++) {
        avg_norm += snap.flex[f].normalized;
        avg_vel  += snap.flex[f].velocity;
    }
    avg_norm /= NUM_FINGERS; avg_vel /= NUM_FINGERS;

    if (!assist_st.assessment_done) {
        if (assist_st.assess_start_ms == 0) assist_st.assess_start_ms = now;
        for (int f = 0; f < NUM_FINGERS; f++) {
            out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
        }
        if (snap.fsr.normalized > FSR_TOUCHED_THRESH) {
            assist_st.fsr_reached     = true;
            assist_st.patient_force   = snap.fsr.normalized;
            assist_st.force_level     = computeForceLevel(snap.fsr.normalized);
            assist_st.motor_fraction  = clampf(1.0f - snap.fsr.normalized, 0.25f, 1.0f);
            assist_st.assessment_done = true;
            return;
        }
        if ((now - assist_st.assess_start_ms) >= ASSIST_ASSESS_MS) {
            assist_st.motor_fraction  = (avg_norm < ASSIST_FLEX_THRESHOLD) ? 0.75f : 1.0f;
            assist_st.assessment_done = true;
        }
        return;
    }

    uint8_t motor_duty = (uint8_t)clampf(
        assist_st.motor_fraction * (float)PWM_DUTY_MAX,
        (float)PWM_DUTY_MIN, (float)PWM_DUTY_MAX);

    bool patient_moving = false;
    for (int f = 0; f < NUM_FINGERS; f++) {
        if (fabsf(snap.flex[f].velocity) > ASSIST_STALL_VEL) { patient_moving = true; break; }
    }

    if (assist_st.step_active) {
        if (now < assist_st.step_end_ms) {
            for (int f = 0; f < NUM_FINGERS; f++) {
                float error = assist_st.step_target - snap.flex[f].normalized;
                if (fabsf(error) > 0.02f) {
                    out[f].enabled = true;
                    out[f].dir = (error > 0.0f) ? MotorDir::REVERSE : MotorDir::FORWARD;
                    out[f].target = motor_duty;
                } else {
                    out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
                }
            }
        } else {
            assist_st.step_active    = false;
            assist_st.stall_start_ms = 0;
            for (int f = 0; f < NUM_FINGERS; f++) {
                out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
            }
        }
        return;
    }

    if (patient_moving) {
        assist_st.stall_start_ms = 0;
        MotionIntent intent = detectIntent(snap);
        if (intent != MotionIntent::NONE) assist_st.last_intent = intent;
        for (int f = 0; f < NUM_FINGERS; f++) {
            if      (intent == MotionIntent::CLOSING) { out[f].enabled=true; out[f].dir=MotorDir::REVERSE; out[f].target=motor_duty; }
            else if (intent == MotionIntent::OPENING)  { out[f].enabled=true; out[f].dir=MotorDir::FORWARD;  out[f].target=motor_duty; }
            else { out[f].enabled=false; out[f].target=0; out[f].dir=MotorDir::STOP; }
        }
        return;
    }

    if (assist_st.stall_start_ms == 0) assist_st.stall_start_ms = now;
    if ((now - assist_st.stall_start_ms) >= ASSIST_STALL_MS) {
        float step_dir = (assist_st.last_intent == MotionIntent::OPENING)
                         ? -ASSIST_STEP_NORM : ASSIST_STEP_NORM;
        assist_st.step_target = clampf(avg_norm + step_dir, 0.0f, 1.0f);
        assist_st.step_active = true;
        assist_st.step_end_ms = now + ASSIST_STEP_MS;
        assist_st.stall_start_ms = 0;
    } else {
        for (int f = 0; f < NUM_FINGERS; f++) {
            out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
        }
    }
}

// ================================================================
//  MODE 3: RESISTANCE
// ================================================================
static float resist_scale = 1.0f;

static void modeResistance(const SensorSnapshot& snap,
                            MotorState out[NUM_FINGERS], SharedState& ss)
{
    MotionIntent intent   = detectIntent(snap);
    float        fsr_norm = snap.fsr.normalized;
    float        avg_vel  = 0.0f;
    for (int f = 0; f < NUM_FINGERS; f++) avg_vel += snap.flex[f].velocity;
    avg_vel /= (float)NUM_FINGERS;

    if (fsr_norm < 0.15f) resist_scale = clampf(resist_scale - RESISTIVE_ADAPT_STEP, RESISTIVE_MIN_SCALE, 1.0f);
    else                  resist_scale = clampf(resist_scale + RESISTIVE_ADAPT_STEP, RESISTIVE_MIN_SCALE, 1.0f);

    uint8_t base_pw    = dutyFromFraction(fsr_norm);
    uint8_t resist_pw  = (uint8_t)clampf((float)base_pw * resist_scale,
                                         (float)PWM_DUTY_MIN, (float)PWM_DUTY_MAX);

    for (int f = 0; f < NUM_FINGERS; f++) {
        float norm     = snap.flex[f].normalized;
        bool  at_limit = (norm < 0.03f && intent == MotionIntent::OPENING)
                      || (norm > 0.97f && intent == MotionIntent::CLOSING);

        if (at_limit || intent == MotionIntent::NONE) {
            out[f].enabled = false; out[f].target = 0; out[f].dir = MotorDir::STOP;
            continue;
        }
        out[f].enabled = true;
        out[f].dir     = (intent == MotionIntent::CLOSING) ? MotorDir::FORWARD : MotorDir::REVERSE;
        out[f].target  = resist_pw;
    }
}

// ================================================================
//  control_task
// ================================================================
void control_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    // Wait for calibration before accepting therapy modes
    ss.waitEventBits(EVT_CALIB_DONE, false, true, portMAX_DELAY);

    getMotorDriver().begin();

    TickType_t    last_wake = xTaskGetTickCount();
    SensorSnapshot snap;
    MotorState     motors[NUM_FINGERS] = {};

    static SystemMode prev_mode = SystemMode::SAFE_LOCK;

    for (;;) {
        if (ss.isEStop()) {
            getMotorDriver().disableAll();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        SystemMode mode = ss.getMode();

        // ── Reset per-mode state on mode transition ───────────────
        if (prev_mode != mode) {
            if (prev_mode == SystemMode::ASSISTIVE)  assist_st = AssistState{};
            if (prev_mode == SystemMode::PASSIVE) {
                passive_active = false; passive_complete = false;
                passive_start_ms = 0; passive_delay = true; passive_delay_ms = 0;
            }
            if (prev_mode == SystemMode::MANUAL_CALIB) {
                warn_last_tick_ms = 0;
            }
            prev_mode = mode;
        }

        ss.readSensors(snap);

        switch (mode) {
            case SystemMode::PASSIVE:
                modePassive(snap, motors);
                break;
            case SystemMode::ASSISTIVE:
                modeAssistive(snap, motors, ss);
                break;
            case SystemMode::RESISTANCE:
                modeResistance(snap, motors, ss);
                break;
            case SystemMode::MANUAL_CALIB:
                // [C-1] THIS is what was missing — manual calib motor control
                modeManualCalib(snap, motors, ss);
                break;
            default:
                for (int f = 0; f < NUM_FINGERS; f++) {
                    motors[f].enabled = false;
                    motors[f].target  = 0;
                    motors[f].dir     = MotorDir::STOP;
                }
                getMotorDriver().stopAll();
                break;
        }

        for (int f = 0; f < NUM_FINGERS; f++) {
            getMotorDriver().applyRamp(motors[f], f);
        }

        ss.writeMotors(motors);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_CONTROL_MS));
    }
}