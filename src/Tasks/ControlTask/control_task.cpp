// ================================================================
//  control_task.cpp  — SOLE FSM AUTHORITY
//
//  Architectural contract:
//
//  ✓ The ONLY task that calls setMode(), setCalibPhase(),
//    setManualCalibStep(), setCalibComplete(), and all other
//    state-mutating SharedState methods.
//  ✓ The ONLY task that consumes EventGroup bits (read + clear).
//  ✓ All FSM transitions are implemented here and nowhere else.
//  ✓ All timing uses xTaskGetTickCount() — never millis().
//  ✓ Motor output is a strict pure function of ManualCalibStep.
//  ✓ Runtime FSM integrity validation on every tick.
//  ✓ Full calibration state cleanup is done HERE after DONE+2s.
//
//  Motor determinism table (ManualCalibStep → motor output):
//    IDLE / WARN_OPEN          → OFF
//    MOVING_OPEN               → FORWARD @ MANUAL_CALIB_DUTY
//    WAIT_OPEN_CONFIRM         → OFF
//    MOVING_CLOSE              → REVERSE @ MANUAL_CALIB_DUTY
//    WAIT_CLOSE_CONFIRM        → OFF
//    SAVING / DONE             → OFF
//
//  Timing constants:
//    MANUAL_WARN_COUNTDOWN  = 3 seconds  (countdown in WARN_OPEN)
//    MANUAL_MOVE_PULSE_MS   = 800 ms     (motor run time per pulse)
//    CALIB_DONE_DISPLAY_MS  = 2000 ms    (completion screen hold)
//
//  All timers: (xTaskGetTickCount() - entry_tick) >= pdMS_TO_TICKS(N)
//  Wrap-safe because TickType_t subtraction is unsigned modular.
// ================================================================

#include "Tasks/ControlTask/control_task.h"
#include "systemstate/System_State.h"
#include "FSM/fsm_events.h"
#include "MotorDriver.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>

// ── Tuning ───────────────────────────────────────────────────────
#ifndef MANUAL_MOVE_PULSE_MS
#define MANUAL_MOVE_PULSE_MS   800u
#endif
#ifndef MANUAL_CALIB_DUTY
#define MANUAL_CALIB_DUTY      60u
#endif
#ifndef MANUAL_WARN_COUNTDOWN
#define MANUAL_WARN_COUNTDOWN  3
#endif
#ifndef CALIB_DONE_DISPLAY_MS
#define CALIB_DONE_DISPLAY_MS  2000u
#endif

// ── ADC pins — same order as sensor_task ─────────────────────────
static const uint8_t FLEX_ADC_PINS[NUM_FINGERS] = {
    PIN_FLEX_0, PIN_FLEX_1, PIN_FLEX_2
};

// ================================================================
//  Atomic event consumption helper
//  Returns true if `bit` was set, and clears it atomically.
//  Non-blocking (timeout = 0). Called only from control_task.
// ================================================================
static inline bool consume(SharedState& ss, EventBits_t bit) {
    EventBits_t got = ss.waitEventBits(bit,
        /*clearOnExit=*/true,
        /*waitAll=*/false,
        /*timeout=*/0);
    return (got & bit) != 0;
}

// ================================================================
//  Motor helpers — strict, no sensor influence
// ================================================================
static void motorsAll(MotorState out[NUM_FINGERS],
                      MotorDir dir, uint8_t duty) {
    for (int f = 0; f < NUM_FINGERS; f++) {
        out[f].enabled = true;
        out[f].dir     = dir;
        out[f].target  = duty;
    }
}

static void motorsOff(MotorState out[NUM_FINGERS]) {
    for (int f = 0; f < NUM_FINGERS; f++) {
        out[f].enabled = false;
        out[f].target  = 0;
        out[f].dir     = MotorDir::STOP;
    }
}

// ── Raw ADC snapshot (called only at confirmation points) ─────────
static void snapshotADC(float out[NUM_FINGERS]) {
    for (int f = 0; f < NUM_FINGERS; f++) {
        out[f] = (float)analogRead(FLEX_ADC_PINS[f]);
    }
    Serial.printf("[CTRL] ADC snap: F0=%.0f F1=%.0f F2=%.0f\n",
                  out[0], out[1], out[2]);
}

// ================================================================
//  FSM runtime integrity check
//  Call once per control_task tick BEFORE any FSM logic.
//  Triggers ESTOP if mode is outside known valid range.
// ================================================================
static void fsm_integrity_check(SharedState& ss, SystemMode mode) {
    // SystemMode is uint8_t; valid range is 0..ESTOP(6)
    const uint8_t raw = static_cast<uint8_t>(mode);
    const uint8_t max_valid = static_cast<uint8_t>(SystemMode::ESTOP);
    if (raw > max_valid) {
        ss.triggerEStop("FSM_CORRUPT");
        Serial.printf("[CTRL] FSM integrity FAIL: mode=%u\n", raw);
    }
}

// ================================================================
//  Full calibration state reset
//  Called by control_task when transitioning back to SAFE_LOCK
//  after calibration completes. ALL calib fields are cleared.
//  [V-12..V-15 fix] This is the ONLY place these are reset.
// ================================================================
static void reset_calib_state(SharedState& ss) {
    ss.setCalibPhase(CalibPhase::IDLE);
    ss.setManualCalibStep(ManualCalibStep::IDLE);
    ss.setManualCountdown(0);
    ss.setCalibDoneTs(0);
    ss.setCalibInProgress(false);
    ss.clearRecalibrationRequest();
    ss.setManualSaveDone(false);
    // NOTE: setCalibComplete(true) is NOT reset here — calibration
    // data remains valid until the user explicitly recalibrates.
    Serial.println("[CTRL] Calib state reset.");
}

// ================================================================
//  modeManualCalib — FSM for manual calibration
//
//  Sole owner of all ManualCalibStep transitions.
//  Runs inside control_task every 20 ms.
//
//  State entry detection: prev_step tracks the last known step.
//  When step != prev_step, entry actions execute and entry_tick
//  is recorded for timer-based transitions.
//
//  [R-1] Motor output = pure function of step. No sensor data.
//  [R-2] Timing uses xTaskGetTickCount(). Never millis().
//  [R-3] MOVING states: motors ON, input events ignored.
//  [R-4] WAIT states: motors OFF, EVT_BTN_CONFIRM/MORE consumed.
//  [R-5] ADC snapshot taken at confirmation instant only.
//  [R-6] DONE: emits EVT_CALIB_DONE, holds 2s, then resets FSM.
// ================================================================
static void modeManualCalib(MotorState out[NUM_FINGERS], SharedState& ss) {

    static ManualCalibStep prev_step  = ManualCalibStep::IDLE;
    static TickType_t      entry_tick = 0;  // tick when current step was entered
    static TickType_t      sec_tick   = 0;  // for per-second countdown decrements

    const ManualCalibStep step     = ss.getManualCalibStep();
    const TickType_t      now_tick = xTaskGetTickCount();

    // ── State entry detection ────────────────────────────────────
    const bool entered = (step != prev_step);
    if (entered) {
        entry_tick = now_tick;
        sec_tick   = now_tick;
        prev_step  = step;
        Serial.printf("[CTRL] ManualCalib step → %d\n", (int)step);
    }

    switch (step) {

        // ── WARN_OPEN ─────────────────────────────────────────────
        // Entry action: countdown is already set to 3 by FSM entry
        //               (set in CALIBRATING/IDLE → MANUAL_CALIB transition)
        // Each second: decrement countdown.
        // At 0: transition to MOVING_OPEN.
        // [R-1] Motors OFF.
        // [R-2] Timer uses RTOS ticks.
        // [R-3] Input ignored.
        case ManualCalibStep::WARN_OPEN: {
            motorsOff(out);  // [R-1]

            // Decrement once per second [R-2]
            if ((now_tick - sec_tick) >= pdMS_TO_TICKS(1000u)) {
                sec_tick = now_tick;  // reset for next second

                int cd = ss.getManualCountdown() - 1;
                if (cd < 0) cd = 0;
                ss.setManualCountdown(cd);
                Serial.printf("[CTRL] WARN_OPEN: %d\n", cd);

                if (cd == 0) {
                    // Transition: countdown complete → MOVING_OPEN
                    ss.setManualCalibStep(ManualCalibStep::MOVING_OPEN);
                }
            }
            break;
        }

        // ── MOVING_OPEN ───────────────────────────────────────────
        // Entry action: none (motors start immediately below)
        // [R-1] Motors FORWARD @ MANUAL_CALIB_DUTY
        // [R-2] Pulse timer: MANUAL_MOVE_PULSE_MS from entry_tick
        // [R-3] Input events NOT consumed here (motors moving)
        case ManualCalibStep::MOVING_OPEN: {
            motorsAll(out, MotorDir::FORWARD, MANUAL_CALIB_DUTY);  // [R-1]

            // Transition: pulse timer expired → WAIT_OPEN_CONFIRM [R-2]
            if ((now_tick - entry_tick) >= pdMS_TO_TICKS(MANUAL_MOVE_PULSE_MS)) {
                ss.setManualCalibStep(ManualCalibStep::WAIT_OPEN_CONFIRM);
            }
            break;
        }

        // ── WAIT_OPEN_CONFIRM ─────────────────────────────────────
        // [R-1] Motors OFF
        // [R-4] Input consumed: CONFIRM → advance, MORE → repeat pulse
        // [R-5] ADC snapshot taken on CONFIRM
        case ManualCalibStep::WAIT_OPEN_CONFIRM: {
            motorsOff(out);  // [R-1]

            if (consume(ss, EVT_BTN_CONFIRM)) {  // [R-4]
                float raw[NUM_FINGERS];
                snapshotADC(raw);                // [R-5]
                ss.setManualOpenRaw(raw);
                ss.setManualCalibStep(ManualCalibStep::MOVING_CLOSE);
                Serial.println("[CTRL] Open confirmed → MOVING_CLOSE");
            } else if (consume(ss, EVT_BTN_MORE)) {
                ss.setManualCalibStep(ManualCalibStep::MOVING_OPEN);
                Serial.println("[CTRL] More open → MOVING_OPEN");
            }
            break;
        }

        // ── MOVING_CLOSE ──────────────────────────────────────────
        // [R-1] Motors REVERSE @ MANUAL_CALIB_DUTY
        // [R-2] Pulse timer from entry_tick
        // [R-3] Input NOT consumed
        case ManualCalibStep::MOVING_CLOSE: {
            motorsAll(out, MotorDir::REVERSE, MANUAL_CALIB_DUTY);  // [R-1]

            if ((now_tick - entry_tick) >= pdMS_TO_TICKS(MANUAL_MOVE_PULSE_MS)) {
                ss.setManualCalibStep(ManualCalibStep::WAIT_CLOSE_CONFIRM);
            }
            break;
        }

        // ── WAIT_CLOSE_CONFIRM ────────────────────────────────────
        // [R-1] Motors OFF
        // [R-4] Input consumed
        // [R-5] ADC snapshot on CONFIRM
        case ManualCalibStep::WAIT_CLOSE_CONFIRM: {
            motorsOff(out);  // [R-1]

            if (consume(ss, EVT_BTN_CONFIRM)) {
                float raw[NUM_FINGERS];
                snapshotADC(raw);                // [R-5]
                ss.setManualCloseRaw(raw);
                ss.setManualCalibStep(ManualCalibStep::SAVING);
                Serial.println("[CTRL] Close confirmed → SAVING");
            } else if (consume(ss, EVT_BTN_MORE)) {
                ss.setManualCalibStep(ManualCalibStep::MOVING_CLOSE);
                Serial.println("[CTRL] More close → MOVING_CLOSE");
            }
            break;
        }

        // ── SAVING ────────────────────────────────────────────────
        // Motors OFF. sensor_task performs NVS write and sets
        // EVT_MANUAL_SAVE_DONE. We poll that event here.
        // [R-6] On save done → DONE entry actions fire.
        case ManualCalibStep::SAVING: {
            motorsOff(out);

            // Consume the save-done handshake from sensor_task
            if (consume(ss, EVT_MANUAL_SAVE_DONE)) {
                // Entry actions for DONE state
                ss.setCalibComplete(true);
                ss.setCalibPhase(CalibPhase::DONE);
                ss.setCalibDoneTs(xTaskGetTickCount());  // [R-2] tick, not millis
                ss.setEventBits(EVT_CALIB_DONE);
                ss.setManualCalibStep(ManualCalibStep::DONE);
                Serial.println("[CTRL] Save done → DONE");
            }
            break;
        }

        // ── DONE ──────────────────────────────────────────────────
        // [R-1] Motors OFF permanently.
        // [R-6] Hold "Completed" display for CALIB_DONE_DISPLAY_MS,
        //       then perform FULL cleanup and transition to SAFE_LOCK.
        //       [V-12..V-15 fix] cleanup is HERE, not in display_task.
        case ManualCalibStep::DONE: {
            motorsOff(out);

            // Read the done timestamp that was set in SAVING→DONE
            const TickType_t done_tick = ss.getCalibDoneTs();
            if (done_tick != 0 &&
                (now_tick - done_tick) >= pdMS_TO_TICKS(CALIB_DONE_DISPLAY_MS))
            {
                // Full FSM cleanup before SAFE_LOCK
                reset_calib_state(ss);
                ss.setMode(SystemMode::SAFE_LOCK);
                Serial.println("[CTRL] Manual calib: → SAFE_LOCK");
            }
            break;
        }

        // ── IDLE (should never occur in MANUAL_CALIB mode) ────────
        default:
            motorsOff(out);
            break;
    }
}

// ================================================================
//  handleSafeLock — FSM for SAFE_LOCK (main menu)
// ================================================================
static bool session_paused = false;
static SystemMode saved_session_mode = SystemMode::SAFE_LOCK;

static void handleSafeLock(MotorState out[NUM_FINGERS], SharedState& ss) {
    motorsOff(out);

    if (session_paused) {
        // Resume path: same key 1 = EVT_SESSION_PAUSE
        if (consume(ss, EVT_SESSION_PAUSE)) {
            session_paused = false;
            ss.clearWarning();
            ss.setMode(saved_session_mode);
            Serial.printf("[CTRL] Session resumed: mode=%d\n",
                          (int)saved_session_mode);
        }
        return;
    }

    // Main menu
    if (consume(ss, EVT_START_PASSIVE)) {
        if (ss.isCalibComplete()) {
            ss.setMode(SystemMode::PASSIVE);
        } else {
            ss.setWarning("NEED CALIB");
        }
    } else if (consume(ss, EVT_START_ASSISTIVE)) {
        if (ss.isCalibComplete()) {
            ss.setMode(SystemMode::ASSISTIVE);
        } else {
            ss.setWarning("NEED CALIB");
        }
    } else if (consume(ss, EVT_START_RESISTANCE)) {
        if (ss.isCalibComplete()) {
            ss.setMode(SystemMode::RESISTANCE);
        } else {
            ss.setWarning("NEED CALIB");
        }
    } else if (consume(ss, EVT_START_CALIB)) {
        // Entry into calibration: full reset before starting
        reset_calib_state(ss);
        ss.setCalibComplete(false);
        ss.requestRecalibration();
        ss.setCalibPhase(CalibPhase::IDLE);
        ss.setMode(SystemMode::CALIBRATING);
        ss.clearWarning();
        Serial.println("[CTRL] → CALIBRATING entry");
    }
}

// ================================================================
//  handleCalibrating — FSM for CALIBRATING mode (entry + auto)
// ================================================================
static void handleCalibrating(MotorState out[NUM_FINGERS], SharedState& ss) {
    motorsOff(out);

    const CalibPhase cp = ss.getCalibPhase();

    if (cp == CalibPhase::IDLE) {
        // Waiting for user to choose auto or manual
        if (consume(ss, EVT_CALIB_AUTO)) {
            ss.setCalibManualMode(false);
            ss.setCalibPhase(CalibPhase::OPEN_HAND);
            // sensor_task takes over: it detects OPEN_HAND and runs
            // its blocking _collectSamples loop.
            Serial.println("[CTRL] → Auto calib OPEN_HAND");
        } else if (consume(ss, EVT_CALIB_MANUAL)) {
            ss.setCalibManualMode(true);
            ss.setManualCalibStep(ManualCalibStep::WARN_OPEN);
            ss.setManualCountdown(MANUAL_WARN_COUNTDOWN);
            ss.setMode(SystemMode::MANUAL_CALIB);
            ss.clearWarning();
            Serial.println("[CTRL] → MANUAL_CALIB WARN_OPEN");
        }
    } else if (cp == CalibPhase::DONE) {
        // Auto calibration completed (sensor_task set DONE).
        // Stamp done timestamp and hold for display.
        static bool done_stamped = false;
        if (!done_stamped) {
            done_stamped = true;
            ss.setCalibDoneTs(xTaskGetTickCount());
            Serial.println("[CTRL] Auto calib DONE, holding display.");
        }
        // After hold period, clean up and return to SAFE_LOCK
        const TickType_t done_tick = ss.getCalibDoneTs();
        const TickType_t now_tick  = xTaskGetTickCount();
        if (done_tick != 0 &&
            (now_tick - done_tick) >= pdMS_TO_TICKS(CALIB_DONE_DISPLAY_MS))
        {
            done_stamped = false;
            reset_calib_state(ss);
            ss.setMode(SystemMode::SAFE_LOCK);
            Serial.println("[CTRL] Auto calib: → SAFE_LOCK");
        }
    }
    // All other CalibPhase values (OPEN_HAND, CLOSE_HAND, FAILED):
    // sensor_task drives the auto-calib sequence; we just wait.
}

// ================================================================
//  handleSession — common handler for therapy modes
// ================================================================
static void handleSession(SystemMode mode, const SensorSnapshot& snap,
                           MotorState out[NUM_FINGERS], SharedState& ss);
// (therapy mode implementations follow below)

// ================================================================
//  Therapy mode implementations (unchanged from previous version,
//  gathered here for completeness)
// ================================================================

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}
static inline uint8_t dutyFromFraction(float f) {
    return (uint8_t)clampf(f * PWM_DUTY_MAX, (float)PWM_DUTY_MIN, (float)PWM_DUTY_MAX);
}
static MotionIntent detectIntent(const SensorSnapshot& snap) {
    if (snap.imu.gyro_mag < INTENT_GYRO_THRESH) return MotionIntent::NONE;
    float avg_vel = 0.0f;
    for (int f = 0; f < NUM_FINGERS; f++) avg_vel += snap.flex[f].velocity;
    avg_vel /= (float)NUM_FINGERS;
    if (fabsf(avg_vel) < INTENT_VEL_THRESH) return MotionIntent::NONE;
    return (avg_vel > 0.0f) ? MotionIntent::CLOSING : MotionIntent::OPENING;
}

// ── Passive ──────────────────────────────────────────────────────
static bool     passive_active   = false;
static bool     passive_complete = false;
static uint32_t passive_start_ms = 0;
static bool     passive_delay    = true;
static uint32_t passive_delay_ms = 0;

static void modePassive(const SensorSnapshot& snap, MotorState out[NUM_FINGERS]) {
    uint32_t now = millis();  // millis() acceptable in therapy modes
    if (!passive_active) {
        passive_active   = true;  passive_complete = false;
        passive_start_ms = now;   passive_delay_ms = now;  passive_delay = true;
    }
    if (passive_complete) { motorsOff(out); return; }
    if (passive_delay) {
        if ((now - passive_delay_ms) < PASSIVE_START_DELAY_MS) { motorsOff(out); return; }
        passive_delay = false;
    }
    if ((now - passive_start_ms) >= PASSIVE_RUNTIME_MS + PASSIVE_START_DELAY_MS) {
        passive_complete = true; motorsOff(out); return;
    }
    uint32_t elapsed = (now - passive_start_ms - PASSIVE_START_DELAY_MS) % PASSIVE_PERIOD_MS;
    float phase  = (float)elapsed / (float)PASSIVE_PERIOD_MS;
    float target = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * phase));
    for (int f = 0; f < NUM_FINGERS; f++) {
        float e = target - snap.flex[f].normalized;
        if (fabsf(e) < 0.03f) {
            out[f].enabled=false; out[f].target=0; out[f].dir=MotorDir::STOP;
        } else {
            out[f].enabled=true;
            out[f].dir   = (e > 0.0f) ? MotorDir::REVERSE : MotorDir::FORWARD;
            out[f].target = dutyFromFraction(fabsf(e));
        }
    }
}

// ── Assistive ────────────────────────────────────────────────────
struct AssistState {
    bool assessment_done=false; bool fsr_reached=false;
    float patient_force=0; uint8_t force_level=0; float motor_fraction=1.0f;
    uint32_t assess_start_ms=0;
    float step_target=0; bool step_active=false; uint32_t step_end_ms=0;
    uint32_t stall_start_ms=0; MotionIntent last_intent=MotionIntent::NONE;
};
static AssistState assist_st;

static void modeAssistive(const SensorSnapshot& snap,
                           MotorState out[NUM_FINGERS]) {
    uint32_t now = millis();
    float avg_norm=0, avg_vel=0;
    for (int f=0;f<NUM_FINGERS;f++){avg_norm+=snap.flex[f].normalized;avg_vel+=snap.flex[f].velocity;}
    avg_norm/=NUM_FINGERS; avg_vel/=NUM_FINGERS;

    if (!assist_st.assessment_done) {
        if (!assist_st.assess_start_ms) assist_st.assess_start_ms=now;
        motorsOff(out);
        if (snap.fsr.normalized > FSR_TOUCHED_THRESH) {
            assist_st.patient_force   = snap.fsr.normalized;
            assist_st.force_level     = (snap.fsr.normalized<.25f)?0:(snap.fsr.normalized<.5f)?1:(snap.fsr.normalized<.75f)?2:3;
            assist_st.motor_fraction  = clampf(1.0f-snap.fsr.normalized,0.25f,1.0f);
            assist_st.fsr_reached     = true;
            assist_st.assessment_done = true;
        } else if ((now-assist_st.assess_start_ms)>=ASSIST_ASSESS_MS) {
            assist_st.motor_fraction  = (avg_norm<ASSIST_FLEX_THRESHOLD)?0.75f:1.0f;
            assist_st.assessment_done = true;
        }
        return;
    }
    uint8_t duty=(uint8_t)clampf(assist_st.motor_fraction*PWM_DUTY_MAX,(float)PWM_DUTY_MIN,(float)PWM_DUTY_MAX);
    bool moving=false;
    for (int f=0;f<NUM_FINGERS;f++) if(fabsf(snap.flex[f].velocity)>ASSIST_STALL_VEL){moving=true;break;}

    if (assist_st.step_active) {
        if (now<assist_st.step_end_ms) {
            for (int f=0;f<NUM_FINGERS;f++){
                float e=assist_st.step_target-snap.flex[f].normalized;
                if(fabsf(e)>0.02f){out[f].enabled=true;out[f].dir=(e>0)?MotorDir::REVERSE:MotorDir::FORWARD;out[f].target=duty;}
                else{out[f].enabled=false;out[f].target=0;out[f].dir=MotorDir::STOP;}
            }
        } else { assist_st.step_active=false; assist_st.stall_start_ms=0; motorsOff(out); }
        return;
    }
    if (moving) {
        assist_st.stall_start_ms=0;
        MotionIntent intent=detectIntent(snap);
        if(intent!=MotionIntent::NONE) assist_st.last_intent=intent;
        for(int f=0;f<NUM_FINGERS;f++){
            if(intent==MotionIntent::CLOSING){out[f].enabled=true;out[f].dir=MotorDir::REVERSE;out[f].target=duty;}
            else if(intent==MotionIntent::OPENING){out[f].enabled=true;out[f].dir=MotorDir::FORWARD;out[f].target=duty;}
            else{out[f].enabled=false;out[f].target=0;out[f].dir=MotorDir::STOP;}
        }
        return;
    }
    if (!assist_st.stall_start_ms) assist_st.stall_start_ms=now;
    if ((now-assist_st.stall_start_ms)>=ASSIST_STALL_MS) {
        float d=(assist_st.last_intent==MotionIntent::OPENING)?-ASSIST_STEP_NORM:ASSIST_STEP_NORM;
        assist_st.step_target=clampf(avg_norm+d,0,1);
        assist_st.step_active=true; assist_st.step_end_ms=now+ASSIST_STEP_MS;
        assist_st.stall_start_ms=0;
    } else motorsOff(out);
}

// ── Resistance ───────────────────────────────────────────────────
static float resist_scale=1.0f;

static void modeResistance(const SensorSnapshot& snap, MotorState out[NUM_FINGERS]) {
    MotionIntent intent=detectIntent(snap);
    float fsr=snap.fsr.normalized;
    resist_scale=(fsr<0.15f)
        ?clampf(resist_scale-RESISTIVE_ADAPT_STEP,RESISTIVE_MIN_SCALE,1.0f)
        :clampf(resist_scale+RESISTIVE_ADAPT_STEP,RESISTIVE_MIN_SCALE,1.0f);
    uint8_t pw=(uint8_t)clampf(dutyFromFraction(fsr)*resist_scale,(float)PWM_DUTY_MIN,(float)PWM_DUTY_MAX);
    for(int f=0;f<NUM_FINGERS;f++){
        float n=snap.flex[f].normalized;
        bool lim=(n<0.03f&&intent==MotionIntent::OPENING)||(n>0.97f&&intent==MotionIntent::CLOSING);
        if(lim||intent==MotionIntent::NONE){out[f].enabled=false;out[f].target=0;out[f].dir=MotorDir::STOP;}
        else{out[f].enabled=true;out[f].dir=(intent==MotionIntent::CLOSING)?MotorDir::FORWARD:MotorDir::REVERSE;out[f].target=pw;}
    }
}

// ================================================================
//  handleSession — therapy modes with session event handling
// ================================================================
static void handleSession(SystemMode mode, const SensorSnapshot& snap,
                           MotorState out[NUM_FINGERS], SharedState& ss) {
    // Session event checks — consume before running mode logic
    if (consume(ss, EVT_GLOBAL_ESTOP)) {
        // Handled in main loop; shouldn't reach here
        motorsOff(out); return;
    }
    if (consume(ss, EVT_SESSION_PAUSE)) {
        saved_session_mode = mode;
        session_paused     = true;
        ss.setWarning("PAUSED");
        ss.setMode(SystemMode::SAFE_LOCK);
        motorsOff(out);
        Serial.println("[CTRL] Session paused.");
        return;
    }
    if (consume(ss, EVT_SESSION_EXIT)) {
        ss.clearWarning();
        ss.setMode(SystemMode::SAFE_LOCK);
        motorsOff(out);
        Serial.println("[CTRL] Session exit.");
        return;
    }
    if (consume(ss, EVT_SESSION_RECALIB)) {
        reset_calib_state(ss);
        ss.setCalibComplete(false);
        ss.requestRecalibration();
        ss.setCalibPhase(CalibPhase::IDLE);
        ss.setMode(SystemMode::CALIBRATING);
        ss.clearWarning();
        motorsOff(out);
        Serial.println("[CTRL] Session recalib.");
        return;
    }

    // Run therapy mode
    switch (mode) {
        case SystemMode::PASSIVE:    modePassive   (snap, out); break;
        case SystemMode::ASSISTIVE:  modeAssistive (snap, out); break;
        case SystemMode::RESISTANCE: modeResistance(snap, out); break;
        default: motorsOff(out); break;
    }
}

// ================================================================
//  control_task — main entry point
//
//  Structure:
//    1. ESTOP guard — highest priority, checked every tick.
//    2. FSM integrity check — validates mode is a known value.
//    3. GLOBAL_ESTOP event — consumed before any mode handler.
//    4. Mode-transition cleanup — resets per-mode statics.
//    5. Mode dispatch — routes to the correct FSM handler.
//    6. Motor ramp application — smooths output.
//    7. Motor state publication — writes to SharedState.
// ================================================================
void control_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    // Motor driver init (once; safety_task also calls begin(),
    // protected by static flag inside MotorDriver::begin())
    getMotorDriver().begin();

    TickType_t    last_wake = xTaskGetTickCount();
    SensorSnapshot snap;
    MotorState     motors[NUM_FINGERS] = {};
    SystemMode     prev_mode = SystemMode::SAFE_LOCK;

    for (;;) {

        // ── 1. ESTOP guard ────────────────────────────────────────
        if (ss.isEStop()) {
            motorsOff(motors);
            getMotorDriver().disableAll();
            // Drain any pending input events so they don't fire on
            // recovery (ESTOP requires power cycle, but be tidy)
            ss.clearEventBits(EVT_ALL_INPUT);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // ── 2. FSM integrity check ────────────────────────────────
        const SystemMode mode = ss.getMode();
        fsm_integrity_check(ss, mode);
        if (ss.isEStop()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // ── 3. Global ESTOP event (from any active mode) ──────────
        if (consume(ss, EVT_GLOBAL_ESTOP)) {
            ss.triggerEStop("MANUAL_OVERRIDE");
            motorsOff(motors);
            getMotorDriver().disableAll();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // ── 4. Mode-transition cleanup ────────────────────────────
        if (mode != prev_mode) {
            Serial.printf("[CTRL] Mode %d → %d\n", (int)prev_mode, (int)mode);
            if (prev_mode == SystemMode::ASSISTIVE)  assist_st = AssistState{};
            if (prev_mode == SystemMode::PASSIVE) {
                passive_active=false; passive_complete=false;
                passive_start_ms=0; passive_delay=true; passive_delay_ms=0;
            }
            if (prev_mode == SystemMode::RESISTANCE) resist_scale = 1.0f;
            prev_mode = mode;
        }

        // ── 5. Sensor snapshot ────────────────────────────────────
        ss.readSensors(snap);

        // ── 6. FSM dispatch ───────────────────────────────────────
        switch (mode) {
            case SystemMode::SAFE_LOCK:
                handleSafeLock(motors, ss);
                break;
            case SystemMode::CALIBRATING:
                handleCalibrating(motors, ss);
                break;
            case SystemMode::MANUAL_CALIB:
                modeManualCalib(motors, ss);
                break;
            case SystemMode::PASSIVE:
            case SystemMode::ASSISTIVE:
            case SystemMode::RESISTANCE:
                handleSession(mode, snap, motors, ss);
                break;
            case SystemMode::ESTOP:
                // Should be caught by guard above; handle defensively
                motorsOff(motors);
                getMotorDriver().disableAll();
                break;
            default:
                // FSM_CORRUPT was already triggered by integrity check
                motorsOff(motors);
                break;
        }

        // ── 7. Apply motor ramp and publish ──────────────────────
        for (int f = 0; f < NUM_FINGERS; f++) {
            getMotorDriver().applyRamp(motors[f], f);
        }
        ss.writeMotors(motors);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_CONTROL_MS));
    }
}