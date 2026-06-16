// ================================================================
//  display_task.cpp  — PURE STATE RENDERER
//
//  Architectural contract:
//
//  ✓ Reads SharedState via atomic readSystemSnapshot() only.
//  ✓ Renders exactly one screen per FSM state.
//  ✓ Releases all mutexes before calling oled.display().
//
//  ✗ NEVER calls setMode(), setCalibPhase(), setCalibDoneTs(),
//    setManualCalibStep(), or any state-mutating method.
//  ✗ NEVER acts as FSM authority for any transition.
//  ✗ NEVER auto-returns to SAFE_LOCK (control_task owns that).
//
//  The calib_done_ts field is READ here to choose between showing
//  "Calibration Complete!" and returning nothing — but
//  display_task does NOT clear it. control_task clears it when
//  it transitions to SAFE_LOCK after the 2-second window.
//
//  Screen map (FSM state → screen function):
//
//  ESTOP                        → screenEStop()
//  SAFE_LOCK                    → screenMainMenu()
//  CALIBRATING / IDLE           → screenCalibEntry()
//  CALIBRATING / OPEN_HAND      → screenAutoOpen(countdown)
//  CALIBRATING / CLOSE_HAND     → screenAutoClose(countdown)
//  CALIBRATING / DONE           → screenCalibDone()   ← read-only
//  CALIBRATING / FAILED         → screenCalibFailed()
//  MANUAL_CALIB / WARN_OPEN     → screenManualWarnOpen(cd)
//  MANUAL_CALIB / MOVING_OPEN   → screenManualMovingOpen()
//  MANUAL_CALIB / WAIT_OPEN     → screenManualWaitOpen()
//  MANUAL_CALIB / MOVING_CLOSE  → screenManualMovingClose()
//  MANUAL_CALIB / WAIT_CLOSE    → screenManualWaitClose()
//  MANUAL_CALIB / SAVING        → screenSaving()
//  MANUAL_CALIB / DONE          → screenCalibDone()   ← read-only
//  PASSIVE                      → screenTherapy("Passive",...)
//  ASSISTIVE                    → screenTherapy("Assist",...)
//  RESISTANCE                   → screenTherapy("Resistance",...)
//
//  Button hints visible rules (enforced by screen function design):
//    Buttons shown ONLY in states where control_task accepts them:
//      SAFE_LOCK           → "1:Passive 2:Assist 3:Resistance 4:Calib"
//      CALIBRATING/IDLE    → "1:Yes (auto) 2:No (manual)"
//      WAIT_OPEN_CONFIRM   → "1:Yes 2:More"
//      WAIT_CLOSE_CONFIRM  → "1:Yes 2:More"
//      Session modes       → "1:Pause 2:Cal 3:Exit 4:STOP"
//    MOVING states show "Please wait..." — NO button hints.
// ================================================================

#include "Tasks/DisplayTask/display_task.h"
#include "systemstate/System_State.h"
#include "FSM/fsm_events.h"
#include "config.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <Arduino.h>

#define SCREEN_W   128
#define SCREEN_H    64
#define OLED_RESET  -1

static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

// ── Layout helpers ────────────────────────────────────────────────
static void sep(int y) {
    oled.drawLine(0, y, SCREEN_W - 1, y, SSD1306_WHITE);
}

// ================================================================
//  Screen functions — each is a self-contained renderer.
//  No function reads from SharedState or makes any decision.
//  All data is passed as parameters from the snapshot.
// ================================================================

// ── MAIN MENU ─────────────────────────────────────────────────────
static void screenMainMenu(bool calib_ok, const char* warning) {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Select Mode");
    sep(10);
    oled.setCursor(0, 14);  oled.print("1:Passive  2:Assist");
    oled.setCursor(0, 26);  oled.print("3:Resistance");
    oled.setCursor(0, 38);  oled.print("4:Calibration");
    sep(50);
    if (!calib_ok) {
        oled.setCursor(14, 54); oled.print("Calib needed!");
    } else if (warning) {
        oled.setCursor(0, 54);  oled.print(warning);
    }
}

// ── CALIB ENTRY: "Can you move your hand?" ────────────────────────
// Button hints shown: 1:Yes (auto) 2:No (manual) — valid here
static void screenCalibEntry() {
    oled.setTextSize(1);
    oled.setCursor(28, 0);  oled.print("Calibration");
    sep(10);
    oled.setCursor(14, 18); oled.print("Can you move");
    oled.setCursor(26, 28); oled.print("your hand?");
    sep(42);
    oled.setCursor(4,  50); oled.print("1:Yes        2:No");
}

// ── AUTO CALIB: OPEN HAND ─────────────────────────────────────────
// No button hints — input_task emits nothing during auto calib run
static void screenAutoOpen(int countdown) {
    oled.setTextSize(1);
    oled.setCursor(16, 0);  oled.print("Auto Calibration");
    sep(10);
    oled.setCursor(14, 18); oled.print("Open your hand");
    oled.setCursor(22, 36); oled.printf("Time: %d sec", countdown);
}

// ── AUTO CALIB: CLOSE HAND ────────────────────────────────────────
static void screenAutoClose(int countdown) {
    oled.setTextSize(1);
    oled.setCursor(16, 0);  oled.print("Auto Calibration");
    sep(10);
    oled.setCursor(10, 18); oled.print("Close your hand");
    oled.setCursor(22, 36); oled.printf("Time: %d sec", countdown);
}

// ── MANUAL CALIB: WARN OPEN ───────────────────────────────────────
// No button hints — FSM ignores input during WARN_OPEN
static void screenManualWarnOpen(int countdown) {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(4,  16); oled.print("Fully open your");
    oled.setCursor(4,  26); oled.print("hand now.");
    sep(40);
    oled.setCursor(10, 48); oled.printf("Starting in: %d", countdown);
}

// ── MANUAL CALIB: MOVING OPEN ─────────────────────────────────────
// NO button hints — control_task ignores EVT_BTN_* in MOVING state
static void screenManualMovingOpen() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(16, 20); oled.print("Moving: OPEN");
    oled.setCursor(20, 36); oled.print("Please wait...");
}

// ── MANUAL CALIB: WAIT OPEN CONFIRM ──────────────────────────────
// Button hints shown — control_task consumes EVT_BTN_CONFIRM/MORE here
static void screenManualWaitOpen() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(22, 20); oled.print("Hand open?");
    sep(36);
    oled.setCursor(4,  44); oled.print("1:Yes    2:More");
}

// ── MANUAL CALIB: MOVING CLOSE ────────────────────────────────────
// NO button hints
static void screenManualMovingClose() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(10, 20); oled.print("Moving: CLOSE");
    oled.setCursor(20, 36); oled.print("Please wait...");
}

// ── MANUAL CALIB: WAIT CLOSE CONFIRM ─────────────────────────────
// Button hints shown
static void screenManualWaitClose() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(16, 20); oled.print("Hand closed?");
    sep(36);
    oled.setCursor(4,  44); oled.print("1:Yes    2:More");
}

// ── MANUAL CALIB: SAVING ──────────────────────────────────────────
static void screenSaving() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(28, 28); oled.print("Saving...");
}

// ── CALIBRATION COMPLETE (shared by auto and manual) ──────────────
// display_task shows this when calib_done_ts != 0 and the 2-second
// window has not yet elapsed. control_task clears calib_done_ts
// and transitions to SAFE_LOCK after the window.
static void screenCalibDone() {
    oled.setTextSize(1);
    oled.setCursor(22, 6);  oled.print("Calibration");
    oled.setTextSize(2);
    oled.setCursor(8,  28); oled.print("Complete!");
    oled.setTextSize(1);
}

// ── CALIB FAILED ──────────────────────────────────────────────────
static void screenCalibFailed() {
    oled.setTextSize(1);
    oled.setCursor(20, 8);  oled.print("CALIB FAILED");
    oled.setCursor(14, 26); oled.print("Range too small.");
    oled.setCursor(14, 40); oled.print("Restart device.");
}

// ── THERAPY MODE ──────────────────────────────────────────────────
// Button hints shown — all four keys valid in session modes
static void screenTherapy(const char* label,
                           const SensorSnapshot& snap,
                           const char* warning) {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.printf("Mode: %s", label);
    sep(10);

    oled.setCursor(0, 14);
    oled.printf("F0:%2d%% F1:%2d%% F2:%2d%%",
        (int)(snap.flex[0].normalized * 100.0f),
        (int)(snap.flex[1].normalized * 100.0f),
        (int)(snap.flex[2].normalized * 100.0f));

    oled.setCursor(0, 26);
    oled.printf("FSR:%2d%% Gyro:%.0fdps",
        (int)(snap.fsr.normalized * 100.0f),
        snap.imu.gyro_mag);

    sep(38);
    oled.setCursor(0, 42); oled.print("1:Pause 2:Cal 3:Exit");
    oled.setCursor(0, 54); oled.print("4:STOP");

    if (warning) {
        // Warning overlays the "4:STOP" text area
        oled.fillRect(36, 54, 92, 10, SSD1306_BLACK);
        oled.setCursor(38, 54); oled.print(warning);
    }
}

// ── ESTOP ─────────────────────────────────────────────────────────
static void screenEStop() {
    oled.setTextSize(2);
    oled.setCursor(4,  6);  oled.print("** ESTOP");
    oled.setCursor(14, 26); oled.print("  **");
    oled.setTextSize(1);
    oled.setCursor(14, 48); oled.print("Restart device");
    oled.setCursor(22, 56); oled.print("to continue");
}

// ================================================================
//  display_task — main entry point
// ================================================================
void display_task(void* pvParam) {
    SharedState& ss = SharedState::get();

    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("[DISPLAY] SSD1306 not found!");
        vTaskDelete(nullptr);
    }

    oled.setTextColor(SSD1306_WHITE);
    oled.clearDisplay();
    oled.display();

    TickType_t      last_wake      = xTaskGetTickCount();
    SensorSnapshot  snap;
    SystemMode      mode           = SystemMode::SAFE_LOCK;
    bool            estop          = false;
    CalibPhase      cp             = CalibPhase::IDLE;
    bool            calib_complete = false;
    bool            calib_manual   = false;
    int             countdown      = 0;
    ManualCalibStep mstep          = ManualCalibStep::IDLE;
    int             manual_cd      = 0;
    TickType_t      calib_done_ts  = 0;   // tick, not millis
    const char*     warning        = nullptr;

    for (;;) {
        // ── Atomic snapshot — all SharedState locks released ──────
        // before oled.display() is called (I2C is slow).
        bool ok = ss.readSystemSnapshot(
            snap, mode, estop, warning,
            cp, calib_complete, calib_manual,
            countdown, mstep, manual_cd, calib_done_ts);

        if (!ok) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        oled.clearDisplay();

        // ── Priority 1: ESTOP ────────────────────────────────────
        if (estop || mode == SystemMode::ESTOP) {
            screenEStop();
            oled.display();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        // ── Priority 2: Calibration "Complete!" overlay ──────────
        // display_task shows this screen when calib_done_ts != 0.
        // It does NOT clear calib_done_ts — control_task owns that.
        // This means the screen is visible as long as control_task
        // has not yet transitioned to SAFE_LOCK.
        if (calib_done_ts != 0) {
            screenCalibDone();
            oled.display();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        // ── Priority 3: Route to per-state screen ─────────────────
        switch (mode) {

            case SystemMode::CALIBRATING:
                switch (cp) {
                    case CalibPhase::IDLE:
                        screenCalibEntry();
                        break;
                    case CalibPhase::OPEN_HAND:
                        screenAutoOpen(countdown);
                        break;
                    case CalibPhase::CLOSE_HAND:
                        screenAutoClose(countdown);
                        break;
                    case CalibPhase::DONE:
                        // calib_done_ts should be non-zero here;
                        // handled by Priority 2 above. If we fall
                        // through, show done screen as fallback.
                        screenCalibDone();
                        break;
                    case CalibPhase::FAILED:
                        screenCalibFailed();
                        break;
                    default:
                        break;
                }
                break;

            case SystemMode::MANUAL_CALIB:
                // Each ManualCalibStep → exactly one screen.
                // MOVING screens: no button hints.
                // WAIT screens: button hints visible.
                switch (mstep) {
                    case ManualCalibStep::WARN_OPEN:
                        screenManualWarnOpen(manual_cd);
                        break;
                    case ManualCalibStep::MOVING_OPEN:
                        screenManualMovingOpen();        // no buttons
                        break;
                    case ManualCalibStep::WAIT_OPEN_CONFIRM:
                        screenManualWaitOpen();          // 1:Yes 2:More
                        break;
                    case ManualCalibStep::MOVING_CLOSE:
                        screenManualMovingClose();       // no buttons
                        break;
                    case ManualCalibStep::WAIT_CLOSE_CONFIRM:
                        screenManualWaitClose();         // 1:Yes 2:More
                        break;
                    case ManualCalibStep::SAVING:
                        screenSaving();
                        break;
                    case ManualCalibStep::DONE:
                        screenCalibDone();
                        break;
                    default:
                        break;
                }
                break;

            case SystemMode::PASSIVE:
                screenTherapy("Passive", snap, warning);
                break;

            case SystemMode::ASSISTIVE:
                screenTherapy("Assist", snap, warning);
                break;

            case SystemMode::RESISTANCE:
                screenTherapy("Resistance", snap, warning);
                break;

            case SystemMode::SAFE_LOCK:
            default:
                screenMainMenu(calib_complete, warning);
                break;
        }

        oled.display();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
    }
}