// ================================================================
//  display_task.cpp  — FIXED (replacement file as supplied,
//                     with two additional corrections applied)
//
//  FIX C7: Include path corrected from
//          "../include/systemstate/System_State.h"
//          to "systemstate/System_State.h" matching every other
//          translation unit in the project.
//
//  FIX H5: All oled.display() calls already wrapped in
//          I2CLockGuard in the supplied file — preserved as-is.
//          The I2CLockGuard itself is now defined in
//          System_State.h (FIX C5) so this file compiles.
//
//  [BUG-16] warning pointer: readSystemSnapshot previously returned
//           a const char* pointing into SharedState's internal
//           _warning buffer after releasing the mutex. Any write
//           to setWarning() from another task would corrupt the
//           string while display_task was rendering.
//
//           Fix: readSystemSnapshot now copies into a caller-owned
//           char[32] buffer (see System_State.h / .cpp).
//           This file declares 'char warning_buf[32]' and passes it.
//           All screen functions receive 'const char*' from this
//           local buffer — safe, mutex-independent lifetime.
//
//  [BUG-17] oled.printf() is not guaranteed available on all
//           Adafruit_GFX versions. Replaced with snprintf() into
//           a local char[32] and oled.print(). Removes implicit
//           dependency on a non-standard extension.
//
//  Architectural contract unchanged — display_task remains a
//  pure renderer with no FSM authority.
// ================================================================

#include "Tasks/DisplayTask/display_task.h"
#include "systemstate/System_State.h"     // FIX C7: corrected path
#include "config.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <Arduino.h>

#define SCREEN_W   128
#define SCREEN_H    64
#define OLED_RESET  -1

static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

// ── Formatting helper ─────────────────────────────────────────────
// [BUG-17 FIX] Use this instead of oled.printf() everywhere.
static char _fmt[32];  // single shared format buffer (display_task only)
#define OFMT(...)  (snprintf(_fmt, sizeof(_fmt), __VA_ARGS__), _fmt)

// ── Layout helpers ────────────────────────────────────────────────
static void sep(int y) {
    oled.drawLine(0, y, SCREEN_W - 1, y, SSD1306_WHITE);
}

// ================================================================
//  Screen functions — no SharedState access, no FSM decisions.
//  All data passed as parameters from the snapshot.
//  [BUG-17 FIX] oled.printf → snprintf + oled.print throughout.
// ================================================================

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
    } else if (warning && warning[0]) {
        oled.setCursor(0, 54);  oled.print(warning);
    }
}

static void screenCalibEntry() {
    oled.setTextSize(1);
    oled.setCursor(28, 0);  oled.print("Calibration");
    sep(10);
    oled.setCursor(14, 18); oled.print("Can you move");
    oled.setCursor(26, 28); oled.print("your hand?");
    sep(42);
    oled.setCursor(4,  50); oled.print("1:Yes        2:No");
}

static void screenAutoOpen(int countdown) {
    oled.setTextSize(1);
    oled.setCursor(16, 0);  oled.print("Auto Calibration");
    sep(10);
    oled.setCursor(14, 18); oled.print("Open your hand");
    oled.setCursor(22, 36);
    oled.print(OFMT("Time: %d sec", countdown));  // [BUG-17 FIX]
}

static void screenAutoClose(int countdown) {
    oled.setTextSize(1);
    oled.setCursor(16, 0);  oled.print("Auto Calibration");
    sep(10);
    oled.setCursor(10, 18); oled.print("Close your hand");
    oled.setCursor(22, 36);
    oled.print(OFMT("Time: %d sec", countdown));  // [BUG-17 FIX]
}

static void screenManualWarnOpen(int countdown) {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(4,  16); oled.print("Fully open your");
    oled.setCursor(4,  26); oled.print("hand now.");
    sep(40);
    oled.setCursor(10, 48);
    oled.print(OFMT("Starting in: %d", countdown));  // [BUG-17 FIX]
}

static void screenManualMovingOpen() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(16, 20); oled.print("Moving: OPEN");
    oled.setCursor(20, 36); oled.print("Please wait...");
}

static void screenManualWaitOpen() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(22, 20); oled.print("Hand open?");
    sep(36);
    oled.setCursor(4,  44); oled.print("1:Yes    2:More");
}

static void screenManualMovingClose() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(10, 20); oled.print("Moving: CLOSE");
    oled.setCursor(20, 36); oled.print("Please wait...");
}

static void screenManualWaitClose() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(16, 20); oled.print("Hand closed?");
    sep(36);
    oled.setCursor(4,  44); oled.print("1:Yes    2:More");
}

static void screenSaving() {
    oled.setTextSize(1);
    oled.setCursor(22, 0);  oled.print("Manual Calib");
    sep(10);
    oled.setCursor(28, 28); oled.print("Saving...");
}

static void screenCalibDone() {
    oled.setTextSize(1);
    oled.setCursor(22, 6);  oled.print("Calibration");
    oled.setTextSize(2);
    oled.setCursor(8,  28); oled.print("Complete!");
    oled.setTextSize(1);
}

static void screenCalibFailed() {
    oled.setTextSize(1);
    oled.setCursor(20, 8);  oled.print("CALIB FAILED");
    oled.setCursor(14, 26); oled.print("Range too small.");
    oled.setCursor(14, 40); oled.print("Press 4:Calib");
    oled.setCursor(14, 52); oled.print("to retry.");
}

// [BUG-17 FIX] snprintf throughout; no oled.printf
static void screenTherapy(const char* label,
                           const SensorSnapshot& snap,
                           const char* warning) {
    char line[24];

    oled.setTextSize(1);
    oled.setCursor(0, 0);
    snprintf(line, sizeof(line), "Mode: %s", label);
    oled.print(line);
    sep(10);

    oled.setCursor(0, 14);
    snprintf(line, sizeof(line), "F0:%2d%% F1:%2d%% F2:%2d%%",
        (int)(snap.flex[0].normalized * 100.0f),
        (int)(snap.flex[1].normalized * 100.0f),
        (int)(snap.flex[2].normalized * 100.0f));
    oled.print(line);

    oled.setCursor(0, 26);
    snprintf(line, sizeof(line), "FSR:%2d%% G:%.0fdps",
        (int)(snap.fsr.normalized * 100.0f),
        snap.imu.gyro_mag);
    oled.print(line);

    sep(38);
    oled.setCursor(0, 42); oled.print("1:Pause 2:Cal 3:Exit");
    oled.setCursor(0, 54); oled.print("4:STOP");

    if (warning && warning[0]) {
        oled.fillRect(36, 54, 92, 10, SSD1306_BLACK);
        oled.setCursor(38, 54); oled.print(warning);
    }
}

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

    // FIX H5: OLED init wrapped in I2CLockGuard so it doesn't
    // race with the IMU init that also uses Wire.
    bool oled_ok = false;
    {
        I2CLockGuard lock;
        if (lock.ok()) {
            oled_ok = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        }
    }

    if (!oled_ok) {
        Serial.println("[DISPLAY] SSD1306 not found!");
        vTaskDelete(nullptr);
    }

    oled.setTextColor(SSD1306_WHITE);
    oled.clearDisplay();
    {
        I2CLockGuard lock;
        if (lock.ok()) oled.display();
    }

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
    TickType_t      calib_done_ts  = 0;

    // [BUG-16 FIX] warning is a local buffer, not a pointer into SharedState.
    char warning_buf[32] = {};

    for (;;) {
        // ── Atomic snapshot — [BUG-16 FIX] warning_buf is local ──
        bool ok = ss.readSystemSnapshot(
            snap, mode, estop,
            warning_buf,
            cp, calib_complete, calib_manual,
            countdown, mstep, manual_cd, calib_done_ts);

        if (!ok) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        // warning_buf is valid for the entire render frame — no race
        const char* warning = (warning_buf[0]) ? warning_buf : nullptr;

        oled.clearDisplay();

        // ── Priority 1: ESTOP ────────────────────────────────────
        if (estop || mode == SystemMode::ESTOP) {
            screenEStop();
            {
                I2CLockGuard lock;
                if (lock.ok()) oled.display();
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        // ── Priority 2: Calibration "Complete!" overlay ──────────
        // FIX H4: calib_done_ts is auto-cleared in readSystemSnapshot
        // after CALIB_DONE_SHOW_MS, so this overlay is time-bounded.
        if (calib_done_ts != 0) {
            screenCalibDone();
            {
                I2CLockGuard lock;
                if (lock.ok()) oled.display();
            }
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
                switch (mstep) {
                    case ManualCalibStep::WARN_OPEN:
                        screenManualWarnOpen(manual_cd);
                        break;
                    case ManualCalibStep::MOVING_OPEN:
                        screenManualMovingOpen();
                        break;
                    case ManualCalibStep::WAIT_OPEN_CONFIRM:
                        screenManualWaitOpen();
                        break;
                    case ManualCalibStep::MOVING_CLOSE:
                        screenManualMovingClose();
                        break;
                    case ManualCalibStep::WAIT_CLOSE_CONFIRM:
                        screenManualWaitClose();
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

        // FIX H5: every oled.display() call is I2C-mutex-guarded
        {
            I2CLockGuard lock;
            if (lock.ok()) oled.display();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
    }
}