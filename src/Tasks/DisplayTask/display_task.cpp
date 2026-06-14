// ================================================================
//  display_task.cpp  — COMPLETE REDESIGN
//
//  Layout budget (128×64, textSize(1) = 6×8 px per char):
//    Max 21 chars per line.  8 usable rows at y = 0,8,16,24,32,40,48,56.
//
//  Screen map:
//  ┌─────────────────────────────────┐
//  │ MAIN MENU                       │
//  │ y= 0  "Select Mode"             │
//  │ y=16  "1:Passive  2:Assist"     │
//  │ y=28  "3:Resistance"            │
//  │ y=40  "4:Calibration"           │
//  ├─────────────────────────────────┤
//  │ CALIB ENTRY (phase==IDLE)       │
//  │ y= 0  "Calibration"             │
//  │ y=20  "Can you move"            │
//  │ y=30  "your hand?"              │
//  │ y=46  "1:Yes     2:No"          │
//  ├─────────────────────────────────┤
//  │ AUTO OPEN / CLOSE               │
//  │ y= 0  "Auto Calibration"        │
//  │ y=20  "Open hand" / "Close hand"│
//  │ y=36  "Time: [N] sec"           │
//  ├─────────────────────────────────┤
//  │ MANUAL WARN_OPEN                │
//  │ y= 0  "Manual Calib"            │
//  │ y=14  "Fully open your hand"    │
//  │ y=34  "Starting in: [N]"        │
//  ├─────────────────────────────────┤
//  │ MANUAL MOVING_OPEN              │
//  │ y= 0  "Manual Calib"            │
//  │ y=14  "Moving: OPEN"            │
//  │ y=34  "Press 1:Yes 2:More"      │
//  ├─────────────────────────────────┤
//  │ MANUAL WAIT_OPEN_CONFIRM        │
//  │ y= 0  "Manual Calib"            │
//  │ y=14  "Hand open?"              │
//  │ y=34  "1:Yes    2:More"         │
//  ├─────────────────────────────────┤
//  │ MANUAL MOVING_CLOSE             │
//  │ y= 0  "Manual Calib"            │
//  │ y=14  "Moving: CLOSE"           │
//  │ y=34  "Press 1:Yes 2:More"      │
//  ├─────────────────────────────────┤
//  │ MANUAL WAIT_CLOSE_CONFIRM       │
//  │ y= 0  "Manual Calib"            │
//  │ y=14  "Hand closed?"            │
//  │ y=34  "1:Yes    2:More"         │
//  ├─────────────────────────────────┤
//  │ CALIB COMPLETED                 │
//  │ y= 0  "Calibration"             │
//  │ y=22  "Completed!"              │
//  │ (2 s then auto → main menu)     │
//  ├─────────────────────────────────┤
//  │ MODE SCREENS (Passive etc.)     │
//  │ y= 0  "Mode: Passive"           │
//  │ y=14  "F0:xx% F1:xx% F2:xx%"   │
//  │ y=28  "FSR:xx%  Gyro:xx dps"   │
//  │ y=44  "1:Paus 2:Cal 3:Exit"     │
//  │ y=56  "4:STOP"                  │
//  ├─────────────────────────────────┤
//  │ ESTOP                           │
//  │ y= 0  "*** ESTOP ***"           │
//  │ y=20  "Press 4 to clear"        │  (future: clear estop button)
//  └─────────────────────────────────┘
//
//  Auto-return to main menu:
//    After calib (auto or manual) completes, display_task shows
//    "Calibration Completed" for 2 seconds then calls
//    ss.setMode(SAFE_LOCK). No blocking, purely time-based.
// ================================================================

#include "Tasks/DisplayTask/display_task.h"
#include "systemstate/System_State.h"
#include "config.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <Arduino.h>

#define SCREEN_W   128
#define SCREEN_H    64
#define OLED_RESET  -1
#define CALIB_DONE_DISPLAY_MS  2000u

static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

// ── String helpers ────────────────────────────────────────────────
static const char* modeStr(SystemMode m) {
    switch (m) {
        case SystemMode::SAFE_LOCK:    return "Safe Lock";
        case SystemMode::CALIBRATING:  return "Calibrating";
        case SystemMode::MANUAL_CALIB: return "Manual Calib";
        case SystemMode::PASSIVE:      return "Passive";
        case SystemMode::ASSISTIVE:    return "Assist";
        case SystemMode::RESISTANCE:   return "Resistance";
        case SystemMode::ESTOP:        return "ESTOP";
        default:                       return "Unknown";
    }
}

// ── Draw a horizontal separator line ─────────────────────────────
static void drawSep(int y) {
    oled.drawLine(0, y, SCREEN_W - 1, y, SSD1306_WHITE);
}

// ================================================================
//  Screen renderers — each clears nothing (caller clears first)
// ================================================================

// ── MAIN MENU ────────────────────────────────────────────────────
static void screenMainMenu(bool calib_ok) {
    oled.setTextSize(1);

    oled.setCursor(22, 0);
    oled.print("Select Mode");
    drawSep(10);

    oled.setCursor(0, 14);
    oled.print("1:Passive  2:Assist");

    oled.setCursor(0, 26);
    oled.print("3:Resistance");

    oled.setCursor(0, 38);
    oled.print("4:Calibration");

    if (!calib_ok) {
        drawSep(50);
        oled.setCursor(0, 54);
        oled.print("! Calib needed !");
    }
}

// ── CALIBRATION ENTRY QUESTION ───────────────────────────────────
static void screenCalibEntry() {
    oled.setTextSize(1);

    oled.setCursor(28, 0);
    oled.print("Calibration");
    drawSep(10);

    oled.setCursor(14, 18);
    oled.print("Can you move");
    oled.setCursor(26, 28);
    oled.print("your hand?");

    drawSep(42);
    oled.setCursor(4, 48);
    oled.print("1:Yes        2:No");
}

// ── AUTO CALIB — OPEN HAND ───────────────────────────────────────
static void screenAutoOpen(int countdown) {
    oled.setTextSize(1);

    oled.setCursor(16, 0);
    oled.print("Auto Calibration");
    drawSep(10);

    oled.setCursor(14, 18);
    oled.print("Open your hand");

    oled.setCursor(22, 34);
    oled.printf("Time: %d sec", countdown);
}

// ── AUTO CALIB — CLOSE HAND ──────────────────────────────────────
static void screenAutoClose(int countdown) {
    oled.setTextSize(1);

    oled.setCursor(16, 0);
    oled.print("Auto Calibration");
    drawSep(10);

    oled.setCursor(10, 18);
    oled.print("Close your hand");

    oled.setCursor(22, 34);
    oled.printf("Time: %d sec", countdown);
}

// ── MANUAL CALIB — WARNING ───────────────────────────────────────
static void screenManualWarn(int countdown) {
    oled.setTextSize(1);

    oled.setCursor(22, 0);
    oled.print("Manual Calib");
    drawSep(10);

    oled.setCursor(0, 16);
    oled.print("Fully open your");
    oled.setCursor(0, 26);
    oled.print("hand now.");

    oled.setCursor(10, 44);
    oled.printf("Starting in: %d", countdown);
}

// ── MANUAL CALIB — MOVING OPEN ───────────────────────────────────
static void screenManualMovingOpen() {
    oled.setTextSize(1);

    oled.setCursor(22, 0);
    oled.print("Manual Calib");
    drawSep(10);

    oled.setCursor(16, 16);
    oled.print("Moving: OPEN");

    drawSep(36);
    oled.setCursor(0, 42);
    oled.print("1:Confirm  2:More");
}

// ── MANUAL CALIB — WAIT OPEN CONFIRM ────────────────────────────
static void screenManualWaitOpen() {
    oled.setTextSize(1);

    oled.setCursor(22, 0);
    oled.print("Manual Calib");
    drawSep(10);

    oled.setCursor(22, 18);
    oled.print("Hand open?");

    drawSep(36);
    oled.setCursor(4, 42);
    oled.print("1:Yes    2:More");
}

// ── MANUAL CALIB — MOVING CLOSE ──────────────────────────────────
static void screenManualMovingClose() {
    oled.setTextSize(1);

    oled.setCursor(22, 0);
    oled.print("Manual Calib");
    drawSep(10);

    oled.setCursor(10, 16);
    oled.print("Moving: CLOSE");

    drawSep(36);
    oled.setCursor(0, 42);
    oled.print("1:Confirm  2:More");
}

// ── MANUAL CALIB — WAIT CLOSE CONFIRM ───────────────────────────
static void screenManualWaitClose() {
    oled.setTextSize(1);

    oled.setCursor(22, 0);
    oled.print("Manual Calib");
    drawSep(10);

    oled.setCursor(16, 18);
    oled.print("Hand closed?");

    drawSep(36);
    oled.setCursor(4, 42);
    oled.print("1:Yes    2:More");
}

// ── CALIBRATION COMPLETED ────────────────────────────────────────
static void screenCalibDone() {
    oled.setTextSize(1);

    oled.setCursor(22, 8);
    oled.print("Calibration");

    oled.setTextSize(2);
    oled.setCursor(8, 30);
    oled.print("Complete!");

    oled.setTextSize(1);
}

// ── MODE SCREEN (therapy session) ────────────────────────────────
static void screenMode(SystemMode mode, const SensorSnapshot& snap,
                        const char* warning) {
    oled.setTextSize(1);

    // Row 0: mode name
    oled.setCursor(0, 0);
    oled.printf("Mode: %s", modeStr(mode));
    drawSep(10);

    // Row 1: flex sensor values
    oled.setCursor(0, 14);
    oled.printf("F0:%2d%% F1:%2d%% F2:%2d%%",
        (int)(snap.flex[0].normalized * 100.0f),
        (int)(snap.flex[1].normalized * 100.0f),
        (int)(snap.flex[2].normalized * 100.0f));

    // Row 2: FSR + gyro
    oled.setCursor(0, 26);
    if (mode == SystemMode::RESISTANCE) {
        oled.printf("FSR:%3d%%  Gyro:%.0fdps",
            (int)(snap.fsr.normalized * 100.0f),
            snap.imu.gyro_mag);
    } else {
        oled.printf("Gyro: %.1f dps", snap.imu.gyro_mag);
    }

    drawSep(38);

    // Row 3: controls
    oled.setCursor(0, 42);
    oled.print("1:Pause 2:Cal 3:Exit");

    oled.setCursor(0, 54);
    oled.print("4:STOP");

    // Warning overlay (bottom right if active)
    if (warning) {
        oled.fillRect(40, 54, 88, 10, SSD1306_BLACK);
        oled.setCursor(42, 54);
        oled.print(warning);
    }
}

// ── ESTOP SCREEN ─────────────────────────────────────────────────
static void screenEStop() {
    oled.setTextSize(2);
    oled.setCursor(4, 4);
    oled.print("** ESTOP");
    oled.setCursor(14, 22);
    oled.print("  **");

    oled.setTextSize(1);
    oled.setCursor(14, 46);
    oled.print("Restart device");
    oled.setCursor(22, 56);
    oled.print("to continue");
}

// ================================================================
//  display_task
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

    TickType_t     last_wake       = xTaskGetTickCount();
    SensorSnapshot snap;
    SystemMode     mode            = SystemMode::SAFE_LOCK;
    bool           estop           = false;
    CalibPhase     cp              = CalibPhase::IDLE;
    bool           calib_complete  = false;
    bool           calib_manual    = false;
    int            countdown       = 0;
    ManualCalibStep mstep          = ManualCalibStep::IDLE;
    int            manual_cd       = 0;
    uint32_t       calib_done_ts   = 0;
    const char*    warning         = nullptr;

    for (;;) {

        // ── Fetch atomic snapshot ─────────────────────────────────
        bool ok = ss.readSystemSnapshot(
            snap, mode, estop, warning,
            cp, calib_complete, calib_manual,
            countdown, mstep, manual_cd, calib_done_ts);

        if (!ok) {
            // SharedState not ready yet
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        oled.clearDisplay();

        // ── ESTOP ─────────────────────────────────────────────────
        if (estop || mode == SystemMode::ESTOP) {
            screenEStop();
            oled.display();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        // ── CALIBRATION COMPLETED (2-second screen + auto-return) ─
        // Both auto and manual calib end here.
        if (calib_done_ts != 0) {
            uint32_t elapsed = millis() - calib_done_ts;
            if (elapsed < CALIB_DONE_DISPLAY_MS) {
                screenCalibDone();
                oled.display();
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
                continue;
            } else {
                // Auto-return to main menu
                ss.setCalibDoneTs(0);
                ss.setMode(SystemMode::SAFE_LOCK);
                ss.setManualCalibStep(ManualCalibStep::IDLE);
                oled.display();
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
                continue;
            }
        }

        // ── AUTO CALIBRATION ──────────────────────────────────────
        if (mode == SystemMode::CALIBRATING) {
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
                    // Handled above via calib_done_ts; shouldn't reach here
                    screenCalibDone();
                    break;
                case CalibPhase::FAILED:
                    oled.setCursor(20, 20);
                    oled.print("CALIB FAILED");
                    oled.setCursor(14, 34);
                    oled.print("Restart device");
                    break;
                default:
                    break;
            }
            oled.display();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        // ── MANUAL CALIBRATION ────────────────────────────────────
        if (mode == SystemMode::MANUAL_CALIB) {
            switch (mstep) {
                case ManualCalibStep::WARN_OPEN:
                    screenManualWarn(manual_cd);
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
                case ManualCalibStep::DONE:
                    screenCalibDone();
                    break;
                default:
                    break;
            }
            oled.display();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        // ── THERAPY MODES ─────────────────────────────────────────
        if (mode == SystemMode::PASSIVE   ||
            mode == SystemMode::ASSISTIVE ||
            mode == SystemMode::RESISTANCE) {
            screenMode(mode, snap, warning);
            oled.display();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
            continue;
        }

        // ── MAIN MENU (SAFE_LOCK) ─────────────────────────────────
        screenMainMenu(calib_complete);
        if (warning) {
            oled.setCursor(0, 56);
            oled.print(warning);
        }
        oled.display();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
    }
}