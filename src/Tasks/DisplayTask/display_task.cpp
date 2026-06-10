#include "Tasks/DisplayTask/display_task.h"
#include "systemstate/System_State.h"
#include "config.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <Arduino.h>

#define SCREEN_W    128
#define SCREEN_H    64
#define OLED_RESET  -1

static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

static const char* modeStr(SystemMode m) {
    switch (m) {
        case SystemMode::SAFE_LOCK:   return "SAFE LOCK";
        case SystemMode::CALIBRATING: return "CALIBRATING";
        case SystemMode::PASSIVE:     return "PASSIVE";
        case SystemMode::ASSISTIVE:   return "ASSISTIVE";
        case SystemMode::RESISTANCE:  return "RESISTANCE";
        case SystemMode::ESTOP:       return "*** ESTOP ***";
        default:                      return "UNKNOWN";
    }
}

static const char* calibStr(CalibPhase p) {
    switch (p) {
        case CalibPhase::OPEN_HAND:  return "OPEN HAND";
        case CalibPhase::CLOSE_HAND: return "CLOSE HAND";
        case CalibPhase::DONE:       return "DONE";
        case CalibPhase::FAILED:     return "FAILED";
        default:                     return "IDLE";
    }
}

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

    TickType_t    last_wake = xTaskGetTickCount();
    SensorSnapshot snap;
    SystemMode mode = SystemMode::SAFE_LOCK;
    bool       estop = false;
    CalibPhase cp   = CalibPhase::IDLE;
    bool       calib_complete = false;
    bool       calib_manual = false;

    for (;;) {
        const char* warning = nullptr;
        if (!ss.readSystemSnapshot(snap, mode, estop, warning, cp, calib_complete, calib_manual)) {
            estop = false;
            mode = SystemMode::SAFE_LOCK;
            cp = CalibPhase::IDLE;
            calib_complete = false;
            calib_manual = false;
            snap = {};
            warning = nullptr;
        }

        oled.clearDisplay();

        // ── Row 0: Mode ────────────────────────────────────────
        oled.setTextSize(1);
        oled.setCursor(0, 0);
        oled.print("MODE: ");
        oled.print(modeStr(mode));

        // ── Row 1: System state ────────────────────────────────
        oled.setCursor(0, 12);
        oled.print("STATE: ");
        oled.print(estop ? "ESTOP" : (calib_complete ? "ACTIVE" : "CALIB"));

        // ── Row 2: Flex values ─────────────────────────────────
        oled.setCursor(0, 24);
        oled.printf("F0:%3d%% F1:%3d%% F2:%3d%%",
            (int)(snap.flex[0].normalized * 100.0f),
            (int)(snap.flex[1].normalized * 100.0f),
            (int)(snap.flex[2].normalized * 100.0f));

        // ── Row 3: IMU gyro magnitude ──────────────────────────
        oled.setCursor(0, 36);
        oled.printf("GYRO: %.1f dps", snap.imu.gyro_mag);

        if (mode == SystemMode::SAFE_LOCK) {
            oled.setCursor(0, 48);
            if (calib_complete && cp == CalibPhase::DONE) {
                oled.print("CAL: DONE 1:P 2:A 3:R 4:E");
            } else {
                oled.print("1:P 2:A 3:R 4:E");
            }
        } else {
            oled.setCursor(0, 48);
            if (mode == SystemMode::CALIBRATING) {
                if (cp == CalibPhase::IDLE) {
                    oled.print("MOVE? 1:Y 2:N");
                } else if (cp == CalibPhase::OPEN_HAND) {
                    oled.print("OPEN HAND 5s");
                } else if (cp == CalibPhase::CLOSE_HAND) {
                    if (calib_manual) {
                        oled.print("MANUAL CLOSE");
                    } else {
                        oled.print("CLOSE HAND 5s");
                    }
                } else {
                    oled.print("CAL: ");
                    oled.print(calibStr(cp));
                }
            } else {
                oled.print("CAL: ");
                oled.print(calibStr(cp));
            }
        }

        if (warning) {
            oled.setCursor(0, 56);
            oled.print("WARN: ");
            oled.print(warning);
        }

        oled.display();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
    }
}