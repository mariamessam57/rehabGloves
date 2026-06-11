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
    int        timer_val = 0; // متغير محلي لاستقبال قيمة العداد التنازلي الحية

    for (;;) {
        const char* warning = nullptr;
        
        // جلب الـ Snapshot متضمناً البارامتر الجديد الـ timer_val تحت حماية الـ Mutex المشترك
        if (!ss.readSystemSnapshot(snap, mode, estop, warning, cp, calib_complete, calib_manual, timer_val)) {
            estop = false;
            mode = SystemMode::SAFE_LOCK;
            cp = CalibPhase::IDLE;
            calib_complete = false;
            calib_manual = false;
            snap = {};
            warning = nullptr;
            timer_val = 0;
        }

        oled.clearDisplay();
        oled.setTextSize(1);

        // ── Row 0: Mode (y = 0) ────────────────────────────────────────
        oled.setCursor(0, 0);
        oled.print("MODE: ");
        oled.print(modeStr(mode));

        // ── Row 1: System state (y = 10) ────────────────────────────────
        oled.setCursor(0, 10);
        oled.print("STATE: ");
        oled.print(estop ? "ESTOP" : (calib_complete ? "ACTIVE" : "CALIB"));

        // ── عرض ديناميكي بناءً على حالة المعايرة ────────────────────────
        if (calib_complete && cp == CalibPhase::DONE) {
            // المعايرة مكتملة والنظام نشط -> اعرض القراءات الحية للمستشعرات
            
            // ── Row 2: Flex values (y = 22)
            oled.setCursor(0, 22);
            oled.printf("F0:%3d%% F1:%3d%% F2:%3d%%",
                (int)(snap.flex[0].normalized * 100.0f),
                (int)(snap.flex[1].normalized * 100.0f),
                (int)(snap.flex[2].normalized * 100.0f));

            // ── Row 3: Dynamic Sensor Feedback (y = 34)
            oled.setCursor(0, 34);
            if (mode == SystemMode::RESISTANCE) {
                oled.printf("FSR: %3d%%", (int)(snap.fsr.normalized * 100.0f));
            } else {
                oled.printf("GYRO: %.1f dps", snap.imu.gyro_mag);
            }

            // ── Row 4: Navigation / Menu (y = 46)
            oled.setCursor(0, 46);
            if (mode == SystemMode::SAFE_LOCK) {
                oled.print("1:M1 2:M2 3:M3 4:CAL");
            } else {
                oled.print("CAL: ");
                oled.print(calibStr(cp));
            }
        } 
        else {
            // النظام لم يُعاير بعد أو يمر بمراحل المعايرة -> اعرض أسطر الأسئلة والإرشادات فقط
            
            // ── Row 3: Navigation / Prompts (y = 28) 
            oled.setCursor(0, 28);
            if (mode == SystemMode::SAFE_LOCK) {
                // إرشاد البداية
                oled.print("PRESS 4 TO CALIBRATE");
            } 
            else if (mode == SystemMode::CALIBRATING) {
                // تفرعات أسئلة المعايرة والتايمر التنازلي المباشر
                if (cp == CalibPhase::IDLE) {
                    oled.print("MOVE? 1:Y 2:N");
                } 
                else if (cp == CalibPhase::OPEN_HAND) {
                    // عرض تحديث الثواني لايف أثناء فرد اليد
                    oled.printf("OPEN HAND... [%d s]", timer_val);
                } 
                else if (cp == CalibPhase::CLOSE_HAND) {
                    if (calib_manual) {
                        oled.print(">> MANUAL CLOSE <<");
                    } else {
                        // عرض تحديث الثواني لايف أثناء غلق اليد
                        oled.printf("CLOSE HAND... [%d s]", timer_val);
                    }
                } 
                else {
                    oled.print("CALIBRATING: ");
                    oled.print(calibStr(cp));
                }
            }
        }

        // ── Row 5: Safety Manager System Warnings (y = 56) ─────────────
        if (warning) {
            oled.setCursor(0, 56);
            oled.print("WARN: ");
            oled.print(warning);
        }

        oled.display();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
    }
}