// ================================================================
//  REHABILITATION GLOVE FIRMWARE
//  ESP32 + FreeRTOS + PlatformIO Arduino Framework
//  3-Finger System | DRV8833 | MPU6050 | SSD1306
// ================================================================
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "systemstate/System_State.h"
#include "config.h"
#include <Wire.h>

// Task entry points
#include "Tasks/SensorTask/sensor_task.h"
#include "Tasks/ControlTask/control_task.h"
#include "Tasks/SafetyTask/safety_task.h"
#include "Tasks/InputTask/input_task.h"
#include "Tasks/DisplayTask/display_task.h"

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println("=== Rehab Glove Firmware Boot ===");

    // 1. Init shared state (mutexes + event group)
    SharedState::get().init();

    // Initialize I2C for shared devices (OLED + IMU)
    Wire.begin(PIN_SDA, PIN_SCL);

    // 2. Create tasks — pinned to cores for determinism
    //    Core 0: Safety, Control, Input
    //    Core 1: Sensor, Display

    xTaskCreatePinnedToCore(
        safety_task, "Safety",
        STACK_SAFETY, nullptr, PRI_SAFETY, nullptr, 0);

    xTaskCreatePinnedToCore(
        sensor_task, "Sensor",
        STACK_SENSOR, nullptr, PRI_SENSOR, nullptr, 1);

    xTaskCreatePinnedToCore(
        control_task, "Control",
        STACK_CONTROL, nullptr, PRI_CONTROL, nullptr, 0);

    xTaskCreatePinnedToCore(
        input_task, "Input",
        STACK_INPUT, nullptr, PRI_INPUT, nullptr, 0);

    xTaskCreatePinnedToCore(
        display_task, "Display",
        STACK_DISPLAY, nullptr, PRI_DISPLAY, nullptr, 1);

    Serial.println("[MAIN] All tasks created. Scheduler running.");
}

void loop() {
    // Unused — FreeRTOS scheduler owns execution
    vTaskDelay(portMAX_DELAY);
}