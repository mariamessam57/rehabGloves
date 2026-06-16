#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// ================================================================
//  fsm_events.h  — All EventGroup bit definitions
//
//  There is exactly ONE EventGroupHandle_t in the system, owned
//  by SharedState. All inter-task signalling uses these bits.
//
//  Writing rules:
//    input_task   may SET:   EVT_INPUT_* bits only
//    sensor_task  may SET:   EVT_CALIB_DONE, EVT_MANUAL_SAVE_DONE
//    control_task may CLEAR: any bit it consumes
//    control_task may SET:   EVT_CALIB_DONE (after manual calib)
//    NO OTHER task may set or clear any bit.
//
//  Consumption rules:
//    ONLY control_task may consume (read+clear) input events.
//    Consumption is atomic: xEventGroupWaitBits with
//    clearOnExit=pdTRUE and xTicksToWait=0.
//
//  Bit allocation (32-bit EventGroup, bits 0-23 usable on ESP32):
// ================================================================

// ── System lifecycle ─────────────────────────────────────────────
#define EVT_CALIB_DONE          (1u <<  0)  // set by sensor/control when calib finishes
#define EVT_ESTOP               (1u <<  1)  // set by SharedState::triggerEStop

// ── Input events from SAFE_LOCK (main menu) ──────────────────────
#define EVT_START_PASSIVE       (1u <<  2)  // key 1 in SAFE_LOCK
#define EVT_START_ASSISTIVE     (1u <<  3)  // key 2 in SAFE_LOCK
#define EVT_START_RESISTANCE    (1u <<  4)  // key 3 in SAFE_LOCK
#define EVT_START_CALIB         (1u <<  5)  // key 4 in SAFE_LOCK

// ── Input events from calibration entry (CALIBRATING/IDLE) ───────
#define EVT_CALIB_AUTO          (1u <<  6)  // key 1: user can move → auto calib
#define EVT_CALIB_MANUAL        (1u <<  7)  // key 2: user cannot move → manual calib

// ── Input events from manual calibration (WAIT states only) ──────
#define EVT_BTN_CONFIRM         (1u <<  8)  // key 1: position confirmed
#define EVT_BTN_MORE            (1u <<  9)  // key 2: more movement needed

// ── Input events from therapy sessions ───────────────────────────
#define EVT_SESSION_PAUSE       (1u << 10)  // key 1: pause/resume toggle
#define EVT_SESSION_RECALIB     (1u << 11)  // key 2: restart calibration
#define EVT_SESSION_EXIT        (1u << 12)  // key 3: exit to main menu

// ── Global emergency stop (any non-SAFE_LOCK mode) ───────────────
#define EVT_GLOBAL_ESTOP        (1u << 13)  // key 4: ESTOP in any active mode

// ── Sensor task handshake ─────────────────────────────────────────
#define EVT_MANUAL_SAVE_DONE    (1u << 14)  // sensor_task: NVS write complete

// ── Mask of all input events (for bulk clearing if needed) ───────
#define EVT_ALL_INPUT           ( EVT_START_PASSIVE | EVT_START_ASSISTIVE | EVT_START_RESISTANCE | EVT_START_CALIB | EVT_CALIB_AUTO | EVT_CALIB_MANUAL | EVT_BTN_CONFIRM | EVT_BTN_MORE | EVT_SESSION_PAUSE | EVT_SESSION_RECALIB | EVT_SESSION_EXIT | EVT_GLOBAL_ESTOP )

