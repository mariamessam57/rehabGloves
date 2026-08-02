#ifndef _CONFIG_H
#define _CONFIG_H

// ─── ADC PINS ───────────────────────────────────────────────────
#define PIN_FLEX_0 32
#define PIN_FLEX_1 33
#define PIN_FLEX_2 34
#define PIN_FSR 35

// Use one master flex sensor for control/calibration/safety
#define MASTER_FLEX_IDX 1

// ─── MOTOR PINS (DRV8833) ───────────────────────────────────────
#define PIN_M0_IN1 18
#define PIN_M0_IN2 19
#define PIN_M1_IN1 25
#define PIN_M1_IN2 26
#define PIN_M2_IN1 27
#define PIN_M2_IN2 23

// ─── I2C ────────────────────────────────────────────────────────
#define PIN_SDA 21
#define PIN_SCL 22

// ─── KEYPAD ─────────────────────────────────────────────────────
#define KEYPAD_ROWS 2
#define KEYPAD_COLS 2
#define PIN_ROW0 13
#define PIN_ROW1 14
#define PIN_COL0 4
#define PIN_COL1 5

// ─── PWM ────────────────────────────────────────────────────────
#define PWM_FREQ_HZ 20000U
#define PWM_RESOLUTION_BITS 8
#define PWM_DUTY_MAX 200U
#define PWM_DUTY_MIN 35U
#define PWM_RAMP_STEP 6U

// ─── TASK PRIORITIES ────────────────────────────────────────────
#define PRI_SAFETY 5
#define PRI_SENSOR 4
#define PRI_CONTROL 3
#define PRI_INPUT 2
#define PRI_DISPLAY 1

// ─── TASK STACK SIZES ───────────────────────────────────────────
#define STACK_SAFETY 3072
#define STACK_SENSOR 5120
#define STACK_CONTROL 4096
#define STACK_INPUT 2048
#define STACK_DISPLAY 4096

// ─── TASK PERIODS ───────────────────────────────────────────────
#define PERIOD_SAFETY_MS 10
#define PERIOD_SENSOR_MS 20
#define PERIOD_CONTROL_MS 20
#define PERIOD_INPUT_MS 50
#define PERIOD_DISPLAY_MS 100

// ─── CALIBRATION ────────────────────────────────────────────────
#define CALIB_DURATION_MS 7000U
#define CALIB_SAMPLES_MAX 400
#define CALIB_SIGMA 2.0f
#define CALIB_MIN_RANGE 60
#define PREFS_NAMESPACE "rehab_cal"

// ─── FILTERS ────────────────────────────────────────────────────
#define EMA_ALPHA_FLEX 0.15f
#define EMA_ALPHA_FSR 0.12f
#define KALMAN_Q 0.001f
#define KALMAN_R 0.03f

// ─── SAFETY ─────────────────────────────────────────────────────
#define FLEX_SAFETY_MARGIN 0.06f
#define IMU_STUCK_MS 500U
#define IMU_SPIKE_DEGS 480.0f
#define STALL_TIMEOUT_MS 2000U
#define STALL_VEL_THRESH 0.004f

// ─── CONTROL ────────────────────────────────────────────────────
#define INTENT_GYRO_THRESH 5.0f
#define INTENT_VEL_THRESH 0.018f
#define ASSISTIVE_DUTY_PCT 0.60f
#define PASSIVE_PERIOD_MS 3200U
#define FSR_TOUCHED_THRESH 0.20f

#define ASSIST_ASSESS_MS 3000U
#define ASSIST_FLEX_THRESHOLD 0.15f

#define ASSIST_STALL_VEL STALL_VEL_THRESH
#define ASSIST_STALL_MS 1500U

#define ASSIST_STEP_NORM 0.08f
#define ASSIST_STEP_MS 500U

// ─── SYSTEM ─────────────────────────────────────────────────────
#define NUM_FINGERS 3
#define SERIAL_BAUD 115200
#define DISPLAY_SERIAL_ONLY 1

// ─── IMU + FLEX CORRELATION SAFETY ──────────────────────────────
#define IMU_FLEX_CORR_GYRO_MIN 12.0f
#define IMU_FLEX_CORR_VEL_MIN 0.020f
#define IMU_FLEX_CONFLICT_MS 1500U

#endif