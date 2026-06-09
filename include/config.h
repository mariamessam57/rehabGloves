#ifndef _CONFIG_H
#define _CONFIG_H


// ─── ADC PINS ───────────────────────────────────────────────────
#define PIN_FLEX_0          36      // VP  – Finger 0
#define PIN_FLEX_1          39      // VN  – Finger 1
#define PIN_FLEX_2          34      //     – Finger 2
#define PIN_FSR             35      //     – Force Sensor

// ─── MOTOR PINS (DRV8833) ───────────────────────────────────────
#define PIN_M0_IN1          18
#define PIN_M0_IN2          19
#define PIN_M1_IN1          21
#define PIN_M1_IN2          22
#define PIN_M2_IN1          23
#define PIN_M2_IN2          25

// ─── I2C ────────────────────────────────────────────────────────
#define PIN_SDA             26
#define PIN_SCL             27

// ─── KEYPAD (1×4 via 2 rows × 2 cols) ──────────────────────────
#define KEYPAD_ROWS         2
#define KEYPAD_COLS         2
#define PIN_ROW0            13
#define PIN_ROW1            14
#define PIN_COL0            15
#define PIN_COL1            16

// ─── BUZZER ─────────────────────────────────────────────────────
#define PIN_BUZZER          32

// ─── PWM (LEDC) ─────────────────────────────────────────────────
#define PWM_FREQ_HZ         20000U
#define PWM_RESOLUTION_BITS 8           // 0–255
#define PWM_DUTY_MAX        200U        // software cap
#define PWM_DUTY_MIN        35U
#define PWM_RAMP_STEP       6U

// ─── TASK PRIORITIES ────────────────────────────────────────────
#define PRI_SAFETY          5
#define PRI_SENSOR          4
#define PRI_CONTROL         3
#define PRI_INPUT           2
#define PRI_DISPLAY         1

// ─── TASK STACK SIZES (words) ───────────────────────────────────
#define STACK_SAFETY        3072
#define STACK_SENSOR        5120
#define STACK_CONTROL       4096
#define STACK_INPUT         2048
#define STACK_DISPLAY       4096

// ─── TASK PERIODS (ms) ──────────────────────────────────────────
#define PERIOD_SAFETY_MS    10
#define PERIOD_SENSOR_MS    20
#define PERIOD_CONTROL_MS   20
#define PERIOD_INPUT_MS     50
#define PERIOD_DISPLAY_MS   100

// ─── CALIBRATION ────────────────────────────────────────────────
#define CALIB_DURATION_MS   7000U
#define CALIB_SAMPLES_MAX   400
#define CALIB_SIGMA         2.0f
#define CALIB_MIN_RANGE     80          // ADC counts minimum valid range
#define PREFS_NAMESPACE     "rehab_cal"

// ─── FILTER PARAMETERS ──────────────────────────────────────────
#define EMA_ALPHA_FLEX      0.15f
#define EMA_ALPHA_FSR       0.12f
#define KALMAN_Q            0.001f
#define KALMAN_R            0.03f

// ─── SAFETY THRESHOLDS ──────────────────────────────────────────
#define FLEX_SAFETY_MARGIN  0.06f       // 6% outside [0,1]
#define IMU_STUCK_MS        500U
#define IMU_SPIKE_DEGS      480.0f      // deg/s spike threshold
#define STALL_TIMEOUT_MS    2000U
#define STALL_VEL_THRESH    0.004f

// ─── CONTROL PARAMETERS ─────────────────────────────────────────
#define INTENT_GYRO_THRESH  14.0f       // deg/s
#define INTENT_VEL_THRESH   0.018f      // normalized/s
#define ASSISTIVE_DUTY_PCT  0.60f       // 60% of PWM_DUTY_MAX
#define PASSIVE_PERIOD_MS   3200U       // full sinusoidal cycle

// ─── SYSTEM ─────────────────────────────────────────────────────
#define NUM_FINGERS         3
#define SERIAL_BAUD         115200

#endif