#pragma once
#include <Arduino.h>

// ======================================
// GPIO - NUCLEO F446RE
// ======================================

// Push button: active HIGH, external 10k pulldown
constexpr uint32_t BUTTON_PIN = D13;    // PA5
constexpr uint32_t RED_LED_PIN = D12;   // PA6
constexpr uint32_t BLUE_LED_PIN = D11;  // PA7

// DRI0058: 3 PWM on TIM2
constexpr uint32_t MOTOR_IN1 = D3;      // PB3
constexpr uint32_t MOTOR_IN2 = D6;      // PB10
constexpr uint32_t MOTOR_IN3 = A0;      // PA0
constexpr uint32_t MOTOR_ENABLE = D8;   // PA9

// Shared I2C bus
constexpr uint32_t I2C_SDA_PIN = D14;   // PB9
constexpr uint32_t I2C_SCL_PIN = D15;   // PB8
constexpr uint32_t I2C_FREQUENCY = 50000;

// Laser: low-side switch (NPN on prototype, AO3400A on PCB), HIGH = on.
// Use PB0 directly: A3 maps to PB0 on the Nucleo, but not necessarily
// in the generic STM32F446RE variant used for the custom PCB.
constexpr uint32_t LASER_PIN = PB0;     // Nucleo A3

// false = laser stays off in every state (bench testing without laser)
constexpr bool LASER_ENABLED = true;

// ======================================
// SENSOR SETTINGS
// ======================================
constexpr uint8_t BNO085_ADDRESS = 0x4A;
constexpr uint8_t AS5600_ADDRESS = 0x36;

constexpr uint32_t IMU_REPORT_US = 20000; // 50 Hz
constexpr uint32_t IMU_TIMEOUT_MS = 500;

// Change if sensor mounting reverses the measured roll
constexpr float IMU_ROLL_SIGN = 1.0f;

// ======================================
// STABILIZATION GEOMETRY
// ======================================
// ---- Height of the laser rotation axis above the ground ----
// true  = prototype on the table, line projected onto the floor
// false = mounted on the bike
constexpr bool  TABLE_TEST             = true;

constexpr float TABLE_SURFACE_HEIGHT_M = 0.75f;    // floor to tabletop, MEASURE YOURS
constexpr float AXIS_ABOVE_TABLE_M     = 0.0307f;  // tabletop to rotation axis
constexpr float BIKE_AXIS_HEIGHT_M     = 0.80f;    // ground to rotation axis on the bike

constexpr float FIXED_HEIGHT_M = TABLE_TEST
    ? (TABLE_SURFACE_HEIGHT_M + AXIS_ABOVE_TABLE_M)
    : BIKE_AXIS_HEIGHT_M;

// ---- Target line distance ----
// Measured sideways from the point on the ground directly below the axis
constexpr float TARGET_DISTANCE_M      = 1.50f;

constexpr float MIN_HEIGHT_M = 0.20f;
constexpr float MAX_HEIGHT_M = 2.00f;

constexpr float MAX_ROLL_DEG = 30.0f;
constexpr float MAX_MOTOR_DELTA_DEG = 35.0f;
constexpr float MAX_ALIGN_TRIM_DEG = 25.0f;
constexpr float ALIGN_STEP_DEG = 1.0f;

// Motor-direction convention must be verified
constexpr float MOTOR_ROLL_SIGN = 1.0f;

// ======================================
// MOTOR - FIT1035
// ======================================
constexpr int MOTOR_POLE_PAIRS = 7;

// 3S battery: about 12.6 V full, 11.1 V nominal. 12 V is a good
// middle value; measure at the DRI0058 VCC terminal if you can.
constexpr float MOTOR_SUPPLY_VOLTAGE = 12.0f;

// Operating limit for Uq (bench-validated)
constexpr float MOTOR_VOLTAGE_LIMIT = 2.0f;

// Electrical alignment (initFOC). Bench: alignment reliable from 3 V up.
constexpr float MOTOR_ALIGN_VOLTAGE = 4.0f;

// Position controller, tuned on the bench (bare motor, no laser):
//   Uq = KP*err - KD*velocity + integral(KI*err)
constexpr float MOTOR_KP = 15.0f;     // V/rad
constexpr float MOTOR_KD = 0.06f;     // V/(rad/s)
constexpr float MOTOR_KI = 32.0f;     // V/(rad*s), integral clamped to +/-1 V
constexpr float MOTOR_VEL_TF = 0.04f; // s, velocity low-pass filter

// Encoder velocity window (bench value: 5 ms)
constexpr float MOTOR_ENC_MIN_ELAPSED_S = 0.005f;

// ======================================
// BUTTON AND TIMING
// ======================================
constexpr uint32_t DEBOUNCE_MS = 35;
constexpr uint32_t MIN_PRESS_MS = 45;
constexpr uint32_t LONG_PRESS_MS = 1400;
constexpr uint32_t DOUBLE_PRESS_MS = 350;

constexpr uint32_t CALIBRATION_TIMEOUT_MS = 7000;
constexpr uint32_t TOF_POLL_MS = 80;
constexpr uint8_t REQUIRED_TOF_FRAMES = 5;

constexpr uint32_t MOTOR_TARGET_PERIOD_MS = 5;
constexpr uint32_t STATUS_PERIOD_MS = 500;

constexpr float MAX_TRACKING_ERROR_DEG = 15.0f;
constexpr uint32_t TRACKING_ERROR_TIMEOUT_MS = 450;