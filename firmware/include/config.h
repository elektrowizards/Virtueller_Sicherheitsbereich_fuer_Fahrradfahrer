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

// BNO085 RST pin (active LOW). Wired to D7 (PA8), the IMU is hardware-
// reset at every begin (boot and IMU recovery). Without it, a reset of
// the STM32 alone leaves the BNO085 running in its old state, which can
// hang the first I2C reads. Set to -1 if RST is not connected.
constexpr int8_t BNO085_RESET_PIN = D7;
constexpr uint32_t BNO085_BOOT_MS = 300;   // wait after the RST pulse
constexpr uint8_t AS5600_ADDRESS = 0x36;

constexpr uint32_t IMU_REPORT_US = 20000; // 50 Hz
constexpr uint32_t IMU_TIMEOUT_MS = 500;

// Sign so that IMU_ROLL_SIGN * (BNO085 roll) is POSITIVE when the
// housing leans to the RIGHT (seen from behind, in riding direction).
constexpr float IMU_ROLL_SIGN = 1.0f;

// ======================================
// STABILIZATION GEOMETRY
// ======================================
// Side on which the laser line is projected (seen from behind):
//   +1 = right, -1 = left.
// The firmware works with phi = roll TOWARD the line side.
constexpr float LINE_SIDE = -1.0f;

// Lateral distance of the line on the ground, measured from the pivot
// (bike: tyre contact line, table: tilt edge of the table).
constexpr float TARGET_DISTANCE_M = 1.50f;

// true  = bench test: device on the table, line on the floor
// false = on the bike
constexpr bool TABLE_TEST = true;

// ---- Bike geometry (pivot = tyre contact line, on the ground) ----
// Laser rotation axis above the tyre contact line, bike upright [m]
// Start value; with TOF_HEIGHT_CALIBRATION the ToF measures it on the bike.
constexpr float BIKE_AXIS_HEIGHT_M = 0.80f;          // FILL IN when mounted
// Rotation axis lateral offset from the bike centre plane, + = toward line
constexpr float BIKE_AXIS_OUTWARD_M = 0.0f;

// ---- ToF height calibration (enclosure_v3, bike only) ----
// The VL53L8CX sits in the bevel on the foot's lower RIGHT edge and looks
// down and to the right, past the rear wheel. On the SHORT press (unit
// mounted, bike held UPRIGHT on flat ground) it fits the ground plane in
// its 8x8 zones and measures the axis height; BIKE_AXIS_HEIGHT_M is then
// only the start value. false = use BIKE_AXIS_HEIGHT_M (ToF bypassed).
constexpr bool TOF_HEIGHT_CALIBRATION = true;
constexpr float TOF_TILT_DEG = 40.0f;      // bevel: sensor axis from straight down (Tof_Angle)
constexpr float TOF_SIDE = 1.0f;           // +1 = sensor looks right, -1 = left
// Sensor optical centre -> laser pivot on the motor axis, housing frame
// (X rearward, Y right, Z up) [m], from build_enclosure_v3.py
constexpr float TOF_TO_PIVOT_X_M = 0.02497f;
constexpr float TOF_TO_PIVOT_Y_M = -0.02053f;
constexpr float TOF_TO_PIVOT_Z_M = 0.01285f;
constexpr float TOF_FOV_DEG = 45.0f;       // per axis, 8 zones each
// VERIFY ONCE ('t' + tape measure): true = each zone reports the distance
// along its own ray, false = along the optical axis.
constexpr bool TOF_DISTANCE_IS_RADIAL = true;
// Zone grid direction along housing X (+1/-1). Only the pitch term uses
// it. 't' prints the housing pitch, + = top tilted rearward (like a
// seatpost): if the sign is wrong on the bike, flip this.
constexpr float TOF_GRID_ALONG_X_SIGN = 1.0f;
constexpr uint8_t TOF_MIN_PLANE_ZONES = 16;      // of 64 zones
constexpr float TOF_PLANE_INLIER_M = 0.04f;      // zone lies on the ground plane
constexpr float TOF_PLANE_MAX_RMS_M = 0.02f;
constexpr float TOF_TILT_TOLERANCE_DEG = 15.0f;  // plane accepted as the ground
constexpr float TOF_MAX_CAL_ROLL_DEG = 5.0f;     // bike must stand upright
constexpr float TOF_MAX_FRAME_SPREAD_M = 0.015f; // the good frames must agree
constexpr uint8_t TOF_TEST_FRAMES = 10;          // 't' command

// ---- Table geometry (pivot = table edge over which the line is cast) ----
constexpr float TABLE_SURFACE_HEIGHT_M = 0.70f;      // FILL IN: measure floor -> table top
constexpr float AXIS_ABOVE_TABLE_M = 0.060f;         // photo: 60 mm (older local config said 0.0307 - CHECK)
// Rotation axis lateral offset from the table edge, + = toward line.
// Photo: axis 36 mm INSIDE the edge -> negative.
constexpr float AXIS_OUTWARD_OF_EDGE_M = -0.036f;

// Selected geometry: pivot height above ground (p), rotation axis above
// pivot (r_up) and toward the line side (r_out), all with housing level.
constexpr float PIVOT_HEIGHT_M = TABLE_TEST ? TABLE_SURFACE_HEIGHT_M : 0.0f;
constexpr float AXIS_ABOVE_PIVOT_M = TABLE_TEST ? AXIS_ABOVE_TABLE_M : BIKE_AXIS_HEIGHT_M;
constexpr float AXIS_OUTWARD_M = TABLE_TEST ? AXIS_OUTWARD_OF_EDGE_M : BIKE_AXIS_OUTWARD_M;

// Perpendicular distance between the beam and the rotation axis
// (+ = beam passes ABOVE the axis when it points horizontally).
// An offset ALONG the beam (11.22 mm emitter offset) has no effect.
constexpr float LASER_BEAM_OFFSET_M = 0.0f;

// RUN mode:
//   true  = hold the LINE DISTANCE on the ground:  alpha = theta(phi) - phi
//   false = hold the world BEAM ANGLE (pure gimbal): alpha = theta0 - phi
constexpr bool RUN_HOLD_DISTANCE = true;

constexpr float MIN_HEIGHT_M = 0.20f;   // ToF plausibility of the measured r_up
constexpr float MAX_HEIGHT_M = 2.00f;

constexpr float MAX_ROLL_DEG = 30.0f;
constexpr float MAX_MOTOR_DELTA_DEG = 35.0f;
constexpr float MAX_ALIGN_TRIM_DEG = 25.0f;
constexpr float ALIGN_STEP_DEG = 1.0f;       // '[' / ']'  (~7 cm on the table floor)
constexpr float ALIGN_FINE_STEP_DEG = 0.1f;  // '{' / '}'  (~7 mm; encoder: 0.088 deg/count)

// ---- Beam angle / encoder ----
// alpha = beam angle BELOW horizontal, relative to the housing
// (+ = pointing down toward the floor).
// BEAM_ENCODER_SIGN: +1 if an INCREASING raw AS5600 angle tilts the beam
// DOWN. Defined in the raw sensor frame, so it does not depend on the
// sensor_direction SimpleFOC detects in initFOC().
constexpr float BEAM_ENCODER_SIGN = -1.0f;            // verify on the bench

// Mechanical travel of the beam relative to the housing:
// 12 o'clock (straight up, -90) -> line side -> 6 o'clock (down, +90).
constexpr float BEAM_MIN_DEG = -90.0f;
constexpr float BEAM_MAX_DEG = 90.0f;

// ---- Reference ('z') and homing ----
// Floor mark used for the one-time reference, distance from the pivot.
constexpr float REFERENCE_DISTANCE_M = TARGET_DISTANCE_M;

constexpr float HOMING_RATE_DEG_S = 60.0f;      // target slew rate
constexpr float HOMING_MAX_TRAVEL_DEG = 185.0f; // separate from MAX_ALIGN_TRIM_DEG
constexpr float HOMING_START_MARGIN_DEG = 45.0f;// tolerated beyond BEAM_MIN/MAX at start
constexpr float HOMED_TOLERANCE_DEG = 1.0f;
constexpr uint32_t HOMING_SETTLE_MS = 3000;     // added to travel/rate as timeout

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
constexpr uint8_t REQUIRED_TOF_FRAMES = 5;   // good ToF frames (5 Hz) for the height

constexpr uint32_t MOTOR_TARGET_PERIOD_MS = 5;
constexpr uint32_t STATUS_PERIOD_MS = 500;

constexpr float MAX_TRACKING_ERROR_DEG = 15.0f;

// Consecutive failed AS5600 reads before the motor is stopped (one read
// per loopFOC pass, ~5.5 kHz: 50 reads ~ 9 ms). Meanwhile the last valid
// angle is used.
constexpr uint32_t MAX_ENCODER_CONSECUTIVE_ERRORS = 50;
constexpr uint32_t TRACKING_ERROR_TIMEOUT_MS = 450;