#include "MotorControl.h"
#include "config.h"

namespace {
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}

MotorControl::MotorControl()
    : encoder_(AS5600_I2C),
      motor_(MOTOR_POLE_PAIRS),
      driver_(
          MOTOR_IN1,
          MOTOR_IN2,
          MOTOR_IN3,
          MOTOR_ENABLE
      ) {
}

bool MotorControl::magnetOK() {
    if (!wire_) return false;

    // AS5600 status register 0x0B, bit 5: magnet detected
    wire_->beginTransmission(AS5600_ADDRESS);
    wire_->write(0x0B);

    if (wire_->endTransmission(false) != 0)
        return false;

    if (wire_->requestFrom(
        (uint8_t)AS5600_ADDRESS,
        (uint8_t)1
    ) != 1) return false;

    return (wire_->read() & 0x20) != 0;
}

bool MotorControl::begin(TwoWire& wire) {
    wire_ = &wire;

    pinMode(MOTOR_ENABLE, OUTPUT);
    digitalWrite(MOTOR_ENABLE, LOW);

    // Shows the MOT:/DRV: alignment messages in the serial monitor
    SimpleFOCDebug::enable(&Serial);

    if (!magnetOK()) return false;

    encoder_.init(&wire);
    // Velocity is computed over at least this window (bench value).
    encoder_.min_elapsed_time = MOTOR_ENC_MIN_ELAPSED_S;
    motor_.linkSensor(&encoder_);

    // ---- Driver ----
    // driver_.voltage_limit is deliberately NOT set. SimpleFOC centres the
    // phase voltages at driver.voltage_limit / 2 and clips each phase to
    // [0, driver.voltage_limit]. Setting it to 2 V (old code) halved and
    // distorted the motor voltage. It stays at the supply voltage; the
    // motor is limited only through motor_.voltage_limit.
    driver_.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver_.pwm_frequency = 20000;
    driver_.enable_active_high = true;   // DRI0058 EN is active HIGH

    if (driver_.init() != 1) return false;

    motor_.linkDriver(&driver_);

    // ---- Motor ----
    // Voltage-torque mode: this class computes Uq itself.
    motor_.controller = MotionControlType::torque;
    motor_.torque_controller = TorqueControlType::voltage;

    // init() clamps the alignment voltage to voltage_limit, so the limit is
    // raised to the alignment voltage here and lowered after initFOC().
    motor_.voltage_limit = MOTOR_ALIGN_VOLTAGE;
    motor_.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;

    motor_.LPF_velocity.Tf = MOTOR_VEL_TF;

    if (motor_.init() != 1) return false;

    initialized_ = true;
    stop();

    return true;
}

bool MotorControl::startAlignment() {
    if (!initialized_) return false;
    if (!magnetOK()) return false;

    // Alignment at MOTOR_ALIGN_VOLTAGE (may rotate the shaft)
    motor_.voltage_limit = MOTOR_ALIGN_VOLTAGE;
    motor_.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
    motor_.enable();

    if (motor_.initFOC() != 1) {
        stop();
        return false;
    }

    // Operating limit
    motor_.voltage_limit = MOTOR_VOLTAGE_LIMIT;

    // Fresh angle/velocity, then hold the current position
    for (int i = 0; i < 20; i++) {
        motor_.loopFOC();
        motor_.move(0.0f);
    }
    targetRad_ = motor_.shaft_angle;

    resetController();
    active_ = true;

    return true;
}

void MotorControl::resetController() {
    integral_ = 0.0f;
    uq_ = 0.0f;
    lastUs_ = micros();

    loopCount_ = 0;
    loopWindowStartMs_ = millis();

    lastSatSign_ = 0;
    satFlips_ = 0;
    flipWindowStartMs_ = millis();
    unstable_ = false;
}

void MotorControl::loopFOC() {
    if (!active_) return;

    // ---- dt ----
    const uint32_t nowUs = micros();
    float dt = (nowUs - lastUs_) * 1e-6f;
    lastUs_ = nowUs;
    if (dt > 0.01f) dt = 0.01f;   // after a pause, don't integrate a jump

    // ---- control law (same as the bench) ----
    // shaft_angle and shaft_velocity were updated by the previous move().
    const float err = targetRad_ - motor_.shaft_angle;

    const float iLim = 0.5f * MOTOR_VOLTAGE_LIMIT;
    integral_ = clampf(integral_ + MOTOR_KI * err * dt, -iLim, iLim);

    uq_ = clampf(MOTOR_KP * err
                 - MOTOR_KD * motor_.shaft_velocity
                 + integral_,
                 -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);

    motor_.loopFOC();
    motor_.move(uq_);

    // ---- loop rate (1 s window) ----
    const uint32_t nowMs = millis();
    loopCount_++;
    if (nowMs - loopWindowStartMs_ >= 1000) {
        loopRateHz_ = loopCount_ * 1000 / (nowMs - loopWindowStartMs_);
        loopCount_ = 0;
        loopWindowStartMs_ = nowMs;
    }

    // ---- instability guard (same criterion as the bench) ----
    // >= 6 sign changes between saturated Uq values within 500 ms.
    if (fabsf(uq_) >= 0.9f * MOTOR_VOLTAGE_LIMIT) {
        const int sign = (uq_ > 0.0f) ? 1 : -1;
        if (lastSatSign_ != 0 && sign != lastSatSign_) satFlips_++;
        lastSatSign_ = sign;
    }
    if (nowMs - flipWindowStartMs_ >= 500) {
        if (satFlips_ >= 6) unstable_ = true;
        satFlips_ = 0;
        flipWindowStartMs_ = nowMs;
    }
}

void MotorControl::setTarget(float targetRad) {
    if (!active_) return;
    targetRad_ = targetRad;   // used by the next loopFOC()
}

void MotorControl::stop() {
    active_ = false;
    uq_ = 0.0f;
    integral_ = 0.0f;

    if (initialized_)
        motor_.disable();

    digitalWrite(MOTOR_ENABLE, LOW);
}

float MotorControl::getAngle() {
    if (!active_) return NAN;
    return motor_.shaft_angle;
}

float MotorControl::getTarget() const {
    return targetRad_;
}

bool MotorControl::isActive() const {
    return active_;
}