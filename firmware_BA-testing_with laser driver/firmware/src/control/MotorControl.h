#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// ============================================================================
//  MotorControl
//
//  Position control with the law tuned and validated on the bench
//  ("Engineering review checklist"):
//
//      Uq = Kp * err - Kd * velocity + integral(Ki * err)
//
//  SimpleFOC runs in voltage-torque mode; this class computes Uq itself
//  inside loopFOC(), on every pass. setTarget() only stores the target.
//
//  Public interface is unchanged, so StateMachine needs no changes.
//  New: loopRateHz(), isUnstable(), getVoltageQ().
// ============================================================================
class MotorControl {
public:
    MotorControl();

    bool begin(TwoWire& wire);
    bool startAlignment();

    void loopFOC();                 // call as often as possible
    void setTarget(float targetRad);
    void stop();

    float getAngle();
    float getTarget() const;

    bool isActive() const;
    bool magnetOK();

    uint32_t loopRateHz() const { return loopRateHz_; }
    bool isUnstable() const { return unstable_; }
    float getVoltageQ() const { return uq_; }

private:
    void resetController();

    TwoWire* wire_ = nullptr;

    MagneticSensorI2C encoder_;
    BLDCMotor motor_;
    BLDCDriver3PWM driver_;

    bool initialized_ = false;
    bool active_ = false;

    float targetRad_ = 0.0f;

    // Controller state
    float integral_ = 0.0f;
    float uq_ = 0.0f;
    uint32_t lastUs_ = 0;

    // Loop-rate measurement
    uint32_t loopCount_ = 0;
    uint32_t loopWindowStartMs_ = 0;
    uint32_t loopRateHz_ = 0;

    // Instability guard: Uq flipping between saturated extremes
    int lastSatSign_ = 0;
    uint32_t satFlips_ = 0;
    uint32_t flipWindowStartMs_ = 0;
    bool unstable_ = false;
};