#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// ============================================================================
//  AS5600Sensor - SimpleFOC sensor for the AS5600 with checked I2C reads.
//
//  SimpleFOC's MagneticSensorI2C does not check the I2C result: a failed
//  read returns 0xFF bytes, i.e. an angle of ~360 deg, and the motor is
//  then commutated for a wrong rotor position (it pushes the wrong way).
//  On the shared bus (BNO085, VL53L8CX) this made the motor run away.
//
//  Here a failed read keeps the last valid angle and is counted. The
//  application stops the motor after too many consecutive failures.
//  Reads the RAW ANGLE register 0x0C (12 bit), same as AS5600_I2C.
// ============================================================================
class AS5600Sensor : public Sensor {
public:
    explicit AS5600Sensor(uint8_t address) : address_(address) {}

    // Returns false if no valid angle could be read.
    bool init(TwoWire* wire);

    float getSensorAngle() override;

    uint32_t errorCount() const { return errors_; }
    uint32_t consecutiveErrors() const { return consecutive_; }

private:
    bool readRaw(uint16_t& raw);

    uint8_t address_;
    TwoWire* wire_ = nullptr;
    float lastAngle_ = 0.0f;
    uint32_t errors_ = 0;
    uint32_t consecutive_ = 0;
};
