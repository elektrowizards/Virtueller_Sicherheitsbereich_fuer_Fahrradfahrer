
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

class IMU {
public:
    bool begin(TwoWire& wire);
    bool update();

    float getRoll() const;
    bool isFresh(uint32_t maxAgeMs) const;
    bool isHealthy() const;
    bool takeReset();

    // Recovery when no data arrives (e.g. after an MCU-only reset the
    // BNO085 keeps running and may not resume its report):
    //   level 0: re-enable the report, level >= 1: full re-init.
    // Returns true if the IMU accepted the command.
    bool recover(uint8_t level);

private:
    Adafruit_BNO08x bno_{-1};
    TwoWire* wire_ = nullptr;
    sh2_SensorValue_t event_{};

    float rollRad_ = NAN;
    uint32_t lastUpdateMs_ = 0;

    bool valid_ = false;
    bool healthy_ = false;
    bool resetDetected_ = false;

    bool enableReports();
};
