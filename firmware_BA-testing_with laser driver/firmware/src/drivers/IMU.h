
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

private:
    Adafruit_BNO08x bno_{-1};
    sh2_SensorValue_t event_{};

    float rollRad_ = NAN;
    uint32_t lastUpdateMs_ = 0;

    bool valid_ = false;
    bool healthy_ = false;
    bool resetDetected_ = false;

    bool enableReports();
};
