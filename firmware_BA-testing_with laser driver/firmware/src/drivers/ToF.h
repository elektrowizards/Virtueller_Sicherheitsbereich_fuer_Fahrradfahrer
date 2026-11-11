
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h>

class ToF {
public:
    ToF();

    bool begin(TwoWire& wire);
    bool start();
    bool update();
    void stop();

    float getDistanceMm() const;
    bool isRanging() const;
    bool isHealthy() const;

private:
    VL53L8CX sensor_;
    float distanceMm_ = NAN;

    bool initialized_ = false;
    bool ranging_ = false;
    bool healthy_ = true;
};
