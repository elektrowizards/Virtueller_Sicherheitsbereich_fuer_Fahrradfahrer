
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h>

// VL53L8CX in 8x8 mode (64 zones, 45 x 45 deg). update() stores every
// zone; the ground-plane fit (estimation/GroundPlane) turns a frame into
// the laser axis height.
class ToF {
public:
    static constexpr uint8_t ZONES = 64;

    ToF();

    bool begin(TwoWire& wire);
    bool start();
    bool update();
    void stop();

    // Last frame, ST zone order. A zone is valid with target status 5 or 9.
    const int16_t* zoneDistancesMm() const { return zoneMm_; }
    const bool* zoneValid() const { return zoneValid_; }
    uint8_t validZones() const { return validCount_; }

    // Mean of the 4 centre zones (diagnostics), NAN if none valid
    float getDistanceMm() const;
    bool isRanging() const;
    bool isHealthy() const;

private:
    VL53L8CX sensor_;
    float distanceMm_ = NAN;

    int16_t zoneMm_[ZONES] = {};
    bool zoneValid_[ZONES] = {};
    uint8_t validCount_ = 0;

    bool initialized_ = false;
    bool ranging_ = false;
    bool healthy_ = true;
};
