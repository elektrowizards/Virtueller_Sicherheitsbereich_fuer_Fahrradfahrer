
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "config.h"

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

    // Diagnostics: events since the last call (then reset to 0)
    uint32_t takeOtherEvents();   // sensor events that were not the RV
    uint32_t takeResetEvents();   // reset notifications from the BNO085

private:
    // The reset pin is driven by IMU::hardwareReset() (with a proper boot
    // wait), not by the library (it only waits 10 ms after the pulse).
    Adafruit_BNO08x bno_{-1};
    TwoWire* wire_ = nullptr;
    sh2_SensorValue_t event_{};

    float rollRad_ = NAN;
    uint32_t lastUpdateMs_ = 0;

    bool valid_ = false;
    bool healthy_ = false;
    bool resetDetected_ = false;

    uint32_t otherEvents_ = 0;
    uint32_t resetEvents_ = 0;

    bool enableReports();
    void hardwareReset();
};
