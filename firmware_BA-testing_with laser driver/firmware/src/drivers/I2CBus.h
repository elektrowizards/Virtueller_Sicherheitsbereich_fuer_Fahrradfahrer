
#pragma once
#include <Arduino.h>
#include <Wire.h>

class I2CBus {
public:
    void begin();
    void scan();
    TwoWire& getWire();
};
