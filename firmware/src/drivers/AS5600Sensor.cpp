#include "AS5600Sensor.h"

namespace {
    constexpr uint8_t RAW_ANGLE_REG = 0x0C;
    constexpr float COUNTS = 4096.0f;
}

bool AS5600Sensor::readRaw(uint16_t& raw) {
    wire_->beginTransmission(address_);
    wire_->write(RAW_ANGLE_REG);
    if (wire_->endTransmission(false) != 0) return false;

    if (wire_->requestFrom(address_, (uint8_t)2) != 2) return false;

    const int msb = wire_->read();
    const int lsb = wire_->read();
    if (msb < 0 || lsb < 0) return false;

    raw = (uint16_t)(((msb & 0x0F) << 8) | (lsb & 0xFF));
    return true;
}

bool AS5600Sensor::init(TwoWire* wire) {
    wire_ = wire;

    // A valid first angle is required before Sensor::init() samples it.
    uint16_t raw = 0;
    bool ok = false;
    for (int i = 0; i < 10 && !ok; i++) {
        ok = readRaw(raw);
        if (!ok) delay(2);
    }
    if (!ok) return false;

    lastAngle_ = raw / COUNTS * _2PI;
    errors_ = 0;
    consecutive_ = 0;

    Sensor::init();
    return true;
}

float AS5600Sensor::getSensorAngle() {
    uint16_t raw = 0;
    if (readRaw(raw)) {
        consecutive_ = 0;
        lastAngle_ = raw / COUNTS * _2PI;
    } else {
        errors_++;
        consecutive_++;
    }
    return lastAngle_;
}
