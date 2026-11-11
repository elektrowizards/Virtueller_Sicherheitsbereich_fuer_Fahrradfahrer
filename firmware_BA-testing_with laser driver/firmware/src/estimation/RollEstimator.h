
#pragma once
#include <Arduino.h>

class RollEstimator {
public:
    void update(float rollRad);
    void setReference();

    float getRoll() const;
    float getRelativeRoll() const;
    bool isValid() const;

private:
    float rollRad_ = NAN;
    float referenceRad_ = 0.0f;
    bool valid_ = false;
};
