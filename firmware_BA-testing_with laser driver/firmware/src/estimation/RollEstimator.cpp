
#include "RollEstimator.h"
#include "config.h"
#include <math.h>

namespace {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float TWO_PI_F = 2.0f * PI_F;

    float wrapPi(float angle) {
        while (angle > PI_F) angle -= TWO_PI_F;
        while (angle < -PI_F) angle += TWO_PI_F;
        return angle;
    }
}

void RollEstimator::update(float rollRad) {
    if (!isfinite(rollRad)) return;

    rollRad_ = wrapPi(IMU_ROLL_SIGN * rollRad);
    valid_ = true;
}

void RollEstimator::setReference() {
    if (valid_)
        referenceRad_ = rollRad_;
}

float RollEstimator::getRoll() const {
    return rollRad_;
}

float RollEstimator::getRelativeRoll() const {
    if (!valid_) return NAN;

    return wrapPi(rollRad_ - referenceRad_);
}

bool RollEstimator::isValid() const {
    return valid_;
}
