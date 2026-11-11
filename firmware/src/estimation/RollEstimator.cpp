#include "RollEstimator.h"
#include "config.h"
#include "control/Geometry.h"
#include <math.h>

void RollEstimator::update(float imuRollRad) {
    if (!isfinite(imuRollRad)) return;

    rightRollRad_ = Geometry::wrapPi(IMU_ROLL_SIGN * imuRollRad);
    valid_ = true;
}

void RollEstimator::setMountOffset(float offsetRad) {
    if (!isfinite(offsetRad)) return;
    offsetRad_ = Geometry::wrapPi(offsetRad);
    offsetValid_ = true;
}

void RollEstimator::clearMountOffset() {
    offsetRad_ = 0.0f;
    offsetValid_ = false;
}

float RollEstimator::getRightRoll() const {
    return valid_ ? rightRollRad_ : NAN;
}

float RollEstimator::getLineRoll() const {
    if (!valid_ || !offsetValid_) return NAN;
    return LINE_SIDE * Geometry::wrapPi(rightRollRad_ - offsetRad_);
}

bool RollEstimator::isValid() const {
    return valid_;
}
