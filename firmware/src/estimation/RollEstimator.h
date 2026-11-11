#pragma once
#include <Arduino.h>

// ============================================================================
//  RollEstimator
//
//  The BNO085 game rotation vector is gravity-referenced in roll/pitch
//  (only yaw drifts), so the roll is absolute.
//
//  rightRoll = wrapPi(IMU_ROLL_SIGN * imuRoll)        (+ = leaning right)
//  phi       = LINE_SIDE * wrapPi(rightRoll - mountOffset)
//                                                      (+ = toward line side)
//  mountOffset = rightRoll measured with the housing level (stored in
//  flash by the 'z' reference procedure).
// ============================================================================
class RollEstimator {
public:
    void update(float imuRollRad);

    void setMountOffset(float offsetRad);
    void clearMountOffset();
    bool hasMountOffset() const { return offsetValid_; }
    float getMountOffset() const { return offsetRad_; }

    // IMU roll in the right-positive convention, without mount offset
    float getRightRoll() const;

    // Absolute roll toward the line side; NAN without data or offset
    float getLineRoll() const;

    bool isValid() const;

private:
    float rightRollRad_ = NAN;
    float offsetRad_ = 0.0f;
    bool offsetValid_ = false;
    bool valid_ = false;
};
