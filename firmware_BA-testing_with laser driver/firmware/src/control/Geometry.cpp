
#include "Geometry.h"
#include "config.h"
#include <math.h>

namespace {
    constexpr float PI_F = 3.14159265358979323846f;
}

float Geometry::nominalAngleRad(float heightM) {
    // Required downward laser angle on level ground
    return atan2f(heightM, TARGET_DISTANCE_M);
}

float Geometry::motorTargetRad(
    float alignedMotorRad,
    float heightM,
    float relativeRollRad
) {
    float theta0 = nominalAngleRad(heightM);

    // Keep world-frame beam angle constant
    float correctedAngle = theta0 - relativeRollRad;

    // In ALIGN, alignedMotorRad is the physical theta0 reference
    float correction = correctedAngle - theta0;

    return alignedMotorRad +
           MOTOR_ROLL_SIGN * correction;
}

float Geometry::radToDeg(float radians) {
    return radians * 180.0f / PI_F;
}

float Geometry::degToRad(float degrees) {
    return degrees * PI_F / 180.0f;
}
