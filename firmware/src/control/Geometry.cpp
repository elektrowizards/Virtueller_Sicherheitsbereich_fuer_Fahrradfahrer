#include "Geometry.h"
#include "config.h"
#include <math.h>

namespace {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float TWO_PI_F = 2.0f * PI_F;
    constexpr float MIN_HIT_ANGLE_RAD = 2.0f * PI_F / 180.0f;

    // Beam origin = rotation axis + perpendicular beam offset.
    // Beam direction (y,z) = (cos t, -sin t); its "up" normal is
    // (sin t, cos t).
    void beamOrigin(float phiRad, float thetaRad, float& y, float& z) {
        Geometry::axisPosition(phiRad, y, z);
        y += LASER_BEAM_OFFSET_M * sinf(thetaRad);
        z += LASER_BEAM_OFFSET_M * cosf(thetaRad);
    }
}

void Geometry::axisPosition(float phiRad, float& yM, float& zM) {
    const float s = sinf(phiRad);
    const float c = cosf(phiRad);
    yM = AXIS_OUTWARD_M * c + AXIS_ABOVE_PIVOT_M * s;
    zM = PIVOT_HEIGHT_M + AXIS_ABOVE_PIVOT_M * c - AXIS_OUTWARD_M * s;
}

float Geometry::requiredWorldAngleRad(float phiRad, float distanceM) {
    float y, z;
    axisPosition(phiRad, y, z);
    float theta = atan2f(z, distanceM - y);

    // With a perpendicular beam offset the origin depends on theta:
    // a few fixed-point iterations converge (offset << distance).
    if (LASER_BEAM_OFFSET_M != 0.0f) {
        for (int i = 0; i < 4; i++) {
            beamOrigin(phiRad, theta, y, z);
            theta = atan2f(z, distanceM - y);
        }
    }
    return theta;
}

float Geometry::nominalAngleRad() {
    return requiredWorldAngleRad(0.0f, TARGET_DISTANCE_M);
}

float Geometry::predictedDistanceM(float alphaRad, float phiRad) {
    if (!isfinite(alphaRad) || !isfinite(phiRad)) return NAN;

    const float theta = alphaRad + phiRad;
    if (theta < MIN_HIT_ANGLE_RAD || theta > 0.5f * PI_F) return NAN;

    float y, z;
    beamOrigin(phiRad, theta, y, z);
    if (z <= 0.0f) return NAN;

    return y + z / tanf(theta);
}

float Geometry::wrapPi(float a) {
    while (a > PI_F) a -= TWO_PI_F;
    while (a <= -PI_F) a += TWO_PI_F;
    return a;
}

float Geometry::wrapTwoPi(float a) {
    while (a >= TWO_PI_F) a -= TWO_PI_F;
    while (a < 0.0f) a += TWO_PI_F;
    return a;
}

float Geometry::radToDeg(float radians) {
    return radians * 180.0f / PI_F;
}

float Geometry::degToRad(float degrees) {
    return degrees * PI_F / 180.0f;
}
