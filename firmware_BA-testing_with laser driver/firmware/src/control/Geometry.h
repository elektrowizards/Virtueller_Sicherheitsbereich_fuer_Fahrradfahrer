
#pragma once

namespace Geometry {
    float nominalAngleRad(float heightM);

    float motorTargetRad(
        float alignedMotorRad,
        float heightM,
        float relativeRollRad
    );

    float radToDeg(float radians);
    float degToRad(float degrees);
}
