#pragma once

// ============================================================================
//  Geometry (2-D, plane perpendicular to the roll axis)
//
//  World frame: y = lateral, positive toward the LINE side; z = up.
//  Origin = pivot projected onto the ground (bike: tyre contact line,
//  table: floor below the table edge).
//
//  phi   : housing roll, positive when leaning TOWARD the line side.
//  alpha : beam angle below horizontal, relative to the housing
//          (+ = pointing down).
//  theta : beam angle below horizontal in the world frame.
//          theta = alpha + phi
//          (leaning toward the line rotates an outward, downward-pointing
//           beam further down).
//
//  Rotation axis position (housing level: AXIS_OUTWARD_M toward the line,
//  AXIS_ABOVE_PIVOT_M above the pivot, pivot PIVOT_HEIGHT_M above ground):
//      y = r_out*cos(phi) + r_up*sin(phi)
//      z = p + r_up*cos(phi) - r_out*sin(phi)
//
//  Required world angle for the line at distance D:
//      theta(phi) = atan2(z, D - y)       (exact for LASER_BEAM_OFFSET_M = 0)
// ============================================================================
namespace Geometry {
    // Rotation-axis position in the world frame for roll phi
    void axisPosition(float phiRad, float& yM, float& zM);

    // World beam angle that puts the line at distanceM for roll phi
    float requiredWorldAngleRad(float phiRad, float distanceM);

    // theta0 = requiredWorldAngleRad(0, TARGET_DISTANCE_M)
    float nominalAngleRad();

    // Line position on the ground [m from pivot] for housing beam angle
    // alpha and roll phi. NAN if the beam does not hit the ground ahead
    // (world angle below ~2 deg).
    float predictedDistanceM(float alphaRad, float phiRad);

    float wrapPi(float angleRad);
    float wrapTwoPi(float angleRad);

    float radToDeg(float radians);
    float degToRad(float degrees);
}
