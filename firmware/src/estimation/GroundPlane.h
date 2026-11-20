#pragma once
#include <Arduino.h>

// ============================================================================
//  GroundPlane - laser axis height from one VL53L8CX 8x8 frame
//
//  enclosure_v3: the sensor sits in the bevel on the foot's lower RIGHT
//  edge and looks down and to the right, past the rear wheel, TOF_TILT_DEG
//  from straight down. Calibrate with the unit mounted and the bike upright.
//
//  1. Each valid zone becomes a 3-D point p = d * ray (sensor frame:
//     z = optical axis, x/y = zone columns/rows, pinhole model over the
//     TOF_FOV_DEG x TOF_FOV_DEG field of view).
//  2. RANSAC + least squares fit the ground plane. Zones that see the
//     frame, a rack or a bag do not lie on it and are dropped.
//  3. sensorHeight = perpendicular distance sensor -> plane. It does not
//     depend on the zone order, the printed bevel angle, the bike lean or
//     the seatpost tilt.
//  4. The plane normal is "up" in the sensor frame. The fixed vector
//     sensor -> laser pivot (CAD, config.h) projected onto it gives the
//     pivot height above the ground.
//  5. The housing roll seen in the same plane converts that height into
//     r_up (axis above the tyre contact line, housing level) for Geometry.
// ============================================================================
namespace GroundPlane {

    struct Result {
        bool ok = false;
        const char* reason = "no data"; // why ok == false
        uint8_t zones = 0;              // valid zones in the frame
        uint8_t inliers = 0;            // zones on the fitted plane
        float rmsM = NAN;               // plane fit RMS (perpendicular)
        float sensorHeightM = NAN;      // sensor -> ground, perpendicular
        float tiltDeg = NAN;            // optical axis from straight down
        float rollDeg = NAN;            // housing roll, + = toward the line
        float pitchDeg = NAN;           // housing top tilted rearward = + (check once)
        float pivotHeightM = NAN;       // laser pivot above ground (vertical)
        float axisAbovePivotM = NAN;    // r_up for Geometry (housing level)
    };

    // distanceMm / valid: 64 zones in the driver's (ST) order
    Result evaluate(const int16_t* distanceMm, const bool* valid);
}
