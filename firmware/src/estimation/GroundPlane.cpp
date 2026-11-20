#include "GroundPlane.h"
#include "config.h"
#include <math.h>

namespace {
    constexpr int ZONES = 64;
    constexpr float DEG = 3.14159265358979f / 180.0f;
    constexpr int RANSAC_TRIALS = 120;

    struct P3 { float x, y, z; };

    // Zone i: column -> x, row -> y, pinhole model (tan units)
    void zoneTan(int i, float& tx, float& ty) {
        const float k = tanf(0.5f * TOF_FOV_DEG * DEG) / 4.0f;  // 4 zones per half FoV
        tx = ((float)(i % 8) - 3.5f) * k;
        ty = ((float)(i / 8) - 3.5f) * k;
    }

    uint32_t nextRand(uint32_t& s) {
        s = s * 1664525u + 1013904223u;
        return s >> 8;
    }

    // Least squares z = a*x + b*y + c over the points marked in 'use'
    bool fitLS(const P3* p, const bool* use, float& a, float& b, float& c) {
        double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, n = 0;
        double sxz = 0, syz = 0, sz = 0;
        for (int i = 0; i < ZONES; i++) {
            if (!use[i]) continue;
            const double x = p[i].x, y = p[i].y, z = p[i].z;
            sxx += x * x; sxy += x * y; syy += y * y;
            sx += x; sy += y; n += 1.0;
            sxz += x * z; syz += y * z; sz += z;
        }
        if (n < 3.0) return false;
        // | sxx sxy sx | |a|   |sxz|
        // | sxy syy sy | |b| = |syz|
        // | sx  sy  n  | |c|   |sz |
        const double det = sxx * (syy * n - sy * sy)
                         - sxy * (sxy * n - sy * sx)
                         + sx * (sxy * sy - syy * sx);
        if (fabs(det) < 1e-12) return false;
        a = (float)((sxz * (syy * n - sy * sy) - sxy * (syz * n - sy * sz)
                     + sx * (syz * sy - syy * sz)) / det);
        b = (float)((sxx * (syz * n - sz * sy) - sxz * (sxy * n - sy * sx)
                     + sx * (sxy * sz - syz * sx)) / det);
        c = (float)((sz - (double)a * sx - (double)b * sy) / n);   // third normal equation
        return isfinite(a) && isfinite(b) && isfinite(c);
    }

    float perpResidual(const P3& q, float a, float b, float c) {
        return fabsf(a * q.x + b * q.y - q.z + c) / sqrtf(a * a + b * b + 1.0f);
    }

    // Keep the points within TOF_PLANE_INLIER_M of z = a*x + b*y + c
    uint8_t selectInliers(const P3* p, const bool* valid, bool* use,
                          float a, float b, float c) {
        uint8_t n = 0;
        for (int i = 0; i < ZONES; i++) {
            use[i] = valid[i] && perpResidual(p[i], a, b, c) < TOF_PLANE_INLIER_M;
            if (use[i]) n++;
        }
        return n;
    }
}

GroundPlane::Result GroundPlane::evaluate(const int16_t* distanceMm,
                                          const bool* valid) {
    Result r;

    // ---- 1. zones -> points (sensor frame) ----
    P3 p[ZONES];
    bool ok[ZONES];
    int idx[ZONES];
    int m = 0;
    for (int i = 0; i < ZONES; i++) {
        ok[i] = false;
        if (!valid[i]) continue;
        const float d = distanceMm[i] / 1000.0f;
        if (d < 0.10f || d > 4.0f) continue;
        float tx, ty;
        zoneTan(i, tx, ty);
        const float s = TOF_DISTANCE_IS_RADIAL ? d / sqrtf(tx * tx + ty * ty + 1.0f) : d;
        p[i] = {tx * s, ty * s, s};
        ok[i] = true;
        idx[m++] = i;
    }
    r.zones = (uint8_t)m;
    if (m < TOF_MIN_PLANE_ZONES) {
        r.reason = "too few valid zones (sunlight? too far?)";
        return r;
    }

    // ---- 2a. RANSAC: plane through 3 zones with the most support ----
    // Candidates must face the sensor roughly like the ground does
    // (rejects the side of a bag or a rack strut).
    uint32_t seed = 0x5EEDu;
    int best = 0;
    float bn[3] = {0, 0, 1}, bc = 0;
    for (int t = 0; t < RANSAC_TRIALS; t++) {
        const int i = idx[nextRand(seed) % m];
        const int j = idx[nextRand(seed) % m];
        const int k = idx[nextRand(seed) % m];
        if (i == j || j == k || i == k) continue;
        const P3 u = {p[j].x - p[i].x, p[j].y - p[i].y, p[j].z - p[i].z};
        const P3 v = {p[k].x - p[i].x, p[k].y - p[i].y, p[k].z - p[i].z};
        float n0 = u.y * v.z - u.z * v.y;
        float n1 = u.z * v.x - u.x * v.z;
        float n2 = u.x * v.y - u.y * v.x;
        const float len = sqrtf(n0 * n0 + n1 * n1 + n2 * n2);
        if (len < 1e-6f) continue;
        n0 /= len; n1 /= len; n2 /= len;
        float c = n0 * p[i].x + n1 * p[i].y + n2 * p[i].z;
        if (c < 0.0f) { n0 = -n0; n1 = -n1; n2 = -n2; c = -c; }  // normal: sensor -> plane
        if (n2 <= 0.0f) continue;
        const float tilt = acosf(fminf(n2, 1.0f)) / DEG;
        if (fabsf(tilt - TOF_TILT_DEG) > TOF_TILT_TOLERANCE_DEG + 20.0f) continue;
        int cnt = 0;
        for (int q = 0; q < m; q++) {
            const P3& w = p[idx[q]];
            if (fabsf(n0 * w.x + n1 * w.y + n2 * w.z - c) < TOF_PLANE_INLIER_M) cnt++;
        }
        if (cnt > best) { best = cnt; bn[0] = n0; bn[1] = n1; bn[2] = n2; bc = c; }
    }
    if (best < TOF_MIN_PLANE_ZONES) {
        r.reason = "no ground plane found";
        return r;
    }

    // ---- 2b. least squares on the inliers, twice ----
    bool use[ZONES];
    for (int i = 0; i < ZONES; i++) {
        use[i] = ok[i] && fabsf(bn[0] * p[i].x + bn[1] * p[i].y + bn[2] * p[i].z - bc) < TOF_PLANE_INLIER_M;
    }
    float a, b, c;
    for (int pass = 0; pass < 2; pass++) {
        if (!fitLS(p, use, a, b, c)) {
            r.reason = "plane fit failed";
            return r;
        }
        r.inliers = selectInliers(p, ok, use, a, b, c);
        if (r.inliers < TOF_MIN_PLANE_ZONES) {
            r.reason = "too few zones on the ground plane";
            return r;
        }
    }
    double ss = 0.0;
    for (int i = 0; i < ZONES; i++) {
        if (use[i]) {
            const float e = perpResidual(p[i], a, b, c);
            ss += (double)e * e;
        }
    }
    r.rmsM = (float)sqrt(ss / r.inliers);

    // Plane a*x + b*y - z + c = 0; c > 0 = in front of the sensor
    const float N = sqrtf(a * a + b * b + 1.0f);
    if (c <= 0.0f) {
        r.reason = "plane behind the sensor";
        return r;
    }

    // ---- 3. sensor height and "up" in the sensor frame ----
    r.sensorHeightM = c / N;
    const float upX = a / N, upY = b / N, upZ = -1.0f / N;
    r.tiltDeg = acosf(fminf(-upZ, 1.0f)) / DEG;

    // Zone grid -> board frame (x = up the bevel, y = along housing X,
    // z = optical axis). The bevel tilt shows up on one grid axis: that
    // one is board x, its sign follows from "up points up the bevel".
    // The other axis' sign is TOF_GRID_ALONG_X_SIGN (only the pitch term).
    float ux, uy;
    if (fabsf(upX) >= fabsf(upY)) {
        ux = fabsf(upX);
        uy = TOF_GRID_ALONG_X_SIGN * upY;
    } else {
        ux = fabsf(upY);
        uy = TOF_GRID_ALONG_X_SIGN * upX;
    }
    const float uz = upZ;

    const float tiltX = atan2f(ux, -uz) / DEG;          // tilt across the bevel
    // world "up" leans toward -X in the housing when its top tilts rearward
    r.pitchDeg = -atan2f(uy, sqrtf(ux * ux + uz * uz)) / DEG;
    // Leaning toward the line turns a sensor on the other side further
    // outward (tilt grows), one on the line side further down.
    r.rollDeg = (tiltX - TOF_TILT_DEG) * (-TOF_SIDE * LINE_SIDE);

    if (fabsf(tiltX - TOF_TILT_DEG) > TOF_TILT_TOLERANCE_DEG) {
        r.reason = "ground not at the expected angle";
        return r;
    }

    // ---- 4. sensor -> pivot offset (housing frame) in the board frame ----
    const float t = TOF_TILT_DEG * DEG;
    const float ox = TOF_SIDE * TOF_TO_PIVOT_Y_M * cosf(t) + TOF_TO_PIVOT_Z_M * sinf(t);
    const float oy = TOF_TO_PIVOT_X_M;
    const float oz = TOF_SIDE * TOF_TO_PIVOT_Y_M * sinf(t) - TOF_TO_PIVOT_Z_M * cosf(t);
    r.pivotHeightM = r.sensorHeightM + ox * ux + oy * uy + oz * uz;

    // ---- 5. r_up (Geometry: z = r_up*cos(phi) - r_out*sin(phi)) ----
    const float phi = r.rollDeg * DEG;
    r.axisAbovePivotM = (r.pivotHeightM + AXIS_OUTWARD_M * sinf(phi)) / cosf(phi);

    if (r.rmsM > TOF_PLANE_MAX_RMS_M) {
        r.reason = "ground plane too noisy";
        return r;
    }
    if (!(r.axisAbovePivotM >= MIN_HEIGHT_M && r.axisAbovePivotM <= MAX_HEIGHT_M)) {
        r.reason = "height out of range";
        return r;
    }

    r.ok = true;
    r.reason = "ok";
    return r;
}
