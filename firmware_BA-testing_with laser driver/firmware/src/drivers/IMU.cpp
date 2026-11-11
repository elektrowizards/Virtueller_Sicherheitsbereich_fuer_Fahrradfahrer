
#include "IMU.h"
#include "config.h"
#include <math.h>

bool IMU::enableReports() {
    return bno_.enableReport(
        SH2_GAME_ROTATION_VECTOR,
        IMU_REPORT_US
    );
}

bool IMU::begin(TwoWire& wire) {
    healthy_ = bno_.begin_I2C(
        BNO085_ADDRESS,
        &wire
    );

    if (!healthy_) return false;

    healthy_ = enableReports();
    return healthy_;
}

bool IMU::update() {
    if (!healthy_) return false;

    if (bno_.wasReset()) {
        resetDetected_ = true;
        valid_ = false;
        healthy_ = enableReports();

        if (!healthy_) return false;
    }

    if (!bno_.getSensorEvent(&event_))
        return false;

    if (event_.sensorId != SH2_GAME_ROTATION_VECTOR)
        return false;

    float qx = event_.un.gameRotationVector.i;
    float qy = event_.un.gameRotationVector.j;
    float qz = event_.un.gameRotationVector.k;
    float qw = event_.un.gameRotationVector.real;

    float roll = atan2f(
        2.0f * (qw * qx + qy * qz),
        1.0f - 2.0f * (qx*qx + qy*qy)
    );

    if (!isfinite(roll)) return false;

    rollRad_ = roll;
    lastUpdateMs_ = millis();
    valid_ = true;

    return true;
}

float IMU::getRoll() const {
    return rollRad_;
}

bool IMU::isFresh(uint32_t maxAgeMs) const {
    return valid_ &&
        (uint32_t)(millis() - lastUpdateMs_) <= maxAgeMs;
}

bool IMU::isHealthy() const {
    return healthy_;
}

bool IMU::takeReset() {
    bool reset = resetDetected_;
    resetDetected_ = false;
    return reset;
}
