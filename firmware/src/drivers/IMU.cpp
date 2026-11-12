
#include "IMU.h"
#include "config.h"
#include <math.h>

bool IMU::enableReports() {
    return bno_.enableReport(
        SH2_GAME_ROTATION_VECTOR,
        IMU_REPORT_US
    );
}

// RST low 10 ms, then wait until the BNO085 has booted before I2C.
void IMU::hardwareReset() {
    if (BNO085_RESET_PIN < 0) return;

    pinMode(BNO085_RESET_PIN, OUTPUT);
    digitalWrite(BNO085_RESET_PIN, LOW);
    delay(10);
    digitalWrite(BNO085_RESET_PIN, HIGH);
    delay(BNO085_BOOT_MS);
}

bool IMU::begin(TwoWire& wire) {
    wire_ = &wire;
    hardwareReset();
    healthy_ = bno_.begin_I2C(
        BNO085_ADDRESS,
        &wire
    );

    if (!healthy_) return false;

    healthy_ = enableReports();
    return healthy_;
}

// Boot diagnostics: the first calls print each step (flushed), so a hang
// shows exactly which library call never returns.
static uint8_t s_traceCalls = 0;
static void trace(const char* step) {
    if (s_traceCalls >= 3) return;
    Serial.print(step);
    Serial.flush();
}

bool IMU::update() {
    if (!healthy_) return false;

    trace("[a:wasReset ");

    if (bno_.wasReset()) {
        resetEvents_++;
        resetDetected_ = true;
        valid_ = false;
        trace("b:enableReport ");
        healthy_ = enableReports();

        if (!healthy_) {
            trace("FAILED] ");
            s_traceCalls++;
            return false;
        }
    }

    trace("c:getSensorEvent ");
    const bool got = bno_.getSensorEvent(&event_);
    trace(got ? "d:event] " : "d:none] ");
    if (s_traceCalls < 3) s_traceCalls++;

    if (!got)
        return false;

    if (event_.sensorId != SH2_GAME_ROTATION_VECTOR) {
        otherEvents_++;
        return false;
    }

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

bool IMU::recover(uint8_t level) {
    if (!wire_) return false;

    valid_ = false;

    if (level == 0) {
        healthy_ = enableReports();
        return healthy_;
    }

    // The SH-2/SHTP layer has a single instance that begin_I2C() does not
    // release: close it first, otherwise every re-init fails.
    sh2_close();

    hardwareReset();
    healthy_ = bno_.begin_I2C(BNO085_ADDRESS, wire_) && enableReports();
    return healthy_;
}

uint32_t IMU::takeOtherEvents() {
    uint32_t n = otherEvents_;
    otherEvents_ = 0;
    return n;
}

uint32_t IMU::takeResetEvents() {
    uint32_t n = resetEvents_;
    resetEvents_ = 0;
    return n;
}
