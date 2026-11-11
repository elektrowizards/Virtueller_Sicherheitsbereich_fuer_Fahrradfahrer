
#include "ToF.h"
#include "config.h"
#include <math.h>

ToF::ToF() : sensor_(&Wire, -1) {}

bool ToF::begin(TwoWire& wire) {
    // This project uses the global Wire instance.
    if (&wire != &Wire) return false;

    sensor_.begin();

    if (sensor_.init() != 0) return false;

    if (sensor_.set_resolution(
        VL53L8CX_RESOLUTION_4X4
    ) != 0) return false;

    if (sensor_.set_ranging_frequency_hz(5) != 0)
        return false;

    initialized_ = true;
    healthy_ = true;
    return true;
}

bool ToF::start() {
    if (!initialized_) return false;

    distanceMm_ = NAN;

    if (sensor_.start_ranging() != 0) {
        healthy_ = false;
        return false;
    }

    ranging_ = true;
    return true;
}


bool ToF::update() {
    if (!ranging_) return false;

    // STEP 1: Check for new measurement
    uint8_t ready = 0;

    int status = sensor_.check_data_ready(&ready);

    if (status != 0) {
        Serial.print("[ToF] Data-ready error: ");
        Serial.println(status);
        healthy_ = false;
        return false;
    }

    if (!ready) {
        static uint32_t lastMessage = 0;

        if (millis() - lastMessage >= 1000) {
            lastMessage = millis();
            Serial.println("[ToF] No new frame ready");
        }

        return false;
    }

    Serial.println("[ToF] NEW FRAME RECEIVED");

    // STEP 2: Read measurement results
    VL53L8CX_ResultsData results;

    status = sensor_.get_ranging_data(&results);

    if (status != 0) {
        Serial.print("[ToF] Read error: ");
        Serial.println(status);
        healthy_ = false;
        return false;
    }

    // STEP 3: Inspect the four central zones
    const uint8_t centerZones[4] = {5, 6, 9, 10};

    float sum = 0.0f;
    int count = 0;

    for (uint8_t zone : centerZones) {

        uint16_t index =
            zone * VL53L8CX_NB_TARGET_PER_ZONE;

        uint8_t targets =
            results.nb_target_detected[zone];

        uint8_t targetStatus =
            results.target_status[index];

        int distance =
            results.distance_mm[index];

        Serial.print("Zone ");
        Serial.print(zone);

        Serial.print(" | targets: ");
        Serial.print(targets);

        Serial.print(" | status: ");
        Serial.print(targetStatus);

        Serial.print(" | distance: ");
        Serial.println(distance);

        if (targets < 1)
            continue;

        if (targetStatus != 5)
            continue;

        if (distance < MIN_HEIGHT_M * 1000.0f ||
            distance > MAX_HEIGHT_M * 1000.0f)
            continue;

        sum += distance;
        count++;
    }

    // STEP 4: Check how many zones passed
    Serial.print("[ToF] Valid zones: ");
    Serial.println(count);

    if (count < 2) {
        Serial.println("[ToF] Frame rejected");
        return false;
    }

    distanceMm_ = sum / count;

    Serial.print("[ToF] Accepted distance: ");
    Serial.println(distanceMm_);

    return true;
}


void ToF::stop() {
    if (!ranging_) return;

    if (sensor_.stop_ranging() != 0)
        healthy_ = false;

    ranging_ = false;
}

float ToF::getDistanceMm() const {
    return distanceMm_;
}

bool ToF::isRanging() const {
    return ranging_;
}

bool ToF::isHealthy() const {
    return healthy_;
}
