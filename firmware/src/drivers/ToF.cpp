
#include "ToF.h"
#include "config.h"
#include <math.h>

ToF::ToF() : sensor_(&Wire, -1) {}

bool ToF::begin(TwoWire& wire) {
    // This project uses the global Wire instance.
    if (&wire != &Wire) return false;

    sensor_.begin();

    if (sensor_.init() != 0) return false;

    // 8x8 zones: the ground-plane fit needs the whole field of view
    if (sensor_.set_resolution(
        VL53L8CX_RESOLUTION_8X8
    ) != 0) return false;

    if (sensor_.set_ranging_frequency_hz(5) != 0)
        return false;

    initialized_ = true;
    healthy_ = true;
    return true;
}

bool ToF::start() {
    if (!initialized_) return false;
    if (ranging_) return true;

    distanceMm_ = NAN;
    validCount_ = 0;

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

    // STEP 2: Read measurement results (large struct: keep it off the stack)
    static VL53L8CX_ResultsData results;

    status = sensor_.get_ranging_data(&results);

    if (status != 0) {
        Serial.print("[ToF] Read error: ");
        Serial.println(status);
        healthy_ = false;
        return false;
    }

    // STEP 3: Store every zone. Status 5 = valid, 9 = valid with a large
    // pulse (both used by ST as valid ranges).
    validCount_ = 0;
    for (uint8_t zone = 0; zone < ZONES; zone++) {
        const uint16_t index = zone * VL53L8CX_NB_TARGET_PER_ZONE;
        const uint8_t st = results.target_status[index];
        const int16_t d = results.distance_mm[index];

        zoneMm_[zone] = d;
        zoneValid_[zone] = results.nb_target_detected[zone] >= 1 &&
                           (st == 5 || st == 9) && d > 0;
        if (zoneValid_[zone]) validCount_++;
    }

    // Centre zones of the 8x8 grid (diagnostics)
    const uint8_t centerZones[4] = {27, 28, 35, 36};
    float sum = 0.0f;
    int count = 0;
    for (uint8_t zone : centerZones) {
        if (!zoneValid_[zone]) continue;
        sum += zoneMm_[zone];
        count++;
    }
    distanceMm_ = count > 0 ? sum / count : NAN;

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
