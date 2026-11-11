
#include "CalibrationStorage.h"
#include "config.h"
#include <EEPROM.h>
#include <math.h>
#include <stddef.h>

namespace {
    constexpr uint32_t MAGIC = 0xBA202611u;
    constexpr uint32_t VERSION = 1;
}

uint32_t CalibrationStorage::calculateChecksum(
    const Data& data
) {
    const uint8_t* bytes =
        reinterpret_cast<const uint8_t*>(&data);

    uint32_t hash = 2166136261u;

    for (size_t i = 0;
         i < offsetof(Data, checksum);
         i++) {

        hash = (hash ^ bytes[i]) * 16777619u;
    }

    return hash;
}

bool CalibrationStorage::saveHeight(float heightM) {
    if (!isfinite(heightM)) return false;

    if (heightM < MIN_HEIGHT_M ||
        heightM > MAX_HEIGHT_M) return false;

    Data data = {};
    data.magic = MAGIC;
    data.version = VERSION;
    data.heightM = heightM;
    data.checksum = calculateChecksum(data);

    EEPROM.put(0, data);

    float verify = NAN;
    return loadHeight(verify) &&
           fabsf(verify - heightM) < 0.001f;
}

bool CalibrationStorage::loadHeight(float& heightM) {
    Data data = {};
    EEPROM.get(0, data);

    if (data.magic != MAGIC ||
        data.version != VERSION ||
        data.checksum != calculateChecksum(data))
        return false;

    if (!isfinite(data.heightM) ||
        data.heightM < MIN_HEIGHT_M ||
        data.heightM > MAX_HEIGHT_M)
        return false;

    heightM = data.heightM;
    return true;
}
