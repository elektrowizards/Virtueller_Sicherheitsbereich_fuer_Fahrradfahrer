#include "CalibrationStorage.h"
#include "config.h"
#include <EEPROM.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

namespace {
    constexpr uint32_t MAGIC = 0xBA202611u;
    constexpr uint32_t VERSION = 2;
    constexpr float TWO_PI_F = 6.28318530717958647692f;
    constexpr float PI_F = 3.14159265358979323846f;
}

uint32_t CalibrationStorage::calculateChecksum(const Data& data) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
    uint32_t hash = 2166136261u;

    for (size_t i = 0; i < offsetof(Data, checksum); i++)
        hash = (hash ^ bytes[i]) * 16777619u;

    return hash;
}

bool CalibrationStorage::save(const Reference& ref) {
    if (!isfinite(ref.rawHorizontalRad) ||
        ref.rawHorizontalRad < 0.0f || ref.rawHorizontalRad >= TWO_PI_F ||
        !isfinite(ref.imuMountOffsetRad) ||
        fabsf(ref.imuMountOffsetRad) > PI_F ||
        !isfinite(ref.refDistanceM) || ref.refDistanceM <= 0.0f)
        return false;

    Data data = {};
    data.magic = MAGIC;
    data.version = VERSION;
    data.rawHorizontalRad = ref.rawHorizontalRad;
    data.imuMountOffsetRad = ref.imuMountOffsetRad;
    data.refDistanceM = ref.refDistanceM;
    data.checksum = calculateChecksum(data);

    // EEPROM.put() would erase+program the whole flash sector once PER
    // BYTE (stm32duino eeprom_write_byte flushes every time). Fill the RAM
    // buffer, change all bytes, flush once.
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
    eeprom_buffer_fill();
    for (size_t i = 0; i < sizeof(Data); i++)
        eeprom_buffered_write_byte(i, bytes[i]);
    eeprom_buffer_flush();

    Reference verify = {};
    return load(verify) == LoadResult::OK &&
           verify.rawHorizontalRad == ref.rawHorizontalRad &&
           verify.imuMountOffsetRad == ref.imuMountOffsetRad &&
           verify.refDistanceM == ref.refDistanceM;
}

CalibrationStorage::LoadResult CalibrationStorage::load(Reference& ref) {
    Data data = {};
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&data);

    eeprom_buffer_fill();
    for (size_t i = 0; i < sizeof(Data); i++)
        bytes[i] = eeprom_buffered_read_byte(i);

    if (data.magic != MAGIC) return LoadResult::EMPTY;
    if (data.version != VERSION) return LoadResult::OLD_VERSION;
    if (data.checksum != calculateChecksum(data)) return LoadResult::CORRUPT;

    if (!isfinite(data.rawHorizontalRad) ||
        data.rawHorizontalRad < 0.0f || data.rawHorizontalRad >= TWO_PI_F ||
        !isfinite(data.imuMountOffsetRad) ||
        fabsf(data.imuMountOffsetRad) > PI_F ||
        !isfinite(data.refDistanceM) || data.refDistanceM <= 0.0f)
        return LoadResult::CORRUPT;

    ref.rawHorizontalRad = data.rawHorizontalRad;
    ref.imuMountOffsetRad = data.imuMountOffsetRad;
    ref.refDistanceM = data.refDistanceM;
    return LoadResult::OK;
}

const char* CalibrationStorage::resultText(LoadResult r) {
    switch (r) {
        case LoadResult::OK:          return "OK";
        case LoadResult::EMPTY:       return "no data";
        case LoadResult::OLD_VERSION: return "old format (v1 height), no reference";
        case LoadResult::CORRUPT:     return "corrupt (checksum/range)";
    }
    return "?";
}
