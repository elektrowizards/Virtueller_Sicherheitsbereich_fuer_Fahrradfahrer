#pragma once
#include <Arduino.h>

// ============================================================================
//  CalibrationStorage - mechanical reference in emulated EEPROM (flash)
//
//  Layout v2 (EEPROM offset 0, 24 bytes):
//    0  uint32 magic            0xBA202611
//    4  uint32 version          2
//    8  float  rawHorizontalRad AS5600 RAW angle (reg 0x0C, [0, 2pi)) at
//                               which the beam is horizontal rel. to housing
//   12  float  imuMountOffsetRad IMU roll (right-positive) with housing level
//   16  float  refDistanceM     floor mark used when the reference was taken
//   20  uint32 checksum         FNV-1a over bytes 0..19
//
//  Version 1 (saved height) is recognised and reported as "no reference".
//
//  save() BLOCKS for ~1-2 s (one 128 KB flash sector erase on the F446,
//  CPU stalls). Call it only with the motor disabled.
// ============================================================================
class CalibrationStorage {
public:
    struct Reference {
        float rawHorizontalRad;
        float imuMountOffsetRad;
        float refDistanceM;
    };

    enum class LoadResult { OK, EMPTY, OLD_VERSION, CORRUPT };

    bool save(const Reference& ref);
    LoadResult load(Reference& ref);

    static const char* resultText(LoadResult r);

private:
    struct Data {
        uint32_t magic;
        uint32_t version;
        float rawHorizontalRad;
        float imuMountOffsetRad;
        float refDistanceM;
        uint32_t checksum;
    };

    static uint32_t calculateChecksum(const Data& data);
};
