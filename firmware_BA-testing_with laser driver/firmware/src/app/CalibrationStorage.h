
#pragma once
#include <Arduino.h>

class CalibrationStorage {
public:
    bool saveHeight(float heightM);
    bool loadHeight(float& heightM);

private:
    struct Data {
        uint32_t magic;
        uint32_t version;
        float heightM;
        uint32_t checksum;
    };

    static uint32_t calculateChecksum(const Data& data);
};
