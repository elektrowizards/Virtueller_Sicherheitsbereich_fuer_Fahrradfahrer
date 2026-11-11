
#include "I2CBus.h"
#include "config.h"

void I2CBus::begin() {
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);

    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);
}

TwoWire& I2CBus::getWire() {
    return Wire;
}

void I2CBus::scan() {
    Serial.println("Scanning I2C bus...");

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);

        if (Wire.endTransmission() == 0) {
            Serial.print("Found 0x");
            if (address < 16) Serial.print('0');
            Serial.println(address, HEX);
        }
    }
}
