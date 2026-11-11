#include <Arduino.h>

#include "app/StateMachine.h"

// ==========================================
// BACHELORARBEIT
// Bicycle Laser Stabilization System
// NUCLEO-F446RE
// ==========================================

// Main application object
StateMachine application;

// ==========================================
// SETUP
// ==========================================

void setup() {

    // Initialize serial communication
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("================================");
    Serial.println(" BICYCLE LASER STABILIZATION");
    Serial.println(" NUCLEO-F446RE / SimpleFOC");
    Serial.println("================================");

    Serial.println("Starting complete system...");
    Serial.println();

    // Initialize complete application:
    // - I2C bus
    // - BNO085 IMU
    // - VL53L8CX ToF
    // - AS5600 encoder
    // - FIT1035 motor + DRI0058 driver
    // - Button and LEDs
    // - State machine
    application.begin();
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

    // Execute application state machine
    // and associated control functions
    application.update();
}