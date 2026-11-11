#include <Arduino.h>

#include "config.h"
#include "app/StateMachine.h"

// Main application object. It owns all drivers, including the laser.
StateMachine application;

// ==========================================
// RESET CAUSE (STM32 RCC->CSR flags)
// Prints why the board started, then clears the flags.
// Order matters: a power-on also sets PIN and BOR.
// ==========================================

static void printResetCause() {
    const uint32_t csr = RCC->CSR;

    Serial.print("Reset cause: ");

    if (csr & RCC_CSR_PORRSTF)       Serial.println("POWER-ON (board was unpowered)");
    else if (csr & RCC_CSR_BORRSTF)  Serial.println("BROWN-OUT (supply voltage dipped!)");
    else if (csr & RCC_CSR_IWDGRSTF) Serial.println("INDEPENDENT WATCHDOG");
    else if (csr & RCC_CSR_WWDGRSTF) Serial.println("WINDOW WATCHDOG");
    else if (csr & RCC_CSR_SFTRSTF)  Serial.println("SOFTWARE (NVIC_SystemReset / upload)");
    else if (csr & RCC_CSR_LPWRRSTF) Serial.println("LOW-POWER");
    else if (csr & RCC_CSR_PINRSTF)  Serial.println("RESET PIN (button or ST-LINK)");
    else                             Serial.println("unknown");

    RCC->CSR |= RCC_CSR_RMVF;   // clear flags for the next reset
}

// ==========================================
// SETUP
// ==========================================

void setup() {
    // Laser output LOW before anything else (the gate/base pull-down
    // already holds it off during reset). StateMachine::begin()
    // takes over the pin later through LaserDriver.
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);

    Serial.begin(115200);
    delay(300);   // give the serial monitor time to connect

    Serial.println();
    Serial.println("================================");
    Serial.println(" BICYCLE LASER STABILIZATION");
    Serial.println(" NUCLEO-F446RE / SimpleFOC");
    Serial.println("================================");

    printResetCause();

    Serial.print("Laser pin: PB0 (A3), ");
    Serial.println(LASER_ENABLED ? "enabled in ALIGN/RUN" : "DISABLED (LASER_ENABLED=false)");

    Serial.println("Starting complete system...");
    Serial.println();

    application.begin();
}

// ==========================================
// LOOP
// ==========================================

void loop() {
    // Never add delay() here: the FOC loop runs inside update().
    application.update();
}