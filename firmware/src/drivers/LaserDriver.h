#pragma once
#include <Arduino.h>

// Driver for the laser module behind a low-side switch.
//   PCB:       PB0 -> 100R -> AO3400A gate, 100k gate pull-down (R10/R11, Q1)
//   Prototype: PB0 (Nucleo A3) -> 1k -> NPN base, 10k base pull-down
// In both cases: pin HIGH = laser on, pin LOW = laser off.
//
// Only plain GPIO is used (no analogWrite). On the F446, PB0's PWM channels
// are TIM1_CH2N, TIM3_CH3 and TIM8_CH2N, and analogWrite would reconfigure a
// timer that SimpleFOC may already use for the motor PWM.
class LaserDriver {
public:
  enum class Mode : uint8_t { Off, On, Blink };

  explicit LaserDriver(uint32_t pin, bool activeHigh = true);

  // Configure the pin and switch the laser off. Call first thing in setup().
  void begin();

  // Call every loop iteration. Only needed for Blink, non-blocking.
  void update();

  void on();
  void off();
  void toggle();                                     // Off -> On, On/Blink -> Off
  void blink(uint16_t periodMs, uint8_t dutyPercent = 50);

  // Safety lock (e.g. motor fault, stabilization disabled, tilt out of range).
  // While locked the laser is off and on()/blink()/toggle() are ignored.
  // Unlocking does NOT switch it back on; call on() or blink() again.
  void setLocked(bool locked);
  bool isLocked() const { return locked_; }

  Mode mode() const { return mode_; }
  bool isEmitting() const { return emitting_; }      // actual output state

private:
  void writeOutput(bool emit);

  uint32_t pin_;
  bool     activeHigh_;
  Mode     mode_       = Mode::Off;
  bool     locked_     = false;
  bool     emitting_   = false;
  uint16_t periodMs_   = 500;
  uint16_t onTimeMs_   = 250;
  uint32_t blinkStart_ = 0;
};