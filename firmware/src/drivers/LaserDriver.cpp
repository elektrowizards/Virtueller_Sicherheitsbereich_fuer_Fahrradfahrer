#include "LaserDriver.h"

LaserDriver::LaserDriver(uint32_t pin, bool activeHigh)
  : pin_(pin), activeHigh_(activeHigh) {}

void LaserDriver::begin() {
  pinMode(pin_, OUTPUT);
  mode_ = Mode::Off;
  emitting_ = true;          // force the first write to actually happen
  writeOutput(false);
}

void LaserDriver::update() {
  if (mode_ != Mode::Blink || locked_) return;
  const uint32_t phase = (millis() - blinkStart_) % periodMs_;
  writeOutput(phase < onTimeMs_);
}

void LaserDriver::on() {
  if (locked_) return;
  mode_ = Mode::On;
  writeOutput(true);
}

void LaserDriver::off() {
  mode_ = Mode::Off;
  writeOutput(false);
}

void LaserDriver::toggle() {
  if (mode_ == Mode::Off) on();
  else off();
}

void LaserDriver::blink(uint16_t periodMs, uint8_t dutyPercent) {
  if (locked_) return;
  if (dutyPercent == 0)   { off(); return; }
  if (dutyPercent >= 100) { on();  return; }
  if (periodMs < 2) periodMs = 2;

  periodMs_   = periodMs;
  onTimeMs_   = static_cast<uint16_t>((uint32_t)periodMs * dutyPercent / 100);
  if (onTimeMs_ == 0) onTimeMs_ = 1;
  blinkStart_ = millis();
  mode_       = Mode::Blink;
  writeOutput(true);         // start each blink cycle with the "on" phase
}

void LaserDriver::setLocked(bool locked) {
  locked_ = locked;
  if (locked_) off();
}

void LaserDriver::writeOutput(bool emit) {
  if (emit == emitting_) return;   // only touch the pin on a change
  emitting_ = emit;
  digitalWrite(pin_, (emit == activeHigh_) ? HIGH : LOW);
}