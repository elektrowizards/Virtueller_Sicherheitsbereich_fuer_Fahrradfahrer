#pragma once
#include <Arduino.h>

// ============================================================================
//  TrackTest: full-system tracking test (RUN state), comparable to the
//  bench 'q' test (+/-10 deg sine, ~0.5 Hz).
//
//  Serial keys (wire them in your command handler):
//    'r'  20 s test, summary only        (loop rate undisturbed)
//    'R'  20 s test, summary + CSV 50 Hz (for plots)
//    'q'  SYNTHETIC test: board held still, the target is a
//         +/-10 deg / 0.5 Hz sine generated in software instead of
//         the IMU roll. Same conditions as the bench 'q' test.
//    'Q'  synthetic test + CSV
//    'x'  abort test
//
//  Flow: 3 s countdown -> 20 s recording with LEFT/RIGHT cues every 1 s
//  (= 0.5 Hz back-and-forth) -> result table + one SUMMARY line.
//  error = target - motor [deg]. Gains/controller are NOT touched.
// ============================================================================
class TrackTest {
public:
  void start(bool csv, bool synthetic = false, uint32_t durationMs = 20000);

  // Synthetic mode: StateMachine replaces the roll-based target by
  // alignedMotor + syntheticOffsetDeg() while this returns true.
  bool isSynthetic() const { return phase_ != Phase::Idle && synthetic_; }
  float syntheticOffsetDeg(uint32_t nowMs) const;
  void abort(const char* reason);

  // Call on EVERY loop() pass, after the motor target has been updated.
  // running  : true only in RUN state
  // rollDeg  : relativeRoll [deg]
  // targetDeg, motorDeg : the same values your status line prints
  void update(bool running, float rollDeg, float targetDeg, float motorDeg);

  // Use to mute your normal 500 ms status print during a test.
  bool isActive() const { return phase_ != Phase::Idle; }

private:
  enum class Phase { Idle, Countdown, Recording };

  void resetStats();
  void finish();

  Phase phase_ = Phase::Idle;
  bool csv_ = false;
  bool synthetic_ = false;
  uint32_t durMs_ = 20000;
  uint32_t phaseStartMs_ = 0;
  uint32_t lastCountMs_ = 0;
  uint32_t lastCueMs_ = 0;
  uint32_t lastCsvMs_ = 0;
  int cueIndex_ = 0;

  uint32_t loops_ = 0;
  double sum_ = 0.0, sumSq_ = 0.0;
  float maxAbsErr_ = 0.0f;
  float rollMin_ = 0.0f, rollMax_ = 0.0f;
  uint32_t within1_ = 0, within2_ = 0;
  int rollSide_ = 0;
  uint32_t sideChanges_ = 0;

  // Loop timing: gap between two update() calls = one full application pass
  uint32_t lastUs_ = 0;
  uint32_t maxGapUs_ = 0;
  uint32_t gapsOver1ms_ = 0;
  uint32_t gapsOver3ms_ = 0;
};

extern TrackTest trackTest;