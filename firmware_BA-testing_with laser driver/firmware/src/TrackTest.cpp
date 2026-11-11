#include "TrackTest.h"

TrackTest trackTest;

namespace {
constexpr uint32_t COUNTDOWN_MS   = 3000;
constexpr uint32_t CUE_PERIOD_MS  = 1000;   // LEFT/RIGHT every 1 s -> 0.5 Hz
constexpr uint32_t CSV_PERIOD_MS  = 20;     // 50 Hz
constexpr float    SIDE_HYST_DEG  = 3.0f;   // for tilt-frequency estimate
constexpr float    SYN_AMP_DEG    = 10.0f;  // synthetic sine, as on the bench
constexpr float    SYN_FREQ_HZ    = 0.5f;
}

float TrackTest::syntheticOffsetDeg(uint32_t nowMs) const {
  if (!synthetic_ || phase_ != Phase::Recording) return 0.0f;
  const float t = (nowMs - phaseStartMs_) * 0.001f;
  return SYN_AMP_DEG * sinf(2.0f * 3.14159265f * SYN_FREQ_HZ * t);
}

void TrackTest::start(bool csv, bool synthetic, uint32_t durationMs) {
  if (phase_ != Phase::Idle) {
    Serial.println("# TRACK: test already running ('x' to abort)");
    return;
  }
  csv_ = csv;
  synthetic_ = synthetic;
  durMs_ = durationMs;
  phase_ = Phase::Countdown;
  phaseStartMs_ = millis();
  lastCountMs_ = phaseStartMs_;
  Serial.println();
  if (synthetic_) {
    Serial.println("# ==== SYNTHETIC TRACK TEST (+/-10 deg, 0.5 Hz sine) ====");
    Serial.println("# Keep the board STILL on the table. The motor moves by itself.");
    Serial.println("# 'x' aborts.");
  } else {
    Serial.println("# ==== TRACK TEST ====");
    Serial.println("# Tilt SMOOTHLY about +/-10 deg, following the LEFT/RIGHT cues.");
    Serial.println("# Don't go near the roll fault limit. 'x' aborts.");
  }
  Serial.print("# Starting in 3 s ... ");
}

void TrackTest::abort(const char* reason) {
  if (phase_ == Phase::Idle) return;
  phase_ = Phase::Idle;
  Serial.println();
  Serial.print("# TRACK ABORTED: ");
  Serial.println(reason);
}

void TrackTest::resetStats() {
  loops_ = 0;
  sum_ = sumSq_ = 0.0;
  maxAbsErr_ = 0.0f;
  rollMin_ = 1e9f;
  rollMax_ = -1e9f;
  within1_ = within2_ = 0;
  rollSide_ = 0;
  sideChanges_ = 0;
  cueIndex_ = 0;
  lastUs_ = micros();
  maxGapUs_ = 0;
  gapsOver1ms_ = gapsOver3ms_ = 0;
}

void TrackTest::update(bool running, float rollDeg, float targetDeg, float motorDeg) {
  if (phase_ == Phase::Idle) return;
  if (!running) { abort("system left RUN state"); return; }

  const uint32_t now = millis();

  if (phase_ == Phase::Countdown) {
    if (now - lastCountMs_ >= 1000) {
      lastCountMs_ += 1000;
      const uint32_t left = (COUNTDOWN_MS - (now - phaseStartMs_) + 999) / 1000;
      if (left > 0) { Serial.print(left); Serial.print(" ... "); }
    }
    if (now - phaseStartMs_ >= COUNTDOWN_MS) {
      Serial.println("GO");
      if (csv_) Serial.println("#CSV,t_ms,roll_deg,target_deg,motor_deg,err_deg");
      resetStats();
      phase_ = Phase::Recording;
      phaseStartMs_ = now;
      lastCueMs_ = now - CUE_PERIOD_MS;   // first cue immediately
      lastCsvMs_ = now - CSV_PERIOD_MS;
    }
    return;
  }

  // ---- Recording ----
  const uint32_t nowUs = micros();
  const uint32_t gap = nowUs - lastUs_;
  lastUs_ = nowUs;
  if (gap > maxGapUs_) maxGapUs_ = gap;
  if (gap > 1000) gapsOver1ms_++;
  if (gap > 3000) gapsOver3ms_++;

  const float err = targetDeg - motorDeg;
  const float ae = fabsf(err);

  loops_++;
  sum_ += err;
  sumSq_ += (double)err * err;
  if (ae > maxAbsErr_) maxAbsErr_ = ae;
  if (ae < 1.0f) within1_++;
  if (ae < 2.0f) within2_++;
  if (rollDeg < rollMin_) rollMin_ = rollDeg;
  if (rollDeg > rollMax_) rollMax_ = rollDeg;

  int side = rollSide_;
  if (rollDeg >  SIDE_HYST_DEG) side = 1;
  if (rollDeg < -SIDE_HYST_DEG) side = -1;
  if (rollSide_ != 0 && side != rollSide_) sideChanges_++;
  rollSide_ = side;

  if (!synthetic_ && now - lastCueMs_ >= CUE_PERIOD_MS) {
    lastCueMs_ += CUE_PERIOD_MS;
    // LEFT = aim for relativeRoll -10 deg, RIGHT = aim for +10 deg.
    // The current roll is shown so you can see how far you are tilting.
    Serial.print((cueIndex_++ % 2 == 0) ? "# <<< LEFT  (aim -10)" : "#          RIGHT >>> (aim +10)");
    Serial.print("   roll now: ");
    Serial.println(rollDeg, 1);
  }

  if (csv_ && now - lastCsvMs_ >= CSV_PERIOD_MS) {
    lastCsvMs_ += CSV_PERIOD_MS;
    Serial.print("CSV,");
    Serial.print(now - phaseStartMs_); Serial.print(',');
    Serial.print(rollDeg, 2);          Serial.print(',');
    Serial.print(targetDeg, 2);        Serial.print(',');
    Serial.print(motorDeg, 2);         Serial.print(',');
    Serial.println(err, 2);
  }

  if (now - phaseStartMs_ >= durMs_) finish();
}

void TrackTest::finish() {
  phase_ = Phase::Idle;
  if (loops_ == 0) { Serial.println("# TRACK: no samples"); return; }

  const float durS   = durMs_ * 0.001f;
  const float loopHz = loops_ / durS;
  const float mean   = (float)(sum_ / loops_);
  const float rms    = (float)sqrt(sumSq_ / loops_);
  const float freq   = sideChanges_ / 2.0f / durS;
  const float in1    = 100.0f * within1_ / loops_;
  const float in2    = 100.0f * within2_ / loops_;

  Serial.println("# ==== TRACK TEST RESULT ====");
  Serial.print("# mode               : ");
  Serial.println(synthetic_ ? "SYNTHETIC sine +/-10 deg 0.5 Hz (board still)" : "hand tilt (IMU)");
  Serial.print("# logging CSV        : "); Serial.println(csv_ ? "on" : "off");
  Serial.print("# duration [s]       : "); Serial.println(durS, 1);
  Serial.print("# loop rate [Hz]     : "); Serial.println(loopHz, 0);
  Serial.print("# max loop gap [ms]  : "); Serial.println(maxGapUs_ / 1000.0f, 2);
  Serial.print("# gaps > 1 ms        : "); Serial.println(gapsOver1ms_);
  Serial.print("# gaps > 3 ms        : "); Serial.println(gapsOver3ms_);
  Serial.print("# roll range [deg]   : "); Serial.print(rollMin_, 1);
  Serial.print(" .. ");                    Serial.println(rollMax_, 1);
  Serial.print("# tilt freq [Hz]     : "); Serial.println(freq, 2);
  Serial.print("# error max [deg]    : "); Serial.println(maxAbsErr_, 2);
  Serial.print("# error rms [deg]    : "); Serial.println(rms, 2);
  Serial.print("# error mean [deg]   : "); Serial.println(mean, 2);
  Serial.print("# time |err|<1 deg   : "); Serial.print(in1, 0); Serial.println(" %");
  Serial.print("# time |err|<2 deg   : "); Serial.print(in2, 0); Serial.println(" %");
  Serial.println("#SUMMARY,mode,csv,dur_s,loop_hz,roll_min,roll_max,freq_hz,err_max,err_rms,err_mean,pct_lt1,pct_lt2,max_gap_ms,gaps_gt1ms,gaps_gt3ms");
  Serial.print("SUMMARY,");
  Serial.print(synthetic_ ? "SYN" : "IMU"); Serial.print(',');
  Serial.print(csv_ ? 1 : 0);    Serial.print(',');
  Serial.print(durS, 1);         Serial.print(',');
  Serial.print(loopHz, 0);       Serial.print(',');
  Serial.print(rollMin_, 2);     Serial.print(',');
  Serial.print(rollMax_, 2);     Serial.print(',');
  Serial.print(freq, 2);         Serial.print(',');
  Serial.print(maxAbsErr_, 2);   Serial.print(',');
  Serial.print(rms, 2);          Serial.print(',');
  Serial.print(mean, 2);         Serial.print(',');
  Serial.print(in1, 0);          Serial.print(',');
  Serial.print(in2, 0);          Serial.print(',');
  Serial.print(maxGapUs_ / 1000.0f, 2); Serial.print(',');
  Serial.print(gapsOver1ms_);    Serial.print(',');
  Serial.println(gapsOver3ms_);
}