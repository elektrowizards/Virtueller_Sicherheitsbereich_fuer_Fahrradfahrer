#include "StateMachine.h"
#include "config.h"
#include "control/Geometry.h"
#include "TrackTest.h"
#include "TiltTest.h"

#include <math.h>

// ==========================================
// I2C CLOCK DURING OPERATION
// ==========================================
// AS5600 (encoder), BNO085 (IMU) and VL53L8CX share one bus.
// At 100 kHz an IMU read can block the encoder (and loopFOC) for
// several ms. 400 kHz is the BNO085 maximum. 'i' toggles 100k/400k
// at runtime for a before/after comparison with the track test.
static constexpr uint32_t I2C_FAST_HZ = 400000;
static constexpr uint32_t I2C_SLOW_HZ = 100000;
static uint32_t s_i2cHz = I2C_FAST_HZ;

// ==========================================
// STATE NAMES
// ==========================================

const char* StateMachine::stateName(State s) const {
    switch (s) {
        case State::BOOT:        return "BOOT";
        case State::IDLE:        return "IDLE";
        case State::CALIBRATION: return "CALIBRATION";
        case State::ALIGN:       return "ALIGN";
        case State::RUN:         return "RUN";
        case State::ERROR:       return "ERROR";
    }
    return "UNKNOWN";
}

const char* StateMachine::laserName() const {
    if (laser_.isLocked()) return "LOCKED";
    switch (laser_.mode()) {
        case LaserDriver::Mode::On:    return "ON";
        case LaserDriver::Mode::Blink: return "BLINK";
        default:                       return "OFF";
    }
}

// ==========================================
// STATE TRANSITIONS
// ==========================================

void StateMachine::changeState(State next) {
    if (state_ == State::CALIBRATION &&
        next != State::CALIBRATION) {
        tof_.stop();
    }

    // Laser off on every transition; applyLaserForState()
    // switches it on again only for ALIGN and RUN.
    laser_.off();

    if (next == State::IDLE || next == State::ERROR) {
        motor_.stop();
    }

    // A running track test is only valid in RUN.
    if (next != State::RUN) {
        trackTest.abort(stateName(next));
        tiltTest.abort(stateName(next));
    }

    state_ = next;
    applyLaserForState();

    Serial.print("STATE -> ");
    Serial.print(stateName(state_));
    Serial.print("  laser=");
    Serial.println(laserName());
}

// ALIGN: laser on, so the line can be aimed at the target distance,
//        but OFF while homing (the beam may sweep up through the room).
// RUN:   laser on while stabilizing.
// All other states: off. In ERROR the laser is also locked.
void StateMachine::applyLaserForState() {
    switch (state_) {
        case State::ALIGN:
            if (homing_) laser_.off();
            else laser_.on();   // ignored while locked
            break;
        case State::RUN:
            laser_.on();    // ignored while locked
            break;
        default:
            laser_.off();
            break;
    }
}

void StateMachine::fail(const char* message) {
    faultText_ = message;

    // Disable outputs before printing. The lock keeps the laser
    // off until reset (ERROR is only left via NVIC_SystemReset).
    laser_.setLocked(true);
    motor_.stop();

    changeState(State::ERROR);

    Serial.print("FAULT: ");
    Serial.println(message);
}

// ==========================================
// INITIALIZATION
// ==========================================

void StateMachine::begin() {
    // Laser first: output LOW before anything slow happens.
    laser_.begin();
    if (!LASER_ENABLED) {
        laser_.setLocked(true);
    }

    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE, OUTPUT);

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(MOTOR_ENABLE, LOW);

    button_.begin();

    Serial.println("Initializing I2C...");
    bus_.begin();
    bus_.scan();

    Serial.println("Initializing VL53L8CX...");
    if (!tof_.begin(bus_.getWire())) {
        fail("ToF initialization failed");
        return;
    }

    Serial.println("Initializing SimpleFOC...");
    if (!motor_.begin(bus_.getWire())) {
        fail("Motor or AS5600 initialization failed");
        return;
    }

    Serial.println("Initializing BNO085...");
    if (!imu_.begin(bus_.getWire())) {
        fail("BNO085 initialization failed");
        return;
    }


    // Every Wire.begin() inside the drivers resets the clock,
    // so set the operating clock after the last driver init.
    bus_.getWire().setClock(s_i2cHz);
    Serial.print("I2C clock [Hz]: ");
    Serial.println(s_i2cHz);

    // Mechanical reference (beam-horizontal encoder angle + IMU mount
    // offset). Electrical alignment (initFOC) still runs on every start.
    CalibrationStorage::LoadResult lr = storage_.load(ref_);
    refValid_ = (lr == CalibrationStorage::LoadResult::OK);
    if (refValid_) {
        roll_.setMountOffset(ref_.imuMountOffsetRad);
    } else {
        roll_.clearMountOffset();
        Serial.print("Reference: ");
        Serial.println(CalibrationStorage::resultText(lr));
    }
    printReference();

    theta0Rad_ = Geometry::nominalAngleRad();
    Serial.print("Geometry: ");
    Serial.print(TABLE_TEST ? "TABLE" : "BIKE");
    Serial.print(" p=");
    Serial.print(PIVOT_HEIGHT_M, 3);
    Serial.print(" r_up=");
    Serial.print(AXIS_ABOVE_PIVOT_M, 3);
    Serial.print(" r_out=");
    Serial.print(AXIS_OUTWARD_M, 3);
    Serial.print(" D=");
    Serial.print(TARGET_DISTANCE_M, 3);
    Serial.print(" side=");
    Serial.print(LINE_SIDE > 0.0f ? "RIGHT" : "LEFT");
    Serial.print(" theta0[deg]=");
    Serial.println(Geometry::radToDeg(theta0Rad_), 2);

    changeState(State::IDLE);

    Serial.println("System ready. Motor OFF.");
    Serial.println("SHORT press -> CALIBRATION");
}

// ==========================================
// BUTTON EVENTS
// ==========================================

void StateMachine::handleButton(
    ButtonEvent event,
    uint32_t now
) {
    if (event == ButtonEvent::NONE) return;

    // Every press is reported, including the ones a state ignores.
    const char* evName =
        event == ButtonEvent::SHORT_PRESS  ? "SHORT" :
        event == ButtonEvent::DOUBLE_PRESS ? "DOUBLE" : "LONG";
    Serial.print("[BTN] ");
    Serial.print(evName);
    Serial.print(" press in ");
    Serial.println(stateName(state_));

    switch (state_) {

        case State::IDLE:
            if (event == ButtonEvent::SHORT_PRESS)
                startCalibration();
            else
                Serial.println("[BTN] ignored in IDLE: use a SHORT press (< 1.4 s)");
            break;

        case State::CALIBRATION:
            if (event == ButtonEvent::SHORT_PRESS)
                changeState(State::IDLE);
            break;

        case State::ALIGN:
            if (event == ButtonEvent::DOUBLE_PRESS)
                beginRun(now);

            if (event == ButtonEvent::LONG_PRESS)
                changeState(State::IDLE);

            if (event == ButtonEvent::SHORT_PRESS)
                Serial.println("[BTN] ignored in ALIGN: DOUBLE = RUN, LONG = IDLE");
            break;

        case State::RUN:
            if (event == ButtonEvent::SHORT_PRESS ||
                event == ButtonEvent::LONG_PRESS)
                changeState(State::IDLE);
            break;

        case State::ERROR:
            if (event != ButtonEvent::LONG_PRESS) {
                Serial.println("[BTN] ERROR: LONG press resets the board");
            }
            if (event == ButtonEvent::LONG_PRESS) {
                motor_.stop();
                laser_.off();
                NVIC_SystemReset();
            }
            break;

        default:
            break;
    }
}

// ==========================================
// CALIBRATION
// ==========================================

void StateMachine::startCalibration() {

    // Verify IMU data
    if (!imu_.isFresh(IMU_TIMEOUT_MS)) {
        Serial.println("WAIT: IMU data unavailable");
        return;
    }

    // Safety: disable motor and laser
    motor_.stop();
    laser_.off();

    // ToF bypassed: geometry comes from config.h
    heightSumM_ = 0.0f;
    calibrationFrames_ = REQUIRED_TOF_FRAMES;

    alignTrimRad_ = 0.0f;
    homing_ = false;
    homed_ = false;

    Serial.println("CALIBRATION: geometry from config.h (ToF bypassed)");

    changeState(State::CALIBRATION);

    // Immediately finish calibration without ToF ranging
    finishCalibration();
}


void StateMachine::updateCalibration(uint32_t now) {
    if (!imu_.isFresh(IMU_TIMEOUT_MS)) {
        fail("IMU lost during calibration");
        return;
    }

    if ((uint32_t)(now - calibrationStartMs_)
        > CALIBRATION_TIMEOUT_MS) {
        fail("Calibration timeout");
        return;
    }

    if ((uint32_t)(now - lastToFPollMs_)
        < TOF_POLL_MS) return;

    lastToFPollMs_ = now;

    if (tof_.update()) {
        float distanceM =
            tof_.getDistanceMm() / 1000.0f;

        if (!isfinite(distanceM)) return;

        heightSumM_ += distanceM;
        calibrationFrames_++;

        Serial.print("ToF frame ");
        Serial.print(calibrationFrames_);
        Serial.print(": ");
        Serial.print(distanceM, 3);
        Serial.println(" m");

        if (calibrationFrames_ >= REQUIRED_TOF_FRAMES)
            finishCalibration();
    }

    if (!tof_.isHealthy()) {
        fail("ToF communication error");
    }
}

void StateMachine::finishCalibration() {
    tof_.stop();

    if (!tof_.isHealthy()) {
        fail("ToF stop failed");
        return;
    }

    if (!imu_.isFresh(IMU_TIMEOUT_MS)) {
        fail("IMU unavailable after calibration");
        return;
    }

    if (calibrationFrames_ < REQUIRED_TOF_FRAMES) {
        fail("Insufficient calibration samples");
        return;
    }

    theta0Rad_ = Geometry::nominalAngleRad();

    Serial.print("Nominal world beam angle theta0 [deg] = ");
    Serial.println(Geometry::radToDeg(theta0Rad_), 2);

    Serial.println("Starting electrical motor alignment...");

    if (!motor_.startAlignment()) {
        fail("SimpleFOC alignment failed");
        return;
    }

    // ---- Map the raw encoder frame onto the shaft frame ----
    // shaft = dir * (raw + 2pi*turns). The stored reference lives in the
    // raw frame; the shaft offset is rebuilt here after every initFOC().
    const int dir = motor_.sensorDirection();
    if (dir != 1 && dir != -1) {
        fail("Sensor direction unknown after alignment");
        return;
    }
    beamToShaft_ = (float)dir * BEAM_ENCODER_SIGN;

    const float shaft = motor_.getAngle();   // same sample as raw
    const float raw = motor_.getRawAngle();
    if (!isfinite(shaft) || !isfinite(raw)) {
        fail("Encoder angle invalid");
        return;
    }

    if (refValid_) {
        const float alphaNow = BEAM_ENCODER_SIGN *
            Geometry::wrapPi(raw - ref_.rawHorizontalRad);
        shaftHorizontalRad_ = shaft - beamToShaft_ * alphaNow;
    } else {
        shaftHorizontalRad_ = NAN;
    }

    // Hold the current position until homing (or trim) moves it.
    alignBaseShaftRad_ = shaft;
    rampShaftRad_ = shaft;
    alignTrimRad_ = 0.0f;
    alphaGoalRad_ = NAN;
    homing_ = false;
    homed_ = false;
    homingPending_ = false;
    rampClockValid_ = false;

    motorTargetRad_ = shaft;
    motor_.setTarget(motorTargetRad_);

    Serial.print("Sensor direction = ");
    Serial.print(dir);
    Serial.print(", raw encoder [deg] = ");
    Serial.println(Geometry::radToDeg(raw), 2);

    // With a reference, homing starts right away: keep the laser off
    // through the state change (applyLaserForState checks homing_).
    homing_ = refValid_;
    changeState(State::ALIGN);

    Serial.print("ALIGN: laser ");
    Serial.println(laserName());

    if (refValid_) {
        // Started from updateAlignTarget() once the IMU is fresh again.
        homingPending_ = true;
        homingPendingSinceMs_ = millis();
        lastLoopMs_ = 0;   // initFOC() blocked on purpose
        Serial.println("HOMING: waiting for fresh IMU data...");
    } else {
        lastLoopMs_ = 0;   // initFOC() blocked on purpose
        Serial.println("NO REFERENCE STORED - homing refused.");
        Serial.println("One-time reference: housing LEVEL on the table,");
        Serial.print("trim with '[' (beam up) / ']' (beam down) until the line hits the mark at ");
        Serial.print(REFERENCE_DISTANCE_M, 3);
        Serial.println(" m from the edge,");
        Serial.println("then send 'z' to save.");
    }
}

// ==========================================
// HOMING / ALIGN TARGET
// ==========================================

// RUN target in the housing frame for roll phi. The ALIGN fine trim is
// kept as a constant world-angle offset.
float StateMachine::runBeamTargetRad(float phiRad) const {
    const float theta = RUN_HOLD_DISTANCE
        ? Geometry::requiredWorldAngleRad(phiRad, TARGET_DISTANCE_M)
        : theta0Rad_;
    return theta + alignTrimRad_ - phiRad;
}

float StateMachine::shaftFromBeam(float alphaRad) const {
    return shaftHorizontalRad_ + beamToShaft_ * alphaRad;
}

float StateMachine::beamFromShaft(float shaftRad) const {
    // beamToShaft_ is +/-1
    return (shaftRad - shaftHorizontalRad_) * beamToShaft_;
}

float StateMachine::actualBeamRad() {
    if (!refValid_ || !isfinite(shaftHorizontalRad_)) return NAN;
    return beamFromShaft(motor_.getAngle());
}

void StateMachine::startHoming(uint32_t now) {
    // On refusal the laser stays as it is (off when called from
    // finishCalibration); 'l' can switch it on for manual trimming.
    homing_ = false;

    const float alphaNow = actualBeamRad();
    const float phi = roll_.getLineRoll();

    if (!isfinite(alphaNow) || !isfinite(phi) ||
        !imu_.isFresh(IMU_TIMEOUT_MS)) {
        Serial.println("HOMING refused: no reference or no IMU roll.");
        return;
    }

    if (fabsf(phi) > Geometry::degToRad(MAX_ROLL_DEG)) {
        Serial.println("HOMING refused: housing roll exceeds MAX_ROLL_DEG.");
        return;
    }

    // World angle theta0 for the current housing roll: alpha = theta0 - phi
    const float goal = theta0Rad_ - phi;

    const float minA = Geometry::degToRad(BEAM_MIN_DEG);
    const float maxA = Geometry::degToRad(BEAM_MAX_DEG);
    const float margin = Geometry::degToRad(HOMING_START_MARGIN_DEG);

    Serial.print("HOMING: alpha now [deg] = ");
    Serial.print(Geometry::radToDeg(alphaNow), 2);
    Serial.print(", goal [deg] = ");
    Serial.print(Geometry::radToDeg(goal), 2);
    Serial.print(", phi [deg] = ");
    Serial.println(Geometry::radToDeg(phi), 2);

    if (alphaNow < minA - margin || alphaNow > maxA + margin) {
        Serial.println("HOMING refused: beam outside the mechanical travel.");
        Serial.println("Check the reference ('p') / BEAM_ENCODER_SIGN, or redo 'z'.");
        return;
    }

    if (goal < minA || goal > maxA) {
        Serial.println("HOMING refused: goal outside the mechanical travel.");
        return;
    }

    // Travel is limited (no wrap-around), so the direct move in alpha is
    // the shortest path that stays inside [BEAM_MIN, BEAM_MAX].
    const float travel = fabsf(goal - alphaNow);
    if (travel > Geometry::degToRad(HOMING_MAX_TRAVEL_DEG)) {
        Serial.println("HOMING refused: travel exceeds HOMING_MAX_TRAVEL_DEG.");
        return;
    }

    alphaGoalRad_ = goal;
    alignBaseShaftRad_ = shaftFromBeam(goal);
    rampShaftRad_ = motor_.getAngle();
    alignTrimRad_ = 0.0f;

    homing_ = true;
    homed_ = false;
    homingStartMs_ = now;
    rampClockValid_ = false;   // started by updateAlignTarget()
    homingTimeoutMs_ = (uint32_t)(Geometry::radToDeg(travel) /
                                  HOMING_RATE_DEG_S * 1000.0f) +
                       HOMING_SETTLE_MS;

    applyLaserForState();   // laser off until HOMED
}

// Called from updateMotor() in ALIGN every MOTOR_TARGET_PERIOD_MS.
void StateMachine::updateAlignTarget(uint32_t now) {
    // 'now' can be older than the timestamps taken inside
    // finishCalibration() (initFOC blocks for a while). Start the ramp
    // and homing clocks here, from the same time base as later calls.
    if (!rampClockValid_) {
        rampClockValid_ = true;
        lastRampMs_ = now;
        homingStartMs_ = now;
    }

    if (homingPending_) {
        if (imu_.isFresh(IMU_TIMEOUT_MS) && roll_.isValid()) {
            homingPending_ = false;
            startHoming(now);
        } else if ((int32_t)(now - homingPendingSinceMs_) > 2000) {
            homingPending_ = false;
            homing_ = false;
            Serial.println("HOMING refused: no fresh IMU data after alignment.");
        }
    }

    // While waiting for the IMU: hold, and do not evaluate "HOMED".
    if (homingPending_) {
        lastRampMs_ = now;
        motorTargetRad_ = rampShaftRad_ + beamToShaft_ * alignTrimRad_;
        return;
    }

    // dt is clamped, so a stalled main loop can never let the ramp jump
    // (the slew limit then just slows down).
    float dt = (int32_t)(now - lastRampMs_) * 0.001f;
    if (dt < 0.0f) dt = 0.0f;
    if (dt > 0.02f) dt = 0.02f;
    lastRampMs_ = now;

    // Slew-rate limited approach of the goal: keeps the position error
    // small, so Uq stays below the 2 V limit (no saturation flips).
    const float maxStep = Geometry::degToRad(HOMING_RATE_DEG_S) * dt;
    const float diff = alignBaseShaftRad_ - rampShaftRad_;

    if (fabsf(diff) <= maxStep) rampShaftRad_ = alignBaseShaftRad_;
    else rampShaftRad_ += (diff > 0.0f) ? maxStep : -maxStep;

    motorTargetRad_ = rampShaftRad_ + beamToShaft_ * alignTrimRad_;

    if (!homing_) return;

    const float actual = motor_.getAngle();
    if (rampShaftRad_ == alignBaseShaftRad_ && isfinite(actual) &&
        fabsf(actual - motorTargetRad_) <
            Geometry::degToRad(HOMED_TOLERANCE_DEG)) {

        homing_ = false;
        homed_ = true;
        applyLaserForState();   // laser back on (unless locked)

        const float alpha = actualBeamRad();
        const float phi = roll_.getLineRoll();

        Serial.print("HOMED in ");
        Serial.print(now - homingStartMs_);
        Serial.print(" ms: alpha [deg] = ");
        Serial.print(Geometry::radToDeg(alpha), 2);
        Serial.print(", theta [deg] = ");
        Serial.print(Geometry::radToDeg(alpha + phi), 2);
        Serial.print(", predicted line [m] = ");
        Serial.println(Geometry::predictedDistanceM(alpha, phi), 3);
        Serial.println("Fine trim with '[' / ']', DOUBLE press -> RUN.");
        return;
    }

    if ((uint32_t)(now - homingStartMs_) > homingTimeoutMs_) {
        homing_ = false;
        fail("Homing did not converge");
    }
}

// ==========================================
// ONE-TIME MECHANICAL REFERENCE ('z')
// ==========================================

void StateMachine::saveReference() {
    if (state_ != State::ALIGN || !motor_.isActive()) {
        Serial.println("[REF] Only in ALIGN (short press first).");
        return;
    }
    if (homing_) {
        Serial.println("[REF] Wait until homing is finished.");
        return;
    }
    if (!imu_.isFresh(IMU_TIMEOUT_MS) || !roll_.isValid()) {
        Serial.println("[REF] IMU data unavailable.");
        return;
    }
    // The line must be visible and on the mark: refuse with laser off.
    if (!laser_.isEmitting()) {
        Serial.println("[REF] Refused: laser is OFF, so the line cannot be on the mark.");
        Serial.println("      Send 'l', trim the line onto the mark, then 'z'.");
        return;
    }

    const float raw = motor_.getRawAngle();
    const float shaft = motor_.getAngle();
    if (!isfinite(raw) || !isfinite(shaft)) {
        Serial.println("[REF] Encoder angle invalid.");
        return;
    }

    // Housing is level by definition (phi = 0) and the line is on the
    // mark at REFERENCE_DISTANCE_M, so the current beam angle is:
    const float alphaRef = Geometry::requiredWorldAngleRad(
        0.0f, REFERENCE_DISTANCE_M);

    CalibrationStorage::Reference next = {};
    next.rawHorizontalRad = Geometry::wrapTwoPi(
        raw - BEAM_ENCODER_SIGN * alphaRef);
    next.imuMountOffsetRad = roll_.getRightRoll();
    next.refDistanceM = REFERENCE_DISTANCE_M;

    Serial.println("=== REFERENCE ===");
    Serial.print("raw encoder now [deg]      = ");
    Serial.println(Geometry::radToDeg(raw), 3);
    Serial.print("beam angle at mark [deg]   = ");
    Serial.println(Geometry::radToDeg(alphaRef), 3);
    Serial.print("raw beam-horizontal [deg]  = ");
    Serial.println(Geometry::radToDeg(next.rawHorizontalRad), 3);
    Serial.print("IMU mount offset [deg]     = ");
    Serial.println(Geometry::radToDeg(next.imuMountOffsetRad), 3);
    Serial.print("reference distance [m]     = ");
    Serial.println(next.refDistanceM, 3);
    Serial.print("trim applied [deg]         = ");
    Serial.println(Geometry::radToDeg(alignTrimRad_), 2);

    if (refValid_) {
        Serial.print("change vs. old ref: horizontal [deg] = ");
        Serial.print(Geometry::radToDeg(Geometry::wrapPi(
            next.rawHorizontalRad - ref_.rawHorizontalRad)), 3);
        Serial.print(", IMU offset [deg] = ");
        Serial.println(Geometry::radToDeg(Geometry::wrapPi(
            next.imuMountOffsetRad - ref_.imuMountOffsetRad)), 3);
    }

    // Flash erase stalls the CPU for ~1-2 s: motor off first.
    changeState(State::IDLE);
    Serial.println("[REF] Writing flash (motor off, ~2 s)...");

    if (!storage_.save(next)) {
        fail("Could not save reference");
        return;
    }

    ref_ = next;
    refValid_ = true;
    roll_.setMountOffset(ref_.imuMountOffsetRad);

    lastLoopMs_ = 0;   // flash write blocked on purpose
    Serial.println("[REF] Saved and verified.");
    Serial.println("SHORT press -> calibration + automatic homing (test it now).");
}

void StateMachine::printReference() {
    if (!refValid_) {
        Serial.println("[REF] none stored");
        return;
    }
    Serial.print("[REF] raw beam-horizontal [deg] = ");
    Serial.print(Geometry::radToDeg(ref_.rawHorizontalRad), 3);
    Serial.print(", IMU mount offset [deg] = ");
    Serial.print(Geometry::radToDeg(ref_.imuMountOffsetRad), 3);
    Serial.print(", taken at [m] = ");
    Serial.println(ref_.refDistanceM, 3);
}

// ==========================================
// START STABILIZATION
// ==========================================

void StateMachine::beginRun(uint32_t now) {
    if (!refValid_ || !isfinite(shaftHorizontalRad_)) {
        Serial.println("No reference stored: save one with 'z' first.");
        return;
    }

    if (homing_) {
        Serial.println("Wait until homing is finished.");
        return;
    }

    if (!motor_.isActive() ||
        !imu_.isFresh(IMU_TIMEOUT_MS) ||
        !isfinite(theta0Rad_)) {
        fail("RUN prerequisites missing");
        return;
    }

    float rollRad = roll_.getLineRoll();

    // Confirm alignment only near upright orientation
    if (!isfinite(rollRad) ||
        fabsf(rollRad) > Geometry::degToRad(10.0f)) {
        Serial.println("Keep bike upright before RUN.");
        return;
    }

    float actual = motor_.getAngle();

    if (!isfinite(actual)) {
        fail("Encoder angle invalid");
        return;
    }

    if (fabsf(actual - motorTargetRad_)
        > Geometry::degToRad(4.0f)) {
        Serial.println("Wait until motor reaches ALIGN target.");
        return;
    }

    // RUN reference: world angle theta0 (+ the ALIGN fine trim) at phi = 0.
    alignedBeamRad_ = theta0Rad_ + alignTrimRad_;
    alignedMotorRad_ = shaftFromBeam(alignedBeamRad_);
    motorTargetRad_ = shaftFromBeam(runBeamTargetRad(rollRad));

    motor_.setTarget(motorTargetRad_);

    trackingErrorSinceMs_ = 0;
    lastEncoderCheckMs_ = now;

    changeState(State::RUN);
    Serial.print(RUN_HOLD_DISTANCE
        ? "STABILIZATION ACTIVE (line distance hold) - LASER "
        : "STABILIZATION ACTIVE (beam angle hold) - LASER ");
    Serial.println(laserName());
}

// ==========================================
// MOTOR STABILIZATION
// ==========================================

void StateMachine::updateMotor(uint32_t now) {
    if (!motor_.isActive()) {
        fail("Motor unexpectedly inactive");
        return;
    }

    if (motor_.isUnstable()) {
        fail("Motor control oscillation");
        return;
    }

    if (state_ == State::RUN) {
        if (!imu_.isFresh(IMU_TIMEOUT_MS)) {
            fail("IMU data timeout");
            return;
        }

        float rollRad = roll_.getLineRoll();

        if (!isfinite(rollRad)) {
            fail("Invalid roll angle");
            return;
        }

        if (fabsf(rollRad) >
            Geometry::degToRad(MAX_ROLL_DEG)) {
            fail("Excessive bicycle roll");
            return;
        }

        // Outer stabilization loop:
        // IMU -> geometry -> motor angle setpoint
        // Outer loop: IMU roll -> geometry -> beam angle -> shaft target.
        // Distance hold: alpha = theta(phi) - phi, so the line stays at
        // TARGET_DISTANCE_M on the ground (laser moves with the lean).
        float target = shaftFromBeam(runBeamTargetRad(rollRad));

        // Synthetic track test ('q'): software sine instead of IMU roll.
        if (trackTest.isSynthetic()) {
            target = alignedMotorRad_ + Geometry::degToRad(
                trackTest.syntheticOffsetDeg(now));
        }

        float correction = target - alignedMotorRad_;

        if (fabsf(correction) >
            Geometry::degToRad(MAX_MOTOR_DELTA_DEG) ||
            beamFromShaft(target) < Geometry::degToRad(BEAM_MIN_DEG) ||
            beamFromShaft(target) > Geometry::degToRad(BEAM_MAX_DEG)) {
            fail("Target exceeds motor travel range");
            return;
        }

        motorTargetRad_ = target;

        float actual = motor_.getAngle();

        if (!isfinite(actual)) {
            fail("Invalid motor angle");
            return;
        }

        float trackingError =
            fabsf(actual - motorTargetRad_);

        if (trackingError >
            Geometry::degToRad(MAX_TRACKING_ERROR_DEG)) {

            if (trackingErrorSinceMs_ == 0)
                trackingErrorSinceMs_ = now;

            if ((uint32_t)(now - trackingErrorSinceMs_)
    > TRACKING_ERROR_TIMEOUT_MS) {

    // Diagnostic information
                Serial.println();
                Serial.println("=== MOTOR TRACKING FAULT ===");

                Serial.print("Actual motor angle: ");
                Serial.println(Geometry::radToDeg(actual), 2);

                Serial.print("Target motor angle: ");
                Serial.println(Geometry::radToDeg(motorTargetRad_), 2);

                Serial.print("Tracking error: ");
                Serial.println(Geometry::radToDeg(trackingError), 2);

                Serial.print("Roll phi: ");
                Serial.println(Geometry::radToDeg(rollRad), 2);

                Serial.print("Error duration [ms]: ");
                Serial.println(now - trackingErrorSinceMs_);

                fail("Motor tracking error");
                return;
            }
        } else {
            trackingErrorSinceMs_ = 0;
        }

        // Periodically check encoder magnet.
        if ((uint32_t)(now - lastEncoderCheckMs_)
            >= 500) {

            lastEncoderCheckMs_ = now;

            if (!motor_.magnetOK()) {
                fail("AS5600 encoder/magnet lost");
                return;
            }
        }
    }

    if (state_ == State::ALIGN) {
        updateAlignTarget(now);
        if (state_ == State::ERROR) return;
    }

    // Inner position/velocity control is handled by SimpleFOC
    motor_.setTarget(motorTargetRad_);
}

// ==========================================
// LED STATE INDICATORS
// ==========================================

void StateMachine::updateLEDs(uint32_t now) {
    bool blue = false;
    bool red = false;

    switch (state_) {
        case State::CALIBRATION:
            blue = (now / 250u) % 2u == 0u; // 2 Hz
            break;

        case State::ALIGN:
            blue = (now / 500u) % 2u == 0u; // 1 Hz
            break;

        case State::RUN:
            blue = true;
            break;

        case State::ERROR:
            red = (now / 150u) % 2u == 0u;
            break;

        default:
            break;
    }

    digitalWrite(BLUE_LED_PIN, blue ? HIGH : LOW);
    digitalWrite(RED_LED_PIN, red ? HIGH : LOW);
}

// ==========================================
// SERIAL MONITOR
// ==========================================

void StateMachine::printStatus(uint32_t now) {
    Serial.print("t=");
    Serial.print(now);

    Serial.print(" state=");
    Serial.print(stateName(state_));

    Serial.print(" rollR[deg]=");
    Serial.print(Geometry::radToDeg(roll_.getRightRoll()), 2);

    const float phi = roll_.getLineRoll();
    Serial.print(" phi[deg]=");
    Serial.print(Geometry::radToDeg(phi), 2);

    Serial.print(" laser=");
    Serial.print(laserName());

    if (motor_.isActive()) {
        Serial.print(" motor[deg]=");
        Serial.print(
            Geometry::radToDeg(motor_.getAngle()), 2
        );

        Serial.print(" target[deg]=");
        Serial.print(
            Geometry::radToDeg(motorTargetRad_), 2
        );

        // Diagnostics from the ACTUAL motor angle and roll:
        // compare D[m] with a tape measure from the pivot.
        const float alpha = actualBeamRad();
        Serial.print(" alpha[deg]=");
        Serial.print(Geometry::radToDeg(alpha), 2);
        Serial.print(" theta[deg]=");
        Serial.print(Geometry::radToDeg(alpha + phi), 2);
        Serial.print(" D[m]=");
        Serial.print(Geometry::predictedDistanceM(alpha, phi), 3);
        Serial.print(" trim[deg]=");
        Serial.print(Geometry::radToDeg(alignTrimRad_), 1);

        if (state_ == State::ALIGN) {
            Serial.print(homing_ ? " HOMING" :
                         homed_ ? " HOMED" :
                         refValid_ ? " HOLD" : " NOREF");
        }

        Serial.print(" Uq[V]=");
        Serial.print(motor_.getVoltageQ(), 2);

        Serial.print(" loop[Hz]=");
        Serial.print(motor_.loopRateHz());
    }

    if (state_ == State::ERROR) {
        Serial.print(" fault=");
        Serial.print(faultText_);
    }

    Serial.println();
}



void StateMachine::handleSerial(uint32_t now) {

    while (Serial.available() > 0) {

        char c = (char)Serial.read();

        // =====================================
        // WAIT FOR ENTER
        // =====================================

        if (c == '\n' || c == '\r') {

            // Ignore empty lines (including CR+LF)
            if (commandLength_ == 0)
                continue;

            commandBuffer_[commandLength_] = '\0';

            commandInProgress_ = false;
            lastSerialInputMs_ = now;

            // Only single-character commands allowed
            if (commandLength_ != 1) {
                Serial.println("[CMD] Invalid command");
                commandLength_ = 0;
                continue;
            }

            char command = commandBuffer_[0];
            commandLength_ = 0;

            // =====================================
            // HELP
            // =====================================

            if (command == '?') {
                Serial.println("=== COMMANDS ===");
                Serial.println("s = Show status");
                Serial.println("m = Toggle monitoring");
                Serial.println("] = Beam 1 deg DOWN (line closer, ALIGN)");
                Serial.println("[ = Beam 1 deg UP (line further, ALIGN)");
                Serial.println("z = Save mechanical reference (ALIGN,");
                Serial.println("    housing level, line on the mark)");
                Serial.println("p = Print stored reference + geometry");
                Serial.println("! = Stop motor");
                Serial.println("r = Track test, summary only (RUN)");
                Serial.println("R = Track test, summary + CSV (RUN)");
                Serial.println("q = Synthetic track test, board still (RUN)");
                Serial.println("Q = Synthetic track test + CSV (RUN)");
                Serial.println("g = Guided tilt test (RUN): prompts the angles");
                Serial.println("n = Next step of the tilt test");
                Serial.println("x = Abort track / tilt test");
                Serial.println("i = Toggle I2C clock 100k/400k");
                Serial.println("l = Toggle laser (ALIGN/RUN)");
            }

            // =====================================
            // TRACK TEST
            // =====================================

            else if (command == 'r' || command == 'R' ||
                     command == 'q' || command == 'Q') {
                if (tiltTest.isActive()) {
                    Serial.println("[TRACK] Not during the tilt test ('x' aborts it)");
                } else if (state_ == State::RUN) {
                    trackTest.start(command == 'R' || command == 'Q',
                                    command == 'q' || command == 'Q');
                } else {
                    Serial.println("[TRACK] Only available in RUN");
                }
            }

            else if (command == 'x') {
                trackTest.abort("user");
                tiltTest.abort("user");
            }

            // =====================================
            // GUIDED TILT TEST
            // =====================================

            else if (command == 'g') {
                if (state_ != State::RUN) {
                    Serial.println("[TILT] Only available in RUN (double press in ALIGN)");
                } else if (trackTest.isActive()) {
                    Serial.println("[TILT] Not during a track test");
                } else {
                    tiltTest.start();
                }
            }

            else if (command == 'n') {
                tiltTest.next();
            }

            // =====================================
            // I2C CLOCK TOGGLE (for comparison)
            // =====================================

            else if (command == 'i') {
                if (trackTest.isActive()) {
                    Serial.println("[I2C] Not during a track test");
                } else {
                    s_i2cHz = (s_i2cHz == I2C_FAST_HZ)
                        ? I2C_SLOW_HZ : I2C_FAST_HZ;
                    bus_.getWire().setClock(s_i2cHz);
                    Serial.print("[I2C] Clock [Hz] = ");
                    Serial.println(s_i2cHz);
                }
            }

            // =====================================
            // LASER ON / OFF
            // =====================================

            else if (command == 'l') {
                if (laser_.isLocked()) {
                    Serial.println("[LASER] Locked (fault or LASER_ENABLED=false)");
                } else if (state_ == State::ALIGN ||
                           state_ == State::RUN) {
                    laser_.toggle();
                    Serial.print("[LASER] ");
                    Serial.println(laserName());
                } else {
                    Serial.println("[LASER] Only available in ALIGN or RUN");
                }
            }

            // =====================================
            // STATUS
            // =====================================

            else if (command == 's') {
                printStatus(now);
            }

            // =====================================
            // MECHANICAL REFERENCE
            // =====================================

            else if (command == 'z') {
                saveReference();
            }

            else if (command == 'p') {
                printReference();
                Serial.print("theta0[deg]=");
                Serial.print(Geometry::radToDeg(theta0Rad_), 3);
                Serial.print(" mark beam angle[deg]=");
                Serial.println(Geometry::radToDeg(
                    Geometry::requiredWorldAngleRad(
                        0.0f, REFERENCE_DISTANCE_M)), 3);
            }

            // =====================================
            // MONITOR ON / OFF
            // =====================================

            else if (command == 'm') {

                autoStatusEnabled_ = !autoStatusEnabled_;

                Serial.print("[MONITOR] ");
                Serial.println(
                    autoStatusEnabled_ ? "ON" : "OFF"
                );
            }

            // =====================================
            // STOP
            // =====================================

            else if (command == '!') {

                if (state_ != State::ERROR) {
                    changeState(State::IDLE);
                    Serial.println("[MOTOR] STOPPED");
                }
            }

            // =====================================
            // MOTOR POSITION CONTROL
            // =====================================

            else if ((command == '[' || command == ']') &&
                     state_ == State::ALIGN) {

                float direction =
                    (command == ']') ? 1.0f : -1.0f;

                alignTrimRad_ += direction *
                    Geometry::degToRad(ALIGN_STEP_DEG);

                float limit =
                    Geometry::degToRad(MAX_ALIGN_TRIM_DEG);

                alignTrimRad_ = fmaxf(
                    -limit,
                    fminf(limit, alignTrimRad_)
                );

                // Applied (slew-limited) by updateAlignTarget().
                Serial.print("[TRIM] beam trim = ");
                Serial.print(Geometry::radToDeg(alignTrimRad_), 1);
                Serial.println(" deg (+ = down)");
            }

            else {
                Serial.println("[CMD] Unknown command");
            }

            continue;
        }

        // =====================================
        // BUFFER INPUT UNTIL ENTER
        // =====================================

        if (c == ' ' || c == '\t')
            continue;

        // Handle backspace
        if (c == '\b' || c == 127) {
            if (commandLength_ > 0)
                commandLength_--;
            continue;
        }

        commandInProgress_ = true;
        lastSerialInputMs_ = now;

        if (commandLength_ < sizeof(commandBuffer_) - 1) {
            commandBuffer_[commandLength_++] = c;
        } else {
            commandLength_ = 0;
            commandInProgress_ = false;
            Serial.println("[CMD] Input too long");
        }
    }
}


// ==========================================
// MAIN APPLICATION UPDATE
// ==========================================


void StateMachine::update() {

    uint32_t now = millis();

    // Diagnostic: report any main-loop stall (whatever caused it).
    if (lastLoopMs_ != 0 && (uint32_t)(now - lastLoopMs_) >= 100) {
        Serial.print("[WARN] main loop stalled for ");
        Serial.print(now - lastLoopMs_);
        Serial.print(" ms in ");
        Serial.println(stateName(state_));
    }
    lastLoopMs_ = now;

    // ==========================================
    // 1. BUTTON HANDLING
    // ==========================================

    ButtonEvent event = button_.update(
        now,
        // IDLE has no double-press meaning, so a short press fires on
        // release instead of after the 350 ms double-press window.
        state_ == State::IDLE ||
        state_ == State::RUN ||
        state_ == State::CALIBRATION
    );

    handleSerial(now);
    handleButton(event, now);
    updateLEDs(now);
    laser_.update();    // only does work in blink mode

    if (state_ == State::ERROR) {
        return;
    }

    // ==========================================
    // 2. INNER MOTOR CONTROL (SimpleFOC)
    // ==========================================

    // Must execute as frequently as possible.
    if (state_ == State::ALIGN ||
        state_ == State::RUN) {

        motor_.loopFOC();
    }

    // ==========================================
    // 3. IMU UPDATE
    // ==========================================

    if ((uint32_t)(now - lastImuPollMs_) >= 5) {

        lastImuPollMs_ = now;

        const uint32_t imuStartMs = millis();
        if (imu_.update()) {
            roll_.update(imu_.getRoll());
        }

        // Diagnostic: report IMU reads that block the main loop
        // (button, motor control and serial all stop meanwhile).
        const uint32_t imuMs = millis() - imuStartMs;
        if (imuMs >= 50) {
            Serial.print("[WARN] IMU read blocked the main loop for ");
            Serial.print(imuMs);
            Serial.println(" ms");
        }

        // Check communication health
        if (!imu_.isHealthy()) {
            fail("BNO085 communication/report error");
            return;
        }

        // Check for unexpected IMU reset
        if (imu_.takeReset() &&
            (state_ == State::RUN ||
             state_ == State::CALIBRATION ||
             state_ == State::ALIGN)) {

            fail("BNO085 reset during operation");
            return;
        }
    }

    // ==========================================
    // 4. CALIBRATION
    // ==========================================

    if (state_ == State::CALIBRATION) {
        updateCalibration(now);
    }

    if (state_ == State::ERROR) {
        return;
    }

    // ==========================================
    // 5. OUTER STABILIZATION CONTROL LOOP
    // ==========================================

    // Recalculate motor target approximately
    // every 5 ms using the measured roll angle.

    if ((state_ == State::ALIGN ||
         state_ == State::RUN) &&
        (uint32_t)(now - lastMotorUpdateMs_)
            >= MOTOR_TARGET_PERIOD_MS) {

        lastMotorUpdateMs_ = now;

        // This function calculates the target,
        // updates SimpleFOC position control,
        // and checks motor safety conditions.
        updateMotor(now);
    }

    if (state_ == State::ERROR) {
        return;
    }

    // ==========================================
    // 5b. TRACK TEST RECORDING
    // ==========================================

    // Does nothing unless a test was started with 'r' or 'R'.
    if (state_ == State::RUN) {
        trackTest.update(
            true,
            Geometry::radToDeg(roll_.getLineRoll()),
            Geometry::radToDeg(motorTargetRad_),
            Geometry::radToDeg(motor_.getAngle())
        );

        // Guided tilt test ('g'): does nothing unless started.
        if (tiltTest.isActive()) {
            const float phi = roll_.getLineRoll();
            const float alpha = actualBeamRad();
            tiltTest.update(
                now,
                Geometry::radToDeg(phi),
                Geometry::predictedDistanceM(alpha, phi),
                Geometry::predictedDistanceM(
                    theta0Rad_ + alignTrimRad_ - phi, phi),
                Geometry::radToDeg(motorTargetRad_ - motor_.getAngle()),
                motor_.getVoltageQ()
            );
        }
    }

    // ==========================================
    // 6. SERIAL MONITOR
    // ==========================================

    // Print only if monitoring is enabled.
    // Motor control continues when printing
    // is disabled.
     if ((state_ == State::ALIGN ||
         state_ == State::RUN) &&
        autoStatusEnabled_ &&
        !commandInProgress_ &&
        !trackTest.isActive() &&
        !tiltTest.isActive() &&
        (uint32_t)(now - lastStatusMs_)
            >= STATUS_PERIOD_MS) {

        lastStatusMs_ = now;
        printStatus(now);
    }
}