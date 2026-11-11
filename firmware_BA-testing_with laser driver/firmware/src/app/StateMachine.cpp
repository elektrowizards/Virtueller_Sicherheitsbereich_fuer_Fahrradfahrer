#include "StateMachine.h"
#include "config.h"
#include "control/Geometry.h"
#include "TrackTest.h"

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

// ==========================================
// STATE TRANSITIONS
// ==========================================

void StateMachine::changeState(State next) {
    if (state_ == State::CALIBRATION &&
        next != State::CALIBRATION) {
        tof_.stop();
    }

    // No laser emission in this bench prototype.
    digitalWrite(LASER_PIN, LOW);

    if (next == State::IDLE || next == State::ERROR) {
        motor_.stop();
    }

    // A running track test is only valid in RUN.
    if (next != State::RUN) {
        trackTest.abort(stateName(next));
    }

    state_ = next;

    Serial.print("STATE -> ");
    Serial.println(stateName(state_));
}

void StateMachine::fail(const char* message) {
    faultText_ = message;

    // Disable outputs before printing.
    digitalWrite(LASER_PIN, LOW);
    motor_.stop();

    changeState(State::ERROR);

    Serial.print("FAULT: ");
    Serial.println(message);
}

// ==========================================
// INITIALIZATION
// ==========================================

void StateMachine::begin() {
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(LASER_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE, OUTPUT);

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(LASER_PIN, LOW);
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

    float savedHeight = NAN;
    if (storage_.loadHeight(savedHeight)) {
        Serial.print("Previously saved height [m]: ");
        Serial.println(savedHeight, 3);
    }

    // A saved height does not replace physical alignment.
    // Require calibration and alignment on each startup.
    heightM_ = NAN;
    theta0Rad_ = NAN;

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

    switch (state_) {

        case State::IDLE:
            if (event == ButtonEvent::SHORT_PRESS)
                startCalibration();
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
            break;

        case State::RUN:
            if (event == ButtonEvent::SHORT_PRESS ||
                event == ButtonEvent::LONG_PRESS)
                changeState(State::IDLE);
            break;

        case State::ERROR:
            if (event == ButtonEvent::LONG_PRESS) {
                motor_.stop();
                digitalWrite(LASER_PIN, LOW);
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
    digitalWrite(LASER_PIN, LOW);

    // Use fixed height instead of ToF measurement
    heightM_ = FIXED_HEIGHT_M;

    // Prepare existing calibration completion logic
    heightSumM_ = FIXED_HEIGHT_M * REQUIRED_TOF_FRAMES;
    calibrationFrames_ = REQUIRED_TOF_FRAMES;

    theta0Rad_ = NAN;
    alignTrimRad_ = 0.0f;

    Serial.println("CALIBRATION: Fixed height mode");
    Serial.print("Height = ");
    Serial.print(FIXED_HEIGHT_M, 2);
    Serial.println(" m");

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

    heightM_ = heightSumM_ / calibrationFrames_;
    theta0Rad_ = Geometry::nominalAngleRad(heightM_);

    // IMU reference: bicycle upright at calibration
    roll_.setReference();

    if (!storage_.saveHeight(heightM_)) {
        fail("Could not save calibrated height");
        return;
    }

    Serial.print("Height [m] = ");
    Serial.println(heightM_, 3);

    Serial.print("Nominal downward angle [deg] = ");
    Serial.println(Geometry::radToDeg(theta0Rad_), 2);

    Serial.println("Starting electrical motor alignment...");

    if (!motor_.startAlignment()) {
        fail("SimpleFOC alignment failed");
        return;
    }

    alignmentBaseRad_ = motor_.getAngle();
    motorTargetRad_ = alignmentBaseRad_;
    alignTrimRad_ = 0.0f;

    motor_.setTarget(motorTargetRad_);

    changeState(State::ALIGN);

    Serial.println("ALIGN: adjust physical laser angle.");
    Serial.println("Use '[' or ']' for +/-1 degree.");
    Serial.println("DOUBLE press once physically aligned.");
}

// ==========================================
// START STABILIZATION
// ==========================================

void StateMachine::beginRun(uint32_t now) {
    if (!motor_.isActive() ||
        !imu_.isFresh(IMU_TIMEOUT_MS) ||
        !isfinite(heightM_) ||
        !isfinite(theta0Rad_)) {
        fail("RUN prerequisites missing");
        return;
    }

    float rollRad = roll_.getRelativeRoll();

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

    // Capture the mechanically verified optical reference.
    alignedMotorRad_ = actual;
    motorTargetRad_ = actual;

    motor_.setTarget(motorTargetRad_);

    trackingErrorSinceMs_ = 0;
    lastEncoderCheckMs_ = now;

    changeState(State::RUN);
    Serial.println("STABILIZATION ACTIVE - LASER OFF");
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

        float rollRad = roll_.getRelativeRoll();

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
        float target = Geometry::motorTargetRad(
            alignedMotorRad_,
            heightM_,
            rollRad
        );

        // Synthetic track test ('q'): software sine instead of IMU roll.
        if (trackTest.isSynthetic()) {
            target = alignedMotorRad_ + Geometry::degToRad(
                trackTest.syntheticOffsetDeg(now));
        }

        float correction = target - alignedMotorRad_;

        if (fabsf(correction) >
            Geometry::degToRad(MAX_MOTOR_DELTA_DEG)) {
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

                Serial.print("Relative roll: ");
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

    Serial.print(" roll[deg]=");
    Serial.print(Geometry::radToDeg(roll_.getRoll()), 2);

    Serial.print(" relativeRoll[deg]=");
    Serial.print(
        Geometry::radToDeg(roll_.getRelativeRoll()), 2
    );

    Serial.print(" height[m]=");
    Serial.print(heightM_, 3);

    if (motor_.isActive()) {
        Serial.print(" motor[deg]=");
        Serial.print(
            Geometry::radToDeg(motor_.getAngle()), 2
        );

        Serial.print(" target[deg]=");
        Serial.print(
            Geometry::radToDeg(motorTargetRad_), 2
        );

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
                Serial.println("] = Motor +1 degree");
                Serial.println("[ = Motor -1 degree");
                Serial.println("! = Stop motor");
                Serial.println("r = Track test, summary only (RUN)");
                Serial.println("R = Track test, summary + CSV (RUN)");
                Serial.println("q = Synthetic track test, board still (RUN)");
                Serial.println("Q = Synthetic track test + CSV (RUN)");
                Serial.println("x = Abort track test");
                Serial.println("i = Toggle I2C clock 100k/400k");
            }

            // =====================================
            // TRACK TEST
            // =====================================

            else if (command == 'r' || command == 'R' ||
                     command == 'q' || command == 'Q') {
                if (state_ == State::RUN) {
                    trackTest.start(command == 'R' || command == 'Q',
                                    command == 'q' || command == 'Q');
                } else {
                    Serial.println("[TRACK] Only available in RUN");
                }
            }

            else if (command == 'x') {
                trackTest.abort("user");
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
            // STATUS
            // =====================================

            else if (command == 's') {
                printStatus(now);
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

                motorTargetRad_ =
                    alignmentBaseRad_ + alignTrimRad_;

                motor_.setTarget(motorTargetRad_);

                Serial.print("[MOTOR] Target = ");
                Serial.print(
                    Geometry::radToDeg(motorTargetRad_), 2
                );
                Serial.println(" deg");
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

    // ==========================================
    // 1. BUTTON HANDLING
    // ==========================================

    ButtonEvent event = button_.update(
        now,
        state_ == State::RUN ||
        state_ == State::CALIBRATION
    );

    handleSerial(now);
    handleButton(event, now);
    updateLEDs(now);

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

        if (imu_.update()) {
            roll_.update(imu_.getRoll());
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
            Geometry::radToDeg(roll_.getRelativeRoll()),
            Geometry::radToDeg(motorTargetRad_),
            Geometry::radToDeg(motor_.getAngle())
        );
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
        (uint32_t)(now - lastStatusMs_)
            >= STATUS_PERIOD_MS) {

        lastStatusMs_ = now;
        printStatus(now);
    }
}