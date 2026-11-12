#pragma once
#include <Arduino.h>

#include "config.h"

#include "drivers/I2CBus.h"
#include "drivers/IMU.h"
#include "drivers/ToF.h"
#include "drivers/LaserDriver.h"

#include "estimation/RollEstimator.h"
#include "control/MotorControl.h"

#include "ButtonHandler.h"
#include "CalibrationStorage.h"

class StateMachine {
public:
    void begin();
    void update();

private:
    enum class State {
        BOOT,
        IDLE,
        CALIBRATION,
        ALIGN,
        RUN,
        ERROR
    };

    // Serial input handling
    bool commandInProgress_ = false;
    bool autoStatusEnabled_ = true;

    uint32_t lastSerialInputMs_ = 0;

    char commandBuffer_[16] = {};
    uint8_t commandLength_ = 0;
    //end

    State state_ = State::BOOT;

    I2CBus bus_;
    IMU imu_;
    ToF tof_;
    RollEstimator roll_;
    MotorControl motor_;
    LaserDriver laser_{LASER_PIN};

    ButtonHandler button_;
    CalibrationStorage storage_;

    // ---- Mechanical reference (flash) ----
    bool refValid_ = false;
    CalibrationStorage::Reference ref_ = {};

    // Valid after initFOC(): shaft angle at which alpha = 0, and
    // d(shaft)/d(alpha) = sensor_direction * BEAM_ENCODER_SIGN (+/-1).
    float shaftHorizontalRad_ = NAN;
    float beamToShaft_ = 1.0f;

    float theta0Rad_ = NAN;          // nominal world beam angle

    // ---- ALIGN ----
    float alignBaseShaftRad_ = NAN;  // ALIGN goal (shaft), without trim
    float rampShaftRad_ = NAN;       // slew-limited position toward goal
    float alignTrimRad_ = 0.0f;      // beam units, + = beam down
    float alphaGoalRad_ = NAN;       // homing goal (housing frame)
    bool homing_ = false;
    bool homed_ = false;
    uint32_t homingStartMs_ = 0;
    uint32_t homingTimeoutMs_ = 0;
    uint32_t lastRampMs_ = 0;
    // The loop's 'now' is taken before initFOC() blocks, so the ramp
    // clock is (re)started on the first updateAlignTarget() call.
    bool rampClockValid_ = false;
    // initFOC() blocks for 1-2 s without IMU polling: homing waits for
    // fresh IMU data before it starts.
    bool homingPending_ = false;

    // Main-loop stall diagnostic; 0 = skip the next check (after the
    // intentionally blocking initFOC() and flash write).
    uint32_t lastLoopMs_ = 0;

    // IMU recovery in IDLE (no data for > 1 s)
    uint32_t lastImuRecoverMs_ = 0;
    uint8_t imuRecoverLevel_ = 0;
    bool imuRecovering_ = false;
    uint32_t homingPendingSinceMs_ = 0;

    // ---- RUN ----
    float alignedBeamRad_ = NAN;     // theta0 + trim (world angle at phi=0)
    float filteredPhiRad_ = 0.0f;    // low-passed roll for the RUN target
    bool rollFilterOn_ = (ROLL_FILTER_TF_S > 0.0f);
    uint32_t lastFilterMs_ = 0;
    float alignedMotorRad_ = NAN;    // shaft for alignedBeamRad_
    float motorTargetRad_ = 0.0f;

    float heightSumM_ = 0.0f;
    uint8_t calibrationFrames_ = 0;

    uint32_t calibrationStartMs_ = 0;
    uint32_t lastToFPollMs_ = 0;
    uint32_t lastImuPollMs_ = 0;
    uint32_t lastMotorUpdateMs_ = 0;
    uint32_t lastEncoderCheckMs_ = 0;
    uint32_t trackingErrorSinceMs_ = 0;
    uint32_t lastStatusMs_ = 0;

    const char* faultText_ = "none";

    void changeState(State next);
    void applyLaserForState();
    void fail(const char* message);

    void handleButton(ButtonEvent event, uint32_t now);
    void handleSerial(uint32_t now);

    void startCalibration();
    void updateCalibration(uint32_t now);
    void finishCalibration();

    void startHoming(uint32_t now);
    void updateAlignTarget(uint32_t now);
    void saveReference();
    void printReference();

    float runBeamTargetRad(float phiRad) const;
    float shaftFromBeam(float alphaRad) const;
    float beamFromShaft(float shaftRad) const;
    float actualBeamRad();

    void beginRun(uint32_t now);
    void updateMotor(uint32_t now);

    void updateLEDs(uint32_t now);
    void printStatus(uint32_t now);

    const char* stateName(State state) const;
    const char* laserName() const;
};