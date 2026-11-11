
#pragma once
#include <Arduino.h>

#include "drivers/I2CBus.h"
#include "drivers/IMU.h"
#include "drivers/ToF.h"

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

    ButtonHandler button_;
    CalibrationStorage storage_;

    float heightM_ = NAN;
    float theta0Rad_ = NAN;
    float alignmentBaseRad_ = NAN;
    float alignedMotorRad_ = NAN;
    float alignTrimRad_ = 0.0f;
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
    void fail(const char* message);

    void handleButton(ButtonEvent event, uint32_t now);
    void handleSerial(uint32_t now);

    void startCalibration();
    void updateCalibration(uint32_t now);
    void finishCalibration();

    void beginRun(uint32_t now);
    void updateMotor(uint32_t now);

    void updateLEDs(uint32_t now);
    void printStatus(uint32_t now);

    const char* stateName(State state) const;
};
