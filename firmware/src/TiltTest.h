#pragma once
#include <Arduino.h>

// ============================================================================
//  TiltTest: guided static tilt test in RUN ('g').
//
//  Steps through tilt angles (phi, + = toward the line side). For each step:
//    1. prompts the angle and shows the current phi once per second,
//    2. waits until phi is within TOLERANCE of the target and steady,
//    3. averages for MEASURE_MS (phi, predicted line D, tracking error, Uq),
//    4. asks for a tape measurement and waits for 'n' (next).
//  At the end a table is printed, including the line position a pure
//  beam-angle hold (gimbal) would give, for comparison.
//
//  'x' aborts. Non-blocking; motor control is not touched.
// ============================================================================
class TiltTest {
public:
    void start();
    void next();                       // 'n'
    void abort(const char* reason);

    bool isActive() const { return phase_ != Phase::Idle; }

    // Call every loop pass in RUN.
    //   phiDeg   : roll toward the line side
    //   dM       : predicted line distance from the actual motor angle
    //   gimbalDM : line distance a pure beam-angle hold would give
    //   errDeg   : target - actual motor angle
    //   uqV      : motor voltage
    void update(uint32_t nowMs, float phiDeg, float dM, float gimbalDM,
                float errDeg, float uqV);

private:
    enum class Phase { Idle, Approach, Measure, WaitNext };

    static constexpr uint8_t STEPS = 7;
    static constexpr float TARGETS_DEG[STEPS] = {0, 10, 20, 0, -10, -20, 0};
    static constexpr float TOLERANCE_DEG = 3.0f;
    static constexpr float STEADY_DEG = 1.0f;      // phi change within 1 s
    static constexpr uint32_t STEADY_MS = 1000;
    static constexpr uint32_t MEASURE_MS = 2000;
    static constexpr uint32_t PROMPT_MS = 1000;

    struct Result {
        float phi, d, dMin, dMax, gimbal, errMax, uqMax;
    };

    void prompt();
    void beginMeasure(uint32_t nowMs);
    void finishStep();
    void printTable();

    Phase phase_ = Phase::Idle;
    uint8_t step_ = 0;
    Result results_[STEPS] = {};

    uint32_t lastPromptMs_ = 0;
    uint32_t steadySinceMs_ = 0;
    float steadyRefPhi_ = 0.0f;
    uint32_t measureStartMs_ = 0;

    // accumulators
    uint32_t n_ = 0;
    double sumPhi_ = 0, sumD_ = 0, sumG_ = 0;
    float dMin_ = 0, dMax_ = 0, errMax_ = 0, uqMax_ = 0;
};

extern TiltTest tiltTest;
