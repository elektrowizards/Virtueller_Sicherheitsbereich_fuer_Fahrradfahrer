#include "TiltTest.h"
#include <math.h>

TiltTest tiltTest;

constexpr float TiltTest::TARGETS_DEG[TiltTest::STEPS];

namespace {
const char* sideText(float deg) {
    if (deg > 0.0f) return " (toward the line)";
    if (deg < 0.0f) return " (away from the line)";
    return " (flat)";
}
}

void TiltTest::start() {
    step_ = 0;
    for (auto& r : results_) r = Result{NAN, NAN, NAN, NAN, NAN, NAN, NAN};

    Serial.println();
    Serial.println("# ==== GUIDED TILT TEST (RUN, line distance) ====");
    Serial.println("# Tilt the housing over the table edge as prompted and HOLD it.");
    Serial.println("# Each step: hold steady -> 2 s measurement -> measure the");
    Serial.println("# line with the tape -> send 'n' for the next step. 'x' aborts.");
    Serial.println("# + = toward the line, - = away from the line.");

    phase_ = Phase::Approach;
    lastPromptMs_ = 0;
    steadySinceMs_ = 0;
    prompt();
}

void TiltTest::prompt() {
    Serial.println();
    Serial.print("# STEP ");
    Serial.print(step_ + 1);
    Serial.print("/");
    Serial.print(STEPS);
    Serial.print(": tilt to phi = ");
    Serial.print(TARGETS_DEG[step_], 0);
    Serial.print(" deg");
    Serial.print(sideText(TARGETS_DEG[step_]));
    Serial.print(" and HOLD (+/-");
    Serial.print(TOLERANCE_DEG, 0);
    Serial.println(" deg)");
}

void TiltTest::next() {
    if (phase_ != Phase::WaitNext) {
        Serial.println("# [TILT] 'n' only after a step is measured");
        return;
    }
    step_++;
    if (step_ >= STEPS) {
        printTable();
        phase_ = Phase::Idle;
        return;
    }
    phase_ = Phase::Approach;
    steadySinceMs_ = 0;
    lastPromptMs_ = 0;
    prompt();
}

void TiltTest::abort(const char* reason) {
    if (phase_ == Phase::Idle) return;
    phase_ = Phase::Idle;
    Serial.print("# [TILT] aborted: ");
    Serial.println(reason);
    if (step_ > 0) printTable();
}

void TiltTest::beginMeasure(uint32_t nowMs) {
    phase_ = Phase::Measure;
    measureStartMs_ = nowMs;
    n_ = 0;
    sumPhi_ = sumD_ = sumG_ = 0.0;
    dMin_ = INFINITY;
    dMax_ = -INFINITY;
    errMax_ = 0.0f;
    uqMax_ = 0.0f;
    Serial.println("# holding steady - measuring 2 s, keep still...");
}

void TiltTest::update(uint32_t nowMs, float phiDeg, float dM, float gimbalDM,
                      float errDeg, float uqV) {
    if (phase_ == Phase::Idle || phase_ == Phase::WaitNext) return;
    if (!isfinite(phiDeg)) return;

    const float target = TARGETS_DEG[step_];

    if (phase_ == Phase::Approach) {
        if ((uint32_t)(nowMs - lastPromptMs_) >= PROMPT_MS) {
            lastPromptMs_ = nowMs;
            Serial.print("#   phi now = ");
            Serial.print(phiDeg, 1);
            Serial.print(" deg, target ");
            Serial.print(target, 0);
            if (fabsf(phiDeg - target) > TOLERANCE_DEG)
                Serial.println(phiDeg < target ? "  -> tilt MORE toward the line"
                                               : "  -> tilt MORE away from the line");
            else
                Serial.println("  -> OK, hold still");
        }

        if (fabsf(phiDeg - target) > TOLERANCE_DEG) {
            steadySinceMs_ = 0;
            return;
        }
        if (steadySinceMs_ == 0 || fabsf(phiDeg - steadyRefPhi_) > STEADY_DEG) {
            steadySinceMs_ = nowMs;
            steadyRefPhi_ = phiDeg;
            return;
        }
        if ((uint32_t)(nowMs - steadySinceMs_) >= STEADY_MS)
            beginMeasure(nowMs);
        return;
    }

    // ---- Measure ----
    if (fabsf(phiDeg - target) > TOLERANCE_DEG + 1.0f) {
        Serial.println("# moved out of range - hold the angle again");
        phase_ = Phase::Approach;
        steadySinceMs_ = 0;
        return;
    }

    if (isfinite(dM)) {
        n_++;
        sumPhi_ += phiDeg;
        sumD_ += dM;
        sumG_ += isfinite(gimbalDM) ? gimbalDM : 0.0;
        if (dM < dMin_) dMin_ = dM;
        if (dM > dMax_) dMax_ = dM;
    }
    if (fabsf(errDeg) > errMax_) errMax_ = fabsf(errDeg);
    if (fabsf(uqV) > uqMax_) uqMax_ = fabsf(uqV);

    if ((uint32_t)(nowMs - measureStartMs_) >= MEASURE_MS) finishStep();
}

void TiltTest::finishStep() {
    Result& r = results_[step_];
    if (n_ > 0) {
        r.phi = sumPhi_ / n_;
        r.d = sumD_ / n_;
        r.gimbal = sumG_ / n_;
        r.dMin = dMin_;
        r.dMax = dMax_;
    }
    r.errMax = errMax_;
    r.uqMax = uqMax_;

    Serial.print("# STEP ");
    Serial.print(step_ + 1);
    Serial.print(" done: phi = ");
    Serial.print(r.phi, 1);
    Serial.print(" deg, predicted line D = ");
    Serial.print(r.d, 3);
    Serial.print(" m (angle-hold would give ");
    Serial.print(r.gimbal, 3);
    Serial.println(" m)");
    Serial.println("# >>> Measure the line with the TAPE now and write it down.");
    Serial.println("# >>> Then send 'n' for the next step (keep holding until then).");

    phase_ = Phase::WaitNext;
}

void TiltTest::printTable() {
    Serial.println();
    Serial.println("# ==== TILT TEST RESULT ====");
    Serial.println("# step | target | phi[deg] | D[m]  | D min..max  | angle-hold D[m] | err max[deg] | |Uq| max[V] | tape[m]");
    for (uint8_t i = 0; i < STEPS && i <= step_; i++) {
        const Result& r = results_[i];
        if (!isfinite(r.phi)) continue;
        Serial.print("#  ");
        Serial.print(i + 1);
        Serial.print("   | ");
        Serial.print(TARGETS_DEG[i], 0);
        Serial.print("     | ");
        Serial.print(r.phi, 1);
        Serial.print("     | ");
        Serial.print(r.d, 3);
        Serial.print(" | ");
        Serial.print(r.dMin, 3);
        Serial.print("..");
        Serial.print(r.dMax, 3);
        Serial.print(" | ");
        Serial.print(r.gimbal, 3);
        Serial.print("           | ");
        Serial.print(r.errMax, 2);
        Serial.print("         | ");
        Serial.print(r.uqMax, 2);
        Serial.println("        | ____");
    }
    // Machine-readable rows for plots
    Serial.println("#TILT,step,target_deg,phi_deg,d_m,d_min_m,d_max_m,gimbal_d_m,err_max_deg,uq_max_v");
    for (uint8_t i = 0; i < STEPS && i <= step_; i++) {
        const Result& r = results_[i];
        if (!isfinite(r.phi)) continue;
        Serial.print("TILT,");
        Serial.print(i + 1); Serial.print(",");
        Serial.print(TARGETS_DEG[i], 0); Serial.print(",");
        Serial.print(r.phi, 2); Serial.print(",");
        Serial.print(r.d, 4); Serial.print(",");
        Serial.print(r.dMin, 4); Serial.print(",");
        Serial.print(r.dMax, 4); Serial.print(",");
        Serial.print(r.gimbal, 4); Serial.print(",");
        Serial.print(r.errMax, 3); Serial.print(",");
        Serial.println(r.uqMax, 3);
    }
    Serial.println("# Fill in the tape column from your notes.");
}
