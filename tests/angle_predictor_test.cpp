#include "core/angle_processor.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

constexpr double kAmplitude = 0.90;
constexpr double kOmega = 1.94;
constexpr double kPhase = 0.35;
constexpr double kOffset = 1.10;

double IntegrateVelocity(double start, double horizon) {
    return kOffset * horizon +
           kAmplitude *
               (std::cos(kOmega * start + kPhase) -
                std::cos(kOmega * (start + horizon) + kPhase)) /
               kOmega;
}

int Fail(const char* message) {
    std::cerr << message << std::endl;
    return 1;
}

} // namespace

int main() {
    // Both rotations must unwrap correctly independent of colour/legacy hint,
    // including multiple turns and arbitrary changes of the selected blade.
    for (double direction : {-1.0, 1.0}) {
        gutcpp::AngleObserver observer(gutcpp::ClockMode::Clockwise);
        double expected = 3.10;
        double bladeOffset = 0.0;
        for (int index = 0; index < 800; ++index) {
            if (index > 0) expected += direction * 0.02;
            if (index == 130 || index == 410) bladeOffset += 2.0 * CV_PI / 5.0;
            const double measured = expected + bladeOffset;
            const double phase = observer.update(100.0 * std::cos(measured), 100.0 * std::sin(measured), 100.0);
            if (std::abs(phase - expected) > 1e-8)
                return Fail("phase unwrap must handle both directions, wrap boundaries and blade changes");
        }
        gutcpp::SmallPredictor small(0.2, 50);
        gutcpp::PredictionResult smallResult;
        for (int index = 0; index < 40; ++index) {
            const double t = index * 0.02;
            smallResult = small.update(direction * CV_PI / 3.0 * t, t);
        }
        if (!smallResult.ready || std::abs(smallResult.deltaAngle - direction * CV_PI / 15.0) > 1e-8 ||
            !std::isfinite(smallResult.phaseCorrection))
            return Fail("small KF must infer rotation and retain the correct prediction horizon");
        if (small.update(0.0, 5.0).ready || small.update(0.0, 4.0).ready)
            return Fail("small prediction must rewarm after gaps/backwards time");
    }
    gutcpp::BigPredictorConfig config;
    config.omegaSearchSteps = 240;
    config.fitUpdateStride = 4;
    config.minInliers = 80;
    config.maxSamples = 180;
    config.minInlierRatio = 0.80;
    config.inlierThreshold = 0.12;
    config.maxObservationGap = 0.20;

    constexpr double predictionHorizon = 0.20;
    gutcpp::BigPredictor predictor(predictionHorizon, 50, config);

    double timestamp = 10.0;
    double modelTime = 0.0;
    double angle = 0.0;
    gutcpp::PredictionResult result = predictor.update(angle, timestamp);
    for (int index = 1; index <= 180; ++index) {
        const double dtPattern[] = {0.016, 0.021, 0.019, 0.024, 0.018};
        const double dt = dtPattern[index % 5];
        angle += IntegrateVelocity(modelTime, dt);
        modelTime += dt;
        timestamp += dt;
        result = predictor.update(angle, timestamp);
    }

    if (!result.ready || !result.modelReady || !predictor.fitState().has_value()) {
        return Fail("irregular timestamp samples should produce a ready big-BUFF model");
    }

    const gutcpp::BigPredictor::FitState& fit = predictor.fitState().value();
    if (std::abs(fit.omega - kOmega) > 0.01 ||
        std::abs(fit.amplitude - kAmplitude) > 0.08 ||
        std::abs(fit.offset - kOffset) > 0.08) {
        return Fail("fitted sine parameters should stay close to the synthetic model");
    }

    const double expectedDelta = IntegrateVelocity(modelTime, predictionHorizon);
    if (std::abs(result.deltaAngle - expectedDelta) > 0.02 ||
        std::abs(predictor.predictDelta(predictionHorizon) - expectedDelta) > 0.02) {
        return Fail("big-BUFF prediction should use the analytic velocity integral");
    }

    timestamp += 1.0;
    angle += IntegrateVelocity(modelTime, 1.0);
    result = predictor.update(angle, timestamp);
    if (result.ready || predictor.sampleCount() != 0) {
        return Fail("a long observation gap should invalidate the old fitted model");
    }

    // Refit both rotation directions with bounded impulsive measurement noise.
    for (double direction : {-1.0, 1.0}) {
        gutcpp::BigPredictor robust(predictionHorizon, 50, config);
        double phase = 0.0;
        robust.update(phase, 0.0);
        gutcpp::PredictionResult fitted;
        for (int index = 1; index <= 260; ++index) {
            const double start = (index - 1) * 0.02;
            const double noiseSpeed = index % 13 == 0 ? -direction * 0.6 : 0.0;
            phase += direction * IntegrateVelocity(start, 0.02) + noiseSpeed * 0.02;
            fitted = robust.update(phase, index * 0.02);
        }
        // Last sample was an outlier; it must not be reported ready.
        if (fitted.ready) return Fail("current outlier must invalidate current prediction");
        phase += direction * IntegrateVelocity(5.20, 0.02);
        fitted = robust.update(phase, 5.22);
        if (!fitted.ready || std::abs(fitted.deltaAngle - direction * IntegrateVelocity(5.22, predictionHorizon)) > 0.04)
            return Fail("RANSAC should recover a signed model in the presence of outliers");
        for (int index = 1; index <= config.maxConsecutiveRejected; ++index) {
            phase += 0.1; // 5 rad/s: above the physical speed gate
            if (robust.update(phase, 5.22 + index * 0.02).ready)
                return Fail("rejected measurements must never keep prediction ready");
        }
        if (robust.fitState().has_value()) return Fail("repeated rejection must discard the stale model");
        if (robust.update(std::numeric_limits<double>::quiet_NaN(), 6.0).ready)
            return Fail("non-finite observation must invalidate prediction");
    }

    return 0;
}
