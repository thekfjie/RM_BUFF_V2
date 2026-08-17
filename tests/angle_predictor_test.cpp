#include "core/angle_processor.hpp"

#include <cmath>
#include <iostream>

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

    return 0;
}
