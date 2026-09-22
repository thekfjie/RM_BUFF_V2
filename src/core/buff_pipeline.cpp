#include "buff_pipeline.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace gutcpp {

BuffPipeline::BuffPipeline(std::unique_ptr<DetectorInterface> detector,
                           const PipelineConfig& config)
    : detector_(std::move(detector)),
      compensator_(config.compensationConfig),
      config_(config) {
    if (!std::isfinite(config_.deltaT) || config_.deltaT < 0.0 || config_.freq <= 0)
        throw std::invalid_argument("BUFF requires nonnegative delta_t and positive frequency");
}

bool BuffPipeline::initialize(const cv::Mat& frame,
                              const Parameter& param,
                              std::optional<cv::Rect> rBoxHint,
                              std::optional<cv::Rect> fanBoxHint) {
    if (!detector_->isInitialized()) {
        if (!detector_->initialize(frame, param, rBoxHint, fanBoxHint)) {
            return false;
        }
    }

    resetMotion();
    initialized_ = true;
    return true;
}

void BuffPipeline::resetMotion() {
    observer_ = std::make_unique<AngleObserver>(config_.clockMode);
    predictor_ = CreatePredictor(config_.moveMode,
                                 config_.deltaT,
                                 config_.freq,
                                 config_.bigPredictorConfig);
    lastObservationTime_ = std::numeric_limits<double>::quiet_NaN();
    lastCenter_.reset();
    lastRadius_ = 0.0;
}

bool BuffPipeline::reseed(const cv::Mat& frame, const Parameter& param,
                          const cv::Rect& rBox, const cv::Rect& fanBox) {
    if (!detector_->initialize(frame, param, rBox, fanBox)) return false;
    const cv::Point2f center(rBox.x + rBox.width * 0.5f, rBox.y + rBox.height * 0.5f);
    if (!initialized_ || !lastCenter_ ||
        cv::norm(center - *lastCenter_) > std::max(20.0, lastRadius_ * 0.5)) resetMotion();
    // Nearby short relocks can change the blade without discarding the rotor
    // model. processFrame still resets on a long/backwards observation gap.
    initialized_ = true;
    return true;
}

PipelineOutput BuffPipeline::processFrame(cv::Mat& frame, double timestampSeconds) {
    PipelineOutput output;

    if (!initialized_) {
        return output;
    }

    const DetectionResult detection = detector_->detect(frame);
    if (!detection.found) {
        return output;
    }

    output.rBox = detection.rBox;
    output.fanBladeBox = detection.fanBladeBox;
    output.radius = detection.radius;
    output.keypoints = detection.keypoints;
    output.classId = detection.classId;
    output.confidence = detection.confidence;

    const cv::Point2f relative = detection.fanBladeBox.center2f() - detection.rBox.center2f();
    if (std::isfinite(timestampSeconds) && std::isfinite(lastObservationTime_)) {
        const double dt = timestampSeconds - lastObservationTime_;
        // Avoid confusing large inter-frame motion with a change of blade.
        if (dt <= 0.0 || dt > std::min(0.25, config_.bigPredictorConfig.maxObservationGap)) resetMotion();
    }
    lastObservationTime_ = timestampSeconds;
    lastCenter_ = detection.rBox.center2f();
    lastRadius_ = detection.radius;
    const double observedAngle = observer_->update(relative.x, relative.y, detection.radius);
    output.observedAngle = observedAngle;
    output.rawAngle = trans(relative.x, relative.y);

    const PredictionResult prediction = predictor_->update(observedAngle, timestampSeconds);
    output.debugState = predictor_->debugState();

    if (!prediction.ready) {
        return output;
    }

    output.predictionReady = true;
    output.deltaAngle = prediction.deltaAngle;
    output.phaseCorrection = prediction.phaseCorrection;

    const double predictedAngle = output.rawAngle + prediction.phaseCorrection + prediction.deltaAngle;
    output.predictedPoint.x = std::cos(predictedAngle) * detection.radius + detection.rBox.center2f().x;
    output.predictedPoint.y = std::sin(predictedAngle) * detection.radius + detection.rBox.center2f().y;

    output.angularVelocity = prediction.angularVelocity;

    if (config_.enableCompensation) {
        // Predict the complete horizon in one step. For big BUFF this integrates
        // the fitted sine velocity instead of applying velocity * delay.
        const double compensatedHorizon = config_.deltaT + compensator_.totalDelay();
        setPredictionHorizon(output, compensatedHorizon);
    } else {
        output.compensatedDelta = prediction.deltaAngle;
        output.compensatedPoint = output.predictedPoint;
        output.predictionHorizon = config_.deltaT;
    }

    return output;
}

void BuffPipeline::updateCompensation(const CompensationConfig& config) {
    compensator_.updateConfig(config);
}

void BuffPipeline::setPredictionHorizon(PipelineOutput& output, double horizonSeconds) const {
    if (!output.predictionReady || !std::isfinite(horizonSeconds) || horizonSeconds < 0.0) return;
    output.predictionHorizon = horizonSeconds;
    output.compensatedDelta = predictor_->predictDelta(horizonSeconds);
    const double angle = output.rawAngle + output.phaseCorrection + output.compensatedDelta;
    output.compensatedPoint = cv::Point2d(
        std::cos(angle) * output.radius + output.rBox.center2f().x,
        std::sin(angle) * output.radius + output.rBox.center2f().y);
}

} // namespace gutcpp
