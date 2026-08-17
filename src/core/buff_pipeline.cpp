#include "buff_pipeline.hpp"

#include <cmath>

namespace gutcpp {

BuffPipeline::BuffPipeline(std::unique_ptr<DetectorInterface> detector,
                           const PipelineConfig& config)
    : detector_(std::move(detector)),
      compensator_(config.compensationConfig),
      config_(config) {}

bool BuffPipeline::initialize(const cv::Mat& frame,
                              const Parameter& param,
                              std::optional<cv::Rect> rBoxHint,
                              std::optional<cv::Rect> fanBoxHint) {
    if (!detector_->isInitialized()) {
        if (!detector_->initialize(frame, param, rBoxHint, fanBoxHint)) {
            return false;
        }
    }

    observer_ = std::make_unique<AngleObserver>(config_.clockMode);
    predictor_ = CreatePredictor(config_.moveMode,
                                 config_.deltaT,
                                 config_.freq,
                                 config_.bigPredictorConfig);
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

    const double predictedAngle = output.rawAngle + prediction.deltaAngle;
    output.predictedPoint.x = std::cos(predictedAngle) * detection.radius + detection.rBox.center2f().x;
    output.predictedPoint.y = std::sin(predictedAngle) * detection.radius + detection.rBox.center2f().y;

    output.angularVelocity = prediction.angularVelocity;

    if (config_.enableCompensation) {
        // Predict the complete horizon in one step. For big BUFF this integrates
        // the fitted sine velocity instead of applying velocity * delay.
        const double compensatedHorizon = config_.deltaT + compensator_.totalDelay();
        output.compensatedDelta = predictor_->predictDelta(compensatedHorizon);

        const double compensatedAngle = output.rawAngle + output.compensatedDelta;
        output.compensatedPoint.x = std::cos(compensatedAngle) * detection.radius + detection.rBox.center2f().x;
        output.compensatedPoint.y = std::sin(compensatedAngle) * detection.radius + detection.rBox.center2f().y;
    } else {
        output.compensatedDelta = prediction.deltaAngle;
        output.compensatedPoint = output.predictedPoint;
    }

    return output;
}

void BuffPipeline::updateCompensation(const CompensationConfig& config) {
    compensator_.updateConfig(config);
}

} // namespace gutcpp
