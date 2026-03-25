#pragma once

#include <memory>
#include "detector_interface.hpp"
#include "angle_processor.hpp"
#include "compensation.hpp"
#include "types.hpp"

namespace gutcpp {

struct PipelineConfig {
    MoveMode moveMode = MoveMode::Small;
    ClockMode clockMode = ClockMode::Anticlockwise;
    double deltaT = 0.2;
    int freq = 50;
    bool enableCompensation = false;
    CompensationConfig compensationConfig;
};

class BuffPipeline {
public:
    BuffPipeline(std::unique_ptr<DetectorInterface> detector,
                 const PipelineConfig& config);

    bool initialize(const cv::Mat& frame,
                   const Parameter& param,
                   std::optional<cv::Rect> rBoxHint = std::nullopt,
                   std::optional<cv::Rect> fanBoxHint = std::nullopt);

    PipelineOutput processFrame(cv::Mat& frame);

    bool isInitialized() const { return initialized_; }

    const PredictorInterface& predictor() const { return *predictor_; }

    void updateCompensation(const CompensationConfig& config);

private:
    std::unique_ptr<DetectorInterface> detector_;
    std::unique_ptr<AngleObserver> observer_;
    std::unique_ptr<PredictorInterface> predictor_;
    FlightTimeCompensator compensator_;
    PipelineConfig config_;
    bool initialized_ = false;
    double prevAngle_ = 0.0;
    bool hasPrevAngle_ = false;
};

} // namespace gutcpp
