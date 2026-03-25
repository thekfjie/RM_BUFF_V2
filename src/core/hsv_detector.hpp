#pragma once

#include "detector_interface.hpp"
#include "buff_tracker.hpp"
#include <memory>

namespace gutcpp {

class HsvDetector final : public DetectorInterface {
public:
    explicit HsvDetector(bool showDebug = false);

    bool initialize(const cv::Mat& frame,
                   const Parameter& param,
                   std::optional<cv::Rect> rBoxHint = std::nullopt,
                   std::optional<cv::Rect> fanBoxHint = std::nullopt) override;

    DetectionResult detect(cv::Mat& frame) override;

    bool isInitialized() const override { return initialized_; }

    std::string name() const override { return "HSV"; }

private:
    std::unique_ptr<F_BuffTracker> tracker_;
    bool showDebug_;
    bool initialized_ = false;
};

} // namespace gutcpp
