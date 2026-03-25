#include "hsv_detector.hpp"

namespace gutcpp {

HsvDetector::HsvDetector(bool showDebug) : showDebug_(showDebug) {}

bool HsvDetector::initialize(const cv::Mat& /*frame*/,
                             const Parameter& param,
                             std::optional<cv::Rect> rBoxHint,
                             std::optional<cv::Rect> fanBoxHint) {
    if (!rBoxHint.has_value() || !fanBoxHint.has_value()) {
        return false;  // HSV mode requires ROI hints (manual or from YOLO)
    }

    const auto& r = rBoxHint.value();
    const auto& f = fanBoxHint.value();
    BBox rBox(r.x, r.y, r.x + r.width, r.y + r.height);
    BBox fanBladeBox(f.x, f.y, f.x + f.width, f.y + f.height);

    tracker_ = std::make_unique<F_BuffTracker>(fanBladeBox, rBox, param, showDebug_);
    initialized_ = true;
    return true;
}

DetectionResult HsvDetector::detect(cv::Mat& frame) {
    DetectionResult result;
    if (!initialized_ || !tracker_) {
        return result;
    }

    const bool ok = tracker_->update(frame, true);
    if (!ok) {
        return result;
    }

    result.found = true;
    result.rBox = tracker_->rBox();
    result.fanBladeBox = tracker_->fanBladeBox();
    result.radius = tracker_->radius();
    return result;
}

} // namespace gutcpp
