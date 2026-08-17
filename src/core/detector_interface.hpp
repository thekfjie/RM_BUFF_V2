#pragma once

#include <memory>
#include <optional>
#include <string>
#include <opencv2/core.hpp>
#include "types.hpp"
#include "parameter.hpp"

namespace gutcpp {

class DetectorInterface {
public:
    virtual ~DetectorInterface() = default;

    virtual bool initialize(const cv::Mat& frame,
                           const Parameter& param,
                           std::optional<cv::Rect> rBoxHint = std::nullopt,
                           std::optional<cv::Rect> fanBoxHint = std::nullopt) = 0;

    virtual DetectionResult detect(cv::Mat& frame) = 0;

    virtual bool isInitialized() const = 0;

    virtual std::string name() const = 0;
};

} // namespace gutcpp
