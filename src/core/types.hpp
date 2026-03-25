#pragma once

#include <array>
#include <string>
#include <opencv2/core.hpp>
#include "buff_tracker.hpp"

namespace gutcpp {

struct Keypoints {
    std::array<cv::Point2f, 5> points{};  // Dataset order: 0,1,3,4 are blade points; 2 is center R
    bool valid = false;
};

struct DetectionResult {
    bool found = false;
    BBox rBox{0, 0, 0, 0};
    BBox fanBladeBox{0, 0, 0, 0};
    double radius = 0.0;
    Keypoints keypoints;
    int classId = -1;       // 0=RR, 1=RW, 2=BR, 3=BW
    float confidence = 0.0f;
};

struct PipelineOutput {
    bool predictionReady = false;
    double observedAngle = 0.0;
    double rawAngle = 0.0;
    double deltaAngle = 0.0;
    double compensatedDelta = 0.0;
    cv::Point2d predictedPoint{0.0, 0.0};
    cv::Point2d compensatedPoint{0.0, 0.0};
    std::string debugState;
    // Tracker outputs forwarded for visualization
    BBox rBox{0, 0, 0, 0};
    BBox fanBladeBox{0, 0, 0, 0};
    double radius = 0.0;
};

} // namespace gutcpp
