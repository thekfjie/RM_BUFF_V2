#pragma once

#include <filesystem>
#include <string>

#include <opencv2/core.hpp>

namespace gutcpp {

struct HSVRange {
    cv::Scalar lowerLimit;
    cv::Scalar upperLimit;
};

struct MaybeTargetThreshold {
    double width = 0.0;
    double height = 0.0;
    double area = 0.0;
};

struct Parameter {
    HSVRange hsv;
    int kernel = 0;
    double insideRate = 0.0;
    double outsideRate = 0.0;
    MaybeTargetThreshold maybeTarget;
    std::string videoRelativePath;
    int start = 0;
    std::filesystem::path parameterPath;

    // --- Optional fields (backward-compatible with older YAML/JSON files) ---
    std::string detectorType = "hsv";
    std::string onnxModelPath;
    float yoloConfidence = 0.25f;
    float yoloNmsThreshold = 0.45f;
    int yoloInputWidth = 640;
    int yoloInputHeight = 640;
    int yoloRefreshInterval = 30;
    bool enableCompensation = false;
    double bulletSpeed = 15.0;
    double targetDistance = 7.0;
    double commLatencySec = 0.01;
    double gimbalDelaySec = 0.05;
    double extraDelaySec = 0.0;
};

Parameter LoadParameter(const std::filesystem::path& parameterPath);
void SaveParameter(const Parameter& parameter);
std::filesystem::path ResolveParameterPath(const std::filesystem::path& pythonRoot,
                                           const std::filesystem::path& parameterPath);
std::filesystem::path ResolveVideoPath(const Parameter& parameter, const std::filesystem::path& pythonRoot);

}
