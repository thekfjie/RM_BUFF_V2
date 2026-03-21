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
};

Parameter LoadParameter(const std::filesystem::path& parameterPath);
void SaveParameter(const Parameter& parameter);
std::filesystem::path ResolveParameterPath(const std::filesystem::path& pythonRoot,
                                           const std::filesystem::path& parameterPath);
std::filesystem::path ResolveVideoPath(const Parameter& parameter, const std::filesystem::path& pythonRoot);

}
