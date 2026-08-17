#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "core/buff_tracker.hpp"
#include "core/parameter.hpp"
#include "standalone_options.hpp"

namespace gutcpp::standalone {

struct TuneControls {
    int lh = 0;
    int ls = 0;
    int lv = 0;
    int uh = 255;
    int us = 255;
    int uv = 255;
    int kernel = 0;
    int outside = 100;
    int inside = 0;
};

void SafeImshow(const std::string& windowName, const cv::Mat& image);
void EnsureResizableWindow(const std::string& windowName, const cv::Size& size);
cv::Rect SelectRoiFitted(const std::string& windowName, const cv::Mat& image);
void RunTuneMode(const Options& options, Parameter parameter, const fs::path& resolvedParameterPath,
                 const fs::path& videoPath);
void WriteAnglesFile(const std::vector<double>& angles, const Parameter& parameter);
void ShowAnglesPlot(const std::vector<double>& angles);
BBox RoiToBBox(const cv::Rect& rect);
cv::Rect BBoxToRect(const BBox& bbox);
int PreferredYoloSeedClassId(const std::string& color);

} // namespace gutcpp::standalone
