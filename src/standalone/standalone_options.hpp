#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "core/angle_processor.hpp"
#include "core/parameter.hpp"

namespace gutcpp::standalone {

namespace fs = std::filesystem;

struct Options {
    fs::path pythonRoot;
    fs::path parameterPath;
    bool parameterExplicit = false;
    std::optional<fs::path> runConfigPath;
    std::optional<fs::path> sourcePath;
    std::optional<fs::path> videoPathOverride;
    std::string color = "blue";
    MoveMode moveMode = MoveMode::Small;
    int freq = 50;
    double deltaT = 0.2;
    bool isImshow = true;
    bool promptPath = false;
    bool tune = false;
    std::optional<int> tuneFrame;
    double tunePreviewSpeed = 1.0;
    std::optional<cv::Rect> rBoxOverride;
    std::optional<cv::Rect> fanBoxOverride;
    std::string detectorOverride;
    std::string onnxPathOverride;
    int yoloRelockIntervalFrames = 3;
    int yoloRelockAfterMisses = 1;
};

class ImageSequenceCapture {
public:
    explicit ImageSequenceCapture(fs::path directory);

    bool read(cv::Mat& frame);
    std::size_t size() const;

private:
    fs::path directory_;
    std::vector<fs::path> files_;
    std::size_t nextIndex_ = 0;
};

bool IsImageSequenceDirectory(const fs::path& path);
fs::path ResolveRawPath(const fs::path& pythonRoot, const fs::path& rawPath);
void ResolveScenarioSource(Options& options);
void PromptScenarioPath(Options& options);
void PromptTuneVideoPathAndSpeed(Options& options, const fs::path& defaultVideoPath);
ClockMode ParseClockMode(const std::string& value);
Options ParseArgs(int argc, char** argv);

} // namespace gutcpp::standalone
