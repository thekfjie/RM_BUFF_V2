#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include <algorithm>
#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/videoio.hpp>

#include "core/angle_processor.hpp"
#include "core/buff_tracker.hpp"
#include "core/parameter.hpp"
#include "core/buff_pipeline.hpp"
#include "core/hsv_detector.hpp"
#include "core/yolo_detector.hpp"

namespace fs = std::filesystem;

namespace {

using gutcpp::BBox;
using gutcpp::ClockMode;
using gutcpp::MoveMode;
using gutcpp::Parameter;
using gutcpp::ResolveParameterPath;
using gutcpp::ResolveVideoPath;
using gutcpp::SaveParameter;

BBox RoiToBBox(const cv::Rect& rect);
cv::Rect BBoxToRect(const BBox& bbox);
void EnsureResizableWindow(const std::string& windowName, const cv::Size& size);
MoveMode ParseMoveMode(const std::string& value);

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
    std::string detectorOverride;  // "yolo" or "hsv", overrides parameter file
    std::string onnxPathOverride;  // override onnxModelPath from parameter file
    int yoloRelockIntervalFrames = 3;
    int yoloRelockAfterMisses = 1;
};

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

fs::path DefaultPythonRoot() {
    return (fs::path(__FILE__).parent_path().parent_path() / ".." / "RM_Buff_Tracker_GUT-main").lexically_normal();
}

std::vector<fs::path> DefaultParameterList() {
    return {
        fs::path("./examples/example_for_prediction/6_dark_blue_big/parameter.yaml"),
        fs::path("./examples/example_for_prediction/7_dark_red_big/parameter.yaml"),
        fs::path("./examples/example_for_prediction/8_dark_blue_small/parameter.yaml"),
        fs::path("./examples/example_for_prediction/9_dark_red_small/parameter.yaml"),
        fs::path("./examples/example_for_prediction/10_dark_red_small_near/parameter.yaml"),
    };
}

void PrintUsage() {
    std::cout << "Usage: predict_example_main [options] [parameter.yaml|video]\n"
              << "  --config <path>\n"
              << "  --python-root <path>\n"
              << "  --prompt-path\n"
              << "  --video <path>\n"
              << "  --parameter <path>\n"
              << "  --tune\n"
              << "  --tune-frame <int>\n"
              << "  --color <blue|red>\n"
              << "  --mode <small|big>\n"
              << "  --freq <int>\n"
              << "  --deltaT <float>\n"
              << "  --yolo-relock-interval <int>\n"
              << "  --yolo-relock-after-misses <int>\n"
              << "  --r-box x,y,w,h\n"
              << "  --fan-box x,y,w,h\n"
              << "  --imshow <0|1>\n";
}

std::string Trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n\"");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n\"");
    return value.substr(first, last - first + 1);
}

std::string NormalizePathInput(std::string value) {
    value = Trim(std::move(value));
    for (char& character : value) {
        if (character == '\\') {
            character = '/';
        }
    }
    return value;
}

std::string ToLower(std::string value) {
    for (char& character : value) {
        if (character >= 'A' && character <= 'Z') {
            character = static_cast<char>(character - 'A' + 'a');
        }
    }
    return value;
}

bool TryParseDouble(const std::string& value, double& result) {
    try {
        std::size_t parsed = 0;
        result = std::stod(value, &parsed);
        return parsed == value.size();
    } catch (...) {
        return false;
    }
}

double ClampPreviewSpeed(double speed) {
    return std::max(0.25, std::min(speed, 8.0));
}

bool HasExtension(const fs::path& path, std::string_view expected) {
    return ToLower(path.extension().string()) == expected;
}

bool IsParameterPath(const fs::path& path) {
    return HasExtension(path, ".yaml") || HasExtension(path, ".yml");
}

bool IsVideoPath(const fs::path& path) {
    const std::string extension = ToLower(path.extension().string());
    return extension == ".mp4" || extension == ".avi" || extension == ".mov" || extension == ".mkv" ||
           extension == ".mpg" || extension == ".mpeg" || extension == ".wmv";
}

fs::path ResolveRawPath(const fs::path& pythonRoot, const fs::path& rawPath) {
    if (rawPath.is_absolute()) {
        return rawPath.lexically_normal();
    }
    const fs::path currentPath = fs::current_path() / rawPath;
    if (fs::exists(currentPath)) {
        return currentPath.lexically_normal();
    }
    return (pythonRoot / rawPath).lexically_normal();
}

fs::path ResolveCliPath(const fs::path& rawPath) {
    if (rawPath.is_absolute()) {
        return rawPath.lexically_normal();
    }
    return (fs::current_path() / rawPath).lexically_normal();
}

fs::path ResolveConfigRelativePath(const fs::path& configPath, const fs::path& rawPath) {
    if (rawPath.is_absolute()) {
        return rawPath.lexically_normal();
    }
    return (configPath.parent_path() / rawPath).lexically_normal();
}

fs::path ResolveConfigSearchPath(const fs::path& configPath, const fs::path& pythonRoot, const fs::path& rawPath) {
    if (rawPath.is_absolute()) {
        return rawPath.lexically_normal();
    }

    const std::array<fs::path, 3> candidates = {
        (configPath.parent_path() / rawPath).lexically_normal(),
        (fs::current_path() / rawPath).lexically_normal(),
        (pythonRoot / rawPath).lexically_normal()
    };
    for (const fs::path& candidate : candidates) {
        if (fs::exists(candidate)) {
            return candidate;
        }
    }
    return candidates.front();
}

std::optional<fs::path> InferParameterPathFromVideo(const fs::path& videoPath) {
    const fs::path parent = videoPath.parent_path();
    const fs::path yamlPath = parent / "parameter.yaml";
    if (fs::exists(yamlPath)) {
        return yamlPath.lexically_normal();
    }
    const fs::path ymlPath = parent / "parameter.yml";
    if (fs::exists(ymlPath)) {
        return ymlPath.lexically_normal();
    }
    return std::nullopt;
}

void ResolveScenarioSource(Options& options) {
    if (!options.sourcePath.has_value()) {
        return;
    }

    const fs::path resolvedSource = ResolveRawPath(options.pythonRoot, options.sourcePath.value());
        if (IsParameterPath(resolvedSource)) {
        options.parameterPath = resolvedSource;
        options.parameterExplicit = true;
        return;
    }
    if (IsVideoPath(resolvedSource)) {
        options.videoPathOverride = resolvedSource;
        if (!options.parameterExplicit) {
            const std::optional<fs::path> inferredParameterPath = InferParameterPathFromVideo(resolvedSource);
            if (!inferredParameterPath.has_value()) {
                throw std::runtime_error(
                    "Video path provided but no sibling parameter.yaml found. Prefer passing a parameter.yaml path.");
            }
            options.parameterPath = inferredParameterPath.value();
        }
        return;
    }

    throw std::runtime_error("Unsupported source path: " + resolvedSource.string());
}

void PromptScenarioPath(Options& options) {
    const fs::path defaultParameterPath = ResolveParameterPath(options.pythonRoot, options.parameterPath);
    std::cout << "Scenario path (parameter.yaml or video, press ENTER to use default):" << std::endl;
    std::cout << "  [" << defaultParameterPath.string() << "]" << std::endl;
    std::cout << "> " << std::flush;
    std::string input;
    std::getline(std::cin, input);
    input = NormalizePathInput(std::move(input));
    if (!input.empty()) {
        options.sourcePath = fs::path(input);
    }
}

void PromptTuneVideoPathAndSpeed(Options& options, const fs::path& defaultVideoPath) {
    std::cout << "Tuner video path (press ENTER to use default):" << std::endl;
    std::cout << "  [" << defaultVideoPath.string() << "]" << std::endl;
    std::cout << "> " << std::flush;
    std::string pathInput;
    std::getline(std::cin, pathInput);
    pathInput = NormalizePathInput(std::move(pathInput));
    if (!pathInput.empty()) {
        options.sourcePath = fs::path(pathInput);
    }

    if (!options.tuneFrame.has_value()) {
        std::cout << "Initial preview speed (0.25~8.0, default 1.0):" << std::endl;
        std::cout << "> " << std::flush;
        std::string speedInput;
        std::getline(std::cin, speedInput);
        speedInput = Trim(std::move(speedInput));
        if (!speedInput.empty()) {
            double parsed = 1.0;
            if (TryParseDouble(speedInput, parsed)) {
                options.tunePreviewSpeed = ClampPreviewSpeed(parsed);
            } else {
                std::cout << "Invalid speed input. Fallback to 1.0x." << std::endl;
            }
        }
        std::cout << "Tuner Preview: SPACE freeze current frame, q quit." << std::endl;
    }
}

cv::Rect ParseRect(const std::string& value) {
    std::vector<int> values;
    std::string current;
    for (char character : value) {
        if (character == ',') {
            if (current.empty()) {
                throw std::runtime_error("Invalid rect argument: " + value);
            }
            values.push_back(std::stoi(current));
            current.clear();
            continue;
        }
        current.push_back(character);
    }
    if (!current.empty()) {
        values.push_back(std::stoi(current));
    }
    if (values.size() != 4) {
        throw std::runtime_error("Rect must be x,y,w,h: " + value);
    }
    return cv::Rect(values[0], values[1], values[2], values[3]);
}

std::optional<cv::Rect> ReadOptionalRectNode(const cv::FileNode& node) {
    if (node.empty()) {
        return std::nullopt;
    }

    std::vector<int> values;
    node >> values;
    if (values.empty()) {
        return std::nullopt;
    }
    if (values.size() != 4) {
        throw std::runtime_error("Rect node must contain 4 integers");
    }
    return cv::Rect(values[0], values[1], values[2], values[3]);
}

std::string ReadOptionalStringNode(const cv::FileNode& node) {
    if (node.empty()) {
        return "";
    }
    std::string value;
    node >> value;
    return Trim(std::move(value));
}

void ReadOptionalIntNode(const cv::FileNode& node, int& value) {
    if (!node.empty()) {
        node >> value;
    }
}

void ReadOptionalDoubleNode(const cv::FileNode& node, double& value) {
    if (!node.empty()) {
        node >> value;
    }
}

void ApplyRunConfig(const fs::path& configPath, Options& options) {
    cv::FileStorage storage;
    if (!storage.open(configPath.string(), cv::FileStorage::READ)) {
        storage.release();
        if (!storage.open(configPath.string(), cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON)) {
            throw std::runtime_error("Failed to open run config: " + configPath.string());
        }
    }

    const std::string pythonRootValue = ReadOptionalStringNode(storage["pythonRoot"]);
    if (!pythonRootValue.empty()) {
        options.pythonRoot = ResolveConfigRelativePath(configPath, fs::path(NormalizePathInput(pythonRootValue)));
    }

    const std::string parameterPathValue = ReadOptionalStringNode(storage["parameterPath"]);
    if (!parameterPathValue.empty()) {
        options.parameterPath = fs::path(NormalizePathInput(parameterPathValue));
        options.parameterExplicit = true;
    }

    const std::string videoPathValue = ReadOptionalStringNode(storage["videoPath"]);
    if (!videoPathValue.empty()) {
        options.videoPathOverride = fs::path(NormalizePathInput(videoPathValue));
    }

    const std::string colorValue = ReadOptionalStringNode(storage["color"]);
    if (!colorValue.empty()) {
        options.color = colorValue;
    }

    const std::string modeValue = ReadOptionalStringNode(storage["mode"]);
    if (!modeValue.empty()) {
        options.moveMode = ParseMoveMode(modeValue);
    }

    ReadOptionalIntNode(storage["freq"], options.freq);
    ReadOptionalDoubleNode(storage["deltaT"], options.deltaT);

    int imshowValue = options.isImshow ? 1 : 0;
    ReadOptionalIntNode(storage["imshow"], imshowValue);
    options.isImshow = (imshowValue != 0);

    const std::string detectorValue = ReadOptionalStringNode(storage["detector"]);
    if (!detectorValue.empty()) {
        options.detectorOverride = detectorValue;
    }

    const std::string onnxPathValue = ReadOptionalStringNode(storage["onnxModelPath"]);
    if (!onnxPathValue.empty()) {
        options.onnxPathOverride =
            ResolveConfigSearchPath(configPath, options.pythonRoot, fs::path(NormalizePathInput(onnxPathValue))).string();
    }

    ReadOptionalIntNode(storage["yoloRelockIntervalFrames"], options.yoloRelockIntervalFrames);
    ReadOptionalIntNode(storage["yoloRelockAfterMisses"], options.yoloRelockAfterMisses);

    int tuneValue = options.tune ? 1 : 0;
    ReadOptionalIntNode(storage["tune"], tuneValue);
    options.tune = (tuneValue != 0);

    int tuneFrameValue = options.tuneFrame.has_value() ? options.tuneFrame.value() : -1;
    ReadOptionalIntNode(storage["tuneFrame"], tuneFrameValue);
    options.tuneFrame = (tuneFrameValue >= 0) ? std::optional<int>(tuneFrameValue) : std::nullopt;

    ReadOptionalDoubleNode(storage["tunePreviewSpeed"], options.tunePreviewSpeed);

    options.rBoxOverride = ReadOptionalRectNode(storage["rBox"]);
    options.fanBoxOverride = ReadOptionalRectNode(storage["fanBox"]);
}

std::optional<fs::path> FindRunConfigPath(int argc, char** argv) {
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--config") {
            if (index + 1 >= argc) {
                throw std::runtime_error("Missing value for argument: --config");
            }
            return ResolveCliPath(fs::path(NormalizePathInput(argv[index + 1])));
        }
    }
    return std::nullopt;
}

MoveMode ParseMoveMode(const std::string& value) {
    if (value == "small") {
        return MoveMode::Small;
    }
    if (value == "big") {
        return MoveMode::Big;
    }
    throw std::runtime_error("Unsupported mode: " + value);
}

ClockMode ParseClockMode(const std::string& value) {
    if (value == "blue") {
        return ClockMode::Anticlockwise;
    }
    if (value == "red") {
        return ClockMode::Clockwise;
    }
    throw std::runtime_error("Unsupported color: " + value);
}

Options ParseArgs(int argc, char** argv) {
    Options options;
    options.pythonRoot = DefaultPythonRoot();
    options.parameterPath = DefaultParameterList()[2];

    if (const std::optional<fs::path> runConfigPath = FindRunConfigPath(argc, argv); runConfigPath.has_value()) {
        options.runConfigPath = runConfigPath;
        ApplyRunConfig(runConfigPath.value(), options);
    }

    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        auto requireValue = [&](const std::string& name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error("Missing value for argument: " + name);
            }
            ++index;
            return argv[index];
        };

        if (arg == "--config") {
            requireValue(arg);
        } else if (arg == "--python-root") {
            options.pythonRoot = requireValue(arg);
        } else if (arg == "--prompt-path") {
            options.promptPath = true;
        } else if (arg == "--video") {
            options.sourcePath = fs::path(NormalizePathInput(requireValue(arg)));
        } else if (arg == "--parameter") {
            options.parameterPath = requireValue(arg);
            options.parameterExplicit = true;
        } else if (arg == "--tune") {
            options.tune = true;
        } else if (arg == "--tune-frame") {
            options.tuneFrame = std::stoi(requireValue(arg));
        } else if (arg == "--color") {
            options.color = requireValue(arg);
        } else if (arg == "--mode") {
            options.moveMode = ParseMoveMode(requireValue(arg));
        } else if (arg == "--freq") {
            options.freq = std::stoi(requireValue(arg));
        } else if (arg == "--deltaT") {
            options.deltaT = std::stod(requireValue(arg));
        } else if (arg == "--yolo-relock-interval") {
            options.yoloRelockIntervalFrames = std::max(1, std::stoi(requireValue(arg)));
        } else if (arg == "--yolo-relock-after-misses") {
            options.yoloRelockAfterMisses = std::max(1, std::stoi(requireValue(arg)));
        } else if (arg == "--imshow") {
            options.isImshow = std::stoi(requireValue(arg)) != 0;
        } else if (arg == "--r-box") {
            options.rBoxOverride = ParseRect(requireValue(arg));
        } else if (arg == "--fan-box") {
            options.fanBoxOverride = ParseRect(requireValue(arg));
        } else if (arg == "--detector") {
            options.detectorOverride = requireValue(arg);
        } else if (arg == "--onnx") {
            options.onnxPathOverride = requireValue(arg);
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage();
            std::exit(0);
        } else if (!arg.empty() && arg[0] != '-') {
            if (options.sourcePath.has_value()) {
                throw std::runtime_error("Only one positional source path is supported");
            }
            options.sourcePath = fs::path(NormalizePathInput(arg));
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.rBoxOverride.has_value() != options.fanBoxOverride.has_value()) {
        throw std::runtime_error("--r-box and --fan-box must be provided together");
    }

    return options;
}

void SafeImshow(const std::string& windowName, const cv::Mat& image) {
    if (!image.empty()) {
        static std::unordered_set<std::string> initializedWindows;
        if (initializedWindows.find(windowName) == initializedWindows.end()) {
            const cv::Size size = [&]() {
                const int width = image.cols;
                const int height = image.rows;
                constexpr int maxWidth = 1600;
                constexpr int maxHeight = 900;
                if (width <= maxWidth && height <= maxHeight) {
                    return cv::Size(width, height);
                }
                const double scale = std::min(static_cast<double>(maxWidth) / static_cast<double>(width),
                                              static_cast<double>(maxHeight) / static_cast<double>(height));
                return cv::Size(std::max(1, static_cast<int>(width * scale)),
                                std::max(1, static_cast<int>(height * scale)));
            }();
            EnsureResizableWindow(windowName, size);
            initializedWindows.insert(windowName);
        }
        cv::imshow(windowName, image);
    }
}

void EnsureResizableWindow(const std::string& windowName, const cv::Size& size) {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(windowName, size.width, size.height);
}

int ClampInt(int value, int low, int high) {
    return std::max(low, std::min(value, high));
}

cv::Size FitWindowSize(const cv::Mat& image, int maxWidth = 1600, int maxHeight = 900) {
    const int width = image.cols;
    const int height = image.rows;
    if (width <= maxWidth && height <= maxHeight) {
        return cv::Size(width, height);
    }
    const double scale = std::min(static_cast<double>(maxWidth) / static_cast<double>(width),
                                  static_cast<double>(maxHeight) / static_cast<double>(height));
    return cv::Size(std::max(1, static_cast<int>(width * scale)), std::max(1, static_cast<int>(height * scale)));
}

cv::Rect SelectRoiFitted(const std::string& windowName, const cv::Mat& image) {
    EnsureResizableWindow(windowName, FitWindowSize(image));
    SafeImshow(windowName, image);
    return cv::selectROI(windowName, image, false, false);
}

TuneControls ControlsFromParameter(const Parameter& parameter) {
    TuneControls controls;
    controls.lh = ClampInt(static_cast<int>(parameter.hsv.lowerLimit[0]), 0, 255);
    controls.ls = ClampInt(static_cast<int>(parameter.hsv.lowerLimit[1]), 0, 255);
    controls.lv = ClampInt(static_cast<int>(parameter.hsv.lowerLimit[2]), 0, 255);
    controls.uh = ClampInt(static_cast<int>(parameter.hsv.upperLimit[0]), 0, 255);
    controls.us = ClampInt(static_cast<int>(parameter.hsv.upperLimit[1]), 0, 255);
    controls.uv = ClampInt(static_cast<int>(parameter.hsv.upperLimit[2]), 0, 255);
    controls.kernel = ClampInt(parameter.kernel, 0, 10);
    controls.outside = ClampInt(static_cast<int>(parameter.outsideRate * 100.0), 0, 200);
    controls.inside = ClampInt(static_cast<int>(parameter.insideRate * 100.0), 0, 100);
    return controls;
}

void CreateTuneTrackbars(TuneControls& controls) {
    EnsureResizableWindow("Tracking", cv::Size(800, 600));
    cv::createTrackbar("LH", "Tracking", &controls.lh, 255);
    cv::createTrackbar("LS", "Tracking", &controls.ls, 255);
    cv::createTrackbar("LV", "Tracking", &controls.lv, 255);
    cv::createTrackbar("UH", "Tracking", &controls.uh, 255);
    cv::createTrackbar("US", "Tracking", &controls.us, 255);
    cv::createTrackbar("UV", "Tracking", &controls.uv, 255);
    cv::createTrackbar("kernel", "Tracking", &controls.kernel, 10);
    cv::createTrackbar("outside", "Tracking", &controls.outside, 200);
    cv::createTrackbar("inside", "Tracking", &controls.inside, 100);
}

cv::Mat BuildTuneMask(const cv::Mat& frame, const TuneControls& controls) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(controls.lh, controls.ls, controls.lv), cv::Scalar(controls.uh, controls.us, controls.uv),
                mask);
    if (controls.kernel > 0) {
        cv::dilate(mask, mask, cv::Mat::ones(controls.kernel, controls.kernel, CV_8U), cv::Point(-1, -1), 1);
    }
    return mask;
}

bool ReadExactFrame(cv::VideoCapture& capture, int targetFrame, cv::Mat& frame) {
    if (targetFrame < 1) {
        return false;
    }
    capture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(targetFrame - 1));
    return capture.read(frame);
}

cv::Mat BrowseTuneFrame(cv::VideoCapture& capture, int& selectedFrameIndex, double previewSpeed) {
    EnsureResizableWindow("frame", cv::Size(1280, 720));
    cv::Mat frame;
    if (!capture.read(frame)) {
        throw std::runtime_error("Failed to read first frame for tuning");
    }

    double sourceFps = capture.get(cv::CAP_PROP_FPS);
    if (sourceFps <= 1e-3) {
        sourceFps = 30.0;
    }
    const int delayMs = std::max(1, static_cast<int>(std::round(1000.0 / sourceFps / ClampPreviewSpeed(previewSpeed))));
    int frameIndex = 1;
    while (true) {
        cv::Mat display = frame.clone();
        cv::putText(display, "Tuner Preview: SPACE freeze current frame, q quit", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);
        cv::putText(display, "frame = " + std::to_string(frameIndex), cv::Point(20, 65), cv::FONT_HERSHEY_SIMPLEX,
                    0.75, cv::Scalar(0, 255, 0), 2);
        cv::putText(display, "speed = " + std::to_string(previewSpeed) + "x", cv::Point(20, 100),
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);
        SafeImshow("frame", display);
        const int key = cv::waitKey(delayMs);
        if (key == 'q' || key == 'Q' || key == 27) {
            throw std::runtime_error("Tune frame selection cancelled");
        }
        if (key == ' ') {
            selectedFrameIndex = frameIndex;
            return frame;
        }
        cv::Mat nextFrame;
        if (!capture.read(nextFrame)) {
            selectedFrameIndex = frameIndex;
            return frame;
        }
        frame = nextFrame;
        ++frameIndex;
    }
}

void RunTuneMode(const Options& options, Parameter parameter, const fs::path& resolvedParameterPath,
                 const fs::path& videoPath) {
    cv::VideoCapture capture(videoPath.string());
    if (!capture.isOpened()) {
        throw std::runtime_error("Failed to open video for tuning: " + videoPath.string());
    }

    cv::Mat tuningFrame;
    int selectedFrameIndex = parameter.start;
    if (options.tuneFrame.has_value()) {
        selectedFrameIndex = options.tuneFrame.value();
        if (!ReadExactFrame(capture, selectedFrameIndex, tuningFrame)) {
            throw std::runtime_error("Failed to read tune frame: " + std::to_string(selectedFrameIndex));
        }
    } else {
        tuningFrame = BrowseTuneFrame(capture, selectedFrameIndex, options.tunePreviewSpeed);
    }

    const cv::Rect rRect = options.rBoxOverride.has_value() ? options.rBoxOverride.value() : SelectRoiFitted("roi", tuningFrame);
    const cv::Rect fanRect = options.fanBoxOverride.has_value() ? options.fanBoxOverride.value()
                                                                : SelectRoiFitted("roi2", tuningFrame);
    const BBox rBox = RoiToBBox(rRect);
    const BBox fanBladeBox = RoiToBBox(fanRect);
    const double radius = rBox.centerDistance(fanBladeBox);

    TuneControls controls = ControlsFromParameter(parameter);
    CreateTuneTrackbars(controls);
    EnsureResizableWindow("frame", cv::Size(1280, 720));
    EnsureResizableWindow("mask", cv::Size(960, 720));
    EnsureResizableWindow("res", cv::Size(960, 720));

    std::cout << "Tune mode: adjust LH/LS/LV/UH/US/UV/kernel/outside/inside, press q to save, ESC to cancel." << std::endl;

    while (true) {
        cv::Mat frameView = tuningFrame.clone();
        cv::Mat mask = BuildTuneMask(tuningFrame, controls);
        cv::Mat res;
        cv::bitwise_and(tuningFrame, tuningFrame, res, mask);

        cv::circle(frameView, rBox.center2i(), static_cast<int>(radius * static_cast<double>(controls.inside) / 100.0),
                   cv::Scalar(255, 0, 0), -1);
        cv::circle(frameView, rBox.center2i(), static_cast<int>(radius * static_cast<double>(controls.outside) / 100.0),
                   cv::Scalar(255, 0, 0), 3);
        cv::circle(mask, rBox.center2i(), static_cast<int>(radius * static_cast<double>(controls.inside) / 100.0),
                   cv::Scalar(0, 0, 0), -1);
        cv::circle(mask, rBox.center2i(), static_cast<int>(radius * static_cast<double>(controls.outside) / 100.0),
                   cv::Scalar(255, 255, 255), 3);

        SafeImshow("frame", frameView);
        SafeImshow("mask", mask);
        SafeImshow("res", res);

        const int key = cv::waitKey(1);
        if (key < 0) {
            continue;
        }
        if (key == 27) {
            std::cout << "Tune cancelled" << std::endl;
            return;
        }
        if (key == 'q' || key == 'Q') {
            parameter.parameterPath = resolvedParameterPath;
            parameter.hsv.lowerLimit = cv::Scalar(controls.lh, controls.ls, controls.lv);
            parameter.hsv.upperLimit = cv::Scalar(controls.uh, controls.us, controls.uv);
            parameter.kernel = controls.kernel;
            parameter.outsideRate = static_cast<double>(controls.outside) / 100.0;
            parameter.insideRate = static_cast<double>(controls.inside) / 100.0;
            parameter.start = selectedFrameIndex;
            if (options.videoPathOverride.has_value()) {
                parameter.videoRelativePath = ResolveRawPath(options.pythonRoot, options.videoPathOverride.value()).string();
            }
            SaveParameter(parameter);
            std::cout << "Saved tuned parameter to: " << resolvedParameterPath.string() << std::endl;
            return;
        }
    }
}

fs::path AngleDumpPathFor(const Parameter& parameter) {
    fs::path baseName = parameter.parameterPath.parent_path().filename();
    if (baseName.empty()) {
        baseName = parameter.parameterPath.stem();
    }
    return fs::current_path() / (baseName.string() + ".txt");
}

void WriteAnglesFile(const std::vector<double>& angles, const Parameter& parameter) {
    const fs::path outputPath = AngleDumpPathFor(parameter);
    std::ofstream output(outputPath);
    if (!output.is_open()) {
        throw std::runtime_error("Failed to open angle dump file: " + outputPath.string());
    }
    for (double angle : angles) {
        output << angle << '\n';
    }
}

cv::Mat BuildAnglesPlotImage(const std::vector<double>& angles) {
    constexpr int width = 1280;
    constexpr int height = 720;
    constexpr int leftMargin = 80;
    constexpr int rightMargin = 40;
    constexpr int topMargin = 40;
    constexpr int bottomMargin = 80;

    cv::Mat plot(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(plot, cv::Rect(0, 0, width, height), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::line(plot, cv::Point(leftMargin, height - bottomMargin), cv::Point(width - rightMargin, height - bottomMargin),
             cv::Scalar(0, 0, 0), 2);
    cv::line(plot, cv::Point(leftMargin, topMargin), cv::Point(leftMargin, height - bottomMargin), cv::Scalar(0, 0, 0), 2);
    cv::putText(plot, "angle", cv::Point(20, topMargin), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    cv::putText(plot, "frame", cv::Point(width - 120, height - 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);

    if (angles.empty()) {
        cv::putText(plot, "No angles collected", cv::Point(120, height / 2), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 0, 255), 2);
        return plot;
    }

    const auto [minIt, maxIt] = std::minmax_element(angles.begin(), angles.end());
    double minAngle = *minIt;
    double maxAngle = *maxIt;
    if (std::fabs(maxAngle - minAngle) < 1e-9) {
        minAngle -= 1.0;
        maxAngle += 1.0;
    }

    const int plotWidth = width - leftMargin - rightMargin;
    const int plotHeight = height - topMargin - bottomMargin;
    std::vector<cv::Point> points;
    points.reserve(angles.size());
    for (std::size_t index = 0; index < angles.size(); ++index) {
        const double xRatio = (angles.size() == 1U) ? 0.0 : static_cast<double>(index) / static_cast<double>(angles.size() - 1U);
        const double yRatio = (angles[index] - minAngle) / (maxAngle - minAngle);
        const int x = leftMargin + static_cast<int>(xRatio * static_cast<double>(plotWidth));
        const int y = topMargin + plotHeight - static_cast<int>(yRatio * static_cast<double>(plotHeight));
        points.emplace_back(x, y);
    }

    for (int tick = 0; tick <= 5; ++tick) {
        const double ratio = static_cast<double>(tick) / 5.0;
        const int y = topMargin + plotHeight - static_cast<int>(ratio * static_cast<double>(plotHeight));
        const double value = minAngle + ratio * (maxAngle - minAngle);
        cv::line(plot, cv::Point(leftMargin - 8, y), cv::Point(leftMargin, y), cv::Scalar(0, 0, 0), 1);
        cv::putText(plot, cv::format("%.3f", value), cv::Point(5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 0, 0), 1);
    }

    if (points.size() == 1U) {
        cv::circle(plot, points.front(), 3, cv::Scalar(0, 128, 255), -1);
    } else {
        cv::polylines(plot, points, false, cv::Scalar(0, 128, 255), 2, cv::LINE_AA);
    }
    return plot;
}

void ShowAnglesPlot(const std::vector<double>& angles) {
    cv::Mat plot = BuildAnglesPlotImage(angles);
    EnsureResizableWindow("angles_plot", FitWindowSize(plot, 1280, 720));
    SafeImshow("angles_plot", plot);
    std::cout << "Angle plot: press any key to close." << std::endl;
    cv::waitKey(0);
}

BBox RoiToBBox(const cv::Rect& rect) {
    if (rect.width <= 0 || rect.height <= 0) {
        throw std::runtime_error("ROI must be non-empty");
    }
    return BBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
}

cv::Rect BBoxToRect(const BBox& bbox) {
    const int width = std::max(1, static_cast<int>(std::lround(bbox.width())));
    const int height = std::max(1, static_cast<int>(std::lround(bbox.height())));
    return cv::Rect(static_cast<int>(std::lround(bbox.xmin)),
                    static_cast<int>(std::lround(bbox.ymin)),
                    width,
                    height);
}

int PreferredYoloSeedClassId(const std::string& color) {
    return (color == "red") ? 1 : 2;
}

}

int main(int argc, char** argv) {
    try {
        Options options = ParseArgs(argc, argv);
        if (options.promptPath) {
            PromptScenarioPath(options);
        }
        if (options.tune && !options.sourcePath.has_value()) {
            const fs::path defaultParameterPath = ResolveParameterPath(options.pythonRoot, options.parameterPath);
            const Parameter defaultParameter = gutcpp::LoadParameter(defaultParameterPath);
            const fs::path defaultVideoPath = ResolveVideoPath(defaultParameter, options.pythonRoot);
            PromptTuneVideoPathAndSpeed(options, defaultVideoPath);
        }
        ResolveScenarioSource(options);
        const fs::path resolvedParameterPath = ResolveParameterPath(options.pythonRoot, options.parameterPath);
        const Parameter parameter = gutcpp::LoadParameter(resolvedParameterPath);
        const fs::path videoPath = options.videoPathOverride.has_value()
                                      ? ResolveRawPath(options.pythonRoot, options.videoPathOverride.value())
                                      : ResolveVideoPath(parameter, options.pythonRoot);

        std::cout << "Using parameter: " << resolvedParameterPath.string() << std::endl;
        std::cout << "Using video: " << videoPath.string() << std::endl;
        if (options.runConfigPath.has_value()) {
            std::cout << "Using run config: " << options.runConfigPath.value().string() << std::endl;
        }

        if (options.tune) {
            RunTuneMode(options, parameter, resolvedParameterPath, videoPath);
            return 0;
        }

        cv::VideoCapture capture(videoPath.string());
        cv::Mat frame;
        if (!capture.read(frame)) {
            return -1;
        }

        std::vector<double> angles;
        std::vector<cv::Point2d> predictedPoints;
        int frameCount = 0;
        int interval = 0;
        int lostFrames = 0;
        int lastYoloAttemptFrame = -1000000;
        const fs::path logDir = fs::current_path() / "logs";
        fs::create_directories(logDir);
        const std::string modeStr = (options.moveMode == MoveMode::Small) ? "small" : "big";
        const fs::path logPath = logDir / (modeStr + "_" + options.color + "_predict.csv");
        std::ofstream logFile(logPath);
        logFile << "frame,observed_angle,raw_angle,delta_angle,pred_x,pred_y,debug_state\n";

        gutcpp::PipelineConfig pipeConfig;
        pipeConfig.moveMode = options.moveMode;
        pipeConfig.clockMode = ParseClockMode(options.color);
        pipeConfig.deltaT = options.deltaT;
        pipeConfig.freq = options.freq;
        pipeConfig.enableCompensation = parameter.enableCompensation;
        pipeConfig.compensationConfig.bulletSpeed = parameter.bulletSpeed;
        pipeConfig.compensationConfig.targetDistance = parameter.targetDistance;
        pipeConfig.compensationConfig.commLatencySec = parameter.commLatencySec;
        pipeConfig.compensationConfig.gimbalDelaySec = parameter.gimbalDelaySec;
        pipeConfig.compensationConfig.extraDelaySec = parameter.extraDelaySec;
        interval = static_cast<int>(static_cast<double>(options.freq) * options.deltaT);

        const std::string detType = !options.detectorOverride.empty()
            ? options.detectorOverride : parameter.detectorType;
        const std::string onnxPath = !options.onnxPathOverride.empty()
            ? options.onnxPathOverride : parameter.onnxModelPath;
        bool useYoloAssist = (detType == "yolo");
        const int preferredYoloClassId = PreferredYoloSeedClassId(options.color);

        std::unique_ptr<gutcpp::BuffPipeline> pipeline;
        std::unique_ptr<gutcpp::YoloDetector> yoloAssist;

        auto buildManualPipeline = [&](const cv::Mat& currentFrame) {
            const cv::Rect rRect = options.rBoxOverride.has_value() ? options.rBoxOverride.value()
                                                                    : SelectRoiFitted("roi", currentFrame);
            const cv::Rect fanRect = options.fanBoxOverride.has_value() ? options.fanBoxOverride.value()
                                                                        : SelectRoiFitted("roi2", currentFrame);
            auto hsvDet = std::make_unique<gutcpp::HsvDetector>(options.isImshow);
            auto newPipeline = std::make_unique<gutcpp::BuffPipeline>(std::move(hsvDet), pipeConfig);
            if (!newPipeline->initialize(currentFrame, parameter, rRect, fanRect)) {
                return false;
            }
            pipeline = std::move(newPipeline);
            lostFrames = 0;
            return true;
        };

        auto buildPipelineFromSeed = [&](const cv::Mat& currentFrame,
                                         const gutcpp::DetectionResult& seed,
                                         const std::string& reason) {
            auto hsvDet = std::make_unique<gutcpp::HsvDetector>(options.isImshow);
            auto newPipeline = std::make_unique<gutcpp::BuffPipeline>(std::move(hsvDet), pipeConfig);
            if (!newPipeline->initialize(currentFrame, parameter, BBoxToRect(seed.rBox), BBoxToRect(seed.fanBladeBox))) {
                std::cout << "Frame " << frameCount << ": YOLO " << reason
                          << " succeeded but HSV tracker init failed" << std::endl;
                return false;
            }
            pipeline = std::move(newPipeline);
            lostFrames = 0;
            std::cout << "Frame " << frameCount << ": YOLO " << reason
                      << " locked target, conf=" << seed.confidence
                      << " r=(" << seed.rBox.center2i().x << "," << seed.rBox.center2i().y << ")"
                      << " fan=(" << seed.fanBladeBox.center2i().x << "," << seed.fanBladeBox.center2i().y << ")"
                      << std::endl;
            return true;
        };

        auto tryYoloLock = [&](const cv::Mat& currentFrame, const std::string& reason) {
            if (!yoloAssist) {
                return false;
            }
            lastYoloAttemptFrame = frameCount;
            const std::optional<gutcpp::DetectionResult> seed =
                yoloAssist->detectTarget(currentFrame, preferredYoloClassId);
            if (!seed.has_value()) {
                std::cout << "Frame " << frameCount << ": YOLO " << reason << " miss" << std::endl;
                return false;
            }
            return buildPipelineFromSeed(currentFrame, seed.value(), reason);
        };

        if (useYoloAssist) {
            gutcpp::YoloDetectorConfig yoloCfg;
            yoloCfg.modelPath = onnxPath;
            yoloCfg.confidence = parameter.yoloConfidence;
            yoloCfg.nmsThreshold = parameter.yoloNmsThreshold;
            yoloCfg.inputWidth = parameter.yoloInputWidth;
            yoloCfg.inputHeight = parameter.yoloInputHeight;
            yoloCfg.refreshInterval = parameter.yoloRefreshInterval;

            yoloAssist = std::make_unique<gutcpp::YoloDetector>(yoloCfg, false);
            if (!yoloAssist->loadModel()) {
                std::cout << "YOLO model load failed, falling back to HSV manual mode" << std::endl;
                yoloAssist.reset();
                useYoloAssist = false;
            } else {
                std::cout << "YOLO assist active: model only initializes/relocks tracker" << std::endl;
            }
        }

        while (capture.read(frame)) {
            ++frameCount;
            std::cout << frameCount << std::endl;

            if (frameCount >= parameter.start) {
                if (!pipeline) {
                    if (useYoloAssist) {
                        if (tryYoloLock(frame, "init")) {
                            continue;
                        }
                        if (options.isImshow) {
                            SafeImshow("frame", frame);
                        }
                        const int key = cv::waitKey(1);
                        if (key == 'q' || key == 'Q') {
                            break;
                        }
                        continue;
                    }

                    if (!buildManualPipeline(frame)) {
                        throw std::runtime_error("Pipeline initialization failed");
                    }
                }

                gutcpp::PipelineOutput output = pipeline->processFrame(frame);
                if (output.rBox.area() == 0.0) {
                    if (useYoloAssist) {
                        ++lostFrames;
                        std::cout << "Frame " << frameCount << ": tracker lost target, missCount="
                                  << lostFrames << std::endl;
                        const bool shouldTryRelock =
                            (lostFrames >= options.yoloRelockAfterMisses) &&
                            ((frameCount - lastYoloAttemptFrame) >= options.yoloRelockIntervalFrames);
                        if (shouldTryRelock && tryYoloLock(frame, "relock")) {
                            continue;
                        }
                        if (options.isImshow) {
                            SafeImshow("frame", frame);
                        }
                        const int key = cv::waitKey(1);
                        if (key == 'q' || key == 'Q') {
                            break;
                        }
                        continue;
                    }
                    throw std::runtime_error("Tracker update failed");
                }
                lostFrames = 0;

                angles.push_back(output.observedAngle);

                if (output.predictionReady) {
                    const double x = output.predictedPoint.x;
                    const double y = output.predictedPoint.y;
                    predictedPoints.emplace_back(x, y);

                    logFile << frameCount << ","
                            << output.observedAngle << ","
                            << output.rawAngle << ","
                            << output.deltaAngle << ","
                            << x << "," << y << ","
                            << output.debugState << "\n";

                    cv::circle(frame, cv::Point(static_cast<int>(x), static_cast<int>(y)), 10, cv::Scalar(0, 255, 0), -1);
                    cv::putText(frame, "now predict", cv::Point(static_cast<int>(x), static_cast<int>(y)),
                                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);

                    if (interval > 0 && static_cast<int>(predictedPoints.size()) >= interval) {
                        const cv::Point2d historyPoint =
                            predictedPoints[static_cast<std::size_t>(static_cast<int>(predictedPoints.size()) - interval)];
                        cv::circle(frame, cv::Point(static_cast<int>(historyPoint.x), static_cast<int>(historyPoint.y)), 10,
                                   cv::Scalar(0, 255, 0), -1);
                        cv::putText(frame, "0.2s before predict",
                                    cv::Point(static_cast<int>(historyPoint.x), static_cast<int>(historyPoint.y)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
                    }
                }

                cv::rectangle(frame, output.fanBladeBox.p1i(), output.fanBladeBox.p2i(), cv::Scalar(0, 255, 0), 3);

                if (options.isImshow) {
                    SafeImshow("frame", frame);
                }
                const int key = cv::waitKey(1);
                if (key == 'q' || key == 'Q') {
                    break;
                }
            }
        }

        WriteAnglesFile(angles, parameter);
        logFile.flush();
        std::cout << "Prediction log: " << logPath.string() << std::endl;
        if (options.isImshow) {
            ShowAnglesPlot(angles);
        }
        std::cout << "DONE" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
