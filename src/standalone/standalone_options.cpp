#include "standalone_options.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>

namespace gutcpp::standalone {

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
    std::cout << "Usage: predict_example_main [options] [parameter.yaml|video|image_sequence_dir]\n"
              << "  --config <path>\n"
              << "  --python-root <path>\n"
              << "  --prompt-path\n"
              << "  --video <path>  (video file or image sequence directory)\n"
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

bool IsImagePath(const fs::path& path) {
    const std::string extension = ToLower(path.extension().string());
    return extension == ".jpg" || extension == ".jpeg" || extension == ".png" || extension == ".bmp" ||
           extension == ".tif" || extension == ".tiff";
}

std::vector<fs::path> CollectImageSequenceFiles(const fs::path& directory) {
    std::vector<fs::path> files;
    if (!fs::exists(directory) || !fs::is_directory(directory)) {
        return files;
    }

    for (const fs::directory_entry& entry : fs::directory_iterator(directory)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        if (!IsImagePath(entry.path())) {
            continue;
        }
        files.push_back(entry.path().lexically_normal());
    }

    std::sort(files.begin(), files.end());
    return files;
}

bool IsImageSequenceDirectory(const fs::path& path) {
    return fs::is_directory(path) && !CollectImageSequenceFiles(path).empty();
}

ImageSequenceCapture::ImageSequenceCapture(fs::path directory)
    : directory_(std::move(directory)), files_(CollectImageSequenceFiles(directory_)) {
    if (files_.empty()) {
        throw std::runtime_error("No image files found in directory: " + directory_.string());
    }
}

bool ImageSequenceCapture::read(cv::Mat& frame) {
    if (nextIndex_ >= files_.size()) {
        return false;
    }
    frame = cv::imread(files_[nextIndex_].string(), cv::IMREAD_COLOR);
    ++nextIndex_;
    return !frame.empty();
}

std::size_t ImageSequenceCapture::size() const {
    return files_.size();
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
    if (IsImageSequenceDirectory(resolvedSource)) {
        options.videoPathOverride = resolvedSource;
        if (!options.parameterExplicit) {
            throw std::runtime_error(
                "Image sequence directory provided. Please also pass --parameter <parameter.yaml> or --config.");
        }
        return;
    }

    throw std::runtime_error("Unsupported source path: " + resolvedSource.string());
}

void PromptScenarioPath(Options& options) {
    const fs::path defaultParameterPath = ResolveParameterPath(options.pythonRoot, options.parameterPath);
    std::cout << "Scenario path (parameter.yaml, video, or image sequence directory; press ENTER to use default):"
              << std::endl;
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

MoveMode ParseMoveMode(const std::string& value) {
    if (value == "small") {
        return MoveMode::Small;
    }
    if (value == "big") {
        return MoveMode::Big;
    }
    throw std::runtime_error("Unsupported mode: " + value);
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

} // namespace gutcpp::standalone
