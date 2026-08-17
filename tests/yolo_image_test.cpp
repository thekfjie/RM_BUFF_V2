#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#if HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#else
#include <opencv2/dnn.hpp>
#endif

namespace fs = std::filesystem;

namespace {

constexpr int kCenterRKeypointIndex = 2;
constexpr std::array<int, 4> kBladeKeypointIndices = {0, 1, 3, 4};

struct LetterboxTransform {
    cv::Mat image;
    float scale = 1.0f;
    int padX = 0;
    int padY = 0;
};

struct Options {
    fs::path imagePath;
    fs::path labelPath;
    fs::path modelPath;
    fs::path outputPath;
    float confidence = 0.25f;
    int inputWidth = 640;
    int inputHeight = 640;
    bool show = false;
};

struct Detection {
    cv::Rect rect;
    int classId = -1;
    float confidence = 0.0f;
    std::array<cv::Point2f, 5> keypoints{};
    bool hasKeypoints = false;
};

struct LabelObject {
    cv::Rect rect;
    int classId = -1;
    std::array<cv::Point2f, 5> keypoints{};
    bool hasKeypoints = false;
};

fs::path ProjectRoot() {
    return fs::path(__FILE__).parent_path().parent_path().lexically_normal();
}

fs::path DefaultOutputPath(const fs::path& imagePath, const fs::path& modelPath) {
    fs::path outDir = ProjectRoot() / "tests" / "output";
    return outDir / (imagePath.stem().string() + "__" + modelPath.stem().string() + "__image_compare.jpg");
}

std::string ClassName(int classId) {
    switch (classId) {
        case 0: return "RR_target";
        case 1: return "RW_hit";
        case 2: return "BR_target";
        case 3: return "BW_hit";
        default: return "cls_" + std::to_string(classId);
    }
}

void PrintUsage() {
    std::cout
        << "Usage: yolo_image_test --image <image> --model <best.onnx>\n"
        << "                      [--label <label.txt>] [--output <compare.jpg>]\n"
        << "                      [--conf <float>] [--show]\n";
}

cv::Rect ClampRect(const cv::Rect& rect, const cv::Size& size) {
    const int x = std::max(0, rect.x);
    const int y = std::max(0, rect.y);
    const int maxWidth = std::max(0, size.width - x);
    const int maxHeight = std::max(0, size.height - y);
    const int width = std::min(rect.width, maxWidth);
    const int height = std::min(rect.height, maxHeight);
    return cv::Rect(x, y, std::max(0, width), std::max(0, height));
}

LetterboxTransform ApplyLetterbox(const cv::Mat& frame, int targetWidth, int targetHeight) {
    LetterboxTransform transform;
    transform.image = cv::Mat(targetHeight, targetWidth, CV_8UC3, cv::Scalar(114, 114, 114));
    if (frame.empty()) {
        return transform;
    }

    const float scale = std::min(
        static_cast<float>(targetWidth) / static_cast<float>(frame.cols),
        static_cast<float>(targetHeight) / static_cast<float>(frame.rows));
    const int resizedWidth = std::max(1, static_cast<int>(std::lround(static_cast<float>(frame.cols) * scale)));
    const int resizedHeight = std::max(1, static_cast<int>(std::lround(static_cast<float>(frame.rows) * scale)));

    const float padWidth = static_cast<float>(targetWidth - resizedWidth) / 2.0f;
    const float padHeight = static_cast<float>(targetHeight - resizedHeight) / 2.0f;
    const int left = static_cast<int>(std::lround(padWidth - 0.1f));
    const int top = static_cast<int>(std::lround(padHeight - 0.1f));

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(resizedWidth, resizedHeight));
    resized.copyTo(transform.image(cv::Rect(left, top, resizedWidth, resizedHeight)));

    transform.scale = scale;
    transform.padX = left;
    transform.padY = top;
    return transform;
}

float UndoLetterboxCoord(float value, int pad, float scale, int limit) {
    const float mapped = (value - static_cast<float>(pad)) / scale;
    return std::clamp(mapped, 0.0f, static_cast<float>(std::max(0, limit - 1)));
}

cv::Rect ExpandRect(const cv::Rect& rect, const cv::Size& size, double scaleX, double scaleY) {
    const int dx = static_cast<int>(std::lround(static_cast<double>(rect.width) * scaleX));
    const int dy = static_cast<int>(std::lround(static_cast<double>(rect.height) * scaleY));
    return ClampRect(
        cv::Rect(rect.x - dx, rect.y - dy, rect.width + dx * 2, rect.height + dy * 2),
        size);
}

cv::Rect RBoxFromCenter(const cv::Point2f& center, int side, const cv::Size& size) {
    const int roundedSide = std::max(8, side);
    const int half = roundedSide / 2;
    const int centerX = static_cast<int>(std::lround(center.x));
    const int centerY = static_cast<int>(std::lround(center.y));
    return ClampRect(cv::Rect(centerX - half, centerY - half, roundedSide, roundedSide), size);
}

std::optional<cv::Rect> DeriveFanRectFromKeypoints(const std::array<cv::Point2f, 5>& keypoints,
                                                   const cv::Size& frameSize) {
    std::vector<cv::Point2f> bladePoints;
    bladePoints.reserve(kBladeKeypointIndices.size());
    for (const int keypointIndex : kBladeKeypointIndices) {
        bladePoints.push_back(keypoints[static_cast<std::size_t>(keypointIndex)]);
    }

    cv::Rect fanRect = ClampRect(cv::boundingRect(bladePoints), frameSize);
    if (fanRect.width <= 0 || fanRect.height <= 0) {
        return std::nullopt;
    }
    return ExpandRect(fanRect, frameSize, 0.08, 0.08);
}

std::optional<cv::Rect> DeriveRRectFromKeypoints(const std::array<cv::Point2f, 5>& keypoints,
                                                 const cv::Size& frameSize) {
    const std::optional<cv::Rect> fanRect = DeriveFanRectFromKeypoints(keypoints, frameSize);
    if (!fanRect.has_value()) {
        return std::nullopt;
    }

    const cv::Point2f rCenter = keypoints[static_cast<std::size_t>(kCenterRKeypointIndex)];
    const int baseSide = std::max(
        16,
        static_cast<int>(std::lround(
            static_cast<double>(std::min(fanRect->width, fanRect->height)) * 0.30)));
    return RBoxFromCenter(rCenter, std::min(baseSide, 48), frameSize);
}

Options ParseArgs(int argc, char** argv) {
    Options options;

    auto requireValue = [&](int& index, const std::string& flag) -> std::string {
        if (index + 1 >= argc) {
            throw std::runtime_error("Missing value for " + flag);
        }
        ++index;
        return argv[index];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            PrintUsage();
            std::exit(0);
        }
        if (arg == "--image") {
            options.imagePath = requireValue(i, arg);
        } else if (arg == "--label") {
            options.labelPath = requireValue(i, arg);
        } else if (arg == "--model" || arg == "--onnx") {
            options.modelPath = requireValue(i, arg);
        } else if (arg == "--output") {
            options.outputPath = requireValue(i, arg);
        } else if (arg == "--conf") {
            options.confidence = std::stof(requireValue(i, arg));
        } else if (arg == "--show") {
            options.show = true;
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.imagePath.empty()) {
        throw std::runtime_error("--image is required");
    }
    if (options.modelPath.empty()) {
        throw std::runtime_error("--model is required");
    }

    return options;
}

std::optional<fs::path> InferLabelPath(const fs::path& imagePath) {
    fs::path candidate = imagePath;
    candidate.replace_extension(".txt");
    if (fs::exists(candidate)) {
        return candidate;
    }

    fs::path rebuilt;
    bool replacedImages = false;
    for (const auto& part : imagePath) {
        if (part == "images") {
            rebuilt /= "labels";
            replacedImages = true;
        } else {
            rebuilt /= part;
        }
    }

    if (replacedImages) {
        rebuilt.replace_extension(".txt");
        if (fs::exists(rebuilt)) {
            return rebuilt;
        }
    }

    return std::nullopt;
}

std::vector<std::string> SplitWhitespace(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<LabelObject> LoadLabels(const fs::path& labelPath, const cv::Size& imageSize) {
    std::ifstream input(labelPath);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open label file: " + labelPath.string());
    }

    std::vector<LabelObject> labels;
    std::string line;
    while (std::getline(input, line)) {
        const std::vector<std::string> tokens = SplitWhitespace(line);
        if (tokens.empty() || tokens.size() < 15) {
            continue;
        }

        LabelObject label;
        label.classId = std::stoi(tokens[0]);

        const double cx = std::stod(tokens[1]);
        const double cy = std::stod(tokens[2]);
        const double width = std::stod(tokens[3]);
        const double height = std::stod(tokens[4]);
        const int x1 = static_cast<int>(std::lround((cx - width / 2.0) * imageSize.width));
        const int y1 = static_cast<int>(std::lround((cy - height / 2.0) * imageSize.height));
        const int x2 = static_cast<int>(std::lround((cx + width / 2.0) * imageSize.width));
        const int y2 = static_cast<int>(std::lround((cy + height / 2.0) * imageSize.height));
        label.rect = ClampRect(cv::Rect(x1, y1, x2 - x1, y2 - y1), imageSize);

        label.hasKeypoints = true;
        for (int keypointIndex = 0; keypointIndex < 5; ++keypointIndex) {
            label.keypoints[static_cast<std::size_t>(keypointIndex)] = cv::Point2f(
                static_cast<float>(std::stod(tokens[5 + keypointIndex * 2]) * imageSize.width),
                static_cast<float>(std::stod(tokens[5 + keypointIndex * 2 + 1]) * imageSize.height));
        }

        labels.push_back(label);
    }

    return labels;
}

class YoloRunner {
public:
    explicit YoloRunner(const Options& options) : options_(options) {}

    void initialize() {
        if (!fs::exists(options_.modelPath)) {
            throw std::runtime_error("Model not found: " + options_.modelPath.string());
        }

#if HAVE_ONNXRUNTIME
        Ort::SessionOptions sessionOptions;
        sessionOptions.SetIntraOpNumThreads(2);
        sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
#ifdef _WIN32
        const std::wstring widePath(options_.modelPath.wstring());
        session_ = std::make_unique<Ort::Session>(env_, widePath.c_str(), sessionOptions);
#else
        session_ = std::make_unique<Ort::Session>(env_, options_.modelPath.string().c_str(), sessionOptions);
#endif
        std::cout << "Loaded model with ONNX Runtime: " << options_.modelPath.string() << std::endl;
#else
        net_ = cv::dnn::readNetFromONNX(options_.modelPath.string());
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        std::cout << "Loaded model with OpenCV DNN: " << options_.modelPath.string() << std::endl;
#endif
    }

    std::vector<Detection> infer(const cv::Mat& frame) {
        std::vector<Detection> detections;
        if (frame.empty()) {
            return detections;
        }

        const LetterboxTransform letterbox = ApplyLetterbox(frame, options_.inputWidth, options_.inputHeight);
        cv::Mat rgb;
        cv::cvtColor(letterbox.image, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32F, 1.0f / 255.0f);

        const int channels = 3;
        const int imgH = options_.inputHeight;
        const int imgW = options_.inputWidth;
        std::vector<float> inputData(static_cast<std::size_t>(channels * imgH * imgW));
        for (int c = 0; c < channels; ++c) {
            for (int h = 0; h < imgH; ++h) {
                for (int w = 0; w < imgW; ++w) {
                    inputData[static_cast<std::size_t>(c * imgH * imgW + h * imgW + w)] =
                        rgb.at<cv::Vec3f>(h, w)[c];
                }
            }
        }

        const float* outputData = nullptr;
        int numDetections = 0;
        int numCols = 0;

#if HAVE_ONNXRUNTIME
        const std::array<int64_t, 4> inputShape = {1, 3, imgH, imgW};
        Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memInfo, inputData.data(), inputData.size(), inputShape.data(), inputShape.size());

        Ort::AllocatorWithDefaultOptions alloc;
        auto inputName = session_->GetInputNameAllocated(0, alloc);
        auto outputName = session_->GetOutputNameAllocated(0, alloc);
        const char* inputNames[] = {inputName.get()};
        const char* outputNames[] = {outputName.get()};

        auto outputTensors = session_->Run(
            Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);
        const auto& outputTensor = outputTensors[0];
        const auto outputShape = outputTensor.GetTensorTypeAndShapeInfo().GetShape();
        if (outputShape.size() != 3) {
            throw std::runtime_error("Unexpected output rank: " + std::to_string(outputShape.size()));
        }

        numDetections = static_cast<int>(outputShape[1]);
        numCols = static_cast<int>(outputShape[2]);
        outputData = outputTensor.GetTensorData<float>();

        if (!printedShape_) {
            std::cout << "Model output shape: [" << outputShape[0] << ", "
                      << outputShape[1] << ", " << outputShape[2] << "]" << std::endl;
            printedShape_ = true;
        }
#else
        cv::Mat blob = cv::dnn::blobFromImage(
            letterbox.image, 1.0 / 255.0, cv::Size(imgW, imgH), cv::Scalar(), true, false);
        net_.setInput(blob);
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        if (outputs.empty()) {
            return detections;
        }
        cv::Mat output = outputs[0];
        if (output.dims == 3) {
            output = output.reshape(1, output.size[1]);
        }
        numDetections = output.rows;
        numCols = output.cols;
        outputData = output.ptr<float>(0);
        if (!printedShape_) {
            std::cout << "Model output shape: [" << 1 << ", "
                      << numDetections << ", " << numCols << "]" << std::endl;
            printedShape_ = true;
        }
#endif

        if (!outputData || numDetections <= 0 || numCols < 6) {
            return detections;
        }

        for (int i = 0; i < numDetections; ++i) {
            const float* row = outputData + static_cast<std::ptrdiff_t>(i) * numCols;
            const float confidence = row[4];
            if (confidence < options_.confidence) {
                continue;
            }

            Detection detection;
            const int x1 = static_cast<int>(std::lround(
                UndoLetterboxCoord(row[0], letterbox.padX, letterbox.scale, frame.cols)));
            const int y1 = static_cast<int>(std::lround(
                UndoLetterboxCoord(row[1], letterbox.padY, letterbox.scale, frame.rows)));
            const int x2 = static_cast<int>(std::lround(
                UndoLetterboxCoord(row[2], letterbox.padX, letterbox.scale, frame.cols)));
            const int y2 = static_cast<int>(std::lround(
                UndoLetterboxCoord(row[3], letterbox.padY, letterbox.scale, frame.rows)));
            detection.rect = ClampRect(cv::Rect(x1, y1, x2 - x1, y2 - y1), frame.size());
            if (detection.rect.width <= 0 || detection.rect.height <= 0) {
                continue;
            }

            detection.classId = static_cast<int>(row[5]);
            detection.confidence = confidence;

            if (numCols >= 16) {
                detection.hasKeypoints = true;
                for (int keypointIndex = 0; keypointIndex < 5; ++keypointIndex) {
                    detection.keypoints[static_cast<std::size_t>(keypointIndex)] = cv::Point2f(
                        UndoLetterboxCoord(row[6 + keypointIndex * 2], letterbox.padX, letterbox.scale, frame.cols),
                        UndoLetterboxCoord(row[6 + keypointIndex * 2 + 1], letterbox.padY, letterbox.scale, frame.rows));
                }
            }

            detections.push_back(detection);
        }

        return detections;
    }

private:
    Options options_;
    bool printedShape_ = false;

#if HAVE_ONNXRUNTIME
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "yolo_image_test"};
    std::unique_ptr<Ort::Session> session_;
#else
    cv::dnn::Net net_;
#endif
};

void DrawKeypointsAndRBox(cv::Mat& image,
                          const std::array<cv::Point2f, 5>& keypoints,
                          const cv::Size& frameSize,
                          const cv::Scalar& bladePointColor,
                          const cv::Scalar& rColor) {
    for (int keypointIndex = 0; keypointIndex < 5; ++keypointIndex) {
        const bool isCenterR = keypointIndex == kCenterRKeypointIndex;
        const cv::Scalar pointColor = isCenterR ? rColor : bladePointColor;
        const cv::Point point(
            static_cast<int>(std::lround(keypoints[static_cast<std::size_t>(keypointIndex)].x)),
            static_cast<int>(std::lround(keypoints[static_cast<std::size_t>(keypointIndex)].y)));
        cv::circle(image, point, isCenterR ? 6 : 4, pointColor, -1);
        cv::putText(image,
                    "kp" + std::to_string(keypointIndex + 1),
                    point + cv::Point(6, -6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, pointColor, 2);
    }

    const std::optional<cv::Rect> rRect = DeriveRRectFromKeypoints(keypoints, frameSize);
    if (rRect.has_value() && rRect->width > 0 && rRect->height > 0) {
        cv::rectangle(image, *rRect, rColor, 2);
        cv::putText(image,
                    "R box",
                    rRect->tl() + cv::Point(0, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, rColor, 2);
    }
}

void DrawLabels(cv::Mat& image, const std::vector<LabelObject>& labels) {
    for (std::size_t index = 0; index < labels.size(); ++index) {
        const LabelObject& label = labels[index];
        const cv::Scalar boxColor(255, 255, 0);
        cv::rectangle(image, label.rect, boxColor, 2);
        cv::putText(image,
                    "GT " + ClassName(label.classId),
                    label.rect.tl() + cv::Point(0, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, boxColor, 2);
        if (label.hasKeypoints) {
            DrawKeypointsAndRBox(
                image, label.keypoints, image.size(), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255));
        }
    }
}

void DrawPredictions(cv::Mat& image, const std::vector<Detection>& detections) {
    for (std::size_t index = 0; index < detections.size(); ++index) {
        const Detection& detection = detections[index];
        const cv::Scalar boxColor = (detection.classId == 0 || detection.classId == 2)
            ? cv::Scalar(0, 255, 0)
            : cv::Scalar(0, 165, 255);
        cv::rectangle(image, detection.rect, boxColor, 2);
        cv::putText(image,
                    "Pred " + ClassName(detection.classId) + " " + cv::format("%.3f", detection.confidence),
                    detection.rect.tl() + cv::Point(0, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, boxColor, 2);
        if (detection.hasKeypoints) {
            DrawKeypointsAndRBox(
                image, detection.keypoints, image.size(), cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255));
        }
    }
}

cv::Mat BuildComparisonCanvas(const cv::Mat& original,
                              const std::vector<LabelObject>& labels,
                              const std::vector<Detection>& detections,
                              bool hasLabels) {
    cv::Mat left = original.clone();
    cv::Mat right = original.clone();

    if (hasLabels) {
        DrawLabels(left, labels);
        cv::putText(left, "Ground Truth", cv::Point(20, 32),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 255, 0), 2);
    } else {
        cv::putText(left, "Ground Truth (missing label)", cv::Point(20, 32),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
    }

    DrawPredictions(right, detections);
    cv::putText(right, "Prediction", cv::Point(20, 32),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);

    cv::Mat canvas(original.rows, original.cols * 2, original.type(), cv::Scalar::all(0));
    left.copyTo(canvas(cv::Rect(0, 0, original.cols, original.rows)));
    right.copyTo(canvas(cv::Rect(original.cols, 0, original.cols, original.rows)));
    return canvas;
}

void WriteSummary(const fs::path& summaryPath,
                  const fs::path& imagePath,
                  const std::optional<fs::path>& labelPath,
                  const std::vector<LabelObject>& labels,
                  const std::vector<Detection>& detections,
                  const cv::Size& imageSize) {
    std::ofstream out(summaryPath);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open summary file: " + summaryPath.string());
    }

    out << "Image: " << imagePath.string() << "\n";
    out << "Label: " << (labelPath.has_value() ? labelPath->string() : "<missing>") << "\n";
    out << "Image size: " << imageSize.width << "x" << imageSize.height << "\n\n";

    out << "[GroundTruth]\n";
    for (std::size_t index = 0; index < labels.size(); ++index) {
        const LabelObject& label = labels[index];
        out << "  #" << index
            << " class=" << label.classId
            << " (" << ClassName(label.classId) << ")"
            << " box=(" << label.rect.x << "," << label.rect.y
            << "," << label.rect.width << "," << label.rect.height << ")\n";
        if (label.hasKeypoints) {
            for (int keypointIndex = 0; keypointIndex < 5; ++keypointIndex) {
                const cv::Point2f point = label.keypoints[static_cast<std::size_t>(keypointIndex)];
                out << "    kp" << (keypointIndex + 1) << "=(" << point.x << "," << point.y << ")\n";
            }
            const std::optional<cv::Rect> rRect = DeriveRRectFromKeypoints(label.keypoints, imageSize);
            if (rRect.has_value()) {
                out << "    derived_r_box=(" << rRect->x << "," << rRect->y
                    << "," << rRect->width << "," << rRect->height << ")\n";
            }
        }
    }

    out << "\n[Prediction]\n";
    for (std::size_t index = 0; index < detections.size(); ++index) {
        const Detection& detection = detections[index];
        out << "  #" << index
            << " class=" << detection.classId
            << " (" << ClassName(detection.classId) << ")"
            << " conf=" << detection.confidence
            << " box=(" << detection.rect.x << "," << detection.rect.y
            << "," << detection.rect.width << "," << detection.rect.height << ")\n";
        if (detection.hasKeypoints) {
            for (int keypointIndex = 0; keypointIndex < 5; ++keypointIndex) {
                const cv::Point2f point = detection.keypoints[static_cast<std::size_t>(keypointIndex)];
                out << "    kp" << (keypointIndex + 1) << "=(" << point.x << "," << point.y << ")\n";
            }
            const std::optional<cv::Rect> rRect = DeriveRRectFromKeypoints(detection.keypoints, imageSize);
            if (rRect.has_value()) {
                out << "    derived_r_box=(" << rRect->x << "," << rRect->y
                    << "," << rRect->width << "," << rRect->height << ")\n";
            }
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        Options options = ParseArgs(argc, argv);
        if (!fs::exists(options.imagePath)) {
            throw std::runtime_error("Image not found: " + options.imagePath.string());
        }

        std::optional<fs::path> labelPath;
        if (!options.labelPath.empty()) {
            if (!fs::exists(options.labelPath)) {
                throw std::runtime_error("Label not found: " + options.labelPath.string());
            }
            labelPath = options.labelPath;
        } else {
            labelPath = InferLabelPath(options.imagePath);
        }

        if (options.outputPath.empty()) {
            options.outputPath = DefaultOutputPath(options.imagePath, options.modelPath);
        }
        fs::create_directories(options.outputPath.parent_path());

        cv::Mat image = cv::imread(options.imagePath.string(), cv::IMREAD_COLOR);
        if (image.empty()) {
            throw std::runtime_error("Failed to load image: " + options.imagePath.string());
        }

        std::vector<LabelObject> labels;
        if (labelPath.has_value()) {
            labels = LoadLabels(*labelPath, image.size());
        }

        YoloRunner runner(options);
        runner.initialize();
        std::vector<Detection> detections = runner.infer(image);

        cv::Mat predictionOnly = image.clone();
        DrawPredictions(predictionOnly, detections);
        cv::putText(predictionOnly, "Prediction Only", cv::Point(20, 32),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);

        const cv::Mat comparison = BuildComparisonCanvas(image, labels, detections, !labels.empty());
        cv::imwrite(options.outputPath.string(), comparison);

        fs::path predictionPath = options.outputPath;
        predictionPath.replace_filename(options.outputPath.stem().string() + "__pred.jpg");
        cv::imwrite(predictionPath.string(), predictionOnly);

        fs::path summaryPath = options.outputPath;
        summaryPath.replace_extension(".txt");
        WriteSummary(summaryPath, options.imagePath, labelPath, labels, detections, image.size());

        if (options.show) {
            cv::namedWindow("yolo_image_test", cv::WINDOW_NORMAL);
            cv::imshow("yolo_image_test", comparison);
            cv::waitKey(0);
            cv::destroyWindow("yolo_image_test");
        }

        std::cout << "Image: " << options.imagePath.string() << std::endl;
        std::cout << "Label: " << (labelPath.has_value() ? labelPath->string() : "<missing>") << std::endl;
        std::cout << "Detections: " << detections.size() << std::endl;
        std::cout << "Compare image: " << options.outputPath.string() << std::endl;
        std::cout << "Prediction image: " << predictionPath.string() << std::endl;
        std::cout << "Summary: " << summaryPath.string() << std::endl;
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "yolo_image_test failed: " << ex.what() << std::endl;
        return 1;
    }
}
