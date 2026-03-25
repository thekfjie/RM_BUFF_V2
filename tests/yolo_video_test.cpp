#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "core/parameter.hpp"

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
    fs::path pythonRoot;
    fs::path parameterPath;
    fs::path videoPath;
    fs::path modelPath;
    fs::path outputPath;
    float confidence = 0.25f;
    int inputWidth = 640;
    int inputHeight = 640;
    int startFrame = -1;
    int maxFrames = -1;
    int waitMs = 1;
    bool show = false;
    bool confidenceExplicit = false;
    bool startFrameExplicit = false;
};

struct Detection {
    cv::Rect rect;
    int classId = -1;
    float confidence = 0.0f;
    std::array<cv::Point2f, 5> keypoints{};
    bool hasKeypoints = false;
};

fs::path ProjectRoot() {
    return fs::path(__FILE__).parent_path().parent_path().lexically_normal();
}

fs::path DefaultPythonRoot() {
    return ProjectRoot().parent_path().lexically_normal();
}

void PrintUsage() {
    std::cout
        << "Usage: yolo_video_test --model <best.onnx> [--parameter <parameter.yaml> | --video <video>]\n"
        << "                      [--python-root <path>] [--output <annotated.avi>]\n"
        << "                      [--conf <float>] [--start <frame>] [--max-frames <int>]\n"
        << "                      [--show] [--wait-ms <int>]\n";
}

std::string ToLower(std::string value) {
    for (char& ch : value) {
        if (ch >= 'A' && ch <= 'Z') {
            ch = static_cast<char>(ch - 'A' + 'a');
        }
    }
    return value;
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

std::optional<cv::Rect> DeriveFanRectFromKeypoints(const Detection& detection, const cv::Size& frameSize) {
    if (!detection.hasKeypoints) {
        return std::nullopt;
    }

    std::vector<cv::Point2f> bladePoints;
    bladePoints.reserve(kBladeKeypointIndices.size());
    for (const int keypointIndex : kBladeKeypointIndices) {
        bladePoints.push_back(detection.keypoints[static_cast<std::size_t>(keypointIndex)]);
    }

    cv::Rect fanRect = ClampRect(cv::boundingRect(bladePoints), frameSize);
    if (fanRect.width <= 0 || fanRect.height <= 0) {
        return std::nullopt;
    }

    return ExpandRect(fanRect, frameSize, 0.08, 0.08);
}

std::optional<cv::Rect> DeriveRRectFromKeypoints(const Detection& detection, const cv::Size& frameSize) {
    const std::optional<cv::Rect> fanRect = DeriveFanRectFromKeypoints(detection, frameSize);
    if (!fanRect.has_value()) {
        return std::nullopt;
    }

    const cv::Point2f rCenter =
        detection.keypoints[static_cast<std::size_t>(kCenterRKeypointIndex)];
    const int baseSide = std::max(
        16,
        static_cast<int>(std::lround(
            static_cast<double>(std::min(fanRect->width, fanRect->height)) * 0.30)));
    return RBoxFromCenter(rCenter, std::min(baseSide, 48), frameSize);
}

Options ParseArgs(int argc, char** argv) {
    Options options;
    options.pythonRoot = DefaultPythonRoot();

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
        if (arg == "--python-root") {
            options.pythonRoot = requireValue(i, arg);
        } else if (arg == "--parameter") {
            options.parameterPath = requireValue(i, arg);
        } else if (arg == "--video") {
            options.videoPath = requireValue(i, arg);
        } else if (arg == "--model" || arg == "--onnx") {
            options.modelPath = requireValue(i, arg);
        } else if (arg == "--output") {
            options.outputPath = requireValue(i, arg);
        } else if (arg == "--conf") {
            options.confidence = std::stof(requireValue(i, arg));
            options.confidenceExplicit = true;
        } else if (arg == "--start") {
            options.startFrame = std::stoi(requireValue(i, arg));
            options.startFrameExplicit = true;
        } else if (arg == "--max-frames") {
            options.maxFrames = std::stoi(requireValue(i, arg));
        } else if (arg == "--show") {
            options.show = true;
        } else if (arg == "--wait-ms") {
            options.waitMs = std::max(1, std::stoi(requireValue(i, arg)));
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.modelPath.empty()) {
        throw std::runtime_error("--model is required");
    }
    if (options.parameterPath.empty() && options.videoPath.empty()) {
        throw std::runtime_error("Either --parameter or --video is required");
    }

    return options;
}

fs::path DefaultOutputPath(const fs::path& videoPath, const fs::path& modelPath) {
    fs::path outDir = ProjectRoot() / "tests" / "output";
    const std::string filename = videoPath.stem().string() + "__" + modelPath.stem().string() + "__annotated.avi";
    return outDir / filename;
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
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "yolo_video_test"};
    std::unique_ptr<Ort::Session> session_;
#else
    cv::dnn::Net net_;
#endif
};

void DrawDetections(cv::Mat& frame,
                    const std::vector<Detection>& detections,
                    std::optional<std::size_t> bestTargetIndex,
                    int sourceFrameIndex) {
    for (std::size_t index = 0; index < detections.size(); ++index) {
        const Detection& detection = detections[index];
        const bool isBestTarget = bestTargetIndex.has_value() && bestTargetIndex.value() == index;
        const cv::Scalar color = isBestTarget ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
        cv::rectangle(frame, detection.rect, color, isBestTarget ? 3 : 2);

        std::string label = ClassName(detection.classId) + " conf=" + cv::format("%.3f", detection.confidence);
        cv::putText(frame, label,
                    detection.rect.tl() + cv::Point(0, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

        if (detection.hasKeypoints) {
            for (int keypointIndex = 0; keypointIndex < 5; ++keypointIndex) {
                const cv::Scalar keypointColor = (keypointIndex == kCenterRKeypointIndex)
                    ? cv::Scalar(255, 0, 255)
                    : cv::Scalar(255, 255, 0);
                const cv::Point point(
                    static_cast<int>(std::lround(detection.keypoints[static_cast<std::size_t>(keypointIndex)].x)),
                    static_cast<int>(std::lround(detection.keypoints[static_cast<std::size_t>(keypointIndex)].y)));
                cv::circle(frame, point, (keypointIndex == kCenterRKeypointIndex) ? 5 : 3, keypointColor, -1);
            }

            const std::optional<cv::Rect> rRect = DeriveRRectFromKeypoints(detection, frame.size());
            if (rRect.has_value() && rRect->width > 0 && rRect->height > 0) {
                cv::rectangle(frame, *rRect, cv::Scalar(255, 0, 255), 2);
                cv::putText(frame,
                            "R box",
                            rRect->tl() + cv::Point(0, -8),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
            }
        }
    }

    cv::putText(frame,
                "frame=" + std::to_string(sourceFrameIndex) + " dets=" + std::to_string(detections.size()),
                cv::Point(20, 32),
                cv::FONT_HERSHEY_SIMPLEX, 0.85, cv::Scalar(0, 255, 255), 2);
}

} // namespace

int main(int argc, char** argv) {
    try {
        Options options = ParseArgs(argc, argv);

        std::optional<gutcpp::Parameter> parameter;
        if (!options.parameterPath.empty()) {
            const fs::path resolvedParameterPath =
                gutcpp::ResolveParameterPath(options.pythonRoot, options.parameterPath);
            parameter = gutcpp::LoadParameter(resolvedParameterPath);
            if (options.videoPath.empty()) {
                options.videoPath = gutcpp::ResolveVideoPath(parameter.value(), options.pythonRoot);
            }
            if (!options.startFrameExplicit) {
                options.startFrame = parameter->start;
            }
            if (!options.confidenceExplicit && parameter->yoloConfidence > 0.0f) {
                options.confidence = parameter->yoloConfidence;
            }
        }

        if (options.outputPath.empty()) {
            options.outputPath = DefaultOutputPath(options.videoPath, options.modelPath);
        }
        fs::create_directories(options.outputPath.parent_path());

        cv::VideoCapture capture(options.videoPath.string());
        if (!capture.isOpened()) {
            throw std::runtime_error("Failed to open video: " + options.videoPath.string());
        }

        if (options.startFrame > 0) {
            capture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(options.startFrame));
        }

        cv::Mat frame;
        if (!capture.read(frame) || frame.empty()) {
            throw std::runtime_error("Failed to read first frame from: " + options.videoPath.string());
        }

        const double fps = std::max(1.0, capture.get(cv::CAP_PROP_FPS));
        cv::VideoWriter writer(
            options.outputPath.string(),
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            fps,
            frame.size());
        if (!writer.isOpened()) {
            throw std::runtime_error("Failed to open output writer: " + options.outputPath.string());
        }

        fs::path csvPath = options.outputPath;
        csvPath.replace_extension(".csv");
        std::ofstream csv(csvPath);
        if (!csv.is_open()) {
            throw std::runtime_error("Failed to open CSV output: " + csvPath.string());
        }
        csv << "frame,class_id,class_name,confidence,x,y,w,h,r_x,r_y\n";

        YoloRunner runner(options);
        runner.initialize();

        if (options.show) {
            cv::namedWindow("yolo_video_test", cv::WINDOW_NORMAL);
        }

        const int startFrame = (options.startFrame > 0) ? options.startFrame : 0;
        int processedFrames = 0;
        int framesWithDetections = 0;
        bool previewSaved = false;

        do {
            const int sourceFrameIndex = startFrame + processedFrames;
            std::vector<Detection> detections = runner.infer(frame);

            std::optional<std::size_t> bestTargetIndex;
            for (std::size_t index = 0; index < detections.size(); ++index) {
                const Detection& detection = detections[index];
                if (detection.classId != 0 && detection.classId != 2) {
                    continue;
                }
                if (!bestTargetIndex.has_value() ||
                    detection.confidence > detections[bestTargetIndex.value()].confidence) {
                    bestTargetIndex = index;
                }
            }

            if (!detections.empty()) {
                ++framesWithDetections;
            }

            for (const Detection& detection : detections) {
                csv << sourceFrameIndex << ','
                    << detection.classId << ','
                    << ClassName(detection.classId) << ','
                    << detection.confidence << ','
                    << detection.rect.x << ','
                    << detection.rect.y << ','
                    << detection.rect.width << ','
                    << detection.rect.height << ',';
                if (detection.hasKeypoints) {
                    csv << detection.keypoints[static_cast<std::size_t>(kCenterRKeypointIndex)].x << ','
                        << detection.keypoints[static_cast<std::size_t>(kCenterRKeypointIndex)].y;
                } else {
                    csv << ','
                        << ',';
                }
                csv << '\n';
            }

            cv::Mat annotated = frame.clone();
            DrawDetections(annotated, detections, bestTargetIndex, sourceFrameIndex);
            writer.write(annotated);

            if (options.show) {
                cv::imshow("yolo_video_test", annotated);
                const int key = cv::waitKey(options.waitMs);
                if (key == 27 || key == 'q' || key == 'Q') {
                    std::cout << "Preview interrupted by user at frame " << sourceFrameIndex << std::endl;
                    break;
                }
            }

            if (!previewSaved) {
                fs::path previewPath = options.outputPath;
                previewPath.replace_extension(".jpg");
                cv::imwrite(previewPath.string(), annotated);
                previewSaved = true;
            }

            ++processedFrames;
            if (processedFrames % 100 == 0) {
                std::cout << "Processed " << processedFrames
                          << " frames, detections on " << framesWithDetections
                          << " frames" << std::endl;
            }
            if (options.maxFrames > 0 && processedFrames >= options.maxFrames) {
                break;
            }
        } while (capture.read(frame) && !frame.empty());

        if (options.show) {
            cv::destroyWindow("yolo_video_test");
        }

        std::cout << "Video: " << options.videoPath.string() << std::endl;
        std::cout << "Model: " << options.modelPath.string() << std::endl;
        std::cout << "Processed frames: " << processedFrames << std::endl;
        std::cout << "Frames with detections: " << framesWithDetections << std::endl;
        std::cout << "Annotated video: " << options.outputPath.string() << std::endl;
        std::cout << "Detection CSV: " << csvPath.string() << std::endl;
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "yolo_video_test failed: " << ex.what() << std::endl;
        return 1;
    }
}
