#include "yolo_detector.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>

#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

namespace gutcpp {

namespace {

constexpr int kCenterRKeypointIndex = 2;
constexpr std::array<int, 4> kBladeKeypointIndices = {0, 1, 3, 4};

struct LetterboxTransform {
    cv::Mat image;
    float scale = 1.0f;
    int padX = 0;
    int padY = 0;
};

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

cv::Rect ClampRectToFrame(const cv::Rect& rect, const cv::Size& frameSize) {
    const int x = std::max(0, rect.x);
    const int y = std::max(0, rect.y);
    const int maxWidth = std::max(0, frameSize.width - x);
    const int maxHeight = std::max(0, frameSize.height - y);
    const int width = std::min(rect.width, maxWidth);
    const int height = std::min(rect.height, maxHeight);
    return cv::Rect(x, y, std::max(0, width), std::max(0, height));
}

cv::Rect ExpandRect(const cv::Rect& rect, const cv::Size& frameSize, double scaleX, double scaleY) {
    const int padX = static_cast<int>(std::lround(static_cast<double>(rect.width) * scaleX));
    const int padY = static_cast<int>(std::lround(static_cast<double>(rect.height) * scaleY));
    return ClampRectToFrame(
        cv::Rect(rect.x - padX, rect.y - padY, rect.width + 2 * padX, rect.height + 2 * padY),
        frameSize);
}

BBox RectToBBox(const cv::Rect& rect) {
    return BBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
}

cv::Rect RBoxFromCenter(const cv::Point2f& center, int sideLength, const cv::Size& frameSize) {
    const int half = sideLength / 2;
    return ClampRectToFrame(
        cv::Rect(static_cast<int>(std::lround(center.x)) - half,
                 static_cast<int>(std::lround(center.y)) - half,
                 sideLength,
                 sideLength),
        frameSize);
}

} // namespace

YoloDetector::YoloDetector(const YoloDetectorConfig& config, bool showDebug)
    : config_(config), showDebug_(showDebug) {}

bool YoloDetector::loadModel() {
    if (modelLoaded_) {
        return true;
    }
    if (config_.modelPath.empty() || !fs::exists(config_.modelPath)) {
        std::cerr << "[YoloDetector] Model not found: " << config_.modelPath << std::endl;
        return false;
    }

#if HAVE_ONNXRUNTIME
    try {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(2);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
#ifdef _WIN32
        // Convert UTF-8 path to wide string for Windows
        std::wstring wpath(config_.modelPath.begin(), config_.modelPath.end());
        ortSession_ = std::make_unique<Ort::Session>(ortEnv_, wpath.c_str(), opts);
#else
        ortSession_ = std::make_unique<Ort::Session>(ortEnv_, config_.modelPath.c_str(), opts);
#endif
    } catch (const Ort::Exception& e) {
        std::cerr << "[YoloDetector] ORT load failed: " << e.what() << std::endl;
        return false;
    }
    std::cout << "[YoloDetector] Model loaded with ONNX Runtime: " << config_.modelPath << std::endl;
#else
    try {
        net_ = cv::dnn::readNetFromONNX(config_.modelPath);
    } catch (const cv::Exception& e) {
        std::cerr << "[YoloDetector] OpenCV DNN load failed: " << e.what() << std::endl;
        return false;
    }
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    std::cout << "[YoloDetector] Model loaded with OpenCV DNN: " << config_.modelPath << std::endl;
#endif

    modelLoaded_ = true;
    return true;
}

bool YoloDetector::initialize(const cv::Mat& frame,
                               const Parameter& param,
                               std::optional<cv::Rect> /*rBoxHint*/,
                               std::optional<cv::Rect> /*fanBoxHint*/) {
    storedParam_ = param;

    if (!loadModel()) {
        return false;
    }

    if (!seedTracker(frame, param)) {
        std::cerr << "[YoloDetector] Initial detection failed" << std::endl;
        return false;
    }

    initialized_ = true;
    frameCounter_ = 0;
    returnSeedDetectionOnce_ = true;
    return true;
}

DetectionResult YoloDetector::detect(cv::Mat& frame) {
    DetectionResult result;
    if (!initialized_ || !tracker_) {
        return result;
    }

    if (returnSeedDetectionOnce_) {
        returnSeedDetectionOnce_ = false;
        return lastYoloResult_;
    }

    ++frameCounter_;

    if (frameCounter_ >= config_.refreshInterval) {
        if (seedTracker(frame, storedParam_)) {
            frameCounter_ = 0;
            if (showDebug_) {
                std::cout << "[YoloDetector] Periodic reseed succeeded" << std::endl;
            }
            return lastYoloResult_;
        }
        std::cerr << "[YoloDetector] Periodic reseed failed" << std::endl;
    }

    const bool ok = tracker_->update(frame, true);
    if (!ok) {
        std::cerr << "[YoloDetector] Tracker update failed: " << tracker_->lastFailureReason() << std::endl;
        if (seedTracker(frame, storedParam_)) {
            frameCounter_ = 0;
            std::cerr << "[YoloDetector] Recovered by reseeding tracker from YOLO output" << std::endl;
            return lastYoloResult_;
        }
        std::cerr << "[YoloDetector] Failed to recover from tracker loss via YOLO reseed" << std::endl;
        return result;
    }

    result.found = true;
    result.rBox = tracker_->rBox();
    result.fanBladeBox = tracker_->fanBladeBox();
    result.radius = tracker_->radius();
    result.keypoints = lastYoloResult_.keypoints;
    result.classId = lastYoloResult_.classId;
    result.confidence = lastYoloResult_.confidence;
    return result;
}

std::optional<DetectionResult> YoloDetector::detectTarget(const cv::Mat& frame,
                                                          std::optional<int> preferredClassId) {
    if (!loadModel()) {
        return std::nullopt;
    }

    const std::vector<YoloPoseBox> boxes = runInference(frame);
    return selectBestTarget(boxes, frame.size(), preferredClassId);
}

std::vector<YoloDetector::YoloPoseBox> YoloDetector::runInference(const cv::Mat& frame) {
    std::vector<YoloPoseBox> results;
    if (!modelLoaded_ || frame.empty()) {
        return results;
    }

    // Preprocess: Ultralytics-style letterbox + normalize to [1, 3, H, W] float32
    const LetterboxTransform letterbox = ApplyLetterbox(frame, config_.inputWidth, config_.inputHeight);
    cv::Mat rgb;
    cv::cvtColor(letterbox.image, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0f / 255.0f);

    // HWC -> CHW
    const int channels = 3;
    const int imgH = config_.inputHeight;
    const int imgW = config_.inputWidth;
    std::vector<float> inputData(channels * imgH * imgW);
    for (int c = 0; c < channels; ++c) {
        for (int h = 0; h < imgH; ++h) {
            for (int w = 0; w < imgW; ++w) {
                inputData[c * imgH * imgW + h * imgW + w] = rgb.at<cv::Vec3f>(h, w)[c];
            }
        }
    }

    // Run inference
    const float* outputData = nullptr;
    int numDetections = 0;
    int numCols = 0;

#if HAVE_ONNXRUNTIME
    const std::array<int64_t, 4> inputShape = {1, 3, imgH, imgW};
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memInfo, inputData.data(), inputData.size(), inputShape.data(), inputShape.size());

    Ort::AllocatorWithDefaultOptions alloc;
    auto inputName = ortSession_->GetInputNameAllocated(0, alloc);
    auto outputName = ortSession_->GetOutputNameAllocated(0, alloc);
    const char* inputNames[] = {inputName.get()};
    const char* outputNames[] = {outputName.get()};

    auto outputTensors = ortSession_->Run(
        Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);

    const auto& outTensor = outputTensors[0];
    const auto outShape = outTensor.GetTensorTypeAndShapeInfo().GetShape();
    // Expected: [1, 300, 16] for pose or [1, 300, 6] for detection-only
    numDetections = static_cast<int>(outShape[1]);
    numCols = static_cast<int>(outShape[2]);
    outputData = outTensor.GetTensorData<float>();

    if (showDebug_) {
        std::cout << "[YoloDetector] ORT output shape: [" << outShape[0];
        for (size_t i = 1; i < outShape.size(); ++i) std::cout << ", " << outShape[i];
        std::cout << "]" << std::endl;
    }
#else
    cv::Mat blob = cv::dnn::blobFromImage(
        letterbox.image, 1.0 / 255.0,
        cv::Size(config_.inputWidth, config_.inputHeight),
        cv::Scalar(), true, false);
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    if (outputs.empty()) return results;

    cv::Mat output = outputs[0];
    if (output.dims == 3) output = output.reshape(1, output.size[1]);
    numDetections = output.rows;
    numCols = output.cols;
    outputData = output.ptr<float>(0);
#endif

    if (!outputData || numDetections <= 0) {
        return results;
    }

    // Parse YOLO26 end-to-end output: [x1, y1, x2, y2, conf, class_id, kp1x, kp1y, ..., kp5x, kp5y]
    for (int i = 0; i < numDetections; ++i) {
        const float* row = outputData + i * numCols;

        const float conf = row[4];
        if (conf < config_.confidence) {
            continue;
        }

        const int classId = static_cast<int>(row[5]);

        const int x1 = static_cast<int>(std::lround(
            UndoLetterboxCoord(row[0], letterbox.padX, letterbox.scale, frame.cols)));
        const int y1 = static_cast<int>(std::lround(
            UndoLetterboxCoord(row[1], letterbox.padY, letterbox.scale, frame.rows)));
        const int x2 = static_cast<int>(std::lround(
            UndoLetterboxCoord(row[2], letterbox.padX, letterbox.scale, frame.cols)));
        const int y2 = static_cast<int>(std::lround(
            UndoLetterboxCoord(row[3], letterbox.padY, letterbox.scale, frame.rows)));

        YoloPoseBox box;
        box.rect = ClampRectToFrame(cv::Rect(x1, y1, x2 - x1, y2 - y1), frame.size());
        if (box.rect.width <= 0 || box.rect.height <= 0) {
            continue;
        }
        box.classId = classId;
        box.confidence = conf;

        // Extract 5 keypoints if present (cols 6-15)
        if (numCols >= 16) {
            box.keypoints.valid = true;
            for (int k = 0; k < 5; ++k) {
                box.keypoints.points[k] = cv::Point2f(
                    UndoLetterboxCoord(row[6 + k * 2], letterbox.padX, letterbox.scale, frame.cols),
                    UndoLetterboxCoord(row[6 + k * 2 + 1], letterbox.padY, letterbox.scale, frame.rows)
                );
            }
        }

        results.push_back(box);

        if (showDebug_) {
            std::cout << "[YoloDetector] Det: cls=" << classId
                      << " conf=" << conf
                      << " box=(" << x1 << "," << y1 << "," << x2 - x1 << "x" << y2 - y1 << ")"
                      << std::endl;
        }
    }

    return results;
}

std::optional<DetectionResult> YoloDetector::selectBestTarget(const std::vector<YoloPoseBox>& boxes,
                                                              const cv::Size& frameSize,
                                                              std::optional<int> preferredClassId) const {
    const YoloPoseBox* bestPreferred = nullptr;
    const YoloPoseBox* bestSameColorFallback = nullptr;
    const YoloPoseBox* bestAny = nullptr;

    auto sameColorFallbackClass = [](int classId) -> int {
        switch (classId) {
            case 0: return 1;
            case 1: return 0;
            case 2: return 3;
            case 3: return 2;
            default: return -1;
        }
    };
    const int fallbackClassId = preferredClassId.has_value()
        ? sameColorFallbackClass(preferredClassId.value())
        : -1;

    for (const auto& box : boxes) {
        if (box.classId < 0 || box.classId > 3) {
            continue;
        }

        if (!bestAny || box.confidence > bestAny->confidence) {
            bestAny = &box;
        }

        if (preferredClassId.has_value() && box.classId == preferredClassId.value()) {
            if (!bestPreferred || box.confidence > bestPreferred->confidence) {
                bestPreferred = &box;
            }
        }

        if (fallbackClassId >= 0 && box.classId == fallbackClassId) {
            if (!bestSameColorFallback || box.confidence > bestSameColorFallback->confidence) {
                bestSameColorFallback = &box;
            }
        }
    }

    const YoloPoseBox* bestTarget = bestPreferred
        ? bestPreferred
        : (bestSameColorFallback ? bestSameColorFallback : bestAny);
    if (!bestTarget) {
        if (showDebug_) {
            std::cout << "[YoloDetector] No target blade found in " << boxes.size() << " detections" << std::endl;
        }
        return std::nullopt;
    }

    if (!bestTarget->keypoints.valid) {
        std::cerr << "[YoloDetector] Best target has no keypoints, cannot infer R box" << std::endl;
        return std::nullopt;
    }

    std::vector<cv::Point2f> bladePoints;
    bladePoints.reserve(kBladeKeypointIndices.size());
    for (const int keypointIndex : kBladeKeypointIndices) {
        bladePoints.push_back(bestTarget->keypoints.points[static_cast<std::size_t>(keypointIndex)]);
    }

    cv::Rect fanRect = ClampRectToFrame(cv::boundingRect(bladePoints), frameSize);
    if (fanRect.width <= 0 || fanRect.height <= 0) {
        fanRect = ClampRectToFrame(bestTarget->rect, frameSize);
    } else {
        fanRect = ExpandRect(fanRect, frameSize, 0.08, 0.08);
    }

    const cv::Point2f rCenter =
        bestTarget->keypoints.points[static_cast<std::size_t>(kCenterRKeypointIndex)];
    const int baseSide = std::max(16, static_cast<int>(std::lround(
        static_cast<double>(std::min(fanRect.width, fanRect.height)) * 0.30)));
    const cv::Rect rRect = RBoxFromCenter(rCenter, std::min(baseSide, 48), frameSize);
    if (rRect.width <= 0 || rRect.height <= 0) {
        std::cerr << "[YoloDetector] Inferred R box is empty" << std::endl;
        return std::nullopt;
    }

    DetectionResult result;
    result.found = true;
    result.rBox = RectToBBox(rRect);
    result.fanBladeBox = RectToBBox(fanRect);
    result.radius = result.rBox.centerDistance(result.fanBladeBox);
    result.keypoints = bestTarget->keypoints;
    result.classId = bestTarget->classId;
    result.confidence = bestTarget->confidence;
    return result;
}

bool YoloDetector::seedTracker(const cv::Mat& frame, const Parameter& param) {
    const std::optional<DetectionResult> detection = detectTarget(frame);
    if (!detection.has_value()) {
        return false;
    }

    return seedTracker(detection.value(), param);
}

bool YoloDetector::seedTracker(const DetectionResult& detection, const Parameter& param) {
    tracker_ = std::make_unique<F_BuffTracker>(detection.fanBladeBox, detection.rBox, param, showDebug_);
    lastYoloResult_ = detection;

    if (showDebug_) {
        const cv::Point rCenter = detection.rBox.center2i();
        const cv::Point fanCenter = detection.fanBladeBox.center2i();
        std::cout << "[YoloDetector] Seeded: cls=" << detection.classId
                  << " conf=" << detection.confidence
                  << " R=(" << rCenter.x << "," << rCenter.y << ")"
                  << " Fan=(" << fanCenter.x << "," << fanCenter.y
                  << "," << detection.fanBladeBox.width() << "x" << detection.fanBladeBox.height() << ")"
                  << std::endl;
    }

    return true;
}

} // namespace gutcpp
