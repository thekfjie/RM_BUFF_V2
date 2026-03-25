#pragma once

#include "detector_interface.hpp"
#include "buff_tracker.hpp"
#include "types.hpp"

#include <memory>
#include <vector>

#if HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#else
#include <opencv2/dnn.hpp>
#endif

namespace gutcpp {

struct YoloDetectorConfig {
    std::string modelPath;
    float confidence = 0.25f;
    float nmsThreshold = 0.45f;
    int inputWidth = 640;
    int inputHeight = 640;
    // Class IDs: 0=RR(red target), 1=RW(red hit), 2=BR(blue target), 3=BW(blue hit)
    int refreshInterval = 30;
};

class YoloDetector final : public DetectorInterface {
public:
    explicit YoloDetector(const YoloDetectorConfig& config, bool showDebug = false);

    bool loadModel();

    bool initialize(const cv::Mat& frame,
                   const Parameter& param,
                   std::optional<cv::Rect> rBoxHint = std::nullopt,
                   std::optional<cv::Rect> fanBoxHint = std::nullopt) override;

    DetectionResult detect(cv::Mat& frame) override;

    std::optional<DetectionResult> detectTarget(const cv::Mat& frame,
                                                std::optional<int> preferredClassId = std::nullopt);

    bool isInitialized() const override { return initialized_; }

    std::string name() const override { return "YOLO"; }

private:
    struct YoloPoseBox {
        cv::Rect rect;
        int classId = -1;
        float confidence = 0.0f;
        Keypoints keypoints;
    };

    std::vector<YoloPoseBox> runInference(const cv::Mat& frame);
    std::optional<DetectionResult> selectBestTarget(const std::vector<YoloPoseBox>& boxes,
                                                    const cv::Size& frameSize,
                                                    std::optional<int> preferredClassId) const;
    bool seedTracker(const cv::Mat& frame, const Parameter& param);
    bool seedTracker(const DetectionResult& detection, const Parameter& param);

    YoloDetectorConfig config_;
    std::unique_ptr<F_BuffTracker> tracker_;
    Parameter storedParam_;
    bool showDebug_;
    bool initialized_ = false;
    bool modelLoaded_ = false;
    int frameCounter_ = 0;
    bool returnSeedDetectionOnce_ = false;
    DetectionResult lastYoloResult_;

#if HAVE_ONNXRUNTIME
    Ort::Env ortEnv_{ORT_LOGGING_LEVEL_WARNING, "YoloDetector"};
    std::unique_ptr<Ort::Session> ortSession_;
#else
    cv::dnn::Net net_;
#endif
};

} // namespace gutcpp
