#pragma once

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/image_encodings.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/core.hpp>

#include "core/buff_pipeline.hpp"
#include "core/hsv_detector.hpp"
#include "core/yolo_detector.hpp"

namespace gutcpp {

class BuffNode : public rclcpp::Node {
public:
    BuffNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void declareParameters();
    void loadRuntimeConfig();
    PipelineConfig buildPipelineConfig() const;
    Parameter buildParameter() const;
    bool ensureYoloAssistLoaded();
    bool initializePipelineFromSeed(const cv::Mat& frame,
                                    const DetectionResult& seed,
                                    const std::string& reason);
    bool initializePipelineWithStaticRoi(const cv::Mat& frame);
    bool tryYoloLock(const cv::Mat& frame, const std::string& reason);
    void publishDebugState(const PipelineOutput& output, bool found) const;
    void publishDebugImage(const std_msgs::msg::Header& header,
                           const cv::Mat& bgr,
                           const PipelineOutput& output,
                           bool found,
                           const std::string& statusText);

    std::unique_ptr<BuffPipeline> pipeline_;
    std::unique_ptr<YoloDetector> yoloAssist_;
    bool pipelineInitialized_ = false;
    bool useYoloAssist_ = false;
    bool publishDebugImage_ = true;
    bool showDebugWindow_ = false;
    int preferredYoloClassId_ = 2;
    int frameCount_ = 0;
    int lostFrames_ = 0;
    int lastYoloAttemptFrame_ = -1000000;
    int yoloRelockIntervalFrames_ = 3;
    int yoloRelockAfterMisses_ = 1;
    Parameter parameter_;
    PipelineConfig pipelineConfig_;
    std::optional<cv::Rect> staticRoi_;
    std::optional<cv::Rect> staticFanRoi_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr predictionPub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debugPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub_;
};

} // namespace gutcpp
