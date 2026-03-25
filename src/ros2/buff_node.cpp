#include "buff_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace gutcpp {

namespace fs = std::filesystem;

namespace {

cv::Rect BBoxToRect(const BBox& bbox) {
    const int width = std::max(1, static_cast<int>(std::lround(bbox.width())));
    const int height = std::max(1, static_cast<int>(std::lround(bbox.height())));
    return cv::Rect(static_cast<int>(std::lround(bbox.xmin)),
                    static_cast<int>(std::lround(bbox.ymin)),
                    width,
                    height);
}

std::optional<cv::Rect> ParseRoiParameter(const std::vector<int64_t>& values) {
    if (values.size() < 4) {
        return std::nullopt;
    }

    const int x = static_cast<int>(values[0]);
    const int y = static_cast<int>(values[1]);
    const int width = static_cast<int>(values[2]);
    const int height = static_cast<int>(values[3]);
    if (width <= 0 || height <= 0) {
        return std::nullopt;
    }
    return cv::Rect(x, y, width, height);
}

int PreferredYoloSeedClassId(const std::string& color) {
    return (color == "red") ? 1 : 2;
}

std::string ResolveRosPath(const std::string& rawPath) {
    if (rawPath.empty()) {
        return rawPath;
    }

    const fs::path inputPath(rawPath);
    if (inputPath.is_absolute()) {
        return inputPath.lexically_normal().string();
    }

    std::vector<fs::path> candidates;
    candidates.emplace_back((fs::current_path() / inputPath).lexically_normal());

    try {
        const fs::path packageShare(ament_index_cpp::get_package_share_directory("rm_buff_tracker"));
        candidates.emplace_back((packageShare / inputPath).lexically_normal());
    } catch (...) {
    }

    for (const fs::path& candidate : candidates) {
        if (fs::exists(candidate)) {
            return candidate.string();
        }
    }

    if (!candidates.empty()) {
        return candidates.back().string();
    }
    return inputPath.lexically_normal().string();
}

sensor_msgs::msg::Image MakeBgrImageMessage(const std_msgs::msg::Header& header, const cv::Mat& image) {
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(image.step);
    msg.data.assign(image.datastart, image.dataend);
    return msg;
}

void DrawBox(cv::Mat& frame, const BBox& bbox, const cv::Scalar& color, const std::string& label) {
    if (bbox.area() <= 0.0) {
        return;
    }
    const cv::Rect rect = BBoxToRect(bbox);
    cv::rectangle(frame, rect, color, 2);
    cv::putText(frame, label, rect.tl() + cv::Point(0, -8),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
}

} // namespace

BuffNode::BuffNode() : Node("buff_tracker_node") {
    declareParameters();
    loadRuntimeConfig();

    imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "~/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&BuffNode::imageCallback, this, std::placeholders::_1));

    predictionPub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "~/prediction", 10);

    debugPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/debug_state", 10);

    debugImagePub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "~/debug_image", 10);

    RCLCPP_INFO(this->get_logger(),
                "BuffNode initialized. detector=%s, waiting for images on ~/image_raw",
                useYoloAssist_ ? "yolo-hybrid" : "hsv");
}

void BuffNode::declareParameters() {
    this->declare_parameter<std::string>("color", "blue");
    this->declare_parameter<std::string>("mode", "small");
    this->declare_parameter<double>("delta_t", 0.2);
    this->declare_parameter<int>("freq", 50);

    this->declare_parameter<std::string>("detector_type", "hsv");
    this->declare_parameter<std::string>("onnx_model_path", "");
    this->declare_parameter<double>("yolo_confidence", 0.5);
    this->declare_parameter<double>("yolo_nms_threshold", 0.45);
    this->declare_parameter<int>("yolo_input_width", 640);
    this->declare_parameter<int>("yolo_input_height", 640);
    this->declare_parameter<int>("yolo_refresh_interval", 30);
    this->declare_parameter<int>("yolo_relock_interval_frames", 3);
    this->declare_parameter<int>("yolo_relock_after_misses", 1);
    this->declare_parameter<bool>("publish_debug_image", true);
    this->declare_parameter<bool>("show_debug_window", false);

    this->declare_parameter<bool>("enable_compensation", false);
    this->declare_parameter<double>("bullet_speed", 15.0);
    this->declare_parameter<double>("target_distance", 7.0);
    this->declare_parameter<double>("comm_latency_sec", 0.01);
    this->declare_parameter<double>("gimbal_delay_sec", 0.05);
    this->declare_parameter<double>("extra_delay_sec", 0.0);

    // HSV parameters
    this->declare_parameter<std::vector<int64_t>>("hsv_lower", {0, 100, 100});
    this->declare_parameter<std::vector<int64_t>>("hsv_upper", {15, 255, 255});
    this->declare_parameter<int>("kernel", 3);
    this->declare_parameter<double>("inside_rate", 0.6);
    this->declare_parameter<double>("outside_rate", 1.5);
    this->declare_parameter<std::vector<int64_t>>("static_r_roi", {});
    this->declare_parameter<std::vector<int64_t>>("static_fan_roi", {});
}

PipelineConfig BuffNode::buildPipelineConfig() const {
    PipelineConfig config;

    const std::string mode = this->get_parameter("mode").as_string();
    config.moveMode = (mode == "big") ? MoveMode::Big : MoveMode::Small;

    const std::string color = this->get_parameter("color").as_string();
    config.clockMode = (color == "red") ? ClockMode::Clockwise : ClockMode::Anticlockwise;

    config.deltaT = this->get_parameter("delta_t").as_double();
    config.freq = static_cast<int>(this->get_parameter("freq").as_int());
    config.enableCompensation = this->get_parameter("enable_compensation").as_bool();
    config.compensationConfig.bulletSpeed = this->get_parameter("bullet_speed").as_double();
    config.compensationConfig.targetDistance = this->get_parameter("target_distance").as_double();
    config.compensationConfig.commLatencySec = this->get_parameter("comm_latency_sec").as_double();
    config.compensationConfig.gimbalDelaySec = this->get_parameter("gimbal_delay_sec").as_double();
    config.compensationConfig.extraDelaySec = this->get_parameter("extra_delay_sec").as_double();

    return config;
}

Parameter BuffNode::buildParameter() const {
    Parameter param;

    const auto lower = this->get_parameter("hsv_lower").as_integer_array();
    const auto upper = this->get_parameter("hsv_upper").as_integer_array();
    if (lower.size() >= 3 && upper.size() >= 3) {
        param.hsv.lowerLimit = cv::Scalar(lower[0], lower[1], lower[2]);
        param.hsv.upperLimit = cv::Scalar(upper[0], upper[1], upper[2]);
    }
    param.kernel = static_cast<int>(this->get_parameter("kernel").as_int());
    param.insideRate = this->get_parameter("inside_rate").as_double();
    param.outsideRate = this->get_parameter("outside_rate").as_double();
    param.detectorType = this->get_parameter("detector_type").as_string();
    param.onnxModelPath = this->get_parameter("onnx_model_path").as_string();
    param.yoloConfidence = static_cast<float>(this->get_parameter("yolo_confidence").as_double());
    param.yoloNmsThreshold = static_cast<float>(this->get_parameter("yolo_nms_threshold").as_double());
    param.yoloInputWidth = static_cast<int>(this->get_parameter("yolo_input_width").as_int());
    param.yoloInputHeight = static_cast<int>(this->get_parameter("yolo_input_height").as_int());
    param.yoloRefreshInterval = static_cast<int>(this->get_parameter("yolo_refresh_interval").as_int());
    param.enableCompensation = this->get_parameter("enable_compensation").as_bool();
    param.bulletSpeed = this->get_parameter("bullet_speed").as_double();
    param.targetDistance = this->get_parameter("target_distance").as_double();
    param.commLatencySec = this->get_parameter("comm_latency_sec").as_double();
    param.gimbalDelaySec = this->get_parameter("gimbal_delay_sec").as_double();
    param.extraDelaySec = this->get_parameter("extra_delay_sec").as_double();

    return param;
}

void BuffNode::loadRuntimeConfig() {
    parameter_ = buildParameter();
    pipelineConfig_ = buildPipelineConfig();
    useYoloAssist_ = parameter_.detectorType == "yolo";
    publishDebugImage_ = this->get_parameter("publish_debug_image").as_bool();
    showDebugWindow_ = this->get_parameter("show_debug_window").as_bool();
    preferredYoloClassId_ = PreferredYoloSeedClassId(this->get_parameter("color").as_string());
    yoloRelockIntervalFrames_ = std::max(1, static_cast<int>(this->get_parameter("yolo_relock_interval_frames").as_int()));
    yoloRelockAfterMisses_ = std::max(1, static_cast<int>(this->get_parameter("yolo_relock_after_misses").as_int()));
    staticRoi_ = ParseRoiParameter(this->get_parameter("static_r_roi").as_integer_array());
    staticFanRoi_ = ParseRoiParameter(this->get_parameter("static_fan_roi").as_integer_array());
}

bool BuffNode::ensureYoloAssistLoaded() {
    if (!useYoloAssist_) {
        return false;
    }
    if (yoloAssist_) {
        return true;
    }

    YoloDetectorConfig yoloConfig;
    yoloConfig.modelPath = ResolveRosPath(parameter_.onnxModelPath);
    yoloConfig.confidence = parameter_.yoloConfidence;
    yoloConfig.nmsThreshold = parameter_.yoloNmsThreshold;
    yoloConfig.inputWidth = parameter_.yoloInputWidth;
    yoloConfig.inputHeight = parameter_.yoloInputHeight;
    yoloConfig.refreshInterval = parameter_.yoloRefreshInterval;

    yoloAssist_ = std::make_unique<YoloDetector>(yoloConfig, false);
    if (!yoloAssist_->loadModel()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO model: %s", yoloConfig.modelPath.c_str());
        yoloAssist_.reset();
        return false;
    }
    return true;
}

bool BuffNode::initializePipelineFromSeed(const cv::Mat& frame,
                                          const DetectionResult& seed,
                                          const std::string& reason) {
    auto hsvDetector = std::make_unique<HsvDetector>(false);
    auto newPipeline = std::make_unique<BuffPipeline>(std::move(hsvDetector), pipelineConfig_);
    if (!newPipeline->initialize(frame, parameter_, BBoxToRect(seed.rBox), BBoxToRect(seed.fanBladeBox))) {
        RCLCPP_WARN(this->get_logger(),
                    "YOLO %s succeeded but HSV tracker init failed", reason.c_str());
        return false;
    }

    pipeline_ = std::move(newPipeline);
    pipelineInitialized_ = true;
    lostFrames_ = 0;
    RCLCPP_INFO(this->get_logger(),
                "YOLO %s locked target: conf=%.3f, R=(%d,%d), fan=(%d,%d)",
                reason.c_str(),
                seed.confidence,
                seed.rBox.center2i().x,
                seed.rBox.center2i().y,
                seed.fanBladeBox.center2i().x,
                seed.fanBladeBox.center2i().y);
    return true;
}

bool BuffNode::initializePipelineWithStaticRoi(const cv::Mat& frame) {
    if (!staticRoi_.has_value() || !staticFanRoi_.has_value()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "HSV mode in ROS requires static_r_roi and static_fan_roi parameters");
        return false;
    }

    auto hsvDetector = std::make_unique<HsvDetector>(false);
    auto newPipeline = std::make_unique<BuffPipeline>(std::move(hsvDetector), pipelineConfig_);
    if (!newPipeline->initialize(frame, parameter_, staticRoi_.value(), staticFanRoi_.value())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize HSV pipeline from static ROI");
        return false;
    }

    pipeline_ = std::move(newPipeline);
    pipelineInitialized_ = true;
    lostFrames_ = 0;
    RCLCPP_INFO(this->get_logger(), "Pipeline initialized from static ROI hints");
    return true;
}

bool BuffNode::tryYoloLock(const cv::Mat& frame, const std::string& reason) {
    if (!ensureYoloAssistLoaded()) {
        return false;
    }

    lastYoloAttemptFrame_ = frameCount_;
    const std::optional<DetectionResult> seed = yoloAssist_->detectTarget(frame, preferredYoloClassId_);
    if (!seed.has_value()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "YOLO %s miss", reason.c_str());
        return false;
    }

    return initializePipelineFromSeed(frame, seed.value(), reason);
}

void BuffNode::publishDebugState(const PipelineOutput& output, bool found) const {
    auto debugMsg = std_msgs::msg::Float64MultiArray();
    debugMsg.data = {
        found ? 1.0 : 0.0,
        output.predictionReady ? 1.0 : 0.0,
        static_cast<double>(lostFrames_),
        output.observedAngle,
        output.rawAngle,
        output.deltaAngle,
        output.compensatedDelta,
        output.predictedPoint.x,
        output.predictedPoint.y,
        output.compensatedPoint.x,
        output.compensatedPoint.y,
        output.rBox.center2f().x,
        output.rBox.center2f().y,
        output.fanBladeBox.center2f().x,
        output.fanBladeBox.center2f().y,
        output.radius
    };
    debugPub_->publish(debugMsg);
}

void BuffNode::publishDebugImage(const std_msgs::msg::Header& header,
                                 const cv::Mat& bgr,
                                 const PipelineOutput& output,
                                 bool found,
                                 const std::string& statusText) {
    cv::Mat debugFrame = bgr.clone();
    if (found) {
        DrawBox(debugFrame, output.rBox, cv::Scalar(255, 0, 255), "R");
        DrawBox(debugFrame, output.fanBladeBox, cv::Scalar(0, 255, 0), "fan");
        if (output.predictionReady) {
            cv::circle(debugFrame,
                       cv::Point(static_cast<int>(std::lround(output.compensatedPoint.x)),
                                 static_cast<int>(std::lround(output.compensatedPoint.y))),
                       5, cv::Scalar(0, 0, 255), -1);
            cv::putText(debugFrame,
                        "pred",
                        cv::Point(static_cast<int>(std::lround(output.compensatedPoint.x)) + 8,
                                  static_cast<int>(std::lround(output.compensatedPoint.y)) - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2);
        }
    }

    cv::putText(debugFrame,
                statusText,
                cv::Point(20, 32),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255),
                2);

    if (publishDebugImage_) {
        debugImagePub_->publish(MakeBgrImageMessage(header, debugFrame));
    }
    if (showDebugWindow_) {
        cv::imshow("buff_tracker_debug", debugFrame);
        cv::waitKey(1);
    }
}

void BuffNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    ++frameCount_;

    // Convert sensor_msgs::Image to cv::Mat
    int cvType = CV_8UC3;
    if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cvType = CV_8UC1;
    } else if (msg->encoding == sensor_msgs::image_encodings::BGRA8 ||
               msg->encoding == sensor_msgs::image_encodings::RGBA8) {
        cvType = CV_8UC4;
    }

    cv::Mat frame(static_cast<int>(msg->height), static_cast<int>(msg->width),
                  cvType, const_cast<uint8_t*>(msg->data.data()),
                  static_cast<size_t>(msg->step));

    // Convert to BGR if needed
    cv::Mat bgr;
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(frame, bgr, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
    } else if (msg->encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cvtColor(frame, bgr, cv::COLOR_RGBA2BGR);
    } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    } else {
        bgr = frame.clone();
    }

    if (!pipelineInitialized_) {
        bool initialized = false;
        if (useYoloAssist_) {
            initialized = tryYoloLock(bgr, "init");
        } else {
            initialized = initializePipelineWithStaticRoi(bgr);
        }

        if (!initialized) {
            publishDebugState(PipelineOutput{}, false);
            publishDebugImage(msg->header, bgr, PipelineOutput{}, false, "waiting for init");
            return;
        }
    }

    PipelineOutput output = pipeline_->processFrame(bgr);
    if (output.rBox.area() == 0.0) {
        ++lostFrames_;

        if (useYoloAssist_) {
            const bool shouldTryRelock =
                (lostFrames_ >= yoloRelockAfterMisses_) &&
                ((frameCount_ - lastYoloAttemptFrame_) >= yoloRelockIntervalFrames_);
            if (shouldTryRelock && tryYoloLock(bgr, "relock")) {
                publishDebugState(PipelineOutput{}, false);
                publishDebugImage(msg->header, bgr, PipelineOutput{}, false, "YOLO relock");
                return;
            }
        }

        publishDebugState(output, false);
        publishDebugImage(msg->header, bgr, output, false, "tracker lost");
        return;
    }

    lostFrames_ = 0;
    publishDebugState(output, true);

    if (output.predictionReady) {
        auto pointMsg = geometry_msgs::msg::PointStamped();
        pointMsg.header.stamp = msg->header.stamp;
        pointMsg.header.frame_id = msg->header.frame_id;
        pointMsg.point.x = output.compensatedPoint.x;
        pointMsg.point.y = output.compensatedPoint.y;
        pointMsg.point.z = 0.0;
        predictionPub_->publish(pointMsg);
    }

    publishDebugImage(
        msg->header,
        bgr,
        output,
        true,
        output.predictionReady ? "tracking + prediction" : "tracking warmup");
}

} // namespace gutcpp
