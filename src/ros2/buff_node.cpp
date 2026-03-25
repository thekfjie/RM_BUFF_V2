#ifdef AMENT_CMAKE_FOUND

#include "buff_node.hpp"

#include <opencv2/imgproc.hpp>

namespace gutcpp {

BuffNode::BuffNode() : Node("buff_tracker_node") {
    declareParameters();

    imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "~/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&BuffNode::imageCallback, this, std::placeholders::_1));

    predictionPub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "~/prediction", 10);

    debugPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/debug_state", 10);

    RCLCPP_INFO(this->get_logger(), "BuffNode initialized, waiting for images on ~/image_raw");
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

    return param;
}

void BuffNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert sensor_msgs::Image to cv::Mat
    int cvType = CV_8UC3;
    if (msg->encoding == "mono8") {
        cvType = CV_8UC1;
    } else if (msg->encoding == "bgra8" || msg->encoding == "rgba8") {
        cvType = CV_8UC4;
    }

    cv::Mat frame(static_cast<int>(msg->height), static_cast<int>(msg->width),
                  cvType, const_cast<uint8_t*>(msg->data.data()),
                  static_cast<size_t>(msg->step));

    // Convert to BGR if needed
    cv::Mat bgr;
    if (msg->encoding == "rgb8") {
        cv::cvtColor(frame, bgr, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "bgra8") {
        cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
    } else if (msg->encoding == "rgba8") {
        cv::cvtColor(frame, bgr, cv::COLOR_RGBA2BGR);
    } else {
        bgr = frame.clone();
    }

    if (!pipelineInitialized_) {
        const std::string detectorType = this->get_parameter("detector_type").as_string();
        std::unique_ptr<DetectorInterface> detector;

        if (detectorType == "yolo") {
            YoloDetectorConfig yoloConfig;
            yoloConfig.modelPath = this->get_parameter("onnx_model_path").as_string();
            yoloConfig.confidence = static_cast<float>(this->get_parameter("yolo_confidence").as_double());
            yoloConfig.nmsThreshold = static_cast<float>(this->get_parameter("yolo_nms_threshold").as_double());
            yoloConfig.inputWidth = static_cast<int>(this->get_parameter("yolo_input_width").as_int());
            yoloConfig.inputHeight = static_cast<int>(this->get_parameter("yolo_input_height").as_int());
            yoloConfig.refreshInterval = static_cast<int>(this->get_parameter("yolo_refresh_interval").as_int());

            auto yoloDet = std::make_unique<YoloDetector>(yoloConfig, false);
            const Parameter param = buildParameter();
            if (yoloDet->initialize(bgr, param)) {
                detector = std::move(yoloDet);
                RCLCPP_INFO(this->get_logger(), "YOLO detector initialized");
            } else {
                RCLCPP_WARN(this->get_logger(), "YOLO init failed, no fallback in ROS mode without ROI");
                return;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "HSV detector requires ROI hints in ROS mode");
            return;
        }

        const PipelineConfig pipeConfig = buildPipelineConfig();
        pipeline_ = std::make_unique<BuffPipeline>(std::move(detector), pipeConfig);
        const Parameter param = buildParameter();
        if (!pipeline_->initialize(bgr, param)) {
            RCLCPP_ERROR(this->get_logger(), "Pipeline initialization failed");
            pipeline_.reset();
            return;
        }

        pipelineInitialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Pipeline initialized successfully");
    }

    const PipelineOutput output = pipeline_->processFrame(bgr);

    if (output.predictionReady) {
        auto pointMsg = geometry_msgs::msg::PointStamped();
        pointMsg.header.stamp = msg->header.stamp;
        pointMsg.header.frame_id = msg->header.frame_id;
        pointMsg.point.x = output.compensatedPoint.x;
        pointMsg.point.y = output.compensatedPoint.y;
        pointMsg.point.z = 0.0;
        predictionPub_->publish(pointMsg);

        auto debugMsg = std_msgs::msg::Float64MultiArray();
        debugMsg.data = {
            output.observedAngle,
            output.rawAngle,
            output.deltaAngle,
            output.compensatedDelta,
            output.predictedPoint.x,
            output.predictedPoint.y
        };
        debugPub_->publish(debugMsg);
    }
}

} // namespace gutcpp

#endif // AMENT_CMAKE_FOUND
