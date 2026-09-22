#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rm_buff_tracker/msg/buff_observation.hpp>

#include "buff_ros_utils.hpp"
#include "buff_spatial_utils.hpp"
#include "core/buff_pipeline.hpp"
#include "core/camera_geometry.hpp"
#include "core/hsv_detector.hpp"
#include "core/yolo_detector.hpp"
#include "latest_frame_mailbox.hpp"

namespace gutcpp {

namespace {

constexpr const char* kSourceNone = "NONE";
constexpr const char* kSourcePixelRay = "PIXEL_RAY";
constexpr const char* kSourcePnp = "PNP";

} // namespace

class BuffDetectorNode final : public rclcpp::Node {
public:
    BuffDetectorNode()
        : Node("buff_detector_node") {
        declareParameters();
        loadRuntimeConfig();

        const std::string imageTopic = this->get_parameter("image_topic").as_string();
        const std::string cameraInfoTopic = this->get_parameter("camera_info_topic").as_string();
        const std::string observationTopic = this->get_parameter("observation_topic").as_string();
        const std::string debugImageTopic = this->get_parameter("debug_image_topic").as_string();

        imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            imageTopic, rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&BuffDetectorNode::imageCallback, this, std::placeholders::_1));
        cameraInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            cameraInfoTopic, rclcpp::SensorDataQoS(),
            std::bind(&BuffDetectorNode::cameraInfoCallback, this, std::placeholders::_1));
        observationPub_ = this->create_publisher<rm_buff_tracker::msg::BuffObservation>(
            observationTopic, rclcpp::SensorDataQoS());
        debugImagePub_ = this->create_publisher<sensor_msgs::msg::Image>(
            debugImageTopic, rclcpp::SensorDataQoS());

        workerThread_ = std::thread(&BuffDetectorNode::workerLoop, this);

        RCLCPP_INFO(this->get_logger(),
                    "BUFF detector channel started: image=%s camera_info=%s observation=%s",
                    imageTopic.c_str(), cameraInfoTopic.c_str(), observationTopic.c_str());
    }

    ~BuffDetectorNode() override {
        mailbox_.stop();
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }

private:
    void declareParameters() {
        this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("camera_info_topic", "/camera_info");
        this->declare_parameter<std::string>("observation_topic", "/buff/detector/observation");
        this->declare_parameter<std::string>("debug_image_topic", "/buff/detector/debug_image");

        this->declare_parameter<std::string>("color", "blue");
        this->declare_parameter<std::string>("mode", "small");
        this->declare_parameter<double>("delta_t", 0.2);
        this->declare_parameter<int>("freq", 50);
        DeclareBigPredictorParameters(*this);

        this->declare_parameter<std::string>("detector_type", "yolo");
        this->declare_parameter<std::string>("onnx_model_path", "models/best.onnx");
        this->declare_parameter<double>("yolo_confidence", 0.5);
        this->declare_parameter<double>("yolo_nms_threshold", 0.45);
        this->declare_parameter<int>("yolo_input_width", 640);
        this->declare_parameter<int>("yolo_input_height", 640);
        this->declare_parameter<int>("yolo_refresh_interval", 1);
        this->declare_parameter<bool>("yolo_show_debug", false);
        this->declare_parameter<bool>("publish_debug_image", true);
        this->declare_parameter<bool>("show_debug_window", false);

        this->declare_parameter<bool>("enable_compensation", false);
        this->declare_parameter<double>("bullet_speed", 15.0);
        this->declare_parameter<double>("target_distance", 7.0);
        this->declare_parameter<double>("comm_latency_sec", 0.01);
        this->declare_parameter<double>("gimbal_delay_sec", 0.05);
        this->declare_parameter<double>("extra_delay_sec", 0.0);

        this->declare_parameter<bool>("enable_pnp", false);
        this->declare_parameter<double>("pnp_depth_max_age", 0.15);
        this->declare_parameter<double>("pnp_max_reprojection_error", 3.0);
        this->declare_parameter<double>("max_image_age", 0.25);
        this->declare_parameter<std::vector<double>>("pnp_object_points", {});

        this->declare_parameter<std::vector<int64_t>>("hsv_lower", {90, 100, 100});
        this->declare_parameter<std::vector<int64_t>>("hsv_upper", {130, 255, 255});
        this->declare_parameter<int>("kernel", 3);
        this->declare_parameter<double>("inside_rate", 0.6);
        this->declare_parameter<double>("outside_rate", 1.5);
        this->declare_parameter<std::vector<int64_t>>("static_r_roi", {});
        this->declare_parameter<std::vector<int64_t>>("static_fan_roi", {});
    }

    PipelineConfig buildPipelineConfig() const {
        PipelineConfig config;

        const std::string mode = this->get_parameter("mode").as_string();
        config.moveMode = (mode == "big") ? MoveMode::Big : MoveMode::Small;

        config.clockMode = ClockMode::Automatic;

        config.deltaT = this->get_parameter("delta_t").as_double();
        config.freq = static_cast<int>(this->get_parameter("freq").as_int());
        config.bigPredictorConfig = ReadBigPredictorConfig(*this);
        config.enableCompensation = this->get_parameter("enable_compensation").as_bool();
        config.compensationConfig.bulletSpeed = this->get_parameter("bullet_speed").as_double();
        config.compensationConfig.targetDistance = this->get_parameter("target_distance").as_double();
        config.compensationConfig.commLatencySec = this->get_parameter("comm_latency_sec").as_double();
        config.compensationConfig.gimbalDelaySec = this->get_parameter("gimbal_delay_sec").as_double();
        config.compensationConfig.extraDelaySec = this->get_parameter("extra_delay_sec").as_double();

        return config;
    }

    Parameter buildParameter() const {
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

    void loadRuntimeConfig() {
        parameter_ = buildParameter();
        pipelineConfig_ = buildPipelineConfig();
        publishDebugImage_ = this->get_parameter("publish_debug_image").as_bool();
        showDebugWindow_ = this->get_parameter("show_debug_window").as_bool();
        enablePnp_ = this->get_parameter("enable_pnp").as_bool();
        depthMaxAge_ = this->get_parameter("pnp_depth_max_age").as_double();
        maxReprojectionError_ = this->get_parameter("pnp_max_reprojection_error").as_double();
        maxImageAge_ = this->get_parameter("max_image_age").as_double();
        targetDistance_ = this->get_parameter("target_distance").as_double();
        pnpObjectPoints_ = ParseObjectPoints(this->get_parameter("pnp_object_points").as_double_array());
        staticRoi_ = ParseRoiParameter(GetIntegerArrayParameterOrEmpty(*this, "static_r_roi"));
        staticFanRoi_ = ParseRoiParameter(GetIntegerArrayParameterOrEmpty(*this, "static_fan_roi"));

        if (enablePnp_ && pnpObjectPoints_.size() != 4)
            throw std::invalid_argument("PnP requires exactly four measured blade points in keypoint order 0,1,3,4");
        if (!std::isfinite(targetDistance_) || targetDistance_ <= 0.0 ||
            !std::isfinite(depthMaxAge_) || depthMaxAge_ < 0.0 ||
            !std::isfinite(maxReprojectionError_) || maxReprojectionError_ <= 0.0 ||
            !std::isfinite(maxImageAge_) || maxImageAge_ <= 0.0)
            throw std::invalid_argument("Invalid BUFF distance, depth or image age parameters");
    }

    std::unique_ptr<DetectorInterface> createDetector() const {
        if (parameter_.detectorType == "yolo") {
            YoloDetectorConfig yoloConfig;
            yoloConfig.modelPath = ResolveRosPath(parameter_.onnxModelPath);
            yoloConfig.confidence = parameter_.yoloConfidence;
            yoloConfig.nmsThreshold = parameter_.yoloNmsThreshold;
            yoloConfig.inputWidth = parameter_.yoloInputWidth;
            yoloConfig.inputHeight = parameter_.yoloInputHeight;
            yoloConfig.refreshInterval = parameter_.yoloRefreshInterval;
            yoloConfig.preferredClassId =
                PreferredYoloSeedClassId(this->get_parameter("color").as_string());
            return std::make_unique<YoloDetector>(yoloConfig, this->get_parameter("yolo_show_debug").as_bool());
        }

        return std::make_unique<HsvDetector>(false);
    }

    bool initializePipeline(const cv::Mat& frame) {
        if (parameter_.detectorType != "yolo" && (!staticRoi_.has_value() || !staticFanRoi_.has_value())) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "HSV detector requires static_r_roi and static_fan_roi");
            return false;
        }

        auto newPipeline = std::make_unique<BuffPipeline>(createDetector(), pipelineConfig_);
        if (!newPipeline->initialize(frame, parameter_, staticRoi_, staticFanRoi_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "BUFF detector pipeline is waiting for a valid first target");
            return false;
        }

        pipeline_ = std::move(newPipeline);
        pipelineInitialized_ = true;
        pnpSolver_.reset();
        lostFrames_ = 0;
        RCLCPP_INFO(this->get_logger(), "BUFF detector pipeline initialized using %s", parameter_.detectorType.c_str());
        return true;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if ((!msg->distortion_model.empty() && msg->distortion_model != "plumb_bob" &&
             msg->distortion_model != "rational_polynomial") ||
            (!msg->d.empty() && !IsSupportedDistortionSize(msg->d.size()))) {
            std::lock_guard<std::mutex> lock(cameraMutex_);
            cameraInfoReady_ = false;
            RCLCPP_WARN(this->get_logger(), "Unsupported CameraInfo distortion model/count");
            return;
        }
        CameraModel camera;
        camera.cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        if (msg->d.empty()) {
            camera.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
        } else if (IsSupportedDistortionSize(msg->d.size())) {
            camera.distCoeffs = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F,
                                        const_cast<double*>(msg->d.data())).clone();
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Unsupported distortion coefficient count %zu, using zero distortion",
                                 msg->d.size());
            camera.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
        }
        camera.imageWidth = static_cast<int>(msg->width);
        camera.imageHeight = static_cast<int>(msg->height);
        camera.frameId = msg->header.frame_id.empty() ? "camera_optical_frame" : msg->header.frame_id;

        {
            std::lock_guard<std::mutex> lock(cameraMutex_);
            cameraModel_ = std::move(camera);
            cameraInfoReady_ = IsCameraModelUsable(cameraModel_);
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        mailbox_.put(MakeFramePacket(*msg));
    }

    void workerLoop() {
        while (rclcpp::ok()) {
            const std::optional<FramePacket> packet = mailbox_.waitAndTake();
            if (!packet.has_value()) {
                return;
            }
            try {
                processFrame(packet.value());
            } catch (const std::exception& ex) {
                pipelineInitialized_ = false;
                depth_.reset();
                pnpSolver_.reset();
                publishObservation(packet->header, PipelineOutput{}, false);
                RCLCPP_ERROR(this->get_logger(), "BUFF frame rejected: %s", ex.what());
            }
        }
    }

    void processFrame(const FramePacket& packet) {
        ++frameCount_;
        const rclcpp::Time stamp(packet.header.stamp);
        const double age = (this->now() - stamp).seconds();
        if (stamp.nanoseconds() == 0 || age < 0.0 || age > maxImageAge_) {
            depth_.reset();
            pnpSolver_.reset();
            if (pipeline_) pipeline_->resetMotion();
            publishObservation(packet.header, PipelineOutput{}, false);
            return;
        }
        const cv::Mat bgr = MakeBgrFrame(packet);
        if (bgr.empty()) {
            publishObservation(packet.header, PipelineOutput{}, false);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(cameraMutex_);
            if (cameraInfoReady_ &&
                (cameraModel_.imageWidth != bgr.cols || cameraModel_.imageHeight != bgr.rows ||
                 (!packet.header.frame_id.empty() && packet.header.frame_id != cameraModel_.frameId))) {
                throw std::invalid_argument("Image dimensions/frame do not match CameraInfo");
            }
        }

        if (!pipelineInitialized_ && !initializePipeline(bgr)) {
            publishObservation(packet.header, PipelineOutput{}, false);
            publishDebugImage(packet.header, bgr, PipelineOutput{}, false, "waiting for init");
            return;
        }

        cv::Mat mutableFrame = bgr.clone();
        const rclcpp::Time imageStamp(packet.header.stamp);
        const double timestampSeconds = imageStamp.nanoseconds() == 0
            ? this->now().seconds()
            : imageStamp.seconds();
        PipelineOutput output = pipeline_->processFrame(mutableFrame, timestampSeconds);
        if (output.rBox.area() == 0.0) {
            ++lostFrames_;
            pnpSolver_.reset();
            depth_.reset();
            publishObservation(packet.header, output, false);
            publishDebugImage(packet.header, bgr, output, false, "detector lost");
            return;
        }

        lostFrames_ = 0;
        output = publishObservation(packet.header, output, true);
        publishDebugImage(packet.header,
                          bgr,
                          output,
                          true,
                          output.predictionReady ? "observation ready" : "tracking warmup");
    }

    PipelineOutput publishObservation(const std_msgs::msg::Header& imageHeader,
                            PipelineOutput output,
                            bool found) {
        CameraModel camera;
        bool cameraReady = false;
        {
            std::lock_guard<std::mutex> lock(cameraMutex_);
            camera = cameraModel_;
            cameraReady = cameraInfoReady_;
        }
        rm_buff_tracker::msg::BuffObservation observation;
        observation.header = imageHeader;
        if (observation.header.frame_id.empty()) observation.header.frame_id = camera.frameId;
        observation.source = kSourceNone;
        observation.depth_age = -1.0;
        observation.camera_info_ready = cameraReady;
        observation.color = this->get_parameter("color").as_string();
        observation.mode = this->get_parameter("mode").as_string();
        observation.r_center_px = ToPointMsg(output.rBox.center2f());
        observation.fan_center_px = ToPointMsg(output.fanBladeBox.center2f());
        observation.radius_px = output.radius;
        observation.confidence = output.confidence;
        observation.class_id = output.classId;
        observation.phase = output.observedAngle + output.phaseCorrection;
        observation.raw_phase = output.rawAngle;
        observation.phase_delta = output.deltaAngle;
        observation.phase_velocity = output.angularVelocity;
        observation.camera_pose.orientation.w = 1.0;

        const double captureTime = rclcpp::Time(imageHeader.stamp).seconds();
        double distance = targetDistance_;
        bool depthValid = !enablePnp_;
        std::optional<PnpResult> pose;
        if (found && cameraReady && enablePnp_) {
            if (hasLastRawPhase_ &&
                std::abs(std::remainder(output.rawAngle - lastRawPhase_, 2.0 * CV_PI)) > CV_PI / 5.0) {
                pnpSolver_.reset();
                depth_.reset();
            }
            pose = pnpSolver_.solve(camera, output.keypoints, pnpObjectPoints_);
            if (pose.has_value() && pose->reprojectionError <= maxReprojectionError_) {
                const double range = cv::norm(pose->tvec);
                if (std::isfinite(range) && range > 0.0) {
                    depth_ = DepthState{range, captureTime};
                }
            } else if (pose.has_value()) {
                pose.reset();
                pnpSolver_.reset();
            }
            if (depth_.has_value()) {
                const double age = captureTime - depth_->timestamp;
                if (age >= 0.0 && age <= depthMaxAge_) {
                    distance = depth_->distance;
                    depthValid = true;
                    observation.depth_age = age;
                    observation.pnp_ready = true; // metric depth is fresh or held within its TTL
                    observation.source = pose.has_value() ? kSourcePnp : "PNP_HELD";
                } else {
                    depth_.reset();
                }
            }
        }
        if (found) {
            lastRawPhase_ = output.rawAngle;
            hasLastRawPhase_ = true;
        } else {
            hasLastRawPhase_ = false;
        }
        if (found && cameraReady && depthValid) {
            // Recompute once using measured frame age and metric depth when available.
            // delta_t is an additional horizon; it must exclude all terms below.
            const double age = this->now().seconds() - captureTime;
            if (age < 0.0 || age > maxImageAge_) {
                observationPub_->publish(observation);
                output.predictionReady = false;
                return output;
            }
            double horizon = std::max(0.0, pipelineConfig_.deltaT) + age;
            if (pipelineConfig_.enableCompensation) {
                CompensationConfig compensation = pipelineConfig_.compensationConfig;
                compensation.targetDistance = distance;
                horizon += FlightTimeCompensator(compensation).totalDelay();
            }
            pipeline_->setPredictionHorizon(output, horizon);
            const cv::Point2d observedPixel(output.fanBladeBox.center2f());
            const cv::Point2d aimPixel = output.predictionReady ? output.compensatedPoint : observedPixel;
            const RayProjection current = ProjectPixelToRay(camera, observedPixel, distance);
            const RayProjection aim = ProjectPixelToRay(camera, aimPixel, distance);
            if (current.valid && aim.valid) {
                observation.tracking = true;
                observation.prediction_ready = output.predictionReady;
                if (!enablePnp_) observation.source = kSourcePixelRay;
                observation.camera_position = ToPointMsg(current.point);
                observation.camera_aim_position = ToPointMsg(aim.point);
                observation.aim_point_px = ToPointMsg(aimPixel);
                observation.aim_ray = ToVectorMsg(aim.ray);
                observation.yaw = current.yaw;
                observation.pitch = current.pitch;
                observation.target_distance = distance;
                observation.prediction_horizon = output.predictionReady ? output.predictionHorizon : 0.0;
                observation.compensated_phase_delta = output.compensatedDelta;
                observation.camera_pose.position = observation.camera_position;
                if (pose.has_value()) {
                    observation.camera_pose.position.x = pose->tvec.at<double>(0);
                    observation.camera_pose.position.y = pose->tvec.at<double>(1);
                    observation.camera_pose.position.z = pose->tvec.at<double>(2);
                    observation.camera_pose.orientation = RvecToQuaternion(pose->rvec);
                }
            }
        }
        observationPub_->publish(observation);
        if (!observation.tracking) output.predictionReady = false;
        return output;
    }

    void publishDebugImage(const std_msgs::msg::Header& header,
                           const cv::Mat& bgr,
                           const PipelineOutput& output,
                           bool found,
                           const std::string& statusText) {
        if (!publishDebugImage_ && !showDebugWindow_) {
            return;
        }

        cv::Mat debugFrame = bgr.clone();
        if (found) {
            DrawBox(debugFrame, output.rBox, cv::Scalar(255, 0, 255), "R");
            DrawBox(debugFrame, output.fanBladeBox, cv::Scalar(0, 255, 0), "fan");
            if (output.predictionReady) {
                cv::circle(debugFrame,
                           cv::Point(static_cast<int>(std::lround(output.compensatedPoint.x)),
                                     static_cast<int>(std::lround(output.compensatedPoint.y))),
                           5, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::putText(debugFrame,
                    statusText,
                    cv::Point(20, 32),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255),
                    2);

        if (publishDebugImage_) {
            debugImagePub_->publish(MakeBgrImageMessage(header, debugFrame));
        }
        if (showDebugWindow_) {
            cv::imshow("buff_detector_debug", debugFrame);
            cv::waitKey(1);
        }
    }

    std::unique_ptr<BuffPipeline> pipeline_;
    bool pipelineInitialized_ = false;
    bool publishDebugImage_ = true;
    bool showDebugWindow_ = false;
    bool enablePnp_ = false;
    int frameCount_ = 0;
    int lostFrames_ = 0;
    double targetDistance_ = 7.0;
    struct DepthState { double distance; double timestamp; };
    std::optional<DepthState> depth_;
    double depthMaxAge_ = 0.15;
    double maxReprojectionError_ = 3.0;
    double maxImageAge_ = 0.25;
    bool hasLastRawPhase_ = false;
    double lastRawPhase_ = 0.0;
    Parameter parameter_;
    PipelineConfig pipelineConfig_;
    std::vector<cv::Point3f> pnpObjectPoints_;
    BuffPnpSolver pnpSolver_;
    std::optional<cv::Rect> staticRoi_;
    std::optional<cv::Rect> staticFanRoi_;

    std::mutex cameraMutex_;
    CameraModel cameraModel_;
    bool cameraInfoReady_ = false;

    LatestFrameMailbox mailbox_;
    std::thread workerThread_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub_;
    rclcpp::Publisher<rm_buff_tracker::msg::BuffObservation>::SharedPtr observationPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub_;
};

} // namespace gutcpp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gutcpp::BuffDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
