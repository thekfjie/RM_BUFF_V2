#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rm_buff_tracker/msg/buff_observation.hpp>
#include <rm_buff_tracker/msg/buff_target_state.hpp>

namespace gutcpp {

namespace {

constexpr const char* kStateLost = "LOST";
constexpr const char* kStateDetecting = "DETECTING";
constexpr const char* kStateTracking = "TRACKING";
constexpr const char* kStateTempLost = "TEMP_LOST";
constexpr double kPi = 3.14159265358979323846;

double NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double AngularDifference(double current, double previous) {
    return NormalizeAngle(current - previous);
}

geometry_msgs::msg::Vector3 ZeroVector() {
    geometry_msgs::msg::Vector3 vector;
    vector.x = 0.0;
    vector.y = 0.0;
    vector.z = 0.0;
    return vector;
}

geometry_msgs::msg::Vector3 EstimateVelocity(const geometry_msgs::msg::Point& current,
                                             const geometry_msgs::msg::Point& previous,
                                             double dt) {
    geometry_msgs::msg::Vector3 velocity;
    if (dt <= 1e-6) {
        return velocity;
    }
    velocity.x = (current.x - previous.x) / dt;
    velocity.y = (current.y - previous.y) / dt;
    velocity.z = (current.z - previous.z) / dt;
    return velocity;
}

geometry_msgs::msg::Vector3 SmoothVector(const geometry_msgs::msg::Vector3& current,
                                         const geometry_msgs::msg::Vector3& previous,
                                         double alpha) {
    geometry_msgs::msg::Vector3 smoothed;
    smoothed.x = alpha * current.x + (1.0 - alpha) * previous.x;
    smoothed.y = alpha * current.y + (1.0 - alpha) * previous.y;
    smoothed.z = alpha * current.z + (1.0 - alpha) * previous.z;
    return smoothed;
}

geometry_msgs::msg::Point AdvancePoint(const geometry_msgs::msg::Point& point,
                                       const geometry_msgs::msg::Vector3& velocity,
                                       double dt) {
    geometry_msgs::msg::Point advanced = point;
    advanced.x += velocity.x * dt;
    advanced.y += velocity.y * dt;
    advanced.z += velocity.z * dt;
    return advanced;
}

double SafeFinite(double value, double fallback = 0.0) {
    return std::isfinite(value) ? value : fallback;
}

} // namespace

class BuffTrackerNode final : public rclcpp::Node {
public:
    BuffTrackerNode()
        : Node("buff_spatial_tracker_node"),
          tfBuffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tfListener_(*tfBuffer_) {
        declareParameters();
        loadRuntimeConfig();

        observationSub_ = this->create_subscription<rm_buff_tracker::msg::BuffObservation>(
            observationTopic_, rclcpp::SensorDataQoS(),
            std::bind(&BuffTrackerNode::observationCallback, this, std::placeholders::_1));
        targetPub_ = this->create_publisher<rm_buff_tracker::msg::BuffTargetState>(
            targetTopic_, rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(),
                    "BUFF tracker channel started: observation=%s target=%s target_frame=%s",
                    observationTopic_.c_str(), targetTopic_.c_str(), targetFrame_.c_str());
    }

private:
    struct LastState {
        bool valid = false;
        bool hasVelocity = false;
        bool tfReady = false;
        rclcpp::Time stamp;
        std::string frameId;
        std::string source;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Vector3 velocity;
        geometry_msgs::msg::Point cameraPosition;
        geometry_msgs::msg::Vector3 cameraVelocity;
        double yaw = 0.0;
        double pitch = 0.0;
        double vYaw = 0.0;
        double vPitch = 0.0;
        double phase = 0.0;
        double phaseVelocity = 0.0;
    };

    void declareParameters() {
        this->declare_parameter<std::string>("observation_topic", "/buff/detector/observation");
        this->declare_parameter<std::string>("target_topic", "/buff/tracker/target");
        this->declare_parameter<std::string>("target_frame", "odom");
        this->declare_parameter<bool>("enable_tf", true);
        this->declare_parameter<double>("lost_time_thres", 0.3);
        this->declare_parameter<double>("prediction_lead_time", 0.0);
        this->declare_parameter<double>("velocity_smoothing_alpha", 0.45);
    }

    void loadRuntimeConfig() {
        observationTopic_ = this->get_parameter("observation_topic").as_string();
        targetTopic_ = this->get_parameter("target_topic").as_string();
        targetFrame_ = this->get_parameter("target_frame").as_string();
        enableTf_ = this->get_parameter("enable_tf").as_bool();
        lostTimeThres_ = std::max(0.0, this->get_parameter("lost_time_thres").as_double());
        predictionLeadTime_ = std::max(0.0, this->get_parameter("prediction_lead_time").as_double());
        velocitySmoothingAlpha_ = std::clamp(this->get_parameter("velocity_smoothing_alpha").as_double(), 0.0, 1.0);
    }

    rclcpp::Time stampOrNow(const builtin_interfaces::msg::Time& stamp) const {
        const rclcpp::Time time(stamp);
        return time.nanoseconds() == 0 ? this->now() : time;
    }

    geometry_msgs::msg::Point transformPosition(const rm_buff_tracker::msg::BuffObservation& observation,
                                                std_msgs::msg::Header& outputHeader,
                                                bool& tfReady) {
        outputHeader = observation.header;
        tfReady = false;

        if (!enableTf_ || targetFrame_.empty() || observation.header.frame_id.empty()) {
            return observation.camera_position;
        }

        if (observation.header.frame_id == targetFrame_) {
            outputHeader.frame_id = targetFrame_;
            tfReady = true;
            return observation.camera_position;
        }

        geometry_msgs::msg::PointStamped input;
        input.header = observation.header;
        input.point = observation.camera_position;

        try {
            const geometry_msgs::msg::PointStamped transformed = tfBuffer_->transform(input, targetFrame_);
            outputHeader = transformed.header;
            tfReady = true;
            return transformed.point;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "BUFF target tf unavailable (%s -> %s): %s",
                                 observation.header.frame_id.c_str(),
                                 targetFrame_.c_str(),
                                 ex.what());
            return observation.camera_position;
        }
    }

    rm_buff_tracker::msg::BuffTargetState makeBaseMessage(
        const rm_buff_tracker::msg::BuffObservation& observation) const {
        rm_buff_tracker::msg::BuffTargetState target;
        target.header = observation.header;
        target.prediction_ready = observation.prediction_ready;
        target.camera_info_ready = observation.camera_info_ready;
        target.tf_ready = false;
        target.pnp_ready = observation.pnp_ready;
        target.tracker_state = kStateLost;
        target.source = observation.source;
        target.color = observation.color;
        target.mode = observation.mode;
        target.r_center_px = observation.r_center_px;
        target.fan_center_px = observation.fan_center_px;
        target.aim_point_px = observation.aim_point_px;
        target.radius_px = observation.radius_px;
        target.target_distance = observation.target_distance;
        target.confidence = observation.confidence;
        target.class_id = observation.class_id;
        target.camera_position = observation.camera_position;
        target.camera_velocity = ZeroVector();
        target.position = observation.camera_position;
        target.velocity = ZeroVector();
        target.predicted_position = target.position;
        target.yaw = observation.yaw;
        target.pitch = observation.pitch;
        target.v_yaw = 0.0;
        target.v_pitch = 0.0;
        target.predicted_yaw = target.yaw;
        target.predicted_pitch = target.pitch;
        target.phase = observation.phase;
        target.phase_velocity = observation.phase_velocity;
        target.predicted_phase = target.phase;
        return target;
    }

    void observationCallback(const rm_buff_tracker::msg::BuffObservation::SharedPtr msg) {
        if (!msg->tracking || !msg->camera_info_ready) {
            publishLostOrTempState(*msg);
            return;
        }

        rm_buff_tracker::msg::BuffTargetState target = makeBaseMessage(*msg);
        bool tfReady = false;
        std_msgs::msg::Header transformedHeader;
        target.position = transformPosition(*msg, transformedHeader, tfReady);
        target.header = transformedHeader;
        target.header.stamp = msg->header.stamp;
        if (!tfReady && target.header.frame_id.empty()) {
            target.header.frame_id = msg->header.frame_id;
        }
        target.tf_ready = tfReady;
        target.tracking = true;
        target.tracker_state = lastState_.valid ? kStateTracking : kStateDetecting;

        const rclcpp::Time stamp = stampOrNow(msg->header.stamp);
        const bool canEstimateVelocity =
            lastState_.valid && lastState_.frameId == target.header.frame_id &&
            (stamp - lastState_.stamp).seconds() > 1e-6;

        if (canEstimateVelocity) {
            const double dt = (stamp - lastState_.stamp).seconds();
            target.velocity = EstimateVelocity(target.position, lastState_.position, dt);
            target.camera_velocity = EstimateVelocity(target.camera_position, lastState_.cameraPosition, dt);
            target.v_yaw = AngularDifference(target.yaw, lastState_.yaw) / dt;
            target.v_pitch = (target.pitch - lastState_.pitch) / dt;
            if (std::isfinite(msg->phase_velocity) && std::fabs(msg->phase_velocity) > 1e-9) {
                target.phase_velocity = msg->phase_velocity;
            } else {
                target.phase_velocity = (target.phase - lastState_.phase) / dt;
            }

            if (lastState_.hasVelocity) {
                target.velocity = SmoothVector(target.velocity, lastState_.velocity, velocitySmoothingAlpha_);
                target.camera_velocity = SmoothVector(target.camera_velocity, lastState_.cameraVelocity, velocitySmoothingAlpha_);
                target.v_yaw = velocitySmoothingAlpha_ * target.v_yaw +
                               (1.0 - velocitySmoothingAlpha_) * lastState_.vYaw;
                target.v_pitch = velocitySmoothingAlpha_ * target.v_pitch +
                                 (1.0 - velocitySmoothingAlpha_) * lastState_.vPitch;
                target.phase_velocity = velocitySmoothingAlpha_ * target.phase_velocity +
                                        (1.0 - velocitySmoothingAlpha_) * lastState_.phaseVelocity;
            }
        }

        target.predicted_position = AdvancePoint(target.position, target.velocity, predictionLeadTime_);
        target.predicted_yaw = target.yaw + target.v_yaw * predictionLeadTime_;
        target.predicted_pitch = target.pitch + target.v_pitch * predictionLeadTime_;
        target.predicted_phase = target.phase + target.phase_velocity * predictionLeadTime_;

        updateLastState(target, stamp);
        targetPub_->publish(target);
    }

    void publishLostOrTempState(const rm_buff_tracker::msg::BuffObservation& observation) {
        rm_buff_tracker::msg::BuffTargetState target = makeBaseMessage(observation);
        const rclcpp::Time stamp = stampOrNow(observation.header.stamp);

        if (lastState_.valid) {
            const double dt = (stamp - lastState_.stamp).seconds();
            if (dt >= 0.0 && dt <= lostTimeThres_) {
                target.header.stamp = observation.header.stamp;
                target.header.frame_id = lastState_.frameId;
                target.tracking = true;
                target.tf_ready = lastState_.tfReady;
                target.tracker_state = kStateTempLost;
                target.source = lastState_.source;
                target.position = AdvancePoint(lastState_.position, lastState_.velocity, dt);
                target.velocity = lastState_.velocity;
                target.predicted_position = AdvancePoint(target.position, target.velocity, predictionLeadTime_);
                target.camera_position = AdvancePoint(lastState_.cameraPosition, lastState_.cameraVelocity, dt);
                target.camera_velocity = lastState_.cameraVelocity;
                target.yaw = lastState_.yaw + lastState_.vYaw * dt;
                target.pitch = lastState_.pitch + lastState_.vPitch * dt;
                target.v_yaw = lastState_.vYaw;
                target.v_pitch = lastState_.vPitch;
                target.predicted_yaw = target.yaw + target.v_yaw * predictionLeadTime_;
                target.predicted_pitch = target.pitch + target.v_pitch * predictionLeadTime_;
                target.phase = lastState_.phase + lastState_.phaseVelocity * dt;
                target.phase_velocity = lastState_.phaseVelocity;
                target.predicted_phase = target.phase + target.phase_velocity * predictionLeadTime_;
                targetPub_->publish(target);
                return;
            }
        }

        target.tracking = false;
        target.tracker_state = kStateLost;
        target.header.stamp = observation.header.stamp;
        targetPub_->publish(target);
    }

    void updateLastState(const rm_buff_tracker::msg::BuffTargetState& target, const rclcpp::Time& stamp) {
        lastState_.valid = true;
        lastState_.hasVelocity = target.tracker_state == kStateTracking;
        lastState_.tfReady = target.tf_ready;
        lastState_.stamp = stamp;
        lastState_.frameId = target.header.frame_id;
        lastState_.source = target.source;
        lastState_.position = target.position;
        lastState_.velocity = target.velocity;
        lastState_.cameraPosition = target.camera_position;
        lastState_.cameraVelocity = target.camera_velocity;
        lastState_.yaw = SafeFinite(target.yaw);
        lastState_.pitch = SafeFinite(target.pitch);
        lastState_.vYaw = SafeFinite(target.v_yaw);
        lastState_.vPitch = SafeFinite(target.v_pitch);
        lastState_.phase = SafeFinite(target.phase);
        lastState_.phaseVelocity = SafeFinite(target.phase_velocity);
    }

    std::string observationTopic_;
    std::string targetTopic_;
    std::string targetFrame_;
    bool enableTf_ = true;
    double lostTimeThres_ = 0.3;
    double predictionLeadTime_ = 0.0;
    double velocitySmoothingAlpha_ = 0.45;
    LastState lastState_;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    rclcpp::Subscription<rm_buff_tracker::msg::BuffObservation>::SharedPtr observationSub_;
    rclcpp::Publisher<rm_buff_tracker::msg::BuffTargetState>::SharedPtr targetPub_;
};

} // namespace gutcpp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gutcpp::BuffTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
