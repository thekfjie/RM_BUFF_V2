#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rm_buff_tracker/msg/buff_observation.hpp>
#include <rm_buff_tracker/msg/buff_target_state.hpp>

#include "core/camera_geometry.hpp"

namespace gutcpp {
namespace {
using Point = geometry_msgs::msg::Point;
using Vector = geometry_msgs::msg::Vector3;
using Observation = rm_buff_tracker::msg::BuffObservation;
using Target = rm_buff_tracker::msg::BuffTargetState;
constexpr double kPi = 3.14159265358979323846;

bool FinitePoint(const Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
Vector Difference(const Point& current, const Point& previous, double dt) {
    Vector v;
    v.x = (current.x - previous.x) / dt;
    v.y = (current.y - previous.y) / dt;
    v.z = (current.z - previous.z) / dt;
    return v;
}
Vector Smooth(const Vector& v, const Vector& previous, double alpha) {
    Vector result;
    result.x = alpha * v.x + (1.0 - alpha) * previous.x;
    result.y = alpha * v.y + (1.0 - alpha) * previous.y;
    result.z = alpha * v.z + (1.0 - alpha) * previous.z;
    return result;
}
Point Advance(Point p, const Vector& v, double dt) {
    p.x += v.x * dt;
    p.y += v.y * dt;
    p.z += v.z * dt;
    return p;
}
void SetAngles(Target& target) {
    auto angles = [&target](const Point& p, double& yaw, double& pitch) {
        const auto projection = target.tf_ready ? ProjectForwardLeftUpPointToAngles({p.x, p.y, p.z}) :
                                                 ProjectCameraPointToAngles({p.x, p.y, p.z});
        yaw = projection.yaw;
        pitch = projection.pitch;
    };
    angles(target.position, target.yaw, target.pitch);
    angles(target.predicted_position, target.predicted_yaw, target.predicted_pitch);
}
} // namespace

class BuffTrackerNode final : public rclcpp::Node {
public:
    BuffTrackerNode()
        : Node("buff_spatial_tracker_node"),
          tfBuffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tfListener_(*tfBuffer_) {
        const auto observationTopic = declare_parameter<std::string>("observation_topic", "/buff/detector/observation");
        const auto targetTopic = declare_parameter<std::string>("target_topic", "/buff/tracker/target");
        targetFrame_ = declare_parameter<std::string>("target_frame", "odom");
        enableTf_ = declare_parameter<bool>("enable_tf", true);
        lostTime_ = declare_parameter<double>("lost_time_thres", 0.3);
        timeout_ = declare_parameter<double>("observation_timeout", 0.15);
        maxAge_ = declare_parameter<double>("max_observation_age", 0.25);
        alpha_ = declare_parameter<double>("velocity_smoothing_alpha", 0.45);
        const double lead = declare_parameter<double>("prediction_lead_time", 0.0);
        if (!std::isfinite(lead) || lead != 0.0)
            throw std::invalid_argument("prediction_lead_time must be zero: detector already predicts the complete horizon");
        if (!std::isfinite(lostTime_) || lostTime_ < 0.0 || !std::isfinite(timeout_) || timeout_ <= 0.0 ||
            !std::isfinite(maxAge_) || maxAge_ <= 0.0 || !std::isfinite(alpha_) || alpha_ < 0.0 || alpha_ > 1.0)
            throw std::invalid_argument("Invalid BUFF tracker time/smoothing parameters");
        observationSub_ = create_subscription<Observation>(observationTopic, rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&BuffTrackerNode::onObservation, this, std::placeholders::_1));
        targetPub_ = create_publisher<Target>(targetTopic, rclcpp::SensorDataQoS());
        watchdog_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
            if (!last_.has_value()) return;
            const double silence = std::chrono::duration<double>(std::chrono::steady_clock::now() - lastReceipt_).count();
            if (silence > timeout_) {
                Observation missing;
                missing.header.stamp = now();
                missing.header.frame_id = cameraFrame_;
                missing.color = last_->color;
                missing.mode = last_->mode;
                publishMissing(missing);
            }
        });
    }

private:
    Target base(const Observation& msg) const {
        Target target;
        target.header = msg.header;
        target.tracker_state = "LOST";
        target.camera_info_ready = msg.camera_info_ready;
        target.pnp_ready = msg.pnp_ready;
        target.source = msg.source;
        target.color = msg.color;
        target.mode = msg.mode;
        target.r_center_px = msg.r_center_px;
        target.fan_center_px = msg.fan_center_px;
        target.aim_point_px = msg.aim_point_px;
        target.radius_px = msg.radius_px;
        target.confidence = msg.confidence;
        target.class_id = msg.class_id;
        target.target_distance = msg.target_distance;
        target.depth_age = msg.depth_age;
        target.camera_position = msg.camera_position;
        target.position = msg.camera_position;
        target.predicted_position = msg.camera_aim_position;
        target.phase = msg.phase;
        target.phase_velocity = msg.phase_velocity;
        target.predicted_phase = msg.phase + msg.compensated_phase_delta;
        target.prediction_horizon = msg.prediction_horizon;
        target.prediction_ready = msg.prediction_ready;
        return target;
    }

    void transform(const Observation& msg, Target& target) {
        if (!enableTf_ || targetFrame_.empty() || msg.header.frame_id.empty()) return;
        try {
            // One transform at capture time for BOTH points. Never mix frames
            // if a second lookup would fail while the first one succeeds.
            const auto tf = tfBuffer_->lookupTransform(targetFrame_, msg.header.frame_id,
                                                        rclcpp::Time(msg.header.stamp));
            geometry_msgs::msg::PointStamped current, aim, outputCurrent, outputAim;
            current.header = aim.header = msg.header;
            current.point = msg.camera_position;
            aim.point = msg.camera_aim_position;
            tf2::doTransform(current, outputCurrent, tf);
            tf2::doTransform(aim, outputAim, tf);
            target.position = outputCurrent.point;
            target.predicted_position = outputAim.point;
            target.header.frame_id = targetFrame_;
            target.tf_ready = true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "BUFF TF unavailable: %s", ex.what());
        }
    }

    void onObservation(const Observation::SharedPtr msg) {
        lastReceipt_ = std::chrono::steady_clock::now();
        const rclcpp::Time stamp(msg->header.stamp);
        const double age = (now() - stamp).seconds();
        if (last_ && (last_->color != msg->color || last_->mode != msg->mode ||
                      cameraFrame_ != msg->header.frame_id)) {
            last_.reset();
            hasVelocity_ = false;
        }
        if (stamp.nanoseconds() == 0 || age < 0.0 || age > maxAge_) {
            // Stale/backwards input must not revive a previous target.
            last_.reset();
            hasVelocity_ = false;
            Observation missing = *msg;
            missing.header.stamp = now();
            publishMissing(missing);
            return;
        }
        if (!msg->tracking || !msg->camera_info_ready || msg->source == "NONE" ||
            !FinitePoint(msg->camera_position) || !FinitePoint(msg->camera_aim_position) ||
            msg->camera_position.z <= 0.0 || msg->camera_aim_position.z <= 0.0 ||
            !std::isfinite(msg->phase) || !std::isfinite(msg->raw_phase) || !std::isfinite(msg->phase_velocity) ||
            !std::isfinite(msg->compensated_phase_delta) ||
            !std::isfinite(msg->prediction_horizon) || msg->prediction_horizon < 0.0) {
            publishMissing(*msg);
            return;
        }
        Target target = base(*msg);
        transform(*msg, target);
        SetAngles(target);
        double dt = 0.0;
        bool continuous = false;
        if (last_) {
            dt = (stamp - rclcpp::Time(last_->header.stamp)).seconds();
            continuous = dt > 1e-6 && dt <= lostTime_ && last_->header.frame_id == target.header.frame_id &&
                last_->tf_ready == target.tf_ready && last_->pnp_ready == target.pnp_ready &&
                std::abs(std::remainder(msg->raw_phase - lastRawPhase_, 2.0 * kPi)) < kPi / 5.0;
            if (dt <= 0.0) {
                last_.reset();
                hasVelocity_ = false;
                publishMissing(*msg);
                return;
            }
        }
        target.tracking = true;
        target.tracker_state = continuous ? "TRACKING" : "DETECTING";
        if (continuous) {
            target.velocity = Difference(target.position, last_->position, dt);
            target.camera_velocity = Difference(target.camera_position, last_->camera_position, dt);
            target.v_yaw = std::remainder(target.yaw - last_->yaw, 2.0 * kPi) / dt;
            target.v_pitch = (target.pitch - last_->pitch) / dt;
            if (hasVelocity_) {
                target.velocity = Smooth(target.velocity, last_->velocity, alpha_);
                target.camera_velocity = Smooth(target.camera_velocity, last_->camera_velocity, alpha_);
                target.v_yaw = alpha_ * target.v_yaw + (1.0 - alpha_) * last_->v_yaw;
                target.v_pitch = alpha_ * target.v_pitch + (1.0 - alpha_) * last_->v_pitch;
            }
        }
        // Preserve the detector's sine-integrated prediction; do not extrapolate
        // this future point again using Cartesian or angular linear velocity.
        last_ = target;
        hasVelocity_ = continuous;
        cameraFrame_ = msg->header.frame_id;
        lastRawPhase_ = msg->raw_phase;
        targetPub_->publish(target);
    }

    void publishMissing(const Observation& msg) {
        Target target;
        target.header = msg.header;
        target.color = msg.color;
        target.mode = msg.mode;
        target.depth_age = -1.0;
        target.tracker_state = "LOST";
        target.source = "NONE";
        if (last_) {
            const double dt = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(last_->header.stamp)).seconds();
            if (dt >= 0.0 && dt <= lostTime_) {
                target = *last_;
                target.header.stamp = msg.header.stamp;
                target.tracker_state = "TEMP_LOST";
                target.position = Advance(last_->position, last_->velocity, dt);
                target.camera_position = Advance(last_->camera_position, last_->camera_velocity, dt);
                target.predicted_position = target.position;
                target.phase = last_->phase + last_->phase_velocity * dt;
                target.predicted_phase = target.phase;
                target.prediction_ready = false;
                target.prediction_horizon = 0.0;
                target.pnp_ready = false;
                if (target.depth_age >= 0.0) target.depth_age += dt;
                SetAngles(target);
                targetPub_->publish(target);
                return;
            }
        }
        last_.reset();
        hasVelocity_ = false;
        targetPub_->publish(target);
    }

    std::string targetFrame_;
    std::string cameraFrame_;
    bool enableTf_ = true;
    bool hasVelocity_ = false;
    double lostTime_ = 0.3;
    double timeout_ = 0.15;
    double maxAge_ = 0.25;
    double alpha_ = 0.45;
    double lastRawPhase_ = 0.0;
    std::optional<Target> last_;
    std::chrono::steady_clock::time_point lastReceipt_ = std::chrono::steady_clock::now();
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    rclcpp::Subscription<Observation>::SharedPtr observationSub_;
    rclcpp::Publisher<Target>::SharedPtr targetPub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
};
} // namespace gutcpp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gutcpp::BuffTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
