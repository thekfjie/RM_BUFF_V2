#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_buff_tracker/msg/buff_target_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace {
// Wire format: 0xA6, validity byte, 11 little-endian floats, CRC16 (48 bytes).
// The first three floats are absolute INS yaw, INS pitch and prediction horizon.
#pragma pack(push, 1)
struct BuffCommand {
    uint8_t header = 0xA6;
    uint8_t valid = 0;
    float yaw = 0;
    float pitch = 0;
    float horizon = 0;
    float reserved[8]{};
    uint16_t checksum = 0;
};
struct Feedback {
    uint8_t header;
    uint8_t flags;
    float roll, pitch, yaw, aim_x, aim_y, aim_z;
    uint16_t checksum;
};
#pragma pack(pop)
static_assert(sizeof(BuffCommand) == 48 && sizeof(Feedback) == 28, "Serial layout mismatch");
constexpr double kPi = 3.14159265358979323846;

uint16_t crc16(const uint8_t* bytes, size_t count) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < count; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    }
    return crc;
}
bool finite(const geometry_msgs::msg::Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
} // namespace

class BuffSerialBridge final : public rclcpp::Node {
public:
    BuffSerialBridge()
        : Node("buff_serial_bridge"), context_(std::make_unique<IoContext>(2)),
          serial_(std::make_unique<drivers::serial_driver::SerialDriver>(*context_)),
          tf_(std::make_shared<tf2_ros::Buffer>(get_clock())), listener_(*tf_), broadcaster_(*this) {
        const auto port = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
        const auto baud = declare_parameter<int>("baud_rate", 115200);
        enabled_ = declare_parameter<bool>("enable_output", false);
        require_pnp_ = declare_parameter<bool>("require_pnp", true);
        max_age_ = declare_parameter<double>("max_target_age", 0.12);
        max_feedback_age_ = declare_parameter<double>("max_feedback_age", 0.12);
        max_horizon_ = declare_parameter<double>("max_prediction_horizon", 0.8);
        const auto topic = declare_parameter<std::string>("target_topic", "/buff/tracker/target");
        if (baud <= 0 || max_age_ <= 0 || max_feedback_age_ <= 0 || max_horizon_ <= 0)
            throw std::invalid_argument("Invalid BUFF bridge parameters");
        serial_->init_port(port, drivers::serial_driver::SerialPortConfig(
            static_cast<uint32_t>(baud), drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE));
        if (!serial_->port()->is_open()) serial_->port()->open();
        receive_thread_ = std::thread([this] { receive(); });
        sub_ = create_subscription<rm_buff_tracker::msg::BuffTargetState>(
            topic, rclcpp::SensorDataQoS().keep_last(1),
            [this](rm_buff_tracker::msg::BuffTargetState::SharedPtr msg) { send(*msg); });
        RCLCPP_WARN(get_logger(), "BUFF bridge output %s; exclusive serial port required",
                    enabled_ ? "enabled" : "disabled");
    }

    ~BuffSerialBridge() override {
        running_ = false;
        if (serial_->port()->is_open()) serial_->port()->close();
        if (receive_thread_.joinable()) receive_thread_.join();
        context_->waitForExit();
    }

private:
    void receive() {
        std::vector<uint8_t> head(1), rest(sizeof(Feedback) - 1);
        while (running_ && rclcpp::ok()) {
            try {
                serial_->port()->receive(head);
                if (head[0] != 0x5A) continue;
                serial_->port()->receive(rest);
                Feedback f{};
                auto* bytes = reinterpret_cast<uint8_t*>(&f);
                bytes[0] = head[0];
                std::memcpy(bytes + 1, rest.data(), rest.size());
                if (crc16(bytes, sizeof(f) - 2) != f.checksum ||
                    !std::isfinite(f.roll) || !std::isfinite(f.pitch) || !std::isfinite(f.yaw) ||
                    std::abs(f.roll) > kPi || std::abs(f.pitch) > kPi / 2 ||
                    std::abs(f.yaw) > 100 * kPi) continue;
                const auto stamp = now();
                geometry_msgs::msg::TransformStamped t;
                t.header.frame_id = "odom";
                t.child_frame_id = "gimbal_link";
                t.header.stamp = stamp;
                tf2::Quaternion q;
                q.setRPY(f.roll, f.pitch, f.yaw);
                t.transform.rotation = tf2::toMsg(q);
                broadcaster_.sendTransform(t);
                last_feedback_ns_.store(stamp.nanoseconds());
            } catch (const std::exception& e) {
                if (running_ && rclcpp::ok())
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "BUFF feedback: %s", e.what());
            }
        }
    }

    void send(const rm_buff_tracker::msg::BuffTargetState& msg) {
        if (!enabled_) return;
        BuffCommand command;
        const auto stamp = rclcpp::Time(msg.header.stamp);
        const double age = (now() - stamp).seconds();
        const double feedback_age = (now().nanoseconds() - last_feedback_ns_.load()) * 1e-9;
        if (msg.tracking && msg.prediction_ready && msg.camera_info_ready && msg.tf_ready &&
            msg.tracker_state == "TRACKING" && (!require_pnp_ || msg.pnp_ready) &&
            msg.header.frame_id == "odom" && finite(msg.predicted_position) &&
            stamp.nanoseconds() > 0 && age >= 0 && age <= max_age_ &&
            feedback_age >= 0 && feedback_age <= max_feedback_age_ &&
            std::isfinite(msg.prediction_horizon) && msg.prediction_horizon >= age &&
            msg.prediction_horizon <= max_horizon_) {
            try {
                // The feedback TF supplies the current gimbal origin and orientation.
                // The target is already predicted; no velocity extrapolation is applied here.
                const auto gimbal = tf_->lookupTransform("odom", "gimbal_link", tf2::TimePointZero);
                const double tf_age = (now() - rclcpp::Time(gimbal.header.stamp)).seconds();
                const double x = msg.predicted_position.x - gimbal.transform.translation.x;
                const double y = msg.predicted_position.y - gimbal.transform.translation.y;
                const double z = msg.predicted_position.z - gimbal.transform.translation.z;
                tf2::Quaternion q;
                tf2::fromMsg(gimbal.transform.rotation, q);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                const double desired_yaw = std::atan2(y, x);
                const double desired_pitch = std::atan2(z, std::hypot(x, y));
                if (tf_age >= 0 && tf_age <= max_feedback_age_ &&
                    std::abs(roll) < 0.12 && std::hypot(x, y) > 0.1 &&
                    desired_yaw > -kPi && desired_yaw < kPi &&
                    std::abs(std::remainder(desired_yaw - yaw, 2 * kPi)) < kPi / 8 &&
                    std::abs(desired_pitch - pitch) < kPi / 4) {
                    command.valid = 1;
                    command.yaw = static_cast<float>(desired_yaw);
                    // Firmware reports pitch = -INS pitch in the existing 0x5A feedback.
                    command.pitch = static_cast<float>(-desired_pitch);
                    command.horizon = static_cast<float>(msg.prediction_horizon);
                }
            } catch (const tf2::TransformException&) {
                // Send an invalid command so the controller drops aim authorization.
            }
        }
        auto* bytes = reinterpret_cast<uint8_t*>(&command);
        command.checksum = crc16(bytes, sizeof(command) - 2);
        try {
            serial_->port()->send(std::vector<uint8_t>(bytes, bytes + sizeof(command)));
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "BUFF command: %s", e.what());
        }
    }

    std::unique_ptr<IoContext> context_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;
    std::atomic<bool> running_{true};
    std::atomic<int64_t> last_feedback_ns_{0};
    std::thread receive_thread_;
    rclcpp::Subscription<rm_buff_tracker::msg::BuffTargetState>::SharedPtr sub_;
    bool enabled_ = false;
    bool require_pnp_ = true;
    double max_age_ = 0.12, max_feedback_age_ = 0.12, max_horizon_ = 0.8;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuffSerialBridge>());
    rclcpp::shutdown();
    return 0;
}
