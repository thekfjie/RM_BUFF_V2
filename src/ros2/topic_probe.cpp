#include <chrono>
#include <exception>
#include <iostream>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace {

using namespace std::chrono_literals;

class TopicProbe : public rclcpp::Node {
public:
    TopicProbe(const std::string& nodeTopicPrefix, const std::string& imageTopic)
        : Node("topic_probe"),
          nodeTopicPrefix_(nodeTopicPrefix),
          imageTopic_(imageTopic) {
        imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            imageTopic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr) { ++imageCount_; });

        debugImageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            nodeTopicPrefix_ + "/debug_image", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr) { ++debugImageCount_; });

        debugStateSub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            nodeTopicPrefix_ + "/debug_state", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr) { ++debugStateCount_; });

        predictionSub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            nodeTopicPrefix_ + "/prediction", 10,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr) { ++predictionCount_; });

        timer_ = this->create_wall_timer(1s, [this]() { printSummary(); });

        RCLCPP_INFO(this->get_logger(),
                    "Watching topics: image=%s, debug_image=%s, debug_state=%s, prediction=%s",
                    imageTopic_.c_str(),
                    (nodeTopicPrefix_ + "/debug_image").c_str(),
                    (nodeTopicPrefix_ + "/debug_state").c_str(),
                    (nodeTopicPrefix_ + "/prediction").c_str());
    }

private:
    void printSummary() {
        const auto imageNow = imageCount_;
        const auto debugImageNow = debugImageCount_;
        const auto debugStateNow = debugStateCount_;
        const auto predictionNow = predictionCount_;

        RCLCPP_INFO(
            this->get_logger(),
            "rate(1s): image=%zu debug_image=%zu debug_state=%zu prediction=%zu | total: image=%zu debug_image=%zu debug_state=%zu prediction=%zu",
            imageNow - lastImageCount_,
            debugImageNow - lastDebugImageCount_,
            debugStateNow - lastDebugStateCount_,
            predictionNow - lastPredictionCount_,
            imageNow,
            debugImageNow,
            debugStateNow,
            predictionNow);

        lastImageCount_ = imageNow;
        lastDebugImageCount_ = debugImageNow;
        lastDebugStateCount_ = debugStateNow;
        lastPredictionCount_ = predictionNow;
    }

    std::string nodeTopicPrefix_;
    std::string imageTopic_;

    std::size_t imageCount_ = 0;
    std::size_t debugImageCount_ = 0;
    std::size_t debugStateCount_ = 0;
    std::size_t predictionCount_ = 0;

    std::size_t lastImageCount_ = 0;
    std::size_t lastDebugImageCount_ = 0;
    std::size_t lastDebugStateCount_ = 0;
    std::size_t lastPredictionCount_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debugImageSub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr debugStateSub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr predictionSub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    bool rclInitialized = false;
    try {
        rclcpp::init(argc, argv);
        rclInitialized = true;

        std::string nodeTopicPrefix = "/buff_tracker_node";
        std::string imageTopic = "/image_raw";
        if (argc >= 2 && argv[1] != nullptr && std::string(argv[1]).size() > 0) {
            nodeTopicPrefix = argv[1];
        }
        if (argc >= 3 && argv[2] != nullptr && std::string(argv[2]).size() > 0) {
            imageTopic = argv[2];
        }

        auto node = std::make_shared<TopicProbe>(nodeTopicPrefix, imageTopic);
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "topic_probe fatal error: " << ex.what() << std::endl;
    } catch (...) {
        std::cerr << "topic_probe fatal error: unknown exception" << std::endl;
    }

    if (rclInitialized) {
        rclcpp::shutdown();
    }
    return 1;
}
