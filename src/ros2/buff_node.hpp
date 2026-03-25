#pragma once

#ifdef AMENT_CMAKE_FOUND

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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
    PipelineConfig buildPipelineConfig() const;
    Parameter buildParameter() const;

    std::unique_ptr<BuffPipeline> pipeline_;
    bool pipelineInitialized_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr predictionPub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debugPub_;
};

} // namespace gutcpp

#endif // AMENT_CMAKE_FOUND
