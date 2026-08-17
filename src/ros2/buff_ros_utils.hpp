#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "core/types.hpp"
#include "core/angle_processor.hpp"

namespace gutcpp {

struct FramePacket {
    std_msgs::msg::Header header;
    std::string encoding;
    uint32_t height = 0;
    uint32_t width = 0;
    uint8_t isBigEndian = 0;
    uint32_t step = 0;
    std::vector<uint8_t> data;
};

cv::Rect BBoxToRect(const BBox& bbox);
std::optional<cv::Rect> ParseRoiParameter(const std::vector<int64_t>& values);
int PreferredYoloSeedClassId(const std::string& color);
void DeclareBigPredictorParameters(rclcpp::Node& node);
BigPredictorConfig ReadBigPredictorConfig(const rclcpp::Node& node);
std::vector<int64_t> GetIntegerArrayParameterOrEmpty(const rclcpp::Node& node, const char* name);
std::string ResolveRosPath(const std::string& rawPath);
sensor_msgs::msg::Image MakeBgrImageMessage(const std_msgs::msg::Header& header, const cv::Mat& image);
void DrawBox(cv::Mat& frame, const BBox& bbox, const cv::Scalar& color, const std::string& label);
FramePacket MakeFramePacket(const sensor_msgs::msg::Image& msg);
cv::Mat MakeBgrFrame(const FramePacket& packet);
cv::Mat MakeBgrFrame(const sensor_msgs::msg::Image& msg);

} // namespace gutcpp
