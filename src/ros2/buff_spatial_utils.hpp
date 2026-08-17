#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/core.hpp>

namespace gutcpp {

geometry_msgs::msg::Point ToPointMsg(const cv::Point2d& point);
geometry_msgs::msg::Point ToPointMsg(const cv::Point2f& point);
geometry_msgs::msg::Point ToPointMsg(const cv::Point3d& point);
geometry_msgs::msg::Vector3 ToVectorMsg(const cv::Point3d& point);
geometry_msgs::msg::Quaternion RvecToQuaternion(const cv::Mat& rvec);
std::vector<cv::Point3f> ParseObjectPoints(const std::vector<double>& values);
bool IsSupportedDistortionSize(std::size_t size);

} // namespace gutcpp
