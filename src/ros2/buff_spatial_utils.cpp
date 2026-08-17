#include "buff_spatial_utils.hpp"

#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gutcpp {

geometry_msgs::msg::Point ToPointMsg(const cv::Point2d& point) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x;
    msg.y = point.y;
    msg.z = 0.0;
    return msg;
}

geometry_msgs::msg::Point ToPointMsg(const cv::Point2f& point) {
    return ToPointMsg(cv::Point2d(point.x, point.y));
}

geometry_msgs::msg::Point ToPointMsg(const cv::Point3d& point) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x;
    msg.y = point.y;
    msg.z = point.z;
    return msg;
}

geometry_msgs::msg::Vector3 ToVectorMsg(const cv::Point3d& point) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = point.x;
    msg.y = point.y;
    msg.z = point.z;
    return msg;
}

geometry_msgs::msg::Quaternion RvecToQuaternion(const cv::Mat& rvec) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.w = 1.0;
    if (rvec.empty()) {
        return quaternion;
    }

    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);
    if (rotationMatrix.rows != 3 || rotationMatrix.cols != 3) {
        return quaternion;
    }

    tf2::Matrix3x3 tfRotation(
        rotationMatrix.at<double>(0, 0), rotationMatrix.at<double>(0, 1), rotationMatrix.at<double>(0, 2),
        rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(1, 1), rotationMatrix.at<double>(1, 2),
        rotationMatrix.at<double>(2, 0), rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));
    tf2::Quaternion tfQuaternion;
    tfRotation.getRotation(tfQuaternion);
    return tf2::toMsg(tfQuaternion);
}

std::vector<cv::Point3f> ParseObjectPoints(const std::vector<double>& values) {
    std::vector<cv::Point3f> points;
    if (values.size() < 12 || values.size() % 3 != 0) {
        return points;
    }

    points.reserve(values.size() / 3);
    for (std::size_t index = 0; index + 2 < values.size(); index += 3) {
        points.emplace_back(static_cast<float>(values[index]),
                            static_cast<float>(values[index + 1]),
                            static_cast<float>(values[index + 2]));
    }
    return points;
}

bool IsSupportedDistortionSize(std::size_t size) {
    return size == 4 || size == 5 || size == 8 || size == 12 || size == 14;
}

} // namespace gutcpp
