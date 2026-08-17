#include "buff_ros_utils.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace gutcpp {

namespace fs = std::filesystem;

cv::Rect BBoxToRect(const BBox& bbox) {
    const int width = std::max(1, static_cast<int>(std::lround(bbox.width())));
    const int height = std::max(1, static_cast<int>(std::lround(bbox.height())));
    return cv::Rect(static_cast<int>(std::lround(bbox.xmin)),
                    static_cast<int>(std::lround(bbox.ymin)),
                    width,
                    height);
}

std::optional<cv::Rect> ParseRoiParameter(const std::vector<int64_t>& values) {
    if (values.size() < 4) {
        return std::nullopt;
    }

    const int x = static_cast<int>(values[0]);
    const int y = static_cast<int>(values[1]);
    const int width = static_cast<int>(values[2]);
    const int height = static_cast<int>(values[3]);
    if (width <= 0 || height <= 0) {
        return std::nullopt;
    }
    return cv::Rect(x, y, width, height);
}

int PreferredYoloSeedClassId(const std::string& color) {
    return (color == "red") ? 1 : 2;
}

std::vector<int64_t> GetIntegerArrayParameterOrEmpty(const rclcpp::Node& node, const char* name) {
    rclcpp::Parameter parameter;
    if (!node.get_parameter(name, parameter)) {
        return {};
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        return {};
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        throw std::runtime_error(std::string("Parameter '") + name + "' must be an integer array");
    }
    return parameter.as_integer_array();
}

std::string ResolveRosPath(const std::string& rawPath) {
    if (rawPath.empty()) {
        return rawPath;
    }

    const fs::path inputPath(rawPath);
    if (inputPath.is_absolute()) {
        return inputPath.lexically_normal().string();
    }

    std::vector<fs::path> candidates;
    candidates.emplace_back((fs::current_path() / inputPath).lexically_normal());

    try {
        const fs::path packageShare(ament_index_cpp::get_package_share_directory("rm_buff_tracker"));
        candidates.emplace_back((packageShare / inputPath).lexically_normal());
    } catch (...) {
    }

    for (const fs::path& candidate : candidates) {
        if (fs::exists(candidate)) {
            return candidate.string();
        }
    }

    return candidates.empty() ? inputPath.lexically_normal().string() : candidates.back().string();
}

sensor_msgs::msg::Image MakeBgrImageMessage(const std_msgs::msg::Header& header, const cv::Mat& image) {
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(image.step);
    msg.data.assign(image.datastart, image.dataend);
    return msg;
}

void DrawBox(cv::Mat& frame, const BBox& bbox, const cv::Scalar& color, const std::string& label) {
    if (bbox.area() <= 0.0) {
        return;
    }
    const cv::Rect rect = BBoxToRect(bbox);
    cv::rectangle(frame, rect, color, 2);
    cv::putText(frame, label, rect.tl() + cv::Point(0, -8),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
}

FramePacket MakeFramePacket(const sensor_msgs::msg::Image& msg) {
    FramePacket packet;
    packet.header = msg.header;
    packet.encoding = msg.encoding;
    packet.height = msg.height;
    packet.width = msg.width;
    packet.isBigEndian = msg.is_bigendian;
    packet.step = msg.step;
    packet.data = msg.data;
    return packet;
}

cv::Mat MakeBgrFrame(const FramePacket& packet) {
    int cvType = CV_8UC3;
    if (packet.encoding == sensor_msgs::image_encodings::MONO8) {
        cvType = CV_8UC1;
    } else if (packet.encoding == sensor_msgs::image_encodings::BGRA8 ||
               packet.encoding == sensor_msgs::image_encodings::RGBA8) {
        cvType = CV_8UC4;
    }

    cv::Mat frame(static_cast<int>(packet.height),
                  static_cast<int>(packet.width),
                  cvType,
                  const_cast<uint8_t*>(packet.data.data()),
                  static_cast<size_t>(packet.step));

    cv::Mat bgr;
    if (packet.encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(frame, bgr, cv::COLOR_RGB2BGR);
    } else if (packet.encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
    } else if (packet.encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cvtColor(frame, bgr, cv::COLOR_RGBA2BGR);
    } else if (packet.encoding == sensor_msgs::image_encodings::MONO8) {
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    } else {
        bgr = frame.clone();
    }
    return bgr;
}

cv::Mat MakeBgrFrame(const sensor_msgs::msg::Image& msg) {
    return MakeBgrFrame(MakeFramePacket(msg));
}

} // namespace gutcpp
